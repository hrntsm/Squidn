//! 階(Story)の自動生成。
//!
//! 節点の標高(Z)をクラスタリングして階を推定し、各階に剛床(ダイアフラム)と
//! 地震重量を設定する。地震静的解析(Ai分布)・プッシュオーバー・偏心率計算の
//! 前提データを 1 操作で用意するための機能。
//!
//! 重量は「自重(ρ·A·L·g) + 指定荷重ケースの鉛直下向き荷重」を節点に配分し、
//! 階ごとに合計する簡易法(節点支配)による。
//!
//! 剛床代表節点は、剛床に含まれる節点の慣性力重心（重量重み付き重心）に
//! 専用の仮想節点として自動生成する（既存節点の流用ではない）。
//! 参考: RESP技術ブログ「剛床に関連する操作や考え方のまとめ」(2026-05-29)。
//! 並進慣性重量は ΣiW、回転慣性重量は ΣiW·ir² となり、スレーブ節点の面内応答は
//! `crates/squid-n-solver/src/constraint.rs` の RigidDiaphragm 縮約で
//! ix = Gx − iry·Gθz, iy = Gy + irx·Gθz として復元される。
//! 回転慣性重量 ΣiW·ir² は質量を代表節点自体に持たせなくても、要素・節点側に残った
//! 質量が Reducer の TᵀMT 縮約（`eigen.rs`）で自動的にマスターへ集約されるため、
//! 代表節点の `mass` は常に `None` とする（二重計上を避ける）。

use squid_n_core::dof::{Dof, Dof6Mask};
use squid_n_core::ids::{LoadCaseId, NodeId, StoryId};
use squid_n_core::model::{
    Constraint, DiaphragmDef, ElementKind, MemberLoadKind, Model, Node, Story,
};

/// 重力加速度 [mm/s²]（内部単位系 N-mm-s、質量 ton）。
const GRAVITY_MM_S2: f64 = 9800.0;

/// 同一階とみなす標高差 [mm]。
const LEVEL_TOL_MM: f64 = 1.0;

/// 生成結果。[`Model`] へ適用するのは呼び出し側（EditCommand 経由）。
#[derive(Clone, Debug, PartialEq)]
pub struct StoryGenResult {
    /// 下から順の階（基部レベルは含まない）。
    pub stories: Vec<Story>,
    /// 各節点の所属階（`model.nodes` と同順。基部レベルは None）。
    /// 長さは `model.nodes.len()`（新規に生成される代表節点は含まない。
    /// 代表節点の所属階は `rep_nodes` 側の `story` フィールドが正）。
    pub node_story: Vec<Option<StoryId>>,
    /// 各階の剛床拘束（`Reducer` が読む `model.constraints` 用）。
    pub constraints: Vec<Constraint>,
    /// 生成・更新される剛床代表節点（座標＝慣性力重心、拘束・所属階設定済み）。
    /// ID が既存の `model.nodes` 範囲内なら置換（再利用）、範囲外なら新規追加。
    pub rep_nodes: Vec<Node>,
    /// 適用後の `model.generated_masters` の全量。
    pub generated_masters: Vec<NodeId>,
}

/// 節点 Z 座標から階を自動生成する。
///
/// - 最下レベルは基部(支点レベル)とみなし階に含めない
/// - 前回生成した剛床代表節点（`model.generated_masters`）は構造節点の
///   クラスタリング対象から除外する（再生成時に過去の代表節点を混ぜない）
/// - 各階の剛床代表節点は、その階の全構造節点の慣性力重心（重量重み付き重心）に
///   新規生成する（既存の `generated_masters` があれば座標・拘束・所属階を更新して再利用）
/// - `gravity_lc` を指定すると、そのケースの鉛直下向き荷重を地震重量に算入する
///   （自重は材料密度から常に算入）
pub fn generate_stories(
    model: &Model,
    gravity_lc: Option<LoadCaseId>,
) -> Result<StoryGenResult, String> {
    if model.nodes.is_empty() {
        return Err("節点がありません".into());
    }

    // --- 0. 構造節点の抽出（前回生成分の剛床代表節点を除外） ---
    let generated: std::collections::HashSet<NodeId> =
        model.generated_masters.iter().copied().collect();
    let struct_nodes: Vec<&Node> = model
        .nodes
        .iter()
        .filter(|n| !generated.contains(&n.id))
        .collect();
    if struct_nodes.is_empty() {
        return Err("節点がありません".into());
    }

    // --- 1. Z レベルのクラスタリング（構造節点のみ） ---
    let mut zs: Vec<f64> = struct_nodes.iter().map(|n| n.coord[2]).collect();
    zs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mut levels: Vec<f64> = Vec::new();
    for z in zs {
        match levels.last() {
            Some(&last) if (z - last).abs() <= LEVEL_TOL_MM => {}
            _ => levels.push(z),
        }
    }
    if levels.len() < 2 {
        return Err(
            "節点の標高(Z)が 1 レベルしかありません。階を生成するには 2 レベル以上必要です。"
                .into(),
        );
    }

    let level_of = |z: f64| -> usize {
        levels
            .iter()
            .position(|&l| (z - l).abs() <= LEVEL_TOL_MM)
            .unwrap_or(0)
    };

    // --- 2. 節点の重量配分 ---
    let mut node_weight = vec![0.0f64; model.nodes.len()];

    // 自重: ρ·A·L·g を両端に半分ずつ
    for elem in &model.elements {
        if !matches!(elem.kind, ElementKind::Beam) || elem.nodes.len() < 2 {
            continue;
        }
        let (Some(sec_id), Some(mat_id)) = (elem.section, elem.material) else {
            continue;
        };
        let (Some(sec), Some(mat)) = (
            model.sections.get(sec_id.index()),
            model.materials.get(mat_id.index()),
        ) else {
            continue;
        };
        let ni = elem.nodes[0].index();
        let nj = elem.nodes[1].index();
        let (ci, cj) = (model.nodes[ni].coord, model.nodes[nj].coord);
        let len =
            ((cj[0] - ci[0]).powi(2) + (cj[1] - ci[1]).powi(2) + (cj[2] - ci[2]).powi(2)).sqrt();
        let w = mat.density * sec.area * len * GRAVITY_MM_S2;
        node_weight[ni] += w / 2.0;
        node_weight[nj] += w / 2.0;
    }

    // 指定荷重ケースの鉛直下向き成分
    if let Some(lc_id) = gravity_lc {
        if let Some(lc) = model.load_cases.iter().find(|c| c.id == lc_id) {
            for nl in &lc.nodal {
                if nl.values[2] < 0.0 {
                    node_weight[nl.node.index()] += -nl.values[2];
                }
            }
            for ml in &lc.member {
                let Some(elem) = model
                    .elements
                    .iter()
                    .find(|e| e.id == ml.elem)
                    .filter(|e| e.nodes.len() >= 2)
                else {
                    continue;
                };
                // 全体座標系の作用方向の鉛直下向き成分
                let dz = ml.dir[2];
                if dz >= 0.0 {
                    continue;
                }
                let total = match ml.kind {
                    MemberLoadKind::Point { p, .. } => p,
                    MemberLoadKind::Distributed { a, b, w1, w2 } => (w1 + w2) / 2.0 * (b - a),
                };
                let w = total * (-dz);
                node_weight[elem.nodes[0].index()] += w / 2.0;
                node_weight[elem.nodes[1].index()] += w / 2.0;
            }
        }
    }

    // --- 3. 階の構築（レベル 1 以上、下から順） ---
    let mut stories = Vec::new();
    let mut node_story = vec![None; model.nodes.len()];
    let mut constraints = Vec::new();
    let mut rep_nodes: Vec<Node> = Vec::new();
    let mut generated_masters: Vec<NodeId> = Vec::new();

    // 既存の代表節点は昇順（下の階から順）に再利用し、足りない分は末尾連番で新規生成する。
    let mut reuse_masters = model.generated_masters.iter().copied();
    let mut next_new_id = model.nodes.len() as u32;

    // 剛床代表節点の拘束: 要素が接続しない浮遊節点のため、剛床が拘束しない
    // 3 自由度（Uz, Rx, Ry）を固定しないと特異行列になる。Ux, Uy, Rz は自由。
    let mut rep_restraint = Dof6Mask::FREE;
    rep_restraint.set_fixed(Dof::Uz);
    rep_restraint.set_fixed(Dof::Rx);
    rep_restraint.set_fixed(Dof::Ry);

    for (si, &elev) in levels.iter().enumerate().skip(1) {
        let story_id = StoryId((si - 1) as u32);
        let node_ids: Vec<NodeId> = struct_nodes
            .iter()
            .filter(|n| level_of(n.coord[2]) == si)
            .map(|n| n.id)
            .collect();
        if node_ids.is_empty() {
            continue;
        }

        let weight: f64 = node_ids.iter().map(|n| node_weight[n.index()]).sum();

        // 慣性力重心（重量重み付き重心）。重量が算定できない場合は幾何重心へフォールバック。
        let (gx, gy) = if weight > 0.0 {
            let gx = node_ids
                .iter()
                .map(|n| node_weight[n.index()] * model.nodes[n.index()].coord[0])
                .sum::<f64>()
                / weight;
            let gy = node_ids
                .iter()
                .map(|n| node_weight[n.index()] * model.nodes[n.index()].coord[1])
                .sum::<f64>()
                / weight;
            (gx, gy)
        } else {
            let gx = node_ids
                .iter()
                .map(|n| model.nodes[n.index()].coord[0])
                .sum::<f64>()
                / node_ids.len() as f64;
            let gy = node_ids
                .iter()
                .map(|n| model.nodes[n.index()].coord[1])
                .sum::<f64>()
                / node_ids.len() as f64;
            (gx, gy)
        };

        // 剛床代表節点（慣性力重心に置く専用の仮想節点）の生成/再利用。
        let master = reuse_masters.next().unwrap_or_else(|| {
            let id = NodeId(next_new_id);
            next_new_id += 1;
            id
        });
        rep_nodes.push(Node {
            id: master,
            coord: [gx, gy, elev],
            restraint: rep_restraint,
            mass: None,
            story: Some(story_id),
        });
        generated_masters.push(master);

        // 当該階の全構造節点がスレーブ（マスターは専用節点のため、既存節点は 1 点でも全てスレーブ）。
        let slaves = node_ids.clone();

        for n in &node_ids {
            node_story[n.index()] = Some(story_id);
        }
        constraints.push(Constraint::RigidDiaphragm {
            story: story_id,
            master,
            slaves: slaves.clone(),
        });
        stories.push(Story {
            id: story_id,
            name: format!("{}F", si),
            elevation: elev,
            node_ids,
            diaphragms: vec![DiaphragmDef {
                master,
                slaves,
                rigid: true,
                weight: Some(weight),
            }],
            seismic_weight: Some(weight),
            structure: Default::default(),
            level_kind: Default::default(),
        });
    }

    if stories.is_empty() {
        return Err("階を構成する節点が見つかりませんでした。".into());
    }

    // 階数が減って余った旧代表節点は不活性化する（拘束固定・所属階なし）が、
    // `generated_masters` には残して次回再生成時に再利用できるようにする。
    for id in reuse_masters {
        rep_nodes.push(Node {
            id,
            coord: model.nodes[id.index()].coord,
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: None,
        });
        generated_masters.push(id);
    }

    Ok(StoryGenResult {
        stories,
        node_story,
        constraints,
        rep_nodes,
        generated_masters,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, MaterialId, SectionId};
    use squid_n_core::model::{
        ElementData, EndCondition, ForceRegime, LoadCase, LocalAxis, Material, MemberLoad,
        NodalLoad, Node, Section,
    };

    /// 2 層 × 1 スパンの平面ラーメン（各レベル 2 節点）。
    fn two_story_model() -> Model {
        let mut model = Model::default();
        let coords = [
            [0.0, 0.0, 0.0],
            [6000.0, 0.0, 0.0],
            [0.0, 0.0, 3500.0],
            [6000.0, 0.0, 3500.0],
            [0.0, 0.0, 7000.0],
            [6000.0, 0.0, 7000.0],
        ];
        for (i, c) in coords.iter().enumerate() {
            model.nodes.push(Node {
                id: NodeId(i as u32),
                coord: *c,
                restraint: if i < 2 {
                    Dof6Mask::FIXED
                } else {
                    Dof6Mask::FREE
                },
                mass: None,
                story: None,
            });
        }
        model.sections.push(Section {
            id: SectionId(0),
            name: "S".into(),
            area: 10000.0,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e8,
            depth: 300.0,
            width: 300.0,
            as_y: 8000.0,
            as_z: 8000.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        });
        model.materials.push(Material {
            id: MaterialId(0),
            name: "SN400B".into(),
            young: 205000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
            fc: None,
            fy: None,
        });
        // 柱4 + 梁2
        let conn: [(u32, u32); 6] = [(0, 2), (1, 3), (2, 4), (3, 5), (2, 3), (4, 5)];
        for (i, (a, b)) in conn.iter().enumerate() {
            model.elements.push(ElementData {
                id: ElemId(i as u32),
                kind: ElementKind::Beam,
                nodes: [NodeId(*a), NodeId(*b)].into_iter().collect(),
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
            });
        }
        model.load_cases.push(LoadCase {
            kind: Default::default(),
            id: LoadCaseId(0),
            name: "DL".into(),
            nodal: vec![NodalLoad {
                node: NodeId(4),
                values: [0.0, 0.0, -50000.0, 0.0, 0.0, 0.0],
            }],
            member: vec![MemberLoad {
                elem: ElemId(4),
                dir: [0.0, 0.0, -1.0],
                kind: MemberLoadKind::Distributed {
                    a: 0.0,
                    b: 6000.0,
                    w1: 10.0,
                    w2: 10.0,
                },
            }],
        });
        model
    }

    #[test]
    fn test_generate_two_stories() {
        let model = two_story_model();
        let gen = generate_stories(&model, Some(LoadCaseId(0))).unwrap();
        assert_eq!(gen.stories.len(), 2);
        assert_eq!(gen.stories[0].elevation, 3500.0);
        assert_eq!(gen.stories[1].elevation, 7000.0);
        // 各階 2 節点 → 代表節点(慣性力重心)を新規生成 + スレーブ2（既存節点は全てスレーブ）
        assert_eq!(gen.stories[0].node_ids.len(), 2);
        assert_eq!(gen.stories[0].diaphragms[0].slaves.len(), 2);
        assert_eq!(gen.stories[1].diaphragms[0].slaves.len(), 2);
        assert_eq!(gen.constraints.len(), 2);
        // 基部節点は無所属
        assert_eq!(gen.node_story[0], None);
        assert_eq!(gen.node_story[2], Some(StoryId(0)));
        assert_eq!(gen.node_story[4], Some(StoryId(1)));
        // 重量: 1F = 梁分布荷重 10 N/mm × 6000 = 60 kN + 自重、2F = 節点荷重 50 kN + 自重
        let w1 = gen.stories[0].seismic_weight.unwrap();
        let w2 = gen.stories[1].seismic_weight.unwrap();
        assert!(w1 > 60000.0, "w1={}", w1);
        assert!(w2 > 50000.0, "w2={}", w2);

        // 代表節点は新規生成（既存節点数=6 の末尾連番）。
        assert_eq!(gen.rep_nodes.len(), 2);
        assert_eq!(gen.generated_masters, vec![NodeId(6), NodeId(7)]);
        for rep in &gen.rep_nodes {
            assert_eq!(rep.mass, None, "質量は Reducer 側の TᵀMT 縮約に委ねる");
            assert!(rep.restraint.is_fixed(squid_n_core::dof::Dof::Uz));
            assert!(rep.restraint.is_fixed(squid_n_core::dof::Dof::Rx));
            assert!(rep.restraint.is_fixed(squid_n_core::dof::Dof::Ry));
            assert!(!rep.restraint.is_fixed(squid_n_core::dof::Dof::Ux));
            assert!(!rep.restraint.is_fixed(squid_n_core::dof::Dof::Uy));
            assert!(!rep.restraint.is_fixed(squid_n_core::dof::Dof::Rz));
        }
        assert_eq!(gen.rep_nodes[0].story, Some(StoryId(0)));
        assert_eq!(gen.rep_nodes[1].story, Some(StoryId(1)));
        // 1F は左右対称な自重＋分布荷重のみなので慣性力重心の X は中央(3000)になる。
        assert!((gen.rep_nodes[0].coord[0] - 3000.0).abs() < 1e-6);
        // 2F は節点荷重(50kN)が NodeId(4)(x=0)側のみに掛かる非対称配置なので、
        // 慣性力重心は x=0 側へ偏る(単純な幾何重心 3000 とは一致しない)。
        assert!(
            (gen.rep_nodes[1].coord[0] - 382.58).abs() < 0.1,
            "{}",
            gen.rep_nodes[1].coord[0]
        );
        assert_eq!(gen.rep_nodes[0].coord[2], 3500.0);
        assert_eq!(gen.rep_nodes[1].coord[2], 7000.0);
    }

    #[test]
    fn test_generate_single_level_is_error() {
        let mut model = Model::default();
        model.nodes.push(Node {
            id: NodeId(0),
            coord: [0.0, 0.0, 0.0],
            restraint: Dof6Mask::FIXED,
            mass: None,
            story: None,
        });
        assert!(generate_stories(&model, None).is_err());
    }

    /// 重量が非対称な 1 層モデル（自重なし・節点荷重のみで重みを制御）。
    fn asymmetric_weight_model() -> Model {
        let mut model = Model::default();
        let coords = [
            [0.0, 0.0, 0.0],
            [4000.0, 0.0, 0.0],
            [0.0, 0.0, 3000.0],
            [4000.0, 0.0, 3000.0],
        ];
        for (i, c) in coords.iter().enumerate() {
            model.nodes.push(Node {
                id: NodeId(i as u32),
                coord: *c,
                restraint: if i < 2 {
                    Dof6Mask::FIXED
                } else {
                    Dof6Mask::FREE
                },
                mass: None,
                story: None,
            });
        }
        model.load_cases.push(LoadCase {
            kind: Default::default(),
            id: LoadCaseId(0),
            name: "DL".into(),
            nodal: vec![
                NodalLoad {
                    node: NodeId(2),
                    values: [0.0, 0.0, -100000.0, 0.0, 0.0, 0.0],
                },
                NodalLoad {
                    node: NodeId(3),
                    values: [0.0, 0.0, -300000.0, 0.0, 0.0, 0.0],
                },
            ],
            member: vec![],
        });
        model
    }

    #[test]
    fn test_generate_weighted_centroid_matches_hand_calc() {
        let model = asymmetric_weight_model();
        let gen = generate_stories(&model, Some(LoadCaseId(0))).unwrap();
        assert_eq!(gen.stories.len(), 1);
        let story = &gen.stories[0];
        // 重量は自重なし・節点荷重のみ: 100kN + 300kN = 400kN
        assert_eq!(story.seismic_weight, Some(400000.0));
        assert_eq!(
            story.diaphragms[0].slaves.len(),
            2,
            "既存節点は全てスレーブ"
        );

        // 手計算: Gx = Σ(iW·ix)/ΣiW = (100000*0 + 300000*4000) / 400000 = 3000
        assert_eq!(gen.rep_nodes.len(), 1);
        let rep = &gen.rep_nodes[0];
        assert!((rep.coord[0] - 3000.0).abs() < 1e-6, "Gx={}", rep.coord[0]);
        assert!((rep.coord[1] - 0.0).abs() < 1e-6, "Gy={}", rep.coord[1]);
        assert_eq!(rep.coord[2], 3000.0);
        assert_eq!(rep.mass, None);
        assert_eq!(rep.story, Some(StoryId(0)));
        assert!(rep.restraint.is_fixed(Dof::Uz));
        assert!(rep.restraint.is_fixed(Dof::Rx));
        assert!(rep.restraint.is_fixed(Dof::Ry));
        assert!(!rep.restraint.is_fixed(Dof::Ux));
        assert!(!rep.restraint.is_fixed(Dof::Uy));
        assert!(!rep.restraint.is_fixed(Dof::Rz));
        // 既存節点数=4 の末尾連番で新規生成される。
        assert_eq!(gen.generated_masters, vec![NodeId(4)]);
    }

    #[test]
    fn test_generate_zero_weight_falls_back_to_geometric_centroid() {
        let mut model = Model::default();
        // 幾何重心が非対称になるよう配置（自重・荷重ケースなし → 重量ゼロ）。
        let coords = [
            [0.0, 0.0, 0.0],
            [4000.0, 0.0, 0.0],
            [0.0, 0.0, 3000.0],
            [6000.0, 0.0, 3000.0],
        ];
        for (i, c) in coords.iter().enumerate() {
            model.nodes.push(Node {
                id: NodeId(i as u32),
                coord: *c,
                restraint: if i < 2 {
                    Dof6Mask::FIXED
                } else {
                    Dof6Mask::FREE
                },
                mass: None,
                story: None,
            });
        }
        let gen = generate_stories(&model, None).unwrap();
        assert_eq!(gen.stories[0].seismic_weight, Some(0.0));
        let rep = &gen.rep_nodes[0];
        // 幾何重心(単純平均) = (0 + 6000) / 2 = 3000
        assert!((rep.coord[0] - 3000.0).abs() < 1e-6, "Gx={}", rep.coord[0]);
    }
}
