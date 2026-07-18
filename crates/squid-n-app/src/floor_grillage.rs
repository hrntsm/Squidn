//! 床格子（小梁）の二段階サブストラクチャ解析（床 Phase F の中核）。
//!
//! 床の小梁を独立した小さな `Model`（格子）として組み、既存の線形静的ソルバで
//! 解いて、(a) 各小梁の部材力（設計用）と (b) 大梁接続点の**支点反力**（大梁へ渡す
//! CMQ 荷重）を取り出す。本体架構は分割せず、受け取るのは反力のみ。
//!
//! 反力はソルバが直接返さないため、要素の全体剛性から `K·u` を集計し、外力
//! （節点荷重＋部材荷重の等価節点力）を差し引いて求める（`reaction = K·u − F_ext`。
//! 拘束自由度でのみ意味を持つ）。交点ジョイントのピン／剛接は、サブモデルの
//! 小梁要素の端部条件（`EndCondition::Pinned`/`Fixed`）で表現する。

use squid_n_core::ids::{ElemId, LoadCaseId};
use squid_n_core::model::Model;
use squid_n_element::beam::MemberForces;

/// 床格子サブモデルの解。
pub struct GrillageSolution {
    /// 各小梁要素の部材力（設計に用いる）。
    pub member_forces: Vec<(ElemId, MemberForces)>,
    /// 各節点の全体系反力 `[Fx,Fy,Fz,Mx,My,Mz]`。拘束自由度でのみ有意
    /// （非拘束自由度は釣合いよりほぼ 0）。大梁 CMQ には鉛直成分 `Fz` を用いる。
    pub reactions: Vec<[f64; 6]>,
}

/// 床格子サブモデル `model` の荷重ケース `lc` を解き、部材力と支点反力を返す。
/// `model` は呼び出し側が構築した独立サブモデル（本体架構を含まない）。
pub fn solve_grillage(model: &Model, lc: LoadCaseId) -> Result<GrillageSolution, String> {
    let once = squid_n_solver::linear::linear_static_once(model, lc)
        .map_err(|e| format!("床格子の求解に失敗: {e:?}"))?;
    let reactions = compute_reactions(model, lc, &once.disp);
    Ok(GrillageSolution {
        member_forces: once.member_forces,
        reactions,
    })
}

/// `reaction = K·u − F_ext` を全節点・全成分について求める。
/// `K·u` は各要素の全体剛性 × 節点変位を集計、`F_ext` は節点荷重＋部材荷重の
/// 等価節点力（`assemble` と同じ `consistent_load_local` を用いる）。
fn compute_reactions(model: &Model, lc: LoadCaseId, disp: &[[f64; 6]]) -> Vec<[f64; 6]> {
    use squid_n_element::behavior::Ctx;
    use squid_n_element::transform::LocalFrame;

    let n = model.nodes.len();
    // 内力 K·u（全体系）を節点へ集計。
    let mut p_int = vec![[0.0f64; 6]; n];
    for elem in &model.elements {
        if elem.nodes.len() < 2 {
            continue;
        }
        let ni = elem.nodes[0].index();
        let nj = elem.nodes[1].index();
        if ni >= n || nj >= n {
            continue;
        }
        let (behavior, state) = squid_n_element::build_behavior(elem, model);
        let k = behavior.tangent_stiffness(&state, &Ctx { model });
        // u_global（12）= [i:0..6, j:6..12]
        let mut u = [0.0f64; 12];
        u[0..6].copy_from_slice(&disp[ni]);
        u[6..12].copy_from_slice(&disp[nj]);
        // f = K·u（全体系）
        for (i, pf) in [ni, nj].into_iter().enumerate() {
            for (d, pd) in p_int[pf].iter_mut().enumerate() {
                let row = i * 6 + d;
                let mut s = 0.0;
                for (j, &uj) in u.iter().enumerate() {
                    s += k.get(row, j) * uj;
                }
                *pd += s;
            }
        }
    }

    // 外力 F_ext（節点荷重＋部材荷重の等価節点力）。
    let mut f_ext = vec![[0.0f64; 6]; n];
    if let Some(case) = model.load_cases.iter().find(|c| c.id == lc) {
        for nl in &case.nodal {
            let idx = nl.node.index();
            if idx < n {
                for (fd, &v) in f_ext[idx].iter_mut().zip(nl.values.iter()) {
                    *fd += v;
                }
            }
        }
        for elem in &model.elements {
            if elem.nodes.len() < 2 {
                continue;
            }
            let loads: Vec<_> = case
                .member
                .iter()
                .filter(|ml| ml.elem == elem.id)
                .cloned()
                .collect();
            if loads.is_empty() {
                continue;
            }
            let ni = elem.nodes[0].index();
            let nj = elem.nodes[1].index();
            if ni >= n || nj >= n {
                continue;
            }
            let p_i = model.nodes[ni].coord;
            let p_j = model.nodes[nj].coord;
            let length = {
                let d = [p_j[0] - p_i[0], p_j[1] - p_i[1], p_j[2] - p_i[2]];
                (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
            };
            if length < 1e-9 {
                continue;
            }
            let frame = LocalFrame::from_nodes(p_i, p_j, elem.local_axis.ref_vector);
            let q_local =
                squid_n_element::member_load::consistent_load_local(&loads, &frame, length);
            let q_global = frame.rotate_to_global(&q_local);
            for (i, pf) in [ni, nj].into_iter().enumerate() {
                for (d, fd) in f_ext[pf].iter_mut().enumerate() {
                    *fd += q_global[i * 6 + d];
                }
            }
        }
    }

    // reaction = K·u − F_ext
    let mut reactions = vec![[0.0f64; 6]; n];
    for ((r, pi), fe) in reactions.iter_mut().zip(p_int.iter()).zip(f_ext.iter()) {
        for (rd, (pd, fd)) in r.iter_mut().zip(pi.iter().zip(fe.iter())) {
            *rd = pd - fd;
        }
    }
    reactions
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{MaterialId, NodeId, SectionId};
    use squid_n_core::model::{
        ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LoadCaseKind, LocalAxis,
        Material, MemberLoad, MemberLoadKind, Node, Section,
    };

    fn beam_section(id: u32) -> Section {
        Section {
            id: SectionId(id),
            name: "H".into(),
            area: 10000.0,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e6,
            depth: 400.0,
            width: 200.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        }
    }
    fn steel(id: u32) -> Material {
        Material {
            concrete_class: Default::default(),
            id: MaterialId(id),
            name: "SN400".into(),
            young: 205_000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
            fc: None,
            fy: Some(235.0),
        }
    }

    /// 両端固定梁の UDL: 鉛直反力は各端 wL/2、総和 wL（反力抽出の検算）。
    #[test]
    fn test_grillage_reaction_fixed_fixed_udl() {
        let l = 4000.0_f64;
        let w = 10.0_f64; // N/mm（下向き）
        let model = Model {
            nodes: vec![
                Node {
                    id: NodeId(0),
                    coord: [0.0, 0.0, 0.0],
                    restraint: Dof6Mask::FIXED,
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(1),
                    coord: [l, 0.0, 0.0],
                    restraint: Dof6Mask::FIXED,
                    mass: None,
                    story: None,
                },
            ],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::Beam,
                nodes: [NodeId(0), NodeId(1)].into_iter().collect(),
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [0.0, 0.0, 1.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
                plastic_zone: None,
                spring: None,
            }],
            sections: vec![beam_section(0)],
            materials: vec![steel(0)],
            load_cases: vec![LoadCase {
                id: LoadCaseId(0),
                name: "床".into(),
                kind: LoadCaseKind::Dead,
                nodal: vec![],
                member: vec![MemberLoad {
                    elem: ElemId(0),
                    dir: [0.0, 0.0, -1.0],
                    kind: MemberLoadKind::Distributed {
                        a: 0.0,
                        b: l,
                        w1: w,
                        w2: w,
                    },
                }],
            }],
            ..Default::default()
        };
        model.validate().expect("submodel validate");

        let sol = solve_grillage(&model, LoadCaseId(0)).expect("solve");
        let total = w * l;
        // 鉛直反力（+Z 上向き）は各端 wL/2。
        assert!(
            (sol.reactions[0][2] - total / 2.0).abs() / (total / 2.0) < 1e-6,
            "R0z={}",
            sol.reactions[0][2]
        );
        assert!(
            (sol.reactions[1][2] - total / 2.0).abs() / (total / 2.0) < 1e-6,
            "R1z={}",
            sol.reactions[1][2]
        );
        // 総和 = 全載荷。
        let sum = sol.reactions[0][2] + sol.reactions[1][2];
        assert!((sum - total).abs() / total < 1e-6, "sum={sum}");
    }
}
