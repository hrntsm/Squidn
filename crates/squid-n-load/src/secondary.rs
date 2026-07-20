//! 二次部材（小梁・間柱）経由の荷重を主架構へ変換する（CMQ 経路）。
//!
//! 二次部材の支持点や床パネルの角は、主架構の梁（大梁）のスパン中間に
//! 節点共有なしで載ることがある（ST-Bridge 取り込みモデルの典型）。
//! そのような「要素が接続しない節点」への集中荷重は解析に載らないため
//! （`DofMap::build` が自由度から除外する）、載っている梁の
//! **中間集中荷重**（`MemberLoadKind::Point`。大梁の CMQ）へ変換する。

use squid_n_core::ids::ElemId;
use squid_n_core::model::{ElementKind, MemberLoad, MemberLoadKind, Model, NodalLoad};

/// 節点座標が梁要素のスパン上（端点を除く、距離 `tol` [mm] 以内）にあれば
/// `(要素 ID, i 端からの距離 a)` を返す。複数の梁に載る場合は最も近いものを返す。
pub fn beam_span_position(model: &Model, coord: [f64; 3], tol: f64) -> Option<(ElemId, f64)> {
    let mut best: Option<(ElemId, f64, f64)> = None; // (elem, a, dist)
    for e in &model.elements {
        if e.kind != ElementKind::Beam || e.nodes.len() != 2 {
            continue;
        }
        // ダングリング参照（未検証モデル）はこの要素だけ読み飛ばす。関数全体を
        // 打ち切ると、後続要素で見つかるはずの正しいスパンを取りこぼす
        // （既に見つけていた `best` も含めて）ため、`?` による早期 return は使わない。
        let (Some(node_a), Some(node_b)) = (
            model.nodes.get(e.nodes[0].index()),
            model.nodes.get(e.nodes[1].index()),
        ) else {
            continue;
        };
        let (a, b) = (node_a.coord, node_b.coord);
        let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let len2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
        if len2 < 1.0 {
            continue;
        }
        let ap = [coord[0] - a[0], coord[1] - a[1], coord[2] - a[2]];
        let t = (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / len2;
        let len = len2.sqrt();
        // 端点そのもの（節点共有で解決すべき位置）は対象外。端点近傍 tol 以内は
        // スパン内側へ丸める。
        let a_pos = (t * len).clamp(0.0, len);
        if a_pos <= tol || a_pos >= len - tol {
            continue;
        }
        let proj = [a[0] + t * ab[0], a[1] + t * ab[1], a[2] + t * ab[2]];
        let d = ((coord[0] - proj[0]).powi(2)
            + (coord[1] - proj[1]).powi(2)
            + (coord[2] - proj[2]).powi(2))
        .sqrt();
        if d <= tol && best.map(|(_, _, bd)| d < bd).unwrap_or(true) {
            best = Some((e.id, a_pos, d));
        }
    }
    best.map(|(e, a, _)| (e, a))
}

/// 各節点に要素（解析部材）が接続しているかを返す。
pub fn node_connected_flags(model: &Model) -> Vec<bool> {
    let mut connected = vec![false; model.nodes.len()];
    for e in &model.elements {
        for n in &e.nodes {
            if let Some(slot) = connected.get_mut(n.index()) {
                *slot = true;
            }
        }
    }
    connected
}

/// 「要素が接続しない節点」への節点荷重を、その節点が載っている主架構梁の
/// 中間集中荷重（CMQ）へ変換する。
///
/// - 要素が接続する節点への荷重: そのまま `NodalLoad` として返す。
/// - 接続しない節点で、力成分（並進）が非零・モーメント成分が零、かつ節点が
///   梁スパン上（±`tol`）にある: `MemberLoad`（`Point{a, p}`、`dir` = 力の方向）
///   へ変換する。
/// - 変換できない荷重（モーメント付き・どの梁にも載らない等）: `NodalLoad` の
///   まま返す（解析では零剛性節点として無視されるが、荷重タブでは見える）。
///
/// 変換は冪等（変換済みの出力を再度通しても変化しない）。
pub fn resolve_nodal_to_primary(
    model: &Model,
    nodal: Vec<NodalLoad>,
    tol: f64,
) -> (Vec<NodalLoad>, Vec<MemberLoad>) {
    let connected = node_connected_flags(model);
    let mut out_nodal = Vec::new();
    let mut out_member = Vec::new();
    for nl in nodal {
        let ni = nl.node.index();
        if connected.get(ni).copied().unwrap_or(false) {
            out_nodal.push(nl);
            continue;
        }
        let f = [nl.values[0], nl.values[1], nl.values[2]];
        let p = (f[0] * f[0] + f[1] * f[1] + f[2] * f[2]).sqrt();
        let has_moment = nl.values[3..6].iter().any(|m| m.abs() > 1e-9);
        if p <= 1e-9 || has_moment {
            out_nodal.push(nl);
            continue;
        }
        let Some(node) = model.nodes.get(ni) else {
            out_nodal.push(nl);
            continue;
        };
        match beam_span_position(model, node.coord, tol) {
            Some((elem, a)) => out_member.push(MemberLoad {
                elem,
                dir: [f[0] / p, f[1] / p, f[2] / p],
                kind: MemberLoadKind::Point { a, p },
            }),
            None => out_nodal.push(nl),
        }
    }
    (out_nodal, out_member)
}

/// 節点→梁スパン変換の既定許容差 [mm]（大梁芯からのずれの許容）。
pub const SPAN_TOL_MM: f64 = 10.0;

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{NodeId, SectionId};
    use squid_n_core::model::{
        ElementData, EndCondition, ForceRegime, LocalAxis, Node, SecondaryMember,
        SecondaryMemberKind,
    };

    fn node(id: u32, x: f64, y: f64, z: f64) -> Node {
        Node {
            id: NodeId(id),
            coord: [x, y, z],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        }
    }

    fn beam(id: u32, a: u32, b: u32) -> ElementData {
        ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: [NodeId(a), NodeId(b)].into_iter().collect(),
            section: None,
            material: None,
            local_axis: LocalAxis {
                ref_vector: [0.0, 0.0, 1.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }
    }

    /// 大梁（0-1, 長さ6000）のスパン上 x=2000 にある非接続節点(2)への鉛直荷重が、
    /// 大梁の中間集中荷重 Point{a=2000} へ変換される。接続節点(0)への荷重はそのまま。
    #[test]
    fn test_resolve_nodal_to_primary_converts_span_node() {
        let model = Model {
            nodes: vec![
                node(0, 0.0, 0.0, 0.0),
                node(1, 6000.0, 0.0, 0.0),
                node(2, 2000.0, 0.0, 0.0),
            ],
            elements: vec![beam(0, 0, 1)],
            secondary_members: vec![SecondaryMember {
                kind: SecondaryMemberKind::Joist,
                nodes: [NodeId(2), NodeId(2)],
                section: Some(SectionId(0)),
                material: None,
                name: "b1".into(),
            }],
            ..Default::default()
        };
        let nodal = vec![
            NodalLoad {
                node: NodeId(2),
                values: [0.0, 0.0, -5000.0, 0.0, 0.0, 0.0],
            },
            NodalLoad {
                node: NodeId(0),
                values: [0.0, 0.0, -1000.0, 0.0, 0.0, 0.0],
            },
        ];
        let (out_nodal, out_member) = resolve_nodal_to_primary(&model, nodal, SPAN_TOL_MM);
        assert_eq!(out_nodal.len(), 1);
        assert_eq!(out_nodal[0].node, NodeId(0));
        assert_eq!(out_member.len(), 1);
        assert_eq!(out_member[0].elem, ElemId(0));
        assert_eq!(out_member[0].dir, [0.0, 0.0, -1.0]);
        match out_member[0].kind {
            MemberLoadKind::Point { a, p } => {
                assert!((a - 2000.0).abs() < 1e-9);
                assert!((p - 5000.0).abs() < 1e-9);
            }
            _ => panic!("Point になるはず"),
        }
        // 冪等: 変換済み nodal を再度通しても変化しない。
        let (again_nodal, again_member) =
            resolve_nodal_to_primary(&model, out_nodal.clone(), SPAN_TOL_MM);
        assert_eq!(again_nodal, out_nodal);
        assert!(again_member.is_empty());
    }

    /// ダングリング参照（未検証モデル）を持つ梁が先に走査されても、後続の
    /// 正しい梁でスパン位置が見つかる（1 要素のダングリング参照で関数全体が
    /// 空振りしないことの回帰テスト）。
    #[test]
    fn test_beam_span_position_skips_dangling_element() {
        let model = Model {
            nodes: vec![node(0, 0.0, 0.0, 0.0), node(1, 6000.0, 0.0, 0.0)],
            elements: vec![
                // 存在しない節点(99)を参照するダングリング要素を先頭に置く。
                beam(0, 99, 98),
                beam(1, 0, 1),
            ],
            ..Default::default()
        };
        let hit = beam_span_position(&model, [2000.0, 0.0, 0.0], SPAN_TOL_MM);
        assert_eq!(hit, Some((ElemId(1), 2000.0)));
    }

    /// どの梁にも載らない非接続節点への荷重は NodalLoad のまま返る。
    #[test]
    fn test_resolve_nodal_to_primary_keeps_unresolvable() {
        let model = Model {
            nodes: vec![
                node(0, 0.0, 0.0, 0.0),
                node(1, 6000.0, 0.0, 0.0),
                node(2, 3000.0, 5000.0, 0.0),
            ],
            elements: vec![beam(0, 0, 1)],
            ..Default::default()
        };
        let nodal = vec![NodalLoad {
            node: NodeId(2),
            values: [0.0, 0.0, -5000.0, 0.0, 0.0, 0.0],
        }];
        let (out_nodal, out_member) = resolve_nodal_to_primary(&model, nodal, SPAN_TOL_MM);
        assert_eq!(out_nodal.len(), 1);
        assert!(out_member.is_empty());
    }
}
