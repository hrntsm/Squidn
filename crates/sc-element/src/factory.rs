use crate::behavior::{ElemState, ElementBehavior};
use sc_core::model::{ElementData, ElementKind, ForceRegime, Model};

/// ForceRegime の自動選択結果（P5 §5）
pub enum ResolvedRegime {
    ConcentratedSpring,
    Fiber,
}

/// ForceRegime::Auto をトポロジから判定する（P5 §5）
/// 剛床所属の階かつ梁で軸力変動が小 → ConcentratedSpring
/// それ以外 → Fiber
pub fn resolve_force_regime(data: &ElementData, model: &Model) -> ResolvedRegime {
    if data.force_regime != ForceRegime::Auto {
        return match data.force_regime {
            ForceRegime::UniaxialBendingShear => ResolvedRegime::ConcentratedSpring,
            ForceRegime::AxialBendingInteract => ResolvedRegime::Fiber,
            ForceRegime::Auto => unreachable!(),
        };
    }

    // Auto の判定ロジック（ヒューリスティック）
    // 剛床に所属する梁（= 鉛直軸でない部材）は集中ばね
    let is_vertical = is_vertical_member(data, model);
    let on_rigid_diaphragm = is_on_rigid_diaphragm(data, model);

    if on_rigid_diaphragm && !is_vertical {
        ResolvedRegime::ConcentratedSpring
    } else {
        ResolvedRegime::Fiber
    }
}

fn is_vertical_member(data: &ElementData, model: &Model) -> bool {
    if data.nodes.len() < 2 {
        return false;
    }
    let n0 = &model.nodes.get(data.nodes[0].index());
    let n1 = &model.nodes.get(data.nodes[1].index());
    match (n0, n1) {
        (Some(n0), Some(n1)) => {
            let dz = (n1.coord[2] - n0.coord[2]).abs();
            let dx = (n1.coord[0] - n0.coord[0]).abs();
            let dy = (n1.coord[1] - n0.coord[1]).abs();
            dz > (dx + dy) * 0.5
        }
        _ => false,
    }
}

fn is_on_rigid_diaphragm(data: &ElementData, model: &Model) -> bool {
    let elem_nodes: Vec<sc_core::ids::NodeId> = data.nodes.iter().copied().collect();
    for story in &model.stories {
        for dia in &story.diaphragms {
            if elem_nodes
                .iter()
                .any(|n| *n == dia.master || dia.slaves.contains(n))
            {
                return true;
            }
        }
    }
    for c in &model.constraints {
        if let sc_core::model::Constraint::RigidDiaphragm { master, slaves, .. } = c {
            if elem_nodes
                .iter()
                .any(|n| *n == *master || slaves.contains(n))
            {
                return true;
            }
        }
    }
    false
}

pub fn build_behavior(data: &ElementData, model: &Model) -> (Box<dyn ElementBehavior>, ElemState) {
    match data.kind {
        ElementKind::Beam => {
            // ForceRegime に基づいて要素種別を選択（P5 §5）
            let regime = resolve_force_regime(data, model);
            match regime {
                ResolvedRegime::ConcentratedSpring => {
                    // T1: ConcentratedSpringBeam が実装されるまでの暫定 BeamElement
                    let elem = crate::beam::BeamElement::new(data, model);
                    (Box::new(elem), ElemState::default())
                }
                ResolvedRegime::Fiber => {
                    // T2: FiberBeam が実装されるまでの暫定 BeamElement
                    let elem = crate::beam::BeamElement::new(data, model);
                    (Box::new(elem), ElemState::default())
                }
            }
        }
        ElementKind::PanelZone => (
            Box::new(crate::panel::PanelZone::new(data, model)),
            ElemState::default(),
        ),
        ElementKind::Shell => (
            Box::new(crate::shell::ShellElement::new(data, model)),
            ElemState::default(),
        ),
        ElementKind::Ms => (
            Box::new(crate::ms::MsElement::new(data, model)),
            ElemState::default(),
        ),
        // Fiber 要素：将来 FiberBeam が実装されるまでの暫定 BeamElement
        ElementKind::Fiber => (
            Box::new(crate::beam::BeamElement::new(data, model)),
            ElemState::default(),
        ),
        // Wall 要素：将来 TvlemWall が実装されるまでの暫定 BeamElement
        ElementKind::Wall => (
            Box::new(crate::beam::BeamElement::new(data, model)),
            ElemState::default(),
        ),
    }
}

/// 非線形解析（pushover）用の要素生成。`ForceRegime` に基づき非線形要素を構築する（P5 §5）。
///
/// 線形弾性解析は従来どおり [`build_behavior`]（弾性 `BeamElement`）を使う。両者を分けるのは、
/// `resolve_force_regime` が剛床に乗らない梁も Fiber へ振り分けるため、共通化すると
/// 線形解析の弾性梁まで非線形要素に置き換わってしまうため。
///
/// 注意（既知の制約）: `ConcentratedSpringBeam` は端ばねスケルトン（降伏モーメント）が必要だが、
/// 現状 `Model` に降伏応力／スケルトン供給経路が無いため、軸-曲げ連成を扱う `FiberBeam` に
/// フォールバックしている（P5 §5 の本来意図は集中ばね梁）。また鋼材はファイバ材料が
/// `Bilinear(My=1e20)` で実質弾性のため、真の降伏は `fc` を持つコンクリート断面でのみ生じる。
/// 鋼材の降伏・集中ばね梁の実体化には Model への降伏応力／スケルトン追加が前提（follow-up）。
pub fn build_nonlinear_behavior(
    data: &ElementData,
    model: &Model,
) -> (Box<dyn ElementBehavior>, ElemState) {
    match data.kind {
        ElementKind::Beam => match resolve_force_regime(data, model) {
            ResolvedRegime::ConcentratedSpring | ResolvedRegime::Fiber => (
                Box::new(crate::fiber_elem::FiberBeam::new(data, model)),
                ElemState::default(),
            ),
        },
        ElementKind::Fiber => (
            Box::new(crate::fiber_elem::FiberBeam::new(data, model)),
            ElemState::default(),
        ),
        // PanelZone / Shell / Ms / Wall は現状の挙動（弾性ベース）を踏襲。
        _ => build_behavior(data, model),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sc_core::dof::Dof6Mask;
    use sc_core::ids::{ElemId, MaterialId, NodeId, SectionId};
    use sc_core::model::{EndCondition, LocalAxis, Material, Node, Section};

    fn make_diaphragm_model() -> Model {
        Model {
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
                    coord: [5000.0, 0.0, 0.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(2),
                    coord: [0.0, 0.0, 3000.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: None,
                },
            ],
            constraints: vec![sc_core::model::Constraint::RigidDiaphragm {
                story: sc_core::ids::StoryId(0),
                master: NodeId(2),
                slaves: vec![NodeId(1)],
            }],
            sections: vec![Section {
                id: SectionId(0),
                name: "sec".into(),
                area: 100.0,
                iy: 833.33,
                iz: 833.33,
                j: 100.0,
                depth: 10.0,
                width: 10.0,
                as_y: 83.33,
                as_z: 83.33,
                panel_thickness: None,
                thickness: None,
            }],
            materials: vec![Material {
                id: MaterialId(0),
                name: "mat".into(),
                young: 20000.0,
                poisson: 0.3,
                density: 0.0,
                shear: None,
                fc: None,
                fy: None,
            }],
            ..Default::default()
        }
    }

    #[test]
    fn test_resolve_force_regime_explicit() {
        let model = make_diaphragm_model();
        let elem = ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::UniaxialBendingShear,
            rigid_zone: Default::default(),
        };
        assert!(matches!(
            resolve_force_regime(&elem, &model),
            ResolvedRegime::ConcentratedSpring
        ));
    }

    #[test]
    fn test_resolve_force_regime_auto() {
        let model = make_diaphragm_model();
        // 水平部材＋剛床あり → ConcentratedSpring
        let beam = ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
        };
        assert!(matches!(
            resolve_force_regime(&beam, &model),
            ResolvedRegime::ConcentratedSpring
        ));

        // 鉛直部材 → Fiber
        let col = ElementData {
            id: ElemId(1),
            kind: ElementKind::Beam,
            nodes: smallvec::smallvec![NodeId(0), NodeId(2)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
        };
        assert!(matches!(
            resolve_force_regime(&col, &model),
            ResolvedRegime::Fiber
        ));
    }
}
