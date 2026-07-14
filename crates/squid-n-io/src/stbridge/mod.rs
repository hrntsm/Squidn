//! ST-Bridge（XML, 2.0 系）入出力。設計書 §12.5 / 仕様 `specs/P8_操作と連携.md` §7.1。
//!
//! # 対応範囲（意味的往復を保証する subset）
//! - **節点**（座標・所属層）、**層**（名称・標高）、**材料**（E・ν・密度・Fc・Fy）、
//!   **断面**（面積・断面二次モーメント等の物性）、**部材**（柱＝鉛直／大梁＝水平、節点・断面・
//!   材料の参照、部材軸 ref_vector）、**荷重ケース**（節点荷重）。
//! - import→export→再import で上記が意味的に一致する（DoD §8.3）。
//!
//! # 非対応（仕様どおり対象外）
//! - 解析結果・独自属性（§12.5）。
//! - 拘束条件・質量（ST-Bridge の幾何スコープ外。import 後は既定値）。
//! - **断面は実 ST-Bridge の形鋼ライブラリ参照（StbSecColumn_S 等）ではなく、内部モデルの物性を
//!   そのまま持つ `StbSecRaw` で表現する**（正準モデルを唯一の真実とする方針）。他社ソフトとの
//!   完全な相互運用は断面形状名のマッピングが要るため将来課題。
//! - 床・ブレース・剛域・端部接合等の詳細。
//!
//! 一次資料: ST-Bridge 公式スキーマ（XML 2.0 系）。要素・属性名はこれに準拠（subset）。
//!
//! # モジュール構成（1 ファイル 1 責務）
//! - [`export`] — 直列化（内部モデル → ST-Bridge XML）。
//! - [`import`] — パース（ST-Bridge XML → 内部モデル）。

mod export;
mod import;

pub use export::export_stbridge;
pub use import::import_stbridge;

#[derive(Debug, thiserror::Error)]
pub enum StbError {
    #[error("xml parse: {0}")]
    Parse(String),
    #[error("unsupported version: {0}")]
    Version(String),
    #[error("unmappable element: {0}")]
    Unmappable(String),
}

const STB_VERSION: &str = "2.0.0";

#[cfg(test)]
use squid_n_core::ids::{ElemId, LoadCaseId, MaterialId, NodeId, SectionId, StoryId};
#[cfg(test)]
use squid_n_core::model::{
    ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LocalAxis, Material, Model,
    NodalLoad, Node, Section, Story,
};

#[cfg(test)]
mod tests {
    use super::*;
    use smallvec::smallvec;

    fn representative_model() -> Model {
        let mut m = Model::default();
        // 4 節点（底2・上2）。
        for (i, c) in [
            [0.0, 0.0, 0.0],
            [6000.0, 0.0, 0.0],
            [0.0, 0.0, 3000.0],
            [6000.0, 0.0, 3000.0],
        ]
        .iter()
        .enumerate()
        {
            m.nodes.push(Node {
                id: NodeId(i as u32),
                coord: *c,
                restraint: squid_n_core::dof::Dof6Mask::FREE,
                mass: None,
                story: if i >= 2 { Some(StoryId(0)) } else { None },
            });
        }
        m.stories.push(Story {
            level_kind: Default::default(),
            structure: Default::default(),
            id: StoryId(0),
            name: "1F".into(),
            elevation: 3000.0,
            node_ids: vec![],
            diaphragms: vec![],
            seismic_weight: None,
        });
        m.materials.push(Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "S400".into(),
            young: 205000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
            fc: None,
            fy: Some(235.0),
        });
        m.sections.push(Section {
            id: SectionId(0),
            name: "C&1<2".into(), // エスケープ確認用
            area: 1.2345e4,
            iy: 1.0e8,
            iz: 2.0e8,
            j: 3.0e6,
            depth: 400.0,
            width: 200.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        });
        // 柱2本（鉛直）＋大梁1本（水平）。
        m.elements.push(ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec![NodeId(0), NodeId(2)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
        m.elements.push(ElementData {
            id: ElemId(1),
            kind: ElementKind::Beam,
            nodes: smallvec![NodeId(1), NodeId(3)],
            section: Some(SectionId(0)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
        m.elements.push(ElementData {
            id: ElemId(2),
            kind: ElementKind::Beam,
            nodes: smallvec![NodeId(2), NodeId(3)],
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
        });
        m.load_cases.push(LoadCase {
            kind: Default::default(),
            id: LoadCaseId(0),
            name: "L1".into(),
            nodal: vec![NodalLoad {
                node: NodeId(2),
                values: [10.5, 0.0, -3.0, 0.0, 0.0, 0.0],
            }],
            member: vec![],
        });
        m
    }

    /// 意味的に一致するか（対象スコープのフィールドのみ）。
    fn assert_semantic_eq(a: &Model, b: &Model) {
        assert_eq!(a.nodes.len(), b.nodes.len(), "node count");
        for (x, y) in a.nodes.iter().zip(&b.nodes) {
            assert_eq!(x.id, y.id);
            assert_eq!(x.coord, y.coord, "coord");
            assert_eq!(x.story, y.story, "story");
        }
        assert_eq!(a.stories.len(), b.stories.len());
        for (x, y) in a.stories.iter().zip(&b.stories) {
            assert_eq!(x.id, y.id);
            assert_eq!(x.name, y.name);
            assert_eq!(x.elevation, y.elevation);
        }
        assert_eq!(a.materials.len(), b.materials.len());
        for (x, y) in a.materials.iter().zip(&b.materials) {
            assert_eq!(x.id, y.id);
            assert_eq!(x.name, y.name);
            assert_eq!(x.young, y.young);
            assert_eq!(x.poisson, y.poisson);
            assert_eq!(x.fy, y.fy);
            assert_eq!(x.fc, y.fc);
        }
        assert_eq!(a.sections.len(), b.sections.len());
        for (x, y) in a.sections.iter().zip(&b.sections) {
            assert_eq!(x.id, y.id);
            assert_eq!(x.name, y.name, "section name (escape)");
            assert_eq!(x.area, y.area);
            assert_eq!(x.iy, y.iy);
            assert_eq!(x.iz, y.iz);
            assert_eq!(x.j, y.j);
            assert_eq!(x.depth, y.depth);
            assert_eq!(x.width, y.width);
        }
        assert_eq!(a.elements.len(), b.elements.len());
        for (x, y) in a.elements.iter().zip(&b.elements) {
            assert_eq!(x.id, y.id);
            assert_eq!(x.nodes.as_slice(), y.nodes.as_slice(), "connectivity");
            assert_eq!(x.section, y.section);
            assert_eq!(x.material, y.material);
            assert_eq!(
                x.local_axis.ref_vector, y.local_axis.ref_vector,
                "ref_vector"
            );
        }
        assert_eq!(a.load_cases.len(), b.load_cases.len());
        for (x, y) in a.load_cases.iter().zip(&b.load_cases) {
            assert_eq!(x.id, y.id);
            assert_eq!(x.name, y.name);
            assert_eq!(x.nodal.len(), y.nodal.len());
            for (p, q) in x.nodal.iter().zip(&y.nodal) {
                assert_eq!(p.node, q.node);
                assert_eq!(p.values, q.values);
            }
        }
    }

    #[test]
    fn test_roundtrip_semantic() {
        let m = representative_model();
        let xml = export_stbridge(&m).expect("export");
        let m2 = import_stbridge(&xml).expect("import");
        assert_semantic_eq(&m, &m2);
    }

    #[test]
    fn test_roundtrip_twice_stable() {
        // import→export→再import で安定（DoD §8.3）。
        let m = representative_model();
        let xml1 = export_stbridge(&m).unwrap();
        let m2 = import_stbridge(&xml1).unwrap();
        let xml2 = export_stbridge(&m2).unwrap();
        assert_eq!(xml1, xml2, "export は冪等であるべき");
        let m3 = import_stbridge(&xml2).unwrap();
        assert_semantic_eq(&m2, &m3);
    }

    #[test]
    fn test_column_girder_classification() {
        let m = representative_model();
        let xml = export_stbridge(&m).unwrap();
        assert!(xml.contains("<StbColumn "), "鉛直材は StbColumn");
        assert!(xml.contains("<StbGirder "), "水平材は StbGirder");
    }

    #[test]
    fn test_reject_non_stbridge() {
        let r = import_stbridge("<foo/>");
        assert!(matches!(r, Err(StbError::Version(_))));
    }

    #[test]
    fn test_reject_v1() {
        let r = import_stbridge("<ST_BRIDGE version=\"1.4.0\"><StbModel/></ST_BRIDGE>");
        assert!(matches!(r, Err(StbError::Version(_))));
    }

    #[test]
    fn test_imported_model_validates() {
        let m = representative_model();
        let xml = export_stbridge(&m).unwrap();
        let m2 = import_stbridge(&xml).unwrap();
        assert!(m2.validate().is_ok(), "取り込んだモデルは検証を通る");
    }
}
