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
//! - **既定（`Raw`）の断面は実 ST-Bridge の形鋼ライブラリ参照（StbSecColumn_S 等）ではなく、
//!   内部モデルの物性をそのまま持つ `StbSecRaw` で表現する**（正準モデルを唯一の真実とする方針）。
//!   BIM/他ソフト向けに標準要素で書き出す `Standard` モードは下記「断面書き出しモード」を参照
//!   （ただし import は `Raw`＝`StbSecRaw` のみ読み戻す）。
//! - 床・ブレース・剛域・端部接合等の詳細。
//!
//! 一次資料: ST-Bridge 公式スキーマ（XML 2.0 系）。要素・属性名はこれに準拠（subset）。
//!
//! # 断面書き出しモード
//! [`export_stbridge`] は既定で `StbSecRaw`（物性直持ち・往復可能）を書き出す。
//! [`export_stbridge_with`] に [`SectionExportMode::Standard`] を渡すと、ST-Bridge 標準の
//! 断面要素（`StbSecColumn_S` 等）＋形鋼ライブラリ（`StbSecSteel`）で書き出す（BIM/他ソフト向け）。
//!
//! # モジュール構成（1 ファイル 1 責務）
//! - [`export`] — 直列化（内部モデル → ST-Bridge XML）。
//! - [`section_std`] — 標準フォーマット断面の直列化（`Standard` モード）。
//! - [`import`] — パース（ST-Bridge XML → 内部モデル）。

mod export;
mod import;
mod section_std;

pub use export::{export_stbridge, export_stbridge_with};
pub use import::import_stbridge;

/// ST-Bridge 書き出し時の断面表現モード。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SectionExportMode {
    /// 物性を独自要素 `StbSecRaw` として直接保持する。`import_stbridge` で往復可能。既定。
    #[default]
    Raw,
    /// ST-Bridge 標準の断面要素（`StbSecColumn_S`/`StbSecBeam_S`/`StbSecColumn_RC`/
    /// `StbSecBeam_RC`）＋形鋼ライブラリ（`StbSecSteel`）で書き出す。BIM/他ソフトとの
    /// 連携向け。形状（`Section.shape`）を持たない断面や SRC・CFT・耐震壁は `StbSecRaw`
    /// へフォールバックする。
    ///
    /// **outbound 専用**: `import_stbridge` は標準断面要素を解釈しないため、本モードで
    /// 書き出したファイルを読み戻すと断面が復元されず（フォールバックした `StbSecRaw` を除く）、
    /// 部材の断面参照がダングリングになり `Model::validate` に通らない。Squid-N で再利用する
    /// モデルは [`Raw`](Self::Raw) か `.scz` で保存すること。
    Standard,
}

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

    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

    fn rebar() -> RcRebar {
        RcRebar {
            main_x: BarSet {
                count: 3,
                dia: 22.0,
                layers: 1,
            },
            main_y: BarSet {
                count: 3,
                dia: 22.0,
                layers: 1,
            },
            cover: 40.0,
            shear: ShearBar {
                dia: 10.0,
                pitch: 100.0,
                legs: 2,
                grade: None,
            },
        }
    }

    fn member(id: u32, kind_col: bool, sec: u32) -> ElementData {
        // kind_col=true は鉛直（柱）、false は水平（梁）になるよう節点を選ぶ。
        let (a, b) = if kind_col {
            (NodeId(0), NodeId(2)) // 鉛直
        } else {
            (NodeId(2), NodeId(3)) // 水平
        };
        ElementData {
            id: ElemId(id),
            kind: ElementKind::Beam,
            nodes: smallvec![a, b],
            section: Some(SectionId(sec)),
            material: Some(MaterialId(0)),
            local_axis: LocalAxis {
                ref_vector: [0.0, 1.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        }
    }

    /// 4 節点だけ持つ骨組（部材・断面は各テストで差し込む）。
    fn frame_nodes() -> Model {
        let mut m = Model::default();
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
                story: None,
            });
        }
        m.materials.push(Material {
            concrete_class: Default::default(),
            id: MaterialId(0),
            name: "SN400B".into(),
            young: 205000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
            fc: None,
            fy: Some(235.0),
        });
        m
    }

    /// Raw モード（既定）は従来どおり StbSecRaw を出力し、標準要素は出さない。
    #[test]
    fn test_raw_mode_unchanged() {
        let mut m = frame_nodes();
        let h = SectionShape::SteelH {
            height: 400.0,
            width: 200.0,
            web_thick: 8.0,
            flange_thick: 13.0,
        };
        m.sections.push(h.to_section(SectionId(0), "H1".into()));
        m.elements.push(member(0, true, 0));

        let xml = export_stbridge(&m).unwrap();
        assert!(xml.contains("<StbSecRaw "), "Raw モードは StbSecRaw");
        assert!(
            !xml.contains("StbSecColumn_S"),
            "Raw モードは標準要素を出さない"
        );
        assert!(
            !xml.contains("StbSecSteel"),
            "Raw モードは形鋼ライブラリを出さない"
        );
        // export_stbridge_with(Raw) と等価。
        assert_eq!(
            xml,
            export_stbridge_with(&m, SectionExportMode::Raw).unwrap()
        );
    }

    /// 標準モード: 鋼 H 断面が形鋼ライブラリ参照付きの StbSecColumn_S として出力される。
    #[test]
    fn test_standard_mode_steel_column() {
        let mut m = frame_nodes();
        let h = SectionShape::SteelH {
            height: 400.0,
            width: 200.0,
            web_thick: 8.0,
            flange_thick: 13.0,
        };
        m.sections.push(h.to_section(SectionId(0), "C1".into()));
        m.elements.push(member(0, true, 0)); // 柱

        let xml = export_stbridge_with(&m, SectionExportMode::Standard).unwrap();
        assert!(xml.contains("<StbSecColumn_S "), "鋼柱は StbSecColumn_S");
        assert!(xml.contains("<StbSecSteel>"), "形鋼ライブラリを出す");
        assert!(
            xml.contains("<StbSecRoll-H name=\"H-400x200x8x13\""),
            "H 形鋼図形が定義される: {xml}"
        );
        assert!(
            xml.contains("shape=\"H-400x200x8x13\""),
            "断面が図形名を参照する"
        );
        assert!(
            !xml.contains("<StbSecRaw "),
            "形状がある鋼断面は Raw にしない"
        );
    }

    /// 標準モード: RC 矩形が梁として使われると StbSecBeam_RC（幾何）で出力される。
    #[test]
    fn test_standard_mode_rc_beam() {
        let mut m = frame_nodes();
        let rc = SectionShape::RcRect {
            b: 400.0,
            d: 700.0,
            rebar: rebar(),
        };
        m.sections.push(rc.to_section(SectionId(0), "G1".into()));
        m.elements.push(member(0, false, 0)); // 梁

        let xml = export_stbridge_with(&m, SectionExportMode::Standard).unwrap();
        assert!(xml.contains("<StbSecBeam_RC "), "RC 梁は StbSecBeam_RC");
        assert!(
            xml.contains("<StbSecBeam_RC_Straight width=\"400\" depth=\"700\"/>"),
            "矩形図形が幅・せいで出力される: {xml}"
        );
    }

    /// 標準モード: 柱と梁で共有される鋼断面は 2 要素に分割され、部材の id_section が
    /// それぞれ別 id を指す。
    #[test]
    fn test_standard_mode_shared_section_split() {
        let mut m = frame_nodes();
        let h = SectionShape::SteelH {
            height: 300.0,
            width: 150.0,
            web_thick: 6.5,
            flange_thick: 9.0,
        };
        m.sections.push(h.to_section(SectionId(0), "S1".into()));
        m.elements.push(member(0, true, 0)); // 柱が section 0 を使用
        m.elements.push(member(1, false, 0)); // 梁も section 0 を使用（共有）

        let xml = export_stbridge_with(&m, SectionExportMode::Standard).unwrap();
        assert!(xml.contains("<StbSecColumn_S "), "柱用に StbSecColumn_S");
        assert!(xml.contains("<StbSecBeam_S "), "梁用に StbSecBeam_S");
        // 形鋼図形は 1 つに重複排除される。
        assert_eq!(
            xml.matches("<StbSecRoll-H ").count(),
            1,
            "形鋼図形は重複排除される"
        );
        // 柱は id_section=0、梁は分割された新 id（=1）を参照する。
        assert!(
            xml.contains("<StbColumn ") && xml.contains("id_section=\"0\""),
            "柱は元の断面 id を参照: {xml}"
        );
        assert!(
            xml.contains("<StbGirder ") && xml.contains("id_section=\"1\""),
            "梁は分割された新しい断面 id を参照: {xml}"
        );
    }

    /// 標準モード: 形状を持たない断面（SRC/CFT/未定義含む）は StbSecRaw へフォールバックする。
    #[test]
    fn test_standard_mode_fallback_raw_for_shapeless() {
        let mut m = frame_nodes();
        m.sections.push(Section {
            id: SectionId(0),
            name: "X1".into(),
            area: 1.0e4,
            iy: 1.0e8,
            iz: 1.0e8,
            j: 1.0e6,
            depth: 300.0,
            width: 300.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        });
        m.elements.push(member(0, true, 0));

        let xml = export_stbridge_with(&m, SectionExportMode::Standard).unwrap();
        assert!(
            xml.contains("<StbSecRaw "),
            "形状の無い断面は Raw にフォールバック"
        );
    }
}
