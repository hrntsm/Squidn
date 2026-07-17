//! ST-Bridge パース（Import）。設計書 §12.5。
//!
//! [`import_stbridge`] は次の 2 系統の断面表現を読み取れる。
//! - **物性直持ち**（`StbSecRaw`）: Squid-N の既定書き出し（[`SectionExportMode::Raw`](super::SectionExportMode)）。
//! - **ST-Bridge 標準の断面要素**（`StbSecColumn_S`/`StbSecBeam_S`/`StbSecColumn_RC`/
//!   `StbSecBeam_RC`）＋形鋼ライブラリ（`StbSecSteel`）: BIM/他社ソフトや
//!   [`SectionExportMode::Standard`](super::SectionExportMode) の書き出し。形鋼名から内部の
//!   [`SectionShape`] を復元し、断面性能を再算定する。
//!
//! 標準断面は柱用（`StbSecColumn_*`）と梁用（`StbSecBeam_*`）に型分けされ、
//! Squid-N が Standard 書き出しで柱・梁共有断面を分割した結果として断面 id が
//! 文書順に整列しないことがある。取り込み後に断面 id を整列・再採番し、部材の
//! 断面参照（`id_section`）を張り替える。

use super::StbError;
use squid_n_core::ids::{ElemId, LoadCaseId, MaterialId, NodeId, SectionId, StoryId};
use squid_n_core::model::{
    ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LocalAxis, Material, Model,
    NodalLoad, Node, Section, Story,
};
use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};
use std::collections::HashMap;

/// 断面が持つ材料参照（ST-Bridge は材料を断面側に持つ）。
/// 数値 id（RC/CFT/SRC の `id_material`）または材料名（鋼の `strength_main` グレード）。
#[derive(Clone)]
enum SecMatRef {
    Id(u32),
    Grade(String),
}

/// 取り込み途中の断面（id 整列・形鋼名解決の前）。
struct PendingSec {
    file_id: u32,
    name: String,
    kind: PendingSecKind,
    /// 断面側に付いた材料参照（部材が id_material を持たないとき部材へ伝播する）。
    mat: Option<SecMatRef>,
}

enum PendingSecKind {
    /// 物性直持ち（`StbSecRaw`）。
    Raw {
        area: f64,
        iy: f64,
        iz: f64,
        j: f64,
        depth: f64,
        width: f64,
    },
    /// 形状が確定済み（RC 図形など）。
    Shape(SectionShape),
    /// 形鋼ライブラリ参照（後で名前解決する鋼断面）。
    SteelRef(Option<String>),
    /// CFT（充填鋼管）。充填鋼管の形鋼名を後で解決して CftBox/CftPipe を作る。
    CftRef(Option<String>),
    /// SRC（RC＋内蔵鉄骨）。コンクリート寸法・配筋・鋼種は確定済み、内蔵鉄骨は
    /// 形鋼名を後で解決する。
    SrcRef {
        b: f64,
        d: f64,
        rebar: RcRebar,
        steel_name: Option<String>,
        grade: String,
    },
}

/// 取り込み途中の部材の種別。
enum PendingMemberKind {
    Beam,
    Brace { tension_only: bool },
}

/// 取り込み途中の部材（id 正規化前。参照はすべて file id）。
struct PendingMember {
    kind: PendingMemberKind,
    n_i: u32,
    n_j: u32,
    section: Option<u32>,
    material: Option<u32>,
    /// `id_material` 属性がファイルに存在したか。存在する（=-1 含む）場合は部材が材料を
    /// 明示しているとみなし、断面材料の伝播を行わない（往復で None→Some 化を防ぐ）。
    /// 属性が無い（実 ST-Bridge 相当）ときのみ断面材料を伝播する。
    has_material_attr: bool,
    ref_vec: [f64; 3],
}

/// 取り込み途中の節点（id 正規化前）。
struct RawNode {
    file_id: u32,
    coord: [f64; 3],
    story: Option<u32>,
}

/// 取り込み途中の層（id 正規化前）。
struct RawStory {
    file_id: u32,
    name: String,
    elevation: f64,
}

/// 取り込み途中の材料（id 正規化前）。
struct RawMaterial {
    file_id: u32,
    name: String,
    young: f64,
    poisson: f64,
    density: f64,
    shear: Option<f64>,
    fc: Option<f64>,
    fy: Option<f64>,
}

/// 取り込み途中の荷重ケース（節点参照は file id）。
struct RawLoadCase {
    name: String,
    nodal: Vec<(u32, [f64; 6])>,
}

/// RC 断面の図形（配筋と組み合わせて `SectionShape` を確定する）。
enum RcGeom {
    Rect { b: f64, d: f64 },
    Circle { d: f64 },
}

/// 現在パース中の標準断面要素（子の図形・配筋要素を集める）。
enum CurSec {
    None,
    Steel {
        file_id: u32,
        name: String,
        shape_name: Option<String>,
        grade: Option<String>,
    },
    Rc {
        file_id: u32,
        name: String,
        geom: Option<RcGeom>,
        rebar: Option<RcRebar>,
        mat_id: Option<u32>,
    },
    Cft {
        file_id: u32,
        name: String,
        steel_name: Option<String>,
        mat_id: Option<u32>,
    },
    Src {
        file_id: u32,
        name: String,
        geom: Option<(f64, f64)>,
        rebar: Option<RcRebar>,
        steel_name: Option<String>,
        grade: String,
        mat_id: Option<u32>,
    },
}

/// 取り込み時に欠落・近似した内容の報告（データ欠損を顕在化させる）。
#[derive(Debug, Default, Clone)]
pub struct ImportReport {
    /// 人間可読の警告メッセージ（未対応要素のスキップ、断面欠落、参照解決失敗など）。
    pub warnings: Vec<String>,
}

impl ImportReport {
    /// 警告が 1 件も無い（＝取り込みで欠落が無かった）か。
    pub fn is_clean(&self) -> bool {
        self.warnings.is_empty()
    }
}

/// ST-Bridge の要素のうち Squid-N が未対応で、取り込み時に警告対象とするもの
/// （構造ラッパ等は対象外。実データを欠落させる部材・断面のみ列挙する）。
const UNSUPPORTED_ELEMENTS: &[&str] = &[
    // 部材（面要素・基礎・開口）
    "StbSlab",
    "StbWall",
    "StbFooting",
    "StbPile",
    "StbFoundationColumn",
    "StbStripFooting",
    "StbParapet",
    "StbOpen",
    // 断面（壁・スラブ・基礎・開口。鋼ブレース断面 StbSecBrace_S は対応済み）
    "StbSecWall_RC",
    "StbSecSlab_RC",
    "StbSecSlab_S",
    "StbSecSlabDeck",
    "StbSecFoundation_RC",
    "StbSecFoundationColumn_RC",
    "StbSecFoundationColumn_SRC",
    "StbSecFoundationColumn_CFT",
    "StbSecPile_RC",
    "StbSecPile_S",
    "StbSecPile_PC",
    "StbSecParapet_RC",
    "StbSecOpen_RC",
];

/// ST-Bridge 2.0 XML を内部モデルへ取り込む（欠落の報告は破棄する）。
pub fn import_stbridge(xml: &str) -> Result<Model, StbError> {
    import_stbridge_with_report(xml).map(|(m, _)| m)
}

/// ST-Bridge 2.0 XML を内部モデルへ取り込み、[`ImportReport`]（欠落・近似の警告）も返す。
pub fn import_stbridge_with_report(xml: &str) -> Result<(Model, ImportReport), StbError> {
    use quick_xml::events::Event;
    use quick_xml::reader::Reader;

    let mut reader = Reader::from_str(xml);
    reader.config_mut().trim_text(true);

    let mut version_ok = false;
    let mut warnings: Vec<String> = Vec::new();
    // 未対応要素はタグごとに件数を集計し、最後にまとめて 1 行の警告にする。
    let mut unsupported: HashMap<&'static str, u32> = HashMap::new();

    // 全要素を一旦 file id 付きの中間表現へ集め、パース後に id を 0 始まり連番へ
    // 正規化して参照を張り替える（他社ファイルの 1 始まり・歯抜け id に対応）。
    let mut raw_nodes: Vec<RawNode> = Vec::new();
    let mut raw_stories: Vec<RawStory> = Vec::new();
    let mut raw_materials: Vec<RawMaterial> = Vec::new();
    let mut raw_load_cases: Vec<RawLoadCase> = Vec::new();
    let mut pending_secs: Vec<PendingSec> = Vec::new();
    let mut pending_members: Vec<PendingMember> = Vec::new();
    let mut steel_lib: HashMap<String, SectionShape> = HashMap::new();
    let mut cur = CurSec::None;

    loop {
        let ev = reader
            .read_event()
            .map_err(|e| StbError::Parse(e.to_string()))?;
        match ev {
            Event::Eof => break,
            Event::Start(e) | Event::Empty(e) => {
                let name = e.name();
                let tag = String::from_utf8_lossy(name.as_ref()).to_string();
                let a = attrs(&e)?;
                match tag.as_str() {
                    "ST_BRIDGE" => {
                        let v = a.get("version").cloned().unwrap_or_default();
                        if !v.starts_with("2.") {
                            return Err(StbError::Version(v));
                        }
                        version_ok = true;
                    }
                    "StbNode" => {
                        let story = match get_i64(&a, "story") {
                            Some(s) if s >= 0 => Some(s as u32),
                            _ => None,
                        };
                        raw_nodes.push(RawNode {
                            file_id: get_u32(&a, "id")?,
                            // 座標属性は Squid-N 方言（小文字）と ST-Bridge 標準（大文字）の双方を許容。
                            coord: [
                                get_f64_any(&a, &["x", "X"])?,
                                get_f64_any(&a, &["y", "Y"])?,
                                get_f64_any(&a, &["z", "Z"])?,
                            ],
                            story,
                        });
                    }
                    "StbStory" => {
                        raw_stories.push(RawStory {
                            file_id: get_u32(&a, "id")?,
                            name: a.get("name").cloned().unwrap_or_default(),
                            elevation: get_f64_any(&a, &["height", "Z"])?,
                        });
                    }
                    "StbMaterial" => {
                        raw_materials.push(RawMaterial {
                            file_id: get_u32(&a, "id")?,
                            name: a.get("name").cloned().unwrap_or_default(),
                            young: get_f64(&a, "young")?,
                            poisson: get_f64(&a, "poisson")?,
                            density: get_f64(&a, "density")?,
                            shear: get_opt_f64(&a, "shear"),
                            fc: get_opt_f64(&a, "fc"),
                            fy: get_opt_f64(&a, "fy"),
                        });
                    }
                    // --- 断面: 物性直持ち（Raw） ---
                    "StbSecRaw" => {
                        pending_secs.push(PendingSec {
                            file_id: get_u32(&a, "id")?,
                            name: a.get("name").cloned().unwrap_or_default(),
                            kind: PendingSecKind::Raw {
                                area: get_f64(&a, "area")?,
                                iy: get_f64(&a, "iy")?,
                                iz: get_f64(&a, "iz")?,
                                j: get_f64(&a, "j")?,
                                depth: get_f64(&a, "depth").unwrap_or(0.0),
                                width: get_f64(&a, "width").unwrap_or(0.0),
                            },
                            mat: None,
                        });
                    }
                    // --- 断面: 標準要素（鋼。柱・梁・ブレース） ---
                    "StbSecColumn_S" | "StbSecBeam_S" | "StbSecBrace_S" => {
                        cur = CurSec::Steel {
                            file_id: get_u32(&a, "id")?,
                            name: a.get("name").cloned().unwrap_or_default(),
                            shape_name: None,
                            // 鋼種は形鋼参照（下）に付くことが多いが、要素側にあれば拾う。
                            grade: a.get("strength_main").cloned(),
                        };
                    }
                    // 鋼／CFT／SRC 断面の図形参照（`*_Same` / `*_Straight`）。`shape` 系属性から
                    // 形鋼名を、`strength_main` から鋼種を取り、現在の断面種別へ格納する。
                    tag if tag.starts_with("StbSecSteelColumn_")
                        || tag.starts_with("StbSecSteelBeam_")
                        || tag.starts_with("StbSecSteelBrace_") =>
                    {
                        let sname = a
                            .get("shape")
                            .or_else(|| a.get("shape_start"))
                            .or_else(|| a.get("shape_center"))
                            .or_else(|| a.get("shape_main"))
                            .cloned();
                        let gr = a.get("strength_main").cloned();
                        match &mut cur {
                            CurSec::Steel {
                                shape_name, grade, ..
                            } => {
                                if shape_name.is_none() {
                                    *shape_name = sname;
                                }
                                if grade.is_none() {
                                    *grade = gr;
                                }
                            }
                            CurSec::Cft { steel_name, .. } if steel_name.is_none() => {
                                *steel_name = sname
                            }
                            CurSec::Src { steel_name, .. } if steel_name.is_none() => {
                                *steel_name = sname
                            }
                            _ => {}
                        }
                    }
                    // --- 断面: 標準要素（RC） ---
                    "StbSecColumn_RC" | "StbSecBeam_RC" => {
                        cur = CurSec::Rc {
                            file_id: get_u32(&a, "id")?,
                            name: a.get("name").cloned().unwrap_or_default(),
                            geom: None,
                            rebar: None,
                            mat_id: mat_id_of(&a),
                        };
                    }
                    "StbSecColumn_RC_Rect" => {
                        if let CurSec::Rc { geom, .. } = &mut cur {
                            if geom.is_none() {
                                *geom = Some(RcGeom::Rect {
                                    b: get_f64_any(&a, &["width_X", "width_x"])?,
                                    d: get_f64_any(&a, &["width_Y", "width_y"])?,
                                });
                            }
                        }
                    }
                    "StbSecColumn_RC_Circle" => {
                        if let CurSec::Rc { geom, .. } = &mut cur {
                            if geom.is_none() {
                                *geom = Some(RcGeom::Circle {
                                    d: get_f64_any(&a, &["D", "d"])?,
                                });
                            }
                        }
                    }
                    "StbSecBeam_RC_Straight" => {
                        if let CurSec::Rc { geom, .. } = &mut cur {
                            if geom.is_none() {
                                *geom = Some(RcGeom::Rect {
                                    b: get_f64_any(&a, &["width", "width_X"])?,
                                    d: get_f64_any(&a, &["depth", "width_Y"])?,
                                });
                            }
                        }
                    }
                    // --- 断面: 標準要素（CFT） ---
                    "StbSecColumn_CFT" => {
                        cur = CurSec::Cft {
                            file_id: get_u32(&a, "id")?,
                            name: a.get("name").cloned().unwrap_or_default(),
                            steel_name: None,
                            mat_id: mat_id_of(&a),
                        };
                    }
                    // --- 断面: 標準要素（SRC） ---
                    "StbSecColumn_SRC" | "StbSecBeam_SRC" => {
                        cur = CurSec::Src {
                            file_id: get_u32(&a, "id")?,
                            name: a.get("name").cloned().unwrap_or_default(),
                            geom: None,
                            rebar: None,
                            steel_name: None,
                            grade: a
                                .get("strength_steel")
                                .or_else(|| a.get("strength_main_S"))
                                .cloned()
                                .unwrap_or_default(),
                            mat_id: mat_id_of(&a),
                        };
                    }
                    "StbSecColumn_SRC_Rect" => {
                        if let CurSec::Src { geom, .. } = &mut cur {
                            if geom.is_none() {
                                *geom = Some((
                                    get_f64_any(&a, &["width_X", "width_x"])?,
                                    get_f64_any(&a, &["width_Y", "width_y"])?,
                                ));
                            }
                        }
                    }
                    "StbSecBeam_SRC_Straight" => {
                        if let CurSec::Src { geom, .. } = &mut cur {
                            if geom.is_none() {
                                *geom = Some((
                                    get_f64_any(&a, &["width", "width_X"])?,
                                    get_f64_any(&a, &["depth", "width_Y"])?,
                                ));
                            }
                        }
                    }
                    // 配筋（RC / SRC の StbSecBarArrangement* 子要素）。現在の断面種別へ格納。
                    tag if tag.starts_with("StbSecBarColumn_")
                        || tag.starts_with("StbSecBarBeam_") =>
                    {
                        match &mut cur {
                            CurSec::Rc { rebar, .. } if rebar.is_none() => {
                                *rebar = Some(parse_rebar(&a))
                            }
                            CurSec::Src { rebar, .. } if rebar.is_none() => {
                                *rebar = Some(parse_rebar(&a))
                            }
                            _ => {}
                        }
                    }
                    // --- 形鋼ライブラリ ---
                    _ if tag.starts_with("StbSecRoll-")
                        || tag.starts_with("StbSecBuild-")
                        || tag == "StbSecPipe" =>
                    {
                        if let (Some(nm), Some(shape)) =
                            (a.get("name").cloned(), steel_shape_from(&tag, &a))
                        {
                            steel_lib.entry(nm).or_insert(shape);
                        }
                    }
                    // --- 部材 ---
                    "StbColumn" => {
                        let bot = get_u32(&a, "id_node_bottom")?;
                        let top = get_u32(&a, "id_node_top")?;
                        pending_members.push(make_member(&a, bot, top, PendingMemberKind::Beam)?);
                    }
                    "StbGirder" | "StbBeam" => {
                        let st = get_u32(&a, "id_node_start")?;
                        let en = get_u32(&a, "id_node_end")?;
                        pending_members.push(make_member(&a, st, en, PendingMemberKind::Beam)?);
                    }
                    // 間柱（鉛直材）は柱と同じく bottom/top を持つ（start/end も許容）。
                    "StbPost" => {
                        let bot = get_u32(&a, "id_node_bottom")
                            .or_else(|_| get_u32(&a, "id_node_start"))?;
                        let top =
                            get_u32(&a, "id_node_top").or_else(|_| get_u32(&a, "id_node_end"))?;
                        pending_members.push(make_member(&a, bot, top, PendingMemberKind::Beam)?);
                    }
                    "StbBrace" => {
                        let st = get_u32(&a, "id_node_start")?;
                        let en = get_u32(&a, "id_node_end")?;
                        let tension_only = a
                            .get("tension_only")
                            .map(|v| v == "true" || v == "1")
                            .unwrap_or(false);
                        pending_members.push(make_member(
                            &a,
                            st,
                            en,
                            PendingMemberKind::Brace { tension_only },
                        )?);
                    }
                    "StbLoadCase" => {
                        raw_load_cases.push(RawLoadCase {
                            name: a.get("name").cloned().unwrap_or_default(),
                            nodal: vec![],
                        });
                    }
                    "StbNodalLoad" => {
                        let node = get_u32(&a, "id_node")?;
                        let values = [
                            get_f64(&a, "fx").unwrap_or(0.0),
                            get_f64(&a, "fy").unwrap_or(0.0),
                            get_f64(&a, "fz").unwrap_or(0.0),
                            get_f64(&a, "mx").unwrap_or(0.0),
                            get_f64(&a, "my").unwrap_or(0.0),
                            get_f64(&a, "mz").unwrap_or(0.0),
                        ];
                        if let Some(lc) = raw_load_cases.last_mut() {
                            lc.nodal.push((node, values));
                        } else {
                            return Err(StbError::Parse("StbNodalLoad outside StbLoadCase".into()));
                        }
                    }
                    // 未対応の部材・断面（壁・スラブ・基礎等）はデータ欠落として集計する。
                    other => {
                        if let Some(&known) = UNSUPPORTED_ELEMENTS.iter().find(|&&u| u == other) {
                            *unsupported.entry(known).or_insert(0) += 1;
                        }
                    }
                }
            }
            Event::End(e) => {
                let name = e.name();
                let tag = String::from_utf8_lossy(name.as_ref()).to_string();
                match tag.as_str() {
                    "StbSecColumn_S" | "StbSecBeam_S" | "StbSecBrace_S" => {
                        if let CurSec::Steel {
                            file_id,
                            name,
                            shape_name,
                            grade,
                        } = std::mem::replace(&mut cur, CurSec::None)
                        {
                            pending_secs.push(PendingSec {
                                file_id,
                                name,
                                kind: PendingSecKind::SteelRef(shape_name),
                                mat: grade.map(SecMatRef::Grade),
                            });
                        }
                    }
                    "StbSecColumn_RC" | "StbSecBeam_RC" => {
                        if let CurSec::Rc {
                            file_id,
                            name,
                            geom,
                            rebar,
                            mat_id,
                        } = std::mem::replace(&mut cur, CurSec::None)
                        {
                            match geom {
                                Some(geom) => {
                                    // 配筋が無い（幾何のみの）ファイルは無筋相当の既定配筋で補う。
                                    let rebar = rebar.unwrap_or_else(default_rebar);
                                    let shape = match geom {
                                        RcGeom::Rect { b, d } => {
                                            SectionShape::RcRect { b, d, rebar }
                                        }
                                        RcGeom::Circle { d } => SectionShape::RcCircle { d, rebar },
                                    };
                                    pending_secs.push(PendingSec {
                                        file_id,
                                        name,
                                        kind: PendingSecKind::Shape(shape),
                                        mat: mat_id.map(SecMatRef::Id),
                                    });
                                }
                                None => warnings.push(format!(
                                    "RC 断面 (id={file_id}, name=\"{name}\") の図形を認識できず取り込めませんでした（テーパ・ハンチ等は未対応）"
                                )),
                            }
                        }
                    }
                    "StbSecColumn_CFT" => {
                        if let CurSec::Cft {
                            file_id,
                            name,
                            steel_name,
                            mat_id,
                        } = std::mem::replace(&mut cur, CurSec::None)
                        {
                            pending_secs.push(PendingSec {
                                file_id,
                                name,
                                kind: PendingSecKind::CftRef(steel_name),
                                mat: mat_id.map(SecMatRef::Id),
                            });
                        }
                    }
                    "StbSecColumn_SRC" | "StbSecBeam_SRC" => {
                        if let CurSec::Src {
                            file_id,
                            name,
                            geom,
                            rebar,
                            steel_name,
                            grade,
                            mat_id,
                        } = std::mem::replace(&mut cur, CurSec::None)
                        {
                            match geom {
                                Some((b, d)) => pending_secs.push(PendingSec {
                                    file_id,
                                    name,
                                    mat: mat_id.map(SecMatRef::Id),
                                    kind: PendingSecKind::SrcRef {
                                        b,
                                        d,
                                        rebar: rebar.unwrap_or_else(default_rebar),
                                        steel_name,
                                        grade,
                                    },
                                }),
                                None => warnings.push(format!(
                                    "SRC 断面 (id={file_id}, name=\"{name}\") の図形を認識できず取り込めませんでした"
                                )),
                            }
                        }
                    }
                    _ => {}
                }
            }
            _ => {}
        }
    }

    if !version_ok {
        return Err(StbError::Version(
            "missing ST_BRIDGE version 2.x root".into(),
        ));
    }

    let mut model = Model::default();

    // 各 id 空間を file id 昇順の 0 始まり連番へ正規化する（内部モデルの不変条件
    // 「配列添字 == id.index()」を満たすため）。返り値は file id → 新 index。
    let node_index = build_index(raw_nodes.iter().map(|n| n.file_id));
    let story_index = build_index(raw_stories.iter().map(|s| s.file_id));
    let material_index = build_index(raw_materials.iter().map(|m| m.file_id));

    raw_stories.sort_by_key(|s| s.file_id);
    for s in raw_stories {
        model.stories.push(Story {
            level_kind: Default::default(),
            structure: Default::default(),
            id: StoryId(story_index[&s.file_id]),
            name: s.name,
            elevation: s.elevation,
            node_ids: vec![],
            diaphragms: vec![],
            seismic_weight: None,
        });
    }

    raw_nodes.sort_by_key(|n| n.file_id);
    for n in raw_nodes {
        model.nodes.push(Node {
            id: NodeId(node_index[&n.file_id]),
            coord: n.coord,
            restraint: squid_n_core::dof::Dof6Mask::FREE,
            mass: None,
            story: n
                .story
                .and_then(|s| story_index.get(&s).copied())
                .map(StoryId),
        });
    }

    raw_materials.sort_by_key(|m| m.file_id);
    for m in raw_materials {
        model.materials.push(Material {
            concrete_class: Default::default(),
            id: MaterialId(material_index[&m.file_id]),
            name: m.name,
            young: m.young,
            poisson: m.poisson,
            density: m.density,
            shear: m.shear,
            fc: m.fc,
            fy: m.fy,
        });
    }

    // 断面側の材料参照を、部材への伝播用に file id → 正規化後 material index へ解決する。
    // ST-Bridge は材料を断面に持つため、部材が id_material を持たない（実 STB 相当の）
    // 場合に断面の材料を部材へ伝播する。数値 id は material_index、鋼のグレード名は
    // 同名の材料へ突き合わせる（同名複数は最初の一致）。
    let mut name_to_mat: HashMap<&str, u32> = HashMap::new();
    for mat in &model.materials {
        name_to_mat.entry(mat.name.as_str()).or_insert(mat.id.0);
    }
    let section_material: HashMap<u32, u32> = pending_secs
        .iter()
        .filter_map(|p| {
            let idx = match p.mat.as_ref()? {
                SecMatRef::Id(mid) => material_index.get(mid).copied(),
                SecMatRef::Grade(name) => name_to_mat.get(name.as_str()).copied(),
            };
            idx.map(|i| (p.file_id, i))
        })
        .collect();

    // 断面 id を整列・連番へ再割当てし、形鋼名を解決してモデルへ格納する。
    let section_index = build_sections(&mut model, pending_secs, &steel_lib, &mut warnings);

    // 部材を格納する（節点・断面・材料の参照を正規化後の index に張り替える）。
    // 参照先が存在しない部材はスキップし、断面/材料の欠落は None にしてダングリングを防ぐ。
    let mut skipped_members = 0u32;
    let mut dangling_section = 0u32;
    let mut dangling_material = 0u32;
    for m in pending_members {
        let (Some(&ni), Some(&nj)) = (node_index.get(&m.n_i), node_index.get(&m.n_j)) else {
            skipped_members += 1;
            continue;
        };
        // 断面参照: 実在しない id を指していれば警告して None にする（ダングリング防止）。
        let section = m.section.and_then(|fid| match section_index.get(&fid) {
            Some(&idx) => Some(SectionId(idx)),
            None => {
                dangling_section += 1;
                None
            }
        });
        // 材料は部材自身の id_material を優先。id_material 属性が無い（実 ST-Bridge 相当の）
        // ときのみ断面の材料を伝播する（属性がある部材の None を上書きしない）。
        let own_material = m.material.and_then(|fid| match material_index.get(&fid) {
            Some(&idx) => Some(idx),
            None => {
                dangling_material += 1;
                None
            }
        });
        let material = own_material
            .or_else(|| {
                if m.has_material_attr {
                    None
                } else {
                    m.section
                        .and_then(|sfid| section_material.get(&sfid).copied())
                }
            })
            .map(MaterialId);
        let id = ElemId(model.elements.len() as u32);
        // ブレースは軸材なので両端ピン、梁・柱は既定で剛接合とする（ST-Bridge は端部
        // 接合条件を持たないため取り込み後の既定値）。
        let (kind, end_cond) = match m.kind {
            PendingMemberKind::Beam => (
                ElementKind::Beam,
                [EndCondition::Fixed, EndCondition::Fixed],
            ),
            PendingMemberKind::Brace { tension_only } => (
                ElementKind::Brace { tension_only },
                [EndCondition::Pinned, EndCondition::Pinned],
            ),
        };
        model.elements.push(ElementData {
            id,
            kind,
            nodes: smallvec::smallvec![NodeId(ni), NodeId(nj)],
            section,
            material,
            local_axis: LocalAxis {
                ref_vector: m.ref_vec,
            },
            end_cond,
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
            plastic_zone: None,
            spring: None,
        });
    }

    if skipped_members > 0 {
        warnings.push(format!(
            "存在しない節点を参照する部材を {skipped_members} 件スキップしました"
        ));
    }
    if dangling_section > 0 {
        warnings.push(format!(
            "存在しない断面を参照する部材が {dangling_section} 件あり、断面リンクを外しました"
        ));
    }
    if dangling_material > 0 {
        warnings.push(format!(
            "存在しない材料を参照する部材が {dangling_material} 件あり、材料リンクを外しました"
        ));
    }

    // 荷重ケース（節点参照を正規化。存在しない節点への荷重は破棄）。
    let mut dropped_loads = 0u32;
    for (i, lc) in raw_load_cases.into_iter().enumerate() {
        let nodal = lc
            .nodal
            .into_iter()
            .filter_map(|(fid, values)| match node_index.get(&fid) {
                Some(&ni) => Some(NodalLoad {
                    node: NodeId(ni),
                    values,
                }),
                None => {
                    dropped_loads += 1;
                    None
                }
            })
            .collect();
        model.load_cases.push(LoadCase {
            kind: Default::default(),
            id: LoadCaseId(i as u32),
            name: lc.name,
            nodal,
            member: vec![],
        });
    }
    if dropped_loads > 0 {
        warnings.push(format!(
            "存在しない節点への節点荷重を {dropped_loads} 件破棄しました"
        ));
    }

    // 未対応要素の集計を 1 行の警告にまとめる（タグ名昇順で決定的に）。
    if !unsupported.is_empty() {
        let mut items: Vec<(&&str, &u32)> = unsupported.iter().collect();
        items.sort_by_key(|(tag, _)| **tag);
        let list = items
            .iter()
            .map(|(tag, n)| format!("{tag}×{n}"))
            .collect::<Vec<_>>()
            .join(", ");
        warnings.push(format!("未対応の要素をスキップしました: {list}"));
    }

    Ok((model, ImportReport { warnings }))
}

/// file id の集合を昇順・重複排除して 0 始まり連番へ写像する（file id → 新 index）。
fn build_index(ids: impl Iterator<Item = u32>) -> HashMap<u32, u32> {
    let mut sorted: Vec<u32> = ids.collect();
    sorted.sort_unstable();
    sorted.dedup();
    sorted
        .into_iter()
        .enumerate()
        .map(|(i, id)| (id, i as u32))
        .collect()
}

/// 保留していた断面を id 昇順に整列・連番へ再割当てし、形鋼名を解決して
/// `model.sections` を構築する。返り値は 元の file id → 再割当て後 index のマップ。
fn build_sections(
    model: &mut Model,
    mut pending: Vec<PendingSec>,
    steel_lib: &HashMap<String, SectionShape>,
    warnings: &mut Vec<String>,
) -> HashMap<u32, u32> {
    // file id 昇順で整列（Standard 書き出しは分割断面を文書順に整列させないため）。
    pending.sort_by_key(|s| s.file_id);

    let mut index_map: HashMap<u32, u32> = HashMap::new();
    for (idx, ps) in pending.into_iter().enumerate() {
        let new_id = SectionId(idx as u32);
        index_map.insert(ps.file_id, idx as u32);
        let section = match ps.kind {
            PendingSecKind::Raw {
                area,
                iy,
                iz,
                j,
                depth,
                width,
            } => Section {
                id: new_id,
                name: ps.name,
                area,
                iy,
                iz,
                j,
                depth,
                width,
                as_y: 0.0,
                as_z: 0.0,
                panel_thickness: None,
                thickness: None,
                shape: None,
            },
            PendingSecKind::Shape(shape) => shape.to_section(new_id, ps.name),
            PendingSecKind::SteelRef(shape_name) => {
                // 形鋼ライブラリに定義が無い参照は物性ゼロの断面として残す
                // （参照する部材の断面リンクを保つため。解析前に要確認）。
                match shape_name.and_then(|nm| steel_lib.get(&nm).cloned()) {
                    Some(shape) => shape.to_section(new_id, ps.name),
                    None => {
                        warnings.push(format!(
                            "鋼断面 (name=\"{}\") の形鋼参照を解決できず物性ゼロで取り込みました",
                            ps.name
                        ));
                        zero_section(new_id, ps.name)
                    }
                }
            }
            PendingSecKind::CftRef(steel_name) => {
                // 充填鋼管の形鋼（BOX/Pipe）を CFT 形状へ読み替える。
                let cft = steel_name
                    .and_then(|nm| steel_lib.get(&nm).cloned())
                    .and_then(|s| match s {
                        SectionShape::SteelBox {
                            height,
                            width,
                            thick,
                        } => Some(SectionShape::CftBox {
                            height,
                            width,
                            thick,
                        }),
                        SectionShape::SteelPipe { outer_dia, thick } => {
                            Some(SectionShape::CftPipe { outer_dia, thick })
                        }
                        _ => None,
                    });
                match cft {
                    Some(shape) => shape.to_section(new_id, ps.name),
                    None => {
                        warnings.push(format!(
                            "CFT 断面 (name=\"{}\") の充填鋼管参照を解決できず物性ゼロで取り込みました",
                            ps.name
                        ));
                        zero_section(new_id, ps.name)
                    }
                }
            }
            PendingSecKind::SrcRef {
                b,
                d,
                rebar,
                steel_name,
                grade,
            } => {
                // 内蔵鉄骨（H 形鋼）の寸法を解決する。未解決なら 0 とし、形状は保持する。
                let steel_dims = steel_name
                    .and_then(|nm| steel_lib.get(&nm).cloned())
                    .and_then(|s| match s {
                        SectionShape::SteelH {
                            height,
                            width,
                            web_thick,
                            flange_thick,
                        } => Some((height, width, web_thick, flange_thick)),
                        _ => None,
                    });
                if steel_dims.is_none() {
                    warnings.push(format!(
                        "SRC 断面 (name=\"{}\") の内蔵鉄骨参照を解決できず鉄骨寸法ゼロで取り込みました",
                        ps.name
                    ));
                }
                let (sh, sw, sweb, sfl) = steel_dims.unwrap_or((0.0, 0.0, 0.0, 0.0));
                SectionShape::SrcRect {
                    b,
                    d,
                    rebar,
                    steel_height: sh,
                    steel_width: sw,
                    steel_web_thick: sweb,
                    steel_flange_thick: sfl,
                    steel_grade: grade,
                }
                .to_section(new_id, ps.name)
            }
        };
        model.sections.push(section);
    }
    index_map
}

/// 物性ゼロ・形状なしの断面（形鋼名未解決などのフォールバック。解析前に要確認）。
fn zero_section(id: SectionId, name: String) -> Section {
    Section {
        id,
        name,
        area: 0.0,
        iy: 0.0,
        iz: 0.0,
        j: 0.0,
        depth: 0.0,
        width: 0.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    }
}

/// 形鋼ライブラリ要素（`StbSecRoll-H` 等）と属性から [`SectionShape`] を復元する。
fn steel_shape_from(tag: &str, a: &HashMap<String, String>) -> Option<SectionShape> {
    // 形鋼の寸法属性は A(せい/長辺)・B(幅/短辺)・t1(ウェブ)・t2(フランジ) を基本とする。
    let a_ = |keys: &[&str]| get_f64_any(a, keys).ok();
    match tag {
        t if t.ends_with("-H") => Some(SectionShape::SteelH {
            height: a_(&["A"])?,
            width: a_(&["B"])?,
            web_thick: a_(&["t1"])?,
            flange_thick: a_(&["t2"])?,
        }),
        t if t.ends_with("-BOX") => {
            let thick = a_(&["t", "t1"])?;
            Some(SectionShape::SteelBox {
                height: a_(&["A"])?,
                width: a_(&["B"])?,
                thick,
            })
        }
        "StbSecPipe" => Some(SectionShape::SteelPipe {
            outer_dia: a_(&["D", "A"])?,
            thick: a_(&["t", "t1"])?,
        }),
        t if t.ends_with("-L") => Some(SectionShape::SteelAngle {
            leg_a: a_(&["A"])?,
            leg_b: a_(&["B"])?,
            thick: a_(&["t1", "t"])?,
        }),
        t if t.ends_with("-C") => Some(SectionShape::SteelChannel {
            height: a_(&["A"])?,
            width: a_(&["B"])?,
            web_thick: a_(&["t1"])?,
            flange_thick: a_(&["t2"])?,
        }),
        t if t.ends_with("-T") => Some(SectionShape::SteelTee {
            height: a_(&["A"])?,
            width: a_(&["B"])?,
            web_thick: a_(&["t1"])?,
            flange_thick: a_(&["t2"])?,
        }),
        _ => None,
    }
}

/// ST-Bridge 標準断面（幾何のみ）から復元する RC 断面の既定配筋（無筋相当）。
/// 弾性断面性能は b・d のみで決まり配筋に依存しないため、往復での剛性は保たれる。
/// 配筋検定を要する場合は取り込み後に別途入力する必要がある。
fn default_rebar() -> RcRebar {
    let zero = BarSet {
        count: 0,
        dia: 0.0,
        layers: 0,
    };
    RcRebar {
        main_x: zero.clone(),
        main_y: zero,
        cover: 0.0,
        shear: ShearBar {
            dia: 0.0,
            pitch: 0.0,
            legs: 0,
            grade: None,
        },
    }
}

/// 鉄筋径の文字列を mm へ解釈する。数値ならそのまま、`D22`/`D10` のような呼び名は
/// 先頭の `D`/`d` を除いた数値を径とする（best-effort。厳密な JIS 公称径ではない）。
fn parse_bar_dia(v: &str) -> Option<f64> {
    if let Ok(x) = v.parse::<f64>() {
        return Some(x);
    }
    let t = v.trim();
    if let Some(rest) = t.strip_prefix(['D', 'd']) {
        return rest.trim().parse::<f64>().ok();
    }
    None
}

/// `StbSecBarArrangement*` の子要素の属性から [`RcRebar`] を復元する。
/// Squid-N の書き出し属性（`count_main_X`・`dia_main_X` 等）を優先しつつ、実 ST-Bridge で
/// 使われる名前（`D_main`・`N_main_X_1st`・`D_band` 等）や呼び名径（`D22`）も best-effort で
/// 拾う。欠落した属性は 0（無筋相当）を既定にする。弾性性能は b・d のみで決まるため、
/// 配筋の欠落・近似は往復での剛性に影響しない。
///
/// なお実 ST-Bridge の配筋スキーマ（段別本数 `N_main_X_1st`/`_2nd` の合算、呼び名→公称径の
/// 正確な対応、段数・かぶりの詳細）への完全準拠は今後の課題。
fn parse_rebar(a: &HashMap<String, String>) -> RcRebar {
    let f = |keys: &[&str]| -> f64 {
        for k in keys {
            if let Some(v) = a.get(*k) {
                if let Ok(x) = v.parse::<f64>() {
                    return x;
                }
            }
        }
        0.0
    };
    // 径（数値 or 呼び名 `D22`）。
    let dia = |keys: &[&str]| -> f64 {
        for k in keys {
            if let Some(v) = a.get(*k) {
                if let Some(x) = parse_bar_dia(v) {
                    return x;
                }
            }
        }
        0.0
    };
    let u = |keys: &[&str]| -> u32 {
        for k in keys {
            if let Some(v) = a.get(*k) {
                if let Ok(x) = v.parse::<u32>() {
                    return x;
                }
            }
        }
        0
    };
    // 段数は指定が無ければ 1 段扱い（配筋自体は 0 本でも段数 1 は無害）。
    let layers = |keys: &[&str]| -> u32 {
        for k in keys {
            if let Some(v) = a.get(*k) {
                if let Ok(x) = v.parse::<u32>() {
                    return x;
                }
            }
        }
        1
    };
    RcRebar {
        main_x: BarSet {
            count: u(&[
                "count_main_X",
                "N_main_X_1st",
                "count_main_top",
                "N_main_top",
            ]),
            dia: dia(&["dia_main_X", "dia_main", "D_main"]),
            layers: layers(&["count_main_layers_X"]),
        },
        main_y: BarSet {
            count: u(&[
                "count_main_Y",
                "N_main_Y_1st",
                "count_main_bottom",
                "N_main_bottom",
            ]),
            dia: dia(&["dia_main_Y", "dia_main", "D_main"]),
            layers: layers(&["count_main_layers_Y"]),
        },
        cover: f(&["cover", "kaburi"]),
        shear: ShearBar {
            dia: dia(&["dia_band", "D_band", "dia_stirrup", "dia_hoop"]),
            pitch: f(&["pitch_band", "pitch_stirrup", "pitch_hoop"]),
            legs: u(&[
                "count_band",
                "N_band_direction_X",
                "count_stirrup",
                "count_hoop",
            ]),
            grade: a
                .get("strength_band")
                .or_else(|| a.get("strength_bar_band"))
                .or_else(|| a.get("strength_main_band"))
                .cloned(),
        },
    }
}

/// 断面要素に付いた材料 id（`id_material` / `id_material_concrete` / `id_material_rc`）。
fn mat_id_of(a: &HashMap<String, String>) -> Option<u32> {
    get_i64(a, "id_material")
        .or_else(|| get_i64(a, "id_material_concrete"))
        .or_else(|| get_i64(a, "id_material_rc"))
        .filter(|v| *v >= 0)
        .map(|v| v as u32)
}

fn make_member(
    a: &HashMap<String, String>,
    n_i: u32,
    n_j: u32,
    kind: PendingMemberKind,
) -> Result<PendingMember, StbError> {
    let section = match get_i64(a, "id_section") {
        Some(s) if s >= 0 => Some(s as u32),
        _ => None,
    };
    let has_material_attr = a.contains_key("id_material");
    let material = match get_i64(a, "id_material") {
        Some(m) if m >= 0 => Some(m as u32),
        _ => None,
    };
    let ref_vec = [
        get_f64(a, "rx").unwrap_or(0.0),
        get_f64(a, "ry").unwrap_or(0.0),
        get_f64(a, "rz").unwrap_or(1.0),
    ];
    Ok(PendingMember {
        kind,
        n_i,
        n_j,
        section,
        material,
        has_material_attr,
        ref_vec,
    })
}

fn attrs(e: &quick_xml::events::BytesStart) -> Result<HashMap<String, String>, StbError> {
    let mut m = HashMap::new();
    for a in e.attributes() {
        let a = a.map_err(|err| StbError::Parse(err.to_string()))?;
        let key = String::from_utf8_lossy(a.key.as_ref()).to_string();
        let val = a
            .normalized_value(quick_xml::XmlVersion::Implicit1_0)
            .map_err(|err| StbError::Parse(err.to_string()))?
            .to_string();
        m.insert(key, val);
    }
    Ok(m)
}

fn get_f64(a: &HashMap<String, String>, k: &str) -> Result<f64, StbError> {
    a.get(k)
        .ok_or_else(|| StbError::Parse(format!("missing attr {k}")))?
        .parse::<f64>()
        .map_err(|_| StbError::Parse(format!("bad f64 attr {k}")))
}

/// 複数の候補キーのいずれかから f64 を取る（属性名の方言差を吸収する）。
fn get_f64_any(a: &HashMap<String, String>, keys: &[&str]) -> Result<f64, StbError> {
    for k in keys {
        if let Some(v) = a.get(*k) {
            return v
                .parse::<f64>()
                .map_err(|_| StbError::Parse(format!("bad f64 attr {k}")));
        }
    }
    Err(StbError::Parse(format!("missing attr {:?}", keys)))
}

fn get_opt_f64(a: &HashMap<String, String>, k: &str) -> Option<f64> {
    match a.get(k) {
        Some(v) if !v.is_empty() => v.parse::<f64>().ok(),
        _ => None,
    }
}

fn get_u32(a: &HashMap<String, String>, k: &str) -> Result<u32, StbError> {
    a.get(k)
        .ok_or_else(|| StbError::Parse(format!("missing attr {k}")))?
        .parse::<u32>()
        .map_err(|_| StbError::Parse(format!("bad u32 attr {k}")))
}

fn get_i64(a: &HashMap<String, String>, k: &str) -> Option<i64> {
    a.get(k).and_then(|v| v.parse::<i64>().ok())
}
