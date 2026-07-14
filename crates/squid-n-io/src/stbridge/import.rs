//! ST-Bridge パース（Import）。設計書 §12.5。
//!
//! - [`import_stbridge`] — ST-Bridge 2.0（subset）XML を内部モデルへ取り込む。
//! - [`make_member`] — 柱／大梁を共通の [`ElementData`] へ変換する（priv）。
//! - [`attrs`] — 開始タグの属性を `HashMap` へ収集する（priv）。
//! - [`get_f64`] — 必須 f64 属性を取得する（priv）。
//! - [`get_opt_f64`] — 省略可能な f64 属性を取得する（priv）。
//! - [`get_u32`] — 必須 u32 属性を取得する（priv）。
//! - [`get_i64`] — 省略可能な i64 属性を取得する（priv）。

use super::StbError;
use squid_n_core::ids::{ElemId, LoadCaseId, MaterialId, NodeId, SectionId, StoryId};
use squid_n_core::model::{
    ElementData, ElementKind, EndCondition, ForceRegime, LoadCase, LocalAxis, Material, Model,
    NodalLoad, Node, Section, Story,
};
use std::collections::HashMap;

/// ST-Bridge 2.0（subset）XML を内部モデルへ取り込む。
pub fn import_stbridge(xml: &str) -> Result<Model, StbError> {
    use quick_xml::events::Event;
    use quick_xml::reader::Reader;

    let mut reader = Reader::from_str(xml);
    reader.config_mut().trim_text(true);

    let mut model = Model::default();
    let mut load_cases: Vec<LoadCase> = Vec::new();
    let mut version_ok = false;

    loop {
        match reader
            .read_event()
            .map_err(|e| StbError::Parse(e.to_string()))?
        {
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
                            Some(s) if s >= 0 => Some(StoryId(s as u32)),
                            _ => None,
                        };
                        model.nodes.push(Node {
                            id: NodeId(get_u32(&a, "id")?),
                            coord: [get_f64(&a, "x")?, get_f64(&a, "y")?, get_f64(&a, "z")?],
                            restraint: squid_n_core::dof::Dof6Mask::FREE,
                            mass: None,
                            story,
                        });
                    }
                    "StbStory" => {
                        model.stories.push(Story {
                            level_kind: Default::default(),
                            structure: Default::default(),
                            id: StoryId(get_u32(&a, "id")?),
                            name: a.get("name").cloned().unwrap_or_default(),
                            elevation: get_f64(&a, "height")?,
                            node_ids: vec![],
                            diaphragms: vec![],
                            seismic_weight: None,
                        });
                    }
                    "StbMaterial" => {
                        model.materials.push(Material {
                            concrete_class: Default::default(),
                            id: MaterialId(get_u32(&a, "id")?),
                            name: a.get("name").cloned().unwrap_or_default(),
                            young: get_f64(&a, "young")?,
                            poisson: get_f64(&a, "poisson")?,
                            density: get_f64(&a, "density")?,
                            shear: get_opt_f64(&a, "shear"),
                            fc: get_opt_f64(&a, "fc"),
                            fy: get_opt_f64(&a, "fy"),
                        });
                    }
                    "StbSecRaw" => {
                        model.sections.push(Section {
                            id: SectionId(get_u32(&a, "id")?),
                            name: a.get("name").cloned().unwrap_or_default(),
                            area: get_f64(&a, "area")?,
                            iy: get_f64(&a, "iy")?,
                            iz: get_f64(&a, "iz")?,
                            j: get_f64(&a, "j")?,
                            depth: get_f64(&a, "depth").unwrap_or(0.0),
                            width: get_f64(&a, "width").unwrap_or(0.0),
                            as_y: 0.0,
                            as_z: 0.0,
                            panel_thickness: None,
                            thickness: None,
                            // ST-Bridge インポート断面はパラメトリック形状を持たない。
                            shape: None,
                        });
                    }
                    "StbColumn" => {
                        let bot = NodeId(get_u32(&a, "id_node_bottom")?);
                        let top = NodeId(get_u32(&a, "id_node_top")?);
                        model.elements.push(make_member(&a, bot, top)?);
                    }
                    "StbGirder" | "StbBeam" => {
                        let st = NodeId(get_u32(&a, "id_node_start")?);
                        let en = NodeId(get_u32(&a, "id_node_end")?);
                        model.elements.push(make_member(&a, st, en)?);
                    }
                    "StbLoadCase" => {
                        load_cases.push(LoadCase {
                            kind: Default::default(),
                            id: LoadCaseId(get_u32(&a, "id")?),
                            name: a.get("name").cloned().unwrap_or_default(),
                            nodal: vec![],
                            member: vec![],
                        });
                    }
                    "StbNodalLoad" => {
                        let nl = NodalLoad {
                            node: NodeId(get_u32(&a, "id_node")?),
                            values: [
                                get_f64(&a, "fx").unwrap_or(0.0),
                                get_f64(&a, "fy").unwrap_or(0.0),
                                get_f64(&a, "fz").unwrap_or(0.0),
                                get_f64(&a, "mx").unwrap_or(0.0),
                                get_f64(&a, "my").unwrap_or(0.0),
                                get_f64(&a, "mz").unwrap_or(0.0),
                            ],
                        };
                        if let Some(lc) = load_cases.last_mut() {
                            lc.nodal.push(nl);
                        } else {
                            return Err(StbError::Parse("StbNodalLoad outside StbLoadCase".into()));
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

    model.load_cases = load_cases;
    Ok(model)
}

fn make_member(
    a: &HashMap<String, String>,
    n_i: NodeId,
    n_j: NodeId,
) -> Result<ElementData, StbError> {
    use smallvec::smallvec;
    let section = match get_i64(a, "id_section") {
        Some(s) if s >= 0 => Some(SectionId(s as u32)),
        _ => None,
    };
    let material = match get_i64(a, "id_material") {
        Some(m) if m >= 0 => Some(MaterialId(m as u32)),
        _ => None,
    };
    let r = [
        get_f64(a, "rx").unwrap_or(0.0),
        get_f64(a, "ry").unwrap_or(0.0),
        get_f64(a, "rz").unwrap_or(1.0),
    ];
    Ok(ElementData {
        id: ElemId(get_u32(a, "id")?),
        kind: ElementKind::Beam,
        nodes: smallvec![n_i, n_j],
        section,
        material,
        local_axis: LocalAxis { ref_vector: r },
        end_cond: [EndCondition::Fixed, EndCondition::Fixed],
        force_regime: ForceRegime::Auto,
        rigid_zone: Default::default(),
        plastic_zone: None,
        spring: None,
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
