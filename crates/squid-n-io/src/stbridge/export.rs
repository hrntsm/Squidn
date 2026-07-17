//! ST-Bridge 直列化（Export）。設計書 §12.5。
//!
//! - [`export_stbridge`] — 内部モデルを ST-Bridge 2.0（subset）XML 文字列へ出力する（既定＝Raw）。
//! - [`export_stbridge_with`] — 断面表現モードを指定して出力する。
//! - [`fmt`] — 整数値は小数点なし、それ以外は既定の f64 表記で整形する（`pub(super)`）。
//! - [`opt`] — `Option<f64>` を空文字列または [`fmt`] で整形する（priv）。
//! - [`esc`] — XML 特殊文字をエスケープする（`pub(super)`）。

use super::section_std::standard_sections;
use super::{SectionExportMode, StbError, STB_VERSION};
use squid_n_core::model::{ElementKind, Model};
use std::collections::HashMap;

/// 内部モデルを ST-Bridge 2.0（subset）XML 文字列へ出力する（断面は既定の
/// [`SectionExportMode::Raw`]＝`StbSecRaw` 物性直持ち）。
pub fn export_stbridge(model: &Model) -> Result<String, StbError> {
    export_stbridge_with(model, SectionExportMode::Raw)
}

/// 断面表現モードを指定して ST-Bridge 2.0（subset）XML を出力する。
///
/// - [`SectionExportMode::Raw`]: 物性を `StbSecRaw` で直接持つ（import 往復可能）。
/// - [`SectionExportMode::Standard`]: ST-Bridge 標準の断面要素＋形鋼ライブラリで書き出す
///   （BIM/他ソフト向け。柱/梁で共有する断面は分割され、部材の `id_section` を張り替える）。
pub fn export_stbridge_with(model: &Model, mode: SectionExportMode) -> Result<String, StbError> {
    // 断面ブロックと、部材参照（id_section）の張り替えマップをモードごとに用意する。
    let (sections_body, col_map, beam_map) = match mode {
        SectionExportMode::Raw => raw_sections(model),
        SectionExportMode::Standard => {
            let std = standard_sections(model);
            (std.sections_xml, std.col_map, std.beam_map)
        }
    };

    let mut s = String::new();
    s.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    s.push_str(&format!("<ST_BRIDGE version=\"{STB_VERSION}\">\n"));
    s.push_str("  <StbModel>\n");

    // 節点
    s.push_str("    <StbNodes>\n");
    for n in &model.nodes {
        let story = n.story.map(|s| s.0 as i64).unwrap_or(-1);
        s.push_str(&format!(
            "      <StbNode id=\"{}\" x=\"{}\" y=\"{}\" z=\"{}\" story=\"{}\"/>\n",
            n.id.0,
            fmt(n.coord[0]),
            fmt(n.coord[1]),
            fmt(n.coord[2]),
            story
        ));
    }
    s.push_str("    </StbNodes>\n");

    // 層
    s.push_str("    <StbStories>\n");
    for st in &model.stories {
        s.push_str(&format!(
            "      <StbStory id=\"{}\" name=\"{}\" height=\"{}\"/>\n",
            st.id.0,
            esc(&st.name),
            fmt(st.elevation)
        ));
    }
    s.push_str("    </StbStories>\n");

    // 材料
    s.push_str("    <StbMaterials>\n");
    for m in &model.materials {
        s.push_str(&format!(
            "      <StbMaterial id=\"{}\" name=\"{}\" young=\"{}\" poisson=\"{}\" density=\"{}\" shear=\"{}\" fc=\"{}\" fy=\"{}\"/>\n",
            m.id.0,
            esc(&m.name),
            fmt(m.young),
            fmt(m.poisson),
            fmt(m.density),
            opt(m.shear),
            opt(m.fc),
            opt(m.fy),
        ));
    }
    s.push_str("    </StbMaterials>\n");

    // 断面（モードに応じて Raw / Standard を切り替え。上で生成済み）
    s.push_str("    <StbSections>\n");
    s.push_str(&sections_body);
    s.push_str("    </StbSections>\n");

    // 部材（柱＝鉛直／大梁＝水平／ブレース＝斜材）
    s.push_str("    <StbMembers>\n");
    for e in &model.elements {
        if e.nodes.len() != 2 {
            continue;
        }
        let n0 = &model.nodes[e.nodes[0].index()];
        let n1 = &model.nodes[e.nodes[1].index()];
        let mat = e.material.map(|m| m.0 as i64).unwrap_or(-1);
        let r = e.local_axis.ref_vector;
        match e.kind {
            ElementKind::Beam => {
                let dz = (n1.coord[2] - n0.coord[2]).abs();
                let dx = n1.coord[0] - n0.coord[0];
                let dy = n1.coord[1] - n0.coord[1];
                let len = (dx * dx + dy * dy + dz * dz).sqrt();
                let is_col = len > 1e-12 && dz / len > 0.707;
                // 断面 id は柱／梁で分割され得るため、役割ごとの張り替えマップを引く
                // （見つからなければ内部 id をそのまま使う）。
                let role_map = if is_col { &col_map } else { &beam_map };
                let sec = e
                    .section
                    .map(|s| role_map.get(&s.0).copied().unwrap_or(s.0) as i64)
                    .unwrap_or(-1);
                if is_col {
                    // 下端→上端で揃える
                    let (bot, top) = if n0.coord[2] <= n1.coord[2] {
                        (e.nodes[0], e.nodes[1])
                    } else {
                        (e.nodes[1], e.nodes[0])
                    };
                    s.push_str(&format!(
                        "      <StbColumn id=\"{}\" id_node_bottom=\"{}\" id_node_top=\"{}\" id_section=\"{}\" id_material=\"{}\" rx=\"{}\" ry=\"{}\" rz=\"{}\"/>\n",
                        e.id.0, bot.0, top.0, sec, mat, fmt(r[0]), fmt(r[1]), fmt(r[2])
                    ));
                } else {
                    s.push_str(&format!(
                        "      <StbGirder id=\"{}\" id_node_start=\"{}\" id_node_end=\"{}\" id_section=\"{}\" id_material=\"{}\" rx=\"{}\" ry=\"{}\" rz=\"{}\"/>\n",
                        e.id.0, e.nodes[0].0, e.nodes[1].0, sec, mat, fmt(r[0]), fmt(r[1]), fmt(r[2])
                    ));
                }
            }
            ElementKind::Brace { tension_only } => {
                // ブレースの断面は柱/梁いずれの役割マップにも載り得るため両方を引く。
                let sec = e
                    .section
                    .map(|s| {
                        col_map
                            .get(&s.0)
                            .or_else(|| beam_map.get(&s.0))
                            .copied()
                            .unwrap_or(s.0) as i64
                    })
                    .unwrap_or(-1);
                s.push_str(&format!(
                    "      <StbBrace id=\"{}\" id_node_start=\"{}\" id_node_end=\"{}\" id_section=\"{}\" id_material=\"{}\" tension_only=\"{}\" rx=\"{}\" ry=\"{}\" rz=\"{}\"/>\n",
                    e.id.0, e.nodes[0].0, e.nodes[1].0, sec, mat, tension_only, fmt(r[0]), fmt(r[1]), fmt(r[2])
                ));
            }
            _ => continue,
        }
    }
    s.push_str("    </StbMembers>\n");

    // 荷重ケース（節点荷重）
    s.push_str("    <StbLoadCases>\n");
    for lc in &model.load_cases {
        s.push_str(&format!(
            "      <StbLoadCase id=\"{}\" name=\"{}\">\n",
            lc.id.0,
            esc(&lc.name)
        ));
        for nl in &lc.nodal {
            let v = nl.values;
            s.push_str(&format!(
                "        <StbNodalLoad id_node=\"{}\" fx=\"{}\" fy=\"{}\" fz=\"{}\" mx=\"{}\" my=\"{}\" mz=\"{}\"/>\n",
                nl.node.0, fmt(v[0]), fmt(v[1]), fmt(v[2]), fmt(v[3]), fmt(v[4]), fmt(v[5])
            ));
        }
        s.push_str("      </StbLoadCase>\n");
    }
    s.push_str("    </StbLoadCases>\n");

    s.push_str("  </StbModel>\n");
    s.push_str("</ST_BRIDGE>\n");
    Ok(s)
}

/// 既定（Raw）モードの `<StbSections>` 本体を生成する。断面は物性を直接持つ
/// `StbSecRaw` として書き出し、id マップは恒等（柱・梁とも内部 id をそのまま参照）。
fn raw_sections(model: &Model) -> (String, HashMap<u32, u32>, HashMap<u32, u32>) {
    let mut body = String::new();
    let mut map: HashMap<u32, u32> = HashMap::new();
    for sec in &model.sections {
        body.push_str(&format!(
            "      <StbSecRaw id=\"{}\" name=\"{}\" area=\"{}\" iy=\"{}\" iz=\"{}\" j=\"{}\" depth=\"{}\" width=\"{}\"/>\n",
            sec.id.0,
            esc(&sec.name),
            fmt(sec.area), fmt(sec.iy), fmt(sec.iz), fmt(sec.j),
            fmt(sec.depth), fmt(sec.width),
        ));
        map.insert(sec.id.0, sec.id.0);
    }
    (body, map.clone(), map)
}

pub(super) fn fmt(x: f64) -> String {
    // 整数値は小数点なしで、それ以外は既定の f64 表記で（往復で値が保たれる）。
    if x == x.trunc() && x.is_finite() {
        format!("{}", x as i64)
    } else {
        format!("{x}")
    }
}

fn opt(x: Option<f64>) -> String {
    match x {
        Some(v) => fmt(v),
        None => String::new(),
    }
}

pub(super) fn esc(s: &str) -> String {
    // & を最初に置換した後で制御空白を文字参照化する（後段で `&` を再エスケープしないため安全）。
    // タブ/改行/CR を文字参照にしないと、XML 属性値正規化（読込側 normalized_value）で
    // 空白 (#x20) に潰れ、属性値（例: 断面名・帯筋グレード）が往復で変化してしまう。
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\t', "&#9;")
        .replace('\n', "&#10;")
        .replace('\r', "&#13;")
}
