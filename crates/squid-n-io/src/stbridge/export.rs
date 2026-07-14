//! ST-Bridge 直列化（Export）。設計書 §12.5。
//!
//! - [`export_stbridge`] — 内部モデルを ST-Bridge 2.0（subset）XML 文字列へ出力する。
//! - [`fmt`] — 整数値は小数点なし、それ以外は既定の f64 表記で整形する（priv）。
//! - [`opt`] — `Option<f64>` を空文字列または [`fmt`] で整形する（priv）。
//! - [`esc`] — XML 特殊文字をエスケープする（priv）。

use super::{StbError, STB_VERSION};
use squid_n_core::model::{ElementKind, Model};

/// 内部モデルを ST-Bridge 2.0（subset）XML 文字列へ出力する。
pub fn export_stbridge(model: &Model) -> Result<String, StbError> {
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

    // 断面（subset: 物性を直接保持）
    s.push_str("    <StbSections>\n");
    for sec in &model.sections {
        s.push_str(&format!(
            "      <StbSecRaw id=\"{}\" name=\"{}\" area=\"{}\" iy=\"{}\" iz=\"{}\" j=\"{}\" depth=\"{}\" width=\"{}\"/>\n",
            sec.id.0,
            esc(&sec.name),
            fmt(sec.area), fmt(sec.iy), fmt(sec.iz), fmt(sec.j),
            fmt(sec.depth), fmt(sec.width),
        ));
    }
    s.push_str("    </StbSections>\n");

    // 部材（柱＝鉛直／大梁＝水平）
    s.push_str("    <StbMembers>\n");
    for e in &model.elements {
        if e.kind != ElementKind::Beam || e.nodes.len() != 2 {
            continue;
        }
        let n0 = &model.nodes[e.nodes[0].index()];
        let n1 = &model.nodes[e.nodes[1].index()];
        let dz = (n1.coord[2] - n0.coord[2]).abs();
        let dx = n1.coord[0] - n0.coord[0];
        let dy = n1.coord[1] - n0.coord[1];
        let len = (dx * dx + dy * dy + dz * dz).sqrt();
        let is_col = len > 1e-12 && dz / len > 0.707;
        let sec = e.section.map(|s| s.0 as i64).unwrap_or(-1);
        let mat = e.material.map(|m| m.0 as i64).unwrap_or(-1);
        let r = e.local_axis.ref_vector;
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

fn fmt(x: f64) -> String {
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

fn esc(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}
