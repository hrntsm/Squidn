//! 鋼材の F 値・許容応力度（鋼構造設計規準 1973・構造規定。板厚 [mm] 区分対応の F 値）。
//!
//! - [`steel_f_value`] — F 値（完全一致、板厚区分対応）
//! - [`steel_f_value_prefix`] — F 値（前方一致、板厚区分対応）
//! - [`plate_thickness`] — F 値区分判定に用いる最大板厚
//! - [`steel_ft`] — 許容引張／曲げ応力度 ft
//! - [`steel_fs`] — 許容せん断応力度 fs
//! - [`big_lambda`] — 限界細長比 Λ
//! - [`steel_fc`] — 許容圧縮応力度 fc（座屈考慮）

use crate::LoadTerm;
use squid_n_core::model::Section;
use squid_n_core::section_shape::SectionShape;

/// 板厚 2 区分（`t<=40` / `40<t<=100`）の F 値を返す。
fn bucket2(t: f64, le40: f64, gt40: f64) -> f64 {
    if t <= 40.0 {
        le40
    } else {
        gt40
    }
}

/// 板厚 3 区分（`t<=40` / `40<t<=75` / `75<t<=100`）の F 値を返す（SM520 専用）。
fn bucket3(t: f64, le40: f64, le75: f64, gt75: f64) -> f64 {
    if t <= 40.0 {
        le40
    } else if t <= 75.0 {
        le75
    } else {
        gt75
    }
}

/// 鋼材グレード一覧（前方一致の探索対象。長い記号を優先するため
/// [`steel_f_value_prefix`] 側で文字数最大のものを選ぶ）。
const STEEL_GRADES: &[&str] = &[
    "SS400", "SS490", "SM400", "SM490", "SM520", "SN400", "SN490", "SA440", "STKR400", "STKR490",
    "BCR295", "BCP235", "BCP325",
];

/// 鋼材の F 値 [N/mm²]（完全一致、板厚 [mm] 区分対応）。
///
/// 厚さ 40mm 以下 / 40mm 超 100mm 以下の 2 区分（SM520 のみ 3 区分）。
/// 100mm を超える板厚は規定が無いため、本実装では最終区分値をそのまま用いる
/// （非保守的になり得るため実運用では要確認）。
///
/// 戻り値は F 値。長期許容引張・圧縮・曲げ `ft = F/1.5`、
/// 長期許容せん断 `fs = F/(1.5·√3)`。短期は長期の 1.5 倍（=F, F/√3）。
pub fn steel_f_value(grade: &str, thickness: f64) -> Option<f64> {
    match grade {
        "SS400" => Some(bucket2(thickness, 235.0, 215.0)),
        "SS490" => Some(bucket2(thickness, 275.0, 255.0)),
        "SM400" => Some(bucket2(thickness, 235.0, 215.0)),
        "SM490" => Some(bucket2(thickness, 325.0, 295.0)),
        "SM520" => Some(bucket3(thickness, 355.0, 335.0, 325.0)),
        "SN400" => Some(bucket2(thickness, 235.0, 215.0)),
        "SN490" => Some(bucket2(thickness, 325.0, 295.0)),
        "SA440" => Some(440.0),
        "STKR400" => Some(bucket2(thickness, 235.0, 215.0)),
        "STKR490" => Some(bucket2(thickness, 325.0, 295.0)),
        "BCR295" => Some(295.0),
        "BCP235" => Some(235.0),
        "BCP325" => Some(325.0),
        _ => None,
    }
}

/// 鋼材の F 値 [N/mm²]（前方一致、板厚 [mm] 区分対応）。
///
/// 材料名が板厚区分や強度記号等の接尾辞を伴う場合（例 "SN400B"→235、
/// "SM490A"→325）に対応するため、[`STEEL_GRADES`] の記号と材料名の前方一致で
/// 判定する。複数の記号が前方一致しうる場合は最も長い（＝最も具体的な）
/// ものを優先する（例: "SN490B" は "SN400" ではなく "SN490" に一致する）。
pub fn steel_f_value_prefix(name: &str, thickness: f64) -> Option<f64> {
    STEEL_GRADES
        .iter()
        .filter(|g| name.starts_with(*g))
        .max_by_key(|g| g.len())
        .and_then(|g| steel_f_value(g, thickness))
}

/// F 値の板厚区分判定に用いる最大板厚 [mm]。
///
/// `Section.shape` があれば形状の最大板厚、無ければ `Section.thickness`、
/// いずれも無ければ 40mm 以下区分として扱う。
pub fn plate_thickness(sec: &Section) -> f64 {
    if let Some(shape) = &sec.shape {
        match *shape {
            SectionShape::SteelH {
                web_thick,
                flange_thick,
                ..
            }
            | SectionShape::SteelChannel {
                web_thick,
                flange_thick,
                ..
            }
            | SectionShape::SteelTee {
                web_thick,
                flange_thick,
                ..
            } => return web_thick.max(flange_thick),
            SectionShape::SteelBox { thick, .. }
            | SectionShape::SteelAngle { thick, .. }
            | SectionShape::SteelPipe { thick, .. }
            | SectionShape::SteelFlatBar { thick, .. }
            | SectionShape::SteelLipChannel { thick, .. }
            | SectionShape::CftBox { thick, .. }
            | SectionShape::CftPipe { thick, .. } => return thick,
            // 中実丸鋼は板要素ではないため、板厚区分は全断面の径で判定する。
            SectionShape::SteelRoundBar { dia } => return dia,
            SectionShape::SrcRect {
                steel_web_thick,
                steel_flange_thick,
                ..
            } => return steel_web_thick.max(steel_flange_thick),
            SectionShape::RcRect { .. }
            | SectionShape::RcCircle { .. }
            | SectionShape::RcWall { .. } => {}
        }
    }
    sec.thickness.unwrap_or(40.0)
}

/// 長期・短期許容引張／曲げ（座屈無視）応力度 ft [N/mm²]。
///
/// `長期 = F/1.5`、`短期 = F`。
pub fn steel_ft(f: f64, term: LoadTerm) -> f64 {
    match term {
        LoadTerm::Long => f / 1.5,
        LoadTerm::Short => f,
    }
}

/// 長期・短期許容せん断応力度 fs [N/mm²]。
///
/// `長期 = F/(1.5·√3)`、`短期 = F/√3`。
pub fn steel_fs(f: f64, term: LoadTerm) -> f64 {
    match term {
        LoadTerm::Long => f / (1.5 * 3.0_f64.sqrt()),
        LoadTerm::Short => f / 3.0_f64.sqrt(),
    }
}

/// 限界細長比 `Λ = 1500/√(F/1.5)`。
pub fn big_lambda(f: f64) -> f64 {
    1500.0 / (f / 1.5).sqrt()
}

/// 長期・短期許容圧縮応力度 fc [N/mm²]（座屈考慮、鋼構造設計規準 1973）。
///
/// 有効細長比 `λ = lk/i`、限界細長比 `Λ = 1500/√(F/1.5)`、
/// 安全率 `ν = 3/2 + (2/3)(λ/Λ)²` として:
/// - `λ ≤ Λ`: `fcL = F·(1 − 0.4(λ/Λ)²) / ν`
/// - `λ > Λ`: `fcL = (18/65)·F/(λ/Λ)²`
///
/// 短期は長期の 1.5 倍。`λ=0` のとき `fcL = F/1.5`（= ft 長期）と一致し、
/// `λ=Λ` で両分岐は連続する（`fcL = (18/65)F`）。
pub fn steel_fc(f: f64, lambda: f64, term: LoadTerm) -> f64 {
    let big_l = big_lambda(f);
    let r = if big_l > 1e-9 { lambda / big_l } else { 0.0 };
    let fc_long = if lambda <= big_l {
        let nu = 1.5 + (2.0 / 3.0) * r * r;
        f * (1.0 - 0.4 * r * r) / nu
    } else {
        (18.0 / 65.0) * f / (r * r)
    };
    match term {
        LoadTerm::Long => fc_long,
        LoadTerm::Short => fc_long * 1.5,
    }
}
