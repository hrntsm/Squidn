//! 鉄筋の許容応力度・降伏点。
//!
//! - [`rebar_allowable_tension`] — 異形鉄筋の許容引張・圧縮応力度 ft
//! - [`rebar_allowable_shear`] — せん断補強筋の許容引張応力度 w_ft
//! - [`rebar_sigma_y`] — 主筋の降伏点 σy（終局曲げ ΣMy 算定用）

use squid_n_core::model::Material;

/// 異形鉄筋の許容引張・圧縮応力度 ft [N/mm²]。
///
/// SD345/SD390/SD490 は径 D29 以上（`dia >= 29.0`）で長期値が低減される
/// （215→195）。USD685（主筋として使う場合の高強度異形棒鋼）は技術評定値
/// どおり長期 215（径によらず、D29 以上の低減対象外）・短期 685 とする。
pub fn rebar_allowable_tension(grade: &str, dia: f64, long_term: bool) -> f64 {
    let g = grade.trim();
    if g == "USD685" {
        return if long_term { 215.0 } else { 685.0 };
    }
    if long_term {
        if g == "SR235" || g == "SR295" {
            155.0
        } else if g.starts_with("SD295") {
            195.0
        } else if g == "SD345" || g == "SD390" || g == "SD490" {
            if dia >= 29.0 {
                195.0
            } else {
                215.0
            }
        } else {
            195.0
        }
    } else if g == "SR235" {
        235.0
    } else if g == "SR295" || g.starts_with("SD295") {
        295.0
    } else if g == "SD345" {
        345.0
    } else if g == "SD390" {
        390.0
    } else if g == "SD490" {
        490.0
    } else {
        295.0
    }
}

/// せん断補強筋の許容引張応力度 w_ft [N/mm²]。
///
/// USD685 は技術評定値どおり長期 195・短期 590。SD490 短期はせん断のみ
/// F=390 に頭打ち。
pub fn rebar_allowable_shear(grade: &str, long_term: bool) -> f64 {
    let g = grade.trim();
    if g == "USD685" {
        return if long_term { 195.0 } else { 590.0 };
    }
    if long_term {
        if g == "SR235" {
            155.0
        } else {
            195.0
        }
    } else if g == "SR235" {
        // 短期は基準強度 F=235（令90条表。従来はフォールバック 295 に落ちて
        // F 値を 25% 超過する非保守側の誤りだった）。
        235.0
    } else if g == "SR295" || g.starts_with("SD295") {
        295.0
    } else if g == "SD345" {
        345.0
    } else if g == "SD390" {
        390.0
    } else if g == "SD490" {
        // F 値スケーリング: SD490 短期はせん断のみ F=390 に頭打ち。
        390.0
    } else {
        295.0
    }
}

/// 主筋の降伏点 σy [N/mm²]（終局曲げ ΣMy 算定用）。
///
/// `Material.fy` があればそれを、無ければ材料名（鉄筋グレード名）の数値部
/// （例 "SD345"→345）を、どちらも無ければ 345（SD345 相当）を用いる。
pub fn rebar_sigma_y(mat: &Material) -> f64 {
    if let Some(fy) = mat.fy {
        if fy > 0.0 {
            return fy;
        }
    }
    let digits: String = mat.name.chars().filter(|c| c.is_ascii_digit()).collect();
    digits
        .parse::<f64>()
        .ok()
        .filter(|v| *v > 0.0)
        .unwrap_or(345.0)
}
