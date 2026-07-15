//! RC 矩形断面の配筋諸元（終局検定で共用する断面量）。
//!
//! - [`bar_set_area`] — 主筋セットの総断面積。
//! - [`hoop_pw`] — せん断補強筋比 pw。

use squid_n_core::section_shape::{BarSet, RcRebar};

/// 主筋セットの総断面積 [mm²]。
pub(super) fn bar_set_area(bar: &BarSet) -> f64 {
    bar.count as f64 * std::f64::consts::PI / 4.0 * bar.dia * bar.dia
}

/// せん断補強筋比 pw = (legs·π/4·dia²)/(b·pitch)。pitch ≤ 0 なら 0。
pub(super) fn hoop_pw(rebar: &RcRebar, b: f64) -> f64 {
    if rebar.shear.pitch <= 0.0 || b <= 0.0 {
        return 0.0;
    }
    let aw =
        rebar.shear.legs as f64 * std::f64::consts::PI / 4.0 * rebar.shear.dia * rebar.shear.dia;
    aw / (b * rebar.shear.pitch)
}
