//! 単純支持梁の静定反力。
//!
//! - [`static_reactions`] — 部材荷重の等価節点重量（両端反力）

use super::*;

/// 単純支持梁（節点間距離 `len` を支間とする静定梁）の等価節点重量（両端反力）。
///
/// §1.4: 令88条の地震用重量算定では、地震用節点重量を「大梁の CMoQo の計算で
/// 求めた梁せん断力（＝ Q0、単純梁反力）」とする実務的取扱いによる。単純梁の反力は集中荷重・
/// 分布荷重いずれも静定なので、両端 1/2 の一律配分ではなく荷重位置に応じた
/// 反力比で配分する（対称荷重では結果的に 1/2 ずつになる）。
///
/// - 集中荷重 `Point{a,p}`: `R_i = p(L-a)/L`, `R_j = p·a/L`
/// - 分布荷重 `Distributed{a,b,w1,w2}`: 合計 `W=(w1+w2)/2·(b-a)`、
///   重心位置 `x̄ = a + (b-a)(w1+2w2)/(3(w1+w2))`（`w1+w2=0` は区間中点）、
///   `R_j = W·x̄/L`, `R_i = W - R_j`
///
/// 戻り値は `(R_i, R_j)`。`len <= 0` は `(0, 0)`。
pub(crate) fn static_reactions(kind: &MemberLoadKind, len: f64) -> (f64, f64) {
    if len <= 0.0 {
        return (0.0, 0.0);
    }
    match *kind {
        MemberLoadKind::Point { a, p } => {
            let a = a.clamp(0.0, len);
            let ri = p * (len - a) / len;
            let rj = p * a / len;
            (ri, rj)
        }
        MemberLoadKind::Distributed { a, b, w1, w2 } => {
            let a = a.max(0.0);
            let b = b.min(len);
            if b <= a {
                return (0.0, 0.0);
            }
            let span = b - a;
            let total = (w1 + w2) / 2.0 * span;
            let sum_w = w1 + w2;
            let xbar = if sum_w.abs() < 1e-12 {
                a + span / 2.0
            } else {
                a + span * (w1 + 2.0 * w2) / (3.0 * sum_w)
            };
            let rj = total * xbar / len;
            let ri = total - rj;
            (ri, rj)
        }
    }
}
