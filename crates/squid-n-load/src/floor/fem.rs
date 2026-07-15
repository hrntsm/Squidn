//! 両端固定梁の固定端モーメント・せん断（CMQ）の閉形式公式。
//!
//! - [`fem_uniform`] — 等分布荷重の CMQ
//! - [`fem_triangle`] — 対称三角形荷重の CMQ
//! - [`fem_trapezoid`] — 対称台形荷重の CMQ（閉形式評価）

use super::types::Cmq;

pub(crate) fn fem_uniform(w: f64, l: f64) -> Cmq {
    Cmq {
        c_i: w * l * l / 12.0,
        c_j: -w * l * l / 12.0,
        q_i: w * l / 2.0,
        q_j: w * l / 2.0,
    }
}

pub(crate) fn fem_triangle(w0: f64, l: f64) -> Cmq {
    Cmq {
        c_i: 5.0 * w0 * l * l / 96.0,
        c_j: -5.0 * w0 * l * l / 96.0,
        q_i: w0 * l / 4.0,
        q_j: w0 * l / 4.0,
    }
}

/// 対称台形荷重（両端 a 区間で 0→w0 に線形立上り、中央 L−2a 区間は等高 w0）の
/// 両端固定梁の固定端モーメント・せん断。
/// 固定端モーメントは閉形式 FEM = (1/L²)∫₀ᴸ w(x)·x·(L−x)² dx を評価して求める。
/// 検算: a→L/2 で対称三角形 5w0L²/96、a→0 で等分布 w0L²/12 に一致する。
#[allow(unused_variables)]
pub(crate) fn fem_trapezoid(w0: f64, a: f64, b: f64, l: f64) -> Cmq {
    // ∫ x(L-x)² dx の不定積分
    let g = |x: f64| l * l * x * x / 2.0 - 2.0 * l * x * x * x / 3.0 + x.powi(4) / 4.0;
    // 両端の三角形立上り区間（[0,a] と [L-a,L]）の寄与（/a を約分済みの閉形式）
    let i_ends = w0 * l * a * a * (l / 3.0 - a / 4.0);
    // 中央の等分布区間 [a, L-a] の寄与
    let i_mid = w0 * (g(l - a) - g(a));
    let fem = (i_ends + i_mid) / (l * l);
    // 総荷重 = 台形面積（単位幅あたり）= w0·(L−a)。せん断は対称なので両端で W/2。
    let total = w0 * (l - a);
    Cmq {
        c_i: fem,
        c_j: -fem,
        q_i: total / 2.0,
        q_j: total / 2.0,
    }
}
