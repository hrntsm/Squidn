//! 全塑性応力分布による断面力の中核積分プリミティブ。
//!
//! 支持点・軸耐力・軸力一定の全塑性モーメント・軸力一定スライス曲線を与える。
//! 曲面構築（[`super::surface`]）と M-φ（[`super::m_phi`]）の双方から使われる。

use super::types::PlasticFiber;

/// 全塑性応力分布による断面力（支持点）。
/// ひずみ速度方向 (e0, ky, kz) の符号のみで各ファイバの応力が決まる。
pub fn plastic_point(fibers: &[PlasticFiber], e0: f64, ky: f64, kz: f64) -> [f64; 3] {
    let mut n = 0.0;
    let mut my = 0.0;
    let mut mz = 0.0;
    for f in fibers {
        let eps = e0 - kz * f.y + ky * f.z;
        let sigma = if eps > 0.0 {
            f.sigma_t
        } else if eps < 0.0 {
            f.sigma_c
        } else {
            0.0
        };
        let sa = sigma * f.area;
        n += sa;
        my += sa * f.z;
        mz += -sa * f.y;
    }
    [n, my, mz]
}

/// 軸耐力 (圧縮 Nc ≤ 0, 引張 Nt ≥ 0)。
pub fn axial_capacity(fibers: &[PlasticFiber]) -> (f64, f64) {
    let nc: f64 = fibers.iter().map(|f| f.sigma_c * f.area).sum();
    let nt: f64 = fibers.iter().map(|f| f.sigma_t * f.area).sum();
    (nc, nt)
}

/// 曲げ方向 (ky, kz) を固定し、軸力が `n_target` となる全塑性中立軸位置での
/// モーメント (My, Mz) を返す。`n_target` が軸耐力範囲外なら `None`。
///
/// ファイバを中立軸からの距離順にソートし、圧縮側から引張側へ順次反転させて
/// 軸力を合わせる（中立軸上のファイバは部分的に応力を負担する）。
/// これは離散ファイバ集合の厳密な凸包（降伏曲面）上の点を与える。
pub fn plastic_moment_at_n(
    fibers: &[PlasticFiber],
    ky: f64,
    kz: f64,
    n_target: f64,
) -> Option<[f64; 2]> {
    let (nc, nt) = axial_capacity(fibers);
    if n_target < nc || n_target > nt || fibers.is_empty() {
        return None;
    }

    // 中立軸からのてこ距離 d = ky·z − kz·y。d が大きいファイバほど先に引張へ反転する。
    let mut order: Vec<usize> = (0..fibers.len()).collect();
    order.sort_by(|&a, &b| {
        let da = ky * fibers[a].z - kz * fibers[a].y;
        let db = ky * fibers[b].z - kz * fibers[b].y;
        db.partial_cmp(&da).unwrap_or(std::cmp::Ordering::Equal)
    });

    // 全断面圧縮から開始し、順に引張へ反転して n_target に到達させる。
    let mut n = nc;
    let mut my: f64 = fibers.iter().map(|f| f.sigma_c * f.area * f.z).sum();
    let mut mz: f64 = fibers.iter().map(|f| -f.sigma_c * f.area * f.y).sum();

    for &i in &order {
        let f = &fibers[i];
        let dn = (f.sigma_t - f.sigma_c) * f.area;
        if n + dn >= n_target {
            // このファイバが部分的に応力を負担して軸力が一致する
            let t = if dn > 0.0 { (n_target - n) / dn } else { 0.0 };
            let ds = t * (f.sigma_t - f.sigma_c) * f.area;
            my += ds * f.z;
            mz += -ds * f.y;
            return Some([my, mz]);
        }
        n += dn;
        my += (f.sigma_t - f.sigma_c) * f.area * f.z;
        mz += -(f.sigma_t - f.sigma_c) * f.area * f.y;
    }
    Some([my, mz])
}

/// 軸力一定 (`n_target`) での My–Mz 相関曲線を `n_pts` 点で返す（閉曲線、始点は繰り返さない）。
/// 軸耐力範囲外なら空。
pub fn slice_at_n(fibers: &[PlasticFiber], n_target: f64, n_pts: usize) -> Vec<[f64; 2]> {
    let mut pts = Vec::with_capacity(n_pts);
    for j in 0..n_pts {
        let beta = 2.0 * std::f64::consts::PI * j as f64 / n_pts as f64;
        let (ky, kz) = (beta.cos(), beta.sin());
        if let Some(m) = plastic_moment_at_n(fibers, ky, kz, n_target) {
            pts.push(m);
        }
    }
    pts
}
