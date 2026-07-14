//! 支持点法による M-N 相関曲面の構築（マルチスプリング/マルチファイバー/単純降伏バネ）。

use super::plastic::{axial_capacity, plastic_moment_at_n, plastic_point};
use super::types::{PlasticFiber, YieldModelKind};

/// M-N 相関曲面。`grid[i][j]` は経線方向 i（引張極 α=0 → 圧縮極 α=π）、
/// 周方向 j（曲げ方向 β、閉曲線）でパラメータ化した曲面上の点 [N, My, Mz]。
pub struct MnSurface {
    pub kind: YieldModelKind,
    /// (n_alpha+1) × n_beta の格子点 [N, My, Mz]（周方向は j=0 と j=n_beta が接続）
    pub grid: Vec<Vec<[f64; 3]>>,
    /// 圧縮軸耐力 [N]（負値）
    pub n_comp: f64,
    /// 引張軸耐力 [N]（正値）
    pub n_tens: f64,
    /// N=0 での y 軸まわり全塑性モーメント [N·mm]
    pub mp_y: f64,
    /// N=0 での z 軸まわり全塑性モーメント [N·mm]
    pub mp_z: f64,
}

/// ファイバ群の特性半径（重心からの最大距離）。曲率方向の無次元化に用いる。
fn char_radius(fibers: &[PlasticFiber]) -> f64 {
    fibers
        .iter()
        .map(|f| (f.y * f.y + f.z * f.z).sqrt())
        .fold(0.0, f64::max)
        .max(1.0)
}

/// 参照耐力（軸耐力 Nc/Nt と N=0 の全塑性モーメント Mp_y/Mp_z）をまとめて算定する。
/// `build_surface` と `build_simple_spring_surface` の共通前処理。
fn reference_capacities(fibers: &[PlasticFiber]) -> (f64, f64, f64, f64) {
    let (nc, nt) = axial_capacity(fibers);
    let mp_y = plastic_moment_at_n(fibers, 1.0, 0.0, 0.0).map_or(0.0, |m| m[0]);
    let mp_z = plastic_moment_at_n(fibers, 0.0, 1.0, 0.0).map_or(0.0, |m| m[1]);
    (nc, nt, mp_y, mp_z)
}

/// 支持点法で M-N 相関曲面を構築する（マルチスプリング/マルチファイバー用）。
///
/// - `n_alpha`: 経線方向（引張極→圧縮極）の分割数
/// - `n_beta`: 周方向（曲げ方向）の分割数
pub fn build_surface(
    fibers: &[PlasticFiber],
    kind: YieldModelKind,
    n_alpha: usize,
    n_beta: usize,
) -> MnSurface {
    let c = char_radius(fibers);
    let (nc, nt, mp_y, mp_z) = reference_capacities(fibers);

    let mut grid = Vec::with_capacity(n_alpha + 1);
    for i in 0..=n_alpha {
        let alpha = std::f64::consts::PI * i as f64 / n_alpha as f64;
        let e0 = alpha.cos();
        let k_mag = alpha.sin() / c;
        let mut row = Vec::with_capacity(n_beta);
        for j in 0..n_beta {
            let beta = 2.0 * std::f64::consts::PI * j as f64 / n_beta as f64;
            row.push(plastic_point(
                fibers,
                e0,
                k_mag * beta.cos(),
                k_mag * beta.sin(),
            ));
        }
        grid.push(row);
    }

    MnSurface {
        kind,
        grid,
        n_comp: nc,
        n_tens: nt,
        mp_y,
        mp_z,
    }
}

/// 単純降伏バネモデルの曲面（軸バネと回転バネの2バネ連成）を構築する。
///
/// バネの降伏値は細分割ファイバから算定した軸耐力 Nc/Nt と N=0 の全塑性モーメント
/// Mp_y/Mp_z とし、軸バネと回転バネの連成を線形相関
///
/// ```text
/// |N|/N許容 + M/M許容 = 1
/// ```
///
/// で考慮する（N許容は引張側 Nt / 圧縮側 |Nc| を使い分け、M/M許容は
/// 正規化モーメントの合成値 √((My/Mp_y)² + (Mz/Mp_z)²)）。
/// 曲面は N 軸を頂点とする双錐（N-M 平面内では直線）になる。
/// 描画の都合上、他モデルと同じ (α, β) 格子トポロジで返す。
pub fn build_simple_spring_surface(
    fibers_fine: &[PlasticFiber],
    n_alpha: usize,
    n_beta: usize,
) -> MnSurface {
    let (nc, nt, mp_y, mp_z) = reference_capacities(fibers_fine);

    // 耐力 0 の軸があると退化するため微小値で下駄を履かせる
    let n_tens_ref = nt.max(1.0);
    let n_comp_ref = nc.abs().max(1.0);
    let my_ref = mp_y.abs().max(1.0);
    let mz_ref = mp_z.abs().max(1.0);

    let mut grid = Vec::with_capacity(n_alpha + 1);
    for i in 0..=n_alpha {
        let alpha = std::f64::consts::PI * i as f64 / n_alpha as f64;
        // α ∈ [0, π/2) は引張側、(π/2, π] は圧縮側の N 許容値を使う
        let n_ref = if alpha.cos() >= 0.0 {
            n_tens_ref
        } else {
            n_comp_ref
        };
        // 正規化空間の方向 (cosα, sinα·cosβ, sinα·sinβ) に対し
        // |N|/N許容 + √((My/Mp_y)² + (Mz/Mp_z)²) = t·(|cosα| + sinα) = 1
        // となるよう倍率 t を定める（sinα ≥ 0）。
        let t = 1.0 / (alpha.cos().abs() + alpha.sin());
        let mut row = Vec::with_capacity(n_beta);
        for j in 0..n_beta {
            let beta = 2.0 * std::f64::consts::PI * j as f64 / n_beta as f64;
            row.push([
                t * alpha.cos() * n_ref,
                t * alpha.sin() * beta.cos() * my_ref,
                t * alpha.sin() * beta.sin() * mz_ref,
            ]);
        }
        grid.push(row);
    }

    MnSurface {
        kind: YieldModelKind::SimpleSpring,
        grid,
        n_comp: nc,
        n_tens: nt,
        mp_y,
        mp_z,
    }
}
