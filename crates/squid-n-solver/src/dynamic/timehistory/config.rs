//! 時刻歴応答解析の入力設定。
//!
//! - [`NewmarkCfg`] — Newmark-β 法のパラメータ（§2）
//! - [`HhtCfg`] — HHT-α 法のパラメータ（§2）
//! - [`GroundMotion`] — 地動加速度入力（基盤一様加振）

/// Newmark-β 法のパラメータ（§2）。
pub struct NewmarkCfg {
    pub beta: f64,
    pub gamma: f64,
    pub dt: f64,
}

impl NewmarkCfg {
    /// 平均加速度法（無条件安定）。dt は後で設定する。
    pub fn average_accel() -> Self {
        Self {
            beta: 0.25,
            gamma: 0.5,
            dt: 0.0,
        }
    }
    /// 線形加速度法（条件付安定）。dt は後で設定する。
    pub fn linear_accel() -> Self {
        Self {
            beta: 1.0 / 6.0,
            gamma: 0.5,
            dt: 0.0,
        }
    }
}

/// HHT-α 法のパラメータ（§2）。α ∈ [−1/3, 0]、既定 −0.1。
pub struct HhtCfg {
    pub alpha: f64,
    pub dt: f64,
}

impl HhtCfg {
    pub fn new(dt: f64) -> Self {
        Self { alpha: -0.1, dt }
    }
}

/// 地動加速度入力（基盤一様加振）。水平1〜2方向（R8）。
/// `dt` はサンプリング間隔。`accel_x`/`accel_y` は同長さの時系列。
/// `accel_theta` は位相差入力によるねじれ地動加速度 [rad/s²]（鉛直軸まわり。
/// 多点位相差入力（構造力学）。`None` はねじれ加振なし）。
pub struct GroundMotion {
    pub dt: f64,
    pub accel_x: Vec<f64>,
    pub accel_y: Option<Vec<f64>>,
    pub accel_theta: Option<Vec<f64>>,
}
