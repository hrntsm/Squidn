//! 断面性能算定に用いる材料・換算定数。
//!
//! - [`N_S_EQ`] — SRC 鉄骨の等価ヤング係数比（暫定既定）
//! - [`KAPPA_RC`] — RC せん断形状係数 κ
//! - [`E_STEEL`] — 鋼材のヤング係数
//! - [`NU_STEEL`] — 鋼材のポアソン比
//! - [`NU_CONCRETE`] — コンクリートのポアソン比
//! - [`GAMMA_CONCRETE`] — コンクリートの単位体積重量 γ

/// SRC 断面の解析剛性算定に用いる鉄骨の等価ヤング係数比（暫定既定）。
pub const N_S_EQ: f64 = 15.0;

/// RC 断面のせん断変形用断面積の形状係数 κ（As = A/κ。材料力学のせん断形状係数）。
pub const KAPPA_RC: f64 = 1.2;

/// 鋼材のヤング係数 [N/mm²]（SRC 内蔵鉄骨・CFT 鋼管の換算用標準値）。
pub const E_STEEL: f64 = 205000.0;

/// 鋼材のポアソン比（せん断弾性係数比 ngs の算定用）。
pub(crate) const NU_STEEL: f64 = 0.3;

/// コンクリートのポアソン比（CFT 充填コンクリートの換算用）。
pub(crate) const NU_CONCRETE: f64 = 0.2;

/// コンクリートの単位体積重量 γ [kN/m³]（普通コンクリートの標準値。
/// ヤング係数式 Ec=3.35e4·(γ/24)²·(Fc/60)^(1/3) に用いる）。
pub(crate) const GAMMA_CONCRETE: f64 = 23.0;
