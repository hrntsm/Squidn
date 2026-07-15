//! 制振ダンパー要素（制振部材の力学モデル。マクスウェル要素・弾塑性バイリニア）。
//!
//! - [`MaxwellDamperElement`] —     マクスウェル型ダンパー要素（`maxwell`）
//! - [`HystereticDamperElement`] — 履歴型（弾塑性バイリニア）ダンパー要素（`hysteretic`）

mod hysteretic;
mod maxwell;

pub use hysteretic::HystereticDamperElement;
pub use maxwell::MaxwellDamperElement;
