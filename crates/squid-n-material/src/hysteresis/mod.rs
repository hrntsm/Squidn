//! 部材レベルの履歴則（設計書 §7 / 仕様書 §5）。集中ばね（one/two-component）系で使う。
//!
//! 責務ごとにサブモジュールへ分割する:
//! - [`rule`] — 履歴則パラメータ [`HysteresisRule`] とスケルトン包絡線
//! - [`material`] — 集中ばね履歴状態機械 [`HysteresisMaterial`]
//! - [`tsuji_yamada`] — 辻・山田モデル（混合硬化）[`TsujiYamada`]
//! - [`steel_buckling`] — 鉄骨大梁の座屈考慮履歴 [`SteelBuckling`] と [`lateral_buckling_mu_ratio`]

pub mod material;
pub mod rule;
pub mod steel_buckling;
pub mod tsuji_yamada;

pub use material::HysteresisMaterial;
pub use rule::HysteresisRule;
pub use steel_buckling::{lateral_buckling_mu_ratio, SteelBuckling};
pub use tsuji_yamada::TsujiYamada;
