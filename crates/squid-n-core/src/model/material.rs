//! 材料の型。
//!
//! - [`Material`] — 材料（ヤング率・ポアソン比・密度・強度等）。

use super::*;

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Material {
    pub id: MaterialId,
    pub name: String,
    pub young: f64,
    pub poisson: f64,
    pub density: f64,
    #[serde(default)]
    pub shear: Option<f64>,
    /// コンクリート設計基準強度 Fc [N/mm²]。
    /// 鋼材では `None`。RC 設計（令91条）の許容圧縮・せん断に用いる。
    #[serde(default)]
    pub fc: Option<f64>,
    /// 降伏応力 fy [N/mm²]。鋼材の弾塑性挙動（ファイバ材料・端ばねスケルトン）に用いる。
    /// `None` の場合、ファイバ材料は弾性（降伏しない）として扱う（P5 非線形）。
    #[serde(default)]
    pub fy: Option<f64>,
    /// コンクリートの種類（普通/軽量1種/軽量2種）。固定荷重の
    /// 単位体積重量表・許容応力度低減（軽量コンクリートは
    /// 普通コンクリートの 0.9 倍。技術基準解説書）に用いる。鋼材では意味を持たない（既定 Normal）。
    /// 旧スキーマ（フィールド無し）は Normal 扱い。
    #[serde(default)]
    pub concrete_class: crate::units::ConcreteClass,
}

impl Material {
    pub fn shear_modulus(&self) -> f64 {
        self.shear
            .unwrap_or_else(|| self.young / (2.0 * (1.0 + self.poisson)))
    }
}
