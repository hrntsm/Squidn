//! 応力解析の計算条件の型。
//!
//! - [`StressAnalysisCfg`] — 長期応力解析の計算条件（令82条）。

/// 長期応力解析の計算条件（令82条の応力解析）。
///
/// 計算条件の指定により、一部の部材（ブレース・柱・制振間柱）について
/// 長期軸力を負担させないことが可能である。
///
/// 制振間柱（damper-equipped mullion column）は本リポジトリに要素種別が未実装のため、
/// 対象外（既知の制約）。ブレースと柱（鉛直部材）のみ対応する。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct StressAnalysisCfg {
    /// 長期応力解析でブレース（`ElementKind::Brace`）に軸力を負担させない。
    pub no_long_axial_brace: bool,
    /// 長期応力解析で柱（鉛直な `ElementKind::Beam`）に軸力を負担させない。
    pub no_long_axial_column: bool,
    /// 剛性率・偏心率算定時の雑壁剛性の n 倍法係数（雑壁の剛性評価。RC 規準。
    /// `Kw' = n·Aw'·ΣKc/ΣAc` の n。入力値）。`None` は雑壁剛性を考慮しない。
    #[serde(default)]
    pub misc_wall_n: Option<f64>,
    /// 層間変形角の制限値の分母（令82条の2）。原則 200（1/200）。帳壁・仕上げ等に
    /// 著しい損傷の恐れがない場合は 120（1/120）へ緩和できる。
    #[serde(default = "default_drift_limit_denom")]
    pub drift_limit_denom: f64,
}

fn default_drift_limit_denom() -> f64 {
    200.0
}

impl Default for StressAnalysisCfg {
    fn default() -> Self {
        StressAnalysisCfg {
            no_long_axial_brace: false,
            no_long_axial_column: false,
            misc_wall_n: None,
            drift_limit_denom: default_drift_limit_denom(),
        }
    }
}
