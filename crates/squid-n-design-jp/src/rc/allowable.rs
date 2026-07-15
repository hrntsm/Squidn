//! RC 検定に用いる許容応力度のまとめ（部材単位で term 依存の値を 1 回だけ計算する）。
//!
//! [`RcAllow`] — 検定に用いる許容応力度一式。
//! [`rc_allow`] — 許容応力度一式を算定する。
//! [`effective_damage_control`] — 高強度せん断補強筋・軽量時の有効 damage_control。

use crate::material_strength::{
    concrete_allowable_compression_class, concrete_allowable_shear_class, rebar_allowable_shear,
    young_ratio_n,
};
// `ConcreteClass` は本モジュールの関数シグネチャで使うほか、`rc::tests`（`use
// super::*`）が参照するため rc 名前空間へ再エクスポートする。
pub(crate) use squid_n_core::units::ConcreteClass;

/// 検定に用いる許容応力度一式（コンクリート・せん断補強筋。ft は主筋径に
/// 依存するため軸別に別途算定する）。
pub(crate) struct RcAllow {
    /// コンクリート許容圧縮応力度 fc [N/mm²]（長期/短期は算定済み）。
    pub(crate) fc: f64,
    /// コンクリート許容せん断応力度 fs [N/mm²]。
    pub(crate) fs: f64,
    /// せん断補強筋許容引張応力度 w_ft [N/mm²]。
    pub(crate) w_ft: f64,
    /// ヤング係数比 n。
    pub(crate) n_ratio: f64,
}

pub(crate) fn rc_allow(fc_raw: f64, class: ConcreteClass, grade: &str, long_term: bool) -> RcAllow {
    RcAllow {
        fc: concrete_allowable_compression_class(fc_raw, class, long_term),
        fs: concrete_allowable_shear_class(fc_raw, class, long_term),
        w_ft: rebar_allowable_shear(grade, long_term),
        n_ratio: young_ratio_n(fc_raw),
    }
}

/// 高強度せん断補強筋使用時の「損傷制御のための検討」の対象可否を反映した
/// 有効 damage_control。各製品の大臣認定（ウルボン1275等）により、高強度
/// せん断補強筋を使用する軽量コンクリート部材は損傷制御のための検討の対象外
/// とし、安全確保のための検討のみを行う（`shear_grade` が `Some` かつ
/// `class` が軽量1種/2種のとき damage_control を強制的に false にする）。
pub(crate) fn effective_damage_control(
    damage_control: bool,
    shear_grade: Option<&str>,
    class: ConcreteClass,
) -> bool {
    if shear_grade.is_some() && class != ConcreteClass::Normal {
        false
    } else {
        damage_control
    }
}
