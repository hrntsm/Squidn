//! 部材の履歴則（復元力特性）の型。
//!
//! - [`HysteresisModel`] — 履歴則の種別（武田型・標準型等）。
//! - [`default_member_hysteresis`] — 構造種別ごとの既定履歴則。
//! - [`MemberHysteresisAttr`] — 部材個別の履歴則指定。

use super::*;

/// 部材の復元力特性（履歴則）。各履歴則の原典（武田モデル等）に基づく
/// 履歴特性で、既定の非線形特性は本実装の既定として与える。
/// 材端集中バネ（`ConcentratedSpringBeam`）の曲げ履歴に適用され、`Auto` は
/// 構造種別ごとの既定（[`default_member_hysteresis`]）へ解決される。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum HysteresisModel {
    /// 既定（構造種別で自動判定: RC/SRC/CFT=武田型、S=標準型）。
    #[default]
    Auto,
    /// 逆行型（常にスケルトン上、履歴ループなし）。
    Retrograde,
    /// 標準型（Masing 則。除荷開始剛性=初期剛性）。
    Standard,
    /// 原点指向型（除荷・再載荷は原点指向の割線）。
    OriginOriented,
    /// 最大点指向型（Clough 系。反対側の最大経験点を指向）。
    MaxPointOriented,
    /// 武田型（剛性低下型トリリニア。RC/SRC/CFT 梁の既定）。
    Takeda,
    /// 辻・山田型（バイリニア＋β 混合硬化。座屈補剛ブレース等）。
    TsujiYamada,
    /// 鉄骨大梁の座屈考慮履歴（耐力劣化型＋RO 除荷。局部/横/連成座屈）。
    SteelBuckling,
}

impl HysteresisModel {
    /// 表示用の日本語名。
    pub fn label(&self) -> &'static str {
        match self {
            HysteresisModel::Auto => "自動",
            HysteresisModel::Retrograde => "逆行型",
            HysteresisModel::Standard => "標準型",
            HysteresisModel::OriginOriented => "原点指向型",
            HysteresisModel::MaxPointOriented => "最大点指向型",
            HysteresisModel::Takeda => "武田型",
            HysteresisModel::TsujiYamada => "辻・山田型",
            HysteresisModel::SteelBuckling => "座屈考慮型",
        }
    }

    /// UI・列挙用の全候補。
    pub const ALL: [HysteresisModel; 8] = [
        HysteresisModel::Auto,
        HysteresisModel::Retrograde,
        HysteresisModel::Standard,
        HysteresisModel::OriginOriented,
        HysteresisModel::MaxPointOriented,
        HysteresisModel::Takeda,
        HysteresisModel::TsujiYamada,
        HysteresisModel::SteelBuckling,
    ];
}

/// 既定の部材曲げ履歴則（本実装の既定の非線形特性。各履歴則の原典による）。
/// 梁の曲げは **RC/SRC/CFT 造＝武田型（トリリニア）**、
/// **S 造＝標準型（バイリニア）** を既定とする。ブレースの軸は S 造＝標準型。
/// `rc_like` は RC/SRC/CFT（コンクリート系）か否か。
pub fn default_member_hysteresis(rc_like: bool) -> HysteresisModel {
    if rc_like {
        HysteresisModel::Takeda
    } else {
        HysteresisModel::Standard
    }
}

/// 部材の履歴則の指定（要素 ID と履歴則の対。`Model::member_hysteresis_attrs`）。
/// 各履歴則の原典による履歴特性。既定（Auto）と異なる履歴則を
/// 部材個別に指定する場合に用いる。
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct MemberHysteresisAttr {
    pub elem: ElemId,
    pub rule: HysteresisModel,
}
