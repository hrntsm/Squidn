//! joint_wiring サブモジュール共通の部材情報・判定ヘルパ。

use squid_n_core::ids::NodeId;
use squid_n_core::model::{ElementData, Material, Section};

/// 1 部材分の内力（評価位置と [N,Qy,Qz,Mx,My,Mz]）。
pub type ForcesAt<'a> = &'a [(f64, [f64; 6])];

/// 鋼材判定（app の `is_steel` と同じ規則。鉄筋 SD/SR は RC 扱い）。
pub(super) fn is_steel(name: &str) -> bool {
    let upper = name.to_uppercase();
    upper.starts_with("SS")
        || upper.starts_with("SN")
        || upper.starts_with("SM")
        || upper.starts_with("STK")
        || upper.starts_with("ST")
        || upper.starts_with("SA")
        || upper.starts_with("BC")
}

/// 収集済みの部材情報。
pub(super) struct MemberInfo<'a> {
    pub(super) elem: &'a ElementData,
    pub(super) sec: &'a Section,
    pub(super) mat: &'a Material,
    pub(super) forces: ForcesAt<'a>,
    /// 部材軸の鉛直成分（|ez|）。
    pub(super) ez: f64,
    pub(super) length: f64,
}

impl MemberInfo<'_> {
    pub(super) fn is_column(&self) -> bool {
        self.ez >= 0.8
    }
    pub(super) fn is_beam_horiz(&self) -> bool {
        self.ez <= 0.2
    }
    /// 節点 `nid` 側の端部内力行（pos 0/1 のうち近い方）。
    pub(super) fn end_forces(&self, nid: NodeId) -> Option<&[f64; 6]> {
        let pos = if self.elem.nodes.first() == Some(&nid) {
            0.0
        } else if self.elem.nodes.get(1) == Some(&nid) {
            1.0
        } else {
            return None;
        };
        self.forces
            .iter()
            .min_by(|a, b| {
                (a.0 - pos)
                    .abs()
                    .partial_cmp(&(b.0 - pos).abs())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(_, f)| f)
    }
}

/// 主筋 1 段の重心位置（引張縁から）k1 = かぶり + 帯筋径 + 主筋径/2。
pub(super) fn rc_dt(rebar: &squid_n_core::section_shape::RcRebar) -> f64 {
    rebar.cover + rebar.shear.dia + rebar.main_x.dia / 2.0
}
