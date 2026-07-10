//! 鋼構造の許容応力度と断面検定（RESP-D マニュアル 04 断面検定準拠）。
//!
//! （実装は Sonnet エージェントが担当。ここは API スタブ。）
use crate::{CheckResult, DesignCheck, DesignCtx, MemberForcesAt};
use squid_n_core::model::{Material, Section};

/// 鋼材の F 値 [N/mm²]（完全一致、板厚 [mm] 区分対応）。
pub fn steel_f_value(grade: &str, thickness: f64) -> Option<f64> {
    let _ = (grade, thickness);
    todo!()
}

/// 鋼材の F 値 [N/mm²]（前方一致、板厚 [mm] 区分対応）。
pub fn steel_f_value_prefix(name: &str, thickness: f64) -> Option<f64> {
    let _ = (name, thickness);
    todo!()
}

pub struct SteelDesign;

impl DesignCheck for SteelDesign {
    fn check(
        &self,
        forces: &MemberForcesAt,
        sec: &Section,
        mat: &Material,
        ctx: &DesignCtx,
    ) -> CheckResult {
        let _ = (forces, sec, mat, ctx);
        todo!()
    }
}
