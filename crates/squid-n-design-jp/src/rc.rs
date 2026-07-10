//! RC 造の許容応力度と断面検定（RESP-D マニュアル 04 断面検定準拠）。
//!
//! （実装は Sonnet エージェントが担当。ここは API スタブ。）
use crate::{CheckResult, DesignCheck, DesignCtx, MemberForcesAt};
use squid_n_core::model::{Material, Section};

pub struct RcDesign;

impl DesignCheck for RcDesign {
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
