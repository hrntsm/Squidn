//! 固有値解析ジョブの純粋計算。
//!
//! - [`compute_eigen_job`] — Eigen ジョブの純粋計算部分。

use super::{model_with_auto_rigid_zones, JobOutcome};
use squid_n_core::model::Model;

/// Eigen ジョブの純粋計算部分。
pub(crate) fn compute_eigen_job(model: &Model, n_modes: usize) -> Result<JobOutcome, String> {
    let model = model_with_auto_rigid_zones(model);
    let analysis = squid_n_solver::analysis::Analysis::prepare(&model)
        .map_err(|e| format!("prepare failed: {e}"))?;
    let modal = analysis
        .eigen(n_modes)
        .map_err(|e| format!("eigen failed: {e}"))?;
    let summary = serde_json::json!({
        "kind": "Eigen",
        "n_modes": modal.period.len(),
        "period": modal.period,
    });
    Ok(JobOutcome::Eigen {
        period: modal.period,
        omega2: modal.omega2,
        participation: modal.participation,
        effective_mass: modal.effective_mass,
        summary,
    })
}
