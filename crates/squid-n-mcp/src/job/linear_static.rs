//! 線形静的解析ジョブの純粋計算。
//!
//! - [`compute_linear_static_job`] — LinearStatic ジョブの純粋計算部分。

use super::{model_with_auto_rigid_zones, resolve_load_case, JobOutcome};
use squid_n_core::model::Model;

/// LinearStatic ジョブの純粋計算部分。
pub(crate) fn compute_linear_static_job(
    model: &Model,
    load_case: Option<u32>,
) -> Result<JobOutcome, String> {
    let model = model_with_auto_rigid_zones(model);
    let model = &model;
    let analysis = squid_n_solver::analysis::Analysis::prepare(model)
        .map_err(|e| format!("prepare failed: {e}"))?;
    let lc = resolve_load_case(model, load_case)?;
    let lc_id = lc.id.0;
    let result = analysis
        .linear_static(lc.id)
        .map_err(|e| format!("solve failed: {e}"))?;

    let node_ids: Vec<u32> = model.nodes.iter().map(|n| n.id.0).collect();
    let mut member_force_rows: Vec<(u32, f64, [f64; 6])> = Vec::new();
    for (elem_id, mf) in &result.member_forces {
        for (pos, forces) in &mf.at {
            member_force_rows.push((elem_id.0, *pos, *forces));
        }
    }
    let max_abs_disp = result
        .disp
        .iter()
        .flat_map(|d| d.iter())
        .fold(0.0_f64, |m, v| m.max(v.abs()));

    let summary = serde_json::json!({
        "kind": "LinearStatic",
        "case": lc_id,
        "n_nodes": node_ids.len(),
        "n_member_force_rows": member_force_rows.len(),
        "max_abs_disp": max_abs_disp,
    });
    Ok(JobOutcome::LinearStatic {
        case: lc_id,
        node_ids,
        disp: result.disp,
        member_force_rows,
        summary,
    })
}
