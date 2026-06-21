use crate::analysis::SeismicDir;
use crate::arc_length::ArcLengthSolver;
use crate::constraint::Reducer;
use crate::transaction::{StateSnapshot, StatefulModel};
use sc_core::dof::DofMap;
use sc_core::ids::{ElemId, StoryId};
use sc_core::model::Model;
use sc_element::behavior::{Ctx, ElemState, ElementBehavior, LocalVec};
use sc_element::factory::build_nonlinear_behavior;
use sc_math::solver::{make_solver, SolverBackend};
use smallvec::SmallVec;

/// 性能曲線の1点（P5 §7.4）
pub struct CapacityPoint {
    pub step: u32,
    pub roof_disp: f64,
    pub base_shear: f64,
    pub story_shear: Vec<f64>,
    pub story_drift: Vec<f64>,
}

/// ヒンジ発生事象（P5 §7.4）
pub struct HingeEvent {
    pub step: u32,
    pub elem: ElemId,
    pub pos: f64,
    pub level: HingeLevel,
    pub ductility: f64,
}

/// ヒンジレベル（P5 §7.4）
pub enum HingeLevel {
    Crack,
    Yield,
    Ultimate,
}

/// 崩壊機構種別（P5 §7.4）
pub enum MechanismType {
    Overall,
    StoryCollapse { story: StoryId },
    Partial,
}

/// プッシュオーバー解析結果（P5 §7.4）
pub struct PushoverResult {
    pub steps: Vec<PushoverStep>,
    pub capacity_curve: Vec<CapacityPoint>,
    pub hinges: Vec<HingeEvent>,
    pub mechanism: MechanismType,
    pub qu: f64,
}

pub struct PushoverStep {
    pub load_factor: f64,
    pub top_disp: f64,
    pub base_shear: f64,
    pub story_drifts: Vec<f64>,
}

fn assemble_k(
    model: &Model,
    dofmap: &DofMap,
    behaviors: &[Box<dyn ElementBehavior>],
    use_kg: bool,
    prescribed: Option<(usize, f64)>,
) -> faer::sparse::SparseColMat<usize, f64> {
    use sc_math::sparse::assemble_csc;
    let ctx = Ctx { model };
    let state = ElemState::default();
    let mut triplets = Vec::new();
    for (_elem, b) in model.elements.iter().zip(behaviors) {
        let gdofs = b.global_dofs(dofmap);
        let mut k = b.tangent_stiffness(&state, &ctx);
        if use_kg {
            let f = b.internal_force(&state, &ctx);
            let n = f.data.first().copied().unwrap_or(0.0);
            let kg = b.geometric_stiffness(n);
            for i in 0..12 {
                for j in 0..12 {
                    let sum = k.get(i, j) + kg.get(i, j);
                    k.set(i, j, sum);
                }
            }
        }
        triplets.extend(k.to_triplets(&gdofs));
    }
    if let Some((d, _u_val)) = prescribed {
        let penalty = 1e16;
        triplets.push(sc_math::sparse::Triplet {
            row: d,
            col: d,
            val: penalty,
        });
    }
    assemble_csc(dofmap.n_active(), triplets)
}

fn compute_f_int(
    model: &Model,
    dofmap: &DofMap,
    behaviors: &[Box<dyn ElementBehavior>],
) -> Vec<f64> {
    let ctx = Ctx { model };
    let state = ElemState::default();
    let mut f = vec![0.0; dofmap.n_active()];
    for (_elem, b) in model.elements.iter().zip(behaviors) {
        let gdofs = b.global_dofs(dofmap);
        let f_local = b.internal_force(&state, &ctx);
        for (&g, &v) in gdofs.iter().zip(f_local.data.iter()) {
            if g != usize::MAX {
                f[g] += v;
            }
        }
    }
    f
}

/// ベースシア（層せん断の総和）を内力の釣合いから求める（P5 §7.4）。
///
/// 静的釣合いでは各自由節点の水平内力 = 外力。よって全自由節点の載荷方向
/// 並進 DOF にわたる内力の総和が、構造全体が支持点へ伝える水平力＝ベースシア
/// に等しい。DOF 添字を直接足す旧実装（`f_int[0..roof].sum()`）は誤り。
fn compute_base_shear(model: &Model, dofmap: &DofMap, f_int: &[f64], dir: SeismicDir) -> f64 {
    let dir_idx = match dir {
        SeismicDir::X => 0,
        SeismicDir::Y => 1,
    };
    let mut v = 0.0;
    for node in &model.nodes {
        let g = node.id.index() * 6 + dir_idx;
        if let Some(a) = dofmap.active(g) {
            v += f_int[a as usize];
        }
    }
    v
}

fn get_roof_disp(total_disp: &[f64], model: &Model, dofmap: &DofMap, dir: SeismicDir) -> f64 {
    if let Some(story) = model.stories.last() {
        if let Some(dia) = story.diaphragms.first() {
            let ni = dia.master.index();
            let dof_idx = match dir {
                SeismicDir::X => 0,
                SeismicDir::Y => 1,
            };
            let g = ni * 6 + dof_idx;
            if let Some(a) = dofmap.active(g) {
                let idx = a as usize;
                if idx < total_disp.len() {
                    return total_disp[idx];
                }
            }
        }
    }
    0.0
}

/// プッシュオーバー解析（P5 §7）
#[allow(clippy::too_many_arguments)]
pub fn pushover_analysis(
    model: &mut Model,
    dofmap: &DofMap,
    reducer: &Reducer,
    dir: SeismicDir,
    max_steps: usize,
    max_disp: f64,
    use_kg: bool,
    use_arc_length: bool,
    arc_length_dl: f64,
) -> Result<PushoverResult, String> {
    let n_active = dofmap.n_active();
    if n_active == 0 {
        return Err("no active DOF".into());
    }

    let mut behaviors: Vec<Box<dyn ElementBehavior>> = Vec::new();
    for elem in &model.elements {
        let (b, _) = build_nonlinear_behavior(elem, model);
        behaviors.push(b);
    }

    let stories = &model.stories;
    if stories.is_empty() {
        return Err("no stories defined".into());
    }
    let height_m = stories.last().map(|s| s.elevation).unwrap_or(0.0) / 1000.0;
    let t = sc_load::ai::approx_t(height_m, 0.0);
    let z = 1.0;
    let tc = sc_load::ai::tc_of(sc_load::ai::SoilClass::II);
    let rt_val = sc_load::ai::rt(t, tc);
    let c0 = 0.2;
    let story_weights: Vec<f64> = stories
        .iter()
        .map(|s| s.seismic_weight.unwrap_or(0.0))
        .collect();
    if story_weights.iter().all(|&w| w == 0.0) {
        return Err("no seismic weight defined".into());
    }
    let ai = sc_load::ai::ai_distribution(&story_weights, z, rt_val, c0, t);

    let dir_vec = match dir {
        SeismicDir::X => [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        SeismicDir::Y => [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
    };
    let mut q = vec![0.0; n_active];
    for (i, story) in stories.iter().enumerate() {
        let pi = ai.pi.get(i).copied().unwrap_or(0.0);
        if pi == 0.0 {
            continue;
        }
        for dia in &story.diaphragms {
            let ni = dia.master.index();
            for d in 0..6 {
                let g = ni * 6 + d;
                if let Some(a) = dofmap.active(g) {
                    q[a as usize] += dir_vec[d] * pi;
                }
            }
        }
    }

    let thresholds = compute_hinge_thresholds(model);
    let mut hinges = Vec::new();
    let mut capacity_curve = Vec::new();
    let mut total_disp = vec![0.0; n_active];
    let n_steps = max_steps.clamp(1, 100);
    let dlambda = 1.0 / n_steps as f64;

    for step in 0..n_steps {
        let mut current_lambda = (step + 1) as f64 * dlambda;
        let mut step_ok = false;

        for _attempt in 0..5 {
            let snap = StateSnapshot::capture(&behaviors);
            let f_ext: Vec<f64> = q.iter().map(|&qi| qi * current_lambda).collect();
            let mut converged = false;
            let mut last_du_free: Vec<f64> = Vec::new();

            for _iter in 0..20 {
                let k_free = assemble_k(model, dofmap, &behaviors, use_kg, None);
                let k_red = reducer.reduce_k(&k_free);
                let f_int = compute_f_int(model, dofmap, &behaviors);
                let r_free: Vec<f64> = f_ext.iter().zip(f_int.iter()).map(|(e, i)| e - i).collect();
                let r_red = reducer.reduce_f(&r_free);

                let f_ext_red = reducer.reduce_f(&f_ext);
                let r_norm: f64 = r_red.iter().map(|x| x * x).sum::<f64>().sqrt();
                let f_norm: f64 = f_ext_red.iter().map(|x| x * x).sum::<f64>().sqrt();
                if r_norm < 1e-6 * f_norm.max(1.0) {
                    converged = true;
                    break;
                }

                let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
                solver
                    .factorize(&k_red)
                    .map_err(|e| format!("factor: {:?}", e))?;
                let du_red = solver
                    .solve(&r_red)
                    .map_err(|e| format!("solve: {:?}", e))?;
                let du_free = reducer.expand_u(&du_red);
                last_du_free = du_free.clone();

                let model_ptr = std::ptr::addr_of_mut!(*model) as *const Model;
                for (_elem, b) in model.elements.iter_mut().zip(behaviors.iter_mut()) {
                    let gdofs = b.global_dofs(dofmap);
                    let mut du_elem = LocalVec {
                        data: SmallVec::from_elem(0.0, 12),
                    };
                    for (i, &g) in gdofs.iter().enumerate() {
                        if g != usize::MAX {
                            du_elem.data[i] = du_free[g];
                        }
                    }
                    let dummy_ctx = Ctx {
                        model: unsafe { &*model_ptr },
                    };
                    b.update_state(&du_elem, false, &dummy_ctx);
                }
            }

            if converged {
                for b in behaviors.iter_mut() {
                    b.commit_state();
                }
                for (&du, td) in last_du_free.iter().zip(total_disp.iter_mut()) {
                    *td += du;
                }
                let roof = get_roof_disp(&total_disp, model, dofmap, dir);
                // ベースシアは内力の釣合いから算定（載荷ベクトル総和でも一致するが、
                // 変位制御フェーズと統一し反力ベースで求める）。
                let f_int_now = compute_f_int(model, dofmap, &behaviors);
                let base_shear = compute_base_shear(model, dofmap, &f_int_now, dir);
                capacity_curve.push(CapacityPoint {
                    step: step as u32,
                    roof_disp: roof,
                    base_shear,
                    story_shear: vec![],
                    story_drift: vec![],
                });
                track_hinges(
                    model,
                    dofmap,
                    &behaviors,
                    &thresholds,
                    step as u32,
                    &mut hinges,
                );
                step_ok = true;
                if max_disp > 0.0 && roof >= max_disp {
                    break;
                }
                break;
            } else {
                model.restore(&snap, &mut behaviors);
                current_lambda *= 0.5;
            }
        }
        if !step_ok {
            // 収束に至らなかった step はスキップ
        }
    }

    // 変位制御フェーズ（P5 §7.1）
    if max_disp > 0.0 {
        if let Some(roof_active) = get_roof_dof(model, dofmap, dir) {
            let initial_disp = total_disp[roof_active];
            let n_disp_steps = 10usize;
            let du_target = (max_disp - initial_disp) / n_disp_steps as f64;

            for step in 0..n_disp_steps {
                let target = initial_disp + du_target * (step + 1) as f64;
                let mut step_ok = false;

                for _attempt in 0..5 {
                    let snap = StateSnapshot::capture(&behaviors);
                    let mut converged = false;
                    let mut last_du_free = Vec::new();

                    for _iter in 0..20 {
                        let k_free = assemble_k(
                            model,
                            dofmap,
                            &behaviors,
                            use_kg,
                            Some((roof_active, target)),
                        );
                        let k_red = reducer.reduce_k(&k_free);
                        let f_int = compute_f_int(model, dofmap, &behaviors);

                        let penalty = 1e16;
                        let f_ext: Vec<f64> = (0..n_active)
                            .map(|i| {
                                if i == roof_active {
                                    target * penalty
                                } else {
                                    0.0
                                }
                            })
                            .collect();
                        let r_free: Vec<f64> =
                            f_ext.iter().zip(f_int.iter()).map(|(e, i)| e - i).collect();
                        let r_red = reducer.reduce_f(&r_free);

                        let r_norm: f64 = r_red.iter().map(|x| x * x).sum::<f64>().sqrt();
                        if r_norm < 1e-6 * target.abs().max(1.0) {
                            converged = true;
                            break;
                        }

                        let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
                        solver.factorize(&k_red).map_err(|e| format!("{:?}", e))?;
                        let du_red = solver.solve(&r_red).map_err(|e| format!("{:?}", e))?;
                        let du_free = reducer.expand_u(&du_red);
                        last_du_free = du_free.clone();

                        let model_ptr = std::ptr::addr_of_mut!(*model) as *const Model;
                        for (_elem, b) in model.elements.iter_mut().zip(behaviors.iter_mut()) {
                            let gdofs = b.global_dofs(dofmap);
                            let mut du_elem = LocalVec {
                                data: SmallVec::from_elem(0.0, 12),
                            };
                            for (i, &g) in gdofs.iter().enumerate() {
                                if g != usize::MAX {
                                    du_elem.data[i] = du_free[g];
                                }
                            }
                            let dummy_ctx = Ctx {
                                model: unsafe { &*model_ptr },
                            };
                            b.update_state(&du_elem, false, &dummy_ctx);
                        }
                    }

                    if converged {
                        for b in behaviors.iter_mut() {
                            b.commit_state();
                        }
                        for (&du, td) in last_du_free.iter().zip(total_disp.iter_mut()) {
                            *td += du;
                        }
                        let roof = get_roof_disp(&total_disp, model, dofmap, dir);
                        let f_int_now = compute_f_int(model, dofmap, &behaviors);
                        let base_shear = compute_base_shear(model, dofmap, &f_int_now, dir);
                        let cstep = (n_steps + 1 + step) as u32;
                        capacity_curve.push(CapacityPoint {
                            step: cstep,
                            roof_disp: roof,
                            base_shear,
                            story_shear: vec![],
                            story_drift: vec![],
                        });
                        track_hinges(model, dofmap, &behaviors, &thresholds, cstep, &mut hinges);
                        step_ok = true;
                        break;
                    } else {
                        model.restore(&snap, &mut behaviors);
                    }
                }
                if !step_ok {
                    break;
                }
            }
        }
    }

    if use_arc_length {
        let arc_solver = ArcLengthSolver::new(arc_length_dl);
        let mut prev_du: Vec<f64> = Vec::new();
        let mut arc_lambda = 1.0;

        for _step in 0..20 {
            let snap = StateSnapshot::capture(&behaviors);
            let k_free = assemble_k(model, dofmap, &behaviors, use_kg, None);
            let k_red = reducer.reduce_k(&k_free);

            let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
            if solver.factorize(&k_red).is_err() {
                model.restore(&snap, &mut behaviors);
                break;
            }

            // 弧長修正子の各反復で内力を再評価するため、変位増分 δu を要素状態へ
            // 反映して更新後 f_int を返すクロージャを渡す（接線 K はステップ開始時で固定＝修正 Newton）。
            let result = {
                let model_ref: &Model = &*model;
                let behaviors_ref = &mut behaviors;
                arc_solver.step(
                    &q,
                    &mut |r: &[f64]| -> Result<Vec<f64>, String> {
                        let r_red = reducer.reduce_f(r);
                        let du_red = solver.solve(&r_red).map_err(|e| format!("{:?}", e))?;
                        Ok(reducer.expand_u(&du_red))
                    },
                    &mut |delta_u: &[f64]| -> Result<Vec<f64>, String> {
                        let ctx = Ctx { model: model_ref };
                        for b in behaviors_ref.iter_mut() {
                            let gdofs = b.global_dofs(dofmap);
                            let mut du_elem = LocalVec {
                                data: SmallVec::from_elem(0.0, 12),
                            };
                            for (i, &g) in gdofs.iter().enumerate() {
                                if g != usize::MAX && g < delta_u.len() {
                                    du_elem.data[i] = delta_u[g];
                                }
                            }
                            b.update_state(&du_elem, false, &ctx);
                        }
                        Ok(compute_f_int(model_ref, dofmap, behaviors_ref))
                    },
                    &prev_du,
                    arc_lambda,
                )
            };

            match result {
                Ok(step_result) if step_result.converged => {
                    // 要素状態は eval_fint で既に δu 反映済み。ここでは確定のみ。
                    for b in behaviors.iter_mut() {
                        b.commit_state();
                    }
                    for (&du, td) in step_result.du.iter().zip(total_disp.iter_mut()) {
                        *td += du;
                    }
                    arc_lambda += step_result.dlambda;
                    prev_du = step_result.du;

                    let roof = get_roof_disp(&total_disp, model, dofmap, dir);
                    let f_int_now = compute_f_int(model, dofmap, &behaviors);
                    let base_shear = compute_base_shear(model, dofmap, &f_int_now, dir);
                    capacity_curve.push(CapacityPoint {
                        step: (n_steps + 1 + _step) as u32,
                        roof_disp: roof,
                        base_shear,
                        story_shear: vec![],
                        story_drift: vec![],
                    });
                }
                _ => {
                    model.restore(&snap, &mut behaviors);
                    break;
                }
            }
        }
    }

    let mechanism = determine_mechanism(&hinges, model);
    // 保有水平耐力 Qu = 性能曲線上の最大ベースシア（崩壊機構形成時の水平耐力）。
    // 単調載荷では機構形成後に頭打ちとなるため、ピーク値を採る。
    let qu = capacity_curve
        .iter()
        .map(|c| c.base_shear)
        .fold(0.0_f64, f64::max);
    Ok(PushoverResult {
        steps: vec![],
        capacity_curve,
        hinges,
        mechanism,
        qu,
    })
}

struct HingeThreshold {
    mc: f64,
    my: f64,
    mu: f64,
}

fn compute_hinge_thresholds(model: &Model) -> Vec<HingeThreshold> {
    model
        .elements
        .iter()
        .map(|elem| {
            let (my, mu) = if let Some(sid) = elem.section {
                if let Some(sec) = model.sections.get(sid.index()) {
                    let depth = sec.depth.max(sec.width);
                    let i = sec.iz.max(sec.iy);
                    let z = if depth > 0.0 { i / (depth / 2.0) } else { 0.0 };
                    // 降伏応力は部材材料の fy を優先。未設定なら鋼材既定 235 N/mm²（SN400 級）。
                    let sigma_y = elem
                        .material
                        .and_then(|mid| model.materials.get(mid.index()))
                        .and_then(|m| m.fy)
                        .unwrap_or(235.0);
                    let my = sigma_y * z;
                    (my, my * 1.2)
                } else {
                    (0.0, 0.0)
                }
            } else {
                (0.0, 0.0)
            };
            HingeThreshold {
                mc: my / 3.0,
                my,
                mu,
            }
        })
        .collect()
}

fn track_hinges(
    model: &Model,
    _dofmap: &DofMap,
    behaviors: &[Box<dyn ElementBehavior>],
    thresholds: &[HingeThreshold],
    step: u32,
    hinges: &mut Vec<HingeEvent>,
) {
    let state = ElemState::default();
    let ctx = Ctx { model };
    for (i, (elem, b)) in model.elements.iter().zip(behaviors).enumerate() {
        let f = b.internal_force(&state, &ctx);
        let m_i = f.data[4].abs().max(f.data[5].abs());
        let m_j = f.data[10].abs().max(f.data[11].abs());
        let m_max = m_i.max(m_j);
        let th = &thresholds[i];
        if m_max < th.mc {
            continue;
        }
        let level = if m_max >= th.mu {
            HingeLevel::Ultimate
        } else if m_max >= th.my {
            HingeLevel::Yield
        } else {
            HingeLevel::Crack
        };
        let pos = if m_i >= m_j { 0.0 } else { 1.0 };
        hinges.push(HingeEvent {
            step,
            elem: elem.id,
            pos,
            level,
            ductility: if th.my > 0.0 { m_max / th.my } else { 0.0 },
        });
    }
}

/// 部材端ヒンジが属する階を返す。ヒンジ位置側の節点 story を優先し、
/// 未割当（基礎節点など story=None）の場合は相手端の節点 story で補完する。
fn hinge_story(model: &Model, h: &HingeEvent) -> Option<StoryId> {
    let elem = model.elements.iter().find(|e| e.id == h.elem)?;
    if elem.nodes.len() < 2 {
        return None;
    }
    let (near, far) = if h.pos < 0.5 {
        (elem.nodes[0], elem.nodes[1])
    } else {
        (elem.nodes[1], elem.nodes[0])
    };
    model
        .nodes
        .get(near.index())
        .and_then(|n| n.story)
        .or_else(|| model.nodes.get(far.index()).and_then(|n| n.story))
}

/// 崩壊機構の判定（P5 §7.4）。
///
/// 旧実装は「同一 step に 3 個以上ヒンジ → Overall」というトポロジ非依存の
/// ヒューリスティックで、ひび割れレベルも数えていた。本実装は降伏以上
/// （Yield/Ultimate）の塑性ヒンジのみを対象とし、その階分布から機構種別を分類する:
/// - 終局ヒンジが無くかつ降伏端が 2 未満 → まだ機構未成立（Partial）
/// - 複数階モデルで降伏ヒンジが単一階に集中 → 層崩壊（StoryCollapse）
/// - それ以外（複数階に分布／単一階構造）→ 全体崩壊（Overall）
///
/// 注: 静的不静定次数+1 の厳密な運動学的機構判定ではなく、塑性化分布に基づく
/// 実務的分類である（将来の精緻化余地あり）。
fn determine_mechanism(hinges: &[HingeEvent], model: &Model) -> MechanismType {
    use std::collections::{BTreeMap, BTreeSet};

    let yielded: Vec<&HingeEvent> = hinges
        .iter()
        .filter(|h| matches!(h.level, HingeLevel::Yield | HingeLevel::Ultimate))
        .collect();

    // メカニズム成立ゲート: 終局ヒンジが 1 つ以上、または降伏した部材端が 2 以上。
    let has_ultimate = yielded
        .iter()
        .any(|h| matches!(h.level, HingeLevel::Ultimate));
    let distinct_ends: BTreeSet<(u32, u8)> = yielded
        .iter()
        .map(|h| (h.elem.index() as u32, if h.pos < 0.5 { 0u8 } else { 1u8 }))
        .collect();
    if yielded.is_empty() || (!has_ultimate && distinct_ends.len() < 2) {
        return MechanismType::Partial;
    }

    // 降伏ヒンジの階分布を集計。
    let mut per_story: BTreeMap<u32, usize> = BTreeMap::new();
    let mut story_ids: BTreeMap<u32, StoryId> = BTreeMap::new();
    let mut unmapped = 0usize;
    for h in &yielded {
        match hinge_story(model, h) {
            Some(s) => {
                *per_story.entry(s.index() as u32).or_default() += 1;
                story_ids.insert(s.index() as u32, s);
            }
            None => unmapped += 1,
        }
    }

    let n_model_stories = model.stories.len();
    if n_model_stories > 1 && per_story.len() == 1 && unmapped == 0 {
        // 単一階に塑性化が集中 → 層崩壊機構。
        let key = *per_story.keys().next().unwrap();
        MechanismType::StoryCollapse {
            story: story_ids[&key],
        }
    } else {
        // 複数階に分布、または単一階構造 → 全体崩壊機構。
        MechanismType::Overall
    }
}

fn get_roof_dof(model: &Model, dofmap: &DofMap, dir: SeismicDir) -> Option<usize> {
    let dir_idx = match dir {
        SeismicDir::X => 0,
        SeismicDir::Y => 1,
    };
    if let Some(story) = model.stories.last() {
        if let Some(dia) = story.diaphragms.first() {
            let ni = dia.master.index();
            let g = ni * 6 + dir_idx;
            return dofmap.active(g).map(|a| a as usize);
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constraint::Reducer;
    use sc_core::dof::{Dof6Mask, DofMap};
    use sc_core::ids::{ElemId, MaterialId, NodeId, SectionId, StoryId};
    use sc_core::model::{
        DiaphragmDef, ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Material,
        Node, Section, Story,
    };

    /// 1層・鉛直ファイバ柱の片持ちプッシュオーバー（P5 §10 相当の最小統合テスト）。
    /// 配線済み非線形要素（FiberBeam）＋座標変換＋NR 反復＋降伏追跡が
    /// エンドツーエンドで動作することを検証する。
    fn single_column_model(fy: f64, seismic_weight: f64) -> Model {
        Model {
            nodes: vec![
                Node {
                    id: NodeId(0),
                    coord: [0.0, 0.0, 0.0],
                    restraint: Dof6Mask::FIXED,
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(1),
                    coord: [0.0, 0.0, 3000.0],
                    // FiberBeam はねじり剛性を持たないため、Z 軸柱の頂部ねじり DOF(rz=bit5)
                    // のみ拘束して特異性を除く。曲げ回転 rx,ry と並進は自由。
                    restraint: Dof6Mask(0b100000),
                    mass: None,
                    story: Some(StoryId(0)),
                },
            ],
            elements: vec![ElementData {
                id: ElemId(0),
                kind: ElementKind::Fiber,
                nodes: smallvec::smallvec![NodeId(0), NodeId(1)],
                section: Some(SectionId(0)),
                material: Some(MaterialId(0)),
                local_axis: LocalAxis {
                    ref_vector: [1.0, 0.0, 0.0],
                },
                end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                force_regime: ForceRegime::Auto,
                rigid_zone: Default::default(),
            }],
            sections: vec![Section {
                id: SectionId(0),
                name: "col".to_string(),
                area: 10000.0,
                iy: 8.333e6,
                iz: 8.333e6,
                j: 1.0e6,
                depth: 100.0,
                width: 100.0,
                as_y: 0.0,
                as_z: 0.0,
                panel_thickness: None,
                thickness: None,
            }],
            materials: vec![Material {
                id: MaterialId(0),
                name: "steel".to_string(),
                young: 205000.0,
                poisson: 0.3,
                density: 0.0,
                shear: Some(0.0),
                fc: None,
                fy: Some(fy),
            }],
            stories: vec![Story {
                id: StoryId(0),
                name: "1F".to_string(),
                elevation: 3000.0,
                node_ids: vec![NodeId(1)],
                diaphragms: vec![DiaphragmDef {
                    master: NodeId(1),
                    slaves: vec![],
                    rigid: true,
                }],
                seismic_weight: Some(seismic_weight),
            }],
            ..Default::default()
        }
    }

    #[test]
    fn test_pushover_single_column_forms_hinge() {
        // 降伏応力を低め、地震重量を降伏荷重をやや超える程度に設定し、
        // 柱脚に曲げヒンジが形成されることを確認する。
        let mut model = single_column_model(235.0, 80_000.0);
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);

        let result = pushover_analysis(
            &mut model,
            &dofmap,
            &reducer,
            SeismicDir::X,
            20,    // max_steps
            0.0,   // max_disp（変位制御に移行しない＝荷重制御のみ）
            false, // use_kg
            false, // use_arc_length
            0.0,
        )
        .expect("pushover should run end-to-end");

        // パイプライン全体が収束ステップを生成していること。
        assert!(
            !result.capacity_curve.is_empty(),
            "capacity curve should have at least one converged step"
        );
        // 荷重−変位曲線の頂部変位は単調に正（水平押し）であること。
        let last = result.capacity_curve.last().unwrap();
        assert!(
            last.roof_disp > 0.0,
            "roof displacement should be positive: {}",
            last.roof_disp
        );
        // 降伏応力を与えた鋼材ファイバ柱で、柱脚に曲げヒンジが追跡されること
        //（座標変換＋ファイバ降伏＋降伏追跡のエンドツーエンド検証）。
        assert!(
            !result.hinges.is_empty(),
            "at least one hinge should form in the column under lateral push"
        );
    }

    #[test]
    fn test_pushover_requires_seismic_weight() {
        // 地震重量未定義ではエラーを返す（入力検証）。
        let mut model = single_column_model(235.0, 0.0);
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);
        let result = pushover_analysis(
            &mut model, &dofmap, &reducer, SeismicDir::X, 10, 0.0, false, false, 0.0,
        );
        assert!(result.is_err(), "should error when no seismic weight defined");
    }

    #[test]
    fn test_pushover_arc_length_path_runs() {
        // 弧長法フェーズ（f_int 反復再評価版）がエンドツーエンドで動作すること。
        let mut model = single_column_model(235.0, 80_000.0);
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);
        let result = pushover_analysis(
            &mut model,
            &dofmap,
            &reducer,
            SeismicDir::X,
            10,    // max_steps（荷重制御）
            0.0,   // max_disp
            false, // use_kg
            true,  // use_arc_length
            1.0,   // arc_length_dl [mm]
        )
        .expect("arc-length pushover should run end-to-end");
        assert!(!result.capacity_curve.is_empty());
        assert!(result.qu > 0.0);
    }

    /// determine_mechanism / hinge_story 用の2層・柱通り（基礎-1F-2F）モデル。
    /// node0=基礎(story None), node1=1F(story0), node2=2F(story1)。
    /// elem0=1F柱(0-1), elem1=2F柱(1-2)。
    fn two_story_model() -> Model {
        let sec = Section {
            id: SectionId(0),
            name: "c".to_string(),
            area: 10000.0,
            iy: 8.333e6,
            iz: 8.333e6,
            j: 1.0e6,
            depth: 100.0,
            width: 100.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
        };
        let mat = Material {
            id: MaterialId(0),
            name: "s".to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: Some(0.0),
            fc: None,
            fy: Some(235.0),
        };
        Model {
            nodes: vec![
                Node { id: NodeId(0), coord: [0.0, 0.0, 0.0], restraint: Dof6Mask::FIXED, mass: None, story: None },
                Node { id: NodeId(1), coord: [0.0, 0.0, 3000.0], restraint: Dof6Mask::FREE, mass: None, story: Some(StoryId(0)) },
                Node { id: NodeId(2), coord: [0.0, 0.0, 6000.0], restraint: Dof6Mask::FREE, mass: None, story: Some(StoryId(1)) },
            ],
            elements: vec![
                ElementData { id: ElemId(0), kind: ElementKind::Fiber, nodes: smallvec::smallvec![NodeId(0), NodeId(1)], section: Some(SectionId(0)), material: Some(MaterialId(0)), local_axis: LocalAxis { ref_vector: [1.0, 0.0, 0.0] }, end_cond: [EndCondition::Fixed, EndCondition::Fixed], force_regime: ForceRegime::Auto, rigid_zone: Default::default() },
                ElementData { id: ElemId(1), kind: ElementKind::Fiber, nodes: smallvec::smallvec![NodeId(1), NodeId(2)], section: Some(SectionId(0)), material: Some(MaterialId(0)), local_axis: LocalAxis { ref_vector: [1.0, 0.0, 0.0] }, end_cond: [EndCondition::Fixed, EndCondition::Fixed], force_regime: ForceRegime::Auto, rigid_zone: Default::default() },
            ],
            sections: vec![sec],
            materials: vec![mat],
            stories: vec![
                Story { id: StoryId(0), name: "1F".to_string(), elevation: 3000.0, node_ids: vec![NodeId(1)], diaphragms: vec![], seismic_weight: None },
                Story { id: StoryId(1), name: "2F".to_string(), elevation: 6000.0, node_ids: vec![NodeId(2)], diaphragms: vec![], seismic_weight: None },
            ],
            ..Default::default()
        }
    }

    fn hinge(elem: u32, pos: f64, level: HingeLevel) -> HingeEvent {
        HingeEvent { step: 0, elem: ElemId(elem), pos, level, ductility: 1.0 }
    }

    #[test]
    fn test_determine_mechanism_partial_when_insufficient() {
        let model = two_story_model();
        // ひび割れのみ → Partial
        assert!(matches!(
            determine_mechanism(&[hinge(0, 0.0, HingeLevel::Crack)], &model),
            MechanismType::Partial
        ));
        // 降伏が1端のみ・終局なし → Partial（機構未成立）
        assert!(matches!(
            determine_mechanism(&[hinge(0, 1.0, HingeLevel::Yield)], &model),
            MechanismType::Partial
        ));
    }

    #[test]
    fn test_determine_mechanism_story_collapse() {
        let model = two_story_model();
        // 1F柱の両端（elem0 pos1.0 → node1=1F, elem1 pos0.0 → node1=1F）が降伏
        // → 降伏ヒンジが1F(story0)に集中 → 層崩壊
        let hinges = vec![
            hinge(0, 1.0, HingeLevel::Yield),
            hinge(1, 0.0, HingeLevel::Yield),
        ];
        match determine_mechanism(&hinges, &model) {
            MechanismType::StoryCollapse { story } => assert_eq!(story, StoryId(0)),
            other => panic!("expected StoryCollapse{{0}}, got {:?}", std::mem::discriminant(&other)),
        }
    }

    #[test]
    fn test_determine_mechanism_overall() {
        let model = two_story_model();
        // 1F(story0)と2F(story1)に分散して降伏 → 全体崩壊
        let hinges = vec![
            hinge(0, 1.0, HingeLevel::Yield), // node1 = 1F
            hinge(1, 1.0, HingeLevel::Yield), // node2 = 2F
        ];
        assert!(matches!(
            determine_mechanism(&hinges, &model),
            MechanismType::Overall
        ));
    }

    #[test]
    fn test_pushover_base_shear_is_real_force() {
        // 最初の（弾性）ステップで base_shear/roof_disp が片持ち柱の弾性剛性
        // 3EI/L³ ≈ 189.8 N/mm に一致することを確認（DOF添字加算の旧バグを排除）。
        let mut model = single_column_model(235.0, 80_000.0);
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);
        let result = pushover_analysis(
            &mut model, &dofmap, &reducer, SeismicDir::X, 20, 0.0, false, false, 0.0,
        )
        .unwrap();
        let first = result.capacity_curve.first().unwrap();
        assert!(first.roof_disp > 0.0 && first.base_shear > 0.0);
        let k = first.base_shear / first.roof_disp;
        assert!(
            (150.0..=230.0).contains(&k),
            "first-step stiffness base_shear/roof_disp={k} should be ~3EI/L^3≈189.8"
        );
        // Qu はピークベースシア（全点以上）であること。
        for c in &result.capacity_curve {
            assert!(result.qu >= c.base_shear - 1e-6, "qu {} must be >= {}", result.qu, c.base_shear);
        }
        assert!(result.qu > 0.0);
    }
}
