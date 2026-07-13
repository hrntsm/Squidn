//! 静的解析のファサード [`Analysis`]。
//!
//! `prepare` で DofMap 構築・全体剛性 K の組立・拘束縮約・分解を一度行い、以降は
//! 分解済み K を再利用して線形静的・荷重組合せ・固有値・時刻歴・地震/風の各解析を
//! 実行する。地震・風の荷重生成は [`seismic`] / [`wind`]、設定型は [`config`]、
//! 解析前のモデル検証は [`precheck`] に分離している。

use crate::assemble::{assemble_global_f, assemble_global_k};
use crate::constraint::Reducer;
use crate::damping::Damping;
use crate::eigen::{self, ModalResult};
use crate::linear::StaticOnce;
use crate::timehistory::{GroundMotion, NewmarkCfg, ResponseResult};

pub type StaticResult = StaticOnce;
use squid_n_core::dof::DofMap;
use squid_n_core::ids::LoadCaseId;
use squid_n_core::model::{LoadCombination, Model};
use squid_n_element::factory::build_behavior;
use squid_n_math::solver::{make_solver, LinearSolver, SolveError, SolverBackend};

mod config;
mod precheck;
mod seismic;
mod wind;

pub use config::{AiMode, SeismicCfg, SeismicDir, WindStaticCfg};
pub(crate) use seismic::distribute_pi_over_diaphragms;
pub use seismic::{building_height_mm, ground_elevation, steel_height_ratio};

pub struct Analysis<'m> {
    model: &'m Model,
    dofmap: DofMap,
    reducer: Reducer,
    solver: Box<dyn LinearSolver>,
    n_indep: usize,
}

impl<'m> Analysis<'m> {
    /// Build DofMap, assemble global K, apply constraint reduction, and factorize.
    /// After this, `linear_static` and `linear_combination` can be called
    /// multiple times reusing the factorized K.
    ///
    /// 解析前にモデルの静的検証（参照整合・拘束・断面/材料割当・孤立節点）を行い、
    /// 問題があればユーザー向けの日本語診断メッセージ付きでエラーを返す。
    pub fn prepare(model: &'m Model) -> Result<Self, SolveError> {
        faer::set_global_parallelism(faer::Par::Seq);
        model
            .validate()
            .map_err(|e| SolveError::InvalidInput(format!("モデル検証エラー: {:?}", e)))?;
        precheck::precheck_model(model)?;
        let dofmap = DofMap::build(model);
        let n_active = dofmap.n_active();

        if n_active == 0 {
            return Ok(Self {
                model,
                dofmap,
                reducer: Reducer {
                    t_rows: vec![],
                    n_indep: 0,
                    n_free: 0,
                },
                solver: make_solver(SolverBackend::DirectSparseCholesky),
                n_indep: 0,
            });
        }

        let k_free = assemble_global_k(model, &dofmap);
        let reducer = Reducer::build(model, &dofmap);
        let n_indep = reducer.n_indep;
        let k_red = reducer.reduce_k(&k_free);

        let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
        if n_indep > 0 {
            solver.factorize(&k_red).map_err(|e| match e {
                SolveError::NotPositiveDefinite => {
                    SolveError::InvalidInput(precheck::singular_diagnosis(model))
                }
                other => other,
            })?;
        }

        Ok(Self {
            model,
            dofmap,
            reducer,
            solver,
            n_indep,
        })
    }

    /// 全自由度ゼロの結果（有効自由度なしのモデル用）。
    fn zero_result(&self) -> StaticOnce {
        StaticOnce {
            disp: vec![[0.0; 6]; self.model.nodes.len()],
            member_forces: Vec::new(),
        }
    }

    /// 自由 DOF 空間の荷重ベクトルを縮約 → 解 → 展開し、
    /// 節点変位と部材断面力を復元する（線形静的系の共通経路）。
    fn solve_and_recover(&self, f_free: &[f64]) -> Result<StaticOnce, SolveError> {
        let f_red = self.reducer.reduce_f(f_free);
        let u_indep = self.solver.solve(&f_red)?;
        let u_free = self.reducer.expand_u(&u_indep);
        Ok(StaticOnce {
            disp: self.expand_disp(&u_free),
            member_forces: self.recover_member_forces(&u_free),
        })
    }

    /// 自由 DOF ベクトルを節点 6 成分配列へ展開する。
    fn expand_disp(&self, u_free: &[f64]) -> Vec<[f64; 6]> {
        let mut disp: Vec<[f64; 6]> = vec![[0.0; 6]; self.model.nodes.len()];
        for (ni, d6) in disp.iter_mut().enumerate() {
            for (d, slot) in d6.iter_mut().enumerate() {
                let g = ni * squid_n_core::dof::DOF_PER_NODE + d;
                if let Some(active) = self.dofmap.active(g) {
                    *slot = u_free[active as usize];
                }
            }
        }
        disp
    }

    /// 自由 DOF ベクトルから全部材の断面力を復元する。
    fn recover_member_forces(
        &self,
        u_free: &[f64],
    ) -> Vec<(
        squid_n_core::ids::ElemId,
        squid_n_element::beam::MemberForces,
    )> {
        let mut member_forces = Vec::new();
        for elem in &self.model.elements {
            let (behavior, _state) = build_behavior(elem, self.model);
            let gdofs = behavior.global_dofs(&self.dofmap);
            let mut u_elem = vec![0.0; gdofs.len()];
            for (k, &g) in gdofs.iter().enumerate() {
                if g != usize::MAX && g < u_free.len() {
                    u_elem[k] = u_free[g];
                }
            }
            if let Some(forces) = behavior.recover_forces(&u_elem) {
                member_forces.push((elem.id, forces));
            }
        }
        member_forces
    }

    /// Solve a single load case (back-substitution only, factorized K is reused).
    pub fn linear_static(&self, lc: LoadCaseId) -> Result<StaticOnce, SolveError> {
        if self.n_indep == 0 {
            return Ok(self.zero_result());
        }
        if !self.model.load_cases.iter().any(|c| c.id == lc) {
            return Err(SolveError::InvalidInput(format!(
                "荷重ケース {} が存在しません",
                lc.0
            )));
        }
        let f_free = assemble_global_f(self.model, &self.dofmap, lc);
        self.solve_and_recover(&f_free)
    }

    /// Solve eigenvalue problem (subspace iteration) for n_modes lowest modes.
    pub fn eigen(&self, n_modes: usize) -> Result<ModalResult, SolveError> {
        eigen::solve_eigen(self.model, &self.dofmap, &self.reducer, n_modes)
    }

    /// Solve a load combination by assembling the weighted sum of load case
    /// force vectors, then solving with the already factorized K.
    pub fn linear_combination(&self, combo: &LoadCombination) -> Result<StaticOnce, SolveError> {
        if self.n_indep == 0 {
            return Ok(self.zero_result());
        }
        let n_active = self.dofmap.n_active();
        let mut f_free = vec![0.0; n_active];
        for (lc_id, factor) in &combo.terms {
            let f_lc = assemble_global_f(self.model, &self.dofmap, *lc_id);
            for (fi, &v) in f_lc.iter().enumerate() {
                f_free[fi] += v * factor;
            }
        }
        self.solve_and_recover(&f_free)
    }

    /// 時刻歴応答解析（Newmark-β / HHT-α、減衰込み）。
    /// 線形専用ラッパ。非線形時刻歴は `timehistory::linear_time_history_analysis`
    /// と同じパターンのフリー関数で実装予定（§4、現在は線形のみ）。
    pub fn time_history(
        &self,
        wave: &GroundMotion,
        newmark: NewmarkCfg,
        damping: Damping,
    ) -> Result<ResponseResult, squid_n_math::solver::SolveError> {
        let n_indep = self.n_indep;
        let init = vec![0.0; n_indep];
        crate::timehistory::linear_time_history_analysis(
            self.model,
            &self.dofmap,
            &self.reducer,
            wave,
            &newmark,
            &damping,
            &init,
            &init,
            false,
        )
    }

    /// 時刻歴応答解析（HHT-α 法、線形）。α=0 で Newmark-β（平均加速度法）に一致。
    pub fn time_history_hht(
        &self,
        wave: &GroundMotion,
        hht: crate::timehistory::HhtCfg,
        damping: Damping,
    ) -> Result<ResponseResult, squid_n_math::solver::SolveError> {
        let n_indep = self.n_indep;
        let init = vec![0.0; n_indep];
        crate::timehistory::linear_hht_alpha_analysis(
            self.model,
            &self.dofmap,
            &self.reducer,
            wave,
            &hht,
            &damping,
            &init,
            &init,
            false,
        )
    }

    /// LoadCase の節点荷重リストから自由 DOF 空間の荷重ベクトルを組み立てる
    /// （地震荷重・風荷重など静的荷重ケースの共通処理）。
    fn assemble_f_free_from_nodal(&self, nodal: &[squid_n_core::model::NodalLoad]) -> Vec<f64> {
        let n_active = self.dofmap.n_active();
        let mut f_free = vec![0.0; n_active];
        for nodal_load in nodal {
            let ni = nodal_load.node.index();
            for d in 0..squid_n_core::dof::DOF_PER_NODE {
                let g = ni * squid_n_core::dof::DOF_PER_NODE + d;
                if let Some(active) = self.dofmap.active(g) {
                    f_free[active as usize] += nodal_load.values[d];
                }
            }
        }
        f_free
    }
}

#[cfg(test)]
mod tests;
