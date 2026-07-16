use std::sync::Mutex;

use faer::sparse::SparseColMat;

use crate::cholesky::CholeskySolver;
use crate::pcg::PcgSolver;
use crate::solver::{LinearSolver, SolveError};

/// AUTO 選択で反復法（PCG）を試みる自由度数の下限。
/// これ未満の系では疎 Cholesky 直接法の分解コストが十分小さく、
/// f64 厳密解が得られる直接法を常に用いる。
pub const AUTO_ITERATIVE_MIN_DOF: usize = 50_000;

/// AUTO 選択時の PCG 収束判定（相対残差 ‖r‖/‖b‖）。
pub const AUTO_PCG_TOL: f64 = 1e-6;

/// AUTO 選択時の PCG 最大反復回数。超過時は直接法へフォールバックする。
pub const AUTO_PCG_MAX_ITER: usize = 10_000;

/// AUTO 選択の結果どちらのバックエンドが選ばれたか（テスト・診断用）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SelectedBackend {
    DirectCholesky,
    IterativePcg,
}

enum State {
    NotFactorized,
    Direct(CholeskySolver),
    Pcg(Box<PcgState>),
}

struct PcgState {
    pcg: PcgSolver,
    /// フォールバック用に保持する係数行列（非ゼロのみなので分解因子より小さい）。
    k: SparseColMat<usize, f64>,
    /// PCG 非収束時に遅延構築する直接法（複数 RHS で分解を再利用する）。
    /// 荷重ケース並列（batch API）から共有されるため `Mutex` で排他する。
    fallback: Mutex<Option<CholeskySolver>>,
}

/// 直接法（疎 Cholesky）と反復法（Jacobi 前処理付き PCG）を自動選択するソルバ。
///
/// 選択規則:
/// - 自由度数が `AUTO_ITERATIVE_MIN_DOF` 未満 → 疎 Cholesky（f64 厳密解）
/// - それ以上 → PCG を試み、規定回数で収束しなければ疎 Cholesky へフォールバック
///
/// 対称正定値系を前提とする（従来 `DirectSparseCholesky` を渡していた箇所の置き換え用）。
/// 非対称・ラグランジュ乗数付き拘束は従来どおり `DirectSparseLu` を明示すること。
pub struct AutoSolver {
    min_dof_for_pcg: usize,
    tol: f64,
    max_iter: usize,
    state: State,
}

impl Default for AutoSolver {
    fn default() -> Self {
        Self::with_params(AUTO_ITERATIVE_MIN_DOF, AUTO_PCG_TOL, AUTO_PCG_MAX_ITER)
    }
}

impl AutoSolver {
    /// 選択しきい値・PCG パラメータを指定して生成する（テストや調整用）。
    pub fn with_params(min_dof_for_pcg: usize, tol: f64, max_iter: usize) -> Self {
        Self {
            min_dof_for_pcg,
            tol,
            max_iter,
            state: State::NotFactorized,
        }
    }

    /// factorize 後に選択されたバックエンドを返す（未分解なら None）。
    pub fn selected(&self) -> Option<SelectedBackend> {
        match &self.state {
            State::NotFactorized => None,
            State::Direct(_) => Some(SelectedBackend::DirectCholesky),
            State::Pcg { .. } => Some(SelectedBackend::IterativePcg),
        }
    }
}

impl LinearSolver for AutoSolver {
    fn factorize(&mut self, k: &SparseColMat<usize, f64>) -> Result<(), SolveError> {
        let n = k.nrows();
        if n >= self.min_dof_for_pcg {
            let mut pcg = PcgSolver::new(self.tol, self.max_iter);
            pcg.factorize(k)?;
            self.state = State::Pcg(Box::new(PcgState {
                pcg,
                k: k.clone(),
                fallback: Mutex::new(None),
            }));
        } else {
            let mut chol = CholeskySolver::default();
            chol.factorize(k)?;
            self.state = State::Direct(chol);
        }
        Ok(())
    }

    fn solve(&self, rhs: &[f64]) -> Result<Vec<f64>, SolveError> {
        match &self.state {
            State::NotFactorized => Err(SolveError::NotFactorized),
            State::Direct(chol) => chol.solve(rhs),
            State::Pcg(state) => match state.pcg.solve(rhs) {
                Ok(x) => Ok(x),
                Err(SolveError::NonConvergence(_)) => {
                    let mut fb = state
                        .fallback
                        .lock()
                        .expect("フォールバック直接法のロックに失敗");
                    if fb.is_none() {
                        let mut chol = CholeskySolver::default();
                        chol.factorize(&state.k)?;
                        *fb = Some(chol);
                    }
                    fb.as_ref()
                        .expect("フォールバック直接法は直前に構築済み")
                        .solve(rhs)
                }
                Err(e) => Err(e),
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solver::{make_solver, SolverBackend};
    use crate::sparse::{assemble_csc, Triplet};

    fn k_2dof() -> SparseColMat<usize, f64> {
        assemble_csc(
            2,
            vec![
                Triplet {
                    row: 0,
                    col: 0,
                    val: 300.0,
                },
                Triplet {
                    row: 1,
                    col: 0,
                    val: -200.0,
                },
                Triplet {
                    row: 0,
                    col: 1,
                    val: -200.0,
                },
                Triplet {
                    row: 1,
                    col: 1,
                    val: 200.0,
                },
            ],
        )
    }

    /// しきい値未満の小規模系では直接法が選ばれ、厳密解が得られる。
    #[test]
    fn test_auto_small_uses_direct() {
        faer::set_global_parallelism(faer::Par::Seq);
        let mut solver = AutoSolver::default();
        solver.factorize(&k_2dof()).unwrap();
        assert_eq!(solver.selected(), Some(SelectedBackend::DirectCholesky));
        let x = solver.solve(&[0.0, 1000.0]).unwrap();
        approx::assert_relative_eq!(x[0], 10.0, max_relative = 1e-9);
        approx::assert_relative_eq!(x[1], 15.0, max_relative = 1e-9);
    }

    /// しきい値を 0 にして PCG 経路を強制し、収束解が得られる。
    #[test]
    fn test_auto_large_uses_pcg() {
        let mut solver = AutoSolver::with_params(0, 1e-6, 1000);
        solver.factorize(&k_2dof()).unwrap();
        assert_eq!(solver.selected(), Some(SelectedBackend::IterativePcg));
        let x = solver.solve(&[0.0, 1000.0]).unwrap();
        approx::assert_relative_eq!(x[0], 10.0, max_relative = 1e-4);
        approx::assert_relative_eq!(x[1], 15.0, max_relative = 1e-4);
    }

    /// PCG が収束しない場合は直接法へフォールバックし、正しい解を返す。
    /// （到達不能な tol と反復 1 回で強制的に非収束にする）
    #[test]
    fn test_auto_falls_back_to_direct_on_nonconvergence() {
        faer::set_global_parallelism(faer::Par::Seq);
        let mut solver = AutoSolver::with_params(0, 1e-300, 1);
        solver.factorize(&k_2dof()).unwrap();
        assert_eq!(solver.selected(), Some(SelectedBackend::IterativePcg));
        let x = solver.solve(&[0.0, 1000.0]).unwrap();
        approx::assert_relative_eq!(x[0], 10.0, max_relative = 1e-9);
        approx::assert_relative_eq!(x[1], 15.0, max_relative = 1e-9);
        // 2 回目の solve もフォールバック分解を再利用して解ける
        let x2 = solver.solve(&[0.0, 2000.0]).unwrap();
        approx::assert_relative_eq!(x2[0], 20.0, max_relative = 1e-9);
    }

    /// 不安定な系（正定値でない）はフォールバック先の直接法でもエラーになる。
    #[test]
    fn test_auto_fallback_reports_not_positive_definite() {
        faer::set_global_parallelism(faer::Par::Seq);
        // [[1,2],[2,1]] は対称だが固有値 3, -1 の不定値行列
        let k = assemble_csc(
            2,
            vec![
                Triplet {
                    row: 0,
                    col: 0,
                    val: 1.0,
                },
                Triplet {
                    row: 1,
                    col: 0,
                    val: 2.0,
                },
                Triplet {
                    row: 0,
                    col: 1,
                    val: 2.0,
                },
                Triplet {
                    row: 1,
                    col: 1,
                    val: 1.0,
                },
            ],
        );
        let mut solver = AutoSolver::with_params(0, 1e-300, 1);
        solver.factorize(&k).unwrap();
        let result = solver.solve(&[1.0, 0.0]);
        assert!(matches!(result, Err(SolveError::NotPositiveDefinite)));
    }

    #[test]
    fn test_auto_not_factorized() {
        let solver = AutoSolver::default();
        assert!(matches!(
            solver.solve(&[1.0]),
            Err(SolveError::NotFactorized)
        ));
    }

    #[test]
    fn test_make_solver_auto() {
        faer::set_global_parallelism(faer::Par::Seq);
        let mut solver = make_solver(SolverBackend::Auto);
        solver.factorize(&k_2dof()).unwrap();
        let x = solver.solve(&[0.0, 1000.0]).unwrap();
        approx::assert_relative_eq!(x[0], 10.0, max_relative = 1e-9);
        approx::assert_relative_eq!(x[1], 15.0, max_relative = 1e-9);
    }
}
