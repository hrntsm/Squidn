use faer::sparse::SparseColMat;

pub trait LinearSolver {
    fn factorize(&mut self, k: &SparseColMat<usize, f64>) -> Result<(), SolveError>;
    fn solve(&self, rhs: &[f64]) -> Result<Vec<f64>, SolveError>;
}

#[derive(Debug, thiserror::Error)]
pub enum SolveError {
    #[error("not factorized yet")]
    NotFactorized,
    #[error("matrix not positive definite")]
    NotPositiveDefinite,
    #[error("dimension mismatch: K={k}, rhs={rhs}")]
    DimMismatch { k: usize, rhs: usize },
    #[error("faer error: {0}")]
    Backend(String),
    /// 入力モデル起因のエラー（拘束不足・断面/材料未割当など）。
    /// メッセージはユーザー向け診断文（日本語）を想定する。
    #[error("{0}")]
    InvalidInput(String),
    /// 反復法・固有値解析が規定回数内に収束しなかった。
    #[error("収束しませんでした: {0}")]
    NonConvergence(String),
}

#[derive(Clone, Copy, Debug)]
pub enum SolverBackend {
    DirectSparseCholesky,
    DirectSparseLu,
    IterativePcg { tol: f64, max_iter: usize },
}

/// 因子分解済みソルバに単一 RHS 列を与えて解く共通処理。
/// `CholeskySolver`／`LuSolver` の `solve` 実装が共有する（次元検査→RHS 構築→解→収集）。
pub(crate) fn solve_dense_column<S: faer::linalg::solvers::Solve<f64>>(
    factor: &S,
    rhs: &[f64],
    n: usize,
) -> Result<Vec<f64>, SolveError> {
    if rhs.len() != n {
        return Err(SolveError::DimMismatch { k: n, rhs: rhs.len() });
    }
    let b = faer::Mat::from_fn(n, 1, |i, _| rhs[i]);
    let x = factor.solve(b.as_ref());
    Ok((0..n).map(|i| x[(i, 0)]).collect())
}

pub fn make_solver(backend: SolverBackend) -> Box<dyn LinearSolver> {
    match backend {
        SolverBackend::DirectSparseCholesky => Box::new(crate::cholesky::CholeskySolver::default()),
        SolverBackend::IterativePcg { tol, max_iter } => {
            Box::new(crate::pcg::PcgSolver::new(tol, max_iter))
        }
        SolverBackend::DirectSparseLu => Box::new(crate::lu::LuSolver::default()),
    }
}
