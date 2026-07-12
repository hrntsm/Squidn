use crate::solver::{LinearSolver, SolveError};
use faer::sparse::linalg::solvers::{Llt, SymbolicLlt};
use faer::sparse::SparseColMat;
use faer::Side;

#[derive(Default)]
pub struct CholeskySolver {
    factor: Option<Llt<usize, f64>>,
    n: usize,
}

impl LinearSolver for CholeskySolver {
    fn factorize(&mut self, k: &SparseColMat<usize, f64>) -> Result<(), SolveError> {
        self.n = k.nrows();
        let symbolic = SymbolicLlt::try_new(k.symbolic(), Side::Lower)
            .map_err(|e| SolveError::Backend(format!("symbolic: {e:?}")))?;
        let llt = Llt::try_new_with_symbolic(symbolic, k.as_ref(), Side::Lower)
            .map_err(|_| SolveError::NotPositiveDefinite)?;
        self.factor = Some(llt);
        Ok(())
    }

    fn solve(&self, rhs: &[f64]) -> Result<Vec<f64>, SolveError> {
        let llt = self.factor.as_ref().ok_or(SolveError::NotFactorized)?;
        crate::solver::solve_dense_column(llt, rhs, self.n)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solver::{make_solver, SolverBackend};
    use crate::sparse::{assemble_csc, Triplet};

    #[test]
    fn test_2dof_spring() {
        faer::set_global_parallelism(faer::Par::Seq);
        let k = assemble_csc(
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
        );
        let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
        solver.factorize(&k).unwrap();
        let x = solver.solve(&[0.0, 1000.0]).unwrap();
        approx::assert_relative_eq!(x[0], 10.0, max_relative = 1e-9);
        approx::assert_relative_eq!(x[1], 15.0, max_relative = 1e-9);
    }

    #[test]
    fn test_2dof_deterministic() {
        faer::set_global_parallelism(faer::Par::Seq);
        let k = assemble_csc(
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
        );
        let mut solver0 = make_solver(SolverBackend::DirectSparseCholesky);
        solver0.factorize(&k).unwrap();
        let x0 = solver0.solve(&[0.0, 1000.0]).unwrap();
        for _ in 0..100 {
            let mut solver = make_solver(SolverBackend::DirectSparseCholesky);
            solver.factorize(&k).unwrap();
            let x = solver.solve(&[0.0, 1000.0]).unwrap();
            assert_eq!(x, x0);
        }
    }

    #[test]
    fn test_not_factorized() {
        let solver = CholeskySolver::default();
        let result = solver.solve(&[1.0, 2.0]);
        assert!(matches!(result, Err(SolveError::NotFactorized)));
    }

    #[test]
    fn test_dim_mismatch() {
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
                    col: 1,
                    val: 1.0,
                },
            ],
        );
        let mut solver = CholeskySolver::default();
        solver.factorize(&k).unwrap();
        let result = solver.solve(&[1.0]);
        assert!(matches!(result, Err(SolveError::DimMismatch { .. })));
    }
}
