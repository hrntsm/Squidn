use faer::sparse::SparseColMat;

#[derive(Clone, Copy, Debug)]
pub struct Triplet {
    pub row: usize,
    pub col: usize,
    pub val: f64,
}

pub fn assemble_csc(n: usize, mut triplets: Vec<Triplet>) -> SparseColMat<usize, f64> {
    triplets.sort_by_key(|a| (a.col, a.row));
    let mut merged: Vec<faer::sparse::Triplet<usize, usize, f64>> =
        Vec::with_capacity(triplets.len());
    for t in triplets {
        match merged.last_mut() {
            Some(m) if m.row == t.row && m.col == t.col => m.val += t.val,
            _ => merged.push(faer::sparse::Triplet::new(t.row, t.col, t.val)),
        }
    }
    SparseColMat::try_new_from_triplets(n, n, &merged).expect("valid triplets")
}

/// 組立済み CSC 疎行列の非ゼロ要素を Triplet のリストへ変換する。
/// 減衰行列の組立（C = a0·M + a1·K）など、行列の加重和を再組立する用途で使う。
pub fn sparse_to_triplets(mat: &SparseColMat<usize, f64>) -> Vec<Triplet> {
    let (sym, vals) = mat.parts();
    let ncols = sym.ncols();
    let mut out = Vec::with_capacity(vals.len());
    for j in 0..ncols {
        let range = sym.col_range(j);
        let rows = sym.row_idx_of_col_raw(j);
        for (k, &row) in rows.iter().enumerate() {
            out.push(Triplet {
                row,
                col: j,
                val: vals[range.start + k],
            });
        }
    }
    out
}

/// 複数の CSC 行列を係数付きで加算し、新しい CSC を返す。
/// `mats: &[(coef, &SparseColMat)]` の各要素を coef 倍して足す。
pub fn weighted_sum_csc(n: usize, mats: &[(f64, &SparseColMat<usize, f64>)]) -> SparseColMat<usize, f64> {
    let mut triplets = Vec::new();
    for (coef, mat) in mats {
        for t in sparse_to_triplets(mat) {
            triplets.push(Triplet {
                row: t.row,
                col: t.col,
                val: coef * t.val,
            });
        }
    }
    assemble_csc(n, triplets)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_assemble_csc_deterministic() {
        let n = 2;
        let triplets_a = vec![
            Triplet {
                row: 0,
                col: 0,
                val: 100.0,
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
        ];
        let triplets_b = vec![
            Triplet {
                row: 1,
                col: 1,
                val: 200.0,
            },
            Triplet {
                row: 0,
                col: 1,
                val: -200.0,
            },
            Triplet {
                row: 1,
                col: 0,
                val: -200.0,
            },
            Triplet {
                row: 0,
                col: 0,
                val: 100.0,
            },
        ];
        let mat_a = assemble_csc(n, triplets_a);
        let mat_b = assemble_csc(n, triplets_b);
        for i in 0..n {
            for j in 0..n {
                assert_eq!(
                    *mat_a.get(i, j).unwrap_or(&0.0),
                    *mat_b.get(i, j).unwrap_or(&0.0)
                );
            }
        }
    }

    #[test]
    fn test_assemble_csc_merge() {
        let n = 1;
        let triplets = vec![
            Triplet {
                row: 0,
                col: 0,
                val: 10.0,
            },
            Triplet {
                row: 0,
                col: 0,
                val: 20.0,
            },
        ];
        let mat = assemble_csc(n, triplets);
        assert_eq!(*mat.get(0, 0).unwrap_or(&0.0), 30.0);
    }

    #[test]
    fn test_sparse_to_triplets_roundtrip() {
        let n = 3;
        let triplets = vec![
            Triplet { row: 0, col: 0, val: 1.0 },
            Triplet { row: 1, col: 0, val: 2.0 },
            Triplet { row: 1, col: 1, val: 3.0 },
            Triplet { row: 2, col: 2, val: 4.0 },
            Triplet { row: 0, col: 2, val: 5.0 },
        ];
        let mat = assemble_csc(n, triplets);
        let recovered = sparse_to_triplets(&mat);
        let rebuilt = assemble_csc(n, recovered);
        for i in 0..n {
            for j in 0..n {
                let a = *mat.get(i, j).unwrap_or(&0.0);
                let b = *rebuilt.get(i, j).unwrap_or(&0.0);
                assert_eq!(a, b, "mismatch at ({},{})", i, j);
            }
        }
    }

    #[test]
    fn test_weighted_sum_csc() {
        let n = 2;
        let m = assemble_csc(
            n,
            vec![
                Triplet { row: 0, col: 0, val: 1.0 },
                Triplet { row: 1, col: 1, val: 1.0 },
            ],
        );
        let k = assemble_csc(
            n,
            vec![
                Triplet { row: 0, col: 0, val: 100.0 },
                Triplet { row: 1, col: 0, val: -50.0 },
                Triplet { row: 0, col: 1, val: -50.0 },
                Triplet { row: 1, col: 1, val: 100.0 },
            ],
        );
        let c = weighted_sum_csc(n, &[(2.0, &m), (0.05, &k)]);
        assert!((*c.get(0, 0).unwrap_or(&0.0) - 7.0).abs() < 1e-12);
        assert!((*c.get(1, 1).unwrap_or(&0.0) - 7.0).abs() < 1e-12);
        assert!((*c.get(1, 0).unwrap_or(&0.0) - (-2.5)).abs() < 1e-12);
        assert!((*c.get(0, 1).unwrap_or(&0.0) - (-2.5)).abs() < 1e-12);
    }
}
