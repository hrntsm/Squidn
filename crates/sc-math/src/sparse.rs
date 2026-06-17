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
}
