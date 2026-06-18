/// 円筒型弧長ステップの結果
pub struct ArcLengthStep {
    pub du: Vec<f64>,
    pub dlambda: f64,
    pub converged: bool,
}

pub type SolverFn<'a> = dyn FnMut(&[f64]) -> Result<Vec<f64>, String> + 'a;

/// 円筒型弧長法ソルバ（Crisfield 1981）
pub struct ArcLengthSolver {
    pub delta_l: f64,
    pub max_iter: u32,
    pub tol: f64,
}

impl ArcLengthSolver {
    pub fn new(delta_l: f64) -> Self {
        ArcLengthSolver {
            delta_l,
            max_iter: 20,
            tol: 1e-6,
        }
    }

    /// 1ステップの弧長制御（予測子＋修正子反復）
    ///
    /// `q`: 参照荷重ベクトル
    /// `solve`: K⁻¹·r を返す線形ソルバクロージャ
    /// `f_int`: 現在の内力ベクトル
    /// `prev_du`: 前ステップの変位増分（符号決定・根選択用）
    pub fn step<'b>(
        &self,
        q: &[f64],
        solve: &mut SolverFn<'b>,
        f_int: &[f64],
        prev_du: &[f64],
        lambda: f64,
    ) -> Result<ArcLengthStep, String> {
        let n = q.len();

        let du_t = solve(q)?;
        let ut_norm = dot(&du_t, &du_t).sqrt();
        if ut_norm < 1e-30 {
            return Err("Zero tangent displacement".into());
        }

        let sign = if prev_du.is_empty() || dot(prev_du, &du_t) >= 0.0 {
            1.0
        } else {
            -1.0
        };

        let mut dlambda = sign * self.delta_l / ut_norm;
        let mut du = scale(&du_t, dlambda);
        let qq = dot(q, q);

        let mut converged = false;
        for _iter in 0..self.max_iter {
            let current_lambda = lambda + dlambda;
            let r: Vec<f64> = (0..n).map(|i| current_lambda * q[i] - f_int[i]).collect();

            let r_norm = dot(&r, &r).sqrt();
            let ext_norm = (current_lambda * current_lambda * qq).sqrt() + 1e-30;
            if r_norm < self.tol * ext_norm {
                converged = true;
                break;
            }

            // du_bar = K⁻¹·r
            let du_bar = solve(&r)?;

            // 円筒型拘束の2次方程式 a·δλ² + b·δλ + c = 0
            // a = du_tᵀ·du_t
            // b = 2·(du + du_bar)ᵀ·du_t
            // c = (du + du_bar)ᵀ·(du + du_bar) - Δl²
            let du_aug = add(&du, &du_bar);
            let a = dot(&du_t, &du_t);
            let b = 2.0 * dot(&du_aug, &du_t);
            let c = dot(&du_aug, &du_aug) - self.delta_l * self.delta_l;

            let disc = b * b - 4.0 * a * c;
            if disc < 0.0 {
                return Err("Negative discriminant in arc-length constraint".into());
            }
            let sqrt_disc = disc.sqrt();
            let dlambda1 = (-b + sqrt_disc) / (2.0 * a);
            let dlambda2 = (-b - sqrt_disc) / (2.0 * a);

            // 根の選択：累積増分方向とのなす角が小さい根を選ぶ
            let d1 = add(&du_bar, &scale(&du_t, dlambda1));
            let d2 = add(&du_bar, &scale(&du_t, dlambda2));
            let cos1 = dot(prev_du, &d1);
            let cos2 = dot(prev_du, &d2);

            let dlambda_sel = if cos1 >= 0.0 && cos2 >= 0.0 {
                if cos1 >= cos2 {
                    dlambda1
                } else {
                    dlambda2
                }
            } else if cos1 >= 0.0 {
                dlambda1
            } else if cos2 >= 0.0 {
                dlambda2
            } else {
                if cos1 > cos2 {
                    dlambda1
                } else {
                    dlambda2
                }
            };

            // 変位・荷重増分の更新
            let du_update = add(&du_bar, &scale(&du_t, dlambda_sel));
            for i in 0..n {
                du[i] += du_update[i];
            }
            dlambda += dlambda_sel;
        }

        Ok(ArcLengthStep {
            du,
            dlambda,
            converged,
        })
    }
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

fn scale(v: &[f64], s: f64) -> Vec<f64> {
    v.iter().map(|x| x * s).collect()
}

fn add(a: &[f64], b: &[f64]) -> Vec<f64> {
    a.iter().zip(b).map(|(x, y)| x + y).collect()
}
