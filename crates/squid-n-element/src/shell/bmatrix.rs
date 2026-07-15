//! ひずみ–節点変位関係の B 行列（膜・曲げ・MITC4 せん断）。
//!
//! - [`ShellElement::membrane_b`] — 膜ひずみ B 行列 (3×24)
//! - [`ShellElement::bending_b`] — 曲率 B 行列 (3×24)
//! - [`ShellElement::shear_b_mitc4`] — MITC4 横せん断 B 行列 (2×24)

use super::element::ShellElement;
use super::shape::{dshape_cart, jacobian, jacobian_inv_transpose, shape_2d};

impl ShellElement {
    /// Membrane B-matrix (3×24): relates membrane strains to nodal DOFs.
    #[allow(non_snake_case)]
    pub(crate) fn membrane_b(&self, _xi: f64, _eta: f64, dNc: &[[f64; 4]; 2]) -> Vec<f64> {
        let ncols = 24;
        let mut b = vec![0.0; 3 * ncols];
        for i in 0..4 {
            let col = i * 6;
            b[col] = dNc[0][i]; // ε_xx = du/dx
            b[ncols + col + 1] = dNc[1][i]; // ε_yy = dv/dy
            b[2 * ncols + col] = dNc[1][i]; // γ_xy: du/dy
            b[2 * ncols + col + 1] = dNc[0][i]; // γ_xy: dv/dx
        }
        b
    }

    /// Bending B-matrix (3×24): relates curvatures to nodal DOFs.
    #[allow(non_snake_case)]
    pub(crate) fn bending_b(&self, _xi: f64, _eta: f64, dNc: &[[f64; 4]; 2]) -> Vec<f64> {
        let ncols = 24;
        let mut b = vec![0.0; 3 * ncols];
        for i in 0..4 {
            let col = i * 6;
            b[col + 4] = dNc[0][i]; // κ_x = dθ_y/dx
            b[ncols + col + 3] = -dNc[1][i]; // κ_y = -dθ_x/dy
            b[2 * ncols + col + 4] = dNc[1][i]; // κ_xy: dθ_y/dy
            b[2 * ncols + col + 3] = -dNc[0][i]; // κ_xy: -dθ_x/dx
        }
        b
    }

    /// MITC4 shear B-matrix (2×24). This is the core of MITC4.
    #[allow(non_snake_case)]
    pub(crate) fn shear_b_mitc4(
        &self,
        xi: f64,
        eta: f64,
        nodes_coords: &[[f64; 3]; 4],
    ) -> Vec<f64> {
        let ncols = 24;
        let mut b = vec![0.0; 2 * ncols];

        // Tying points in natural coordinates
        // Tying points per MITC4 spec:
        let tying: [(f64, f64, usize); 4] = [
            (0.0, 1.0, 0),  // A: (0,+1), used for e_ξζ interpolation (η=+1 side)
            (-1.0, 0.0, 1), // B: (-1,0), used for e_ηζ interpolation (ξ=-1 side)
            (0.0, -1.0, 0), // C: (0,-1), used for e_ξζ interpolation (η=-1 side)
            (1.0, 0.0, 1),  // D: (+1,0), used for e_ηζ interpolation (ξ=+1 side)
        ];

        // Compute the covariant B-matrices at each tying point
        // e_ξζ relates to γ_xz,γ_yz via Jacobian: e_ξζ = J[0][0]*γ_xz + J[0][1]*γ_yz
        // e_ηζ relates to γ_xz,γ_yz via Jacobian: e_ηζ = J[1][0]*γ_xz + J[1][1]*γ_yz
        // We compute the 1×24 B-matrix for e_ξζ and e_ηζ at each tying point.

        // For each tying point, compute the standard shear B (2×24) and then project to covariant.
        // Store separately for ξζ and ηζ:
        // b_cov_ezeta[0..3] = B matrices for e_ξζ at A, C (1×24 each)
        // b_cov_nzeta[0..3] = B matrices for e_ηζ at B, D (1×24 each)

        let mut b_cov_ezeta_at = [vec![0.0; ncols], vec![0.0; ncols]]; // at [A, C]
        let mut b_cov_nzeta_at = [vec![0.0; ncols], vec![0.0; ncols]]; // at [B, D]

        let mut idx_ezeta = 0usize;
        let mut idx_nzeta = 0usize;

        for &(txi, teta, kind) in &tying {
            let dNc_t = dshape_cart(txi, teta, nodes_coords);
            let N_t = shape_2d(txi, teta);
            let jac_t = jacobian(txi, teta, nodes_coords);

            // Standard shear B at this tying point (2×24):
            // [γ_xz; γ_yz] = B_std * u
            // For γ_xz: ∂w/∂x N_i + N_i * θ_y,i  (for each node i)
            // Actually B_std_shear is 2×24:
            // Row 0 (γ_xz): for node i: dNdx_i (for Uz=index 2) and N_i (for Ry=index 4)
            // Row 1 (γ_yz): for node i: dNdy_i (for Uz=index 2) and -N_i (for Rx=index 3)
            let mut b_std = vec![0.0; 2 * ncols];
            for i_node in 0..4 {
                let col = i_node * 6;
                b_std[col + 2] = dNc_t[0][i_node]; // γ_xz: dw/dx
                b_std[col + 4] = N_t[i_node]; // γ_xz: θ_y
                b_std[ncols + col + 2] = dNc_t[1][i_node]; // γ_yz: dw/dy
                b_std[ncols + col + 3] = -N_t[i_node]; // γ_yz: -θ_x
            }

            if kind == 0 {
                let b_cov = &mut b_cov_ezeta_at[idx_ezeta];
                for j in 0..ncols {
                    b_cov[j] = jac_t[0][0] * b_std[j] + jac_t[0][1] * b_std[ncols + j];
                }
                idx_ezeta += 1;
            } else {
                let b_cov = &mut b_cov_nzeta_at[idx_nzeta];
                for j in 0..ncols {
                    b_cov[j] = jac_t[1][0] * b_std[j] + jac_t[1][1] * b_std[ncols + j];
                }
                idx_nzeta += 1;
            }
        }

        let interp_ezeta = |j: usize| -> f64 {
            0.5 * (1.0 + eta) * b_cov_ezeta_at[0][j] + 0.5 * (1.0 - eta) * b_cov_ezeta_at[1][j]
        };
        let interp_nzeta = |j: usize| -> f64 {
            0.5 * (1.0 + xi) * b_cov_nzeta_at[1][j] + 0.5 * (1.0 - xi) * b_cov_nzeta_at[0][j]
        };

        let mut b_cov_mitc = vec![0.0; 2 * ncols];
        for j in 0..ncols {
            b_cov_mitc[j] = interp_ezeta(j);
            b_cov_mitc[ncols + j] = interp_nzeta(j);
        }

        let jac_here = jacobian(xi, eta, nodes_coords);
        let jit = jacobian_inv_transpose(&jac_here);
        for j in 0..ncols {
            b[j] = jit[0][0] * b_cov_mitc[j] + jit[0][1] * b_cov_mitc[ncols + j];
            b[ncols + j] = jit[1][0] * b_cov_mitc[j] + jit[1][1] * b_cov_mitc[ncols + j];
        }

        b
    }
}
