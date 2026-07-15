//! 断面力の回復とコンター用の節点値外挿。
//!
//! - [`ShellElement::recover_resultants`] — 2×2 ガウス点で断面力を回復
//! - [`ShellElement::compute_contour`] — ガウス点値を節点へ外挿しコンターデータ化
//! - [`extrap_2x2`] — 2×2 ガウス点 → 節点の外挿行列（内部用）

use super::constitutive::{d_bending, d_membrane, d_shear};
use super::element::ShellElement;
use super::resultants::{ShellContourData, ShellContourPoint, ShellResultants};
use super::shape::{dshape_cart, shape_2d, GAUSS_PTS_2};

/// Pre‑computed 2×2 Gauss‑point → node extrapolation matrix.
///
/// For Gauss points at (ξ=±g, η=±g) with g=1/√3, the inverse of the
/// shape‑function matrix is:
/// ```text
///     H = ½ · [ 1+α   -1   1-α   -1 ]   (α = √3)
///               -1   1+α   -1   1-α
///               1-α   -1   1+α   -1
///               -1   1-α   -1   1+α
/// ```
use std::sync::OnceLock;
fn extrap_2x2() -> &'static [[f64; 4]; 4] {
    static H: OnceLock<[[f64; 4]; 4]> = OnceLock::new();
    H.get_or_init(|| {
        let s3 = (3.0_f64).sqrt();
        let a = 0.5 * (1.0 + s3);
        let b = -0.5;
        let c = 0.5 * (1.0 - s3);
        [[a, b, c, b], [b, a, b, c], [c, b, a, b], [b, c, b, a]]
    })
}

impl ShellElement {
    #[allow(non_snake_case)]
    pub fn recover_resultants(
        &self,
        u_elem_global: &[f64; 24],
    ) -> Vec<([f64; 2], ShellResultants)> {
        let u_local = self.frame.rotate_to_local_24(u_elem_global);
        let lc = self.local_coords();
        let mut results = Vec::with_capacity(4);

        for gi in 0..2 {
            for gj in 0..2 {
                let gp = gi * 2 + gj;
                let xi = GAUSS_PTS_2[gp].0;
                let eta = GAUSS_PTS_2[gp].1;
                let dNc = dshape_cart(xi, eta, &lc);

                let bm = self.membrane_b(xi, eta, &dNc);
                let bb = self.bending_b(xi, eta, &dNc);
                let bs = self.shear_b_mitc4(xi, eta, &lc);

                let mut eps_m = [0.0; 3];
                let mut eps_b = [0.0; 3];
                let mut eps_s = [0.0; 2];

                for j in 0..24 {
                    for r in 0..3 {
                        eps_m[r] += bm[r * 24 + j] * u_local[j];
                        eps_b[r] += bb[r * 24 + j] * u_local[j];
                    }
                    for r in 0..2 {
                        eps_s[r] += bs[r * 24 + j] * u_local[j];
                    }
                }

                let dm = d_membrane(self.e, self.nu, self.t);
                let db = d_bending(self.e, self.nu, self.t);
                let ds = d_shear(self.e, self.nu, self.t);

                let nx = dm[0][0] * eps_m[0] + dm[0][1] * eps_m[1];
                let ny = dm[1][0] * eps_m[0] + dm[1][1] * eps_m[1];
                let nxy = dm[2][2] * eps_m[2];
                let mx = db[0][0] * eps_b[0] + db[0][1] * eps_b[1];
                let my = db[1][0] * eps_b[0] + db[1][1] * eps_b[1];
                let mxy = db[2][2] * eps_b[2];
                let qx = ds[0][0] * eps_s[0];
                let qy = ds[1][1] * eps_s[1];

                let N = shape_2d(xi, eta);
                let mut x = 0.0;
                let mut y = 0.0;
                for i in 0..4 {
                    x += N[i] * lc[i][0];
                    y += N[i] * lc[i][1];
                }

                results.push((
                    [x, y],
                    ShellResultants {
                        nx,
                        ny,
                        nxy,
                        mx,
                        my,
                        mxy,
                        qx,
                        qy,
                    },
                ));
            }
        }

        results
    }

    /// Compute per‑node contour data from element nodal displacements.
    ///
    /// 1. Recover resultants at the 4 Gauss points via [`recover_resultants`].
    /// 2. Extrapolate each resultant component to the 4 corner nodes using
    ///    the inverse shape‑function matrix.
    ///
    /// The returned [`ShellContourData`] holds one [`ShellContourPoint`] per
    /// element node; the UI layer can consume this for smooth colour‑fringe
    /// plots (UI‑11).
    pub fn compute_contour(&self, u_elem_global: &[f64; 24]) -> ShellContourData {
        let gp = self.recover_resultants(u_elem_global);
        let h = extrap_2x2();

        // Helper: extrapolate a single component across all 4 Gauss points.
        let extrap = |comp: fn(&ShellResultants) -> f64| -> [f64; 4] {
            let mut v = [0.0; 4];
            for i in 0..4 {
                v[i] = h[i][0] * comp(&gp[0].1)
                    + h[i][1] * comp(&gp[1].1)
                    + h[i][2] * comp(&gp[2].1)
                    + h[i][3] * comp(&gp[3].1);
            }
            v
        };

        let nx = extrap(|r| r.nx);
        let ny = extrap(|r| r.ny);
        let nxy = extrap(|r| r.nxy);
        let mx = extrap(|r| r.mx);
        let my = extrap(|r| r.my);
        let mxy = extrap(|r| r.mxy);
        let qx = extrap(|r| r.qx);
        let qy = extrap(|r| r.qy);

        // Node coordinates (in the element‑local xy‑plane).
        let node_xy: [[f64; 2]; 4] = {
            let f = &self.frame;
            let to_xy = |c: &[f64; 3]| -> [f64; 2] {
                [
                    c[0] * f.e1[0] + c[1] * f.e1[1] + c[2] * f.e1[2],
                    c[0] * f.e2[0] + c[1] * f.e2[1] + c[2] * f.e2[2],
                ]
            };
            [
                to_xy(&self.coords[0]),
                to_xy(&self.coords[1]),
                to_xy(&self.coords[2]),
                to_xy(&self.coords[3]),
            ]
        };

        let make_pt = |i: usize| ShellContourPoint {
            x: node_xy[i][0],
            y: node_xy[i][1],
            resultants: ShellResultants {
                nx: nx[i],
                ny: ny[i],
                nxy: nxy[i],
                mx: mx[i],
                my: my[i],
                mxy: mxy[i],
                qx: qx[i],
                qy: qy[i],
            },
        };

        ShellContourData {
            node_values: [make_pt(0), make_pt(1), make_pt(2), make_pt(3)],
        }
    }
}
