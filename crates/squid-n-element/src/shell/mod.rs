use crate::behavior::{ElemState, LocalMat, LocalVec, MassOption};
use smallvec::SmallVec;
use squid_n_core::dof::{DofMap, DOF_PER_NODE};
use squid_n_core::ids::NodeId;
use squid_n_core::model::Model;

pub const DEFAULT_DRILLING_FACTOR: f64 = 1.0e-3;
pub const N_GAUSS: usize = 2;

/// Shell resultants per unit width at a point.
#[derive(Clone, Debug, PartialEq)]
pub struct ShellResultants {
    pub nx: f64,
    pub ny: f64,
    pub nxy: f64,
    pub mx: f64,
    pub my: f64,
    pub mxy: f64,
    pub qx: f64,
    pub qy: f64,
}

/// Contour result at a single point (e.g. an element node).
///
/// Stores the physical coordinates (in the element-local xy‑plane) together
/// with the 8 resultant components.  This is the unit datum used by the
/// contour renderer (UI‑11) to draw filled‑colour fringe plots.
#[derive(Clone, Debug, PartialEq)]
pub struct ShellContourPoint {
    /// Element‑local x‑coordinate [mm]
    pub x: f64,
    /// Element‑local y‑coordinate [mm]
    pub y: f64,
    pub resultants: ShellResultants,
}

/// Per‑element contour data: one `ShellContourPoint` per corner node.
///
/// The 4 entries correspond to the element corner order (node 0 … node 3).
/// Values are obtained by extrapolating from the 2×2 Gauss‑point resultants
/// to the nodes, which gives a visually smooth contour across element
/// boundaries.
#[derive(Clone, Debug, PartialEq)]
pub struct ShellContourData {
    pub node_values: [ShellContourPoint; 4],
}

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

/// Element-local orthonormal frame for a 4-node shell.
#[derive(Clone, Copy)]
pub struct ShellFrame {
    pub e1: [f64; 3],
    pub e2: [f64; 3],
    pub n: [f64; 3],
}

impl ShellFrame {
    pub fn from_nodes(p: [[f64; 3]; 4]) -> Self {
        let v13 = [p[2][0] - p[0][0], p[2][1] - p[0][1], p[2][2] - p[0][2]];
        let v24 = [p[3][0] - p[1][0], p[3][1] - p[1][1], p[3][2] - p[1][2]];
        let n = [
            v13[1] * v24[2] - v13[2] * v24[1],
            v13[2] * v24[0] - v13[0] * v24[2],
            v13[0] * v24[1] - v13[1] * v24[0],
        ];
        let nl = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        let n = if nl > 1e-12 {
            [n[0] / nl, n[1] / nl, n[2] / nl]
        } else {
            [0.0, 0.0, 1.0]
        };

        let e1 = [p[1][0] - p[0][0], p[1][1] - p[0][1], p[1][2] - p[0][2]];
        let e1l = (e1[0] * e1[0] + e1[1] * e1[1] + e1[2] * e1[2]).sqrt();
        let e1 = if e1l > 1e-12 {
            [e1[0] / e1l, e1[1] / e1l, e1[2] / e1l]
        } else {
            [1.0, 0.0, 0.0]
        };

        let e2 = [
            n[1] * e1[2] - n[2] * e1[1],
            n[2] * e1[0] - n[0] * e1[2],
            n[0] * e1[1] - n[1] * e1[0],
        ];

        Self { e1, e2, n }
    }

    fn rot_6x6(&self) -> [f64; 36] {
        let mut r = [0.0; 36];
        for i in 0..3 {
            r[i * 6] = self.e1[i];
            r[i * 6 + 1] = self.e2[i];
            r[i * 6 + 2] = self.n[i];
            r[(i + 3) * 6 + 3] = self.e1[i];
            r[(i + 3) * 6 + 4] = self.e2[i];
            r[(i + 3) * 6 + 5] = self.n[i];
        }
        r
    }

    fn rot_6x6_transpose(&self) -> [f64; 36] {
        let mut rt = [0.0; 36];
        for i in 0..3 {
            rt[i] = self.e1[i];
            rt[6 + i] = self.e2[i];
            rt[12 + i] = self.n[i];
            rt[3 * 6 + (i + 3)] = self.e1[i];
            rt[4 * 6 + (i + 3)] = self.e2[i];
            rt[5 * 6 + (i + 3)] = self.n[i];
        }
        rt
    }

    pub fn to_global(&self, k_local: &LocalMat) -> LocalMat {
        let n = 24;
        let r = self.rot_6x6();
        let rt = self.rot_6x6_transpose();
        let mut r_block = vec![0.0; n * n];
        for b in 0..4 {
            let bo = b * 6;
            for i in 0..6 {
                for j in 0..6 {
                    r_block[(bo + i) * n + (bo + j)] = r[i * 6 + j];
                }
            }
        }
        let mut rt_block = vec![0.0; n * n];
        for b in 0..4 {
            let bo = b * 6;
            for i in 0..6 {
                for j in 0..6 {
                    rt_block[(bo + i) * n + (bo + j)] = rt[i * 6 + j];
                }
            }
        }
        // 標準規約: R=[e1 e2 n]（列＝ローカル基底）が local→global。
        // K_global = R · K_local · Rᵀ。
        let mut tmp = vec![0.0; n * n];
        for i in 0..n {
            for j in 0..n {
                let mut s = 0.0;
                for k in 0..n {
                    s += k_local.get(i, k) * rt_block[k * n + j];
                }
                tmp[i * n + j] = s;
            }
        }
        let mut kg = LocalMat::zeros(n);
        for i in 0..n {
            for j in 0..n {
                let mut s = 0.0;
                for k in 0..n {
                    s += r_block[i * n + k] * tmp[k * n + j];
                }
                kg.set(i, j, s);
            }
        }
        kg
    }

    /// Rotate a 24-vector from local to global: v_g = R v_l（R=[e1 e2 n]列）。
    pub fn rotate_to_global_24(&self, v_local: &[f64; 24]) -> [f64; 24] {
        let r = self.rot_6x6();
        let n = 24;
        let mut r_block = vec![0.0; n * n];
        for b in 0..4 {
            let bo = b * 6;
            for i in 0..6 {
                for j in 0..6 {
                    r_block[(bo + i) * n + (bo + j)] = r[i * 6 + j];
                }
            }
        }
        let mut vg = [0.0; 24];
        for i in 0..24 {
            let mut s = 0.0;
            for j in 0..24 {
                s += r_block[i * 24 + j] * v_local[j];
            }
            vg[i] = s;
        }
        vg
    }

    /// Rotate a 24-vector from global to local: v_l = Rᵀ v_g。
    pub fn rotate_to_local_24(&self, v_global: &[f64; 24]) -> [f64; 24] {
        let rt = self.rot_6x6_transpose();
        let n = 24;
        let mut rt_block = vec![0.0; n * n];
        for b in 0..4 {
            let bo = b * 6;
            for i in 0..6 {
                for j in 0..6 {
                    rt_block[(bo + i) * n + (bo + j)] = rt[i * 6 + j];
                }
            }
        }
        let mut vl = [0.0; 24];
        for i in 0..24 {
            let mut s = 0.0;
            for j in 0..24 {
                s += rt_block[i * 24 + j] * v_global[j];
            }
            vl[i] = s;
        }
        vl
    }
}

// ---------------------------------------------------------------------------
// 2D bilinear shape functions and derivatives
// ---------------------------------------------------------------------------
fn shape_2d(xi: f64, eta: f64) -> [f64; 4] {
    [
        0.25 * (1.0 - xi) * (1.0 - eta),
        0.25 * (1.0 + xi) * (1.0 - eta),
        0.25 * (1.0 + xi) * (1.0 + eta),
        0.25 * (1.0 - xi) * (1.0 + eta),
    ]
}

fn dshape_2d(xi: f64, eta: f64) -> [[f64; 4]; 2] {
    let dxi = [
        -0.25 * (1.0 - eta),
        0.25 * (1.0 - eta),
        0.25 * (1.0 + eta),
        -0.25 * (1.0 + eta),
    ];
    let deta = [
        -0.25 * (1.0 - xi),
        -0.25 * (1.0 + xi),
        0.25 * (1.0 + xi),
        0.25 * (1.0 - xi),
    ];
    [dxi, deta]
}

#[allow(non_snake_case)]
fn jacobian(xi: f64, eta: f64, nodes: &[[f64; 3]; 4]) -> [[f64; 2]; 2] {
    let dN = dshape_2d(xi, eta);
    let mut jac = [[0.0; 2]; 2];
    for i in 0..4 {
        jac[0][0] += dN[0][i] * nodes[i][0];
        jac[0][1] += dN[0][i] * nodes[i][1];
        jac[1][0] += dN[1][i] * nodes[i][0];
        jac[1][1] += dN[1][i] * nodes[i][1];
    }
    jac
}

fn jacobian_det(jac: &[[f64; 2]; 2]) -> f64 {
    jac[0][0] * jac[1][1] - jac[0][1] * jac[1][0]
}

fn jacobian_inv_transpose(jac: &[[f64; 2]; 2]) -> [[f64; 2]; 2] {
    let det = jacobian_det(jac);
    if det.abs() < 1e-30 {
        return [[1.0, 0.0], [0.0, 1.0]];
    }
    let inv_det = 1.0 / det;
    [
        [jac[1][1] * inv_det, -jac[1][0] * inv_det],
        [-jac[0][1] * inv_det, jac[0][0] * inv_det],
    ]
}

/// Cartesian derivatives: [dNdx; dNdy] = J^{-1} * [dN_dxi; dN_deta]
#[allow(non_snake_case)]
fn dshape_cart(xi: f64, eta: f64, nodes: &[[f64; 3]; 4]) -> [[f64; 4]; 2] {
    let jac = jacobian(xi, eta, nodes);
    let jit = jacobian_inv_transpose(&jac);
    let dN = dshape_2d(xi, eta);
    let mut dNc = [[0.0; 4]; 2];
    for i in 0..4 {
        dNc[0][i] = jit[0][0] * dN[0][i] + jit[1][0] * dN[1][i];
        dNc[1][i] = jit[0][1] * dN[0][i] + jit[1][1] * dN[1][i];
    }
    dNc
}

// ---------------------------------------------------------------------------
// Gauss integration points and weights for 2×2
// ---------------------------------------------------------------------------
const G2: f64 = 0.577_350_269_189_625_7; // 1/sqrt(3)
const GAUSS_PTS_2: [(f64, f64, f64); 4] = [
    (-G2, -G2, 1.0),
    (G2, -G2, 1.0),
    (G2, G2, 1.0),
    (-G2, G2, 1.0),
];

// ---------------------------------------------------------------------------
// Constitutive matrices (plane stress bending and shear) for isotropic material
// ---------------------------------------------------------------------------
fn d_membrane(e: f64, nu: f64, t: f64) -> [[f64; 3]; 3] {
    let c = e * t / (1.0 - nu * nu);
    [
        [c, c * nu, 0.0],
        [c * nu, c, 0.0],
        [0.0, 0.0, c * (1.0 - nu) / 2.0],
    ]
}

fn d_bending(e: f64, nu: f64, t: f64) -> [[f64; 3]; 3] {
    let d0 = e * t * t * t / (12.0 * (1.0 - nu * nu));
    [
        [d0, d0 * nu, 0.0],
        [d0 * nu, d0, 0.0],
        [0.0, 0.0, d0 * (1.0 - nu) / 2.0],
    ]
}

fn d_shear(e: f64, nu: f64, t: f64) -> [[f64; 2]; 2] {
    let g = e / (2.0 * (1.0 + nu));
    let c = g * t * 5.0 / 6.0;
    [[c, 0.0], [0.0, c]]
}

// ---------------------------------------------------------------------------
// ShellElement
// ---------------------------------------------------------------------------
#[derive(Clone)]
pub struct ShellElement {
    pub nodes: [NodeId; 4],
    pub coords: [[f64; 3]; 4],
    pub t: f64,
    pub e: f64,
    pub nu: f64,
    pub density: f64,
    pub frame: ShellFrame,
    pub drilling_factor: f64,
    pub membrane_active: bool,
}

impl ShellElement {
    pub fn new(data: &squid_n_core::model::ElementData, model: &Model) -> Self {
        let nids = [data.nodes[0], data.nodes[1], data.nodes[2], data.nodes[3]];
        let coords = [
            model.nodes[nids[0].index()].coord,
            model.nodes[nids[1].index()].coord,
            model.nodes[nids[2].index()].coord,
            model.nodes[nids[3].index()].coord,
        ];
        let frame = ShellFrame::from_nodes(coords);

        let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
        let t = sec.and_then(|s| s.thickness).unwrap_or(100.0);

        let mat = data
            .material
            .and_then(|mid| model.materials.get(mid.index()));
        let e = mat.map(|m| m.young).unwrap_or(205000.0);
        let nu = mat.map(|m| m.poisson).unwrap_or(0.3);

        // Determine membrane_active: true unless every node is part of a rigid diaphragm
        let membrane_active = {
            let node_in_rigid_diaphragm = |nid: NodeId| -> bool {
                model
                    .nodes
                    .get(nid.index())
                    .and_then(|n| n.story)
                    .and_then(|sid| model.stories.get(sid.index()))
                    .map(|s| {
                        s.diaphragms
                            .iter()
                            .any(|d| d.rigid && (d.master == nid || d.slaves.contains(&nid)))
                    })
                    .unwrap_or(false)
            };
            !nids.iter().all(|&n| node_in_rigid_diaphragm(n))
        };

        ShellElement {
            nodes: nids,
            coords,
            t,
            e,
            nu,
            density: mat.map(|m| m.density).unwrap_or(0.0),
            frame,
            drilling_factor: DEFAULT_DRILLING_FACTOR,
            membrane_active,
        }
    }

    /// Membrane B-matrix (3×24): relates membrane strains to nodal DOFs.
    #[allow(non_snake_case)]
    fn membrane_b(&self, _xi: f64, _eta: f64, dNc: &[[f64; 4]; 2]) -> Vec<f64> {
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
    fn bending_b(&self, _xi: f64, _eta: f64, dNc: &[[f64; 4]; 2]) -> Vec<f64> {
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
    fn shear_b_mitc4(&self, xi: f64, eta: f64, nodes_coords: &[[f64; 3]; 4]) -> Vec<f64> {
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

    /// Add drilling stabilization to the stiffness matrix.
    /// Uses a 4×4 element matrix that is zero for uniform drilling rotation
    /// (rigid body mode) and stiff for relative drilling modes.
    fn add_drilling(&self, k: &mut LocalMat) {
        let gamma = self.drilling_factor;
        let g_mod = self.e / (2.0 * (1.0 + self.nu));
        let area = element_area(&self.coords);
        let scale = gamma * g_mod * self.t * area;

        // Q = I - (1/4) * 1*1^T  =>  diag=3/4, off-diag=-1/4
        let q_diag = 0.75 * scale;
        let q_off = -0.25 * scale;

        for i in 0..4 {
            let ri = i * 6 + 5;
            for j in 0..4 {
                let rj = j * 6 + 5;
                let val = if i == j { q_diag } else { q_off };
                k.set(ri, rj, k.get(ri, rj) + val);
            }
        }
    }

    /// 節点座標を要素ローカル面内 2D 座標（e1,e2 への射影）へ変換する。
    /// B 行列・ヤコビアンはこのローカル座標で評価しなければならない
    /// （`to_global` でフレーム回転を掛けるため、座標も同じフレームに揃える）。
    /// グローバル x,y を直接使うと、第1辺がグローバル x に沿わない要素で
    /// 二重回転になりパッチテストが破綻する。
    fn local_coords(&self) -> [[f64; 3]; 4] {
        let f = &self.frame;
        let mut lc = [[0.0; 3]; 4];
        for i in 0..4 {
            let c = self.coords[i];
            lc[i][0] = c[0] * f.e1[0] + c[1] * f.e1[1] + c[2] * f.e1[2];
            lc[i][1] = c[0] * f.e2[0] + c[1] * f.e2[1] + c[2] * f.e2[2];
            lc[i][2] = 0.0;
        }
        lc
    }

    #[allow(non_snake_case)]
    pub fn local_stiffness(&self) -> LocalMat {
        let n = 24;
        let mut k = LocalMat::zeros(n);
        let lc = self.local_coords();

        // Proper Gauss integration:
        for gi in 0..2 {
            for gj in 0..2 {
                let gp = gi * 2 + gj;
                let xi = GAUSS_PTS_2[gp].0;
                let eta = GAUSS_PTS_2[gp].1;
                let det_j = jacobian_det(&jacobian(xi, eta, &lc));
                if det_j.abs() < 1e-30 {
                    continue;
                }
                let weight = det_j; // product of weights = 1*1 = 1

                let dNc = dshape_cart(xi, eta, &lc);

                // Membrane contribution
                if self.membrane_active {
                    let bm = self.membrane_b(xi, eta, &dNc);
                    let dm = d_membrane(self.e, self.nu, self.t);
                    // K += B^T * D * B * weight * t  (membrane: integrated over thickness = multiply by t)
                    let mut btd = vec![0.0; 24 * 3];
                    for i in 0..24 {
                        for r in 0..3 {
                            let mut s = 0.0;
                            for c in 0..3 {
                                s += bm[c * 24 + i] * dm[r][c];
                            }
                            btd[i * 3 + r] = s;
                        }
                    }
                    for i in 0..24 {
                        for j in 0..24 {
                            let mut s = 0.0;
                            for r in 0..3 {
                                s += btd[i * 3 + r] * bm[r * 24 + j];
                            }
                            k.set(i, j, k.get(i, j) + s * weight);
                        }
                    }
                }

                // Bending contribution
                {
                    let bb = self.bending_b(xi, eta, &dNc);
                    let db = d_bending(self.e, self.nu, self.t);
                    let mut btd = vec![0.0; 24 * 3];
                    for i in 0..24 {
                        for r in 0..3 {
                            let mut s = 0.0;
                            for c in 0..3 {
                                s += bb[c * 24 + i] * db[r][c];
                            }
                            btd[i * 3 + r] = s;
                        }
                    }
                    for i in 0..24 {
                        for j in 0..24 {
                            let mut s = 0.0;
                            for r in 0..3 {
                                s += btd[i * 3 + r] * bb[r * 24 + j];
                            }
                            k.set(i, j, k.get(i, j) + s * weight);
                        }
                    }
                }

                // MITC4 shear contribution
                {
                    let bs = self.shear_b_mitc4(xi, eta, &lc);
                    let ds = d_shear(self.e, self.nu, self.t);
                    let mut btd = vec![0.0; 24 * 2];
                    for i in 0..24 {
                        for r in 0..2 {
                            let mut s = 0.0;
                            for c in 0..2 {
                                s += bs[c * 24 + i] * ds[r][c];
                            }
                            btd[i * 2 + r] = s;
                        }
                    }
                    for i in 0..24 {
                        for j in 0..24 {
                            let mut s = 0.0;
                            for r in 0..2 {
                                s += btd[i * 2 + r] * bs[r * 24 + j];
                            }
                            k.set(i, j, k.get(i, j) + s * weight);
                        }
                    }
                }
            }
        }

        // Drilling stabilization
        self.add_drilling(&mut k);

        k
    }

    pub fn apply_rigid_floor_membrane_off(&self, k: &mut LocalMat) {
        if !self.membrane_active {
            // Zero out rows/cols for Ux (0), Uy (1), Rz (5) at each node
            let n = 24;
            let mut mask = vec![true; n];
            for i in 0..4 {
                let bo = i * 6;
                mask[bo] = false; // Ux
                mask[bo + 1] = false; // Uy
                mask[bo + 5] = false; // Rz
            }
            for i in 0..n {
                if !mask[i] {
                    for j in 0..n {
                        k.set(i, j, 0.0);
                        k.set(j, i, 0.0);
                    }
                    k.set(i, i, 1.0);
                }
            }
        }
    }

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

fn element_area(coords: &[[f64; 3]; 4]) -> f64 {
    // Area of quadrilateral as sum of two triangles
    let v01 = [
        coords[1][0] - coords[0][0],
        coords[1][1] - coords[0][1],
        coords[1][2] - coords[0][2],
    ];
    let v02 = [
        coords[2][0] - coords[0][0],
        coords[2][1] - coords[0][1],
        coords[2][2] - coords[0][2],
    ];
    let v12 = [
        coords[2][0] - coords[1][0],
        coords[2][1] - coords[1][1],
        coords[2][2] - coords[1][2],
    ];
    let v13 = [
        coords[3][0] - coords[1][0],
        coords[3][1] - coords[1][1],
        coords[3][2] - coords[1][2],
    ];

    let cross012 = [
        v01[1] * v02[2] - v01[2] * v02[1],
        v01[2] * v02[0] - v01[0] * v02[2],
        v01[0] * v02[1] - v01[1] * v02[0],
    ];
    let area012 = 0.5
        * (cross012[0] * cross012[0] + cross012[1] * cross012[1] + cross012[2] * cross012[2])
            .sqrt();

    // Using triangles 0-1-2 and 1-2-3
    let cross123 = [
        v12[1] * v13[2] - v12[2] * v13[1],
        v12[2] * v13[0] - v12[0] * v13[2],
        v12[0] * v13[1] - v12[1] * v13[0],
    ];
    let area123 = 0.5
        * (cross123[0] * cross123[0] + cross123[1] * cross123[1] + cross123[2] * cross123[2])
            .sqrt();

    area012 + area123
}

// ---------------------------------------------------------------------------
// ElementBehavior implementation
// ---------------------------------------------------------------------------
impl crate::behavior::ElementBehavior for ShellElement {
    fn n_dof(&self) -> usize {
        24
    }

    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        let mut gdofs = SmallVec::new();
        for &nid in &self.nodes {
            let ni = nid.index();
            for d in 0..DOF_PER_NODE {
                let g = ni * DOF_PER_NODE + d;
                gdofs.push(dof.active(g).map(|a| a as usize).unwrap_or(usize::MAX));
            }
        }
        gdofs
    }

    fn tangent_stiffness(&self, _state: &ElemState, _ctx: &crate::behavior::Ctx) -> LocalMat {
        let mut k_local = self.local_stiffness();
        self.apply_rigid_floor_membrane_off(&mut k_local);
        self.frame.to_global(&k_local)
    }

    fn internal_force(&self, _state: &ElemState, _ctx: &crate::behavior::Ctx) -> LocalVec {
        LocalVec {
            data: smallvec::smallvec![0.0; 24],
        }
    }

    fn update_state(&mut self, _du: &LocalVec, _commit: bool, _ctx: &crate::behavior::Ctx) {}

    fn mass_matrix(&self, opt: MassOption) -> LocalMat {
        let area = element_area(&self.coords);
        let m_total = self.density * self.t * area;
        let mut mm = LocalMat::zeros(24);
        match opt {
            MassOption::Lumped => {
                let m_node = m_total / 4.0;
                for i in 0..4 {
                    let bo = i * 6;
                    mm.set(bo, bo, m_node);
                    mm.set(bo + 1, bo + 1, m_node);
                    mm.set(bo + 2, bo + 2, m_node);
                }
            }
            MassOption::Consistent => {
                // Consistent mass uses 2×2 Gauss integration of NᵀρtN
                let lc = self.local_coords();
                for gi in 0..2 {
                    for gj in 0..2 {
                        let gp = gi * 2 + gj;
                        let xi = GAUSS_PTS_2[gp].0;
                        let eta = GAUSS_PTS_2[gp].1;
                        let det_j = jacobian_det(&jacobian(xi, eta, &lc));
                        let weight = det_j;
                        let n = shape_2d(xi, eta);
                        let rho_t = self.density * self.t;
                        for a in 0..4 {
                            let bo_a = a * 6;
                            let na = n[a];
                            for b in 0..4 {
                                let bo_b = b * 6;
                                let nb = n[b];
                                let contrib = na * nb * rho_t * weight;
                                for d in 0..3 {
                                    let ia = bo_a + d;
                                    let ib = bo_b + d;
                                    mm.set(ia, ib, mm.get(ia, ib) + contrib);
                                }
                            }
                        }
                    }
                }
            }
        }
        mm
    }

    fn recover_forces(&self, u_elem: &[f64]) -> Option<crate::beam::MemberForces> {
        if u_elem.len() < 24 {
            return None;
        }
        let mut arr = [0.0; 24];
        arr.copy_from_slice(&u_elem[..24]);
        let resultants = self.recover_resultants(&arr);
        let data: Vec<(f64, [f64; 6])> = resultants
            .into_iter()
            .map(|(pt, r)| (pt[0], [r.nx, r.ny, r.nxy, r.mx, r.my, r.mxy]))
            .collect();
        Some(crate::beam::MemberForces { at: data })
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
#[allow(non_snake_case)]
mod tests;
