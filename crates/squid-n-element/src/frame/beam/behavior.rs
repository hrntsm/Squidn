//! [`ElementBehavior`] トレイト実装（自由度写像・接線/幾何剛性・内力・質量行列）。

use super::element::BeamElement;
use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalMat, LocalVec, MassOption};
use smallvec::SmallVec;
use squid_n_core::dof::{DofMap, DOF_PER_NODE};

impl ElementBehavior for BeamElement {
    fn n_dof(&self) -> usize {
        12
    }

    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        let mut gdofs = SmallVec::new();
        for &nid in &self.nodes {
            let ni = nid.index();
            for d in 0..DOF_PER_NODE {
                let g = ni * DOF_PER_NODE + d;
                if let Some(active) = dof.active(g) {
                    gdofs.push(active as usize);
                } else {
                    gdofs.push(usize::MAX);
                }
            }
        }
        gdofs
    }

    fn tangent_stiffness(&self, _state: &ElemState, _ctx: &Ctx) -> LocalMat {
        // 要素ローカルの 12×12 を全体系へ回す（K_global = Rᵀ K_local R）。
        // ElementBehavior::tangent_stiffness は全体系を返す契約（シェルと同じ）。
        // これを欠くと、ローカル系とグローバル系が一致しない部材（鉛直柱・
        // 任意方向材・非対称断面 iy≠iz）で組立 K が誤る。
        self.axis.to_global(&self.local_stiffness())
    }

    fn geometric_stiffness(&self, n: f64) -> LocalMat {
        let l = self.length;
        let c = n / l;
        let mut kg = LocalMat::zeros(12);
        let mut s = |i: usize, j: usize, v: f64| {
            kg.set(i, j, v);
            if i != j {
                kg.set(j, i, v);
            }
        };
        // xy面（uy=1,rz=5 / uy_j=7,rz_j=11）
        s(1, 1, c * 6.0 / 5.0);
        s(7, 7, c * 6.0 / 5.0);
        s(1, 7, -c * 6.0 / 5.0);
        s(1, 5, c * l / 10.0);
        s(1, 11, c * l / 10.0);
        s(5, 7, -c * l / 10.0);
        s(7, 11, -c * l / 10.0);
        s(5, 5, c * 2.0 * l * l / 15.0);
        s(11, 11, c * 2.0 * l * l / 15.0);
        s(5, 11, -c * l * l / 30.0);
        // xz面（uz=2,ry=4 / uz_j=8,ry_j=10）§4.1 規約で並進-回転結合項の符号が逆（ry の向き）
        s(2, 2, c * 6.0 / 5.0);
        s(8, 8, c * 6.0 / 5.0);
        s(2, 8, -c * 6.0 / 5.0);
        s(2, 4, -c * l / 10.0);
        s(2, 10, -c * l / 10.0);
        s(4, 8, c * l / 10.0);
        s(8, 10, c * l / 10.0);
        s(4, 4, c * 2.0 * l * l / 15.0);
        s(10, 10, c * 2.0 * l * l / 15.0);
        s(4, 10, -c * l * l / 30.0);
        // 幾何剛性もグローバル系へ回転（P-Δ を組立系で正しく加算するため）
        self.axis.to_global(&kg)
    }

    fn internal_force(&self, _state: &ElemState, _ctx: &Ctx) -> LocalVec {
        // committed_disp はグローバル系で蓄積されるため、グローバル剛性で内力を評価する。
        // f_global = (R^T·K_local·R)·u_global
        let k = self.axis.to_global(&self.local_stiffness());
        let mut f = LocalVec {
            data: SmallVec::from_elem(0.0, 12),
        };
        for i in 0..12 {
            let mut s = 0.0;
            for j in 0..12 {
                s += k.get(i, j) * self.committed_disp[j];
            }
            f.data[i] = s;
        }
        f
    }

    fn update_state(&mut self, du: &LocalVec, commit: bool, _ctx: &Ctx) {
        for i in 0..12 {
            if commit {
                self.committed_disp[i] += du.data[i];
            }
        }
    }

    fn mass_matrix(&self, opt: MassOption) -> LocalMat {
        let m = self.density * self.a_mass * self.length;
        let mut mm = LocalMat::zeros(12);
        match opt {
            MassOption::Lumped => {
                for d in [0, 1, 2, 6, 7, 8] {
                    mm.set(d, d, m / 2.0);
                }
            }
            MassOption::Consistent => {
                let c1 = m / 6.0;
                let c2 = m / 420.0;
                let l = self.length;
                let l2 = l * l;
                // Axial (Ux):  indices 0,6
                mm.set(0, 0, 2.0 * c1);
                mm.set(0, 6, 1.0 * c1);
                mm.set(6, 0, 1.0 * c1);
                mm.set(6, 6, 2.0 * c1);
                // Torsion (Rx): indices 3,9
                let ct = self.density * self.j * l / 6.0;
                mm.set(3, 3, 2.0 * ct);
                mm.set(3, 9, 1.0 * ct);
                mm.set(9, 3, 1.0 * ct);
                mm.set(9, 9, 2.0 * ct);
                // Bending: Hermite 梁の一貫質量（4x4 ブロック）。
                // DOF は連続ではないためインデックス配列で指定する。
                //   Uy-Rz 面: [Uy_i=1, Rz_i=5, Uy_j=7, Rz_j=11]
                //   Uz-Ry 面: [Uz_i=2, Ry_i=4, Uz_j=8, Ry_j=10]（回転符号は逆）
                let b4 = |mm: &mut LocalMat, idx: [usize; 4], sign: f64| {
                    let [d0, r0, d1, r1] = idx;
                    // 並進-並進
                    mm.set(d0, d0, 156.0 * c2);
                    mm.set(d0, d1, 54.0 * c2);
                    mm.set(d1, d0, 54.0 * c2);
                    mm.set(d1, d1, 156.0 * c2);
                    // 並進-回転
                    mm.set(d0, r0, 22.0 * l * c2 * sign);
                    mm.set(r0, d0, 22.0 * l * c2 * sign);
                    mm.set(d0, r1, -13.0 * l * c2 * sign);
                    mm.set(r1, d0, -13.0 * l * c2 * sign);
                    mm.set(d1, r0, 13.0 * l * c2 * sign);
                    mm.set(r0, d1, 13.0 * l * c2 * sign);
                    mm.set(d1, r1, -22.0 * l * c2 * sign);
                    mm.set(r1, d1, -22.0 * l * c2 * sign);
                    // 回転-回転
                    mm.set(r0, r0, 4.0 * l2 * c2);
                    mm.set(r0, r1, -3.0 * l2 * c2);
                    mm.set(r1, r0, -3.0 * l2 * c2);
                    mm.set(r1, r1, 4.0 * l2 * c2);
                };
                b4(&mut mm, [1, 5, 7, 11], 1.0);
                b4(&mut mm, [2, 4, 8, 10], -1.0);
            }
        }
        mm
    }

    fn recover_forces(&self, u_elem: &[f64]) -> Option<crate::beam::MemberForces> {
        if u_elem.len() < 12 {
            return None;
        }
        let mut arr = [0.0; 12];
        arr.copy_from_slice(&u_elem[..12]);
        Some(self.recover_forces(&arr))
    }
}
