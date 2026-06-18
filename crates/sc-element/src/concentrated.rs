use crate::beam::invert_small;
use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalMat, LocalVec, MassOption};
use sc_core::dof::{DofMap, DOF_PER_NODE};

use sc_material::uniaxial::UniaxialMaterial;
use smallvec::SmallVec;
use std::any::Any;

pub struct ConcentratedSpringBeam {
    pub elastic: crate::beam::BeamElement,
    pub spring_i: Box<dyn UniaxialMaterial>,
    pub spring_j: Box<dyn UniaxialMaterial>,
    rot_i: f64,
    rot_j: f64,
    trial_rot_i: f64,
    trial_rot_j: f64,
}

impl ConcentratedSpringBeam {
    pub fn new(
        elastic: crate::beam::BeamElement,
        spring_i: Box<dyn UniaxialMaterial>,
        spring_j: Box<dyn UniaxialMaterial>,
    ) -> Self {
        Self {
            elastic,
            spring_i,
            spring_j,
            rot_i: 0.0,
            rot_j: 0.0,
            trial_rot_i: 0.0,
            trial_rot_j: 0.0,
        }
    }
}

fn condense_springs(k_elem: &LocalMat, k_i: f64, k_j: f64) -> LocalMat {
    let n = 18;
    let mut k = vec![0.0; n * n];

    let map18 = |i: usize| -> usize {
        match i {
            0..=2 => i,
            3..=5 => i + 9,
            6..=8 => i,
            9..=11 => i + 6,
            _ => i,
        }
    };
    for i in 0..12 {
        for j in 0..12 {
            k[map18(i) * n + map18(j)] = k_elem.get(i, j);
        }
    }

    let ext_rot = [3usize, 4, 5, 9, 10, 11];
    let int_rot = [12usize, 13, 14, 15, 16, 17];
    for (idx, &er) in ext_rot.iter().enumerate() {
        let ir = int_rot[idx];
        let ks = if idx < 3 { k_i } else { k_j };
        k[er * n + er] += ks;
        k[ir * n + ir] += ks;
        k[er * n + ir] -= ks;
        k[ir * n + er] -= ks;
    }

    let na = 12;
    let nb = 6;
    let mut kaa = vec![0.0; na * na];
    let mut kab = vec![0.0; na * nb];
    let mut kba = vec![0.0; nb * na];
    let mut kbb = vec![0.0; nb * nb];

    for i in 0..na {
        for j in 0..na {
            kaa[i * na + j] = k[i * n + j];
        }
        for j in 0..nb {
            kab[i * nb + j] = k[i * n + (na + j)];
            kba[j * na + i] = k[(na + j) * n + i];
        }
    }
    for i in 0..nb {
        for j in 0..nb {
            kbb[i * nb + j] = k[(na + i) * n + (na + j)];
        }
    }

    let kbb_inv = invert_small(&kbb, nb);

    let mut kab_kbbinv = vec![0.0; na * nb];
    for i in 0..na {
        for j in 0..nb {
            let mut s = 0.0;
            for l in 0..nb {
                s += kab[i * nb + l] * kbb_inv[l * nb + j];
            }
            kab_kbbinv[i * nb + j] = s;
        }
    }

    let mut kstar = LocalMat::zeros(na);
    for i in 0..na {
        for j in 0..na {
            let mut s = kaa[i * na + j];
            for l in 0..nb {
                s -= kab_kbbinv[i * nb + l] * kba[l * na + j];
            }
            kstar.set(i, j, s);
        }
    }
    kstar
}

impl ElementBehavior for ConcentratedSpringBeam {
    fn n_dof(&self) -> usize {
        12
    }

    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        let mut gdofs = SmallVec::new();
        for &nid in &self.elastic.nodes {
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
        let kti = {
            let mut m = self.spring_i.clone_box();
            m.trial(self.trial_rot_i).1
        };
        let ktj = {
            let mut m = self.spring_j.clone_box();
            m.trial(self.trial_rot_j).1
        };

        let k_raw = self.elastic.local_stiffness_raw();
        let k_end = condense_springs(&k_raw, kti, ktj);

        let li = self.elastic.rigid.length_i;
        let lj = self.elastic.rigid.length_j;
        if li.abs() > 1e-12 || lj.abs() > 1e-12 {
            self.elastic.apply_rigid_zone_transform(&k_end, li, lj)
        } else {
            k_end
        }
    }

    fn internal_force(&self, _state: &ElemState, _ctx: &Ctx) -> LocalVec {
        let moment_i = {
            let mut m = self.spring_i.clone_box();
            m.trial(self.trial_rot_i).0
        };
        let moment_j = {
            let mut m = self.spring_j.clone_box();
            m.trial(self.trial_rot_j).0
        };

        let mut f = self.elastic.internal_force(_state, _ctx);
        f.data[3] += moment_i;
        f.data[4] += moment_i;
        f.data[5] += moment_i;
        f.data[9] += moment_j;
        f.data[10] += moment_j;
        f.data[11] += moment_j;
        f
    }

    fn update_state(&mut self, du: &LocalVec, commit: bool, _ctx: &Ctx) {
        if commit {
            self.elastic.update_state(du, true, _ctx);
            self.rot_i += du.data[4];
            self.rot_j += du.data[10];
            self.spring_i.trial(self.rot_i);
            self.spring_i.commit();
            self.spring_j.trial(self.rot_j);
            self.spring_j.commit();
            self.trial_rot_i = self.rot_i;
            self.trial_rot_j = self.rot_j;
        } else {
            self.elastic.update_state(du, false, _ctx);
            self.trial_rot_i = self.rot_i + du.data[4];
            self.trial_rot_j = self.rot_j + du.data[10];
            self.spring_i.trial(self.trial_rot_i);
            self.spring_j.trial(self.trial_rot_j);
        }
    }

    fn mass_matrix(&self, opt: MassOption) -> LocalMat {
        self.elastic.mass_matrix(opt)
    }

    fn geometric_stiffness(&self, n: f64) -> LocalMat {
        self.elastic.geometric_stiffness(n)
    }

    fn snapshot_state(&self) -> Box<dyn Any> {
        let materials: Vec<Box<dyn UniaxialMaterial>> =
            vec![self.spring_i.clone_box(), self.spring_j.clone_box()];
        Box::new(materials)
    }

    fn restore_state(&mut self, state: &dyn Any) {
        if let Some(materials) = state.downcast_ref::<Vec<Box<dyn UniaxialMaterial>>>() {
            if materials.len() == 2 {
                self.spring_i = materials[0].clone_box();
                self.spring_j = materials[1].clone_box();
            }
        }
    }

    fn commit_state(&mut self) {
        self.elastic.commit_state();
        self.spring_i.commit();
        self.spring_j.commit();
        self.rot_i = self.trial_rot_i;
        self.rot_j = self.trial_rot_j;
    }

    fn revert_state(&mut self) {
        self.elastic.revert_state();
        self.spring_i.revert();
        self.spring_j.revert();
        self.trial_rot_i = self.rot_i;
        self.trial_rot_j = self.rot_j;
    }
}
