use crate::beam::invert_small;
use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalMat, LocalVec, MassOption};
use sc_core::dof::{DofMap, DOF_PER_NODE};

use sc_material::uniaxial::UniaxialMaterial;
use smallvec::SmallVec;
use std::any::Any;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SpringModel {
    OneComponent,
    TwoComponent,
}

pub struct ConcentratedSpringBeam {
    pub elastic: crate::beam::BeamElement,
    pub spring_i: Box<dyn UniaxialMaterial>,
    pub spring_j: Box<dyn UniaxialMaterial>,
    pub model: SpringModel,
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
        model: SpringModel,
    ) -> Self {
        Self {
            elastic,
            spring_i,
            spring_j,
            model,
            rot_i: 0.0,
            rot_j: 0.0,
            trial_rot_i: 0.0,
            trial_rot_j: 0.0,
        }
    }

    pub fn new_one_component(
        elastic: crate::beam::BeamElement,
        spring_i: Box<dyn UniaxialMaterial>,
        spring_j: Box<dyn UniaxialMaterial>,
    ) -> Self {
        Self::new(elastic, spring_i, spring_j, SpringModel::OneComponent)
    }
}

fn condense_springs(k_elem: &LocalMat, k_i: f64, k_j: f64) -> LocalMat {
    let n = 14;
    let mut k = vec![0.0; n * n];

    let map14 = |i: usize| -> usize {
        match i {
            0..=3 => i,
            4 => 12,
            5 => 5,
            6..=9 => i,
            10 => 13,
            11 => 11,
            _ => i,
        }
    };
    for i in 0..12 {
        for j in 0..12 {
            k[map14(i) * n + map14(j)] = k_elem.get(i, j);
        }
    }

    let ext_rot = [4usize, 10];
    let int_rot = [12usize, 13];
    for (idx, &er) in ext_rot.iter().enumerate() {
        let ir = int_rot[idx];
        let ks = if idx == 0 { k_i } else { k_j };
        k[er * n + er] += ks;
        k[ir * n + ir] += ks;
        k[er * n + ir] -= ks;
        k[ir * n + er] -= ks;
    }

    let na = 12;
    let nb = 2;
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

fn compute_kstar(elastic: &crate::beam::BeamElement, kti: f64, ktj: f64) -> LocalMat {
    let k_raw = elastic.local_stiffness_raw();
    let k_end = condense_springs(&k_raw, kti, ktj);
    let li = elastic.rigid.length_i;
    let lj = elastic.rigid.length_j;
    if li.abs() > 1e-12 || lj.abs() > 1e-12 {
        elastic.apply_rigid_zone_transform(&k_end, li, lj)
    } else {
        k_end
    }
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

        let k_local = match self.model {
            SpringModel::OneComponent => compute_kstar(&self.elastic, kti, ktj),
            SpringModel::TwoComponent => unimplemented!(
                "TwoComponent spring model is not yet implemented (P5 §3). Use OneComponent."
            ),
        };
        // 静縮約済みローカル剛性をグローバル節点系へ回転
        self.elastic.axis.to_global(&k_local)
    }

    fn internal_force(&self, _state: &ElemState, _ctx: &Ctx) -> LocalVec {
        let kti = {
            let mut m = self.spring_i.clone_box();
            m.trial(self.trial_rot_i).1
        };
        let ktj = {
            let mut m = self.spring_j.clone_box();
            m.trial(self.trial_rot_j).1
        };

        let k_local = match self.model {
            SpringModel::OneComponent => compute_kstar(&self.elastic, kti, ktj),
            SpringModel::TwoComponent => unimplemented!(
                "TwoComponent spring model is not yet implemented (P5 §3). Use OneComponent."
            ),
        };
        // committed_disp はグローバル系のため、グローバル剛性で内力を評価する。
        let k_node = self.elastic.axis.to_global(&k_local);

        let u = &self.elastic.committed_disp;
        let mut f = LocalVec {
            data: SmallVec::from_elem(0.0, 12),
        };
        for i in 0..12 {
            let mut s = 0.0;
            for j in 0..12 {
                s += k_node.get(i, j) * u[j];
            }
            f.data[i] = s;
        }
        f
    }

    fn update_state(&mut self, du: &LocalVec, commit: bool, _ctx: &Ctx) {
        // 端ばねはローカル ry（DOF 4,10）に作用するため、グローバル du を
        // ローカル系へ回転してから回転増分を取り出す。
        // elastic.committed_disp 側はグローバル系で蓄積（internal_force と整合）。
        let du_global: [f64; 12] = std::array::from_fn(|i| du.data[i]);
        let du_local = self.elastic.axis.rotate_to_local(&du_global);
        if commit {
            self.elastic.update_state(du, true, _ctx);
            self.rot_i += du_local[4];
            self.rot_j += du_local[10];
            self.spring_i.trial(self.rot_i);
            self.spring_i.commit();
            self.spring_j.trial(self.rot_j);
            self.spring_j.commit();
            self.trial_rot_i = self.rot_i;
            self.trial_rot_j = self.rot_j;
        } else {
            self.elastic.update_state(du, false, _ctx);
            self.trial_rot_i = self.rot_i + du_local[4];
            self.trial_rot_j = self.rot_j + du_local[10];
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
        Box::new((
            materials,
            self.rot_i,
            self.rot_j,
            self.trial_rot_i,
            self.trial_rot_j,
        ))
    }

    fn restore_state(&mut self, state: &dyn Any) {
        if let Some(snapshot) =
            state.downcast_ref::<(Vec<Box<dyn UniaxialMaterial>>, f64, f64, f64, f64)>()
        {
            if snapshot.0.len() == 2 {
                self.spring_i = snapshot.0[0].clone_box();
                self.spring_j = snapshot.0[1].clone_box();
            }
            self.rot_i = snapshot.1;
            self.rot_j = snapshot.2;
            self.trial_rot_i = snapshot.3;
            self.trial_rot_j = snapshot.4;
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

#[cfg(test)]
mod tests {
    use super::*;
    use sc_core::model::RigidZone;
    use approx::assert_relative_eq;
    use sc_core::ids::{ElemId, NodeId};
    use sc_material::uniaxial::Bilinear;

    fn make_test_beam() -> crate::beam::BeamElement {
        crate::beam::BeamElement {
            id: ElemId(0),
            e: 205000.0,
            g: 78846.15,
            a: 80000.0,
            iy: 1.0666667e9,
            iz: 1.0666667e9,
            j: 0.0,
            as_y: 66666.67,
            as_z: 66666.67,
            length: 3000.0,
            density: 0.0,
            nodes: [NodeId(0), NodeId(1)],
            axis: crate::transform::LocalFrame {
                rot: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            },
            rigid: RigidZone::default(),
            end_cond: [
                sc_core::model::EndCondition::Fixed,
                sc_core::model::EndCondition::Fixed,
            ],
            eval_sections: vec![0.0, 0.5, 1.0],
            section: None,
            material: None,
            committed_disp: [0.0; 12],
        }
    }

    fn make_test_element() -> ConcentratedSpringBeam {
        let elastic = make_test_beam();
        let spring_i = Box::new(Bilinear::new(1.0e10, 1.0e20, 0.01));
        let spring_j = Box::new(Bilinear::new(1.0e10, 1.0e20, 0.01));
        ConcentratedSpringBeam::new_one_component(elastic, spring_i, spring_j)
    }

    fn make_yield_element() -> ConcentratedSpringBeam {
        let mut elastic = make_test_beam();
        elastic.iz = 1.0e8;
        elastic.iy = 1.0e8;
        let spring_i = Box::new(Bilinear::new(1.0e12, 1.0e7, 0.01));
        let spring_j = Box::new(Bilinear::new(1.0e12, 1.0e7, 0.01));
        ConcentratedSpringBeam::new_one_component(elastic, spring_i, spring_j)
    }

    #[test]
    fn test_internal_force_no_double_count() {
        let mut elem = make_test_element();
        let ctx = Ctx {
            model: &sc_core::model::Model::default(),
        };
        let state = ElemState::default();
        let du = LocalVec {
            data: smallvec::smallvec![
                0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, -0.001, 0.0
            ],
        };
        elem.update_state(&du, true, &ctx);
        let k = elem.tangent_stiffness(&state, &ctx);
        let f = elem.internal_force(&state, &ctx);
        let mut k_u = [0.0; 12];
        for i in 0..12 {
            let mut s = 0.0;
            for j in 0..12 {
                s += k.get(i, j) * elem.elastic.committed_disp[j];
            }
            k_u[i] = s;
        }
        for i in 0..12 {
            assert_relative_eq!(f.data[i], k_u[i], epsilon = 1.0);
        }
    }

    #[test]
    fn test_dof_only_ry() {
        let mut elem = make_test_element();
        let ctx = Ctx {
            model: &sc_core::model::Model::default(),
        };
        let state = ElemState::default();
        let du = LocalVec {
            data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        };
        elem.update_state(&du, true, &ctx);
        let f = elem.internal_force(&state, &ctx);
        assert!(f.data[3].abs() < 1.0, "rx_i should not have spring moment");
        assert!(f.data[5].abs() < 1.0, "rz_i should not have spring moment");
        assert!(f.data[9].abs() < 1.0, "rx_j should not have spring moment");
        assert!(f.data[11].abs() < 1.0, "rz_j should not have spring moment");

        let k = elem.tangent_stiffness(&state, &ctx);
        let k_sym = |i: usize, j: usize| {
            if k.get(i, j) != k.get(j, i) {
                (k.get(i, j) - k.get(j, i)).abs() < 1e-6
            } else {
                true
            }
        };
        for i in 0..12 {
            for j in 0..12 {
                assert!(k_sym(i, j), "K[{i}][{j}] != K[{j}][{i}]");
            }
        }
    }

    #[test]
    fn test_spring_yield() {
        let mut elem = make_yield_element();
        let ctx = Ctx {
            model: &sc_core::model::Model::default(),
        };
        let state = ElemState::default();

        let rot_yield = 1.0e7 / 1.0e12;
        let du_large = LocalVec {
            data: smallvec::smallvec![
                0.0,
                0.0,
                0.0,
                0.0,
                rot_yield * 10.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0
            ],
        };

        let k_elastic = elem.tangent_stiffness(&state, &ctx);

        elem.update_state(&du_large, true, &ctx);
        let k_yielded = elem.tangent_stiffness(&state, &ctx);

        let k44_elastic = k_elastic.get(4, 4);
        let k44_yielded = k_yielded.get(4, 4);
        assert!(
            k44_yielded < k44_elastic * 0.99,
            "yielded tangent should drop: elastic={} yielded={}",
            k44_elastic,
            k44_yielded
        );
    }

    #[test]
    fn test_commit_revert() {
        let mut elem = make_test_element();
        let ctx = Ctx {
            model: &sc_core::model::Model::default(),
        };
        let du = LocalVec {
            data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        };

        elem.update_state(&du, false, &ctx);
        assert_relative_eq!(elem.trial_rot_i, 0.001, epsilon = 1e-12);
        assert_relative_eq!(elem.rot_i, 0.0, epsilon = 1e-12);
        elem.revert_state();
        assert_relative_eq!(elem.trial_rot_i, 0.0, epsilon = 1e-12);
        assert_relative_eq!(elem.rot_i, 0.0, epsilon = 1e-12);

        elem.update_state(&du, false, &ctx);
        elem.commit_state();
        assert_relative_eq!(elem.trial_rot_i, 0.001, epsilon = 1e-12);
        assert_relative_eq!(elem.rot_i, 0.001, epsilon = 1e-12);

        let du2 = LocalVec {
            data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        };
        elem.update_state(&du2, false, &ctx);
        assert_relative_eq!(elem.trial_rot_i, 0.003, epsilon = 1e-12);
        elem.revert_state();
        assert_relative_eq!(elem.trial_rot_i, 0.001, epsilon = 1e-12);
        assert_relative_eq!(elem.rot_i, 0.001, epsilon = 1e-12);
    }

    #[test]
    fn test_snapshot_restore() {
        let mut elem = make_test_element();
        let ctx = Ctx {
            model: &sc_core::model::Model::default(),
        };
        let du = LocalVec {
            data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        };

        elem.update_state(&du, true, &ctx);
        let snap = elem.snapshot_state();

        let du2 = LocalVec {
            data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        };
        elem.update_state(&du2, false, &ctx);
        assert_relative_eq!(elem.trial_rot_i, 0.003, epsilon = 1e-12);

        elem.restore_state(&*snap);
        assert_relative_eq!(elem.rot_i, 0.001, epsilon = 1e-12);
        assert_relative_eq!(elem.trial_rot_i, 0.001, epsilon = 1e-12);
    }

    #[test]
    fn test_tangent_stiffness_symmetric() {
        let mut elem = make_test_element();
        let ctx = Ctx {
            model: &sc_core::model::Model::default(),
        };
        let state = ElemState::default();

        let du = LocalVec {
            data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        };
        elem.update_state(&du, true, &ctx);
        let k = elem.tangent_stiffness(&state, &ctx);
        for i in 0..12 {
            for j in 0..12 {
                assert!(
                    (k.get(i, j) - k.get(j, i)).abs() < 1e-6,
                    "K[{i}][{j}] != K[{j}][{i}]: {} vs {}",
                    k.get(i, j),
                    k.get(j, i)
                );
            }
        }
    }

    #[test]
    fn test_spring_model_default() {
        let elem = make_test_element();
        assert_eq!(elem.model, SpringModel::OneComponent);
        let k_node = compute_kstar(&elem.elastic, 1.0e10, 1.0e10);
        let u = &elem.elastic.committed_disp;
        let mut f = [0.0; 12];
        for i in 0..12 {
            let mut s = 0.0;
            for j in 0..12 {
                s += k_node.get(i, j) * u[j];
            }
            f[i] = s;
        }
        assert!(
            f.iter().all(|&v| v.abs() < 1e-12),
            "zero disp => zero force"
        );
    }

    #[test]
    fn test_condense_springs_zero_stiffness() {
        let beam = make_test_beam();
        let k_raw = beam.local_stiffness_raw();
        let k_pinned = condense_springs(&k_raw, 0.0, 0.0);
        let k_fixed = condense_springs(&k_raw, 1e30, 1e30);
        assert!(
            k_pinned.get(4, 4) < k_fixed.get(4, 4) * 0.5,
            "pinned ry_i should be much softer than fixed"
        );
    }

    #[test]
    fn test_rx_rz_unaffected_by_spring() {
        let beam = make_test_beam();
        let k_raw = beam.local_stiffness_raw();
        let k_soft = condense_springs(&k_raw, 1.0, 1.0);
        let k_stiff = condense_springs(&k_raw, 1e30, 1e30);
        for &dof in &[3, 5, 9, 11] {
            assert_relative_eq!(k_soft.get(dof, dof), k_stiff.get(dof, dof), epsilon = 1.0);
        }
    }
}
