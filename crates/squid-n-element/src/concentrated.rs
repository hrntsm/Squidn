use crate::beam::invert_small;
use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalMat, LocalVec, MassOption};
use squid_n_core::dof::{DofMap, DOF_PER_NODE};

use smallvec::SmallVec;
use squid_n_material::uniaxial::UniaxialMaterial;
use std::any::Any;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SpringModel {
    OneComponent,
    TwoComponent,
}

/// 端バネの N-M 相関パラメータ（2バネ連成の線形相関）。
/// 現在軸力 N に応じて回転バネの降伏モーメントを
/// M_lim = my0 · (1 − |N|/n_allow) で更新する（下限 0.02·my0）。
#[derive(Clone, Copy, Debug)]
pub struct MnInteraction {
    /// N=0 での降伏モーメント My0 [N·mm]
    pub my0: f64,
    /// 軸許容耐力 [N]（正値。引張・圧縮共通）
    pub n_allow: f64,
}

pub struct ConcentratedSpringBeam {
    pub elastic: crate::beam::BeamElement,
    pub spring_i: Box<dyn UniaxialMaterial>,
    pub spring_j: Box<dyn UniaxialMaterial>,
    pub model: SpringModel,
    /// N-M 相関（None = 従来どおり降伏モーメント一定）
    pub mn: Option<MnInteraction>,
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
            mn: None,
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

    /// N-M 相関を有効化する（ビルダー）。
    pub fn with_mn_interaction(mut self, my0: f64, n_allow: f64) -> Self {
        self.mn = Some(MnInteraction {
            my0,
            n_allow: n_allow.max(1.0),
        });
        self
    }

    /// 現在の軸力 [N]（引張正）。確定変位（＋任意の増分）から
    /// 弾性部の軸ひずみを取り出して評価する。
    fn current_axial_force(&self, du_local: Option<&[f64; 12]>) -> f64 {
        let ul = self
            .elastic
            .axis
            .rotate_to_local(&self.elastic.committed_disp);
        let mut d = ul[6] - ul[0];
        if let Some(du) = du_local {
            d += du[6] - du[0];
        }
        self.elastic.e * self.elastic.a / self.elastic.length.max(1.0) * d
    }

    /// N-M 相関が有効なら、現在軸力に応じて両端バネの降伏モーメントを更新する。
    fn apply_mn_interaction(&mut self, du_local: Option<&[f64; 12]>) {
        let Some(mn) = self.mn else {
            return;
        };
        let n = self.current_axial_force(du_local);
        let m_lim = (mn.my0 * (1.0 - n.abs() / mn.n_allow)).max(0.02 * mn.my0);
        self.spring_i.set_yield(m_lim);
        self.spring_j.set_yield(m_lim);
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
        // N-M 相関: バネの trial より先に現在軸力で降伏モーメントを更新する
        self.apply_mn_interaction(Some(&du_local));
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

    fn serialize_checkpoint(&self) -> Vec<u8> {
        #[derive(serde::Serialize, serde::Deserialize)]
        struct ConcentratedSpringCheckpoint {
            rot_i: f64,
            rot_j: f64,
            trial_rot_i: f64,
            trial_rot_j: f64,
            spring_i: Vec<u8>,
            spring_j: Vec<u8>,
        }
        let cp = ConcentratedSpringCheckpoint {
            rot_i: self.rot_i,
            rot_j: self.rot_j,
            trial_rot_i: self.trial_rot_i,
            trial_rot_j: self.trial_rot_j,
            spring_i: self.spring_i.serialize_state(),
            spring_j: self.spring_j.serialize_state(),
        };
        bincode::serialize(&cp).expect("serialize checkpoint")
    }

    fn deserialize_checkpoint(&mut self, data: &[u8]) {
        #[derive(serde::Serialize, serde::Deserialize)]
        struct ConcentratedSpringCheckpoint {
            rot_i: f64,
            rot_j: f64,
            trial_rot_i: f64,
            trial_rot_j: f64,
            spring_i: Vec<u8>,
            spring_j: Vec<u8>,
        }
        if let Ok(cp) = bincode::deserialize::<ConcentratedSpringCheckpoint>(data) {
            self.rot_i = cp.rot_i;
            self.rot_j = cp.rot_j;
            self.trial_rot_i = cp.trial_rot_i;
            self.trial_rot_j = cp.trial_rot_j;
            self.spring_i.deserialize_state(&cp.spring_i);
            self.spring_j.deserialize_state(&cp.spring_j);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use squid_n_core::ids::{ElemId, NodeId};
    use squid_n_core::model::RigidZone;
    use squid_n_material::uniaxial::Bilinear;

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
                squid_n_core::model::EndCondition::Fixed,
                squid_n_core::model::EndCondition::Fixed,
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

    /// Box<dyn UniaxialMaterial> から Bilinear の降伏値を読み出す（テスト用）。
    fn spring_fy(spring: &dyn UniaxialMaterial) -> f64 {
        let mut b = Bilinear::new(1.0, 1.0, 0.0);
        b.deserialize_state(&spring.serialize_state());
        b.fy
    }

    #[test]
    fn test_internal_force_no_double_count() {
        let mut elem = make_test_element();
        let ctx = Ctx {
            model: &squid_n_core::model::Model::default(),
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
            model: &squid_n_core::model::Model::default(),
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
            model: &squid_n_core::model::Model::default(),
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
            model: &squid_n_core::model::Model::default(),
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
            model: &squid_n_core::model::Model::default(),
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
            model: &squid_n_core::model::Model::default(),
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

    #[test]
    fn test_concentrated_spring_checkpoint_roundtrip() {
        let mut elem = make_test_element();
        let ctx = Ctx {
            model: &squid_n_core::model::Model::default(),
        };
        let du = LocalVec {
            data: smallvec::smallvec![
                0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0005, 0.0
            ],
        };
        elem.update_state(&du, true, &ctx);

        let snap_before = elem.snapshot_state();
        let checkpoint = elem.serialize_checkpoint();

        let mut restored = make_test_element();
        restored.deserialize_checkpoint(&checkpoint);
        let snap_after = restored.snapshot_state();

        // スナップショットの型で比較（Vec<Box<dyn UniaxialMaterial>>, f64, f64, f64, f64）
        let before = snap_before
            .downcast_ref::<(Vec<Box<dyn UniaxialMaterial>>, f64, f64, f64, f64)>()
            .unwrap();
        let after = snap_after
            .downcast_ref::<(Vec<Box<dyn UniaxialMaterial>>, f64, f64, f64, f64)>()
            .unwrap();
        assert_relative_eq!(before.1, after.1, epsilon = 1e-12);
        assert_relative_eq!(before.2, after.2, epsilon = 1e-12);
        assert_relative_eq!(before.3, after.3, epsilon = 1e-12);
        assert_relative_eq!(before.4, after.4, epsilon = 1e-12);
    }
    #[test]
    fn test_mn_interaction_reduces_spring_yield() {
        // 軸力 |N| = 0.5·n_allow で降伏モーメントが my0 の半分に更新される
        let my0 = 1.0e7;
        let elastic = make_test_beam(); // E=205000, A=80000, L=3000 → EA/L=5.4667e6
        let ea_over_l = 205000.0 * 80000.0 / 3000.0;
        let n_allow = ea_over_l; // 軸変位 0.5mm で |N|/n_allow = 0.5 になるよう設定
        let spring_i = Box::new(Bilinear::new(1.0e12, my0, 0.01));
        let spring_j = Box::new(Bilinear::new(1.0e12, my0, 0.01));
        let mut elem = ConcentratedSpringBeam::new_one_component(elastic, spring_i, spring_j)
            .with_mn_interaction(my0, n_allow);

        let ctx = Ctx {
            model: &squid_n_core::model::Model::default(),
        };
        // j端に軸方向（ローカルx=グローバルx）圧縮変位 0.5mm
        let du = LocalVec {
            data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
        };
        elem.update_state(&du, false, &ctx);
        assert_relative_eq!(spring_fy(&*elem.spring_i), 0.5 * my0, max_relative = 1e-9);
        assert_relative_eq!(spring_fy(&*elem.spring_j), 0.5 * my0, max_relative = 1e-9);

        // 引張でも同じ低減（|N| 基準）。trial は非累積（rot と同様に committed 基準）
        let du_t = LocalVec {
            data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
        };
        elem.update_state(&du_t, false, &ctx);
        assert_relative_eq!(spring_fy(&*elem.spring_i), 0.5 * my0, max_relative = 1e-9);
    }

    #[test]
    fn test_mn_interaction_disabled_keeps_yield() {
        // mn 未設定なら軸力がかかっても降伏モーメントは変わらない
        let my0 = 1.0e7;
        let elastic = make_test_beam();
        let spring_i = Box::new(Bilinear::new(1.0e12, my0, 0.01));
        let spring_j = Box::new(Bilinear::new(1.0e12, my0, 0.01));
        let mut elem = ConcentratedSpringBeam::new_one_component(elastic, spring_i, spring_j);
        let ctx = Ctx {
            model: &squid_n_core::model::Model::default(),
        };
        let du = LocalVec {
            data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
        };
        elem.update_state(&du, false, &ctx);
        assert_relative_eq!(spring_fy(&*elem.spring_i), my0, max_relative = 1e-12);
    }

    #[test]
    fn test_mn_interaction_floor_at_high_axial() {
        // |N| が n_allow を超えても降伏モーメントは 0.02·my0 で下げ止まる
        let my0 = 1.0e7;
        let elastic = make_test_beam();
        let ea_over_l = 205000.0 * 80000.0 / 3000.0;
        let spring_i = Box::new(Bilinear::new(1.0e12, my0, 0.01));
        let spring_j = Box::new(Bilinear::new(1.0e12, my0, 0.01));
        let mut elem = ConcentratedSpringBeam::new_one_component(elastic, spring_i, spring_j)
            .with_mn_interaction(my0, ea_over_l);
        let ctx = Ctx {
            model: &squid_n_core::model::Model::default(),
        };
        let du = LocalVec {
            data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        };
        elem.update_state(&du, false, &ctx);
        assert_relative_eq!(spring_fy(&*elem.spring_i), 0.02 * my0, max_relative = 1e-9);
    }

    #[test]
    fn test_mn_interaction_yield_moment_in_response() {
        // 降伏後のバネモーメント上限が M_lim に低減されることを応答で確認:
        // 軸圧縮 0.5mm（M_lim = 0.5·my0）の状態で大回転を与えると、
        // バネの trial モーメントは ≈ M_lim で頭打ちになる
        let my0 = 1.0e7;
        let elastic = make_test_beam();
        let ea_over_l = 205000.0 * 80000.0 / 3000.0;
        let spring_i = Box::new(Bilinear::new(1.0e12, my0, 0.0));
        let spring_j = Box::new(Bilinear::new(1.0e12, my0, 0.0));
        let mut elem = ConcentratedSpringBeam::new_one_component(elastic, spring_i, spring_j)
            .with_mn_interaction(my0, ea_over_l);
        let ctx = Ctx {
            model: &squid_n_core::model::Model::default(),
        };
        // 軸圧縮 + i端大回転を同時に与える
        let du = LocalVec {
            data: smallvec::smallvec![0.0, 0.0, 0.0, 0.0, 0.1, 0.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
        };
        elem.update_state(&du, false, &ctx);
        // バネ i の trial 応力（モーメント）は M_lim = 0.5·my0 で飽和
        let (m, _) = elem.spring_i.clone_box().trial(0.1);
        assert_relative_eq!(m, 0.5 * my0, max_relative = 1e-6);
    }
}
