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

    fn deserialize_checkpoint(
        &mut self,
        data: &[u8],
    ) -> Result<(), crate::behavior::CheckpointError> {
        #[derive(serde::Serialize, serde::Deserialize)]
        struct ConcentratedSpringCheckpoint {
            rot_i: f64,
            rot_j: f64,
            trial_rot_i: f64,
            trial_rot_j: f64,
            spring_i: Vec<u8>,
            spring_j: Vec<u8>,
        }
        let cp: ConcentratedSpringCheckpoint = bincode::deserialize(data)
            .map_err(|e| crate::behavior::CheckpointError::Decode(e.to_string()))?;
        self.rot_i = cp.rot_i;
        self.rot_j = cp.rot_j;
        self.trial_rot_i = cp.trial_rot_i;
        self.trial_rot_j = cp.trial_rot_j;
        self.spring_i.deserialize_state(&cp.spring_i)?;
        self.spring_j.deserialize_state(&cp.spring_j)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests;
