use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalMat, LocalVec, MassOption};
use sc_core::dof::DofMap;
use sc_core::ids::NodeId;
use sc_material::uniaxial::UniaxialMaterial;
use sc_section::fiber::FiberSection;
use smallvec::SmallVec;
use std::any::Any;

pub struct GaussPoint {
    pub xi: f64,
    pub weight: f64,
    pub section: FiberSection,
    pub mats: Vec<Box<dyn UniaxialMaterial>>,
    pub trial_stress: Vec<f64>,
    pub trial_et: Vec<f64>,
}

impl GaussPoint {
    pub fn new(
        xi: f64,
        weight: f64,
        section: FiberSection,
        mats: Vec<Box<dyn UniaxialMaterial>>,
    ) -> Self {
        let n = section.fibers.len();
        GaussPoint {
            xi,
            weight,
            section,
            mats,
            trial_stress: vec![0.0; n],
            trial_et: vec![0.0; n],
        }
    }
}

pub struct FiberBeam {
    pub length: f64,
    pub nodes: [NodeId; 2],
    pub gauss_points: Vec<GaussPoint>,
    pub shear: crate::shear_spring::ShearSpring,
    pub committed_disp: [f64; 12],
    pub trial_disp: [f64; 12],
}

impl FiberBeam {
    pub fn new(data: &sc_core::model::ElementData, model: &sc_core::model::Model) -> Self {
        let n0 = &model.nodes[data.nodes[0].index()];
        let n1 = &model.nodes[data.nodes[1].index()];
        let dx = n1.coord[0] - n0.coord[0];
        let dy = n1.coord[1] - n0.coord[1];
        let dz = n1.coord[2] - n0.coord[2];
        let length = (dx * dx + dy * dy + dz * dz).sqrt();

        let sec = data.section.and_then(|sid| model.sections.get(sid.index()));
        let mat = data
            .material
            .and_then(|mid| model.materials.get(mid.index()));
        let _e = mat.map(|m| m.young).unwrap_or(205000.0);
        let g = mat.map(|m| m.shear_modulus()).unwrap_or(78846.0);
        let area = sec.map(|s| s.area).unwrap_or(0.0);
        let as_y = sec.map(|s| s.as_y).unwrap_or(area * 5.0 / 6.0);
        let as_z = sec.map(|s| s.as_z).unwrap_or(area * 5.0 / 6.0);

        let shear = crate::shear_spring::ShearSpring::new(length, g, as_y, as_z);

        let gauss = vec![
            GaussPoint::new(
                -0.5773502691896257,
                1.0,
                FiberSection { fibers: vec![] },
                vec![],
            ),
            GaussPoint::new(
                0.5773502691896257,
                1.0,
                FiberSection { fibers: vec![] },
                vec![],
            ),
        ];

        FiberBeam {
            length,
            nodes: [data.nodes[0], data.nodes[1]],
            gauss_points: gauss,
            shear,
            committed_disp: [0.0; 12],
            trial_disp: [0.0; 12],
        }
    }

    fn beam_global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        let mut gdofs = SmallVec::new();
        for &n in &self.nodes {
            let ni = n.index();
            for d in 0..6 {
                let g = ni * 6 + d;
                gdofs.push(dof.active(g).map(|a| a as usize).unwrap_or(usize::MAX));
            }
        }
        gdofs
    }

    fn section_response_from_cache(gp: &GaussPoint) -> ([f64; 3], [[f64; 3]; 3]) {
        let mut force = [0.0; 3];
        let mut stiff = [[0.0; 3]; 3];
        for (i, fiber) in gp.section.fibers.iter().enumerate() {
            let a = fiber.area;
            let sigma = gp.trial_stress[i];
            let et = gp.trial_et[i];
            force[0] += sigma * a;
            force[1] += sigma * a * fiber.z;
            force[2] += -sigma * a * fiber.y;
            stiff[0][0] += et * a;
            stiff[0][1] += et * a * fiber.z;
            stiff[0][2] += -et * a * fiber.y;
            stiff[1][1] += et * a * fiber.z * fiber.z;
            stiff[1][2] += -et * a * fiber.y * fiber.z;
            stiff[2][2] += et * a * fiber.y * fiber.y;
        }
        stiff[1][0] = stiff[0][1];
        stiff[2][0] = stiff[0][2];
        stiff[2][1] = stiff[1][2];
        (force, stiff)
    }
}

impl ElementBehavior for FiberBeam {
    fn n_dof(&self) -> usize {
        12
    }

    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        self.beam_global_dofs(dof)
    }

    fn tangent_stiffness(&self, _state: &ElemState, _ctx: &Ctx) -> LocalMat {
        let mut k = LocalMat::zeros(12);
        let l = self.length;
        if l <= 0.0 {
            return k;
        }
        let half = l / 2.0;

        for gp in &self.gauss_points {
            let (_, d) = Self::section_response_from_cache(gp);
            let w = gp.weight * half;

            let b00 = -1.0 / l;
            let b06 = 1.0 / l;
            let b14 = -1.0 / l;
            let b110 = 1.0 / l;
            let b25 = -1.0 / l;
            let b211 = 1.0 / l;

            let d00 = d[0][0];
            let d01 = d[0][1];
            let d02 = d[0][2];
            let d11 = d[1][1];
            let d12 = d[1][2];
            let d22 = d[2][2];

            let add = |k: &mut LocalMat, i: usize, j: usize, v: f64| {
                let old = k.get(i, j);
                k.set(i, j, old + v);
            };

            let s = |k: &mut LocalMat, i: usize, j: usize, v: f64| {
                add(k, i, j, v);
                if i != j {
                    add(k, j, i, v);
                }
            };

            s(&mut k, 0, 0, b00 * d00 * b00 * w);
            s(&mut k, 0, 6, b00 * d00 * b06 * w);
            s(&mut k, 6, 6, b06 * d00 * b06 * w);

            s(&mut k, 0, 4, b00 * d01 * b14 * w);
            s(&mut k, 0, 10, b00 * d01 * b110 * w);
            s(&mut k, 6, 4, b06 * d01 * b14 * w);
            s(&mut k, 6, 10, b06 * d01 * b110 * w);

            s(&mut k, 0, 5, b00 * d02 * b25 * w);
            s(&mut k, 0, 11, b00 * d02 * b211 * w);
            s(&mut k, 6, 5, b06 * d02 * b25 * w);
            s(&mut k, 6, 11, b06 * d02 * b211 * w);

            s(&mut k, 4, 4, b14 * d11 * b14 * w);
            s(&mut k, 4, 10, b14 * d11 * b110 * w);
            s(&mut k, 10, 10, b110 * d11 * b110 * w);

            s(&mut k, 4, 5, b14 * d12 * b25 * w);
            s(&mut k, 4, 11, b14 * d12 * b211 * w);
            s(&mut k, 10, 5, b110 * d12 * b25 * w);
            s(&mut k, 10, 11, b110 * d12 * b211 * w);

            s(&mut k, 5, 5, b25 * d22 * b25 * w);
            s(&mut k, 5, 11, b25 * d22 * b211 * w);
            s(&mut k, 11, 11, b211 * d22 * b211 * w);
        }

        let ks = self.shear.tangent_stiffness(&ElemState::default());
        for i in 0..12 {
            for j in 0..12 {
                let old = k.get(i, j);
                k.set(i, j, old + ks.get(i, j));
            }
        }
        k
    }

    fn internal_force(&self, _state: &ElemState, _ctx: &Ctx) -> LocalVec {
        let mut f = LocalVec {
            data: SmallVec::from_elem(0.0, 12),
        };
        let l = self.length;
        if l <= 0.0 {
            return f;
        }
        let half = l / 2.0;

        for gp in &self.gauss_points {
            let (force, _) = Self::section_response_from_cache(gp);
            let w = gp.weight * half;
            let n = force[0];
            let my = force[1];
            let mz = force[2];

            let b00 = -1.0 / l;
            let b06 = 1.0 / l;
            let b14 = -1.0 / l;
            let b110 = 1.0 / l;
            let b25 = -1.0 / l;
            let b211 = 1.0 / l;

            f.data[0] += b00 * n * w;
            f.data[6] += b06 * n * w;
            f.data[4] += b14 * my * w;
            f.data[10] += b110 * my * w;
            f.data[5] += b25 * mz * w;
            f.data[11] += b211 * mz * w;
        }
        f
    }

    fn update_state(&mut self, du: &LocalVec, commit: bool, _ctx: &Ctx) {
        for i in 0..12 {
            self.trial_disp[i] += du.data[i];
        }
        let l = self.length;
        if l <= 0.0 {
            return;
        }
        let eps0 = (self.trial_disp[6] - self.trial_disp[0]) / l;
        let ky = (self.trial_disp[10] - self.trial_disp[4]) / l;
        let kz = (self.trial_disp[11] - self.trial_disp[5]) / l;

        for gp in &mut self.gauss_points {
            for (i, fiber) in gp.section.fibers.iter().enumerate() {
                let eps = eps0 - kz * fiber.y + ky * fiber.z;
                let (sigma, et) = gp.mats[i].trial(eps);
                gp.trial_stress[i] = sigma;
                gp.trial_et[i] = et;
            }
        }
        if commit {
            for gp in &mut self.gauss_points {
                for mat in &mut gp.mats {
                    mat.commit();
                }
            }
            self.committed_disp = self.trial_disp;
        }
    }

    fn mass_matrix(&self, opt: MassOption) -> LocalMat {
        let m = self
            .gauss_points
            .iter()
            .map(|gp| gp.section.fibers.iter().map(|f| f.area).sum::<f64>() * gp.weight)
            .sum::<f64>()
            * self.length
            / 2.0;

        let density = 1.0;
        let total_mass = density * m;
        let mut mm = LocalMat::zeros(12);
        match opt {
            MassOption::Lumped => {
                for d in [0, 1, 2, 6, 7, 8] {
                    mm.set(d, d, total_mass / 6.0);
                }
            }
            MassOption::Consistent => {
                let c2 = total_mass / 2.0;
                mm.set(0, 0, c2);
                mm.set(0, 6, total_mass / 6.0);
                mm.set(6, 6, c2);
                for d in [1, 2, 7, 8] {
                    mm.set(d, d, c2);
                }
            }
        }
        mm
    }

    fn geometric_stiffness(&self, _n: f64) -> LocalMat {
        LocalMat::zeros(12)
    }

    fn snapshot_state(&self) -> Box<dyn Any> {
        let gauss_data: Vec<Vec<Box<dyn UniaxialMaterial>>> = self
            .gauss_points
            .iter()
            .map(|gp| gp.mats.iter().map(|m| m.clone_box()).collect())
            .collect();
        Box::new((self.trial_disp, self.committed_disp, gauss_data))
    }

    fn restore_state(&mut self, state: &dyn Any) {
        if let Some((trial, committed, mats_data)) =
            state.downcast_ref::<([f64; 12], [f64; 12], Vec<Vec<Box<dyn UniaxialMaterial>>>)>()
        {
            self.trial_disp = *trial;
            self.committed_disp = *committed;
            for (gp, gp_mats) in self.gauss_points.iter_mut().zip(mats_data) {
                for (mat, new_mat) in gp.mats.iter_mut().zip(gp_mats) {
                    *mat = new_mat.clone_box();
                }
            }
        }
    }

    fn commit_state(&mut self) {
        for gp in &mut self.gauss_points {
            for mat in &mut gp.mats {
                mat.commit();
            }
        }
        self.committed_disp = self.trial_disp;
    }

    fn revert_state(&mut self) {
        for gp in &mut self.gauss_points {
            for mat in &mut gp.mats {
                mat.revert();
            }
        }
        self.trial_disp = self.committed_disp;
    }
}
