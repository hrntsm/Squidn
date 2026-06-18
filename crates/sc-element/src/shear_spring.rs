use crate::behavior::{ElemState, LocalMat, MassOption};
use sc_skeleton::MemberSkeleton;

/// 独立せん断ばね（設計書 §6.3/R26）。
/// 軸–曲げと別系統のせん断変形を表現。スケルトンは P4 sc-skeleton から供給。
pub struct ShearSpring {
    pub skeleton: MemberSkeleton,
    pub k_shear_y: f64,
    pub k_shear_z: f64,
}

impl ShearSpring {
    pub fn new(length: f64, g: f64, as_y: f64, as_z: f64) -> Self {
        let k_shear_y = if length > 1e-12 {
            g * as_y / length
        } else {
            0.0
        };
        let k_shear_z = if length > 1e-12 {
            g * as_z / length
        } else {
            0.0
        };
        ShearSpring {
            skeleton: MemberSkeleton::default(),
            k_shear_y,
            k_shear_z,
        }
    }

    /// 12×12 剛性行列（せん断成分のみ）
    pub fn stiffness_12x12(&self) -> LocalMat {
        let mut k = LocalMat::zeros(12);
        let ky = self.k_shear_y;
        let kz = self.k_shear_z;
        k.set(1, 1, ky);
        k.set(7, 7, ky);
        k.set(1, 7, -ky);
        k.set(7, 1, -ky);
        k.set(2, 2, kz);
        k.set(8, 8, kz);
        k.set(2, 8, -kz);
        k.set(8, 2, -kz);
        k
    }

    pub fn tangent_stiffness(&self, _state: &ElemState) -> LocalMat {
        self.stiffness_12x12()
    }

    pub fn mass_matrix(&self, _opt: MassOption) -> LocalMat {
        LocalMat::zeros(12)
    }
}
