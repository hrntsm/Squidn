//! 弾性剛性行列 12×12 の構築。
//!
//! Timoshenko 梁の raw 剛性、剛域変換、端部回転ばねの静縮約を経て節点自由度の
//! 局所剛性 [`BeamElement::local_stiffness`] を組み立てる。

use super::element::BeamElement;
use super::linalg::invert_small;
use crate::behavior::LocalMat;
use squid_n_core::model::EndCondition;

impl BeamElement {
    pub fn local_stiffness_raw(&self) -> LocalMat {
        let (e, g, a, iy, iz, jj, l) = (
            self.e,
            self.g,
            self.a,
            self.iy,
            self.iz,
            self.j,
            self.length,
        );
        if l < 1e-12 {
            return LocalMat::zeros(12);
        }
        let phiz = 12.0 * e * iz / (g * self.as_y * l * l);
        let phiy = 12.0 * e * iy / (g * self.as_z * l * l);
        let az = e * iz / ((1.0 + phiz) * l * l * l);
        let ay = e * iy / ((1.0 + phiy) * l * l * l);

        let mut k = LocalMat::zeros(12);
        let mut s = |i: usize, j: usize, v: f64| {
            k.set(i, j, v);
            if i != j {
                k.set(j, i, v);
            }
        };

        s(0, 0, e * a / l);
        s(6, 6, e * a / l);
        s(0, 6, -e * a / l);
        s(3, 3, g * jj / l);
        s(9, 9, g * jj / l);
        s(3, 9, -g * jj / l);

        s(1, 1, 12.0 * az);
        s(7, 7, 12.0 * az);
        s(1, 7, -12.0 * az);
        s(1, 5, 6.0 * az * l);
        s(1, 11, 6.0 * az * l);
        s(5, 7, -6.0 * az * l);
        s(7, 11, -6.0 * az * l);
        s(5, 5, (4.0 + phiz) * az * l * l);
        s(11, 11, (4.0 + phiz) * az * l * l);
        s(5, 11, (2.0 - phiz) * az * l * l);

        s(2, 2, 12.0 * ay);
        s(8, 8, 12.0 * ay);
        s(2, 8, -12.0 * ay);
        s(2, 4, -6.0 * ay * l);
        s(2, 10, -6.0 * ay * l);
        s(4, 8, 6.0 * ay * l);
        s(8, 10, 6.0 * ay * l);
        s(4, 4, (4.0 + phiy) * ay * l * l);
        s(10, 10, (4.0 + phiy) * ay * l * l);
        s(4, 10, (2.0 - phiy) * ay * l * l);

        k
    }

    pub(crate) fn apply_rigid_zone_transform(
        &self,
        k_flex: &LocalMat,
        li: f64,
        lj: f64,
    ) -> LocalMat {
        if li.abs() < 1e-12 && lj.abs() < 1e-12 {
            return LocalMat {
                n: k_flex.n,
                data: k_flex.data.clone(),
            };
        }
        // Tr: 12×12 — flex端自由度(i', j') → 節点自由度(i, j)
        // i' = i を li だけずらし, j' = j を lj だけずらす
        // Tr はほとんど単位行列。i端: ux_i'=ux_i, uy_i'=uy_i-li*rz_i, uz_i'=uz_i+li*ry_i,
        //   rx_i'=rx_i, ry_i'=ry_i, rz_i'=rz_i
        // j端: ux_j'=ux_j, uy_j'=uy_j+lj*rz_j, uz_j'=uz_j-lj*ry_j,
        //   rx_j'=rx_j, ry_j'=ry_j, rz_j'=rz_j
        let mut tr = LocalMat::zeros(12);
        for i in 0..12 {
            tr.set(i, i, 1.0);
        }
        // i端 (index 0..5): uy方向(1) ← rz方向(5) の項
        tr.set(1, 5, -li);
        tr.set(2, 4, li);
        // j端 (index 6..11): uy方向(7) ← rz方向(11) の項
        tr.set(7, 11, lj);
        tr.set(8, 10, -lj);

        // K_node = Tr^T * K_flex * Tr
        let mut tmp = LocalMat::zeros(12);
        for i in 0..12 {
            for j in 0..12 {
                let mut s = 0.0;
                for k in 0..12 {
                    s += k_flex.get(i, k) * tr.get(k, j);
                }
                tmp.set(i, j, s);
            }
        }
        let mut kn = LocalMat::zeros(12);
        for i in 0..12 {
            for j in 0..12 {
                let mut s = 0.0;
                for k in 0..12 {
                    s += tr.get(k, i) * tmp.get(k, j);
                }
                kn.set(i, j, s);
            }
        }
        kn
    }

    /// 端部回転ばねを「外部回転＋内部回転」の 18 自由度で表し、
    /// 静縮約で 12×12（節点自由度のみ）に戻す。
    /// 18 並び: [外部 0..11（節点 ux,uy,uz,rx,ry,rz ×2）, 内部 12..17（要素端 rx,ry,rz ×2）]
    fn condense_end_springs(&self, k_elem: &LocalMat) -> LocalMat {
        // 18×18 を組む
        let n = 18;
        let mut k = vec![0.0; n * n];

        // 要素剛性: 並進は外部 DOF、回転は内部 DOF へ配置
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

        // 回転ばね: 外部回転 ↔ 内部回転
        // 剛接ペナルティは「部材回転剛性 E·I/L のスケールに対する倍率」で与える。
        // 係数 1e8 なら剛性比 ~1e8（剛接を 8 桁の精度で再現＝結果への影響 ~1e-8<1e-6）
        // でありながら、静縮約 K*=Kaa−Kab·Kbb⁻¹·Kba の丸め誤差（~ペナルティ·eps）が
        // 他剛性成分を下回るため、現実的な大断面（iz≥1e7）でも全体 K が
        // 非正定値化しない。1e12 だと iz が大きいとき誤差が並進剛性を超えて破綻する。
        let rot_scale = self.e * self.iz.max(self.iy) / self.length.max(1.0);
        let spring_stiffness = |cond: &EndCondition| -> f64 {
            match cond {
                EndCondition::Fixed => 1e8 * rot_scale,
                EndCondition::Pinned => 0.0,
                EndCondition::SemiRigid { k_theta } => *k_theta,
            }
        };

        let ext_rot = [3usize, 4, 5, 9, 10, 11];
        let int_rot = [12usize, 13, 14, 15, 16, 17];
        for (idx, &er) in ext_rot.iter().enumerate() {
            let ir = int_rot[idx];
            let kspring = if idx < 3 {
                spring_stiffness(&self.end_cond[0])
            } else {
                spring_stiffness(&self.end_cond[1])
            };
            k[er * n + er] += kspring;
            k[ir * n + ir] += kspring;
            k[er * n + ir] -= kspring;
            k[ir * n + er] -= kspring;
        }

        // 内部 DOF (12..17) を静縮約
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

        // kab_kbbinv = Kab * Kbb^-1
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

    pub fn local_stiffness(&self) -> LocalMat {
        let l_flex = self.length - self.rigid.length_i - self.rigid.length_j;
        let k_raw = if l_flex > 1e-12 {
            let mut beam = self.clone();
            beam.length = l_flex;
            beam.end_cond = [EndCondition::Fixed, EndCondition::Fixed];
            // 軸剛性は剛域で増大させない（軸断面積の l0/l 補正）。可撓長 l0 で
            // 組み立てるため A·(l0/l) を用いると軸剛性が EA/l（節点間長基準）と
            // なり、曲げのみ剛域変換で剛とする扱いに揃う。剛域なしでは補正 1。
            beam.a = self.a * (l_flex / self.length);
            beam.local_stiffness_raw()
        } else {
            LocalMat::zeros(12)
        };

        // 剛域を持たない可とう部で端部ばね静縮約 → 12×12
        let k_end = self.condense_end_springs(&k_raw);

        // 剛域変換で節点自由度へ
        let li = self.rigid.length_i;
        let lj = self.rigid.length_j;
        self.apply_rigid_zone_transform(&k_end, li, lj)
    }
}
