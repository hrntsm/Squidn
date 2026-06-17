use std::fmt::Debug;

/// 一軸応力–ひずみ履歴則を示すトレイト。
/// trial/commit/revert パターンで非線形解析の試行収束に対応する。
pub trait UniaxialMaterial: Send + Sync + Debug {
    /// 試行ひずみ strain に対する (応力, 接線剛性)。
    /// 状態は内部に試行値として保持。
    fn trial(&mut self, strain: f64) -> (f64, f64);
    /// 試行を確定（収束後にコミット）。
    fn commit(&mut self);
    /// 試行を破棄して直前のコミット状態へ戻す（リジェクト時）。
    fn revert(&mut self);
}

// ──────────────────────────── バイリニア鋼材 ────────────────────────────

/// バイリニア鋼材（弾性＋ひずみ硬化）。
#[derive(Clone, Debug)]
pub struct Bilinear {
    pub e: f64,
    pub fy: f64,
    pub hardening: f64,
    committed: BilinearState,
    trial: BilinearState,
}

#[derive(Clone, Debug)]
struct BilinearState {
    strain: f64,
    stress: f64,
    tangent: f64,
}

impl Bilinear {
    pub fn new(e: f64, fy: f64, hardening: f64) -> Self {
        let init = BilinearState {
            strain: 0.0,
            stress: 0.0,
            tangent: e,
        };
        Self {
            e,
            fy,
            hardening,
            committed: init.clone(),
            trial: init,
        }
    }
}

impl UniaxialMaterial for Bilinear {
    fn trial(&mut self, strain: f64) -> (f64, f64) {
        let de = strain - self.committed.strain;
        let stress_trial = self.committed.stress + self.e * de;
        let abs_stress = stress_trial.abs();
        if abs_stress <= self.fy {
            self.trial = BilinearState {
                strain,
                stress: stress_trial,
                tangent: self.e,
            };
        } else {
            let sgn = if stress_trial >= 0.0 { 1.0 } else { -1.0 };
            let overshoot = abs_stress - self.fy;
            let plastic_strain = overshoot / self.e;
            let stress = sgn * (self.fy + self.hardening * plastic_strain * self.e);
            self.trial = BilinearState {
                strain,
                stress,
                tangent: self.hardening * self.e,
            };
        }
        (self.trial.stress, self.trial.tangent)
    }

    fn commit(&mut self) {
        self.committed = self.trial.clone();
    }

    fn revert(&mut self) {
        self.trial = self.committed.clone();
    }
}

// ──────────────────────────── Menegotto–Pinto 鉄筋 ────────────────────────────

/// Menegotto–Pinto モデル（バウシンガー効果を滑らかに表現）。
#[derive(Clone, Debug)]
pub struct MenegottoPinto {
    pub e: f64,
    pub fy: f64,
    pub b: f64,
    pub r0: f64,
    pub a1: f64,
    pub a2: f64,
    committed: MpState,
    trial: MpState,
}

#[derive(Clone, Debug)]
struct MpState {
    strain: f64,
    stress: f64,
    tangent: f64,
    eps_r: f64,
    sig_r: f64,
    eps_0: f64,
    sig_0: f64,
    xi: f64,
}

impl MenegottoPinto {
    pub fn new(e: f64, fy: f64) -> Self {
        let b = 0.01;
        let init = MpState {
            strain: 0.0,
            stress: 0.0,
            tangent: e,
            eps_r: 0.0,
            sig_r: 0.0,
            eps_0: 0.0,
            sig_0: 0.0,
            xi: 0.0,
        };
        Self {
            e,
            fy,
            b,
            r0: 20.0,
            a1: 18.5,
            a2: 0.15,
            committed: init.clone(),
            trial: init,
        }
    }

    pub fn with_params(e: f64, fy: f64, b: f64, r0: f64, a1: f64, a2: f64) -> Self {
        let init = MpState {
            strain: 0.0,
            stress: 0.0,
            tangent: e,
            eps_r: 0.0,
            sig_r: 0.0,
            eps_0: 0.0,
            sig_0: 0.0,
            xi: 0.0,
        };
        Self {
            e,
            fy,
            b,
            r0,
            a1,
            a2,
            committed: init.clone(),
            trial: init,
        }
    }
}

impl UniaxialMaterial for MenegottoPinto {
    fn trial(&mut self, strain: f64) -> (f64, f64) {
        let c = &self.committed;
        if (strain - c.eps_r).abs() < 1e-15 {
            self.trial = c.clone();
            return (c.stress, c.tangent);
        }
        let deps = c.eps_0 - c.eps_r;
        let dsig = c.sig_0 - c.sig_r;
        if deps.abs() < 1e-15 {
            let stress = self.e * strain;
            let tangent = self.e;
            self.trial = MpState {
                strain,
                stress,
                tangent,
                ..c.clone()
            };
            return (stress, tangent);
        }
        let eps_star = (strain - c.eps_r) / deps;
        let r = self.r0 - self.a1 * c.xi / (self.a2 + c.xi);
        let r = r.max(1.0);
        let eps_star_abs = eps_star.abs();
        let denom = (1.0 + eps_star_abs.powf(r)).powf(1.0 / r);
        let sig_star = self.b * eps_star + (1.0 - self.b) * eps_star / denom;
        let stress = c.sig_r + dsig * sig_star;
        let ddenom = if eps_star_abs > 1e-15 {
            eps_star_abs.powf(r - 1.0) * eps_star.signum()
                / (1.0 + eps_star_abs.powf(r)).powf(1.0 + 1.0 / r)
        } else {
            0.0
        };
        let dsig_star = self.b + (1.0 - self.b) * (1.0 / denom - eps_star * ddenom);
        let tangent = dsig / deps * dsig_star;
        let tangent = tangent.clamp(self.b * self.e, self.e);
        self.trial = MpState {
            strain,
            stress,
            tangent,
            ..c.clone()
        };
        (stress, tangent)
    }

    fn commit(&mut self) {
        let prev = &self.committed;
        let sgn_now = (self.trial.strain - prev.eps_r).signum();
        let sgn_prev = (prev.strain - prev.eps_r).signum();
        if sgn_now != sgn_prev && sgn_prev != 0.0 {
            let new_r = prev.strain;
            let new_s = prev.stress;
            let deps = new_r - prev.eps_r;
            let dsig = new_s - prev.sig_r;
            let xi_new = prev.xi + deps * dsig.signum() - (self.b / (1.0 - self.b)) * dsig / self.e;
            let xi_new = xi_new.abs().max(0.0);
            self.trial.eps_r = new_r;
            self.trial.sig_r = new_s;
            self.trial.eps_0 = new_r + deps;
            self.trial.sig_0 = new_s + dsig;
            self.trial.xi = xi_new;
        }
        self.committed = self.trial.clone();
    }

    fn revert(&mut self) {
        self.trial = self.committed.clone();
    }
}

// ──────────────────────────── コンクリートモデル ────────────────────────────

/// コンクリートの一軸履歴モデル。
/// 圧縮：放物線上昇＋直線軟化、引張：ひび割れ＋テンションスティフニング。
#[derive(Clone, Debug)]
pub struct Concrete {
    pub fc: f64,
    pub ec0: f64,
    pub ecu: f64,
    pub ft: f64,
    pub tension_stiffening: f64,
    committed: ConcreteState,
    trial: ConcreteState,
}

#[derive(Clone, Debug)]
#[allow(dead_code)]
struct ConcreteState {
    strain: f64,
    stress: f64,
    tangent: f64,
    max_comp_strain: f64,
    crack_strain: f64,
    is_cracked: bool,
}

impl Concrete {
    pub fn new(fc: f64, ft: f64) -> Self {
        let init = ConcreteState {
            strain: 0.0,
            stress: 0.0,
            tangent: 0.0,
            max_comp_strain: 0.0,
            crack_strain: 0.0,
            is_cracked: false,
        };
        Self {
            fc,
            ec0: -0.002,
            ecu: -0.0035,
            ft,
            tension_stiffening: 0.5,
            committed: init.clone(),
            trial: init,
        }
    }

    pub fn with_params(fc: f64, ec0: f64, ecu: f64, ft: f64, tension_stiffening: f64) -> Self {
        let init = ConcreteState {
            strain: 0.0,
            stress: 0.0,
            tangent: 0.0,
            max_comp_strain: 0.0,
            crack_strain: 0.0,
            is_cracked: false,
        };
        Self {
            fc,
            ec0,
            ecu,
            ft,
            tension_stiffening,
            committed: init.clone(),
            trial: init,
        }
    }

    fn envelope_compression(&self, strain: f64) -> (f64, f64) {
        if strain >= self.ec0 {
            let ratio = strain / self.ec0;
            let c = 2.0 * ratio - ratio * ratio;
            let stress = -c * self.fc;
            let tangent = -(2.0 - 2.0 * ratio) * self.fc / self.ec0.abs();
            (stress, tangent)
        } else if strain >= self.ecu {
            let slope = 0.15 * self.fc / (self.ec0 - self.ecu);
            let stress = -self.fc + slope * (strain - self.ec0);
            let tangent = slope;
            (stress, tangent)
        } else {
            let stress = -self.fc * (1.0 - 0.15 * (strain - self.ecu) / self.ecu);
            (stress, 0.0)
        }
    }

    fn envelope_tension(&self, strain: f64) -> (f64, f64) {
        let e0 = self.fc.abs() / self.ec0.abs();
        if strain <= 0.0 {
            return (0.0, 0.0);
        }
        let eps_cr = self.ft / e0;
        if strain <= eps_cr {
            let stress = e0 * strain;
            let tangent = e0;
            (stress, tangent)
        } else {
            let stress = self.ft * (-self.tension_stiffening * (strain / eps_cr - 1.0)).exp();
            let tangent = -self.ft
                * (self.tension_stiffening / eps_cr)
                * (-self.tension_stiffening * (strain / eps_cr - 1.0)).exp();
            (stress, tangent)
        }
    }
}

impl UniaxialMaterial for Concrete {
    fn trial(&mut self, strain: f64) -> (f64, f64) {
        let c = &self.committed;
        if strain <= 0.0 {
            let mut max_comp = c.max_comp_strain;
            let (s, e_t) = if strain < max_comp {
                let (env_s, env_t) = self.envelope_compression(strain);
                max_comp = strain;
                (env_s, env_t)
            } else {
                let reload_e = if c.max_comp_strain < 0.0 && c.stress < 0.0 {
                    c.stress / c.max_comp_strain
                } else {
                    self.fc / self.ec0.abs()
                };
                let stress = reload_e * strain;
                (stress, reload_e)
            };
            self.trial = ConcreteState {
                strain,
                stress: s,
                tangent: e_t,
                max_comp_strain: max_comp,
                ..c.clone()
            };
            (s, e_t)
        } else {
            let is_cracked = c.is_cracked || c.crack_strain > 0.0;
            let (s, e_t) = if !is_cracked {
                self.envelope_tension(strain)
            } else {
                let crack_e = if c.crack_strain > 0.0 {
                    let e0 = self.fc.abs() / self.ec0.abs();
                    let eps_cr = self.ft / e0;
                    let sig_ref = self.ft
                        * (-self.tension_stiffening * (c.crack_strain / eps_cr - 1.0)).exp();
                    sig_ref / c.crack_strain
                } else {
                    0.0
                };
                (crack_e * strain, crack_e)
            };
            let crack_strain = if strain > 0.0 && s >= self.ft * 0.9 && !is_cracked {
                strain
            } else {
                c.crack_strain
            };
            self.trial = ConcreteState {
                strain,
                stress: s,
                tangent: e_t,
                is_cracked: is_cracked || strain > c.crack_strain,
                crack_strain: crack_strain.max(c.crack_strain),
                ..c.clone()
            };
            (s, e_t)
        }
    }

    fn commit(&mut self) {
        self.committed = self.trial.clone();
    }

    fn revert(&mut self) {
        self.trial = self.committed.clone();
    }
}

// ──────────────────────────── 既存別名（後方互換） ────────────────────────────

pub type ElasticSteel = Bilinear;
pub type ElasticConcrete = Concrete;

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_bilinear_elastic() {
        let mut s = Bilinear::new(205000.0, 235.0, 0.01);
        let (stress, _) = s.trial(0.001);
        assert_relative_eq!(stress, 205.0, epsilon = 1.0);
    }

    #[test]
    fn test_bilinear_yield() {
        let mut s = Bilinear::new(205000.0, 235.0, 0.01);
        let (stress, _) = s.trial(0.01);
        assert_relative_eq!(
            stress,
            235.0 + 0.01 * 205000.0 * (0.01 - 235.0 / 205000.0),
            epsilon = 1.0
        );
    }

    #[test]
    fn test_bilinear_commit_revert() {
        let mut s = Bilinear::new(205000.0, 235.0, 0.01);
        let (s1, _) = s.trial(0.001);
        s.commit();
        let (s2, _) = s.trial(0.002);
        assert!(s2.abs() > s1.abs());
        s.revert();
        let (s3, _) = s.trial(0.0005);
        assert!(s3.abs() < s1.abs());
    }

    #[test]
    fn test_menegotto_pinto_elastic() {
        let mut mp = MenegottoPinto::new(205000.0, 235.0);
        let (stress, _) = mp.trial(0.001);
        assert_relative_eq!(stress, 205.0, epsilon = 5.0);
    }

    #[test]
    fn test_concrete_compression_envelope() {
        let mut c = Concrete::new(30.0, 2.0);
        let (stress, _) = c.trial(-0.002);
        assert_relative_eq!(stress, -30.0, epsilon = 1.0);
    }

    #[test]
    fn test_concrete_tension_crack() {
        let mut c = Concrete::new(30.0, 2.0);
        let (stress, _) = c.trial(0.00005);
        assert!(stress > 0.0);
        let (stress2, _) = c.trial(0.0005);
        assert!(stress2 < stress);
    }

    #[test]
    fn test_concrete_commit_revert() {
        let mut c = Concrete::new(30.0, 2.0);
        c.trial(-0.001);
        c.commit();
        c.trial(-0.002);
        c.revert();
        let (stress, _) = c.trial(-0.0005);
        assert!(stress < 0.0);
    }
}
