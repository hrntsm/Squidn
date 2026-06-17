use sc_core::model::{Material, Section};
use sc_material::{HysteresisRule, UniaxialMaterial};
use sc_section::fiber::{section_response, FiberSection, SectionStrain};

/// 配筋情報（RC）。
#[derive(Clone, Debug)]
pub struct Reinforcement {
    /// 主筋の位置（y, z）[mm] と断面積 [mm²] のリスト
    pub main_bars: Vec<(f64, f64, f64)>,
    /// 帯筋ピッチ [mm]
    pub hoop_pitch: f64,
    /// 帯筋1本の断面積 [mm²]
    pub hoop_area: f64,
}

/// N–M 相関情報。
#[derive(Clone, Debug)]
pub struct AxialInteraction {
    /// 複数軸力レベルでのスケルトン
    pub skeletons: Vec<(f64 /* N */, MemberSkeleton)>,
}

/// 部材スケルトン曲線（トリリニア折れ点）。
#[derive(Clone, Debug)]
pub struct MemberSkeleton {
    /// トリリニア折れ点 (変形 θ, 耐力 M)
    pub points: Vec<(f64, f64)>,
    /// 履歴則
    pub hysteresis: HysteresisRule,
    /// N によるスケルトン補正
    pub axial_dependency: AxialInteraction,
}

/// スケルトン算定に必要な部材情報。
pub struct MemberData<'a> {
    pub section: &'a Section,
    pub reinforcement: &'a Reinforcement,
    pub material: &'a Material,
    pub fibers: &'a FiberSection,
    pub span: f64,
    pub inflection_ratio: f64,
}

/// M–φ カーブ上の点。
#[derive(Clone, Debug)]
struct MPhiPoint {
    curvature: f64,
    moment: f64,
}

/// ファイバ断面から M–φ 関係を数値積分で算定する。
fn compute_m_phi_curve(
    fibers: &FiberSection,
    mats: &mut [Box<dyn UniaxialMaterial>],
    n_axial: f64,
    max_curvature: f64,
    num_steps: usize,
) -> Vec<MPhiPoint> {
    let mut points = Vec::with_capacity(num_steps + 1);
    let dk = max_curvature / num_steps as f64;

    for mat in mats.iter_mut() {
        mat.revert();
    }

    for i in 0..=num_steps {
        let ky = i as f64 * dk;
        for mat in mats.iter_mut() {
            mat.revert();
        }

        let mut n_iter = 0;
        let mut eps0 = 0.0;
        let max_iter = 50;
        let tol = 1e-6;
        loop {
            let strain = SectionStrain { eps0, ky, kz: 0.0 };
            let (force, _) = section_response(fibers, strain, mats);
            let residual = force.n - n_axial;
            if residual.abs() < n_axial.abs().max(1.0) * tol || n_iter >= max_iter {
                break;
            }
            let dndeps0 = {
                let strain_p = SectionStrain {
                    eps0: eps0 + 1e-8,
                    ky,
                    kz: 0.0,
                };
                let (force_p, _) = section_response(fibers, strain_p, mats);
                (force_p.n - force.n) / 1e-8
            };
            if dndeps0.abs() < 1e-15 {
                break;
            }
            eps0 -= residual / dndeps0;
            n_iter += 1;
        }

        let strain = SectionStrain { eps0, ky, kz: 0.0 };
        let (force, _) = section_response(fibers, strain, mats);
        points.push(MPhiPoint {
            curvature: ky,
            moment: force.my,
        });
    }

    points
}

/// M–φ 曲線からトリリニア折れ点を抽出する。
fn extract_trilinear(mphi: &[MPhiPoint]) -> Vec<(f64 /* θ */, f64 /* M */)> {
    if mphi.is_empty() {
        return vec![(0.0, 0.0)];
    }

    let n = mphi.len();
    let i_start = 1;

    let (crack_idx, _crack_m) = mphi[i_start..]
        .iter()
        .enumerate()
        .find(|(_, p)| p.moment > 0.0 && p.curvature > 1e-12)
        .map(|(i, p)| (i + i_start, p.moment))
        .unwrap_or((0, 0.0));

    let yield_idx = (crack_idx + 1..n)
        .find(|&i| {
            let dmdk = (mphi[i].moment - mphi[i - 1].moment)
                / (mphi[i].curvature - mphi[i - 1].curvature).max(1e-15);
            let init_slope = if crack_idx > 0 && mphi[crack_idx].curvature > 1e-15 {
                mphi[crack_idx].moment / mphi[crack_idx].curvature
            } else {
                mphi[1].moment / mphi[1].curvature
            };
            init_slope > 0.0 && dmdk / init_slope < 0.1
        })
        .unwrap_or(n - 1);

    let ultimate_idx = (yield_idx..n)
        .rev()
        .find(|&i| i > 0 && (mphi[i].moment - mphi[i - 1].moment).abs() < 1.0)
        .unwrap_or(n - 1);

    let mut points = Vec::new();
    points.push((0.0, 0.0));
    points.push((mphi[crack_idx].curvature, mphi[crack_idx].moment));
    points.push((mphi[yield_idx].curvature, mphi[yield_idx].moment));
    if ultimate_idx > yield_idx {
        points.push((mphi[ultimate_idx].curvature, mphi[ultimate_idx].moment));
    }
    points
}

/// M–φ → M–θ 変換（柔性法）。
/// θ = ∫ φ dx を、反曲点比から仮定した曲率分布で計算。
fn mphi_to_mtheta(mphi_points: &[(f64, f64)], span: f64, inflection_ratio: f64) -> Vec<(f64, f64)> {
    mphi_points
        .iter()
        .map(|&(phi, m)| {
            if phi.abs() < 1e-15 {
                (0.0, 0.0)
            } else {
                let l = span * inflection_ratio;
                let theta = phi * l / 3.0;
                (theta, m)
            }
        })
        .collect()
}

/// 部材スケルトンを構築する。
///
/// 1. 断面ファイバモデルから M–φ 関係を数値積分で算定。
/// 2. ひび割れ・降伏・終局点を抽出し トリリニア化。
/// 3. 部材の境界条件・スパン・反曲点位置から M–φ → M–θ へ積分変換。
pub fn build_member_skeleton(
    member: &MemberData,
    n_axial: f64,
    mats: &mut [Box<dyn UniaxialMaterial>],
) -> MemberSkeleton {
    let max_curvature = 0.01;
    let num_steps = 200;

    let mphi = compute_m_phi_curve(member.fibers, mats, n_axial, max_curvature, num_steps);
    let trilinear = extract_trilinear(&mphi);
    let mtheta = mphi_to_mtheta(&trilinear, member.span, member.inflection_ratio);

    let points: Vec<(f64, f64)> = mtheta;

    let hysteresis = HysteresisRule::Takeda {
        crack: points.get(1).copied().unwrap_or((0.0, 0.0)),
        yield_point: points.get(2).copied().unwrap_or((0.0, 0.0)),
        ultimate: points.last().copied().unwrap_or((0.0, 0.0)),
        alpha: 0.4,
    };

    MemberSkeleton {
        points,
        hysteresis: hysteresis.clone(),
        axial_dependency: AxialInteraction {
            skeletons: vec![(
                n_axial,
                MemberSkeleton {
                    points: vec![],
                    hysteresis,
                    axial_dependency: AxialInteraction { skeletons: vec![] },
                },
            )],
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sc_material::Bilinear;
    use sc_section::fiber::rect_fiber_section;

    #[test]
    fn test_member_skeleton_basic() {
        let sec = Section {
            id: sc_core::ids::SectionId(0),
            name: "test".into(),
            area: 10000.0,
            iy: 1e8,
            iz: 1e8,
            j: 1e8,
            depth: 200.0,
            width: 100.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
        };
        let mat_data = Material {
            id: sc_core::ids::MaterialId(0),
            name: "steel".into(),
            young: 205000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
        };
        let fibers = rect_fiber_section(100.0, 200.0, 10, 20, 0);
        let reinforcement = Reinforcement {
            main_bars: vec![],
            hoop_pitch: 100.0,
            hoop_area: 0.0,
        };
        let member = MemberData {
            section: &sec,
            reinforcement: &reinforcement,
            material: &mat_data,
            fibers: &fibers,
            span: 4000.0,
            inflection_ratio: 0.5,
        };
        let mut mats: Vec<Box<dyn UniaxialMaterial>> =
            vec![Box::new(Bilinear::new(205000.0, 235.0, 0.01))];
        let skeleton = build_member_skeleton(&member, 0.0, &mut mats);
        assert!(!skeleton.points.is_empty());
        assert!(skeleton.points.last().unwrap().1 >= skeleton.points.first().unwrap().1);
    }
}
