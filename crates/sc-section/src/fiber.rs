use sc_material::UniaxialMaterial;

/// ファイバ（断面小片）。
pub struct Fiber {
    pub y: f64,
    pub z: f64,
    pub area: f64,
    /// 材料に対応するインデックス（section_response に渡す mats 配列内の位置）。
    pub material: usize,
}

/// ファイバ断面：多数のファイバで構成される。
pub struct FiberSection {
    pub fibers: Vec<Fiber>,
}

/// 断面ひずみ（軸ひずみ ε0、曲率 κy, κz）。平面保持。
pub struct SectionStrain {
    pub eps0: f64,
    pub ky: f64,
    pub kz: f64,
}

/// 断面力（N, My, Mz）。
pub struct SectionForce {
    pub n: f64,
    pub my: f64,
    pub mz: f64,
}

/// 接線断面剛性（3×3：N–My–Mz と ε0–κy–κz の関係）。
pub struct SectionStiffness {
    pub d: [[f64; 3]; 3],
}

/// 各ファイバのひずみ = ε0 − κz·y + κy·z（平面保持）。
/// 各ファイバ材料の trial で (σ, Et) を得て、断面力・接線 D を積分する。
pub fn section_response(
    sec: &FiberSection,
    strain: SectionStrain,
    mats: &mut [Box<dyn UniaxialMaterial>],
) -> (SectionForce, SectionStiffness) {
    let mut n = 0.0;
    let mut my = 0.0;
    let mut mz = 0.0;
    let mut d = [[0.0; 3]; 3];

    for fiber in &sec.fibers {
        let eps = strain.eps0 - strain.kz * fiber.y + strain.ky * fiber.z;
        let mat = &mut mats[fiber.material];
        let (sigma, et) = mat.trial(eps);
        let a = fiber.area;

        n += sigma * a;
        my += sigma * a * fiber.z;
        mz += -sigma * a * fiber.y;

        let ea = et * a;
        let ey = ea * fiber.y;
        let ez = ea * fiber.z;

        d[0][0] += ea;
        d[0][1] += ez;
        d[0][2] += -ey;

        d[1][0] += ez;
        d[1][1] += ez * fiber.z;
        d[1][2] += -ey * fiber.z;

        d[2][0] += -ey;
        d[2][1] += -ey * fiber.z;
        d[2][2] += ey * fiber.y;
    }

    (SectionForce { n, my, mz }, SectionStiffness { d })
}

/// 矩形断面をファイバに分割するヘルパー。
pub fn rect_fiber_section(
    width: f64,
    depth: f64,
    nw: usize,
    nd: usize,
    material: usize,
) -> FiberSection {
    let mut fibers = Vec::with_capacity(nw * nd);
    let dw = width / nw as f64;
    let dd = depth / nd as f64;
    for i in 0..nw {
        for j in 0..nd {
            let y = (i as f64 + 0.5) * dw - width / 2.0;
            let z = (j as f64 + 0.5) * dd - depth / 2.0;
            fibers.push(Fiber {
                y,
                z,
                area: dw * dd,
                material,
            });
        }
    }
    FiberSection { fibers }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use sc_material::Bilinear;

    #[test]
    fn test_section_axial() {
        let sec = rect_fiber_section(100.0, 200.0, 10, 20, 0);
        let mut mats: Vec<Box<dyn UniaxialMaterial>> =
            vec![Box::new(Bilinear::new(205000.0, 235.0, 0.01))];
        let (force, _) = section_response(
            &sec,
            SectionStrain {
                eps0: 0.001,
                ky: 0.0,
                kz: 0.0,
            },
            &mut mats,
        );
        let area = 100.0 * 200.0;
        let expected_n = 0.001 * 205000.0 * area;
        assert_relative_eq!(force.n, expected_n, epsilon = 1.0);
        assert_relative_eq!(force.my, 0.0, epsilon = 1.0);
        assert_relative_eq!(force.mz, 0.0, epsilon = 1.0);
    }

    #[test]
    fn test_section_pure_bending() {
        let sec = rect_fiber_section(100.0, 200.0, 20, 40, 0);
        let mut mats: Vec<Box<dyn UniaxialMaterial>> =
            vec![Box::new(Bilinear::new(205000.0, 235.0, 0.01))];
        let ky = 1e-6;
        let (force, stiff) = section_response(
            &sec,
            SectionStrain {
                eps0: 0.0,
                ky,
                kz: 0.0,
            },
            &mut mats,
        );
        let iy = 100.0 * 200.0_f64.powi(3) / 12.0;
        let expected_my = ky * 205000.0 * iy;
        assert_relative_eq!(force.my, expected_my, epsilon = expected_my * 0.01);
        assert!(force.n.abs() < 1.0);
        assert_relative_eq!(stiff.d[1][1], 205000.0 * iy, epsilon = 205000.0 * iy * 0.01);
    }
}
