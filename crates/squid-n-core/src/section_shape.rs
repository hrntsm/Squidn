use crate::ids::SectionId;
use crate::model::Section;

/// RC 配筋の主筋セット（方向別）。
///
/// `count`: 本数, `dia`: 径 [mm], `layers`: 段数。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct BarSet {
    pub count: u32,
    pub dia: f64,
    pub layers: u32,
}

/// RC せん断補強筋。
///
/// `dia`: 径 [mm], `pitch`: ピッチ [mm], `legs`: 組数。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ShearBar {
    pub dia: f64,
    pub pitch: f64,
    pub legs: u32,
}

/// RC 配筋情報。
///
/// `main_x`: せい方向（X）主筋, `main_y`: 幅方向（Y）主筋,
/// `cover`: かぶり [mm], `shear`: せん断補強筋。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RcRebar {
    pub main_x: BarSet,
    pub main_y: BarSet,
    pub cover: f64,
    pub shear: ShearBar,
}

/// Parametric section shape definition.
///
/// Each variant carries the minimal parameters needed to define the geometry.
/// Call `to_section()` to compute the derived section properties (A, Iy, Iz, J, ...)
/// and produce a `squid_n_core::Section`.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum SectionShape {
    /// Steel H‑shape (H形鋼).
    SteelH {
        height: f64,
        width: f64,
        web_thick: f64,
        flange_thick: f64,
    },
    /// Steel rectangular hollow section / box (角形鋼管).
    SteelBox { height: f64, width: f64, thick: f64 },
    /// Steel L‑angle (山形鋼).
    SteelAngle { leg_a: f64, leg_b: f64, thick: f64 },
    /// Steel C‑channel (溝形鋼).
    SteelChannel {
        height: f64,
        width: f64,
        web_thick: f64,
        flange_thick: f64,
    },
    /// Steel T‑shape (T形鋼).
    SteelTee {
        height: f64,
        width: f64,
        web_thick: f64,
        flange_thick: f64,
    },
    /// Steel round pipe (鋼管).
    SteelPipe { outer_dia: f64, thick: f64 },
    /// Reinforced concrete rectangle (RC 矩形).
    RcRect { b: f64, d: f64, rebar: RcRebar },
    /// Reinforced concrete circle column (RC 円形).
    RcCircle { d: f64, rebar: RcRebar },
}

impl SectionShape {
    /// Compute the cross‑sectional area [mm²].
    pub fn calc_area(&self) -> f64 {
        match *self {
            SectionShape::SteelH {
                height,
                width,
                web_thick,
                flange_thick,
            } => 2.0 * width * flange_thick + (height - 2.0 * flange_thick) * web_thick,
            SectionShape::SteelBox {
                height,
                width,
                thick,
            } => width * height - (width - 2.0 * thick) * (height - 2.0 * thick),
            SectionShape::SteelAngle {
                leg_a,
                leg_b,
                thick,
            } => thick * (leg_a + leg_b - thick),
            SectionShape::SteelChannel {
                height,
                width,
                web_thick,
                flange_thick,
            } => 2.0 * width * flange_thick + (height - 2.0 * flange_thick) * web_thick,
            SectionShape::SteelTee {
                height,
                width,
                web_thick,
                flange_thick,
            } => width * flange_thick + (height - flange_thick) * web_thick,
            SectionShape::SteelPipe { outer_dia, thick } => {
                let r = outer_dia / 2.0;
                let ri = r - thick;
                std::f64::consts::PI * (r * r - ri * ri)
            }
            SectionShape::RcRect { b, d, .. } => b * d,
            SectionShape::RcCircle { d, .. } => std::f64::consts::PI * d * d / 4.0,
        }
    }

    /// Moment of inertia about the local y‑axis [mm⁴] (strong axis for beams).
    pub fn calc_iy(&self) -> f64 {
        match *self {
            SectionShape::SteelH {
                height,
                width,
                web_thick,
                flange_thick,
            } => {
                let hw = height - 2.0 * flange_thick;
                (width * height.powi(3) - (width - web_thick) * hw.powi(3)) / 12.0
            }
            SectionShape::SteelBox {
                height,
                width,
                thick,
            } => {
                let hi = height - 2.0 * thick;
                (width * height.powi(3) - (width - 2.0 * thick) * hi.powi(3)) / 12.0
            }
            SectionShape::SteelAngle {
                leg_a,
                leg_b,
                thick,
            } => {
                let (_, cy, _) = angle_centroid(leg_a, leg_b, thick);
                let a1 = leg_a * thick;
                let y1 = leg_a / 2.0;
                let a2 = (leg_b - thick) * thick;
                let y2 = thick / 2.0;
                let i1 = thick * leg_a.powi(3) / 12.0;
                let i2 = (leg_b - thick) * thick.powi(3) / 12.0;
                (i1 + a1 * (y1 - cy).powi(2)) + (i2 + a2 * (y2 - cy).powi(2))
            }
            SectionShape::SteelChannel {
                height,
                width,
                web_thick,
                flange_thick,
            } => {
                let hw = height - 2.0 * flange_thick;
                (width * height.powi(3) - (width - web_thick) * hw.powi(3)) / 12.0
            }
            SectionShape::SteelTee {
                height,
                width,
                web_thick,
                flange_thick,
            } => {
                let y_bar = tee_centroid(height, width, web_thick, flange_thick);
                let a_f = width * flange_thick;
                let a_w = (height - flange_thick) * web_thick;
                let i_f = width * flange_thick.powi(3) / 12.0
                    + a_f * (height - flange_thick / 2.0 - y_bar).powi(2);
                let i_w = web_thick * (height - flange_thick).powi(3) / 12.0
                    + a_w * ((height - flange_thick) / 2.0 - y_bar).powi(2);
                i_f + i_w
            }
            SectionShape::SteelPipe { outer_dia, thick } => {
                let r = outer_dia / 2.0;
                let ri = r - thick;
                std::f64::consts::PI / 4.0 * (r.powi(4) - ri.powi(4))
            }
            SectionShape::RcRect { b, d, .. } => b * d.powi(3) / 12.0,
            SectionShape::RcCircle { d, .. } => std::f64::consts::PI * d.powi(4) / 64.0,
        }
    }

    /// Moment of inertia about the local z‑axis [mm⁴] (weak axis for beams).
    pub fn calc_iz(&self) -> f64 {
        match *self {
            SectionShape::SteelH {
                height,
                width,
                web_thick,
                flange_thick,
            } => {
                let hw = height - 2.0 * flange_thick;
                (2.0 * flange_thick * width.powi(3) + hw * web_thick.powi(3)) / 12.0
            }
            SectionShape::SteelBox {
                height,
                width,
                thick,
            } => {
                let wi = width - 2.0 * thick;
                (height * width.powi(3) - (height - 2.0 * thick) * wi.powi(3)) / 12.0
            }
            SectionShape::SteelAngle {
                leg_a,
                leg_b,
                thick,
            } => {
                let (cx, _, _) = angle_centroid(leg_a, leg_b, thick);
                let a1 = leg_a * thick;
                let z1 = thick / 2.0;
                let a2 = (leg_b - thick) * thick;
                let z2 = thick + (leg_b - thick) / 2.0;
                let i1 = leg_a * thick.powi(3) / 12.0;
                let i2 = thick * (leg_b - thick).powi(3) / 12.0;
                (i1 + a1 * (z1 - cx).powi(2)) + (i2 + a2 * (z2 - cx).powi(2))
            }
            SectionShape::SteelChannel {
                height,
                width,
                web_thick,
                flange_thick,
            } => {
                let hw = height - 2.0 * flange_thick;
                let a_f = width * flange_thick;
                let a_w = hw * web_thick;
                let a_total = 2.0 * a_f + a_w;
                let z_bar = if a_total > 0.0 {
                    (2.0 * a_f * width / 2.0 + a_w * web_thick / 2.0) / a_total
                } else {
                    0.0
                };
                let i_f = flange_thick * width.powi(3) / 12.0 + a_f * (width / 2.0 - z_bar).powi(2);
                let i_f2 =
                    flange_thick * width.powi(3) / 12.0 + a_f * (width / 2.0 - z_bar).powi(2);
                let i_w = hw * web_thick.powi(3) / 12.0 + a_w * (web_thick / 2.0 - z_bar).powi(2);
                i_f + i_f2 + i_w
            }
            SectionShape::SteelTee {
                height,
                width,
                web_thick,
                flange_thick,
            } => {
                let iz = flange_thick * width.powi(3) / 12.0;
                let iz_w = (height - flange_thick) * web_thick.powi(3) / 12.0;
                iz + iz_w
            }
            SectionShape::SteelPipe { .. } => self.calc_iy(),
            SectionShape::RcRect { b, d, .. } => d * b.powi(3) / 12.0,
            SectionShape::RcCircle { .. } => self.calc_iy(),
        }
    }

    /// Torsional constant J [mm⁴].
    pub fn calc_j(&self) -> f64 {
        match *self {
            SectionShape::SteelH {
                height,
                width,
                web_thick,
                flange_thick,
            } => {
                (2.0 * width * flange_thick.powi(3)
                    + (height - 2.0 * flange_thick) * web_thick.powi(3))
                    / 3.0
            }
            SectionShape::SteelBox {
                height,
                width,
                thick,
            } => {
                let a0 = (height - thick) * (width - thick);
                let perim = 2.0 * (height + width - 2.0 * thick);
                4.0 * a0 * a0 * thick / perim
            }
            SectionShape::SteelAngle {
                leg_a,
                leg_b,
                thick,
            } => ((leg_a + leg_b - thick) * thick.powi(3)) / 3.0,
            SectionShape::SteelChannel {
                height,
                width,
                web_thick,
                flange_thick,
            } => {
                (2.0 * width * flange_thick.powi(3)
                    + (height - 2.0 * flange_thick) * web_thick.powi(3))
                    / 3.0
            }
            SectionShape::SteelTee {
                height,
                width,
                web_thick,
                flange_thick,
            } => (width * flange_thick.powi(3) + (height - flange_thick) * web_thick.powi(3)) / 3.0,
            SectionShape::SteelPipe { outer_dia, thick } => {
                let r = outer_dia / 2.0;
                let ri = r - thick;
                std::f64::consts::PI / 2.0 * (r.powi(4) - ri.powi(4))
            }
            SectionShape::RcRect { b, d, .. } => {
                let b_min = b.min(d);
                let h = b.max(d);
                let r = h / b_min;
                let beta = if r < 10.0 {
                    (1.0 / 3.0) * (1.0 - 0.630 / r + 0.052 / r.powi(5))
                } else {
                    1.0 / 3.0
                };
                beta * b_min.powi(3) * h
            }
            SectionShape::RcCircle { d, .. } => std::f64::consts::PI * d.powi(4) / 32.0,
        }
    }

    /// Build a fully-populated `squid_n_core::Section` from the shape parameters.
    ///
    /// `id` and `name` must be supplied by the caller; all section properties
    /// are computed automatically.
    pub fn to_section(&self, id: SectionId, name: String) -> Section {
        let area = self.calc_area();
        let iy = self.calc_iy();
        let iz = self.calc_iz();
        let j = self.calc_j();
        let (depth, width, as_y, as_z) = match *self {
            SectionShape::SteelH { height, width, .. } => (height, width, 0.0, 0.0),
            SectionShape::SteelBox { height, width, .. } => (height, width, 0.0, 0.0),
            SectionShape::SteelAngle { leg_a, leg_b, .. } => {
                (leg_a.max(leg_b), leg_a.min(leg_b), 0.0, 0.0)
            }
            SectionShape::SteelChannel { height, width, .. } => (height, width, 0.0, 0.0),
            SectionShape::SteelTee { height, width, .. } => (height, width, 0.0, 0.0),
            SectionShape::SteelPipe { outer_dia, .. } => (outer_dia, outer_dia, 0.0, 0.0),
            SectionShape::RcRect {
                b, d, ref rebar, ..
            } => {
                let as_y = bar_set_area(&rebar.main_x);
                let as_z = bar_set_area(&rebar.main_y);
                (d, b, as_y, as_z)
            }
            SectionShape::RcCircle { d, ref rebar, .. } => {
                let as_y = bar_set_area(&rebar.main_x);
                let as_z = bar_set_area(&rebar.main_y);
                (d, d, as_y, as_z)
            }
        };
        Section {
            id,
            name,
            area,
            iy,
            iz,
            j,
            depth,
            width,
            as_y,
            as_z,
            panel_thickness: None,
            thickness: None,
            // UI設計 §4.2: Section は SectionShape の派生。生成元の形状を保持する。
            shape: Some(self.clone()),
        }
    }
}

fn bar_set_area(bs: &BarSet) -> f64 {
    let r = bs.dia / 2.0;
    bs.count as f64 * std::f64::consts::PI * r * r
}

fn angle_centroid(leg_a: f64, leg_b: f64, thick: f64) -> (f64, f64, f64) {
    let a1 = leg_a * thick;
    let a2 = (leg_b - thick) * thick;
    let a_total = a1 + a2;
    if a_total < 1e-30 {
        return (0.0, 0.0, 0.0);
    }
    let cy = (a1 * leg_a / 2.0 + a2 * thick / 2.0) / a_total;
    let cx = (a1 * thick / 2.0 + a2 * (thick + (leg_b - thick) / 2.0)) / a_total;
    (cx, cy, a_total)
}

fn tee_centroid(height: f64, width: f64, web_thick: f64, flange_thick: f64) -> f64 {
    let a_f = width * flange_thick;
    let a_w = (height - flange_thick) * web_thick;
    let a_total = a_f + a_w;
    if a_total < 1e-30 {
        return 0.0;
    }
    (a_f * (height - flange_thick / 2.0) + a_w * (height - flange_thick) / 2.0) / a_total
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_steel_h_shape() {
        let shape = SectionShape::SteelH {
            height: 300.0,
            width: 300.0,
            web_thick: 10.0,
            flange_thick: 15.0,
        };
        let sec = shape.to_section(SectionId(0), "H-300x300x10x15".into());
        assert!(sec.area > 0.0);
        assert!(sec.iy > sec.iz);
        assert!(sec.j > 0.0);
    }

    #[test]
    fn test_steel_box() {
        let shape = SectionShape::SteelBox {
            height: 200.0,
            width: 200.0,
            thick: 12.0,
        };
        let sec = shape.to_section(SectionId(0), "BOX-200x200x12".into());
        assert!(sec.area > 0.0);
        assert!((sec.iy - sec.iz).abs() < 1.0);
    }

    #[test]
    fn test_steel_pipe() {
        let shape = SectionShape::SteelPipe {
            outer_dia: 216.3,
            thick: 8.2,
        };
        let sec = shape.to_section(SectionId(0), "PIPE-216.3x8.2".into());
        assert!(sec.area > 0.0);
        assert!((sec.iy - sec.iz).abs() < 1e-6);
        assert!(sec.j > sec.iy);
    }

    #[test]
    fn test_rc_rect() {
        let shape = SectionShape::RcRect {
            b: 500.0,
            d: 500.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 8,
                    dia: 16.0,
                    layers: 2,
                },
                main_y: BarSet {
                    count: 4,
                    dia: 16.0,
                    layers: 2,
                },
                cover: 40.0,
                shear: ShearBar {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                },
            },
        };
        let sec = shape.to_section(SectionId(0), "RC-500x500".into());
        assert!(sec.area > 0.0);
        assert!(sec.as_y > 0.0);
        assert!(sec.iz > 0.0);
    }

    #[test]
    fn test_rc_circle() {
        let shape = SectionShape::RcCircle {
            d: 600.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 12,
                    dia: 22.0,
                    layers: 1,
                },
                main_y: BarSet {
                    count: 12,
                    dia: 22.0,
                    layers: 1,
                },
                cover: 40.0,
                shear: ShearBar {
                    dia: 6.0,
                    pitch: 80.0,
                    legs: 1,
                },
            },
        };
        let sec = shape.to_section(SectionId(0), "RC-600".into());
        assert!(sec.area > 0.0);
        assert!(sec.as_y > 0.0);
        assert!(sec.as_z > 0.0);
    }

    #[test]
    fn test_steel_l_angle() {
        let shape = SectionShape::SteelAngle {
            leg_a: 150.0,
            leg_b: 100.0,
            thick: 12.0,
        };
        let sec = shape.to_section(SectionId(0), "L-150x100x12".into());
        assert!(sec.area > 0.0);
        assert!(sec.iy > 0.0);
        assert!(sec.iz > 0.0);
    }

    #[test]
    fn test_steel_tee() {
        let shape = SectionShape::SteelTee {
            height: 200.0,
            width: 200.0,
            web_thick: 10.0,
            flange_thick: 15.0,
        };
        let sec = shape.to_section(SectionId(0), "T-200x200x10x15".into());
        assert!(sec.area > 0.0);
        assert!(sec.iy > 0.0);
        assert!(sec.iz > 0.0);
    }

    #[test]
    fn test_steel_channel() {
        let shape = SectionShape::SteelChannel {
            height: 250.0,
            width: 90.0,
            web_thick: 7.5,
            flange_thick: 12.0,
        };
        let sec = shape.to_section(SectionId(0), "C-250x90x7.5x12".into());
        assert!(sec.area > 0.0);
        assert!(sec.iy > sec.iz);
    }

    #[test]
    fn test_section_roundtrip_serde() {
        let shape = SectionShape::SteelH {
            height: 300.0,
            width: 300.0,
            web_thick: 10.0,
            flange_thick: 15.0,
        };
        let json = serde_json::to_string(&shape).unwrap();
        let restored: SectionShape = serde_json::from_str(&json).unwrap();
        assert_eq!(shape, restored);
    }

    #[test]
    fn test_rc_rebar_serde_roundtrip() {
        let shape = SectionShape::RcRect {
            b: 500.0,
            d: 600.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 6,
                    dia: 22.0,
                    layers: 2,
                },
                main_y: BarSet {
                    count: 2,
                    dia: 16.0,
                    layers: 1,
                },
                cover: 50.0,
                shear: ShearBar {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                },
            },
        };
        let json = serde_json::to_string(&shape).unwrap();
        let restored: SectionShape = serde_json::from_str(&json).unwrap();
        assert_eq!(shape, restored);
    }

    #[test]
    fn test_rc_rect_area() {
        let shape = SectionShape::RcRect {
            b: 400.0,
            d: 600.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 6,
                    dia: 19.0,
                    layers: 2,
                },
                main_y: BarSet {
                    count: 2,
                    dia: 13.0,
                    layers: 1,
                },
                cover: 40.0,
                shear: ShearBar {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 2,
                },
            },
        };
        assert!((shape.calc_area() - 240_000.0).abs() < 1e-9);
        let iy = shape.calc_iy();
        let iz = shape.calc_iz();
        assert!((iy - 400.0 * 600.0_f64.powi(3) / 12.0).abs() < 1e-6);
        assert!((iz - 600.0 * 400.0_f64.powi(3) / 12.0).abs() < 1e-6);
    }

    #[test]
    fn test_rc_circle_area() {
        let shape = SectionShape::RcCircle {
            d: 800.0,
            rebar: RcRebar {
                main_x: BarSet {
                    count: 16,
                    dia: 25.0,
                    layers: 1,
                },
                main_y: BarSet {
                    count: 16,
                    dia: 25.0,
                    layers: 1,
                },
                cover: 50.0,
                shear: ShearBar {
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 1,
                },
            },
        };
        let expected_area = std::f64::consts::PI * 800.0_f64.powi(2) / 4.0;
        assert!((shape.calc_area() - expected_area).abs() < 1e-6);
        let iy = shape.calc_iy();
        assert!((iy - std::f64::consts::PI * 800.0_f64.powi(4) / 64.0).abs() < 1e-6);
        assert!((shape.calc_iy() - shape.calc_iz()).abs() < 1e-6);
    }
}
