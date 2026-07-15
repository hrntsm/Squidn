//! SRC/CFT の複合換算断面性能。
//!
//! - [`CompositeProps`] — 複合換算後の断面性能
//! - [`SectionShape::src_equivalent_props`] — SRC の等価断面性能
//! - [`SectionShape::cft_equivalent_props`] — CFT の等価断面性能

use super::constants::{E_STEEL, KAPPA_RC, NU_CONCRETE, NU_STEEL};
use super::geometry::{h_web_shear_area, rect_torsion_j};
use super::material::concrete_young_modulus;
use super::types::SectionShape;

/// SRC/CFT の複合換算断面性能（要素剛性用。各種合成構造設計指針）。
///
/// いずれも要素に割り当てた材料（SRC はコンクリート、CFT は鋼管）を基準とした
/// 等価値。質量算定用の断面積（`calc_area`）とは区別して用いること。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CompositeProps {
    /// 軸剛性用断面積 [mm²]
    pub area_ax: f64,
    pub iy: f64,
    pub iz: f64,
    pub j: f64,
    pub as_y: f64,
    pub as_z: f64,
}

impl SectionShape {
    /// SRC 断面の等価断面性能を、実際のヤング係数比 ns=Es/Ec から算定する
    /// （各種合成構造設計指針: An=rcAn+sAn·(ns−1)、Ie=rcIe+sIe·(ns−1)、
    /// As=rcAs+sAs·(ngs−1)、J=cJ+(sG/cG)·sJ）。
    ///
    /// `ec`/`nu_c`: 要素材料（コンクリート）のヤング係数・ポアソン比。
    /// 鉄骨は Es=`E_STEEL`・νs=0.3 とする。ngs=ns·(1+νc)/(1+νs)。
    /// SrcRect 以外、または ec≤0 では None（呼び出し側は `to_section` の
    /// 既定値=N_S_EQ 固定へフォールバックする）。
    pub fn src_equivalent_props(&self, ec: f64, nu_c: f64) -> Option<CompositeProps> {
        let SectionShape::SrcRect {
            b,
            d,
            steel_height: sh,
            steel_width: sw,
            steel_web_thick: tw,
            steel_flange_thick: tf,
            ..
        } = *self
        else {
            return None;
        };
        if ec <= 0.0 {
            return None;
        }
        let ns = E_STEEL / ec;
        let ngs = ns * (1.0 + nu_c) / (1.0 + NU_STEEL);

        let s_a = 2.0 * sw * tf + (sh - 2.0 * tf) * tw;
        let hw = sh - 2.0 * tf;
        let s_iy = (sw * sh.powi(3) - (sw - tw) * hw.powi(3)) / 12.0;
        let s_iz = (2.0 * tf * sw.powi(3) + hw * tw.powi(3)) / 12.0;
        let s_j = (2.0 * sw * tf.powi(3) + hw * tw.powi(3)) / 3.0;

        let rc_as = b * d / KAPPA_RC;
        Some(CompositeProps {
            area_ax: b * d + (ns - 1.0) * s_a,
            iy: b * d.powi(3) / 12.0 + (ns - 1.0) * s_iy,
            iz: d * b.powi(3) / 12.0 + (ns - 1.0) * s_iz,
            j: rect_torsion_j(b, d) + ngs * s_j,
            as_y: rc_as + (ngs - 1.0) * 2.0 * sw * tf,
            as_z: rc_as + (ngs - 1.0) * h_web_shear_area(sh, tw),
        })
    }

    /// CFT 断面（CftBox/CftPipe）の等価断面性能を鋼管基準で算定する
    /// （各種合成構造設計指針: SRC 柱に準じる累加（CFT もこれに準じる）を鋼基準の
    /// 1/n 換算で適用。J は S 柱の J=(sG/cG)·sJ+cJ を鋼基準 J=sJ+cJ/ngs に換算）。
    ///
    /// `es`/`nu_s`: 要素材料（鋼管）のヤング係数・ポアソン比、
    /// `fc`: 充填コンクリート強度（`Material.fc`）。
    /// Ec は `concrete_young_modulus`（γ=23）・νc=0.2 とする。
    /// CftBox/CftPipe 以外、または Ec≤0 では None（鋼管のみの既定値へ
    /// フォールバック）。
    pub fn cft_equivalent_props(&self, es: f64, nu_s: f64, fc: f64) -> Option<CompositeProps> {
        let ec = concrete_young_modulus(fc);
        if ec <= 0.0 || es <= 0.0 {
            return None;
        }
        let n = es / ec;
        let ngs = n * (1.0 + NU_CONCRETE) / (1.0 + nu_s);
        match *self {
            SectionShape::CftBox {
                height: h,
                width: w,
                thick: t,
            } => {
                let (bi, hi) = (w - 2.0 * t, h - 2.0 * t);
                if bi <= 0.0 || hi <= 0.0 {
                    return None;
                }
                let c_a = bi * hi;
                let s_as_z = 2.0 * t * hi;
                let s_as_y = 2.0 * t * bi;
                Some(CompositeProps {
                    area_ax: self.calc_area() + c_a / n,
                    iy: self.calc_iy() + bi * hi.powi(3) / 12.0 / n,
                    iz: self.calc_iz() + hi * bi.powi(3) / 12.0 / n,
                    j: self.calc_j() + rect_torsion_j(bi, hi) / ngs,
                    as_y: s_as_y + c_a / KAPPA_RC / ngs,
                    as_z: s_as_z + c_a / KAPPA_RC / ngs,
                })
            }
            SectionShape::CftPipe { outer_dia, thick } => {
                let di = outer_dia - 2.0 * thick;
                if di <= 0.0 {
                    return None;
                }
                let c_a = std::f64::consts::PI * di * di / 4.0;
                let c_i = std::f64::consts::PI * di.powi(4) / 64.0;
                let c_j = std::f64::consts::PI * di.powi(4) / 32.0;
                let s_as = self.calc_area() / 2.0;
                Some(CompositeProps {
                    area_ax: self.calc_area() + c_a / n,
                    iy: self.calc_iy() + c_i / n,
                    iz: self.calc_iz() + c_i / n,
                    j: self.calc_j() + c_j / ngs,
                    as_y: s_as + c_a / KAPPA_RC / ngs,
                    as_z: s_as + c_a / KAPPA_RC / ngs,
                })
            }
            _ => None,
        }
    }
}
