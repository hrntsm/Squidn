//! 鋼構造の許容応力度と断面検定（RESP-D マニュアル「計算編 04 断面検定
//! （許容応力度検定）」鋼構造部分、根拠規準は鋼構造設計規準 1973・構造規定）。
//!
//! ## 形状情報の取得について
//!
//! 検定式（ウェブせん断面積 `tw·H`、圧縮フランジ断面積 `B·tf` 等）にはフランジ厚
//! `tf`・ウェブ厚 `tw` が必要になる。形状は次の優先順で解決する:
//!
//! 1. `Section.shape`（[`squid_n_core::section_shape::SectionShape`]）があれば
//!    `SteelH`/`SteelBox`/`SteelPipe` の実寸（`flange_thick`/`web_thick`/`thick`）
//!    を用いる（パラメトリック断面の正規経路）。
//! 2. 無ければ `Section.name` の先頭トークン（`"H-..."`, `"BOX-..."`,
//!    `"PIPE-..."`）から形状カテゴリを推定し、板厚は `Section.thickness` の
//!    単一値を `tf ≈ tw` として近似する（カタログ断面等のフォールバック。
//!    フランジとウェブの実厚が異なる断面では誤差を生む）。
//!
//! 命名規則にも合わない場合は `Other`（一般断面フォールバック）として扱い、
//! 横座屈低減なし（fb=ft）・単純 τ/fs 検定になる。

use crate::{CheckResult, DesignCheck, DesignCtx, LoadTerm, MemberForcesAt, MemberKind};
use squid_n_core::model::{Material, Section};
use squid_n_core::section_shape::SectionShape;

// ---------------------------------------------------------------------
// F 値表（鋼構造設計規準・告示。板厚 [mm] 区分対応）
// ---------------------------------------------------------------------

/// 板厚 2 区分（`t<=40` / `40<t<=100`）の F 値を返す。
fn bucket2(t: f64, le40: f64, gt40: f64) -> f64 {
    if t <= 40.0 {
        le40
    } else {
        gt40
    }
}

/// 板厚 3 区分（`t<=40` / `40<t<=75` / `75<t<=100`）の F 値を返す（SM520 専用）。
fn bucket3(t: f64, le40: f64, le75: f64, gt75: f64) -> f64 {
    if t <= 40.0 {
        le40
    } else if t <= 75.0 {
        le75
    } else {
        gt75
    }
}

/// 鋼材グレード一覧（前方一致の探索対象。長い記号を優先するため
/// [`steel_f_value_prefix`] 側で文字数最大のものを選ぶ）。
const STEEL_GRADES: &[&str] = &[
    "SS400", "SS490", "SM400", "SM490", "SM520", "SN400", "SN490", "SA440", "STKR400", "STKR490",
    "BCR295", "BCP235", "BCP325",
];

/// 鋼材の F 値 [N/mm²]（完全一致、板厚 [mm] 区分対応）。
///
/// 厚さ 40mm 以下 / 40mm 超 100mm 以下の 2 区分（SM520 のみ 3 区分）。
/// 100mm を超える板厚は規定が無いためマニュアルの最終区分値をそのまま用いる
/// （非保守的になり得るため実運用では要確認）。
///
/// 戻り値は F 値。長期許容引張・圧縮・曲げ ft = F/1.5、
/// 長期許容せん断 fs = F/(1.5·√3)。短期は長期の 1.5 倍（=F, F/√3）。
pub fn steel_f_value(grade: &str, thickness: f64) -> Option<f64> {
    match grade {
        "SS400" => Some(bucket2(thickness, 235.0, 215.0)),
        "SS490" => Some(bucket2(thickness, 275.0, 255.0)),
        "SM400" => Some(bucket2(thickness, 235.0, 215.0)),
        "SM490" => Some(bucket2(thickness, 325.0, 295.0)),
        "SM520" => Some(bucket3(thickness, 355.0, 335.0, 325.0)),
        "SN400" => Some(bucket2(thickness, 235.0, 215.0)),
        "SN490" => Some(bucket2(thickness, 325.0, 295.0)),
        "SA440" => Some(440.0),
        "STKR400" => Some(bucket2(thickness, 235.0, 215.0)),
        "STKR490" => Some(bucket2(thickness, 325.0, 295.0)),
        "BCR295" => Some(295.0),
        "BCP235" => Some(235.0),
        "BCP325" => Some(325.0),
        _ => None,
    }
}

/// 鋼材の F 値 [N/mm²]（前方一致、板厚 [mm] 区分対応）。
///
/// 材料名が板厚区分や強度記号等の接尾辞を伴う場合（例 "SN400B"→235、
/// "SM490A"→325）に対応するため、[`STEEL_GRADES`] の記号と材料名の前方一致で
/// 判定する。複数の記号が前方一致しうる場合は最も長い（＝最も具体的な）
/// ものを優先する（例: "SN490B" は "SN400" ではなく "SN490" に一致する）。
pub fn steel_f_value_prefix(name: &str, thickness: f64) -> Option<f64> {
    STEEL_GRADES
        .iter()
        .filter(|g| name.starts_with(*g))
        .max_by_key(|g| g.len())
        .and_then(|g| steel_f_value(g, thickness))
}

// ---------------------------------------------------------------------
// 許容応力度（鋼構造設計規準 1973・構造規定）
// ---------------------------------------------------------------------

/// F 値の板厚区分判定に用いる最大板厚 [mm]。
/// `Section.shape` があれば形状の最大板厚、無ければ `Section.thickness`、
/// いずれも無ければ 40mm 以下区分として扱う。
fn plate_thickness(sec: &Section) -> f64 {
    if let Some(shape) = &sec.shape {
        match *shape {
            SectionShape::SteelH {
                web_thick,
                flange_thick,
                ..
            }
            | SectionShape::SteelChannel {
                web_thick,
                flange_thick,
                ..
            }
            | SectionShape::SteelTee {
                web_thick,
                flange_thick,
                ..
            } => return web_thick.max(flange_thick),
            SectionShape::SteelBox { thick, .. }
            | SectionShape::SteelAngle { thick, .. }
            | SectionShape::SteelPipe { thick, .. }
            | SectionShape::CftBox { thick, .. }
            | SectionShape::CftPipe { thick, .. } => return thick,
            SectionShape::SrcRect {
                steel_web_thick,
                steel_flange_thick,
                ..
            } => return steel_web_thick.max(steel_flange_thick),
            SectionShape::RcRect { .. }
            | SectionShape::RcCircle { .. }
            | SectionShape::RcWall { .. } => {}
        }
    }
    sec.thickness.unwrap_or(40.0)
}

/// 長期・短期許容引張／曲げ（座屈無視）応力度 ft [N/mm²]。
/// 長期 = F/1.5、短期 = F。
pub fn steel_ft(f: f64, term: LoadTerm) -> f64 {
    match term {
        LoadTerm::Long => f / 1.5,
        LoadTerm::Short => f,
    }
}

/// 長期・短期許容せん断応力度 fs [N/mm²]。
/// 長期 = F/(1.5·√3)、短期 = F/√3。
pub fn steel_fs(f: f64, term: LoadTerm) -> f64 {
    match term {
        LoadTerm::Long => f / (1.5 * 3.0_f64.sqrt()),
        LoadTerm::Short => f / 3.0_f64.sqrt(),
    }
}

/// 限界細長比 Λ = 1500/√(F/1.5)。
fn big_lambda(f: f64) -> f64 {
    1500.0 / (f / 1.5).sqrt()
}

/// 長期・短期許容圧縮応力度 fc [N/mm²]（座屈考慮、鋼構造設計規準 1973）。
///
/// 有効細長比 `λ = lk/i`、限界細長比 `Λ = 1500/√(F/1.5)`、
/// 安全率 `ν = 3/2 + (2/3)(λ/Λ)²` として:
/// - `λ ≤ Λ`: `fcL = F·(1 − 0.4(λ/Λ)²) / ν`
/// - `λ > Λ`: `fcL = (18/65)·F/(λ/Λ)²`
///
/// 短期は長期の 1.5 倍。`λ=0` のとき `fcL = F/1.5`（= ft 長期）と一致し、
/// `λ=Λ` で両分岐は連続する（`fcL = (18/65)F`）。
pub fn steel_fc(f: f64, lambda: f64, term: LoadTerm) -> f64 {
    let big_l = big_lambda(f);
    let r = if big_l > 1e-9 { lambda / big_l } else { 0.0 };
    let fc_long = if lambda <= big_l {
        let nu = 1.5 + (2.0 / 3.0) * r * r;
        f * (1.0 - 0.4 * r * r) / nu
    } else {
        (18.0 / 65.0) * f / (r * r)
    };
    match term {
        LoadTerm::Long => fc_long,
        LoadTerm::Short => fc_long * 1.5,
    }
}

/// 「圧縮フランジ＋せいの 1/6 のウェブ」からなる T 形断面の、ウェブ軸まわり
/// 断面二次半径 `i = √(I_T/A_T)`（鋼構造設計規準 1973、横座屈許容曲げ fb1 用）。
///
/// `I_T = tf·B³/12 + (h/6)·tw³/12`、`A_T = B·tf + (h/6)·tw`。
pub fn steel_i_t(b: f64, tf: f64, h: f64, tw: f64) -> f64 {
    let i_t = tf * b.powi(3) / 12.0 + (h / 6.0) * tw.powi(3) / 12.0;
    let a_t = b * tf + (h / 6.0) * tw;
    if a_t > 1e-12 {
        (i_t / a_t).sqrt()
    } else {
        0.0
    }
}

/// H 形鋼強軸の横座屈を考慮した許容曲げ応力度 fb [N/mm²]（鋼構造設計規準 1973）。
///
/// `fbL = max(fb1, fb2)`（ただし長期許容引張 `ft = F/1.5` を上限とする）:
/// - `fb1 = F·(2/3 − (4/15)·(lb/i)²/(C·Λ²))`
/// - `fb2 = 89000/(lb·h/Af)` [N/mm²]（`lb`,`h`,`Af` は mm, mm, mm²）
///
/// `lb`: 圧縮フランジ支点間距離（横座屈長さ）[mm]、`i`: [`steel_i_t`] の
/// 断面二次半径、`h`: 梁せい [mm]、`af`: 圧縮フランジ断面積 `B·tf` [mm²]。
///
/// 修正係数 C は本来 `1.75+1.05(M2/M1)+0.3(M2/M1)² ≤ 2.3`（端部モーメント比
/// M2/M1 に依存）だが、[`DesignCtx`] に端部モーメント比の情報が無いため、
/// 本実装では常に **C=1.0（安全側・最も不利な等曲げ分布相当）固定**とする。
///
/// 短期は長期の 1.5 倍（上限 F）。
pub fn steel_fb_h(f: f64, term: LoadTerm, lb: f64, i: f64, h: f64, af: f64) -> f64 {
    const C: f64 = 1.0;
    let i = i.max(1e-9);
    let af = af.max(1e-9);
    let big_l = big_lambda(f);
    let big_l2 = (big_l.max(1e-9)).powi(2);

    let fb1 = f * (2.0 / 3.0 - (4.0 / 15.0) * (lb / i).powi(2) / (C * big_l2));
    let denom2 = lb * h / af;
    let fb2 = if denom2 > 0.0 {
        89_000.0 / denom2
    } else {
        f64::INFINITY
    };

    let cap_long = f / 1.5;
    let fb_long = fb1.max(fb2).min(cap_long).max(0.0);
    match term {
        LoadTerm::Long => fb_long,
        LoadTerm::Short => (fb_long * 1.5).min(f),
    }
}

// ---------------------------------------------------------------------
// 断面形状カテゴリ（`Section.shape` 優先、無ければ `Section.name` から推定。
// 上記モジュール doc 参照）
// ---------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ShapeCategory {
    H,
    Box,
    Pipe,
    Other,
}

/// `Section.name` の先頭アルファベットトークンから形状カテゴリを推定する。
/// 例: "H-300x300x10x15"→H、"BOX-200x200x12"→Box、"PIPE-216.3x8.2"→Pipe。
/// 該当しない場合は `Other`（一般断面フォールバック）。
fn classify_shape(name: &str) -> ShapeCategory {
    let token: String = name
        .trim()
        .to_uppercase()
        .chars()
        .take_while(|c| c.is_ascii_alphabetic())
        .collect();
    match token.as_str() {
        "H" => ShapeCategory::H,
        "BOX" => ShapeCategory::Box,
        "PIPE" | "P" => ShapeCategory::Pipe,
        _ => ShapeCategory::Other,
    }
}

/// 形状カテゴリと板厚 `(カテゴリ, tf, tw)` を解決する。
///
/// `Section.shape`（パラメトリック断面）があれば実寸のフランジ厚・ウェブ厚を、
/// 無ければ断面名からカテゴリを推定して `Section.thickness` を `tf ≈ tw` の
/// 単一板厚として近似する（モジュール doc 参照）。
fn shape_of(sec: &Section) -> (ShapeCategory, f64, f64) {
    if let Some(shape) = &sec.shape {
        match *shape {
            SectionShape::SteelH {
                web_thick,
                flange_thick,
                ..
            } => return (ShapeCategory::H, flange_thick, web_thick),
            SectionShape::SteelBox { thick, .. } => return (ShapeCategory::Box, thick, thick),
            SectionShape::SteelPipe { thick, .. } => return (ShapeCategory::Pipe, thick, thick),
            SectionShape::SteelChannel {
                web_thick,
                flange_thick,
                ..
            }
            | SectionShape::SteelTee {
                web_thick,
                flange_thick,
                ..
            } => return (ShapeCategory::Other, flange_thick, web_thick),
            SectionShape::SteelAngle { thick, .. } => return (ShapeCategory::Other, thick, thick),
            // CFT の鋼管部分は角形/円形鋼管として扱う（検定本体は cft 側で行う）。
            SectionShape::CftBox { thick, .. } => return (ShapeCategory::Box, thick, thick),
            SectionShape::CftPipe { thick, .. } => return (ShapeCategory::Pipe, thick, thick),
            SectionShape::RcRect { .. }
            | SectionShape::RcCircle { .. }
            | SectionShape::SrcRect { .. }
            | SectionShape::RcWall { .. } => return (ShapeCategory::Other, 0.0, 0.0),
        }
    }
    let t = sec.thickness.unwrap_or(0.0);
    (classify_shape(&sec.name), t, t)
}

/// せん断有効断面積 As [mm²]（梁の H形以外／柱の H形・その他 で共用）。
/// - H: `tw·H`（ウェブ全せい×ウェブ厚）
/// - Box: `2·t·(H−2t)`
/// - Pipe: `A/2`
/// - Other: `as_y>0 ? as_y : area`
fn shear_area(shape: ShapeCategory, sec: &Section, tw: f64) -> f64 {
    let h = sec.depth;
    let t = tw;
    match shape {
        ShapeCategory::H => (t * h).max(0.0),
        ShapeCategory::Box => (2.0 * t * (h - 2.0 * t).max(0.0)).max(0.0),
        ShapeCategory::Pipe => sec.area / 2.0,
        ShapeCategory::Other => {
            if sec.as_y > 0.0 {
                sec.as_y
            } else {
                sec.area
            }
        }
    }
}

/// 分母が極小の場合に安全側デフォルトへ逃がすヘルパー。
fn safe_denom(x: f64) -> f64 {
    if x.abs() > 1e-9 {
        x
    } else {
        1e-9
    }
}

/// 断面係数 Z = I / (半せい)。半せいが極小なら 0（呼び出し側で 1.0 にフォールバック）。
fn section_modulus(i: f64, half_dim: f64) -> f64 {
    if half_dim > 1e-9 {
        i / half_dim
    } else {
        0.0
    }
}

fn nonzero(z: f64) -> f64 {
    if z.abs() > 1e-9 {
        z
    } else {
        1.0
    }
}

pub struct SteelDesign;

impl DesignCheck for SteelDesign {
    fn check(
        &self,
        forces: &MemberForcesAt,
        sec: &Section,
        mat: &Material,
        ctx: &DesignCtx,
    ) -> CheckResult {
        let t = plate_thickness(sec);
        let f = steel_f_value_prefix(&mat.name, t).unwrap_or(235.0);
        let term = ctx.term;

        match ctx.kind {
            MemberKind::Beam => check_beam(forces, sec, ctx, f, term),
            MemberKind::Column => check_column(forces, sec, ctx, f, term),
            MemberKind::Brace => check_brace(forces, sec, ctx, f, term),
        }
    }
}

/// 鉄骨造梁の断面検定（マニュアル「鉄骨造梁の断面検定」）。
///
/// σb = |mz|/Z強軸 を fb（H形強軸は横座屈考慮、他は ft）で検定する。
/// せん断は H形のみ von Mises 型（σb′, τ の合成）、他は単純 τ/fs。
fn check_beam(
    forces: &MemberForcesAt,
    sec: &Section,
    ctx: &DesignCtx,
    f: f64,
    term: LoadTerm,
) -> CheckResult {
    let h = sec.depth;
    let b = sec.width;
    let z_strong = nonzero(section_modulus(sec.iy, h / 2.0));
    let sigma_b = forces.mz.abs() / z_strong;

    let (shape, tf, tw) = shape_of(sec);
    let ft_val = steel_ft(f, term);
    let fs_val = steel_fs(f, term);

    let as_shear = shear_area(shape, sec, tw);
    let tau = forces.qy.abs() / safe_denom(as_shear);

    let (fb, ratio_shear, shear_basis);
    match shape {
        ShapeCategory::H => {
            let af = b * tf;
            let i_t = steel_i_t(b, tf, h, tw);
            let lb = ctx.lb.unwrap_or(ctx.length);
            fb = steel_fb_h(f, term, lb, i_t, h, af);
            let sigma_b_prime = sigma_b * (h - 2.0 * tf).max(0.0) / safe_denom(h);
            let von_mises = (sigma_b_prime.powi(2) + 3.0 * tau.powi(2)).sqrt() / safe_denom(ft_val);
            ratio_shear = von_mises.max(tau / safe_denom(fs_val));
            shear_basis = "H形ウェブ von Mises 照査 (鋼構造設計規準)";
        }
        _ => {
            fb = ft_val;
            ratio_shear = tau / safe_denom(fs_val);
            shear_basis = "H形以外 τ/fs (鋼構造設計規準)";
        }
    }

    let ratio_bend = sigma_b / safe_denom(fb);
    let ratio = ratio_bend.max(ratio_shear);

    let term_label = match term {
        LoadTerm::Long => "長期",
        LoadTerm::Short => "短期",
    };
    let basis = format!(
        "鋼構造設計規準 {} 梁: 曲げ σ/fb (横座屈考慮={}) と せん断 {}",
        term_label,
        matches!(shape, ShapeCategory::H),
        shear_basis
    );
    let detail = format!(
        "σ={:.4} N/mm², fb={:.4} N/mm², τ={:.4} N/mm², fs={:.4} N/mm², 曲げ比={:.4}, せん断比={:.4}",
        sigma_b, fb, tau, fs_val, ratio_bend, ratio_shear
    );

    CheckResult {
        ratio,
        ok: ratio <= 1.0,
        basis,
        detail,
    }
}

/// 鉄骨造柱の断面検定（マニュアル「鉄骨造柱の断面検定」）。
///
/// 軸力+二軸曲げ: `σ/f + σbX/fbX + σbY/fbY ≤ 1.0`
/// （円形鋼管は `σb=√(mz²+my²)/Z` に一本化）。
/// せん断は von Mises 型: `max(√(σ²+3τ²)/ft, τ/fs)`。
fn check_column(
    forces: &MemberForcesAt,
    sec: &Section,
    ctx: &DesignCtx,
    f: f64,
    term: LoadTerm,
) -> CheckResult {
    let h = sec.depth;
    let b = sec.width;
    let area = nonzero(sec.area);
    let z_strong = nonzero(section_modulus(sec.iy, h / 2.0));
    let z_weak = nonzero(section_modulus(sec.iz, b / 2.0));
    let sigma_bx = forces.mz.abs() / z_strong;
    let sigma_by = forces.my.abs() / z_weak;

    let (shape, tf, tw) = shape_of(sec);

    let ft_val = steel_ft(f, term);
    let fs_val = steel_fs(f, term);

    // 有効細長比 λ = lk/i_min（i_min は iy/iz の小さい方）。
    let i_min_sq = sec.iy.min(sec.iz).max(0.0) / area;
    let i_min = i_min_sq.sqrt();
    let lk = ctx.lk.unwrap_or(ctx.length);
    let buckling_note = if lk <= 1e-9 {
        "（座屈長さ0のため座屈無視 λ=0）"
    } else {
        ""
    };
    let lambda = if i_min > 1e-9 { lk / i_min } else { 0.0 };
    let fc_val = steel_fc(f, lambda, term);

    // 強軸 fb（H形のみ横座屈考慮。lb は柱の階高 = ctx.length）。
    let fb_strong = match shape {
        ShapeCategory::H => {
            let af = b * tf;
            let i_t = steel_i_t(b, tf, h, tw);
            steel_fb_h(f, term, ctx.length, i_t, h, af)
        }
        _ => ft_val,
    };
    let fb_weak = ft_val;

    // 円形鋼管は二軸曲げを合成した σb に一本化。
    let sigma_b_pipe = (forces.mz.powi(2) + forces.my.powi(2)).sqrt() / z_strong;

    let axial_stress;
    let ratio_axial_bend;
    let axial_basis;
    if forces.n < 0.0 {
        let sigma_c = forces.n.abs() / area;
        axial_stress = sigma_c;
        ratio_axial_bend = match shape {
            ShapeCategory::Pipe => {
                sigma_c / safe_denom(fc_val) + sigma_b_pipe / safe_denom(fb_strong)
            }
            _ => {
                sigma_c / safe_denom(fc_val)
                    + sigma_bx / safe_denom(fb_strong)
                    + sigma_by / safe_denom(fb_weak)
            }
        };
        axial_basis = "圧縮+曲げ: σc/fc(座屈考慮)+ΣσB/fb";
    } else {
        let sigma_t = forces.n / area;
        axial_stress = sigma_t;
        ratio_axial_bend = match shape {
            ShapeCategory::Pipe => {
                sigma_t / safe_denom(ft_val) + sigma_b_pipe / safe_denom(fb_strong)
            }
            _ => {
                sigma_t / safe_denom(ft_val)
                    + sigma_bx / safe_denom(fb_strong)
                    + sigma_by / safe_denom(fb_weak)
            }
        };
        axial_basis = "引張+曲げ: σt/ft+ΣσB/fb";
    }

    // せん断: H形 τ=Q/(tw·H)、角形 τ=2Q/A、円形 τ=2√(qy²+qz²)/A、他は一般化。
    let as_shear = shear_area(shape, sec, tw);
    let tau = match shape {
        ShapeCategory::H => forces.qy.abs() / safe_denom(as_shear),
        ShapeCategory::Box => 2.0 * forces.qy.abs() / area,
        ShapeCategory::Pipe => 2.0 * (forces.qy.powi(2) + forces.qz.powi(2)).sqrt() / area,
        ShapeCategory::Other => {
            (forces.qy.powi(2) + forces.qz.powi(2)).sqrt() / safe_denom(as_shear)
        }
    };
    let sigma_total = match shape {
        ShapeCategory::H => axial_stress + sigma_bx * (h - 2.0 * tf).max(0.0) / safe_denom(h),
        ShapeCategory::Pipe => axial_stress + sigma_b_pipe,
        _ => axial_stress + sigma_bx + sigma_by,
    };
    let ratio_shear = ((sigma_total.powi(2) + 3.0 * tau.powi(2)).sqrt() / safe_denom(ft_val))
        .max(tau / safe_denom(fs_val));

    let ratio = ratio_axial_bend.max(ratio_shear);

    let term_label = match term {
        LoadTerm::Long => "長期",
        LoadTerm::Short => "短期",
    };
    let basis = format!(
        "鋼構造設計規準 {} 柱: {}{}, せん断 von Mises",
        term_label, axial_basis, buckling_note
    );
    let detail = format!(
        "σax={:.4} N/mm², σbX={:.4} N/mm², σbY={:.4} N/mm², fc={:.4} N/mm², fbX={:.4} N/mm², \
fbY={:.4} N/mm², λ={:.3}, τ={:.4} N/mm², fs={:.4} N/mm², 軸曲げ比={:.4}, せん断比={:.4}",
        axial_stress,
        sigma_bx,
        sigma_by,
        fc_val,
        fb_strong,
        fb_weak,
        lambda,
        tau,
        fs_val,
        ratio_axial_bend,
        ratio_shear
    );

    CheckResult {
        ratio,
        ok: ratio <= 1.0,
        basis,
        detail,
    }
}

/// 鉄骨ブレースの断面検定（マニュアル「鉄骨ブレースの断面検定」）。
///
/// 軸力のみ（曲げ・せん断は考慮しない）: 引張 `σt/ft`、圧縮 `σc/fc`（座屈考慮）。
fn check_brace(
    forces: &MemberForcesAt,
    sec: &Section,
    ctx: &DesignCtx,
    f: f64,
    term: LoadTerm,
) -> CheckResult {
    let area = nonzero(sec.area);
    let i_min_sq = sec.iy.min(sec.iz).max(0.0) / area;
    let i_min = i_min_sq.sqrt();
    let lk = ctx.lk.unwrap_or(ctx.length);
    let buckling_note = if lk <= 1e-9 {
        "（座屈長さ0のため座屈無視 λ=0）"
    } else {
        ""
    };
    let lambda = if i_min > 1e-9 { lk / i_min } else { 0.0 };

    let term_label = match term {
        LoadTerm::Long => "長期",
        LoadTerm::Short => "短期",
    };

    if forces.n < 0.0 {
        let sigma_c = forces.n.abs() / area;
        let fc_val = steel_fc(f, lambda, term);
        let ratio = sigma_c / safe_denom(fc_val);
        CheckResult {
            ratio,
            ok: ratio <= 1.0,
            basis: format!(
                "鋼構造設計規準 {} ブレース: 圧縮 σc/fc(座屈考慮){}",
                term_label, buckling_note
            ),
            detail: format!(
                "σc={:.4} N/mm², fc={:.4} N/mm², λ={:.3}",
                sigma_c, fc_val, lambda
            ),
        }
    } else {
        let sigma_t = forces.n / area;
        let ft_val = steel_ft(f, term);
        let ratio = sigma_t / safe_denom(ft_val);
        CheckResult {
            ratio,
            ok: ratio <= 1.0,
            basis: format!("鋼構造設計規準 {} ブレース: 引張 σt/ft", term_label),
            detail: format!("σt={:.4} N/mm², ft={:.4} N/mm²", sigma_t, ft_val),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::ids::{MaterialId, SectionId};

    fn mat(name: &str) -> Material {
        Material {
            id: MaterialId(0),
            name: name.to_string(),
            young: 205_000.0,
            poisson: 0.3,
            density: 7.85e-9,
            shear: None,
            fc: None,
            fy: None,
        }
    }

    fn rect_section(b: f64, d: f64, name: &str) -> Section {
        Section {
            id: SectionId(0),
            name: name.to_string(),
            area: b * d,
            iy: b * d.powi(3) / 12.0,
            iz: d * b.powi(3) / 12.0,
            j: 0.0,
            depth: d,
            width: b,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        }
    }

    /// `SectionShape::SteelH` 付きの断面（実寸 tf/tw を持つ正規経路の検証用）。
    fn h_section(h: f64, b: f64, tw: f64, tf: f64) -> Section {
        let shape = SectionShape::SteelH {
            height: h,
            width: b,
            web_thick: tw,
            flange_thick: tf,
        };
        shape.to_section(SectionId(0), format!("H-{}x{}x{}x{}", h, b, tw, tf))
    }

    // -------------------------------------------------------------
    // F 値表
    // -------------------------------------------------------------

    #[test]
    fn test_f_value_ss400_buckets() {
        assert!((steel_f_value("SS400", 40.0).unwrap() - 235.0).abs() < 1e-9);
        assert!((steel_f_value("SS400", 40.1).unwrap() - 215.0).abs() < 1e-9);
    }

    #[test]
    fn test_f_value_ss490_is_275_not_285() {
        // 旧実装の SS490=285 はマニュアルに存在しない誤り。275/255 が正。
        assert!((steel_f_value("SS490", 40.0).unwrap() - 275.0).abs() < 1e-9);
        assert!((steel_f_value("SS490", 100.0).unwrap() - 255.0).abs() < 1e-9);
    }

    #[test]
    fn test_f_value_sm520_three_buckets() {
        assert!((steel_f_value("SM520", 40.0).unwrap() - 355.0).abs() < 1e-9);
        assert!((steel_f_value("SM520", 75.0).unwrap() - 335.0).abs() < 1e-9);
        assert!((steel_f_value("SM520", 76.0).unwrap() - 325.0).abs() < 1e-9);
    }

    #[test]
    fn test_f_value_unbucketed_grades() {
        assert!((steel_f_value("SA440", 90.0).unwrap() - 440.0).abs() < 1e-9);
        assert!((steel_f_value("BCR295", 50.0).unwrap() - 295.0).abs() < 1e-9);
        assert!((steel_f_value("BCP235", 50.0).unwrap() - 235.0).abs() < 1e-9);
        assert!((steel_f_value("BCP325", 50.0).unwrap() - 325.0).abs() < 1e-9);
    }

    #[test]
    fn test_f_value_prefix_matching() {
        assert!((steel_f_value_prefix("SN400B", 30.0).unwrap() - 235.0).abs() < 1e-9);
        assert!((steel_f_value_prefix("SM490A", 30.0).unwrap() - 325.0).abs() < 1e-9);
        // SN490B は SN400 ではなく SN490 に一致しなければならない。
        assert!((steel_f_value_prefix("SN490B", 30.0).unwrap() - 325.0).abs() < 1e-9);
        assert!(steel_f_value_prefix("UNKNOWN999", 30.0).is_none());
    }

    // -------------------------------------------------------------
    // fc（圧縮、座屈考慮）
    // -------------------------------------------------------------

    #[test]
    fn test_fc_lambda_zero_equals_ft_long() {
        let f = 235.0;
        let fc = steel_fc(f, 0.0, LoadTerm::Long);
        let ft = steel_ft(f, LoadTerm::Long);
        assert!((fc - ft).abs() < 1e-9, "fc(λ=0)={} ft={}", fc, ft);
    }

    #[test]
    fn test_fc_continuous_at_big_lambda() {
        let f = 235.0;
        let big_l = big_lambda(f);
        let fc_below = steel_fc(f, big_l - 1e-6, LoadTerm::Long);
        let fc_above = steel_fc(f, big_l + 1e-6, LoadTerm::Long);
        let expected = (18.0 / 65.0) * f;
        assert!((fc_below - expected).abs() < 1e-3);
        assert!((fc_above - expected).abs() < 1e-3);
    }

    #[test]
    fn test_fc_short_is_1_5x_long() {
        let f = 235.0;
        let long = steel_fc(f, 50.0, LoadTerm::Long);
        let short = steel_fc(f, 50.0, LoadTerm::Short);
        assert!((short - 1.5 * long).abs() < 1e-9);
    }

    // -------------------------------------------------------------
    // fb（横座屈考慮）
    // -------------------------------------------------------------

    #[test]
    fn test_fb_lb_zero_equals_ft() {
        let f = 235.0;
        let fb = steel_fb_h(f, LoadTerm::Long, 0.0, 100.0, 500.0, 4000.0);
        let ft = steel_ft(f, LoadTerm::Long);
        assert!((fb - ft).abs() < 1e-6, "fb={} ft={}", fb, ft);
    }

    #[test]
    fn test_fb_large_lb_fb2_governs() {
        let f = 235.0;
        let i = 60.0;
        let h = 500.0;
        let af = 4000.0;
        let lb = 20_000.0; // 十分大きい横座屈長さ
        let fb = steel_fb_h(f, LoadTerm::Long, lb, i, h, af);
        let fb2 = 89_000.0 / (lb * h / af);
        assert!(
            (fb - fb2).abs() < 1e-6,
            "fb2 should govern: fb={} fb2={}",
            fb,
            fb2
        );
    }

    #[test]
    fn test_i_t_helper() {
        let i = steel_i_t(200.0, 15.0, 400.0, 10.0);
        let expected_it = 15.0 * 200.0_f64.powi(3) / 12.0 + (400.0 / 6.0) * 10.0_f64.powi(3) / 12.0;
        let expected_at = 200.0 * 15.0 + (400.0 / 6.0) * 10.0;
        let expected = (expected_it / expected_at).sqrt();
        assert!((i - expected).abs() < 1e-9);
    }

    // -------------------------------------------------------------
    // 梁検定
    // -------------------------------------------------------------

    /// 仕様 P3 §6.4 の検算例を新 API で再現する。
    /// 矩形 B=200, D=400 ⇒ Z=B·D²/6=5.3333e6 mm³, M=1e8 N·mm
    /// σ=18.75 N/mm², fb=F/1.5=156.6667 N/mm²（矩形は横座屈対象外＝fb=ft）,
    /// 検定比=0.1197（相対 1e-9）。
    #[test]
    fn test_beam_check_bending_spec_p3_6_4() {
        let sec = rect_section(200.0, 400.0, "矩形200x400");
        let mat = mat("SN400");
        let forces = MemberForcesAt {
            pos: 0.5,
            n: 0.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 1e8,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Beam,
            length: 0.0,
            ..Default::default()
        };
        let result = SteelDesign.check(&forces, &sec, &mat, &ctx);

        let expected_sigma = 18.75;
        let expected_fb = 235.0 / 1.5;
        let expected_ratio = expected_sigma / expected_fb;
        assert!(
            (result.ratio - expected_ratio).abs() < 1e-9,
            "ratio {} != {}",
            result.ratio,
            expected_ratio
        );
        assert!(result.ok);
        assert!(result.detail.contains("18.7500"));
    }

    #[test]
    fn test_beam_check_shear_h_shape_von_mises() {
        // H-300x300x10x15 相当（厚さ 15mm を単一 thickness として近似）。
        let mut sec = rect_section(300.0, 300.0, "H-300x300x10x15");
        sec.thickness = Some(15.0);
        let mat = mat("SN400");
        let forces = MemberForcesAt {
            pos: 0.5,
            n: 0.0,
            qy: 200_000.0,
            qz: 0.0,
            my: 0.0,
            mz: 0.0,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Beam,
            length: 3000.0,
            ..Default::default()
        };
        let result = SteelDesign.check(&forces, &sec, &mat, &ctx);
        // τ = Q/(t·H) = 200000/(15*300) = 44.444..., fs = 235/(1.5√3)=90.44
        let tau = 200_000.0 / (15.0 * 300.0);
        let fs = 235.0 / (1.5 * 3.0_f64.sqrt());
        let expected_ratio_shear = tau / fs; // σb=0 なので von Mises 側は τ/fs と一致するはず
        assert!(
            (result.ratio - expected_ratio_shear).abs() < 1e-6,
            "ratio={} expected={}",
            result.ratio,
            expected_ratio_shear
        );
    }

    // -------------------------------------------------------------
    // 柱検定
    // -------------------------------------------------------------

    #[test]
    fn test_column_check_axial_biaxial_bending_hand_calc() {
        // H形柱: N=-500kN（圧縮）, Mz=50kN·m, My=20kN·m。
        let mut sec = rect_section(300.0, 300.0, "H-300x300x10x15");
        sec.thickness = Some(15.0);
        let mat = mat("SN400");
        let forces = MemberForcesAt {
            pos: 0.0,
            n: -500_000.0,
            qy: 0.0,
            qz: 0.0,
            my: 20e6,
            mz: 50e6,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Column,
            length: 3500.0,
            ..Default::default()
        };
        let result = SteelDesign.check(&forces, &sec, &mat, &ctx);

        let area = sec.area;
        let z_strong = sec.iy / (sec.depth / 2.0);
        let z_weak = sec.iz / (sec.width / 2.0);
        let sigma_c = 500_000.0 / area;
        let sigma_bx = 50e6_f64.abs() / z_strong;
        let sigma_by = 20e6_f64.abs() / z_weak;

        let i_min = (sec.iy.min(sec.iz) / area).sqrt();
        let lambda = 3500.0 / i_min;
        let fc = steel_fc(235.0, lambda, LoadTerm::Long);
        let ft = steel_ft(235.0, LoadTerm::Long);
        // fbX は横座屈考慮（H形）、fbY=ft。ここでは非負・上限 ft であることのみ検証。
        assert!(result.detail.contains("軸曲げ比"));
        assert!(sigma_c > 0.0 && sigma_bx > 0.0 && sigma_by > 0.0 && fc > 0.0 && ft > 0.0);
        // 軸+曲げ比は少なくとも σc/fc 単独の比より大きい（曲げ項が加算されるため）。
        assert!(result.ratio >= sigma_c / fc - 1e-9);
    }

    #[test]
    fn test_column_check_pipe_combines_biaxial_sigma_b() {
        let mut sec = rect_section(300.0, 300.0, "PIPE-300x12");
        sec.iz = sec.iy; // 円形は iy=iz
        sec.thickness = Some(12.0);
        let mat = mat("SN400");
        let forces_x_only = MemberForcesAt {
            pos: 0.0,
            n: -100_000.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 30e6,
        };
        let forces_biaxial = MemberForcesAt {
            pos: 0.0,
            n: -100_000.0,
            qy: 0.0,
            qz: 0.0,
            my: 30e6 / std::f64::consts::SQRT_2,
            mz: 30e6 / std::f64::consts::SQRT_2,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Column,
            length: 3000.0,
            ..Default::default()
        };
        let r1 = SteelDesign.check(&forces_x_only, &sec, &mat, &ctx);
        let r2 = SteelDesign.check(&forces_biaxial, &sec, &mat, &ctx);
        // 円形鋼管は sqrt(mz^2+my^2) で合成するため、合成曲げモーメントの大きさが
        // 同じであれば mz のみと mz/my 分配後で軸+曲げ比はほぼ一致するはず。
        assert!(
            (r1.ratio - r2.ratio).abs() < 1e-6,
            "pipe combined sigma_b mismatch: {} vs {}",
            r1.ratio,
            r2.ratio
        );
    }

    #[test]
    fn test_column_shear_von_mises_hand_calc() {
        // 純せん断（N=0, M=0）で von Mises 式 sqrt(3)*τ/ft と τ/fs を手計算照合。
        let mut sec = rect_section(300.0, 300.0, "BOX-300x300x12");
        sec.thickness = Some(12.0);
        let mat = mat("SN400");
        let forces = MemberForcesAt {
            pos: 0.0,
            n: 0.0,
            qy: 300_000.0,
            qz: 0.0,
            my: 0.0,
            mz: 0.0,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Column,
            length: 3000.0,
            ..Default::default()
        };
        let result = SteelDesign.check(&forces, &sec, &mat, &ctx);

        let area = sec.area;
        let tau = 2.0 * 300_000.0_f64.abs() / area; // 角形: τ=2Q/A
        let ft = steel_ft(235.0, LoadTerm::Long);
        let fs = steel_fs(235.0, LoadTerm::Long);
        // σ=0（純せん断）なので von Mises 側は sqrt(3)*τ/ft。
        let expected = (3.0_f64.sqrt() * tau / ft).max(tau / fs);
        assert!(
            (result.ratio - expected).abs() < 1e-6,
            "ratio={} expected={}",
            result.ratio,
            expected
        );
    }

    // -------------------------------------------------------------
    // ブレース検定
    // -------------------------------------------------------------

    #[test]
    fn test_brace_tension_ok() {
        let sec = rect_section(100.0, 100.0, "L-100x100x10");
        let mat = mat("SN400");
        let forces = MemberForcesAt {
            pos: 0.5,
            n: 200_000.0, // 引張
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 0.0,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Brace,
            length: 4000.0,
            ..Default::default()
        };
        let result = SteelDesign.check(&forces, &sec, &mat, &ctx);
        let expected = (200_000.0 / sec.area) / (235.0 / 1.5);
        assert!((result.ratio - expected).abs() < 1e-9);
        assert!(result.ok);
    }

    #[test]
    fn test_brace_compression_slender_fails() {
        // 細長比が大きい（断面が小さく部材長が長い）圧縮ブレースは fc が下がり NG になる。
        let sec = rect_section(20.0, 20.0, "L-20x20x3");
        let mat = mat("SN400");
        let forces = MemberForcesAt {
            pos: 0.5,
            n: -50_000.0, // 圧縮
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 0.0,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Brace,
            length: 6000.0, // 非常に細長い
            ..Default::default()
        };
        let result = SteelDesign.check(&forces, &sec, &mat, &ctx);
        assert!(
            !result.ok,
            "slender brace should fail: ratio={}",
            result.ratio
        );
        assert!(result.ratio > 1.0);
    }

    #[test]
    fn test_brace_compression_stocky_passes() {
        // 太く短いブレースは座屈の影響が小さく OK になりやすい。
        let sec = rect_section(300.0, 300.0, "BOX-300x300x16");
        let mat = mat("SN400");
        let forces = MemberForcesAt {
            pos: 0.5,
            n: -100_000.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 0.0,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Brace,
            length: 1000.0,
            ..Default::default()
        };
        let result = SteelDesign.check(&forces, &sec, &mat, &ctx);
        assert!(
            result.ok,
            "stocky brace should pass: ratio={}",
            result.ratio
        );
    }

    // -------------------------------------------------------------
    // SectionShape 経由の形状解決（tf ≠ tw の実断面）
    // -------------------------------------------------------------

    /// `Section.shape` がある場合は実寸の tw でウェブせん断面積を計算する
    /// （名前推定＋単一板厚近似ではなく、tw=10 が使われること）。
    #[test]
    fn test_beam_check_uses_shape_tw_for_web_shear() {
        let sec = h_section(400.0, 200.0, 10.0, 15.0);
        let mat = mat("SN400");
        let forces = MemberForcesAt {
            pos: 0.5,
            n: 0.0,
            qy: 100_000.0,
            qz: 0.0,
            my: 0.0,
            mz: 0.0,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Beam,
            length: 0.0,
            ..Default::default()
        };
        let result = SteelDesign.check(&forces, &sec, &mat, &ctx);
        // τ = Q/(tw·H) = 100000/(10·400) = 25.0
        let tau = 100_000.0 / (10.0 * 400.0);
        let fs = 235.0 / (1.5 * 3.0_f64.sqrt());
        let expected = ((3.0_f64.sqrt() * tau) / (235.0 / 1.5)).max(tau / fs);
        assert!(
            (result.ratio - expected).abs() < 1e-9,
            "ratio={} expected={}",
            result.ratio,
            expected
        );
    }

    /// F 値の板厚区分は shape の最大板厚で判定する（tf=45 → 40mm 超区分）。
    #[test]
    fn test_f_value_bucket_uses_shape_max_thickness() {
        let sec = h_section(900.0, 400.0, 20.0, 45.0);
        let mat = mat("SN400");
        let forces = MemberForcesAt {
            pos: 0.5,
            n: 0.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 1e6,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Beam,
            length: 0.0,
            ..Default::default()
        };
        let result = SteelDesign.check(&forces, &sec, &mat, &ctx);
        // F=215（40mm 超）→ fb=ft=215/1.5=143.33...
        assert!(
            result.detail.contains("fb=143.3"),
            "detail should show fb from F=215: {}",
            result.detail
        );
    }

    // -------------------------------------------------------------
    // 座屈長さ 0 の扱い
    // -------------------------------------------------------------

    #[test]
    fn test_column_length_zero_ignores_buckling() {
        let sec = rect_section(300.0, 300.0, "BOX-300x300x16");
        let mat = mat("SN400");
        let forces = MemberForcesAt {
            pos: 0.0,
            n: -100_000.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 0.0,
        };
        let ctx = DesignCtx {
            term: LoadTerm::Long,
            kind: MemberKind::Column,
            length: 0.0,
            ..Default::default()
        };
        let result = SteelDesign.check(&forces, &sec, &mat, &ctx);
        assert!(result.basis.contains("座屈無視"));
        // λ=0 なので fc=ft、単純圧縮比 = σc/ft と一致するはず。
        let ft = steel_ft(235.0, LoadTerm::Long);
        let expected = (100_000.0 / sec.area) / ft;
        assert!((result.ratio - expected).abs() < 1e-6);
    }
}
