//! RC 造の許容応力度と断面検定（RESP-D マニュアル「計算編 04 断面検定
//! （許容応力度検定）」の RC 造部分に準拠）。
//!
//! 準拠する規準:
//! - 許容応力度・ヤング係数比: 2010年版 RC 規準・構造規定
//! - 梁の曲げ・せん断検定: RC 規準 13条
//! - 柱の軸力＋曲げ検定: RC 規準 14条
//!
//! # 実装方針（全体）
//! - `Section.shape` が `RcRect`/`RcCircle` でない場合（配筋情報なし）は
//!   検定をスキップし `ok=true` で返す（旧実装と同じフォールバック）。
//! - `Material.fc` が未設定/0 の場合も同様にスキップする。
//! - 梁は強軸曲げ（`mz`）とそれに対のせん断（`qy`）のみを検定する
//!   （マニュアルの梁断面検定の対象と一致）。
//! - 柱は軸力（M=0）・軸力＋二軸曲げ・二方向せん断を検定する。
//! - `MemberKind::Brace` は RC 部材としては未対応のため、梁の検定式で代用する。

use crate::{CheckResult, DesignCheck, DesignCtx, LoadTerm, MemberForcesAt, MemberKind};
use squid_n_core::model::{Material, Section};
use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

// ============================================================================
// 1. 許容応力度（2010年版 RC 規準・構造規定）
// ============================================================================

/// コンクリートの許容圧縮応力度 fc [N/mm²]。長期 = Fc/3、短期 = 長期 × 2。
pub fn concrete_allowable_compression(fc: f64, long_term: bool) -> f64 {
    let long = fc / 3.0;
    if long_term {
        long
    } else {
        long * 2.0
    }
}

/// コンクリートの許容せん断応力度 fs [N/mm²]。
/// 長期 = min(Fc/30, 0.49+Fc/100)、短期 = 長期 × 1.5
/// （圧縮の ×2 と異なり、せん断は ×1.5 である点に注意）。
pub fn concrete_allowable_shear(fc: f64, long_term: bool) -> f64 {
    let long = (fc / 30.0).min(0.49 + fc / 100.0);
    if long_term {
        long
    } else {
        long * 1.5
    }
}

/// 断面算定用のヤング係数比 n（Fc に応じた区分値）。
pub fn young_ratio_n(fc: f64) -> f64 {
    if fc <= 27.0 {
        15.0
    } else if fc <= 36.0 {
        13.0
    } else if fc <= 48.0 {
        11.0
    } else if fc <= 60.0 {
        9.0
    } else {
        // 60 < Fc <= 120 の区分値をそれ以上にも代表値として適用する。
        7.0
    }
}

/// コンクリートのヤング係数 Ec [N/mm²]（参考実装）。
/// `Ec = 3.35e4・(γ/24)²・(Fc/60)^(1/3)`、γ は単位容積重量 [kN/m³]（既定 23）。
pub fn concrete_young_modulus(fc: f64, gamma_kn_m3: Option<f64>) -> f64 {
    let gamma = gamma_kn_m3.unwrap_or(23.0);
    3.35e4 * (gamma / 24.0).powi(2) * (fc / 60.0).powf(1.0 / 3.0)
}

/// 異形鉄筋の許容引張・圧縮応力度 ft [N/mm²]。
/// SD345/SD390/SD490 は径 D29 以上（`dia >= 29.0`）で長期値が低減される。
pub fn rebar_allowable_tension(grade: &str, dia: f64, long_term: bool) -> f64 {
    let g = grade.trim();
    if long_term {
        if g == "SR235" || g == "SR295" {
            155.0
        } else if g.starts_with("SD295") {
            195.0
        } else if g == "SD345" || g == "SD390" || g == "SD490" {
            if dia >= 29.0 {
                195.0
            } else {
                215.0
            }
        } else {
            195.0
        }
    } else if g == "SR235" {
        235.0
    } else if g == "SR295" || g.starts_with("SD295") {
        295.0
    } else if g == "SD345" {
        345.0
    } else if g == "SD390" {
        390.0
    } else if g == "SD490" {
        490.0
    } else {
        295.0
    }
}

/// せん断補強筋の許容引張応力度 w_ft [N/mm²]。
pub fn rebar_allowable_shear(grade: &str, long_term: bool) -> f64 {
    let g = grade.trim();
    if long_term {
        if g == "SR235" {
            155.0
        } else {
            195.0
        }
    } else if g.starts_with("SD295") {
        295.0
    } else if g == "SD345" {
        345.0
    } else if g == "SD390" {
        390.0
    } else if g == "SD490" {
        // F 値スケーリング: SD490 短期はせん断のみ F=390 に頭打ち。
        390.0
    } else {
        295.0
    }
}

// ============================================================================
// 2. 断面諸元の抽出
// ============================================================================

/// 検討方向 1 軸分の断面諸元。
struct AxisProps {
    /// 検討方向の幅 [mm]（強軸曲げなら sec.width 等）。
    b: f64,
    /// 検討方向のせい D [mm]。
    d_full: f64,
    /// 引張縁から引張筋重心までの距離 dt [mm]。
    dt: f64,
    /// 有効せい d = D - dt [mm]。
    d: f64,
    /// 引張鉄筋断面積 at [mm²]（片側）。
    at: f64,
    /// 圧縮鉄筋断面積 ac [mm²]（片側、at と同値の対称複筋仮定）。
    ac: f64,
    /// 応力中心間距離 j = 7d/8 [mm]。
    j: f64,
    /// せん断補強筋比 pw。
    pw: f64,
}

/// 主筋 1 本あたりの断面積 [mm²]。
fn one_bar_area(dia: f64) -> f64 {
    let r = dia / 2.0;
    std::f64::consts::PI * r * r
}

/// 主筋セットの総断面積 [mm²]。
fn bar_set_area(bar: &BarSet) -> f64 {
    bar.count as f64 * one_bar_area(bar.dia)
}

/// 引張縁 → 引張筋重心までの距離 dt [mm]。
///
/// 1 段筋（`layers<=1`）は重心 k1 = cover + shear.dia + main.dia/2。
/// 2 段以上は RC 配筋指針式（2 段の場合）
/// `k2 = k1 + D1/2 + k' + D2/2`（`k' = max(25, 1.5・dia)`, `D1=D2=main.dia`）
/// により `dt = (k1+k2)/2` とする。3 段以上は各段が等間隔 `s = dia + k'` で
/// 並び、各段の本数が等しいと仮定して重心を平均で一般化する:
/// `dt = k1 + (layers-1)/2・s`（layers=2 で上式に一致）。
fn tension_dt(cover: f64, shear_dia: f64, main: &BarSet) -> f64 {
    let k1 = cover + shear_dia + main.dia / 2.0;
    if main.layers <= 1 {
        return k1;
    }
    let k_prime = 25.0_f64.max(1.5 * main.dia);
    let s = main.dia + k_prime;
    k1 + (main.layers as f64 - 1.0) / 2.0 * s
}

/// せん断補強筋比 pw = (legs・π/4・dia²) / (b・pitch)。pitch<=0 のときは 0。
fn pw_ratio(shear: &ShearBar, b: f64) -> f64 {
    if shear.pitch <= 0.0 || b <= 0.0 {
        return 0.0;
    }
    let aw = shear.legs as f64 * std::f64::consts::PI / 4.0 * shear.dia * shear.dia;
    aw / (b * shear.pitch)
}

/// 矩形断面 1 軸分の断面諸元を算定する。
///
/// `width_dir_b`: 検討方向の幅、`depth_dir_d`: 検討方向のせい、
/// `main`: 当該方向の主筋（強軸曲げは main_x、弱軸曲げは main_y）。
fn rect_axis_props(
    width_dir_b: f64,
    depth_dir_d: f64,
    main: &BarSet,
    rebar: &RcRebar,
) -> AxisProps {
    let dt = tension_dt(rebar.cover, rebar.shear.dia, main);
    let d = depth_dir_d - dt;
    let at = bar_set_area(main) / 2.0;
    AxisProps {
        b: width_dir_b,
        d_full: depth_dir_d,
        dt,
        d,
        at,
        ac: at,
        j: 7.0 * d / 8.0,
        pw: pw_ratio(&rebar.shear, width_dir_b),
    }
}

/// 強軸曲げ（mz）用の断面諸元。b=sec.width, D=sec.depth, 主筋=main_x。
fn rect_axis_props_strong(sec: &Section, rebar: &RcRebar) -> AxisProps {
    rect_axis_props(sec.width, sec.depth, &rebar.main_x, rebar)
}

/// 弱軸曲げ（my）用の断面諸元。b=sec.depth, D=sec.width, 主筋=main_y。
fn rect_axis_props_weak(sec: &Section, rebar: &RcRebar) -> AxisProps {
    rect_axis_props(sec.depth, sec.width, &rebar.main_y, rebar)
}

/// 円形柱の等価矩形断面諸元。b=(D/2)√π、せい=D。
/// 引張筋本数 nt = ng/4+1（ng = 全主筋本数、`rebar.main_x.count` を採用）。
/// 対称複筋（at=ac）を仮定する。
fn circle_axis_props(d_full: f64, rebar: &RcRebar) -> AxisProps {
    let b = (d_full / 2.0) * std::f64::consts::PI.sqrt();
    let ng = rebar.main_x.count as f64;
    let nt = ng / 4.0 + 1.0;
    let at = nt * one_bar_area(rebar.main_x.dia);
    let dt = tension_dt(rebar.cover, rebar.shear.dia, &rebar.main_x);
    let d = d_full - dt;
    AxisProps {
        b,
        d_full,
        dt,
        d,
        at,
        ac: at,
        j: 7.0 * d / 8.0,
        pw: pw_ratio(&rebar.shear, b),
    }
}

// ============================================================================
// 3. 許容応力度のまとめ（部材単位で term 依存の値を 1 回だけ計算する）
// ============================================================================

/// 検定に用いる許容応力度一式（コンクリート・せん断補強筋。ft は主筋径に
/// 依存するため軸別に別途算定する）。
struct RcAllow {
    /// コンクリート許容圧縮応力度 fc [N/mm²]（長期/短期は算定済み）。
    fc: f64,
    /// コンクリート許容せん断応力度 fs [N/mm²]。
    fs: f64,
    /// せん断補強筋許容引張応力度 w_ft [N/mm²]。
    w_ft: f64,
    /// ヤング係数比 n。
    n_ratio: f64,
}

fn rc_allow(fc_raw: f64, grade: &str, long_term: bool) -> RcAllow {
    RcAllow {
        fc: concrete_allowable_compression(fc_raw, long_term),
        fs: concrete_allowable_shear(fc_raw, long_term),
        w_ft: rebar_allowable_shear(grade, long_term),
        n_ratio: young_ratio_n(fc_raw),
    }
}

// ============================================================================
// 4. せん断スパン比 α とせん断耐力
// ============================================================================

/// せん断スパン比による割増係数 α = 4/(M/(Q・d)+1)。`max_alpha` でクランプ
/// （梁 2.0、柱 1.5）。下限は共通で 1.0。
fn shear_alpha(m: f64, q: f64, d: f64, max_alpha: f64) -> f64 {
    if q.abs() < 1e-9 || d <= 0.0 {
        return max_alpha;
    }
    let mqd = m.abs() / (q.abs() * d);
    let alpha = 4.0 / (mqd + 1.0);
    alpha.clamp(1.0, max_alpha)
}

/// 許容せん断力 QA [N]。
///
/// 梁（`is_column=false`）:
/// - 長期  `QAL = b・j・(α・fs + 0.5・w_ft・(pw-0.002))`（pw は 0.6% 上限）
/// - 短期・損傷制御 `QAS = b・j・(2/3・α・fs + 0.5・w_ft・(pw-0.002))`
/// - 短期・安全確保 `QAS = b・j・(α・fs + 0.5・w_ft・(pw-0.002))`（pw は 1.2% 上限）
///
/// 柱（`is_column=true`）:
/// - 長期  `QAL = b・j・α・fs`（補強筋項なし）
/// - 短期・損傷制御 `QAS = b・j・(2/3・α・fs + 0.5・w_ft・(pw-0.002))`
/// - 短期・安全確保 `QAS = b・j・(fs + 0.5・w_ft・(pw-0.002))`（**α を含まない**）
///
/// いずれも pw<0.002 のときせん断補強筋項は 0（マイナスにしない）。
fn shear_capacity(
    props: &AxisProps,
    allow: &RcAllow,
    alpha: f64,
    term: LoadTerm,
    damage_control: bool,
    is_column: bool,
) -> f64 {
    let pw_cap = if term == LoadTerm::Long { 0.006 } else { 0.012 };
    let pw = props.pw.min(pw_cap);
    let pw_term = if props.pw < 0.002 {
        0.0
    } else {
        0.5 * allow.w_ft * (pw - 0.002)
    };

    match term {
        LoadTerm::Long => {
            if is_column {
                props.b * props.j * alpha * allow.fs
            } else {
                props.b * props.j * (alpha * allow.fs + pw_term)
            }
        }
        LoadTerm::Short => {
            if damage_control {
                props.b * props.j * ((2.0 / 3.0) * alpha * allow.fs + pw_term)
            } else if is_column {
                // 柱の安全確保のための検討式は α を含まない。
                props.b * props.j * (allow.fs + pw_term)
            } else {
                props.b * props.j * (alpha * allow.fs + pw_term)
            }
        }
    }
}

// ============================================================================
// 5. 梁の曲げ耐力（RC 規準 13条）
// ============================================================================

struct BeamMoment {
    /// 引張鉄筋支配の許容曲げモーメント MA_t = at・ft・j。
    ma_t: f64,
    /// 圧縮縁コンクリート支配の許容曲げモーメント MA_c。
    ma_c: f64,
    /// MA = min(MA_t, MA_c)。
    ma: f64,
}

/// 梁の許容曲げモーメント MA を算定する（RC 規準 13条）。
///
/// `MA_t = at・ft・j` は引張鉄筋が ft に達する状態（pt が釣合鉄筋比以下）の
/// 許容曲げモーメント。`MA_c` は複筋断面の弾性（全ひび割れ断面）解析により
/// 圧縮縁コンクリート応力度が fc に達するモーメントで、pt が釣合鉄筋比を
/// 超える（圧縮側支配）場合に効く。中立軸位置 xn を
/// `b・xn²/2 + (n-1)・ac・(xn-dc) = n・at・(d-xn)`（dc=dt）から解き、
/// `Icr = b・xn³/3 + (n-1)・ac・(xn-dc)² + n・at・(d-xn)²`、
/// `MA_c = fc・Icr/xn` とする。
///
/// `MA = min(MA_t, MA_c)` をとることで、マニュアルの
/// 「pt <= pt_balance なら C1（引張支配）、それを超えれば C2（圧縮支配）」
/// という分岐と等価な結果が得られる（過小配筋では MA_c が大きく MA_t が支配、
/// 過大配筋では逆になる）。
fn beam_moment_capacity(props: &AxisProps, ft: f64, fc: f64, n_ratio: f64) -> BeamMoment {
    let ma_t = props.at * ft * props.j;

    let dc = props.dt;
    let d = props.d;
    let b = props.b;
    let ac = props.ac;
    let at = props.at;

    let a_coef = b / 2.0;
    let b_coef = (n_ratio - 1.0) * ac + n_ratio * at;
    let c_coef = -((n_ratio - 1.0) * ac * dc + n_ratio * at * d);

    let ma_c = if a_coef > 0.0 {
        let disc = b_coef * b_coef - 4.0 * a_coef * c_coef;
        if disc >= 0.0 {
            let xn = (-b_coef + disc.sqrt()) / (2.0 * a_coef);
            if xn > 0.0 {
                let icr = b * xn.powi(3) / 3.0
                    + (n_ratio - 1.0) * ac * (xn - dc).powi(2)
                    + n_ratio * at * (d - xn).powi(2);
                fc * icr / xn
            } else {
                f64::INFINITY
            }
        } else {
            f64::INFINITY
        }
    } else {
        f64::INFINITY
    };

    BeamMoment {
        ma_t,
        ma_c,
        ma: ma_t.min(ma_c),
    }
}

// ============================================================================
// 6. 柱の軸力・軸力+曲げ耐力（RC 規準 14条）
// ============================================================================

/// 許容軸力 NA（M=0）[N]。`Ae = A + (n-1)・As_total`、
/// `NA = min(fc・Ae, ft・Ae/n)`。
fn column_axial_capacity(gross_area: f64, as_total: f64, fc: f64, ft: f64, n_ratio: f64) -> f64 {
    let ae = gross_area + (n_ratio - 1.0) * as_total;
    (fc * ae).min(ft * ae / n_ratio)
}

/// N-M 相関曲線を構成する 1 軸分の状態（断面諸元＋直交方向鉄筋の集約分）。
struct ColumnAxis {
    props: AxisProps,
    /// 直交方向の主筋総断面積（断面中央 D/2 に集約、RC 規準 14条の慣習）。
    at_perp: f64,
    /// 当該軸の主筋径に応じた許容引張・圧縮応力度 ft(=r_fc)。
    ft: f64,
}

/// 中立軸位置 xn における (N_allow, |M_allow|) を、圧縮縁コンクリート・
/// 圧縮鉄筋・引張鉄筋の 3 条件のうち最も厳しいもので支配させて求める。
///
/// 応力分布はひずみ線形（弾性、コンクリート引張無視）を仮定し、圧縮縁
/// （y=0）からの距離 y の位置での「仮想コンクリート応力」を
/// `σ(y) = s・(xn-y)` とする（s は未知のスケール）。鉄筋位置 y_bar が
/// 圧縮側（y_bar<=xn）なら (n-1) 倍換算（コンクリートが既に積分域に
/// 含まれるため二重計上を避ける）、引張側（y_bar>xn）なら n 倍換算とする。
/// この定式化は `xn>D`（全断面圧縮）でも自然に成立し、`xn→∞` の極限で
/// `column_axial_capacity` と一致する。
fn column_nm_at_xn(axis: &ColumnAxis, allow: &RcAllow, xn: f64) -> Option<(f64, f64)> {
    if xn <= 0.0 {
        return None;
    }
    let p = &axis.props;
    let d_full = p.d_full;
    let b = p.b;
    let n_ratio = allow.n_ratio;
    let fc = allow.fc;
    let r_fc = axis.ft;
    let ft = axis.ft;

    // 各条件の限界応力スケール s。
    let s_bar = |y: f64, area: f64| -> f64 {
        if area <= 0.0 {
            return f64::INFINITY;
        }
        let diff = xn - y;
        if diff.abs() < 1e-9 {
            return f64::INFINITY;
        }
        if diff > 0.0 {
            r_fc / (n_ratio * diff)
        } else {
            ft / (n_ratio * (-diff))
        }
    };

    let s1 = fc / xn;
    let s2 = s_bar(p.dt, p.ac);
    let s3 = s_bar(d_full - p.dt, p.at);
    let s = s1.min(s2).min(s3);
    if !s.is_finite() || s <= 0.0 {
        return None;
    }

    let xc = xn.min(d_full);
    if xc <= 0.0 {
        return None;
    }

    // コンクリート圧縮域（0..xc）の合力・重心まわりモーメント。
    let nc = b * s * (xn * xc - xc * xc / 2.0);
    let mc =
        b * s * (xn * (d_full / 2.0) * xc - (xn + d_full / 2.0) * xc * xc / 2.0 + xc.powi(3) / 3.0);

    let bar_contrib = |y: f64, area: f64| -> (f64, f64) {
        if area <= 0.0 {
            return (0.0, 0.0);
        }
        let mult = if y <= xn { n_ratio - 1.0 } else { n_ratio };
        let force = area * mult * s * (xn - y);
        let moment = force * (d_full / 2.0 - y);
        (force, moment)
    };

    let (n_c, m_c) = bar_contrib(p.dt, p.ac);
    let (n_t, m_t) = bar_contrib(d_full - p.dt, p.at);
    let (n_p, m_p) = bar_contrib(d_full / 2.0, axis.at_perp);

    let n_total = nc + n_c + n_t + n_p;
    let m_total = mc + m_c + m_t + m_p;
    Some((n_total, m_total.abs()))
}

const XN_SCAN_POINTS: usize = 400;
const XN_RATIO_MIN: f64 = 0.02;
const XN_RATIO_MAX: f64 = 10.0;

/// 軸力 N（圧縮を正、圧縮負の内力を反転して渡すこと）に対する許容曲げ
/// モーメント MA(N) を求めるための N-M 相関曲線を構成する。
/// `xn/D = 0.02〜10` を対数的にスキャンし、`column_axial_capacity` による
/// M=0 の端点を明示的に追加する。
fn column_nm_curve(axis: &ColumnAxis, allow: &RcAllow, na_point: f64) -> Vec<(f64, f64)> {
    let mut pts = Vec::with_capacity(XN_SCAN_POINTS + 1);
    let log_min = XN_RATIO_MIN.ln();
    let log_max = XN_RATIO_MAX.ln();
    for i in 0..XN_SCAN_POINTS {
        let t = i as f64 / (XN_SCAN_POINTS as f64 - 1.0);
        let ratio = (log_min + t * (log_max - log_min)).exp();
        let xn = axis.props.d_full * ratio;
        if let Some(pt) = column_nm_at_xn(axis, allow, xn) {
            if pt.0.is_finite() && pt.1.is_finite() {
                pts.push(pt);
            }
        }
    }
    pts.push((na_point, 0.0));
    pts.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    pts
}

/// N-M 相関曲線から、設計軸力 `n_design`（圧縮正）に対する許容曲げモーメント
/// MA を線形補間で求める。範囲外は端点値でクランプする。
fn interp_ma(points: &[(f64, f64)], n_design: f64) -> f64 {
    if points.is_empty() {
        return 0.0;
    }
    if n_design <= points[0].0 {
        return points[0].1;
    }
    let last = points.len() - 1;
    if n_design >= points[last].0 {
        return points[last].1;
    }
    for w in points.windows(2) {
        let (n0, m0) = w[0];
        let (n1, m1) = w[1];
        if n_design >= n0 && n_design <= n1 {
            if (n1 - n0).abs() < 1e-9 {
                return m0.max(m1);
            }
            let t = (n_design - n0) / (n1 - n0);
            return m0 + t * (m1 - m0);
        }
    }
    points[last].1
}

// ============================================================================
// 7. DesignCheck 実装
// ============================================================================

pub struct RcDesign;

impl DesignCheck for RcDesign {
    fn check(
        &self,
        forces: &MemberForcesAt,
        sec: &Section,
        mat: &Material,
        ctx: &DesignCtx,
    ) -> CheckResult {
        let fc_raw = mat.fc.unwrap_or(0.0);
        if fc_raw <= 0.0 {
            return CheckResult {
                ratio: 0.0,
                ok: true,
                basis: "RC 検定: Fc 未設定".to_string(),
                detail: "Material.fc が None/0 です。コンクリート強度を設定してください。"
                    .to_string(),
            };
        }

        let shape = match &sec.shape {
            Some(s @ SectionShape::RcRect { .. }) => s,
            Some(s @ SectionShape::RcCircle { .. }) => s,
            _ => {
                return CheckResult {
                    ratio: 0.0,
                    ok: true,
                    basis: "RC 検定: 配筋情報なし".to_string(),
                    detail: "Section.shape が RcRect/RcCircle ではないため検定をスキップしました。"
                        .to_string(),
                };
            }
        };

        match ctx.kind {
            MemberKind::Beam | MemberKind::Brace => {
                beam_check(forces, sec, mat, ctx, shape, fc_raw)
            }
            MemberKind::Column => column_check(forces, sec, mat, ctx, shape, fc_raw),
        }
    }
}

/// 梁の断面検定（RC 規準 13条）。強軸曲げ mz とそれに対のせん断 qy のみを扱う。
fn beam_check(
    forces: &MemberForcesAt,
    sec: &Section,
    mat: &Material,
    ctx: &DesignCtx,
    shape: &SectionShape,
    fc_raw: f64,
) -> CheckResult {
    let rebar = match shape {
        SectionShape::RcRect { rebar, .. } => rebar,
        SectionShape::RcCircle { rebar, .. } => rebar,
        _ => unreachable!(),
    };
    let long_term = ctx.term == LoadTerm::Long;
    let grade = mat.name.as_str();
    let allow = rc_allow(fc_raw, grade, long_term);

    let props = if let SectionShape::RcCircle { d, .. } = shape {
        circle_axis_props(*d, rebar)
    } else {
        rect_axis_props_strong(sec, rebar)
    };
    let ft = rebar_allowable_tension(grade, rebar.main_x.dia, long_term);

    let bm = beam_moment_capacity(&props, ft, allow.fc, allow.n_ratio);
    let ratio_m = if bm.ma > 0.0 {
        forces.mz.abs() / bm.ma
    } else {
        0.0
    };

    let (m_for_alpha, q_for_alpha) = ctx.shear_span.unwrap_or((forces.mz.abs(), forces.qy.abs()));
    let alpha = shear_alpha(m_for_alpha, q_for_alpha, props.d, 2.0);
    let qa = shear_capacity(
        &props,
        &allow,
        alpha,
        ctx.term,
        ctx.rc_damage_control,
        false,
    );
    let ratio_q = if qa > 0.0 { forces.qy.abs() / qa } else { 0.0 };

    let ratio = ratio_m.max(ratio_q);
    let basis = "RC 規準13条（梁の曲げ・せん断）".to_string();
    let detail = format!(
        "MA_t={:.1} N·mm, MA_c={:.1} N·mm, MA={:.1} N·mm, |mz|={:.1} N·mm, \
         QA={:.1} N, |qy|={:.1} N, α={:.3}, pw={:.5}, at={:.1} mm², d={:.1} mm, j={:.1} mm",
        bm.ma_t,
        bm.ma_c,
        bm.ma,
        forces.mz,
        qa,
        forces.qy,
        alpha,
        props.pw,
        props.at,
        props.d,
        props.j
    );

    CheckResult {
        ratio,
        ok: ratio <= 1.0,
        basis,
        detail,
    }
}

/// 柱の断面検定（RC 規準 14条）。軸力・軸力+二軸曲げ・二方向せん断を扱う。
fn column_check(
    forces: &MemberForcesAt,
    sec: &Section,
    mat: &Material,
    ctx: &DesignCtx,
    shape: &SectionShape,
    fc_raw: f64,
) -> CheckResult {
    let long_term = ctx.term == LoadTerm::Long;
    let grade = mat.name.as_str();
    let allow = rc_allow(fc_raw, grade, long_term);

    // 圧縮を正とする設計軸力（forces.n は引張正・圧縮負）。
    let n_design = -forces.n;

    if let SectionShape::RcCircle { d, rebar } = shape {
        let d_full = *d;
        let props = circle_axis_props(d_full, rebar);
        let ft = rebar_allowable_tension(grade, rebar.main_x.dia, long_term);

        let gross_area = std::f64::consts::PI * d_full * d_full / 4.0;
        let as_total = rebar.main_x.count as f64 * one_bar_area(rebar.main_x.dia);
        let na = column_axial_capacity(gross_area, as_total, allow.fc, ft, allow.n_ratio);

        let axis = ColumnAxis {
            props,
            at_perp: 0.0,
            ft,
        };
        let curve = column_nm_curve(&axis, &allow, na);
        let ma = interp_ma(&curve, n_design);

        let ratio_axial = if forces.n < 0.0 && na > 0.0 {
            (-forces.n) / na
        } else {
            0.0
        };
        let ratio_moment = if ma > 0.0 {
            (forces.mz / ma).powi(2) + (forces.my / ma).powi(2)
        } else {
            0.0
        };

        let (m_for_alpha_y, q_for_alpha_y) =
            ctx.shear_span.unwrap_or((forces.mz.abs(), forces.qy.abs()));
        let alpha_y = shear_alpha(m_for_alpha_y, q_for_alpha_y, axis.props.d, 1.5);
        let qay = shear_capacity(
            &axis.props,
            &allow,
            alpha_y,
            ctx.term,
            ctx.rc_damage_control,
            true,
        );
        let ratio_qy = if qay > 0.0 {
            forces.qy.abs() / qay
        } else {
            0.0
        };

        let (m_for_alpha_z, q_for_alpha_z) =
            ctx.shear_span.unwrap_or((forces.my.abs(), forces.qz.abs()));
        let alpha_z = shear_alpha(m_for_alpha_z, q_for_alpha_z, axis.props.d, 1.5);
        let qaz = shear_capacity(
            &axis.props,
            &allow,
            alpha_z,
            ctx.term,
            ctx.rc_damage_control,
            true,
        );
        let ratio_qz = if qaz > 0.0 {
            forces.qz.abs() / qaz
        } else {
            0.0
        };

        let ratio = ratio_axial.max(ratio_moment).max(ratio_qy).max(ratio_qz);

        let basis = "RC 規準14条（円形柱、等価矩形近似）".to_string();
        let detail = format!(
            "NA={:.1} N, N={:.1} N, MA={:.1} N·mm（等価矩形近似）, mz={:.1} N·mm, my={:.1} N·mm, \
             QAy={:.1} N, QAz={:.1} N, αy={:.3}, αz={:.3}, pw={:.5}, at={:.1} mm², d={:.1} mm",
            na,
            n_design,
            ma,
            forces.mz,
            forces.my,
            qay,
            qaz,
            alpha_y,
            alpha_z,
            axis.props.pw,
            axis.props.at,
            axis.props.d
        );

        return CheckResult {
            ratio,
            ok: ratio <= 1.0,
            basis,
            detail,
        };
    }

    let rebar = match shape {
        SectionShape::RcRect { rebar, .. } => rebar,
        _ => unreachable!(),
    };

    let props_z = rect_axis_props_strong(sec, rebar); // mz 方向
    let props_y = rect_axis_props_weak(sec, rebar); // my 方向
    let ft_z = rebar_allowable_tension(grade, rebar.main_x.dia, long_term);
    let ft_y = rebar_allowable_tension(grade, rebar.main_y.dia, long_term);

    let gross_area = sec.width * sec.depth;
    let as_total = bar_set_area(&rebar.main_x) + bar_set_area(&rebar.main_y);
    // NA 用の ft は D29 以上の低減を保守的に反映するため、両方向のうち
    // 大径側（許容応力度が低い方）を採用する。
    let ft_axial =
        rebar_allowable_tension(grade, rebar.main_x.dia.max(rebar.main_y.dia), long_term);
    let na = column_axial_capacity(gross_area, as_total, allow.fc, ft_axial, allow.n_ratio);

    let at_perp_for_z = bar_set_area(&rebar.main_y);
    let at_perp_for_y = bar_set_area(&rebar.main_x);

    let axis_z = ColumnAxis {
        props: props_z,
        at_perp: at_perp_for_z,
        ft: ft_z,
    };
    let axis_y = ColumnAxis {
        props: props_y,
        at_perp: at_perp_for_y,
        ft: ft_y,
    };

    let curve_z = column_nm_curve(&axis_z, &allow, na);
    let curve_y = column_nm_curve(&axis_y, &allow, na);
    let ma_z = interp_ma(&curve_z, n_design);
    let ma_y = interp_ma(&curve_y, n_design);

    let ratio_axial = if forces.n < 0.0 && na > 0.0 {
        (-forces.n) / na
    } else {
        0.0
    };
    let ratio_z = if ma_z > 0.0 {
        forces.mz.abs() / ma_z
    } else {
        0.0
    };
    let ratio_y = if ma_y > 0.0 {
        forces.my.abs() / ma_y
    } else {
        0.0
    };
    let ratio_moment = ratio_z + ratio_y;

    let (m_for_alpha_y, q_for_alpha_y) =
        ctx.shear_span.unwrap_or((forces.mz.abs(), forces.qy.abs()));
    let alpha_y = shear_alpha(m_for_alpha_y, q_for_alpha_y, axis_z.props.d, 1.5);
    let qay = shear_capacity(
        &axis_z.props,
        &allow,
        alpha_y,
        ctx.term,
        ctx.rc_damage_control,
        true,
    );
    let ratio_qy = if qay > 0.0 {
        forces.qy.abs() / qay
    } else {
        0.0
    };

    let (m_for_alpha_z, q_for_alpha_z) =
        ctx.shear_span.unwrap_or((forces.my.abs(), forces.qz.abs()));
    let alpha_z = shear_alpha(m_for_alpha_z, q_for_alpha_z, axis_y.props.d, 1.5);
    let qaz = shear_capacity(
        &axis_y.props,
        &allow,
        alpha_z,
        ctx.term,
        ctx.rc_damage_control,
        true,
    );
    let ratio_qz = if qaz > 0.0 {
        forces.qz.abs() / qaz
    } else {
        0.0
    };

    let ratio = ratio_axial.max(ratio_moment).max(ratio_qy).max(ratio_qz);

    let basis = "RC 規準14条（柱、軸力+二軸曲げ+せん断）".to_string();
    let detail = format!(
        "NA={:.1} N, N={:.1} N, MA_z={:.1} N·mm, MA_y={:.1} N·mm, mz={:.1} N·mm, my={:.1} N·mm, \
         QAy={:.1} N, QAz={:.1} N, αy={:.3}, αz={:.3}, pw_z={:.5}, pw_y={:.5}",
        na,
        n_design,
        ma_z,
        ma_y,
        forces.mz,
        forces.my,
        qay,
        qaz,
        alpha_y,
        alpha_z,
        axis_z.props.pw,
        axis_y.props.pw
    );

    CheckResult {
        ratio,
        ok: ratio <= 1.0,
        basis,
        detail,
    }
}

// ============================================================================
// テスト
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::ids::{MaterialId, SectionId};
    use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape, ShearBar};

    fn make_material(fc: f64, grade: &str) -> Material {
        Material {
            id: MaterialId(0),
            name: grade.to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: Some(fc),
            fy: None,
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn rc_rect_shape(
        b: f64,
        d: f64,
        main_count: u32,
        main_dia: f64,
        main_layers: u32,
        cover: f64,
        shear_dia: f64,
        shear_pitch: f64,
        shear_legs: u32,
    ) -> SectionShape {
        SectionShape::RcRect {
            b,
            d,
            rebar: RcRebar {
                main_x: BarSet {
                    count: main_count,
                    dia: main_dia,
                    layers: main_layers,
                },
                main_y: BarSet {
                    count: main_count,
                    dia: main_dia,
                    layers: main_layers,
                },
                cover,
                shear: ShearBar {
                    dia: shear_dia,
                    pitch: shear_pitch,
                    legs: shear_legs,
                },
            },
        }
    }

    fn make_section(shape: SectionShape) -> Section {
        shape.to_section(SectionId(0), "test".to_string())
    }

    fn ctx_beam(term: LoadTerm) -> DesignCtx {
        DesignCtx {
            term,
            kind: MemberKind::Beam,
            ..Default::default()
        }
    }

    fn ctx_column(term: LoadTerm) -> DesignCtx {
        DesignCtx {
            term,
            kind: MemberKind::Column,
            ..Default::default()
        }
    }

    // ------------------------------------------------------------------
    // 許容応力度
    // ------------------------------------------------------------------

    #[test]
    fn test_concrete_shear_long_term_min_branch() {
        // Fc=21: Fc/30=0.7, 0.49+Fc/100=0.7 で同値。
        let fs = concrete_allowable_shear(21.0, true);
        assert!((fs - 0.7).abs() < 1e-9);
    }

    #[test]
    fn test_concrete_shear_long_term_second_branch() {
        // Fc=60: min(2.0, 1.09) = 1.09
        let fs = concrete_allowable_shear(60.0, true);
        assert!((fs - 1.09).abs() < 1e-9);
    }

    #[test]
    fn test_concrete_shear_short_term_is_1_5x_long() {
        let long = concrete_allowable_shear(24.0, true);
        let short = concrete_allowable_shear(24.0, false);
        assert!((short - long * 1.5).abs() < 1e-9);
    }

    #[test]
    fn test_concrete_compression_short_is_2x_long() {
        let long = concrete_allowable_compression(24.0, true);
        let short = concrete_allowable_compression(24.0, false);
        assert!((long - 8.0).abs() < 1e-9);
        assert!((short - 16.0).abs() < 1e-9);
    }

    #[test]
    fn test_young_ratio_n_table() {
        assert_eq!(young_ratio_n(21.0), 15.0);
        assert_eq!(young_ratio_n(27.0), 15.0);
        assert_eq!(young_ratio_n(30.0), 13.0);
        assert_eq!(young_ratio_n(36.0), 13.0);
        assert_eq!(young_ratio_n(40.0), 11.0);
        assert_eq!(young_ratio_n(48.0), 11.0);
        assert_eq!(young_ratio_n(50.0), 9.0);
        assert_eq!(young_ratio_n(60.0), 9.0);
        assert_eq!(young_ratio_n(90.0), 7.0);
    }

    #[test]
    fn test_concrete_young_modulus_plausible() {
        // Fc=21, γ=23 で AIJ 表の目安値（約 2.0〜2.3 × 10^4 N/mm²）に近い。
        let ec = concrete_young_modulus(21.0, Some(23.0));
        assert!(ec > 20000.0 && ec < 23000.0, "Ec={ec}");
    }

    #[test]
    fn test_rebar_allowable_tension_sd345_d25_vs_d29() {
        assert!((rebar_allowable_tension("SD345", 25.0, true) - 215.0).abs() < 1e-9);
        assert!((rebar_allowable_tension("SD345", 29.0, true) - 195.0).abs() < 1e-9);
        assert!((rebar_allowable_tension("SD345", 25.0, false) - 345.0).abs() < 1e-9);
    }

    #[test]
    fn test_rebar_allowable_tension_table() {
        assert!((rebar_allowable_tension("SR235", 16.0, true) - 155.0).abs() < 1e-9);
        assert!((rebar_allowable_tension("SR235", 16.0, false) - 235.0).abs() < 1e-9);
        assert!((rebar_allowable_tension("SR295", 16.0, true) - 155.0).abs() < 1e-9);
        assert!((rebar_allowable_tension("SR295", 16.0, false) - 295.0).abs() < 1e-9);
        assert!((rebar_allowable_tension("SD295A", 16.0, true) - 195.0).abs() < 1e-9);
        assert!((rebar_allowable_tension("SD390", 22.0, true) - 215.0).abs() < 1e-9);
        assert!((rebar_allowable_tension("SD390", 32.0, true) - 195.0).abs() < 1e-9);
        assert!((rebar_allowable_tension("SD390", 22.0, false) - 390.0).abs() < 1e-9);
        assert!((rebar_allowable_tension("SD490", 22.0, false) - 490.0).abs() < 1e-9);
        assert!((rebar_allowable_tension("UNKNOWN", 22.0, true) - 195.0).abs() < 1e-9);
        assert!((rebar_allowable_tension("UNKNOWN", 22.0, false) - 295.0).abs() < 1e-9);
    }

    #[test]
    fn test_rebar_allowable_shear_table() {
        assert!((rebar_allowable_shear("SR235", true) - 155.0).abs() < 1e-9);
        assert!((rebar_allowable_shear("SD345", true) - 195.0).abs() < 1e-9);
        assert!((rebar_allowable_shear("SD295A", false) - 295.0).abs() < 1e-9);
        assert!((rebar_allowable_shear("SD345", false) - 345.0).abs() < 1e-9);
        assert!((rebar_allowable_shear("SD390", false) - 390.0).abs() < 1e-9);
        assert!((rebar_allowable_shear("SD490", false) - 390.0).abs() < 1e-9);
        assert!((rebar_allowable_shear("UNKNOWN", false) - 295.0).abs() < 1e-9);
    }

    // ------------------------------------------------------------------
    // dt（引張筋重心）
    // ------------------------------------------------------------------

    #[test]
    fn test_tension_dt_single_layer() {
        let bar = BarSet {
            count: 4,
            dia: 22.0,
            layers: 1,
        };
        let dt = tension_dt(40.0, 10.0, &bar);
        assert!((dt - (40.0 + 10.0 + 11.0)).abs() < 1e-9);
    }

    #[test]
    fn test_tension_dt_two_layers() {
        let bar = BarSet {
            count: 8,
            dia: 22.0,
            layers: 2,
        };
        let cover = 40.0;
        let shear_dia = 10.0;
        let k1 = cover + shear_dia + bar.dia / 2.0;
        let k_prime = 25.0_f64.max(1.5 * bar.dia);
        let k2 = k1 + bar.dia / 2.0 + k_prime + bar.dia / 2.0;
        let expected = (k1 + k2) / 2.0;
        let dt = tension_dt(cover, shear_dia, &bar);
        assert!((dt - expected).abs() < 1e-6);
    }

    // ------------------------------------------------------------------
    // 梁の曲げ
    // ------------------------------------------------------------------

    #[test]
    fn test_beam_moment_light_reinforcement_tension_governs() {
        // 軽配筋（1段筋）: MA_t が支配するはず。
        let shape = rc_rect_shape(300.0, 600.0, 4, 19.0, 1, 40.0, 10.0, 100.0, 2);
        let rebar = match &shape {
            SectionShape::RcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let props = rect_axis_props(300.0, 600.0, &rebar.main_x, &rebar);
        let ft = rebar_allowable_tension("SD345", 19.0, true);
        let fc = concrete_allowable_compression(24.0, true);
        let n_ratio = young_ratio_n(24.0);
        let bm = beam_moment_capacity(&props, ft, fc, n_ratio);

        let expected_ma_t = props.at * ft * props.j;
        assert!((bm.ma_t - expected_ma_t).abs() < 1e-6);
        assert!(bm.ma_t <= bm.ma_c, "軽配筋では MA_t が支配するはず");
        assert!((bm.ma - bm.ma_t).abs() < 1e-6);
    }

    #[test]
    fn test_beam_moment_heavy_reinforcement_compression_governs() {
        // 過大配筋（多段・多本数）: MA_c が MA_t を下回り支配するはず。
        let shape = rc_rect_shape(300.0, 600.0, 20, 32.0, 4, 40.0, 10.0, 100.0, 2);
        let rebar = match &shape {
            SectionShape::RcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let props = rect_axis_props(300.0, 600.0, &rebar.main_x, &rebar);
        let ft = rebar_allowable_tension("SD345", 32.0, true);
        let fc = concrete_allowable_compression(24.0, true);
        let n_ratio = young_ratio_n(24.0);
        let bm = beam_moment_capacity(&props, ft, fc, n_ratio);

        assert!(bm.ma_c < bm.ma_t, "過大配筋では MA_c が支配するはず");
        assert!((bm.ma - bm.ma_c).abs() < 1e-6);
    }

    #[test]
    fn test_beam_check_via_design_check_trait() {
        let shape = rc_rect_shape(300.0, 600.0, 4, 19.0, 1, 40.0, 10.0, 100.0, 2);
        let sec = make_section(shape);
        let mat = make_material(24.0, "SD345");
        let ctx = ctx_beam(LoadTerm::Long);
        let forces = MemberForcesAt {
            pos: 0.0,
            n: 0.0,
            qy: 20_000.0,
            qz: 0.0,
            my: 0.0,
            mz: 30_000_000.0,
        };
        let design = RcDesign;
        let result = design.check(&forces, &sec, &mat, &ctx);
        assert!(result.ratio > 0.0);
        assert!(result.basis.contains("13条"));
    }

    // ------------------------------------------------------------------
    // 梁のせん断
    // ------------------------------------------------------------------

    #[test]
    fn test_shear_alpha_clamp_at_upper_bound() {
        // M/(Q・d) = 1 -> α = 4/2 = 2.0（上限に一致）
        let d = 500.0;
        let q = 100_000.0;
        let m = q * d * 1.0;
        let alpha = shear_alpha(m, q, d, 2.0);
        assert!((alpha - 2.0).abs() < 1e-9);
    }

    #[test]
    fn test_shear_alpha_clamp_at_lower_bound() {
        // M/(Q・d) = 3 -> α = 4/4 = 1.0（下限に一致）
        let d = 500.0;
        let q = 100_000.0;
        let m = q * d * 3.0;
        let alpha = shear_alpha(m, q, d, 2.0);
        assert!((alpha - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_shear_alpha_clamp_engages_beyond_bounds() {
        let d = 500.0;
        let q = 100_000.0;
        // M/(Q・d)=0 -> 素の α=4.0 は上限 2.0 にクランプされる。
        let alpha_hi = shear_alpha(0.0, q, d, 2.0);
        assert!((alpha_hi - 2.0).abs() < 1e-9);
        // M/(Q・d)=10 -> 素の α=4/11≈0.364 は下限 1.0 にクランプされる。
        let m = q * d * 10.0;
        let alpha_lo = shear_alpha(m, q, d, 2.0);
        assert!((alpha_lo - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_shear_alpha_intermediate_value() {
        // M/(Q・d) = 1 と 3 の中間、M/(Q・d)=2 -> α = 4/3 ≈ 1.333
        let d = 500.0;
        let q = 100_000.0;
        let m = q * d * 2.0;
        let alpha = shear_alpha(m, q, d, 2.0);
        assert!((alpha - 4.0 / 3.0).abs() < 1e-9);
    }

    #[test]
    fn test_pw_ratio_capped_long_term() {
        // 過大なせん断補強筋比を作り、長期は 0.6% に制限されることを確認する。
        let shape = rc_rect_shape(300.0, 600.0, 4, 19.0, 1, 40.0, 13.0, 30.0, 4);
        let rebar = match &shape {
            SectionShape::RcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let props = rect_axis_props(300.0, 600.0, &rebar.main_x, &rebar);
        assert!(props.pw > 0.006, "テストの前提として pw > 0.6% が必要");

        let allow = rc_allow(24.0, "SD345", true);
        let alpha = 1.5;
        let qa_capped = shear_capacity(&props, &allow, alpha, LoadTerm::Long, true, false);

        // 手計算: pw を 0.6% に制限した式と一致すること。
        let pw_term = 0.5 * allow.w_ft * (0.006 - 0.002);
        let expected = props.b * props.j * (alpha * allow.fs + pw_term);
        assert!((qa_capped - expected).abs() / expected < 1e-6);
    }

    #[test]
    fn test_beam_shear_damage_control_vs_safety() {
        let shape = rc_rect_shape(300.0, 600.0, 4, 19.0, 1, 40.0, 10.0, 100.0, 2);
        let rebar = match &shape {
            SectionShape::RcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let props = rect_axis_props(300.0, 600.0, &rebar.main_x, &rebar);
        let allow = rc_allow(24.0, "SD345", false);
        let alpha = 1.4;

        let qa_damage = shear_capacity(&props, &allow, alpha, LoadTerm::Short, true, false);
        let qa_safety = shear_capacity(&props, &allow, alpha, LoadTerm::Short, false, false);

        let pw_term = if props.pw < 0.002 {
            0.0
        } else {
            0.5 * allow.w_ft * (props.pw.min(0.012) - 0.002)
        };
        let expected_damage = props.b * props.j * ((2.0 / 3.0) * alpha * allow.fs + pw_term);
        let expected_safety = props.b * props.j * (alpha * allow.fs + pw_term);

        assert!((qa_damage - expected_damage).abs() / expected_damage < 1e-6);
        assert!((qa_safety - expected_safety).abs() / expected_safety < 1e-6);
        assert!(
            qa_damage < qa_safety,
            "損傷制御式は安全確保式より小さいはず"
        );
    }

    // ------------------------------------------------------------------
    // 柱: 軸力・軸力+曲げ
    // ------------------------------------------------------------------

    #[test]
    fn test_column_axial_capacity_handcalc() {
        let fc = 8.0; // 長期許容圧縮（Fc=24 なら 8.0）
        let ft = 215.0;
        let n_ratio = 15.0;
        let gross = 400.0 * 400.0;
        let as_total = 8.0 * std::f64::consts::PI * (22.0 / 2.0f64).powi(2);
        let na = column_axial_capacity(gross, as_total, fc, ft, n_ratio);

        let ae = gross + (n_ratio - 1.0) * as_total;
        let expected = (fc * ae).min(ft * ae / n_ratio);
        assert!((na - expected).abs() < 1e-6);
    }

    #[test]
    fn test_column_n0_moment_close_to_beam_ma_t() {
        // N=0 のときの柱 MA が、対応する梁の MA_t とおおむね一致すること
        // （j≒7d/8 の近似差程度、20% 程度の許容）を確認する。
        let b = 400.0;
        let d_full = 400.0;
        let shape = rc_rect_shape(b, d_full, 8, 22.0, 2, 40.0, 10.0, 100.0, 2);
        let rebar = match &shape {
            SectionShape::RcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let sec = make_section(shape);

        let allow = rc_allow(24.0, "SD345", true);
        let ft = rebar_allowable_tension("SD345", 22.0, true);

        let props_z = rect_axis_props_strong(&sec, &rebar);
        let bm = beam_moment_capacity(&props_z, ft, allow.fc, allow.n_ratio);

        let gross_area = sec.width * sec.depth;
        let as_total = bar_set_area(&rebar.main_x) + bar_set_area(&rebar.main_y);
        let na = column_axial_capacity(gross_area, as_total, allow.fc, ft, allow.n_ratio);

        let axis_z = ColumnAxis {
            props: props_z,
            at_perp: bar_set_area(&rebar.main_y),
            ft,
        };
        let curve = column_nm_curve(&axis_z, &allow, na);
        let ma_at_n0 = interp_ma(&curve, 0.0);

        let rel_diff = (ma_at_n0 - bm.ma).abs() / bm.ma;
        assert!(
            rel_diff < 0.2,
            "N=0 の柱 MA={ma_at_n0} が梁 MA={} と 20% 以上乖離",
            bm.ma
        );
    }

    #[test]
    fn test_column_moment_increases_then_decreases_with_compression() {
        // 軽配筋（N=0 では引張鉄筋支配）の断面を用いる。RC 規準14条の N-M
        // 相関曲線は一般に「引張支配の隅（大きな引張軸力・小さな M）→
        // 釣合点（M最大）→ 全断面圧縮の隅（N=NA, M=0）」という山型になる。
        // 釣合点（ピーク）の位置は配筋量に依存し、既に N=0 でコンクリート縁
        // 応力が支配する（過大配筋の）断面ではピークが引張側にずれることも
        // あるため、ここではピークが正の圧縮軸力側に来る軽配筋断面で検証する
        // （`test_beam_moment_heavy_reinforcement_compression_governs` が過大
        // 配筋側の挙動を別途カバーする）。
        let b = 400.0;
        let d_full = 400.0;
        let shape = rc_rect_shape(b, d_full, 4, 19.0, 1, 40.0, 10.0, 100.0, 2);
        let rebar = match &shape {
            SectionShape::RcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let sec = make_section(shape);

        let allow = rc_allow(24.0, "SD345", true);
        let ft = rebar_allowable_tension("SD345", 19.0, true);
        let props_z = rect_axis_props_strong(&sec, &rebar);
        let gross_area = sec.width * sec.depth;
        let as_total = bar_set_area(&rebar.main_x) + bar_set_area(&rebar.main_y);
        let na = column_axial_capacity(gross_area, as_total, allow.fc, ft, allow.n_ratio);

        let axis_z = ColumnAxis {
            props: props_z,
            at_perp: bar_set_area(&rebar.main_y),
            ft,
        };
        let curve = column_nm_curve(&axis_z, &allow, na);

        let m_at_0 = interp_ma(&curve, 0.0);
        let m_at_mid = interp_ma(&curve, na * 0.3);
        let m_at_near_na = interp_ma(&curve, na * 0.98);

        assert!(
            m_at_mid > m_at_0,
            "圧縮軸力の増加で MA は一旦増加するはず: m0={m_at_0}, mid={m_at_mid}"
        );
        assert!(
            m_at_near_na < m_at_mid,
            "軸力が NA に近づくと MA は減少するはず: mid={m_at_mid}, near_na={m_at_near_na}"
        );
    }

    #[test]
    fn test_column_biaxial_linear_sum() {
        let b = 400.0;
        let d_full = 400.0;
        let shape = rc_rect_shape(b, d_full, 8, 22.0, 2, 40.0, 10.0, 100.0, 2);
        let sec = make_section(shape);
        let mat = make_material(24.0, "SD345");
        let ctx = ctx_column(LoadTerm::Long);

        // まず微小な mz を与えて ratio から MA_z を逆算する。
        let forces_z_only = MemberForcesAt {
            pos: 0.0,
            n: 0.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 1.0,
        };
        let design = RcDesign;
        let r0 = design.check(&forces_z_only, &sec, &mat, &ctx);
        let ma_z_approx = 1.0 / r0.ratio.max(1e-30);

        let mz_test = ma_z_approx * 0.3;
        let forces = MemberForcesAt {
            pos: 0.0,
            n: 0.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: mz_test,
        };
        let r = design.check(&forces, &sec, &mat, &ctx);
        assert!(
            (r.ratio - 0.3).abs() < 0.05,
            "mz 単独 0.3 割合のとき ratio ≒ 0.3 のはず: ratio={}",
            r.ratio
        );
    }

    #[test]
    fn test_column_shear_alpha_upper_bound_1_5() {
        let d = 400.0;
        let q = 50_000.0;
        // M/(Q・d)=0 -> 素の α=4.0 は柱の上限 1.5 にクランプされる。
        let alpha = shear_alpha(0.0, q, d, 1.5);
        assert!((alpha - 1.5).abs() < 1e-9);
    }

    #[test]
    fn test_column_safety_check_excludes_alpha() {
        let shape = rc_rect_shape(400.0, 400.0, 8, 22.0, 2, 40.0, 10.0, 100.0, 2);
        let rebar = match &shape {
            SectionShape::RcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let props = rect_axis_props_strong(&make_section(shape), &rebar);
        let allow = rc_allow(24.0, "SD345", false);

        let qa_alpha_1 = shear_capacity(&props, &allow, 1.0, LoadTerm::Short, false, true);
        let qa_alpha_1_5 = shear_capacity(&props, &allow, 1.5, LoadTerm::Short, false, true);
        // 柱の「安全確保のための検討」式は α を含まないため、α を変えても
        // QA は変化しない。
        assert!((qa_alpha_1 - qa_alpha_1_5).abs() < 1e-6);

        // 損傷制御式は α に依存するため異なる値になる。
        let qa_damage_1 = shear_capacity(&props, &allow, 1.0, LoadTerm::Short, true, true);
        let qa_damage_1_5 = shear_capacity(&props, &allow, 1.5, LoadTerm::Short, true, true);
        assert!((qa_damage_1 - qa_damage_1_5).abs() > 1e-6);
    }

    #[test]
    fn test_column_long_term_shear_has_no_rebar_term() {
        let shape = rc_rect_shape(400.0, 400.0, 8, 22.0, 2, 40.0, 10.0, 60.0, 4);
        let rebar = match &shape {
            SectionShape::RcRect { rebar, .. } => rebar.clone(),
            _ => unreachable!(),
        };
        let props = rect_axis_props_strong(&make_section(shape), &rebar);
        let allow = rc_allow(24.0, "SD345", true);
        let alpha = 1.3;
        let qal = shear_capacity(&props, &allow, alpha, LoadTerm::Long, true, true);
        let expected = props.b * props.j * alpha * allow.fs;
        assert!((qal - expected).abs() / expected < 1e-9);
    }

    // ------------------------------------------------------------------
    // フォールバック
    // ------------------------------------------------------------------

    #[test]
    fn test_fc_missing_fallback() {
        let shape = rc_rect_shape(300.0, 600.0, 4, 19.0, 1, 40.0, 10.0, 100.0, 2);
        let sec = make_section(shape);
        let mat = Material {
            id: MaterialId(0),
            name: "SD345".to_string(),
            young: 205000.0,
            poisson: 0.3,
            density: 0.0,
            shear: None,
            fc: None,
            fy: None,
        };
        let ctx = ctx_beam(LoadTerm::Long);
        let forces = MemberForcesAt {
            pos: 0.0,
            n: 0.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 0.0,
        };
        let design = RcDesign;
        let result = design.check(&forces, &sec, &mat, &ctx);
        assert!(result.ok);
        assert_eq!(result.ratio, 0.0);
        assert!(result.basis.contains("Fc"));
    }

    #[test]
    fn test_shape_missing_fallback() {
        // shape を持たない Section（数値直入力等）。
        let sec = Section {
            id: SectionId(0),
            name: "no-shape".to_string(),
            area: 300.0 * 600.0,
            iy: 1.0,
            iz: 1.0,
            j: 1.0,
            depth: 600.0,
            width: 300.0,
            as_y: 0.0,
            as_z: 0.0,
            panel_thickness: None,
            thickness: None,
            shape: None,
        };
        let mat = make_material(24.0, "SD345");
        let ctx = ctx_beam(LoadTerm::Long);
        let forces = MemberForcesAt {
            pos: 0.0,
            n: 0.0,
            qy: 0.0,
            qz: 0.0,
            my: 0.0,
            mz: 0.0,
        };
        let design = RcDesign;
        let result = design.check(&forces, &sec, &mat, &ctx);
        assert!(result.ok);
        assert_eq!(result.ratio, 0.0);
        assert!(result.basis.contains("配筋情報なし"));
    }

    #[test]
    fn test_rc_circle_beam_and_column_smoke() {
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
                    dia: 10.0,
                    pitch: 100.0,
                    legs: 1,
                },
            },
        };
        let sec = make_section(shape);
        let mat = make_material(24.0, "SD345");
        let design = RcDesign;

        let forces = MemberForcesAt {
            pos: 0.0,
            n: -200_000.0,
            qy: 30_000.0,
            qz: 20_000.0,
            my: 10_000_000.0,
            mz: 20_000_000.0,
        };

        let ctx_col = ctx_column(LoadTerm::Short);
        let r_col = design.check(&forces, &sec, &mat, &ctx_col);
        assert!(r_col.ratio.is_finite() && r_col.ratio >= 0.0);
        assert!(r_col.basis.contains("円形柱"));

        let ctx_b = ctx_beam(LoadTerm::Short);
        let r_beam = design.check(&forces, &sec, &mat, &ctx_b);
        assert!(r_beam.ratio.is_finite() && r_beam.ratio >= 0.0);
    }
}
