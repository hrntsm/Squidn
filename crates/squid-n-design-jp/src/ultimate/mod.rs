//! **終局検定（RESP-D マニュアル「計算編 06 終局検定」）**。
//!
//! 非線形解析（荷重増分解析）で崩壊機構が形成された後、各部材が終局せん断強度・
//! 付着割裂耐力に対して十分な余裕（せん断・付着が曲げに先行して破壊しないこと）を
//! 持つかを検定する。RESP-D で「終局強度型設計指針」を選択した場合の
//! 塑性理論式（[`rc_shear`]）と、柱の軸終局耐力（[`rc_axial`]）を実装する。
//!
//! # 検定の考え方（RESP-D「06 終局検定」採用応力・RC 柱 b) 余裕度）
//! - 両端ヒンジを仮定した終局せん断応力 `Qmu = 上限強度倍率·(Mu上+Mu下)/内法` を
//!   設計用せん断力とし、終局せん断強度 `Qsu`（塑性理論式）・付着割裂耐力 `Qbu`
//!   との比（余裕度 `Qsu/Qmu`, `Qbu/Qmu`）を算定する。
//! - 余裕度 ≥ 1.0（せん断・付着が曲げ降伏に先行しない）で OK。
//!
//! # 曲げ終局強度 Mu
//! 曲げ終局強度は既存の [`squid_n_core::rc_capacity`]（構造規定 at 式）を再利用する
//! （梁は [`squid_n_core::rc_capacity::rc_mu_simple`]、柱は軸力を考慮した
//! [`squid_n_core::rc_capacity::rc_column_mu_simple`]）。RESP-D は柱について ACI
//! 規準（平面保持）も選択できるが、本モジュールは構造規定 at 式を用いる。
//!
//! # 適用範囲・簡略化（doc 兼申し送り）
//! - 対象は `SectionShape::RcRect`（矩形 RC 断面）のみ。円形柱・SRC・CFT・鋼は
//!   別途（本モジュールの対象外）。
//! - 強軸（せい方向主筋 main_x）まわりのせん断・付着を検定する（二軸せん断余裕度・
//!   ACI 規準による曲げ・靭性指針式 Vu は今後の課題）。
//! - 主筋は上下対称配筋を仮定し、引張側主筋量は main_x の総断面積の半分とする。

use crate::MemberKind;
use squid_n_core::ids::ElemId;
use squid_n_core::model::{ElementData, Material, Model, Section};
use squid_n_core::rc_capacity::{rc_column_mu_simple, rc_mu_simple, RcCapacityInput};
use squid_n_core::section_shape::{BarSet, RcRebar, SectionShape};

pub mod joint;
pub mod rc_axial;
pub mod rc_shear;

pub use joint::{
    joint_fj, joint_kappa, rc_joint_ultimate, RcJointUltimateInput, RcJointUltimateResult,
};
pub use rc_axial::{rc_axial_margin, rc_column_axial_ultimate, RcAxialUltimate};
pub use rc_shear::{
    bond_reliable_strength_deformed, bond_split_ratio, plastic_cot_phi, plastic_k1, plastic_k2,
    plastic_nu, plastic_nu0, rc_shear_qbu_bond, rc_shear_qsu_plastic, BondStrengthInput,
    RcBondSplitInput, RcPlasticShearInput,
};

/// 終局検定（塑性理論式）の算定オプション。
#[derive(Clone, Copy, Debug)]
pub struct UltimateShearOptions {
    /// 終局限界状態でのヒンジ領域の回転角 Rp [rad]（ν・cotφ に用いる。既定 0）。
    pub rp: f64,
    /// 軽量コンクリートを使用する場合 true（せん断終局耐力を 0.9 倍に低減）。
    pub lightweight: bool,
    /// 上限強度倍率（Qmu = 上限強度倍率·(Mu上+Mu下)/内法。既定 1.0）。
    pub upper_strength_factor: f64,
    /// せん断補強筋の降伏強度算定用強度 σwy [N/mm²]（モデルに材質情報が無い場合の
    /// 代表値。既定 295 = SD295 相当）。
    pub sigma_wy: f64,
    /// 付着割裂の検定を含める場合 true。
    pub include_bond: bool,
}

impl Default for UltimateShearOptions {
    fn default() -> Self {
        Self {
            rp: 0.0,
            lightweight: false,
            upper_strength_factor: 1.0,
            sigma_wy: 295.0,
            include_bond: true,
        }
    }
}

/// 1 部材分の終局検定結果。
#[derive(Clone, Debug)]
pub struct UltimateCheck {
    /// 部材 ID。
    pub elem: ElemId,
    /// 部材種別（梁/柱）。
    pub kind: MemberKind,
    /// 曲げ終局強度 Mu [N·mm]。
    pub mu: f64,
    /// 両端ヒンジ時せん断力 Qmu = 上限強度倍率·2·Mu/内法 [N]。
    pub qmu: f64,
    /// 塑性理論式による終局せん断強度 Qsu [N]。
    pub qsu: f64,
    /// 付着割裂による終局せん断耐力 Qbu [N]（`include_bond=false` なら 0）。
    pub qbu: f64,
    /// せん断余裕度 Qsu/Qmu。
    pub shear_margin: f64,
    /// 付着余裕度 Qbu/Qmu（`include_bond=false` なら `f64::INFINITY`）。
    pub bond_margin: f64,
    /// 軸終局耐力（柱のみ Some）。
    pub axial: Option<RcAxialUltimate>,
    /// 判定（せん断余裕度・付着余裕度が共に 1.0 以上で true）。
    pub ok: bool,
    /// 根拠（表示用）。
    pub basis: String,
    /// 詳細（表示用）。
    pub detail: String,
}

/// 主筋セットの総断面積 [mm²]。
fn bar_set_area(bar: &BarSet) -> f64 {
    bar.count as f64 * std::f64::consts::PI / 4.0 * bar.dia * bar.dia
}

/// せん断補強筋比 pw = (legs·π/4·dia²)/(b·pitch)。pitch ≤ 0 なら 0。
fn hoop_pw(rebar: &RcRebar, b: f64) -> f64 {
    if rebar.shear.pitch <= 0.0 || b <= 0.0 {
        return 0.0;
    }
    let aw =
        rebar.shear.legs as f64 * std::f64::consts::PI / 4.0 * rebar.shear.dia * rebar.shear.dia;
    aw / (b * rebar.shear.pitch)
}

/// 部材軸の鉛直成分 |ez| から部材種別を判定する（app の `member_kind_of` と同規則）。
fn member_kind(elem: &ElementData, model: &Model) -> MemberKind {
    let coords: Vec<[f64; 3]> = elem
        .nodes
        .iter()
        .filter_map(|nid| model.nodes.get(nid.index()))
        .map(|n| n.coord)
        .take(2)
        .collect();
    let (Some(p0), Some(p1)) = (coords.first(), coords.get(1)) else {
        return MemberKind::Beam;
    };
    let d = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
    let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    if len < 1e-9 {
        return MemberKind::Beam;
    }
    let ez = (d[2] / len).abs();
    if ez >= 0.8 {
        MemberKind::Column
    } else if ez <= 0.2 {
        MemberKind::Beam
    } else {
        MemberKind::Brace
    }
}

/// 部材両端節点間の幾何長 [mm]。
fn geometric_length(elem: &ElementData, model: &Model) -> f64 {
    let coords: Vec<[f64; 3]> = elem
        .nodes
        .iter()
        .filter_map(|nid| model.nodes.get(nid.index()))
        .map(|n| n.coord)
        .take(2)
        .collect();
    let (Some(p0), Some(p1)) = (coords.first(), coords.get(1)) else {
        return 0.0;
    };
    let d = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}

/// 内法長さ [mm] = 幾何長 − 両端フェイス距離。フェイス合計が幾何長以上の
/// 不整合入力では幾何長のままとする（app の rank-auto と同規則）。
fn clear_span(elem: &ElementData, model: &Model) -> f64 {
    let geom = geometric_length(elem, model);
    let face_sum = elem.rigid_zone.face_i + elem.rigid_zone.face_j;
    if geom - face_sum > 0.0 {
        geom - face_sum
    } else {
        geom
    }
}

/// 1 部材の終局検定を実行する（`RcRect` 以外・Fc 未設定は `None`）。
fn check_member(
    elem: &ElementData,
    sec: &Section,
    mat: &Material,
    model: &Model,
    n_axial: f64,
    opts: &UltimateShearOptions,
) -> Option<UltimateCheck> {
    let SectionShape::RcRect { b, d, rebar } = sec.shape.as_ref()? else {
        return None;
    };
    let (b, d) = (*b, *d);
    let fc = mat.fc?;
    if fc <= 0.0 || b <= 0.0 || d <= 0.0 {
        return None;
    }
    let kind = member_kind(elem, model);
    let sigma_y = mat.fy.unwrap_or(345.0);
    let l_clear = clear_span(elem, model);

    // 断面諸元（強軸＝せい方向主筋 main_x）。
    let dt = rebar.cover + rebar.shear.dia + rebar.main_x.dia / 2.0;
    let d_eff = d - dt;
    if d_eff <= 0.0 {
        return None;
    }
    let jt = 7.0 * d_eff / 8.0;
    let at = bar_set_area(&rebar.main_x) / 2.0;
    let ag = bar_set_area(&rebar.main_x) + bar_set_area(&rebar.main_y);
    let pw = hoop_pw(rebar, b);

    // 曲げ終局強度 Mu（柱は軸力考慮、梁は軸力なし）。
    let cap = RcCapacityInput {
        b,
        d,
        at,
        d_eff,
        sigma_y,
        fc,
        pw,
        sigma_wy: opts.sigma_wy,
        clear_span: l_clear.max(1.0),
        sigma_0: 0.0,
    };
    let mu = match kind {
        MemberKind::Column => rc_column_mu_simple(&cap, ag, n_axial),
        _ => rc_mu_simple(&cap),
    };

    // 両端ヒンジ時せん断力 Qmu = 上限強度倍率·2·Mu/内法。
    let qmu = if l_clear > 0.0 {
        opts.upper_strength_factor * 2.0 * mu / l_clear
    } else {
        0.0
    };

    // 終局せん断強度 Qsu（塑性理論式）。
    let qsu = rc_shear_qsu_plastic(&RcPlasticShearInput {
        b,
        d_full: d,
        jt,
        pw,
        sigma_wy: opts.sigma_wy,
        l_clear,
        fc,
        rp: opts.rp,
        lightweight: opts.lightweight,
    });

    // 付着割裂耐力 Qbu。
    let (qbu, tau_bu) = if opts.include_bond {
        // 引張側主筋本数（対称配筋の半分、外側一列を代表）。
        let n_tension = (rebar.main_x.count as f64 / 2.0).max(1.0);
        let tau_bu = bond_reliable_strength_deformed(&BondStrengthInput {
            fc,
            b,
            db1: rebar.main_x.dia,
            n_bars: n_tension.round() as u32,
            cover_side: rebar.cover,
            cover_bottom: rebar.cover,
            hoop_area: rebar.shear.legs as f64 * std::f64::consts::PI / 4.0
                * rebar.shear.dia
                * rebar.shear.dia,
            hoop_pitch: rebar.shear.pitch,
            pw,
            top_bar: false,
        });
        let sum_phi = n_tension * std::f64::consts::PI * rebar.main_x.dia;
        let qbu = rc_shear_qbu_bond(&RcBondSplitInput {
            b,
            d_full: d,
            jt,
            tau_bu,
            sum_phi,
            l_clear,
            fc,
            rp: opts.rp,
            lightweight: opts.lightweight,
        });
        (qbu, tau_bu)
    } else {
        (0.0, 0.0)
    };

    let shear_margin = if qmu > 0.0 { qsu / qmu } else { f64::INFINITY };
    let bond_margin = if !opts.include_bond {
        f64::INFINITY
    } else if qmu > 0.0 {
        qbu / qmu
    } else {
        f64::INFINITY
    };

    let axial = if kind == MemberKind::Column {
        Some(rc_column_axial_ultimate(b, d, fc, ag, sigma_y))
    } else {
        None
    };

    let ok = shear_margin >= 1.0 && bond_margin >= 1.0;

    let basis = match kind {
        MemberKind::Column => "RC柱 終局検定（塑性理論式 Qsu/Qbu）".to_string(),
        _ => "RC梁 終局検定（塑性理論式 Qsu/Qbu）".to_string(),
    };
    let detail = format!(
        "Mu={:.0} N·mm, Qmu={:.0} N, Qsu={:.0} N, Qbu={:.0} N, τbu={:.3} N/mm², \
         Qsu/Qmu={:.3}, Qbu/Qmu={:.3}, pw={:.5}, jt={:.1} mm, L={:.0} mm, Rp={:.4}",
        mu, qmu, qsu, qbu, tau_bu, shear_margin, bond_margin, pw, jt, l_clear, opts.rp
    );

    Some(UltimateCheck {
        elem: elem.id,
        kind,
        mu,
        qmu,
        qsu,
        qbu,
        shear_margin,
        bond_margin,
        axial,
        ok,
        basis,
        detail,
    })
}

/// モデルの RC 矩形部材について終局検定（塑性理論式）を一括実行する。
///
/// - `axial_by_elem`: 部材の設計軸力 [N]（**圧縮正**）。柱の曲げ終局強度 Mu・
///   軸余裕度に用いる。該当 ID が無い部材は軸力 0（安全側）で評価する。
///   通常は長期（G+P）静的解析の軸力を渡す。
/// - 対象外（`RcRect` 以外・断面/材料未解決・Fc 未設定・有効せい ≤ 0）の部材は
///   結果に含めない。
pub fn collect_rc_ultimate_checks(
    model: &Model,
    axial_by_elem: &[(ElemId, f64)],
    opts: &UltimateShearOptions,
) -> Vec<UltimateCheck> {
    let mut out = Vec::new();
    for elem in &model.elements {
        let Some(sec) = elem.section.and_then(|sid| model.sections.get(sid.index())) else {
            continue;
        };
        let Some(mat) = elem
            .material
            .and_then(|mid| model.materials.get(mid.index()))
        else {
            continue;
        };
        let n_axial = axial_by_elem
            .iter()
            .find(|(id, _)| *id == elem.id)
            .map(|(_, n)| *n)
            .unwrap_or(0.0);
        if let Some(check) = check_member(elem, sec, mat, model, n_axial, opts) {
            out.push(check);
        }
    }
    out
}

#[cfg(test)]
mod tests;
