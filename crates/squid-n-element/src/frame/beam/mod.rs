use crate::behavior::{Ctx, ElemState, ElementBehavior, LocalMat, LocalVec, MassOption};
use crate::transform::LocalFrame;
use smallvec::SmallVec;
use squid_n_core::dof::{DofMap, DOF_PER_NODE};
use squid_n_core::ids::{ElemId, NodeId};
use squid_n_core::model::{EndCondition, Material, Model, RigidZone, Section};

mod rigid_zone;
pub use rigid_zone::{
    apply_auto_rigid_zones, auto_rigid_zones, recompute_auto_zones, RigidZoneRule,
};

#[derive(Clone, Debug)]
pub struct MemberForces {
    pub at: Vec<(f64, [f64; 6])>,
}

#[derive(Clone)]
pub struct BeamElement {
    pub id: ElemId,
    pub e: f64,
    pub g: f64,
    /// 軸剛性（EA）用断面積。SRC では鉄骨の等価換算断面を累加した値になる。
    pub a: f64,
    /// 質量算定（ρ·A·L）用の幾何断面積。SRC の等価換算で質量が過大に
    /// ならないよう `a` と区別する（RESP-D 計算編 02 の An はあくまで剛性用）。
    pub a_mass: f64,
    pub iy: f64,
    pub iz: f64,
    pub j: f64,
    pub as_y: f64,
    pub as_z: f64,
    pub length: f64,
    pub density: f64,
    pub nodes: [NodeId; 2],
    pub axis: LocalFrame,
    pub rigid: RigidZone,
    pub end_cond: [EndCondition; 2],
    pub eval_sections: Vec<f64>,
    pub section: Option<squid_n_core::ids::SectionId>,
    pub material: Option<squid_n_core::ids::MaterialId>,
    /// 確定変位（線形要素の内力計算用。非線形では ElemState が保持）
    pub committed_disp: [f64; 12],
}

fn get_section(model: &Model, sid: Option<squid_n_core::ids::SectionId>) -> Section {
    sid.and_then(|s| {
        if s.index() < model.sections.len() {
            let sec = &model.sections[s.index()];
            if sec.id == s {
                Some(sec.clone())
            } else {
                None
            }
        } else {
            None
        }
    })
    .unwrap_or_else(|| Section {
        id: squid_n_core::ids::SectionId(0),
        name: String::new(),
        area: 0.0,
        iy: 0.0,
        iz: 0.0,
        j: 0.0,
        depth: 0.0,
        width: 0.0,
        as_y: 0.0,
        as_z: 0.0,
        panel_thickness: None,
        thickness: None,
        shape: None,
    })
}

fn get_material(model: &Model, mid: Option<squid_n_core::ids::MaterialId>) -> Material {
    mid.and_then(|m| {
        if m.index() < model.materials.len() {
            let mat = &model.materials[m.index()];
            if mat.id == m {
                Some(mat.clone())
            } else {
                None
            }
        } else {
            None
        }
    })
    .unwrap_or_else(|| Material {
        concrete_class: Default::default(),
        id: squid_n_core::ids::MaterialId(0),
        name: String::new(),
        young: 0.0,
        poisson: 0.0,
        density: 0.0,
        shear: None,
        fc: None,
        fy: None,
    })
}

/// スラブ協力幅による強軸曲げ剛性の増大率（RESP-D 計算編 02「RC大梁」。
/// 協力幅は RC 規準 8 条による）。
///
/// 対象は水平な RC 矩形梁のみ。梁の両端節点をともに境界節点に含むスラブを
/// 「梁に取り付く床」とみなし、梁軸から左右のスラブ奥行き a を求めて
/// ba=(0.5−0.6·a/l)·a（a≥l/2 のとき 0.1·l）で協力幅を算定する。
/// スラブ（厚さ t=`Model::slab_thickness`、建物一律・上端は梁上端と同面）を
/// 考慮した中立軸による T 形断面の Ie を元断面 I0=b·D³/12 で除した値を返す。
/// t≤0・非水平・取り付く床なしでは 1.0（増大なし）。
/// 連続梁の λ・吹抜け補正・二重スラブ/片持ちスラブの区別は未対応（v1。
/// docs/v_and_v/剛性計算_RESP-D照合.md 参照）。
fn slab_stiffness_factor(
    model: &Model,
    data: &squid_n_core::model::ElementData,
    b: f64,
    d: f64,
) -> f64 {
    let t = model.slab_thickness;
    if t <= 0.0 || b <= 0.0 || d <= 0.0 || data.nodes.len() < 2 || model.slabs.is_empty() {
        return 1.0;
    }
    let n0 = data.nodes[0];
    let n1 = data.nodes[data.nodes.len() - 1];
    let (Some(node0), Some(node1)) = (model.nodes.get(n0.index()), model.nodes.get(n1.index()))
    else {
        return 1.0;
    };
    let (p0, p1) = (node0.coord, node1.coord);
    let (dx, dy, dz) = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
    let lp = (dx * dx + dy * dy).sqrt();
    // 水平材のみ対象（勾配 5% までは水平とみなす）
    if lp < 1e-9 || dz.abs() > 0.05 * lp {
        return 1.0;
    }
    let l = (lp * lp + dz * dz).sqrt();
    let (ex, ey) = (dx / lp, dy / lp);

    // 梁軸の左右それぞれのスラブ奥行き a（複数スラブは大きい方を採用）
    let mut a_pos: f64 = 0.0;
    let mut a_neg: f64 = 0.0;
    for slab in &model.slabs {
        if !(slab.boundary.contains(&n0) && slab.boundary.contains(&n1)) {
            continue;
        }
        for nid in &slab.boundary {
            let Some(q) = model.nodes.get(nid.index()) else {
                continue;
            };
            // 平面内で梁軸に直交する符号付き距離
            let s = -(q.coord[0] - p0[0]) * ey + (q.coord[1] - p0[1]) * ex;
            a_pos = a_pos.max(s);
            a_neg = a_neg.max(-s);
        }
    }

    // RC 規準 8 条の片側協力幅
    let ba = |a: f64| -> f64 {
        if a <= 0.0 {
            0.0
        } else if a < 0.5 * l {
            (0.5 - 0.6 * a / l) * a
        } else {
            0.1 * l
        }
    };
    let bf = b + ba(a_pos) + ba(a_neg);
    if bf <= b {
        return 1.0;
    }

    // スラブを考慮した中立軸による T 形断面の Ie
    let tf = t.min(d);
    let aw = b * d;
    let af = (bf - b) * tf;
    let g = (aw * d / 2.0 + af * (d - tf / 2.0)) / (aw + af);
    let i0 = b * d.powi(3) / 12.0;
    let ie = i0
        + aw * (g - d / 2.0).powi(2)
        + (bf - b) * tf.powi(3) / 12.0
        + af * (d - tf / 2.0 - g).powi(2);
    (ie / i0).max(1.0)
}

/// 壁エレメントモデルの上下大梁の剛性倍率（RESP-D 計算編 02「壁エレメントモデル
/// (壁エレメントモデル)の剛性」上下大梁の断面性能）。
///
/// 「上下大梁の断面性能: 通常の大梁に対し、倍率を乗じた剛性を採用します。倍率は
/// 剛性計算条件で設定できます。既定値は100倍となります。」
/// 壁エレメントモデルの上下大梁は、壁の剛性を四隅の節点へ正しく伝えるため剛体に
/// 近い扱いとする。剛性計算条件 UI からの倍率変更は将来対応（現状は既定値固定）。
pub const WALL_GIRDER_STIFF_FACTOR: f64 = 100.0;

/// 自部材（両端節点 n0, n1）が壁エレメントモデルの上辺・下辺大梁かどうかを判定する。
///
/// `model.elements` 中に節点数4以上（四隅を持つ）の `ElementKind::Wall` 要素があり、
/// その壁の節点集合に自部材の両端節点がともに含まれていれば、その壁の上辺または
/// 下辺の大梁とみなす（RESP-D 計算編 02「壁エレメントモデルの上下大梁」）。
///
/// ただし対象は耐震壁が成立した壁のみ（`misc_wall::wall_is_seismic`）。不成立の
/// フレーム内雑壁の上下梁は 100 倍せず、代わりに腰壁/垂壁として断面性能へ算入する
/// （`BeamElement::new` 内の雑壁算入処理）。
fn is_wall_top_bottom_girder(model: &Model, n0: NodeId, n1: NodeId) -> bool {
    model.elements.iter().any(|e| {
        matches!(e.kind, squid_n_core::model::ElementKind::Wall)
            && e.nodes.len() >= 4
            && e.nodes.contains(&n0)
            && e.nodes.contains(&n1)
            && crate::misc_wall::wall_is_seismic(e, model)
    })
}

impl BeamElement {
    pub fn new(data: &squid_n_core::model::ElementData, model: &Model) -> Self {
        let n0 = data.nodes[0];
        let n1 = data.nodes[1];
        let p0 = if n0.index() < model.nodes.len() {
            model.nodes[n0.index()].coord
        } else {
            [0.0; 3]
        };
        let p1 = if n1.index() < model.nodes.len() {
            model.nodes[n1.index()].coord
        } else {
            [0.0; 3]
        };
        let dx = p1[0] - p0[0];
        let dy = p1[1] - p0[1];
        let dz = p1[2] - p0[2];
        let len = (dx * dx + dy * dy + dz * dz).sqrt();

        let axis = LocalFrame::from_nodes(p0, p1, data.local_axis.ref_vector);
        let sec = get_section(model, data.section);
        let mat = get_material(model, data.material);
        let g = mat.shear_modulus();

        // 危険断面位置（§6.2.3、既定は柱フェース＝節点から face_i/j）を正規化座標へ変換し、
        // 節点芯 [0.0, 1.0] と部材中央 0.5 に加えて評価断面リストへ含める。
        // face=0（直交材が無い端）では従来どおり [0.0, 0.5, 1.0] と完全一致する。
        let eval_sections = if len > 1e-12 {
            let xi_i = (data.rigid_zone.face_i / len).clamp(0.0, 0.5 - 1e-9);
            let xi_j = (1.0 - data.rigid_zone.face_j / len).clamp(0.5 + 1e-9, 1.0);
            let mut xs = vec![0.0, xi_i, 0.5, xi_j, 1.0];
            xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
            xs.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
            xs
        } else {
            vec![0.0, 0.5, 1.0]
        };

        let as_y = if sec.as_y != 0.0 {
            sec.as_y
        } else {
            squid_n_core::model::rect_shear_area(sec.area)
        };
        let as_z = if sec.as_z != 0.0 {
            sec.as_z
        } else {
            squid_n_core::model::rect_shear_area(sec.area)
        };

        // SRC/CFT の複合換算断面性能（RESP-D 計算編 02「剛性計算」）。
        // 要素材料からヤング係数比を算定して剛性用の断面性能を上書きする。
        // - SRC: 材料=コンクリート（fc あり）のとき ns=Es/Ec で累加。
        // - CFT: 材料=鋼管の young と充填コンクリート強度 fc から 1/n 換算で累加。
        // 算定不能（fc 無し・Ec≤0 等）なら to_section の既定値
        // （SRC: N_S_EQ 固定、CFT: 鋼管のみ）のまま。質量用 a_mass は常に幾何断面。
        use squid_n_core::section_shape::SectionShape;
        let composite = sec.shape.as_ref().and_then(|shape| match shape {
            SectionShape::SrcRect { .. } => mat
                .fc
                .is_some()
                .then(|| shape.src_equivalent_props(mat.young, mat.poisson))
                .flatten(),
            SectionShape::CftBox { .. } | SectionShape::CftPipe { .. } => mat
                .fc
                .and_then(|fc| shape.cft_equivalent_props(mat.young, mat.poisson, fc)),
            _ => None,
        });

        // SRC で材料から算定できない場合も、軸剛性だけは既定 N_S_EQ の累加を維持する。
        let a_stiff = match (&composite, &sec.shape) {
            (Some(p), _) => p.area_ax,
            (None, Some(shape @ SectionShape::SrcRect { .. })) => shape.calc_axial_stiffness_area(),
            _ => sec.area,
        };
        let (iy, iz, j, as_y, as_z) = match &composite {
            Some(p) => (p.iy, p.iz, p.j, p.as_y, p.as_z),
            None => (sec.iy, sec.iz, sec.j, as_y, as_z),
        };

        // スラブ協力幅による強軸剛性増大（RESP-D 計算編 02「RC大梁」）。
        // RC 矩形梁のみ対象。Model::slab_thickness=0（既定）では 1.0 で無効。
        let iy = if matches!(&sec.shape, Some(SectionShape::RcRect { .. })) {
            iy * slab_stiffness_factor(model, data, sec.width, sec.depth)
        } else {
            iy
        };

        // 壁エレメントモデルの上下大梁の剛性倍率（RESP-D 計算編 02「上下大梁の断面性能」）。
        // 対象は水平材（協力幅判定と同様、勾配 5% までは水平とみなす）かつ、両端節点が
        // 四隅を持つ Wall 要素の節点集合に含まれる（=壁の上辺・下辺の大梁）場合のみ。
        // 倍率は剛性用の値（a, iy, iz, j, as_y, as_z）にのみ乗じ、質量用 a_mass は
        // 幾何断面のまま変更しない。
        let lp = (dx * dx + dy * dy).sqrt();
        let is_horizontal = lp > 1e-9 && dz.abs() <= 0.05 * lp;
        let wall_girder_factor = if is_horizontal && is_wall_top_bottom_girder(model, n0, n1) {
            WALL_GIRDER_STIFF_FACTOR
        } else {
            1.0
        };
        let mut a_stiff = a_stiff * wall_girder_factor;
        let mut iy = iy * wall_girder_factor;
        let mut iz = iz * wall_girder_factor;
        let j = j * wall_girder_factor;
        let mut as_y = as_y * wall_girder_factor;
        let mut as_z = as_z * wall_girder_factor;

        // フレーム内雑壁（耐震壁不成立）の周辺部材への断面性能算入
        // （RESP-D 計算編 02「フレーム内雑壁のモデル化」）。柱（鉛直材）には袖壁を、
        // 梁（水平材）には腰壁/垂壁を、平行軸の定理で剛性用断面性能へ合成する。
        // 対象は不成立壁のみ（成立壁は上下大梁100倍で別途考慮済み・排他）。
        let misc_walls = crate::misc_wall::collect_misc_walls(model);
        if !misc_walls.is_empty() {
            // 不変条件の確認: 壁要素と周辺部材（柱・梁）の ElemId は別空間ではなく
            // モデル全体で一意のはずなので、自部材自身が雑壁として収集される
            // （壁要素IDと自部材IDの衝突）ことはない。
            for w in &misc_walls {
                debug_assert_ne!(
                    w.elem, data.id,
                    "壁要素と周辺部材のIDが衝突している（model 不整合）"
                );
            }
            let is_vertical_member = dz.abs() > (dx.abs() + dy.abs()) * 0.5;

            // 自部材の両端節点集合が節点対 b と一致するか（順序不問）
            let same_pair = |a: [NodeId; 2], b: (NodeId, NodeId)| -> bool {
                (a[0] == b.0 && a[1] == b.1) || (a[0] == b.1 && a[1] == b.0)
            };
            // 平行軸の定理による合成: contrib = (合成断面積 Aw, 部材中心からの
            // 符号付き距離 e, 合成断面の自身回りの断面2次モーメント)。
            // 図心 g = Σ(Aw·e)/(Ac+ΣAw) を求めた上で I を再合成する。
            let compose = |i0: f64, ac: f64, contrib: &[(f64, f64, f64)]| -> f64 {
                let sum_aw: f64 = contrib.iter().map(|c| c.0).sum();
                if sum_aw <= 0.0 {
                    return i0;
                }
                let sum_aw_e: f64 = contrib.iter().map(|c| c.0 * c.1).sum();
                let g = sum_aw_e / (ac + sum_aw);
                let mut i_new = i0 + ac * g * g;
                for &(aw, e, self_i) in contrib {
                    i_new += self_i + aw * (e - g).powi(2);
                }
                i_new
            };

            if is_vertical_member {
                // 柱（鉛直材）: 袖壁の算入。面内せいは近似として断面の大きい方の
                // 辺（sec.depth.max(sec.width)）を用いる。
                let d_col = sec.depth.max(sec.width);
                let ac = a_stiff;
                let mut contrib_y: Vec<(f64, f64, f64)> = Vec::new(); // iz・as_y を増強
                let mut contrib_z: Vec<(f64, f64, f64)> = Vec::new(); // iy・as_z を増強
                let mut a_add = 0.0;

                for wall in &misc_walls {
                    for s in 0..2 {
                        let pair = (wall.bottom_pair[s], wall.top_pair[s]);
                        if !same_pair([n0, n1], pair) {
                            continue;
                        }
                        let lww = (wall.wing_length(s) - d_col / 2.0).clamp(0.0, wall.lw);
                        if lww <= 0.0 {
                            continue;
                        }
                        let Some(pa) = model
                            .nodes
                            .get(wall.bottom_pair[0].index())
                            .map(|n| n.coord)
                        else {
                            continue;
                        };
                        let Some(pb) = model
                            .nodes
                            .get(wall.bottom_pair[1].index())
                            .map(|n| n.coord)
                        else {
                            continue;
                        };
                        let wdx = pb[0] - pa[0];
                        let wdy = pb[1] - pa[1];
                        let wl = (wdx * wdx + wdy * wdy).sqrt();
                        if wl < 1e-9 {
                            continue;
                        }
                        // 壁下辺方向の水平単位ベクトルと柱の局所 ey・ez との内積で
                        // 面内たわみ方向（iz↔as_y か iy↔as_z か）を判定する。
                        let e_wall = [wdx / wl, wdy / wl, 0.0];
                        let dot_ey = (axis.rot[1][0] * e_wall[0]
                            + axis.rot[1][1] * e_wall[1]
                            + axis.rot[1][2] * e_wall[2])
                            .abs();
                        let dot_ez = (axis.rot[2][0] * e_wall[0]
                            + axis.rot[2][1] * e_wall[1]
                            + axis.rot[2][2] * e_wall[2])
                            .abs();

                        let aw = wall.t * lww;
                        let e_i = if s == 0 {
                            -(d_col / 2.0 + lww / 2.0)
                        } else {
                            d_col / 2.0 + lww / 2.0
                        };
                        let self_i = wall.t * lww.powi(3) / 12.0;
                        a_add += aw;
                        if dot_ey >= dot_ez {
                            contrib_y.push((aw, e_i, self_i));
                        } else {
                            contrib_z.push((aw, e_i, self_i));
                        }
                    }
                }

                if !contrib_y.is_empty() {
                    iz = compose(iz, ac, &contrib_y);
                    as_y += contrib_y.iter().map(|c| c.0).sum::<f64>() / 1.2;
                }
                if !contrib_z.is_empty() {
                    iy = compose(iy, ac, &contrib_z);
                    as_z += contrib_z.iter().map(|c| c.0).sum::<f64>() / 1.2;
                }
                a_stiff += a_add;
            } else if is_horizontal {
                // 梁（水平材）: 腰壁（下辺の梁に載る壁）・垂壁（上辺の梁から垂れる壁）の
                // 算入。鉛直曲げ（iy・as_z）へ合成する。
                let d_beam = sec.depth;
                let ac = a_stiff;
                let mut contrib: Vec<(f64, f64, f64)> = Vec::new();
                let mut a_add = 0.0;

                for wall in &misc_walls {
                    let bottom = (wall.bottom_pair[0], wall.bottom_pair[1]);
                    let top = (wall.top_pair[0], wall.top_pair[1]);
                    // 下辺の梁なら壁は上に載る（腰壁）、上辺の梁なら壁は下に垂れる（垂壁）。
                    let (matched, hw_raw, sign) = if same_pair([n0, n1], bottom) {
                        (true, wall.strip_height(false), 1.0)
                    } else if same_pair([n0, n1], top) {
                        (true, wall.strip_height(true), -1.0)
                    } else {
                        (false, 0.0, 0.0)
                    };
                    if !matched {
                        continue;
                    }
                    let hw = (hw_raw - d_beam / 2.0).clamp(0.0, wall.h);
                    if hw <= 0.0 {
                        continue;
                    }
                    let aw = wall.t * hw;
                    let e_i = sign * (d_beam / 2.0 + hw / 2.0);
                    let self_i = wall.t * hw.powi(3) / 12.0;
                    a_add += aw;
                    contrib.push((aw, e_i, self_i));
                }

                if !contrib.is_empty() {
                    iy = compose(iy, ac, &contrib);
                    as_z += contrib.iter().map(|c| c.0).sum::<f64>() / 1.2;
                }
                a_stiff += a_add;
            }
        }

        Self {
            id: data.id,
            e: mat.young,
            g,
            a: a_stiff,
            a_mass: sec.area,
            iy,
            iz,
            j,
            as_y,
            as_z,
            length: len,
            density: mat.density,
            nodes: [n0, n1],
            axis,
            rigid: data.rigid_zone,
            end_cond: data.end_cond,
            eval_sections,
            section: data.section,
            material: data.material,
            committed_disp: [0.0; 12],
        }
    }

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
            let mut beam = BeamElement {
                length: l_flex,
                ..BeamElement {
                    end_cond: [EndCondition::Fixed, EndCondition::Fixed],
                    ..self.clone()
                }
            };
            beam.end_cond = [EndCondition::Fixed, EndCondition::Fixed];
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

    pub fn recover_forces(&self, u_elem_global: &[f64; 12]) -> MemberForces {
        let u_local = self.axis.rotate_to_local(u_elem_global);
        let k_local = self.local_stiffness();
        // f_local = K_local * u_local (in local coords, at node ends)
        let mut f_local = [0.0; 12];
        for (i, fi) in f_local.iter_mut().enumerate() {
            let mut s = 0.0;
            for (j, &uj) in u_local.iter().enumerate() {
                s += k_local.get(i, j) * uj;
            }
            *fi = s;
        }

        // N, Qy, Qz, Mx, My, Mz at i-end: f_local[0], f_local[1], f_local[2], f_local[3], f_local[4], f_local[5]
        // j-end: f_local[6], f_local[7], f_local[8], f_local[9], f_local[10], f_local[11]

        let mut at = Vec::new();
        for &xi in &self.eval_sections {
            // 軸力 N は部材内力（引張正）。スパン内軸方向荷重が無い限り一定で、
            // i 端側は節点力 f_local[0]（引張時に -N）、j 端側は f_local[6]（+N）。
            // 旧実装の f0·(1-ξ)+f6·ξ は両端で符号が逆の節点力を線形補間しており、
            // 中央で N=0 となる誤りだったため、せん断と同じ端別採用に修正。
            let (n, qy, qz, mx, my, mz) = if xi < 0.5 {
                let n = -f_local[0];
                let qy = f_local[1];
                let qz = f_local[2];
                let mx = f_local[3];
                let my = f_local[4] - f_local[2] * xi * self.length;
                let mz = f_local[5] + f_local[1] * xi * self.length;
                (n, qy, qz, mx, my, mz)
            } else {
                let n = f_local[6];
                let qy = -f_local[7];
                let qz = -f_local[8];
                let mx = f_local[9];
                let my = f_local[10] - f_local[8] * (1.0 - xi) * self.length;
                let mz = f_local[11] + f_local[7] * (1.0 - xi) * self.length;
                (n, qy, qz, mx, my, mz)
            };
            at.push((xi, [n, qy, qz, mx, my, mz]));
        }

        MemberForces { at }
    }
}

pub(crate) fn invert_small(a: &[f64], n: usize) -> Vec<f64> {
    let mut aug = vec![0.0; n * n * 2];
    for i in 0..n {
        for j in 0..n {
            aug[i * (2 * n) + j] = a[i * n + j];
        }
        aug[i * (2 * n) + n + i] = 1.0;
    }
    for col in 0..n {
        let mut pivot = aug[col * (2 * n) + col];
        if pivot.abs() < 1e-15 {
            pivot = 1.0;
        }
        for j in 0..2 * n {
            aug[col * (2 * n) + j] /= pivot;
        }
        for row in 0..n {
            if row != col {
                let factor = aug[row * (2 * n) + col];
                for j in 0..2 * n {
                    aug[row * (2 * n) + j] -= factor * aug[col * (2 * n) + j];
                }
            }
        }
    }
    let mut inv = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..n {
            inv[i * n + j] = aug[i * (2 * n) + n + j];
        }
    }
    inv
}

impl ElementBehavior for BeamElement {
    fn n_dof(&self) -> usize {
        12
    }

    fn global_dofs(&self, dof: &DofMap) -> SmallVec<[usize; 24]> {
        let mut gdofs = SmallVec::new();
        for &nid in &self.nodes {
            let ni = nid.index();
            for d in 0..DOF_PER_NODE {
                let g = ni * DOF_PER_NODE + d;
                if let Some(active) = dof.active(g) {
                    gdofs.push(active as usize);
                } else {
                    gdofs.push(usize::MAX);
                }
            }
        }
        gdofs
    }

    fn tangent_stiffness(&self, _state: &ElemState, _ctx: &Ctx) -> LocalMat {
        // 要素ローカルの 12×12 を全体系へ回す（K_global = Rᵀ K_local R）。
        // ElementBehavior::tangent_stiffness は全体系を返す契約（シェルと同じ）。
        // これを欠くと、ローカル系とグローバル系が一致しない部材（鉛直柱・
        // 任意方向材・非対称断面 iy≠iz）で組立 K が誤る。
        self.axis.to_global(&self.local_stiffness())
    }

    fn geometric_stiffness(&self, n: f64) -> LocalMat {
        let l = self.length;
        let c = n / l;
        let mut kg = LocalMat::zeros(12);
        let mut s = |i: usize, j: usize, v: f64| {
            kg.set(i, j, v);
            if i != j {
                kg.set(j, i, v);
            }
        };
        // xy面（uy=1,rz=5 / uy_j=7,rz_j=11）
        s(1, 1, c * 6.0 / 5.0);
        s(7, 7, c * 6.0 / 5.0);
        s(1, 7, -c * 6.0 / 5.0);
        s(1, 5, c * l / 10.0);
        s(1, 11, c * l / 10.0);
        s(5, 7, -c * l / 10.0);
        s(7, 11, -c * l / 10.0);
        s(5, 5, c * 2.0 * l * l / 15.0);
        s(11, 11, c * 2.0 * l * l / 15.0);
        s(5, 11, -c * l * l / 30.0);
        // xz面（uz=2,ry=4 / uz_j=8,ry_j=10）§4.1 規約で並進-回転結合項の符号が逆（ry の向き）
        s(2, 2, c * 6.0 / 5.0);
        s(8, 8, c * 6.0 / 5.0);
        s(2, 8, -c * 6.0 / 5.0);
        s(2, 4, -c * l / 10.0);
        s(2, 10, -c * l / 10.0);
        s(4, 8, c * l / 10.0);
        s(8, 10, c * l / 10.0);
        s(4, 4, c * 2.0 * l * l / 15.0);
        s(10, 10, c * 2.0 * l * l / 15.0);
        s(4, 10, -c * l * l / 30.0);
        // 幾何剛性もグローバル系へ回転（P-Δ を組立系で正しく加算するため）
        self.axis.to_global(&kg)
    }

    fn internal_force(&self, _state: &ElemState, _ctx: &Ctx) -> LocalVec {
        // committed_disp はグローバル系で蓄積されるため、グローバル剛性で内力を評価する。
        // f_global = (R^T·K_local·R)·u_global
        let k = self.axis.to_global(&self.local_stiffness());
        let mut f = LocalVec {
            data: SmallVec::from_elem(0.0, 12),
        };
        for i in 0..12 {
            let mut s = 0.0;
            for j in 0..12 {
                s += k.get(i, j) * self.committed_disp[j];
            }
            f.data[i] = s;
        }
        f
    }

    fn update_state(&mut self, du: &LocalVec, commit: bool, _ctx: &Ctx) {
        for i in 0..12 {
            if commit {
                self.committed_disp[i] += du.data[i];
            }
        }
    }

    fn mass_matrix(&self, opt: MassOption) -> LocalMat {
        let m = self.density * self.a_mass * self.length;
        let mut mm = LocalMat::zeros(12);
        match opt {
            MassOption::Lumped => {
                for d in [0, 1, 2, 6, 7, 8] {
                    mm.set(d, d, m / 2.0);
                }
            }
            MassOption::Consistent => {
                let c1 = m / 6.0;
                let c2 = m / 420.0;
                let l = self.length;
                let l2 = l * l;
                // Axial (Ux):  indices 0,6
                mm.set(0, 0, 2.0 * c1);
                mm.set(0, 6, 1.0 * c1);
                mm.set(6, 0, 1.0 * c1);
                mm.set(6, 6, 2.0 * c1);
                // Torsion (Rx): indices 3,9
                let ct = self.density * self.j * l / 6.0;
                mm.set(3, 3, 2.0 * ct);
                mm.set(3, 9, 1.0 * ct);
                mm.set(9, 3, 1.0 * ct);
                mm.set(9, 9, 2.0 * ct);
                // Bending: Hermite 梁の一貫質量（4x4 ブロック）。
                // DOF は連続ではないためインデックス配列で指定する。
                //   Uy-Rz 面: [Uy_i=1, Rz_i=5, Uy_j=7, Rz_j=11]
                //   Uz-Ry 面: [Uz_i=2, Ry_i=4, Uz_j=8, Ry_j=10]（回転符号は逆）
                let b4 = |mm: &mut LocalMat, idx: [usize; 4], sign: f64| {
                    let [d0, r0, d1, r1] = idx;
                    // 並進-並進
                    mm.set(d0, d0, 156.0 * c2);
                    mm.set(d0, d1, 54.0 * c2);
                    mm.set(d1, d0, 54.0 * c2);
                    mm.set(d1, d1, 156.0 * c2);
                    // 並進-回転
                    mm.set(d0, r0, 22.0 * l * c2 * sign);
                    mm.set(r0, d0, 22.0 * l * c2 * sign);
                    mm.set(d0, r1, -13.0 * l * c2 * sign);
                    mm.set(r1, d0, -13.0 * l * c2 * sign);
                    mm.set(d1, r0, 13.0 * l * c2 * sign);
                    mm.set(r0, d1, 13.0 * l * c2 * sign);
                    mm.set(d1, r1, -22.0 * l * c2 * sign);
                    mm.set(r1, d1, -22.0 * l * c2 * sign);
                    // 回転-回転
                    mm.set(r0, r0, 4.0 * l2 * c2);
                    mm.set(r0, r1, -3.0 * l2 * c2);
                    mm.set(r1, r0, -3.0 * l2 * c2);
                    mm.set(r1, r1, 4.0 * l2 * c2);
                };
                b4(&mut mm, [1, 5, 7, 11], 1.0);
                b4(&mut mm, [2, 4, 8, 10], -1.0);
            }
        }
        mm
    }

    fn recover_forces(&self, u_elem: &[f64]) -> Option<crate::beam::MemberForces> {
        if u_elem.len() < 12 {
            return None;
        }
        let mut arr = [0.0; 12];
        arr.copy_from_slice(&u_elem[..12]);
        Some(self.recover_forces(&arr))
    }
}

#[cfg(test)]
mod tests;
