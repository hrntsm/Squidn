use faer::sparse::SparseColMat;
use squid_n_core::dof::{Dof, DofMap, DOF_PER_NODE};
use squid_n_core::model::{Constraint, Model};
use squid_n_math::sparse::{assemble_csc, Triplet};

pub struct Reducer {
    pub t_rows: Vec<Vec<(usize, f64)>>,
    pub n_indep: usize,
    pub n_free: usize,
}

impl Reducer {
    pub fn build(model: &Model, dofmap: &DofMap) -> Self {
        let n_free = dofmap.n_active();
        let mut t_rows: Vec<Vec<(usize, f64)>> = (0..n_free).map(|i| vec![(i, 1.0)]).collect();
        let node_coords: Vec<[f64; 3]> = model.nodes.iter().map(|n| n.coord).collect();

        // MPC: master フィールドはスレーブ節点、terms は (マスター節点, マスター DOF, 係数)
        for constraint in &model.constraints {
            if let Constraint::Mpc { master, terms } = constraint {
                let slave_node = master.index();
                // スレーブ DOF d を、同じ d のマスター寄与の和で表す
                for d in 0..DOF_PER_NODE {
                    let sg = slave_node * DOF_PER_NODE + d;
                    if let Some(sa) = dofmap.active(sg) {
                        let s_idx = sa as usize;
                        let mut row = Vec::new();
                        for &(m_node, m_dof, coef) in terms {
                            if m_dof as usize == d {
                                let mg = m_node.index() * DOF_PER_NODE + d;
                                if let Some(ma) = dofmap.active(mg) {
                                    row.push((ma as usize, coef));
                                }
                            }
                        }
                        if s_idx < t_rows.len() && !row.is_empty() {
                            t_rows[s_idx] = row;
                        }
                    }
                }
            }
        }

        // RigidLink: 指定 DOF をマスター節点に拘束
        for constraint in &model.constraints {
            if let Constraint::RigidLink {
                master,
                slaves,
                dofs,
            } = constraint
            {
                let mi = master.index();
                for &slave in slaves {
                    let si = slave.index();
                    for d in 0..DOF_PER_NODE {
                        let dof = match d {
                            0 => Dof::Ux,
                            1 => Dof::Uy,
                            2 => Dof::Uz,
                            3 => Dof::Rx,
                            4 => Dof::Ry,
                            _ => Dof::Rz,
                        };
                        if dofs.is_fixed(dof) {
                            let sg = si * DOF_PER_NODE + d;
                            let mg = mi * DOF_PER_NODE + d;
                            if let Some(sa) = dofmap.active(sg) {
                                if let Some(ma) = dofmap.active(mg) {
                                    let s_idx = sa as usize;
                                    if s_idx < t_rows.len() {
                                        t_rows[s_idx] = vec![(ma as usize, 1.0)];
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // RigidDiaphragm
        for constraint in &model.constraints {
            if let Constraint::RigidDiaphragm {
                story: _,
                master,
                slaves,
            } = constraint
            {
                let mi = master.index();
                let mx = node_coords[mi][0];
                let my = node_coords[mi][1];
                for &slave in slaves {
                    let si = slave.index();
                    let sx = node_coords[si][0];
                    let sy = node_coords[si][1];
                    let dx = sx - mx;
                    let dy = sy - my;
                    // Ux
                    let sg_ux = si * DOF_PER_NODE;
                    let mg_ux = mi * DOF_PER_NODE;
                    let mg_rz = mi * DOF_PER_NODE + 5;
                    if let Some(sa) = dofmap.active(sg_ux) {
                        let s_idx = sa as usize;
                        let mut row = Vec::new();
                        if let Some(ma) = dofmap.active(mg_ux) {
                            row.push((ma as usize, 1.0));
                        }
                        if let Some(ma) = dofmap.active(mg_rz) {
                            row.push((ma as usize, -dy));
                        }
                        if s_idx < t_rows.len() {
                            t_rows[s_idx] = row;
                        }
                    }
                    // Uy
                    let sg_uy = si * DOF_PER_NODE + 1;
                    let mg_uy = mi * DOF_PER_NODE + 1;
                    if let Some(sa) = dofmap.active(sg_uy) {
                        let s_idx = sa as usize;
                        let mut row = Vec::new();
                        if let Some(ma) = dofmap.active(mg_uy) {
                            row.push((ma as usize, 1.0));
                        }
                        if let Some(ma) = dofmap.active(mg_rz) {
                            row.push((ma as usize, dx));
                        }
                        if s_idx < t_rows.len() {
                            t_rows[s_idx] = row;
                        }
                    }
                    // Rz
                    let sg_rz = si * DOF_PER_NODE + 5;
                    if let Some(sa) = dofmap.active(sg_rz) {
                        let s_idx = sa as usize;
                        if let Some(ma) = dofmap.active(mg_rz) {
                            if s_idx < t_rows.len() {
                                t_rows[s_idx] = vec![(ma as usize, 1.0)];
                            }
                        }
                    }
                }
            }
        }

        let mut is_indep = vec![false; t_rows.len()];
        for (i, row) in t_rows.iter().enumerate() {
            if row.len() == 1 && row[0].0 == i && (row[0].1 - 1.0).abs() < 1e-12 {
                is_indep[i] = true;
            }
        }

        // 連鎖拘束の合成: あるスレーブ DOF のマスターがさらに別拘束のスレーブを
        // 兼ねる場合（剛床スレーブが剛リンク/MPC のマスターを兼ねる等）、各従属行を
        // 独立 DOF のみで表すよう推移的に代入する。従来は未合成で、被参照の従属 DOF に
        // 偽の独立番号が割り当てられ、連鎖側の拘束が無言で破れていた。非連鎖の通常
        // ケースでは 1 パスで変化なし（既存挙動と一致）。
        fn merge(row: &mut Vec<(usize, f64)>, idx: usize, coef: f64) {
            if let Some(e) = row.iter_mut().find(|(j, _)| *j == idx) {
                e.1 += coef;
            } else {
                row.push((idx, coef));
            }
        }
        // 非巡回な拘束連鎖は連鎖長回で収束する。上限はサイクル入力への安全弁。
        let max_pass = t_rows.len() + 1;
        for _ in 0..max_pass {
            let mut changed = false;
            for i in 0..t_rows.len() {
                if is_indep[i] {
                    continue;
                }
                // 従属 DOF（自己以外）を参照している行のみ展開する。
                if t_rows[i].iter().any(|&(j, _)| j != i && !is_indep[j]) {
                    let old = std::mem::take(&mut t_rows[i]);
                    let mut newrow: Vec<(usize, f64)> = Vec::new();
                    for (j, c) in old {
                        if is_indep[j] || j == i {
                            merge(&mut newrow, j, c);
                        } else {
                            // マスター j 自身が従属 → その行を係数 c 倍で代入。
                            let sub = t_rows[j].clone();
                            for (k, ck) in sub {
                                merge(&mut newrow, k, c * ck);
                            }
                        }
                    }
                    newrow.retain(|&(_, c)| c.abs() > 1e-15);
                    t_rows[i] = newrow;
                    changed = true;
                }
            }
            if !changed {
                break;
            }
        }

        let mut new_indep = vec![usize::MAX; t_rows.len()];
        let mut counter = 0usize;
        for i in 0..t_rows.len() {
            if is_indep[i] {
                new_indep[i] = counter;
                counter += 1;
            }
        }
        for row in &t_rows {
            for &(idx, _) in row {
                if idx < new_indep.len() && new_indep[idx] == usize::MAX {
                    new_indep[idx] = counter;
                    counter += 1;
                }
            }
        }

        let remapped: Vec<Vec<(usize, f64)>> = t_rows
            .into_iter()
            .map(|row| {
                row.into_iter()
                    .filter_map(|(idx, val)| {
                        if idx < new_indep.len() && new_indep[idx] != usize::MAX {
                            Some((new_indep[idx], val))
                        } else {
                            None
                        }
                    })
                    .collect()
            })
            .collect();

        Reducer {
            t_rows: remapped,
            n_indep: counter,
            n_free,
        }
    }

    /// 拘束縮約 Tᵀ·K·T を計算する。
    ///
    /// K は疎行列なので、格納された非ゼロ要素（CSC 列ごと）だけを走査する。
    /// 従来の n_free² 全ペア走査＋要素ごとの二分探索 (`get`) を廃し、非ゼロ数に
    /// 比例するコストに落とす（結果は同一）。K[i][j] が列 j・行 i の格納値。
    pub fn reduce_k(&self, k_free: &SparseColMat<usize, f64>) -> SparseColMat<usize, f64> {
        let col_ptr = k_free.col_ptr();
        let row_idx = k_free.row_idx();
        let values = k_free.val();
        let ncols = k_free.ncols();
        let mut triplets = Vec::new();
        for j in 0..ncols {
            let tj_list = &self.t_rows[j];
            if tj_list.is_empty() {
                continue;
            }
            for pos in col_ptr[j]..col_ptr[j + 1] {
                let i = row_idx[pos];
                let v = values[pos];
                if v == 0.0 {
                    continue;
                }
                let ti_list = &self.t_rows[i];
                if ti_list.is_empty() {
                    continue;
                }
                for &(a, ta) in ti_list {
                    for &(b, tb) in tj_list {
                        triplets.push(Triplet {
                            row: a,
                            col: b,
                            val: ta * v * tb,
                        });
                    }
                }
            }
        }
        assemble_csc(self.n_indep, triplets)
    }

    pub fn reduce_f(&self, f_free: &[f64]) -> Vec<f64> {
        let mut f_red = vec![0.0; self.n_indep];
        for i in 0..self.n_free {
            if f_free[i] != 0.0 {
                for &(a, ta) in &self.t_rows[i] {
                    f_red[a] += ta * f_free[i];
                }
            }
        }
        f_red
    }

    pub fn expand_u(&self, u_indep: &[f64]) -> Vec<f64> {
        let mut u_free = vec![0.0; self.n_free];
        for i in 0..self.n_free {
            for &(a, ta) in &self.t_rows[i] {
                u_free[i] += ta * u_indep[a];
            }
        }
        u_free
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use squid_n_core::dof::Dof6Mask;
    use squid_n_core::ids::{ElemId, MaterialId, NodeId, SectionId, StoryId};
    use squid_n_core::model::{
        Constraint, ElementData, ElementKind, LocalAxis, Material, Model, Node, Section,
    };

    fn make_3node_model() -> Model {
        Model {
            nodes: vec![
                Node {
                    id: NodeId(0),
                    coord: [0.0, 0.0, 0.0],
                    restraint: Dof6Mask::FIXED,
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(1),
                    coord: [0.0, 1000.0, 0.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: None,
                },
                Node {
                    id: NodeId(2),
                    coord: [1000.0, 1000.0, 0.0],
                    restraint: Dof6Mask::FREE,
                    mass: None,
                    story: None,
                },
            ],
            // 全節点に要素を接続する（要素が接続しない節点は DofMap が解析自由度
            // から除外するため、拘束縮約のテスト対象にならない）。
            elements: (0..2)
                .map(|i| ElementData {
                    id: ElemId(i),
                    kind: ElementKind::Beam,
                    nodes: smallvec::smallvec![NodeId(i), NodeId(i + 1)],
                    section: Some(SectionId(0)),
                    material: Some(MaterialId(0)),
                    local_axis: LocalAxis {
                        ref_vector: [0.0, 0.0, 1.0],
                    },
                    end_cond: [
                        squid_n_core::model::EndCondition::Fixed,
                        squid_n_core::model::EndCondition::Fixed,
                    ],
                    force_regime: squid_n_core::model::ForceRegime::Auto,
                    rigid_zone: Default::default(),
                    plastic_zone: None,
                    spring: None,
                })
                .collect(),
            sections: vec![Section {
                id: SectionId(0),
                name: "sec".to_string(),
                area: 100.0,
                iy: 1000.0,
                iz: 1000.0,
                j: 100.0,
                depth: 100.0,
                width: 100.0,
                as_y: 83.33,
                as_z: 83.33,
                panel_thickness: None,
                thickness: None,
                shape: None,
            }],
            materials: vec![Material {
                concrete_class: Default::default(),
                id: MaterialId(0),
                name: "mat".to_string(),
                young: 1000.0,
                poisson: 0.3,
                density: 0.0,
                shear: None,
                fc: None,
                fy: None,
            }],
            ..Default::default()
        }
    }

    #[test]
    fn test_no_constraint_identity() {
        let model = make_3node_model();
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);
        assert_eq!(reducer.n_indep, reducer.n_free);
        for i in 0..reducer.n_free {
            assert_eq!(reducer.t_rows[i], vec![(i, 1.0)]);
        }
    }

    #[test]
    fn test_rigid_diaphragm() {
        let mut model = make_3node_model();
        model.constraints.push(Constraint::RigidDiaphragm {
            story: StoryId(0),
            master: NodeId(1),
            slaves: vec![NodeId(2)],
        });
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);
        // slave Ux/Uy/Rz が master に従うため独立 DOF が減る
        assert!(reducer.n_indep < reducer.n_free);
    }

    #[test]
    fn test_rigid_link() {
        let mut model = make_3node_model();
        model.constraints.push(Constraint::RigidLink {
            master: NodeId(1),
            slaves: vec![NodeId(2)],
            dofs: Dof6Mask::FIXED,
        });
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);
        // スレーブ 6 DOF がマスターに従う
        assert!(reducer.n_indep < reducer.n_free);
    }

    /// 代表節点（要素非接続・Uz/Rx/Ry 固定の浮遊節点）をマスターとした剛床で、
    /// スレーブの面内変位が ix = Gx − iry·Gθz, iy = Gy + irx·Gθz（剛床仮定に
    /// よる面内剛体変位の運動学）どおりに復元されることを確認する。
    #[test]
    fn test_rigid_diaphragm_master_recovers_translation_and_torsion() {
        let mut model = make_3node_model();
        let mut rep_restraint = Dof6Mask::FREE;
        rep_restraint.set_fixed(Dof::Uz);
        rep_restraint.set_fixed(Dof::Rx);
        rep_restraint.set_fixed(Dof::Ry);
        let master_coord = [500.0, 1000.0, 0.0];
        model.nodes.push(Node {
            id: NodeId(3),
            coord: master_coord,
            restraint: rep_restraint,
            mass: None,
            story: None,
        });
        model.constraints.push(Constraint::RigidDiaphragm {
            story: StoryId(0),
            master: NodeId(3),
            slaves: vec![NodeId(1), NodeId(2)],
        });
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);

        let g_master_ux = 3 * DOF_PER_NODE;
        let g_master_uy = 3 * DOF_PER_NODE + 1;
        let g_master_rz = 3 * DOF_PER_NODE + 5;
        let a_ux = dofmap.active(g_master_ux).unwrap() as usize;
        let a_uy = dofmap.active(g_master_uy).unwrap() as usize;
        let a_rz = dofmap.active(g_master_rz).unwrap() as usize;
        // マスター自身の DOF は独立(恒等写像の行)のはず。
        let idx_ux = reducer.t_rows[a_ux][0].0;
        let idx_uy = reducer.t_rows[a_uy][0].0;
        let idx_rz = reducer.t_rows[a_rz][0].0;

        let (gx, gy, gtheta) = (2.0, -1.5, 0.002);
        let mut u_indep = vec![0.0; reducer.n_indep];
        u_indep[idx_ux] = gx;
        u_indep[idx_uy] = gy;
        u_indep[idx_rz] = gtheta;

        let u_free = reducer.expand_u(&u_indep);

        for &slave in &[NodeId(1), NodeId(2)] {
            let si = slave.index();
            let dx = model.nodes[si].coord[0] - master_coord[0];
            let dy = model.nodes[si].coord[1] - master_coord[1];
            let expected_ux = gx - dy * gtheta;
            let expected_uy = gy + dx * gtheta;
            let a_slave_ux = dofmap.active(si * DOF_PER_NODE).unwrap() as usize;
            let a_slave_uy = dofmap.active(si * DOF_PER_NODE + 1).unwrap() as usize;
            assert!(
                (u_free[a_slave_ux] - expected_ux).abs() < 1e-9,
                "ix: got={} want={}",
                u_free[a_slave_ux],
                expected_ux
            );
            assert!(
                (u_free[a_slave_uy] - expected_uy).abs() < 1e-9,
                "iy: got={} want={}",
                u_free[a_slave_uy],
                expected_uy
            );
        }
    }

    #[test]
    fn test_mpc() {
        let mut model = make_3node_model();
        // スレーブ NodeId(2) の Ux = 0.5 * NodeId(1) の Ux
        model.constraints.push(Constraint::Mpc {
            master: NodeId(2),
            terms: vec![(NodeId(1), squid_n_core::dof::Dof::Ux, 0.5)],
        });
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);
        assert!(reducer.n_indep < reducer.n_free);
    }

    /// 連鎖拘束（スレーブのマスターがさらに別拘束のスレーブ）の合成を検証する。
    /// node1.Ux = node0.Ux（MPC）、node2.Ux = node1.Ux（MPC）の連鎖では、
    /// node2.Ux は推移的に node0.Ux に一致しなければならない。従来は node2 が
    /// node1 の従属自由度へ偽の独立番号で結び付き、連鎖が無言で破れていた。
    #[test]
    fn test_chained_mpc_constraints_compose_transitively() {
        let mut model = make_3node_model();
        model.nodes[0].restraint = Dof6Mask::FREE; // 3 節点とも自由に
                                                   // node1.Ux = node0.Ux
        model.constraints.push(Constraint::Mpc {
            master: NodeId(1),
            terms: vec![(NodeId(0), squid_n_core::dof::Dof::Ux, 1.0)],
        });
        // node2.Ux = node1.Ux（node1 は上の MPC のスレーブ＝連鎖）
        model.constraints.push(Constraint::Mpc {
            master: NodeId(2),
            terms: vec![(NodeId(1), squid_n_core::dof::Dof::Ux, 1.0)],
        });
        let dofmap = DofMap::build(&model);
        let reducer = Reducer::build(&model, &dofmap);

        // 各ノードの Ux グローバル DOF = node_index * DOF_PER_NODE。
        let a0 = dofmap.active(0).unwrap() as usize; // node0 Ux
        let a1 = dofmap.active(DOF_PER_NODE).unwrap() as usize; // node1 Ux
        let a2 = dofmap.active(2 * DOF_PER_NODE).unwrap() as usize; // node2 Ux

        // node0.Ux は独立（行は自身への単位行）。その独立番号にだけ単位値を与える。
        assert_eq!(reducer.t_rows[a0].len(), 1);
        let idx0 = reducer.t_rows[a0][0].0;
        let mut u_indep = vec![0.0; reducer.n_indep];
        u_indep[idx0] = 1.0;
        let u_free = reducer.expand_u(&u_indep);

        assert!((u_free[a0] - 1.0).abs() < 1e-12, "node0.Ux={}", u_free[a0]);
        assert!(
            (u_free[a1] - 1.0).abs() < 1e-12,
            "node1.Ux={} should follow node0",
            u_free[a1]
        );
        assert!(
            (u_free[a2] - 1.0).abs() < 1e-12,
            "node2.Ux={} should follow node0 transitively",
            u_free[a2]
        );
        // 連鎖で独立 DOF が 2 個（node1.Ux, node2.Ux）減る。
        assert_eq!(reducer.n_indep, reducer.n_free - 2);
    }
}
