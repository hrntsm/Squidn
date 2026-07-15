//! 梁要素とその内力のデータ型定義（ロジックを持たない純粋なデータ層）。

use crate::transform::LocalFrame;
use squid_n_core::ids::{ElemId, NodeId};
use squid_n_core::model::{EndCondition, RigidZone};

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
    /// ならないよう `a` と区別する（材料力学の An はあくまで剛性用）。
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
