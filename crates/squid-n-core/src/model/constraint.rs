//! 拘束条件の型。
//!
//! - [`Constraint`] — 剛床・MPC・剛リンクの拘束定義。

use super::*;
use crate::dof::Dof;

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Constraint {
    RigidDiaphragm {
        story: StoryId,
        master: NodeId,
        slaves: Vec<NodeId>,
    },
    Mpc {
        master: NodeId,
        terms: Vec<(NodeId, Dof, f64)>,
    },
    RigidLink {
        master: NodeId,
        slaves: Vec<NodeId>,
        dofs: Dof6Mask,
    },
}
