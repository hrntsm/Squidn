//! 節点の型。
//!
//! - [`Node`] — 節点（座標・拘束・質量・所属階）。

use super::*;

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Node {
    pub id: NodeId,
    pub coord: [f64; 3],
    pub restraint: Dof6Mask,
    pub mass: Option<[f64; 6]>,
    pub story: Option<StoryId>,
}
