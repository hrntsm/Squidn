//! シェル断面力とコンター描画用のデータ構造。
//!
//! - [`ShellResultants`] — 単位幅あたりの断面力（膜・曲げ・せん断）
//! - [`ShellContourPoint`] — 1点のコンター値（ローカル座標＋断面力）
//! - [`ShellContourData`] — 要素4隅節点のコンター値

/// Shell resultants per unit width at a point.
#[derive(Clone, Debug, PartialEq)]
pub struct ShellResultants {
    pub nx: f64,
    pub ny: f64,
    pub nxy: f64,
    pub mx: f64,
    pub my: f64,
    pub mxy: f64,
    pub qx: f64,
    pub qy: f64,
}

/// Contour result at a single point (e.g. an element node).
///
/// Stores the physical coordinates (in the element-local xy‑plane) together
/// with the 8 resultant components.  This is the unit datum used by the
/// contour renderer (UI‑11) to draw filled‑colour fringe plots.
#[derive(Clone, Debug, PartialEq)]
pub struct ShellContourPoint {
    /// Element‑local x‑coordinate [mm]
    pub x: f64,
    /// Element‑local y‑coordinate [mm]
    pub y: f64,
    pub resultants: ShellResultants,
}

/// Per‑element contour data: one `ShellContourPoint` per corner node.
///
/// The 4 entries correspond to the element corner order (node 0 … node 3).
/// Values are obtained by extrapolating from the 2×2 Gauss‑point resultants
/// to the nodes, which gives a visually smooth contour across element
/// boundaries.
#[derive(Clone, Debug, PartialEq)]
pub struct ShellContourData {
    pub node_values: [ShellContourPoint; 4],
}
