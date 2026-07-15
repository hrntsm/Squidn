//! フレーム外雑壁（部材化しない壁）の重量集計。
//!
//! - [`misc_wall_weight_shares`] — 雑壁重量の近傍節点への配分（`model.nodes` 添字 → [N]）
//! - [`accumulate_misc_wall_weight`] — 配分結果を節点重量へ加算する
//! - [`nearest_node`] — 指定点に最も近い節点
//! - [`nearest_column_element`] — 指定点に最も近い柱要素

use super::geom::{dist3, is_vertical_pair};
use super::*;

/// モデル全節点のうち、指定点に最も近い節点。
fn nearest_node(model: &Model, pt: [f64; 3]) -> Option<NodeId> {
    model
        .nodes
        .iter()
        .min_by(|a, b| dist3(a.coord, pt).partial_cmp(&dist3(b.coord, pt)).unwrap())
        .map(|n| n.id)
}

/// 「柱要素」（鉛直な `Beam` 要素）のうち、指定点に最も近い節点を持つ要素。
/// §フレーム外雑壁の「柱」伝達タイプに用いる（最近接の柱節点→その柱要素の上下節点）。
fn nearest_column_element(model: &Model, pt: [f64; 3]) -> Option<&ElementData> {
    let mut best: Option<(&ElementData, f64)> = None;
    for e in &model.elements {
        if e.kind != ElementKind::Beam || e.nodes.len() < 2 {
            continue;
        }
        let (a, b) = (
            model.nodes[e.nodes[0].index()].coord,
            model.nodes[e.nodes[1].index()].coord,
        );
        if !is_vertical_pair(a, b) {
            continue;
        }
        let d = dist3(a, pt).min(dist3(b, pt));
        if best.map(|(_, bd)| d < bd).unwrap_or(true) {
            best = Some((e, d));
        }
    }
    best.map(|(e, _)| e)
}

/// フレーム外雑壁（`Model.misc_walls`）の重量を近傍節点へ集計する（§フレーム外雑壁）。
///
/// 壁を長さ 0.5m（端数含む）ごとの領域に分割し、各領域重量
/// `weight_per_area × height × 領域長` を、領域中心位置（下端線分上の点、
/// 高さ方向に `height/2` を加えた 3D 点）から近い節点へ `transfer` の規則で伝達する。
/// - `Column`: 最も近い柱要素（鉛直 `Beam`）の上下 2 節点へ 1/2 ずつ
///   （柱が見つからない場合はモデル全節点中の最近接節点へ全量）
/// - `Beam`: モデル全節点中の最近接節点へ全量集中
/// - `SelfStanding`: 自立壁の簡易扱い。解析用の専用節点を新設する代わりに
///   モデル全節点中の最近接節点（配置階の代表的な節点）へ全量を伝達する
///   （配置階の剛床へ伝達する扱いの簡易近似）。
pub(super) fn accumulate_misc_wall_weight(model: &Model, node_weight: &mut [f64]) {
    for (i, w) in misc_wall_weight_shares(model) {
        node_weight[i] += w;
    }
}

/// [`accumulate_misc_wall_weight`] の配分内容（`model.nodes` 添字 → [N]）を返す。
/// 地震用重量の集計と自重(自動)荷重ケース（[`crate::self_weight`]）で共有する。
pub(crate) fn misc_wall_weight_shares(model: &Model) -> Vec<(usize, f64)> {
    const SEGMENT_LEN: f64 = 500.0;
    let mut shares: Vec<(usize, f64)> = Vec::new();
    for wall in &model.misc_walls {
        let (s, e) = (wall.start, wall.end);
        let (dx, dy, dz) = (e[0] - s[0], e[1] - s[1], e[2] - s[2]);
        let total_len = (dx * dx + dy * dy + dz * dz).sqrt();
        if total_len <= 0.0 {
            continue;
        }
        let mut offset = 0.0;
        while offset < total_len - 1e-9 {
            let seg_len = SEGMENT_LEN.min(total_len - offset);
            let t_center = (offset + seg_len / 2.0) / total_len;
            let center = [
                s[0] + dx * t_center,
                s[1] + dy * t_center,
                s[2] + dz * t_center + wall.height / 2.0,
            ];
            let seg_weight = wall.weight_per_area * wall.height * seg_len;

            match wall.transfer {
                MiscWallTransfer::Column => {
                    if let Some(col) = nearest_column_element(model, center) {
                        let ni = col.nodes[0].index();
                        let nj = col.nodes[1].index();
                        shares.push((ni, seg_weight / 2.0));
                        shares.push((nj, seg_weight / 2.0));
                    } else if let Some(n) = nearest_node(model, center) {
                        shares.push((n.index(), seg_weight));
                    }
                }
                MiscWallTransfer::Beam | MiscWallTransfer::SelfStanding => {
                    if let Some(n) = nearest_node(model, center) {
                        shares.push((n.index(), seg_weight));
                    }
                }
            }

            offset += SEGMENT_LEN;
        }
    }
    shares
}
