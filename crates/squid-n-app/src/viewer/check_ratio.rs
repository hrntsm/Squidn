//! 検定比図（部材検定・節点検定の最大検定比による着色）の描画。
//!
//! 検定表（`design_view.rs`）と同じ [`theme::status_color`] の3色規約
//! （≤0.8 緑=OK／≤1.0 黄=注意／>1.0 赤=NG）で部材・節点を着色し、
//! 検定表と3Dビューの見え方を一貫させる。連続的なコンター配色は採らない
//! （判定基準 0.8/1.0 との対応を優先するため。詳細は dev_docs の申し送りを参照）。

use std::collections::HashMap;

use crate::app::App;
use crate::theme;
use squid_n_core::ids::{ElemId, NodeId};

/// 部材（`ElemId`）ごとに、その部材の全検定位置の最大検定比と「全位置 OK か」
/// （1つでも NG があれば false）を集計する（純粋関数）。
///
/// `items` は `(部材ID, 検定比, OK フラグ)` のイテレータ。`ResultsBundle::checks`
/// の `(ElemId, xi, CheckResult)` から呼び出し側で変換して渡す想定
/// （`CheckResult` は文字列フィールドを持ち型が重いため、テストしやすいよう
/// 軽量なタプルに変換してから渡す設計にしている）。
fn max_ratio_by_key<K, I>(items: I) -> HashMap<K, (f64, bool)>
where
    K: Eq + std::hash::Hash,
    I: IntoIterator<Item = (K, f64, bool)>,
{
    let mut map: HashMap<K, (f64, bool)> = HashMap::new();
    for (key, ratio, ok) in items {
        let entry = map.entry(key).or_insert((0.0_f64, true));
        if ratio > entry.0 {
            entry.0 = ratio;
        }
        entry.1 &= ok;
    }
    map
}

/// 部材ごとの最大検定比・OK フラグを集計する。
fn max_ratio_by_elem<I: IntoIterator<Item = (ElemId, f64, bool)>>(
    items: I,
) -> HashMap<ElemId, (f64, bool)> {
    max_ratio_by_key(items)
}

/// 節点ごとの最大検定比・OK フラグを集計する。
fn max_ratio_by_node<I: IntoIterator<Item = (NodeId, f64, bool)>>(
    items: I,
) -> HashMap<NodeId, (f64, bool)> {
    max_ratio_by_key(items)
}

/// 検定比図を描く。`pts` は `viewer_panel` で計算済みの節点スクリーン座標
/// （`app.model.nodes` と同じ順序）。
pub(super) fn draw_check_ratio(painter: &egui::Painter, app: &App, pts: &[[f32; 2]]) {
    let Some(results) = &app.results else {
        draw_no_result_legend(painter);
        return;
    };
    // 部材検定・節点検定のどちらかがあれば描画する（耐震壁のみのモデル等では
    // 部材検定が空でも節点検定だけが存在しうる）。
    if results.checks.is_empty() && results.joint_checks.is_empty() {
        draw_no_result_legend(painter);
        return;
    }

    let elem_ratios = max_ratio_by_elem(
        results
            .checks
            .iter()
            .map(|(id, _xi, cr)| (*id, cr.ratio, cr.ok)),
    );
    let node_ratios = max_ratio_by_node(
        results
            .joint_checks
            .iter()
            .map(|(id, _label, cr)| (*id, cr.ratio, cr.ok)),
    );

    // --- 部材の着色 ---
    for elem in &app.model.elements {
        let Some(&(ratio, ok)) = elem_ratios.get(&elem.id) else {
            continue;
        };
        let color = theme::status_color(ratio);

        // 壁（面要素）: 半透明ポリゴンで塗り、輪郭を検定比の色で強調する
        if elem.kind == squid_n_core::model::ElementKind::Wall && elem.nodes.len() >= 3 {
            let poly: Vec<egui::Pos2> = elem
                .nodes
                .iter()
                .filter_map(|n| {
                    let idx = n.index();
                    (idx < pts.len()).then(|| egui::pos2(pts[idx][0], pts[idx][1]))
                })
                .collect();
            if poly.len() == elem.nodes.len() {
                painter.add(egui::Shape::convex_polygon(
                    poly,
                    theme::translucent(color, 70),
                    egui::Stroke::new(2.0_f32, color),
                ));
            }
            continue;
        }

        // 線材: 両端を結ぶ線を検定比の色で描き、中点に数値ラベルを添える。
        if elem.nodes.len() < 2 {
            continue;
        }
        let n0 = elem.nodes[0].index();
        let n1 = elem.nodes[1].index();
        if n0 >= pts.len() || n1 >= pts.len() {
            continue;
        }
        let p0 = egui::pos2(pts[n0][0], pts[n0][1]);
        let p1 = egui::pos2(pts[n1][0], pts[n1][1]);
        // NG 部材は太さで目立たせる
        let width = if ok { 4.0_f32 } else { 5.0_f32 };
        painter.line_segment([p0, p1], egui::Stroke::new(width, color));

        let mid = egui::pos2((p0.x + p1.x) * 0.5, (p0.y + p1.y) * 0.5);
        let (font_size, label_color) = if ok {
            (11.0, theme::GRAY_700)
        } else {
            // NG はフォントを大きくし赤字で目立たせる
            (12.0, theme::PARETO_RED)
        };
        painter.text(
            mid,
            egui::Align2::CENTER_BOTTOM,
            format!("{:.2}", ratio),
            egui::FontId::proportional(font_size),
            label_color,
        );
    }

    // --- 節点検定（接合部・パネルゾーン・耐震壁など）の表示 ---
    // NodeId の内部値はそのまま配列添字とは限らないため、`app.model.nodes` を
    // 走査してインデックスを求め（`enumerate` の添字が実際の `pts` の添字）、
    // `node.id` と突き合わせてから `pts` を引く。
    for (idx, node) in app.model.nodes.iter().enumerate() {
        let Some(&(ratio, _ok)) = node_ratios.get(&node.id) else {
            continue;
        };
        if idx >= pts.len() {
            continue;
        }
        let p = egui::pos2(pts[idx][0], pts[idx][1]);
        let color = theme::status_color(ratio);
        painter.circle_filled(p, 5.0, color);
        painter.circle_stroke(p, 5.0, egui::Stroke::new(1.0_f32, theme::VIEW_BG));
    }

    draw_legend(painter, app, &elem_ratios, &node_ratios);
}

/// 検定結果が無い場合の案内表示。
fn draw_no_result_legend(painter: &egui::Painter) {
    painter.text(
        egui::pos2(
            painter.clip_rect().min.x + 10.0,
            painter.clip_rect().min.y + 10.0,
        ),
        egui::Align2::LEFT_TOP,
        "検定結果がありません。解析タブから静的解析を実行してください。",
        egui::FontId::proportional(14.0),
        theme::GRAY_600,
    );
}

/// ビュー左上に検定比図の凡例（最大値・NG件数・色見本・陳腐化注記）を描く。
fn draw_legend(
    painter: &egui::Painter,
    app: &App,
    elem_ratios: &HashMap<ElemId, (f64, bool)>,
    node_ratios: &HashMap<NodeId, (f64, bool)>,
) {
    let rect = painter.clip_rect();
    let x0 = rect.min.x + 10.0;
    let mut y = rect.min.y + 10.0;

    let max_ratio = elem_ratios
        .values()
        .chain(node_ratios.values())
        .map(|&(r, _)| r)
        .fold(0.0_f64, f64::max);
    let ng_count = elem_ratios
        .values()
        .chain(node_ratios.values())
        .filter(|&&(_, ok)| !ok)
        .count();

    let title_rect = painter.text(
        egui::pos2(x0, y),
        egui::Align2::LEFT_TOP,
        format!("検定比図 (max={:.2}, NG {}件)", max_ratio, ng_count),
        egui::FontId::proportional(14.0),
        theme::GRAY_700,
    );
    y = title_rect.max.y + 4.0;

    // 色見本: ≤0.8=緑／≤1.0=黄／>1.0 NG=赤 の順に横並びで描き、末尾に未検定の注記
    const SWATCH: f32 = 12.0;
    let mut x = x0;
    for (color, label) in [
        (theme::GOOD_GREEN, "≤0.8"),
        (theme::BEST_YELLOW, "≤1.0"),
        (theme::PARETO_RED, ">1.0 NG"),
    ] {
        painter.rect_filled(
            egui::Rect::from_min_size(egui::pos2(x, y), egui::vec2(SWATCH, SWATCH)),
            0.0,
            color,
        );
        let text_rect = painter.text(
            egui::pos2(x + SWATCH + 4.0, y),
            egui::Align2::LEFT_TOP,
            label,
            egui::FontId::proportional(11.0),
            theme::GRAY_600,
        );
        x = text_rect.max.x + 12.0;
    }
    let untested_rect = painter.text(
        egui::pos2(x, y),
        egui::Align2::LEFT_TOP,
        "未検定: グレー",
        egui::FontId::proportional(11.0),
        theme::GRAY_600,
    );
    y = untested_rect.max.y.max(y + SWATCH) + 4.0;

    if app.staleness.design_stale {
        painter.text(
            egui::pos2(x0, y),
            egui::Align2::LEFT_TOP,
            "⚠ モデルが編集されています。解析を再実行してください。",
            egui::FontId::proportional(12.0),
            theme::BEST_YELLOW,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 同一部材に複数の検定位置がある場合、最大の検定比が採用される。
    #[test]
    fn max_ratio_by_elem_picks_max_ratio() {
        let id = ElemId(0);
        let map = max_ratio_by_elem([(id, 0.5, true), (id, 0.9, true), (id, 0.3, true)]);
        assert_eq!(map[&id].0, 0.9);
    }

    /// 1つでも NG（ok=false）の位置があれば、部材全体として NG（false）になる。
    #[test]
    fn max_ratio_by_elem_ng_propagates() {
        let id = ElemId(1);
        let map = max_ratio_by_elem([(id, 0.5, true), (id, 1.2, false), (id, 0.3, true)]);
        assert_eq!(map[&id].0, 1.2);
        assert!(!map[&id].1);
    }

    /// 全位置が OK なら OK フラグは true のまま。
    #[test]
    fn max_ratio_by_elem_all_ok_stays_ok() {
        let id = ElemId(2);
        let map = max_ratio_by_elem([(id, 0.4, true), (id, 0.6, true)]);
        assert!(map[&id].1);
    }

    /// 複数部材のデータは部材ごとに分離して集計される。
    #[test]
    fn max_ratio_by_elem_separates_by_id() {
        let a = ElemId(0);
        let b = ElemId(1);
        let map = max_ratio_by_elem([(a, 0.5, true), (b, 1.5, false), (a, 0.8, true)]);
        assert_eq!(map.len(), 2);
        assert_eq!(map[&a].0, 0.8);
        assert!(map[&a].1);
        assert_eq!(map[&b].0, 1.5);
        assert!(!map[&b].1);
    }

    /// 空入力は空の集計結果を返す。
    #[test]
    fn max_ratio_by_elem_empty_input() {
        let map = max_ratio_by_elem(std::iter::empty::<(ElemId, f64, bool)>());
        assert!(map.is_empty());
    }

    /// 節点単位の集計も同じ規則（最大値採用・NG 伝播）で動作する。
    #[test]
    fn max_ratio_by_node_picks_max_and_propagates_ng() {
        let n = NodeId(0);
        let map = max_ratio_by_node([(n, 0.7, true), (n, 1.1, false)]);
        assert_eq!(map[&n].0, 1.1);
        assert!(!map[&n].1);
    }

    /// 節点集計は複数節点を分離して保持する。
    #[test]
    fn max_ratio_by_node_separates_by_id() {
        let a = NodeId(0);
        let b = NodeId(1);
        let map = max_ratio_by_node([(a, 0.2, true), (b, 0.95, true)]);
        assert_eq!(map.len(), 2);
        assert_eq!(map[&a].0, 0.2);
        assert_eq!(map[&b].0, 0.95);
    }

    /// 節点集計の空入力は空の結果を返す。
    #[test]
    fn max_ratio_by_node_empty_input() {
        let map = max_ratio_by_node(std::iter::empty::<(NodeId, f64, bool)>());
        assert!(map.is_empty());
    }
}
