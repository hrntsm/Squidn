use crate::app::App;
use squid_n_core::ids::{NodeId, SlabId};
use squid_n_core::model::{AreaLoad, DistributionMethod, OneWayDir, SlabKind, SlabUsage};
use squid_n_edit::{AddSlab, DeleteSlab, SetSlabKind, SetSlabOneWay, SetSlabUsage};

/// スラブ追加フォームのドラフト状態（GUI 専用）。
/// `nodes` は境界4節点（頂点0→1→2→3→0 の順で外周を辿る）の選択状態。
#[derive(Clone, Debug)]
pub struct SlabDraft {
    pub nodes: [Option<NodeId>; 4],
    /// 荷重種別（既定 "DL"）
    pub load_kind: String,
    /// 荷重値の入力文字列。**UI 表示は kN/m²**（内部格納は ×1e-3 した N/mm²）。
    pub load_value: String,
    pub method: DistributionMethod,
    /// スラブ用途（積載荷重プリセット。`None` は積載寄与なし）。
    pub usage: Option<SlabUsage>,
}

impl Default for SlabDraft {
    fn default() -> Self {
        Self {
            nodes: [None; 4],
            load_kind: "DL".to_string(),
            load_value: "0".to_string(),
            method: DistributionMethod::TriTrapezoid,
            usage: None,
        }
    }
}

/// 用途選択で提示するプリセット（令別表第1）。`None` は「なし（積載寄与なし）」。
/// `Custom` は UI からは扱わない（モデル/シリアライズでは利用可）。
const USAGE_PRESETS: &[Option<SlabUsage>] = &[
    None,
    Some(SlabUsage::Residential),
    Some(SlabUsage::Office),
    Some(SlabUsage::Classroom),
    Some(SlabUsage::Store),
    Some(SlabUsage::AssemblyFixed),
    Some(SlabUsage::AssemblyOther),
    Some(SlabUsage::Corridor),
    Some(SlabUsage::Garage),
    Some(SlabUsage::RoofResidential),
    Some(SlabUsage::RoofStore),
];

fn usage_label(u: Option<SlabUsage>) -> &'static str {
    match u {
        None => "なし",
        Some(SlabUsage::Residential) => "住宅の居室・寝室・病室",
        Some(SlabUsage::Office) => "事務室",
        Some(SlabUsage::Classroom) => "教室",
        Some(SlabUsage::Store) => "百貨店・店舗の売場",
        Some(SlabUsage::AssemblyFixed) => "集会室・客席（固定席）",
        Some(SlabUsage::AssemblyOther) => "集会室・客席（その他）",
        Some(SlabUsage::Corridor) => "廊下・玄関・階段",
        Some(SlabUsage::Garage) => "自動車車庫・通路",
        Some(SlabUsage::RoofResidential) => "屋上・バルコニー（住宅系）",
        Some(SlabUsage::RoofStore) => "屋上・バルコニー（学校・百貨店系）",
        Some(SlabUsage::Custom { .. }) => "任意入力",
    }
}

fn method_label(m: DistributionMethod) -> &'static str {
    match m {
        DistributionMethod::TriTrapezoid => "三角/台形(45°法)",
        DistributionMethod::OneWay => "一方向",
        DistributionMethod::TributaryArea => "負担面積",
    }
}

fn kind_label(k: SlabKind) -> &'static str {
    match k {
        SlabKind::Interior => "一般",
        SlabKind::Cantilever => "片持ち",
        SlabKind::Corner => "出隅",
    }
}

fn one_way_label(o: Option<OneWayDir>) -> &'static str {
    match o {
        None => "なし",
        Some(OneWayDir::X) => "X",
        Some(OneWayDir::Y) => "Y",
    }
}

pub fn slabs_table(ui: &mut egui::Ui, app: &mut App) {
    use egui_extras::{Column, TableBuilder};

    ui.label(
        "スラブは境界4節点・外周の梁があって初めて機能します（結果タブ/モデルタブの3Dビューで表示モード「CMQ図」を選ぶと分配結果を確認できます）。",
    );
    ui.separator();

    // ── 一覧表 ──────────────────────────────────────────
    let n = app.model.slabs.len();
    let mut pending_delete: Option<SlabId> = None;
    let mut pending_kind: Vec<(SlabId, SlabKind)> = Vec::new();
    let mut pending_one_way: Vec<(SlabId, Option<OneWayDir>)> = Vec::new();
    let mut pending_usage: Vec<(SlabId, Option<SlabUsage>)> = Vec::new();

    TableBuilder::new(ui)
        .striped(true)
        .column(Column::auto())
        .column(Column::initial(140.0))
        .column(Column::initial(200.0))
        .column(Column::initial(140.0))
        .column(Column::initial(90.0))
        .column(Column::initial(90.0))
        .column(Column::initial(180.0))
        .column(Column::auto())
        .header(20.0, |mut h| {
            for t in &[
                "ID",
                "境界節点",
                "荷重",
                "分配法",
                "種別",
                "一方向",
                "用途",
                "",
            ] {
                h.col(|ui| {
                    ui.strong(*t);
                });
            }
        })
        .body(|body| {
            body.rows(22.0, n, |mut row| {
                let i = row.index();
                let slab = &app.model.slabs[i];
                row.col(|ui| {
                    ui.label(slab.id.0.to_string());
                });
                row.col(|ui| {
                    let s = slab
                        .boundary
                        .iter()
                        .map(|n| n.0.to_string())
                        .collect::<Vec<_>>()
                        .join("-");
                    ui.label(s);
                });
                row.col(|ui| {
                    let s = slab
                        .loads
                        .iter()
                        .map(|l| format!("{} {:.2}kN/m²", l.kind, l.value * 1e3))
                        .collect::<Vec<_>>()
                        .join(", ");
                    ui.label(if s.is_empty() { "―".to_string() } else { s });
                });
                row.col(|ui| {
                    ui.label(method_label(slab.method));
                });
                row.col(|ui| {
                    egui::ComboBox::from_id_salt(("slab_kind", slab.id.0))
                        .selected_text(kind_label(slab.kind))
                        .show_ui(ui, |ui| {
                            for kind in [SlabKind::Interior, SlabKind::Cantilever, SlabKind::Corner]
                            {
                                if ui
                                    .selectable_label(slab.kind == kind, kind_label(kind))
                                    .clicked()
                                    && slab.kind != kind
                                {
                                    pending_kind.push((slab.id, kind));
                                }
                            }
                        });
                });
                row.col(|ui| {
                    egui::ComboBox::from_id_salt(("slab_one_way", slab.id.0))
                        .selected_text(one_way_label(slab.one_way))
                        .show_ui(ui, |ui| {
                            for ow in [None, Some(OneWayDir::X), Some(OneWayDir::Y)] {
                                if ui
                                    .selectable_label(slab.one_way == ow, one_way_label(ow))
                                    .clicked()
                                    && slab.one_way != ow
                                {
                                    pending_one_way.push((slab.id, ow));
                                }
                            }
                        });
                });
                row.col(|ui| {
                    egui::ComboBox::from_id_salt(("slab_usage", slab.id.0))
                        .selected_text(usage_label(slab.usage))
                        .show_ui(ui, |ui| {
                            for &u in USAGE_PRESETS {
                                if ui
                                    .selectable_label(slab.usage == u, usage_label(u))
                                    .clicked()
                                    && slab.usage != u
                                {
                                    pending_usage.push((slab.id, u));
                                }
                            }
                        });
                });
                row.col(|ui| {
                    if ui.button("🗑").on_hover_text("このスラブを削除").clicked() {
                        pending_delete = Some(slab.id);
                    }
                });
            });
        });

    let had_pending = !pending_kind.is_empty()
        || !pending_one_way.is_empty()
        || !pending_usage.is_empty()
        || pending_delete.is_some();
    for (id, kind) in pending_kind {
        app.undo
            .run(&mut app.model, Box::new(SetSlabKind { id, kind }));
    }
    for (id, one_way) in pending_one_way {
        app.undo
            .run(&mut app.model, Box::new(SetSlabOneWay { id, one_way }));
    }
    for (id, usage) in pending_usage {
        app.undo
            .run(&mut app.model, Box::new(SetSlabUsage { id, usage }));
    }
    if let Some(id) = pending_delete {
        app.undo.run(&mut app.model, Box::new(DeleteSlab { id }));
    }
    if had_pending {
        app.staleness.mark_edited();
    }

    ui.separator();
    // ── スラブ追加フォーム ──────────────────────────────────
    ui.strong("スラブを追加");

    if app.model.nodes.len() < 4 {
        ui.label("スラブを追加するには節点が4つ以上必要です");
        return;
    }

    // 借用衝突を避けるため、節点一覧は先にローカルへ複製しておく
    // （app.model への参照を保持したまま app.slab_draft を可変参照しないため）。
    let node_ids: Vec<NodeId> = app.model.nodes.iter().map(|n| n.id).collect();

    ui.label(
        "境界節点（頂点0→1→2→3→0 の順で外周を辿り、その辺 i=節点i→節点i+1 を持つ梁を検索します）:",
    );
    ui.horizontal_wrapped(|ui| {
        for k in 0..4 {
            let text = app.slab_draft.nodes[k]
                .map(|n| format!("N{}", n.0))
                .unwrap_or_else(|| "―".to_string());
            egui::ComboBox::from_id_salt(format!("slab_draft_node_{}", k))
                .selected_text(format!("頂点{}: {}", k, text))
                .show_ui(ui, |ui| {
                    for &nid in &node_ids {
                        let label = format!("N{}", nid.0);
                        if ui
                            .selectable_label(app.slab_draft.nodes[k] == Some(nid), &label)
                            .clicked()
                        {
                            app.slab_draft.nodes[k] = Some(nid);
                        }
                    }
                });
        }
    });

    ui.horizontal(|ui| {
        ui.label("荷重種別:");
        ui.add(egui::TextEdit::singleline(&mut app.slab_draft.load_kind).desired_width(60.0));
        ui.label("荷重 [kN/m²]:");
        ui.add(egui::TextEdit::singleline(&mut app.slab_draft.load_value).desired_width(80.0));
    });

    ui.horizontal(|ui| {
        ui.label("用途（積載荷重）:")
            .on_hover_text("令別表第1 の積載荷重（骨組用）を「床積載(自動)」ケースへ分配します");
        egui::ComboBox::from_id_salt("slab_draft_usage")
            .selected_text(usage_label(app.slab_draft.usage))
            .show_ui(ui, |ui| {
                for &u in USAGE_PRESETS {
                    ui.selectable_value(&mut app.slab_draft.usage, u, usage_label(u));
                }
            });
        if let Some(u) = app.slab_draft.usage {
            use squid_n_core::model::LoadPurpose;
            // 表示は kN/m²（内部 N/mm² を ×1e3）。
            ui.label(format!(
                "床用 {:.2} / 骨組用 {:.2} / 地震用 {:.2} kN/m²",
                u.live_load(LoadPurpose::Floor) * 1e3,
                u.live_load(LoadPurpose::Frame) * 1e3,
                u.live_load(LoadPurpose::Seismic) * 1e3,
            ));
        }
    });

    ui.horizontal(|ui| {
        ui.label("分配法:");
        ui.selectable_value(
            &mut app.slab_draft.method,
            DistributionMethod::TriTrapezoid,
            "三角/台形(45°法)",
        );
        ui.selectable_value(
            &mut app.slab_draft.method,
            DistributionMethod::OneWay,
            "一方向",
        );
        ui.selectable_value(
            &mut app.slab_draft.method,
            DistributionMethod::TributaryArea,
            "負担面積",
        );
    });

    let selected: Vec<NodeId> = app.slab_draft.nodes.iter().filter_map(|n| *n).collect();
    let mut dedup = selected.clone();
    dedup.sort_by_key(|n| n.0);
    dedup.dedup();
    let can_add = selected.len() == 4 && dedup.len() == 4;

    if ui
        .add_enabled(can_add, egui::Button::new("+ 追加"))
        .on_hover_text("境界節点4つがすべて選択され、かつ重複が無い場合に追加できます")
        .clicked()
    {
        let boundary: Vec<NodeId> = app
            .slab_draft
            .nodes
            .iter()
            .map(|n| n.expect("can_add で4つとも Some を確認済み"))
            .collect();
        let value_kn_m2 = app
            .slab_draft
            .load_value
            .trim()
            .parse::<f64>()
            .unwrap_or(0.0);
        // kN/m² → N/mm²（内部単位系）。1 kN/m² = 1e-3 N/mm²。
        let value = value_kn_m2 * 1e-3;
        let kind = app.slab_draft.load_kind.trim();
        let kind = if kind.is_empty() { "DL" } else { kind }.to_string();
        app.undo.run(
            &mut app.model,
            Box::new(AddSlab {
                boundary,
                joists: Vec::new(),
                loads: vec![AreaLoad { kind, value }],
                method: app.slab_draft.method,
                usage: app.slab_draft.usage,
            }),
        );
        app.staleness.mark_edited();
    }
}
