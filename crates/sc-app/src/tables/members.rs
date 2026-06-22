use crate::app::App;
use sc_core::ids::SectionId;
use sc_edit::SetElementSection;

pub fn members_table(ui: &mut egui::Ui, app: &mut App) {
    use egui_extras::{Column, TableBuilder};

    let n = app.model.elements.len();
    let mut pending_section: Vec<(usize, u32)> = Vec::new();

    TableBuilder::new(ui)
        .striped(true)
        .column(Column::auto())
        .column(Column::auto())
        .column(Column::initial(90.0))
        .column(Column::initial(60.0))
        .header(20.0, |mut h| {
            for t in &["ID", "種別", "節点", "断面"] {
                h.col(|ui| {
                    ui.strong(*t);
                });
            }
        })
        .body(|body| {
            body.rows(22.0, n, |mut row| {
                let i = row.index();
                let elem = &app.model.elements[i];
                let is_focus = app.nav.focus_member == Some(elem.id);
                row.col(|ui| {
                    let text = elem.id.0.to_string();
                    let rich = egui::RichText::new(text).color(if is_focus {
                        egui::Color32::from_rgb(40, 80, 200)
                    } else {
                        egui::Color32::PLACEHOLDER
                    });
                    if ui.selectable_label(is_focus, rich).clicked() {
                        app.nav.focus_member = Some(elem.id);
                    }
                });
                row.col(|ui| {
                    ui.label(format!("{:?}", elem.kind));
                });
                row.col(|ui| {
                    let ids: Vec<String> = elem.nodes.iter().map(|n| n.0.to_string()).collect();
                    ui.label(ids.join(","));
                });
                row.col(|ui| {
                    let current = elem.section.map(|s| s.0).unwrap_or(u32::MAX);
                    let selected = current;
                    let combo = egui::ComboBox::from_id_salt(format!("elem_sec_{}", i))
                        .selected_text(
                            elem.section
                                .map(|s| format!("S{}", s.0))
                                .unwrap_or_else(|| "―".to_string()),
                        );
                    combo.show_ui(ui, |ui| {
                        if ui.selectable_label(selected == u32::MAX, "―").clicked() {
                            pending_section.push((i, u32::MAX));
                        }
                        for sec in &app.model.sections {
                            if ui
                                .selectable_label(selected == sec.id.0, format!("S{}", sec.id.0))
                                .clicked()
                            {
                                pending_section.push((i, sec.id.0));
                            }
                        }
                    });
                });
            });
        });

    // 確定処理
    let had_pending = !pending_section.is_empty();
    for (i, sec_id) in pending_section {
        let elem_id = app.model.elements[i].id;
        let section = if sec_id == u32::MAX {
            None
        } else {
            // 参照先が存在するか確認
            let sid = SectionId(sec_id);
            if app.model.sections.iter().any(|s| s.id == sid) {
                Some(sid)
            } else {
                None
            }
        };
        app.undo.run(
            &mut app.model,
            Box::new(SetElementSection {
                elem: elem_id,
                section,
            }),
        );
    }

    // 編集があった場合は下流（結果・設計）を stale にする（UI設計 §5）
    if had_pending {
        app.staleness.mark_edited();
    }
}
