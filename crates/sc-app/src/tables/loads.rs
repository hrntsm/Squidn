use crate::app::App;
use sc_core::ids::{LoadCaseId, NodeId};
use sc_edit::{SetLoadCaseName, SetNodalLoad};

pub fn loads_table(ui: &mut egui::Ui, app: &mut App) {
    use egui_extras::{Column, TableBuilder};

    // --- 荷重ケース一覧（名称編集可能） ---
    ui.strong("荷重ケース");
    let n_lc = app.model.load_cases.len();
    let mut pending_name: Vec<(usize, String)> = Vec::new();
    let mut name_bufs: Vec<String> = app
        .model
        .load_cases
        .iter()
        .map(|lc| lc.name.clone())
        .collect();

    TableBuilder::new(ui)
        .striped(true)
        .column(Column::auto())
        .column(Column::initial(120.0))
        .column(Column::initial(60.0))
        .header(20.0, |mut h| {
            for t in &["ID", "名称", "荷重数"] {
                h.col(|ui| {
                    ui.strong(*t);
                });
            }
        })
        .body(|body| {
            body.rows(22.0, n_lc, |mut row| {
                let i = row.index();
                let lc = &app.model.load_cases[i];
                row.col(|ui| {
                    ui.label(lc.id.0.to_string());
                });
                row.col(|ui| {
                    if i < name_bufs.len() {
                        let resp = ui.add(
                            egui::TextEdit::singleline(&mut name_bufs[i])
                                .desired_width(110.0)
                                .clip_text(false),
                        );
                        if resp.lost_focus() && resp.changed() {
                            let trimmed = name_bufs[i].trim().to_string();
                            if trimmed != lc.name && !trimmed.is_empty() {
                                pending_name.push((i, trimmed));
                            }
                        }
                    }
                });
                row.col(|ui| {
                    ui.label(lc.nodal.len().to_string());
                });
            });
        });

    let had_name = !pending_name.is_empty();
    for (i, name) in pending_name {
        let lc_id = LoadCaseId(app.model.load_cases[i].id.0);
        app.undo.run(
            &mut app.model,
            Box::new(SetLoadCaseName { id: lc_id, name }),
        );
    }
    if had_name {
        app.staleness.mark_edited();
    }

    ui.add_space(8.0);

    // --- 節点荷重詳細（選択中の荷重ケース） ---
    ui.strong("節点荷重");
    if app.model.load_cases.is_empty() {
        ui.label("荷重ケースがありません");
        return;
    }
    let lc_idx = app
        .last_lc
        .and_then(|id| app.model.load_cases.iter().position(|lc| lc.id == id))
        .unwrap_or(0);
    let lc_id = app.model.load_cases[lc_idx].id;
    ui.label(format!(
        "ケース: {} ({})",
        lc_id.0, app.model.load_cases[lc_idx].name
    ));

    let nodal_count = app.model.load_cases[lc_idx].nodal.len();
    let mut pending_load: Vec<(NodeId, [f64; 6])> = Vec::new();
    let mut value_bufs: Vec<[String; 6]> = app.model.load_cases[lc_idx]
        .nodal
        .iter()
        .map(|n| n.values.map(|v| format!("{:.2}", v)))
        .collect();

    TableBuilder::new(ui)
        .striped(true)
        .column(Column::auto())
        .columns(Column::initial(70.0), 6)
        .header(20.0, |mut h| {
            h.col(|ui| {
                ui.strong("節点");
            });
            for t in &["Fx", "Fy", "Fz", "Mx", "My", "Mz"] {
                h.col(|ui| {
                    ui.strong(*t);
                });
            }
        })
        .body(|body| {
            body.rows(22.0, nodal_count, |mut row| {
                let i = row.index();
                let nodal = &app.model.load_cases[lc_idx].nodal[i];
                row.col(|ui| {
                    ui.label(nodal.node.0.to_string());
                });
                for k in 0..6 {
                    row.col(|ui| {
                        let buf = &mut value_bufs[i][k];
                        let resp = ui.add(
                            egui::TextEdit::singleline(buf)
                                .desired_width(60.0)
                                .clip_text(false),
                        );
                        if resp.lost_focus() && resp.changed() {
                            if let Ok(val) = buf.trim().parse::<f64>() {
                                if (val - nodal.values[k]).abs() > 1e-9 {
                                    let mut new_vals = nodal.values;
                                    new_vals[k] = val;
                                    pending_load.push((nodal.node, new_vals));
                                }
                            }
                        }
                        if buf.trim().parse::<f64>().is_err() {
                            ui.painter().rect_filled(
                                resp.rect,
                                0.0,
                                egui::Color32::from_rgba_unmultiplied(200, 50, 50, 60),
                            );
                        }
                    });
                }
            });
        });

    let had_load = !pending_load.is_empty();
    for (node, values) in pending_load {
        app.undo.run(
            &mut app.model,
            Box::new(SetNodalLoad {
                lc: lc_id,
                node,
                values,
            }),
        );
    }
    if had_load {
        app.staleness.mark_edited();
    }
}
