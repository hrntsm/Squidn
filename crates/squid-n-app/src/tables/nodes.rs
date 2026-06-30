use crate::app::App;
use squid_n_core::dof::Dof6Mask;
use squid_n_core::ids::NodeId;
use squid_n_edit::{AddNode, DeleteNode, SetNodeCoord, SetNodeRestraint};

pub fn nodes_table(ui: &mut egui::Ui, app: &mut App) {
    use egui_extras::{Column, TableBuilder};

    // バッファを model に同期（長さ合わせ + 未編集セルの更新）
    app.sync_node_edit();

    // 節点追加フォーム（座標のみを扱う。境界条件は別パネルで編集する）。
    // 座標を入力してから「追加」を押すことで、その座標を持つ節点を作成する。
    ui.group(|ui| {
        ui.strong("節点を追加");
        // 左パネルが狭い場合でも「追加」ボタンが見切れないよう折り返す
        ui.horizontal_wrapped(|ui| {
            for (label, k) in [("X", 0), ("Y", 1), ("Z", 2)] {
                ui.label(label);
                let slot = &mut app.node_draft[k];
                let resp = ui.add(
                    egui::TextEdit::singleline(slot)
                        .desired_width(70.0)
                        .clip_text(false),
                );
                if slot.trim().parse::<f64>().is_err() {
                    ui.painter().rect_filled(
                        resp.rect,
                        0.0,
                        crate::theme::translucent(crate::theme::ERROR_RED, 60),
                    );
                }
            }
            if ui.button("+ 追加").clicked() {
                let mut coord = [0.0; 3];
                for (k, slot) in app.node_draft.iter().enumerate() {
                    coord[k] = slot.trim().parse::<f64>().unwrap_or(0.0);
                }
                app.undo.run(
                    &mut app.model,
                    Box::new(AddNode {
                        coord,
                        restraint: Dof6Mask::FREE,
                    }),
                );
                // model.nodes が +1 されたので node_edit の長さを再同期
                // （同期しないと body.rows が新しい行数で描画し node_edit[i] が範囲外になる）
                app.sync_node_edit();
                app.staleness.mark_edited();
            }
        });
    });
    ui.separator();

    let n = app.model.nodes.len();
    // node_edit を一時的に取り出し、クロージャ内で app.model/undo と共存させる
    let mut node_edit = std::mem::take(&mut app.node_edit);
    // 確定待ちの編集（行、列、パース結果）
    let mut pending: Vec<(usize, usize, f64)> = Vec::new();
    // 削除対象（末尾の節点のみ許可。DeleteNode の制約に合わせる）
    let mut pending_delete: Option<NodeId> = None;

    TableBuilder::new(ui)
        .striped(true)
        .column(Column::auto())
        .columns(Column::initial(80.0), 3)
        .column(Column::auto())
        .header(20.0, |mut h| {
            for t in &["ID", "X", "Y", "Z", ""] {
                h.col(|ui| {
                    ui.strong(*t);
                });
            }
        })
        .body(|body| {
            body.rows(22.0, n, |mut row| {
                let i = row.index();
                let node = &app.model.nodes[i];
                let is_focus = app.nav.focus_node == Some(node.id);
                row.col(|ui| {
                    let text = node.id.0.to_string();
                    // 選択行は blue-500 背景になるため文字は白、非選択は既定色
                    let rich = egui::RichText::new(text).color(if is_focus {
                        crate::theme::WHITE
                    } else {
                        egui::Color32::PLACEHOLDER
                    });
                    if ui.selectable_label(is_focus, rich).clicked() {
                        app.nav.focus_node = Some(node.id);
                    }
                });
                for (k, slot) in node_edit[i].iter_mut().enumerate().take(3) {
                    row.col(|ui| {
                        let resp = ui.add(
                            egui::TextEdit::singleline(slot)
                                .desired_width(70.0)
                                .clip_text(false),
                        );
                        if resp.lost_focus() && resp.changed() {
                            if let Ok(val) = slot.trim().parse::<f64>() {
                                if (val - node.coord[k]).abs() > 1e-9 {
                                    pending.push((i, k, val));
                                }
                            }
                        }
                        // 数値以外は赤背景
                        if slot.trim().parse::<f64>().is_err() {
                            ui.painter().rect_filled(
                                resp.rect,
                                0.0,
                                crate::theme::translucent(crate::theme::ERROR_RED, 60),
                            );
                        }
                    });
                }
                row.col(|ui| {
                    // 部材・節点荷重などから参照中の節点は削除すると参照が壊れるため、
                    // 先に参照を解消するまで無効化する（DeleteNode 側でも安全のため再確認する）。
                    let in_use = app.model.node_in_use(node.id);
                    let resp = ui.add_enabled(!in_use, egui::Button::new("🗑"));
                    if in_use {
                        resp.on_hover_text(
                            "この節点は部材などに使用されているため削除できません（先に参照を解消してください）",
                        );
                    } else if resp.on_hover_text("この節点を削除").clicked() {
                        pending_delete = Some(node.id);
                    }
                });
            });
        });

    // 確定処理（クロージャ外で app.model と app.undo にアクセス）
    let had_pending = !pending.is_empty();
    for (i, k, val) in pending {
        let node_id = NodeId(app.model.nodes[i].id.0);
        let mut new_coord = app.model.nodes[i].coord;
        new_coord[k] = val;
        app.undo.run(
            &mut app.model,
            Box::new(SetNodeCoord {
                node: node_id,
                coord: new_coord,
            }),
        );
    }

    let had_delete = pending_delete.is_some();
    if let Some(node_id) = pending_delete {
        app.undo
            .run(&mut app.model, Box::new(DeleteNode { id: node_id }));
        app.sync_node_edit();
        if app.nav.focus_node == Some(node_id) {
            app.nav.focus_node = None;
        }
    }

    // 編集があった場合は下流（結果・設計）を stale にする（UI設計 §5）
    if had_pending || had_delete {
        app.staleness.mark_edited();
    }

    // バッファを戻す
    app.node_edit = node_edit;

    ui.add_space(8.0);
    ui.separator();
    boundary_condition_panel(ui, app);
}

/// 境界条件（拘束）の編集パネル。節点一覧・追加フォームとは完全に独立した UI。
/// 節点を選んでから 自由／ピン／固定 やチェックボックスで拘束成分を設定する。
fn boundary_condition_panel(ui: &mut egui::Ui, app: &mut App) {
    ui.group(|ui| {
        ui.strong("境界条件");
        if app.model.nodes.is_empty() {
            ui.label("節点がありません");
            return;
        }

        let node_ids: Vec<NodeId> = app.model.nodes.iter().map(|n| n.id).collect();
        let selected = app
            .nav
            .focus_node
            .filter(|id| node_ids.contains(id))
            .unwrap_or(node_ids[0]);
        app.nav.focus_node = Some(selected);

        ui.horizontal(|ui| {
            ui.label("対象節点:");
            egui::ComboBox::from_id_salt("bc_node_select")
                .selected_text(format!("N{}", selected.0))
                .show_ui(ui, |ui| {
                    for id in &node_ids {
                        if ui
                            .selectable_label(selected == *id, format!("N{}", id.0))
                            .clicked()
                        {
                            app.nav.focus_node = Some(*id);
                        }
                    }
                });
        });

        let selected = app.nav.focus_node.unwrap_or(selected);
        let Some(node) = app.model.nodes.iter().find(|n| n.id == selected) else {
            return;
        };
        let r = node.restraint;
        let mut pending_restraint: Option<Dof6Mask> = None;

        ui.horizontal(|ui| {
            // プリセットボタン（自由／ピン／固定）
            if ui.small_button("自由").clicked() {
                pending_restraint = Some(Dof6Mask::FREE);
            }
            if ui.small_button("ピン").clicked() {
                pending_restraint = Some(Dof6Mask::PINNED);
            }
            if ui.small_button("固定").clicked() {
                pending_restraint = Some(Dof6Mask::FIXED);
            }
        });
        ui.horizontal_wrapped(|ui| {
            // 各成分チェックボックス
            use squid_n_core::dof::Dof;
            for (d, lbl) in [
                (Dof::Ux, "X"),
                (Dof::Uy, "Y"),
                (Dof::Uz, "Z"),
                (Dof::Rx, "RX"),
                (Dof::Ry, "RY"),
                (Dof::Rz, "RZ"),
            ] {
                let mut on = r.is_fixed(d);
                if ui.checkbox(&mut on, lbl).changed() {
                    let mut new_mask = r;
                    new_mask.set(d, on);
                    pending_restraint = Some(new_mask);
                }
            }
        });

        if let Some(mask) = pending_restraint {
            app.undo.run(
                &mut app.model,
                Box::new(SetNodeRestraint {
                    node: selected,
                    restraint: mask,
                }),
            );
            app.staleness.mark_edited();
        }
    });
}
