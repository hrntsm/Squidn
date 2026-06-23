use sc_core::ids::{ElemId, LoadCaseId, NodeId, SectionId};
use sc_core::model::Model;

pub trait EditCommand {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand>;
    fn label(&self) -> &str;
}

impl<T: EditCommand + 'static> From<T> for Box<dyn EditCommand> {
    fn from(cmd: T) -> Self {
        Box::new(cmd)
    }
}

pub struct UndoStack {
    done: Vec<Box<dyn EditCommand>>,
    undone: Vec<Box<dyn EditCommand>>,
    max_undo: usize,
}

impl UndoStack {
    pub fn new() -> Self {
        Self {
            done: Vec::new(),
            undone: Vec::new(),
            max_undo: 100,
        }
    }

    pub fn with_max(max_undo: usize) -> Self {
        Self {
            done: Vec::new(),
            undone: Vec::new(),
            max_undo,
        }
    }

    pub fn run(&mut self, model: &mut Model, cmd: Box<dyn EditCommand>) {
        let inv = cmd.apply(model);
        self.done.push(inv);
        if self.done.len() > self.max_undo {
            self.done.remove(0);
        }
        self.undone.clear();
    }

    pub fn undo(&mut self, model: &mut Model) {
        if let Some(cmd) = self.done.pop() {
            let redo_cmd = cmd.apply(model);
            self.undone.push(redo_cmd);
        }
    }

    pub fn redo(&mut self, model: &mut Model) {
        if let Some(cmd) = self.undone.pop() {
            let undo_cmd = cmd.apply(model);
            self.done.push(undo_cmd);
        }
    }

    pub fn can_undo(&self) -> bool {
        !self.done.is_empty()
    }

    pub fn can_redo(&self) -> bool {
        !self.undone.is_empty()
    }

    pub fn undo_label(&self) -> Option<&str> {
        self.done.last().map(|c| c.label())
    }

    pub fn redo_label(&self) -> Option<&str> {
        self.undone.last().map(|c| c.label())
    }
}

impl Default for UndoStack {
    fn default() -> Self {
        Self::new()
    }
}

pub fn push_edit_command(model: &mut Model, stack: &mut UndoStack, cmd: Box<dyn EditCommand>) {
    stack.run(model, cmd);
}

/// 節点座標変更。
pub struct SetNodeCoord {
    pub node: NodeId,
    pub coord: [f64; 3],
}

impl EditCommand for SetNodeCoord {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.node.index();
        if idx >= model.nodes.len() || model.nodes[idx].id != self.node {
            return Box::new(Noop);
        }
        let old_coord = model.nodes[idx].coord;
        model.nodes[idx].coord = self.coord;
        Box::new(SetNodeCoord {
            node: self.node,
            coord: old_coord,
        })
    }

    fn label(&self) -> &str {
        "節点座標変更"
    }
}

/// 節点拘束（支点条件）変更。逆操作は変更前マスクへの復元。
pub struct SetNodeRestraint {
    pub node: NodeId,
    pub restraint: sc_core::dof::Dof6Mask,
}

impl EditCommand for SetNodeRestraint {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.node.index();
        if idx >= model.nodes.len() || model.nodes[idx].id != self.node {
            return Box::new(Noop);
        }
        let old = model.nodes[idx].restraint;
        model.nodes[idx].restraint = self.restraint;
        Box::new(SetNodeRestraint {
            node: self.node,
            restraint: old,
        })
    }

    fn label(&self) -> &str {
        "節点拘束変更"
    }
}

/// 節点追加。末尾に `NodeId(len)` で追加する（ID＝配列インデックスの不変条件を維持）。
/// 逆操作は節点削除。
pub struct AddNode {
    pub coord: [f64; 3],
    pub restraint: sc_core::dof::Dof6Mask,
}

impl EditCommand for AddNode {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let new_id = NodeId(model.nodes.len() as u32);
        model.nodes.push(sc_core::model::Node {
            id: new_id,
            coord: self.coord,
            restraint: self.restraint,
            mass: None,
            story: None,
        });
        Box::new(DeleteNode { id: new_id })
    }

    fn label(&self) -> &str {
        "節点追加"
    }
}

/// 節点削除。逆操作は節点追加。
///
/// ID＝配列インデックスの不変条件を保つため、末尾の節点のみ削除可能とする
/// （中間節点の削除は ID 再採番と部材参照の更新が必要なため未対応）。
pub struct DeleteNode {
    pub id: NodeId,
}

impl EditCommand for DeleteNode {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        // 末尾以外、または ID 不一致は不正参照として Noop
        if idx + 1 != model.nodes.len() || model.nodes[idx].id != self.id {
            return Box::new(Noop);
        }
        let removed = model.nodes.remove(idx);
        Box::new(AddNode {
            coord: removed.coord,
            restraint: removed.restraint,
        })
    }

    fn label(&self) -> &str {
        "節点削除"
    }
}

/// 部材追加。逆操作は部材削除。
pub struct AddMember {
    pub elem: sc_core::model::ElementData,
}

impl EditCommand for AddMember {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        model.elements.push(self.elem.clone());
        Box::new(DeleteMember { id: self.elem.id })
    }

    fn label(&self) -> &str {
        "部材追加"
    }
}

/// 部材削除。逆操作は部材追加。
pub struct DeleteMember {
    pub id: ElemId,
}

impl EditCommand for DeleteMember {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.elements.len() || model.elements[idx].id != self.id {
            return Box::new(Noop);
        }
        let removed = model.elements.remove(idx);
        Box::new(AddMember { elem: removed })
    }

    fn label(&self) -> &str {
        "部材削除"
    }
}

/// 何もしないコマンド（参照不正時の安全なフォールバック）。
pub struct Noop;

impl EditCommand for Noop {
    fn apply(&self, _model: &mut Model) -> Box<dyn EditCommand> {
        Box::new(Noop)
    }

    fn label(&self) -> &str {
        "Noop"
    }
}

/// 断面プロパティ変更（名称・A・Iy・Iz・J 等）。
pub struct SetSectionField {
    pub id: SectionId,
    pub field: SectionField,
    pub value: f64,
}

/// 編集対象の断面プロパティ。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SectionField {
    Area,
    Iy,
    Iz,
    J,
    Depth,
    Width,
    AsY,
    AsZ,
}

impl EditCommand for SetSectionField {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.sections.len() || model.sections[idx].id != self.id {
            return Box::new(Noop);
        }
        let sec = &mut model.sections[idx];
        let old = match self.field {
            SectionField::Area => {
                let o = sec.area;
                sec.area = self.value;
                o
            }
            SectionField::Iy => {
                let o = sec.iy;
                sec.iy = self.value;
                o
            }
            SectionField::Iz => {
                let o = sec.iz;
                sec.iz = self.value;
                o
            }
            SectionField::J => {
                let o = sec.j;
                sec.j = self.value;
                o
            }
            SectionField::Depth => {
                let o = sec.depth;
                sec.depth = self.value;
                o
            }
            SectionField::Width => {
                let o = sec.width;
                sec.width = self.value;
                o
            }
            SectionField::AsY => {
                let o = sec.as_y;
                sec.as_y = self.value;
                o
            }
            SectionField::AsZ => {
                let o = sec.as_z;
                sec.as_z = self.value;
                o
            }
        };
        Box::new(SetSectionField {
            id: self.id,
            field: self.field,
            value: old,
        })
    }

    fn label(&self) -> &str {
        "断面プロパティ変更"
    }
}

/// 断面名称変更。
pub struct SetSectionName {
    pub id: SectionId,
    pub name: String,
}

impl EditCommand for SetSectionName {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.sections.len() || model.sections[idx].id != self.id {
            return Box::new(Noop);
        }
        let old = std::mem::replace(&mut model.sections[idx].name, self.name.clone());
        Box::new(SetSectionName {
            id: self.id,
            name: old,
        })
    }

    fn label(&self) -> &str {
        "断面名称変更"
    }
}

/// 部材の断面割当変更。
pub struct SetElementSection {
    pub elem: ElemId,
    pub section: Option<SectionId>,
}

impl EditCommand for SetElementSection {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.elem.index();
        if idx >= model.elements.len() || model.elements[idx].id != self.elem {
            return Box::new(Noop);
        }
        let old = model.elements[idx].section;
        model.elements[idx].section = self.section;
        Box::new(SetElementSection {
            elem: self.elem,
            section: old,
        })
    }

    fn label(&self) -> &str {
        "部材断面割当変更"
    }
}

/// 部材の材料割当変更。
pub struct SetElementMaterial {
    pub elem: ElemId,
    pub material: Option<sc_core::ids::MaterialId>,
}

impl EditCommand for SetElementMaterial {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.elem.index();
        if idx >= model.elements.len() || model.elements[idx].id != self.elem {
            return Box::new(Noop);
        }
        let old = model.elements[idx].material;
        model.elements[idx].material = self.material;
        Box::new(SetElementMaterial {
            elem: self.elem,
            material: old,
        })
    }

    fn label(&self) -> &str {
        "部材材料割当変更"
    }
}

/// 荷重ケース名変更。
pub struct SetLoadCaseName {
    pub id: LoadCaseId,
    pub name: String,
}

impl EditCommand for SetLoadCaseName {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.load_cases.len() || model.load_cases[idx].id != self.id {
            return Box::new(Noop);
        }
        let old = std::mem::replace(&mut model.load_cases[idx].name, self.name.clone());
        Box::new(SetLoadCaseName {
            id: self.id,
            name: old,
        })
    }

    fn label(&self) -> &str {
        "荷重ケース名変更"
    }
}

/// 節点荷重値変更（6成分）。
pub struct SetNodalLoad {
    pub lc: LoadCaseId,
    pub node: NodeId,
    pub values: [f64; 6],
}

impl EditCommand for SetNodalLoad {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let lc_idx = self.lc.index();
        if lc_idx >= model.load_cases.len() || model.load_cases[lc_idx].id != self.lc {
            return Box::new(Noop);
        }
        let nodal = &mut model.load_cases[lc_idx].nodal;
        if let Some(entry) = nodal.iter_mut().find(|n| n.node == self.node) {
            let old = entry.values;
            entry.values = self.values;
            Box::new(SetNodalLoad {
                lc: self.lc,
                node: self.node,
                values: old,
            })
        } else {
            nodal.push(sc_core::model::NodalLoad {
                node: self.node,
                values: self.values,
            });
            Box::new(DeleteNodalLoad {
                lc: self.lc,
                node: self.node,
            })
        }
    }

    fn label(&self) -> &str {
        "節点荷重変更"
    }
}

/// 節点荷重削除。
pub struct DeleteNodalLoad {
    pub lc: LoadCaseId,
    pub node: NodeId,
}

impl EditCommand for DeleteNodalLoad {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let lc_idx = self.lc.index();
        if lc_idx >= model.load_cases.len() || model.load_cases[lc_idx].id != self.lc {
            return Box::new(Noop);
        }
        let nodal = &mut model.load_cases[lc_idx].nodal;
        if let Some(pos) = nodal.iter().position(|n| n.node == self.node) {
            let removed = nodal.remove(pos);
            Box::new(SetNodalLoad {
                lc: self.lc,
                node: removed.node,
                values: removed.values,
            })
        } else {
            Box::new(Noop)
        }
    }

    fn label(&self) -> &str {
        "節点荷重削除"
    }
}

/// 部材（梁）荷重を荷重ケースへ追加。逆操作は末尾要素の削除。
pub struct AddMemberLoad {
    pub lc: LoadCaseId,
    pub load: sc_core::model::MemberLoad,
}

impl EditCommand for AddMemberLoad {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.lc.index();
        if idx >= model.load_cases.len() || model.load_cases[idx].id != self.lc {
            return Box::new(Noop);
        }
        model.load_cases[idx].member.push(self.load.clone());
        let pos = model.load_cases[idx].member.len() - 1;
        Box::new(DeleteMemberLoad {
            lc: self.lc,
            index: pos,
        })
    }

    fn label(&self) -> &str {
        "部材荷重追加"
    }
}

/// 部材荷重を index 指定で削除。逆操作は同位置への挿入。
pub struct DeleteMemberLoad {
    pub lc: LoadCaseId,
    pub index: usize,
}

impl EditCommand for DeleteMemberLoad {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.lc.index();
        if idx >= model.load_cases.len() || model.load_cases[idx].id != self.lc {
            return Box::new(Noop);
        }
        let member = &mut model.load_cases[idx].member;
        if self.index >= member.len() {
            return Box::new(Noop);
        }
        let removed = member.remove(self.index);
        Box::new(InsertMemberLoad {
            lc: self.lc,
            index: self.index,
            load: removed,
        })
    }

    fn label(&self) -> &str {
        "部材荷重削除"
    }
}

/// 部材荷重を index 位置へ挿入（DeleteMemberLoad の逆操作）。
pub struct InsertMemberLoad {
    pub lc: LoadCaseId,
    pub index: usize,
    pub load: sc_core::model::MemberLoad,
}

impl EditCommand for InsertMemberLoad {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.lc.index();
        if idx >= model.load_cases.len() || model.load_cases[idx].id != self.lc {
            return Box::new(Noop);
        }
        let member = &mut model.load_cases[idx].member;
        if self.index > member.len() {
            return Box::new(Noop);
        }
        member.insert(self.index, self.load.clone());
        Box::new(DeleteMemberLoad {
            lc: self.lc,
            index: self.index,
        })
    }

    fn label(&self) -> &str {
        "部材荷重挿入"
    }
}

/// 断面形状を新規追加（UI-3 の新規断面作成）。
pub struct AddSectionShape {
    pub shape: sc_section::shape::SectionShape,
    pub new_id: SectionId,
    pub name: String,
}

impl EditCommand for AddSectionShape {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let sec = self.shape.to_section(self.new_id, self.name.clone());
        model.sections.push(sec);
        Box::new(DeleteSection { id: self.new_id })
    }

    fn label(&self) -> &str {
        "断面形状追加"
    }
}

/// 断面形状変更。逆操作は RestoreSection。
pub struct EditSectionShape {
    pub section: SectionId,
    pub new_shape: sc_section::shape::SectionShape,
}

impl EditCommand for EditSectionShape {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.section.index();
        if idx >= model.sections.len() || model.sections[idx].id != self.section {
            return Box::new(Noop);
        }
        let old = model.sections[idx].clone();
        let new_sec = self.new_shape.to_section(self.section, old.name.clone());
        model.sections[idx] = new_sec;
        Box::new(RestoreSection { old })
    }

    fn label(&self) -> &str {
        "断面形状変更"
    }
}

/// 断面データを指定した Section で復元する（EditSectionShape の逆操作）。
pub struct RestoreSection {
    pub old: sc_core::model::Section,
}

impl EditCommand for RestoreSection {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.old.id.index();
        if idx >= model.sections.len() || model.sections[idx].id != self.old.id {
            return Box::new(Noop);
        }
        let replaced = std::mem::replace(&mut model.sections[idx], self.old.clone());
        Box::new(RestoreSection { old: replaced })
    }

    fn label(&self) -> &str {
        "断面復元"
    }
}

/// 部材が参照する断面を複製し、部材に新断面を割り当てる。
pub struct DuplicateSectionForMember {
    pub member: ElemId,
}

impl EditCommand for DuplicateSectionForMember {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let elem_idx = self.member.index();
        if elem_idx >= model.elements.len() || model.elements[elem_idx].id != self.member {
            return Box::new(Noop);
        }
        let sid = match model.elements[elem_idx].section {
            Some(s) => s,
            None => return Box::new(Noop),
        };
        let sec_idx = sid.index();
        if sec_idx >= model.sections.len() || model.sections[sec_idx].id != sid {
            return Box::new(Noop);
        }
        let orig = &model.sections[sec_idx];
        let new_id = SectionId(model.sections.len() as u32);
        let mut new_sec = orig.clone();
        new_sec.id = new_id;
        new_sec.name = format!("{}(複製)", orig.name);
        model.sections.push(new_sec);
        model.elements[elem_idx].section = Some(new_id);
        Box::new(RestoreElementSectionAndDeleteSection {
            elem: self.member,
            old_section: Some(sid),
            new_section: new_id,
        })
    }

    fn label(&self) -> &str {
        "部材断面複製"
    }
}

/// DuplicateSectionForMember の逆操作。
pub struct RestoreElementSectionAndDeleteSection {
    pub elem: ElemId,
    pub old_section: Option<SectionId>,
    pub new_section: SectionId,
}

impl EditCommand for RestoreElementSectionAndDeleteSection {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let elem_idx = self.elem.index();
        if elem_idx >= model.elements.len() || model.elements[elem_idx].id != self.elem {
            return Box::new(Noop);
        }
        let new_idx = self.new_section.index();
        if new_idx >= model.sections.len() || model.sections[new_idx].id != self.new_section {
            return Box::new(Noop);
        }
        model.elements[elem_idx].section = self.old_section;
        model.sections.remove(new_idx);
        Box::new(DuplicateSectionForMember { member: self.elem })
    }

    fn label(&self) -> &str {
        "部材断面複製解除"
    }
}

/// 断面削除。逆操作は AddSection。
pub struct DeleteSection {
    pub id: SectionId,
}

impl EditCommand for DeleteSection {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.sections.len() || model.sections[idx].id != self.id {
            return Box::new(Noop);
        }
        let removed = model.sections.remove(idx);
        Box::new(AddSection {
            old: removed,
            index: idx,
        })
    }

    fn label(&self) -> &str {
        "断面削除"
    }
}

/// 断面追加（DeleteSection の逆操作）。
pub struct AddSection {
    pub old: sc_core::model::Section,
    pub index: usize,
}

impl EditCommand for AddSection {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.index > model.sections.len() {
            return Box::new(Noop);
        }
        model.sections.insert(self.index, self.old.clone());
        Box::new(DeleteSection { id: self.old.id })
    }

    fn label(&self) -> &str {
        "断面追加"
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sc_core::dof::Dof6Mask;
    use sc_core::ids::NodeId;
    use sc_core::model::{ElementData, ElementKind, EndCondition, ForceRegime, LocalAxis, Node};
    use smallvec::smallvec;

    fn empty_model() -> Model {
        Model::default()
    }

    #[test]
    fn test_set_node_coord_roundtrip() {
        let mut model = empty_model();
        model.nodes.push(Node {
            id: NodeId(0),
            coord: [0.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        });
        let mut stack = UndoStack::new();

        let cmd = SetNodeCoord {
            node: NodeId(0),
            coord: [1000.0, 2000.0, 0.0],
        };
        stack.run(&mut model, Box::new(cmd));
        assert_eq!(model.nodes[0].coord, [1000.0, 2000.0, 0.0]);

        stack.undo(&mut model);
        assert_eq!(model.nodes[0].coord, [0.0, 0.0, 0.0]);

        stack.redo(&mut model);
        assert_eq!(model.nodes[0].coord, [1000.0, 2000.0, 0.0]);
    }

    #[test]
    fn test_set_node_coord_invalid_id_is_noop() {
        let mut model = empty_model();
        let mut stack = UndoStack::new();
        stack.run(
            &mut model,
            Box::new(SetNodeCoord {
                node: NodeId(99),
                coord: [1.0, 2.0, 3.0],
            }),
        );
        assert!(stack.can_undo());
        stack.undo(&mut model);
        assert!(model.nodes.is_empty());
    }

    #[test]
    fn test_set_node_restraint_roundtrip() {
        let mut model = empty_model();
        model.nodes.push(Node {
            id: NodeId(0),
            coord: [0.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        });
        let mut stack = UndoStack::new();

        stack.run(
            &mut model,
            Box::new(SetNodeRestraint {
                node: NodeId(0),
                restraint: Dof6Mask::PINNED,
            }),
        );
        assert_eq!(model.nodes[0].restraint, Dof6Mask::PINNED);

        stack.undo(&mut model);
        assert_eq!(model.nodes[0].restraint, Dof6Mask::FREE);

        stack.redo(&mut model);
        assert_eq!(model.nodes[0].restraint, Dof6Mask::PINNED);
    }

    #[test]
    fn test_set_node_restraint_invalid_id_is_noop() {
        let mut model = empty_model();
        let mut stack = UndoStack::new();
        stack.run(
            &mut model,
            Box::new(SetNodeRestraint {
                node: NodeId(99),
                restraint: Dof6Mask::FIXED,
            }),
        );
        assert!(stack.can_undo());
        stack.undo(&mut model);
        assert!(model.nodes.is_empty());
    }

    #[test]
    fn test_add_node_roundtrip() {
        let mut model = empty_model();
        let mut stack = UndoStack::new();

        stack.run(
            &mut model,
            Box::new(AddNode {
                coord: [1000.0, 2000.0, 3000.0],
                restraint: Dof6Mask::FREE,
            }),
        );
        assert_eq!(model.nodes.len(), 1);
        assert_eq!(model.nodes[0].id, NodeId(0));
        assert_eq!(model.nodes[0].coord, [1000.0, 2000.0, 3000.0]);

        stack.undo(&mut model);
        assert_eq!(model.nodes.len(), 0);

        stack.redo(&mut model);
        assert_eq!(model.nodes.len(), 1);
        assert_eq!(model.nodes[0].id, NodeId(0));
    }

    #[test]
    fn test_add_node_id_equals_index() {
        let mut model = empty_model();
        let mut stack = UndoStack::new();
        for i in 0..3 {
            stack.run(
                &mut model,
                Box::new(AddNode {
                    coord: [i as f64, 0.0, 0.0],
                    restraint: Dof6Mask::FREE,
                }),
            );
        }
        for (i, node) in model.nodes.iter().enumerate() {
            assert_eq!(node.id, NodeId(i as u32));
        }
    }

    #[test]
    fn test_delete_node_non_tail_is_noop() {
        let mut model = empty_model();
        let mut stack = UndoStack::new();
        model.nodes.push(Node {
            id: NodeId(0),
            coord: [0.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        });
        model.nodes.push(Node {
            id: NodeId(1),
            coord: [1.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        });
        // 中間節点（末尾でない）の削除は Noop
        stack.run(&mut model, Box::new(DeleteNode { id: NodeId(0) }));
        assert_eq!(model.nodes.len(), 2);
    }

    #[test]
    fn test_add_delete_member_load_roundtrip() {
        use sc_core::ids::LoadCaseId;
        use sc_core::model::{LoadCase, MemberLoad, MemberLoadKind};
        let mut model = empty_model();
        model.load_cases.push(LoadCase {
            id: LoadCaseId(0),
            name: "lc".into(),
            nodal: vec![],
            member: vec![],
        });
        let mut stack = UndoStack::new();
        let load = MemberLoad {
            elem: sc_core::ids::ElemId(0),
            dir: [0.0, 0.0, -1.0],
            kind: MemberLoadKind::Distributed {
                a: 0.0,
                b: 1000.0,
                w1: 2.0,
                w2: 2.0,
            },
        };
        stack.run(
            &mut model,
            Box::new(AddMemberLoad {
                lc: LoadCaseId(0),
                load: load.clone(),
            }),
        );
        assert_eq!(model.load_cases[0].member.len(), 1);
        assert_eq!(model.load_cases[0].member[0], load);

        stack.undo(&mut model);
        assert_eq!(model.load_cases[0].member.len(), 0);

        stack.redo(&mut model);
        assert_eq!(model.load_cases[0].member.len(), 1);

        // 削除と復元（位置保持）
        stack.run(
            &mut model,
            Box::new(DeleteMemberLoad {
                lc: LoadCaseId(0),
                index: 0,
            }),
        );
        assert_eq!(model.load_cases[0].member.len(), 0);
        stack.undo(&mut model);
        assert_eq!(model.load_cases[0].member.len(), 1);
        assert_eq!(model.load_cases[0].member[0], load);
    }

    #[test]
    fn test_add_delete_member_roundtrip() {
        let mut model = empty_model();
        let mut stack = UndoStack::new();
        let elem = ElementData {
            id: sc_core::ids::ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec![NodeId(0), NodeId(1)],
            section: None,
            material: None,
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
        };
        stack.run(&mut model, Box::new(AddMember { elem }));
        assert_eq!(model.elements.len(), 1);
        stack.undo(&mut model);
        assert_eq!(model.elements.len(), 0);
        stack.redo(&mut model);
        assert_eq!(model.elements.len(), 1);
    }

    #[test]
    fn test_add_section_shape_roundtrip() {
        let mut model = empty_model();
        let mut stack = UndoStack::new();
        let shape = sc_section::shape::SectionShape::SteelH {
            height: 300.0,
            width: 300.0,
            web_thick: 10.0,
            flange_thick: 15.0,
        };
        let cmd = AddSectionShape {
            shape,
            new_id: SectionId(0),
            name: "H-300x300x10x15".into(),
        };
        stack.run(&mut model, Box::new(cmd));
        assert_eq!(model.sections.len(), 1);
        assert_eq!(model.sections[0].id, SectionId(0));

        stack.undo(&mut model);
        assert_eq!(model.sections.len(), 0);

        stack.redo(&mut model);
        assert_eq!(model.sections.len(), 1);
        assert_eq!(model.sections[0].id, SectionId(0));
    }

    #[test]
    fn test_edit_section_shape_roundtrip() {
        let mut model = empty_model();
        let mut stack = UndoStack::new();

        let shape1 = sc_section::shape::SectionShape::SteelH {
            height: 300.0,
            width: 300.0,
            web_thick: 10.0,
            flange_thick: 15.0,
        };
        let sec = shape1.to_section(SectionId(0), "H-300".into());
        let area_h = sec.area;
        model.sections.push(sec);

        let shape2 = sc_section::shape::SectionShape::SteelBox {
            height: 200.0,
            width: 200.0,
            thick: 12.0,
        };
        let cmd = EditSectionShape {
            section: SectionId(0),
            new_shape: shape2,
        };
        stack.run(&mut model, Box::new(cmd));
        assert!((model.sections[0].area - 9024.0).abs() < 1.0);

        stack.undo(&mut model);
        assert!((model.sections[0].area - area_h).abs() < 1.0);

        stack.redo(&mut model);
        assert!((model.sections[0].area - 9024.0).abs() < 1.0);
    }

    #[test]
    fn test_duplicate_section_for_member_roundtrip() {
        let mut model = empty_model();
        let mut stack = UndoStack::new();

        let shape = sc_section::shape::SectionShape::SteelH {
            height: 300.0,
            width: 300.0,
            web_thick: 10.0,
            flange_thick: 15.0,
        };
        let sec = shape.to_section(SectionId(0), "H-300".into());
        model.sections.push(sec);

        model.nodes.push(Node {
            id: NodeId(0),
            coord: [0.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        });
        model.nodes.push(Node {
            id: NodeId(1),
            coord: [1000.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        });
        model.elements.push(ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec![NodeId(0), NodeId(1)],
            section: Some(SectionId(0)),
            material: None,
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
        });

        let cmd = DuplicateSectionForMember { member: ElemId(0) };
        stack.run(&mut model, Box::new(cmd));
        assert_eq!(model.sections.len(), 2);
        assert_eq!(model.elements[0].section, Some(SectionId(1)));

        stack.undo(&mut model);
        assert_eq!(model.sections.len(), 1);
        assert_eq!(model.elements[0].section, Some(SectionId(0)));

        stack.redo(&mut model);
        assert_eq!(model.sections.len(), 2);
        assert_eq!(model.elements[0].section, Some(SectionId(1)));
    }

    #[test]
    fn test_edit_section_shape_invalid_id_noop() {
        let mut model = empty_model();
        let mut stack = UndoStack::new();

        let shape = sc_section::shape::SectionShape::SteelBox {
            height: 200.0,
            width: 200.0,
            thick: 12.0,
        };
        let cmd = EditSectionShape {
            section: SectionId(99),
            new_shape: shape,
        };
        stack.run(&mut model, Box::new(cmd));
        assert!(stack.can_undo());
        stack.undo(&mut model);
        assert!(model.sections.is_empty());
    }

    #[test]
    fn test_delete_add_section_roundtrip() {
        let mut model = empty_model();
        let mut stack = UndoStack::new();

        let shape = sc_section::shape::SectionShape::SteelH {
            height: 300.0,
            width: 300.0,
            web_thick: 10.0,
            flange_thick: 15.0,
        };
        let sec = shape.to_section(SectionId(0), "H-300".into());
        model.sections.push(sec);

        let cmd = DeleteSection { id: SectionId(0) };
        stack.run(&mut model, Box::new(cmd));
        assert_eq!(model.sections.len(), 0);

        stack.undo(&mut model);
        assert_eq!(model.sections.len(), 1);
        assert_eq!(model.sections[0].id, SectionId(0));

        stack.redo(&mut model);
        assert_eq!(model.sections.len(), 0);
    }

    #[test]
    fn test_duplicate_section_no_section_noop() {
        let mut model = empty_model();
        let mut stack = UndoStack::new();

        model.nodes.push(Node {
            id: NodeId(0),
            coord: [0.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        });
        model.nodes.push(Node {
            id: NodeId(1),
            coord: [1000.0, 0.0, 0.0],
            restraint: Dof6Mask::FREE,
            mass: None,
            story: None,
        });
        model.elements.push(ElementData {
            id: ElemId(0),
            kind: ElementKind::Beam,
            nodes: smallvec![NodeId(0), NodeId(1)],
            section: None,
            material: None,
            local_axis: LocalAxis {
                ref_vector: [1.0, 0.0, 0.0],
            },
            end_cond: [EndCondition::Fixed, EndCondition::Fixed],
            force_regime: ForceRegime::Auto,
            rigid_zone: Default::default(),
        });

        let cmd = DuplicateSectionForMember { member: ElemId(0) };
        stack.run(&mut model, Box::new(cmd));
        assert!(stack.can_undo());
        stack.undo(&mut model);
    }
}
