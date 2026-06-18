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
}
