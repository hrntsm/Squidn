//! 荷重（荷重ケース名・節点荷重・部材荷重）の編集コマンド。

use super::*;
use squid_n_core::ids::*;

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
            nodal.push(squid_n_core::model::NodalLoad {
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
    pub load: squid_n_core::model::MemberLoad,
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
    pub load: squid_n_core::model::MemberLoad,
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
