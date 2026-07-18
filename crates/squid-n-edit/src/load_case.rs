//! 荷重ケース・荷重組み合わせ・階・スラブの編集コマンド。

use super::*;
use squid_n_core::ids::*;

/// 荷重ケース追加。末尾に `LoadCaseId(len)` で追加する。逆操作は荷重ケース削除。
pub struct AddLoadCase {
    pub name: String,
}

impl EditCommand for AddLoadCase {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let new_id = LoadCaseId(model.load_cases.len() as u32);
        model.load_cases.push(squid_n_core::model::LoadCase {
            kind: Default::default(),
            id: new_id,
            name: self.name.clone(),
            nodal: Vec::new(),
            member: Vec::new(),
        });
        Box::new(DeleteLoadCase { id: new_id })
    }

    fn label(&self) -> &str {
        "荷重ケース追加"
    }
}

/// 荷重ケース削除（中身の節点荷重・部材荷重ごと削除し、undo で復元する）。
/// 荷重組合せから参照中のケースは Noop。
/// ID＝配列インデックスの不変条件を保つため、後続のケース ID と組合せからの参照を繰り上げる。
pub struct DeleteLoadCase {
    pub id: LoadCaseId,
}

impl EditCommand for DeleteLoadCase {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.load_cases.len() || model.load_cases[idx].id != self.id {
            return Box::new(Noop);
        }
        if model
            .combinations
            .iter()
            .any(|c| c.terms.iter().any(|(lc, _)| *lc == self.id))
        {
            return Box::new(Noop);
        }
        let removed = model.load_cases.remove(idx);
        shift_load_case_ids(model, |lcid| {
            if lcid.0 > self.id.0 {
                lcid.0 -= 1;
            }
        });
        Box::new(InsertLoadCase {
            index: idx,
            old: removed,
        })
    }

    fn label(&self) -> &str {
        "荷重ケース削除"
    }
}

/// 指定インデックスへ荷重ケースを再挿入する（[`DeleteLoadCase`] の逆操作専用）。
pub struct InsertLoadCase {
    pub index: usize,
    pub old: squid_n_core::model::LoadCase,
}

impl EditCommand for InsertLoadCase {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.index > model.load_cases.len() {
            return Box::new(Noop);
        }
        let id = LoadCaseId(self.index as u32);
        shift_load_case_ids(model, |lcid| {
            if lcid.0 >= id.0 {
                lcid.0 += 1;
            }
        });
        let mut lc = self.old.clone();
        lc.id = id;
        model.load_cases.insert(self.index, lc);
        Box::new(DeleteLoadCase { id })
    }

    fn label(&self) -> &str {
        "荷重ケース削除の取り消し"
    }
}

/// モデル内の全ての `LoadCaseId` 参照（ケース自身の ID を含む）に `f` を適用する。
fn shift_load_case_ids(model: &mut Model, mut f: impl FnMut(&mut LoadCaseId)) {
    for lc in &mut model.load_cases {
        f(&mut lc.id);
    }
    for combo in &mut model.combinations {
        for (lcid, _) in &mut combo.terms {
            f(lcid);
        }
    }
}

/// 荷重組合せ追加。末尾に追加する。逆操作は末尾の組合せ削除。
///
/// `LoadCombination` は ID を持たず配列インデックスのみで管理されるため、
/// 他の追加系コマンド（[`AddLoadCase`] 等）と異なり ID 採番は発生しない。
/// 参照する `LoadCaseId` の存在チェックは行わない（[`Model::validate`] も
/// 組合せの `LoadCaseId` 参照はダングリングチェックの対象外であり、既存の
/// [`DeleteLoadCase`] が参照側で削除を防ぐことで整合性を保っている）。
pub struct AddCombination {
    pub combo: squid_n_core::model::LoadCombination,
}

impl EditCommand for AddCombination {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        model.combinations.push(self.combo.clone());
        let index = model.combinations.len() - 1;
        Box::new(DeleteCombination { index })
    }

    fn label(&self) -> &str {
        "荷重組合せ追加"
    }
}

/// 荷重組合せを index 指定で削除。逆操作は [`InsertCombination`]（同じ位置への復元）。
/// 組合せは他のデータから参照されないため ID 再採番は不要。index が範囲外なら Noop。
pub struct DeleteCombination {
    pub index: usize,
}

impl EditCommand for DeleteCombination {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.index >= model.combinations.len() {
            return Box::new(Noop);
        }
        let removed = model.combinations.remove(self.index);
        Box::new(InsertCombination {
            index: self.index,
            combo: removed,
        })
    }

    fn label(&self) -> &str {
        "荷重組合せ削除"
    }
}

/// 指定インデックスへ荷重組合せを再挿入する（[`DeleteCombination`] の逆操作専用）。
pub struct InsertCombination {
    pub index: usize,
    pub combo: squid_n_core::model::LoadCombination,
}

impl EditCommand for InsertCombination {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.index > model.combinations.len() {
            return Box::new(Noop);
        }
        model.combinations.insert(self.index, self.combo.clone());
        Box::new(DeleteCombination { index: self.index })
    }

    fn label(&self) -> &str {
        "荷重組合せ削除の取り消し"
    }
}

/// 階定義の一括適用（階自動生成の結果を反映する）。
///
/// `model.stories`・各節点の所属階・剛床拘束(`Constraint::RigidDiaphragm`)を
/// まとめて差し替える。既存の RigidDiaphragm 拘束は除去し、Mpc / RigidLink は
/// 保持する。逆操作は差し替え前の状態の復元。
pub struct ApplyStories {
    pub stories: Vec<squid_n_core::model::Story>,
    /// `model.nodes` と同順の所属階。長さが合わない分は無視する。
    pub node_story: Vec<Option<squid_n_core::ids::StoryId>>,
    /// 追加する剛床拘束（既存の RigidDiaphragm と置換）。
    pub constraints: Vec<squid_n_core::model::Constraint>,
    /// 剛床代表節点。ID が既存範囲内なら置換（再利用）、範囲外（＝末尾連番）なら追加。
    pub rep_nodes: Vec<squid_n_core::model::Node>,
    /// 適用後の `model.generated_masters` の全量。
    pub generated_masters: Vec<NodeId>,
}

impl EditCommand for ApplyStories {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        use squid_n_core::model::Constraint;
        // 変更前の全量スナップショット（rep_nodes の置換/追加も含めて丸ごと復元できるようにする）。
        let old_nodes = model.nodes.clone();
        let old_generated_masters = model.generated_masters.clone();

        let old_stories = std::mem::replace(&mut model.stories, self.stories.clone());
        for (node, st) in model.nodes.iter_mut().zip(self.node_story.iter()) {
            node.story = *st;
        }
        // RigidDiaphragm のみ差し替え、それ以外の拘束は保持
        let old_constraints = model.constraints.clone();
        model
            .constraints
            .retain(|c| !matches!(c, Constraint::RigidDiaphragm { .. }));
        model.constraints.extend(self.constraints.iter().cloned());

        // 剛床代表節点：ID＝配列インデックス不変条件を保って置換 or 追加する。
        for rn in &self.rep_nodes {
            let idx = rn.id.index();
            if idx < model.nodes.len() {
                model.nodes[idx] = rn.clone();
            } else {
                debug_assert_eq!(idx, model.nodes.len(), "rep_nodes は昇順の連番である前提");
                model.nodes.push(rn.clone());
            }
        }
        model.generated_masters = self.generated_masters.clone();

        Box::new(RestoreStories {
            stories: old_stories,
            nodes: old_nodes,
            constraints: old_constraints,
            generated_masters: old_generated_masters,
        })
    }

    fn label(&self) -> &str {
        "階定義の適用"
    }
}

/// [`ApplyStories`] の逆操作。`model.nodes` を丸ごと復元することで、
/// 追加された剛床代表節点の除去（truncate）や既存節点の置換をまとめて元に戻す。
pub struct RestoreStories {
    pub stories: Vec<squid_n_core::model::Story>,
    pub nodes: Vec<squid_n_core::model::Node>,
    pub constraints: Vec<squid_n_core::model::Constraint>,
    pub generated_masters: Vec<NodeId>,
}

impl EditCommand for RestoreStories {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let new_stories = std::mem::replace(&mut model.stories, self.stories.clone());
        let new_nodes = std::mem::replace(&mut model.nodes, self.nodes.clone());
        let new_constraints = std::mem::replace(&mut model.constraints, self.constraints.clone());
        let new_generated_masters =
            std::mem::replace(&mut model.generated_masters, self.generated_masters.clone());
        Box::new(RestoreStories {
            stories: new_stories,
            nodes: new_nodes,
            constraints: new_constraints,
            generated_masters: new_generated_masters,
        })
    }

    fn label(&self) -> &str {
        "階定義の復元"
    }
}

/// 床追加。末尾に `SlabId(len)` で追加する（ID＝配列インデックスの不変条件を維持）。
/// 逆操作は床削除。
pub struct AddSlab {
    pub boundary: Vec<NodeId>,
    pub joists: Vec<squid_n_core::model::JoistLine>,
    pub loads: Vec<squid_n_core::model::AreaLoad>,
    pub method: squid_n_core::model::DistributionMethod,
    /// スラブ用途（積載荷重プリセット。`None` は積載寄与なし）。
    pub usage: Option<squid_n_core::model::SlabUsage>,
}

impl EditCommand for AddSlab {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let new_id = SlabId(model.slabs.len() as u32);
        model.slabs.push(squid_n_core::model::Slab {
            edge_supported: None,
            kind: Default::default(),
            one_way: None,
            usage: self.usage,
            id: new_id,
            boundary: self.boundary.clone(),
            joists: self.joists.clone(),
            loads: self.loads.clone(),
            method: self.method,
        });
        Box::new(DeleteSlab { id: new_id })
    }

    fn label(&self) -> &str {
        "床追加"
    }
}

/// 床削除（中間の床も可）。逆操作は [`InsertSlab`]。
///
/// ID＝配列インデックスの不変条件を保つため、削除後は当該床より後ろの
/// 床 ID を 1 つずつ繰り上げる。`SlabId` は床自身の ID 以外からは参照されない
/// （`crates` 全体で grep 済み）ため、他データへの追従は不要。
pub struct DeleteSlab {
    pub id: SlabId,
}

impl EditCommand for DeleteSlab {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        let idx = self.id.index();
        if idx >= model.slabs.len() || model.slabs[idx].id != self.id {
            return Box::new(Noop);
        }
        let removed = model.slabs.remove(idx);
        shift_slab_ids(model, |id| {
            if id.0 > self.id.0 {
                id.0 -= 1;
            }
        });
        Box::new(InsertSlab {
            index: idx,
            old: removed,
        })
    }

    fn label(&self) -> &str {
        "床削除"
    }
}

/// 指定インデックスへ床を再挿入する（[`DeleteSlab`] の逆操作専用）。
pub struct InsertSlab {
    pub index: usize,
    pub old: squid_n_core::model::Slab,
}

impl EditCommand for InsertSlab {
    fn apply(&self, model: &mut Model) -> Box<dyn EditCommand> {
        if self.index > model.slabs.len() {
            return Box::new(Noop);
        }
        let id = SlabId(self.index as u32);
        shift_slab_ids(model, |sid| {
            if sid.0 >= id.0 {
                sid.0 += 1;
            }
        });
        let mut slab = self.old.clone();
        slab.id = id;
        model.slabs.insert(self.index, slab);
        Box::new(DeleteSlab { id })
    }

    fn label(&self) -> &str {
        "床削除の取り消し"
    }
}

/// モデル内の全ての `SlabId` 参照（床自身の ID を含む）に `f` を適用する。
fn shift_slab_ids(model: &mut Model, mut f: impl FnMut(&mut SlabId)) {
    for slab in &mut model.slabs {
        f(&mut slab.id);
    }
}
