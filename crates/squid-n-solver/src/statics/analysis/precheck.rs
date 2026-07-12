//! 解析前のモデル静的検証と特異行列診断。
//!
//! よくあるモデリングミス（節点・部材・拘束の欠如、断面/材料未割当、孤立節点）を
//! 特異行列エラーの前に検出し、「何をすれば直るか」を含む日本語メッセージで返す。

use squid_n_core::model::Model;
use squid_n_math::solver::SolveError;

/// 解析前のモデル静的検証。よくあるモデリングミスを特異行列エラーの前に検出し、
/// 「何をすれば直るか」を含むメッセージで返す。
pub(super) fn precheck_model(model: &Model) -> Result<(), SolveError> {
    use squid_n_core::model::ElementKind;

    if model.nodes.is_empty() {
        return Err(SolveError::InvalidInput(
            "節点がありません。モデルタブで節点を追加してください。".into(),
        ));
    }
    if model.elements.is_empty() {
        return Err(SolveError::InvalidInput(
            "部材がありません。モデルタブで部材を追加してください。".into(),
        ));
    }
    if !model.nodes.iter().any(|n| n.restraint.0 != 0) {
        return Err(SolveError::InvalidInput(
            "拘束(支点)が 1 つもありません。境界条件タブで支点を設定してください。".into(),
        ));
    }

    // 梁要素の断面・材料未割当
    let missing: Vec<u32> = model
        .elements
        .iter()
        .filter(|e| {
            matches!(e.kind, ElementKind::Beam) && (e.section.is_none() || e.material.is_none())
        })
        .map(|e| e.id.0)
        .collect();
    if !missing.is_empty() {
        let head: Vec<String> = missing.iter().take(5).map(|id| id.to_string()).collect();
        let more = if missing.len() > 5 {
            format!(" 他{}件", missing.len() - 5)
        } else {
            String::new()
        };
        return Err(SolveError::InvalidInput(format!(
            "断面または材料が未割当の部材があります: ID {}{}。部材タブで割り当ててください。",
            head.join(", "),
            more
        )));
    }

    // 孤立節点（要素・拘束・剛床から参照されず、完全固定でもない）
    // → 剛性ゼロの自由 DOF となり特異行列の典型原因
    let mut referenced = vec![false; model.nodes.len()];
    for e in &model.elements {
        for n in &e.nodes {
            referenced[n.index()] = true;
        }
    }
    for c in &model.constraints {
        use squid_n_core::model::Constraint;
        match c {
            Constraint::RigidDiaphragm { master, slaves, .. }
            | Constraint::RigidLink { master, slaves, .. } => {
                referenced[master.index()] = true;
                for s in slaves {
                    referenced[s.index()] = true;
                }
            }
            Constraint::Mpc { master, terms } => {
                referenced[master.index()] = true;
                for (n, _, _) in terms {
                    referenced[n.index()] = true;
                }
            }
        }
    }
    for story in &model.stories {
        for d in &story.diaphragms {
            referenced[d.master.index()] = true;
            for s in &d.slaves {
                referenced[s.index()] = true;
            }
        }
    }
    let isolated: Vec<u32> = model
        .nodes
        .iter()
        .filter(|n| !referenced[n.id.index()] && n.restraint != squid_n_core::dof::Dof6Mask::FIXED)
        .map(|n| n.id.0)
        .collect();
    if !isolated.is_empty() {
        let head: Vec<String> = isolated.iter().take(5).map(|id| id.to_string()).collect();
        let more = if isolated.len() > 5 {
            format!(" 他{}件", isolated.len() - 5)
        } else {
            String::new()
        };
        return Err(SolveError::InvalidInput(format!(
            "どの部材にも接続されていない節点があります: ID {}{}。削除するか完全固定にしてください(剛性ゼロの自由度は解析できません)。",
            head.join(", "),
            more
        )));
    }

    Ok(())
}

/// 剛性行列の分解に失敗した（特異・非正定値）ときの診断メッセージ。
pub(super) fn singular_diagnosis(model: &Model) -> String {
    let n_restrained = model.nodes.iter().filter(|n| n.restraint.0 != 0).count();
    format!(
        "剛性行列が特異(非正定値)です。構造が機構(不安定)になっている可能性があります。\
         考えられる原因: (1) 拘束が不足している(現在 {} 節点に拘束あり)、\
         (2) ピン接合が連続し回転が拘束されない部材がある、\
         (3) 断面性能(A・I)が 0 の断面がある。",
        n_restrained
    )
}
