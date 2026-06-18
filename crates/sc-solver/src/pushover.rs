use crate::analysis::{AiMode, Analysis, SeismicDir};
use sc_core::ids::{ElemId, StoryId};
use sc_core::model::Model;

/// 性能曲線の1点（P5 §7.4）
pub struct CapacityPoint {
    pub step: u32,
    pub roof_disp: f64,
    pub base_shear: f64,
    pub story_shear: Vec<f64>,
    pub story_drift: Vec<f64>,
}

/// ヒンジ発生事象（P5 §7.4）
pub struct HingeEvent {
    pub step: u32,
    pub elem: ElemId,
    pub pos: f64,
    pub level: HingeLevel,
    pub ductility: f64,
}

/// ヒンジレベル（P5 §7.4）
pub enum HingeLevel {
    Crack,
    Yield,
    Ultimate,
}

/// 崩壊機構種別（P5 §7.4）
pub enum MechanismType {
    Overall,
    StoryCollapse { story: StoryId },
    Partial,
}

/// プッシュオーバー解析結果（P5 §7.4）
pub struct PushoverResult {
    pub steps: Vec<PushoverStep>,
    pub capacity_curve: Vec<CapacityPoint>,
    pub hinges: Vec<HingeEvent>,
    pub mechanism: MechanismType,
    pub qu: f64,
}

pub struct PushoverStep {
    pub load_factor: f64,
    pub top_disp: f64,
    pub base_shear: f64,
    pub story_drifts: Vec<f64>,
}

/// プッシュオーバー解析（P5 §7）
/// 現在は弾性1ステップのスタブから拡張中。
/// TODO: 増分NRループ（荷重制御→変位制御）、降伏追跡、崩壊機構判定、弧長法
pub fn pushover_analysis(
    analysis: &Analysis,
    _model: &mut Model,
    dir: SeismicDir,
    _max_steps: usize,
    _max_disp: f64,
) -> Result<PushoverResult, String> {
    // TODO: behaviors を作成し、増分NR反復で非線形解析
    //   let mut behaviors: Vec<Box<dyn ElementBehavior>> = Vec::new();
    //   for elem in &_model.elements {
    //       let (b, _) = sc_element::factory::build_behavior(elem, _model);
    //       behaviors.push(b);
    //   }
    //   let snap = StateSnapshot::capture(&behaviors);
    //   loop { NR反復 → commit_all / revert_all }

    let result = analysis
        .seismic_static(dir, AiMode::Approx)
        .map_err(|e| format!("solver: {:?}", e))?;
    let top_disp = result
        .disp
        .last()
        .map(|d| match dir {
            SeismicDir::X => d[0],
            SeismicDir::Y => d[1],
        })
        .unwrap_or(0.0);
    let base_shear: f64 = result
        .member_forces
        .iter()
        .flat_map(|(_, f)| f.at.first())
        .map(|(_, f)| f[0].abs())
        .sum();

    Ok(PushoverResult {
        steps: vec![PushoverStep {
            load_factor: 1.0,
            top_disp,
            base_shear,
            story_drifts: vec![],
        }],
        capacity_curve: vec![],
        hinges: vec![],
        mechanism: MechanismType::Partial,
        qu: base_shear,
    })
}
