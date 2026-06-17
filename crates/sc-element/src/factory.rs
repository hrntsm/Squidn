use crate::behavior::{ElemState, ElementBehavior};
use sc_core::model::{ElementData, ElementKind, Model};

pub fn build_behavior(data: &ElementData, model: &Model) -> (Box<dyn ElementBehavior>, ElemState) {
    match data.kind {
        ElementKind::Beam => (
            Box::new(crate::beam::BeamElement::new(data, model)),
            ElemState::default(),
        ),
        ElementKind::PanelZone => (
            Box::new(crate::panel::PanelZone::new(data, model)),
            ElemState::default(),
        ),
        ElementKind::Shell => (
            Box::new(crate::shell::ShellElement::new(data, model)),
            ElemState::default(),
        ),
        ElementKind::Ms => (
            Box::new(crate::ms::MsElement::new(data, model)),
            ElemState::default(),
        ),
        ElementKind::Fiber | ElementKind::Wall => (
            Box::new(crate::beam::BeamElement::new(data, model)),
            ElemState::default(),
        ),
    }
}
