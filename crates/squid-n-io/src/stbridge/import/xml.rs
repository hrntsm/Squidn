//! ST-Bridge パースの XML 属性・数値ヘルパ（属性辞書化と型付き取得）。

use super::super::StbError;
use std::collections::HashMap;

/// `StbNodeIdOrder` の内容文字列（空白区切りの節点 id 列）を解析し、
/// 数値として読める token を境界（`boundary`）へ追加する（スラブ・壁共用）。
pub(super) fn push_node_id_tokens(text: &str, boundary: &mut Vec<u32>) {
    for tok in text.split_whitespace() {
        if let Ok(id) = tok.parse::<u32>() {
            boundary.push(id);
        }
    }
}

pub(super) fn attrs(
    e: &quick_xml::events::BytesStart,
) -> Result<HashMap<String, String>, StbError> {
    let mut m = HashMap::new();
    for a in e.attributes() {
        let a = a.map_err(|err| StbError::Parse(err.to_string()))?;
        let key = String::from_utf8_lossy(a.key.as_ref()).to_string();
        let val = a
            .normalized_value(quick_xml::XmlVersion::Implicit1_0)
            .map_err(|err| StbError::Parse(err.to_string()))?
            .to_string();
        m.insert(key, val);
    }
    Ok(m)
}

pub(super) fn get_f64(a: &HashMap<String, String>, k: &str) -> Result<f64, StbError> {
    a.get(k)
        .ok_or_else(|| StbError::Parse(format!("missing attr {k}")))?
        .parse::<f64>()
        .map_err(|_| StbError::Parse(format!("bad f64 attr {k}")))
}

/// 複数の候補キーのいずれかから f64 を取る（属性名の方言差を吸収する）。
pub(super) fn get_f64_any(a: &HashMap<String, String>, keys: &[&str]) -> Result<f64, StbError> {
    for k in keys {
        if let Some(v) = a.get(*k) {
            return v
                .parse::<f64>()
                .map_err(|_| StbError::Parse(format!("bad f64 attr {k}")));
        }
    }
    Err(StbError::Parse(format!("missing attr {:?}", keys)))
}

pub(super) fn get_opt_f64(a: &HashMap<String, String>, k: &str) -> Option<f64> {
    match a.get(k) {
        Some(v) if !v.is_empty() => v.parse::<f64>().ok(),
        _ => None,
    }
}

pub(super) fn get_u32(a: &HashMap<String, String>, k: &str) -> Result<u32, StbError> {
    a.get(k)
        .ok_or_else(|| StbError::Parse(format!("missing attr {k}")))?
        .parse::<u32>()
        .map_err(|_| StbError::Parse(format!("bad u32 attr {k}")))
}

pub(super) fn get_i64(a: &HashMap<String, String>, k: &str) -> Option<i64> {
    a.get(k).and_then(|v| v.parse::<i64>().ok())
}
