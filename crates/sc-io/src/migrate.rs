use crate::scz::{IoError, CURRENT_SCHEMA_VERSION};
use sc_core::model::Model;

pub fn migrate(version: u32, bytes: Vec<u8>) -> Result<Vec<u8>, IoError> {
    match version {
        v if v == CURRENT_SCHEMA_VERSION => Ok(bytes),
        1 => {
            let model: Model = rmp_serde::from_slice(&bytes)
                .map_err(|e| IoError::Decode(format!("v1 deserialize: {e}")))?;
            rmp_serde::to_vec(&model).map_err(|e| IoError::Decode(format!("v2 serialize: {e}")))
        }
        2 => {
            let model: Model = rmp_serde::from_slice(&bytes)
                .map_err(|e| IoError::Decode(format!("v2 deserialize: {e}")))?;
            rmp_serde::to_vec(&model).map_err(|e| IoError::Decode(format!("v3 serialize: {e}")))
        }
        v => Err(IoError::UnsupportedVersion(v)),
    }
}
