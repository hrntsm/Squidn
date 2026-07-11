use crate::scz::{IoError, CURRENT_SCHEMA_VERSION};

/// 読み込んだ生バイト（model.msgpack）を、宣言された版から最新版へ移行する。
/// 未リリースのため後方互換なし: 現行版のみ受け付ける。
/// リリース後に版を上げる際は、ここに旧版→新版の変換分岐を追加する
/// （破壊的変更には版ごとの専用型を用意し、現行 Model 型での再デコードに頼らないこと）。
pub fn migrate(version: u32, bytes: Vec<u8>) -> Result<Vec<u8>, IoError> {
    match version {
        CURRENT_SCHEMA_VERSION => Ok(bytes),
        v => Err(IoError::UnsupportedVersion(v)),
    }
}
