//! `UniaxialMaterial` の状態直列化まわりの共通実装。
//!
//! `clone_box` / `serialize_state` / `deserialize_state` は全材料で同一の
//! bincode ラウンドトリップであり、各 impl にベタ書きすると drift の温床になる。
//! [`impl_material_serde!`] マクロで一箇所に集約する。

/// `impl UniaxialMaterial for T` の本体に、状態直列化の定型 3 メソッドを展開する。
///
/// - `clone_box`: ファイバごとの独立状態のための `Box<dyn UniaxialMaterial>` 複製。
/// - `serialize_state`: `self` を bincode 直列化（保存時失敗はプログラムエラーとして panic）。
/// - `deserialize_state`: bincode 復元。失敗時は状態を変えずに
///   [`crate::MaterialStateError`] を返す（従来は黙って握り潰していた）。
///
/// 対象型は `Clone + serde::Serialize + serde::Deserialize` を満たすこと。
macro_rules! impl_material_serde {
    () => {
        fn clone_box(&self) -> Box<dyn $crate::UniaxialMaterial> {
            Box::new(self.clone())
        }

        fn serialize_state(&self) -> Vec<u8> {
            ::bincode::serialize(self).expect("material serialize")
        }

        fn deserialize_state(&mut self, data: &[u8]) -> Result<(), $crate::MaterialStateError> {
            *self =
                ::bincode::deserialize::<Self>(data).map_err($crate::MaterialStateError::decode)?;
            Ok(())
        }
    };
}

pub(crate) use impl_material_serde;
