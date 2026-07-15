//! 解析結果の永続化・問い合わせ。責務ごとにサブモジュールへ分割する。
//!
//! - [`types`] — 抽象IF・型（[`CaseId`]・[`ResultKind`]・[`ResultQuery`]・[`ResultBatch`]・[`ResultWriter`]・[`ResultManifest`]・[`ResultEntry`]・[`ResultStore`]）
//! - [`parquet_io`] — Parquet IO プリミティブ（[`ParquetWriter`]・[`read_partial`]・[`read_all`]）
//! - [`schema`] — Arrow スキーマ＋バッチ生成（`*_schema` / `*_batch`）
//! - [`time_history`] — 時刻歴専用（[`TimeHistoryWriter`]・[`read_time_history_range`]）
//! - [`fs_store`] — [`FsResultStore`] 具体実装＋マニフェスト
//!
//! 全 pub 項目を `pub use` で再エクスポートし、`squid_n_io::results::*` の
//! フラットなパスを維持する（MCP サーバ等が参照するため）。

mod fs_store;
mod parquet_io;
mod schema;
mod time_history;
mod types;

pub use fs_store::*;
pub use parquet_io::*;
pub use schema::*;
pub use time_history::*;
pub use types::*;

#[cfg(test)]
use arrow::array::{Float64Array, UInt32Array, UInt64Array};
#[cfg(test)]
use arrow::datatypes::DataType;
#[cfg(test)]
use squid_n_core::ids::NodeId;

#[cfg(test)]
mod tests;
