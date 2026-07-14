use arrow::array::{BooleanArray, UInt32Array};
use arrow::record_batch::RecordBatch;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use super::{
    member_force_schema, modal_schema, nodal_disp_schema, read_all, read_time_history_range,
    time_history_schema, CaseId, ParquetWriter, ResultBatch, ResultEntry, ResultKind,
    ResultManifest, ResultQuery, ResultStore, ResultWriter,
};

/// ディレクトリ配下に Parquet ファイルとマニフェスト(manifest.json)を置く結果ストア。
///
/// ファイル名は `case{case}-{kind:?}.parquet`(例: `case1-NodalDisp.parquet`)。
///
/// ## マニフェスト同期の設計
/// `ResultWriter::finish` は `Box<Self>` を consume するため、ライタ単体からは
/// ストア本体(`&mut FsResultStore`)へ直接書き戻すことができない。そこで:
/// - ライタは `Arc<Mutex<Vec<ResultEntry>>>`(`pending`)の clone を保持し、
///   `finish` 時にはそこへエントリを push するだけに留める(`Mutex` は `Send` なので
///   `ResultStore: Send` 制約はそのまま満たせる)。
/// - ストア本体は `pending` を drain して `ResultManifest` 本体へ吸収し、
///   manifest.json へ永続化する `sync(&mut self)` を持つ。`writer()`(`&mut self`)の
///   先頭で自動的に `sync()` を呼ぶため、直前に finish したライタの結果は次に
///   writer を取得した時点で必ず manifest に反映される。
/// - トレイトの `manifest(&self)` / `query(&self)` は `&self` を返す都合上、自動では
///   同期できない。ライタの `finish` 直後に manifest/query を使いたい場合は、
///   呼び出し側(MCP サーバ)が明示的に `sync()` を呼ぶこと。
///
/// ## query の対応範囲(素朴な実装)
/// - `NodalDisp` / `MemberForce` / `Modal`: 全行読み出し後にフィルタを適用する。
///   `NodalDisp` は `node_filter`(node_id 列)、`MemberForce` は `member_filter`
///   (elem_id 列)に対応する。`Modal` には node/member の概念が無いためフィルタは
///   無視する。
/// - `TimeHistory`: 既存の `read_time_history_range` を利用し、`step_range` /
///   `node_filter` に対応する(`member_filter` は概念が無いため無視)。
/// - `Story` はスキーマ関数が未実装のため `writer()` / `query()` ともに
///   `unimplemented!()` とする。MCP サーバはこの kind を使わない前提。
/// - `query` はマニフェストに該当エントリが無い場合 panic する(トレイトが `Result`
///   を返せないため)。呼び出し側は必ず `manifest()` で存在確認してから呼ぶこと。
pub struct FsResultStore {
    dir: PathBuf,
    manifest_path: PathBuf,
    manifest: ResultManifest,
    pending: Arc<Mutex<Vec<ResultEntry>>>,
}

impl FsResultStore {
    /// ディレクトリを作成(なければ)し、既存の manifest.json があれば読み込んで開く。
    pub fn open(dir: impl Into<PathBuf>) -> std::io::Result<Self> {
        let dir = dir.into();
        std::fs::create_dir_all(&dir)?;
        let manifest_path = dir.join("manifest.json");
        let manifest = if manifest_path.exists() {
            let data = std::fs::read_to_string(&manifest_path)?;
            serde_json::from_str(&data).map_err(std::io::Error::other)?
        } else {
            ResultManifest { entries: vec![] }
        };
        Ok(Self {
            dir,
            manifest_path,
            manifest,
            pending: Arc::new(Mutex::new(Vec::new())),
        })
    }

    fn file_path(&self, case: CaseId, kind: ResultKind) -> PathBuf {
        self.dir.join(format!("case{case}-{kind:?}.parquet"))
    }

    /// finish 済みライタが積んだ保留エントリを manifest 本体へ吸収し、manifest.json
    /// へ永続化する。同一 case+kind のエントリは上書きする。
    pub fn sync(&mut self) -> std::io::Result<()> {
        let drained: Vec<ResultEntry> = {
            let mut pending = self.pending.lock().expect("pending mutex poisoned");
            pending.drain(..).collect()
        };
        if drained.is_empty() {
            return Ok(());
        }
        for entry in drained {
            if let Some(existing) = self
                .manifest
                .entries
                .iter_mut()
                .find(|e| e.case == entry.case && e.kind == entry.kind)
            {
                *existing = entry;
            } else {
                self.manifest.entries.push(entry);
            }
        }
        self.persist()
    }

    fn persist(&self) -> std::io::Result<()> {
        let data = serde_json::to_string_pretty(&self.manifest).map_err(std::io::Error::other)?;
        std::fs::write(&self.manifest_path, data)
    }
}

/// `col_idx` 列(UInt32)の値が `ids` に含まれる行だけを残す素朴なフィルタ。
fn filter_by_u32_column(
    batches: Vec<RecordBatch>,
    col_idx: usize,
    ids: &[u32],
) -> Vec<RecordBatch> {
    let id_set: std::collections::HashSet<u32> = ids.iter().copied().collect();
    let mut result = Vec::with_capacity(batches.len());
    for batch in batches {
        let col = batch
            .column(col_idx)
            .as_any()
            .downcast_ref::<UInt32Array>()
            .expect("フィルタ対象列は UInt32 であるべき");
        let num_rows = batch.num_rows();
        let keep: Vec<bool> = (0..num_rows)
            .map(|i| id_set.contains(&col.value(i)))
            .collect();
        let mask = BooleanArray::from(keep);
        let filtered =
            arrow::compute::filter_record_batch(&batch, &mask).expect("filter_record_batch");
        if filtered.num_rows() > 0 {
            result.push(filtered);
        }
    }
    result
}

struct FsResultWriter {
    inner: ParquetWriter,
    rows: u64,
    case: CaseId,
    kind: ResultKind,
    path: String,
    pending: Arc<Mutex<Vec<ResultEntry>>>,
}

impl ResultWriter for FsResultWriter {
    fn write_rows(&mut self, batch: &RecordBatch) {
        self.rows += batch.num_rows() as u64;
        self.inner.write_rows(batch);
    }

    fn finish(self: Box<Self>) {
        let FsResultWriter {
            inner,
            rows,
            case,
            kind,
            path,
            pending,
        } = *self;
        Box::new(inner).finish();
        pending
            .lock()
            .expect("pending mutex poisoned")
            .push(ResultEntry {
                case,
                kind,
                rows,
                path,
            });
    }
}

impl ResultStore for FsResultStore {
    fn writer(&mut self, case: CaseId, kind: ResultKind) -> Box<dyn ResultWriter> {
        // 直前に finish したライタの結果を manifest へ反映してから新規書き込みを開始する。
        let _ = self.sync();
        let path = self.file_path(case, kind);
        let path_str = path.to_string_lossy().into_owned();
        let schema = match kind {
            ResultKind::NodalDisp => nodal_disp_schema(),
            ResultKind::MemberForce => member_force_schema(),
            ResultKind::Modal => modal_schema(),
            ResultKind::TimeHistory => time_history_schema(),
            ResultKind::Story => {
                unimplemented!("Story kind はスキーマ未定義のため未対応(MCP からは呼ばれない前提)")
            }
        };
        let inner = ParquetWriter::create(&path_str, schema).expect("parquet writer 作成に失敗");
        Box::new(FsResultWriter {
            inner,
            rows: 0,
            case,
            kind,
            path: path_str,
            pending: Arc::clone(&self.pending),
        })
    }

    fn query(&self, q: &ResultQuery) -> ResultBatch {
        // manifest 内に該当エントリが存在することのみを確認する。実ファイルパスは
        // manifest に記録された `path` を信用せず case/kind から再計算する。
        // （manifest.json はユーザーが書換え可能で、`../` や絶対パスを混入させると
        //   任意ファイル読み出しに悪用され得るため。書込み側も常に file_path を使う。）
        assert!(
            self.manifest
                .entries
                .iter()
                .any(|e| e.case == q.case && e.kind == q.kind),
            "manifest に case={} kind={:?} のエントリが無い(query 前に manifest() で存在確認すること)",
            q.case,
            q.kind
        );
        let path = self.file_path(q.case, q.kind);
        let path = path.to_string_lossy();

        match q.kind {
            ResultKind::TimeHistory => {
                let node_ids: Option<Vec<u32>> = q
                    .node_filter
                    .as_ref()
                    .map(|ids| ids.iter().map(|n| n.0).collect());
                let batches = read_time_history_range(&path, q.step_range, node_ids.as_deref())
                    .expect("time_history 部分読み出しに失敗");
                let batch = arrow::compute::concat_batches(&time_history_schema(), &batches)
                    .expect("concat_batches (time_history)");
                ResultBatch { batch }
            }
            ResultKind::NodalDisp => {
                let mut batches = read_all(&path).expect("nodal_disp 読み出しに失敗");
                if let Some(ids) = &q.node_filter {
                    let ids: Vec<u32> = ids.iter().map(|n| n.0).collect();
                    batches = filter_by_u32_column(batches, 0, &ids);
                }
                let batch = arrow::compute::concat_batches(&nodal_disp_schema(), &batches)
                    .expect("concat_batches (nodal_disp)");
                ResultBatch { batch }
            }
            ResultKind::MemberForce => {
                let mut batches = read_all(&path).expect("member_force 読み出しに失敗");
                if let Some(ids) = &q.member_filter {
                    let ids: Vec<u32> = ids.iter().map(|e| e.0).collect();
                    batches = filter_by_u32_column(batches, 0, &ids);
                }
                let batch = arrow::compute::concat_batches(&member_force_schema(), &batches)
                    .expect("concat_batches (member_force)");
                ResultBatch { batch }
            }
            ResultKind::Modal => {
                // モーダル結果に node/member の概念は無いため node_filter/member_filter は無視する。
                let batches = read_all(&path).expect("modal 読み出しに失敗");
                let batch = arrow::compute::concat_batches(&modal_schema(), &batches)
                    .expect("concat_batches (modal)");
                ResultBatch { batch }
            }
            ResultKind::Story => {
                unimplemented!("Story kind の query は未対応(MCP からは呼ばれない前提)")
            }
        }
    }

    fn manifest(&self) -> &ResultManifest {
        &self.manifest
    }
}
