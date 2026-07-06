use arrow::array::{BooleanArray, Float64Array, UInt32Array, UInt64Array};
use arrow::datatypes::{DataType, Field, Schema};
use arrow::record_batch::RecordBatch;
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
use parquet::arrow::arrow_writer::ArrowWriter;
use parquet::arrow::ProjectionMask;
use parquet::file::properties::WriterProperties;
use squid_n_core::ids::{ElemId, NodeId};
use std::fs::File;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

pub type CaseId = u32;

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ResultKind {
    NodalDisp,
    MemberForce,
    Story,
    Modal,
    TimeHistory,
}

pub struct ResultQuery {
    pub case: CaseId,
    pub kind: ResultKind,
    pub node_filter: Option<Vec<NodeId>>,
    pub member_filter: Option<Vec<ElemId>>,
    pub step_range: Option<(u64, u64)>,
}

pub struct ResultBatch {
    pub batch: RecordBatch,
}

pub trait ResultWriter {
    fn write_rows(&mut self, batch: &RecordBatch);
    fn finish(self: Box<Self>);
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ResultManifest {
    pub entries: Vec<ResultEntry>,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ResultEntry {
    pub case: CaseId,
    pub kind: ResultKind,
    pub rows: u64,
    pub path: String,
}

/// 結果ストア。`Send` を要求するのは、MCP サーバ(P8)が `ServerState` を
/// スレッド間で共有する(`rmcp::ServerHandler: Send + Sync`)ため。
pub trait ResultStore: Send {
    fn writer(&mut self, case: CaseId, kind: ResultKind) -> Box<dyn ResultWriter>;
    fn query(&self, q: &ResultQuery) -> ResultBatch;
    fn manifest(&self) -> &ResultManifest;
}

pub struct ParquetWriter {
    inner: ArrowWriter<File>,
    rows: u64,
}

impl ParquetWriter {
    pub fn create(path: &str, schema: Arc<Schema>) -> parquet::errors::Result<Self> {
        let file = File::create(path)?;
        let props = WriterProperties::builder()
            .set_max_row_group_row_count(Some(64 * 1024))
            .build();
        Ok(Self {
            inner: ArrowWriter::try_new(file, schema, Some(props))?,
            rows: 0,
        })
    }
}

impl ResultWriter for ParquetWriter {
    fn write_rows(&mut self, batch: &RecordBatch) {
        self.rows += batch.num_rows() as u64;
        self.inner.write(batch).expect("parquet write");
    }

    fn finish(self: Box<Self>) {
        self.inner.close().expect("parquet close");
    }
}

pub fn read_partial(path: &str, columns: Vec<usize>) -> parquet::errors::Result<Vec<RecordBatch>> {
    let file = File::open(path)?;
    let builder = ParquetRecordBatchReaderBuilder::try_new(file)?;
    let mask = ProjectionMask::roots(builder.parquet_schema(), columns);
    let reader = builder.with_projection(mask).build()?;
    reader
        .collect::<Result<Vec<_>, _>>()
        .map_err(|e| parquet::errors::ParquetError::General(format!("{e:?}")))
}

pub fn read_all(path: &str) -> parquet::errors::Result<Vec<RecordBatch>> {
    let file = File::open(path)?;
    let builder = ParquetRecordBatchReaderBuilder::try_new(file)?;
    let reader = builder.build()?;
    reader
        .collect::<Result<Vec<_>, _>>()
        .map_err(|e| parquet::errors::ParquetError::General(format!("{e:?}")))
}

pub fn nodal_disp_schema() -> Arc<Schema> {
    Arc::new(Schema::new(vec![
        Field::new("node_id", DataType::UInt32, false),
        Field::new("ux", DataType::Float64, false),
        Field::new("uy", DataType::Float64, false),
        Field::new("uz", DataType::Float64, false),
        Field::new("rx", DataType::Float64, false),
        Field::new("ry", DataType::Float64, false),
        Field::new("rz", DataType::Float64, false),
    ]))
}

pub fn nodal_disp_batch(node_ids: &[u32], disp: &[[f64; 6]]) -> arrow::error::Result<RecordBatch> {
    let n = node_ids.len();
    let id_arr = UInt32Array::from(node_ids.to_vec());
    let mut ux = Vec::with_capacity(n);
    let mut uy = Vec::with_capacity(n);
    let mut uz = Vec::with_capacity(n);
    let mut rx = Vec::with_capacity(n);
    let mut ry = Vec::with_capacity(n);
    let mut rz = Vec::with_capacity(n);
    for d in disp {
        ux.push(d[0]);
        uy.push(d[1]);
        uz.push(d[2]);
        rx.push(d[3]);
        ry.push(d[4]);
        rz.push(d[5]);
    }
    RecordBatch::try_new(
        nodal_disp_schema(),
        vec![
            Arc::new(id_arr),
            Arc::new(Float64Array::from(ux)),
            Arc::new(Float64Array::from(uy)),
            Arc::new(Float64Array::from(uz)),
            Arc::new(Float64Array::from(rx)),
            Arc::new(Float64Array::from(ry)),
            Arc::new(Float64Array::from(rz)),
        ],
    )
}

pub fn member_force_schema() -> Arc<Schema> {
    Arc::new(Schema::new(vec![
        Field::new("elem_id", DataType::UInt32, false),
        Field::new("pos", DataType::Float64, false), // 評価位置 0..1
        Field::new("n", DataType::Float64, false),
        Field::new("qy", DataType::Float64, false),
        Field::new("qz", DataType::Float64, false),
        Field::new("mx", DataType::Float64, false),
        Field::new("my", DataType::Float64, false),
        Field::new("mz", DataType::Float64, false),
    ]))
}

/// 部材内力（評価位置別）を RecordBatch 化する。
/// `rows`: (要素ID, 評価位置 0..1, [N,Qy,Qz,Mx,My,Mz])
pub fn member_force_batch(rows: &[(u32, f64, [f64; 6])]) -> arrow::error::Result<RecordBatch> {
    let n = rows.len();
    let mut elem = Vec::with_capacity(n);
    let mut pos = Vec::with_capacity(n);
    let mut cols: [Vec<f64>; 6] = Default::default();
    for (e, p, f) in rows {
        elem.push(*e);
        pos.push(*p);
        for (c, v) in cols.iter_mut().zip(f.iter()) {
            c.push(*v);
        }
    }
    let [n_, qy, qz, mx, my, mz] = cols;
    RecordBatch::try_new(
        member_force_schema(),
        vec![
            Arc::new(UInt32Array::from(elem)),
            Arc::new(Float64Array::from(pos)),
            Arc::new(Float64Array::from(n_)),
            Arc::new(Float64Array::from(qy)),
            Arc::new(Float64Array::from(qz)),
            Arc::new(Float64Array::from(mx)),
            Arc::new(Float64Array::from(my)),
            Arc::new(Float64Array::from(mz)),
        ],
    )
}

pub fn modal_schema() -> Arc<Schema> {
    Arc::new(Schema::new(vec![
        Field::new("mode", DataType::UInt32, false),
        Field::new("period", DataType::Float64, false),
        Field::new("omega2", DataType::Float64, false),
        Field::new("part_x", DataType::Float64, false),
        Field::new("part_y", DataType::Float64, false),
        Field::new("part_z", DataType::Float64, false),
        Field::new("eff_x", DataType::Float64, false),
        Field::new("eff_y", DataType::Float64, false),
        Field::new("eff_z", DataType::Float64, false),
    ]))
}

pub fn time_history_schema() -> Arc<Schema> {
    Arc::new(Schema::new(vec![
        Field::new("step", DataType::UInt64, false),
        Field::new("time", DataType::Float64, false),
        Field::new("node_id", DataType::UInt32, false),
        Field::new("ux", DataType::Float64, false),
        Field::new("uy", DataType::Float64, false),
        Field::new("uz", DataType::Float64, false),
        Field::new("rx", DataType::Float64, false),
        Field::new("ry", DataType::Float64, false),
        Field::new("rz", DataType::Float64, false),
    ]))
}

/// モーダル結果（固有周期・刺激係数・有効質量）を RecordBatch 化する。
pub fn modal_batch(
    period: &[f64],
    omega2: &[f64],
    participation: &[[f64; 3]],
    effective_mass: &[[f64; 3]],
) -> arrow::error::Result<RecordBatch> {
    let n = period.len();
    let mode: Vec<u32> = (0..n as u32).collect();
    let mut part: [Vec<f64>; 3] = Default::default();
    let mut eff: [Vec<f64>; 3] = Default::default();
    for i in 0..n {
        for d in 0..3 {
            part[d].push(participation.get(i).map(|p| p[d]).unwrap_or(0.0));
            eff[d].push(effective_mass.get(i).map(|e| e[d]).unwrap_or(0.0));
        }
    }
    let [px, py, pz] = part;
    let [ex, ey, ez] = eff;
    RecordBatch::try_new(
        modal_schema(),
        vec![
            Arc::new(UInt32Array::from(mode)),
            Arc::new(Float64Array::from(period.to_vec())),
            Arc::new(Float64Array::from(omega2.to_vec())),
            Arc::new(Float64Array::from(px)),
            Arc::new(Float64Array::from(py)),
            Arc::new(Float64Array::from(pz)),
            Arc::new(Float64Array::from(ex)),
            Arc::new(Float64Array::from(ey)),
            Arc::new(Float64Array::from(ez)),
        ],
    )
}

pub fn time_history_batch(
    step: u64,
    time: f64,
    node_ids: &[u32],
    disp: &[[f64; 6]],
) -> arrow::error::Result<RecordBatch> {
    let n = node_ids.len();
    let step_arr = UInt64Array::from(vec![step; n]);
    let time_arr = Float64Array::from(vec![time; n]);
    let id_arr = UInt32Array::from(node_ids.to_vec());
    let mut ux = Vec::with_capacity(n);
    let mut uy = Vec::with_capacity(n);
    let mut uz = Vec::with_capacity(n);
    let mut rx = Vec::with_capacity(n);
    let mut ry = Vec::with_capacity(n);
    let mut rz = Vec::with_capacity(n);
    for d in disp {
        ux.push(d[0]);
        uy.push(d[1]);
        uz.push(d[2]);
        rx.push(d[3]);
        ry.push(d[4]);
        rz.push(d[5]);
    }
    RecordBatch::try_new(
        time_history_schema(),
        vec![
            Arc::new(step_arr),
            Arc::new(time_arr),
            Arc::new(id_arr),
            Arc::new(Float64Array::from(ux)),
            Arc::new(Float64Array::from(uy)),
            Arc::new(Float64Array::from(uz)),
            Arc::new(Float64Array::from(rx)),
            Arc::new(Float64Array::from(ry)),
            Arc::new(Float64Array::from(rz)),
        ],
    )
}

pub struct TimeHistoryWriter {
    writer: ParquetWriter,
    step: u64,
}

impl TimeHistoryWriter {
    pub fn create(path: &str) -> parquet::errors::Result<Self> {
        let schema = time_history_schema();
        Ok(Self {
            writer: ParquetWriter::create(path, schema)?,
            step: 0,
        })
    }

    pub fn write_step(
        &mut self,
        time: f64,
        node_ids: &[u32],
        disp: &[[f64; 6]],
    ) -> arrow::error::Result<()> {
        let batch = time_history_batch(self.step, time, node_ids, disp)?;
        self.writer.write_rows(&batch);
        self.step += 1;
        Ok(())
    }

    pub fn finish(self) {
        Box::new(self.writer).finish();
    }

    pub fn current_step(&self) -> u64 {
        self.step
    }
}

pub fn read_time_history_range(
    path: &str,
    step_range: Option<(u64, u64)>,
    node_filter: Option<&[u32]>,
) -> parquet::errors::Result<Vec<RecordBatch>> {
    let batches = read_all(path)?;
    if step_range.is_none() && node_filter.is_none() {
        return Ok(batches);
    }

    let node_set: Option<std::collections::HashSet<u32>> =
        node_filter.map(|ids| ids.iter().copied().collect());

    let mut result = Vec::new();
    for batch in batches {
        let step_col = batch
            .column(0)
            .as_any()
            .downcast_ref::<UInt64Array>()
            .expect("step column should be UInt64");
        let node_col = batch
            .column(2)
            .as_any()
            .downcast_ref::<UInt32Array>()
            .expect("node_id column should be UInt32");

        let num_rows = batch.num_rows();
        let mut keep = vec![true; num_rows];

        if let Some((start, end)) = step_range {
            for (i, k) in keep.iter_mut().enumerate().take(num_rows) {
                let s = step_col.value(i);
                if s < start || s > end {
                    *k = false;
                }
            }
        }

        if let Some(ref ids) = node_set {
            for (i, k) in keep.iter_mut().enumerate().take(num_rows) {
                if *k && !ids.contains(&node_col.value(i)) {
                    *k = false;
                }
            }
        }

        let mask = BooleanArray::from(keep);
        let filtered = arrow::compute::filter_record_batch(&batch, &mask)
            .map_err(|e| parquet::errors::ParquetError::General(format!("{e:?}")))?;
        if filtered.num_rows() > 0 {
            result.push(filtered);
        }
    }

    Ok(result)
}

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
        let entry = self
            .manifest
            .entries
            .iter()
            .find(|e| e.case == q.case && e.kind == q.kind)
            .unwrap_or_else(|| {
                panic!(
                    "manifest に case={} kind={:?} のエントリが無い(query 前に manifest() で存在確認すること)",
                    q.case, q.kind
                )
            });

        match q.kind {
            ResultKind::TimeHistory => {
                let node_ids: Option<Vec<u32>> = q
                    .node_filter
                    .as_ref()
                    .map(|ids| ids.iter().map(|n| n.0).collect());
                let batches =
                    read_time_history_range(&entry.path, q.step_range, node_ids.as_deref())
                        .expect("time_history 部分読み出しに失敗");
                let batch = arrow::compute::concat_batches(&time_history_schema(), &batches)
                    .expect("concat_batches (time_history)");
                ResultBatch { batch }
            }
            ResultKind::NodalDisp => {
                let mut batches = read_all(&entry.path).expect("nodal_disp 読み出しに失敗");
                if let Some(ids) = &q.node_filter {
                    let ids: Vec<u32> = ids.iter().map(|n| n.0).collect();
                    batches = filter_by_u32_column(batches, 0, &ids);
                }
                let batch = arrow::compute::concat_batches(&nodal_disp_schema(), &batches)
                    .expect("concat_batches (nodal_disp)");
                ResultBatch { batch }
            }
            ResultKind::MemberForce => {
                let mut batches = read_all(&entry.path).expect("member_force 読み出しに失敗");
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
                let batches = read_all(&entry.path).expect("modal 読み出しに失敗");
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parquet_roundtrip() {
        let dir = std::env::temp_dir();
        let path = dir.join("p2_test_nodal.parquet");
        let path_str = path.to_str().unwrap();

        {
            let mut writer = ParquetWriter::create(path_str, nodal_disp_schema()).unwrap();
            let batch = nodal_disp_batch(
                &[0, 1, 2],
                &[
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [1.5, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 2.0, 0.0, 0.0, 0.0, 0.0],
                ],
            )
            .unwrap();
            writer.write_rows(&batch);
            Box::new(writer).finish();
        }

        let batches = read_all(path_str).unwrap();
        assert_eq!(batches.len(), 1);
        let batch = &batches[0];
        assert_eq!(batch.num_rows(), 3);
        assert_eq!(batch.num_columns(), 7);
    }

    #[test]
    fn test_modal_roundtrip_partial() {
        let dir = std::env::temp_dir();
        let path = dir.join("p2_test_modal.parquet");
        let path_str = path.to_str().unwrap();
        {
            let mut writer = ParquetWriter::create(path_str, modal_schema()).unwrap();
            let batch = modal_batch(
                &[0.3215, 0.1228],
                &[382.0, 2618.0],
                &[[1.0, 0.0, 0.0], [0.5, 0.0, 0.0]],
                &[[1.894, 0.0, 0.0], [0.106, 0.0, 0.0]],
            )
            .unwrap();
            writer.write_rows(&batch);
            Box::new(writer).finish();
        }
        // 部分読出し: period 列(=1)のみ射影
        let batches = read_partial(path_str, vec![1]).unwrap();
        let b = &batches[0];
        assert_eq!(b.num_columns(), 1);
        assert_eq!(b.num_rows(), 2);
    }

    #[test]
    fn test_member_force_roundtrip() {
        let dir = std::env::temp_dir();
        let path = dir.join("p2_test_member.parquet");
        let path_str = path.to_str().unwrap();
        {
            let mut writer = ParquetWriter::create(path_str, member_force_schema()).unwrap();
            let batch = member_force_batch(&[
                (1, 0.0, [100.0, 5.0, 0.0, 0.0, 0.0, 200.0]),
                (1, 1.0, [100.0, 5.0, 0.0, 0.0, 0.0, -200.0]),
            ])
            .unwrap();
            writer.write_rows(&batch);
            Box::new(writer).finish();
        }
        let batches = read_all(path_str).unwrap();
        assert_eq!(batches[0].num_rows(), 2);
        assert_eq!(batches[0].num_columns(), 8);
    }

    #[test]
    fn test_time_history_schema_fields() {
        let schema = time_history_schema();
        assert_eq!(schema.fields().len(), 9);
        assert_eq!(schema.field(0).name(), "step");
        assert_eq!(schema.field(1).name(), "time");
        assert_eq!(schema.field(2).name(), "node_id");
        assert_eq!(schema.field(3).name(), "ux");
        assert_eq!(schema.field(4).name(), "uy");
        assert_eq!(schema.field(5).name(), "uz");
        assert_eq!(schema.field(6).name(), "rx");
        assert_eq!(schema.field(7).name(), "ry");
        assert_eq!(schema.field(8).name(), "rz");
        assert_eq!(schema.field(0).data_type(), &DataType::UInt64);
        assert_eq!(schema.field(1).data_type(), &DataType::Float64);
        assert_eq!(schema.field(2).data_type(), &DataType::UInt32);
    }

    #[test]
    fn test_time_history_batch_values() {
        let batch = time_history_batch(
            5,
            2.5,
            &[10, 20],
            &[
                [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
                [1.1, 1.2, 1.3, 1.4, 1.5, 1.6],
            ],
        )
        .unwrap();
        assert_eq!(batch.num_rows(), 2);
        assert_eq!(batch.num_columns(), 9);

        let step_col = batch
            .column(0)
            .as_any()
            .downcast_ref::<UInt64Array>()
            .unwrap();
        assert_eq!(step_col.value(0), 5);
        assert_eq!(step_col.value(1), 5);

        let time_col = batch
            .column(1)
            .as_any()
            .downcast_ref::<Float64Array>()
            .unwrap();
        assert!((time_col.value(0) - 2.5).abs() < f64::EPSILON);

        let node_col = batch
            .column(2)
            .as_any()
            .downcast_ref::<UInt32Array>()
            .unwrap();
        assert_eq!(node_col.value(0), 10);
        assert_eq!(node_col.value(1), 20);

        let ux_col = batch
            .column(3)
            .as_any()
            .downcast_ref::<Float64Array>()
            .unwrap();
        assert!((ux_col.value(1) - 1.1).abs() < f64::EPSILON);
    }

    #[test]
    fn test_time_history_write_read_roundtrip() {
        let dir = std::env::temp_dir();
        let path = dir.join("p6_th_roundtrip.parquet");
        let path_str = path.to_str().unwrap();

        {
            let mut writer = TimeHistoryWriter::create(path_str).unwrap();
            let nodes = [1u32, 2, 3];
            for s in 0..3 {
                let t = s as f64 * 0.1;
                let d = [
                    [t + 0.01, t + 0.02, t + 0.03, t + 0.04, t + 0.05, t + 0.06],
                    [t + 0.11, t + 0.12, t + 0.13, t + 0.14, t + 0.15, t + 0.16],
                    [t + 0.21, t + 0.22, t + 0.23, t + 0.24, t + 0.25, t + 0.26],
                ];
                writer.write_step(t, &nodes, &d).unwrap();
            }
            assert_eq!(writer.current_step(), 3);
            writer.finish();
        }

        let batches = read_all(path_str).unwrap();
        let total_rows: usize = batches.iter().map(|b| b.num_rows()).sum();
        assert_eq!(total_rows, 9);
    }

    #[test]
    fn test_time_history_partial_read_step_range() {
        let dir = std::env::temp_dir();
        let path = dir.join("p6_th_step_range.parquet");
        let path_str = path.to_str().unwrap();

        {
            let mut writer = TimeHistoryWriter::create(path_str).unwrap();
            for s in 0..3 {
                writer
                    .write_step(s as f64 * 0.1, &[1, 2], &[[0.1; 6], [0.2; 6]])
                    .unwrap();
            }
            writer.finish();
        }

        let batches = read_time_history_range(path_str, Some((1, 2)), None).unwrap();
        let total_rows: usize = batches.iter().map(|b| b.num_rows()).sum();
        assert_eq!(total_rows, 4);

        for batch in &batches {
            let step_col = batch
                .column(0)
                .as_any()
                .downcast_ref::<UInt64Array>()
                .unwrap();
            for i in 0..batch.num_rows() {
                let s = step_col.value(i);
                assert!((1..=2).contains(&s));
            }
        }
    }

    #[test]
    fn test_time_history_partial_read_node_filter() {
        let dir = std::env::temp_dir();
        let path = dir.join("p6_th_node_filter.parquet");
        let path_str = path.to_str().unwrap();

        {
            let mut writer = TimeHistoryWriter::create(path_str).unwrap();
            writer
                .write_step(0.0, &[1, 2, 3], &[[0.1; 6], [0.2; 6], [0.3; 6]])
                .unwrap();
            writer.finish();
        }

        let batches = read_time_history_range(path_str, None, Some(&[1u32])).unwrap();
        let total_rows: usize = batches.iter().map(|b| b.num_rows()).sum();
        assert_eq!(total_rows, 1);

        let batch = &batches[0];
        let node_col = batch
            .column(2)
            .as_any()
            .downcast_ref::<UInt32Array>()
            .unwrap();
        assert_eq!(node_col.value(0), 1);
    }

    #[test]
    fn test_fs_result_store_writer_and_manifest() {
        let dir = std::env::temp_dir().join("p8_fsrs_basic");
        let _ = std::fs::remove_dir_all(&dir);
        let mut store = FsResultStore::open(&dir).unwrap();

        {
            let mut writer = store.writer(1, ResultKind::NodalDisp);
            let batch = nodal_disp_batch(
                &[1, 2, 3],
                &[
                    [0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.2, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.3, 0.0, 0.0, 0.0, 0.0, 0.0],
                ],
            )
            .unwrap();
            writer.write_rows(&batch);
            writer.finish();
        }
        store.sync().unwrap();

        let manifest = store.manifest();
        assert_eq!(manifest.entries.len(), 1);
        let entry = &manifest.entries[0];
        assert_eq!(entry.case, 1);
        assert_eq!(entry.kind, ResultKind::NodalDisp);
        assert_eq!(entry.rows, 3);

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn test_fs_result_store_rewrite_dedup() {
        let dir = std::env::temp_dir().join("p8_fsrs_dedup");
        let _ = std::fs::remove_dir_all(&dir);
        let mut store = FsResultStore::open(&dir).unwrap();

        for rows in [2usize, 4usize] {
            let mut writer = store.writer(1, ResultKind::NodalDisp);
            let ids: Vec<u32> = (0..rows as u32).collect();
            let disp: Vec<[f64; 6]> = vec![[0.0; 6]; rows];
            let batch = nodal_disp_batch(&ids, &disp).unwrap();
            writer.write_rows(&batch);
            writer.finish();
            store.sync().unwrap();
        }

        let manifest = store.manifest();
        assert_eq!(manifest.entries.len(), 1);
        assert_eq!(manifest.entries[0].rows, 4);

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn test_fs_result_store_query_node_filter() {
        let dir = std::env::temp_dir().join("p8_fsrs_query_filter");
        let _ = std::fs::remove_dir_all(&dir);
        let mut store = FsResultStore::open(&dir).unwrap();
        {
            let mut writer = store.writer(1, ResultKind::NodalDisp);
            let batch = nodal_disp_batch(&[1, 2, 3], &[[0.1; 6], [0.2; 6], [0.3; 6]]).unwrap();
            writer.write_rows(&batch);
            writer.finish();
        }
        store.sync().unwrap();

        let result = store.query(&ResultQuery {
            case: 1,
            kind: ResultKind::NodalDisp,
            node_filter: Some(vec![NodeId(2)]),
            member_filter: None,
            step_range: None,
        });
        assert_eq!(result.batch.num_rows(), 1);
        let node_col = result
            .batch
            .column(0)
            .as_any()
            .downcast_ref::<UInt32Array>()
            .unwrap();
        assert_eq!(node_col.value(0), 2);

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn test_fs_result_store_reopen_restores_manifest() {
        let dir = std::env::temp_dir().join("p8_fsrs_reopen");
        let _ = std::fs::remove_dir_all(&dir);
        {
            let mut store = FsResultStore::open(&dir).unwrap();
            let mut writer = store.writer(1, ResultKind::NodalDisp);
            let batch = nodal_disp_batch(&[1, 2], &[[0.1; 6], [0.2; 6]]).unwrap();
            writer.write_rows(&batch);
            writer.finish();
            store.sync().unwrap();
        }

        let store2 = FsResultStore::open(&dir).unwrap();
        let manifest = store2.manifest();
        assert_eq!(manifest.entries.len(), 1);
        assert_eq!(manifest.entries[0].rows, 2);
        assert_eq!(manifest.entries[0].case, 1);

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn test_fs_result_store_time_history_step_range_query() {
        let dir = std::env::temp_dir().join("p8_fsrs_th_steprange");
        let _ = std::fs::remove_dir_all(&dir);
        let mut store = FsResultStore::open(&dir).unwrap();
        {
            let mut writer = store.writer(1, ResultKind::TimeHistory);
            for s in 0..3u64 {
                let batch =
                    time_history_batch(s, s as f64 * 0.1, &[1, 2], &[[0.1; 6], [0.2; 6]]).unwrap();
                writer.write_rows(&batch);
            }
            writer.finish();
        }
        store.sync().unwrap();

        let result = store.query(&ResultQuery {
            case: 1,
            kind: ResultKind::TimeHistory,
            node_filter: None,
            member_filter: None,
            step_range: Some((1, 2)),
        });
        assert_eq!(result.batch.num_rows(), 4);
        let step_col = result
            .batch
            .column(0)
            .as_any()
            .downcast_ref::<UInt64Array>()
            .unwrap();
        for i in 0..result.batch.num_rows() {
            let s = step_col.value(i);
            assert!((1..=2).contains(&s));
        }

        let _ = std::fs::remove_dir_all(&dir);
    }
}
