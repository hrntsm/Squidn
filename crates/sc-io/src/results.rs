use arrow::array::{Float64Array, UInt32Array};
use arrow::datatypes::{DataType, Field, Schema};
use arrow::record_batch::RecordBatch;
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
use parquet::arrow::arrow_writer::ArrowWriter;
use parquet::arrow::ProjectionMask;
use parquet::file::properties::WriterProperties;
use sc_core::ids::{ElemId, NodeId};
use std::fs::File;
use std::sync::Arc;

pub type CaseId = u32;

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ResultKind {
    NodalDisp,
    MemberForce,
    Story,
    Modal,
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

pub trait ResultStore {
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
}
