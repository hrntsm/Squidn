pub mod spmv;

#[cfg(feature = "gpu")]
pub mod gpu_context;

#[cfg(feature = "gpu")]
mod pcg;

#[cfg(feature = "gpu")]
pub use gpu_context::GpuContext;

#[cfg(feature = "gpu")]
pub use pcg::PcgGpu;

pub use spmv::{make_spmv, CpuSpMv, SpMv};
