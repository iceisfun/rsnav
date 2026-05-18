//! Runtime navmesh: a flat, query-friendly view of a CDT plus its derived
//! adjacency, per-triangle metadata, and reachability regions. Plus a
//! versioned little-endian binary file format.
//!
//! See `FORMAT.md` in this crate for the on-disk spec.

#![forbid(unsafe_code)]

pub mod binary;
pub mod build;
pub mod navmesh;

pub use binary::{LoadError, SaveError, FORMAT_VERSION, MAGIC};
pub use build::build_from_cdt;
pub use navmesh::{NavMesh, NavTriangle};
