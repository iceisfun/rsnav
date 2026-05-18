//! Constrained Delaunay triangulation.
//!
//! Direct Rust port of Jonathan Shewchuk's Triangle 1.6 (`triangle.c`/`triangle.h`),
//! restricted to the CDT subset (`-DCDT_ONLY` build equivalent): no quality refinement,
//! no Steiner-point insertion, no Voronoi output.
//!
//! Work in progress — see crate README for porting status.

pub mod flip;
pub mod io;
pub mod mesh;
pub mod predicates;
pub mod pslg;
pub mod sort;

pub use mesh::{CdtMesh, Otri, Osub, VertexSlot, VertexType};
pub use predicates::{incircle, orient2d};
pub use pslg::{Pslg, PslgHole, PslgSegment, PslgVertex};
