//! Constrained Delaunay triangulation.
//!
//! Direct Rust port of Jonathan Shewchuk's Triangle 1.6 (`triangle.c`/`triangle.h`),
//! restricted to the CDT subset (`-DCDT_ONLY` build equivalent): no quality refinement,
//! no Steiner-point insertion, no Voronoi output.
//!
//! Work in progress — see crate README for porting status.

pub mod clip;
pub mod divconq;
pub mod flip;
pub mod holes;
pub mod inset;
pub mod io;
pub mod mesh;
pub mod predicates;
pub mod pslg;
pub mod segment;
pub mod sort;
pub mod winding;

pub use clip::clip_ears;
pub use divconq::{delaunay, DivConqOptions};
pub use holes::carve_holes;
pub use inset::{build_cdt_with_inset, InsetBuild, InsetError, InsetOptions, InsetRing, RingKind};
pub use mesh::{CdtMesh, Otri, Osub, VertexSlot, VertexType};
pub use predicates::{incircle, orient2d};
pub use pslg::{Pslg, PslgHole, PslgSegment, PslgVertex};
pub use winding::{carve_by_winding, drop_interior_constraints, winding_number};
pub use segment::{
    form_skeleton, insert_segment, make_vertex_map, mark_hull, SegmentInsertError,
};
