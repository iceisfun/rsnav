//! Constrained Delaunay triangulation.
//!
//! An independent Rust implementation of the constrained Delaunay
//! triangulation algorithms popularized by Jonathan Richard Shewchuk's
//! *Triangle*, restricted to the CDT subset: no quality refinement, no
//! Steiner-point insertion, no Voronoi output. It is built from the
//! published algorithms and other implementations and is validated for
//! agreement against `triangle.c`, but it is **not** a port, copy, or
//! translation of Triangle's source, and is **not** covered by Triangle's
//! license. Credit to Shewchuk's work — see the crate README.
//!
//! The robust geometric predicates ([`orient2d`], [`incircle`]) follow
//! Shewchuk's separately public-domain adaptive-precision predicates.

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
