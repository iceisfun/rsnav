//! Shared 2D geometry primitives for the rsnav workspace.
//!
//! All coordinates are `f64`. Indices into vertex/triangle/polygon arrays are `u32`,
//! exposed as the [`VertexId`], [`TriangleId`], and [`PolygonId`] newtypes.

#![forbid(unsafe_code)]
#![warn(missing_debug_implementations)]

pub mod aabb;
pub mod geom;
pub mod ids;
pub mod mesh2d;
pub mod par;
pub mod polygon;
pub mod triangle;
pub mod vertex;

pub use aabb::Aabb;
pub use ids::{PolygonId, TriangleId, VertexId};
pub use mesh2d::Mesh2d;
pub use polygon::{Polygon, PolygonWithHoles, Winding};
pub use triangle::Triangle;
pub use vertex::Vertex;
