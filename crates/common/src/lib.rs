//! Shared geometry primitives for the rsnav workspace.
//!
//! All coordinates are `f64`. Indices into vertex/triangle/polygon arrays are `u32`,
//! exposed as the [`VertexId`], [`TriangleId`], and [`PolygonId`] newtypes.
//!
//! The crate's primary surface is 2D ([`Vertex`], [`Aabb`], [`Polygon`], ...) — that's
//! what the navigation stack runs on. Sibling 3D primitives [`Vec3`] and [`Aabb3`] live
//! here too; they're used by the voxel / region-extraction pipeline and are deliberately
//! kept thin (no cross-pollination with the 2D types beyond what's needed).

#![forbid(unsafe_code)]
#![warn(missing_debug_implementations)]

pub mod aabb;
pub mod aabb3;
pub mod geom;
pub mod ids;
pub mod mesh2d;
pub mod polygon;
pub mod triangle;
pub mod vec3;
pub mod vertex;

pub use aabb::Aabb;
pub use aabb3::Aabb3;
pub use ids::{PolygonId, TriangleId, VertexId};
pub use mesh2d::Mesh2d;
pub use polygon::{Polygon, PolygonWithHoles, Winding};
pub use triangle::Triangle;
pub use vec3::Vec3;
pub use vertex::Vertex;
