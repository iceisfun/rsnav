//! Path search and queries over an [`rsnav_navmesh::NavMesh`].
//!
//! End-user entry points:
//!
//! - [`find_path`] — A* across triangle adjacency, then funnel/string-pull
//!   to a polyline. Honors `PathOptions::distance_from_wall`.
//! - [`line_of_sight`] — walk a directed segment triangle-by-triangle,
//!   stopping at the first constrained / boundary edge it hits.
//! - [`path_clear`] — revalidate a planned polyline leg-by-leg against
//!   the current mesh; the cheap "do I need to replan?" check after the
//!   navmesh has changed.
//! - [`nearest_point`] — thin wrapper around [`rsnav_bsp::Bsp::nearest`]
//!   that exposes the result as a [`Vertex`] plus the triangle it landed in.
//! - [`visibility_region`] — an approximate visibility polygon from a
//!   point, ray-cast at a configurable angular resolution.

#![forbid(unsafe_code)]

pub mod astar;
pub mod funnel;
pub mod los;
pub mod path;
pub mod visibility;
pub mod wall;

pub use los::{line_of_sight, LineOfSightResult};
pub use path::{
    find_path, nearest_point, path_clear, NearestPoint, PathError, PathOptions, PathResult,
};
pub use visibility::{visibility_region, VisibilityRegion};
