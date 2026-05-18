//! Path search and queries over an [`rsnav_navmesh::NavMesh`].
//!
//! Three end-user entry points:
//!
//! - [`find_path`] — A* across triangle adjacency, then funnel/string-pull
//!   to a polyline. Honors `PathOptions::distance_from_wall`.
//! - [`line_of_sight`] — walk a directed segment triangle-by-triangle,
//!   stopping at the first constrained / boundary edge it hits.
//! - [`nearest_point`] — thin wrapper around [`rsnav_bsp::Bsp::nearest`]
//!   that exposes the result as a [`Vertex`] plus the triangle it landed in.
//!
//! Visibility region (polygon-from-point) is intentionally deferred.

#![forbid(unsafe_code)]

pub mod astar;
pub mod funnel;
pub mod los;
pub mod path;
pub mod wall;

pub use los::{line_of_sight, LineOfSightResult};
pub use path::{find_path, nearest_point, NearestPoint, PathError, PathOptions, PathResult};
