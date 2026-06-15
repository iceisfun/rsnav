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
//!
//! Doors ([`DoorSet`]) cut internal portal edges at runtime: a *closed* door
//! behaves as a wall for A*, the funnel, and line-of-sight. Build a
//! [`WallInfo`] with [`WallInfo::from_navmesh_with_doors`] and pass it to
//! [`find_path_with_walls`], [`line_of_sight`], [`path_clear`], and
//! [`visibility_region`]; the mesh and [`Bsp`](rsnav_bsp::Bsp) are never
//! rebuilt when a door toggles.

#![forbid(unsafe_code)]

pub mod astar;
pub mod doors;
pub mod funnel;
pub mod los;
pub mod path;
pub mod tiled;
pub mod visibility;
pub mod wall;
pub mod wall_clearance;
pub mod world;

pub use doors::{nearest_portal_edge, resolve_door_edges, Door, DoorId, DoorSet, DoorState};
pub use los::{line_of_sight, LineOfSightResult};
pub use path::{
    find_path, find_path_with_walls, nearest_point, path_clear, NearestPoint, PathError,
    PathOptions, PathResult,
};
pub use tiled::{GlobalTri, Link, TileId, TiledWorld};
pub use visibility::{visibility_region, VisibilityRegion};
pub use wall::WallInfo;
pub use wall_clearance::WallClearance;
pub use world::{zone_crossings, NavMetadata, NavWorld, NoMetadata, ZoneCrossing};
