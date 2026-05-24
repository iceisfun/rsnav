//! Voxelize polygon-soup meshes and extract traversable regions.
//!
//! This crate owns the 3D → 2D bridge for the rsnav stack. Given an arbitrary
//! [`PolySoup`] (no manifold, watertight, or winding guarantees), the planned
//! pipeline is:
//!
//! ```text
//!     PolySoup
//!         ↓                          Voxelizer
//!     Occupancy voxel grid
//!         ↓                          Walkability classifier
//!     Compact heightfield  (top walkable surface + slope/step filter)
//!         ↓                          Distance-field + watershed (Recast-style)
//!     Region grid          (every walkable cell tagged with a RegionId)
//!         ↓                          Contour + portal extraction
//!     PipelineOutput { regions, portals }
//! ```
//!
//! Phase 1 of this crate establishes the **input** and **output** boundaries
//! plus the watershed core. Voxelization and walkability classification land
//! in subsequent commits.
//!
//! # Conventions
//!
//! - All coordinates are `f64`, matching the rest of `rsnav`.
//! - `Vec3.z` is treated as "up" by the walkability filter. Source data using
//!   any other convention must be reoriented (via [`MeshBuilder`]) before
//!   ingestion. The crate makes no attempt to auto-detect axis orientation.
//! - Determinism: all algorithms in this crate produce bit-identical output
//!   for the same input + config when run twice on the same machine. Set
//!   iteration order is never allowed to influence output — use [`BTreeMap`]
//!   or sort explicitly. (Stability across small input perturbations is
//!   **not** promised; that's a research problem we intentionally don't
//!   take on.)
//!
//! [`BTreeMap`]: std::collections::BTreeMap

#![forbid(unsafe_code)]
#![warn(missing_debug_implementations)]

pub mod builder;
pub mod config;
pub mod contour;
pub mod distance;
pub mod funnel;
pub mod grid;
pub mod navmesh;
pub mod output;
pub mod pathfind;
pub mod polysoup;
pub mod portals;
pub mod synth;
pub mod voxelize;
pub mod walkability;
pub mod watershed;

pub use builder::{MeshBuilder, Transform};
pub use config::{PipelineConfig, WalkabilityConfig, WatershedConfig};
pub use contour::{extract_contours, ContourVertex, RegionContour, RegionContours};
pub use distance::{build_distance_field, DistanceField};
pub use funnel::{
    channel_to_portals, find_triangle_at_xy, find_triangle_channel, funnel, funnel_in_region,
    triangle_adjacency,
};
pub use grid::{area_type, VoxelGrid};
pub use navmesh::{build_all_navmeshes, build_region_navmesh, AllRegionNavMeshes, RegionNavMesh};
pub use output::{Heightfield, PipelineOutput, Portal, Region, RegionId};
pub use pathfind::{
    densify_path_on_hf, densify_path_on_navmesh, find_path, find_region_at_xy,
    find_region_at_xyz, region_mean_z, sample_navmesh_z, Path,
};
pub use polysoup::{PolySoup, PolySoupError};
pub use portals::extract_portals;
pub use voxelize::{classify_triangle, rasterize, tri_box_overlap};
pub use walkability::{classify_walkability, CompactHeightfield, WalkableCell, NEIGHBOR_DELTAS};
pub use watershed::{assign_regions, segment, RegionMap};
