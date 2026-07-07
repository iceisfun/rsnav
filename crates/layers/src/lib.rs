//! Span-heightfield layer discovery: triangle soup → multi-layer
//! [`rsnav_world::World`].
//!
//! The front end rsnav's 3D story was missing. The 2D pipeline
//! (`Pslg → CDT → NavMesh`) and the world stitcher both assume someone
//! already decomposed the 3D walkable surface into planar-projectable
//! layers with conformed seams; this crate does that automatically:
//!
//! ```text
//!   PolySoup                       (rsnav-voxel)
//!     → VoxelGrid                  slope-classified rasterization
//!     → CompactHeightfield         stacked walkable spans + step links
//!     → Assignment                 conflict-aware greedy layer growth
//!     → tagged outlines            walls vs seams, conformed chains
//!     → per-layer Pslg → NavMesh   (rsnav-triangle / rsnav-navmesh)
//!     → World                      (rsnav-world)
//! ```
//!
//! **Where the cuts land.** A layer grows by BFS over walk-links and
//! admits a span unless it conflicts — same column already occupied, or
//! an 8-adjacent cell of the layer out of vertical step reach. Growth
//! therefore stops exactly where the surface begins to overlap itself
//! (a bridge deck returning over the ground, a ramp cresting beside the
//! floor it left): the rim of the vertical-overlap set. Open continuous
//! ground never conflicts with itself, so no seam ever crosses it. The
//! cut links are step-continuous floor by construction; they become
//! conformed connection edges — identical vertex chains and identical
//! heights on both sides — which is what `World::build` matches
//! bit-exactly.
//!
//! Residual limits: a self-overlapping sheet (helix) splits at the BFS
//! collision frontier (valid but arbitrary placement), and surfaces
//! that only exist through interpenetrating art still depend on the
//! voxel pass to see them.

#![forbid(unsafe_code)]

pub mod assign;
pub mod mesh;
pub mod outline;

use std::collections::{BTreeSet, HashMap};

use rsnav_common::Vec3;
use rsnav_voxel::{classify_walkability, PolySoup, VoxelGrid, WalkabilityConfig};
use rsnav_world::{World, WorldBuildError};

pub use assign::{assign_layers, Assignment, Span, UNASSIGNED};
pub use mesh::LayerMeshError;

/// Configuration for [`build_layered_world`].
#[derive(Clone, Debug)]
pub struct LayersConfig {
    /// Voxel edge length in world units.
    pub voxel_size: f64,
    /// Slope / step / clearance rules (shared with rsnav-voxel).
    pub walkability: WalkabilityConfig,
    /// Layers with fewer spans than this are pruned as dust (isolated
    /// ledges, rasterization noise).
    pub min_layer_spans: usize,
    /// Interior surface heights are sampled every N grid vertices
    /// (`0` = boundary heights only; flat interiors are always skipped
    /// since boundary interpolation is exact there).
    pub height_sample_step: u32,
    /// Empty voxel border around the geometry.
    pub padding_cells: u32,
}

impl Default for LayersConfig {
    fn default() -> Self {
        Self {
            voxel_size: 0.2,
            walkability: WalkabilityConfig::default(),
            min_layer_spans: 8,
            height_sample_step: 2,
            padding_cells: 1,
        }
    }
}

/// Per-layer summary in [`LayerStats`].
#[derive(Clone, Debug)]
pub struct LayerInfo {
    pub spans: usize,
    pub triangles: usize,
    pub walkable_area: f64,
}

/// Build summary.
#[derive(Clone, Debug, Default)]
pub struct LayerStats {
    pub walkable_spans: usize,
    pub pruned_spans: usize,
    pub seam_links: usize,
    pub connections: usize,
    pub per_layer: Vec<LayerInfo>,
}

/// A stitched multi-layer world plus the frame it lives in.
pub struct LayeredWorld {
    pub world: World,
    /// World-space position of the meshes' local origin: mesh XY
    /// coordinates are relative to `(origin.x, origin.y)` (keeps CDT
    /// predicates well-conditioned for far-from-origin geometry);
    /// heights (`vertex_z`, path point `z`) are world-space absolute.
    pub origin: Vec3,
    pub cell_size: f64,
    pub stats: LayerStats,
}

#[derive(Debug)]
pub enum LayersError {
    /// Nothing walkable survived voxelization + filtering.
    NoWalkableArea,
    Mesh(LayerMeshError),
    World(WorldBuildError),
}

impl std::fmt::Display for LayersError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LayersError::NoWalkableArea => write!(f, "no walkable area in input geometry"),
            LayersError::Mesh(e) => write!(f, "layer meshing failed: {e}"),
            LayersError::World(e) => write!(f, "world stitching failed: {e}"),
        }
    }
}

impl std::error::Error for LayersError {}

impl From<LayerMeshError> for LayersError {
    fn from(e: LayerMeshError) -> Self {
        Self::Mesh(e)
    }
}
impl From<WorldBuildError> for LayersError {
    fn from(e: WorldBuildError) -> Self {
        Self::World(e)
    }
}

/// Turn a raw triangle soup into a stitched multi-layer [`World`].
pub fn build_layered_world(
    soup: &PolySoup,
    cfg: &LayersConfig,
) -> Result<LayeredWorld, LayersError> {
    // A step must span at least two voxel layers, or the strip split
    // where a rising surface leaves its floor jumps farther than a
    // walk-link can reach — the upper surface then isolates with no
    // seam at all. (The gap at a split is two voxel layers by
    // construction of the heightfield's strip collapsing.)
    assert!(
        cfg.walkability.max_step_layers(cfg.voxel_size) >= 2,
        "voxel_size {} too coarse for max_step_height {}: need voxel_size <= step/2",
        cfg.voxel_size,
        cfg.walkability.max_step_height,
    );
    let grid = VoxelGrid::from_polysoup(
        soup,
        cfg.voxel_size,
        cfg.padding_cells,
        cfg.walkability.cos_max_slope(),
    );
    let chf = classify_walkability(&grid, &cfg.walkability);
    let asg = assign_layers(&chf, cfg.walkability.max_step_layers(chf.cell_size), cfg.min_layer_spans);
    if asg.layer_count == 0 {
        return Err(LayersError::NoWalkableArea);
    }

    // Per-layer plan-view cell maps.
    let mut cells: Vec<HashMap<(u32, u32), u32>> =
        vec![HashMap::new(); asg.layer_count as usize];
    for (sid, &l) in asg.layer_of.iter().enumerate() {
        if l != UNASSIGNED {
            let sp = &asg.spans[sid];
            cells[l as usize].insert((sp.c, sp.r), sid as u32);
        }
    }

    // Connection id per unordered layer pair, allocated in sorted order
    // so both sides agree.
    let mut pairs: BTreeSet<(u32, u32)> = BTreeSet::new();
    let mut seam_links = 0usize;
    for (s, t) in asg.seam_links() {
        let (a, b) = (asg.layer_of[s as usize], asg.layer_of[t as usize]);
        pairs.insert((a.min(b), a.max(b)));
        seam_links += 1;
    }
    let pair_ids: HashMap<(u32, u32), u32> = pairs
        .iter()
        .enumerate()
        .map(|(i, &p)| (p, i as u32))
        .collect();

    let mut meshes = Vec::with_capacity(asg.layer_count as usize);
    let mut per_layer = Vec::with_capacity(asg.layer_count as usize);
    for layer in 0..asg.layer_count {
        let nav = mesh::build_layer_mesh(
            &asg,
            &cells[layer as usize],
            layer,
            chf.cell_size,
            cfg.walkability.max_step_layers(chf.cell_size),
            cfg.height_sample_step,
            &pair_ids,
        )?;
        per_layer.push(LayerInfo {
            spans: cells[layer as usize].len(),
            triangles: nav.triangle_count(),
            walkable_area: nav.triangles.iter().map(|t| t.area).sum(),
        });
        meshes.push(nav);
    }

    let world = World::build(meshes)?;
    Ok(LayeredWorld {
        world,
        origin: chf.origin,
        cell_size: chf.cell_size,
        stats: LayerStats {
            walkable_spans: asg.spans.len() - asg.pruned_spans,
            pruned_spans: asg.pruned_spans,
            seam_links,
            connections: pairs.len(),
            per_layer,
        },
    })
}
