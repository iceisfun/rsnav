//! Cross-layer pathfinding: world A* plus the cross-seam funnel.
//!
//! The corridor comes from [`crate::astar`], which crosses seams as
//! ordinary portals. String-pulling runs **once over the whole
//! corridor** ([`crate::funnel`]): portals from every layer concatenate
//! in the shared horizontal frame, self-overlapping stretches (stacked
//! floors, switchbacks) are hinge-unfolded first, and the single pull
//! crosses every seam without a kink or restart.

use rsnav_common::{TriangleId, Vertex};

use crate::astar::{world_astar, WorldAstarError};
use crate::funnel::cross_seam_funnel;
use crate::{LayerId, World};

/// A position on one layer of the world.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct WorldPoint {
    pub layer: LayerId,
    pub pos: Vertex,
}

/// Options for [`World::find_path`]. Mirrors
/// [`rsnav_navigation::PathOptions`].
#[derive(Copy, Clone, Debug, Default)]
pub struct WorldPathOptions {
    /// Agent clearance radius; see
    /// [`rsnav_navigation::PathOptions::distance_from_wall`]. Applies on
    /// every layer. Seam edges are not walls, so clearance never pinches
    /// a path against a seam.
    pub distance_from_wall: f64,
}

/// Reasons [`World::find_path`] can fail.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum WorldPathError {
    /// `start.pos` doesn't lie inside any triangle of `start.layer`.
    StartOutsideMesh,
    /// `goal.pos` doesn't lie inside any triangle of `goal.layer`.
    GoalOutsideMesh,
    /// Both endpoints are on their meshes but no route connects them —
    /// different world components, or every connecting portal too
    /// narrow for the agent.
    Unreachable,
}

/// A cross-layer path.
#[derive(Clone, Debug)]
pub struct WorldPath {
    /// String-pulled polyline from start to goal, inclusive. Each point
    /// carries the layer it lies on and its interpolated height; a seam
    /// crossing appears once, attributed to the layer being *exited*
    /// (its position and height are identical on both layers).
    pub points: Vec<WorldPathPoint>,
    /// The `(layer, triangle)` corridor A* selected.
    pub triangles: Vec<(LayerId, TriangleId)>,
}

/// One corner of a [`WorldPath`].
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct WorldPathPoint {
    pub layer: LayerId,
    pub pos: Vertex,
    pub z: f64,
}

impl WorldPath {
    /// Total 3D length of the polyline.
    pub fn length(&self) -> f64 {
        self.points
            .windows(2)
            .map(|w| {
                let dx = w[1].pos.x - w[0].pos.x;
                let dy = w[1].pos.y - w[0].pos.y;
                let dz = w[1].z - w[0].z;
                (dx * dx + dy * dy + dz * dz).sqrt()
            })
            .sum()
    }
}

impl World {
    /// Find a string-pulled path from `start` to `goal`, crossing seams
    /// as needed. Both endpoints must already lie on their layer's mesh
    /// (snap first via the layer's [`rsnav_bsp::Bsp::nearest`] if they
    /// might be off-mesh).
    pub fn find_path(
        &self,
        start: WorldPoint,
        goal: WorldPoint,
        opts: &WorldPathOptions,
    ) -> Result<WorldPath, WorldPathError> {
        let start_layer = self.layer(start.layer);
        let goal_layer = self.layer(goal.layer);
        let start_tri = start_layer
            .bsp
            .locate(&start_layer.navmesh, start.pos)
            .ok_or(WorldPathError::StartOutsideMesh)?;
        let goal_tri = goal_layer
            .bsp
            .locate(&goal_layer.navmesh, goal.pos)
            .ok_or(WorldPathError::GoalOutsideMesh)?;

        let corridor = world_astar(
            self,
            (start.layer, start_tri),
            (goal.layer, goal_tri),
            start.pos,
            goal.pos,
            opts.distance_from_wall,
        )
        .map_err(|e| match e {
            WorldAstarError::UnreachableComponent | WorldAstarError::Unreachable => {
                WorldPathError::Unreachable
            }
        })?;

        let points = cross_seam_funnel(self, &corridor, start, goal, opts.distance_from_wall);
        let triangles = corridor.iter().map(|s| (s.layer, s.tri)).collect();
        Ok(WorldPath { points, triangles })
    }
}

/// Convenience free-function mirror of [`World::find_path`].
pub fn world_find_path(
    world: &World,
    start: WorldPoint,
    goal: WorldPoint,
    opts: &WorldPathOptions,
) -> Result<WorldPath, WorldPathError> {
    world.find_path(start, goal, opts)
}
