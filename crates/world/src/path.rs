//! Cross-layer pathfinding: world A* plus per-layer funnel stitching.
//!
//! The corridor comes from [`crate::astar`], which crosses seams as
//! ordinary portals. String-pulling currently runs **per layer run**:
//! the corridor is split at seam crossings and each run is funneled
//! from its entry point to the next crossing point with the ordinary 2D
//! funnel. That renders correct, wall-respecting paths, but the path is
//! pinned to the exact crossing point A* happened to enter each layer
//! at — expect a small kink at seams until the concatenated cross-seam
//! funnel (hinge unfolding) replaces this splice.

use rsnav_common::{TriangleId, Vertex};
use rsnav_navigation::funnel::funnel;

use crate::astar::{world_astar, CorridorStep, WorldAstarError};
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

        let points = self.stitch_funnel(&corridor, start, goal, opts.distance_from_wall);
        let triangles = corridor.iter().map(|s| (s.layer, s.tri)).collect();
        Ok(WorldPath { points, triangles })
    }

    /// Split the corridor into per-layer runs at seam crossings and
    /// funnel each run independently.
    fn stitch_funnel(
        &self,
        corridor: &[CorridorStep],
        start: WorldPoint,
        goal: WorldPoint,
        distance_from_wall: f64,
    ) -> Vec<WorldPathPoint> {
        let mut points: Vec<WorldPathPoint> = Vec::new();
        let mut run_start = 0usize;
        let mut leg_from = start.pos;

        for i in 0..corridor.len() {
            let last_of_run =
                i + 1 == corridor.len() || corridor[i + 1].layer != corridor[i].layer;
            if !last_of_run {
                continue;
            }
            let layer_id = corridor[run_start].layer;
            let layer = self.layer(layer_id);
            // The run's exit point: the entry point of the next run's
            // first triangle (a seam crossing), or the goal.
            let leg_to = if i + 1 == corridor.len() {
                goal.pos
            } else {
                corridor[i + 1].entry
            };
            let tris: Vec<TriangleId> =
                corridor[run_start..=i].iter().map(|s| s.tri).collect();
            let leg = funnel(
                &layer.navmesh,
                &layer.walls,
                &tris,
                leg_from,
                leg_to,
                distance_from_wall,
            );
            // First leg keeps its start point; later legs already have
            // the crossing point as their predecessor's last point.
            let skip = usize::from(run_start != 0);
            let leg_len = leg.len();
            for (k, p) in leg.into_iter().enumerate().skip(skip) {
                // The leg's last point at a seam is the next run's entry
                // point — A* already knows its height exactly.
                let z = if k + 1 == leg_len && i + 1 < corridor.len() {
                    corridor[i + 1].entry_z
                } else {
                    let tri = layer
                        .bsp
                        .locate(&layer.navmesh, p)
                        .unwrap_or(corridor[run_start].tri);
                    layer.navmesh.z_at(tri, p)
                };
                points.push(WorldPathPoint {
                    layer: layer_id,
                    pos: p,
                    z,
                });
            }
            leg_from = leg_to;
            run_start = i + 1;
        }
        if corridor.is_empty() {
            points.push(WorldPathPoint {
                layer: start.layer,
                pos: start.pos,
                z: 0.0,
            });
            points.push(WorldPathPoint {
                layer: goal.layer,
                pos: goal.pos,
                z: 0.0,
            });
        }
        points
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
