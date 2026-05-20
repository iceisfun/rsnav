//! Top-level pathfinding API: [`find_path`] = A* + funnel.

use rsnav_bsp::Bsp;
use rsnav_common::{TriangleId, Vertex};
use rsnav_navmesh::NavMesh;

use crate::astar::{astar, AstarError};
use crate::funnel::funnel;
use crate::los::{line_of_sight, LineOfSightResult};
use crate::wall::WallInfo;

/// Options that tune `find_path`.
#[derive(Copy, Clone, Debug)]
pub struct PathOptions {
    /// Agent clearance radius — how far the routed path is kept from
    /// any constrained / boundary edge. Models a disc-shaped agent of
    /// this radius. When `> 0`:
    /// - A* rejects a portal unless it is wide enough for the agent
    ///   *body* to pass: the edge length must exceed the inward shift
    ///   the funnel will apply — `distance_from_wall` for each portal
    ///   endpoint that is a wall vertex. A portal flanked by two walls
    ///   therefore needs more than `2 * distance_from_wall` of width.
    /// - Funnel shifts each portal endpoint that sits on a wall vertex
    ///   inward along the portal by `distance_from_wall`. When both
    ///   endpoints are wall vertices and the portal is too short to
    ///   accommodate both shifts, the portal collapses to its midpoint
    ///   (the path is forced through the narrow gap but never crosses
    ///   the wall).
    ///
    /// The two stages share one wall-vertex set, so A* never routes
    /// through a corridor the funnel would collapse below body width.
    pub distance_from_wall: f64,
}

impl Default for PathOptions {
    fn default() -> Self {
        Self {
            distance_from_wall: 0.0,
        }
    }
}

/// Reasons `find_path` can fail.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PathError {
    /// `start` doesn't lie inside any triangle of the mesh.
    StartOutsideMesh,
    /// `goal` doesn't lie inside any triangle of the mesh.
    GoalOutsideMesh,
    /// Both endpoints are inside the mesh but A* could not connect them
    /// (different regions, or every connecting portal too narrow).
    Unreachable,
}

#[derive(Clone, Debug)]
pub struct PathResult {
    /// Polyline from `start` to `goal`, inclusive.
    pub points: Vec<Vertex>,
    /// Triangle sequence A* selected. Useful for diagnostics / debug
    /// rendering.
    pub triangles: Vec<TriangleId>,
}

/// Run A* + funnel to produce a string-pulled path from `start` to `goal`.
pub fn find_path(
    nav: &NavMesh,
    bsp: &Bsp,
    start: Vertex,
    goal: Vertex,
    opts: &PathOptions,
) -> Result<PathResult, PathError> {
    let start_tri = bsp.locate(nav, start).ok_or(PathError::StartOutsideMesh)?;
    let goal_tri = bsp.locate(nav, goal).ok_or(PathError::GoalOutsideMesh)?;

    // Built once and shared by A* and the funnel so the two stages
    // apply the *same* clearance model — A* won't route through a
    // portal the funnel would then have to collapse.
    let walls = WallInfo::from_navmesh(nav);

    let triangles = astar(nav, &walls, start_tri, goal_tri, goal, opts.distance_from_wall)
        .map_err(|e| match e {
            AstarError::UnreachableRegion | AstarError::Unreachable => PathError::Unreachable,
        })?;

    let points = funnel(nav, &walls, &triangles, start, goal, opts.distance_from_wall);

    Ok(PathResult { points, triangles })
}

// --- Path revalidation ---------------------------------------------------

/// Check that every segment of the polyline `points` can be walked on
/// the current mesh — no segment crosses a constrained edge or leaves
/// the mesh.
///
/// This is the cheap way to revalidate a previously-planned path after
/// the navmesh changes (a building went up, a forest spawned). A
/// corner-only on-mesh test is *not* enough: a new obstacle can land
/// between two corners that both still locate fine, leaving the
/// straight leg between them blocked. `path_clear` walks each segment
/// with [`line_of_sight`], so it catches that case.
///
/// Pass only the part of the route the agent has yet to traverse —
/// typically `[agent_pos, remaining_corners..]`. A `false` result
/// means: replan. Returns `true` for an empty or single-point slice
/// (nothing to walk).
pub fn path_clear(nav: &NavMesh, bsp: &Bsp, points: &[Vertex]) -> bool {
    for seg in points.windows(2) {
        let (a, b) = (seg[0], seg[1]);
        let Some(start_tri) = bsp.locate(nav, a) else {
            return false; // segment starts off the mesh
        };
        if !matches!(
            line_of_sight(nav, start_tri, a, b),
            LineOfSightResult::Clear,
        ) {
            return false;
        }
    }
    true
}

// --- Nearest-point convenience -------------------------------------------

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct NearestPoint {
    pub triangle: TriangleId,
    pub point: Vertex,
    pub distance: f64,
}

/// Snap `p` to the closest point on the navmesh. Returns `None` only when
/// the mesh is empty.
pub fn nearest_point(nav: &NavMesh, bsp: &Bsp, p: Vertex) -> Option<NearestPoint> {
    bsp.nearest(nav, p).map(|n| NearestPoint {
        triangle: n.triangle,
        point: n.point,
        distance: n.distance,
    })
}

// --- Tests ---------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::los::{line_of_sight, LineOfSightResult};
    use rsnav_common::Vertex;
    use rsnav_navmesh::build_from_cdt;
    use rsnav_triangle::pslg::{Pslg, PslgHole, PslgSegment, PslgVertex};
    use rsnav_triangle::{
        carve_holes, delaunay, form_skeleton, CdtMesh, DivConqOptions, VertexSlot,
    };

    fn build_square_with_hole_navmesh() -> (NavMesh, Bsp) {
        let pts = [
            (0.0, 0.0),
            (4.0, 0.0),
            (4.0, 4.0),
            (0.0, 4.0),
            (1.5, 1.5),
            (2.5, 1.5),
            (2.5, 2.5),
            (1.5, 2.5),
        ];
        let mut mesh = CdtMesh::new();
        for (x, y) in pts {
            mesh.push_vertex(VertexSlot::new(Vertex::new(x, y), 0));
        }
        delaunay(&mut mesh, DivConqOptions::default());
        let pslg = Pslg {
            vertices: pts
                .iter()
                .map(|(x, y)| PslgVertex::new(Vertex::new(*x, *y)))
                .collect(),
            segments: vec![
                PslgSegment { a: 0, b: 1, marker: 1 },
                PslgSegment { a: 1, b: 2, marker: 1 },
                PslgSegment { a: 2, b: 3, marker: 1 },
                PslgSegment { a: 3, b: 0, marker: 1 },
                PslgSegment { a: 4, b: 5, marker: 2 },
                PslgSegment { a: 5, b: 6, marker: 2 },
                PslgSegment { a: 6, b: 7, marker: 2 },
                PslgSegment { a: 7, b: 4, marker: 2 },
            ],
            holes: vec![PslgHole { point: Vertex::new(2.0, 2.0) }],
        };
        form_skeleton(&mut mesh, &pslg, None).unwrap();
        carve_holes(&mut mesh, &pslg, false);
        let nav = build_from_cdt(&mesh);
        let bsp = Bsp::build(&nav);
        (nav, bsp)
    }

    #[test]
    fn straight_line_path_inside_open_region() {
        let (nav, bsp) = build_square_with_hole_navmesh();
        // Bottom corridor: y < 1.5 is fully walkable.
        let start = Vertex::new(0.5, 0.5);
        let goal = Vertex::new(3.5, 0.5);
        let path = find_path(&nav, &bsp, start, goal, &PathOptions::default()).unwrap();
        // The string-pulled path between two visible points in the same
        // open band must be just [start, goal] — no detour.
        assert_eq!(path.points.first(), Some(&start));
        assert_eq!(path.points.last(), Some(&goal));
        // Direct straight line should be 3.0 long.
        let total: f64 = path.points.windows(2).map(|w| w[0].distance(w[1])).sum();
        assert!(
            (total - 3.0).abs() < 1e-9,
            "expected straight 3.0, got {} via {:?}",
            total,
            path.points
        );
    }

    #[test]
    fn path_around_hole_bends_at_corner() {
        let (nav, bsp) = build_square_with_hole_navmesh();
        // From bottom-left to top-right, with the central hole forcing a
        // detour around either side.
        let start = Vertex::new(0.5, 2.0);
        let goal = Vertex::new(3.5, 2.0);
        let path = find_path(&nav, &bsp, start, goal, &PathOptions::default()).unwrap();
        assert!(path.points.len() >= 3, "expected at least one bend: {:?}", path.points);
        // No point inside the hole.
        for p in &path.points {
            let in_hole = p.x > 1.5 && p.x < 2.5 && p.y > 1.5 && p.y < 2.5;
            assert!(!in_hole, "path point {:?} is inside the hole", p);
        }
        // The direct euclidean distance is 3.0; routed around the hole
        // must be longer.
        let total: f64 = path.points.windows(2).map(|w| w[0].distance(w[1])).sum();
        assert!(total > 3.0);
    }

    #[test]
    fn line_of_sight_through_open_region() {
        let (nav, bsp) = build_square_with_hole_navmesh();
        let a = Vertex::new(0.5, 0.5);
        let b = Vertex::new(3.5, 0.5);
        let start_tri = bsp.locate(&nav, a).unwrap();
        assert_eq!(line_of_sight(&nav, start_tri, a, b), LineOfSightResult::Clear);
    }

    #[test]
    fn line_of_sight_blocked_by_hole_wall() {
        let (nav, bsp) = build_square_with_hole_navmesh();
        // Horizontal segment across the middle of the hole.
        let a = Vertex::new(0.5, 2.0);
        let b = Vertex::new(3.5, 2.0);
        let start_tri = bsp.locate(&nav, a).unwrap();
        match line_of_sight(&nav, start_tri, a, b) {
            LineOfSightResult::Blocked { point } => {
                // Should be blocked at the inner-ring wall around x=1.5, y=2.0.
                assert!((point.x - 1.5).abs() < 1e-9, "blocked at {:?}", point);
                assert!((point.y - 2.0).abs() < 1e-9);
            }
            other => panic!("expected Blocked, got {:?}", other),
        }
    }

    /// Build a corridor with a narrow squeeze in the middle; A* should
    /// refuse to use the squeeze when `distance_from_wall` is larger than
    /// half the squeeze's width.
    fn build_squeeze_navmesh(squeeze_width: f64) -> (NavMesh, Bsp) {
        // Layout:  a 10-wide x 4-tall corridor, pinched at the centre by
        // two triangular bumps that bring the gap down to `squeeze_width`.
        //
        //     (0,4)──────────(10,4)
        //       │   \      /   │
        //       │    \    /    │
        //       │     ▼  ▼     │   ← bumps pointing into the corridor
        //       │     ▲  ▲     │
        //       │    /    \    │
        //       │   /      \   │
        //     (0,0)──────────(10,0)
        //
        // The four bump vertices live at x = 4 and x = 6, with y values
        // that leave a vertical gap of `squeeze_width` centred on y = 2.
        let half_gap = squeeze_width * 0.5;
        let pts: Vec<(f64, f64)> = vec![
            (0.0, 0.0),                 // 0
            (10.0, 0.0),                // 1
            (10.0, 4.0),                // 2
            (0.0, 4.0),                 // 3
            (4.0, 0.0),                 // 4  bottom-left of pinch
            (6.0, 0.0),                 // 5  bottom-right of pinch
            (4.0, 4.0),                 // 6  top-left of pinch
            (6.0, 4.0),                 // 7  top-right of pinch
            (5.0, 2.0 - half_gap),      // 8  inner tip of bottom bump
            (5.0, 2.0 + half_gap),      // 9  inner tip of top bump
        ];
        let mut mesh = CdtMesh::new();
        for (x, y) in &pts {
            mesh.push_vertex(VertexSlot::new(Vertex::new(*x, *y), 0));
        }
        delaunay(&mut mesh, DivConqOptions::default());
        let pslg = Pslg {
            vertices: pts
                .iter()
                .map(|(x, y)| PslgVertex::new(Vertex::new(*x, *y)))
                .collect(),
            // Outer rectangle.
            segments: vec![
                PslgSegment { a: 0, b: 4, marker: 1 },
                PslgSegment { a: 4, b: 5, marker: 1 },
                PslgSegment { a: 5, b: 1, marker: 1 },
                PslgSegment { a: 1, b: 2, marker: 1 },
                PslgSegment { a: 2, b: 7, marker: 1 },
                PslgSegment { a: 7, b: 6, marker: 1 },
                PslgSegment { a: 6, b: 3, marker: 1 },
                PslgSegment { a: 3, b: 0, marker: 1 },
                // Bottom bump (triangle 4 → 5 → 8).
                PslgSegment { a: 4, b: 8, marker: 1 },
                PslgSegment { a: 8, b: 5, marker: 1 },
                // Top bump (triangle 6 → 9 → 7, walking CCW around the hole).
                PslgSegment { a: 6, b: 9, marker: 1 },
                PslgSegment { a: 9, b: 7, marker: 1 },
            ],
            holes: vec![
                PslgHole { point: Vertex::new(5.0, 0.5) }, // inside bottom bump
                PslgHole { point: Vertex::new(5.0, 3.5) }, // inside top bump
            ],
        };
        form_skeleton(&mut mesh, &pslg, None).unwrap();
        carve_holes(&mut mesh, &pslg, false);
        let nav = build_from_cdt(&mesh);
        let bsp = Bsp::build(&nav);
        (nav, bsp)
    }

    #[test]
    fn distance_from_wall_zero_uses_narrow_squeeze() {
        // squeeze_width = 0.4 — the corridor narrows but is still
        // traversable for a point agent.
        let (nav, bsp) = build_squeeze_navmesh(0.4);
        let opts = PathOptions { distance_from_wall: 0.0 };
        let path = find_path(
            &nav,
            &bsp,
            Vertex::new(1.0, 2.0),
            Vertex::new(9.0, 2.0),
            &opts,
        )
        .expect("point agent should always find a path");
        // Direct distance is 8.0; the funnel may add tiny corrections
        // around the bump tips but should stay reasonably short.
        let total: f64 = path.points.windows(2).map(|w| w[0].distance(w[1])).sum();
        assert!(total < 9.0, "path was {} long: {:?}", total, path.points);
    }

    #[test]
    fn distance_from_wall_blocks_narrow_squeeze() {
        // squeeze_width = 0.4; require 0.5 clearance. The pinch portal is
        // too narrow → A* must refuse to use it; since the bumps fully
        // span the corridor, the goal is unreachable.
        let (nav, bsp) = build_squeeze_navmesh(0.4);
        let opts = PathOptions { distance_from_wall: 0.5 };
        let err = find_path(
            &nav,
            &bsp,
            Vertex::new(1.0, 2.0),
            Vertex::new(9.0, 2.0),
            &opts,
        )
        .unwrap_err();
        assert_eq!(err, PathError::Unreachable);
    }

    #[test]
    fn distance_from_wall_blocks_portal_narrower_than_body() {
        // Regression: A* clearance must reject a portal the agent's
        // *body* can't fit through, not just one narrower than a single
        // radius. squeeze_width = 0.7 with clearance 0.4: the pinch is
        // wider than one radius (0.4) but narrower than the full body
        // span the funnel reserves (0.4 on each wall-vertex side = 0.8).
        // The agent cannot pass, so A* must report the goal unreachable.
        // Before the fix A* only rejected portals `<= 0.4` and would
        // route a doomed path straight into the pinch.
        let (nav, bsp) = build_squeeze_navmesh(0.7);
        let opts = PathOptions { distance_from_wall: 0.4 };
        let err = find_path(
            &nav,
            &bsp,
            Vertex::new(1.0, 2.0),
            Vertex::new(9.0, 2.0),
            &opts,
        )
        .unwrap_err();
        assert_eq!(err, PathError::Unreachable);
    }

    #[test]
    fn distance_from_wall_pushes_path_off_walls() {
        // Squeeze of 2.0 — plenty wide for both 0.0 and 0.4 clearance.
        // With clearance 0.4, the funnel should keep the path roughly
        // off the bump tips (no point with y exactly at 1.0 or 3.0).
        let (nav, bsp) = build_squeeze_navmesh(2.0);
        let path_tight = find_path(
            &nav,
            &bsp,
            Vertex::new(1.0, 2.0),
            Vertex::new(9.0, 2.0),
            &PathOptions { distance_from_wall: 0.0 },
        )
        .unwrap();
        let path_safe = find_path(
            &nav,
            &bsp,
            Vertex::new(1.0, 2.0),
            Vertex::new(9.0, 2.0),
            &PathOptions { distance_from_wall: 0.4 },
        )
        .unwrap();

        // Both should reach the goal.
        assert_eq!(path_tight.points.last(), Some(&Vertex::new(9.0, 2.0)));
        assert_eq!(path_safe.points.last(), Some(&Vertex::new(9.0, 2.0)));
        // The clearance-aware path should never bend through a portal
        // endpoint that's on a wall — i.e. no path point should sit
        // exactly on a bump tip (y = 1.0 or y = 3.0 at x = 5.0).
        for p in &path_safe.points {
            let on_top_tip = (p.x - 5.0).abs() < 1e-9 && (p.y - 3.0).abs() < 1e-9;
            let on_bot_tip = (p.x - 5.0).abs() < 1e-9 && (p.y - 1.0).abs() < 1e-9;
            assert!(
                !on_top_tip && !on_bot_tip,
                "clearance path still bent at a wall tip: {:?}",
                p
            );
        }
    }

    #[test]
    fn nearest_point_snaps_outside_to_boundary() {
        let (nav, bsp) = build_square_with_hole_navmesh();
        let np = nearest_point(&nav, &bsp, Vertex::new(-1.0, 0.5)).unwrap();
        assert!((np.point.x - 0.0).abs() < 1e-9);
        assert!((np.distance - 1.0).abs() < 1e-9);
    }

    #[test]
    fn path_clear_accepts_open_routes_and_rejects_blocked() {
        let (nav, bsp) = build_square_with_hole_navmesh();

        // Straight leg through the open bottom band — clear.
        assert!(path_clear(
            &nav,
            &bsp,
            &[Vertex::new(0.5, 0.5), Vertex::new(3.5, 0.5)],
        ));

        // Multi-segment route that detours around the hole — clear.
        assert!(path_clear(
            &nav,
            &bsp,
            &[
                Vertex::new(0.5, 2.0),
                Vertex::new(0.5, 0.5),
                Vertex::new(3.5, 0.5),
                Vertex::new(3.5, 2.0),
            ],
        ));

        // Straight leg across the central hole — blocked.
        assert!(!path_clear(
            &nav,
            &bsp,
            &[Vertex::new(0.5, 2.0), Vertex::new(3.5, 2.0)],
        ));

        // A leg whose far end leaves the mesh — blocked.
        assert!(!path_clear(
            &nav,
            &bsp,
            &[Vertex::new(0.5, 0.5), Vertex::new(-1.0, 0.5)],
        ));

        // Nothing to walk — trivially clear.
        assert!(path_clear(&nav, &bsp, &[]));
        assert!(path_clear(&nav, &bsp, &[Vertex::new(0.5, 0.5)]));
    }
}
