//! Keep a free-moving agent a fixed distance off the walls.
//!
//! [`find_path`](crate::find_path) already honors `PathOptions::distance_from_wall`,
//! so a *pathing* agent never hugs a wall. But an agent you move by hand —
//! WASD, a steering controller, knockback, anything that isn't the planner —
//! can walk its center right up to a constrained or boundary edge.
//! [`WallClearance`] fixes that without a second, geometrically-inset mesh:
//! it precomputes the mesh's wall segments once, then [`clamp`](WallClearance::clamp)
//! pushes a proposed position back out so the agent's center stays at least
//! `radius` from every wall.
//!
//! This is the free-movement analogue of what the funnel does when it pulls
//! portal endpoints inward by `distance_from_wall`: same boundary invariant,
//! so a hand-moved agent and a planned agent of equal radius agree on how
//! close to a wall they may sit. Because `radius` is a per-call argument, one
//! `WallClearance` serves agents of every size — no per-radius mesh.
//!
//! ```no_run
//! # use rsnav_common::Vertex;
//! # use rsnav_navmesh::NavMesh;
//! # use rsnav_bsp::Bsp;
//! # use rsnav_navigation::WallClearance;
//! # fn demo(nav: &NavMesh, bsp: &Bsp) {
//! let clearance = WallClearance::from_navmesh(nav); // once per mesh
//!
//! // Per frame, after moving the agent yourself:
//! let proposed = Vertex::new(3.0, 4.0);
//! // 1. keep it on the mesh, then 2. push it off the walls.
//! let on_mesh = bsp.nearest(nav, proposed).map(|n| n.point).unwrap_or(proposed);
//! let safe = clearance.clamp(on_mesh, 0.4 /* agent radius */);
//! # let _ = safe;
//! # }
//! ```
//!
//! Rebuild a `WallClearance` whenever the underlying `NavMesh` changes — it is
//! cheap (`O(triangles)`), the same lifecycle as [`Bsp`](rsnav_bsp::Bsp).

use std::collections::HashSet;

use rsnav_common::{geom::nearest_point_on_segment, Vertex};
use rsnav_navmesh::NavMesh;

use crate::wall::is_wall_edge_local;

/// Number of relaxation passes [`WallClearance::clamp`] runs. A single pass
/// resolves a flat wall; the extra passes settle an agent jammed into a
/// concave corner where pushing off one wall violates the next.
const RELAX_ITERS: usize = 4;

/// Below this distance a point is treated as lying *on* a wall, so the
/// push-out direction is taken from the wall normal rather than the
/// (degenerate) point-to-wall vector.
const ON_WALL_EPS: f64 = 1e-9;

/// Precomputed wall segments of a [`NavMesh`], with a [`clamp`](Self::clamp)
/// that holds a position `radius` off the nearest wall.
///
/// A "wall" is any constrained edge (PSLG marker `!= 0`) or any boundary edge
/// (no triangle on the far side) — exactly the edges
/// [`is_wall_edge_local`](crate::wall::is_wall_edge_local) reports, which is
/// the same set A* and the funnel treat as impassable. Interior constrained
/// edges (a wall with walkable area on both sides) are stored once, not twice.
#[derive(Clone, Debug)]
pub struct WallClearance {
    /// Each wall as an (a, b) endpoint pair. Order within a pair is arbitrary;
    /// [`nearest_point_on_segment`] is direction-agnostic.
    segments: Vec<(Vertex, Vertex)>,
}

impl WallClearance {
    /// Collect every wall segment of `nav`. `O(triangles)`.
    pub fn from_navmesh(nav: &NavMesh) -> Self {
        // A constrained edge shared by two triangles is reported by both (with
        // opposite endpoint order); a boundary edge by its single owner. Key on
        // the canonical (min, max) vertex-id pair so each physical wall lands in
        // the set exactly once, however many triangles touch it.
        let mut seen: HashSet<(u32, u32)> = HashSet::new();
        let mut segments = Vec::new();
        for tri in &nav.triangles {
            for i in 0..3 {
                if !is_wall_edge_local(tri, i) {
                    continue;
                }
                let (a, b) = tri.edge_vertices(i);
                let key = if a.index() <= b.index() {
                    (a.index() as u32, b.index() as u32)
                } else {
                    (b.index() as u32, a.index() as u32)
                };
                if seen.insert(key) {
                    segments.push((nav.vertex(a), nav.vertex(b)));
                }
            }
        }
        Self { segments }
    }

    /// Number of stored wall segments. Mostly useful for tests / diagnostics.
    #[inline]
    pub fn segment_count(&self) -> usize {
        self.segments.len()
    }

    /// Push `pos` to the nearest position whose distance to every wall is at
    /// least `radius`, and return it. A point already `radius`-clear of all
    /// walls is returned unchanged.
    ///
    /// The result keeps the agent's *center* off the walls; pair it with a
    /// [`Bsp::nearest`](rsnav_bsp::Bsp::nearest) snap (before this call) to also
    /// guarantee the agent stays on the mesh. In a corridor narrower than
    /// `2 * radius` no clear position exists — the agent is pinned toward the
    /// channel center, consistent with A* refusing such a portal.
    ///
    /// `radius <= 0.0` is a no-op.
    pub fn clamp(&self, pos: Vertex, radius: f64) -> Vertex {
        if radius <= 0.0 || self.segments.is_empty() {
            return pos;
        }
        let r2 = radius * radius;
        let mut p = pos;
        for _ in 0..RELAX_ITERS {
            let mut adjusted = false;
            for &(a, b) in &self.segments {
                let near = nearest_point_on_segment(a, b, p);
                let d2 = p.distance_sq(near);
                if d2 >= r2 {
                    continue;
                }
                let d = d2.sqrt();
                let dir = if d > ON_WALL_EPS {
                    (p - near) * (1.0 / d)
                } else {
                    // p sits on the wall: push along the segment's normal,
                    // biased toward where the agent came from so we move it
                    // back to the walkable side it was on.
                    let seg = (b - a).normalize_or_zero();
                    let normal = Vertex::new(-seg.y, seg.x);
                    let toward = pos - near;
                    if normal.dot(toward) < 0.0 { normal * -1.0 } else { normal }
                };
                p = near + dir * radius;
                adjusted = true;
            }
            if !adjusted {
                break;
            }
        }
        p
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rsnav_bsp::Bsp;
    use rsnav_navmesh::{build_from_cdt, NavMesh};
    use rsnav_triangle::pslg::{Pslg, PslgHole, PslgSegment, PslgVertex};
    use rsnav_triangle::{carve_holes, delaunay, form_skeleton, CdtMesh, DivConqOptions, VertexSlot};

    /// 10x10 square (outer wall) with a 4..6 square hole in the middle.
    fn square_with_hole() -> (NavMesh, Bsp) {
        let pts = [
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0),
            (4.0, 4.0),
            (6.0, 4.0),
            (6.0, 6.0),
            (4.0, 6.0),
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
            holes: vec![PslgHole { point: Vertex::new(5.0, 5.0) }],
        };
        form_skeleton(&mut mesh, &pslg, None).unwrap();
        carve_holes(&mut mesh, &pslg, false);
        let nav = build_from_cdt(&mesh);
        let bsp = Bsp::build(&nav);
        (nav, bsp)
    }

    /// Smallest distance from `p` to any wall segment — the invariant `clamp`
    /// is supposed to lift to `>= radius`.
    fn dist_to_nearest_wall(wc: &WallClearance, p: Vertex) -> f64 {
        wc.segments
            .iter()
            .map(|&(a, b)| p.distance(nearest_point_on_segment(a, b, p)))
            .fold(f64::INFINITY, f64::min)
    }

    #[test]
    fn collects_outer_and_hole_walls_without_duplicates() {
        let (nav, _) = square_with_hole();
        let wc = WallClearance::from_navmesh(&nav);
        // The outer ring and the hole ring are both walls. Every boundary edge
        // the mesh reports must appear exactly once in the segment set.
        let boundary = nav.boundary_edges().count();
        assert!(wc.segment_count() >= boundary);
        // No segment is stored twice (canonical-pair dedup).
        let mut keys: Vec<(u64, u64)> = wc
            .segments
            .iter()
            .map(|&(a, b)| {
                let ka = (a.x.to_bits(), a.y.to_bits());
                let kb = (b.x.to_bits(), b.y.to_bits());
                if ka <= kb {
                    (hash2(ka), hash2(kb))
                } else {
                    (hash2(kb), hash2(ka))
                }
            })
            .collect();
        let before = keys.len();
        keys.sort_unstable();
        keys.dedup();
        assert_eq!(before, keys.len(), "duplicate wall segment stored");
    }

    fn hash2((a, b): (u64, u64)) -> u64 {
        a.wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ b
    }

    #[test]
    fn pushes_off_the_outer_wall() {
        let (nav, _) = square_with_hole();
        let wc = WallClearance::from_navmesh(&nav);
        // Hard against the left wall, deep in the open band.
        let p = Vertex::new(0.05, 2.0);
        let safe = wc.clamp(p, 0.5);
        assert!(
            dist_to_nearest_wall(&wc, safe) >= 0.5 - 1e-6,
            "clamped point still within radius of a wall"
        );
        // It moved inward (to the right), not along/through the wall.
        assert!(safe.x > p.x);
    }

    #[test]
    fn pushes_out_of_a_concave_corner() {
        let (nav, _) = square_with_hole();
        let wc = WallClearance::from_navmesh(&nav);
        // Jammed into the bottom-left corner where two walls meet.
        let p = Vertex::new(0.1, 0.1);
        let safe = wc.clamp(p, 0.5);
        // Must clear BOTH walls of the corner, not just the last one visited.
        assert!(
            dist_to_nearest_wall(&wc, safe) >= 0.5 - 1e-6,
            "corner clamp left the point within radius of a wall"
        );
        assert!(safe.x >= 0.5 - 1e-6 && safe.y >= 0.5 - 1e-6);
    }

    #[test]
    fn pushes_away_from_the_hole() {
        let (nav, _) = square_with_hole();
        let wc = WallClearance::from_navmesh(&nav);
        // Just left of the hole's left edge (x = 4), in walkable space.
        let p = Vertex::new(3.9, 5.0);
        let safe = wc.clamp(p, 0.5);
        assert!(dist_to_nearest_wall(&wc, safe) >= 0.5 - 1e-6);
        // Pushed further from the hole, i.e. to smaller x.
        assert!(safe.x < p.x);
    }

    #[test]
    fn already_clear_point_is_unchanged() {
        let (nav, _) = square_with_hole();
        let wc = WallClearance::from_navmesh(&nav);
        // Dead center of the bottom band, far from every wall.
        let p = Vertex::new(5.0, 1.5);
        assert!(dist_to_nearest_wall(&wc, p) > 0.5);
        let safe = wc.clamp(p, 0.5);
        assert!(safe.approx_eq(p, 1e-12));
    }

    #[test]
    fn non_positive_radius_is_a_noop() {
        let (nav, _) = square_with_hole();
        let wc = WallClearance::from_navmesh(&nav);
        let p = Vertex::new(0.01, 2.0);
        assert!(wc.clamp(p, 0.0).approx_eq(p, 1e-12));
        assert!(wc.clamp(p, -1.0).approx_eq(p, 1e-12));
    }
}
