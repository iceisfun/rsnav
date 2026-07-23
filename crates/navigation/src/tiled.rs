//! Multi-tile worlds: place independent navmeshes in one world space and path
//! across the seams.
//!
//! Each `Tile` is a self-contained [`NavMesh`] + [`Bsp`] placed at a world
//! `offset` (translation only — overworld tiles are axis-aligned). Tiles are
//! never merged or re-triangulated; instead a thin **link** layer connects
//! them. This is the tiled-navmesh model: streaming a tile in or out is an
//! `add_tile` / re-stitch, never the mega-mesh rebuild.
//!
//! ## Links are created by placement, matched by world geometry
//!
//! [`TiledWorld::stitch_all`] finds, for every pair of tiles, boundary edges
//! whose **world-space segments are collinear and overlap** — and links the
//! triangles that own them across the overlapping portion. Vertices need not
//! match: a long border edge in one tile links to the several shorter edges it
//! overlaps in its neighbor, each link carrying the clipped overlap segment as
//! its portal. So you place tiles by offset and stitch; you don't hand-weld
//! edges (though [`Link`] is the primitive a manual weld or a future
//! door-as-link would emit).
//!
//! ## Pathfinding
//!
//! [`TiledWorld::find_path`] runs A* over the global triangle graph —
//! intra-tile adjacency **plus** links — in world space, then string-pulls the
//! result with the same funnel the single-mesh path uses. A boundary edge that
//! is linked is traversable via the link; an unlinked boundary edge stays a
//! wall.
//!
//! v1 scope: translation-only offsets, links always open, agent clearance not
//! yet applied across seams. Per-tile doors and `distance_from_wall` are the
//! marked extension points (a door is just a link you can close).
//!
//! ## Agent radius across seams
//!
//! **Baked contour inset is incompatible with per-tile builds**: eroding a
//! tile's mesh (`rsnav_dynamic::BuildOptions::inset = Some(r)`) recedes its
//! seam edges by `r`, so `stitch_all`'s collinear-overlap matching finds no
//! shared boundary and the tiles silently disconnect. Keep `inset: None`
//! for tiled meshes (or exempt seam edges — a future extension).
//!
//! For grid-sourced worlds, bake the radius into the **bitfield** instead,
//! with `rsnav_polygon_extract::Bitfield::eroded`. The ordering rule is the
//! whole trick — **erode the global grid first, slice it into tiles
//! second**:
//!
//! ```no_run
//! # use rsnav_common::Vertex;
//! # use rsnav_polygon_extract::{Bitfield, ErodeOptions};
//! # use rsnav_navigation::TiledWorld;
//! # fn demo(global: &Bitfield, tiles: &[(u32, u32)]) -> Result<(), Box<dyn std::error::Error>> {
//! # let opts = ();
//! const TS: u32 = 256;
//! let eroded = global.eroded(&ErodeOptions { radius: 2.0, threads: 0 })?;
//! let mut world = TiledWorld::new();
//! for &(tx, ty) in tiles {
//!     let tile_bits = eroded.subgrid(tx * TS, ty * TS, TS, TS); // erode FIRST
//!     // let nav = build_navmesh_from_bitfield(&tile_bits, &opts /* inset: None */)?;
//!     // world.add_tile(nav.navmesh, Vertex::new((tx * TS) as f64, (ty * TS) as f64));
//! }
//! world.stitch_all(1e-9);
//! # Ok(())
//! # }
//! ```
//!
//! Slicing an already-eroded grid puts each tile's boundary exactly on the
//! tile border line, at identical integer coordinates in both neighbours,
//! so `stitch_all` links them normally. **Eroding a tile** would treat the
//! tile border as wall and eat `radius` cells at every seam — reproducing
//! precisely the contour-inset failure above.
//!
//! Two pre-existing hazards get slightly likelier once erosion roughens
//! seam-adjacent geometry, because both can act asymmetrically between
//! neighbours: `ExtractOptions::min_area` can drop a seam-adjacent fragment
//! in one tile only, and `BuildOptions::clip_ears_max_area` can shave a
//! small sliver whose wall edge lies *on* a seam. For tiled builds prefer
//! `min_area = 0.0`, and drop `clip_ears_max_area` to 0.0 if a seam ever
//! fails to link.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use rsnav_bsp::Bsp;
use rsnav_common::geom::{nearest_point_on_segment, orient2d, point_in_triangle, segment_intersection};
use rsnav_common::{Aabb, TriangleId, Vertex, VertexId};
use rsnav_navmesh::NavMesh;

use crate::funnel::string_pull;
use crate::los::LineOfSightResult;
use crate::wall::is_wall_edge_local;

/// Handle to a tile within a [`TiledWorld`].
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct TileId(pub u32);

/// A triangle named globally: which tile, and which triangle within it.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct GlobalTri {
    pub tile: TileId,
    pub tri: TriangleId,
}

/// One emplaced navmesh.
struct Tile {
    nav: NavMesh,
    bsp: Bsp,
    /// world = local + offset.
    offset: Vertex,
    world_aabb: Aabb,
}

/// A cross-tile connection: two triangles joined across the world-space portal
/// segment where their borders overlap. Undirected — stored once, traversable
/// either way.
#[derive(Copy, Clone, Debug)]
pub struct Link {
    pub a: GlobalTri,
    pub b: GlobalTri,
    /// The shared crossing, in world space — the funnel portal for this hop.
    pub portal: (Vertex, Vertex),
}

/// A set of tiles placed in a shared world space, plus the links stitched
/// between them.
#[derive(Default)]
pub struct TiledWorld {
    tiles: Vec<Tile>,
    links: Vec<Link>,
    /// `GlobalTri` → indices into `links`, both endpoints registered.
    link_index: HashMap<GlobalTri, Vec<usize>>,
    /// Global triangle index base per tile, and the running total — so A*
    /// scratch arrays can be flat.
    tile_base: Vec<usize>,
    total_tris: usize,
}

impl TiledWorld {
    pub fn new() -> Self {
        Self::default()
    }

    /// Emplace `nav` at world `offset` (translation). Returns its [`TileId`].
    /// Call [`stitch_all`](Self::stitch_all) afterward to (re)connect seams.
    pub fn add_tile(&mut self, nav: NavMesh, offset: Vertex) -> TileId {
        let bsp = Bsp::build(&nav);
        let world_aabb = Aabb {
            min: Vertex::new(nav.aabb.min.x + offset.x, nav.aabb.min.y + offset.y),
            max: Vertex::new(nav.aabb.max.x + offset.x, nav.aabb.max.y + offset.y),
        };
        self.tile_base.push(self.total_tris);
        self.total_tris += nav.triangle_count();
        self.tiles.push(Tile {
            nav,
            bsp,
            offset,
            world_aabb,
        });
        TileId((self.tiles.len() - 1) as u32)
    }

    #[inline]
    pub fn tile_count(&self) -> usize {
        self.tiles.len()
    }

    #[inline]
    pub fn links(&self) -> &[Link] {
        &self.links
    }

    /// World-space bounds of a tile, for culling / rendering. `None` if
    /// `tile` is not in this world.
    pub fn tile_world_aabb(&self, tile: TileId) -> Option<Aabb> {
        self.tiles.get(tile.0 as usize).map(|t| t.world_aabb)
    }

    /// A tile's navmesh (local coordinates), for rendering its triangles.
    /// `None` if `tile` is not in this world.
    pub fn tile_nav(&self, tile: TileId) -> Option<&NavMesh> {
        self.tiles.get(tile.0 as usize).map(|t| &t.nav)
    }

    /// A tile's current world offset. `None` if `tile` is not in this world.
    pub fn tile_offset(&self, tile: TileId) -> Option<Vertex> {
        self.tiles.get(tile.0 as usize).map(|t| t.offset)
    }

    /// Move a tile to a new world `offset`. Invalidates the links — call
    /// [`stitch_all`](Self::stitch_all) afterward to reconnect seams at the
    /// new position. Returns `false` (and does nothing) if `tile` is not in
    /// this world.
    pub fn set_tile_offset(&mut self, tile: TileId, offset: Vertex) -> bool {
        let Some(t) = self.tiles.get_mut(tile.0 as usize) else {
            return false;
        };
        let local = t.nav.aabb;
        t.offset = offset;
        t.world_aabb = Aabb {
            min: Vertex::new(local.min.x + offset.x, local.min.y + offset.y),
            max: Vertex::new(local.max.x + offset.x, local.max.y + offset.y),
        };
        true
    }

    /// Rebuild every cross-tile link from scratch. `tol` is the world-space
    /// slack for "collinear and touching" — set it to a small fraction of your
    /// tile size (e.g. `1e-6` for exact grids, larger if borders are noisy).
    pub fn stitch_all(&mut self, tol: f64) {
        let mut links = Vec::new();
        let n = self.tiles.len();
        for i in 0..n {
            for j in (i + 1)..n {
                self.discover_links(TileId(i as u32), TileId(j as u32), tol, &mut links);
            }
        }
        self.links = links;
        self.link_index.clear();
        for (idx, l) in self.links.iter().enumerate() {
            self.link_index.entry(l.a).or_default().push(idx);
            self.link_index.entry(l.b).or_default().push(idx);
        }
    }

    /// Append the links between tiles `a` and `b` to `out`.
    fn discover_links(&self, a: TileId, b: TileId, tol: f64, out: &mut Vec<Link>) {
        let ta = &self.tiles[a.0 as usize];
        let tb = &self.tiles[b.0 as usize];
        // Quick reject: tiles whose (slightly grown) bounds don't touch can't
        // share a border.
        if !expand(ta.world_aabb, tol).intersects(&tb.world_aabb) {
            return;
        }
        let edges_a = self.boundary_world_edges(a);
        let edges_b = self.boundary_world_edges(b);
        for (ga, a1, a2) in &edges_a {
            for (gb, b1, b2) in &edges_b {
                if let Some(portal) = collinear_overlap(*a1, *a2, *b1, *b2, tol) {
                    out.push(Link {
                        a: *ga,
                        b: *gb,
                        portal,
                    });
                }
            }
        }
    }

    /// Every boundary edge of a tile as `(owner triangle, world p0, world p1)`.
    fn boundary_world_edges(&self, tile: TileId) -> Vec<(GlobalTri, Vertex, Vertex)> {
        let t = &self.tiles[tile.0 as usize];
        t.nav
            .boundary_edges()
            .map(|e| {
                (
                    GlobalTri {
                        tile,
                        tri: e.triangle,
                    },
                    self.world_vertex(tile, e.from),
                    self.world_vertex(tile, e.to),
                )
            })
            .collect()
    }

    #[inline]
    fn world_vertex(&self, tile: TileId, v: VertexId) -> Vertex {
        let t = &self.tiles[tile.0 as usize];
        let p = t.nav.vertex(v);
        Vertex::new(p.x + t.offset.x, p.y + t.offset.y)
    }

    #[inline]
    fn world_centroid(&self, g: GlobalTri) -> Vertex {
        let t = &self.tiles[g.tile.0 as usize];
        let c = t.nav.triangle(g.tri).centroid;
        Vertex::new(c.x + t.offset.x, c.y + t.offset.y)
    }

    #[inline]
    fn gindex(&self, g: GlobalTri) -> usize {
        self.tile_base[g.tile.0 as usize] + g.tri.index()
    }

    /// Locate the global triangle containing world point `p`.
    pub fn locate(&self, p: Vertex) -> Option<GlobalTri> {
        for (ti, t) in self.tiles.iter().enumerate() {
            if !t.world_aabb.contains(p) {
                continue;
            }
            let local = Vertex::new(p.x - t.offset.x, p.y - t.offset.y);
            if let Some(tri) = t.bsp.locate(&t.nav, local) {
                return Some(GlobalTri {
                    tile: TileId(ti as u32),
                    tri,
                });
            }
        }
        None
    }

    /// Plan a path from world `start` to world `goal` across tiles. Returns the
    /// string-pulled world-space polyline, or `None` if either endpoint is off
    /// every tile or no linked route connects them.
    pub fn find_path(&self, start: Vertex, goal: Vertex) -> Option<Vec<Vertex>> {
        let s = self.locate(start)?;
        let g = self.locate(goal)?;
        let seq = self.astar(s, g, start, goal)?;
        Some(self.funnel(&seq, start, goal))
    }

    /// Line of sight from world `from` to world `to`, walking triangle-by-
    /// triangle across tiles. Crosses internal portals and open seam links;
    /// stops at the first wall — a constrained interior edge or an *unlinked*
    /// boundary edge. Mirrors the single-mesh [`crate::line_of_sight`].
    pub fn line_of_sight(&self, from: Vertex, to: Vertex) -> LineOfSightResult {
        let Some(mut cur) = self.locate(from) else {
            return LineOfSightResult::SourceOutsideMesh;
        };
        let max_steps = self.total_tris * 2 + 8;
        for _ in 0..max_steps {
            if self.tri_world_contains(cur, to) {
                return LineOfSightResult::Clear;
            }
            let verts = self.tri_world_vertices(cur);
            // Exit edge = the one the segment crosses furthest along (max t).
            let mut best: Option<(usize, Vertex, f64)> = None;
            for i in 0..3 {
                let pa = verts[(i + 1) % 3];
                let pb = verts[(i + 2) % 3];
                if let Some(hit) = segment_intersection(from, to, pa, pb) {
                    if hit.t < -1e-9 {
                        continue;
                    }
                    if best.map_or(true, |(_, _, t)| hit.t > t) {
                        best = Some((i, hit.point, hit.t));
                    }
                }
            }
            let Some((edge, hit, _)) = best else {
                return LineOfSightResult::Indeterminate;
            };

            let tri = self.tiles[cur.tile.0 as usize].nav.triangle(cur.tri);
            if tri.neighbors[edge].is_valid() {
                if tri.edge_markers[edge] != 0 {
                    return LineOfSightResult::Blocked { point: hit }; // interior wall
                }
                cur = GlobalTri {
                    tile: cur.tile,
                    tri: tri.neighbors[edge],
                };
                continue;
            }
            // Boundary edge: cross it only if a link covers the hit point.
            match self.link_across(cur, hit) {
                Some(next) => cur = next,
                None => return LineOfSightResult::Blocked { point: hit },
            }
        }
        LineOfSightResult::Indeterminate
    }

    /// If a link out of `g` has the world point `p` on its portal, return the
    /// triangle on the far side.
    fn link_across(&self, g: GlobalTri, p: Vertex) -> Option<GlobalTri> {
        for &li in self.link_index.get(&g)? {
            let l = &self.links[li];
            let (a, b) = l.portal;
            // p lies on the portal if it's on the segment within a small slack.
            let near = nearest_point_on_segment(a, b, p);
            if p.distance(near) < 1e-6 {
                return Some(if l.a == g { l.b } else { l.a });
            }
        }
        None
    }

    fn tri_world_vertices(&self, g: GlobalTri) -> [Vertex; 3] {
        let t = &self.tiles[g.tile.0 as usize];
        let tri = t.nav.triangle(g.tri);
        [0, 1, 2].map(|i| {
            let v = t.nav.vertex(tri.vertices[i]);
            Vertex::new(v.x + t.offset.x, v.y + t.offset.y)
        })
    }

    fn tri_world_contains(&self, g: GlobalTri, p: Vertex) -> bool {
        let [a, b, c] = self.tri_world_vertices(g);
        point_in_triangle(a, b, c, p)
    }

    // -- A* over the global triangle graph -----------------------------

    fn astar(
        &self,
        start: GlobalTri,
        goal: GlobalTri,
        start_pt: Vertex,
        goal_pt: Vertex,
    ) -> Option<Vec<GlobalTri>> {
        if start == goal {
            return Some(vec![start]);
        }
        let n = self.total_tris;
        let mut g_score = vec![f64::INFINITY; n];
        let mut came: Vec<Option<GlobalTri>> = vec![None; n];
        let mut entry = vec![Vertex::ZERO; n];
        let mut closed = vec![false; n];
        let mut heap: BinaryHeap<Node> = BinaryHeap::new();

        g_score[self.gindex(start)] = 0.0;
        entry[self.gindex(start)] = start_pt;
        heap.push(Node {
            tri: start,
            f: start_pt.distance(goal_pt),
        });

        while let Some(Node { tri: cur, .. }) = heap.pop() {
            if cur == goal {
                return Some(self.reconstruct(&came, start, goal));
            }
            let ci = self.gindex(cur);
            if closed[ci] {
                continue;
            }
            closed[ci] = true;
            let cur_entry = entry[ci];

            for (nb, portal) in self.successors(cur) {
                let nbi = self.gindex(nb);
                if closed[nbi] {
                    continue;
                }
                // Portal-crossing cost in world space — identical metric to the
                // single-mesh A*, so the world funnel renders the right channel.
                let crossing = nearest_point_on_segment(portal.0, portal.1, cur_entry);
                let mut step = cur_entry.distance(crossing);
                let h = if nb == goal {
                    step += crossing.distance(goal_pt);
                    0.0
                } else {
                    crossing.distance(goal_pt)
                };
                let tentative = g_score[ci] + step;
                if tentative < g_score[nbi] {
                    g_score[nbi] = tentative;
                    came[nbi] = Some(cur);
                    entry[nbi] = crossing;
                    heap.push(Node {
                        tri: nb,
                        f: tentative + h,
                    });
                }
            }
        }
        None
    }

    /// Successors of `g`: intra-tile non-wall neighbors (portal = shared edge)
    /// plus links (portal = the link's world segment).
    fn successors(&self, g: GlobalTri) -> Vec<(GlobalTri, (Vertex, Vertex))> {
        let mut out = Vec::new();
        let t = &self.tiles[g.tile.0 as usize];
        let tri = t.nav.triangle(g.tri);
        for i in 0..3 {
            if is_wall_edge_local(tri, i) {
                continue; // wall or boundary — a linked boundary is handled below
            }
            let nb = tri.neighbors[i];
            let (va, vb) = tri.edge_vertices(i);
            out.push((
                GlobalTri {
                    tile: g.tile,
                    tri: nb,
                },
                (self.world_vertex(g.tile, va), self.world_vertex(g.tile, vb)),
            ));
        }
        if let Some(idxs) = self.link_index.get(&g) {
            for &li in idxs {
                let l = &self.links[li];
                let other = if l.a == g { l.b } else { l.a };
                out.push((other, l.portal));
            }
        }
        out
    }

    fn reconstruct(
        &self,
        came: &[Option<GlobalTri>],
        start: GlobalTri,
        goal: GlobalTri,
    ) -> Vec<GlobalTri> {
        let mut path = vec![goal];
        let mut cur = goal;
        while cur != start {
            cur = came[self.gindex(cur)].expect("A* predecessor chain is contiguous");
            path.push(cur);
        }
        path.reverse();
        path
    }

    // -- world-space funnel --------------------------------------------

    fn funnel(&self, seq: &[GlobalTri], start: Vertex, goal: Vertex) -> Vec<Vertex> {
        if seq.len() < 2 {
            return vec![start, goal];
        }
        let mut portals: Vec<(Vertex, Vertex)> = Vec::with_capacity(seq.len() + 1);
        portals.push((start, start));
        for w in seq.windows(2) {
            if let Some((p0, p1)) = self.portal_between(w[0], w[1]) {
                // Orient (left, right) relative to travel direction, matching
                // the single-mesh funnel's convention.
                let from_c = self.world_centroid(w[0]);
                let to_c = self.world_centroid(w[1]);
                if orient2d(from_c, to_c, p0) > 0.0 {
                    portals.push((p0, p1));
                } else {
                    portals.push((p1, p0));
                }
            }
        }
        portals.push((goal, goal));
        string_pull(&portals)
    }

    /// The world-space portal segment between adjacent global triangles —
    /// their shared intra-tile edge, or the link joining them.
    fn portal_between(&self, prev: GlobalTri, cur: GlobalTri) -> Option<(Vertex, Vertex)> {
        if prev.tile == cur.tile {
            let t = &self.tiles[prev.tile.0 as usize];
            let tri = t.nav.triangle(prev.tri);
            for i in 0..3 {
                if tri.neighbors[i] == cur.tri {
                    let (va, vb) = tri.edge_vertices(i);
                    return Some((
                        self.world_vertex(prev.tile, va),
                        self.world_vertex(prev.tile, vb),
                    ));
                }
            }
        }
        let idxs = self.link_index.get(&prev)?;
        for &li in idxs {
            let l = &self.links[li];
            let other = if l.a == prev { l.b } else { l.a };
            if other == cur {
                return Some(l.portal);
            }
        }
        None
    }
}

// =========================================================================
// Heap node + geometry helpers
// =========================================================================

#[derive(Copy, Clone)]
struct Node {
    tri: GlobalTri,
    f: f64,
}
impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.f == other.f
    }
}
impl Eq for Node {}
impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse → min-heap on f.
        other.f.partial_cmp(&self.f).unwrap_or(Ordering::Equal)
    }
}
impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[inline]
fn expand(a: Aabb, t: f64) -> Aabb {
    Aabb {
        min: Vertex::new(a.min.x - t, a.min.y - t),
        max: Vertex::new(a.max.x + t, a.max.y + t),
    }
}

/// If segments `(a1,a2)` and `(b1,b2)` are collinear (within `tol`) and their
/// overlap is longer than `tol`, return the overlap as a segment along
/// `(a1,a2)`'s direction. Otherwise `None`.
fn collinear_overlap(
    a1: Vertex,
    a2: Vertex,
    b1: Vertex,
    b2: Vertex,
    tol: f64,
) -> Option<(Vertex, Vertex)> {
    let d = a2 - a1;
    let len = d.length();
    if len <= tol {
        return None;
    }
    // Perpendicular distance of a point from the infinite line a1→a2 is
    // |cross| / len; both of b's endpoints must lie on the line.
    let perp = |p: Vertex| orient2d(a1, a2, p).abs() / len;
    if perp(b1) > tol || perp(b2) > tol {
        return None;
    }
    // Project everything onto the unit direction and intersect the intervals.
    let inv = 1.0 / len;
    let proj = |p: Vertex| ((p - a1).dot(d)) * inv;
    let (tb_lo, tb_hi) = {
        let (x, y) = (proj(b1), proj(b2));
        if x <= y {
            (x, y)
        } else {
            (y, x)
        }
    };
    let lo = 0.0_f64.max(tb_lo);
    let hi = len.min(tb_hi);
    if hi - lo <= tol {
        return None;
    }
    let u = d * inv;
    Some((a1 + u * lo, a1 + u * hi))
}

#[cfg(test)]
mod tests {
    use super::*;
    use rsnav_navmesh::build_from_cdt;
    use rsnav_triangle::pslg::{Pslg, PslgSegment, PslgVertex};
    use rsnav_triangle::{
        carve_holes, delaunay, form_skeleton, CdtMesh, DivConqOptions, VertexSlot,
    };

    /// An open `w × h` rectangle navmesh (local origin at 0,0).
    fn rect(w: f64, h: f64) -> NavMesh {
        let pts = [(0.0, 0.0), (w, 0.0), (w, h), (0.0, h)];
        let mut cdt = CdtMesh::new();
        let mut pslg = Pslg::new();
        for (x, y) in pts {
            cdt.push_vertex(VertexSlot::new(Vertex::new(x, y), 0));
            pslg.vertices.push(PslgVertex::new(Vertex::new(x, y)));
        }
        for &(a, b) in &[(0, 1), (1, 2), (2, 3), (3, 0)] {
            pslg.segments.push(PslgSegment { a, b, marker: 1 });
        }
        delaunay(&mut cdt, DivConqOptions::default());
        form_skeleton(&mut cdt, &pslg, None).unwrap();
        build_from_cdt(&cdt)
    }

    fn len(path: &[Vertex]) -> f64 {
        path.windows(2).map(|w| w[0].distance(w[1])).sum()
    }

    #[test]
    fn two_tiles_stitch_and_path_straight_across_seam() {
        let mut world = TiledWorld::new();
        world.add_tile(rect(10.0, 10.0), Vertex::new(0.0, 0.0));
        world.add_tile(rect(10.0, 10.0), Vertex::new(10.0, 0.0)); // east neighbor
        world.stitch_all(1e-6);

        assert!(!world.links().is_empty(), "seam at x=10 should produce links");

        // Straight shot from tile A into tile B at y = 5.
        let start = Vertex::new(3.0, 5.0);
        let goal = Vertex::new(17.0, 5.0);
        let path = world.find_path(start, goal).expect("a linked route exists");
        assert_eq!(path.first(), Some(&start));
        assert_eq!(path.last(), Some(&goal));
        // Open corridor → the funnel pulls it perfectly straight (length 14).
        assert!(
            (len(&path) - 14.0).abs() < 1e-6,
            "expected straight 14.0, got {} via {:?}",
            len(&path),
            path
        );
    }

    #[test]
    fn line_of_sight_crosses_open_seam_and_stops_at_unlinked_boundary() {
        let mut world = TiledWorld::new();
        world.add_tile(rect(10.0, 10.0), Vertex::new(0.0, 0.0));
        world.add_tile(rect(10.0, 10.0), Vertex::new(10.0, 0.0));
        world.stitch_all(1e-6);

        // Clear across the stitched seam.
        assert_eq!(
            world.line_of_sight(Vertex::new(3.0, 5.0), Vertex::new(17.0, 5.0)),
            LineOfSightResult::Clear
        );
        // Looking off the east edge of the world (no tile there) → blocked at
        // the unlinked outer wall (x = 20).
        match world.line_of_sight(Vertex::new(3.0, 5.0), Vertex::new(25.0, 5.0)) {
            LineOfSightResult::Blocked { point } => assert!((point.x - 20.0).abs() < 1e-9),
            other => panic!("expected Blocked at x=20, got {other:?}"),
        }
    }

    #[test]
    fn no_stitch_means_no_cross_tile_route() {
        let mut world = TiledWorld::new();
        world.add_tile(rect(10.0, 10.0), Vertex::new(0.0, 0.0));
        world.add_tile(rect(10.0, 10.0), Vertex::new(10.0, 0.0));
        // Deliberately do NOT stitch.
        assert!(world.find_path(Vertex::new(3.0, 5.0), Vertex::new(17.0, 5.0)).is_none());
    }

    #[test]
    fn mismatched_seam_triangulations_still_link() {
        // Tile A's right border is one edge [0..10]; tile B's left border is
        // split by an extra vertex at y=4, so it's two edges [0..4],[4..10].
        // The overlap matcher must link A's edge to BOTH of B's.
        let a = rect(10.0, 10.0);

        let pts = [
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0),
            (0.0, 4.0), // extra vertex on the left (seam) border
        ];
        let mut cdt = CdtMesh::new();
        let mut pslg = Pslg::new();
        for (x, y) in pts {
            cdt.push_vertex(VertexSlot::new(Vertex::new(x, y), 0));
            pslg.vertices.push(PslgVertex::new(Vertex::new(x, y)));
        }
        // Left border now two segments 3→4 and 4→0.
        for &(s, e) in &[(0, 1), (1, 2), (2, 3), (3, 4), (4, 0)] {
            pslg.segments.push(PslgSegment { a: s, b: e, marker: 1 });
        }
        delaunay(&mut cdt, DivConqOptions::default());
        form_skeleton(&mut cdt, &pslg, None).unwrap();
        carve_holes(&mut cdt, &pslg, false);
        let b = build_from_cdt(&cdt);

        let mut world = TiledWorld::new();
        world.add_tile(a, Vertex::new(0.0, 0.0));
        world.add_tile(b, Vertex::new(10.0, 0.0));
        world.stitch_all(1e-6);

        // A long edge in A must link to BOTH of B's short seam edges.
        let seam_links = world
            .links()
            .iter()
            .filter(|l| (l.portal.0.x - 10.0).abs() < 1e-9)
            .count();
        assert!(seam_links >= 2, "one A edge should link to both B edges, got {seam_links}");

        // A route exists and is near-straight. It can bend slightly at the
        // seam vertex (10,4): where one edge links to two, that shared vertex
        // is a T-junction the v1 funnel treats as a soft corner (exact paths
        // would need collinear link portals merged — deferred). Across an
        // aligned grid (matching seam vertices) the path is exact; see
        // `two_tiles_stitch_and_path_straight_across_seam`.
        let path = world
            .find_path(Vertex::new(3.0, 5.0), Vertex::new(17.0, 5.0))
            .expect("mismatched seam still links");
        assert!(
            len(&path) < 14.2,
            "expected a near-straight ~14 path, got {} via {:?}",
            len(&path),
            path
        );
    }

    #[test]
    fn path_bends_around_obstacle_in_far_tile() {
        // Tile A open; tile B has a hole straddling y=5, forcing a detour
        // after crossing the seam.
        let a = rect(10.0, 10.0);

        let pts = [
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0),
            (3.0, 4.0), // hole
            (7.0, 4.0),
            (7.0, 6.0),
            (3.0, 6.0),
        ];
        let mut cdt = CdtMesh::new();
        let mut pslg = Pslg::new();
        for (x, y) in pts {
            cdt.push_vertex(VertexSlot::new(Vertex::new(x, y), 0));
            pslg.vertices.push(PslgVertex::new(Vertex::new(x, y)));
        }
        for &(s, e) in &[(0, 1), (1, 2), (2, 3), (3, 0)] {
            pslg.segments.push(PslgSegment { a: s, b: e, marker: 1 });
        }
        for &(s, e) in &[(4, 5), (5, 6), (6, 7), (7, 4)] {
            pslg.segments.push(PslgSegment { a: s, b: e, marker: 2 });
        }
        pslg.holes.push(rsnav_triangle::pslg::PslgHole {
            point: Vertex::new(5.0, 5.0),
        });
        delaunay(&mut cdt, DivConqOptions::default());
        form_skeleton(&mut cdt, &pslg, None).unwrap();
        carve_holes(&mut cdt, &pslg, false);
        let b = build_from_cdt(&cdt);

        let mut world = TiledWorld::new();
        world.add_tile(a, Vertex::new(0.0, 0.0));
        world.add_tile(b, Vertex::new(10.0, 0.0));
        world.stitch_all(1e-6);

        let start = Vertex::new(5.0, 5.0); // tile A
        let goal = Vertex::new(19.0, 5.0); // tile B, beyond the hole
        let path = world.find_path(start, goal).expect("route around the hole");
        // Must detour, so longer than the 14.0 straight line, and no point
        // inside the world-space hole [13,17]×[4,6].
        assert!(len(&path) > 14.0, "expected a detour, got {:?}", path);
        for p in &path {
            let in_hole = p.x > 13.0 && p.x < 17.0 && p.y > 4.0 && p.y < 6.0;
            assert!(!in_hole, "path crosses the hole: {:?}", path);
        }
    }
}
