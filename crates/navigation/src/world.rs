//! [`NavWorld`]: the owning container that ties a [`NavMesh`], its [`Bsp`],
//! the [`DoorSet`], and the derived [`WallInfo`] into one value, and layers
//! user **metadata** on top.
//!
//! Loose `(nav, bsp, doors, walls)` tuples force the caller to remember the
//! invariant "rebuild `walls` whenever a door changes". `NavWorld` owns that:
//! every door mutator rebuilds the wall oracle, and every query method routes
//! through it, so A*, line-of-sight, and visibility all stay door-consistent
//! by construction. The mesh and BSP are never rebuilt when a door toggles.
//!
//! ## Metadata
//!
//! `NavWorld<M>` is generic over a [`NavMetadata`] implementation you supply.
//! It answers two things, both keyed by the triangle a world point lands in:
//! a cheap **zone** identity (used to detect "entered / left" transitions
//! along a path) and an arbitrary **value** lookup. The flow you author is:
//!
//! ```text
//! NavMesh ── NavWorld::new(nav, meta) ──▶ world
//! world.find_path(a, b) ──▶ PathResult
//! world.zone_crossings(&path) ──▶ [ZoneCrossing { point, from, into }, …]
//! world.metadata_at(p) ──▶ Option<&Value>
//! ```
//!
//! so you can render "in town … now leaving town" from the crossings, and
//! query "what is at this position" directly. rsnav never sees your metadata
//! type — it only ever hands the trait a [`TriangleId`].

use rsnav_bsp::Bsp;
use rsnav_common::{TriangleId, Vertex};
use rsnav_navmesh::NavMesh;

use crate::doors::{DoorId, DoorSet, DoorState};
use crate::los::{line_of_sight as los_walk, LineOfSightResult};
use crate::path::{
    find_path_with_walls, path_clear as path_clear_walls, NearestPoint, PathError, PathOptions,
    PathResult,
};
use crate::visibility::{visibility_region as vis_region, VisibilityRegion};
use crate::wall::WallInfo;

// =========================================================================
// Metadata
// =========================================================================

/// User metadata attached to a [`NavWorld`], resolved per triangle.
///
/// Implement this on your own store (a `Vec` indexed by triangle, a
/// `HashMap`, a spatial DB — whatever). rsnav locates the triangle a world
/// point falls in and hands you its [`TriangleId`]; you map that to your
/// game-domain data. Zones drive path annotation; values are arbitrary.
pub trait NavMetadata {
    /// A cheap zone identity. Equal values mean "same zone", which is how
    /// [`zone_crossings`] detects a boundary along a path (e.g. `town` vs
    /// `wilderness`). `None` = the triangle belongs to no zone.
    type Zone: Clone + PartialEq;

    /// An arbitrary payload returned by point queries.
    type Value;

    /// The zone of triangle `tri`, if any.
    fn zone(&self, tri: TriangleId) -> Option<Self::Zone>;

    /// The value at triangle `tri` (the exact world point `p` is provided
    /// for stores that vary within a triangle). `None` = nothing here.
    fn value_at(&self, tri: TriangleId, p: Vertex) -> Option<&Self::Value>;
}

/// The no-op metadata used by a [`NavWorld`] that carries none. Every query
/// returns `None`; zone crossings are always empty.
#[derive(Clone, Copy, Debug, Default)]
pub struct NoMetadata;

impl NavMetadata for NoMetadata {
    type Zone = ();
    type Value = ();

    #[inline]
    fn zone(&self, _tri: TriangleId) -> Option<()> {
        None
    }

    #[inline]
    fn value_at(&self, _tri: TriangleId, _p: Vertex) -> Option<&()> {
        None
    }
}

/// A point along a path where the [`NavMetadata::Zone`] changes — emitted by
/// [`zone_crossings`]. `from`/`into` are the zones on either side (`None` for
/// a zoneless triangle), and `point` is the boundary location (the midpoint
/// of the portal between the two triangles).
#[derive(Clone, Debug, PartialEq)]
pub struct ZoneCrossing<Z> {
    pub point: Vertex,
    pub from: Option<Z>,
    pub into: Option<Z>,
}

/// Walk a path's triangle sequence and report every zone boundary it crosses.
///
/// `zone_of` maps a triangle to its zone (typically `|t| meta.zone(t)`). A
/// crossing is emitted whenever consecutive triangles disagree, located at the
/// midpoint of the portal between them. The path's *starting* zone is implicit
/// (query it with `zone_of(path.triangles[0])`); this reports the transitions.
///
/// Operates on [`PathResult::triangles`] (the A* sequence), so it costs
/// nothing extra in the path output.
pub fn zone_crossings<Z: Clone + PartialEq>(
    nav: &NavMesh,
    path: &PathResult,
    zone_of: impl Fn(TriangleId) -> Option<Z>,
) -> Vec<ZoneCrossing<Z>> {
    let tris = &path.triangles;
    let mut out = Vec::new();
    if tris.len() < 2 {
        return out;
    }
    let mut prev = zone_of(tris[0]);
    for w in tris.windows(2) {
        let (a, b) = (w[0], w[1]);
        let next = zone_of(b);
        if next != prev {
            let point = shared_edge_midpoint(nav, a, b).unwrap_or_else(|| nav.triangle(b).centroid);
            out.push(ZoneCrossing {
                point,
                from: prev.clone(),
                into: next.clone(),
            });
            prev = next;
        }
    }
    out
}

/// Midpoint of the edge shared by adjacent triangles `a` and `b`. `None` if
/// they aren't actually neighbors (shouldn't happen for an A* sequence).
fn shared_edge_midpoint(nav: &NavMesh, a: TriangleId, b: TriangleId) -> Option<Vertex> {
    let ta = nav.triangle(a);
    for i in 0..3 {
        if ta.neighbors[i] == b {
            let (va, vb) = ta.edge_vertices(i);
            let pa = nav.vertex(va);
            let pb = nav.vertex(vb);
            return Some(Vertex::new((pa.x + pb.x) * 0.5, (pa.y + pb.y) * 0.5));
        }
    }
    None
}

// =========================================================================
// NavWorld
// =========================================================================

/// A navmesh with its spatial index, doors, derived wall oracle, and user
/// metadata — the one value a client holds and queries.
pub struct NavWorld<M = NoMetadata> {
    nav: NavMesh,
    bsp: Bsp,
    doors: DoorSet,
    walls: WallInfo,
    meta: M,
}

impl NavWorld<NoMetadata> {
    /// Build a world with no metadata. Equivalent to `new(nav, NoMetadata)`.
    pub fn without_metadata(nav: NavMesh) -> Self {
        Self::new(nav, NoMetadata)
    }
}

impl<M: NavMetadata> NavWorld<M> {
    /// Emplace a built `nav` into a world with `meta`. Builds the BSP and the
    /// (door-free) wall oracle.
    pub fn new(nav: NavMesh, meta: M) -> Self {
        let bsp = Bsp::build(&nav);
        let walls = WallInfo::from_navmesh(&nav);
        Self {
            nav,
            bsp,
            doors: DoorSet::new(),
            walls,
            meta,
        }
    }

    // -- accessors -----------------------------------------------------

    #[inline]
    pub fn nav(&self) -> &NavMesh {
        &self.nav
    }

    #[inline]
    pub fn bsp(&self) -> &Bsp {
        &self.bsp
    }

    #[inline]
    pub fn doors(&self) -> &DoorSet {
        &self.doors
    }

    /// The current wall oracle (static walls ∪ closed doors). Handy if you
    /// call the free `find_path_with_walls` / `line_of_sight` directly.
    #[inline]
    pub fn walls(&self) -> &WallInfo {
        &self.walls
    }

    #[inline]
    pub fn meta(&self) -> &M {
        &self.meta
    }

    #[inline]
    pub fn meta_mut(&mut self) -> &mut M {
        &mut self.meta
    }

    /// Change counter for repathing: bumps whenever a door state changes.
    /// A path planned at generation `g` is stale once this differs from `g`,
    /// so clients holding a `PathResult` repath instead of mutating it.
    #[inline]
    pub fn generation(&self) -> u64 {
        self.doors.generation()
    }

    // -- door authoring (each rebuilds the wall oracle) ----------------

    /// Add a door from an authoring segment `a → b`; see [`DoorSet::add`].
    pub fn add_door(&mut self, a: Vertex, b: Vertex, state: DoorState) -> DoorId {
        let id = self.doors.add(&self.nav, &self.bsp, a, b, state);
        self.rebuild_walls();
        id
    }

    /// Add a door on one named portal edge; see [`DoorSet::add_edge`].
    pub fn add_door_edge(
        &mut self,
        va: rsnav_common::VertexId,
        vb: rsnav_common::VertexId,
        state: DoorState,
    ) -> DoorId {
        let id = self.doors.add_edge(&self.nav, va, vb, state);
        self.rebuild_walls();
        id
    }

    pub fn set_door(&mut self, id: DoorId, state: DoorState) {
        self.doors.set_state(id, state);
        self.rebuild_walls();
    }

    pub fn open_door(&mut self, id: DoorId) {
        self.set_door(id, DoorState::Open);
    }

    pub fn close_door(&mut self, id: DoorId) {
        self.set_door(id, DoorState::Closed);
    }

    pub fn toggle_door(&mut self, id: DoorId) {
        self.doors.toggle(id);
        self.rebuild_walls();
    }

    pub fn remove_door(&mut self, id: DoorId) -> bool {
        let removed = self.doors.remove(id);
        if removed {
            self.rebuild_walls();
        }
        removed
    }

    pub fn clear_doors(&mut self) {
        self.doors.clear();
        self.rebuild_walls();
    }

    fn rebuild_walls(&mut self) {
        self.walls = WallInfo::from_navmesh_with_doors(&self.nav, &self.doors);
    }

    // -- queries (door-aware via the owned wall oracle) ----------------

    /// Locate the triangle containing `p`, or `None` if `p` is off the mesh.
    #[inline]
    pub fn locate(&self, p: Vertex) -> Option<TriangleId> {
        self.bsp.locate(&self.nav, p)
    }

    /// Snap `p` to the nearest point on the mesh.
    #[inline]
    pub fn nearest_point(&self, p: Vertex) -> Option<NearestPoint> {
        crate::path::nearest_point(&self.nav, &self.bsp, p)
    }

    /// A* + funnel from `start` to `goal`, routing around closed doors.
    pub fn find_path(
        &self,
        start: Vertex,
        goal: Vertex,
        opts: &PathOptions,
    ) -> Result<PathResult, PathError> {
        find_path_with_walls(&self.nav, &self.bsp, &self.walls, start, goal, opts)
    }

    /// Line of sight from `from` to `to`, occluded by closed doors. Locates
    /// the starting triangle for you.
    pub fn line_of_sight(&self, from: Vertex, to: Vertex) -> LineOfSightResult {
        match self.bsp.locate(&self.nav, from) {
            Some(tri) => los_walk(&self.nav, &self.walls, tri, from, to),
            None => LineOfSightResult::SourceOutsideMesh,
        }
    }

    /// Revalidate a planned polyline against the current world (closed doors
    /// included). `false` means replan.
    pub fn path_clear(&self, points: &[Vertex]) -> bool {
        path_clear_walls(&self.nav, &self.bsp, &self.walls, points)
    }

    /// Approximate visibility polygon from `source`, occluded by closed doors.
    pub fn visibility(
        &self,
        source: Vertex,
        max_radius: f64,
        samples: usize,
    ) -> Option<VisibilityRegion> {
        vis_region(&self.nav, &self.bsp, &self.walls, source, max_radius, samples)
    }

    // -- metadata queries ----------------------------------------------

    /// The metadata value at world point `p`, or `None` if `p` is off the
    /// mesh or its triangle has no value.
    pub fn metadata_at(&self, p: Vertex) -> Option<&M::Value> {
        let tri = self.locate(p)?;
        self.meta.value_at(tri, p)
    }

    /// The zone at world point `p`.
    pub fn zone_at(&self, p: Vertex) -> Option<M::Zone> {
        let tri = self.locate(p)?;
        self.meta.zone(tri)
    }

    /// Zone boundaries crossed along `path` — "entered / left" events you can
    /// surface to the player. See [`zone_crossings`].
    pub fn zone_crossings(&self, path: &PathResult) -> Vec<ZoneCrossing<M::Zone>> {
        zone_crossings(&self.nav, path, |t| self.meta.zone(t))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rsnav_navmesh::build_from_cdt;
    use rsnav_triangle::pslg::{Pslg, PslgSegment, PslgVertex};
    use rsnav_triangle::{delaunay, form_skeleton, CdtMesh, DivConqOptions, VertexSlot};

    /// 20×4 open corridor running along x, single region.
    fn corridor() -> NavMesh {
        let pts = [(0.0, 0.0), (20.0, 0.0), (20.0, 4.0), (0.0, 4.0)];
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

    /// Town to the left of x = 10, wilderness to the right — classified by
    /// triangle centroid. A path down the corridor crosses the boundary once.
    struct TownMap {
        nav_centroids: Vec<Vertex>,
    }

    impl TownMap {
        fn new(nav: &NavMesh) -> Self {
            Self {
                nav_centroids: nav.triangles.iter().map(|t| t.centroid).collect(),
            }
        }
    }

    impl NavMetadata for TownMap {
        type Zone = &'static str;
        type Value = &'static str;

        fn zone(&self, tri: TriangleId) -> Option<&'static str> {
            let c = self.nav_centroids[tri.index()];
            Some(if c.x < 10.0 { "town" } else { "wilderness" })
        }

        fn value_at(&self, tri: TriangleId, _p: Vertex) -> Option<&&'static str> {
            // Borrow a 'static str by zone — fine for the test.
            match self.zone(tri) {
                Some("town") => Some(&"town"),
                Some(_) => Some(&"wilderness"),
                None => None,
            }
        }
    }

    #[test]
    fn world_emplace_path_and_zone_crossings() {
        let nav = corridor();
        let meta = TownMap::new(&nav);
        let world = NavWorld::new(nav, meta);

        let start = Vertex::new(2.0, 2.0); // in town
        let goal = Vertex::new(18.0, 2.0); // in wilderness
        assert_eq!(world.zone_at(start), Some("town"));
        assert_eq!(world.zone_at(goal), Some("wilderness"));

        let path = world.find_path(start, goal, &PathOptions::default()).unwrap();
        let crossings = world.zone_crossings(&path);
        assert_eq!(crossings.len(), 1, "expected one town→wilderness crossing");
        let c = &crossings[0];
        assert_eq!(c.from, Some("town"));
        assert_eq!(c.into, Some("wilderness"));
        // Boundary is at x = 10; the crossing point sits on the portal there.
        assert!(
            (c.point.x - 10.0).abs() < 3.0,
            "crossing should be near x=10, got {:?}",
            c.point
        );

        assert_eq!(world.metadata_at(start), Some(&"town"));
        assert_eq!(world.metadata_at(Vertex::new(-5.0, -5.0)), None);
    }

    #[test]
    fn world_doors_are_door_aware_and_bump_generation() {
        let nav = corridor();
        let mut world = NavWorld::without_metadata(nav);
        let start = Vertex::new(2.0, 2.0);
        let goal = Vertex::new(18.0, 2.0);
        let opts = PathOptions::default();

        assert!(world.find_path(start, goal, &opts).is_ok());
        let g0 = world.generation();

        // Close a door across the corridor middle.
        let id = world.add_door(
            Vertex::new(10.0, -1.0),
            Vertex::new(10.0, 5.0),
            DoorState::Closed,
        );
        assert!(world.generation() > g0, "adding a door must bump generation");
        assert_eq!(
            world.find_path(start, goal, &opts).unwrap_err(),
            PathError::Unreachable
        );
        assert!(matches!(
            world.line_of_sight(start, goal),
            LineOfSightResult::Blocked { .. }
        ));

        // Open it: queries recover, mesh/BSP never rebuilt.
        world.open_door(id);
        assert!(world.find_path(start, goal, &opts).is_ok());
        assert_eq!(world.line_of_sight(start, goal), LineOfSightResult::Clear);
    }

    #[test]
    fn no_metadata_has_no_zones() {
        let nav = corridor();
        let world = NavWorld::without_metadata(nav);
        let path = world
            .find_path(Vertex::new(2.0, 2.0), Vertex::new(18.0, 2.0), &PathOptions::default())
            .unwrap();
        assert!(world.zone_crossings(&path).is_empty());
        assert_eq!(world.zone_at(Vertex::new(2.0, 2.0)), None);
    }
}
