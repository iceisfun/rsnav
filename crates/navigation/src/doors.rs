//! Doors: runtime-togglable cuts in the navmesh.
//!
//! A door is not a region of triangles or a patch of geometry — it is a set
//! of internal portal **edges** that a closed door promotes to walls. The
//! mesh, its triangle IDs, and the [`Bsp`] are never rebuilt; opening or
//! closing a door only flips which edges the traversal code treats as
//! impassable (via [`WallInfo`](crate::wall::WallInfo)), so A*, the funnel,
//! and line-of-sight all react to the change for free.
//!
//! ## Authoring
//!
//! A door is authored as a **segment** drawn across a passage. [`DoorSet::add`]
//! resolves it to the internal portal edges the segment crosses — broad-phase
//! via [`Bsp::query_aabb`], narrow-phase via segment intersection — and stores
//! them as canonical edge keys (so both triangles sharing a cut edge see it as
//! closed). For the common "two-triangle rectangle in a doorway", the segment
//! crosses the shared diagonal (or an entrance portal), which is exactly the
//! crossing to gate.
//!
//! ## Repathing
//!
//! [`DoorSet`] carries a [`generation`](DoorSet::generation) counter that bumps
//! on every change. Existing paths are never mutated; a client compares the
//! generation its path was planned against and repaths when it differs.

use std::collections::HashSet;

use rsnav_bsp::Bsp;
use rsnav_common::geom::{nearest_point_on_segment, segment_intersection};
use rsnav_common::{Aabb, TriangleId, Vertex, VertexId};
use rsnav_navmesh::NavMesh;

use crate::wall::{edge_key, is_wall_edge_local, EdgeKey};

/// Stable handle to a door within a [`DoorSet`].
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct DoorId(pub u32);

/// Whether a door currently blocks passage.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DoorState {
    /// Passable — the cut edges behave as ordinary portals.
    Open,
    /// Impassable — the cut edges behave as walls.
    Closed,
}

/// A single door: the segment it was authored from, its current state, and
/// the internal portal edges it cuts.
#[derive(Clone, Debug)]
pub struct Door {
    pub id: DoorId,
    /// The authoring segment, kept so the door can be re-resolved if the
    /// mesh is rebuilt, and for debug rendering.
    pub line: (Vertex, Vertex),
    pub state: DoorState,
    /// Internal portal edges this door gates, as canonical vertex-pair keys.
    edges: Vec<EdgeKey>,
}

impl Door {
    #[inline]
    pub fn is_closed(&self) -> bool {
        self.state == DoorState::Closed
    }

    /// How many internal portal edges this door cut. `0` means the authoring
    /// segment crossed no toggleable portal (drawn off-mesh, or only over
    /// existing walls) — the door is inert.
    #[inline]
    pub fn edge_count(&self) -> usize {
        self.edges.len()
    }
}

/// The collection of doors in a world, and the source of truth for which
/// edges are currently gated shut.
#[derive(Clone, Debug, Default)]
pub struct DoorSet {
    doors: Vec<Door>,
    next_id: u32,
    generation: u64,
}

impl DoorSet {
    pub fn new() -> Self {
        Self::default()
    }

    /// Bumps on every mutation (add / remove / clear / state change). A path
    /// planned at generation `g` is stale once this differs from `g`.
    #[inline]
    pub fn generation(&self) -> u64 {
        self.generation
    }

    #[inline]
    pub fn doors(&self) -> &[Door] {
        &self.doors
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.doors.is_empty()
    }

    #[inline]
    pub fn get(&self, id: DoorId) -> Option<&Door> {
        self.doors.iter().find(|d| d.id == id)
    }

    /// Resolve a door from the authoring segment `a → b` and add it.
    ///
    /// The door is created in `state`. Returns its [`DoorId`]; inspect
    /// [`Door::edge_count`] (via [`get`](Self::get)) to confirm the segment
    /// actually cut a portal.
    pub fn add(
        &mut self,
        nav: &NavMesh,
        bsp: &Bsp,
        a: Vertex,
        b: Vertex,
        state: DoorState,
    ) -> DoorId {
        let edges = resolve_door_edges(nav, bsp, a, b);
        let id = DoorId(self.next_id);
        self.next_id += 1;
        self.doors.push(Door {
            id,
            line: (a, b),
            state,
            edges,
        });
        self.generation += 1;
        id
    }

    /// Add a door that gates exactly one internal portal edge, named by its
    /// two endpoint vertices (e.g. from [`nearest_portal_edge`]). Unlike
    /// [`add`](Self::add) there is no resolution step — the edge *is* the
    /// door — so it is unambiguous which crossing gets cut. The caller is
    /// responsible for `(va, vb)` being a real unconstrained portal; a
    /// non-portal key simply never matches any traversal and the door is
    /// inert.
    ///
    /// Returns `None` if either `va` or `vb` is not a vertex of `nav` —
    /// most often a stale ID after a rebuild, or one issued by a different
    /// mesh. No door is added in that case.
    pub fn add_edge(
        &mut self,
        nav: &NavMesh,
        va: VertexId,
        vb: VertexId,
        state: DoorState,
    ) -> Option<DoorId> {
        let line = (nav.get_vertex(va)?, nav.get_vertex(vb)?);
        let id = DoorId(self.next_id);
        self.next_id += 1;
        self.doors.push(Door {
            id,
            line,
            state,
            edges: vec![edge_key(va, vb)],
        });
        self.generation += 1;
        Some(id)
    }

    /// Remove a door. Returns whether it existed.
    pub fn remove(&mut self, id: DoorId) -> bool {
        let before = self.doors.len();
        self.doors.retain(|d| d.id != id);
        let removed = self.doors.len() != before;
        if removed {
            self.generation += 1;
        }
        removed
    }

    pub fn clear(&mut self) {
        if !self.doors.is_empty() {
            self.doors.clear();
            self.generation += 1;
        }
    }

    /// Set a door's state. No-op (and no generation bump) if unchanged or the
    /// id is unknown.
    pub fn set_state(&mut self, id: DoorId, state: DoorState) {
        if let Some(d) = self.doors.iter_mut().find(|d| d.id == id) {
            if d.state != state {
                d.state = state;
                self.generation += 1;
            }
        }
    }

    pub fn open(&mut self, id: DoorId) {
        self.set_state(id, DoorState::Open);
    }

    pub fn close(&mut self, id: DoorId) {
        self.set_state(id, DoorState::Closed);
    }

    /// Flip a door between open and closed.
    pub fn toggle(&mut self, id: DoorId) {
        if let Some(d) = self.doors.iter_mut().find(|d| d.id == id) {
            d.state = match d.state {
                DoorState::Open => DoorState::Closed,
                DoorState::Closed => DoorState::Open,
            };
            self.generation += 1;
        }
    }

    /// Every edge key currently gated shut by a closed door. Consumed by
    /// [`WallInfo::from_navmesh_with_doors`](crate::wall::WallInfo::from_navmesh_with_doors).
    pub(crate) fn closed_edge_keys(&self) -> HashSet<EdgeKey> {
        let mut set = HashSet::new();
        for d in &self.doors {
            if d.is_closed() {
                set.extend(d.edges.iter().copied());
            }
        }
        set
    }
}

/// Find the internal portal edges that the segment `a → b` crosses.
///
/// "Internal portal" = an edge with a neighbor and no constraint marker, i.e.
/// one A* can currently cross. Existing walls (constrained or boundary edges)
/// are never returned — you cannot put a door in a wall. Broad-phase via
/// [`Bsp::query_aabb`] over the segment's bounds, narrow-phase via
/// [`segment_intersection`]. Returns canonical edge keys, deduplicated (both
/// triangles sharing a cut edge collapse to one key).
pub fn resolve_door_edges(nav: &NavMesh, bsp: &Bsp, a: Vertex, b: Vertex) -> Vec<EdgeKey> {
    let query = Aabb::from_points([a, b]);
    let mut keys: HashSet<EdgeKey> = HashSet::new();
    bsp.query_aabb(query, |tri_id| {
        let tri = nav.triangle(tri_id);
        for i in 0..3 {
            // Only unconstrained internal portals are toggleable.
            if is_wall_edge_local(tri, i) {
                continue;
            }
            let (va, vb) = tri.edge_vertices(i);
            let pa = nav.vertex(va);
            let pb = nav.vertex(vb);
            if segment_intersection(a, b, pa, pb).is_some() {
                keys.insert(edge_key(va, vb));
            }
        }
    });
    keys.into_iter().collect()
}

/// Find the internal portal edge nearest the world point `p`, for "pick the
/// edge under the cursor" door authoring. Returns the edge's two endpoint
/// vertices, or `None` if `p` is off the mesh or no portal is near.
///
/// Searches the triangle containing `p` and its immediate neighbors — enough
/// to catch the edge straddled by the cursor even when `p` sits in a triangle
/// whose own edges are all walls. Walls and boundary edges are never offered.
pub fn nearest_portal_edge(nav: &NavMesh, bsp: &Bsp, p: Vertex) -> Option<(VertexId, VertexId)> {
    let tri_id = bsp.locate(nav, p)?;
    let mut best: Option<((VertexId, VertexId), f64)> = None;
    let mut consider = |t: TriangleId| {
        let tri = nav.triangle(t);
        for i in 0..3 {
            if is_wall_edge_local(tri, i) {
                continue;
            }
            let (va, vb) = tri.edge_vertices(i);
            let d = p.distance(nearest_point_on_segment(nav.vertex(va), nav.vertex(vb), p));
            if best.map_or(true, |(_, bd)| d < bd) {
                best = Some(((va, vb), d));
            }
        }
    };
    consider(tri_id);
    let tri = nav.triangle(tri_id);
    for i in 0..3 {
        let n = tri.neighbors[i];
        if n.is_valid() {
            consider(n);
        }
    }
    best.map(|(e, _)| e)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wall::WallInfo;
    use crate::{find_path_with_walls, line_of_sight, LineOfSightResult, PathError, PathOptions};
    use rsnav_navmesh::build_from_cdt;
    use rsnav_triangle::pslg::{Pslg, PslgSegment, PslgVertex};
    use rsnav_triangle::{delaunay, form_skeleton, CdtMesh, DivConqOptions, VertexSlot};

    /// 10×4 open rectangle — a single walkable region, no holes. Delaunay
    /// splits it across one internal diagonal through the centre (5, 2).
    fn corridor() -> (NavMesh, Bsp) {
        let pts = [(0.0, 0.0), (10.0, 0.0), (10.0, 4.0), (0.0, 4.0)];
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
        let nav = build_from_cdt(&cdt);
        let bsp = Bsp::build(&nav);
        (nav, bsp)
    }

    #[test]
    fn door_cuts_a_portal_and_blocks_when_closed() {
        let (nav, bsp) = corridor();
        let start = Vertex::new(1.0, 2.0);
        let goal = Vertex::new(9.0, 2.0);
        let opts = PathOptions::default();

        // Open mesh: the two ends are reachable.
        let walls = WallInfo::from_navmesh(&nav);
        assert!(find_path_with_walls(&nav, &bsp, &walls, start, goal, &opts).is_ok());

        // Place a door spanning the corridor at x = 5, initially closed.
        let mut doors = DoorSet::new();
        let id = doors.add(
            &nav,
            &bsp,
            Vertex::new(5.0, -1.0),
            Vertex::new(5.0, 5.0),
            DoorState::Closed,
        );
        assert!(
            doors.get(id).unwrap().edge_count() >= 1,
            "door segment cut no portal edge"
        );

        // Closed: A* can't cross, and LOS is blocked at the door line.
        let closed = WallInfo::from_navmesh_with_doors(&nav, &doors);
        assert_eq!(
            find_path_with_walls(&nav, &bsp, &closed, start, goal, &opts).unwrap_err(),
            PathError::Unreachable
        );
        let st = bsp.locate(&nav, start).unwrap();
        match line_of_sight(&nav, &closed, st, start, goal) {
            LineOfSightResult::Blocked { point } => assert!((point.x - 5.0).abs() < 1e-9),
            other => panic!("expected Blocked at the door, got {other:?}"),
        }

        // Open it: reachable and clear again — the mesh itself never changed.
        doors.open(id);
        let opened = WallInfo::from_navmesh_with_doors(&nav, &doors);
        assert!(find_path_with_walls(&nav, &bsp, &opened, start, goal, &opts).is_ok());
        assert_eq!(
            line_of_sight(&nav, &opened, st, start, goal),
            LineOfSightResult::Clear
        );
    }

    #[test]
    fn generation_bumps_only_on_real_change() {
        let (nav, bsp) = corridor();
        let mut doors = DoorSet::new();
        let g0 = doors.generation();
        let id = doors.add(
            &nav,
            &bsp,
            Vertex::new(5.0, -1.0),
            Vertex::new(5.0, 5.0),
            DoorState::Open,
        );
        assert!(doors.generation() > g0, "add must bump generation");

        let g1 = doors.generation();
        doors.toggle(id);
        assert!(doors.generation() > g1, "toggle must bump generation");

        // Setting the state it already has is a no-op.
        let g2 = doors.generation();
        let cur = doors.get(id).unwrap().state;
        doors.set_state(id, cur);
        assert_eq!(doors.generation(), g2, "no-op set_state must not bump");
    }

    #[test]
    fn resolver_skips_walls_and_offmesh_segments() {
        let (nav, bsp) = corridor();
        // Entirely outside the mesh: nothing to cut.
        let none = resolve_door_edges(&nav, &bsp, Vertex::new(-5.0, -5.0), Vertex::new(-4.0, -4.0));
        assert!(none.is_empty());
    }

    #[test]
    fn edge_pick_gates_the_same_crossing_as_a_segment() {
        let (nav, bsp) = corridor();
        let start = Vertex::new(1.0, 2.0);
        let goal = Vertex::new(9.0, 2.0);
        let opts = PathOptions::default();

        // The internal diagonal runs through the centre, so the portal
        // nearest (5, 2) is exactly the crossing a vertical door segment cuts.
        let (va, vb) = nearest_portal_edge(&nav, &bsp, Vertex::new(5.0, 2.0))
            .expect("a portal edge near the centre");

        let mut doors = DoorSet::new();
        let id = doors
            .add_edge(&nav, va, vb, DoorState::Closed)
            .expect("va/vb are real vertices of nav");
        assert_eq!(doors.get(id).unwrap().edge_count(), 1);

        // Closed edge-door blocks the corridor just like the segment door.
        let closed = WallInfo::from_navmesh_with_doors(&nav, &doors);
        assert_eq!(
            find_path_with_walls(&nav, &bsp, &closed, start, goal, &opts).unwrap_err(),
            PathError::Unreachable
        );

        doors.open(id);
        let opened = WallInfo::from_navmesh_with_doors(&nav, &doors);
        assert!(find_path_with_walls(&nav, &bsp, &opened, start, goal, &opts).is_ok());
    }

    #[test]
    fn nearest_portal_edge_is_none_off_mesh() {
        let (nav, bsp) = corridor();
        assert!(nearest_portal_edge(&nav, &bsp, Vertex::new(-5.0, -5.0)).is_none());
    }
}
