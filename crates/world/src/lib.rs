//! Multi-layer navmesh world.
//!
//! A 3D walkable area is decomposed into N **layers** — each a planar
//! projection triangulated by the ordinary 2D pipeline (`Pslg → CDT →
//! NavMesh`) with per-vertex heights in [`NavMesh::vertex_z`]. Where two
//! layers meet along continuous walkable floor, the cut is a **seam**:
//! one 3D polyline inserted *verbatim* into both layers' PSLGs as
//! constrained segments carrying the same connection marker
//! ([`rsnav_navmesh::connection_marker`]). The CDT never invents or
//! moves vertices (no Steiner points), so both meshes come out holding
//! bit-identical seam vertices — every seam sub-edge exists in both
//! layers with exactly equal endpoints, and stitching is exact key
//! matching, not tolerance-based snapping.
//!
//! [`World::build`] performs that match, records every crossing as a
//! [`Connection`], and unions per-layer regions into world-wide
//! reachability components. [`find_path`](World::find_path) then runs a
//! portal-crossing A* over `(layer, triangle)` nodes in which a seam
//! crossing is an ordinary portal expansion — same cost model as any
//! interior edge, no teleport, no funnel restart at the seam.
//!
//! Genuine discontinuities (ledge drops, ladders, teleports) are *not*
//! seams; model those as application-level jump links between world
//! positions, with their own traversal semantics.

#![forbid(unsafe_code)]

mod astar;
mod funnel;
mod path;

pub use astar::WorldAstarError;
pub use path::{
    world_find_path, WorldPath, WorldPathError, WorldPathOptions, WorldPathPoint, WorldPoint,
};

use std::collections::HashMap;

use rsnav_bsp::Bsp;
use rsnav_common::{TriangleId, Vertex};
use rsnav_navmesh::{connection_id, NavMesh};
use rsnav_navigation::WallInfo;

/// Index of a layer inside a [`World`].
pub type LayerId = u32;

/// One layer: a navmesh plus the per-layer query structures the world
/// keeps warm for it.
pub struct Layer {
    pub navmesh: NavMesh,
    pub bsp: Bsp,
    /// Wall info with permeable connection semantics — seam vertices are
    /// not wall vertices, so clearance never shrinks a seam portal.
    pub walls: WallInfo,
}

/// One side of a seam sub-edge: edge `edge` of triangle `tri` in layer
/// `layer`.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct EdgeRef {
    pub layer: LayerId,
    pub tri: TriangleId,
    pub edge: u8,
}

/// A matched seam crossing: the same physical edge as seen from both
/// layers. Endpoints are bit-identical (position and height).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct SubEdge {
    pub a: EdgeRef,
    pub b: EdgeRef,
}

/// All matched sub-edges carrying one connection id.
#[derive(Clone, Debug)]
pub struct Connection {
    pub id: u32,
    pub sub_edges: Vec<SubEdge>,
}

/// Result of [`World::nearest`]: the closest on-surface point across
/// all layers, by 3D distance.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct WorldNearest {
    pub layer: LayerId,
    pub triangle: TriangleId,
    pub point: Vertex,
    /// Surface height at `point`.
    pub z: f64,
    /// 3D distance from the query point.
    pub distance: f64,
}

/// Why [`World::build`] rejected its input.
#[derive(Clone, Debug, PartialEq)]
pub enum WorldBuildError {
    /// A connection-marked boundary edge has no bit-exact partner in any
    /// layer. Either the seam polyline differs between the two PSLGs
    /// (it must be inserted verbatim into both) or the partner layer is
    /// missing from the build.
    UnmatchedSeamEdge {
        connection: u32,
        layer: LayerId,
        from: Vertex,
        to: Vertex,
    },
    /// More than two boundary edges share one connection id and endpoint
    /// pair — the same seam was inserted into three or more layers, or
    /// two distinct seams reused a connection id along the same
    /// coordinates.
    AmbiguousSeamEdge {
        connection: u32,
        from: Vertex,
        to: Vertex,
    },
}

impl std::fmt::Display for WorldBuildError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            WorldBuildError::UnmatchedSeamEdge { connection, layer, from, to } => write!(
                f,
                "seam edge ({},{})-({},{}) of connection {connection} in layer {layer} \
                 has no bit-exact partner",
                from.x, from.y, to.x, to.y
            ),
            WorldBuildError::AmbiguousSeamEdge { connection, from, to } => write!(
                f,
                "seam edge ({},{})-({},{}) of connection {connection} matches more \
                 than two layers",
                from.x, from.y, to.x, to.y
            ),
        }
    }
}

impl std::error::Error for WorldBuildError {}

/// N stitched layers plus the connection table and global reachability.
pub struct World {
    layers: Vec<Layer>,
    connections: Vec<Connection>,
    /// Seam crossing lookup: which edge of which triangle in which layer
    /// continues where.
    links: HashMap<EdgeRef, EdgeRef>,
    /// `component[layer][region]` — world-wide reachability component of
    /// each per-layer region.
    component: Vec<Vec<u32>>,
    /// Global flat-index offset of each layer's triangles (prefix sums),
    /// shared by the A* scratch arrays.
    tri_offset: Vec<usize>,
    total_triangles: usize,
}

impl World {
    /// Stitch `meshes` into a world. Every connection-marked boundary
    /// edge must find its bit-exact partner, or the build fails — a
    /// dangling seam means the input decomposition is inconsistent, and
    /// silently keeping it would reintroduce exactly the invisible-wall
    /// bugs seams exist to avoid.
    pub fn build(meshes: Vec<NavMesh>) -> Result<Self, WorldBuildError> {
        let layers: Vec<Layer> = meshes
            .into_iter()
            .map(|navmesh| {
                let bsp = Bsp::build(&navmesh);
                let walls = WallInfo::from_navmesh_permeable(&navmesh);
                Layer { navmesh, bsp, walls }
            })
            .collect();

        // --- Match seam sub-edges bit-exactly. ---
        //
        // Key: (connection id, canonical endpoint pair). Positions and
        // heights are keyed by their f64 bit patterns — the seam chain
        // is inserted verbatim into both PSLGs and the CDT never moves
        // a vertex, so equal-by-construction is exact equality.
        type EndpointKey = (u64, u64, u64);
        type SeamKey = (u32, EndpointKey, EndpointKey);
        let endpoint_key = |nav: &NavMesh, v: rsnav_common::VertexId| -> EndpointKey {
            let p = nav.vertex(v);
            (p.x.to_bits(), p.y.to_bits(), nav.vertex_z(v).to_bits())
        };

        let mut pending: HashMap<SeamKey, Vec<EdgeRef>> = HashMap::new();
        for (li, layer) in layers.iter().enumerate() {
            for be in layer.navmesh.boundary_edges() {
                let Some(cid) = connection_id(be.marker) else {
                    continue;
                };
                let ka = endpoint_key(&layer.navmesh, be.from);
                let kb = endpoint_key(&layer.navmesh, be.to);
                let (lo, hi) = if ka <= kb { (ka, kb) } else { (kb, ka) };
                // Recover the edge index on the owning triangle: the
                // boundary edge (from, to) is edge `e` with
                // edge_vertices(e) == (from, to).
                let tri = layer.navmesh.triangle(be.triangle);
                let edge = (0..3u8)
                    .find(|&e| tri.edge_vertices(e as usize) == (be.from, be.to))
                    .expect("boundary edge is an edge of its own triangle");
                pending.entry((cid, lo, hi)).or_default().push(EdgeRef {
                    layer: li as LayerId,
                    tri: be.triangle,
                    edge,
                });
            }
        }

        let mut links: HashMap<EdgeRef, EdgeRef> = HashMap::new();
        let mut by_connection: HashMap<u32, Vec<SubEdge>> = HashMap::new();
        for ((cid, _, _), mut refs) in pending {
            match refs.len() {
                2 => {
                    let (a, b) = (refs[0], refs[1]);
                    links.insert(a, b);
                    links.insert(b, a);
                    by_connection.entry(cid).or_default().push(SubEdge { a, b });
                }
                1 => {
                    let r = refs.pop().expect("len checked");
                    let nav = &layers[r.layer as usize].navmesh;
                    let (from, to) = nav.triangle(r.tri).edge_vertices(r.edge as usize);
                    return Err(WorldBuildError::UnmatchedSeamEdge {
                        connection: cid,
                        layer: r.layer,
                        from: nav.vertex(from),
                        to: nav.vertex(to),
                    });
                }
                _ => {
                    let r = refs[0];
                    let nav = &layers[r.layer as usize].navmesh;
                    let (from, to) = nav.triangle(r.tri).edge_vertices(r.edge as usize);
                    return Err(WorldBuildError::AmbiguousSeamEdge {
                        connection: cid,
                        from: nav.vertex(from),
                        to: nav.vertex(to),
                    });
                }
            }
        }
        let mut connections: Vec<Connection> = by_connection
            .into_iter()
            .map(|(id, sub_edges)| Connection { id, sub_edges })
            .collect();
        connections.sort_by_key(|c| c.id);

        // --- Union per-layer regions into world components. ---
        let region_offset: Vec<usize> = layers
            .iter()
            .scan(0usize, |acc, l| {
                let off = *acc;
                *acc += l.navmesh.region_count as usize;
                Some(off)
            })
            .collect();
        let total_regions: usize = layers
            .iter()
            .map(|l| l.navmesh.region_count as usize)
            .sum();
        let mut uf = UnionFind::new(total_regions);
        for c in &connections {
            for s in &c.sub_edges {
                let ra = region_offset[s.a.layer as usize]
                    + layers[s.a.layer as usize].navmesh.triangle(s.a.tri).region as usize;
                let rb = region_offset[s.b.layer as usize]
                    + layers[s.b.layer as usize].navmesh.triangle(s.b.tri).region as usize;
                uf.union(ra, rb);
            }
        }
        let component: Vec<Vec<u32>> = layers
            .iter()
            .enumerate()
            .map(|(li, l)| {
                (0..l.navmesh.region_count)
                    .map(|r| uf.find(region_offset[li] + r as usize) as u32)
                    .collect()
            })
            .collect();

        let tri_offset: Vec<usize> = layers
            .iter()
            .scan(0usize, |acc, l| {
                let off = *acc;
                *acc += l.navmesh.triangle_count();
                Some(off)
            })
            .collect();
        let total_triangles = layers.iter().map(|l| l.navmesh.triangle_count()).sum();

        Ok(Self {
            layers,
            connections,
            links,
            component,
            tri_offset,
            total_triangles,
        })
    }

    #[inline]
    pub fn layer_count(&self) -> usize {
        self.layers.len()
    }

    /// Borrow one layer. **Panics** on an out-of-range id.
    #[inline]
    pub fn layer(&self, id: LayerId) -> &Layer {
        &self.layers[id as usize]
    }

    /// All matched connections, sorted by id.
    #[inline]
    pub fn connections(&self) -> &[Connection] {
        &self.connections
    }

    /// Where edge `edge` of `tri` in `layer` continues, if it is a
    /// matched seam crossing.
    #[inline]
    pub fn seam_neighbor(&self, r: EdgeRef) -> Option<EdgeRef> {
        self.links.get(&r).copied()
    }

    /// Locate the triangle containing the 3D point `(pos, z)`.
    ///
    /// Stacked floors give a plain 2D locate several valid answers —
    /// one per layer whose footprint covers `pos`. This collects every
    /// layer's candidate and keeps the one whose interpolated surface
    /// height is closest to `z`. `max_dz` bounds the vertical snap
    /// (pass the agent's height, or `f64::INFINITY` to always take the
    /// closest floor); a point vertically farther than `max_dz` from
    /// every surface returns `None`.
    pub fn locate(
        &self,
        pos: Vertex,
        z: f64,
        max_dz: f64,
    ) -> Option<(LayerId, TriangleId)> {
        let mut best: Option<(f64, LayerId, TriangleId)> = None;
        for (li, layer) in self.layers.iter().enumerate() {
            let Some(tri) = layer.bsp.locate(&layer.navmesh, pos) else {
                continue;
            };
            let dz = (layer.navmesh.z_at(tri, pos) - z).abs();
            if dz <= max_dz && best.map_or(true, |(bdz, _, _)| dz < bdz) {
                best = Some((dz, li as LayerId, tri));
            }
        }
        best.map(|(_, l, t)| (l, t))
    }

    /// Snap the 3D point `(pos, z)` to the closest point on any layer's
    /// walkable surface, by full 3D distance (horizontal snap distance
    /// and height difference both count). Returns the layer, triangle,
    /// snapped position, its surface height, and the 3D distance.
    /// `None` only when every layer is empty.
    pub fn nearest(&self, pos: Vertex, z: f64) -> Option<WorldNearest> {
        let mut best: Option<WorldNearest> = None;
        for (li, layer) in self.layers.iter().enumerate() {
            let Some(n) = layer.bsp.nearest(&layer.navmesh, pos) else {
                continue;
            };
            let nz = layer.navmesh.z_at(n.triangle, n.point);
            let d3 = (n.distance * n.distance + (nz - z) * (nz - z)).sqrt();
            if best.as_ref().map_or(true, |b| d3 < b.distance) {
                best = Some(WorldNearest {
                    layer: li as LayerId,
                    triangle: n.triangle,
                    point: n.point,
                    z: nz,
                    distance: d3,
                });
            }
        }
        best
    }

    /// World-wide reachability: `true` if a path can exist between the
    /// two triangles, crossing seams as needed. The cross-layer
    /// counterpart of [`NavMesh::reachable`] — same O(1) cost.
    pub fn reachable(
        &self,
        a: (LayerId, TriangleId),
        b: (LayerId, TriangleId),
    ) -> bool {
        let ca = self.component[a.0 as usize]
            [self.layers[a.0 as usize].navmesh.triangle(a.1).region as usize];
        let cb = self.component[b.0 as usize]
            [self.layers[b.0 as usize].navmesh.triangle(b.1).region as usize];
        ca == cb
    }

    #[inline]
    pub(crate) fn flat_index(&self, layer: LayerId, tri: TriangleId) -> usize {
        self.tri_offset[layer as usize] + tri.index()
    }

    /// Inverse of [`flat_index`](Self::flat_index).
    pub(crate) fn node_to_tri(&self, node: usize) -> (LayerId, TriangleId) {
        let layer = match self.tri_offset.binary_search(&node) {
            // A triangle-less layer shares its offset with the next
            // layer; the node belongs to the *last* layer at that
            // offset (an empty layer owns no nodes).
            Ok(mut i) => {
                while i + 1 < self.tri_offset.len() && self.tri_offset[i + 1] == node {
                    i += 1;
                }
                i
            }
            Err(i) => i - 1,
        };
        (
            layer as LayerId,
            TriangleId::new((node - self.tri_offset[layer]) as u32),
        )
    }

    #[inline]
    pub(crate) fn total_triangles(&self) -> usize {
        self.total_triangles
    }
}

// --- Union-find -------------------------------------------------------------

struct UnionFind {
    parent: Vec<usize>,
}

impl UnionFind {
    fn new(n: usize) -> Self {
        Self {
            parent: (0..n).collect(),
        }
    }

    fn find(&mut self, mut x: usize) -> usize {
        while self.parent[x] != x {
            self.parent[x] = self.parent[self.parent[x]];
            x = self.parent[x];
        }
        x
    }

    fn union(&mut self, a: usize, b: usize) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra != rb {
            self.parent[ra.max(rb)] = ra.min(rb);
        }
    }
}
