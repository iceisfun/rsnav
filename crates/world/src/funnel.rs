//! Cross-seam funnel: one string-pull over the whole corridor.
//!
//! All layers live in one shared horizontal frame, so portals from
//! different layers usually concatenate directly and the classic 2D
//! funnel runs straight through every seam — zero kink, no restart.
//! That breaks only when the corridor's projection self-overlaps: a
//! switchback ramp or stacked floors put the entered layer's portals
//! *on top of* the exited layer's, and the funnel's orientation tests
//! stop meaning anything.
//!
//! The fix is the paper-fold picture. Two charts meeting at a seam are
//! two pages sharing a fold line; a corridor that doubles back is a
//! folded page. Unfolding rotates the entered page by π about the
//! hinge — in the shared 2D frame that is a **reflection across the
//! hinge line**. We detect a fold at each crossing (the entered
//! triangle's interior lands on the same side of the hinge as the
//! exited one), compose the reflection into a running rigid transform,
//! pull the string once over the unfolded portals, and map every
//! corner back bit-exactly (string-pull corners are copies of portal
//! endpoints, so the inverse map is a hash lookup, not arithmetic).
//!
//! Reflections flip orientation, so while the running transform has
//! odd parity the portal's left/right endpoints swap.
//!
//! Residual limit, by design: a fold is detected side-of-line, so
//! partial-angle overlaps (a helix chart rotated < π) can evade it.
//! Cut helices so consecutive charts don't overlap in projection.

use std::collections::HashMap;

use rsnav_common::geom::orient2d;
use rsnav_common::{TriangleId, Vertex};
use rsnav_navigation::funnel::string_pull;
use rsnav_navmesh::NavMesh;

use crate::astar::{interp_edge_z, CorridorStep};
use crate::path::{WorldPathPoint, WorldPoint};
use crate::{EdgeRef, LayerId, World};

/// One funnel portal in corridor order, before unfolding.
struct Portal {
    left: Vertex,
    right: Vertex,
    /// Heights of `left` / `right` (post-shrink, interpolated on the
    /// portal edge).
    left_z: f64,
    right_z: f64,
    /// Layer that owns the portal geometry. For a seam this is the
    /// layer being *exited*; the entered layer shares the identical
    /// coordinates.
    layer: LayerId,
    /// Present when this portal is a seam crossing.
    seam: Option<Seam>,
}

struct Seam {
    /// Raw (unshrunk) seam endpoints — the hinge line — and their
    /// heights (identical on both layers by construction).
    a: Vertex,
    b: Vertex,
    a_z: f64,
    b_z: f64,
    /// Interior probes for the fold test.
    exit_interior: Vertex,
    enter_interior: Vertex,
}

/// A rigid 2D isometry (rotation/reflection + translation).
#[derive(Copy, Clone, Debug)]
struct Iso2 {
    // | a b | x + tx
    // | c d | y + ty
    a: f64,
    b: f64,
    c: f64,
    d: f64,
    tx: f64,
    ty: f64,
    /// Odd number of reflections composed in?
    flip: bool,
}

impl Iso2 {
    const IDENTITY: Self = Self {
        a: 1.0,
        b: 0.0,
        c: 0.0,
        d: 1.0,
        tx: 0.0,
        ty: 0.0,
        flip: false,
    };

    #[inline]
    fn apply(&self, p: Vertex) -> Vertex {
        Vertex::new(
            self.a * p.x + self.b * p.y + self.tx,
            self.c * p.x + self.d * p.y + self.ty,
        )
    }

    /// Reflection across the line through `p0` with (non-zero)
    /// direction `dir`.
    fn reflection(p0: Vertex, dir: Vertex) -> Self {
        let len2 = dir.x * dir.x + dir.y * dir.y;
        let (ux, uy) = (dir.x / len2.sqrt(), dir.y / len2.sqrt());
        let (a, b) = (ux * ux - uy * uy, 2.0 * ux * uy);
        // Reflect about the parallel line through the origin, then fix
        // up the translation so `p0` stays put.
        let (c, d) = (b, -a);
        let tx = p0.x - (a * p0.x + b * p0.y);
        let ty = p0.y - (c * p0.x + d * p0.y);
        Self { a, b, c, d, tx, ty, flip: true }
    }

    /// `self` after `other` (i.e. apply `other` first).
    fn compose_after(&self, other: &Self) -> Self {
        Self {
            a: self.a * other.a + self.b * other.c,
            b: self.a * other.b + self.b * other.d,
            c: self.c * other.a + self.d * other.c,
            d: self.c * other.b + self.d * other.d,
            tx: self.a * other.tx + self.b * other.ty + self.tx,
            ty: self.c * other.tx + self.d * other.ty + self.ty,
            flip: self.flip != other.flip,
        }
    }
}

/// A hinge (seam crossing) in unfolded coordinates, kept for crossing-
/// point insertion after the pull.
struct UnfoldedHinge {
    /// Transformed hinge endpoints.
    a: Vertex,
    b: Vertex,
    /// Original (world) hinge endpoints and their heights.
    a_orig: Vertex,
    b_orig: Vertex,
    a_z: f64,
    b_z: f64,
    /// Layer on the exiting side — crossing points are attributed to it.
    exit_layer: LayerId,
}

pub(crate) fn cross_seam_funnel(
    world: &World,
    corridor: &[CorridorStep],
    start: WorldPoint,
    goal: WorldPoint,
    distance_from_wall: f64,
) -> Vec<WorldPathPoint> {
    let start_z = if corridor.is_empty() {
        0.0
    } else {
        world
            .layer(start.layer)
            .navmesh
            .z_at(corridor[0].tri, start.pos)
    };
    let goal_z = if corridor.is_empty() {
        0.0
    } else {
        world
            .layer(goal.layer)
            .navmesh
            .z_at(corridor[corridor.len() - 1].tri, goal.pos)
    };
    if corridor.len() <= 1 {
        let mut pts = vec![WorldPathPoint { layer: start.layer, pos: start.pos, z: start_z }];
        if goal.pos != start.pos || goal.layer != start.layer {
            pts.push(WorldPathPoint { layer: goal.layer, pos: goal.pos, z: goal_z });
        }
        return pts;
    }

    // --- Build the raw portal sequence. ---
    let mut portals: Vec<Portal> = Vec::with_capacity(corridor.len() + 1);
    for w in corridor.windows(2) {
        let (from, to) = (&w[0], &w[1]);
        let portal = if from.layer == to.layer {
            interior_portal(world, from.layer, from.tri, to.tri, distance_from_wall)
        } else {
            seam_portal(world, from, to, distance_from_wall)
        };
        if let Some(p) = portal {
            portals.push(p);
        }
    }

    // --- Unfold: assign a rigid transform to every portal. ---
    let mut transform = Iso2::IDENTITY;
    let mut unfolded: Vec<(Vertex, Vertex)> = Vec::with_capacity(portals.len() + 2);
    let mut hinges: Vec<UnfoldedHinge> = Vec::new();
    // Bit-exact inverse map from unfolded corner → world point.
    let mut back: HashMap<(u64, u64), WorldPathPoint> = HashMap::new();
    let mut remember = |map: &mut HashMap<(u64, u64), WorldPathPoint>,
                        unfolded_p: Vertex,
                        world_p: WorldPathPoint| {
        map.entry((unfolded_p.x.to_bits(), unfolded_p.y.to_bits()))
            .or_insert(world_p);
    };

    let u_start = start.pos; // first run is always identity
    unfolded.push((u_start, u_start));
    remember(&mut back, u_start, WorldPathPoint { layer: start.layer, pos: start.pos, z: start_z });

    for p in &portals {
        // The portal itself is placed and oriented by the transform in
        // force *before* any fold at this crossing: a seam portal lies
        // on the hinge (the reflection maps it to itself) and its
        // left/right orientation belongs to the exiting side. Only
        // portals *after* the crossing feel the new reflection.
        let (ul, ur) = (transform.apply(p.left), transform.apply(p.right));
        // Reflections invert orientation: swap left/right while the
        // running parity is odd so the funnel's sign tests stay valid.
        let (ul, ur, wl, wr) = if transform.flip {
            (ur, ul, (p.right, p.right_z), (p.left, p.left_z))
        } else {
            (ul, ur, (p.left, p.left_z), (p.right, p.right_z))
        };
        remember(&mut back, ul, WorldPathPoint { layer: p.layer, pos: wl.0, z: wl.1 });
        remember(&mut back, ur, WorldPathPoint { layer: p.layer, pos: wr.0, z: wr.1 });
        unfolded.push((ul, ur));

        if let Some(seam) = &p.seam {
            // Fold test in *unfolded* space: does the entered side land
            // on the same side of the hinge as the exited side?
            let ha = transform.apply(seam.a);
            let hb = transform.apply(seam.b);
            let exit_side = orient2d(ha, hb, transform.apply(seam.exit_interior));
            let enter_side = orient2d(ha, hb, transform.apply(seam.enter_interior));
            if exit_side != 0.0 && enter_side != 0.0 && exit_side.signum() == enter_side.signum()
            {
                transform = Iso2::reflection(ha, hb - ha).compose_after(&transform);
            }
            hinges.push(UnfoldedHinge {
                a: ha,
                b: hb,
                a_orig: seam.a,
                b_orig: seam.b,
                a_z: seam.a_z,
                b_z: seam.b_z,
                exit_layer: p.layer,
            });
        }
    }

    let u_goal = transform.apply(goal.pos);
    unfolded.push((u_goal, u_goal));
    remember(&mut back, u_goal, WorldPathPoint { layer: goal.layer, pos: goal.pos, z: goal_z });

    // --- One pull over the whole unfolded corridor. ---
    let pulled = string_pull(&unfolded);

    // --- Insert seam crossing points and map back. ---
    //
    // Corners are bit-exact copies of unfolded portal endpoints, so the
    // reverse map is a hash lookup. Crossing points (where a straight
    // funnel segment passes through a hinge without a corner) are
    // inserted so the output polyline switches layer at a concrete
    // point on the seam; they are attributed to the exited layer, and
    // their height interpolates along the seam edge.
    let mut out: Vec<WorldPathPoint> = Vec::with_capacity(pulled.len() + hinges.len());
    let mut hinge_iter = hinges.iter().peekable();
    for (i, corner) in pulled.iter().enumerate() {
        let mapped = back
            .get(&(corner.x.to_bits(), corner.y.to_bits()))
            .copied()
            .expect("string-pull corner is a portal endpoint");
        if i > 0 {
            let prev = pulled[i - 1];
            // Consume every hinge this funnel segment crosses.
            while let Some(h) = hinge_iter.peek() {
                let Some((t_seg, t_hinge)) = segment_intersection(prev, *corner, h.a, h.b)
                else {
                    break;
                };
                // A corner sitting exactly on the hinge already marks
                // the switch; don't duplicate it.
                if t_seg > 0.0 && t_seg < 1.0 {
                    let pos = h.a_orig.lerp(h.b_orig, t_hinge);
                    let z = h.a_z + (h.b_z - h.a_z) * t_hinge;
                    out.push(WorldPathPoint { layer: h.exit_layer, pos, z });
                }
                hinge_iter.next();
            }
        }
        out.push(mapped);
    }
    out
}

/// Portal between two triangles of the same layer, oriented (left,
/// right) relative to travel and shrunk by the clearance radius. Mirrors
/// `rsnav_navigation::funnel::oriented_portal`, additionally carrying
/// endpoint heights.
fn interior_portal(
    world: &World,
    layer_id: LayerId,
    from: TriangleId,
    to: TriangleId,
    distance_from_wall: f64,
) -> Option<Portal> {
    let layer = world.layer(layer_id);
    let nav = &layer.navmesh;
    let t_from = nav.triangle(from);
    let i = (0..3).find(|&i| t_from.neighbors[i] == to)?;
    let (va, vb) = t_from.edge_vertices(i);
    let (pa, pb) = (nav.vertex(va), nav.vertex(vb));

    let from_c = t_from.centroid;
    let to_c = nav.triangle(to).centroid;
    let (left_v, right_v, left_p, right_p) = if orient2d(from_c, to_c, pa) > 0.0 {
        (va, vb, pa, pb)
    } else {
        (vb, va, pb, pa)
    };

    let left_wall = layer.walls.is_wall_vertex(left_v);
    let right_wall = layer.walls.is_wall_vertex(right_v);
    let (left, right) = shrink_portal(
        left_p,
        right_p,
        left_wall,
        right_wall,
        distance_from_wall,
    );
    Some(Portal {
        left,
        right,
        left_z: interp_edge_z(nav, va, vb, pa, pb, left),
        right_z: interp_edge_z(nav, va, vb, pa, pb, right),
        layer: layer_id,
        seam: None,
    })
}

/// Portal across a seam crossing, oriented from the exiting triangle's
/// CCW boundary edge: the walkable interior of the exiting triangle is
/// on the left of the directed edge `from → to`, so relative to travel
/// (outward) the `to` endpoint is left.
fn seam_portal(
    world: &World,
    from: &CorridorStep,
    to: &CorridorStep,
    distance_from_wall: f64,
) -> Option<Portal> {
    let layer = world.layer(from.layer);
    let nav = &layer.navmesh;
    let tri = nav.triangle(from.tri);
    // Find the edge of `from.tri` whose seam link leads to `to.tri`.
    let edge = (0..3u8).find(|&e| {
        world
            .seam_neighbor(EdgeRef { layer: from.layer, tri: from.tri, edge: e })
            .is_some_and(|r| r.layer == to.layer && r.tri == to.tri)
    })?;
    let (v_from, v_to) = tri.edge_vertices(edge as usize);
    let (p_from, p_to) = (nav.vertex(v_from), nav.vertex(v_to));

    let (left_v, right_v, left_p, right_p) = (v_to, v_from, p_to, p_from);
    let left_wall = layer.walls.is_wall_vertex(left_v);
    let right_wall = layer.walls.is_wall_vertex(right_v);
    let (left, right) = shrink_portal(
        left_p,
        right_p,
        left_wall,
        right_wall,
        distance_from_wall,
    );

    let enter_nav = &world.layer(to.layer).navmesh;
    Some(Portal {
        left,
        right,
        left_z: interp_edge_z(nav, v_from, v_to, p_from, p_to, left),
        right_z: interp_edge_z(nav, v_from, v_to, p_from, p_to, right),
        layer: from.layer,
        seam: Some(Seam {
            a: p_from,
            b: p_to,
            a_z: nav.vertex_z(v_from),
            b_z: nav.vertex_z(v_to),
            exit_interior: tri.centroid,
            enter_interior: enter_nav.triangle(to.tri).centroid,
        }),
    })
}

/// Shift wall endpoints inward along the portal, mirroring
/// `rsnav_navigation::funnel::oriented_portal`'s clearance model.
fn shrink_portal(
    left_p: Vertex,
    right_p: Vertex,
    left_wall: bool,
    right_wall: bool,
    distance_from_wall: f64,
) -> (Vertex, Vertex) {
    if distance_from_wall <= 0.0 {
        return (left_p, right_p);
    }
    let len = left_p.distance(right_p);
    if len == 0.0 {
        return (left_p, right_p);
    }
    let raw_left = if left_wall { distance_from_wall } else { 0.0 };
    let raw_right = if right_wall { distance_from_wall } else { 0.0 };
    let total = raw_left + raw_right;
    let (s_left, s_right) = if total <= len {
        (raw_left, raw_right)
    } else {
        (raw_left * (len / total), raw_right * (len / total))
    };
    let dir = (right_p - left_p) * (1.0 / len);
    (left_p + dir * s_left, right_p + dir * -s_right)
}

/// Proper intersection of segments `(a0, a1)` and `(b0, b1)`. Returns
/// the parameters `(t_a, t_b)` when the segments cross (inclusive of
/// endpoints); `None` for parallel or disjoint segments.
fn segment_intersection(a0: Vertex, a1: Vertex, b0: Vertex, b1: Vertex) -> Option<(f64, f64)> {
    let r = a1 - a0;
    let s = b1 - b0;
    let denom = r.cross(s);
    if denom == 0.0 {
        return None;
    }
    let t = (b0 - a0).cross(s) / denom;
    let u = (b0 - a0).cross(r) / denom;
    const EPS: f64 = 1e-12;
    ((-EPS..=1.0 + EPS).contains(&t) && (-EPS..=1.0 + EPS).contains(&u))
        .then_some((t.clamp(0.0, 1.0), u.clamp(0.0, 1.0)))
}
