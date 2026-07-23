//! The inset build pipeline: offset -> planarize -> CDT -> winding cull.
//!
//! [`build_cdt_with_inset`] is the crossing-tolerant front-end that
//! replaces the delaunay / `form_skeleton` / `carve_holes` sequence for
//! authored perimeter+hole scenes:
//!
//! 1. **Normalize**: consecutive-duplicate removal, degenerate-ring
//!    skip (uniform across all inset values — the same ring set enters
//!    the soup at inset 0 and inset r), winding normalization
//!    (perimeter CCW, hole CW).
//! 2. **Offset** (skipped at inset 0): every edge pushed left by the
//!    inset — erosion for both ring kinds — allowing self-intersection
//!    and flipped lobes ([`offset_ring_left`]).
//! 3. **Planarize** with the robust orientation predicate
//!    ([`planarize_with`] + [`crate::predicates::orient2d`]): the
//!    constraint set that reaches `form_skeleton` cannot self-intersect,
//!    which is what makes hole-crosses-perimeter input build at all.
//! 4. **Cull** by signed winding ([`carve_by_winding`]), then drop soup
//!    constraints stranded between kept triangles
//!    ([`drop_interior_constraints`]) so no phantom region splits or
//!    phantom walls survive.
//!
//! Erosion by radius r == inset the perimeters by r AND dilate the
//! holes by r; both are the same left-offset once rings carry their
//! natural orientation. Fully-eroded input is `Ok` with zero live
//! triangles, not an error.

use rsnav_common::offset::{offset_ring_left, OffsetOptions, SoupContour};
use rsnav_common::planarize::{planarize_with, PlanarizeError, SnapGrid};
use rsnav_common::polygon::{Polygon, Winding};
use rsnav_common::{Aabb, Vertex};

use crate::divconq::{delaunay, DivConqOptions};
use crate::mesh::{CdtMesh, VertexSlot};
use crate::pslg::{Pslg, PslgSegment, PslgVertex};
use crate::segment::{form_skeleton, SegmentInsertError};
use crate::winding::{carve_by_winding, drop_interior_constraints};

/// What a ring means. Passed structurally — never inferred from
/// markers, whose numbering schemes are caller-specific.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum RingKind {
    Perimeter,
    Hole,
}

/// One input ring: an implicitly closed point sequence (any winding —
/// normalization is internal), its kind, and the marker its derived
/// constraint segments carry.
#[derive(Copy, Clone, Debug)]
pub struct InsetRing<'a> {
    pub points: &'a [Vertex],
    pub kind: RingKind,
    pub marker: i32,
}

/// Options for [`build_cdt_with_inset`].
#[derive(Copy, Clone, Debug, Default)]
pub struct InsetOptions {
    /// Join behavior for the offset stage.
    pub offset: OffsetOptions,
    /// Snap-grid target cell; `None` picks [`SnapGrid::auto`] from the
    /// soup bounding box and the inset. When `Some`, must be finite and
    /// positive.
    pub snap_cell: Option<f64>,
}

/// Failure modes of [`build_cdt_with_inset`].
#[derive(Debug)]
pub enum InsetError {
    /// `inset` was not finite and `>= 0` (i.e. NaN, negative, or infinite).
    InvalidInset(f64),
    /// `InsetOptions::snap_cell` was `Some(v)` with `v` not a positive,
    /// finite, normal number.
    InvalidSnapCell(f64),
    /// A ring vertex coordinate was not finite (NaN or infinite).
    NonFiniteVertex,
    /// Planarization failed (degenerate input or non-convergence).
    Planarize(PlanarizeError),
    /// `form_skeleton` rejected a planarized segment. The planarizer's
    /// contract makes this unreachable for crossing reasons — if it
    /// fires, it is a planarizer bug and must be surfaced, never
    /// swallowed.
    Segment(SegmentInsertError),
}

impl std::fmt::Display for InsetError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidInset(v) => write!(f, "inset must be finite and >= 0, got {v}"),
            Self::InvalidSnapCell(v) => {
                write!(f, "snap_cell must be a positive finite number, got {v}")
            }
            Self::NonFiniteVertex => {
                write!(f, "ring vertex coordinate was not finite (NaN or infinite)")
            }
            Self::Planarize(e) => write!(f, "planarize: {e}"),
            Self::Segment(e) => write!(f, "internal: planarized segments still crossed: {e}"),
        }
    }
}

impl std::error::Error for InsetError {}

/// Result of [`build_cdt_with_inset`].
#[derive(Debug)]
pub struct InsetBuild {
    /// The carved CDT, ready for `build_from_cdt` (optionally after
    /// `clip_ears`).
    pub mesh: CdtMesh,
    /// The offset soup the cull classified against — kept for debug
    /// rendering.
    pub soup: Vec<SoupContour>,
    /// Input indices (into `rings`) of degenerate rings dropped at
    /// entry: fewer than 3 distinct points after consecutive-duplicate
    /// removal, or zero area. Callers surface these — a skipped
    /// perimeter should be a build error, not silently missing
    /// geometry.
    pub skipped_rings: Vec<(usize, RingKind)>,
}

/// Build a carved CDT from `rings` eroded by `inset` (`>= 0`, finite;
/// `0` = no erosion but still the crossing-tolerant path). See the
/// module docs for the pipeline; degenerate-ring policy and marker
/// handling are uniform across all inset values.
pub fn build_cdt_with_inset(
    rings: &[InsetRing<'_>],
    inset: f64,
    opts: &InsetOptions,
) -> Result<InsetBuild, InsetError> {
    if !(inset.is_finite() && inset >= 0.0) {
        return Err(InsetError::InvalidInset(inset));
    }
    // A non-finite ring coordinate would otherwise slip through to the
    // snap-grid sizing and panic there; reject it up front instead.
    for ring in rings {
        if ring.points.iter().any(|v| !v.x.is_finite() || !v.y.is_finite()) {
            return Err(InsetError::NonFiniteVertex);
        }
    }

    // Entry policy: normalize and skip degenerate rings, identically
    // for every inset value.
    let mut soup: Vec<SoupContour> = Vec::with_capacity(rings.len());
    let mut skipped: Vec<(usize, RingKind)> = Vec::new();
    let mut walkable_contours = 0usize;
    for (idx, ring) in rings.iter().enumerate() {
        match normalize_ring(ring) {
            Some(poly) => {
                // A perimeter whose bbox min-dimension is <= 2*inset is
                // provably fully eroded (erosion of a shape is contained
                // in the erosion of its bbox) — drop it BEFORE offsetting.
                // Beyond correctness this also keeps the naive corner
                // joins from flinging chords far outside the scene at
                // extreme insets, where they could wind up positive.
                if ring.kind == RingKind::Perimeter && inset > 0.0 {
                    let bbox = poly.aabb();
                    if bbox.width().min(bbox.height()) <= 2.0 * inset {
                        continue;
                    }
                }
                let contour = if inset == 0.0 {
                    SoupContour {
                        points: poly.vertices.clone(),
                        marker: ring.marker,
                    }
                } else {
                    // The entry check filtered degenerates, so the
                    // offset primitive cannot refuse this ring.
                    offset_ring_left(&poly, inset, ring.marker, &opts.offset)
                        .expect("non-degenerate ring refused by offset_ring_left")
                };
                if ring.kind == RingKind::Perimeter {
                    walkable_contours += 1;
                }
                soup.push(contour);
            }
            None => skipped.push((idx, ring.kind)),
        }
    }
    // No perimeter contour left (none provided, all degenerate, or all
    // fully eroded): nothing can be walkable. Ok-and-empty, not an error
    // — callers distinguish the cases via `skipped_rings`.
    if walkable_contours == 0 {
        return Ok(InsetBuild {
            mesh: CdtMesh::new(),
            soup,
            skipped_rings: skipped,
        });
    }

    // Snap grid from explicit target or the soup's bounding box.
    let grid = match opts.snap_cell {
        Some(cell) => {
            if !(cell.is_normal() && cell > 0.0) {
                return Err(InsetError::InvalidSnapCell(cell));
            }
            SnapGrid::from_target(cell)
        }
        None => {
            let bbox = Aabb::from_points(soup.iter().flat_map(|c| c.points.iter().copied()));
            SnapGrid::auto(&bbox, inset)
        }
    };

    let planar = planarize_with(&soup, grid, crate::predicates::orient2d)
        .map_err(InsetError::Planarize)?;
    if planar.vertices.len() < 3 {
        return Err(InsetError::Planarize(PlanarizeError::Degenerate));
    }

    // CDT over exactly the planarized vertex set.
    let mut mesh = CdtMesh::new();
    let mut pslg = Pslg::new();
    for &v in &planar.vertices {
        mesh.push_vertex(VertexSlot::new(v, 0));
        pslg.vertices.push(PslgVertex::new(v));
    }
    for &(a, b, marker) in &planar.segments {
        pslg.segments.push(PslgSegment { a, b, marker });
    }
    delaunay(&mut mesh, DivConqOptions::default());
    form_skeleton(&mut mesh, &pslg, None).map_err(InsetError::Segment)?;

    carve_by_winding(&mut mesh, &soup);

    // Soup markers: the exact set carried by the surviving rings —
    // derived here, never a caller-supplied predicate or threshold.
    let mut soup_markers: Vec<i32> = soup.iter().map(|c| c.marker).collect();
    soup_markers.sort_unstable();
    soup_markers.dedup();
    drop_interior_constraints(&mut mesh, &soup_markers);

    Ok(InsetBuild {
        mesh,
        soup,
        skipped_rings: skipped,
    })
}

/// Consecutive-duplicate removal (closing wrap included), then `None`
/// for rings with fewer than 3 points or zero area; otherwise the ring
/// with its natural orientation: perimeter CCW, hole CW.
fn normalize_ring(ring: &InsetRing<'_>) -> Option<Polygon> {
    let mut pts: Vec<Vertex> = Vec::with_capacity(ring.points.len());
    for &p in ring.points {
        if pts.last() != Some(&p) {
            pts.push(p);
        }
    }
    while pts.len() > 1 && pts.first() == pts.last() {
        pts.pop();
    }
    if pts.len() < 3 {
        return None;
    }
    let mut poly = Polygon::from_vertices(pts);
    if poly.winding() == Winding::Degenerate {
        return None;
    }
    match ring.kind {
        RingKind::Perimeter => poly.ensure_winding(Winding::CounterClockwise),
        RingKind::Hole => poly.ensure_winding(Winding::Clockwise),
    }
    Some(poly)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn verts(pts: &[(f64, f64)]) -> Vec<Vertex> {
        pts.iter().map(|&(x, y)| Vertex::new(x, y)).collect()
    }

    fn live_area(mesh: &CdtMesh) -> f64 {
        let mut area = 0.0;
        for i in 1..mesh.triangles.len() as u32 {
            let slot = mesh.triangle(i);
            if slot.is_dead() || !slot.vertices.iter().all(|v| v.is_valid()) {
                continue;
            }
            let a = mesh.vertex_pos(slot.vertices[0]);
            let b = mesh.vertex_pos(slot.vertices[1]);
            let c = mesh.vertex_pos(slot.vertices[2]);
            area += rsnav_common::geom::signed_area2(a, b, c).abs() * 0.5;
        }
        area
    }

    fn covers(mesh: &CdtMesh, p: Vertex) -> bool {
        (1..mesh.triangles.len() as u32).any(|i| {
            let slot = mesh.triangle(i);
            if slot.is_dead() || !slot.vertices.iter().all(|v| v.is_valid()) {
                return false;
            }
            let a = mesh.vertex_pos(slot.vertices[0]);
            let b = mesh.vertex_pos(slot.vertices[1]);
            let c = mesh.vertex_pos(slot.vertices[2]);
            rsnav_common::geom::point_in_triangle(a, b, c, p)
        })
    }

    #[test]
    fn square_with_hole_inset_zero() {
        let outer = verts(&[(0.0, 0.0), (40.0, 0.0), (40.0, 40.0), (0.0, 40.0)]);
        let hole = verts(&[(15.0, 15.0), (25.0, 15.0), (25.0, 25.0), (15.0, 25.0)]);
        let rings = [
            InsetRing { points: &outer, kind: RingKind::Perimeter, marker: 10 },
            InsetRing { points: &hole, kind: RingKind::Hole, marker: 20 },
        ];
        let built = build_cdt_with_inset(&rings, 0.0, &InsetOptions::default()).unwrap();
        assert!(built.skipped_rings.is_empty());
        let area = live_area(&built.mesh);
        assert!((area - (1600.0 - 100.0)).abs() < 1e-6, "area {area}");
        assert!(covers(&built.mesh, Vertex::new(5.0, 5.0)));
        assert!(!covers(&built.mesh, Vertex::new(20.0, 20.0)));
    }

    #[test]
    fn square_inset_yields_exact_inner_square() {
        // Erosion of an axis-aligned square is exact: [r, 40-r]^2.
        let outer = verts(&[(0.0, 0.0), (40.0, 0.0), (40.0, 40.0), (0.0, 40.0)]);
        let rings = [InsetRing { points: &outer, kind: RingKind::Perimeter, marker: 1 }];
        let built = build_cdt_with_inset(&rings, 5.0, &InsetOptions::default()).unwrap();
        let area = live_area(&built.mesh);
        assert!((area - 900.0).abs() < 1e-3, "expected 30x30, got area {area}");
        assert!(covers(&built.mesh, Vertex::new(20.0, 20.0)));
        assert!(!covers(&built.mesh, Vertex::new(2.0, 20.0)));
    }

    #[test]
    fn hole_dilates() {
        let outer = verts(&[(0.0, 0.0), (40.0, 0.0), (40.0, 40.0), (0.0, 40.0)]);
        let hole = verts(&[(15.0, 15.0), (25.0, 15.0), (25.0, 25.0), (15.0, 25.0)]);
        let rings = [
            InsetRing { points: &outer, kind: RingKind::Perimeter, marker: 1 },
            InsetRing { points: &hole, kind: RingKind::Hole, marker: 2 },
        ];
        let built = build_cdt_with_inset(&rings, 2.0, &InsetOptions::default()).unwrap();
        // Point 1 unit outside the original hole edge: swallowed by the
        // dilated hole.
        assert!(!covers(&built.mesh, Vertex::new(14.0, 20.0)));
        // Point 3.5 units outside: still walkable (allowing miter slack).
        assert!(covers(&built.mesh, Vertex::new(11.0, 20.0)));
    }

    #[test]
    fn full_erosion_is_ok_and_empty() {
        let outer = verts(&[(0.0, 0.0), (40.0, 0.0), (40.0, 40.0), (0.0, 40.0)]);
        let rings = [InsetRing { points: &outer, kind: RingKind::Perimeter, marker: 1 }];
        let built = build_cdt_with_inset(&rings, 1000.0, &InsetOptions::default()).unwrap();
        assert_eq!(built.mesh.live_triangle_count(), 0, "must be Ok-and-empty");
    }

    #[test]
    fn degenerate_rings_skipped_uniformly() {
        let outer = verts(&[(0.0, 0.0), (40.0, 0.0), (40.0, 40.0), (0.0, 40.0)]);
        let bad_hole = verts(&[(10.0, 10.0), (12.0, 10.0)]); // 2 points
        for inset in [0.0, 5.0] {
            let rings = [
                InsetRing { points: &outer, kind: RingKind::Perimeter, marker: 1 },
                InsetRing { points: &bad_hole, kind: RingKind::Hole, marker: 2 },
            ];
            let built = build_cdt_with_inset(&rings, inset, &InsetOptions::default()).unwrap();
            assert_eq!(
                built.skipped_rings,
                vec![(1, RingKind::Hole)],
                "inset {inset}: degenerate ring must be skipped and reported"
            );
            assert!(built.mesh.live_triangle_count() > 0);
        }
        // A sole degenerate perimeter yields Ok-and-empty with the skip
        // reported, identically at every radius — the caller decides to
        // surface it as a build error.
        for inset in [0.0, 5.0] {
            let rings = [InsetRing { points: &bad_hole, kind: RingKind::Perimeter, marker: 1 }];
            let built = build_cdt_with_inset(&rings, inset, &InsetOptions::default()).unwrap();
            assert_eq!(built.mesh.live_triangle_count(), 0);
            assert_eq!(built.skipped_rings, vec![(0, RingKind::Perimeter)]);
        }
    }

    #[test]
    fn input_winding_is_irrelevant() {
        // Perimeter given CW and hole given CCW: normalization fixes both.
        let outer_cw = verts(&[(0.0, 40.0), (40.0, 40.0), (40.0, 0.0), (0.0, 0.0)]);
        let hole_ccw = verts(&[(15.0, 15.0), (25.0, 15.0), (25.0, 25.0), (15.0, 25.0)]);
        let rings = [
            InsetRing { points: &outer_cw, kind: RingKind::Perimeter, marker: 1 },
            InsetRing { points: &hole_ccw, kind: RingKind::Hole, marker: 2 },
        ];
        let built = build_cdt_with_inset(&rings, 0.0, &InsetOptions::default()).unwrap();
        assert!(covers(&built.mesh, Vertex::new(5.0, 5.0)));
        assert!(!covers(&built.mesh, Vertex::new(20.0, 20.0)));
    }

    /// Seeded property sweep over random star-shaped scenes: perimeter
    /// plus 0-2 star holes placed inside or straddling. For radii
    /// 0 < r1 < r2, checks per case:
    ///   1. every radius builds (Ok, never a panic);
    ///   2. containment — every kept centroid has winding >= 1 against
    ///      the normalized INPUT rings (output never exceeds the
    ///      inset-0 region);
    ///   3. area monotonically non-increasing in r;
    ///   4. nesting — mesh(r2) centroids are covered by mesh(r1);
    ///   5. determinism — same input twice gives bit-identical vertices.
    #[test]
    fn property_random_star_scenes() {
        use rsnav_common::rng::Lcg;

        let mut rng = Lcg(0x5EED_CAFE);
        let mut star = |cx: f64, cy: f64, base: f64, rng: &mut Lcg| -> Vec<Vertex> {
            let n = 6 + (rng.next_u64() % 7) as usize; // 6..12 points
            (0..n)
                .map(|i| {
                    let ang = (i as f64 / n as f64) * std::f64::consts::TAU;
                    let rad = base * (0.55 + 0.45 * rng.next_f64());
                    Vertex::new(cx + ang.cos() * rad, cy + ang.sin() * rad)
                })
                .collect()
        };

        for case in 0..25 {
            let perimeter = star(50.0, 50.0, 40.0, &mut rng);
            let n_holes = (rng.next_u64() % 3) as usize;
            let holes: Vec<Vec<Vertex>> = (0..n_holes)
                .map(|_| {
                    // Center anywhere in the scene: inside, straddling,
                    // or outside the perimeter.
                    let cx = 10.0 + rng.next_f64() * 80.0;
                    let cy = 10.0 + rng.next_f64() * 80.0;
                    star(cx, cy, 8.0 + rng.next_f64() * 10.0, &mut rng)
                })
                .collect();

            let build = |r: f64| -> InsetBuild {
                let mut rings = vec![InsetRing {
                    points: &perimeter,
                    kind: RingKind::Perimeter,
                    marker: 1,
                }];
                for (i, h) in holes.iter().enumerate() {
                    rings.push(InsetRing {
                        points: h,
                        kind: RingKind::Hole,
                        marker: 2 + i as i32,
                    });
                }
                build_cdt_with_inset(&rings, r, &InsetOptions::default())
                    .unwrap_or_else(|e| panic!("case {case}, r {r}: {e}"))
            };

            // Normalized input rings for the containment oracle.
            let mut input_contours: Vec<rsnav_common::SoupContour> = Vec::new();
            {
                let mut p = rsnav_common::Polygon::from_vertices(perimeter.clone());
                p.ensure_winding(rsnav_common::Winding::CounterClockwise);
                input_contours.push(rsnav_common::SoupContour { points: p.vertices, marker: 1 });
                for h in &holes {
                    let mut hp = rsnav_common::Polygon::from_vertices(h.clone());
                    hp.ensure_winding(rsnav_common::Winding::Clockwise);
                    input_contours
                        .push(rsnav_common::SoupContour { points: hp.vertices, marker: 2 });
                }
            }

            let radii = [0.0, 1.5, 4.0];
            let mut prev_area = f64::INFINITY;
            let mut meshes: Vec<InsetBuild> = Vec::new();
            for &r in &radii {
                let built = build(r);

                // (2) containment against the input scene.
                for i in 1..built.mesh.triangles.len() as u32 {
                    let slot = built.mesh.triangle(i);
                    if slot.is_dead() || !slot.vertices.iter().all(|v| v.is_valid()) {
                        continue;
                    }
                    let a = built.mesh.vertex_pos(slot.vertices[0]);
                    let b = built.mesh.vertex_pos(slot.vertices[1]);
                    let c = built.mesh.vertex_pos(slot.vertices[2]);
                    let centroid =
                        Vertex::new((a.x + b.x + c.x) / 3.0, (a.y + b.y + c.y) / 3.0);
                    let wn = crate::winding::winding_number(centroid, &input_contours);
                    assert!(
                        wn >= 1,
                        "case {case}, r {r}: kept centroid {centroid:?} outside the \
                         input walkable region (winding {wn})"
                    );
                }

                // (3) area monotone.
                let area = live_area(&built.mesh);
                assert!(
                    area <= prev_area + 1e-6,
                    "case {case}, r {r}: area grew {prev_area} -> {area}"
                );
                prev_area = area;
                meshes.push(built);
            }

            // (4) nesting: mesh(4.0) centroids covered by mesh(1.5).
            let coarse = &meshes[2].mesh;
            let fine = &meshes[1].mesh;
            for i in 1..coarse.triangles.len() as u32 {
                let slot = coarse.triangle(i);
                if slot.is_dead() || !slot.vertices.iter().all(|v| v.is_valid()) {
                    continue;
                }
                let a = coarse.vertex_pos(slot.vertices[0]);
                let b = coarse.vertex_pos(slot.vertices[1]);
                let c = coarse.vertex_pos(slot.vertices[2]);
                let centroid = Vertex::new((a.x + b.x + c.x) / 3.0, (a.y + b.y + c.y) / 3.0);
                assert!(
                    covers(fine, centroid),
                    "case {case}: mesh(4.0) centroid {centroid:?} not inside mesh(1.5)"
                );
            }

            // (5) determinism.
            let again = build(1.5);
            let dump = |m: &CdtMesh| -> Vec<(u64, u64)> {
                m.vertices
                    .iter()
                    .map(|v| (v.position.x.to_bits(), v.position.y.to_bits()))
                    .collect()
            };
            assert_eq!(
                dump(&meshes[1].mesh),
                dump(&again.mesh),
                "case {case}: non-deterministic rebuild"
            );
        }
    }

    #[test]
    fn deterministic_across_runs() {
        let outer = verts(&[(0.1, 0.2), (40.3, 0.1), (39.8, 40.2), (0.2, 39.9)]);
        let hole = verts(&[(15.1, 15.2), (25.3, 15.1), (24.9, 25.2), (15.2, 24.8)]);
        let build = || {
            let rings = [
                InsetRing { points: &outer, kind: RingKind::Perimeter, marker: 1 },
                InsetRing { points: &hole, kind: RingKind::Hole, marker: 2 },
            ];
            build_cdt_with_inset(&rings, 3.0, &InsetOptions::default()).unwrap()
        };
        let (b1, b2) = (build(), build());
        let dump = |b: &InsetBuild| {
            let mut out: Vec<(u64, u64)> = Vec::new();
            for v in &b.mesh.vertices {
                out.push((v.position.x.to_bits(), v.position.y.to_bits()));
            }
            out
        };
        assert_eq!(dump(&b1), dump(&b2));
        assert_eq!(b1.mesh.live_triangle_count(), b2.mesh.live_triangle_count());
    }
}
