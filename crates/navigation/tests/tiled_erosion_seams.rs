//! Grid erosion is seam-safe; contour inset is not.
//!
//! [`TiledWorld::stitch_all`] links tiles by matching boundary edges whose
//! *world-space* segments are collinear and overlap. Anything that moves a
//! tile's boundary off the tile border line therefore disconnects it from
//! its neighbour. `rsnav_dynamic::BuildOptions::inset` does exactly that,
//! per tile, which is why `tiled.rs` forbids it.
//!
//! `rsnav_polygon_extract::Bitfield::eroded` does not, provided the caller
//! obeys the ordering rule: **erode the global grid, then `subgrid` it into
//! tiles.** Clipping a tile out of an already-eroded grid leaves the
//! boundary on the tile edge in *both* tiles at identical integer
//! coordinates, so the match is exact.
//!
//! These tests pin that difference, plus the geometry hazard worth being
//! nervous about (`diagonal_smoothing` pulling a seam vertex off the border
//! line) and the negative control (per-tile inset, which must break).

use rsnav_common::Vertex;
use rsnav_dynamic::{build_navmesh_from_bitfield, BuildOptions};
use rsnav_navigation::TiledWorld;
use rsnav_navmesh::NavMesh;
use rsnav_polygon_extract::{Bitfield, ErodeOptions};

const TILE: u32 = 16;
const SEAM_X: f64 = 16.0;

fn build(bits: &Bitfield, smoothing: bool, inset: Option<f64>) -> NavMesh {
    let mut opts = BuildOptions::default();
    opts.extract.diagonal_smoothing = smoothing;
    opts.inset = inset;
    // Serial so a failure is reproducible; the build is thread-invariant
    // anyway (asserted by par_bench).
    opts.threads = 1;
    build_navmesh_from_bitfield(bits, &opts)
        .expect("build should succeed on these fixtures")
        .navmesh
}

/// Boundary edges of `nav` that lie entirely on the world line `x = SEAM_X`
/// once the tile is placed at `offset_x`. `stitch_all` can only link a pair
/// of tiles if both have at least one.
fn seam_edges(nav: &NavMesh, offset_x: f64) -> usize {
    nav.boundary_edges()
        .filter(|be| {
            let (a, b) = (nav.vertex(be.from), nav.vertex(be.to));
            (a.x + offset_x - SEAM_X).abs() < 1e-9 && (b.x + offset_x - SEAM_X).abs() < 1e-9
        })
        .count()
}

/// Slice `bits` down the middle at x = 16, build both halves, stitch, and
/// report `(seam edges left, seam edges right, links, path found)`.
fn stitch_halves(
    bits: &Bitfield,
    height: u32,
    smoothing: bool,
    inset: Option<f64>,
    from: Vertex,
    to: Vertex,
) -> (usize, usize, usize, bool) {
    let left = build(&bits.subgrid(0, 0, TILE, height), smoothing, inset);
    let right = build(&bits.subgrid(TILE, 0, TILE, height), smoothing, inset);
    let (ea, eb) = (seam_edges(&left, 0.0), seam_edges(&right, SEAM_X));
    let mut world = TiledWorld::new();
    world.add_tile(left, Vertex::new(0.0, 0.0));
    world.add_tile(right, Vertex::new(SEAM_X, 0.0));
    world.stitch_all(1e-9);
    let links = world.links().len();
    let path = world.find_path(from, to).is_some();
    (ea, eb, links, path)
}

/// A 32×16 map: two rooms joined by a 3-cell-tall corridor that crosses the
/// seam at x = 16.
fn two_rooms() -> Bitfield {
    let (w, h) = (32u32, 16u32);
    let mut bits = Bitfield::empty(w, h);
    for r in 2..14 {
        for c in 2..30 {
            bits.set(c, r, true);
        }
    }
    for r in 2..14 {
        for c in 12..20 {
            bits.set(c, r, (7..=9).contains(&r));
        }
    }
    bits
}

fn erode(bits: &Bitfield, radius: f64) -> Bitfield {
    bits.eroded(&ErodeOptions { radius, threads: 0 })
        .expect("finite non-negative radius")
}

/// The headline claim: erode globally, slice, stitch — seams still link and
/// a cross-seam path exists.
#[test]
fn global_erosion_then_subgrid_keeps_seams_linked() {
    let bits = two_rooms();
    let (from, to) = (Vertex::new(4.0, 8.0), Vertex::new(28.0, 8.0));
    // radius 1.0 peels one ring; the 3-cell corridor survives as 1 cell.
    for &radius in &[0.0f64, 1.0] {
        let eroded = erode(&bits, radius);
        for &smoothing in &[true, false] {
            let (ea, eb, links, path) =
                stitch_halves(&eroded, 16, smoothing, None, from, to);
            assert!(
                ea > 0 && eb > 0,
                "radius={radius} smoothing={smoothing}: seam edges vanished (A={ea} B={eb})"
            );
            assert!(
                links > 0,
                "radius={radius} smoothing={smoothing}: tiles did not stitch"
            );
            assert!(
                path,
                "radius={radius} smoothing={smoothing}: no cross-seam path"
            );
        }
    }
}

/// The feature *working*, not failing: a 3-cell corridor cannot hold a
/// radius-2 agent, so it erodes shut — and because erosion ran once
/// globally, both tiles agree on that without any cross-tile coordination.
#[test]
fn erosion_closes_a_corridor_symmetrically_in_both_tiles() {
    let eroded = erode(&two_rooms(), 2.0);
    for &smoothing in &[true, false] {
        let (ea, eb, links, path) = stitch_halves(
            &eroded,
            16,
            smoothing,
            None,
            Vertex::new(4.0, 8.0),
            Vertex::new(28.0, 8.0),
        );
        assert_eq!((ea, eb, links), (0, 0, 0), "smoothing={smoothing}");
        assert!(!path, "a radius-2 agent must not fit through a 3-cell corridor");
    }
}

/// Negative control. Per-tile contour inset recedes the seam edges by `r`,
/// so `stitch_all` finds nothing to match — the documented incompatibility
/// in `tiled.rs`, reproduced.
#[test]
fn per_tile_contour_inset_breaks_the_seam() {
    let bits = two_rooms();
    let (from, to) = (Vertex::new(4.0, 8.0), Vertex::new(28.0, 8.0));

    // Baseline: no inset, the tiles link.
    let (_, _, links, path) = stitch_halves(&bits, 16, true, None, from, to);
    assert!(links > 0 && path, "control baseline must link");

    for &inset in &[0.5f64, 1.0] {
        let (ea, eb, links, path) = stitch_halves(&bits, 16, true, Some(inset), from, to);
        assert_eq!(
            (ea, eb, links),
            (0, 0, 0),
            "per-tile inset={inset} unexpectedly kept seam geometry"
        );
        assert!(!path, "per-tile inset={inset} must disconnect the tiles");
    }
}

/// The hazard worth being nervous about: `diagonal_smoothing` removes a
/// stair vertex only when its two incident edges are unit-length *and*
/// perpendicular *and* the pre-edge runs parallel-same-direction to the
/// out-edge. On a clipped tile border the ring can only turn inward, so the
/// parallel test fails and the border edge survives. Pin that with a
/// diagonal staircase crossing the seam, which is the shape most likely to
/// trip it.
#[test]
fn diagonal_smoothing_does_not_eat_seam_edges_on_a_staircase() {
    let (w, h) = (32u32, 32u32);
    let mut bits = Bitfield::empty(w, h);
    for r in 2..30 {
        for c in 2..8 {
            bits.set(c, r, true);
        }
        for c in 24..30 {
            bits.set(c, r, true);
        }
    }
    // A 2-cell-wide diagonal staircase joining the two rooms.
    let mut row = 4i32;
    for c in 8..24u32 {
        for rr in row..row + 2 {
            if rr >= 0 && (rr as u32) < h {
                bits.set(c, rr as u32, true);
            }
        }
        if c % 2 == 1 {
            row += 1;
        }
    }
    let (from, to) = (Vertex::new(4.0, 4.0), Vertex::new(27.0, 27.0));
    for &smoothing in &[true, false] {
        let (ea, eb, links, path) = stitch_halves(&bits, h, smoothing, None, from, to);
        assert!(
            ea > 0 && eb > 0 && links > 0,
            "smoothing={smoothing}: staircase seam lost (A={ea} B={eb} links={links})"
        );
        assert!(path, "smoothing={smoothing}: no path across the staircase seam");
    }
}
