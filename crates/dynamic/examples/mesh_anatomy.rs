//! What a built navmesh actually contains — the answer to "I had a cell
//! array, what do I have now?".
//!
//! Builds a small ASCII bitfield through the real
//! `build_navmesh_from_bitfield` pipeline and then walks the result:
//! triangle count against cell count, one triangle's vertices /
//! neighbors / edge_markers with `TriangleId::INVALID` on the rim,
//! per-region area / bounds / centroid, the boundary-edge iterator, and
//! area-uniform spawn sampling.
//!
//! Run with:
//!   cargo run --release -p rsnav-dynamic --example mesh_anatomy

use rsnav_common::{TriangleId, Vertex};
use rsnav_dynamic::{build_navmesh_from_bitfield, BuildOptions};
use rsnav_navmesh::NavMesh;
use rsnav_polygon_extract::Bitfield;

fn main() {
    // '#' = walkable, '.' = wall. Rows are written top-down for
    // readability; grid() flips them so row 0 is the BOTTOM row, which is
    // the convention the whole pipeline uses.
    //
    // Three disconnected walkable regions: an upper hall with an enclosed
    // hole, and two lower rooms with no route between them.
    let rows = [
        "........................",
        ".######################.",
        ".######################.",
        ".####........##########.",
        ".####........##########.",
        ".######################.",
        ".######################.",
        "........................",
        "..######......#########.",
        "..######......#########.",
        "..######......#########.",
        "........................",
    ];
    let bf = grid(24, &rows);

    let cells = (bf.width as usize) * (bf.height as usize);
    let walkable = bf.data.iter().filter(|c| **c).count();
    println!("bitfield: {}x{} = {} cells, {} walkable", bf.width, bf.height, cells, walkable);

    let build = build_navmesh_from_bitfield(&bf, &BuildOptions::default())
        .expect("this grid has walkable regions");
    let nav = &build.navmesh;

    println!(
        "navmesh:  {} vertices, {} triangles, {} region(s), built in {:.3} ms",
        nav.vertex_count(),
        nav.triangle_count(),
        nav.region_count,
        build.build_ms,
    );
    println!(
        "          {} walkable cells collapsed into {} triangles ({:.1}x fewer objects)",
        walkable,
        nav.triangle_count(),
        walkable as f64 / nav.triangle_count() as f64,
    );
    println!("          aabb: ({:.1}, {:.1}) .. ({:.1}, {:.1})",
        nav.aabb.min.x, nav.aabb.min.y, nav.aabb.max.x, nav.aabb.max.y);

    // --- one triangle, in full ------------------------------------------
    //
    // Pick a triangle that has at least one boundary edge, so the
    // INVALID neighbor sentinel actually shows up.
    let rim = (0..nav.triangle_count())
        .map(|i| TriangleId::new(i as u32))
        .find(|&id| (0..3).any(|e| nav.triangle(id).is_edge_boundary(e)))
        .expect("a closed mesh has boundary triangles");

    let t = nav.triangle(rim);
    println!("\ntriangle {}:", rim.get());
    println!("  region  {}", t.region);
    println!("  area    {:.3}", t.area);
    println!("  centroid ({:.3}, {:.3})", t.centroid.x, t.centroid.y);
    for e in 0..3 {
        // Edge e is the edge OPPOSITE vertices[e].
        let (a, b) = t.edge_vertices(e);
        let (pa, pb) = (nav.vertex(a), nav.vertex(b));
        let neighbor = if t.neighbors[e].is_valid() {
            format!("tri {}", t.neighbors[e].get())
        } else {
            "INVALID (mesh rim)".to_string()
        };
        println!(
            "  edge {}: ({:.1},{:.1})->({:.1},{:.1})  marker {}  {}  neighbor {}",
            e,
            pa.x, pa.y, pb.x, pb.y,
            t.edge_markers[e],
            if t.is_edge_constrained(e) { "WALL " } else { "open " },
            neighbor,
        );
    }

    // --- regions ---------------------------------------------------------
    //
    // A region is a connected component under "these two triangles share
    // an edge that is not constrained". It is not a room, not a zone, and
    // not anything you authored; it is what nav.reachable() compares.
    println!("\nregions:");
    for r in 0..nav.region_count {
        let tris = nav.region_triangles(r).count();
        let area = nav.region_area(r);
        let bounds = nav.region_bounds(r).expect("region has triangles");
        let centroid = nav.region_centroid(r).expect("region has triangles");
        println!(
            "  region {}: {:>3} tris, area {:>7.2}, bounds ({:.1},{:.1})..({:.1},{:.1}), centroid ({:.2},{:.2})",
            r, tris, area,
            bounds.min.x, bounds.min.y, bounds.max.x, bounds.max.y,
            centroid.x, centroid.y,
        );
    }

    // Reachability is a region-id comparison, so it is O(1) and it is the
    // cheap pre-check A* runs before searching.
    let first_of = |r: u32| nav.region_triangles(r).next().expect("non-empty region");
    if nav.region_count >= 2 {
        let (a, b) = (first_of(0), first_of(1));
        println!(
            "  reachable(region 0 tri {}, region 1 tri {}) = {}",
            a.get(), b.get(), nav.reachable(a, b),
        );
    }

    // --- boundary edges --------------------------------------------------
    //
    // Every edge with no triangle on the far side: the outer rim of each
    // region plus every hole rim. Yielded once each, directed so the
    // walkable interior is on the left.
    let boundary: Vec<_> = nav.boundary_edges().collect();
    let perimeter: f64 = boundary
        .iter()
        .map(|e| nav.vertex(e.from).distance(nav.vertex(e.to)))
        .sum();
    println!(
        "\nboundary: {} edges, total length {:.2} (markers seen: {:?})",
        boundary.len(),
        perimeter,
        {
            let mut m: Vec<i32> = boundary.iter().map(|e| e.marker).collect();
            m.sort_unstable();
            m.dedup();
            m
        },
    );
    for e in boundary.iter().take(4) {
        let (a, b) = (nav.vertex(e.from), nav.vertex(e.to));
        println!(
            "  tri {:>3}  ({:.1},{:.1}) -> ({:.1},{:.1})  marker {}",
            e.triangle.get(), a.x, a.y, b.x, b.y, e.marker,
        );
    }
    println!("  ... {} more", boundary.len().saturating_sub(4));

    // --- spawn points ----------------------------------------------------
    //
    // random_point is uniform over walkable AREA, not over triangles, so
    // a mesh with one huge triangle and many slivers still spawns sensibly.
    // It consumes three uniform f64 in [0,1) per call and is O(triangles).
    let mut unit = splitmix(0x5EED_1234);
    println!("\nspawn points (uniform over area):");
    for _ in 0..4 {
        let p = nav.random_point(&mut unit).expect("mesh is non-empty");
        println!("  ({:>6.3}, {:>6.3})  region {}", p.x, p.y, region_at(nav, p));
    }
    println!("spawn points restricted to region 0:");
    for _ in 0..3 {
        let p = nav
            .random_point_in_region(0, &mut unit)
            .expect("region 0 has triangles");
        println!("  ({:>6.3}, {:>6.3})  region {}", p.x, p.y, region_at(nav, p));
    }

    // The Bsp arrives with the build; locate() is the "which triangle is
    // this point in" query the whole runtime is built on.
    let probe = Vertex::new(3.5, 3.5);
    match build.bsp.locate(nav, probe) {
        Some(id) => println!(
            "\nbsp.locate({:.1}, {:.1}) = tri {} (region {})",
            probe.x, probe.y, id.get(), nav.triangle(id).region,
        ),
        None => println!("\nbsp.locate({:.1}, {:.1}) = off-mesh", probe.x, probe.y),
    }
}

/// Which region a sampled point landed in — used only to show that the
/// region-restricted sampler respects its argument.
fn region_at(nav: &NavMesh, p: Vertex) -> String {
    for t in nav.triangles.iter() {
        let v = [nav.vertex(t.vertices[0]), nav.vertex(t.vertices[1]), nav.vertex(t.vertices[2])];
        if rsnav_common::geom::point_in_triangle(v[0], v[1], v[2], p) {
            return t.region.to_string();
        }
    }
    "?".to_string()
}

/// Parse `rows` of `#` (walkable) / `.` (wall) into a `Bitfield`. Rows are
/// given top-down for readability; this flips to math-up, so row 0 of the
/// bitfield is the LAST string in `rows`.
fn grid(width: u32, rows: &[&str]) -> Bitfield {
    let height = rows.len() as u32;
    let mut data = vec![false; (width as usize) * (height as usize)];
    for (i, row) in rows.iter().enumerate() {
        let math_row = height as usize - 1 - i;
        for (col, ch) in row.chars().enumerate() {
            data[math_row * (width as usize) + col] = ch == '#';
        }
    }
    Bitfield::new(width, height, data).expect("every row is `width` chars")
}

/// Deterministic uniform `[0, 1)` source, so this example prints the same
/// spawn points on every run and on every machine.
fn splitmix(seed: u64) -> impl FnMut() -> f64 {
    let mut state = seed;
    move || {
        state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        (((z ^ (z >> 31)) >> 11) as f64) / ((1u64 << 53) as f64)
    }
}
