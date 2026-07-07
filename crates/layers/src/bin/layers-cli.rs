//! layers-cli — run the span-heightfield layer pipeline on a mesh file.
//!
//! ```text
//! cargo run -p rsnav-layers --bin layers-cli --release -- <file.obj|file.stl>
//!     [--voxel N] [--step N] [--clearance N] [--slope DEG]
//!     [--min-spans N] [--sample-step N] [--yup] [--path]
//! ```
//!
//! `--yup` treats the input as Y-up (typical OBJ exports) and maps it
//! into the pipeline's Z-up frame. `--path` picks two far-apart points
//! on the two largest layers and runs a cross-layer path as a smoke
//! test.

use std::io::{BufRead, BufReader, Read};
use std::path::Path;
use std::time::Instant;

use rsnav_common::Vec3;
use rsnav_layers::{build_layered_world, LayersConfig};
use rsnav_voxel::PolySoup;
use rsnav_world::{WorldPathOptions, WorldPoint};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut file: Option<String> = None;
    let mut cfg = LayersConfig::default();
    let mut yup = false;
    let mut run_path = false;

    let mut it = args.iter();
    while let Some(a) = it.next() {
        let mut num = |name: &str| -> f64 {
            it.next()
                .unwrap_or_else(|| die(&format!("{name} needs a value")))
                .parse()
                .unwrap_or_else(|_| die(&format!("{name}: not a number")))
        };
        match a.as_str() {
            "--voxel" => cfg.voxel_size = num("--voxel"),
            "--step" => cfg.walkability.max_step_height = num("--step"),
            "--clearance" => cfg.walkability.min_clearance = num("--clearance"),
            "--slope" => cfg.walkability.max_slope_rad = num("--slope").to_radians(),
            "--min-spans" => cfg.min_layer_spans = num("--min-spans") as usize,
            "--sample-step" => cfg.height_sample_step = num("--sample-step") as u32,
            "--yup" => yup = true,
            "--path" => run_path = true,
            other if !other.starts_with('-') && file.is_none() => {
                file = Some(other.to_string());
            }
            other => die(&format!("unknown argument: {other}")),
        }
    }
    let file = file.unwrap_or_else(|| die("usage: layers-cli <file.obj|file.stl> [options]"));

    let mut soup = load(&file).unwrap_or_else(|e| die(&format!("{file}: {e}")));
    if yup {
        for v in &mut soup.vertices {
            *v = Vec3::new(v.x, v.z, v.y);
        }
    }
    let b = soup.bounds();
    println!(
        "{file}: {} vertices, {} triangles, bounds {:.1}..{:.1} x {:.1}..{:.1} x {:.1}..{:.1}",
        soup.vertex_count(),
        soup.triangle_count(),
        b.min.x, b.max.x, b.min.y, b.max.y, b.min.z, b.max.z
    );
    println!(
        "config: voxel {} step {} clearance {} slope {:.0}deg min-spans {} sample-step {}",
        cfg.voxel_size,
        cfg.walkability.max_step_height,
        cfg.walkability.min_clearance,
        cfg.walkability.max_slope_rad.to_degrees(),
        cfg.min_layer_spans,
        cfg.height_sample_step,
    );

    let t0 = Instant::now();
    let built = match build_layered_world(&soup, &cfg) {
        Ok(b) => b,
        Err(e) => die(&format!("build failed: {e}")),
    };
    let dt = t0.elapsed();

    let s = &built.stats;
    println!(
        "built in {:.1?}: {} layers, {} connections ({} seam links), {} spans ({} pruned)",
        dt,
        s.per_layer.len(),
        s.connections,
        s.seam_links,
        s.walkable_spans,
        s.pruned_spans,
    );
    let mut order: Vec<usize> = (0..s.per_layer.len()).collect();
    order.sort_by(|&a, &b| s.per_layer[b].walkable_area.total_cmp(&s.per_layer[a].walkable_area));
    for (rank, &i) in order.iter().take(12).enumerate() {
        let l = &s.per_layer[i];
        println!(
            "  #{rank} layer {i}: {} spans, {} triangles, area {:.1}",
            l.spans, l.triangles, l.walkable_area
        );
    }
    if order.len() > 12 {
        println!("  … {} more layers", order.len() - 12);
    }
    for c in built.world.connections() {
        println!("  connection {}: {} sub-edges", c.id, c.sub_edges.len());
    }

    if run_path {
        smoke_path(&built, &order);
    }
}

/// Cross-layer path smoke test between the two largest layers (or the
/// two most distant points of the largest layer when there's only one).
fn smoke_path(built: &rsnav_layers::LayeredWorld, order: &[usize]) {
    let w = &built.world;
    let centroid = |layer: u32| {
        // Representative interior point of the layer's largest region,
        // snapped on that layer's own mesh (a world-level nearest could
        // slide onto a different floor).
        let l = w.layer(layer);
        let nav = &l.navmesh;
        let region = (0..nav.region_count)
            .max_by(|&a, &b| nav.region_area(a).total_cmp(&nav.region_area(b)))
            .unwrap_or(0);
        let c = nav.region_centroid(region).unwrap_or(nav.triangles[0].centroid);
        let n = l.bsp.nearest(nav, c).expect("layer is non-empty");
        (layer, n.point, nav.z_at(n.triangle, n.point))
    };
    // Prefer the biggest layer connected to the biggest layer via the
    // world components; fall back to far corners of the biggest layer.
    let start = centroid(order[0] as u32);
    let start_tri = w
        .layer(start.0)
        .bsp
        .locate(&w.layer(start.0).navmesh, start.1)
        .unwrap();
    let goal = order[1..]
        .iter()
        .map(|&i| centroid(i as u32))
        .find(|g| {
            let tri = w.layer(g.0).bsp.locate(&w.layer(g.0).navmesh, g.1);
            tri.is_some_and(|t| w.reachable((start.0, start_tri), (g.0, t)))
        })
        .unwrap_or_else(|| {
            let nav = &w.layer(order[0] as u32).navmesh;
            let nb = w.nearest(nav.aabb.max, 0.0).unwrap();
            (nb.layer, nb.point, nb.z)
        });
    println!(
        "path: layer {} ({:.1},{:.1},{:.1}) -> layer {} ({:.1},{:.1},{:.1})",
        start.0, start.1.x, start.1.y, start.2, goal.0, goal.1.x, goal.1.y, goal.2
    );
    match w.find_path(
        WorldPoint { layer: start.0, pos: start.1 },
        WorldPoint { layer: goal.0, pos: goal.1 },
        &WorldPathOptions::default(),
    ) {
        Ok(p) => {
            let layers: std::collections::BTreeSet<u32> =
                p.triangles.iter().map(|&(l, _)| l).collect();
            println!(
                "  ok: {} corners, 3D length {:.1}, corridor {} triangles across layers {:?}",
                p.points.len(),
                p.length(),
                p.triangles.len(),
                layers
            );
        }
        Err(e) => println!("  failed: {e:?}"),
    }
}

fn die(msg: &str) -> ! {
    eprintln!("{msg}");
    std::process::exit(1);
}

// --- Loaders ---------------------------------------------------------------

fn load(path: &str) -> std::io::Result<PolySoup> {
    let lower = path.to_ascii_lowercase();
    if lower.ends_with(".stl") {
        load_stl(path)
    } else {
        load_obj(path)
    }
}

/// Minimal Wavefront OBJ loader (v + f, fan triangulation, 1-based and
/// negative indices).
fn load_obj<P: AsRef<Path>>(path: P) -> std::io::Result<PolySoup> {
    let reader = BufReader::new(std::fs::File::open(path)?);
    let mut soup = PolySoup::new();
    for line in reader.lines() {
        let line = line?;
        let t = line.trim();
        let mut tok = t.split_ascii_whitespace();
        match tok.next() {
            Some("v") => {
                let mut f = || -> f64 {
                    tok.next().and_then(|s| s.parse().ok()).unwrap_or(0.0)
                };
                let (x, y, z) = (f(), f(), f());
                soup.vertices.push(Vec3::new(x, y, z));
            }
            Some("f") => {
                let idx: Vec<u32> = tok
                    .filter_map(|s| s.split('/').next())
                    .filter_map(|s| s.parse::<i64>().ok())
                    .map(|i| {
                        let n = soup.vertices.len() as i64;
                        (if i < 0 { n + i } else { i - 1 }) as u32
                    })
                    .collect();
                for k in 1..idx.len().saturating_sub(1) {
                    soup.triangles.push([idx[0], idx[k], idx[k + 1]]);
                }
            }
            _ => {}
        }
    }
    Ok(soup)
}

/// Binary or ASCII STL.
fn load_stl<P: AsRef<Path>>(path: P) -> std::io::Result<PolySoup> {
    let mut bytes = Vec::new();
    std::fs::File::open(path)?.read_to_end(&mut bytes)?;
    let mut soup = PolySoup::new();
    let is_ascii = bytes.len() >= 6 && &bytes[0..5] == b"solid" && {
        // Binary files can also start with "solid": verify the length math.
        let tri_count = if bytes.len() >= 84 {
            u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize
        } else {
            usize::MAX
        };
        bytes.len() != 84 + tri_count * 50
    };
    if is_ascii {
        let text = String::from_utf8_lossy(&bytes);
        for line in text.lines() {
            let t = line.trim();
            if let Some(rest) = t.strip_prefix("vertex") {
                let vals: Vec<f64> = rest
                    .split_ascii_whitespace()
                    .filter_map(|s| s.parse().ok())
                    .collect();
                if vals.len() == 3 {
                    soup.vertices.push(Vec3::new(vals[0], vals[1], vals[2]));
                }
            }
        }
    } else {
        if bytes.len() < 84 {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "binary STL too short",
            ));
        }
        let n = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
        for i in 0..n {
            let base = 84 + i * 50 + 12; // skip normal
            if base + 36 > bytes.len() {
                break;
            }
            for k in 0..3 {
                let off = base + k * 12;
                let f = |o: usize| {
                    f32::from_le_bytes(bytes[o..o + 4].try_into().unwrap()) as f64
                };
                soup.vertices
                    .push(Vec3::new(f(off), f(off + 4), f(off + 8)));
            }
        }
    }
    // Every consecutive vertex triple is one triangle.
    for i in (0..soup.vertices.len() / 3).map(|i| (i * 3) as u32) {
        soup.triangles.push([i, i + 1, i + 2]);
    }
    Ok(soup)
}
