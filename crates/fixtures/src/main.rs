//! Run the full navmesh pipeline against a directory of PSLG fixtures
//! and print a status table.
//!
//! Default directory: `~/work/gonav/testdata`. Override with a positional
//! arg (file or directory).
//!
//!     cargo run -p rsnav-fixtures                       # scan default dir
//!     cargo run -p rsnav-fixtures -- ~/my/fixtures      # scan custom dir
//!     cargo run -p rsnav-fixtures -- ./broken.json -v   # detail one file
//!
//! Supported JSON schema (gonav fixture format):
//!
//! ```jsonc
//! {
//!   "version": 1,
//!   "outer_polygon": [{"x": ..., "y": ...}, ...],   // optional
//!   "perimeters":    [[{"x": ..., "y": ...}, ...]], // optional (our format)
//!   "holes":         [[{"x": ..., "y": ...}, ...]]  // optional
//! }
//! ```
//!
//! `outer_polygon` (single perimeter) and `perimeters` (multiple) are
//! both accepted; if both are present, `outer_polygon` becomes the first
//! entry of the merged perimeter list. Files with no perimeter at all
//! (e.g. gonav's `trouble*.json` snapshots) are reported as `no_outer`
//! and skipped — the navmesh pipeline needs a bounding region to work.

use std::path::{Path, PathBuf};
use std::time::Instant;

use serde::Deserialize;

use rsnav_common::{Polygon, Vertex};
use rsnav_navmesh::{build_from_cdt, NavMesh};
use rsnav_triangle::{
    carve_holes, delaunay,
    form_skeleton,
    pslg::{Pslg, PslgHole, PslgSegment, PslgVertex},
    CdtMesh, DivConqOptions, VertexSlot,
};

// --- Fixture schema -----------------------------------------------------

#[derive(Deserialize, Debug)]
struct Fixture {
    #[serde(default)]
    outer_polygon: Option<Vec<Point>>,
    #[serde(default)]
    perimeters: Vec<Vec<Point>>,
    #[serde(default)]
    holes: Vec<Vec<Point>>,
}

#[derive(Deserialize, Debug, Clone, Copy)]
struct Point {
    x: f64,
    y: f64,
}

impl Point {
    fn to_vertex(self) -> Vertex {
        Vertex::new(self.x, self.y)
    }
}

// --- Per-fixture result -------------------------------------------------

#[derive(Debug)]
struct Outcome {
    file: String,
    n_perimeters: usize,
    n_outer_verts: Option<usize>,
    n_holes: usize,
    n_hole_verts: usize,
    bbox: Option<Bbox>,
    status: Status,
    triangles: Option<usize>,
    regions: Option<u32>,
    build_ms: Option<f64>,
    // Per-hole interior-point + which-hole-it-fell-in details, only
    // populated with --verbose.
    hole_diagnostics: Vec<HoleDiagnostic>,
}

#[derive(Debug, Clone, Copy)]
struct Bbox {
    min_x: f64,
    min_y: f64,
    max_x: f64,
    max_y: f64,
}

#[derive(Debug)]
#[allow(dead_code)] // `index` / `n_verts` are read via Debug-print fall-back; suppress the warning.
struct HoleDiagnostic {
    index: usize,
    n_verts: usize,
    seed: Option<Vertex>,
    centroid_outside_polygon: bool,
}

#[derive(Debug)]
enum Status {
    Ok,
    NoPerimeter,
    ParseError(String),
    IoError(String),
    BuildEmpty,
    InteriorPointFailed { hole_index: usize },
    Panic(String),
}

impl Status {
    fn short(&self) -> &'static str {
        match self {
            Status::Ok => "ok",
            Status::NoPerimeter => "no_outer",
            Status::ParseError(_) => "parse_err",
            Status::IoError(_) => "io_err",
            Status::BuildEmpty => "empty",
            Status::InteriorPointFailed { .. } => "no_seed",
            Status::Panic(_) => "PANIC",
        }
    }

    fn detail(&self) -> Option<String> {
        match self {
            Status::Ok | Status::NoPerimeter | Status::BuildEmpty => None,
            Status::ParseError(s) | Status::IoError(s) | Status::Panic(s) => Some(s.clone()),
            Status::InteriorPointFailed { hole_index } => {
                Some(format!("hole {} has no interior point", hole_index))
            }
        }
    }
}

// --- main ---------------------------------------------------------------

fn main() {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    let verbose = args.iter().any(|a| a == "-v" || a == "--verbose");
    args.retain(|a| !(a == "-v" || a == "--verbose"));

    let target: PathBuf = args.into_iter().next().map(PathBuf::from).unwrap_or_else(default_dir);

    let paths: Vec<PathBuf> = if target.is_file() {
        vec![target.clone()]
    } else if target.is_dir() {
        let mut entries: Vec<PathBuf> = match std::fs::read_dir(&target) {
            Ok(r) => r
                .filter_map(|e| e.ok().map(|e| e.path()))
                .filter(|p| p.extension().map_or(false, |x| x == "json"))
                .collect(),
            Err(e) => {
                eprintln!("cannot read {}: {}", target.display(), e);
                std::process::exit(1);
            }
        };
        entries.sort();
        entries
    } else {
        eprintln!("not found: {}", target.display());
        std::process::exit(1);
    };

    if paths.is_empty() {
        eprintln!("no .json fixtures under {}", target.display());
        std::process::exit(1);
    }

    println!("scanning {} fixture(s) under {}", paths.len(), target.display());
    println!();
    print_header();
    let mut outcomes = Vec::with_capacity(paths.len());
    for p in &paths {
        let outcome = run_fixture(p, verbose);
        print_row(&outcome);
        outcomes.push(outcome);
    }

    if verbose {
        println!();
        for outcome in &outcomes {
            print_details(outcome);
        }
    }

    // Bottom-line summary.
    let n_total = outcomes.len();
    let n_ok = outcomes.iter().filter(|o| matches!(o.status, Status::Ok)).count();
    let n_skipped = outcomes.iter().filter(|o| matches!(o.status, Status::NoPerimeter)).count();
    let n_failed = n_total - n_ok - n_skipped;
    println!();
    println!(
        "summary: {} fixture(s) — {} ok, {} skipped (no perimeter), {} failed",
        n_total, n_ok, n_skipped, n_failed
    );
    if n_failed > 0 {
        std::process::exit(2);
    }
}

fn default_dir() -> PathBuf {
    std::env::var("HOME")
        .map(PathBuf::from)
        .unwrap_or_default()
        .join("work/gonav/testdata")
}

// --- Run a single fixture ----------------------------------------------

fn run_fixture(path: &Path, want_diagnostics: bool) -> Outcome {
    let name = path
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_else(|| path.display().to_string());

    let text = match std::fs::read_to_string(path) {
        Ok(t) => t,
        Err(e) => return blank(name).with_status(Status::IoError(format!("{e}"))),
    };
    let fixture: Fixture = match serde_json::from_str(&text) {
        Ok(f) => f,
        Err(e) => return blank(name).with_status(Status::ParseError(format!("{e}"))),
    };

    // Compose perimeters: outer_polygon (gonav) merged with perimeters (ours).
    let mut perimeters: Vec<Vec<Vertex>> = fixture
        .perimeters
        .iter()
        .map(|p| p.iter().map(|q| q.to_vertex()).collect())
        .collect();
    if let Some(outer) = &fixture.outer_polygon {
        perimeters.insert(0, outer.iter().map(|q| q.to_vertex()).collect());
    }
    let holes: Vec<Vec<Vertex>> = fixture
        .holes
        .iter()
        .map(|h| h.iter().map(|q| q.to_vertex()).collect())
        .collect();

    let n_perimeters = perimeters.len();
    let n_outer_verts = perimeters.first().map(|p| p.len());
    let n_holes = holes.len();
    let n_hole_verts: usize = holes.iter().map(|h| h.len()).sum();

    // BBox over everything we authored.
    let mut all_pts: Vec<Vertex> = perimeters.iter().flatten().copied().collect();
    all_pts.extend(holes.iter().flatten().copied());
    let bbox = compute_bbox(&all_pts);

    if perimeters.is_empty() {
        return Outcome {
            file: name,
            n_perimeters,
            n_outer_verts,
            n_holes,
            n_hole_verts,
            bbox,
            status: Status::NoPerimeter,
            triangles: None,
            regions: None,
            build_ms: None,
            hole_diagnostics: Vec::new(),
        };
    }

    // Per-hole interior point. If any hole has none we bail before the
    // CDT — that's a strictly malformed input (e.g. all-collinear points)
    // and the navmesh build can't proceed without a seed.
    let mut hole_seeds: Vec<Vertex> = Vec::with_capacity(n_holes);
    let mut diagnostics: Vec<HoleDiagnostic> = Vec::new();
    for (i, h) in holes.iter().enumerate() {
        let poly = Polygon::from_vertices(h.clone());
        let seed = poly.interior_point();
        if want_diagnostics {
            diagnostics.push(HoleDiagnostic {
                index: i,
                n_verts: h.len(),
                seed,
                centroid_outside_polygon: !poly.contains(centroid(h)),
            });
        }
        match seed {
            Some(s) => hole_seeds.push(s),
            None => {
                return Outcome {
                    file: name,
                    n_perimeters,
                    n_outer_verts,
                    n_holes,
                    n_hole_verts,
                    bbox,
                    status: Status::InteriorPointFailed { hole_index: i },
                    triangles: None,
                    regions: None,
                    build_ms: None,
                    hole_diagnostics: diagnostics,
                };
            }
        }
    }

    // Build, catching panics so a single fixture never kills the run.
    let build = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        run_pipeline(&perimeters, &holes, &hole_seeds)
    }));

    match build {
        Ok((nav, build_ms)) => {
            let tris = nav.triangle_count();
            Outcome {
                file: name,
                n_perimeters,
                n_outer_verts,
                n_holes,
                n_hole_verts,
                bbox,
                status: if tris == 0 { Status::BuildEmpty } else { Status::Ok },
                triangles: Some(tris),
                regions: Some(nav.region_count),
                build_ms: Some(build_ms),
                hole_diagnostics: diagnostics,
            }
        }
        Err(panic_info) => {
            let msg = panic_info
                .downcast_ref::<String>()
                .cloned()
                .or_else(|| {
                    panic_info
                        .downcast_ref::<&'static str>()
                        .map(|s| s.to_string())
                })
                .unwrap_or_else(|| "(non-string panic payload)".into());
            Outcome {
                file: name,
                n_perimeters,
                n_outer_verts,
                n_holes,
                n_hole_verts,
                bbox,
                status: Status::Panic(msg),
                triangles: None,
                regions: None,
                build_ms: None,
                hole_diagnostics: diagnostics,
            }
        }
    }
}

fn run_pipeline(
    perimeters: &[Vec<Vertex>],
    holes: &[Vec<Vertex>],
    hole_seeds: &[Vertex],
) -> (NavMesh, f64) {
    let start = Instant::now();
    let mut pslg = Pslg::new();
    let mut next_idx = 0u32;

    for poly in perimeters.iter().chain(holes.iter()) {
        let start_idx = next_idx;
        for v in poly {
            pslg.vertices.push(PslgVertex::new(*v));
            next_idx += 1;
        }
        let n = poly.len() as u32;
        for i in 0..n {
            pslg.segments.push(PslgSegment {
                a: start_idx + i,
                b: start_idx + (i + 1) % n,
                marker: 1,
            });
        }
    }
    for s in hole_seeds {
        pslg.holes.push(PslgHole { point: *s });
    }

    // Collapse exact-position duplicate vertices BEFORE building the CDT.
    // delaunay() drops them silently from the triangulation and any
    // segment referencing a dropped ID then crashes the segment-insertion
    // pass. (Real fixtures from gonav hit this; e.g. lut_gholein has 22
    // duplicate positions where adjacent ring vertices coincide.)
    let pslg = pslg.deduplicate();

    let mut cdt = CdtMesh::new();
    for v in &pslg.vertices {
        cdt.push_vertex(VertexSlot::new(v.position, 0));
    }

    delaunay(&mut cdt, DivConqOptions::default());
    form_skeleton(&mut cdt, &pslg, None);
    carve_holes(&mut cdt, &pslg, false);
    let nav = build_from_cdt(&cdt);
    let ms = start.elapsed().as_secs_f64() * 1000.0;
    (nav, ms)
}

// --- Geometry helpers --------------------------------------------------

fn centroid(verts: &[Vertex]) -> Vertex {
    let n = verts.len().max(1) as f64;
    let cx: f64 = verts.iter().map(|v| v.x).sum::<f64>() / n;
    let cy: f64 = verts.iter().map(|v| v.y).sum::<f64>() / n;
    Vertex::new(cx, cy)
}

fn compute_bbox(verts: &[Vertex]) -> Option<Bbox> {
    if verts.is_empty() {
        return None;
    }
    let mut bb = Bbox {
        min_x: f64::INFINITY,
        min_y: f64::INFINITY,
        max_x: f64::NEG_INFINITY,
        max_y: f64::NEG_INFINITY,
    };
    for v in verts {
        if v.x < bb.min_x {
            bb.min_x = v.x;
        }
        if v.y < bb.min_y {
            bb.min_y = v.y;
        }
        if v.x > bb.max_x {
            bb.max_x = v.x;
        }
        if v.y > bb.max_y {
            bb.max_y = v.y;
        }
    }
    Some(bb)
}

// --- Output formatting --------------------------------------------------

const FILE_COL: usize = 38;

fn print_header() {
    println!(
        "{:<width$} {:>5} {:>5} {:>6} {:>10} {:>7} {:>7} {:>9}",
        "file", "v_out", "holes", "v_hole", "status", "tris", "regions", "build_ms",
        width = FILE_COL,
    );
    println!("{}", "-".repeat(FILE_COL + 56));
}

fn print_row(o: &Outcome) {
    let file = truncate(&o.file, FILE_COL);
    let v_out = o
        .n_outer_verts
        .map(|n| format!("{n}"))
        .unwrap_or_else(|| "-".into());
    let tris = o
        .triangles
        .map(|n| format!("{n}"))
        .unwrap_or_else(|| "-".into());
    let regs = o
        .regions
        .map(|n| format!("{n}"))
        .unwrap_or_else(|| "-".into());
    let ms = o
        .build_ms
        .map(|m| format!("{:.2}", m))
        .unwrap_or_else(|| "-".into());
    println!(
        "{:<width$} {:>5} {:>5} {:>6} {:>10} {:>7} {:>7} {:>9}",
        file,
        v_out,
        o.n_holes,
        o.n_hole_verts,
        o.status.short(),
        tris,
        regs,
        ms,
        width = FILE_COL,
    );
    if let Some(d) = o.status.detail() {
        // wrap long detail under the row
        for chunk in d.split('\n') {
            println!("  ↳ {chunk}");
        }
    }
}

fn print_details(o: &Outcome) {
    println!("─── {} ───", o.file);
    println!("  perimeters: {}", o.n_perimeters);
    println!("  holes:      {} ({} total vertices)", o.n_holes, o.n_hole_verts);
    if let Some(bb) = &o.bbox {
        println!(
            "  bbox:       [{:.1}..{:.1}] × [{:.1}..{:.1}]  (size {:.1} × {:.1})",
            bb.min_x,
            bb.max_x,
            bb.min_y,
            bb.max_y,
            bb.max_x - bb.min_x,
            bb.max_y - bb.min_y
        );
    }
    if !o.hole_diagnostics.is_empty() {
        let n_concave: usize = o
            .hole_diagnostics
            .iter()
            .filter(|d| d.centroid_outside_polygon)
            .count();
        let n_no_seed: usize = o
            .hole_diagnostics
            .iter()
            .filter(|d| d.seed.is_none())
            .count();
        println!(
            "  holes: {} with centroid OUTSIDE polygon (would have broken pre-fix), {} with no interior_point",
            n_concave, n_no_seed
        );
    }
    if let Some(t) = o.triangles {
        println!(
            "  result: {} triangles, {} regions, built in {:.2} ms",
            t,
            o.regions.unwrap_or(0),
            o.build_ms.unwrap_or(0.0)
        );
    }
    if let Some(d) = o.status.detail() {
        println!("  status: {} ({})", o.status.short(), d);
    } else {
        println!("  status: {}", o.status.short());
    }
    println!();
}

fn truncate(s: &str, n: usize) -> String {
    if s.len() <= n {
        s.to_string()
    } else {
        let mut t = s[..n.saturating_sub(1)].to_string();
        t.push('…');
        t
    }
}

fn blank(name: String) -> Outcome {
    Outcome {
        file: name,
        n_perimeters: 0,
        n_outer_verts: None,
        n_holes: 0,
        n_hole_verts: 0,
        bbox: None,
        status: Status::Ok, // placeholder, replaced via with_status
        triangles: None,
        regions: None,
        build_ms: None,
        hole_diagnostics: Vec::new(),
    }
}

trait WithStatus {
    fn with_status(self, s: Status) -> Self;
}
impl WithStatus for Outcome {
    fn with_status(mut self, s: Status) -> Self {
        self.status = s;
        self
    }
}
