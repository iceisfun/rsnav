//! Dynamic navmesh updates for game loops.
//!
//! A [`NavWorker`] owns a background thread that turns [`Bitfield`]
//! snapshots into a [`NavBuild`] (a [`NavMesh`] + [`Bsp`]). The worker
//! coalesces submissions — if you submit faster than it can build, only
//! the most recent snapshot is processed. The main thread reads the
//! latest published build at frame start via [`NavWorker::poll_swap`].
//!
//! ```no_run
//! use std::sync::Arc;
//! use rsnav_dynamic::{NavWorker, BuildOptions};
//! use rsnav_polygon_extract::Bitfield;
//!
//! // Some 32x32 walkable map with a central wall.
//! let bf = Bitfield::empty(32, 32);
//! let mut worker = NavWorker::spawn(BuildOptions::default());
//! worker.submit_snapshot(Arc::new(bf));
//!
//! // In the game loop:
//! loop {
//!     if worker.poll_swap() {
//!         // a new mesh is available via worker.current()
//!     }
//!     if let Some(build) = worker.current() {
//!         // use build.navmesh / build.bsp for queries this frame
//!     }
//!     # break;
//! }
//! ```
//!
//! v0 strategy: every submission triggers a full pipeline rebuild
//! (`polygon-extract → CDT → NavMesh → BSP`). v1 will swap in a
//! cavity-remesh strategy behind the same public API.

use std::sync::mpsc::{channel, Receiver, Sender, TryRecvError};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Instant;

use arc_swap::ArcSwapOption;

use rsnav_bsp::Bsp;
use rsnav_navmesh::{build_from_cdt, NavMesh};
use rsnav_polygon_extract::{extract, Bitfield, ExtractOptions};
use rsnav_triangle::{
    carve_holes, delaunay, form_skeleton,
    pslg::{Pslg, PslgHole, PslgSegment, PslgVertex},
    CdtMesh, DivConqOptions, SegmentInsertError, VertexSlot,
};

/// Knobs the worker uses on every rebuild.
#[derive(Clone, Debug)]
pub struct BuildOptions {
    /// Forwarded to [`rsnav_polygon_extract::extract`].
    pub extract: ExtractOptions,
    /// Marker assigned to outer-perimeter constraint segments.
    pub perimeter_marker: i32,
    /// Marker assigned to hole-perimeter constraint segments.
    pub hole_marker: i32,
}

impl Default for BuildOptions {
    fn default() -> Self {
        Self {
            extract: ExtractOptions::default(),
            perimeter_marker: 1,
            hole_marker: 2,
        }
    }
}

/// A successfully-built navmesh + its query index, with timing and a
/// generation counter the caller can use to detect "is this newer than
/// the build I last looked at?".
#[derive(Debug)]
pub struct NavBuild {
    pub navmesh: NavMesh,
    pub bsp: Bsp,
    pub build_ms: f64,
    /// Monotonically increasing per worker. The first published build
    /// has generation 1.
    pub generation: u64,
}

/// Reasons the worker (or [`build_navmesh_from_bitfield`]) can fail.
#[derive(Debug)]
pub enum BuildError {
    /// `extract` returned no regions — the bitfield has no walkable
    /// cells, or none survived `min_polygon_area`.
    NoPerimeter,
    /// `form_skeleton` rejected a constraint (only [`SegmentInsertError::SelfIntersection`]
    /// is a likely cause when input comes from `polygon-extract`).
    SegmentInsertion(SegmentInsertError),
    /// Pipeline ran but produced zero live triangles after hole carving.
    EmptyMesh,
}

impl core::fmt::Display for BuildError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::NoPerimeter => write!(f, "bitfield has no walkable regions"),
            Self::SegmentInsertion(e) => write!(f, "segment insertion failed: {e}"),
            Self::EmptyMesh => write!(f, "pipeline produced zero triangles"),
        }
    }
}

impl std::error::Error for BuildError {}

// =========================================================================
// One-shot pipeline (also used by NavWorker internally).
// =========================================================================

/// Run the full `polygon-extract → CDT → NavMesh → BSP` pipeline against
/// a single bitfield snapshot. Used by [`NavWorker`] and exposed for
/// callers that want a synchronous build.
pub fn build_navmesh_from_bitfield(
    bf: &Bitfield,
    opts: &BuildOptions,
) -> Result<NavBuild, BuildError> {
    let start = Instant::now();

    let regions = extract(bf, &opts.extract);
    if regions.is_empty() {
        return Err(BuildError::NoPerimeter);
    }

    let mut pslg = Pslg::new();
    let mut next_idx: u32 = 0;

    for region in &regions {
        // Outer ring.
        let start_idx = next_idx;
        for v in &region.outer.vertices {
            pslg.vertices.push(PslgVertex::new(*v));
            next_idx += 1;
        }
        let n = region.outer.vertices.len() as u32;
        if n >= 3 {
            for i in 0..n {
                pslg.segments.push(PslgSegment {
                    a: start_idx + i,
                    b: start_idx + (i + 1) % n,
                    marker: opts.perimeter_marker,
                });
            }
        }

        // Hole rings.
        for hole in &region.holes {
            let start_idx = next_idx;
            for v in &hole.vertices {
                pslg.vertices.push(PslgVertex::new(*v));
                next_idx += 1;
            }
            let n = hole.vertices.len() as u32;
            if n >= 3 {
                for i in 0..n {
                    pslg.segments.push(PslgSegment {
                        a: start_idx + i,
                        b: start_idx + (i + 1) % n,
                        marker: opts.hole_marker,
                    });
                }
            }
            if let Some(seed) = hole.interior_point() {
                pslg.holes.push(PslgHole { point: seed });
            }
        }
    }

    let mut cdt = CdtMesh::new();
    for v in &pslg.vertices {
        cdt.push_vertex(VertexSlot::new(v.position, 0));
    }
    delaunay(&mut cdt, DivConqOptions::default());
    form_skeleton(&mut cdt, &pslg, None).map_err(BuildError::SegmentInsertion)?;
    carve_holes(&mut cdt, &pslg, false);

    let navmesh = build_from_cdt(&cdt);
    if navmesh.triangle_count() == 0 {
        return Err(BuildError::EmptyMesh);
    }
    let bsp = Bsp::build(&navmesh);
    let build_ms = start.elapsed().as_secs_f64() * 1000.0;

    Ok(NavBuild {
        navmesh,
        bsp,
        build_ms,
        generation: 0, // filled in by the worker; 0 for direct callers
    })
}

// =========================================================================
// NavWorker
// =========================================================================

enum Cmd {
    Rebuild(Arc<Bitfield>),
    Shutdown,
}

/// A worker thread that rebuilds the navmesh in response to bitfield
/// snapshots. See module docs.
pub struct NavWorker {
    tx: Sender<Cmd>,
    shared: Arc<ArcSwapOption<NavBuild>>,
    last_error: Arc<Mutex<Option<String>>>,
    snapshot: Option<Arc<NavBuild>>,
    snapshot_gen: u64,
    handle: Option<JoinHandle<()>>,
}

impl NavWorker {
    /// Spawn the worker. The returned handle owns the thread and joins
    /// it on `shutdown()` (or Drop).
    pub fn spawn(opts: BuildOptions) -> Self {
        let shared = Arc::new(ArcSwapOption::empty());
        let last_error = Arc::new(Mutex::new(None));
        let (tx, rx) = channel::<Cmd>();
        let shared_w = shared.clone();
        let err_w = last_error.clone();
        let handle = thread::Builder::new()
            .name("rsnav-dynamic-worker".into())
            .spawn(move || run_worker(rx, shared_w, err_w, opts))
            .expect("spawning worker thread");
        Self {
            tx,
            shared,
            last_error,
            snapshot: None,
            snapshot_gen: 0,
            handle: Some(handle),
        }
    }

    /// Submit a new bitfield snapshot. Non-blocking. If a previous
    /// snapshot is still queued, the worker silently coalesces and only
    /// builds against the newest one.
    pub fn submit_snapshot(&self, bitfield: Arc<Bitfield>) {
        // send can only fail if the worker thread is gone — which means
        // the caller is already in shutdown, so dropping is fine.
        let _ = self.tx.send(Cmd::Rebuild(bitfield));
    }

    /// Call once per frame, before any system reads `current()`. If the
    /// worker has published a newer build than the one currently
    /// presented to game systems, swap it in atomically. Returns `true`
    /// if a swap happened this call.
    pub fn poll_swap(&mut self) -> bool {
        let latest = self.shared.load_full();
        if let Some(latest) = latest {
            if latest.generation > self.snapshot_gen {
                self.snapshot_gen = latest.generation;
                self.snapshot = Some(latest);
                return true;
            }
        }
        false
    }

    /// The build that was active for this frame, or `None` if the
    /// worker hasn't published its first build yet.
    pub fn current(&self) -> Option<Arc<NavBuild>> {
        self.snapshot.clone()
    }

    /// The most recent build that the worker has published (regardless
    /// of whether `poll_swap` has surfaced it yet). Useful for tests.
    pub fn latest_published(&self) -> Option<Arc<NavBuild>> {
        self.shared.load_full()
    }

    /// The last build error reported by the worker, if any. Cleared
    /// when a subsequent build succeeds.
    pub fn last_error(&self) -> Option<String> {
        self.last_error.lock().expect("worker error mutex").clone()
    }

    /// Cleanly stop the worker and join its thread.
    pub fn shutdown(mut self) {
        let _ = self.tx.send(Cmd::Shutdown);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

impl Drop for NavWorker {
    fn drop(&mut self) {
        let _ = self.tx.send(Cmd::Shutdown);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

fn run_worker(
    rx: Receiver<Cmd>,
    shared: Arc<ArcSwapOption<NavBuild>>,
    last_error: Arc<Mutex<Option<String>>>,
    opts: BuildOptions,
) {
    let mut generation: u64 = 0;
    loop {
        // Block until we get at least one snapshot.
        let cmd = match rx.recv() {
            Ok(c) => c,
            Err(_) => return, // sender dropped
        };
        let mut latest = match cmd {
            Cmd::Rebuild(bf) => bf,
            Cmd::Shutdown => return,
        };

        // Drain anything queued behind it — only build against the newest.
        loop {
            match rx.try_recv() {
                Ok(Cmd::Rebuild(bf)) => latest = bf,
                Ok(Cmd::Shutdown) => return,
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => return,
            }
        }

        generation += 1;
        match build_navmesh_from_bitfield(&latest, &opts) {
            Ok(mut build) => {
                build.generation = generation;
                shared.store(Some(Arc::new(build)));
                *last_error.lock().expect("worker error mutex") = None;
            }
            Err(e) => {
                *last_error.lock().expect("worker error mutex") = Some(format!("{e}"));
                // Keep the previous published build (if any) intact.
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a tiny walkable bitfield: 8×8 all walkable.
    fn solid_bitfield(w: u32, h: u32) -> Bitfield {
        let data = vec![true; (w as usize) * (h as usize)];
        Bitfield::new(w, h, data).expect("test bitfield dims match")
    }

    #[test]
    fn direct_build_produces_navmesh() {
        let bf = solid_bitfield(8, 8);
        let build = build_navmesh_from_bitfield(&bf, &BuildOptions::default())
            .expect("8x8 solid should build");
        assert!(build.navmesh.triangle_count() > 0);
        assert!(build.build_ms >= 0.0);
        assert_eq!(build.generation, 0); // direct callers see 0
    }

    #[test]
    fn worker_publishes_a_build_after_submit() {
        let mut worker = NavWorker::spawn(BuildOptions::default());
        let bf = Arc::new(solid_bitfield(8, 8));
        worker.submit_snapshot(bf);

        // Spin briefly waiting for the worker. 8x8 builds in well under 1s.
        let deadline = Instant::now() + std::time::Duration::from_secs(2);
        loop {
            if worker.poll_swap() {
                break;
            }
            if Instant::now() > deadline {
                panic!("worker did not publish within 2s");
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        let build = worker.current().expect("a build should be current after swap");
        assert!(build.navmesh.triangle_count() > 0);
        assert_eq!(build.generation, 1);
    }

    #[test]
    fn worker_coalesces_rapid_submits() {
        let mut worker = NavWorker::spawn(BuildOptions::default());
        // Submit ten snapshots back-to-back. The worker may pick any of
        // them up — what we require is that the number of *published*
        // builds is strictly less than the number submitted, i.e. some
        // were coalesced.
        for _ in 0..10 {
            worker.submit_snapshot(Arc::new(solid_bitfield(8, 8)));
        }

        // Wait for the dust to settle.
        std::thread::sleep(std::time::Duration::from_millis(200));
        worker.poll_swap();
        let build = worker.current().expect("at least one build");
        assert!(
            build.generation >= 1 && build.generation <= 10,
            "generation {} not in (1..=10)",
            build.generation,
        );
        assert!(
            build.generation < 10,
            "all 10 submissions ran without coalescing (got gen {})",
            build.generation,
        );
    }

    #[test]
    fn worker_reports_no_perimeter_for_empty_bitfield() {
        let mut worker = NavWorker::spawn(BuildOptions::default());
        // All-false bitfield → no walkable regions.
        let bf = Bitfield::empty(8, 8);
        worker.submit_snapshot(Arc::new(bf));
        std::thread::sleep(std::time::Duration::from_millis(100));
        worker.poll_swap();
        assert!(worker.current().is_none(), "should not publish a build");
        let err = worker.last_error().expect("error should be set");
        assert!(err.contains("no walkable"), "got: {err}");
    }

    #[test]
    fn shutdown_joins_cleanly() {
        let worker = NavWorker::spawn(BuildOptions::default());
        worker.submit_snapshot(Arc::new(solid_bitfield(8, 8)));
        worker.shutdown(); // shouldn't hang
    }
}
