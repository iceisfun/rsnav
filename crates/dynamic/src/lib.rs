//! Dynamic navmesh updates for game loops.
//!
//! A [`NavWorker`] owns a background thread that turns [`Bitfield`]
//! snapshots into a [`NavBuild`] (a [`NavMesh`] + [`Bsp`]). The worker
//! coalesces submissions — if you submit faster than it can build, only
//! the most recent snapshot is processed. The main thread reads the
//! latest published build at frame start via [`NavWorker::poll_swap`].
//!
//! ## Listener panic isolation
//!
//! [`NavListener::on_event`] callbacks are invoked on the worker thread.
//! Any panic inside a listener is caught and swallowed — a buggy listener
//! will not kill the worker thread. Use [`NavWorker::is_running`] to
//! check whether the worker thread is still alive (it becomes `false` only
//! on clean shutdown or if a panic in the build pipeline itself kills it).
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

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
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
// Telemetry: typed events + counters.
// =========================================================================

/// Typed observability events emitted by [`NavWorker`].
///
/// Events are dispatched synchronously on the worker thread between
/// builds, so listener implementations should be cheap. If you need to
/// do heavyweight work (network I/O, file writes, expensive formatting)
/// push the event into a channel and consume it from your own thread.
#[derive(Copy, Clone, Debug)]
pub enum NavEvent<'a> {
    /// A new rebuild is about to start.
    BuildStarted { generation: u64 },
    /// A rebuild finished successfully and was published.
    BuildCompleted {
        generation: u64,
        build_ms: f64,
        triangles: usize,
        regions: u32,
    },
    /// A rebuild failed; the previous published build (if any) is kept.
    BuildFailed {
        generation: u64,
        error: &'a BuildError,
    },
}

/// Callback interface for worker telemetry. Implement this on any
/// type, or pass a closure — there's a blanket impl below.
///
/// `Send + Sync` because the listener is stored as an `Arc<dyn NavListener>`
/// and invoked from the worker thread; `'static` because the worker
/// thread outlives the calling scope.
pub trait NavListener: Send + Sync + 'static {
    fn on_event(&self, event: &NavEvent<'_>);
}

/// Blanket impl so any `Fn(&NavEvent<'_>)` closure can be used directly
/// as a listener:
///
/// ```no_run
/// use std::sync::Arc;
/// use rsnav_dynamic::{NavWorker, NavListener, NavEvent, BuildOptions};
///
/// let listener: Arc<dyn NavListener> = Arc::new(|ev: &NavEvent<'_>| {
///     eprintln!("nav event: {:?}", ev);
/// });
/// let _worker = NavWorker::spawn_with_listener(BuildOptions::default(), listener);
/// ```
impl<F> NavListener for F
where
    F: Fn(&NavEvent<'_>) + Send + Sync + 'static,
{
    fn on_event(&self, event: &NavEvent<'_>) {
        (self)(event)
    }
}

/// A snapshot of running worker counters, returned by
/// [`NavWorker::stats`]. Plain `Copy` struct so callers can read it
/// any number of times without worrying about consistency.
///
/// All counters are monotonic from worker start. The caller derives
/// rates / averages itself (e.g. `total_build_ms / builds_completed`).
#[derive(Copy, Clone, Debug, Default)]
pub struct NavStats {
    /// Total snapshots handed to [`NavWorker::submit_snapshot`].
    pub snapshots_submitted: u64,
    /// Snapshots silently dropped because a newer snapshot arrived
    /// before the worker started building this one.
    pub snapshots_coalesced: u64,
    /// Builds that completed successfully.
    pub builds_completed: u64,
    /// Builds that failed (extract / segment-insertion / empty mesh).
    pub builds_failed: u64,
    /// Generation of the most recent completed build (0 if none).
    pub last_completed_generation: u64,
    /// Most recent successful build time, in milliseconds.
    pub last_build_ms: f64,
    /// Highest successful build time observed this session, in
    /// milliseconds.
    pub max_build_ms: f64,
    /// Sum of all successful build times, in milliseconds. Divide by
    /// `builds_completed` for the running average.
    pub total_build_ms: f64,
}

#[derive(Debug, Default)]
struct StatsInner {
    snapshots_submitted: AtomicU64,
    snapshots_coalesced: AtomicU64,
    builds_completed: AtomicU64,
    builds_failed: AtomicU64,
    last_completed_generation: AtomicU64,
    timing: Mutex<TimingStats>,
    /// `true` while the worker thread is running; flipped to `false` by
    /// `AliveGuard` on every exit path including panics.
    alive: AtomicBool,
}

#[derive(Copy, Clone, Debug, Default)]
struct TimingStats {
    last_build_ms: f64,
    max_build_ms: f64,
    total_build_ms: f64,
}

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
    stats: Arc<StatsInner>,
    snapshot: Option<Arc<NavBuild>>,
    snapshot_gen: u64,
    handle: Option<JoinHandle<()>>,
}

impl NavWorker {
    /// Spawn the worker without a telemetry listener. Equivalent to
    /// `spawn_with_listener(opts, None)` but lets callers avoid
    /// constructing an `Arc<dyn NavListener>` they don't need.
    pub fn spawn(opts: BuildOptions) -> Self {
        Self::spawn_inner(opts, None)
    }

    /// Spawn the worker with a telemetry listener. The listener
    /// receives [`NavEvent`]s synchronously on the worker thread; keep
    /// handlers cheap (push to a channel for any heavy work).
    ///
    /// Pass either a struct that implements [`NavListener`] wrapped in
    /// `Arc`, or `Arc::new(|ev| { ... })` for a closure.
    pub fn spawn_with_listener(
        opts: BuildOptions,
        listener: Arc<dyn NavListener>,
    ) -> Self {
        Self::spawn_inner(opts, Some(listener))
    }

    fn spawn_inner(opts: BuildOptions, listener: Option<Arc<dyn NavListener>>) -> Self {
        let shared = Arc::new(ArcSwapOption::empty());
        let last_error = Arc::new(Mutex::new(None));
        let stats = Arc::new(StatsInner::default());
        stats.alive.store(true, Ordering::Relaxed);
        let (tx, rx) = channel::<Cmd>();
        let shared_w = shared.clone();
        let err_w = last_error.clone();
        let stats_w = stats.clone();
        let handle = thread::Builder::new()
            .name("rsnav-dynamic-worker".into())
            .spawn(move || run_worker(rx, shared_w, err_w, stats_w, listener, opts))
            .expect("spawning worker thread");
        Self {
            tx,
            shared,
            last_error,
            stats,
            snapshot: None,
            snapshot_gen: 0,
            handle: Some(handle),
        }
    }

    /// Submit a new bitfield snapshot. Non-blocking. If a previous
    /// snapshot is still queued, the worker silently coalesces and only
    /// builds against the newest one.
    pub fn submit_snapshot(&self, bitfield: Arc<Bitfield>) {
        self.stats
            .snapshots_submitted
            .fetch_add(1, Ordering::Relaxed);
        // A send failure means the worker thread is no longer running —
        // either a clean shutdown or the thread died. Callers who need to
        // detect that can check `is_running()`.
        let _ = self.tx.send(Cmd::Rebuild(bitfield));
    }

    /// `true` while the background worker thread is alive. Becomes
    /// `false` if the worker thread has exited — normally only at
    /// shutdown, but also if a panic in the build pipeline killed it.
    /// A `false` here means submitted snapshots will no longer build.
    pub fn is_running(&self) -> bool {
        self.stats.alive.load(Ordering::Relaxed)
    }

    /// A snapshot of the worker's running counters. Cheap (takes one
    /// short mutex lock for the timing fields). Safe to call every
    /// frame.
    pub fn stats(&self) -> NavStats {
        let timing = *self.stats.timing.lock().expect("stats timing mutex");
        NavStats {
            snapshots_submitted: self.stats.snapshots_submitted.load(Ordering::Relaxed),
            snapshots_coalesced: self.stats.snapshots_coalesced.load(Ordering::Relaxed),
            builds_completed: self.stats.builds_completed.load(Ordering::Relaxed),
            builds_failed: self.stats.builds_failed.load(Ordering::Relaxed),
            last_completed_generation: self
                .stats
                .last_completed_generation
                .load(Ordering::Relaxed),
            last_build_ms: timing.last_build_ms,
            max_build_ms: timing.max_build_ms,
            total_build_ms: timing.total_build_ms,
        }
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
    stats: Arc<StatsInner>,
    listener: Option<Arc<dyn NavListener>>,
    opts: BuildOptions,
) {
    // RAII guard: flip `alive` to false on every exit path, including
    // panics that unwind out of the build pipeline.
    struct AliveGuard(Arc<StatsInner>);
    impl Drop for AliveGuard {
        fn drop(&mut self) {
            self.0.alive.store(false, Ordering::Relaxed);
        }
    }
    let _alive = AliveGuard(stats.clone());

    let dispatch = |event: &NavEvent<'_>| {
        if let Some(l) = listener.as_ref() {
            // Catch any panic from the listener so a buggy callback
            // cannot unwind into and kill the worker thread.
            let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                l.on_event(event);
            }));
        }
    };

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
        let mut coalesced: u64 = 0;
        loop {
            match rx.try_recv() {
                Ok(Cmd::Rebuild(bf)) => {
                    coalesced += 1;
                    latest = bf;
                }
                Ok(Cmd::Shutdown) => return,
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => return,
            }
        }
        if coalesced > 0 {
            stats
                .snapshots_coalesced
                .fetch_add(coalesced, Ordering::Relaxed);
        }

        generation += 1;
        dispatch(&NavEvent::BuildStarted { generation });

        match build_navmesh_from_bitfield(&latest, &opts) {
            Ok(mut build) => {
                build.generation = generation;
                // Pull the fields we want for the completion event
                // before moving `build` into the Arc.
                let build_ms = build.build_ms;
                let triangles = build.navmesh.triangle_count();
                let regions = build.navmesh.region_count;
                shared.store(Some(Arc::new(build)));
                *last_error.lock().expect("worker error mutex") = None;

                stats.builds_completed.fetch_add(1, Ordering::Relaxed);
                stats
                    .last_completed_generation
                    .store(generation, Ordering::Relaxed);
                {
                    let mut t = stats.timing.lock().expect("stats timing mutex");
                    t.last_build_ms = build_ms;
                    if build_ms > t.max_build_ms {
                        t.max_build_ms = build_ms;
                    }
                    t.total_build_ms += build_ms;
                }

                dispatch(&NavEvent::BuildCompleted {
                    generation,
                    build_ms,
                    triangles,
                    regions,
                });
            }
            Err(e) => {
                *last_error.lock().expect("worker error mutex") = Some(format!("{e}"));
                stats.builds_failed.fetch_add(1, Ordering::Relaxed);
                dispatch(&NavEvent::BuildFailed {
                    generation,
                    error: &e,
                });
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

    // ---- telemetry --------------------------------------------------

    /// Collector listener that records every event it receives.
    #[derive(Default)]
    struct Collector {
        events: Mutex<Vec<String>>,
    }
    impl Collector {
        fn snapshot(&self) -> Vec<String> {
            self.events.lock().unwrap().clone()
        }
    }
    impl NavListener for Collector {
        fn on_event(&self, ev: &NavEvent<'_>) {
            let s = match ev {
                NavEvent::BuildStarted { generation } => format!("started:{generation}"),
                NavEvent::BuildCompleted {
                    generation,
                    triangles,
                    regions,
                    ..
                } => format!("completed:{generation}:tris={triangles}:regs={regions}"),
                NavEvent::BuildFailed { generation, error } => {
                    format!("failed:{generation}:{error}")
                }
            };
            self.events.lock().unwrap().push(s);
        }
    }

    fn wait_until<F: FnMut() -> bool>(mut cond: F, label: &str) {
        let deadline = Instant::now() + std::time::Duration::from_secs(2);
        while !cond() {
            if Instant::now() > deadline {
                panic!("timeout waiting for: {label}");
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
    }

    #[test]
    fn listener_receives_started_and_completed_for_a_build() {
        let listener = Arc::new(Collector::default());
        let worker = NavWorker::spawn_with_listener(
            BuildOptions::default(),
            listener.clone() as Arc<dyn NavListener>,
        );
        worker.submit_snapshot(Arc::new(solid_bitfield(8, 8)));

        wait_until(
            || {
                listener
                    .events
                    .lock()
                    .unwrap()
                    .iter()
                    .any(|e| e.starts_with("completed:"))
            },
            "completion event",
        );

        let evs = listener.snapshot();
        assert!(
            evs.iter().any(|e| e == "started:1"),
            "expected started:1, got {evs:?}"
        );
        assert!(
            evs.iter()
                .any(|e| e.starts_with("completed:1:tris=") && e.contains(":regs=1")),
            "expected completed:1 with tris/regs, got {evs:?}"
        );
    }

    #[test]
    fn listener_receives_failed_for_bad_input() {
        let listener = Arc::new(Collector::default());
        let worker = NavWorker::spawn_with_listener(
            BuildOptions::default(),
            listener.clone() as Arc<dyn NavListener>,
        );
        worker.submit_snapshot(Arc::new(Bitfield::empty(8, 8)));

        wait_until(
            || {
                listener
                    .events
                    .lock()
                    .unwrap()
                    .iter()
                    .any(|e| e.starts_with("failed:"))
            },
            "failure event",
        );

        let evs = listener.snapshot();
        assert!(
            evs.iter()
                .any(|e| e.starts_with("failed:1:") && e.contains("no walkable")),
            "expected failed:1 with no-walkable message, got {evs:?}"
        );
    }

    #[test]
    fn closure_can_be_used_as_listener() {
        // Demonstrates the blanket Fn impl + Arc-coercion ergonomics.
        let n = Arc::new(AtomicU64::new(0));
        let n_l = n.clone();
        let listener: Arc<dyn NavListener> = Arc::new(move |ev: &NavEvent<'_>| {
            if matches!(ev, NavEvent::BuildCompleted { .. }) {
                n_l.fetch_add(1, Ordering::Relaxed);
            }
        });
        let worker = NavWorker::spawn_with_listener(BuildOptions::default(), listener);
        worker.submit_snapshot(Arc::new(solid_bitfield(8, 8)));
        wait_until(|| n.load(Ordering::Relaxed) >= 1, "one completion");
    }

    #[test]
    fn stats_track_completions_and_max_build_ms() {
        let worker = NavWorker::spawn(BuildOptions::default());
        worker.submit_snapshot(Arc::new(solid_bitfield(8, 8)));
        wait_until(
            || worker.stats().builds_completed >= 1,
            "first build complete",
        );
        let s = worker.stats();
        assert!(s.snapshots_submitted >= 1);
        assert_eq!(s.builds_completed, 1);
        assert_eq!(s.builds_failed, 0);
        assert_eq!(s.last_completed_generation, 1);
        assert!(s.last_build_ms >= 0.0);
        assert!(s.max_build_ms >= s.last_build_ms);
        assert!(s.total_build_ms >= s.last_build_ms);
    }

    #[test]
    fn stats_track_failures() {
        let worker = NavWorker::spawn(BuildOptions::default());
        worker.submit_snapshot(Arc::new(Bitfield::empty(8, 8)));
        wait_until(|| worker.stats().builds_failed >= 1, "first build failure");
        let s = worker.stats();
        assert_eq!(s.builds_failed, 1);
        assert_eq!(s.builds_completed, 0);
        assert_eq!(s.last_completed_generation, 0);
    }

    #[test]
    fn stats_track_coalescing() {
        let worker = NavWorker::spawn(BuildOptions::default());
        for _ in 0..10 {
            worker.submit_snapshot(Arc::new(solid_bitfield(8, 8)));
        }
        // Let the worker chew through whatever it can.
        std::thread::sleep(std::time::Duration::from_millis(200));
        let s = worker.stats();
        assert_eq!(
            s.snapshots_submitted, 10,
            "all 10 submissions should count"
        );
        // builds_completed + snapshots_coalesced should account for
        // every submission. (Plus possibly 1 still queued; account for
        // it by allowing >= 9.)
        let accounted = s.builds_completed + s.snapshots_coalesced;
        assert!(
            accounted >= 9,
            "accounted={} (completed={} coalesced={}) should cover at least 9 of 10",
            accounted,
            s.builds_completed,
            s.snapshots_coalesced,
        );
        assert!(
            s.snapshots_coalesced >= 1,
            "expected at least one coalesced drop, got {}",
            s.snapshots_coalesced,
        );
    }

    // ---- liveness & panic isolation ------------------------------------

    #[test]
    fn is_running_true_for_live_worker() {
        let worker = NavWorker::spawn(BuildOptions::default());
        assert!(worker.is_running(), "worker should be alive after spawn");
    }

    #[test]
    fn panicking_listener_does_not_kill_worker() {
        // A listener that always panics; the worker must survive it.
        // NOTE: the default panic hook will print to stderr for each
        // caught panic — that is expected and the test still passes.
        struct BoomListener;
        impl NavListener for BoomListener {
            fn on_event(&self, _ev: &NavEvent<'_>) {
                panic!("boom");
            }
        }

        let worker = NavWorker::spawn_with_listener(
            BuildOptions::default(),
            Arc::new(BoomListener) as Arc<dyn NavListener>,
        );
        worker.submit_snapshot(Arc::new(solid_bitfield(8, 8)));

        // The worker must still complete the build despite the panicking listener.
        wait_until(
            || worker.stats().builds_completed >= 1,
            "build completes despite panicking listener",
        );
        assert!(
            worker.is_running(),
            "worker should still be alive after listener panic"
        );
    }
}
