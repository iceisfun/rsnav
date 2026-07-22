# When the world changes: NavWorker and the game loop

For RTS, base-builders, destructible terrain, construction — anything where walls
appear and disappear while the game is running.

Prerequisites: a working build from a bitfield ([05-building-navmeshes.md](05-building-navmeshes.md))
and a working query ([07-paths-and-queries.md](07-paths-and-queries.md)).
Runnable reference for everything on this page:
[`crates/dynamic/examples/live_worker.rs`](../crates/dynamic/examples/live_worker.rs)
— `cargo run -p rsnav-dynamic --example live_worker`.

---

## The ownership model

This is the whole design, and everything else follows from it.

**The game owns the ground-truth grid.** You edit it — paint a wall, clear a forest
cell, drop a building footprint — exactly as you would any other game state, on the
main thread, with no synchronisation. `Bitfield::set(col, row, v)` exists, but only
before the value is handed over: once submitted the snapshot lives behind a shared
`Arc` and is no longer mutable. Both reference integrations therefore keep the cell
data in their own storage — a `Vec<bool>` in `live_worker.rs`, a `Vec<Cell>` in
`rtsim` — and construct a fresh `Bitfield` per submission.

**The worker owns the derived navmesh.** You never mutate a published build. You
hand the worker an immutable `Arc<Bitfield>` snapshot and it hands you back an
immutable `Arc<NavBuild>`. `NavBuild` is not `Clone`; the `Arc` is the only way it
moves, and the only thing it is shared through.

```
game thread                          worker thread
-----------                          -------------
edit Bitfield  --submit_snapshot-->  build_navmesh_from_bitfield
                                       |
poll_swap()  <-- ArcSwapOption<NavBuild> --  publish NavBuild (gen N)
current()  -> Arc<NavBuild>
```

The consequence people miss: a rebuild is a *replacement*, not an update. There is
no incremental patch, no in-place edit of the live mesh. Every submission runs the
full pipeline (`extract → per-region CDT → NavMesh merge → Bsp`). The module header
at [`crates/dynamic/src/lib.rs`](../crates/dynamic/src/lib.rs) calls this the v0
strategy and reserves a cavity-remesh strategy behind the same public API for later.

## Spawning

```rust
pub fn spawn(opts: BuildOptions) -> Self
pub fn spawn_with_listener(opts: BuildOptions, listener: Arc<dyn NavListener>) -> Self
```

`BuildOptions` is captured **once, at spawn**, and is immutable for the worker's
lifetime. There is no `set_options`. Changing the build configuration — a different
`inset`, a different `extract.min_area`, a different thread count — means dropping the
worker and spawning a new one, then re-submitting a snapshot to get a build under
the new options. See [05-building-navmeshes.md](05-building-navmeshes.md) for what
those options do.

`spawn_with_listener` takes a non-`Option` `Arc<dyn NavListener>`. A blanket impl
covers any `Fn(&NavEvent<'_>) + Send + Sync + 'static`, so a closure works directly,
but the argument type must be annotated for the coercion to land:

```rust
let listener: Arc<dyn NavListener> = Arc::new(|ev: &NavEvent<'_>| { /* ... */ });
let worker = NavWorker::spawn_with_listener(BuildOptions::default(), listener);
```

## The frame protocol

```rust
pub fn poll_swap(&mut self) -> bool     // once per frame, first
pub fn current(&self) -> Option<Arc<NavBuild>>
pub fn latest_published(&self) -> Option<Arc<NavBuild>>
```

Call `poll_swap` **exactly once per frame, before any system reads `current()`**. It
looks at what the worker has published and, if the generation is newer than the one
currently presented, pins it. It returns `true` when a swap happened, which is your
signal to run the invalidation checklist below.

`NavBuild::generation` counts mesh rebuilds. It is unrelated to `DoorSet::generation()`,
which counts door-state changes against a mesh that did not change
([09](09-doors-and-navworld.md)). Running both means tracking two independent counters;
neither implies the other.

`current()` returns the build pinned for *this* frame. It does not change underneath
you mid-frame no matter how many builds the worker completes while your frame is in
flight. That stability is the entire point, and it is why the two methods have
different receivers: `poll_swap` takes `&mut self` and `current()` takes `&self`. The
borrow checker therefore enforces that the swap cannot happen while any system is
holding a shared reference to the worker and reading. Pathfinding, rendering, AI and
debug overlays all see the same mesh for the same frame, for free.

`current()` returns `None` until the first `poll_swap` that finds a published build —
it reads the pinned snapshot, not the worker's latest. `latest_published()` bypasses
the pin and reads whatever the worker last stored. Its doc comment says "useful for
tests"; that is the honest scope. Using it in game code reintroduces exactly the
mid-frame instability `poll_swap` exists to remove.

```rust
// once, at the top of the frame
if worker.poll_swap() {
    // new mesh this frame — see "What a swap invalidates"
}
// anywhere, any number of systems, for the rest of the frame
if let Some(build) = worker.current() {
    // build.navmesh, build.bsp
}
```

## Coalescing

`submit_snapshot(&self, bitfield: Arc<Bitfield>)` is non-blocking and takes `&self`,
so any system holding a shared reference can submit.

If snapshots arrive faster than the worker builds, the worker drains its queue and
builds only against the **newest**, counting the rest as coalesced. A player
dragging out a wall at 144 Hz over a map that takes 30 ms to rebuild does not
produce a growing backlog and does not make the worker fall progressively further
behind; it produces one build per ~30 ms against the most recent grid state. The
drops are counted in `NavStats::snapshots_coalesced`, so `snapshots_submitted`
minus `snapshots_coalesced` minus `builds_completed` minus `builds_failed` tells you
what is still in flight.

This means you should submit freely rather than trying to rate-limit submissions
yourself. The worker's coalescing is better informed than your timer is.

## Telemetry

Two independent surfaces.

**`NavStats`**, via `worker.stats()`. A `Copy` struct of counters, all monotonic
from worker start: `snapshots_submitted`, `snapshots_coalesced`, `builds_completed`,
`builds_failed`, `last_completed_generation`, `last_build_ms`, `max_build_ms`,
`total_build_ms`. Deliberately raw — the caller derives rates and averages, e.g.
`total_build_ms / builds_completed`, which is what `live_worker.rs` prints at the
end. `stats()` takes one short mutex lock for the three timing fields and is safe to
call every frame.

**`NavEvent` / `NavListener`**, typed events dispatched *synchronously on the worker
thread* between builds: `BuildStarted { generation }`, `BuildCompleted { generation,
build_ms, triangles, regions }`, `BuildFailed { generation, error }`. Because
dispatch is synchronous and on the worker thread, a slow handler directly delays the
next build. Keep handlers cheap; push anything heavyweight — file writes, network
I/O, expensive formatting — into a channel and consume it from your own thread.

`NavEvent::BuildFailed` carries `error: &'a BuildError` and is **the only place the
typed error is observable**. `last_error()` returns `Option<String>` — the typed
`BuildError` is formatted and discarded. If you need to branch on the variant (retry
on `EmptyMesh`, hard-fail on `InvalidInset`), you must attach a listener.

## Failure, and the four places it is silent

The worker is built to survive bad input rather than to be loud about it, which
makes the silence a class of bug on its own.

- **A panicking build does not kill the worker.** `build_navmesh_from_bitfield` runs
  under `catch_unwind`; a panic becomes `BuildError::Panicked(String)`. That variant
  is *only* reachable through the worker — a direct caller of
  `build_navmesh_from_bitfield` gets the panic itself. The previously published build
  stays live and serving, so the game keeps pathing on a stale-but-valid mesh.
- **A failed build is not a swap.** On `Err` the worker records the error, bumps
  `builds_failed`, dispatches `BuildFailed`, and leaves the published build alone.
  `poll_swap` returns `false`. Nothing in the query path notices; you are silently
  navigating an out-of-date world until the next successful build.
- **`submit_snapshot` ignores send failure.** If the worker thread is gone, the send
  is discarded with `let _ = ...`. Snapshots vanish with no error, no counter, and no
  event — `snapshots_submitted` still increments. Only `is_running()` reveals it.
  Poll it if a stalled navmesh would be a correctness problem rather than a glitch.
- **A panicking listener is caught and swallowed with no diagnostic at all.**
  Dispatch wraps every callback in `catch_unwind` and drops the payload. A listener
  that panics on its first event produces total silence, not an error — which reads
  exactly like "the worker never fired an event". If your telemetry goes quiet,
  suspect the listener before the worker.

`Drop for NavWorker` already sends `Shutdown` and joins the thread, so calling
`shutdown()` explicitly is optional. It exists for when you want the join to happen
at a point you choose rather than wherever the value falls out of scope. Both ignore
join errors.

## What a swap invalidates

`poll_swap` returning `true` means every derived structure you are holding now
refers to a mesh that no longer exists. This is the actual bug source in dynamic
worlds, so treat it as a checklist:

| Thing you hold | After a swap |
|---|---|
| `TriangleId`, `VertexId` | **Dead.** IDs are per-instance. `NavMesh::vertex`/`triangle` *panic* on an out-of-range id, and an id that happens to be in range now points at unrelated geometry. |
| `Bsp` | Replaced. The new one arrives inside `NavBuild`; do not keep the old one, and do not build your own from the new mesh. |
| `WallInfo` | Rebuild from the new `NavMesh`. |
| `WallClearance` | Rebuild. It caches vertex coordinates. |
| `DoorSet` | **Rebuild from scratch.** Resolved edges are canonical vertex-pair keys into the old mesh, and there is no in-place re-resolve method. Re-`add` each door against the new mesh from its `Door::line` (the authoring segment, kept public for exactly this) and `Door::state`. See [09-doors-and-navworld.md](09-doors-and-navworld.md). |
| `NavWorld<M>` | Not mutable in place — the mesh is immutable once emplaced. Construct a new one. |
| In-flight `PathResult` | `points` are world coordinates and still *mean* something; the route may no longer be walkable. Validate with `path_clear` against the new mesh and walls, and replan on `false`. |
| `Crowd` | Hand it the new build with `Crowd::set_nav(Arc<NavBuild>)`, which does the per-agent revalidation sweep for you. See [11-crowds.md](11-crowds.md). |
| Your own world coordinates | **Survive.** They are the only thing that does. |

The rule that falls out of this: store positions, never handles. A goal is a
`Vertex`, not a `TriangleId`. A patrol route is a list of `Vertex`, re-located each
time it is used. Anything you cache that was derived from the mesh needs an explicit
invalidation path keyed on `poll_swap` returning `true`.

## Rebuild, or door?

The decision this page exists to set up.

A **`DoorSet` entry** is right when the barrier is a pure open/shut cut across an
already-existing portal — a gate, a drawbridge, a hatch, an unlockable shortcut. The
geometry does not change, so the mesh, the triangle IDs and the `Bsp` are never
rebuilt. Only the `WallInfo` is, and paths, IDs and caches all survive. That is
[09-doors-and-navworld.md](09-doors-and-navworld.md).

A **rebuild** is right when the walkable geometry itself changes — a building placed
or destroyed, terrain deformed, a wall painted, a bridge dropped into a gap where
there was no floor at all. There is no way to express new geometry as a door.

The tell is whether the walkable area changes. If the floor is the same and only its
passability changed, use a door. If the floor changed, submit a snapshot.

Note that a door cannot be *created* by a rebuild either: after a rebuild the
`DoorSet` must be reconstructed, so a world with both needs the door authoring step
to be re-runnable from world coordinates, not from stored edge keys.

## See it run

- `cargo run -p rsnav-dynamic --example live_worker` — the headless walkthrough:
  spawn with a printing listener, place a 4×4 building, block for the rebuild,
  demolish it, rebuild again, then print the counters. Its header states the same
  ownership pattern as this page.
- `cargo run --release -p rsnav-rtsim` — the interactive testbed. A 128×128 bitfield
  as ground truth, mouse tools that paint and clear walls, a `NavWorker` keeping up,
  and agents pathing through the churn. This is the page's content with a picture.
- `cargo run --release -p rsnav-door-demo` — the contrast: doors carved out of the
  *bitfield* and rebuilt, rather than expressed as a `DoorSet`. A different and
  equally valid design, and worth running next to the above to see what each costs.

Run the interactive demos in `--release`. Build times in a debug build are not
representative of anything.

---

Related: [05-building-navmeshes.md](05-building-navmeshes.md) for `BuildOptions` and
every `BuildError` variant · [09-doors-and-navworld.md](09-doors-and-navworld.md)
for the no-rebuild alternative · [11-crowds.md](11-crowds.md) for `Crowd::set_nav` ·
[15-performance-and-determinism.md](15-performance-and-determinism.md) for what a
rebuild costs and for the thread-count story · [12-large-worlds.md](12-large-worlds.md)
if your world is tiled — a `TiledWorld` is not rebuilt this way.
