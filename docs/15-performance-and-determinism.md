# Threads, determinism, and what things cost

Reference page. Read it if you are shipping at scale, running rsnav in CI, chasing a
build that got slower, or need reproducible output for caching, replay or lockstep.

Units and conventions are owned by [04-units-and-conventions.md](04-units-and-conventions.md).
The clearance *decision* (inset vs erosion vs query-time) is owned by
[06-clearance.md](06-clearance.md); this page only reports what each choice costs.

---

## 1. Determinism

Navmesh output is **byte-identical across thread counts**. This is a design commitment,
not a tuning outcome: the same bitfield and the same `BuildOptions` (modulo `threads`)
produce the same `NavMesh::to_bytes()` on 1 thread and on 32.

### The gate

[`crates/dynamic/examples/par_bench.rs`](../crates/dynamic/examples/par_bench.rs) is the
artifact that substantiates the claim. It builds every `testdata/*.pbm` at
`threads = 1, 2, 4, 8, 16, 32, 0(auto)`, best-of-3 for timing, serializes each result and
compares it against the first run's bytes, printing `ok` or `DIVERGED` per file
(par_bench.rs:89-109).

```text
cargo run --release -p rsnav-dynamic --example par_bench -- testdata
cargo run --release -p rsnav-dynamic --example par_bench -- --inset 0.128 testdata
```

`--inset <r>` runs the same byte-identity check over the offset/planarize/winding path,
so both build front-ends are covered.

### Why it holds structurally

Four independent properties, each in the source rather than in a test:

- **`par_map_indexed` fixes output order by index.** Workers pull indices from a shared
  `AtomicUsize`, so uneven per-item cost balances, but results are scattered back by index
  after the join. Which worker ran what is unobservable
  ([`crates/common/src/par.rs`](../crates/common/src/par.rs), module header lines 3-9).
- **`par_bands_mut` gives each band exactly one writer.** `f` sees only its band plus
  shared immutable state. No reduction, no accumulation across bands, no reassociation —
  so the bytes written do not depend on `threads` (par.rs:99-107).
- **Region order is extraction order.** `extract` emits regions in border-edge discovery
  order (row-major over cells), and `build_navmesh_from_bitfield` merges the per-region
  meshes with `NavMesh::append` in *extraction* order regardless of completion order.
  Regions are scheduled largest-first purely for makespan and scattered back before the
  merge (dynamic/src/lib.rs:467-495).
- **Erosion never touches a float distance.** `Bitfield::eroded` carries squared distances
  as exact integers — "no floating point ever enters a distance" — so the field is
  bit-reproducible across platforms as well as thread counts
  (polygon-extract/src/lib.rs:727-728). The one f64 operation in the whole transform is the
  parabola intersection inside the lower envelope, and a column is processed start-to-finish
  by a single worker, so even that operation sequence is thread-count invariant
  (polygon-extract/src/lib.rs:1135-1138).

The invariance properties of the two primitives are pinned by unit tests —
`preserves_input_order`, `serial_and_parallel_agree`, `bands_cover_every_element_exactly_once`
(deliberately 1000 elements at `band_len` 64, so the last band is short), and
`bands_are_thread_count_invariant` — at
[`crates/common/src/par.rs:169-274`](../crates/common/src/par.rs). That is `#[cfg(test)]`
code, not runnable examples; read it as the specification, not as usage.

### What determinism does not promise

- It is a claim about **ordering**, not about floating-point arithmetic. Nothing here
  makes float addition associative; it removes the need for it.
- It cannot make a non-deterministic closure deterministic. `par_map_indexed` fixes where
  a result lands, not what `f` returns.
- **`Crowd` is not covered.** Nothing in `rsnav-crowd` uses the `par` primitives (it is
  single-threaded throughout), and there is no test asserting tick-to-tick reproducibility.
  A reading of the source suggests it is deterministic, but that is inference. Do not build
  lockstep networking on it without writing your own gate first.
- **`resolve_door_edges` returns a `Vec` collected from a `HashSet`**, so its element
  order varies run to run (doors.rs:233). This is invisible to door semantics — the set is
  what matters and the consumer re-collects into a `HashSet` — but do not hash or serialize
  that vector.

---

## 2. The threads convention

Every options struct in the workspace uses the same encoding:

| value | meaning |
|---|---|
| `0` | one worker per available core (`std::thread::available_parallelism`) |
| `1` | fully serial — **no thread is spawned anywhere** |
| `n` | up to `n`, subject to each phase's own cap |

Two details that bite:

- **`extract.threads` inherits.** `build_navmesh_from_bitfield` copies `opts.threads` into
  `extract_opts.threads` when the latter is `0` (dynamic/src/lib.rs:437-441), which is what
  makes `BuildOptions { threads: 1, .. }` serial end-to-end rather than serial-except-extract.
  An explicitly set `extract.threads` still wins.
- **`threads` counts the caller.** Both `par` helpers spawn `1..threads` workers and then
  run the same work loop on the calling thread (par.rs:66-71, par.rs:157-160). `threads = 1`
  spawns nothing.

### The `par.rs` trap

`0` means opposite things depending on where you pass it. This matters only if you call the
primitives directly:

```rust
// WRONG: threads.min(items.len()) == 0 trips the `<= 1` branch — fully serial.
let out = par_map_indexed(&items, opts.threads, |i, it| work(i, it));

// RIGHT: resolve first, cap second.
let threads = resolve_threads(opts.threads).min(items.len()).min(MY_CAP);
let out = par_map_indexed(&items, threads, |i, it| work(i, it));
```

`resolve_threads(0)` is available parallelism; any other value is returned **literally and
unclamped**, and callers cap it themselves (par.rs:21-31). Every real call site in the
workspace routes through it first (polygon-extract/src/lib.rs:146 and :1016,
dynamic/src/lib.rs:463).

Second trap, for `par_bands_mut` only: the final band is **short** whenever `band_len` does
not divide `out.len()`, exactly as `chunks_mut` yields it. `f` must use `band.len()`, never
`band_len`. Recover the absolute offset as `band_index * band_len`. `band_len == 0` is a
documented degenerate that treats the whole buffer as one band rather than looping forever.

---

## 3. Gating constants — current tuning, not contract

These are private consts. They explain observed behavior. They carry no stability guarantee
and may change without notice; do not encode them in your own scheduling.

| where | constant | current value | why |
|---|---|---|---|
| dynamic/src/lib.rs:408-412 | `PAR_MIN_REGIONS` | 4 | below this, spawn/join costs more than the work |
| | `PAR_MIN_RING_VERTS` | 2048 | measured on `ring_verts - largest_region_verts` |
| | `PAR_MAX_THREADS` | 16 | region skew pins the makespan to the largest region |
| polygon-extract/src/lib.rs:257-264 | `PAR_MIN_CELLS` | 500 000 | the border-edge scan stays serial below this |
| | `PAR_MIN_HOLES` | 64 | hole parenting cutover |
| | `PAR_MIN_REGIONS` | 8 | per-region post-processing cutover |
| | `PARENT_MAX_THREADS` / `POST_MAX_THREADS` | 32 / 16 | memory-bound scans |
| polygon-extract/src/lib.rs:650-654 | `ERODE_BAND` | 64 | rows per band; fits L2, matches the transpose tile |
| | `ERODE_MAX_THREADS` | 16 | erosion is memory-bandwidth-bound, already past the knee |

The `ring_verts - largest` test is the one worth understanding, because it explains why a
map does not speed up when you add threads: per-region parallelism can never finish sooner
than its single largest region, so threads only pay off when the work *outside* that region
is worth overlapping (dynamic/src/lib.rs:456-466). A map that is one huge region plus a
hundred specks stays effectively serial no matter what you pass.

---

## 4. Where the time goes

All figures below were **measured on this machine** — AMD EPYC 7502P (32 cores / 64
threads), release build, working tree at the current commit — against the 13-file
`testdata/` corpus. They are not portable to your hardware; re-run the given command
before quoting anything.

### Per stage

```text
cargo run --release -p rsnav-dynamic --example stage_bench -- testdata
```

```text
file                       extract      pslg  delaunay  skeleton     carve      clip   navmesh       bsp
act3-town.pbm                23.7m      0.7m     11.2m      8.3m      3.8m      1.0m      2.0m      2.1m
act5-spine.pbm               16.6m      0.5m      7.8m      5.3m      2.9m      0.5m      1.1m      1.2m
synth-pillars-1024.pbm        9.4m      0.2m     11.8m      1.5m      2.4m      0.1m      1.2m      2.0m
synth-open-2048.pbm           4.5m      0.0m      0.0m      0.0m      0.0m      0.0m      0.0m      0.0m
```

The shape to take away: `extract` dominates on real maps, `delaunay` is second, and
everything downstream of the CDT is noise. `synth-open-2048` is 4.2M cells that produce two
triangles — the entire cost is the grid scan, and no amount of CDT tuning touches it.

### Across thread counts

```text
cargo run --release -p rsnav-dynamic --example par_bench -- testdata
```

```text
file                          t=1      t=2      t=4      t=8     t=16     t=32     auto
act3-town.pbm               57.6m    48.0m    45.8m    41.4m    41.5m    43.2m    42.6m  ok
act5-spine.pbm              37.5m    32.5m    31.7m    29.6m    28.7m    29.1m    29.2m  ok
synth-open-2048.pbm         17.7m     9.8m     6.7m     5.1m     4.2m     4.0m     4.3m  ok
synth-pillars-1024.pbm      31.9m    31.1m    30.0m    29.6m    29.5m    29.3m    29.3m  ok
```

Every row printed `ok` — byte-identical across all seven settings, on both the legacy path
and (verified separately with `--inset 0.128`) the offset/planarize/winding path.

Scaling is modest and that is expected. `synth-open-2048` scales well because its cost is
the parallel cell scan. `act3-town` gains about 1.4x and then flattens, because a town is
one dominant region. `synth-pillars-1024` barely moves at all. Threads are not the lever
for build time on region-skewed maps.

### The clearance cost flip

```text
cargo run --release -p rsnav-dynamic --example erode_vs_inset -- 1.0
```

```text
file                       cells    erode    build    TOTAL |     inset     tris |   ratio
act3-town.pbm              2611k     4.9m    50.8m    55.7m |    168.4m    17181 |   3.02x
act5-spine.pbm             2217k     4.3m    28.9m    33.1m |    104.4m    12813 |   3.15x
synth-open-2048.pbm        4194k     5.3m     4.7m    10.1m |      4.7m        2 |   0.47x
synth-thin-64x4096.pbm      262k     0.7m     1.4m     2.1m |      1.4m        2 |   0.65x
```

Grid erosion is `O(cells)` and pays that even to produce two triangles; contour inset is
`O(boundary)` and pays in proportion to how convoluted the walls are. So erosion wins on a
dense town (3x) and loses on a large mostly-open grid (0.47x). Radius 1.0 is used because
grid radii are cell-quantized — see [06-clearance.md](06-clearance.md) for why anything
sub-cell is not an apples-to-apples comparison at all.

The simplest corpus driver, if you only want end-to-end wall time per file, is
[`pbm_bench.rs`](../crates/dynamic/examples/pbm_bench.rs) —
`cargo run --release -p rsnav-dynamic --example pbm_bench -- testdata`.

---

## 5. Per-query costs

These are scattered across the source and are collected here once. Each is read off the
implementation, not benchmarked; the complexity is certain, the wall time is yours to
measure.

| operation | cost | note |
|---|---|---|
| `find_path` | one `WallInfo::from_navmesh` per call, `O(triangles)` | its body is two lines (path.rs:71-80). Hoist a `WallInfo` and call `find_path_with_walls`, or use `NavWorld`, for anything repeated |
| `find_path_with_walls` | A* over triangle adjacency + funnel | no per-call allocation of the oracle |
| `WallClearance::clamp` | `O(all wall segments)` × `RELAX_ITERS = 4`, per call | no spatial index (wall_clearance.rs:70, :155). Not a per-agent-per-frame operation on a town-scale mesh |
| `WallClearance::from_navmesh` | `O(triangles)` | rebuild on any mesh change **or** any door state change |
| `visibility_region` | `samples` × one full LOS walk | each walk is bounded by `2 × triangle_count`. `samples` is clamped up to 8 |
| `Bsp::locate` / `nearest` | average `O(log n)` | `query_aabb` is `O(log n + k)` and broad-phase only |
| `DoorSet::get` / `set_state` / `toggle` | `O(doors)` linear scan | plus a full `WallInfo` rebuild per `NavWorld` door mutator, with no batching API |
| `TiledWorld::find_path` | four `Vec`s of length `total_tris` **per call** | allocation scales with total world size, not the explored region (tiled.rs:399-402) |
| `TiledWorld::locate` | `O(tiles)` linear scan with an AABB reject | no world-level index; called at the head of `find_path` and `line_of_sight` |
| `TiledWorld::stitch_all` | `O(tiles²)` pairs; `O(\|edges_a\| × \|edges_b\|)` for each pair that survives a grown-AABB reject | the quadratic pair loop always runs, but the edge cross-product and the two boundary re-materializations only happen for tiles that actually touch. This, not pathfinding, is the tiled scaling wall |
| `Crowd::tick` | `O(slots)` per pass, where `slots` is the **high-water mark** | `slots` never shrinks; a crowd that peaked at 10k pays 10k-wide loops at 10 live agents |
| `Crowd` replan | full A* **every tick** for an agent with an unreachable goal | `plan_failed` does not latch or suppress, despite its own field doc |

See [07-paths-and-queries.md](07-paths-and-queries.md) and [11-crowds.md](11-crowds.md) for
what to do about each.

---

## 6. Measurement protocol

**Always release. Never debug.** This is not general advice; there is a specific reason.

`carve_by_winding` runs a brute-force `winding_number` cross-check against its accelerated
index for *every* triangle under `debug_assert_eq!`
([`crates/triangle/src/winding.rs:336`](../crates/triangle/src/winding.rs)). It is a
permanent differential oracle and it is deliberate, but it makes a debug build of the inset
path dramatically slower than release. Benchmarking or demoing the inset pipeline in a debug
build measures the oracle, not the pipeline.

The same applies to `drop_interior_constraints`, whose sorted-markers precondition is a
`debug_assert!` (winding.rs:374) — in release an unsorted slice silently produces wrong
geometry rather than failing. Run your test suite in debug so the asserts fire; run your
benchmarks in release.

A figure worth acting on carries four things: a release build with a warm cache and
best-of-N (par_bench and stage_bench do this internally), the exact harness command, the
hardware, and the corpus. Corpus shape dominates everything — the same code is 3x faster
with erosion on a town and 2x slower on an open grid.

---

## 7. The digest workflow

`stage_bench --digest` runs the real `build_navmesh_from_bitfield` pipeline and prints an
FNV-1a 64-bit hash of each serialized `NavMesh`:

```text
cargo run --release -p rsnav-dynamic --example stage_bench -- --digest testdata
```

```text
act3-town.pbm                18295 tris 23a215ccaf7911ea
act5-spine.pbm               13921 tris 7a4c506f5721af6f
synth-pillars-1024.pbm       23816 tris 809ce7ad7a7ef6c3
synth-open-2048.pbm              2 tris b2de0e4c0df4152a
```

(Measured here, same machine and tree as section 4. The digests are intended to be a
property of the code rather than of the hardware, so a different machine building the same
commit should print the same values — but note the scope: what is *gated* is byte-identity
across thread counts on one machine (par_bench). Cross-machine identity is an inference from
the same structural properties plus IEEE-754 f64 with no fast-math and no FMA contraction;
no harness in this tree checks it. If you are gating CI on these digests across
heterogeneous runners, record a baseline on each target first rather than assuming they
agree.)

This is the bit-identity change gate. Record the digests before a refactor that is meant to
be behavior-preserving; if one moves, the refactor changed output.

`--digest` does **not** ignore `--inset`: `base_opts()` sets `inset` for every mode, so
`--digest --inset 0.128` hashes the offset/planarize/winding path and prints entirely
different values. That is useful for gating a change to the inset pipeline against itself,
but the two sets are not comparable. Record digests without `--inset` if you want the
long-lived baseline the harness's own comment describes (stage_bench.rs:89-91,
stage_bench.rs:183-198).

The same property is what makes a content hash of the input bitfield a valid cache key for a
baked navmesh — see [14-saving-and-loading.md](14-saving-and-loading.md).

---

## Related pages

- [06-clearance.md](06-clearance.md) — which clearance mechanism to use, and its cost model
- [10-dynamic-rebuilds.md](10-dynamic-rebuilds.md) — coalescing, `NavStats`, the frame protocol
- [12-large-worlds.md](12-large-worlds.md) — the `TiledWorld` scaling limits listed above
- [16-troubleshooting.md](16-troubleshooting.md) — "output differs between runs", "build got slower"
