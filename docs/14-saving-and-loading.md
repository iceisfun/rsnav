# Saving a navmesh and loading it back

For build-pipeline and tools authors: bake a navmesh once at content-build time, load it
fast at runtime. Also for anyone writing a reader in another language — the byte layout is
normative in [`crates/navmesh/FORMAT.md`](../crates/navmesh/FORMAT.md) and is not
restated here.

Prerequisites: you have a `NavMesh` (see [building navmeshes](05-building-navmeshes.md) for
the grid path, [authored geometry](13-authored-geometry.md) for the vector path).

Working reference: [`crates/navmesh/examples/save_and_load.rs`](../crates/navmesh/examples/save_and_load.rs)
— builds a 10x4 corridor split by a marker-99 wall, serializes, reloads, and asserts the
round trip is exact triangle slot by triangle slot.

```
cargo run -p rsnav-navmesh --example save_and_load

built navmesh: 6 vertices, 4 triangles, 2 region(s)
serialized to 560 bytes (0.5 KiB)
round trip verified — all sections identical
8 constrained edge incidences in the mesh (markers != 0)
```

## Why bake

Building from a bitfield costs extraction, one CDT per region, a merge and a BVH build.
Loading costs parsing six sections of packed little-endian scalars. Nothing in the file
needs to be recomputed if you write all six sections, which `rsnav-navmesh` always does.

## The four calls

```rust
pub fn to_bytes(&self) -> Vec<u8>
pub fn write_to<W: Write>(&self, w: &mut W) -> Result<usize, SaveError>
pub fn from_bytes(bytes: &[u8]) -> Result<Self, LoadError>
pub fn read_from<R: Read>(r: &mut R) -> Result<Self, LoadError>
```

All four in [`crates/navmesh/src/binary.rs`](../crates/navmesh/src/binary.rs) at lines
117, 124, 252 and 258. `write_to` returns the byte count written; `SaveError` has exactly
one variant, `Io`. `to_bytes` cannot fail — it writes into a `Vec` and `.expect`s that a
`Vec` never errors (binary.rs:119).

`LoadError` covers `Io`, `BadMagic`, `UnsupportedVersion`, `MissingRequiredSection`,
`SectionLengthMismatch`, `Truncated` and `BadIndex`. Both error types implement `Display`
and `std::error::Error`, and both have a `From<io::Error>`, so they compose with `?`.

### Trap: `read_from` consumes the entire reader

`read_from` opens with `r.read_to_end(&mut all)?` (binary.rs:259-260). It does not read the
header, learn the file length, and stop. A navmesh therefore **cannot be embedded in the
middle of a larger stream with your own data following it** — the loader swallows the rest
of the stream along with the mesh. It does not error: section offsets are absolute and all
still resolve inside the oversized buffer, so the mesh loads correctly and the trailing
bytes are silently ignored. On a non-seekable reader they are then gone.

If you need a container format, store the navmesh as its own length-prefixed blob and hand
`from_bytes` an exact subslice. Note `from_bytes` routes through `read_from` over a
`Cursor`, so it copies the whole slice into a fresh `Vec` before parsing.

## Sections: required, optional, and the cost of omitting them

Six section types. `rsnav-navmesh` always writes all six, so this section only matters if
you are writing a file from another language.

| ID | Section | Required | If absent |
|---:|---|:---:|---|
| 1 | `META` | yes | `MissingRequiredSection` |
| 2 | `VERTICES` | yes | `MissingRequiredSection` |
| 3 | `TRIANGLES` | yes | `MissingRequiredSection` |
| 4 | `ADJACENCY` | no | rebuilt by hashing undirected edges |
| 5 | `EDGE_MARKERS` | no | every marker silently becomes 0 |
| 6 | `TRI_INFO` | no | area, centroid and region recomputed |

Omitting `ADJACENCY` is benign: `recompute_adjacency` (binary.rs:503) hashes each
undirected edge and bonds the two triangles that share it, which reproduces the writer's
output for any manifold mesh — and the CDT never produces a non-manifold one.

Omitting the other two is not benign.

**Without `EDGE_MARKERS`, every edge loads as marker 0**, which by the convention in
[units and conventions](04-units-and-conventions.md) means unconstrained. The loader
raises no error. Every wall in the mesh is gone as far as `WallInfo`, the funnel, and
line-of-sight are concerned: paths route straight through them. If `TRI_INFO` is also
absent, region labelling is done by BFS that never sees a constrained edge to stop at, so
the entire connected mesh collapses into a single region and `reachable()` returns true
everywhere. FORMAT.md states the collapse for the minimal three-section file; with
`TRI_INFO` present the region IDs still load correctly from the file while the walls are
still gone, which is the more confusing of the two failures.

**Without `TRI_INFO`, `META`'s `region_count` is ignored.** The loader derives it as
`max(region) + 1` over the recomputed labels instead (binary.rs:457-461). A writer that
emits a considered `region_count` in `META` but no `TRI_INFO` will see that value
discarded.

Unknown section types are skipped by `continue` (binary.rs:302), by design — that is what
lets a future version add optional sections without breaking a v1 reader.

## What is in the file, and what is not

Round-trips exactly: vertex positions, triangle vertex indices, neighbor indices, per-edge
markers, per-triangle area, centroid and region ID, `region_count`, and the mesh AABB. The
example asserts the three counts and `NavTriangle` equality slot by slot, which covers
everything except the vertex positions and the AABB — those it does not compare.

Not stored, because it is derived and cheap to rebuild:

- `Bsp` — deliberately excluded; FORMAT.md's rationale is that spatial indices have
  competing implementations and belong in a sidecar if load time ever matters.
- `WallInfo` and `WallClearance` — derived from the mesh, O(triangles) to build.
- `DoorSet` — its resolved edges are vertex indices into a specific mesh instance.
- Anything implementing `NavMetadata` — that is your game data, not rsnav's.
- Per-vertex markers, which exist only inside the CDT during construction.

## Load order

Do these in order. Step 2 before step 3 is the one that fails silently.

1. `NavMesh::from_bytes(&bytes)` (or `read_from` on a file handle).
2. `nav.translate(offset)` **if** you are baking a world placement. Skip this if you are
   placing with a `TiledWorld` per-tile offset instead — doing both applies the shift
   twice. See [large worlds](12-large-worlds.md).
3. `Bsp::build(&nav)`. A BVH built before step 2 stores absolute AABBs and is silently
   invalidated by the translate, with no error at build time and no error at query time —
   just wrong answers (navmesh.rs:113-118).
4. `WallInfo::from_navmesh(&nav)`, and a `WallClearance` if you use one. `WallClearance`
   caches vertex coordinates and has the same translate-first hazard.
5. Rebuild any `DoorSet` from scratch. Stored edge keys from before the bake are vertex
   indices into a different mesh instance. There is no in-place re-resolve method; `Door`
   retains its authoring segment in `Door::line` for exactly this, so re-add each door
   with `DoorSet::add(&nav, &bsp, a, b, state)`. See
   [doors and NavWorld](09-doors-and-navworld.md).

`TriangleId` and `VertexId` values are per-instance and are not portable across a save/load
boundary any more than across a rebuild. Anything you persisted alongside the mesh keyed
by triangle ID is only valid against the exact bytes it was computed from.

## Format orientation for external implementers

Enough to know what you are looking at; [FORMAT.md](../crates/navmesh/FORMAT.md) is
normative and this is not a substitute for it.

A 16-byte header (`"RSNAVMSH"`, then u32 version, then u32 section count), followed by
`section_count` table entries of 24 bytes each (u32 type, u32 reserved, u64 offset, u64
length), followed by the section bodies. The offsets are **absolute from the start of the
file**. Do not assume bodies appear in table order, do not assume they are contiguous, and
do not identify a section by its position in the table — only by its type ID.

The example's 560 bytes decompose as 16 header + 144 table + 48 META + 96 VERTICES +
48 TRIANGLES + 48 ADJACENCY + 48 EDGE_MARKERS + 112 TRI_INFO, for 6 vertices and 4
triangles. Useful as a first check on a reader you are writing.

## Determinism makes a content hash a valid cache key

Navmesh output is byte-identical across thread counts for the same input and options. That
is a design commitment, not an accident of scheduling, and it is what makes hashing
`to_bytes()` a sound cache key for a bake step: identical input plus identical
`BuildOptions` produce identical bytes on any machine configuration, so a hash miss means
the content actually changed.

[`crates/dynamic/examples/stage_bench.rs`](../crates/dynamic/examples/stage_bench.rs)
exploits exactly this. `--digest` runs the real pipeline and prints an FNV-1a hash of each
serialized navmesh, which is the bit-identity change gate used when refactoring the build:

```
cargo run --release -p rsnav-dynamic --example stage_bench -- --digest
```

The full determinism argument — what it does and does not promise — belongs to
[performance and determinism](15-performance-and-determinism.md).
