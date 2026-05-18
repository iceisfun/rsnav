# rsnav2 navmesh binary format

**Version:** 1
**Endianness:** little-endian, always
**Sizes are bytes unless noted otherwise**

This document is normative. A reader written in any language (Rust,
TypeScript, Go, Python, C++…) that follows it correctly will load any
file written by `rsnav-navmesh` v1.

There is no compression, no varints, no alignment padding beyond what the
spec calls out explicitly, and no offsets relative to anything other than
the start of the file. Reading the file is byte-counting.

## File layout

```
┌─────────────────────────────────────────────────┐
│ File header                  16 bytes           │
├─────────────────────────────────────────────────┤
│ Section table                24 × N bytes       │
├─────────────────────────────────────────────────┤
│ Section bodies               variable           │
└─────────────────────────────────────────────────┘
```

The section bodies appear *after* the section table in the file. A reader
must use each section table entry's absolute byte offset to find its body —
the bodies are not guaranteed to appear in any particular order, and gaps
between them (though not produced by `rsnav-navmesh` itself) are allowed.

## File header (16 bytes)

| Offset | Size | Type | Field          | Notes                                     |
| -----: | ---: | :--- | :------------- | :---------------------------------------- |
| `0`    | `8`  | ASCII | `magic`       | Must equal `"RSNAVMSH"` (no terminator). |
| `8`    | `4`  | u32   | `version`     | `1` for this spec. Bumped on breaking changes. |
| `12`   | `4`  | u32   | `section_count` | `N`, the number of section table entries that follow. |

A reader that finds different magic bytes or an unsupported version must
reject the file. A v1 reader presented with a future version may attempt
to keep going only if every section it requires is still present and
length-compatible, but the safe default is to refuse.

## Section table (24 × N bytes)

The section table is `N` consecutive entries of 24 bytes each.

| Offset | Size | Type | Field      | Notes                                     |
| -----: | ---: | :--- | :--------- | :---------------------------------------- |
| `0`    | `4`  | u32  | `type`     | See "Section types" below.                |
| `4`    | `4`  | u32  | `reserved` | Must be 0. Future flags can use it.       |
| `8`    | `8`  | u64  | `offset`   | Absolute byte offset of the section body from the start of the file. |
| `16`   | `8`  | u64  | `length`   | Length of the section body in bytes.      |

**Unknown section types must be silently ignored.** That is the contract
that lets future versions add optional sections without breaking v1
readers. A section is identified solely by its `type` ID; do not try to
match by order.

## Section types

| ID   | Name           | Required? | Purpose                                |
| ---: | :------------- | :-------: | :------------------------------------- |
| `1`  | `META`         | yes       | Vertex/triangle/region counts + AABB.  |
| `2`  | `VERTICES`     | yes       | Vertex positions.                      |
| `3`  | `TRIANGLES`    | yes       | Triangle vertex indices, CCW.          |
| `4`  | `ADJACENCY`    | no        | Triangle neighbor indices.             |
| `5`  | `EDGE_MARKERS` | no        | Per-edge constraint markers.           |
| `6`  | `TRI_INFO`     | no        | Area, centroid, region ID per triangle.|

If an optional section is absent, the reader is expected to recompute its
contents from the required sections (plus `EDGE_MARKERS` for region IDs).
This means a minimal valid file is `META + VERTICES + TRIANGLES` (3
sections, no per-edge marker data → all edges treated as interior, region
labelling reflects mesh topology only).

If `EDGE_MARKERS` is absent, every edge is treated as unconstrained (which
makes the whole connected mesh one region). This is correct behavior for
"raw" meshes that never went through `form_skeleton` + `carve_holes`, but
will give wrong reachability results for navmeshes that did.

### Section 1 — META (40 bytes, fixed)

| Offset | Size | Type | Field            | Notes                          |
| -----: | ---: | :--- | :--------------- | :----------------------------- |
| `0`    | `4`  | u32  | `vertex_count`   |                                |
| `4`    | `4`  | u32  | `triangle_count` |                                |
| `8`    | `4`  | u32  | `region_count`   | Equal to `1 + max(region)`.    |
| `12`   | `4`  | u32  | (padding)        | Must be 0.                     |
| `16`   | `8`  | f64  | `aabb.min.x`     |                                |
| `24`   | `8`  | f64  | `aabb.min.y`     |                                |
| `32`   | `8`  | f64  | `aabb.max.x`     |                                |
| `40`   | `8`  | f64  | `aabb.max.y`     | (Section ends at offset 48.)   |

> Yes, "40 bytes, fixed" but the last field starts at offset 40 and ends
> at 48. The section length in the table will be 48; the trailing `_x` /
> `_y` of the max corner is part of the section.

### Section 2 — VERTICES (`vertex_count × 16` bytes)

One vertex per `(f64, f64)` pair, indexed `0..vertex_count`. `x` first,
then `y`. No padding.

### Section 3 — TRIANGLES (`triangle_count × 12` bytes)

One triangle per three `u32` vertex indices, listed in CCW order. The CDT
builder guarantees CCW orientation; downstream code relies on it.

Each `u32` must be `< vertex_count`. The reader must reject any out-of-
range index with an error.

### Section 4 — ADJACENCY (`triangle_count × 12` bytes) — optional

One triangle per three `u32` neighbor indices. The neighbor at index `i`
is the triangle sharing the edge opposite vertex `i` of this triangle.

Value `0xFFFFFFFF` (i.e. `u32::MAX`) means "no neighbor" — that edge is on
the mesh boundary.

If present, every non-`MAX` value must be `< triangle_count`, and bonds
must be symmetric (if triangle `A` lists `B` as a neighbor, `B` must list
`A`). Readers may verify; the writer guarantees this.

If absent, the reader must rebuild adjacency by hashing each undirected
edge of every triangle into a `HashMap`; two triangles sharing an edge get
bonded. The bond produces undefined results for non-manifold input (an
edge shared by more than two triangles), which the CDT builder never
produces.

### Section 5 — EDGE_MARKERS (`triangle_count × 12` bytes) — optional

One triangle per three `i32` marker values. Marker `0` means the edge is
unconstrained (an interior Delaunay edge). Any non-zero value means the
edge was a constrained PSLG segment and the marker is that segment's
marker value (which the PSLG input chose — typically `1` for "outer hull",
or any user-supplied tag for a specific wall).

Edge `i` is the edge opposite vertex `i`, same convention as `ADJACENCY`.

For triangles whose constrained edge is shared by two triangles in the
mesh (a "two-sided wall"), both triangles store the same non-zero marker
on their respective view of that edge. For boundary walls (constrained
edges with no neighbor on the other side) the marker appears on the one
triangle that exists.

### Section 6 — TRI_INFO (`triangle_count × 28` bytes) — optional

Per-triangle derived metadata: area, centroid, region ID.

| Offset within entry | Size | Type | Field         |
| ------------------: | ---: | :--- | :------------ |
| `0`                 | `8`  | f64  | `area`        |
| `8`                 | `8`  | f64  | `centroid.x`  |
| `16`                | `8`  | f64  | `centroid.y`  |
| `24`                | `4`  | u32  | `region`      |

Each entry is 28 bytes; entries are packed (no padding).

If absent, the reader recomputes all three:
* `area` from the shoelace formula on the three vertex positions.
* `centroid` from the arithmetic mean of the three vertices.
* `region` by BFS through the adjacency graph, **not crossing any edge
  whose marker is non-zero**. Each BFS root gets a fresh region ID
  starting at `0` and incrementing.

Two triangles share a `region` ID iff a path exists between them through
the navmesh without crossing a constrained edge. Use this as a cheap
reachability pre-check (`a.region == b.region`) before running A*.

## What's intentionally not in the file

* **Per-vertex markers.** They never matter post-construction; they exist
  in the CDT only to remember which PSLG vertex created which mesh
  vertex.
* **Spatial indices (BSP / kd-tree / quad-tree).** Derived structures
  with multiple competing implementations. Each consumer can build the
  index it prefers at load time (typically sub-millisecond for the mesh
  sizes navmeshes hit in practice). If a future deployment needs zero-
  load-time queries, ship the index as a sidecar file (e.g.
  `mymesh.navmesh.bsp`) that's regenerable from the `.navmesh`.
* **Compression.** Trivial to add at the I/O layer (gzip, zstd, …) if
  file size matters. The on-disk format itself stays uncompressed so
  random-access loaders work and the format spec stays a one-pager.

## Forward compatibility rules

* The magic bytes never change.
* `version` increments on any change to the layout of any *required*
  section (`META`, `VERTICES`, `TRIANGLES`) or the file header.
* Adding a new optional section type does **not** increment `version`.
  Readers will see the new section type and ignore it per the "unknown
  section types must be silently ignored" rule.
* Changing the layout of an existing optional section requires picking a
  *new* section type ID. Old IDs may be deprecated but not repurposed.
