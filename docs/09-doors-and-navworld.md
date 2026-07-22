# Doors, zones, and letting NavWorld hold your pieces

Prerequisites: a built navmesh ([05](05-building-navmeshes.md)) and the query
surface ([07](07-paths-and-queries.md)). This page covers gates that open and
shut without rebuilding anything, and attaching your own per-triangle data so a
path can report "entered the tavern".

Runnable example: [`crates/navigation/examples/doors_and_zones.rs`](../crates/navigation/examples/doors_and_zones.rs)

```
cargo run -p rsnav-navigation --example doors_and_zones
```

Everything below is quoted from that example or cited to `#[cfg(test)]` code,
which is flagged as such each time. There is also
[`rsnav-door-demo`](../crates/door-demo/src/main.rs) to run interactively,
but it uses a *different* design — see the last section.

---

## Start with NavWorld

`NavWorld<M>` owns a `NavMesh`, its `Bsp`, a `DoorSet`, and the derived
`WallInfo`, and keeps the wall oracle rebuilt for you
([`world.rs:157`](../crates/navigation/src/world.rs)):

```rust
pub struct NavWorld<M = NoMetadata> {
    nav: NavMesh,
    bsp: Bsp,
    doors: DoorSet,
    walls: WallInfo,
    meta: M,
}
```

Two reasons to reach for it rather than holding the four pieces yourself.

**It fixes the per-call rebuild.** The free `find_path` rebuilds an
`O(triangles)` `WallInfo` on every single call — its body is two lines
([`path.rs:70`](../crates/navigation/src/path.rs)). `NavWorld::find_path`
delegates to `find_path_with_walls` against the oracle it already owns, so
repeated querying costs nothing extra. See [07](07-paths-and-queries.md) for
the cost breakdown.

**It keeps door state and query results consistent by construction.** Every
door mutator calls the private `rebuild_walls()`, and every query threads
`&self.walls`, so A*, line-of-sight, `path_clear` and `visibility_region` all
react to a door with no per-feature plumbing.

Construct it with `NavWorld::without_metadata(nav)` or `NavWorld::new(nav, meta)`.
It takes the `NavMesh` **by value** and builds its own `Bsp`, so a `NavBuild`
from `rsnav-dynamic` contributes only its `navmesh` field. There is no `nav_mut`:
the mesh is immutable once emplaced, and a rebuild means constructing a new
`NavWorld`.

`NavWorld` derives nothing — no `Debug`, no `Clone`.

**What it does not own: `WallClearance`.** If you need free (non-path-following)
movement held off walls, build and maintain that alongside the world yourself,
and rebuild it on every door change too. See [08](08-moving-agents.md).

`NavWorld` and `TiledWorld` are unrelated types with no bridge, and `TiledWorld`
has no doors at all: `Link` carries no open/closed state and the tiled traversal
consults `is_wall_edge_local` directly with no `WallInfo` overlay. There is no
NavWorld-of-tiles. See [12](12-large-worlds.md). `rsnav-crowd` likewise consults
no doors ([11](11-crowds.md)).

---

## A door is a set of edges

A door is not geometry and not a region of triangles. It is a set of internal
portal **edges** that a closed door promotes to walls. Opening or closing one
flips which edges the traversal code treats as impassable — nothing else.

What is **not** rebuilt when a door toggles: the `NavMesh`, every `TriangleId`
and `VertexId`, and the `Bsp`.

What **must** be rebuilt: the `WallInfo`, and any `WallClearance` you keep.
"No rebuild" means the mesh and the BVH, not the oracle. `NavWorld` handles the
`WallInfo` for you; the `WallClearance` is yours.

Only unconstrained, non-boundary edges are eligible. `resolve_door_edges` skips
anything `is_wall_edge_local` reports
([`doors.rs:233`](../crates/navigation/src/doors.rs)) — you cannot put a door
in a wall.

---

## Two ways to author one

### From a drawn segment

`DoorSet::add(nav, bsp, a, b, state)` resolves the segment `a → b` to the portal
edges it crosses: broad-phase `Bsp::query_aabb` over the segment bounds,
narrow-phase `segment_intersection`, results collected as canonical
(smaller-index-first) vertex pairs and deduplicated. This is the right call when
a level author drags a stroke across a passage and the passage may be several
triangles wide.

```rust
let seg_door = world.add_door(
    Vertex::new(9.5, 3.0),
    Vertex::new(9.5, 7.0),
    DoorState::Closed,
);
```

`add` itself returns a `DoorId`. The resolution step is also exposed on its own
as `resolve_door_edges(nav, bsp, a, b)`, whose return type is `Vec<(u32, u32)>`:
the source writes `Vec<EdgeKey>`, but `EdgeKey` is a `pub(crate)` alias
([`wall.rs:25`](../crates/navigation/src/wall.rs)) and is not nameable
downstream. That vector's order is non-deterministic — the keys come out of a
`HashSet` — which is invisible for door semantics but would break anything that
hashes or serialises it.

### From a picked edge

`nearest_portal_edge(nav, bsp, p)` returns the internal portal edge nearest `p`,
searching the triangle containing `p` plus its three immediate neighbours. Pair
it with `add_edge`, which skips resolution entirely — the edge *is* the door:

```rust
let (va, vb) = nearest_portal_edge(world.nav(), world.bsp(), cursor)
    .expect("an internal portal edge in the doorway");
let edge_door = world.add_door_edge(va, vb, DoorState::Closed);
```

Use this when the crossing must be unambiguous: cursor-driven authoring, or a
doorway one triangle-edge wide. `nearest_portal_edge` returns `None` if `p` is
off the mesh or neither the located triangle nor its neighbours has an
unconstrained edge. `add_edge` does not validate the pair; a key that is not a
real portal simply never matches a traversal and the door is inert.

Spec for both routes, as test code:
[`doors.rs:317`](../crates/navigation/src/doors.rs) `door_cuts_a_portal_and_blocks_when_closed`
and [`doors.rs:398`](../crates/navigation/src/doors.rs)
`edge_pick_gates_the_same_crossing_as_a_segment`. Both are `#[cfg(test)]` blocks,
not runnable examples.

---

## The inert-door trap

If the authoring segment crosses no toggleable portal, the door does nothing and
`add` reports no error. It returns a `DoorId` exactly as it would for a working
door. The three ways to land here:

- the segment is drawn off-mesh, or across a wall rather than across the opening;
- it lies exactly *along* an edge — `segment_intersection` returns `None` for
  parallel and collinear pairs ([`geom.rs:203`](../crates/common/src/geom.rs));
- it passes exactly through a vertex.

The only signal is `edge_count()`, and checking it is your job. Nothing enforces
it. From the example, a stroke drawn across the dividing wall instead of the
doorway:

```
[inert]  edge_count = 0 (door does nothing, no error was reported)
[inert]  find_path still ok: true
```

```rust
let d = world.doors().get(id).unwrap();
assert!(d.edge_count() > 0, "door {id:?} is inert");
```

---

## generation() as the repath trigger

`DoorSet::generation()` bumps on `add`, `add_edge`, `remove`, `clear`, `toggle`,
and on any `set_state` that is a *real* change. Setting a door to the state it
already has is a no-op and correctly does **not** bump
(pinned by `generation_bumps_only_on_real_change`,
[`doors.rs:365`](../crates/navigation/src/doors.rs), test code).
`NavWorld::generation()` forwards it.

A path planned at generation `g` is stale once the counter differs. It is a
change counter, not a state hash: close-then-reopen returns you to the original
world with a strictly larger generation, so a stale flag can be a false positive.
Confirm with `path_clear`, which re-walks the polyline against the current
oracle:

```
[repath] planned at generation 0, world is at 1 -> stale
[repath] path_clear on the old polyline: false
```

`path_clear` ignores clearance entirely — it is a zero-width test
([07](07-paths-and-queries.md)).

**Not the same counter as `NavBuild::generation`.** `rsnav-dynamic` has its own unrelated
`generation` field on `NavBuild` ([dynamic/src/lib.rs:152](../crates/dynamic/src/lib.rs))
counting *mesh rebuilds*, and a `NavStats::last_completed_generation` alongside it
([10](10-dynamic-rebuilds.md)). `DoorSet::generation()` counts *door-state changes* against
a mesh that never changed. If you run both a `NavWorker` and doors, you are tracking two
independent counters and neither implies the other — see
[§Doors do not survive a mesh rebuild](#doors-do-not-survive-a-mesh-rebuild).

One asymmetry worth knowing: `NavWorld::set_door` calls `rebuild_walls()`
unconditionally ([`world.rs:250`](../crates/navigation/src/world.rs)), even
when the underlying `DoorSet::set_state` was a no-op. Reading `generation()` to
decide whether to repath is correct; assuming that no work happened is not.

---

## Doors do not survive a mesh rebuild

Every edge a door stores is a pair of vertex indices into one specific `NavMesh`.
After any rebuild — a `NavWorker` swap ([10](10-dynamic-rebuilds.md)), a
different erosion radius, a reload — those indices name different vertices or
none at all, and the door silently gates the wrong edges or nothing.

`Door::line` keeps the authoring segment and its doc comment says it is kept "so
the door can be re-resolved if the mesh is rebuilt"
([`doors.rs:53`](../crates/navigation/src/doors.rs)). **No such API exists.**
Nothing calls `resolve_door_edges` outside `DoorSet::add` and a test. The
segment is there for you to re-author from; the re-resolution is your loop.
This is errata (f) in [16](16-troubleshooting.md).

Rebuild the `DoorSet` from scratch after every mesh change, replaying your own
door records through `add` / `add_edge` against the new mesh.

---

## Costs

- `get`, `set_state`, `remove` and `toggle` are linear scans over a `Vec<Door>`.
  Fine for tens of doors; not a spatial structure.
- Every `NavWorld` door mutator is a whole-mesh `WallInfo` rebuild —
  `O(triangles)`. There is **no batching API**, so toggling N doors in one frame
  costs N full rebuilds. If you must flip many at once, mutate a `DoorSet`
  directly and build one `WallInfo::from_navmesh_with_doors` yourself, then use
  the free `find_path_with_walls` against it.
- `DoorId` is a plain `pub u32` newtype with no generation tag. The counter is
  monotonic and `clear()` does not reset it, so ids are never reused within one
  `DoorSet` — a stale id resolves to `None` from `get`, and `set_state` /
  `toggle` on it are silent no-ops rather than a detectable error. A
  *reconstructed* set does restart at 0, so ids held across a rebuild can
  collide with fresh ones.

---

## What doors do not reach

`NavMesh::reachable(a, b)` is `self.triangle(a).region == self.triangle(b).region`
([`navmesh.rs:105`](../crates/navmesh/src/navmesh.rs)), and `region` is
computed at build time from *constrained* edges only. Doors are invisible to it.

`astar` runs `nav.reachable(start, goal)` as a cheap pre-check
([`astar.rs:92`](../crates/navigation/src/astar.rs)). A goal sealed off by a
fully-closed set of doors therefore **passes** that pre-check and only fails
after a full A* exhausts the open set. Correct, but not free: a door-sealed
query costs a whole search, not an O(1) reject. The example shows the mesh
unchanged under a closed door:

```
[closed] mesh still has 10 triangles, 1 regions (regions ignore doors)
```

If you need a cheap "is that room sealed?" test, track it in your own game state.

---

## Zones and per-triangle data

`NavMetadata` is the general mechanism for attaching your data to triangles
([`world.rs:52`](../crates/navigation/src/world.rs)):

```rust
pub trait NavMetadata {
    type Zone: Clone + PartialEq;
    type Value;
    fn zone(&self, tri: TriangleId) -> Option<Self::Zone>;
    fn value_at(&self, tri: TriangleId, p: Vertex) -> Option<&Self::Value>;
}
```

rsnav locates the triangle a world point falls in and hands you its
`TriangleId`. It never sees your type. You own the mapping — a `Vec` indexed by
triangle, a `HashMap`, whatever.

One structural constraint: **`value_at` returns a borrow.** A store that
computes a value on the fly has nowhere to return it from. The example
materialises the labels once at construction and hands out references:

```rust
struct Rooms { labels: Vec<&'static str> }

fn value_at(&self, tri: TriangleId, _p: Vertex) -> Option<&&'static str> {
    self.labels.get(tri.index())
}
```

`zone` returns by value and only needs `Clone + PartialEq`, so it has no such
constraint. `NoMetadata` (the default type parameter) answers `None` to both.

`zone_at(p)` and `metadata_at(p)` return `None` both when `p` is off the mesh
and when the triangle carries nothing — the two are indistinguishable. Call
`locate(p)` first if you need to tell them apart.

```
[zones]  zone_at(start) = Some("hall")
[zones]  metadata_at(off-mesh) = None (indistinguishable from 'no value here')
```

### zone_crossings

`zone_crossings` walks `PathResult::triangles` and emits a `ZoneCrossing`
wherever consecutive triangles disagree. It costs nothing beyond the path you
already have. Two caveats.

**The starting zone is never emitted.** It reports transitions only. Query it
yourself from the first triangle of the channel:

```rust
let opening = world.meta().zone(path.triangles[0]);
```

```
[zones]  starting zone (from path.triangles[0]) = Some("hall")
[zones]  Some("hall") -> Some("tavern") at (9.50, 5.00)
```

**Crossing points are portal midpoints on the unpulled corridor.** `point` is
the midpoint of the shared edge between the two disagreeing triangles — the raw
A* channel, not the string-pulled polyline you render. It may not lie on
`path.points` at all, and falls back to `nav.triangle(b).centroid` if the two
triangles are somehow not neighbours. For a trigger volume this is close enough;
for an exact "crossed the line at t = …" you must intersect the polyline
yourself.

Spec as test code: [`world.rs:413-483`](../crates/navigation/src/world.rs)
(`world_emplace_path_and_zone_crossings`, `world_doors_are_door_aware_and_bump_generation`,
`no_metadata_has_no_zones`) — `#[cfg(test)]`, not runnable examples.

---

## The other door design: carve the bitfield

`rsnav-door-demo` does **not** use `DoorSet`. It treats a door as an obstacle in
the source `Bitfield`: closing one marks its cells non-walkable, resubmits the
snapshot to a `NavWorker`, and the navmesh rebuilds without the gap
([`door-demo/src/main.rs`](../crates/door-demo/src/main.rs)). Both designs are
valid; they answer different requirements.

| | `DoorSet` (this page) | Carve the bitfield ([10](10-dynamic-rebuilds.md)) |
|---|---|---|
| Cost per toggle | one `O(triangles)` `WallInfo` rebuild | a full navmesh rebuild, off-thread |
| Latency | same frame | next published generation |
| IDs survive | yes | no — everything is reminted |
| Doors survive | yes | no — the `DoorSet` must be rebuilt |
| Geometry changes | no | yes |
| Regions / `reachable()` react | no | yes |
| Works with a moving/animated leaf | no | yes, at rebuild granularity |
| Needs `rsnav-dynamic` | no | yes |

Rule of thumb: a pure open/shut barrier across an existing portal is a
`DoorSet` entry. Anything that changes the shape of walkable space — a
portcullis with a thickness, a collapsing bridge, a placed building — is a
rebuild. If you need `reachable()` and `region` to reflect the door (for a cheap
sealed-room test, or for `NavMesh::random_point_in_region` to stay inside the
room an agent is trapped in, which is exactly what the demo exploits), you need
the rebuild design.
