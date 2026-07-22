# What a navmesh is, and why not a grid

You have run [the quickstart](01-quickstart.md): a grid of booleans went in, a
list of points came out, and the points bent around a wall. This page explains
what happened, in the order the data actually moved. It is prose and pictures
with almost no API — the pages that follow assume the five words defined at the
bottom, and this is where they are defined.

If you already have a working grid A\*, read [03-from-grid-astar.md](03-from-grid-astar.md)
instead. It covers the same ground as a translation table and is written to be
read without this page.

---

## Walkable space as polygons, not cells

A grid answers one question well: *is this cell walkable?* It answers it by
storing one bit per cell, which means the storage — and the search — grows with
the **area** of the world, whether or not that area contains anything.

A navmesh stores the same information as the *shape of the walkable region*: a
closed outer boundary, plus a closed boundary around each obstacle inside it.
Nothing is stored for the interior. A 2048x2048 open field and a 64x64 open
field are the same polygon at different scales, and cost the same to represent.

That is not a rhetorical claim. `testdata/synth-open-2048.pbm` is a
2048 x 2048 grid — 4,194,304 cells — with a large open walkable area. Built
through `build_navmesh_from_bitfield` it becomes **2 triangles**. Meanwhile
`testdata/synth-pillars-1024.pbm` has a quarter as many cells (1,048,576) and
becomes **23,816 triangles**, because it is full of pillars and every pillar is
boundary. Reproduce both with:

```
cargo run --release -p rsnav-dynamic --example pbm_bench -- testdata
```

The size of a navmesh tracks the **complexity of the boundary**, not the size
of the world. This single property is why the rest of the library is shaped the
way it is.

---

## Why triangles

Once you have decided to store walkable space as polygons, you still have to
cut those polygons into pieces small enough to search. Triangles are the choice
here for three reasons, and each one is load-bearing later.

**A triangle is convex.** Any two points inside a triangle are connected by a
straight line that stays inside it. So movement *within* one piece never needs
to be planned — only movement *between* pieces does. Convex polygons in general
would do, but triangles are the only shape you can always get.

**Adjacency is trivial.** A triangle has exactly three edges, so it has at most
three neighbours, and "which neighbour is across edge *i*" is one array lookup.
There is no ambiguity about diagonals, no 4-versus-8 connectivity question, and
no special case at a boundary — a boundary edge simply has no neighbour.

**The count follows the boundary.** A big empty room is two triangles whether
it is 10 units across or 10,000. Adding detail costs triangles only where the
detail is.

```
   grid: cost ~ area                     navmesh: cost ~ boundary

   +--+--+--+--+--+--+                   +--------------------+
   |  |  |  |  |  |  |                   |                  / |
   +--+--+--+--+--+--+                   |               /    |
   |  |  |  |  |  |  |                   |            /       |
   +--+--+--+--+--+--+                   |         /          |
   |  |  |  |  |  |  |                   |      /             |
   +--+--+--+--+--+--+                   |   /                |
   |  |  |  |  |  |  |                   |/                   |
   +--+--+--+--+--+--+                   +--------------------+

   24 cells to search                    2 triangles to search
```

The triangulator is a faithful Rust port of Jonathan Shewchuk's
Triangle 1.6, restricted to the constrained-Delaunay subset (see the crate
header at [`crates/triangle/src/lib.rs`](../crates/triangle/src/lib.rs)). Its
internals — the divide-and-conquer sweep, the flip machinery, circumcircle
predicates — are deliberately not documented in this set. You do not need them
to use the library, and the module headers already own that material.

---

## The four stages of a build

`build_navmesh_from_bitfield` is four stages in a row. Here they are in
data-flow order, with the toy grid from
[`crates/polygon-extract/examples/grid_to_polygons.rs`](../crates/polygon-extract/examples/grid_to_polygons.rs)
carried through all four. Run it yourself:

```
cargo run -p rsnav-polygon-extract --example grid_to_polygons
```

### 1. A grid of booleans

`#` is walkable, `.` is wall — **the inverse of the quickstart's map**, which
drew walls as `#`. Neither spelling is the library's: the ASCII is a fixture of
each example's own parser, and only `true` = walkable is normative
([04](04-units-and-conventions.md)). Read the art below as a walkable slab with
three holes punched through it, not as a room inside a border. (The example's
`grid()` helper takes rows top-down for readability and flips them; the grid's
own row 0 is the bottom row. Coordinate conventions are owned by
[04-units-and-conventions.md](04-units-and-conventions.md) and are not
restated here.)

```
   ############
   ###.....#.##
   ###.....#.##
   ############
   ############
   ####....####
   ############
   ............
```

This is 12 x 8 = 96 cells, 68 of them walkable.

### 2. Extraction traces the boundary into rings

The extractor walks the border between walkable and non-walkable cells and
emits closed rings: one **outer ring** per connected walkable area, plus one
ring per enclosed obstacle — a **hole**. The interior is discarded; only the
outline survives.

```
   +----------------------+          outer ring: 4 vertices, area 84.0
   |    +-----+   +--+    |          hole 0:     4 vertices, area  4.0
   |    |     |   |  |    |          hole 1:     4 vertices, area 10.0
   |    +-----+   +--+    |          hole 2:     4 vertices, area  2.0
   |                      |
   |      +------+        |          => 84.0 - (4.0 + 10.0 + 2.0) = 68.0
   |      |      |        |             which is the walkable cell count
   +------+------+--------+
```

That is the example's real printed output. The polygon area exactly equals the
walkable cell count, and 68 cells collapsed into 4 rings of 4 vertices each.
(The all-wall bottom row is why the outer ring covers 84 and not 96.)

### 3. Rings are cut into triangles

The rings — outer boundary plus holes — are handed to the triangulator, which
fills the space between them with triangles. The ring edges become
**constrained edges**: edges the triangulation is required to contain, and which
mark where the walls are.

```
   +--------+--------+          Every edge drawn on the outline is
   | \      | \      |          constrained (a wall). Every edge drawn
   |   \    |   \    |          inside is unconstrained (a portal).
   |     \  |     \  |
   +--------+--------+          The hole in the middle has no triangles:
   | \      |########|          it was carved away.
   |   \    |########|
   |     \  |########|
   +--------+--------+
```

### 4. Neighbours make a graph

Two triangles that share an edge are graph **neighbours** if that shared edge is
not constrained. That single rule turns the pile of triangles into a graph you
can search. A constrained edge is a dead end; an unconstrained shared edge is a
door between two cells of the graph.

```
   T0 --- T1 --- T2        (unconstrained shared edges = graph edges)
    |      |
   T3 --- T4  ###          (### = the hole; no triangle, no neighbour)
```

The result is a `NavMesh`: vertices, triangles, per-triangle neighbour indices,
per-edge markers, and region labels. Plus a BVH over the triangles, built at
the end of the same call, used for point lookup.

---

## The runtime model: locate, search, pull

Every query in this library is some subset of three stages. Later pages assume
you know which stage they are talking about.

### LOCATE — which triangle contains this point

A world position means nothing to the graph until it is resolved to a triangle.
That is a spatial index problem, and it is what the BVH in
[`crates/bsp/src/lib.rs`](../crates/bsp/src/lib.rs) is for: average `O(log n)`
to find the triangle containing a point, or `None` if the point is outside the
mesh entirely. (The crate is named `bsp` for historical reasons; the structure
is a bounding-volume hierarchy, not a BSP tree.)

A click, a spawn point, an AI target — all of them start here.

### SEARCH — A\* over triangle adjacency

With a start triangle and a goal triangle, A\* runs over the neighbour graph
from stage 4. It is ordinary A\* with a Euclidean heuristic; what it returns is
not a path but a **corridor** — an ordered list of triangles from start to
goal, each one sharing an unconstrained edge with the next.

```
   start                                     goal
     *                                         *
   +----+----+----+----+----+
   | T0 | T1 | T2 | T3 | T4 |     corridor = [T0, T1, T2, T3, T4]
   +----+----+----+----+----+
        ^    ^    ^    ^
        portals crossed
```

The corridor says *which triangles*, not *where in them*. It is a channel, not
a route.

### PULL — the funnel turns a corridor into a polyline

The shared edge between two consecutive corridor triangles is a **portal**: a
segment the path must cross, but it may cross it anywhere along its length.
The funnel algorithm (Simple Stupid Funnel; see
[`crates/navigation/src/funnel.rs`](../crates/navigation/src/funnel.rs)) walks
the portal sequence and pulls the route taut, producing the shortest polyline
through that corridor. It bends only where geometry forces it to.

```
   corridor of triangles           after the funnel

   +----+----+----+----+           *---------------.
   |    |    |    |    |            \               \
   |    |    |    |    |             \               \
   +----+----+----+----+              *---------------*
```

This is why the quickstart's path had two segments and not one per triangle.
From [`crates/navigation/examples/find_path.rs`](../crates/navigation/examples/find_path.rs)
— a 4x4 square with a 1x1 hole, 8 triangles — the route from (0.5, 0.5) to
(3.5, 3.5) is:

```
    (0.500, 0.500)
    (1.500, 2.500)     <- the hole's corner; the only place it must bend
    (3.500, 3.500)
    total length: 4.472
```

The corridor it was pulled from has more triangles than that path has corners.
The corner at (1.500, 2.500) is exactly on the obstacle, which is the point at
which [clearance](06-clearance.md) becomes your problem.

`find_path` does all three stages in one call. The stages are separable and
each is separately public, which matters once you are doing anything other than
one path at a time — see [07-paths-and-queries.md](07-paths-and-queries.md).

---

## Five words, defined exactly

Later pages use these as load-bearing terms.

**Triangle.** One cell of the navmesh, holding three vertex indices, three
neighbour indices (or an invalid sentinel at a boundary), three edge markers,
its area, its centroid, and its region. Triangles are not uniform: a triangle
may cover 0.5 square units or 400,000. Triangle count is not a resolution knob
and more triangles does not mean a more accurate path.

**Portal.** The shared edge between two adjacent triangles — an edge you may
cross. A portal has length, and that length is what query-time clearance
operates on: `PathOptions::distance_from_wall` shifts each portal endpoint that
sits on a *wall vertex* inward along the portal, narrowing the window the path
may use. Endpoints that are not wall vertices are not shifted, so a short
portal is not by itself a barrier. It defaults to `0.0`, and even when set, a
portal too short for both shifts collapses to its midpoint rather than being
refused by the funnel — A\* is what declines to route through it. See
[06-clearance.md](06-clearance.md).

**Constrained edge.** An edge that came from an input ring rather than from
free triangulation, carrying a non-zero marker (`edge_markers[i] != 0`). A
constrained edge is a wall: A\* will not cross it, line-of-sight stops at it,
and the funnel treats its endpoints as corners to pull around. That is a
query-layer rule, not a topology one — the `neighbors[i]` field still points at
the triangle across a constrained edge when one survived hole carving (it is
`TriangleId::INVALID` only at the mesh boundary), so the wall test is
`!neighbors[i].is_valid() || edge_markers[i] != 0`, not the neighbour link
alone. A marker of `0` means the edge
is interior and unconstrained. Which non-zero value appears is a build-time
choice; see [04-units-and-conventions.md](04-units-and-conventions.md).

**Corridor.** The ordered triangle sequence A\* returns — the channel a path
runs through, before the funnel decides where inside it the path goes. A
corridor is not a path and its length is unrelated to the path's point count.

**Region.** A connected component of the triangle graph, computed at build
time by flood-filling across unconstrained shared edges only
([`crates/navmesh/src/build.rs:128`](../crates/navmesh/src/build.rs)). Two
triangles carry the same `region` label if and only if you can walk from one to
the other. `NavMesh::reachable(a, b)` is literally `region(a) == region(b)`,
and `region_count` is how many disconnected walkable areas the mesh has.

Region deserves the extra care because the word invites two wrong readings:

- A region is **not a room, zone, or area of interest.** It is purely a
  reachability class. A 40-room mansion where every door is an open portal is
  one region. If you want named areas, attach your own data per triangle — see
  [09-doors-and-navworld.md](09-doors-and-navworld.md).
- A region is **not affected by anything at runtime.** It is baked in at build
  time from constrained edges. A closed door does not change it, so a
  door-sealed goal still passes the `reachable` pre-check and fails later.
  Two meshes merged with `NavMesh::append` never share a region even if they
  touch geometrically ([12-large-worlds.md](12-large-worlds.md)).

The toy grid above is a single region: all 68 walkable cells are mutually
reachable, and the three holes are obstacles inside it, not separate regions.
`testdata/act3-town.pbm` builds to 130 regions — 130 areas with no walkable
connection between them.

---

## Which pipeline is yours

There are two ways into a navmesh, and they diverge at the very first stage.

**Your walkable space is a grid** — a tilemap, an image, a thresholded
heightmap, procedural noise. You have a `Bitfield` and the four stages above
are your build. Go to [05-building-navmeshes.md](05-building-navmeshes.md).

**Your walkable space is authored vector geometry** — a level editor, a CAD
export, hand-drawn collision contours. You already have rings, so stage 2 does
not apply and stages 3 and 4 are reached by a different entry point, one that
tolerates input rings that cross each other. Go to
[13-authored-geometry.md](13-authored-geometry.md). Most readers never need
this page.

Both converge on the same `NavMesh` and the same runtime model, so everything
after this point is shared.

Next, in order: [04-units-and-conventions.md](04-units-and-conventions.md),
because a coordinate convention you get wrong is silent; then
[06-clearance.md](06-clearance.md), because the path above runs through the
obstacle's corner.
