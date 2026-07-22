# Your first path in forty lines

A grid of booleans goes in, a polyline comes out. Nothing on this page requires
knowing what a triangulation is.

The whole page is one program:
[`crates/navigation/examples/first_path.rs`](../crates/navigation/examples/first_path.rs).
Inside a checkout of this repository, run it with:

```
cargo run -p rsnav-navigation --example first_path
```

## Using rsnav from your own project

rsnav is **not published to crates.io**, so `cargo add rsnav-navigation` will not
work. Depend on it by git instead. In your own `Cargo.toml`:

```toml
[dependencies]
rsnav-common          = { git = "https://github.com/iceisfun/rsnav" }
rsnav-polygon-extract = { git = "https://github.com/iceisfun/rsnav" }
rsnav-dynamic         = { git = "https://github.com/iceisfun/rsnav" }
rsnav-navigation      = { git = "https://github.com/iceisfun/rsnav" }
```

Pin a commit with `rev = "..."` if you want reproducible builds; the crates are
at `0.1.0` and the API is not yet stable.

Two naming rules that trip people up immediately:

- Package names are **hyphenated** (`rsnav-navigation`), but `use` paths are
  **underscored** (`use rsnav_navigation::find_path;`). Cargo does this
  translation for every crate; it is not an rsnav quirk.
- The crates are split by role, so a working program usually needs four of them:
  `common` for `Vertex`, `polygon-extract` for `Bitfield`, `dynamic` to build,
  `navigation` to query. [17-api-map.md](17-api-map.md) says which crate owns what.

Those four are all you need for everything on this page.

Five words are used throughout, one sentence each:

- **bitfield** — a grid of booleans marking which cells you can stand in.
- **navmesh** — that same walkable area stored as triangles instead of cells.
- **path** — a polyline from a start point to a goal point that stays inside it.
- **wall** — an edge of the navmesh you cannot cross.
- **agent radius** — how fat your character is. Named here, deferred entirely to
  [06-clearance.md](06-clearance.md).

## 1. The bitfield, and the one thing everybody gets wrong

`true` is walkable. `false` is **wall**. So
[`Bitfield::empty`](../crates/polygon-extract/src/lib.rs) is not an empty
map, it is a solid block of rock, and building from it fails:

```rust
let solid = Bitfield::empty(24, 14);
match build_navmesh_from_bitfield(&solid, &BuildOptions::default()) {
    Err(BuildError::NoPerimeter) => { /* all cells are wall */ }
    other => panic!("expected NoPerimeter, got {other:?}"),
}
```

This is the single most common first failure, and it is worth producing on
purpose once so the error is recognisable later. (The crate-level doctest in
`crates/dynamic/src/lib.rs` labels exactly this construction "a 32x32 walkable
map". That comment is wrong; the block is `no_run`, so it never executes.)

The example's real map is ASCII, written top row first because that is how a
human reads it:

```
########################
#.........######.......#
#.........######.......#
#.........######.......#
#.........######.......#
#.........######.......#
#.........######.......#
#.........######.......#
#.........######.......#
#.........######.......#
#....####.######.......#
#....#..#.######.......#
#....####..............#
########################
```

Row 0 of a `Bitfield` is the **bottom** row — y points up — while `MAP[0]` is
the top row as printed, so `ascii_to_bitfield` flips them on load:

```rust
let bitfield_row = height as usize - 1 - d;
```

That flip is load-bearing, and so is every other coordinate convention in this
library. They all live in one place:
[04-units-and-conventions.md](04-units-and-conventions.md). Read it before you
touch a single option.

## 2. Build the navmesh

```rust
let build = build_navmesh_from_bitfield(&bf, &BuildOptions::default())
    .expect("the map has walkable cells");
let nav = &build.navmesh;
let bsp = &build.bsp;
```

`NavBuild` ([`crates/dynamic/src/lib.rs:146`](../crates/dynamic/src/lib.rs))
holds four things: `navmesh` (the triangles), `bsp` (the index that answers
"which triangle contains this point"), `build_ms` (how long the build took), and
`generation` (a counter, always `0` unless a background worker produced the
build — see [10-dynamic-rebuilds.md](10-dynamic-rebuilds.md)).

For this 24x14 map it prints (the millisecond figure is machine-dependent;
everything else is deterministic):

```
navmesh: 12 triangles, 2 regions, built in 0.136 ms (generation 0)
```

Twelve triangles for 188 walkable cells. Triangle count tracks how complicated
the *boundary* is, not how large the area is.

Two **regions**, because the map has a sealed box on the lower left. A region is
a set of triangles reachable from each other without crossing a wall; it is not
a room, a zone, or anything you named.

## 3. Ask for a path

A cell `(col, row)` covers the square `[col, col+1] x [row, row+1]`, so the
centre of cell `(3, 6)` is `(3.5, 6.5)`.

```rust
let start = Vertex::new(3.5, 6.5);
let goal  = Vertex::new(20.5, 6.5);
let path  = find_path(nav, bsp, start, goal, &PathOptions::default())
    .expect("both endpoints are on the mesh and connected");
```

`PathResult::points` is the polyline. `points[0]` is *literally* your `start`
and `points.last()` is *literally* your `goal` — both are copied through
verbatim and are never adjusted for anything, not even clearance. A start
position already jammed against a wall stays jammed against it.

```
path: 5 points
  [0] (3.500, 6.500)
  [1] (9.000, 4.000)
  [2] (10.000, 2.000)
  [3] (16.000, 2.000)
  [4] (20.500, 6.500)
```

Rasterised back over the map, with `S`/`G` for the endpoints:

```
  ########################
  #.........######.......#
  #.........######.......#
  #.........######.......#
  #.........######.......#
  #.........######.......#
  #.........######.......#
  #..S*.....######....G..#
  #...***...######...*...#
  #......***######..*....#
  #....####*######.*.....#
  #....#..#*######*......#
  #....####..*****.......#
  ########################
```

The centre block reaches the top border, so the only route runs along the
bottom. The path bends at three corners and takes each of them tight — points
`[1]`, `[2]` and `[3]` are exactly on wall vertices. That is correct and it is
also the first thing you will want to change; see step 5.

`PathResult` also carries `triangles`, the raw sequence of triangles the search
passed through. It is a different length from `points` and the two must never be
zipped together.

The three ways this can fail, all produced deliberately in the example:

```
start off-mesh   -> StartOutsideMesh
goal off-mesh    -> GoalOutsideMesh
goal sealed off  -> Unreachable
```

`Unreachable` here is the sealed box: `(6.5, 2.5)` is walkable floor in a
region with no connection to the start. Note that `PathError` implements
neither `Display` nor `std::error::Error`, so it will not compose with `?` into
a boxed error ([07-paths-and-queries.md](07-paths-and-queries.md)).

## 4. The click that missed

A mouse click is a world position with no guarantee of landing on the mesh, so
the naive click-to-walk loop fails with `GoalOutsideMesh` constantly. Snap it
first:

```rust
let click = Vertex::new(21.5, 15.0);           // above the top of the map
let snapped = nearest_point(nav, bsp, click).expect("mesh is not empty");
let path = find_path(nav, bsp, start, snapped.point, &PathOptions::default())
    .expect("the snapped point is on the mesh");
```

```
click (21.5, 15.0) is off-mesh; snapped to (21.500, 13.000), 2.000 away
click-to-walk path: 5 points
```

`nearest_point` returns `None` only when the mesh is empty. It snaps to the
nearest point on the *surface*, which includes the inside edges of holes — a
click in the middle of a carved-out building snaps to that building's wall, not
to the open floor beyond it.

## 5. You are not done

Four things are true of the program above, in the order they will bite you.

- **The path hugs walls.** Points `[1]`–`[3]` sit exactly on wall corners,
  which is fine for a point and wrong for a character with a body. There are
  three different mechanisms for keeping an agent off walls, they are not
  interchangeable, and choosing wrong fails silently:
  [06-clearance.md](06-clearance.md). Do not reach for
  `PathOptions::distance_from_wall` before reading it — it is not a Euclidean
  clearance guarantee.
- **The polyline is not movement.** Turning `points` into a character that
  walks, without corner-shaving or jitter, is
  [08-moving-agents.md](08-moving-agents.md).
- **`find_path` is not free to call repeatedly.** Its body is two lines and it
  rebuilds an `O(triangles)` wall oracle on *every* call. Anything repeated — a
  crowd, a per-frame replan, a benchmark — must hoist that oracle or use
  `NavWorld`: [07-paths-and-queries.md](07-paths-and-queries.md).
- **Nothing here reacts to a changing world.** If your walls appear and
  disappear at runtime, the build belongs on a background worker:
  [10-dynamic-rebuilds.md](10-dynamic-rebuilds.md).

If you have never implemented pathfinding before, read
[02-concepts.md](02-concepts.md) next — it explains, retroactively, what the
twelve triangles were. If you have a working grid A* and want to know which of
your habits will mislead you here, read
[03-from-grid-astar.md](03-from-grid-astar.md) instead and skip 02 entirely.
Either way, [04-units-and-conventions.md](04-units-and-conventions.md) comes
before you change any option.
