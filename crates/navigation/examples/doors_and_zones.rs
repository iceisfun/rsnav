//! Doors and zones behind a [`NavWorld`].
//!
//! Two rooms joined by a one-cell doorway. The example walks the whole
//! runtime-gating surface, in the order the docs teach it:
//!
//!   1. `NavWorld` owns nav + bsp + doors + walls; every query is door-aware
//!      with no plumbing and no per-call `WallInfo` rebuild.
//!   2. A door authored from a drawn segment (`add_door`) gates the doorway.
//!   3. The same crossing gated unambiguously via `nearest_portal_edge` +
//!      `add_door_edge`.
//!   4. An INERT door: the authoring segment crossed no toggleable portal,
//!      `edge_count() == 0`, and `add_door` reported no error.
//!   5. `generation()` as the repath trigger.
//!   6. A two-zone `NavMetadata` impl, `zone_at`, and `zone_crossings` —
//!      including the workaround for the never-emitted starting zone.
//!
//! Run: `cargo run -p rsnav-navigation --example doors_and_zones`

use rsnav_common::{TriangleId, Vertex};
use rsnav_dynamic::{build_navmesh_from_bitfield, BuildOptions};
use rsnav_navigation::{
    nearest_portal_edge, DoorState, LineOfSightResult, NavMetadata, NavWorld, PathOptions,
};
use rsnav_navmesh::NavMesh;
use rsnav_polygon_extract::Bitfield;

/// Rows are written top-down for legibility and flipped on load: the
/// `Bitfield`'s row 0 is the BOTTOM row. `#` is wall, `.` is walkable.
const MAP: [&str; 12] = [
    "########################",
    "#........#.............#",
    "#........#.............#",
    "#........#.............#",
    "#........#.............#",
    "#........#.............#",
    "#......................#",
    "#......................#",
    "#........#.............#",
    "#........#.............#",
    "#........#.............#",
    "########################",
];

fn load_map() -> Bitfield {
    let h = MAP.len() as u32;
    let w = MAP[0].len() as u32;
    let mut data = vec![false; (w * h) as usize];
    for (r, line) in MAP.iter().enumerate() {
        let row = h as usize - 1 - r; // top-down text -> bottom-up grid
        for (c, ch) in line.chars().enumerate() {
            data[row * w as usize + c] = ch == '.';
        }
    }
    Bitfield::new(w, h, data).expect("map rows are all the same width")
}

// =========================================================================
// Metadata: the left room is "hall", the right room is "tavern".
// =========================================================================

/// `value_at` returns a BORROW, so anything a store reports must already
/// live somewhere. Here the per-triangle labels are materialised once at
/// construction and handed out by reference.
struct Rooms {
    labels: Vec<&'static str>,
}

impl Rooms {
    /// Classify every triangle by its centroid. The dividing wall sits on
    /// column 9, so its world x is 9.0..10.0; 9.5 splits the rooms.
    fn new(nav: &NavMesh) -> Self {
        Self {
            labels: nav
                .triangles
                .iter()
                .map(|t| if t.centroid.x < 9.5 { "hall" } else { "tavern" })
                .collect(),
        }
    }
}

impl NavMetadata for Rooms {
    type Zone = &'static str;
    type Value = &'static str;

    fn zone(&self, tri: TriangleId) -> Option<&'static str> {
        self.labels.get(tri.index()).copied()
    }

    fn value_at(&self, tri: TriangleId, _p: Vertex) -> Option<&&'static str> {
        self.labels.get(tri.index())
    }
}

fn main() {
    let bf = load_map();
    let build = build_navmesh_from_bitfield(&bf, &BuildOptions::default())
        .expect("the map has a walkable perimeter");
    println!(
        "navmesh: {} triangles, {} regions",
        build.navmesh.triangles.len(),
        build.navmesh.region_count
    );

    let rooms = Rooms::new(&build.navmesh);
    // NavWorld takes the mesh by value and builds its own Bsp and WallInfo.
    let mut world = NavWorld::new(build.navmesh, rooms);

    let start = Vertex::new(4.0, 5.0); // hall
    let goal = Vertex::new(18.0, 5.0); // tavern
    let opts = PathOptions::default();

    // -- 1. Baseline: the doorway is open ------------------------------
    let path = world
        .find_path(start, goal, &opts)
        .expect("rooms are joined by the doorway");
    let planned_at = world.generation();
    println!(
        "\n[open]   path: {} points, {} triangles, generation {}",
        path.points.len(),
        path.triangles.len(),
        planned_at
    );
    println!("[open]   line_of_sight: {:?}", world.line_of_sight(start, goal));

    // -- 2. A door authored from a drawn segment -----------------------
    // A vertical stroke across the doorway at x = 9.5, y = 3..7.
    let seg_door = world.add_door(
        Vertex::new(9.5, 3.0),
        Vertex::new(9.5, 7.0),
        DoorState::Closed,
    );
    let cut = world.doors().get(seg_door).unwrap().edge_count();
    println!("\n[segment door] cut {cut} portal edge(s)");
    assert!(cut > 0, "the authoring segment must cross a portal");

    println!("[closed] find_path: {:?}", world.find_path(start, goal, &opts));
    println!("[closed] line_of_sight: {:?}", world.line_of_sight(start, goal));

    // The mesh never changed — only the wall oracle did.
    println!(
        "[closed] mesh still has {} triangles, {} regions (regions ignore doors)",
        world.nav().triangles.len(),
        world.nav().region_count
    );

    // -- 3. generation() drives the repath -----------------------------
    let now = world.generation();
    println!(
        "\n[repath] planned at generation {planned_at}, world is at {now} -> {}",
        if now != planned_at { "stale" } else { "current" }
    );
    println!(
        "[repath] path_clear on the old polyline: {}",
        world.path_clear(&path.points)
    );

    world.open_door(seg_door);
    println!(
        "[reopen] find_path ok again: {}",
        world.find_path(start, goal, &opts).is_ok()
    );
    // Reopening is itself a change, so the counter keeps climbing; it is a
    // change counter, not a state hash.
    println!("[reopen] generation {}", world.generation());

    world.remove_door(seg_door);

    // -- 4. The same crossing, gated by picking an edge ----------------
    // "Pick the portal under the cursor": unambiguous, exactly one edge.
    let cursor = Vertex::new(9.5, 5.0);
    let (va, vb) = nearest_portal_edge(world.nav(), world.bsp(), cursor)
        .expect("an internal portal edge in the doorway");
    let edge_door = world.add_door_edge(va, vb, DoorState::Closed);
    println!(
        "\n[edge door] gates {} edge; find_path now: {:?}",
        world.doors().get(edge_door).unwrap().edge_count(),
        world.find_path(start, goal, &opts).map(|p| p.points.len())
    );
    world.remove_door(edge_door);

    // -- 5. An inert door ----------------------------------------------
    // Drawn across the WALL rather than across the opening. It crosses no
    // toggleable portal, so it does nothing — and add_door still returns a
    // DoorId with no error. Checking edge_count() is the caller's job.
    let inert = world.add_door(
        Vertex::new(9.5, 0.5),
        Vertex::new(9.5, 1.5),
        DoorState::Closed,
    );
    println!(
        "\n[inert]  edge_count = {} (door does nothing, no error was reported)",
        world.doors().get(inert).unwrap().edge_count()
    );
    println!(
        "[inert]  find_path still ok: {}",
        world.find_path(start, goal, &opts).is_ok()
    );
    world.remove_door(inert);

    // -- 6. Zones -------------------------------------------------------
    let path = world.find_path(start, goal, &opts).unwrap();
    println!("\n[zones]  zone_at(start) = {:?}", world.zone_at(start));
    println!("[zones]  zone_at(goal)  = {:?}", world.zone_at(goal));
    println!("[zones]  metadata_at(start) = {:?}", world.metadata_at(start));
    println!(
        "[zones]  metadata_at(off-mesh) = {:?} (indistinguishable from 'no value here')",
        world.metadata_at(Vertex::new(-5.0, -5.0))
    );

    // zone_crossings reports TRANSITIONS only. The starting zone is never
    // emitted; recover it from the first triangle of the A* channel.
    let opening = world.meta().zone(path.triangles[0]);
    println!("[zones]  starting zone (from path.triangles[0]) = {opening:?}");
    for c in world.zone_crossings(&path) {
        println!(
            "[zones]  {:?} -> {:?} at ({:.2}, {:.2})",
            c.from, c.into, c.point.x, c.point.y
        );
    }
    println!(
        "[zones]  note: crossing points are portal midpoints on the unpulled\n\
         [zones]        A* corridor, so they need not lie on path.points."
    );

    // LOS through the reopened doorway, for completeness.
    match world.line_of_sight(start, goal) {
        LineOfSightResult::Clear => println!("\n[final]  line of sight is clear"),
        other => println!("\n[final]  line of sight: {other:?}"),
    }
}
