//! Visual debugger for the rsnav voxel pipeline.
//!
//! Phase 1 capability: load a synthetic mesh, render it with flat shading
//! and a real wireframe overlay, fly around with mouse + WASD/QE, switch
//! between fixtures via an egui side panel. As the pipeline stages come
//! online (voxelizer, walkability, watershed) this demo will gain
//! corresponding layer toggles.
//!
//! Camera:
//! - Drag left mouse: yaw + pitch
//! - Drag right mouse: pan
//! - Mouse wheel: dolly forward/back
//! - W/A/S/D: fly forward/left/back/right along view
//! - Q/E: descend/ascend along world Z
//!
//! World convention is Z-up.

mod obj;
mod pick;
mod stl;

use rsnav_common::Vec3;
use rsnav_voxel::{
    area_type, build_all_navmeshes, classify_walkability, densify_path_on_navmesh,
    extract_contours, extract_portals, find_path, find_region_at_xyz, funnel_in_region,
    region_mean_z, segment, synth, AllRegionNavMeshes, CompactHeightfield, MeshBuilder, Path,
    PolySoup, Portal, RegionContours, RegionId, RegionMap, Transform, VoxelGrid,
    WalkabilityConfig, WatershedConfig,
};
use three_d::{
    egui, radians, AmbientLight, Camera, ClearState, ColorMaterial, Context, CpuMaterial, CpuMesh,
    DirectionalLight, Event, FrameOutput, Gm, Indices, InnerSpace, InstancedMesh, Instances, Key,
    Mat4, Mesh, MouseButton, Object, PhysicalMaterial, PhysicalPoint, Positions, Srgba,
    SurfaceSettings, Window, WindowSettings, Wireframe, GUI,
};

/// Which endpoint the next viewport click should place.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum PickTarget {
    Src,
    Dst,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum SceneChoice {
    Plane,
    PlaneSubdivided,
    Ramp,
    FloorRampPlatform,
    StlLand,
    ObjNavTest,
    ObjDungeon,
    ObjUndulating,
}

impl SceneChoice {
    fn label(self) -> &'static str {
        match self {
            Self::Plane => "Plane (1 quad)",
            Self::PlaneSubdivided => "Plane (8×8 subdivided)",
            Self::Ramp => "Ramp",
            Self::FloorRampPlatform => "Floor + Ramp + Platform",
            Self::StlLand => "STL: ./Land/Land1.stl",
            Self::ObjNavTest => "OBJ: ./nav_test.obj (Recast)",
            Self::ObjDungeon => "OBJ: ./dungeon.obj (Recast)",
            Self::ObjUndulating => "OBJ: ./undulating.obj (Recast)",
        }
    }

    /// Whether this scene reads from disk and needs ImportOptions.
    fn is_imported(self) -> bool {
        matches!(
            self,
            Self::StlLand | Self::ObjNavTest | Self::ObjDungeon | Self::ObjUndulating
        )
    }

    /// Sensible defaults per imported scene — different source apps use
    /// different unit conventions, so we pre-seed reasonable values that
    /// the user can then tweak via the UI.
    fn default_import_options(self) -> ImportOptions {
        match self {
            Self::StlLand => ImportOptions {
                scale: 0.03,
                y_up_to_z_up: true,
                recenter_to_origin: true,
            },
            Self::ObjNavTest | Self::ObjDungeon | Self::ObjUndulating => ImportOptions {
                // Recast demo meshes are already in meters.
                scale: 1.0,
                y_up_to_z_up: true,
                recenter_to_origin: true,
            },
            _ => ImportOptions::default(),
        }
    }

    fn build(self, opts: &ImportOptions) -> PolySoup {
        match self {
            Self::Plane => synth::plane(10.0, 10.0, 1),
            Self::PlaneSubdivided => synth::plane(10.0, 10.0, 8),
            Self::Ramp => synth::ramp(0.0, 4.0, 2.0, 3.0),
            Self::FloorRampPlatform => synth::floor_with_ramp_and_platform(),
            Self::StlLand => load_stl_with_options("Land/Land1.stl", opts),
            Self::ObjNavTest => load_obj_with_options("nav_test.obj", opts),
            Self::ObjDungeon => load_obj_with_options("dungeon.obj", opts),
            Self::ObjUndulating => load_obj_with_options("undulating.obj", opts),
        }
    }
}

const SCENE_OPTIONS: [SceneChoice; 8] = [
    SceneChoice::Plane,
    SceneChoice::PlaneSubdivided,
    SceneChoice::Ramp,
    SceneChoice::FloorRampPlatform,
    SceneChoice::StlLand,
    SceneChoice::ObjNavTest,
    SceneChoice::ObjDungeon,
    SceneChoice::ObjUndulating,
];

/// Knobs applied to imported meshes (STL, OBJ) — we don't know the
/// source app's axis convention or unit scale, so the user picks.
#[derive(Copy, Clone, Debug, PartialEq)]
struct ImportOptions {
    /// Multiplied onto every vertex coord (useful when source is mm/cm).
    scale: f64,
    /// If true, rotate Y-up → Z-up (90° around X). Most CAD tools that
    /// export OBJ/STL default to Y-up; rsnav expects Z-up.
    y_up_to_z_up: bool,
    /// Translate so the loaded mesh's AABB min sits at (0, 0, 0)
    /// (after scale + axis fix). Easier to fly the camera to.
    recenter_to_origin: bool,
}

impl Default for ImportOptions {
    fn default() -> Self {
        // The Land1.stl from Autodesk is ~1072 × 40 × 815 source units;
        // 0.03 maps that to ~32 × 1.2 × 24 m which fits the target scale
        // the artist was given (16–32 m bounding region). Adjust via UI.
        Self {
            scale: 0.03,
            y_up_to_z_up: true,
            recenter_to_origin: true,
        }
    }
}

fn load_stl_with_options(path: &str, opts: &ImportOptions) -> PolySoup {
    match stl::load_binary_stl(path) {
        Ok(raw) => apply_import_options(&raw, opts),
        Err(e) => {
            eprintln!("failed to load {}: {}", path, e);
            // Fall back to a tiny visible marker so the demo doesn't crash.
            synth::box_aabb(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.5, 0.5, 0.5))
        }
    }
}

fn load_obj_with_options(path: &str, opts: &ImportOptions) -> PolySoup {
    match obj::load_obj(path) {
        Ok(raw) => apply_import_options(&raw, opts),
        Err(e) => {
            eprintln!("failed to load {}: {}", path, e);
            synth::box_aabb(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.5, 0.5, 0.5))
        }
    }
}

fn apply_import_options(soup: &PolySoup, opts: &ImportOptions) -> PolySoup {
    let mut t = Transform::uniform_scale(opts.scale);
    if opts.y_up_to_z_up {
        // +Y in source → +Z in our world: 90° rotation around X.
        t = t.then(Transform::rotation_x(std::f64::consts::FRAC_PI_2));
    }
    let mut b = MeshBuilder::new();
    b.add(soup, t);
    let mut out = b.build();
    if opts.recenter_to_origin {
        let bounds = out.bounds();
        if !bounds.is_empty() {
            let offset = -bounds.min;
            for v in &mut out.vertices {
                *v = *v + offset;
            }
        }
    }
    out
}

/// Convert a PolySoup into a three-d mesh with flat per-face normals.
/// Triangles are duplicated (3 verts per face) so each face gets a sharp
/// shade — necessary because PolySoup has no smoothing groups.
fn polysoup_to_cpu_mesh(soup: &PolySoup) -> CpuMesh {
    let mut positions: Vec<three_d::Vec3> = Vec::with_capacity(soup.triangle_count() * 3);
    let mut normals: Vec<three_d::Vec3> = Vec::with_capacity(soup.triangle_count() * 3);
    let mut indices: Vec<u32> = Vec::with_capacity(soup.triangle_count() * 3);

    for [a, b, c] in soup.triangle_positions() {
        let pa = three_d::vec3(a.x as f32, a.y as f32, a.z as f32);
        let pb = three_d::vec3(b.x as f32, b.y as f32, b.z as f32);
        let pc = three_d::vec3(c.x as f32, c.y as f32, c.z as f32);
        let n = (pb - pa).cross(pc - pa);
        let n = if n.magnitude() > 0.0 {
            n.normalize()
        } else {
            three_d::vec3(0.0, 0.0, 1.0)
        };
        let base = positions.len() as u32;
        positions.push(pa);
        positions.push(pb);
        positions.push(pc);
        normals.push(n);
        normals.push(n);
        normals.push(n);
        indices.push(base);
        indices.push(base + 1);
        indices.push(base + 2);
    }

    CpuMesh {
        positions: Positions::F32(positions),
        indices: Indices::U32(indices),
        normals: Some(normals),
        ..Default::default()
    }
}

/// Color a voxel by its area type. Walkable triangles paint cells green,
/// non-walkable (walls, steep slopes, ceilings) stay grey.
fn voxel_color(area: u8) -> Srgba {
    match area {
        area_type::WALKABLE => Srgba::new_opaque(70, 200, 90),
        area_type::SOLID => Srgba::new_opaque(140, 140, 150),
        _ => Srgba::new_opaque(255, 0, 255), // empty should never reach here
    }
}

/// HSV → sRGB conversion. h in degrees [0, 360), s and v in [0, 1].
fn hsv_to_srgba(h: f32, s: f32, v: f32) -> Srgba {
    let h = h.rem_euclid(360.0);
    let c = v * s;
    let x = c * (1.0 - ((h / 60.0) % 2.0 - 1.0).abs());
    let m = v - c;
    let (r, g, b) = match (h / 60.0) as u32 {
        0 => (c, x, 0.0),
        1 => (x, c, 0.0),
        2 => (0.0, c, x),
        3 => (0.0, x, c),
        4 => (x, 0.0, c),
        _ => (c, 0.0, x),
    };
    Srgba::new_opaque(
        ((r + m) * 255.0).round().clamp(0.0, 255.0) as u8,
        ((g + m) * 255.0).round().clamp(0.0, 255.0) as u8,
        ((b + m) * 255.0).round().clamp(0.0, 255.0) as u8,
    )
}

/// Distinct color per region ID using golden-ratio hue spread. Unassigned
/// (filtered) cells return a dim grey so they're visually distinct from
/// real regions.
fn region_color(rid: RegionId) -> Srgba {
    if rid == RegionId::INVALID {
        return Srgba::new_opaque(80, 80, 80);
    }
    let phi: f32 = 0.618_033_99;
    let hue = (rid.0 as f32 * phi).fract() * 360.0;
    hsv_to_srgba(hue, 0.65, 0.92)
}

/// Build a voxel grid from the scene + voxel size + slope threshold, plus
/// an InstancedMesh of cubes (one per occupied cell, colored by area type).
/// Cubes are scaled to 90% of the cell size so adjacent voxels are visually
/// distinguishable.
fn build_voxel_model(
    ctx: &Context,
    soup: &PolySoup,
    cell_size: f64,
    cos_max_slope: f64,
) -> (VoxelGrid, Gm<InstancedMesh, ColorMaterial>) {
    let grid = VoxelGrid::from_polysoup(soup, cell_size, 1, cos_max_slope);
    let half_scale = (cell_size * 0.45) as f32; // cube primitive is side 2
    let occupied = grid.occupied_count();
    let mut transformations = Vec::with_capacity(occupied);
    let mut colors = Vec::with_capacity(occupied);
    for (c, r, l, area) in grid.iter_with_area() {
        let center = grid.cell_center(c, r, l);
        transformations.push(
            Mat4::from_translation(three_d::vec3(
                center.x as f32,
                center.y as f32,
                center.z as f32,
            )) * Mat4::from_scale(half_scale),
        );
        colors.push(voxel_color(area));
    }
    let instances = Instances {
        transformations,
        colors: Some(colors),
        ..Default::default()
    };
    let cube_cpu = CpuMesh::cube();
    let mut material = ColorMaterial {
        color: Srgba::WHITE,
        ..Default::default()
    };
    material.render_states.cull = three_d::Cull::Back;
    let mesh = InstancedMesh::new(ctx, &instances, &cube_cpu);
    (grid, Gm::new(mesh, material))
}

/// How walkable-surface tiles are colored in the demo.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum TileColoring {
    /// Solid green tint, brightness scaled by neighbor connectivity.
    Connectivity,
    /// Distinct color per region (from watershed).
    Region,
}

/// Construct the model matrix that turns three-d's default cylinder
/// (X-aligned, x in [0, 1], radius 1 in YZ) into a thin "edge tube"
/// from world point `a` to world point `b`.
fn edge_transform(a: three_d::Vec3, b: three_d::Vec3, radius: f32) -> Mat4 {
    let delta = b - a;
    let length = delta.magnitude();
    if length < 1e-6 {
        return Mat4::from_scale(0.0);
    }
    let dir = delta / length;
    let x_axis = three_d::vec3(1.0_f32, 0.0, 0.0);
    let identity = Mat4::from_scale(1.0);
    let rot = if (dir - x_axis).magnitude() < 1e-6 {
        identity
    } else if (dir + x_axis).magnitude() < 1e-6 {
        Mat4::from_angle_z(three_d::degrees(180.0))
    } else {
        let axis = x_axis.cross(dir).normalize();
        let cos_angle = x_axis.dot(dir).clamp(-1.0, 1.0);
        let angle = cos_angle.acos();
        Mat4::from_axis_angle(axis, three_d::radians(angle))
    };
    Mat4::from_translation(a) * rot * Mat4::from_nonuniform_scale(length, radius, radius)
}

/// Build a 3D triangle mesh from all region navmeshes, colored per
/// region. Triangles are lifted slightly in Z so they sit above the
/// flat walkable-surface tiles (which sit just above the source mesh).
/// Returns (filled, wireframe) — render both for thick edge outlines.
fn build_navmesh_model(
    ctx: &Context,
    navmeshes: &AllRegionNavMeshes,
    cell_size: f64,
) -> (Gm<Mesh, ColorMaterial>, Wireframe) {
    let lift = (cell_size * 0.18) as f32; // above tile + contour layers
    let mut positions: Vec<three_d::Vec3> = Vec::new();
    let mut colors: Vec<Srgba> = Vec::new();
    let mut indices: Vec<u32> = Vec::new();

    for (i, mesh_opt) in navmeshes.meshes.iter().enumerate() {
        let Some(mesh) = mesh_opt else { continue };
        let rid = RegionId(i as u32);
        let color = region_color(rid);
        // Duplicate vertices per triangle so each face gets its own
        // flat shade and we can color uniformly per region without any
        // shared-vertex bleeding.
        for tri in &mesh.triangles {
            for &vi in tri {
                let v = mesh.vertices[vi as usize];
                indices.push(positions.len() as u32);
                positions.push(three_d::vec3(v.x as f32, v.y as f32, v.z as f32 + lift));
                colors.push(color);
            }
        }
    }

    let cpu_mesh = CpuMesh {
        positions: Positions::F32(positions),
        indices: Indices::U32(indices),
        colors: Some(colors),
        ..Default::default()
    };
    let mut material = ColorMaterial {
        color: Srgba::WHITE,
        ..Default::default()
    };
    material.render_states.cull = three_d::Cull::None;
    let filled = Gm::new(Mesh::new(ctx, &cpu_mesh), material);
    // Thin dark wireframe overlay on the CDT triangles.
    let wire = Wireframe::new_from_cpu_mesh(ctx, &cpu_mesh, 1.0, Srgba::new_opaque(20, 20, 30));
    (filled, wire)
}

/// Build an instanced-cylinder model showing portal polylines between
/// regions. Each portal is colored by a hash of its (a, b) pair so
/// adjacent portals look distinct.
fn build_portals_model(
    ctx: &Context,
    portals: &[Portal],
    cell_size: f64,
) -> Gm<InstancedMesh, ColorMaterial> {
    let radius = (cell_size * 0.08) as f32;
    let lift = (cell_size * 0.22) as f32; // above contour lines
    let mut transformations: Vec<Mat4> = Vec::new();
    let mut colors: Vec<Srgba> = Vec::new();
    for portal in portals {
        // Color seeded by canonical (a, b) so the same portal is the same color.
        let seed = portal.a.0.wrapping_mul(2654435761).wrapping_add(portal.b.0);
        let hue = ((seed as f32 * 0.61803399).fract()) * 360.0;
        let color = hsv_to_srgba(hue, 0.85, 0.95);
        for i in 0..portal.edge.len().saturating_sub(1) {
            let a = portal.edge[i];
            let b = portal.edge[i + 1];
            let a3 = three_d::vec3(a.x as f32, a.y as f32, a.z as f32 + lift);
            let b3 = three_d::vec3(b.x as f32, b.y as f32, b.z as f32 + lift);
            transformations.push(edge_transform(a3, b3, radius));
            colors.push(color);
        }
    }
    let cyl = CpuMesh::cylinder(8);
    let instances = Instances {
        transformations,
        colors: Some(colors),
        ..Default::default()
    };
    let mut material = ColorMaterial {
        color: Srgba::WHITE,
        ..Default::default()
    };
    material.render_states.cull = three_d::Cull::Back;
    Gm::new(InstancedMesh::new(ctx, &instances, &cyl), material)
}

/// Render a path as a chain of short yellow cylinder segments connecting
/// successive dense waypoints. Also draws small markers at src and dst.
fn build_path_model(
    ctx: &Context,
    dense_waypoints: &[Vec3],
    src: Vec3,
    dst: Vec3,
    cell_size: f64,
) -> Gm<InstancedMesh, ColorMaterial> {
    let radius = (cell_size * 0.13) as f32;
    let lift = (cell_size * 0.35) as f32; // above everything else
    let mut transformations: Vec<Mat4> = Vec::new();
    let mut colors: Vec<Srgba> = Vec::new();

    // Marker at src (green) and dst (red): a small upright cylinder.
    push_marker(
        &src,
        lift * 4.0,
        radius * 2.5,
        Srgba::new_opaque(50, 230, 50),
        &mut transformations,
        &mut colors,
    );
    push_marker(
        &dst,
        lift * 4.0,
        radius * 2.5,
        Srgba::new_opaque(230, 50, 50),
        &mut transformations,
        &mut colors,
    );

    for i in 0..dense_waypoints.len().saturating_sub(1) {
        let a = dense_waypoints[i];
        let b = dense_waypoints[i + 1];
        let a3 = three_d::vec3(a.x as f32, a.y as f32, a.z as f32 + lift);
        let b3 = three_d::vec3(b.x as f32, b.y as f32, b.z as f32 + lift);
        transformations.push(edge_transform(a3, b3, radius));
        colors.push(Srgba::new_opaque(250, 220, 30)); // bright yellow
    }

    let cyl = CpuMesh::cylinder(8);
    let instances = Instances {
        transformations,
        colors: Some(colors),
        ..Default::default()
    };
    let mut material = ColorMaterial {
        color: Srgba::WHITE,
        ..Default::default()
    };
    material.render_states.cull = three_d::Cull::Back;
    Gm::new(InstancedMesh::new(ctx, &instances, &cyl), material)
}

/// Helper: a vertical cylinder used as a "marker" at a world position.
fn push_marker(
    pos: &Vec3,
    height: f32,
    radius: f32,
    color: Srgba,
    transformations: &mut Vec<Mat4>,
    colors: &mut Vec<Srgba>,
) {
    let a = three_d::vec3(pos.x as f32, pos.y as f32, pos.z as f32);
    let b = three_d::vec3(pos.x as f32, pos.y as f32, pos.z as f32 + height);
    transformations.push(edge_transform(a, b, radius));
    colors.push(color);
}

/// For slider-driven src/dst input, snap Z to the mean walkable Z of
/// the region whose XY contour contains (x, y). Uses Z-aware lookup
/// so 2.5D overlaps (lower floor under upper deck) pick the region
/// closest in Z to `fallback_z` — slide horizontally and the marker
/// "sticks" to whichever floor matches the current Z.
///
/// **Not** used by the click-to-pick handler — that path already has
/// the real surface Z from the raycast and should not be mean-snapped.
fn snap_to_region(contours: &RegionContours, x: f64, y: f64, fallback_z: f64) -> Vec3 {
    if let Some(rid) = find_region_at_xyz(contours, x, y, fallback_z) {
        let z = region_mean_z(&contours.contours[rid.index()]);
        Vec3::new(x, y, z)
    } else {
        Vec3::new(x, y, fallback_z)
    }
}

/// Run A* and build the visualization model in one shot. The path is
/// densified against the heightfield so it visually follows real surface
/// Z (ramps, steps) instead of jumping between portal midpoints. The
/// dense polyline is then lifted by `lift` so it draws above the
/// walkable surface (use the agent clearance — the line then visualizes
/// the actual head-height an agent of that clearance would trace).
///
/// For same-region paths the coarse [`find_path`] result is refined
/// with the within-region funnel ([`funnel_in_region`]) so the line
/// routes around holes in the region's CDT (e.g., stairwell cutouts,
/// pillars) instead of clipping straight through them.
fn compute_and_build_path(
    ctx: &Context,
    contours: &RegionContours,
    portals: &[Portal],
    _chf: &CompactHeightfield,
    _region_map: &RegionMap,
    navmeshes: &AllRegionNavMeshes,
    src: Vec3,
    dst: Vec3,
    cell_size: f64,
    lift: f64,
) -> (Option<Path>, Gm<InstancedMesh, ColorMaterial>) {
    let path = find_path(contours, portals, src, dst).map(|p| refine_with_funnel(p, navmeshes));
    let dense: Vec<Vec3> = match path.as_ref() {
        Some(p) => densify_path_on_navmesh(p, navmeshes, cell_size)
            .into_iter()
            .map(|v| Vec3::new(v.x, v.y, v.z + lift))
            .collect(),
        None => Vec::new(),
    };
    let lifted_src = Vec3::new(src.x, src.y, src.z + lift);
    let lifted_dst = Vec3::new(dst.x, dst.y, dst.z + lift);
    let model = build_path_model(ctx, &dense, lifted_src, lifted_dst, cell_size);
    (path, model)
}

/// For pure same-region paths (`regions.len() == 1`), replace the coarse
/// `[src, dst]` straight line with a funneled polyline that routes
/// around holes in the region's CDT navmesh. Cross-region paths are
/// returned unchanged — they need a cross-region funnel pass, which is
/// a follow-up (the per-region segments between portal midpoints still
/// clip through intra-region holes today).
fn refine_with_funnel(path: Path, navmeshes: &AllRegionNavMeshes) -> Path {
    if path.regions.len() != 1 || path.waypoints.len() != 2 {
        return path;
    }
    let rid = path.regions[0];
    let mesh = match navmeshes.meshes.get(rid.index()).and_then(|m| m.as_ref()) {
        Some(m) => m,
        None => return path,
    };
    let src = path.waypoints[0];
    let dst = path.waypoints[1];
    let funneled = match funnel_in_region(mesh, (src.x, src.y), (dst.x, dst.y)) {
        Some(p) if p.len() >= 2 => p,
        _ => return path,
    };
    let waypoints: Vec<Vec3> = funneled
        .into_iter()
        .map(|(x, y)| Vec3::new(x, y, src.z))
        .collect();
    let regions = vec![rid; waypoints.len() - 1];
    Path { waypoints, regions }
}

/// Render region contours as thin instanced-cylinder edges, lifted
/// slightly above the walkable tiles so they're visible on top.
fn build_contour_model(
    ctx: &Context,
    contours: &RegionContours,
) -> Gm<InstancedMesh, ColorMaterial> {
    let cs = contours.cell_size;
    let radius = (cs * 0.04) as f32; // 4% of voxel size
    let lift = (cs * 0.12) as f32; // sit above tile by ~10% of voxel size
    let mut transformations: Vec<Mat4> = Vec::new();
    let mut colors: Vec<Srgba> = Vec::new();

    for contour in &contours.contours {
        // Outer ring (dark outline)
        push_loop(&contour.outer, lift, radius, Srgba::new_opaque(10, 10, 10), &mut transformations, &mut colors);
        // Holes (magenta — rare, helps spot them)
        for hole in &contour.holes {
            push_loop(hole, lift, radius, Srgba::new_opaque(220, 60, 220), &mut transformations, &mut colors);
        }
    }

    // Empty instances handled — three-d accepts zero instances.
    let cyl = CpuMesh::cylinder(6);
    let instances = Instances {
        transformations,
        colors: Some(colors),
        ..Default::default()
    };
    let mut material = ColorMaterial {
        color: Srgba::WHITE,
        ..Default::default()
    };
    material.render_states.cull = three_d::Cull::Back;
    Gm::new(InstancedMesh::new(ctx, &instances, &cyl), material)
}

fn push_loop(
    verts: &[rsnav_voxel::ContourVertex],
    lift: f32,
    radius: f32,
    color: Srgba,
    transformations: &mut Vec<Mat4>,
    colors: &mut Vec<Srgba>,
) {
    if verts.len() < 2 {
        return;
    }
    for i in 0..verts.len() {
        let a = verts[i];
        let b = verts[(i + 1) % verts.len()];
        let a3 = three_d::vec3(a.xy.x as f32, a.xy.y as f32, a.z as f32 + lift);
        let b3 = three_d::vec3(b.xy.x as f32, b.xy.y as f32, b.z as f32 + lift);
        transformations.push(edge_transform(a3, b3, radius));
        colors.push(color);
    }
}

/// Build the walkability classifier output + watershed region map plus
/// an InstancedMesh of thin tiles visualizing the walkable surface,
/// colored per `tile_coloring`.
fn build_walkable_model(
    ctx: &Context,
    grid: &VoxelGrid,
    walk_config: &WalkabilityConfig,
    ws_config: &WatershedConfig,
    tile_coloring: TileColoring,
) -> (
    CompactHeightfield,
    RegionMap,
    Gm<InstancedMesh, ColorMaterial>,
) {
    let chf = classify_walkability(grid, walk_config);
    let region_map = segment(&chf, ws_config);
    let cs = grid.cell_size as f32;
    let half_xy = cs * 0.5 * 0.95;
    let half_z = 0.01_f32;
    let walkable_count = chf.walkable_count();
    let mut transformations = Vec::with_capacity(walkable_count);
    let mut colors = Vec::with_capacity(walkable_count);
    for (c, r, si, cell) in chf.iter() {
        let surface = chf
            .surface_point(c, r, si)
            .expect("walkable cell has surface");
        transformations.push(
            Mat4::from_translation(three_d::vec3(
                surface.x as f32,
                surface.y as f32,
                surface.z as f32 + half_z,
            )) * Mat4::from_nonuniform_scale(half_xy, half_xy, half_z),
        );
        let color = match tile_coloring {
            TileColoring::Connectivity => {
                let n = cell.neighbors.iter().filter(|n| n.is_some()).count() as f32;
                let brightness = 90 + (n * 35.0) as u8; // 90..=230
                Srgba::new_opaque(40, brightness, 60)
            }
            TileColoring::Region => region_color(region_map.at(c, r, si)),
        };
        colors.push(color);
    }
    let instances = Instances {
        transformations,
        colors: Some(colors),
        ..Default::default()
    };
    let cube_cpu = CpuMesh::cube();
    let mut material = ColorMaterial {
        color: Srgba::WHITE,
        ..Default::default()
    };
    material.render_states.cull = three_d::Cull::Back;
    let mesh = InstancedMesh::new(ctx, &instances, &cube_cpu);
    (chf, region_map, Gm::new(mesh, material))
}

fn build_scene_models(
    ctx: &Context,
    soup: &PolySoup,
) -> (Gm<Mesh, PhysicalMaterial>, Wireframe) {
    let cpu_mesh = polysoup_to_cpu_mesh(soup);

    let mut solid_material = PhysicalMaterial::new_opaque(
        ctx,
        &CpuMaterial {
            albedo: Srgba::new_opaque(180, 180, 195),
            roughness: 0.85,
            metallic: 0.0,
            ..Default::default()
        },
    );
    solid_material.render_states.cull = three_d::Cull::None;
    let solid = Gm::new(Mesh::new(ctx, &cpu_mesh), solid_material);

    let wireframe = Wireframe::new_from_cpu_mesh(
        ctx,
        &cpu_mesh,
        /* wire_width pixels */ 1.5,
        Srgba::new_opaque(30, 30, 30),
    );

    (solid, wireframe)
}

/// FPS-style camera controls: WASD fly along view, Space/E ascend, Q descend.
/// Mouse drag (left button) rotates the camera with standard WoW/FPS
/// conventions — mouse-left makes the camera turn left (scene moves right),
/// mouse-down makes the camera look down (scene moves up).
#[derive(Default)]
struct FpsKeys {
    forward: bool,
    back: bool,
    left: bool,
    right: bool,
    up: bool,
    down: bool,
}

impl FpsKeys {
    fn handle_events(&mut self, events: &mut [Event]) {
        for event in events {
            match event {
                Event::KeyPress { kind, handled, .. } => {
                    if Self::apply(self, *kind, true) {
                        *handled = true;
                    }
                }
                Event::KeyRelease { kind, handled, .. } => {
                    if Self::apply(self, *kind, false) {
                        *handled = true;
                    }
                }
                _ => {}
            }
        }
    }

    fn apply(&mut self, key: Key, down: bool) -> bool {
        match key {
            Key::W => {
                self.forward = down;
                true
            }
            Key::S => {
                self.back = down;
                true
            }
            Key::A => {
                self.left = down;
                true
            }
            Key::D => {
                self.right = down;
                true
            }
            // Both Space and E ascend (Space matches FPS muscle memory).
            Key::Space | Key::E => {
                self.up = down;
                true
            }
            Key::Q => {
                self.down = down;
                true
            }
            _ => false,
        }
    }

    /// Integrate one frame's worth of movement onto `camera`.
    /// `dt_seconds` and `speed` are world units / second.
    fn step(&self, camera: &mut Camera, dt_seconds: f32, speed: f32) {
        let view = camera.view_direction();
        let right = camera.right_direction();
        let world_up = three_d::vec3(0.0, 0.0, 1.0);
        let mut delta = three_d::vec3(0.0, 0.0, 0.0);
        if self.forward {
            delta += view;
        }
        if self.back {
            delta -= view;
        }
        if self.right {
            delta += right;
        }
        if self.left {
            delta -= right;
        }
        if self.up {
            delta += world_up;
        }
        if self.down {
            delta -= world_up;
        }
        if delta.magnitude() > 0.0 {
            camera.translate(delta.normalize() * speed * dt_seconds);
        }
    }
}

/// Mouse-look with standard FPS conventions.
/// - Left-button drag yaws + pitches the camera.
/// - Mouse moves LEFT → camera turns LEFT (scene moves RIGHT).
/// - Mouse moves DOWN → camera looks DOWN (scene moves UP).
/// - Right-button drag pans (camera-relative).
/// - Mouse wheel dollies along view direction.
struct MouseLook {
    /// World units per pixel of pan / pixel of dolly.
    pan_speed: f32,
}

impl MouseLook {
    fn new() -> Self {
        Self { pan_speed: 0.08 }
    }

    fn handle_events(&self, camera: &mut Camera, events: &mut [Event]) {
        const YAW_PER_PX: f32 = std::f32::consts::PI / 900.0;
        const PITCH_PER_PX: f32 = std::f32::consts::PI / 900.0;
        for event in events.iter_mut() {
            match event {
                Event::MouseMotion {
                    delta,
                    button,
                    handled,
                    ..
                } if !*handled => {
                    if Some(MouseButton::Left) == *button {
                        // three-d's camera.yaw() rotates CCW around the
                        // up axis for positive angles; with up = +Z, a
                        // positive yaw moves the view to the LEFT.
                        // delta.0 > 0 means mouse moved RIGHT, which in
                        // standard FPS should turn the camera RIGHT
                        // (positive yaw is left, so we need NEGATIVE).
                        camera.yaw(radians(-delta.0 * YAW_PER_PX));
                        // delta.1 > 0 means mouse moved DOWN. Standard
                        // FPS: mouse down → camera looks down. three-d's
                        // pitch with positive angle looks UP, so negate.
                        camera.pitch(radians(-delta.1 * PITCH_PER_PX));
                        *handled = true;
                    } else if Some(MouseButton::Right) == *button {
                        let right = camera.right_direction();
                        let up = right.cross(camera.view_direction());
                        camera.translate(
                            -right * delta.0 * self.pan_speed + up * delta.1 * self.pan_speed,
                        );
                        *handled = true;
                    }
                }
                Event::MouseWheel { delta, handled, .. } if !*handled => {
                    let v = camera.view_direction() * self.pan_speed * delta.1;
                    camera.translate(v);
                    *handled = true;
                }
                _ => {}
            }
        }
    }
}

fn main() {
    // MSAA defaults to 4, which some Linux GL drivers refuse on the default
    // visual and the call panics with BadAttribute. Disable MSAA — the debug
    // viewer doesn't need it, and we can re-enable later behind a flag if
    // someone wants smoother edges.
    let window = Window::new(WindowSettings {
        title: "rsnav voxel-demo".to_string(),
        initial_size: Some((1280, 800)),
        // No max_size → user can resize/maximize the window freely.
        // Frame-input viewport updates each frame, so render scales.
        surface_settings: SurfaceSettings {
            multisamples: 0,
            ..Default::default()
        },
        ..Default::default()
    })
    .expect("create window");
    let ctx = window.gl();

    let mut camera = Camera::new_perspective(
        window.viewport(),
        three_d::vec3(14.0, -14.0, 10.0), // eye
        three_d::vec3(0.0, 0.0, 1.0),     // target (slightly above origin)
        three_d::vec3(0.0, 0.0, 1.0),     // Z is up
        three_d::degrees(45.0),
        0.1,
        500.0,
    );
    let mouse_control = MouseLook::new();
    let mut fps_keys = FpsKeys::default();

    let mut scene = SceneChoice::FloorRampPlatform;
    let mut import_opts = ImportOptions::default();
    let mut current_soup: PolySoup = scene.build(&import_opts);
    let (mut solid_model, mut wireframe_model) = build_scene_models(&ctx, &current_soup);
    let mut voxel_cell_size: f64 = 0.2;
    let mut walkability = WalkabilityConfig::default();
    let mut watershed = WatershedConfig::default();
    let mut tile_coloring = TileColoring::Region;
    let mut max_slope_deg: f32 = walkability.max_slope_rad.to_degrees() as f32;
    let (mut voxel_grid, mut voxel_model) = build_voxel_model(
        &ctx,
        &current_soup,
        voxel_cell_size,
        walkability.cos_max_slope(),
    );
    let (mut compact_hf, mut region_map, mut walkable_model) = build_walkable_model(
        &ctx,
        &voxel_grid,
        &walkability,
        &watershed,
        tile_coloring,
    );
    let mut region_contours = extract_contours(&compact_hf, &region_map);
    let mut contour_model = build_contour_model(&ctx, &region_contours);
    let mut region_navmeshes = build_all_navmeshes(&region_contours);
    let (mut navmesh_model, mut navmesh_wire) =
        build_navmesh_model(&ctx, &region_navmeshes, voxel_cell_size);
    let mut portals = extract_portals(&region_contours);
    let mut portals_model = build_portals_model(&ctx, &portals, voxel_cell_size);
    let mut path_src = Vec3::new(-3.0, 0.0, 0.0);
    let mut path_dst = Vec3::new(8.0, 0.0, 1.6);
    let (mut current_path, mut path_model) = compute_and_build_path(
        &ctx,
        &region_contours,
        &portals,
        &compact_hf,
        &region_map,
        &region_navmeshes,
        path_src,
        path_dst,
        voxel_cell_size,
        walkability.min_clearance,
    );
    let mut show_solid = true;
    let mut show_wireframe = false;
    let mut show_voxels = false;
    let mut show_walkable = true;
    let mut show_contours = true;
    let mut show_navmesh = false;
    let mut show_portals = true;
    let mut show_path = true;
    let mut fly_speed: f32 = 5.0;
    let mut pending_pick: Option<PickTarget> = None;
    let mut show_pick_debug: bool = false;
    let mut cursor_pos: Option<PhysicalPoint> = None;
    let mut last_pick_hit: Option<Vec3> = None;
    // Re-positioned each frame by mutating its transformation matrix.
    let pick_debug_cpu = CpuMesh::sphere(16);
    let mut pick_debug_material = ColorMaterial {
        color: Srgba::new_opaque(255, 230, 30),
        ..Default::default()
    };
    pick_debug_material.render_states.cull = three_d::Cull::Back;
    let mut pick_debug_model = Gm::new(Mesh::new(&ctx, &pick_debug_cpu), pick_debug_material);
    // Default off-screen scale; set each frame when the hover is valid.
    pick_debug_model.set_transformation(Mat4::from_scale(0.0));

    let ambient = AmbientLight::new(&ctx, 0.45, Srgba::WHITE);
    let directional1 = DirectionalLight::new(
        &ctx,
        0.7,
        Srgba::WHITE,
        three_d::vec3(-0.5, -0.4, -0.8).normalize(),
    );
    let directional2 = DirectionalLight::new(
        &ctx,
        0.3,
        Srgba::new_opaque(180, 200, 255),
        three_d::vec3(0.6, 0.3, -0.4).normalize(),
    );

    let mut gui = GUI::new(&ctx);

    window.render_loop(move |mut frame_input| {
        let mut scene_changed = false;
        let mut voxel_changed = false;
        let mut walkable_changed = false;
        gui.update(
            &mut frame_input.events,
            frame_input.accumulated_time,
            frame_input.viewport,
            frame_input.device_pixel_ratio,
            |egui_ctx| {
                #[allow(deprecated)] // see voxel-demo: top-level Panel::show is fine in egui 0.34
                egui::Panel::left("controls")
                    .resizable(false)
                    .default_size(260.0)
                    .show(egui_ctx, |ui| {
                        ui.heading("voxel-demo");
                        ui.separator();
                        ui.label("Scene");
                        for opt in SCENE_OPTIONS {
                            if ui.selectable_label(scene == opt, opt.label()).clicked()
                                && scene != opt
                            {
                                scene = opt;
                                scene_changed = true;
                                voxel_changed = true;
                                if scene.is_imported() {
                                    import_opts = scene.default_import_options();
                                }
                            }
                        }
                        if scene.is_imported() {
                            ui.indent("import_opts", |ui| {
                                if ui
                                    .add(
                                        egui::Slider::new(&mut import_opts.scale, 0.001..=10.0)
                                            .text("scale")
                                            .logarithmic(true),
                                    )
                                    .changed()
                                {
                                    scene_changed = true;
                                    voxel_changed = true;
                                }
                                if ui
                                    .checkbox(&mut import_opts.y_up_to_z_up, "Y-up → Z-up")
                                    .changed()
                                {
                                    scene_changed = true;
                                    voxel_changed = true;
                                }
                                if ui
                                    .checkbox(
                                        &mut import_opts.recenter_to_origin,
                                        "Recenter to origin",
                                    )
                                    .changed()
                                {
                                    scene_changed = true;
                                    voxel_changed = true;
                                }
                                let b = voxel_grid.world_bounds();
                                let size = b.size();
                                ui.label(format!(
                                    "bounds: ({:.2}, {:.2}, {:.2}) m",
                                    size.x, size.y, size.z
                                ));
                            });
                        }
                        ui.separator();
                        ui.label("Layers");
                        ui.checkbox(&mut show_solid, "Solid mesh");
                        ui.checkbox(&mut show_wireframe, "Wireframe overlay");
                        ui.checkbox(&mut show_voxels, "Voxel grid (green=walk, grey=solid)");
                        ui.checkbox(&mut show_walkable, "Walkable surfaces");
                        ui.checkbox(&mut show_contours, "Region contours");
                        ui.checkbox(&mut show_navmesh, "Navmesh (CDT, 3D)");
                        ui.checkbox(&mut show_portals, "Portals");
                        ui.checkbox(&mut show_path, "Path");
                        ui.checkbox(&mut show_pick_debug, "Pick debug (hover sphere)")
                            .on_hover_text(
                                "Casts a ray each frame from the cursor and draws a yellow \
                                 sphere where it hits the mesh. Use to verify the picker is \
                                 aimed where you think it is.",
                            );
                        if show_pick_debug {
                            match last_pick_hit {
                                Some(h) => {
                                    ui.label(format!(
                                        "cursor hit: ({:.3}, {:.3}, {:.3})",
                                        h.x, h.y, h.z
                                    ));
                                }
                                None => {
                                    ui.label("cursor hit: (miss)");
                                }
                            }
                        }
                        ui.separator();
                        ui.label("Voxelization");
                        if ui
                            .add(
                                egui::Slider::new(&mut voxel_cell_size, 0.05..=1.0)
                                    .text("voxel size (m)")
                                    .logarithmic(true),
                            )
                            .changed()
                        {
                            voxel_changed = true;
                        }
                        if ui
                            .add(
                                egui::Slider::new(&mut max_slope_deg, 5.0..=89.0)
                                    .text("max slope (°)"),
                            )
                            .changed()
                        {
                            walkability.max_slope_rad = (max_slope_deg as f64).to_radians();
                            voxel_changed = true;
                        }
                        ui.label(format!(
                            "{} × {} × {} cells   |   {} occupied   |   {} walkable",
                            voxel_grid.cols,
                            voxel_grid.rows,
                            voxel_grid.layers,
                            voxel_grid.occupied_count(),
                            voxel_grid.walkable_count(),
                        ));
                        ui.separator();
                        ui.label("Walkability");
                        if ui
                            .add(
                                egui::Slider::new(&mut walkability.min_clearance, 0.1..=4.0)
                                    .text("clearance (m)"),
                            )
                            .changed()
                        {
                            walkable_changed = true;
                        }
                        if ui
                            .add(
                                egui::Slider::new(&mut walkability.max_step_height, 0.05..=1.0)
                                    .text("max step (m)"),
                            )
                            .changed()
                        {
                            walkable_changed = true;
                        }
                        ui.label(format!(
                            "{} walkable cells across {} columns",
                            compact_hf.walkable_count(),
                            compact_hf.walkable_column_count(),
                        ));
                        ui.separator();
                        ui.label("Watershed");
                        if ui
                            .add(
                                egui::Slider::new(&mut watershed.min_region_cells, 0..=200)
                                    .text("min region cells"),
                            )
                            .changed()
                        {
                            walkable_changed = true;
                        }
                        if ui
                            .add(
                                egui::Slider::new(&mut watershed.max_layer_step, 0..=8)
                                    .text("max layer step in region"),
                            )
                            .on_hover_text(
                                "Cells whose layer differs by more than this become separate \
                                 regions (so stair treads triangulate flat instead of as a \
                                 diagonal slope). 1 = smooth ramps stay merged, stairs split.",
                            )
                            .changed()
                        {
                            walkable_changed = true;
                        }
                        ui.horizontal(|ui| {
                            ui.label("Tile color:");
                            if ui
                                .selectable_label(tile_coloring == TileColoring::Region, "region")
                                .clicked()
                            {
                                tile_coloring = TileColoring::Region;
                                walkable_changed = true;
                            }
                            if ui
                                .selectable_label(
                                    tile_coloring == TileColoring::Connectivity,
                                    "connectivity",
                                )
                                .clicked()
                            {
                                tile_coloring = TileColoring::Connectivity;
                                walkable_changed = true;
                            }
                        });
                        ui.label(format!(
                            "{} regions   |   {} navmesh tris across {} regions",
                            region_map.region_count,
                            region_navmeshes.total_triangle_count(),
                            region_navmeshes.built_count(),
                        ));
                        ui.separator();
                        ui.label("Camera");
                        ui.add(egui::Slider::new(&mut fly_speed, 0.5..=30.0).text("speed (m/s)"));
                        ui.separator();
                        ui.label(format!("{} portals", portals.len()));
                        ui.separator();
                        ui.label("Path");
                        // Slider ranges scale with the loaded mesh bounds so
                        // OBJ scenes (dungeon ≈ 60×60 m) are reachable too.
                        let bounds = voxel_grid.world_bounds();
                        let pad = 1.0_f64;
                        let xr = (bounds.min.x - pad)..=(bounds.max.x + pad);
                        let yr = (bounds.min.y - pad)..=(bounds.max.y + pad);
                        ui.horizontal(|ui| {
                            let src_btn = if pending_pick == Some(PickTarget::Src) {
                                ui.add(
                                    egui::Button::new("Click scene to place src")
                                        .fill(egui::Color32::from_rgb(60, 110, 60)),
                                )
                            } else {
                                ui.button("Pick src")
                            };
                            if src_btn.clicked() {
                                pending_pick = Some(PickTarget::Src);
                            }
                            let dst_btn = if pending_pick == Some(PickTarget::Dst) {
                                ui.add(
                                    egui::Button::new("Click scene to place dst")
                                        .fill(egui::Color32::from_rgb(110, 60, 60)),
                                )
                            } else {
                                ui.button("Pick dst")
                            };
                            if dst_btn.clicked() {
                                pending_pick = Some(PickTarget::Dst);
                            }
                        });
                        let mut path_changed = false;
                        path_changed |= ui
                            .add(egui::Slider::new(&mut path_src.x, xr.clone()).text("src x"))
                            .changed();
                        path_changed |= ui
                            .add(egui::Slider::new(&mut path_src.y, yr.clone()).text("src y"))
                            .changed();
                        path_changed |= ui
                            .add(egui::Slider::new(&mut path_dst.x, xr.clone()).text("dst x"))
                            .changed();
                        path_changed |= ui
                            .add(egui::Slider::new(&mut path_dst.y, yr.clone()).text("dst y"))
                            .changed();
                        match &current_path {
                            Some(p) => {
                                ui.label(format!(
                                    "{} waypoints across {} regions",
                                    p.waypoints.len(),
                                    p.regions.len()
                                ));
                            }
                            None => {
                                ui.label(
                                    egui::RichText::new("no path (endpoint off-mesh or unreachable)")
                                        .color(egui::Color32::from_rgb(220, 120, 80)),
                                );
                            }
                        }
                        if path_changed {
                            // Snap src/dst Z to their containing regions' mean Z.
                            let snapped_src = snap_to_region(
                                &region_contours,
                                path_src.x,
                                path_src.y,
                                path_src.z,
                            );
                            let snapped_dst = snap_to_region(
                                &region_contours,
                                path_dst.x,
                                path_dst.y,
                                path_dst.z,
                            );
                            path_src.z = snapped_src.z;
                            path_dst.z = snapped_dst.z;
                            let (p, m) = compute_and_build_path(
                                &ctx,
                                &region_contours,
                                &portals,
                                &compact_hf,
                                &region_map,
                                &region_navmeshes,
                                snapped_src,
                                snapped_dst,
                                voxel_cell_size,
                                walkability.min_clearance,
                            );
                            current_path = p;
                            path_model = m;
                        }
                        ui.separator();
                        ui.label(egui::RichText::new("Controls").strong());
                        ui.label("Drag L: look (FPS-style)");
                        ui.label("Drag R: pan");
                        ui.label("Wheel: dolly");
                        ui.label("WASD: fly");
                        ui.label("Space/E: up");
                        ui.label("Q: down");
                        ui.label("Pick src/dst → click on scene");
                    });
                let _ = egui_ctx.globally_used_rect();
            },
        );

        if scene_changed {
            current_soup = scene.build(&import_opts);
            let (s, w) = build_scene_models(&ctx, &current_soup);
            solid_model = s;
            wireframe_model = w;
        }
        if voxel_changed {
            // import_opts can change without scene_changed (sliders); rebuild
            // soup here too if scene_changed didn't already do it. Cheap to
            // re-do — bounded by scene triangle count.
            if !scene_changed {
                current_soup = scene.build(&import_opts);
            }
            let (g, m) = build_voxel_model(
                &ctx,
                &current_soup,
                voxel_cell_size,
                walkability.cos_max_slope(),
            );
            voxel_grid = g;
            voxel_model = m;
            walkable_changed = true;
        }
        if walkable_changed {
            let (chf, rm, m) = build_walkable_model(
                &ctx,
                &voxel_grid,
                &walkability,
                &watershed,
                tile_coloring,
            );
            compact_hf = chf;
            region_map = rm;
            walkable_model = m;
            region_contours = extract_contours(&compact_hf, &region_map);
            contour_model = build_contour_model(&ctx, &region_contours);
            region_navmeshes = build_all_navmeshes(&region_contours);
            let (m, w) = build_navmesh_model(&ctx, &region_navmeshes, voxel_cell_size);
            navmesh_model = m;
            navmesh_wire = w;
            portals = extract_portals(&region_contours);
            portals_model = build_portals_model(&ctx, &portals, voxel_cell_size);
            // Re-snap src/dst against new contours and re-path.
            let snapped_src =
                snap_to_region(&region_contours, path_src.x, path_src.y, path_src.z);
            let snapped_dst =
                snap_to_region(&region_contours, path_dst.x, path_dst.y, path_dst.z);
            path_src.z = snapped_src.z;
            path_dst.z = snapped_dst.z;
            let (p, m) = compute_and_build_path(
                &ctx,
                &region_contours,
                &portals,
                &compact_hf,
                &region_map,
                &region_navmeshes,
                snapped_src,
                snapped_dst,
                voxel_cell_size,
                walkability.min_clearance,
            );
            current_path = p;
            path_model = m;
        }

        camera.set_viewport(frame_input.viewport);

        // Track the latest cursor position from any mouse event with a
        // position field. Used by the pick-debug overlay below and by
        // the pick handler when arming.
        for event in frame_input.events.iter() {
            match event {
                Event::MouseMotion { position, .. }
                | Event::MousePress { position, .. }
                | Event::MouseRelease { position, .. }
                | Event::MouseWheel { position, .. } => {
                    cursor_pos = Some(*position);
                }
                _ => {}
            }
        }

        // Mouse-pick: when armed, consume the next non-egui left-click and
        // place src/dst at the ray's hit on the source mesh. Runs BEFORE
        // mouse_control so the pick can claim the press and mouse-look
        // doesn't fire a phantom yaw on the same down-event.
        if let Some(target) = pending_pick {
            let mut consumed = false;
            for event in frame_input.events.iter_mut() {
                if let Event::MousePress {
                    button: MouseButton::Left,
                    position,
                    handled,
                    ..
                } = event
                {
                    if *handled {
                        continue;
                    }
                    let pixel = *position;
                    let origin_f32 = camera.position_at_pixel(pixel);
                    let dir_f32 = camera.view_direction_at_pixel(pixel);
                    let origin = Vec3::new(
                        origin_f32.x as f64,
                        origin_f32.y as f64,
                        origin_f32.z as f64,
                    );
                    let dir = Vec3::new(
                        dir_f32.x as f64,
                        dir_f32.y as f64,
                        dir_f32.z as f64,
                    );
                    if let Some(hit) = pick::pick_polysoup(&current_soup, origin, dir) {
                        // Use the raycast hit directly — DON'T re-snap
                        // through `snap_to_region`. snap overrides Z
                        // with the matched region's mean Z, which on a
                        // 2.5D scene can land on the upper deck when
                        // the user clicked on the lower one. find_path
                        // disambiguates the region by Z internally.
                        match target {
                            PickTarget::Src => path_src = hit,
                            PickTarget::Dst => path_dst = hit,
                        }
                        let (p, m) = compute_and_build_path(
                            &ctx,
                            &region_contours,
                            &portals,
                            &compact_hf,
                            &region_map,
                            &region_navmeshes,
                            path_src,
                            path_dst,
                            voxel_cell_size,
                            walkability.min_clearance,
                        );
                        current_path = p;
                        path_model = m;
                    }
                    *handled = true;
                    consumed = true;
                    break;
                }
            }
            if consumed {
                pending_pick = None;
            }
        }

        mouse_control.handle_events(&mut camera, &mut frame_input.events);
        fps_keys.handle_events(&mut frame_input.events);
        fps_keys.step(
            &mut camera,
            (frame_input.elapsed_time as f32) / 1000.0,
            fly_speed,
        );

        // Pick-debug overlay: per-frame raycast from the cursor against
        // the source mesh. Updates `last_pick_hit` (shown in the UI next
        // frame) and re-positions the yellow debug sphere at the hit.
        last_pick_hit = None;
        if show_pick_debug {
            if let Some(pixel) = cursor_pos {
                let origin_f32 = camera.position_at_pixel(pixel);
                let dir_f32 = camera.view_direction_at_pixel(pixel);
                let origin = Vec3::new(
                    origin_f32.x as f64,
                    origin_f32.y as f64,
                    origin_f32.z as f64,
                );
                let dir = Vec3::new(
                    dir_f32.x as f64,
                    dir_f32.y as f64,
                    dir_f32.z as f64,
                );
                if let Some(hit) = pick::pick_polysoup(&current_soup, origin, dir) {
                    let radius = (voxel_cell_size as f32).max(0.05) * 0.6;
                    let pos = three_d::vec3(hit.x as f32, hit.y as f32, hit.z as f32);
                    pick_debug_model.set_transformation(
                        Mat4::from_translation(pos) * Mat4::from_scale(radius),
                    );
                    last_pick_hit = Some(hit);
                } else {
                    pick_debug_model.set_transformation(Mat4::from_scale(0.0));
                }
            }
        } else {
            pick_debug_model.set_transformation(Mat4::from_scale(0.0));
        }

        let screen = frame_input.screen();
        let target = screen.clear(ClearState::color_and_depth(0.07, 0.08, 0.10, 1.0, 1.0));

        let mut renderables: Vec<&dyn Object> = Vec::new();
        if show_solid {
            renderables.push(&solid_model);
        }
        if show_wireframe {
            renderables.push(&wireframe_model);
        }
        if show_voxels {
            renderables.push(&voxel_model);
        }
        if show_walkable {
            renderables.push(&walkable_model);
        }
        if show_contours {
            renderables.push(&contour_model);
        }
        if show_navmesh {
            renderables.push(&navmesh_model);
            renderables.push(&navmesh_wire);
        }
        if show_portals {
            renderables.push(&portals_model);
        }
        if show_path {
            renderables.push(&path_model);
        }
        if show_pick_debug && last_pick_hit.is_some() {
            renderables.push(&pick_debug_model);
        }
        target.render(&camera, renderables, &[&ambient, &directional1, &directional2]);

        let _ = target.write(|| gui.render());

        FrameOutput::default()
    });
}
