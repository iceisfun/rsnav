//! Regression: the scene captured from the demo where a concave C-shape
//! hole made `carve_holes` invert walkable / unwalkable.
//!
//! Failure mode (pre-fix): the demo seeded each hole with the polygon's
//! arithmetic centroid. For the big C-shaped hole below the centroid
//! lands *outside* the C (inside the walkable area). carve_holes then
//! flood-filled the walkable area instead of the hole, leaving a tiny
//! ~26-triangle island. After the fix (Polygon::interior_point's ear-
//! based seed) the same scene yields a substantial walkable navmesh.

use rsnav_common::{Polygon, Vertex};
use rsnav_navmesh::build_from_cdt;
use rsnav_triangle::pslg::{Pslg, PslgHole, PslgSegment, PslgVertex};
use rsnav_triangle::{carve_holes, delaunay, form_skeleton, CdtMesh, DivConqOptions, VertexSlot};

const PERIMETER: &[(f64, f64)] = &[
    (96.71875, 53.257667541503906),
    (515.8316650390625, 22.765527725219727),
    (948.6207275390625, 23.227319717407227),
    (1159.6558837890625, 11.752416610717773),
    (1540.489501953125, 41.45001220703125),
    (1530.098876953125, 390.11041259765625),
    (1451.4019775390625, 694.1659545898438),
    (1508.349609375, 866.7415161132812),
    (1507.727294921875, 985.0538940429688),
    (1440.22265625, 1019.298828125),
    (1287.7972412109375, 1020.724365234375),
    (1138.669189453125, 980.8052978515625),
    (1047.154296875, 1022.4189453125),
    (959.53125, 1023.9764404296875),
    (931.4534912109375, 940.4892578125),
    (972.1539306640625, 871.5235595703125),
    (1103.3980712890625, 816.7947998046875),
    (1206.2445068359375, 751.72998046875),
    (1218.046875, 669.8062133789062),
    (1198.0543212890625, 583.1365966796875),
    (1000.7425537109375, 556.7381591796875),
    (796.780029296875, 550.5831909179688),
    (697.28515625, 756.0361328125),
    (691.9019775390625, 930.2848510742188),
    (653.7437744140625, 1036.2906494140625),
    (368.4765625, 1018.8941650390625),
    (126.65194702148438, 1032.288818359375),
    (54.44451904296875, 945.5076293945312),
    (27.678131103515625, 721.3399658203125),
    (36.064056396484375, 485.62762451171875),
    (35.536712646484375, 301.4266357421875),
    (30.369537353515625, 169.69326782226562),
];

const HOLES: &[&[(f64, f64)]] = &[
    &[
        (1227.1898193359375, 919.6720581054688),
        (1402.666748046875, 935.3277587890625),
        (1436.6527099609375, 839.3518676757812),
        (1408.1304931640625, 770.4261474609375),
        (1373.843017578125, 672.0240478515625),
        (1416.833251953125, 512.1901245117188),
        (1439.1968994140625, 413.2657775878906),
        (1432.680419921875, 343.17083740234375),
        (1386.99609375, 282.0688781738281),
        (1300.430908203125, 297.7632141113281),
        (1227.832763671875, 325.751220703125),
        (1178.233642578125, 393.15008544921875),
        (1254.129638671875, 454.5335998535156),
        (1297.6956787109375, 518.248291015625),
        (1303.236328125, 630.0496826171875),
        (1319.370361328125, 746.5244140625),
        (1262.1976318359375, 826.4230346679688),
        (1193.6448974609375, 861.0186767578125),
    ],
    &[
        (197.76727294921875, 608.8704833984375),
        (146.29278564453125, 510.5064697265625),
        (167.75421142578125, 341.43231201171875),
        (269.74462890625, 283.44549560546875),
        (426.9664306640625, 298.89776611328125),
        (481.0911865234375, 417.392333984375),
        (433.58087158203125, 597.69677734375),
        (336.388671875, 635.1934814453125),
    ],
    &[
        (636.4515380859375, 228.02005004882812),
        (764.6031494140625, 98.6396255493164),
        (914.682861328125, 92.4375991821289),
        (996.841796875, 201.76473999023438),
        (940.341796875, 398.7813415527344),
        (811.84765625, 431.51031494140625),
        (678.5078125, 364.5113830566406),
    ],
    // The bad actor — concave C-shape whose arithmetic centroid lands
    // OUTSIDE the polygon. Pre-fix this seed point caused carve_holes
    // to consume the walkable area and leave only the hole interior.
    &[
        (222.67715454101562, 793.3591918945312),
        (310.74884033203125, 811.28955078125),
        (394.646484375, 775.07568359375),
        (457.2906494140625, 725.2399291992188),
        (528.6380615234375, 632.5850219726562),
        (546.2283325195312, 539.8514404296875),
        (549.9279174804688, 442.7733459472656),
        (544.9652099609375, 351.5472717285156),
        (514.0079956054688, 285.6241455078125),
        (474.0396728515625, 243.13510131835938),
        (436.89471435546875, 195.61196899414062),
        (428.0947265625, 130.5213623046875),
        (475.35858154296875, 95.41302490234375),
        (526.6546630859375, 105.63594055175781),
        (581.21875, 250.49398803710938),
        (613.5667724609375, 327.83697509765625),
        (639.533203125, 400.94415283203125),
        (670.0462646484375, 471.55035400390625),
        (638.4226684570312, 642.542724609375),
        (628.6570434570312, 743.3483276367188),
        (620.4921875, 870.26953125),
        (591.7955322265625, 942.5095825195312),
        (500.73126220703125, 957.1277465820312),
        (386.34588623046875, 957.1277465820312),
        (266.9659423828125, 952.3270263671875),
        (151.24697875976562, 918.1017456054688),
        (129.62118530273438, 848.09814453125),
        (122.27911376953125, 786.6417236328125),
    ],
];

fn to_polygon(pts: &[(f64, f64)]) -> Polygon {
    Polygon::from_vertices(pts.iter().map(|(x, y)| Vertex::new(*x, *y)))
}

#[test]
fn concave_c_shape_hole_does_not_invert_carve() {
    // Sanity: pre-fix, the C-shape's arithmetic centroid is OUTSIDE the
    // polygon. If this ever stops being true (e.g. someone "normalized"
    // the fixture coords), the regression isn't actually exercising the
    // failure mode anymore.
    let c_shape = to_polygon(HOLES[3]);
    let n = c_shape.vertices.len() as f64;
    let cx = c_shape.vertices.iter().map(|v| v.x).sum::<f64>() / n;
    let cy = c_shape.vertices.iter().map(|v| v.y).sum::<f64>() / n;
    assert!(
        !c_shape.contains(Vertex::new(cx, cy)),
        "fixture invariant broken: centroid is NOT outside the C-shape anymore"
    );

    // Build the full PSLG.
    let mut cdt = CdtMesh::new();
    let mut pslg = Pslg::new();
    let mut next_idx = 0u32;
    let mut push_ring = |verts: &[(f64, f64)], marker: i32, cdt: &mut CdtMesh, pslg: &mut Pslg, next_idx: &mut u32| {
        let start = *next_idx;
        for (x, y) in verts {
            cdt.push_vertex(VertexSlot::new(Vertex::new(*x, *y), 0));
            pslg.vertices.push(PslgVertex::new(Vertex::new(*x, *y)));
            *next_idx += 1;
        }
        let n = verts.len() as u32;
        for i in 0..n {
            pslg.segments.push(PslgSegment {
                a: start + i,
                b: start + (i + 1) % n,
                marker,
            });
        }
    };
    push_ring(PERIMETER, 1, &mut cdt, &mut pslg, &mut next_idx);
    for (i, hole) in HOLES.iter().enumerate() {
        push_ring(hole, 10 + i as i32, &mut cdt, &mut pslg, &mut next_idx);
    }

    // Hole seeds via the FIX — interior_point, NOT centroid.
    for hole in HOLES.iter() {
        let p = to_polygon(hole);
        let seed = p
            .interior_point()
            .expect("interior_point failed on a non-degenerate hole");
        assert!(
            p.contains(seed),
            "interior_point produced a point outside its hole: {:?}",
            seed
        );
        pslg.holes.push(PslgHole { point: seed });
    }

    delaunay(&mut cdt, DivConqOptions::default());
    form_skeleton(&mut cdt, &pslg, None).unwrap();
    carve_holes(&mut cdt, &pslg, false);
    let nav = build_from_cdt(&cdt);

    // Pre-fix this gave 26 triangles in 1 region (the C-shape interior).
    // Post-fix it produces ~99 triangles for the actual walkable area
    // (the gaps between the perimeter and the four holes). The exact
    // count depends on Delaunay flip choices for cocircular cases; use
    // 60 as a comfortable lower bound that catches inversions but
    // tolerates minor algorithmic changes.
    assert!(
        nav.triangle_count() > 60,
        "expected substantial walkable navmesh, got {} triangles \
         (carve_holes likely inverted again)",
        nav.triangle_count()
    );

    // A point that's clearly walkable: bottom-right of the perimeter,
    // outside every hole. Walk through every triangle and verify at least
    // one contains it.
    let known_walkable = Vertex::new(1400.0, 100.0); // top-right strip
    let in_perim = to_polygon(PERIMETER).contains(known_walkable);
    let in_any_hole = HOLES.iter().any(|h| to_polygon(h).contains(known_walkable));
    assert!(in_perim && !in_any_hole, "test point isn't actually walkable");
    let found = nav.triangles.iter().any(|t| {
        let a = nav.vertex(t.vertices[0]);
        let b = nav.vertex(t.vertices[1]);
        let c = nav.vertex(t.vertices[2]);
        point_in_tri(a, b, c, known_walkable)
    });
    assert!(
        found,
        "no navmesh triangle covers the known-walkable point {:?}",
        known_walkable
    );

    // And the C-shape hole's interior should NOT be covered (it's a hole).
    // Use interior_point itself to find a point guaranteed inside (the C
    // shape is concave so the centroid isn't reliable for picking one).
    let c_poly = to_polygon(HOLES[3]);
    let known_hole = c_poly
        .interior_point()
        .expect("interior_point should always find a point in the C-shape");
    assert!(
        c_poly.contains(known_hole),
        "interior_point gave a point not inside the C-shape"
    );
    let found = nav.triangles.iter().any(|t| {
        let a = nav.vertex(t.vertices[0]);
        let b = nav.vertex(t.vertices[1]);
        let c = nav.vertex(t.vertices[2]);
        point_in_tri(a, b, c, known_hole)
    });
    assert!(
        !found,
        "navmesh triangle covers a point INSIDE the C-shape hole {:?} \
         (carve_holes didn't remove it)",
        known_hole
    );
}

fn point_in_tri(a: Vertex, b: Vertex, c: Vertex, p: Vertex) -> bool {
    use rsnav_common::geom::orient2d;
    let d1 = orient2d(a, b, p);
    let d2 = orient2d(b, c, p);
    let d3 = orient2d(c, a, p);
    let has_neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
    let has_pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
    !(has_neg && has_pos)
}
