//! CPU-side ray/polysoup intersection for click-to-place picking.
//!
//! Implements Möller-Trumbore against every triangle in the source
//! [`PolySoup`] and returns the nearest hit (in `t` along the ray).
//! The demo uses this on left-click to convert cursor → world point, so
//! the user can place path src/dst on the visible mesh surface.

use rsnav_common::Vec3;
use rsnav_voxel::PolySoup;

/// Cast a ray against every triangle in `soup` and return the nearest
/// world-space hit position (in front of `origin` along `dir`).
pub fn pick_polysoup(soup: &PolySoup, origin: Vec3, dir: Vec3) -> Option<Vec3> {
    let mut best_t = f64::INFINITY;
    let mut best_pos: Option<Vec3> = None;
    for [a, b, c] in soup.triangle_positions() {
        if let Some(t) = ray_triangle(origin, dir, a, b, c) {
            if t < best_t {
                best_t = t;
                best_pos = Some(origin + dir * t);
            }
        }
    }
    best_pos
}

/// Möller-Trumbore ray-triangle intersection. Returns the `t` distance
/// along the ray (always non-negative on hit), or `None` if missed.
fn ray_triangle(origin: Vec3, dir: Vec3, a: Vec3, b: Vec3, c: Vec3) -> Option<f64> {
    const EPS: f64 = 1e-9;
    let edge1 = b - a;
    let edge2 = c - a;
    let h = dir.cross(edge2);
    let det = edge1.dot(h);
    if det.abs() < EPS {
        return None;
    }
    let inv_det = 1.0 / det;
    let s = origin - a;
    let u = inv_det * s.dot(h);
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = s.cross(edge1);
    let v = inv_det * dir.dot(q);
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = inv_det * edge2.dot(q);
    if t < EPS {
        return None;
    }
    Some(t)
}

#[cfg(test)]
mod tests {
    use super::*;
    use rsnav_voxel::synth;

    #[test]
    fn ray_hits_plane_at_origin() {
        let soup = synth::plane(4.0, 4.0, 1);
        // Plane sits on z=0. Ray from above pointing straight down.
        let hit = pick_polysoup(
            &soup,
            Vec3::new(0.0, 0.0, 5.0),
            Vec3::new(0.0, 0.0, -1.0),
        )
        .expect("ray should hit the plane");
        assert!(hit.x.abs() < 1e-6);
        assert!(hit.y.abs() < 1e-6);
        assert!(hit.z.abs() < 1e-6);
    }

    #[test]
    fn ray_misses_plane() {
        let soup = synth::plane(4.0, 4.0, 1);
        let hit = pick_polysoup(
            &soup,
            Vec3::new(100.0, 100.0, 5.0),
            Vec3::new(0.0, 0.0, -1.0),
        );
        assert!(hit.is_none());
    }

    #[test]
    fn closest_hit_wins() {
        // Two stacked planes; ray from above should hit the upper one.
        let mut soup = synth::plane(4.0, 4.0, 1);
        let upper = synth::plane(4.0, 4.0, 1);
        let base = soup.vertices.len() as u32;
        for v in &upper.vertices {
            soup.vertices.push(Vec3::new(v.x, v.y, v.z + 2.0));
        }
        for t in &upper.triangles {
            soup.triangles.push([t[0] + base, t[1] + base, t[2] + base]);
        }
        let hit = pick_polysoup(
            &soup,
            Vec3::new(0.0, 0.0, 5.0),
            Vec3::new(0.0, 0.0, -1.0),
        )
        .expect("ray should hit");
        assert!((hit.z - 2.0).abs() < 1e-6);
    }
}
