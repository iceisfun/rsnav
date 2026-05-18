//! Path-following helpers for steering an agent along a polyline.
//!
//! [`PathFollower`] owns a polyline and tracks the agent's arc-length
//! progress. On each step the caller passes the agent's current position;
//! the follower projects it forward onto the path (never backtracking),
//! advances by a lookahead distance, and returns a steering target.
//!
//! With `corner_avoidance > 0` the target is biased outward at real
//! corners (turn angle > `corner_angle_threshold`) so the agent takes a
//! wider turn instead of cutting across an inside-corner wall — a fix for
//! the classic shortcutting failure mode.
//!
//! Pure geometric helpers; no navmesh dependency. Feed it any polyline
//! you like (typically the output of `rsnav_navigation::find_path`).

#![forbid(unsafe_code)]

use rsnav_common::{Vertex, geom};

/// Knobs that tune [`PathFollower::target`].
#[derive(Copy, Clone, Debug)]
pub struct FollowerOptions {
    /// Arc-length to look ahead from the agent's projected position.
    pub lookahead: f64,
    /// Maximum perpendicular bias applied at a corner. Zero disables
    /// anti-shortcutting and the follower just returns the linear
    /// lookahead point.
    pub corner_avoidance: f64,
    /// Turn-angle threshold (radians). Corners with smaller turn angles
    /// don't trigger anti-shortcutting — useful for ignoring numerical-
    /// noise corners on what should be a straight run. Suggested 0.1
    /// (~5.7°).
    pub corner_angle_threshold: f64,
}

impl Default for FollowerOptions {
    fn default() -> Self {
        Self {
            lookahead: 1.0,
            corner_avoidance: 0.0,
            corner_angle_threshold: 0.1,
        }
    }
}

#[derive(Clone, Debug)]
pub struct PathFollower {
    points: Vec<Vertex>,
    /// `cum_lengths[i]` = total path length from `points[0]` to `points[i]`.
    cum_lengths: Vec<f64>,
    total_length: f64,
    /// Latest projected arc-length of the agent. Monotonically non-
    /// decreasing across calls to [`target`](Self::target).
    arc: f64,
}

impl PathFollower {
    /// Wrap a polyline. Panics if `points` is empty.
    pub fn new(points: Vec<Vertex>) -> Self {
        assert!(!points.is_empty(), "PathFollower needs at least one point");
        let n = points.len();
        let mut cum = vec![0.0; n];
        for i in 1..n {
            cum[i] = cum[i - 1] + points[i - 1].distance(points[i]);
        }
        let total_length = cum[n - 1];
        Self {
            points,
            cum_lengths: cum,
            total_length,
            arc: 0.0,
        }
    }

    /// Total arc length of the wrapped polyline.
    #[inline]
    pub fn total_length(&self) -> f64 {
        self.total_length
    }

    /// Latest projected arc-length of the agent. `0` until the first
    /// [`target`](Self::target) call.
    #[inline]
    pub fn arc_length(&self) -> f64 {
        self.arc
    }

    /// Fraction of the path the agent has traversed. Returns `0.0` for
    /// zero-length paths.
    pub fn progress(&self) -> f64 {
        if self.total_length > 0.0 {
            self.arc / self.total_length
        } else {
            0.0
        }
    }

    /// `true` when the agent's projected arc has reached the path's end
    /// (within numerical tolerance).
    pub fn at_end(&self) -> bool {
        self.arc >= self.total_length - 1e-9
    }

    /// Project the agent forward onto the path and return a steering
    /// target `lookahead` arc-length ahead of that projection, biased
    /// outward at corners if `corner_avoidance > 0`.
    pub fn target(&mut self, agent_pos: Vertex, opts: &FollowerOptions) -> Vertex {
        self.arc = self.project_forward(agent_pos);

        let target_arc = (self.arc + opts.lookahead).min(self.total_length);
        let base = self.point_at_arc(target_arc);

        if opts.corner_avoidance <= 0.0 {
            return base;
        }
        self.apply_corner_avoidance(target_arc, base, opts)
    }

    // --- internals ----------------------------------------------------

    fn point_at_arc(&self, s: f64) -> Vertex {
        let s = s.clamp(0.0, self.total_length);
        let n = self.points.len();
        if n == 1 {
            return self.points[0];
        }
        // Linear scan from the segment containing the current arc — the
        // path is short by gameplay standards (tens to low hundreds of
        // vertices), so a binary search would be over-engineering.
        let mut k = 0;
        for i in 0..n - 1 {
            if s <= self.cum_lengths[i + 1] {
                k = i;
                break;
            }
            k = i + 1; // beyond the last segment (shouldn't happen post-clamp)
        }
        if k >= n - 1 {
            return self.points[n - 1];
        }
        let seg_len = self.cum_lengths[k + 1] - self.cum_lengths[k];
        if seg_len < 1e-12 {
            return self.points[k];
        }
        let t = (s - self.cum_lengths[k]) / seg_len;
        self.points[k].lerp(self.points[k + 1], t)
    }

    /// Find the point on the path closest to `agent_pos`. Searches a small
    /// window of segments around the current arc — far enough to recover
    /// from a brief detour, but not so far that a loopy path re-snaps to
    /// an old segment.
    fn project_forward(&self, agent_pos: Vertex) -> f64 {
        let n = self.points.len();
        if n == 1 {
            return 0.0;
        }
        let cur_seg = self.segment_for_arc(self.arc);
        // Allow one segment back (recover from a tiny overshoot) and search
        // forward to the end. The cost is O(remaining segments) per call;
        // fine for the path sizes we deal with.
        let from = cur_seg.saturating_sub(1);
        let mut best_arc = self.arc;
        let mut best_dist_sq = f64::INFINITY;
        for k in from..n - 1 {
            let a = self.points[k];
            let b = self.points[k + 1];
            let proj = geom::nearest_point_on_segment(a, b, agent_pos);
            let d_sq = (proj - agent_pos).length_sq();
            if d_sq < best_dist_sq {
                best_dist_sq = d_sq;
                let seg_len = self.cum_lengths[k + 1] - self.cum_lengths[k];
                let t = if seg_len > 1e-12 {
                    (proj - a).length() / seg_len
                } else {
                    0.0
                };
                best_arc = self.cum_lengths[k] + t * seg_len;
            }
        }
        // Monotone-forward: never lose ground if the projection wanders
        // backward (e.g. through a tight loop in the path).
        best_arc.max(self.arc)
    }

    fn segment_for_arc(&self, s: f64) -> usize {
        let n = self.points.len();
        for i in 0..n - 1 {
            if s <= self.cum_lengths[i + 1] {
                return i;
            }
        }
        n - 2
    }

    fn apply_corner_avoidance(
        &self,
        target_arc: f64,
        target: Vertex,
        opts: &FollowerOptions,
    ) -> Vertex {
        let n = self.points.len();
        let mut shifted = target;
        for i in 1..n - 1 {
            let v_arc = self.cum_lengths[i];
            let d = (target_arc - v_arc).abs();
            if d >= opts.corner_avoidance {
                continue;
            }
            let d_in = (self.points[i] - self.points[i - 1]).normalize_or_zero();
            let d_out = (self.points[i + 1] - self.points[i]).normalize_or_zero();
            if d_in == Vertex::ZERO || d_out == Vertex::ZERO {
                continue;
            }
            let cos_turn = d_in.dot(d_out).clamp(-1.0, 1.0);
            let turn_angle = cos_turn.acos();
            if turn_angle < opts.corner_angle_threshold {
                continue;
            }
            // Corner bisector (CW perp = outside for left turn, CCW perp =
            // outside for right turn).
            let d_avg = (d_in + d_out).normalize_or_zero();
            if d_avg == Vertex::ZERO {
                continue;
            }
            let cross = d_in.cross(d_out);
            let outside = if cross > 0.0 {
                // Left turn → outside is CW perp of d_avg.
                Vertex::new(d_avg.y, -d_avg.x)
            } else {
                // Right turn → outside is CCW perp of d_avg.
                Vertex::new(-d_avg.y, d_avg.x)
            };
            // Linear fade: full magnitude at the corner, zero at the
            // avoidance radius.
            let fade = 1.0 - (d / opts.corner_avoidance);
            shifted = shifted + outside * (opts.corner_avoidance * fade);
        }
        shifted
    }
}

// --- Tests --------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn v(x: f64, y: f64) -> Vertex {
        Vertex::new(x, y)
    }

    #[test]
    fn straight_path_lookahead_is_linear() {
        let mut f = PathFollower::new(vec![v(0.0, 0.0), v(10.0, 0.0)]);
        let opts = FollowerOptions {
            lookahead: 2.0,
            corner_avoidance: 0.0,
            corner_angle_threshold: 0.1,
        };
        let t = f.target(v(3.0, 0.0), &opts);
        assert!(t.approx_eq(v(5.0, 0.0), 1e-9));
        assert!((f.arc_length() - 3.0).abs() < 1e-9);
    }

    #[test]
    fn lookahead_past_end_clamps_to_last_point() {
        let mut f = PathFollower::new(vec![v(0.0, 0.0), v(10.0, 0.0)]);
        let opts = FollowerOptions {
            lookahead: 100.0,
            corner_avoidance: 0.0,
            corner_angle_threshold: 0.1,
        };
        let t = f.target(v(5.0, 0.0), &opts);
        assert!(t.approx_eq(v(10.0, 0.0), 1e-9));
    }

    #[test]
    fn off_path_agent_projects_to_nearest() {
        let mut f = PathFollower::new(vec![v(0.0, 0.0), v(10.0, 0.0)]);
        let opts = FollowerOptions {
            lookahead: 1.0,
            corner_avoidance: 0.0,
            corner_angle_threshold: 0.1,
        };
        // Agent at (4, 3) projects onto (4, 0); target is 1 unit ahead.
        let t = f.target(v(4.0, 3.0), &opts);
        assert!(t.approx_eq(v(5.0, 0.0), 1e-9));
    }

    #[test]
    fn progress_is_monotone_forward() {
        let mut f = PathFollower::new(vec![v(0.0, 0.0), v(10.0, 0.0)]);
        let opts = FollowerOptions::default();
        f.target(v(5.0, 0.0), &opts);
        assert!((f.arc_length() - 5.0).abs() < 1e-9);
        // Pretend the agent briefly backtracks. The follower must not lose
        // its advance.
        f.target(v(3.0, 0.0), &opts);
        assert!((f.arc_length() - 5.0).abs() < 1e-9, "lost progress to {}", f.arc_length());
    }

    #[test]
    fn at_end_reports_completion() {
        let mut f = PathFollower::new(vec![v(0.0, 0.0), v(10.0, 0.0)]);
        let opts = FollowerOptions::default();
        f.target(v(10.0, 0.0), &opts);
        assert!(f.at_end());
        assert!((f.progress() - 1.0).abs() < 1e-9);
    }

    /// L-shaped path with a left turn at (5,0). With corner_avoidance = 0
    /// the target on the post-corner segment is plain linear interpolation.
    /// With corner_avoidance > 0 the target is shifted outward (south-east
    /// for a left turn from east-then-north).
    #[test]
    fn corner_anti_shortcut_biases_target_outward() {
        let path = vec![v(0.0, 0.0), v(5.0, 0.0), v(5.0, 5.0)];
        let opts_none = FollowerOptions {
            lookahead: 2.5,
            corner_avoidance: 0.0,
            corner_angle_threshold: 0.1,
        };
        let opts_safe = FollowerOptions {
            lookahead: 2.5,
            corner_avoidance: 1.0,
            corner_angle_threshold: 0.1,
        };

        let mut f1 = PathFollower::new(path.clone());
        let t_none = f1.target(v(3.0, 0.0), &opts_none);
        assert!(t_none.approx_eq(v(5.0, 0.5), 1e-9), "got {:?}", t_none);

        let mut f2 = PathFollower::new(path);
        let t_safe = f2.target(v(3.0, 0.0), &opts_safe);
        // For a left turn at (5, 0) (east → north), outward = south-east.
        // Target shifted east (x > 5) and downward (y < 0.5).
        assert!(t_safe.x > t_none.x, "expected east shift, t_safe={:?}", t_safe);
        assert!(t_safe.y < t_none.y, "expected south shift, t_safe={:?}", t_safe);
    }

    /// Sub-threshold turn (almost straight) should NOT trigger corner
    /// avoidance — keeps the follower from twitching on numerical-noise
    /// corners along a "straight" run.
    #[test]
    fn small_turn_below_threshold_skipped() {
        // 1° turn at (5, 0): from (0,0) → (5, 0) → (10, 0.0875).
        let path = vec![v(0.0, 0.0), v(5.0, 0.0), v(10.0, 0.0875)];
        let opts = FollowerOptions {
            lookahead: 2.5,
            corner_avoidance: 1.0,
            corner_angle_threshold: 0.05, // ~3°, larger than the 1° turn
        };
        let mut f = PathFollower::new(path);
        let t = f.target(v(3.0, 0.0), &opts);
        // Plain lookahead at arc 5.5 on the second segment.
        let expected = v(5.0, 0.0).lerp(v(10.0, 0.0875), 0.5 / 5.000766);
        assert!(
            t.approx_eq(expected, 1e-3),
            "follower should have ignored the tiny turn: t={:?}, expected≈{:?}",
            t, expected
        );
    }

    #[test]
    fn right_turn_biases_to_the_left() {
        // Mirror of the left-turn test: a right turn at (5, 0) (east → south).
        let path = vec![v(0.0, 0.0), v(5.0, 0.0), v(5.0, -5.0)];
        let opts = FollowerOptions {
            lookahead: 2.5,
            corner_avoidance: 1.0,
            corner_angle_threshold: 0.1,
        };
        let mut f = PathFollower::new(path);
        let t = f.target(v(3.0, 0.0), &opts);
        // Outward for a right turn (east → south) is north-east. Target
        // unbiased would be (5, -0.5); biased should be x > 5 and y > -0.5.
        assert!(t.x > 5.0, "expected east shift, got {:?}", t);
        assert!(t.y > -0.5, "expected north shift, got {:?}", t);
    }

    #[test]
    fn far_from_any_corner_no_bias() {
        // Long straight segment, then a corner. Lookahead lands well
        // before the corner; no bias should apply.
        let path = vec![v(0.0, 0.0), v(20.0, 0.0), v(20.0, 5.0)];
        let opts = FollowerOptions {
            lookahead: 2.0,
            corner_avoidance: 1.0,
            corner_angle_threshold: 0.1,
        };
        let mut f = PathFollower::new(path);
        let t = f.target(v(5.0, 0.0), &opts);
        // Target arc = 7, corner at arc 20. Distance 13 >> avoidance.
        assert!(t.approx_eq(v(7.0, 0.0), 1e-9), "got {:?}", t);
    }
}
