//! AI driver: pure-pursuit steering and PI speed control.
//!
//! Pure and std only: no physics, no track types. The driver sees a car state, a closed
//! polyline (x, z) and one target speed per polyline point, and answers with the same
//! throttle / brake / steer a player gives (throttle 0..1, brake 0..1, steer -1..1).
//!
//! Conventions: heading is an angle in radians, the car's forward direction is
//! (cos heading, sin heading) in the (x, z) plane, and positive steer increases heading.
//! The control point is the rear axle (the usual pure-pursuit reference).

/// What the driver reads from the car.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CarState {
    pub x: f64,
    pub z: f64,
    pub heading: f64,
    pub speed: f64,
    pub wheelbase: f64,
    /// Steer angle (radians) at steer = 1.
    pub max_steer: f64,
}

/// Same ranges as the input script: throttle 0..1, brake 0..1, steer -1..1.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Controls {
    pub throttle: f64,
    pub brake: f64,
    pub steer: f64,
}

impl Controls {
    /// Safe fallback: no throttle, full brake, wheel straight.
    pub const STOP: Controls = Controls { throttle: 0.0, brake: 1.0, steer: 0.0 };
}

/// Tuning. Every field is required; there are no hidden defaults.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DriverParams {
    /// Lookahead per unit speed (seconds): L = clamp(k * v, l_min, l_max).
    pub lookahead_k: f64,
    pub lookahead_min: f64,
    pub lookahead_max: f64,
    /// PI gains on the speed error (m/s), output is throttle (positive) or brake (negative).
    pub kp: f64,
    pub ki: f64,
    /// Anti-windup clamp on the integral of the speed error (m/s * s).
    pub integral_max: f64,
}

#[derive(Clone, Debug)]
pub struct Driver {
    params: DriverParams,
    integral: f64,
}

fn finite(v: f64) -> bool {
    v.is_finite()
}

fn wrap_angle(a: f64) -> f64 {
    let two_pi = std::f64::consts::TAU;
    let mut r = a % two_pi;
    if r > std::f64::consts::PI {
        r -= two_pi;
    } else if r < -std::f64::consts::PI {
        r += two_pi;
    }
    r
}

/// Nearest point on the closed polyline: (segment index, fraction 0..1 along it).
fn nearest(path: &[(f64, f64)], px: f64, pz: f64) -> (usize, f64) {
    let n = path.len();
    let mut best = (0usize, 0.0f64);
    let mut best_d = f64::INFINITY;
    for i in 0..n {
        let a = path[i];
        let b = path[(i + 1) % n];
        let (dx, dz) = (b.0 - a.0, b.1 - a.1);
        let len2 = dx * dx + dz * dz;
        let t = if len2 > 0.0 {
            (((px - a.0) * dx + (pz - a.1) * dz) / len2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let (cx, cz) = (a.0 + t * dx, a.1 + t * dz);
        let d = (px - cx) * (px - cx) + (pz - cz) * (pz - cz);
        if d < best_d {
            best_d = d;
            best = (i, t);
        }
    }
    best
}

/// Point `dist` of arc length ahead of (seg, t), walking the closed polyline forwards.
fn point_ahead(path: &[(f64, f64)], seg: usize, t: f64, dist: f64) -> (f64, f64) {
    let n = path.len();
    let mut i = seg;
    let a = path[i];
    let b = path[(i + 1) % n];
    let mut remaining = dist;
    let mut cur = (a.0 + t * (b.0 - a.0), a.1 + t * (b.1 - a.1));
    // Bounded: at most two laps, so degenerate (zero length) paths cannot loop forever.
    for _ in 0..2 * n + 1 {
        let next = path[(i + 1) % n];
        let (dx, dz) = (next.0 - cur.0, next.1 - cur.1);
        let seg_len = (dx * dx + dz * dz).sqrt();
        if seg_len >= remaining && seg_len > 0.0 {
            let f = remaining / seg_len;
            return (cur.0 + f * dx, cur.1 + f * dz);
        }
        remaining -= seg_len;
        cur = next;
        i = (i + 1) % n;
    }
    cur
}

impl Driver {
    pub fn new(params: DriverParams) -> Driver {
        Driver { params, integral: 0.0 }
    }

    pub fn integral(&self) -> f64 {
        self.integral
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
    }

    fn valid(&self, s: &CarState, path: &[(f64, f64)], target_speed: &[f64]) -> bool {
        let p = &self.params;
        path.len() >= 2
            && target_speed.len() == path.len()
            && path.iter().all(|q| finite(q.0) && finite(q.1))
            && target_speed.iter().all(|v| finite(*v))
            && [s.x, s.z, s.heading, s.speed, s.wheelbase, s.max_steer].iter().all(|v| finite(*v))
            && s.wheelbase > 0.0
            && s.max_steer > 0.0
            && [p.lookahead_k, p.lookahead_min, p.lookahead_max, p.kp, p.ki, p.integral_max]
                .iter()
                .all(|v| finite(*v))
            && p.lookahead_min > 0.0
            && p.lookahead_max >= p.lookahead_min
    }

    /// Target speed at the nearest path point (interpolated along its segment).
    fn speed_error(&self, s: &CarState, path: &[(f64, f64)], target_speed: &[f64]) -> f64 {
        let (seg, t) = nearest(path, s.x, s.z);
        let n = path.len();
        let target = target_speed[seg] + t * (target_speed[(seg + 1) % n] - target_speed[seg]);
        target - s.speed
    }

    fn controls(&self, s: &CarState, path: &[(f64, f64)], target_speed: &[f64], integral: f64) -> Controls {
        let p = &self.params;
        let (seg, t) = nearest(path, s.x, s.z);
        let look = (p.lookahead_k * s.speed.abs()).clamp(p.lookahead_min, p.lookahead_max);
        let (tx, tz) = point_ahead(path, seg, t, look);
        let (dx, dz) = (tx - s.x, tz - s.z);
        let d = (dx * dx + dz * dz).sqrt();
        let steer = if d > 1e-9 {
            let alpha = wrap_angle(dz.atan2(dx) - s.heading);
            // d is the real distance to the target point, so on a circle the angle is exact.
            let delta = (2.0 * s.wheelbase * alpha.sin() / d).atan();
            (delta / s.max_steer).clamp(-1.0, 1.0)
        } else {
            0.0
        };
        let err = self.speed_error(s, path, target_speed);
        let u = p.kp * err + p.ki * integral;
        let (throttle, brake) = if u >= 0.0 { (u.min(1.0), 0.0) } else { (0.0, (-u).min(1.0)) };
        Controls { throttle, brake, steer }
    }

    /// Pure function of its inputs and the stored integral (no state change).
    /// Invalid input (path shorter than 2 points, length mismatch, NaN or infinity,
    /// non-positive wheelbase or max steer) gives `Controls::STOP`.
    pub fn control(&self, state: &CarState, path: &[(f64, f64)], target_speed: &[f64]) -> Controls {
        if !self.valid(state, path, target_speed) {
            return Controls::STOP;
        }
        self.controls(state, path, target_speed, self.integral)
    }

    /// Advance the PI integrator by `dt` seconds (anti-windup clamped), then control.
    /// Invalid input or a non-positive `dt` leaves the integrator alone.
    pub fn update(&mut self, state: &CarState, path: &[(f64, f64)], target_speed: &[f64], dt: f64) -> Controls {
        if !self.valid(state, path, target_speed) || !(dt > 0.0) || !dt.is_finite() {
            return self.control(state, path, target_speed);
        }
        let err = self.speed_error(state, path, target_speed);
        let m = self.params.integral_max;
        self.integral = (self.integral + err * dt).clamp(-m, m);
        self.controls(state, path, target_speed, self.integral)
    }
}
