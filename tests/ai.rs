//! AI driver tests: a kinematic bicycle model (no crash-sim physics) closes the loop.

#[path = "../src/ai.rs"]
mod ai;

use ai::{CarState, Controls, Driver, DriverParams};
use std::f64::consts::{FRAC_PI_2, PI, TAU};

const WHEELBASE: f64 = 2.6;
const MAX_STEER: f64 = 0.5;
const DT: f64 = 0.01;

fn params() -> DriverParams {
    DriverParams {
        lookahead_k: 0.6,
        lookahead_min: 3.0,
        lookahead_max: 12.0,
        kp: 0.8,
        ki: 0.3,
        integral_max: 2.0,
    }
}

/// Kinematic bicycle (rear axle reference) with simple longitudinal dynamics.
#[derive(Clone, Copy)]
struct Model {
    x: f64,
    z: f64,
    heading: f64,
    speed: f64,
}

impl Model {
    fn state(&self) -> CarState {
        CarState {
            x: self.x,
            z: self.z,
            heading: self.heading,
            speed: self.speed,
            wheelbase: WHEELBASE,
            max_steer: MAX_STEER,
        }
    }
    fn step(&mut self, c: &Controls) {
        let delta = c.steer * MAX_STEER;
        self.x += self.speed * self.heading.cos() * DT;
        self.z += self.speed * self.heading.sin() * DT;
        self.heading += self.speed / WHEELBASE * delta.tan() * DT;
        let accel = 4.0 * c.throttle - 8.0 * c.brake - 0.05 * self.speed;
        self.speed = (self.speed + accel * DT).max(0.0);
    }
}

fn circle(r: f64, n: usize) -> Vec<(f64, f64)> {
    (0..n).map(|i| { let a = TAU * i as f64 / n as f64; (r * a.cos(), r * a.sin()) }).collect()
}

/// Stadium: two straights of length `straight` joined by semicircles of radius `r`,
/// counter-clockwise, starting at the middle of the bottom straight heading +x.
fn oval(straight: f64, r: f64, step: f64) -> Vec<(f64, f64)> {
    let mut p = Vec::new();
    let h = straight / 2.0;
    let mut x = 0.0;
    while x < h {
        p.push((x, -r));
        x += step;
    }
    let arc_n = (PI * r / step).ceil() as usize;
    for i in 0..arc_n {
        let a = -FRAC_PI_2 + PI * i as f64 / arc_n as f64;
        p.push((h + r * a.cos(), r * a.sin()));
    }
    let mut x = h;
    while x > -h {
        p.push((x, r));
        x -= step;
    }
    for i in 0..arc_n {
        let a = FRAC_PI_2 + PI * i as f64 / arc_n as f64;
        p.push((-h + r * a.cos(), r * a.sin()));
    }
    let mut x = -h;
    while x < 0.0 {
        p.push((x, -r));
        x += step;
    }
    p
}

fn dist_to_path(path: &[(f64, f64)], x: f64, z: f64) -> f64 {
    let n = path.len();
    let mut best = f64::INFINITY;
    for i in 0..n {
        let a = path[i];
        let b = path[(i + 1) % n];
        let (dx, dz) = (b.0 - a.0, b.1 - a.1);
        let l2 = dx * dx + dz * dz;
        let t = if l2 > 0.0 { (((x - a.0) * dx + (z - a.1) * dz) / l2).clamp(0.0, 1.0) } else { 0.0 };
        let d = ((x - a.0 - t * dx).powi(2) + (z - a.1 - t * dz).powi(2)).sqrt();
        best = best.min(d);
    }
    best
}

#[test]
fn circle_steady_steer_matches_ackermann() {
    for (r, dir) in [(25.0, 1.0), (40.0, 1.0), (25.0, -1.0)] {
        let mut path = circle(r, 720);
        if dir < 0.0 {
            path.reverse();
        }
        let tspeed = vec![8.0; path.len()];
        let mut d = Driver::new(params());
        let mut m = Model { x: path[0].0, z: path[0].1, heading: if dir > 0.0 { FRAC_PI_2 } else { -FRAC_PI_2 }, speed: 8.0 };
        let mut last = 0.0;
        for _ in 0..4000 {
            let c = d.update(&m.state(), &path, &tspeed, DT);
            m.step(&c);
            last = c.steer;
        }
        let want = dir * (WHEELBASE / r).atan();
        let got = last * MAX_STEER;
        assert!((got - want).abs() <= 0.02 * want.abs(), "R={r} dir={dir}: steer angle {got} want {want}");
    }
}

#[test]
fn oval_cross_track_under_half_metre_after_first_20m() {
    let path = oval(80.0, 25.0, 0.5);
    let tspeed = vec![10.0; path.len()];
    let mut d = Driver::new(params());
    let mut m = Model { x: path[0].0, z: path[0].1, heading: 0.0, speed: 10.0 };
    let mut travelled = 0.0;
    let mut worst: f64 = 0.0;
    let mut checked = 0;
    for _ in 0..12000 {
        let c = d.update(&m.state(), &path, &tspeed, DT);
        let (px, pz) = (m.x, m.z);
        m.step(&c);
        travelled += ((m.x - px).powi(2) + (m.z - pz).powi(2)).sqrt();
        if travelled > 20.0 {
            worst = worst.max(dist_to_path(&path, m.x, m.z));
            checked += 1;
        }
    }
    assert!(travelled > 500.0, "should have driven several laps, drove {travelled}");
    assert!(checked > 10000);
    assert!(worst < 0.5, "worst cross-track error {worst}");
}

#[test]
fn speed_settles_within_0_3_of_constant_target() {
    let path = oval(80.0, 25.0, 0.5);
    for target in [6.0, 12.0] {
        let tspeed = vec![target; path.len()];
        let mut d = Driver::new(params());
        // Start from standstill, and (second pass) too fast so the brake is exercised.
        for start in [0.0, 20.0] {
            let mut dd = d.clone();
            let mut m = Model { x: path[0].0, z: path[0].1, heading: 0.0, speed: start };
            for _ in 0..6000 {
                let c = dd.update(&m.state(), &path, &tspeed, DT);
                assert!(!(c.throttle > 0.0 && c.brake > 0.0));
                m.step(&c);
            }
            for _ in 0..1000 {
                let c = dd.update(&m.state(), &path, &tspeed, DT);
                m.step(&c);
                assert!((m.speed - target).abs() < 0.3, "start {start} target {target} speed {}", m.speed);
            }
        }
        d.reset();
    }
}

#[test]
fn brakes_when_over_target_and_throttles_when_under() {
    let path = circle(30.0, 360);
    let tspeed = vec![10.0; path.len()];
    let d = Driver::new(params());
    let mut s = Model { x: 30.0, z: 0.0, heading: FRAC_PI_2, speed: 15.0 }.state();
    let c = d.control(&s, &path, &tspeed);
    assert!(c.brake > 0.0 && c.throttle == 0.0);
    s.speed = 5.0;
    let c = d.control(&s, &path, &tspeed);
    assert!(c.throttle > 0.0 && c.brake == 0.0);
}

#[test]
fn outputs_always_in_range() {
    let path = oval(60.0, 20.0, 1.0);
    let tspeed: Vec<f64> = (0..path.len()).map(|i| (i % 7) as f64 * 4.0).collect();
    let mut d = Driver::new(params());
    let mut seed = 12345u64;
    let mut rnd = || {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (seed >> 11) as f64 / (1u64 << 53) as f64
    };
    for _ in 0..5000 {
        let s = CarState {
            x: rnd() * 400.0 - 200.0,
            z: rnd() * 400.0 - 200.0,
            heading: rnd() * 40.0 - 20.0,
            speed: rnd() * 100.0 - 20.0,
            wheelbase: 0.5 + rnd() * 4.0,
            max_steer: 0.05 + rnd() * 1.0,
        };
        for c in [d.control(&s, &path, &tspeed), d.update(&s, &path, &tspeed, 0.05)] {
            assert!((0.0..=1.0).contains(&c.throttle), "{c:?}");
            assert!((0.0..=1.0).contains(&c.brake), "{c:?}");
            assert!((-1.0..=1.0).contains(&c.steer), "{c:?}");
            assert!(!(c.throttle > 0.0 && c.brake > 0.0), "{c:?}");
        }
    }
}

#[test]
fn steer_saturates_for_a_target_behind_the_car() {
    let path = circle(30.0, 360);
    let tspeed = vec![5.0; path.len()];
    let d = Driver::new(params());
    // On the circle but rotated 2 rad off the path: the lookahead point is far to the side
    // so the wanted angle exceeds max steer and the output saturates at -1 / +1.
    let s = Model { x: 30.0, z: 0.0, heading: FRAC_PI_2 + 2.0, speed: 5.0 }.state();
    let c = d.control(&s, &path, &tspeed);
    assert_eq!(c.steer, -1.0, "{c:?}");
    let s = Model { x: 30.0, z: 0.0, heading: FRAC_PI_2 - 2.0, speed: 5.0 }.state();
    assert_eq!(d.control(&s, &path, &tspeed).steer, 1.0);
}

#[test]
fn same_inputs_give_identical_outputs() {
    let path = oval(80.0, 25.0, 0.5);
    let tspeed = vec![10.0; path.len()];
    let run = || {
        let mut d = Driver::new(params());
        let mut m = Model { x: path[0].0, z: path[0].1, heading: 0.0, speed: 3.0 };
        let mut out = Vec::new();
        for _ in 0..3000 {
            let c = d.update(&m.state(), &path, &tspeed, DT);
            out.push((c.throttle.to_bits(), c.brake.to_bits(), c.steer.to_bits()));
            m.step(&c);
        }
        out
    };
    assert_eq!(run(), run());
    let d = Driver::new(params());
    let s = Model { x: 1.0, z: -24.0, heading: 0.1, speed: 9.0 }.state();
    assert_eq!(d.control(&s, &path, &tspeed), d.control(&s, &path, &tspeed));
}

#[test]
fn invalid_input_stops_the_car() {
    let path = circle(30.0, 36);
    let tspeed = vec![5.0; path.len()];
    let d = Driver::new(params());
    let good = Model { x: 30.0, z: 0.0, heading: FRAC_PI_2, speed: 5.0 }.state();
    assert_ne!(d.control(&good, &path, &tspeed), Controls::STOP);
    assert_eq!(d.control(&good, &path[..1], &tspeed[..1]), Controls::STOP);
    assert_eq!(d.control(&good, &path, &tspeed[..5]), Controls::STOP);
    assert_eq!(d.control(&good, &[], &[]), Controls::STOP);
    let mut bad = good;
    bad.speed = f64::NAN;
    assert_eq!(d.control(&bad, &path, &tspeed), Controls::STOP);
    bad = good;
    bad.wheelbase = 0.0;
    assert_eq!(d.control(&bad, &path, &tspeed), Controls::STOP);
    let mut ts = tspeed.clone();
    ts[3] = f64::INFINITY;
    assert_eq!(d.control(&good, &path, &ts), Controls::STOP);
    // A degenerate path (all points equal) must not hang or panic.
    let same = vec![(1.0, 1.0); 4];
    let _ = d.control(&good, &same, &[1.0; 4]);
}

#[test]
fn integral_is_clamped() {
    let path = circle(30.0, 36);
    let tspeed = vec![50.0; path.len()];
    let mut d = Driver::new(params());
    let s = Model { x: 30.0, z: 0.0, heading: FRAC_PI_2, speed: 0.0 }.state();
    for _ in 0..10000 {
        d.update(&s, &path, &tspeed, 0.1);
    }
    assert!((d.integral() - params().integral_max).abs() < 1e-12);
}

#[test]
fn invalid_params_stop_without_panic() {
    let path = circle(30.0, 36);
    let tspeed = vec![5.0; path.len()];
    let s = Model { x: 30.0, z: 0.0, heading: FRAC_PI_2, speed: 5.0 }.state();
    let mut p = params();
    p.integral_max = -1.0;
    let mut d = Driver::new(p);
    assert_eq!(d.update(&s, &path, &tspeed, 0.1), Controls::STOP);
    assert_eq!(d.control(&s, &path, &tspeed), Controls::STOP);
    assert_eq!(d.integral(), 0.0);
    for bad in [f64::NAN, f64::INFINITY] {
        let mut p = params();
        p.integral_max = bad;
        let mut d = Driver::new(p);
        assert_eq!(d.update(&s, &path, &tspeed, 0.1), Controls::STOP);
    }
    // Zero is a legal clamp (integral stays 0), not a panic.
    let mut p = params();
    p.integral_max = 0.0;
    let mut d = Driver::new(p);
    let _ = d.update(&s, &path, &tspeed, 0.1);
    assert_eq!(d.integral(), 0.0);
}

#[test]
fn extreme_finite_inputs_never_give_nan() {
    let path = vec![(0.0, 0.0), (1e200, 0.0), (1e200, 1e200)];
    let tspeed = vec![5.0; 3];
    let mut d = Driver::new(params());
    let s = Model { x: 0.0, z: 0.0, heading: 0.0, speed: 5.0 }.state();
    let c = d.update(&s, &path, &tspeed, 0.1);
    assert!(c.throttle.is_finite() && c.brake.is_finite() && c.steer.is_finite());
}

#[test]
fn lookahead_longer_than_two_laps_still_turns() {
    // Unit square, lookahead 10 = 2.5 laps: the target is 2 m along, at (1, 1).
    let path = vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
    let tspeed = vec![5.0; 4];
    let mut p = params();
    p.lookahead_min = 10.0;
    p.lookahead_max = 10.0;
    let d = Driver::new(p);
    let s = Model { x: 0.0, z: 0.0, heading: 0.0, speed: 5.0 }.state();
    assert!(d.control(&s, &path, &tspeed).steer > 0.1);
}

#[test]
fn ki_changes_the_speed_command() {
    let path = circle(30.0, 36);
    let tspeed = vec![10.0; path.len()];
    let s = Model { x: 30.0, z: 0.0, heading: FRAC_PI_2, speed: 9.0 }.state();
    let mut with_ki = Driver::new(params());
    let mut p = params();
    p.ki = 0.0;
    let mut no_ki = Driver::new(p);
    let (mut a, mut b) = (Controls::STOP, Controls::STOP);
    for _ in 0..20 {
        a = with_ki.update(&s, &path, &tspeed, 0.1);
        b = no_ki.update(&s, &path, &tspeed, 0.1);
    }
    assert!(a.throttle > b.throttle + 0.05);
}

impl Model {
    /// Same as `step`, plus a constant retarding acceleration `grade` (m/s^2), like a hill.
    fn step_with_grade(&mut self, c: &Controls, grade: f64) {
        let delta = c.steer * MAX_STEER;
        self.x += self.speed * self.heading.cos() * DT;
        self.z += self.speed * self.heading.sin() * DT;
        self.heading += self.speed / WHEELBASE * delta.tan() * DT;
        let accel = 4.0 * c.throttle - 8.0 * c.brake - 0.05 * self.speed - grade;
        self.speed = (self.speed + accel * DT).max(0.0);
    }
}

/// Drives 60 s on a constant grade; returns (min, max) of |speed - target| over the last 5 s.
fn grade_error_band(ki: f64, grade: f64, target: f64) -> (f64, f64) {
    let path = oval(80.0, 25.0, 0.5);
    let tspeed = vec![target; path.len()];
    let mut p = params();
    p.ki = ki;
    let mut d = Driver::new(p);
    let mut m = Model {
        x: path[0].0,
        z: path[0].1,
        heading: 0.0,
        speed: target,
    };
    let (mut lo, mut hi) = (f64::INFINITY, 0.0f64);
    let steps = (65.0 / DT) as usize;
    let tail = (5.0 / DT) as usize;
    for i in 0..steps {
        let c = d.update(&m.state(), &path, &tspeed, DT);
        m.step_with_grade(&c, grade);
        if i >= steps - tail {
            let e = (m.speed - target).abs();
            lo = lo.min(e);
            hi = hi.max(e);
        }
    }
    assert!(lo.is_finite(), "no samples taken");
    (lo, hi)
}

#[test]
fn pi_integral_removes_steady_error_that_p_only_leaves() {
    // Grade 1.5 m/s^2 at 10 m/s needs throttle (1.5 + 0.5) / 4 = 0.5 to hold speed.
    // P-only (kp 0.8) can only make that with an error of 0.5 / 0.8 = 0.625 m/s.
    let (target, grade) = (10.0, 1.5);
    let (p_lo, p_hi) = grade_error_band(0.0, grade, target);
    println!("P-only error over last 5 s: min {p_lo} max {p_hi}");
    assert!(
        p_lo > 0.3,
        "P-only should leave a steady error above 0.3 m/s, min was {p_lo}"
    );
    let (pi_lo, pi_hi) = grade_error_band(params().ki, grade, target);
    println!("PI error over last 5 s: min {pi_lo} max {pi_hi}");
    assert!(
        pi_hi < 0.05,
        "PI should be within 0.05 m/s after 60 s, worst was {pi_hi}"
    );
}
