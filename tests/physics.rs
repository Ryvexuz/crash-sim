use crash_sim::vec3::Vec3;
use crash_sim::{
    max_stable_step, max_strain, run, run_checked, to_csv, Config, Sample, Sim, BEAM_STIFFNESS, GRAVITY,
    NODE_MASS,
};
use std::process::Command;

fn default_run() -> Vec<Sample> {
    run(&Config::default())
}

/// Index of first sample where the lowest node touches the ground.
fn first_contact(s: &[Sample]) -> usize {
    s.iter().position(|r| r.min_y <= 1e-9).expect("cube never reached the ground")
}

/// Local maxima of center_y after first contact, ignoring wiggles under `prominence` metres.
fn bounce_peaks(s: &[Sample], prominence: f64) -> Vec<f64> {
    let mut peaks = Vec::new();
    let mut lo = s[first_contact(s)].center_y; // lowest since last peak
    let mut hi = lo; // highest since last trough
    let mut rising = false;
    for r in &s[first_contact(s)..] {
        let y = r.center_y;
        if rising {
            if y > hi {
                hi = y;
            } else if hi - y > prominence {
                peaks.push(hi);
                rising = false;
                lo = y;
            }
        } else if y < lo {
            lo = y;
        } else if y - lo > prominence {
            rising = true;
            hi = y;
        }
    }
    peaks
}

#[test]
fn free_fall_matches_analytic_height() {
    let cfg = Config::default();
    let s = run(&cfg);
    let c0 = s[0].center_y;
    let contact = first_contact(&s);
    assert!(contact > 10, "contact too early to test free fall");
    // Stop one output step before contact: still airborne.
    for r in &s[..contact - 1] {
        let expect = c0 - 0.5 * GRAVITY * r.t * r.t;
        // symplectic Euler has an O(h) position bias of 0.5*g*h*t (~2.5 mm at 0.5 s, h = 1 ms)
        assert!((r.center_y - expect).abs() < 0.005, "t={} got {} want {}", r.t, r.center_y, expect);
        assert!(r.max_beam_strain < 1e-9, "beams strained in free fall");
    }
    // first sample at 2 m minimum height
    assert!((s[0].min_y - cfg.drop_height).abs() < 1e-9);
}

#[test]
fn cube_has_28_beams_and_8_nodes() {
    let sim = Sim::new(Config::default());
    assert_eq!(sim.pos.len(), 8);
    assert_eq!(sim.beams.len(), 28);
}

#[test]
fn cube_hits_ground_and_bounces() {
    let s = default_run();
    let c = first_contact(&s);
    let peak = s[c..].windows(2).any(|w| w[1].center_y > w[0].center_y + 0.01);
    assert!(peak, "center_y never rose after first contact");
    assert!(!bounce_peaks(&s, 0.02).is_empty(), "no bounce peak found");
}

#[test]
fn bounces_lose_energy_and_cube_settles() {
    let s = default_run();
    let peaks = bounce_peaks(&s, 0.02);
    assert!(!peaks.is_empty());
    for w in peaks.windows(2) {
        assert!(w[1] < w[0], "bounce peak rose: {} -> {}", w[0], w[1]);
    }
    assert!(peaks[0] < s[0].center_y, "first bounce higher than the drop");

    let last = s.last().unwrap();
    assert!((last.center_y - 0.5).abs() < 0.02, "not resting on a face: center_y={}", last.center_y);
    assert!(last.min_y.abs() < 0.01);

    // Velocities small at the end: run again keeping the Sim to inspect it.
    let cfg = Config::default();
    let mut sim = Sim::new(cfg.clone());
    for _ in 0..(cfg.duration / cfg.dt).round() as usize {
        sim.advance();
    }
    assert!(sim.max_speed() < 0.05, "still moving: {} m/s", sim.max_speed());
    // Resting on a face: 4 nodes on the ground, 4 at one edge length up.
    let mut ys: Vec<f64> = sim.pos.iter().map(|p| p.y).collect();
    ys.sort_by(|a, b| a.partial_cmp(b).unwrap());
    for y in &ys[..4] {
        assert!(y.abs() < 0.01, "bottom node not on ground: {ys:?}");
    }
    for y in &ys[4..] {
        assert!((y - cfg.edge).abs() < 0.02, "top node not one edge up: {ys:?}");
    }
}

#[test]
fn total_energy_never_exceeds_initial() {
    let cfg = Config::default();
    let mut sim = Sim::new(cfg.clone());
    let energy = |s: &Sim| {
        let ke: f64 = s.vel.iter().map(|v| 0.5 * NODE_MASS * v.dot(*v)).sum();
        let pe: f64 = s.pos.iter().map(|p| NODE_MASS * GRAVITY * p.y).sum();
        let spring: f64 = s
            .beams
            .iter()
            .map(|b| {
                let l = (s.pos[b.a] - s.pos[b.b]).length();
                0.5 * BEAM_STIFFNESS * (l - b.rest) * (l - b.rest)
            })
            .sum();
        ke + pe + spring
    };
    let e0 = energy(&sim);
    for _ in 0..(cfg.duration / cfg.dt).round() as usize {
        sim.advance();
        // 0.1% slack for symplectic Euler's bounded energy ripple (measured max E/E0 is ~0.99998)
        assert!(energy(&sim) <= e0 * 1.001, "energy grew: {} > {}", energy(&sim), e0);
    }
}

#[test]
fn stable_no_nan_no_penetration_low_strain() {
    for tilt in [true, false] {
        let s = run(&Config { tilt, ..Config::default() });
        for r in &s {
            assert!(r.t.is_finite() && r.min_y.is_finite() && r.center_y.is_finite() && r.max_beam_strain.is_finite());
            assert!(r.min_y > -0.01, "node below ground: {}", r.min_y);
            assert!(r.max_beam_strain < 0.10, "strain {} at t={}", r.max_beam_strain, r.t);
        }
    }
}

#[test]
fn deterministic_output() {
    let a = to_csv(&default_run());
    let b = to_csv(&default_run());
    assert_eq!(a, b);
    assert!(a.starts_with("t,min_y,center_y,max_beam_strain\n"));
    assert_eq!(a.lines().count(), 502); // header + t = 0..5 s at 0.01
}

#[test]
fn nan_state_is_not_hidden_by_strain() {
    let sim = Sim::new(Config::default());
    let mut pos = sim.pos.clone();
    pos[3] = Vec3::new(f64::NAN, f64::NAN, f64::NAN);
    assert!(max_strain(&sim.beams, &pos).is_nan());
    let mut bad = Sim::new(Config::default());
    bad.pos[0].y = f64::INFINITY;
    assert!(bad.check().is_err());
    let mut bad = Sim::new(Config::default());
    bad.vel[2].x = f64::NAN;
    assert!(bad.check().is_err());
    let mut bad = Sim::new(Config::default());
    bad.pos[0].x += 0.9; // stretches beams far beyond the strain limit
    assert!(bad.check().unwrap_err().contains("strain"));
    assert!(Sim::new(Config::default()).check().is_ok());
}

#[test]
fn unstable_step_is_refused_up_front() {
    let limit = max_stable_step();
    assert!(limit > 0.001 && limit < 0.005, "limit {limit}");
    let (rows, res) = run_checked(&Config { dt: 0.02, substeps: 1, ..Config::default() });
    assert!(rows.is_empty());
    assert!(res.unwrap_err().contains("stable limit"));
    // just under the limit is accepted
    let ok = Config { dt: limit * 0.99, substeps: 1, duration: 0.1, ..Config::default() };
    assert!(ok.validate().is_ok());
}

#[test]
fn binary_fails_loudly_on_harsh_flags() {
    let exe = crash_sim_bin();
    let out = Command::new(&exe)
        .args(["--drop-height", "10", "--substeps", "1", "--dt", "0.02"])
        .output()
        .unwrap();
    assert_eq!(out.status.code(), Some(1));
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(err.contains("exceeds the stable limit"), "stderr: {err}");
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(!stdout.contains("NaN") && !stdout.contains("inf"));

    // Default flags still exit 0.
    let ok = Command::new(&exe).output().unwrap();
    assert!(ok.status.success());
}

#[test]
fn non_dividing_dt_ends_at_the_last_whole_step_not_after_duration() {
    let s = run(&Config { dt: 0.006, ..Config::default() });
    let last = s.last().unwrap().t;
    assert!(last <= 5.0 && 5.0 - last < 0.006, "last t {last}");
}

#[test]
fn gravity_must_be_non_negative_and_finite() {
    use crash_sim::Config;
    assert!(Config::default().validate().is_ok());
    assert_eq!(Config::default().gravity, 9.81);
    // gravity 0 is valid since n20-07 (a car flying level into a wall)
    assert!(Config { gravity: 0.0, ..Config::default() }.validate().is_ok());
    for g in [-1.0, f64::NAN, f64::INFINITY] {
        assert!(Config { gravity: g, ..Config::default() }.validate().is_err(), "gravity {g} accepted");
    }
    assert!(Config { gravity: 1.62, ..Config::default() }.validate().is_ok());
}

#[test]
fn lower_gravity_makes_the_cube_fall_more_slowly() {
    use crash_sim::{run_checked, Config};
    // Time of first ground contact (min_y reaches 0) for a run; free fall from 2 m takes sqrt(2h/g).
    let first_contact = |g: f64| {
        let (samples, res) = run_checked(&Config { gravity: g, ..Config::default() });
        res.expect("stable run");
        samples.iter().find(|s| s.min_y <= 1e-9).map(|s| s.t).expect("cube reaches the ground")
    };
    let (earth, moon) = (first_contact(9.81), first_contact(1.62));
    assert!((earth - (2.0 * 2.0 / 9.81f64).sqrt()).abs() < 0.03, "earth contact at {earth}");
    assert!((moon - (2.0 * 2.0 / 1.62f64).sqrt()).abs() < 0.05, "moon contact at {moon}");
    assert!(moon > 2.0 * earth, "moon contact {moon} should be well after earth contact {earth}");
}

/// Path of the crash-sim binary, read at run time: the copy a broadsword test step points
/// `CARGO_BIN_EXE_crash-sim` at, else the cargo layout next to the test executable.
fn crash_sim_bin() -> std::path::PathBuf {
    if let Some(p) = std::env::var_os("CARGO_BIN_EXE_crash-sim") {
        return std::path::PathBuf::from(p);
    }
    let exe = std::env::current_exe().expect("current_exe of the test executable");
    let name = format!("crash-sim{}", std::env::consts::EXE_SUFFIX);
    let dir = exe.parent().and_then(|p| p.parent()).unwrap_or_else(|| {
        panic!("CARGO_BIN_EXE_crash-sim is not set and {} has no grandparent folder", exe.display())
    });
    let bin = dir.join(name);
    assert!(
        bin.is_file(),
        "CARGO_BIN_EXE_crash-sim is not set and the cargo layout path {} does not exist",
        bin.display()
    );
    bin
}
