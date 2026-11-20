//! Determinism and run-length checks, using only crash_sim's public API.

use crash_sim::{run, step_count, Config, Sample, Sim};

fn moon() -> Config {
    Config { gravity: 1.62, tilt: false, ..Config::default() }
}

fn assert_bit_identical(a: &[Sample], b: &[Sample], what: &str) {
    assert_eq!(a.len(), b.len(), "{what}: sample counts differ");
    for (i, (x, y)) in a.iter().zip(b).enumerate() {
        assert_eq!(x.t.to_bits(), y.t.to_bits(), "{what}: t differs at row {i}");
        assert_eq!(x.min_y.to_bits(), y.min_y.to_bits(), "{what}: min_y differs at row {i}");
        assert_eq!(x.center_y.to_bits(), y.center_y.to_bits(), "{what}: center_y differs at row {i}");
        assert_eq!(
            x.max_beam_strain.to_bits(),
            y.max_beam_strain.to_bits(),
            "{what}: max_beam_strain differs at row {i}"
        );
    }
}

/// Step a Sim by hand the way `run` does: sample at t = 0, then after each advance.
fn by_hand(cfg: &Config, steps: usize) -> Vec<Sample> {
    let mut sim = Sim::new(cfg.clone());
    let mut out = vec![sim.sample()];
    for _ in 0..steps {
        sim.advance();
        sim.check().expect("hand-stepped sim failed its check");
        out.push(sim.sample());
    }
    out
}

/// The rule in lib.rs: steps = floor(duration / dt) with a 1e-9 relative epsilon (`step_count`), samples = steps + 1.
fn expected_samples(cfg: &Config) -> usize {
    step_count(cfg) as usize + 1
}

#[test]
fn default_config_is_bit_identical_across_runs() {
    let cfg = Config::default();
    assert_bit_identical(&run(&cfg), &run(&cfg), "default");
}

#[test]
fn moon_gravity_no_tilt_is_bit_identical_across_runs() {
    let cfg = moon();
    assert_bit_identical(&run(&cfg), &run(&cfg), "moon");
}

#[test]
fn hand_stepped_sim_matches_run_default() {
    let cfg = Config::default();
    let r = run(&cfg);
    let h = by_hand(&cfg, r.len() - 1);
    assert_bit_identical(&r, &h, "default by hand");
}

#[test]
fn hand_stepped_sim_matches_run_moon() {
    let cfg = moon();
    let r = run(&cfg);
    let h = by_hand(&cfg, r.len() - 1);
    assert_bit_identical(&r, &h, "moon by hand");
}

#[test]
fn sample_count_default_is_501() {
    let cfg = Config::default();
    assert_eq!(run(&cfg).len(), 501);
    assert_eq!(run(&cfg).len(), expected_samples(&cfg));
}

#[test]
fn sample_count_follows_rounding_rule() {
    for (duration, dt, substeps) in [(1.0, 0.01, 10), (0.25, 0.1, 100), (0.26, 0.1, 100), (0.34, 0.1, 100), (2.0, 0.05, 50)] {
        let cfg = Config { duration, dt, substeps, ..Config::default() };
        let r = run(&cfg);
        assert_eq!(r.len(), expected_samples(&cfg), "duration {duration}, dt {dt}");
    }
}

#[test]
fn zero_duration_gives_exactly_the_initial_sample() {
    let cfg = Config { duration: 0.0, ..Config::default() };
    let r = run(&cfg);
    assert_eq!(r.len(), 1);
    assert_eq!(r[0].t.to_bits(), 0.0_f64.to_bits());
    assert_bit_identical(&r, &[Sim::new(cfg.clone()).sample()], "duration 0");
}

#[test]
fn sample_times_are_step_times() {
    let cfg = Config { duration: 0.5, ..Config::default() };
    for (i, s) in run(&cfg).iter().enumerate() {
        assert_eq!(s.t.to_bits(), (i as f64 * cfg.dt).to_bits(), "row {i}");
    }
}
