//! Tests for the risky paths (audit S15): energy after impact, the stiffness limit in a
//! real run, the mid-run abort, restitution and friction.
use crash_sim::*;

fn total(sim: &Sim) -> f64 {
    let k: f64 = sim.vel.iter().map(|v| 0.5 * NODE_MASS * v.length().powi(2)).sum();
    let p: f64 = sim.pos.iter().map(|p| NODE_MASS * sim.cfg.gravity * p.y).sum();
    let e: f64 = sim
        .beams
        .iter()
        .map(|b| {
            let x = (sim.pos[b.a] - sim.pos[b.b]).length() - b.rest;
            0.5 * BEAM_STIFFNESS * x * x
        })
        .sum();
    k + p + e
}

/// (a) From the substep of first contact on, total energy never rises above its value at
/// that substep. The sim is stepped one physics substep at a time (dt = h = 1 ms, one
/// substep per advance, the same physics as the default run) and energy is sampled after
/// every substep, including the first contact ones. Measured on the default drop:
/// the energy at the first contact substep is 188.74 J and the largest rise after it is
/// -2.80 J (negative: every later substep is below it, so energy only falls). The bound
/// is a tight 0.01 J, about 5e-5 of the impact energy:
/// enough for rounding, far below any real energy gain from a bad contact or friction rule.
#[test]
fn energy_never_rises_after_impact() {
    let cfg = Config { dt: 0.001, substeps: 1, duration: 5.0, ..Config::default() };
    let steps = step_count(&cfg);
    let mut sim = Sim::new(cfg);
    let mut at_impact: Option<f64> = None;
    let mut worst_rise = f64::MIN;
    let mut after = 0;
    for _ in 0..steps {
        sim.advance();
        match at_impact {
            None if sim.first_contact_s.is_some() => at_impact = Some(total(&sim)),
            Some(e0) => {
                worst_rise = worst_rise.max(total(&sim) - e0);
                after += 1;
            }
            None => {}
        }
    }
    eprintln!("energy at impact {:?} J, worst rise {worst_rise} J over {after} substeps", at_impact);
    assert!(at_impact.is_some() && after > 1000, "no impact, or too few substeps after it ({after})");
    assert!(worst_rise < 0.01, "energy rose by {worst_rise} J after impact");
}

/// (b) A physics step just inside the stiffness limit stays stable for the full default
/// duration and passes `check()` after every output step.
#[test]
fn step_just_inside_stiffness_limit_runs_a_full_default_duration() {
    let cfg = Config { dt: max_stable_step() * 0.99, substeps: 1, ..Config::default() };
    cfg.validate().expect("a step inside the limit must be accepted");
    let steps = step_count(&cfg);
    let mut sim = Sim::new(cfg);
    for i in 0..steps {
        sim.advance();
        sim.check().unwrap_or_else(|e| panic!("step {i}: {e}"));
    }
    assert!(sim.first_contact_s.is_some(), "the cube never hit the ground");
    let (samples, res) = run_checked(&sim.cfg);
    assert!(res.is_ok(), "{res:?}");
    assert_eq!(samples.len() as u64, steps + 1);
}

/// (c) The mid-run abort is reachable: `validate` looks only at the step size, so a very
/// strong gravity passes it, the cube is crushed on impact, strain passes
/// MAX_STRAIN_LIMIT and `run_checked` stops with the good samples so far.
#[test]
fn mid_run_abort_is_reached_for_a_config_validate_accepts() {
    let cfg = Config { gravity: 5000.0, ..Config::default() };
    cfg.validate().expect("validate must accept this config");
    let full = step_count(&cfg) as usize + 1;
    let (samples, res) = run_checked(&cfg);
    let err = res.expect_err("the run must abort");
    assert!(err.starts_with("unstable at t = "), "{err}");
    assert!(err.contains("exceeds limit"), "{err}");
    assert!(!samples.is_empty() && samples.len() < full, "{} samples of {full}", samples.len());
    assert!(samples.iter().all(|s| s.max_beam_strain <= MAX_STRAIN_LIMIT), "a returned sample was already bad");
}

/// (d) Restitution. Flat drop (no tilt), one substep per step so every step is observable.
/// In the contact substep a node's downward speed v (before) becomes about
/// RESTITUTION * v. Exactly: the velocity update adds gravity first, so the speed after is
/// RESTITUTION * (v + g*h). Tolerance 1% of RESTITUTION * v covers the g*h term (0.16%).
#[test]
fn bounce_returns_restitution_times_the_impact_speed() {
    let h = 0.001;
    let mut sim = Sim::new(Config { dt: h, substeps: 1, tilt: false, ..Config::default() });
    let g = sim.cfg.gravity;
    for _ in 0..5000 {
        let before = sim.vel[0].y;
        sim.advance();
        if sim.vel.iter().any(|v| v.y > 0.0) {
            let after = sim.vel[0].y;
            assert!(before < -REST_SPEED, "bounce at a resting speed {before}");
            let expect = RESTITUTION * (-before + g * h);
            assert!((after - expect).abs() < 1e-9, "after {after} vs {expect}");
            assert!((after - RESTITUTION * -before).abs() < 0.01 * RESTITUTION * -before, "after {after}, before {before}");
            return;
        }
    }
    panic!("no bounce happened");
}

/// (e) Friction. A flat cube sliding on the ground with uniform horizontal speed 1 m/s.
/// The rule: a node touching the ground has its x and z speed multiplied by
/// keep = 1 - FRICTION_RATE*h each substep. Beams are internal, so total x momentum
/// before a step equals the sum of the nodes' post-force speeds, and the step removes
/// exactly (1 - keep)/keep * (sum of touching nodes' speeds after the step). That must
/// hold for every step (the cube is never exactly rigid, so a plain exp(-5 t) fit would
/// be wrong; the beam damper moves the top nodes' speed into the bottom ones, and its
/// diagonal beams even make a little z speed, which the x identity does not care about).
#[test]
fn sliding_contact_loses_speed_by_the_friction_rule() {
    let h = 0.001;
    let mut sim = Sim::new(Config { dt: h, substeps: 1, tilt: false, drop_height: 0.0, ..Config::default() });
    for v in sim.vel.iter_mut() {
        v.x = 1.0;
    }
    let keep = 1.0 - FRICTION_RATE * h;
    let px = |s: &Sim| s.vel.iter().map(|v| v.x).sum::<f64>();
    let start = px(&sim);
    for i in 0..100 {
        let before = px(&sim);
        sim.advance();
        let touching: f64 = (0..8).filter(|&n| sim.pos[n].y <= 0.0).map(|n| sim.vel[n].x).sum();
        let expect = before - (1.0 - keep) / keep * touching;
        assert!((px(&sim) - expect).abs() < 1e-9, "step {i}: px {} expected {expect}", px(&sim));
    }
    assert!(px(&sim) < 0.95 * start, "sliding did not slow down: {} of {start}", px(&sim));
    assert!((0..8).filter(|&n| sim.pos[n].y <= 0.0).count() >= 4, "cube is not touching the ground");
}

/// (e, exact form) One substep from uniform sliding: every touching node's speed becomes
/// exactly keep * 1.0, for several step sizes.
#[test]
fn one_sliding_substep_multiplies_speed_by_keep() {
    for h in [0.0005, 0.001, 0.002, 0.003] {
        let mut sim = Sim::new(Config { dt: h, substeps: 1, tilt: false, drop_height: 0.0, ..Config::default() });
        for v in sim.vel.iter_mut() {
            v.x = 1.0;
        }
        sim.advance();
        let keep = 1.0 - FRICTION_RATE * h;
        let touching: Vec<usize> = (0..8).filter(|&n| sim.pos[n].y <= 0.0).collect();
        assert_eq!(touching.len(), 4);
        for n in touching {
            assert!((sim.vel[n].x - keep).abs() < 1e-12, "h {h}: {} vs {keep}", sim.vel[n].x);
        }
    }
}
