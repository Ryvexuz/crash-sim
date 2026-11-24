#[path = "../examples/crash_energy_a.rs"]
mod measurement;

use crash_sim::collide::{self, Plane};
use crash_sim::solver::{Solver, State, SymplecticEuler};
use crash_sim::vec3::Vec3;
use crash_sim::vehicle::Substep;
use crash_sim::{Beam, Config, Sim};
use measurement::*;
use std::io::{self, Write};
use std::sync::OnceLock;

fn run() -> &'static Measurement {
    static RUN: OnceLock<std::result::Result<Measurement, String>> = OnceLock::new();
    RUN.get_or_init(measure)
        .as_ref()
        .unwrap_or_else(|e| panic!("measurement_a: FAIL {e}"))
}

fn close(label: &str, actual: f64, expected: f64, tolerance: f64) {
    assert!(
        actual.is_finite() && expected.is_finite(),
        "{label}: non-finite"
    );
    assert!(
        (actual - expected).abs() <= tolerance,
        "{label}: actual={actual:.12} expected={expected:.12} tolerance={tolerance:.12}"
    );
}

/// Dyadic parameters make the analytic work and velocity expectations exact in binary.
fn beam_fixture(length: f64, yield_strain: Option<f64>) -> Sim {
    let mut sim = Sim::new(Config {
        gravity: 0.0,
        ..Config::default()
    });
    sim.pos = vec![Vec3::new(0.0, 2.0, 0.0), Vec3::new(length, 2.0, 0.0)];
    sim.vel = vec![Vec3::new(-2.0, 0.0, 0.0), Vec3::new(2.0, 0.0, 0.0)];
    sim.mass = vec![2.0, 2.0];
    sim.beams = vec![Beam {
        a: 0,
        b: 1,
        rest: 1.0,
        stiffness: 128.0,
        damping: 16.0,
    }];
    sim.rest0 = vec![1.0];
    sim.yield_strain = vec![yield_strain];
    sim.break_strain = vec![None];
    sim.broken = vec![false];
    sim
}

/// Exact isolated-beam ledger used below: K0=128, K1=98, E1=49/64,
/// Wd=32, residual=-177/64 J. Only 0 or 32 J of uncoupled translation
/// is permitted; it changes neither work nor the signed integration defect.
fn integrator_fixture(transverse_kinetic: f64) -> Measurement {
    assert!([0.0, 32.0].contains(&transverse_kinetic));
    let initial = 128.0 + transverse_kinetic;
    let start = Row {
        elastic: [0.0; 4],
        totals: Totals::default(),
        velocity: [1.0; 2],
        crush: 0.0,
        kinetic: initial,
        vehicle_elastic: 0.0,
        wheel_kinetic: 0.0,
        residual: 0.0,
    };
    let end = Row {
        elastic: [0.0, 0.0, 49.0 / 64.0, 0.0],
        totals: Totals {
            damper: [0.0, 0.0, 32.0, 0.0],
            ..Totals::default()
        },
        kinetic: 98.0 + transverse_kinetic,
        crush: 1.0,
        residual: -177.0 / 64.0,
        ..start.clone()
    };
    let rows = vec![start, end.clone(), end];
    let mut budget = budget::Budget::new(initial).unwrap();
    for (i, row) in rows.iter().enumerate() {
        budget.record(i, row.residual).unwrap();
    }
    Measurement {
        rows,
        h: 1.0 / 128.0,
        contact: 1,
        split: 1,
        initial,
        max_residual: 177.0 / 64.0,
        budget,
    }
}

#[test]
fn measurement_a_matches_unobserved_bits() {
    // measure compares observation on/off and advance(h,1) at EVERY substep, then
    // advance(dt,10) at EVERY output boundary, including complete derived-Debug car state.
    let m = run();
    let (cfg, body) = inputs().unwrap();
    let expected = crash_sim::step_count(&cfg) * u64::from(cfg.substeps) + 1;
    assert_eq!(m.rows.len() as u64, expected, "all substeps must be observed");
    let a = Sim::from_car(cfg.clone(), &body).unwrap();
    let mut b = Sim::from_car(cfg, &body).unwrap();
    same_state(&a, &b).unwrap();
    // Behavioural negative controls, not a compile-only proof.
    b.vel[0].x = f64::from_bits(b.vel[0].x.to_bits() + 1);
    assert!(same_state(&a, &b).unwrap_err().contains("bit mismatch"));
    b.vel[0] = a.vel[0];
    b.car.as_mut().unwrap().wheels[0].load[1] = -0.0;
    assert!(
        same_state(&a, &b).is_err(),
        "signed-zero wheel load must differ"
    );
    b.car = a.car.clone();
    b.yield_refused.push(0);
    assert!(same_state(&a, &b).is_err(), "yield error/state must differ");
}

#[test]
fn measurement_a_energy_budget_closes_every_substep() {
    // An initially unstrained, isolated beam has no contact, projection, yield or vehicle
    // work. Symplectic Euler and left-end dashpot quadrature nevertheless leave a signed
    // integration defect. This independently solvable case must NOT become a physical loss.
    let h = 1.0 / 128.0;
    let mut isolated = beam_fixture(1.0, None);
    isolated.mass.fill(32.0);
    isolated.beams[0].stiffness = 2048.0;
    isolated.beams[0].damping = 256.0;
    let mut ledger = Totals::default();
    record_damper(&isolated, h, &[2], &mut ledger).unwrap();
    // Initial K=2*(1/2)*32*2^2=128. Force=256*4=1024 N, so each speed
    // decreases by 1024/(128*32)=1/4. End K=32*(7/4)^2=98 J.
    // Extension=(7/2)/128=7/256; end E=(1/2)*2048*(7/256)^2=49/64 J.
    // Left-end dashpot work=256*4^2/128=32 J. Residual=-177/64 J.
    assert_eq!(ledger.damper[2], 32.0);
    let mut solver = SymplecticEuler::default();
    solver.step(
        State {
            pos: &mut isolated.pos,
            vel: &mut isolated.vel,
            beams: &isolated.beams,
            mass: &isolated.mass,
            gravity: 0.0,
        },
        h,
    );
    assert_eq!(solver.check(), None);
    assert_eq!(isolated.vel[0].x, -1.75);
    assert_eq!(isolated.vel[1].x, 1.75);
    let k: f64 = isolated
        .vel
        .iter()
        .zip(&isolated.mass)
        .map(|(v, m)| 0.5 * m * v.dot(*v))
        .sum();
    let e: f64 = beam_elastic(&isolated).iter().sum();
    assert_eq!(k, 98.0);
    assert_eq!(e, 49.0 / 64.0);
    assert_eq!(128.0 - (k + e + ledger.damper_sum()), -177.0 / 64.0);
    assert_eq!(ledger.contact_sum(), 0.0);
    assert_eq!(yield_work(&mut isolated).unwrap(), vec![0.0]);
    let failure = check_budget(128.0, k, e, 0.0, ledger.damper_sum(), 0.0).unwrap_err();
    assert_eq!(failure, "budget residual_j=-2.765625 tolerance_j=2.560000");

    // Add 32 J of uncoupled transverse translation to the same analytic ledger:
    // two 32 kg nodes at z-velocity 1 m/s add 32 J at both endpoints, no work.
    // The defect remains -177/64 J, now inside 2% of 160 J. It must be
    // reported, NOT suppressed or transferred into a physical loss column.
    let residual = check_budget(160.0, k + 32.0, e, 0.0, ledger.damper_sum(), 0.0).unwrap();
    assert_eq!(residual, -177.0 / 64.0);
    // Repeated endpoint proves the summary chooses the earliest maximum on ties.
    let fixture = integrator_fixture(32.0);
    let mut output = Vec::new();
    fixture.render(&mut output).unwrap();
    let output = String::from_utf8(output).unwrap();
    assert!(output.contains("residual t_s=-0.007812 residual_j=0.000000\n"));
    assert!(output.contains("residual t_s=0.000000 residual_j=-2.765625\n"));
    assert!(output.contains("residual t_s=0.007812 residual_j=-2.765625\n"));
    assert!(output.contains(
        "integrator_residual max_abs_j=2.765625 at_substep=1 fraction_of_initial_ke=0.017285\n"
    ));
    assert_eq!(fixture.rows[1].totals.contact_sum(), 0.0);
    assert_eq!(fixture.rows[1].totals.damper_sum(), 32.0);
    assert_eq!(fixture.rows[1].totals.plastic_sum(), 0.0);

    // Inclusive 2% boundary in both directions, with no old 1 J floor.
    assert_eq!(BUDGET_FRACTION, 0.02);
    assert_eq!(check_budget(50.0, 49.0, 0.0, 0.0, 0.0, 0.0).unwrap(), 1.0);
    assert_eq!(check_budget(50.0, 51.0, 0.0, 0.0, 0.0, 0.0).unwrap(), -1.0);
    for k in [
        f64::from_bits(49.0_f64.to_bits() - 1),
        f64::from_bits(51.0_f64.to_bits() + 1),
    ] {
        assert!(check_budget(50.0, k, 0.0, 0.0, 0.0, 0.0).is_err());
    }
    assert!(check_budget(1.0, 0.5, 0.0, 0.0, 0.0, 0.0).is_err());

    let mut zero = fixture.clone();
    zero.max_residual = 0.0;
    zero.budget = budget::Budget::new(160.0).unwrap();
    for row in &mut zero.rows {
        row.kinetic = 160.0;
        row.elastic = [0.0; 4];
        row.totals = Totals::default();
        row.residual = 0.0;
    }
    for i in 0..zero.rows.len() {
        zero.budget.record(i, 0.0).unwrap();
    }
    let mut output = Vec::new();
    zero.render(&mut output).unwrap();
    assert!(String::from_utf8(output).unwrap().contains(
        "integrator_residual max_abs_j=0.000000 at_substep=0 fraction_of_initial_ke=0.000000\n"
    ));

    let m = run();
    m.validate().unwrap();
    assert!(m.budget.budget_ok, "real Test A budget failed: {:?}", m.budget);
    let tolerance = 0.02 * m.initial;
    assert!(
        m.max_residual <= tolerance,
        "max_abs_residual_j={} tolerance_j={tolerance}",
        m.max_residual
    );
    for (i, r) in m.rows.iter().enumerate() {
        let e: f64 = r.elastic.iter().sum::<f64>() + r.vehicle_elastic;
        let sum = r.kinetic
            + e
            + r.totals.plastic_sum()
            + r.totals.damper_sum()
            + r.totals.contact_sum();
        close(&format!("budget substep={i}"), sum, m.initial, tolerance);
        assert_eq!(r.residual.to_bits(), (m.initial - sum).to_bits());
    }
    let r0 = &m.rows[0];
    // Independent kinetic reference from the scenario: total translating mass 1330 kg,
    // four I=1.512 kg m^2 wheels, radius .3175 m, initial speed 56.2 km/h.
    let v0 = 56.2 / 3.6;
    let expected_initial =
        0.5 * 1330.0 * v0 * v0 + 4.0 * 0.5 * 1.512 * (v0 / 0.3175) * (v0 / 0.3175);
    close("initial kinetic J", m.initial, expected_initial, 1e-8);
    close(
        "initial stored J",
        r0.elastic.iter().sum::<f64>() + r0.vehicle_elastic,
        0.0,
        1e-8,
    );
    assert!(
        r0.wheel_kinetic > 0.0,
        "wheel translation and spin must not be omitted"
    );
    // Independent exact budget: two 2 kg nodes at +/-2 m/s have 8 J; a pure
    // dashpot c=16, relative speed=4, h=1/128 dissipates 2 J (left-end rule).
    // The test deliberately asks the budget guard to judge that analytic ledger.
    assert_eq!(check_budget(8.0, 6.0, 0.0, 0.0, 2.0, 0.0).unwrap(), 0.0);
    assert!(
        check_budget(8.0, 6.0, 0.0, 0.0, 0.0, 0.0).is_err(),
        "omitting the analytic dashpot must fail closure"
    );
    let sim = beam_fixture(1.0, None);
    let mut ledger = Totals::default();
    record_damper(&sim, 1.0 / 128.0, &[2], &mut ledger).unwrap();
    let work = ledger.damper_sum();
    // The same measurement used in the material ledger must close the independent budget.
    check_budget(8.0, 6.0, 0.0, 0.0, work, 0.0).unwrap();
    for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(check_budget(bad, 1.0, 0.0, 0.0, 0.0, 0.0).is_err());
    }
    assert!(check_budget(f64::MAX, f64::MAX, f64::MAX, 0.0, 0.0, 0.0).is_err());
}

#[test]
fn budget_failure_still_reports_the_whole_run() {
    // Removing the uncoupled 32 J scales the initial energy down to 128 J.
    // The unchanged exact -177/64 J defect exceeds 2% (2.56 J).
    let fixture = integrator_fixture(0.0);
    fixture.validate().unwrap();
    assert!(!fixture.budget.budget_ok, "ignoring the residual must fail");
    let first = fixture.budget.first_breach.unwrap();
    assert_eq!(first.substep, 1);
    assert_eq!(first.residual, -177.0 / 64.0);
    assert_eq!(fixture.budget.at_substep, 1, "earliest maximum on ties");
    let mut out = Vec::new();
    assert_eq!(budget::report(Ok(fixture.clone()), &mut out).unwrap(), 1);
    let text = String::from_utf8(out).unwrap();
    assert!(text.ends_with(
        "measurement_a: FAIL budget first_substep=1 residual_j=-2.765625 tolerance_j=2.560000 max_abs_residual_j=2.765625 at_substep=1\n"
    ));
    assert!(!text.contains("measurement_a: ok"));
    assert!(text.contains("residual t_s=0.007812 residual_j=-2.765625\n"));
    assert!(text.contains(
        "integrator_residual max_abs_j=2.765625 at_substep=1 fraction_of_initial_ke=0.021606\n"
    ));
    for (prefix, count) in [
        ("energy ", 12),
        ("velocity ", 3),
        ("residual ", 3),
        ("split ", 1),
        ("phase ", 8),
        ("wall ", 3),
        ("rebound ", 2),
        ("aux ", 1),
        ("budget ", 1),
        ("integrator_residual ", 1),
    ] {
        assert_eq!(
            text.lines().filter(|line| line.starts_with(prefix)).count(),
            count,
            "{prefix}"
        );
    }
    assert!(text.contains("phase phase=unloading material=crush_rear "));
    assert!(text.contains("rebound channel=com t_s=none speed_mps=0.000000 fraction_v0=0.000000\n"));
    let mut out = Vec::new();
    assert_eq!(
        budget::report(Ok(integrator_fixture(32.0)), &mut out).unwrap(),
        0
    );
    assert!(String::from_utf8(out).unwrap().ends_with("measurement_a: ok\n"));
    let mut corrupt = fixture.clone();
    corrupt.budget.budget_ok = true;
    let mut out = Vec::new();
    assert!(budget::report(Ok(corrupt), &mut out).is_err());
    assert!(out.is_empty(), "validate verdict before writing");
    assert!(budget::report(Ok(fixture.clone()), &mut FailWrite)
        .unwrap_err()
        .contains("output write"));
    let mut partial = FailAfter {
        bytes: Vec::new(),
        remaining: text.find("measurement_a: FAIL").unwrap(),
        flush_failure: false,
    };
    assert!(budget::report(Ok(fixture), &mut partial).is_err());
    assert!(String::from_utf8(partial.bytes)
        .unwrap()
        .ends_with("fraction_of_initial_ke=0.021606\n"));
}

#[test]
fn budget_tracking_rejects_invalid_input_and_preserves_first_breach() {
    for bad in [0.0, -1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(budget::Budget::new(bad).is_err());
    }
    let mut tracked = budget::Budget::new(50.0).unwrap();
    for (i, residual) in [1.0, -1.0, -2.0, 3.0, -3.0, 0.0].into_iter().enumerate() {
        tracked.record(i, residual).unwrap();
    }
    assert!(!tracked.budget_ok);
    assert_eq!(tracked.first_breach.unwrap().substep, 2);
    assert_eq!(tracked.first_breach.unwrap().residual, -2.0);
    assert_eq!(tracked.max_abs_residual, 3.0);
    assert_eq!(tracked.at_substep, 3);
    let saved = tracked.clone();
    for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(tracked.record(6, bad).is_err());
        assert_eq!(tracked, saved, "invalid input cannot change judgment");
    }
    assert!(tracked.record(7, 0.0).is_err());
    assert_eq!(tracked, saved);
    // Exact references across the binary64 band: a zero ledger
    // closes; losing all initial energy breaches even when 2% underflows to zero.
    for initial in [f64::from_bits(1), f64::MIN_POSITIVE, 1.0, f64::MAX] {
        assert_eq!(check_budget(initial, initial, 0.0, 0.0, 0.0, 0.0).unwrap(), 0.0);
        assert!(check_budget(initial, 0.0, 0.0, 0.0, 0.0, 0.0).is_err());
    }
    let mut tiny = budget::Budget::new(f64::from_bits(1)).unwrap();
    let before = tiny.clone();
    assert!(tiny.record(0, f64::MAX).is_err(), "fraction overflow");
    assert_eq!(tiny, before);
    // Adjacent values at the largest exponent differ by exactly 2^971.
    assert_eq!(
        budget::residual(
            f64::MAX,
            f64::from_bits(f64::MAX.to_bits() - 1),
            0.0,
            0.0,
            0.0,
            0.0,
        )
        .unwrap(),
        2.0_f64.powi(971)
    );
    for field in 0..6 {
        let mut inputs = [0.0; 6];
        inputs[0] = 1.0;
        inputs[field] = f64::NAN;
        let [initial, k, e, p, d, c] = inputs;
        assert!(budget::residual(initial, k, e, p, d, c).is_err());
    }
    let mut out = Vec::new();
    assert_eq!(
        budget::report(Err("fixture instability".into()), &mut out).unwrap_err(),
        "fixture instability"
    );
    assert!(out.is_empty(), "instability must remain an immediate error");
}

#[test]
fn measurement_a_yield_and_dashpot_work_are_separate() {
    let h = 1.0 / 128.0;
    let mut compression = beam_fixture(0.25, Some(0.125));
    // E_before = 64*(1-1/4)^2 = 36 J.
    // rest_after = 1/4 + 1/8 = 3/8, E_after = 64*(1/8)^2 = 1 J.
    // Plastic=16*(5/8)=10 J; the other 25 J is report-only yield overshoot.
    assert_eq!(yield_work(&mut compression).unwrap(), vec![10.0]);
    assert_eq!(compression.beams[0].rest, 0.375);
    assert_eq!(beam_elastic(&compression), vec![1.0]);
    // c*v_rel^2*h = 16*4^2/128 = 2 J, even after the spring has yielded.
    assert_eq!(beam_damper(&compression, h).unwrap(), vec![2.0]);
    for material in 0..4 {
        let mut ledger = Totals::default();
        record_damper(&compression, h, &[material], &mut ledger).unwrap();
        assert_eq!(ledger.damper[material], 2.0, "material={material} dashpot J");
    }
    let mut solver = SymplecticEuler::default();
    solver.step(
        State {
            pos: &mut compression.pos,
            vel: &mut compression.vel,
            mass: &compression.mass,
            beams: &compression.beams,
            gravity: 0.0,
        },
        h,
    );
    // Spring -16 N PLUS dashpot +64 N = +48 N. Capping the combined force
    // at the 16 N spring yield force would give the wrong velocity here.
    assert_eq!(compression.vel[0].x, -1.8125);
    assert_eq!(compression.vel[1].x, 1.8125);
    let mut tension = beam_fixture(2.25, Some(0.5));
    // rest_after = (9/4)/(3/2) = 3/2; E_before=100, E_after=36.
    // Plastic=96*(1/2)=48 J; the other 16 J is report-only yield overshoot.
    assert_eq!(yield_work(&mut tension).unwrap(), vec![48.0]);
    assert_eq!(tension.beams[0].rest, 1.5);
    assert_eq!(beam_damper(&tension, h).unwrap(), vec![2.0]);
    assert_eq!(yield_work(&mut tension).unwrap(), vec![0.0]);
    let mut invalid = beam_fixture(1.0, None);
    invalid.vel[0].x = f64::INFINITY;
    assert!(beam_damper(&invalid, h).is_err());
    invalid.vel[0].x = f64::NAN;
    assert!(beam_damper(&invalid, h).is_err());
    for h in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        assert!(beam_damper(&tension, h).is_err());
    }
    invalid = beam_fixture(1.0, None);
    invalid.beams[0].stiffness = f64::MAX;
    invalid.pos[1].x = f64::MAX;
    let rest = invalid.beams[0].rest.to_bits();
    assert!(yield_work(&mut invalid).is_err());
    assert_eq!(
        invalid.beams[0].rest.to_bits(),
        rest,
        "refuse before yield mutation"
    );
    // Independent powers-of-two references (exact significands, no decimal approximation).
    // Exercise underflow, subnormal work and large finite work with exact references.
    let mut scaled = beam_fixture(1.0, None);
    scaled.vel[0] = Vec3::ZERO;
    for exponent in [-538, -536, -535, -511, 0, 509] {
        scaled.vel[1].x = 2.0_f64.powi(exponent);
        let power = 2 * exponent - 3; // 16*v^2/128
        let expected = if power < -1074 {
            0.0
        } else if power < -1022 {
            f64::from_bits(1_u64 << (power + 1074))
        } else {
            f64::from_bits(((power + 1023) as u64) << 52)
        };
        assert_eq!(
            beam_damper(&scaled, h).unwrap()[0],
            expected,
            "exponent={exponent}"
        );
    }
    scaled.vel[1].x = 2.0_f64.powi(510);
    assert!(beam_damper(&scaled, h).is_err(), "intermediate work overflow");
    scaled.vel[1].x = f64::from_bits(1);
    assert_eq!(beam_damper(&scaled, h).unwrap(), vec![0.0]);
    scaled.vel[1].x = f64::MAX;
    assert!(beam_damper(&scaled, h).is_err());
    scaled = beam_fixture(1.0 + f64::EPSILON, None);
    // Near-cancelling extension is exactly one ulp of 1; E=64*(2^-52)^2=2^-98.
    assert_eq!(beam_elastic(&scaled), vec![2.0_f64.powi(-98)]);
}

#[test]
fn measurement_a_wall_impulses_include_tyres() {
    let measured = run();
    let (cfg, body) = inputs().unwrap();
    let mut sim = Sim::from_car(cfg.clone(), &body).unwrap();
    sim.cfg.dt = measured.h;
    sim.cfg.substeps = 1;
    let mut expected = 0.0;
    let mut expected_compression = 0.0;
    let mut positive_without_ground = false;
    for i in 1..measured.rows.len() {
        // Independent radial law from the pre-force positions and velocities, NOT the
        // measurement's impulse or momentum residual. Also inspect actual pre_step loads.
        let normal = sim.planes[1].normal;
        let mut radial = 0.0;
        for w in &sim.car.as_ref().unwrap().wheels {
            let penetration = w.p.radius - sim.planes[1].distance(w.pos);
            if penetration > 0.0 {
                let load = (w.p.radial_stiffness * penetration
                    - w.p.radial_damping * w.vel.dot(normal))
                .max(0.0);
                radial += load * measured.h;
            }
        }
        let mut car = sim.car.clone().unwrap();
        let mut extra = vec![Vec3::ZERO; sim.pos.len()];
        car.pre_step(
            &sim.pos,
            &sim.vel,
            &sim.mass,
            &sim.planes,
            Substep {
                gravity: Vec3::ZERO,
                h: measured.h,
            },
            &mut extra,
        );
        let actual_load_work: f64 = car.wheels.iter().map(|w| w.load[1] * measured.h).sum();
        close("radial wall impulse Ns", actual_load_work, radial, 1e-10);
        if actual_load_work > 0.0
            && !collide::any_at_or_behind(&sim.pos, &sim.planes[..1])
            && car
                .wheels
                .iter()
                .all(|w| sim.planes[0].distance(w.pos) > w.p.radius)
        {
            positive_without_ground = true;
        }
        expected += radial;
        if i <= measured.split {
            expected_compression += radial;
        }
        sim.advance();
        close(
            "cumulative tyre impulse Ns",
            measured.rows[i].totals.tyre_impulse,
            expected,
            1e-8,
        );
    }
    assert!(
        expected > 0.0 && positive_without_ground,
        "tyres must load the wall in flight"
    );
    close(
        "compression tyre impulse Ns",
        measured.rows[measured.split].totals.tyre_impulse,
        expected_compression,
        1e-8,
    );
    // Independent node wall-only fixture: incoming vx=4, restitution=1/2
    // gives outgoing vx=-2, m=2 => normal impulse 12 Ns.
    let mut node = beam_fixture(0.25, None);
    node.pos[0].x = 1.0;
    node.pos[1].x = -1.0;
    node.vel[0] = Vec3::new(4.0, 0.0, 0.0);
    node.vel[1] = Vec3::ZERO;
    node.planes = vec![
        Plane::ground(),
        Plane::new(Vec3::ZERO, Vec3::new(-1.0, 0.0, 0.0)).unwrap(),
    ];
    let before = node.vel.clone();
    let wall = node.planes[1];
    collide::apply(&mut node.pos, &mut node.vel, &[wall], measured.h);
    assert_eq!(node_wall_impulse(&before, &node).unwrap(), 12.0);
    assert!(node_wall_impulse(&[], &node).is_err());
}

#[test]
fn measurement_a_sensor_com_and_rebound_are_distinct() {
    let m = run();
    assert!(m.rows.iter().any(|r| r.velocity[0] != r.velocity[1]));
    let (cfg, body) = inputs().unwrap();
    let mut unequal = Sim::from_car(cfg, &body).unwrap();
    unequal.vel.fill(Vec3::ZERO);
    for w in &mut unequal.car.as_mut().unwrap().wheels {
        w.vel = Vec3::new(3.0, 0.0, 0.0);
    }
    let sensor = group(&body, "sensor").unwrap();
    unequal.vel[sensor[0]].x = 2.0;
    unequal.vel[sensor[1]].x = 6.0;
    // Both sensor nodes weigh 40.25 kg: sensor=(2+6)/2=4.
    // Scenario node-mass sum 1210 kg, four wheels of 30 kg: COM=(322+360)/1330.
    let actual = velocities(&unequal, &sensor).unwrap();
    close("unequal sensor m/s", actual[0], 4.0, 1e-12);
    close("unequal COM m/s", actual[1], 682.0 / 1330.0, 1e-12);
    assert_ne!(actual[0], actual[1], "substituting COM for sensor must fail");
    // Different channel v0, maxima and earliest ties; endpoint is not the peak.
    let sensor_trace = [8.0, 8.0, -2.0, -2.0, -1.0];
    let com_trace = [4.0, 4.0, 1.0, -1.0, -0.5];
    assert_eq!(
        rebound(&sensor_trace, 1, 0.125).unwrap(),
        (Some(0.125), 2.0, 0.25)
    );
    assert_eq!(
        rebound(&com_trace, 1, 0.125).unwrap(),
        (Some(0.25), 1.0, 0.25)
    );
    assert_eq!(
        rebound(&[2.0, 1.0, 0.0, 0.5], 1, 0.125).unwrap(),
        (None, 0.0, 0.0)
    );
    assert_eq!(earliest_max(&[0.0, 2.0, 2.0, 1.0]).unwrap(), 1);
    assert!(earliest_max(&[]).is_err());
    assert!(earliest_max(&[f64::NAN]).is_err());
    assert!(rebound(&[1.0, f64::INFINITY], 1, 0.125).is_err());
    assert!(rebound(&[0.0, -1.0], 1, 0.125).is_err());
    assert!(rebound(&[], 0, 0.125).is_err());
    assert!(velocities(&unequal, &[]).is_err());
    assert!(velocities(&unequal, &[usize::MAX]).is_err());
}

struct FailWrite;

impl Write for FailWrite {
    fn write(&mut self, _: &[u8]) -> io::Result<usize> {
        Err(io::Error::new(io::ErrorKind::BrokenPipe, "fixture closed"))
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

struct FailAfter {
    bytes: Vec<u8>,
    remaining: usize,
    flush_failure: bool,
}

impl Write for FailAfter {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        if self.remaining == 0 {
            return Err(io::Error::new(io::ErrorKind::BrokenPipe, "partial fixture"));
        }
        let n = self.remaining.min(bytes.len());
        self.bytes.extend_from_slice(&bytes[..n]);
        self.remaining -= n;
        Ok(n)
    }

    fn flush(&mut self) -> io::Result<()> {
        if self.flush_failure {
            Err(io::Error::other("flush fixture"))
        } else {
            Ok(())
        }
    }
}

#[test]
fn measurement_a_phase_sums_and_output_contract() {
    let m = run();
    let mut out = Vec::new();
    m.render(&mut out).unwrap();
    let text = String::from_utf8(out).unwrap();
    if m.budget.budget_ok {
        assert!(text.ends_with("measurement_a: ok\n"));
    } else {
        let first = m.budget.first_breach.unwrap();
        assert!(text.ends_with(&format!(
            "measurement_a: FAIL budget first_substep={} residual_j={:.6} tolerance_j={:.6} max_abs_residual_j={:.6} at_substep={}\n",
            first.substep, first.residual, BUDGET_FRACTION * m.initial,
            m.max_residual, m.budget.at_substep
        )));
        assert!(!text.contains("measurement_a: ok"));
    }
    let lines: Vec<_> = text.lines().collect();
    assert_eq!(
        lines.iter().filter(|l| l.starts_with("energy ")).count(),
        m.rows.len() * 4
    );
    assert_eq!(
        lines.iter().filter(|l| l.starts_with("velocity ")).count(),
        m.rows.len()
    );
    assert_eq!(
        lines.iter().filter(|l| l.starts_with("residual ")).count(),
        m.rows.len()
    );
    for (prefix, count) in [
        ("phase ", 8),
        ("wall ", 3),
        ("rebound ", 2),
        ("aux ", 1),
        ("budget ", 1),
        ("integrator_residual ", 1),
    ] {
        assert_eq!(
            lines.iter().filter(|l| l.starts_with(prefix)).count(),
            count,
            "{prefix}"
        );
    }
    for (i, row) in lines[..m.rows.len() * 6].as_chunks::<6>().0.iter().enumerate() {
        for (line, material) in row[..4].iter().zip(MATERIALS) {
            assert!(line.contains(&format!("material={material} ")), "{line}");
            let expected_phase = if i <= m.split {
                "compression"
            } else {
                "unloading"
            };
            assert!(line.contains(&format!("phase={expected_phase} ")), "{line}");
        }
        assert!(row[4].starts_with("velocity "));
        assert_eq!(
            row[5],
            format!(
                "residual t_s={:.6} residual_j={:.6}",
                (i as f64 - m.contact as f64) * m.h,
                m.rows[i].residual
            )
        );
    }
    assert!(lines[m.rows.len() * 6].starts_with("split "));
    // Recompute the maximum from the signed ledger equation, not the recorded maximum.
    let (mut peak, mut max_abs) = (0, 0.0_f64);
    for (i, row) in m.rows.iter().enumerate() {
        let residual = m.initial
            - (row.kinetic
                + (row.elastic.iter().sum::<f64>() + row.vehicle_elastic)
                + row.totals.plastic_sum()
                + row.totals.damper_sum()
                + row.totals.contact_sum());
        if residual.abs() > max_abs {
            peak = i;
            max_abs = residual.abs();
        }
    }
    assert_eq!(
        lines[lines.len() - 2],
        format!(
            "integrator_residual max_abs_j={max_abs:.6} at_substep={peak} fraction_of_initial_ke={:.6}",
            max_abs / m.initial
        )
    );
    // Every float field has finite invariant six-decimal formatting; at_substep is an integer.
    for line in &lines[..lines.len() - 1] {
        let (kind, _) = line.split_once(' ').unwrap();
        let expected_keys: &[&str] = match kind {
            "energy" => &[
                "t_s",
                "phase",
                "material",
                "elastic_j",
                "plastic_j",
                "damper_j",
            ],
            "velocity" => &["t_s", "sensor_mps", "com_mps", "sensor_minus_com_mps"],
            "residual" => &["t_s", "residual_j"],
            "integrator_residual" => &["max_abs_j", "at_substep", "fraction_of_initial_ke"],
            "split" => &["contact_s", "compression_end_s", "end_s"],
            "phase" => &[
                "phase",
                "material",
                "elastic_start_j",
                "elastic_end_j",
                "elastic_peak_j",
                "plastic_j",
                "damper_j",
                "yield_at_cap_j",
                "yield_overshoot_j",
            ],
            "wall" => &["phase", "node_ns", "tyre_ns", "total_ns"],
            "rebound" => &["channel", "t_s", "speed_mps", "fraction_v0"],
            "aux" => &[
                "suspension_damper_j",
                "tyre_damper_j",
                "aero_j",
                "node_collision_j",
                "node_projection_j",
                "tyre_friction_rolling_j",
                "carrier_constraint_j",
                "vehicle_elastic_j",
                "wheel_kinetic_j",
                "yield_at_cap_j",
                "yield_overshoot_j",
            ],
            "budget" => &[
                "initial_kinetic_j",
                "kinetic_j",
                "elastic_j",
                "plastic_j",
                "damper_j",
                "contact_loss_j",
                "residual_j",
                "max_abs_residual_j",
                "tolerance_j",
            ],
            _ => panic!("unexpected output kind {kind}"),
        };
        let keys: Vec<_> = line
            .split_ascii_whitespace()
            .skip(1)
            .map(|f| f.split_once('=').unwrap().0)
            .collect();
        assert_eq!(keys, expected_keys, "field contract {kind}");
        for field in line.split_ascii_whitespace().skip(1) {
            let (key, value) = field.split_once('=').expect("key=value output");
            if ["phase", "material", "channel"].contains(&key) || value == "none" {
                continue;
            }
            if key == "at_substep" {
                let index = value.parse::<usize>().expect("substep index");
                assert!(index < m.rows.len(), "{field}");
                assert_eq!(value, index.to_string(), "{field}");
                continue;
            }
            let number = value
                .parse::<f64>()
                .unwrap_or_else(|_| panic!("invalid {field}"));
            assert!(number.is_finite(), "{field}");
            assert_eq!(value, format!("{number:.6}"), "{field}");
        }
    }
    let walls: Vec<Vec<f64>> = lines
        .iter()
        .filter(|line| line.starts_with("wall "))
        .map(|line| {
            line.split_ascii_whitespace()
                .skip(2)
                .map(|f| f.split_once('=').unwrap().1.parse::<f64>().unwrap())
                .collect()
        })
        .collect();
    // Three independently rounded six-place quantities permit at most 1.5 micro-Ns
    // rounding error in these printed identities (not a physical impulse tolerance).
    for wall in &walls {
        close("printed node+tyre Ns", wall[0] + wall[1], wall[2], 1.5e-6);
    }
    for c in 0..3 {
        close(
            "printed wall phase sum Ns",
            walls[0][c] + walls[1][c],
            walls[2][c],
            1.5e-6,
        );
    }
    let peak = m.rows[m.split].crush;
    assert!(
        m.rows[..m.split].iter().all(|r| r.crush < peak),
        "earliest global peak"
    );
    assert!(
        m.rows[m.split..].iter().all(|r| r.crush <= peak),
        "complete-run global peak"
    );
    let phases: Vec<_> = lines.iter().filter(|l| l.starts_with("phase ")).collect();
    for (i, line) in phases.iter().enumerate() {
        let phase = if i < 4 {
            "compression"
        } else {
            "unloading"
        };
        assert!(line.starts_with(&format!(
            "phase phase={phase} material={} ",
            MATERIALS[i % 4]
        )));
    }
    for j in 0..4 {
        let start = &m.rows[0].totals;
        let split = &m.rows[m.split].totals;
        let end = &m.rows.last().unwrap().totals;
        close(
            "plastic phase sum J",
            (split.plastic[j] - start.plastic[j]) + (end.plastic[j] - split.plastic[j]),
            end.plastic[j],
            1e-8,
        );
        close(
            "damper phase sum J",
            (split.damper[j] - start.damper[j]) + (end.damper[j] - split.damper[j]),
            end.damper[j],
            1e-8,
        );
    }
    let z = [
        -0.844, -0.844, -0.75425, -0.75425, 0.75425, 0.75425, 0.844, 0.844,
    ];
    // Six evenly spaced points: outer columns contribute one point each,
    // inner columns two points each, shared between their two heights.
    for (w, expected) in profile_weights(&z).unwrap().iter().zip([
        1.0 / 12.0,
        1.0 / 12.0,
        2.0 / 12.0,
        2.0 / 12.0,
        2.0 / 12.0,
        2.0 / 12.0,
        1.0 / 12.0,
        1.0 / 12.0,
    ]) {
        close("profile weight", *w, expected, 1e-12);
    }
    assert!(profile_weights(&[]).is_err());
    assert!(profile_weights(&[f64::NAN]).is_err());
    assert!(profile_weights(&[-f64::MAX, f64::MAX]).is_err());
    let mut err = Vec::new();
    for args in [
        vec![],
        vec!["--test-a".into(), "--body".into(), "C:\\other.bsn".into()],
        vec!["--test-a\r".into()],
        vec!["--test-a".into(), "--test-a".into()],
    ] {
        let mut stdout = Vec::new();
        assert_eq!(cli(&args, &mut stdout, &mut err), 2);
        assert!(stdout.is_empty(), "refusal stdout must be empty");
    }
    let failure = m.render(&mut FailWrite).unwrap_err();
    assert!(failure.contains("output write") && failure.contains("fixture closed"));
    for (remaining, flush_failure) in [
        (text.len() / 2, false),
        (text.find("integrator_residual ").unwrap(), false),
        (usize::MAX, true),
    ] {
        let mut writer = FailAfter {
            bytes: Vec::new(),
            remaining,
            flush_failure,
        };
        assert!(m.render(&mut writer).unwrap_err().contains("output write"));
        assert!(
            !String::from_utf8(writer.bytes)
                .unwrap()
                .contains("measurement_a: ok"),
            "partial data or pre-success flush failure must not emit success"
        );
    }
    err.clear();
    assert_eq!(cli(&["--test-a".into()], &mut FailWrite, &mut err), 1);
    let err = String::from_utf8(err).unwrap();
    assert!(err.starts_with("measurement_a: FAIL output write:"));
    assert!(!err.contains("measurement_a: ok"));
    let doc = include_str!("../docs/STAGE6-M4-M5.md");
    assert!(frozen(doc).is_ok());
    assert!(frozen("").is_err());
    for bad in [
        doc.replace("--gravity 0", "--gravity NaN"),
        doc.replace("--dt 0.001", "--dt inf"),
        doc.replace("--substeps 10", "--substeps -1"),
        doc.replace("--substeps 10", "--substeps 4294967296"),
        doc.replace("--pulse-sensor sensor", "--pulse-sensor C:\\sensor"),
        doc.replace("--dt 0.001", "--dt 0.001 --dt 0.001"),
        doc.replace("--gravity 0", &format!("--{}", "x".repeat(1_048_576))),
    ] {
        assert!(frozen(&bad).is_err(), "invalid frozen flags must fail closed");
    }
    let (_, mut body) = inputs().unwrap();
    body.materials[0].break_strain = Some(0.5);
    assert!(validate_body(&body).unwrap_err().contains("broken-beam"));
    let (cfg, mut body) = inputs().unwrap();
    let mut broken = Sim::from_car(cfg, &body).unwrap();
    broken.break_strain[0] = Some(0.5);
    let original = broken.pos.clone();
    let mut solver = SymplecticEuler::default();
    let materials = vec![0; broken.beams.len()];
    for enabled in [false, true] {
        let mut totals = Totals::default();
        assert!(
            split_step(
                &mut broken,
                &mut solver,
                m.h,
                &materials,
                enabled.then_some(&mut totals),
            )
            .is_err()
        );
        assert_eq!(broken.pos, original, "refuse before physical update");
    }
    body.tets.push(crash_sim::format::Tet {
        nodes: [0, 1, 2, 3],
        solid: "refused_fixture".into(),
    });
    assert!(validate_body(&body).unwrap_err().contains("tet"));
    let mut corrupt = m.clone();
    corrupt.rows[0].velocity[0] = f64::NAN;
    let mut stdout = Vec::new();
    assert!(corrupt.render(&mut stdout).is_err());
    assert!(stdout.is_empty(), "validate before output");
}
