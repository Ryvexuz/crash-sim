//! Rolling resistance and aero drag (task t-20261120-221319-2; docs/DRIVE_FEEL.md fix 3): a car
//! that lifts off slows down, at the rate the two forces predict, and a car without the
//! `rolling_resistance` and `drag_area` keys runs exactly as before the keys existed.

use crash_sim::format::{parse, Body};
use crash_sim::vec3::Vec3;
use crash_sim::vehicle::{
    rolling_resistance_moment, Car, AIR_DENSITY, ROLLING_RESISTANCE_RAMP_SPEED,
};
use crash_sim::{Config, Sim};

fn scenario(name: &str) -> String {
    let path = format!("{}/scenarios/{name}", env!("CARGO_MANIFEST_DIR"));
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"))
}

fn car_basic() -> Body {
    parse(&scenario("car_basic.bsn")).unwrap_or_else(|e| panic!("car_basic.bsn: {e}"))
}

/// `body` with both keys removed (the car as it was before the keys existed).
fn without_keys(body: &Body) -> Body {
    let mut b = body.clone();
    let c = b.controls.as_mut().unwrap();
    c.rolling_resistance = None;
    c.drag_area = None;
    b
}

/// One `advance` is one 1 ms substep.
const H: f64 = 0.001;

fn cfg(duration: f64) -> Config {
    Config {
        duration,
        dt: H,
        substeps: 1,
        ..Config::default()
    }
}

/// The car's total mass (chassis nodes and wheels, kg) and its effective mass for a change of
/// speed, M + sum of I / r^2 (kg): rolling wheels must be spun down with the car.
fn masses(sim: &Sim) -> (f64, f64) {
    let car = sim.car.as_ref().unwrap();
    let m = sim.mass.iter().sum::<f64>() + car.wheels.iter().map(|w| w.p.mass).sum::<f64>();
    let rot = car
        .wheels
        .iter()
        .map(|w| w.p.inertia / (w.p.radius * w.p.radius))
        .sum::<f64>();
    (m, m + rot)
}

/// Horizontal speed (m/s) of the whole car's centre of mass (chassis and wheels).
fn speed(sim: &Sim, m: f64) -> f64 {
    let car = sim.car.as_ref().unwrap();
    let mut p = Vec3::ZERO;
    for (v, mi) in sim.vel.iter().zip(&sim.mass) {
        p += *v * *mi;
    }
    for w in &car.wheels {
        p += w.vel * w.p.mass;
    }
    let v = p * (1.0 / m);
    (v.x * v.x + v.z * v.z).sqrt()
}

/// `body` (car_basic.bsn's car, 0.2 m above flat ground) started rolling along +x at `v0` m/s:
/// every chassis node and wheel centre moves at v0, and every wheel spins at v0 / r (no slip).
/// Throttle, brake and steer stay 0. Returns the speed after every substep, (t, v).
fn coast(body: &Body, v0: f64, seconds: f64) -> (Vec<(f64, f64)>, f64, f64) {
    let mut sim = Sim::from_car(cfg(seconds), body).unwrap_or_else(|e| panic!("{e}"));
    for v in sim.vel.iter_mut() {
        v.x += v0;
    }
    for w in sim.car.as_mut().unwrap().wheels.iter_mut() {
        w.vel.x += v0;
        w.spin = v0 / w.p.radius;
    }
    let (m, m_eff) = masses(&sim);
    let n = (seconds / H).round() as u64;
    let mut out = Vec::with_capacity(n as usize);
    for i in 1..=n {
        sim.advance();
        sim.check()
            .unwrap_or_else(|e| panic!("unstable at {}: {e}", i as f64 * H));
        out.push((i as f64 * H, speed(&sim, m)));
    }
    (out, m, m_eff)
}

/// The deceleration (m/s^2, positive when slowing) at speed `v_at`: the least-squares slope of
/// v(t) over the `half` s on each side of the first substep at or after `from` s where the speed
/// falls to `v_at`. None when it never gets there.
fn decel_at(run: &[(f64, f64)], from: f64, v_at: f64, half: f64) -> Option<(f64, f64)> {
    let tc = run.iter().find(|(t, v)| *t >= from && *v <= v_at)?.0;
    let win: Vec<_> = run
        .iter()
        .filter(|(t, _)| (*t - tc).abs() <= half + 1e-9)
        .collect();
    assert!(
        win.len() > 100,
        "window around t = {tc} has {} samples",
        win.len()
    );
    let n = win.len() as f64;
    let (mt, mv) = (
        win.iter().map(|p| p.0).sum::<f64>() / n,
        win.iter().map(|p| p.1).sum::<f64>() / n,
    );
    let sxy: f64 = win.iter().map(|p| (p.0 - mt) * (p.1 - mv)).sum();
    let sxx: f64 = win.iter().map(|p| (p.0 - mt) * (p.0 - mt)).sum();
    Some((tc, -sxy / sxx))
}

/// The speed at which the done check measures (m/s): the control lap's 13.85 m/s
/// (docs/DRIVE_FEEL.md, fix 3).
const V_CHECK: f64 = 13.85;
/// The start speed (m/s): above V_CHECK by about 4 s of the predicted deceleration, so the car
/// has dropped its 0.2 m, landed and stopped bouncing before it gets down to V_CHECK.
const V_START: f64 = 14.6;
/// The car's suspension (1.25 Hz, damping ratio 0.3, docs/FORMAT.md) has decayed by e^-9 after
/// 3 s, so the speed is read only from then on.
const SETTLED: f64 = 3.0;

/// The done check: from 13.85 m/s with throttle, brake and steer 0 on flat ground, the speed
/// falls monotonically, and the deceleration at 13.85 m/s matches
/// (c_rr m g + 0.5 rho CdA v^2) / m_eff within 5%, with m the car's mass (chassis and wheels)
/// and m_eff = m + sum of I / r^2, its effective mass (the wheels spin down with the car; for
/// car_basic.bsn m = 1200 kg and m_eff = 1260 kg). The same car without the two keys (the car
/// as it was) does not slow at all, so this test fails without the forces.
#[test]
fn coast_down_matches_rolling_resistance_and_drag() {
    let body = car_basic();
    let c = body.controls.as_ref().unwrap();
    let (c_rr, cda) = (c.rolling_resistance.unwrap(), c.drag_area.unwrap());
    let g = Config::default().gravity;
    let (run, m, m_eff) = coast(&body, V_START, 12.0);
    let force = c_rr * m * g + 0.5 * AIR_DENSITY * cda * V_CHECK * V_CHECK;
    let predicted = force / m_eff;
    let predicted_plain_m = force / m;
    let (tc, measured) = decel_at(&run, SETTLED, V_CHECK, 1.0).unwrap_or_else(|| {
        panic!(
            "the car never slowed to {V_CHECK} m/s: last {:?}",
            run.last()
        )
    });
    println!(
        "coast_down c_rr={c_rr} cda_m2={cda} m_kg={m:.1} m_eff_kg={m_eff:.1} at_t={tc:.3} \
         measured_mps2={measured:.5} predicted_m_eff_mps2={predicted:.5} \
         predicted_plain_m_mps2={predicted_plain_m:.5} measured_g={:.5} \
         err_vs_m_eff={:.4} err_vs_plain_m={:.4}",
        measured / g,
        measured / predicted - 1.0,
        measured / predicted_plain_m - 1.0
    );
    assert!(
        (measured / predicted - 1.0).abs() <= 0.05,
        "deceleration {measured} m/s^2 at {V_CHECK} m/s is not within 5% of {predicted}"
    );
    // monotonic: from the landing settled on, every substep is slower than the one before
    let after: Vec<_> = run.iter().filter(|(t, _)| *t >= SETTLED).collect();
    let mut rises = 0;
    let mut worst = 0.0_f64;
    for w in after.windows(2) {
        if w[1].1 >= w[0].1 {
            rises += 1;
            worst = worst.max(w[1].1 - w[0].1);
        }
    }
    println!(
        "coast_down monotonic from_t={SETTLED} to_t={:.3} substeps={} rises={rises} worst_rise_mps={worst:e} \
         v_from={:.4} v_to={:.4}",
        after.last().unwrap().0,
        after.len(),
        after[0].1,
        after.last().unwrap().1
    );
    assert_eq!(
        rises, 0,
        "the speed rose at {rises} substeps (worst {worst} m/s)"
    );

    // control: the same car without the keys (as before they existed) keeps its speed
    let (old, _, _) = coast(&without_keys(&body), V_START, 12.0);
    let v_old: Vec<f64> = old
        .iter()
        .filter(|(t, _)| *t >= SETTLED)
        .map(|p| p.1)
        .collect();
    let drop_old = v_old[0] - v_old[v_old.len() - 1];
    let drop_new = after[0].1 - after.last().unwrap().1;
    println!(
        "coast_down control_without_keys speed_drop_mps={drop_old:.6} with_keys={drop_new:.6}"
    );
    assert!(
        drop_old.abs() < 0.05 * drop_new,
        "without the keys the car must not slow: it lost {drop_old} m/s against {drop_new}"
    );
}

/// The rolling resistance is zero at standstill: the car dropped onto flat ground and left at
/// rest comes to rest exactly as the car without the keys does, with no creep.
#[test]
fn rolling_resistance_does_not_creep_at_rest() {
    let body = car_basic();
    for b in [body.clone(), without_keys(&body)] {
        let mut sim = Sim::from_car(cfg(12.0), &b).unwrap();
        let (m, _) = masses(&sim);
        let mut late = 0.0_f64;
        let mut x_at_8 = 0.0;
        for i in 1..=12_000u64 {
            sim.advance();
            sim.check().unwrap();
            let t = i as f64 * H;
            if (t - 8.0).abs() < 0.5 * H {
                x_at_8 = sim.pos.iter().map(|p| p.x).sum::<f64>() / sim.pos.len() as f64;
            }
            if t >= 8.0 {
                late = late.max(speed(&sim, m));
                late = late.max(sim.car.as_ref().unwrap().max_speed());
            }
        }
        let x_end = sim.pos.iter().map(|p| p.x).sum::<f64>() / sim.pos.len() as f64;
        let keys = b.controls.as_ref().unwrap().rolling_resistance.is_some();
        println!(
            "at_rest keys={keys} max_speed_after_8s={late:e} drift_8_to_12_m={:e}",
            x_end - x_at_8
        );
        assert!(
            late < 1e-4,
            "keys={keys}: still moving at {late} m/s after 8 s"
        );
        assert!(
            (x_end - x_at_8).abs() < 1e-5,
            "keys={keys}: crept {} m",
            x_end - x_at_8
        );
    }
}

/// A missing key means 0: a car whose keys are written as 0 runs bit for bit as the car without
/// them, over a drive that uses throttle, brake and steer.
#[test]
fn zero_keys_run_bit_for_bit_as_missing_keys() {
    let body = without_keys(&car_basic());
    let mut zero = body.clone();
    let c = zero.controls.as_mut().unwrap();
    c.rolling_resistance = Some(0.0);
    c.drag_area = Some(0.0);
    let run = |b: &Body| {
        let mut sim = Sim::from_car(cfg(6.0), b).unwrap();
        let mut bits = Vec::new();
        for i in 0..6000u64 {
            let t = i as f64 * H;
            let (thr, brk, st) = if t < 3.0 {
                (0.6, 0.0, 0.4)
            } else {
                (0.0, 0.3, -0.2)
            };
            sim.car
                .as_mut()
                .unwrap()
                .set_controls(thr, brk, st)
                .unwrap();
            sim.advance();
            sim.check().unwrap();
        }
        for p in &sim.pos {
            bits.push((p.x.to_bits(), p.y.to_bits(), p.z.to_bits()));
        }
        for w in &sim.car.as_ref().unwrap().wheels {
            bits.push((w.pos.x.to_bits(), w.spin.to_bits(), w.vel.x.to_bits()));
        }
        bits
    };
    assert_eq!(run(&zero), run(&body));
}

/// `Car::new` checks the two numbers again (a `Body` need not come from `format::parse`):
/// negative, NaN or infinite values are refused by name.
#[test]
fn car_refuses_bad_rolling_resistance_or_drag_area() {
    let body = car_basic();
    let built = body.build();
    for bad in [
        -0.01,
        -f64::MIN_POSITIVE,
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
    ] {
        for key in ["rolling_resistance", "drag_area"] {
            let mut b = body.clone();
            let c = b.controls.as_mut().unwrap();
            match key {
                "rolling_resistance" => c.rolling_resistance = Some(bad),
                _ => c.drag_area = Some(bad),
            }
            let e = Car::new(&b, &built.pos, &built.vel, &built.mass, 1, 9.81)
                .err()
                .unwrap_or_else(|| panic!("{key} {bad} was accepted"));
            assert!(
                e.contains(&format!("{key} {bad} must be a finite number >= 0")),
                "{key} {bad}: {e}"
            );
            let e = Sim::from_car(cfg(1.0), &b)
                .err()
                .unwrap_or_else(|| panic!("sim took {key} {bad}"));
            assert!(e.contains(key), "{e}");
        }
    }
}

/// The low-speed ramp of the rolling resistance moment: exactly 0 at zero spin, odd-symmetric in
/// size (the same for either spin direction), smooth and linear near zero with the slope
/// c_rr N r^2 / v0, full x tanh(1) at the rolling speed v0, monotone, and full (to 1e-12) at
/// 15 v0. Checked against hand values, not against the function's own
/// formula: tanh(1) = 0.7615941559557649.
#[test]
fn rolling_resistance_moment_ramps_smoothly_from_zero() {
    let (c_rr, n, r) = (0.012, 2943.0, 0.3175);
    let full = c_rr * n * r; // 11.2128... N m
    let v0 = ROLLING_RESISTANCE_RAMP_SPEED;
    let m = |spin: f64| rolling_resistance_moment(c_rr, n, r, spin);
    assert_eq!(m(0.0), 0.0);
    assert_eq!(m(-0.0), 0.0);
    for s in [1e-300, 0.01, 0.3, 3.0, 300.0] {
        assert_eq!(
            m(s),
            m(-s),
            "spin {s}: the size must not depend on the direction"
        );
        assert!(m(s) > 0.0 && m(s) <= full, "spin {s}: {}", m(s));
    }
    // at the rolling speed v0 (spin v0 / r): full x tanh(1)
    let at_v0 = m(v0 / r);
    assert!(
        (at_v0 / full - 0.761_594_155_955_764_9).abs() < 1e-12,
        "{at_v0}"
    );
    // linear near zero: slope c_rr N r^2 / v0
    let tiny = 1e-6;
    let slope = m(tiny) / tiny;
    let want = c_rr * n * r * r / v0;
    assert!((slope / want - 1.0).abs() < 1e-9, "slope {slope} vs {want}");
    // a subnormal spin gives a finite, non-negative moment (no NaN)
    assert!(m(5e-324).is_finite() && m(5e-324) >= 0.0);
    // full above 15 v0
    assert!((m(15.0 * v0 / r) / full - 1.0).abs() < 1e-12);
    // monotone in |spin|
    let mut last = 0.0;
    for k in 1..=2000 {
        let v = m(k as f64 * 1e-3 / r);
        assert!(
            v >= last,
            "not monotone at rolling speed {} m/s",
            k as f64 * 1e-3
        );
        last = v;
    }
}

/// Extreme but valid values stay finite and only slow the car: a drag area of 1e6 m^2 and a
/// rolling resistance coefficient of 1e6 (both far beyond any car) cannot reverse the car or
/// blow the run up, because each force can at most stop its motion within a substep. The
/// smallest positive values (subnormal) change nothing measurable.
#[test]
fn extreme_values_stay_finite_and_never_reverse() {
    let base = car_basic();
    for (rr, cda) in [
        (1e6, 0.0),
        (0.0, 1e6),
        (1e6, 1e6),
        (5e-324, 5e-324),
        (f64::MAX, f64::MAX),
    ] {
        let mut b = base.clone();
        let c = b.controls.as_mut().unwrap();
        c.rolling_resistance = Some(rr);
        c.drag_area = Some(cda);
        let (run, _, _) = coast(&b, 10.0, 4.0);
        let vx_ok = run.iter().all(|(_, v)| v.is_finite() && *v <= 10.0 + 1e-6);
        let last = run.last().unwrap().1;
        println!("extreme rr={rr:e} cda={cda:e} v_end={last:.6}");
        assert!(
            vx_ok,
            "rr {rr} cda {cda}: speed not finite or above the start"
        );
        if rr >= 1.0 || cda >= 1.0 {
            assert!(
                last < 9.0,
                "rr {rr} cda {cda}: huge resistance must slow the car, v = {last}"
            );
        }
    }
}
