//! Wheels, suspension and tyres (nb-39, docs/STAGE6.md checks 3.3 and 3.4, docs/PHYSICS.md
//! "Wheels, suspension and tyres"). The settle numbers are nb-30's (docs/STAGE6.md check 3.2):
//! T = 8 s, S = 4 s, V = 0.001 m/s, E = 0.01 J, G = 0.01, measured at every 1 ms substep.

use crash_sim::format::{parse, Body};
use crash_sim::vec3::Vec3;
use crash_sim::{settle, solver, vehicle, Config, Sim};
use std::process::Command;

fn scenario(name: &str) -> String {
    let path = format!("{}/scenarios/{name}", env!("CARGO_MANIFEST_DIR"));
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"))
}

fn car() -> Body {
    parse(&scenario("car_basic.bsn")).unwrap_or_else(|e| panic!("car_basic.bsn: {e}"))
}

/// One `advance` is one 1 ms substep, as in the settle report.
fn substep_cfg(duration: f64) -> Config {
    Config {
        duration,
        dt: 0.001,
        substeps: 1,
        ..Config::default()
    }
}

/// What a settle-style run measured.
struct Run {
    sim: Sim,
    max_speed_after: f64,
    rise_after: f64,
    gain: f64,
    /// Chassis centre-of-mass velocity along x at each whole second (index = second).
    u_at_second: Vec<f64>,
}

fn chassis_com_vx(sim: &Sim) -> f64 {
    let m: f64 = sim.mass.iter().sum();
    sim.vel
        .iter()
        .zip(&sim.mass)
        .map(|(v, m)| v.x * m)
        .sum::<f64>()
        / m
}

/// Run `sim` for `seconds` of 1 ms substeps and measure the settle numbers after `by`.
fn run(mut sim: Sim, seconds: u64, by: f64) -> Run {
    let total = seconds * 1000;
    let e0 = sim.total_energy();
    assert!(e0.is_finite() && e0 > 0.0, "E(0) = {e0}");
    let mut max_speed_after = 0.0_f64;
    let mut min_e_after = f64::INFINITY;
    let mut rise_after = 0.0_f64;
    let mut gain = 0.0_f64;
    let mut u_at_second = vec![chassis_com_vx(&sim)];
    for i in 1..=total {
        sim.advance();
        sim.check()
            .unwrap_or_else(|e| panic!("unstable at substep {i}: {e}"));
        let t = i as f64 * 0.001;
        let e = sim.total_energy();
        assert!(e.is_finite(), "energy at substep {i} is {e}");
        gain = gain.max((e - e0) / e0);
        if t >= by - 1e-9 {
            max_speed_after = max_speed_after.max(sim.max_speed());
            min_e_after = min_e_after.min(e);
            rise_after = rise_after.max(e - min_e_after);
        }
        if i % 1000 == 0 {
            u_at_second.push(chassis_com_vx(&sim));
        }
    }
    Run {
        sim,
        max_speed_after,
        rise_after,
        gain,
        u_at_second,
    }
}

fn assert_settled(r: &Run, what: &str) {
    println!(
        "{what}: max_speed_after_8s={:.3e} energy_rise_after_8s_j={:.3e} energy_gain={:.3e}",
        r.max_speed_after, r.rise_after, r.gain
    );
    assert!(
        r.max_speed_after < 0.001,
        "{what}: speed {:.3e} m/s >= 0.001 after 8 s",
        r.max_speed_after
    );
    assert!(
        r.rise_after <= 0.01,
        "{what}: energy rise {:.3e} J > 0.01 after 8 s",
        r.rise_after
    );
    assert!(
        r.gain <= 0.01,
        "{what}: energy gain {:.3e} > 0.01 of E(0)",
        r.gain
    );
}

/// docs/STAGE6.md check 3.3: the car dropped from 0.2 m settles, free-rolling, and rests level
/// as statics predicts, (a) to (e).
#[test]
fn car_settles_level() {
    let body = car();
    let sim = Sim::from_car(substep_cfg(12.0), &body).unwrap();
    let r = run(sim, 12, 8.0);
    assert_settled(&r, "flat, free-rolling");
    let sim = &r.sim;
    let car = sim.car.as_ref().unwrap();
    let g = sim.cfg.gravity;
    let wheel = |name: &str| {
        car.wheels
            .iter()
            .find(|w| w.p.name == name)
            .unwrap_or_else(|| panic!("wheel {name}"))
    };
    let m_total: f64 =
        sim.mass.iter().sum::<f64>() + car.wheels.iter().map(|w| w.p.mass).sum::<f64>();
    // (a) the four tyre loads carry the whole weight
    let loads: f64 = car.wheels.iter().map(|w| w.total_load()).sum();
    println!("(a) sum of loads {loads:.3} N, weight {:.3} N", m_total * g);
    assert!(
        (loads - m_total * g).abs() <= 0.001 * m_total * g,
        "(a) loads {loads} vs weight {}",
        m_total * g
    );
    // (b) left and right agree on each axle
    for (l, rgt) in [("fl", "fr"), ("rl", "rr")] {
        let (a, b) = (wheel(l), wheel(rgt));
        println!(
            "(b) {l} {:.3} N {:.6} m, {rgt} {:.3} N {:.6} m",
            a.total_load(),
            a.compression(),
            b.total_load(),
            b.compression()
        );
        assert!(
            (a.total_load() - b.total_load()).abs() <= 0.01 * a.total_load().max(b.total_load()),
            "(b) loads {l}/{rgt}"
        );
        assert!(
            (a.compression() - b.compression()).abs() <= 0.001,
            "(b) compressions {l}/{rgt}"
        );
    }
    for w in &car.wheels {
        let p = &w.p;
        // (c) compression from the spring's share of the load, off both stops
        let want = (w.total_load() - p.mass * g) / p.spring;
        println!(
            "(c) {} compression {:.6} m, statics {want:.6} m, length {:.6} m",
            p.name,
            w.compression(),
            w.length
        );
        assert!(
            (w.compression() - want).abs() <= 0.05 * want.abs(),
            "(c) {} compression {} vs {want}",
            p.name,
            w.compression()
        );
        assert!(
            p.travel_min < w.length && w.length < p.travel_max,
            "(c) {} on a travel stop",
            p.name
        );
        // (e) ride frequency and damping ratio of the corner
        let mc = (w.total_load() - p.mass * g) / g;
        let f = (p.spring / mc).sqrt() / (2.0 * std::f64::consts::PI);
        let zeta = p.damper / (2.0 * (p.spring * mc).sqrt());
        println!(
            "(e) {} corner mass {mc:.2} kg, ride {f:.3} Hz, damping ratio {zeta:.3}",
            p.name
        );
        assert!(
            (1.0..=2.0).contains(&f),
            "(e) {} ride frequency {f}",
            p.name
        );
        assert!(
            (0.2..=0.5).contains(&zeta),
            "(e) {} damping ratio {zeta}",
            p.name
        );
    }
    // (d) axle loads from the moment balance about the centre of mass, at the contact points
    let x_cm = (sim
        .pos
        .iter()
        .zip(&sim.mass)
        .map(|(p, m)| p.x * m)
        .sum::<f64>()
        + car.wheels.iter().map(|w| w.pos.x * w.p.mass).sum::<f64>())
        / m_total;
    let x_front = (wheel("fl").pos.x + wheel("fr").pos.x) / 2.0;
    let x_rear = (wheel("rl").pos.x + wheel("rr").pos.x) / 2.0;
    let w_front = wheel("fl").total_load() + wheel("fr").total_load();
    let w_front_pred = m_total * g * (x_rear - x_cm) / (x_rear - x_front);
    println!("(d) front axle {w_front:.3} N, moment balance {w_front_pred:.3} N (x_cm {x_cm:.6})");
    assert!(
        (w_front - w_front_pred).abs() <= 0.02 * w_front_pred,
        "(d) front {w_front} vs {w_front_pred}"
    );
    // level: the chassis up axis is vertical
    println!(
        "level: up = ({:.2e}, {:.9}, {:.2e})",
        car.frame.up.x, car.frame.up.y, car.frame.up.z
    );
    assert!(
        (car.frame.up - Vec3::new(0.0, 1.0, 0.0)).length() < 1e-3,
        "chassis not level: up {:?}",
        car.frame.up
    );
}

fn braked_on_slope(grip_frac: f64, seconds: u64) -> Run {
    let body = car();
    let mut sim = Sim::from_car(substep_cfg(seconds as f64), &body).unwrap();
    sim.set_gravity_tilt(vehicle::slope_angle(&body, grip_frac).unwrap())
        .unwrap();
    sim.car.as_mut().unwrap().brake = true;
    run(sim, seconds, 8.0)
}

/// docs/STAGE6.md check 3.4 (a): braked, on a slope of 0.8 times the grip slope, the car drops,
/// settles and holds, with the settle numbers of check 3.2.
#[test]
fn braked_car_holds_below_grip_slope() {
    let r = braked_on_slope(0.8, 12);
    assert_settled(&r, "slope 0.8 x grip, braked");
    // Held, the car's loads follow statics on the slope: the tyres carry M g cos theta, and
    // taking moments about the rear contact points (y = 0, where the friction forces act),
    //     W_front (x_f - x_r) = M g (cos theta (x_cm - x_r) + sin theta y_cm),
    // so the downhill (front) axle carries more than the uphill one.
    let sim = &r.sim;
    let car = sim.car.as_ref().unwrap();
    let theta = sim.gravity_tilt();
    let g = sim.cfg.gravity;
    let m_total: f64 =
        sim.mass.iter().sum::<f64>() + car.wheels.iter().map(|w| w.p.mass).sum::<f64>();
    let moment = |f: &dyn Fn(Vec3) -> f64| {
        (sim.pos
            .iter()
            .zip(&sim.mass)
            .map(|(p, m)| f(*p) * m)
            .sum::<f64>()
            + car.wheels.iter().map(|w| f(w.pos) * w.p.mass).sum::<f64>())
            / m_total
    };
    let (x_cm, y_cm) = (moment(&|p| p.x), moment(&|p| p.y));
    let axle = |a: &str, b: &str| {
        let (wa, wb) = (
            car.wheels.iter().find(|w| w.p.name == a).unwrap(),
            car.wheels.iter().find(|w| w.p.name == b).unwrap(),
        );
        (
            (wa.pos.x + wb.pos.x) / 2.0,
            wa.total_load() + wb.total_load(),
        )
    };
    let ((x_f, w_f), (x_r, w_r)) = (axle("fl", "fr"), axle("rl", "rr"));
    let normal = m_total * g * theta.cos();
    println!(
        "slope loads: front {w_f:.3} N, rear {w_r:.3} N, sum {:.3} N, M g cos theta {normal:.3} N",
        w_f + w_r
    );
    assert!(
        (w_f + w_r - normal).abs() <= 0.001 * normal,
        "loads {} vs M g cos theta {normal}",
        w_f + w_r
    );
    let w_f_pred = m_total * g * (theta.cos() * (x_cm - x_r) + theta.sin() * y_cm) / (x_f - x_r);
    println!("slope front axle {w_f:.3} N, moment balance {w_f_pred:.3} N (x_cm {x_cm:.4}, y_cm {y_cm:.4})");
    assert!(
        w_f > w_r,
        "the downhill axle carries less than the uphill one: front {w_f}, rear {w_r}"
    );
    assert!(
        (w_f - w_f_pred).abs() <= 0.02 * w_f_pred,
        "front axle {w_f} vs moment balance {w_f_pred}"
    );
}

/// docs/STAGE6.md check 3.4 (b): above the grip slope (1.2 times), the braked car slides with
/// the friction acceleration g (sin theta - mu cos theta), within 15%, between 2 and 4 s.
#[test]
fn braked_car_slides_above_grip_slope() {
    let body = car();
    let theta = vehicle::slope_angle(&body, 1.2).unwrap();
    let mu = body.tyres[0].mu;
    let r = braked_on_slope(1.2, 4);
    let accel = (r.u_at_second[4] - r.u_at_second[2]) / 2.0;
    let pred = 9.81 * (theta.sin() - mu * theta.cos());
    println!(
        "slope_tan={:.6} slope_accel_mps2={accel:.4} slope_predicted_mps2={pred:.4}",
        theta.tan()
    );
    assert!(pred > 0.0);
    assert!(
        (accel - pred).abs() <= 0.15 * pred,
        "accel {accel} vs predicted {pred}"
    );
    // the wheels stay locked (they turn only with the chassis): the slide is on the tyres, not
    // rolling, so every rim speed is a tiny part of the slide speed
    let u = r.u_at_second[4];
    for w in &r.sim.car.as_ref().unwrap().wheels {
        println!(
            "wheel {} rim speed {:.3e} m/s, slide {u:.4} m/s",
            w.p.name,
            w.spin.abs() * w.p.radius
        );
        assert!(
            w.spin.abs() * w.p.radius < 0.01 * u,
            "wheel {} rolls while braked: spin {}",
            w.p.name,
            w.spin
        );
    }
}

/// Free-rolling on the same slope the car rolls away (the hold above is the brake's doing).
#[test]
fn unbraked_car_rolls_on_the_hold_slope() {
    let body = car();
    let mut sim = Sim::from_car(substep_cfg(3.0), &body).unwrap();
    sim.set_gravity_tilt(vehicle::slope_angle(&body, 0.8).unwrap())
        .unwrap();
    let r = run(sim, 3, 0.0);
    println!("free-rolling u(3 s) = {:.4} m/s", r.u_at_second[3]);
    assert!(
        r.u_at_second[3] > 1.0,
        "the free-rolling car did not roll: u = {}",
        r.u_at_second[3]
    );
}

/// The wheels' springs and masses are in the step-limit check: a car whose chassis alone passes
/// at 1 ms is refused when its tyres are made too stiff for that step.
#[test]
fn car_step_limit_counts_suspension_and_wheels() {
    let mut body = car();
    let built = body.build();
    assert_eq!(
        solver::body_step_error(0.001, &built.beams, &built.mass),
        None,
        "the chassis alone passes"
    );
    assert!(
        Sim::from_car(Config::default(), &body).is_ok(),
        "car_basic passes at 1 ms"
    );
    for t in &mut body.tyres {
        t.radial_stiffness = 5.0e7;
    }
    let e = Sim::from_car(Config::default(), &body)
        .err()
        .expect("a 50 MN/m tyre on a 30 kg wheel is refused at 1 ms");
    println!("{e}");
    assert!(e.contains("wheels, suspension and tyres"), "{e}");
    let mut body = car();
    for w in &mut body.wheels {
        w.inertia = 0.0005;
    }
    let e = Sim::from_car(Config::default(), &body)
        .err()
        .expect("a nearly massless spin is refused at 1 ms");
    assert!(e.contains("stability limit"), "{e}");
}

/// Until the command-line runs simulate wheels, they refuse a body with wheels instead of
/// dropping them (`Sim::from_body` is the one constructor they all use).
#[test]
fn body_with_wheels_is_refused_where_wheels_are_not_simulated() {
    let e = Sim::from_body(Config::default(), &car())
        .err()
        .expect("from_body refuses wheels");
    assert!(e.contains("4 wheels") && e.contains("from_car"), "{e}");
    let path = format!("{}/scenarios/car_basic.bsn", env!("CARGO_MANIFEST_DIR"));
    let cases: [&[&str]; 3] = [
        &["--body", &path],
        &["--body", &path, "--summary"],
        &[
            "--body",
            &path,
            "--settle-report",
            "--settle-by",
            "8",
            "--settle-hold",
            "4",
            "--settle-speed",
            "0.001",
            "--energy-rise",
            "0.01",
            "--energy-gain",
            "0.01",
            "--crumple-report",
            "--crumple-groups",
            "rear,front",
            "--crumple-min",
            "0.05",
            "--crumple-max",
            "1.0",
            "--crumple-drift",
            "0.001",
        ],
    ];
    for args in cases {
        let o = Command::new(crash_sim_bin()).args(args).output().unwrap();
        let (out, err) = (
            String::from_utf8_lossy(&o.stdout),
            String::from_utf8_lossy(&o.stderr),
        );
        println!(
            "{args:?}: exit {:?}, stdout {out:?}, stderr {err:?}",
            o.status.code()
        );
        assert!(
            matches!(o.status.code(), Some(1) | Some(2)),
            "{args:?}: exit {:?}",
            o.status.code()
        );
        assert!(
            format!("{out}{err}").contains("4 wheels, which this run does not simulate"),
            "{args:?}"
        );
        assert!(
            !out.contains("settle: ok") && !out.starts_with("t,"),
            "{args:?} printed a result"
        );
    }
}

#[test]
fn slope_angle_follows_grip_and_refuses_what_has_no_grip() {
    let body = car();
    let theta = vehicle::slope_angle(&body, 1.0).unwrap();
    assert!((theta.tan() - 0.88).abs() < 1e-12);
    let mut mixed = body.clone();
    mixed.tyres[1].mu = 0.7;
    assert!(vehicle::slope_angle(&mixed, 1.0)
        .unwrap_err()
        .contains("mu differ"));
    let bare = parse(&scenario("box.bsn")).unwrap();
    assert!(vehicle::slope_angle(&bare, 1.0)
        .unwrap_err()
        .contains("no tyres"));
    assert!(vehicle::slope_angle(&body, f64::NAN).is_err());
    assert!(vehicle::slope_angle(&body, -1.0).is_err());
    let mut sim = Sim::from_car(Config::default(), &body).unwrap();
    assert!(sim.set_gravity_tilt(f64::NAN).is_err());
    assert!(sim.set_gravity_tilt(2.0).is_err());
}

/// Wheel numbers are checked again in `from_car` (a `Body` need not come from `parse`).
#[test]
fn from_car_refuses_bad_wheel_numbers() {
    type Spoil = fn(&mut Body);
    let cases: [(&str, Spoil); 5] = [
        ("inertia must be > 0", |b| b.wheels[0].inertia = -1.0),
        ("radius must be > 0", |b| b.wheels[0].radius = f64::NAN),
        ("travel min < rest", |b| b.suspensions[0].rest = 5.0),
        ("lat_stiffness must be > 0", |b| {
            b.tyres[0].lat_stiffness = 0.0
        }),
        ("has no tyre", |b| {
            b.tyres.remove(0);
        }),
    ];
    for (want, f) in cases {
        let mut b = car();
        f(&mut b);
        let e = Sim::from_car(Config::default(), &b)
            .err()
            .unwrap_or_else(|| panic!("not refused: {want}"));
        assert!(e.contains(want), "{want}: {e}");
    }
    let mut b = car();
    b.controls = None;
    assert!(Sim::from_car(Config::default(), &b)
        .err()
        .unwrap()
        .contains("controls"));
}

/// A car in free fall, nothing touching: no tyre or stop force, and the constraint that keeps
/// each wheel on its axis moves wheel and chassis together, so the total momentum grows by
/// exactly gravity's impulse.
#[test]
fn free_fall_keeps_momentum_and_wheels_on_their_axes() {
    let mut text = scenario("car_basic.bsn");
    // the same car, every node 10 m higher (the wheels hang from their mounts, so they follow)
    text = text
        .lines()
        .map(|l| {
            let t: Vec<&str> = l.split(' ').collect();
            if t[0] == "node" {
                let y: f64 = t[3].parse().unwrap();
                format!(
                    "node {} {} {} {} mass {}\n",
                    t[1],
                    t[2],
                    y + 10.0,
                    t[4],
                    t[6]
                )
            } else {
                format!("{l}\n")
            }
        })
        .collect();
    let body = parse(&text).unwrap();
    let momentum = |s: &Sim| {
        let mut p = Vec3::ZERO;
        for (v, m) in s.vel.iter().zip(&s.mass) {
            p += *v * *m;
        }
        for w in &s.car.as_ref().unwrap().wheels {
            p += w.vel * w.p.mass;
        }
        p
    };
    // Untilted gravity acts along every axis; tilted by 0.5 rad it also pulls each wheel across
    // its axis, so the carrier and the constraint must take the wheel's mass along.
    for tilt in [0.0, 0.5] {
        let mut sim = Sim::from_car(substep_cfg(0.5), &body).unwrap();
        sim.set_gravity_tilt(tilt).unwrap();
        let m_total: f64 = sim.mass.iter().sum::<f64>() + 120.0;
        for _ in 0..500 {
            sim.advance();
            sim.check().unwrap();
        }
        let p = momentum(&sim);
        let want = sim.gravity_vector() * (m_total * 0.5);
        let err = (p - want).length();
        println!("tilt {tilt}: momentum after 0.5 s free fall ({:.6}, {:.6}, {:.3e}), want ({:.6}, {:.6}, 0), error {err:.3e}", p.x, p.y, p.z, want.x, want.y);
        assert!(
            err < 1e-6 * want.length(),
            "tilt {tilt}: momentum {p:?} vs {want:?}"
        );
        let car = sim.car.as_ref().unwrap();
        // the axis the constraint used: the frame of the positions after the solver's step
        let up = car.fit_frame(&sim.pos).unwrap().up;
        for w in &car.wheels {
            assert!(w.total_load() == 0.0);
            let rel = w.pos - sim.pos[w.p.mount[0]];
            let off = rel - up * rel.dot(up);
            assert!(
                off.length() < 1e-9,
                "tilt {tilt}: wheel {} off its axis by {off:?}",
                w.p.name
            );
        }
    }
}

const SETTLE_ARGS: [&str; 11] = [
    "--settle-report",
    "--settle-by",
    "8",
    "--settle-hold",
    "4",
    "--settle-speed",
    "0.001",
    "--energy-rise",
    "0.01",
    "--energy-gain",
    "0.01",
];

fn car_path() -> String {
    format!("{}/scenarios/car_basic.bsn", env!("CARGO_MANIFEST_DIR"))
}

/// A one-line input file in the build's own temp folder (the test binary's, not the repo).
fn input_file(name: &str, text: &str) -> String {
    let dir = std::path::PathBuf::from(env!("CARGO_TARGET_TMPDIR"));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join(name);
    std::fs::write(&path, text).unwrap();
    path.to_string_lossy().into_owned()
}

/// Run the command line; (exit code, stdout, stderr).
fn cli(args: &[&str]) -> (Option<i32>, String, String) {
    let o = Command::new(crash_sim_bin()).args(args).output().unwrap();
    let (out, err) = (
        String::from_utf8_lossy(&o.stdout).into_owned(),
        String::from_utf8_lossy(&o.stderr).into_owned(),
    );
    println!("{args:?}: exit {:?}\n{out}{err}", o.status.code());
    (o.status.code(), out, err)
}

fn report_value(out: &str, key: &str) -> f64 {
    let line = out
        .lines()
        .find_map(|l| l.strip_prefix(&format!("{key}=")))
        .unwrap_or_else(|| panic!("no `{key}=` line in {out:?}"));
    line.parse()
        .unwrap_or_else(|_| panic!("`{key}={line}` is not a number"))
}

/// docs/STAGE6.md check 3.2 from the command line: the car's settle report simulates the wheels.
#[test]
fn cli_settle_report_simulates_the_car() {
    let path = car_path();
    let mut args = vec!["--body", path.as_str(), "--duration", "12"];
    args.extend(&SETTLE_ARGS);
    let (code, out, _) = cli(&args);
    assert_eq!(code, Some(0));
    assert_eq!(out.lines().last(), Some("settle: ok"));
    // the same report from the library entry point
    let params = settle::SettleParams {
        by: 8.0,
        hold: 4.0,
        speed: 0.001,
        energy_rise: 0.01,
        energy_gain: 0.01,
    };
    let setup = settle::CarSetup::default();
    let lib = settle::measure_car(&substep_cfg(12.0), &car(), &setup, &params);
    assert!(lib.is_ok(), "{:?}", lib.verdict);
    let y = report_value(&out, "final_center_y");
    assert!(
        (y - lib.final_center_y).abs() < 1e-8,
        "{y} vs {}",
        lib.final_center_y
    );
    // the plain settle entry still refuses the wheels instead of dropping them
    let plain = settle::measure_body(&substep_cfg(12.0), Some(&car()), &params);
    assert!(plain.aborted && !plain.is_ok(), "{:?}", plain.verdict);
}

/// Check 3.4 (a) from the command line: braked on 0.8 of the grip slope, the car holds.
#[test]
fn cli_braked_car_holds_below_grip_slope() {
    let path = car_path();
    let input = input_file("brake_hold.input", "0 0 1 0\n");
    let mut args = vec![
        "--body",
        path.as_str(),
        "--input",
        input.as_str(),
        "--duration",
        "12",
        "--slope-grip-frac",
        "0.8",
    ];
    args.extend(&SETTLE_ARGS);
    let (code, out, _) = cli(&args);
    assert_eq!(code, Some(0));
    assert!(out.lines().any(|l| l == "settle: ok"), "{out}");
    // the same slope without the brake does not hold: the input is what holds the car
    let mut free = vec![
        "--body",
        path.as_str(),
        "--duration",
        "12",
        "--slope-grip-frac",
        "0.8",
    ];
    free.extend(&SETTLE_ARGS);
    let (code, out, _) = cli(&free);
    assert_eq!(
        code,
        Some(1),
        "an unbraked car must not read as settled on the slope"
    );
    assert!(out.contains("settle: FAIL"), "{out}");
}

/// Check 3.4 (b) from the command line: above the grip slope the braked car slides with the
/// friction acceleration, and the report's four lines are the library's numbers.
#[test]
fn cli_slope_report_matches_friction() {
    let path = car_path();
    let input = input_file("brake_slide.input", "0 0 1 0\n");
    let (code, out, _) = cli(&[
        "--body",
        &path,
        "--input",
        &input,
        "--duration",
        "4",
        "--slope-grip-frac",
        "1.2",
        "--slope-report",
        "--slope-from",
        "2",
        "--slope-to",
        "4",
    ]);
    assert_eq!(code, Some(0));
    let (accel, pred) = (
        report_value(&out, "slope_accel_mps2"),
        report_value(&out, "slope_predicted_mps2"),
    );
    assert!(pred > 0.0);
    assert!(
        (accel - pred).abs() <= 0.15 * pred,
        "accel {accel} vs predicted {pred}"
    );
    let body = car();
    let mu = body.tyres[0].mu;
    assert!((report_value(&out, "slope_mu") - mu).abs() < 1e-4);
    let theta = vehicle::slope_angle(&body, 1.2).unwrap();
    assert!((report_value(&out, "slope_tan") - theta.tan()).abs() < 1e-6);
    // the same numbers from the library entry point
    let setup = settle::CarSetup {
        brake: true,
        grip_frac: Some(1.2),
    };
    let r = settle::measure_slope(&substep_cfg(4.0), &body, &setup, 2.0, 4.0).unwrap();
    assert!((r.accel - accel).abs() < 1e-3, "{} vs {accel}", r.accel);
    assert!((r.predicted - pred).abs() < 1e-3);
    assert_eq!(
        out.lines().count(),
        4,
        "exactly the four slope lines: {out}"
    );
}

#[test]
fn slope_and_input_flags_are_refused_without_wheels_or_a_report() {
    let path = car_path();
    let input = input_file("brake_refuse.input", "0 0 1 0\n");
    let cube = format!("{}/scenarios/box.bsn", env!("CARGO_MANIFEST_DIR"));
    let slope = [
        "--slope-grip-frac",
        "0.8",
        "--slope-report",
        "--slope-from",
        "1",
        "--slope-to",
        "2",
    ];
    let mut cases: Vec<Vec<&str>> = Vec::new();
    // a body without wheels
    let mut c = vec!["--body", cube.as_str()];
    c.extend(&slope);
    cases.push(c);
    cases.push(vec!["--body", &cube, "--input", &input, "--settle-report"]);
    // no body at all
    cases.push(slope.to_vec());
    // a car with no report to print
    cases.push(vec!["--body", &path, "--input", &input]);
    cases.push(vec!["--body", &path, "--slope-grip-frac", "0.8"]);
    // the report's own flags
    cases.push(vec![
        "--body",
        &path,
        "--slope-grip-frac",
        "0.8",
        "--slope-report",
    ]);
    cases.push(vec![
        "--body",
        &path,
        "--slope-grip-frac",
        "0.8",
        "--slope-from",
        "1",
        "--slope-to",
        "2",
    ]);
    cases.push(vec![
        "--body",
        &path,
        "--slope-report",
        "--slope-from",
        "1",
        "--slope-to",
        "2",
    ]);
    cases.push(vec![
        "--body",
        &path,
        "--slope-grip-frac",
        "0.8",
        "--slope-report",
        "--slope-from",
        "2",
        "--slope-to",
        "1",
    ]);
    cases.push(vec![
        "--body",
        &path,
        "--slope-grip-frac",
        "-1",
        "--slope-report",
        "--slope-from",
        "1",
        "--slope-to",
        "2",
    ]);
    cases.push(vec![
        "--body",
        &path,
        "--slope-grip-frac",
        "NaN",
        "--slope-report",
        "--slope-from",
        "1",
        "--slope-to",
        "2",
    ]);
    for args in cases {
        let (code, out, _) = cli(&args);
        assert!(matches!(code, Some(1) | Some(2)), "{args:?}: exit {code:?}");
        assert!(
            !out.contains("slope_accel_mps2") && !out.contains("settle: ok"),
            "{args:?} printed a result"
        );
    }
    // below the command line: a body without wheels is refused by the car entry points
    let bare = parse(&scenario("box.bsn")).unwrap();
    let setup = settle::CarSetup {
        brake: true,
        grip_frac: None,
    };
    assert!(settle::car_sim(substep_cfg(1.0), &bare, &setup).is_err());
    let sloped = settle::CarSetup {
        brake: true,
        grip_frac: Some(1.0),
    };
    assert!(settle::measure_slope(&substep_cfg(1.0), &bare, &sloped, 0.0, 1.0).is_err());
    // the settle report of a car with a bad slope aborts, it does not pass
    let params = settle::SettleParams {
        by: 8.0,
        hold: 4.0,
        speed: 0.001,
        energy_rise: 0.01,
        energy_gain: 0.01,
    };
    let nan = settle::CarSetup {
        brake: true,
        grip_frac: Some(f64::NAN),
    };
    let r = settle::measure_car(&substep_cfg(12.0), &car(), &nan, &params);
    assert!(r.aborted && !r.is_ok());
}

#[test]
fn input_file_is_read_strictly() {
    assert_eq!(settle::parse_brake_input("0 0 1 0\n"), Ok(true));
    assert_eq!(settle::parse_brake_input("0 0 0 0"), Ok(false));
    for bad in [
        "",
        "\n",
        "0 0 1\n",
        "0 0 1 0 0\n",
        "0 0 2 0\n",
        "0 0 0.5 0\n",
        "0 1 1 0\n",
        "0 0 1 0.3\n",
        "1 0 1 0\n",
        "0 0 NaN 0\n",
        "0 0 inf 0\n",
        "0 0 x 0\n",
        "0 0 1 0\r\n",
        "0 0 0 0\n10 0 1 0\n",
    ] {
        assert!(
            settle::parse_brake_input(bad).is_err(),
            "{bad:?} must be refused"
        );
    }
}

/// The settle report of a car uses the car's energy (wheels included), not the nodes-only sum.
#[test]
fn car_settle_energy_includes_wheels() {
    let body = car();
    let sim = Sim::from_car(substep_cfg(1.0), &body).unwrap();
    let nodes_only = settle::total_energy(&sim.pos, &sim.vel, &sim.live_beams(), &sim.mass, 9.81);
    assert!(
        sim.total_energy() > nodes_only,
        "wheel masses and springs must add to the energy"
    );
    let params = settle::SettleParams {
        by: 8.0,
        hold: 4.0,
        speed: 0.001,
        energy_rise: 0.01,
        energy_gain: 0.01,
    };
    let r = settle::measure_car(
        &substep_cfg(12.0),
        &body,
        &settle::CarSetup::default(),
        &params,
    );
    assert!(r.is_ok(), "{:?}", r.verdict);
    assert_eq!(r.render().lines().last(), Some("settle: ok"));
}

fn crash_sim_bin() -> std::path::PathBuf {
    if let Some(p) = std::env::var_os("CARGO_BIN_EXE_crash-sim") {
        return std::path::PathBuf::from(p);
    }
    let exe = std::env::current_exe().expect("current_exe of the test executable");
    let name = format!("crash-sim{}", std::env::consts::EXE_SUFFIX);
    let dir = exe
        .parent()
        .and_then(|p| p.parent())
        .unwrap_or_else(|| panic!("{} has no grandparent folder", exe.display()));
    let bin = dir.join(name);
    assert!(
        bin.is_file(),
        "CARGO_BIN_EXE_crash-sim is not set and {} does not exist",
        bin.display()
    );
    bin
}
