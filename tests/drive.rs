//! Driving a car from an input script (nb-40, docs/STAGE6.md milestone 3: "The drive report",
//! checks 3.1, 3.4, 3.5 and 3.8). The three named tests of check 3.5 assert their rule against
//! an independent formula (statics, kinematic steering), never against a recorded position.

use crash_sim::format::{parse, Body};
use crash_sim::input::{DriveMeasure, DriveParams, FinalState, Script};
use crash_sim::settle::{car_sim, CarSetup};
use crash_sim::vec3::Vec3;
use crash_sim::{Config, Sim};
use std::process::Command;

fn scenario(name: &str) -> String {
    let path = format!("{}/scenarios/{name}", env!("CARGO_MANIFEST_DIR"));
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"))
}

fn car_text() -> String {
    scenario("car_basic.bsn")
}

fn car() -> Body {
    parse(&car_text()).unwrap_or_else(|e| panic!("car_basic.bsn: {e}"))
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

const H: f64 = 0.001;

fn script(text: &str) -> Script {
    Script::parse(text).unwrap_or_else(|e| panic!("{text:?}: {e}"))
}

/// A car sim driven by `text` for `seconds` of 1 ms substeps, calling `each(sim, t)` after every
/// substep (t = time at its end). The script is applied at the start of every substep, as the
/// command line does.
fn drive(body: &Body, text: &str, seconds: f64, mut each: impl FnMut(&mut Sim, f64)) -> Sim {
    let setup = CarSetup {
        script: Some(script(text)),
        until_script_end: false,
        grip_frac: None,
    };
    let mut sim = car_sim(substep_cfg(seconds), body, &setup).unwrap();
    let s = setup.script.as_ref().unwrap();
    let total = (seconds / H).round() as u64;
    for i in 1..=total {
        s.apply(&mut sim, (i - 1) as f64 * H, H * 1e-6).unwrap();
        sim.advance();
        sim.check()
            .unwrap_or_else(|e| panic!("unstable at {}: {e}", i as f64 * H));
        each(&mut sim, i as f64 * H);
    }
    sim
}

/// Path of the crash-sim binary, read at run time: the copy a broadsword test step points
/// `CARGO_BIN_EXE_crash-sim` at, else the cargo layout next to the test executable (as tests/csv.rs).
/// Never `env!` at compile time: that path is in the shared build folder, which another checkout's
/// stale-package clean can empty while these tests run.
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

/// A file in the build's own temp folder (the test binary's, not the repo).
fn tmp_file(name: &str, text: &str) -> String {
    let dir = std::path::PathBuf::from(env!("CARGO_TARGET_TMPDIR"));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join(name);
    std::fs::write(&path, text).unwrap();
    path.to_string_lossy().into_owned()
}

fn car_path() -> String {
    format!("{}/scenarios/car_basic.bsn", env!("CARGO_MANIFEST_DIR"))
}

/// Run the command line; (exit code, stdout bytes as text, stderr).
fn cli(args: &[&str]) -> (Option<i32>, String, String) {
    let o = Command::new(crash_sim_bin()).args(args).output().unwrap();
    let (out, err) = (
        String::from_utf8_lossy(&o.stdout).into_owned(),
        String::from_utf8_lossy(&o.stderr).into_owned(),
    );
    println!("{args:?}: exit {:?}\n{out}{err}", o.status.code());
    (o.status.code(), out, err)
}

fn value(out: &str, key: &str) -> f64 {
    let line = out
        .lines()
        .find_map(|l| l.strip_prefix(&format!("{key}=")))
        .unwrap_or_else(|| panic!("no `{key}=` line in {out:?}"));
    line.parse()
        .unwrap_or_else(|_| panic!("`{key}={line}` is not a number"))
}

// ---------------------------------------------------------------------------------------------
// check 3.5
// ---------------------------------------------------------------------------------------------

/// 3.5: the same input file run twice gives byte-identical stdout. The input uses all three
/// controls and changes them, so a run that ignored the file could not match it by luck: a
/// different file must give different bytes.
#[test]
fn same_input_same_bytes() {
    let path = car_path();
    let text = "0 0 0 0\n1 0.6 0 0\n2.5 0.3 0 -0.7\n4 0 0.5 0.2\n";
    let a = tmp_file("same_a.input", text);
    let other = tmp_file(
        "same_b.input",
        "0 0 0 0\n1 0.6 0 0\n2.5 0.3 0 0.7\n4 0 0.5 0.2\n",
    );
    let args = |input: &str| -> Vec<String> {
        ["--body", &path, "--input", input, "--duration", "5"]
            .iter()
            .map(|s| s.to_string())
            .collect()
    };
    let run = |input: &str| {
        let owned = args(input);
        let refs: Vec<&str> = owned.iter().map(|s| s.as_str()).collect();
        let (code, out, _) = cli(&refs);
        assert_eq!(code, Some(0));
        out
    };
    let (first, second) = (run(&a), run(&a));
    assert!(
        first.lines().count() > 100,
        "a drive prints its rows: {first}"
    );
    assert_eq!(
        first.as_bytes(),
        second.as_bytes(),
        "the same input must give the same bytes"
    );
    let different = run(&other);
    assert_ne!(
        first, different,
        "a different input (steer sign) must change the output"
    );
    // the same run from the library entry point gives the same final state twice, bit for bit
    let body = car();
    let pos = |s: &Sim| {
        s.pos
            .iter()
            .map(|p| (p.x.to_bits(), p.y.to_bits(), p.z.to_bits()))
            .collect::<Vec<_>>()
    };
    let (s1, s2) = (
        drive(&body, text, 5.0, |_, _| {}),
        drive(&body, text, 5.0, |_, _| {}),
    );
    assert_eq!(pos(&s1), pos(&s2));
}

/// The total (chassis and wheels) centre of mass velocity along x.
fn total_com_vx(sim: &Sim, total_mass: f64) -> f64 {
    let car = sim.car.as_ref().unwrap();
    let nodes: f64 = sim.vel.iter().zip(&sim.mass).map(|(v, m)| v.x * m).sum();
    let wheels: f64 = car.wheels.iter().map(|w| w.vel.x * w.p.mass).sum();
    (nodes + wheels) / total_mass
}

/// The mean x of a wheel's mount nodes in the file: where its load acts, for statics.
fn wheel_x(body: &Body, w: &crash_sim::format::Wheel) -> f64 {
    w.mount
        .iter()
        .map(|id| body.nodes[body.node_index(*id).unwrap()].pos.x)
        .sum::<f64>()
        / w.mount.len() as f64
}

/// The expected launch acceleration (m/s^2) from the car file and statics alone (docs/STAGE6.md
/// 3.5): a = F / M_eff, M_eff = M + (sum of wheel inertias) / radius^2, F = min(total drive
/// torque / tyre radius, F_grip). F_grip is mu x the load on the driven wheels while accelerating
/// at a = F_grip / M_eff, which is the static load on them (moment balance about the centre of
/// mass, from the file) plus the load transfer M a h / L from the other axle (h the centre of mass
/// height above the ground at the suspension's rest length, L the wheelbase):
/// F_grip = mu N_static / (1 - mu M h / (L M_eff)). Returns (a, torque limit, F_grip).
fn expected_launch(body: &Body) -> (f64, f64, f64) {
    let controls = body.controls.as_ref().unwrap();
    let g = Config::default().gravity;
    let m = body.nodes.iter().map(|n| n.mass).sum::<f64>()
        + body.wheels.iter().map(|w| w.mass).sum::<f64>();
    let x_cm = (body.nodes.iter().map(|n| n.mass * n.pos.x).sum::<f64>()
        + body
            .wheels
            .iter()
            .map(|w| w.mass * wheel_x(body, w))
            .sum::<f64>())
        / m;
    // the wheel centres are at their rest length below their mounts, so the ground is a tyre
    // radius lower; the car is symmetric, so every wheel gives the same ground height
    let ground_y = |w: &crash_sim::format::Wheel| {
        let mount_y = w
            .mount
            .iter()
            .map(|id| body.nodes[body.node_index(*id).unwrap()].pos.y)
            .sum::<f64>()
            / w.mount.len() as f64;
        mount_y - body.suspension(&w.name).unwrap().rest - w.radius
    };
    let y0 = ground_y(&body.wheels[0]);
    assert!(body.wheels.iter().all(|w| (ground_y(w) - y0).abs() < 1e-9));
    let y_cm = (body.nodes.iter().map(|n| n.mass * n.pos.y).sum::<f64>()
        + body
            .wheels
            .iter()
            .map(|w| w.mass * (ground_y(w) + w.radius))
            .sum::<f64>())
        / m;
    let h = y_cm - y0;
    // two axles: the load on the axle at x_a is M g (x_other - x_cm) / (x_other - x_a)
    let mut xs: Vec<f64> = body.wheels.iter().map(|w| wheel_x(body, w)).collect();
    xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    xs.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    assert_eq!(xs.len(), 2, "the statics below are for a two-axle car");
    let wheelbase = xs[1] - xs[0];
    let driven: Vec<_> = body.wheels.iter().filter(|w| w.driven).collect();
    assert!(!driven.is_empty());
    let r = driven[0].radius;
    assert!(driven.iter().all(|w| w.radius == r));
    let axle_load = |x_a: f64| {
        let other = if (x_a - xs[0]).abs() < 1e-9 {
            xs[1]
        } else {
            xs[0]
        };
        m * g * (other - x_cm) / (other - x_a)
    };
    // each axle's load is shared by its wheels (the car is symmetric left to right)
    let per_wheel = |w: &crash_sim::format::Wheel| {
        let n = body
            .wheels
            .iter()
            .filter(|o| (wheel_x(body, o) - wheel_x(body, w)).abs() < 1e-9)
            .count() as f64;
        axle_load(wheel_x(body, w)) / n
    };
    let mu_n: f64 = driven
        .iter()
        .map(|w| body.tyre(&w.name).unwrap().mu * per_wheel(w))
        .sum();
    let m_eff = m + body
        .wheels
        .iter()
        .map(|w| w.inertia / (w.radius * w.radius))
        .sum::<f64>();
    // the driven axle is behind the centre of mass (smaller x) when the car faces +x: load moves onto it
    let behind = driven.iter().all(|w| wheel_x(body, w) < x_cm);
    let mu: f64 = driven
        .iter()
        .map(|w| body.tyre(&w.name).unwrap().mu)
        .sum::<f64>()
        / driven.len() as f64;
    let sign = if behind { 1.0 } else { -1.0 };
    let f_grip = mu_n / (1.0 - sign * mu * m * h / (wheelbase * m_eff));
    let torque_limit = controls.max_drive_torque / r;
    println!("M {m:.1} kg, M_eff {m_eff:.2} kg, h {h:.3} m, L {wheelbase} m, mu x static load {mu_n:.1} N, F_grip with load transfer {f_grip:.1} N, torque limit {torque_limit:.1} N");
    (torque_limit.min(f_grip) / m_eff, torque_limit, f_grip)
}

/// Settle for 10 s with no input, then full throttle: the mean acceleration of the whole car
/// (chassis and wheels) over the first 0.5 s.
fn measured_launch(body: &Body) -> f64 {
    let m = body.nodes.iter().map(|n| n.mass).sum::<f64>()
        + body.wheels.iter().map(|w| w.mass).sum::<f64>();
    let (mut u0, mut u1) = (None, None);
    drive(
        body,
        "0 0 0 0
10 1 0 0
",
        10.5,
        |sim, t| {
            if (t - 10.0).abs() < H * 0.5 {
                u0 = Some(total_com_vx(sim, m));
            }
            if (t - 10.5).abs() < H * 0.5 {
                u1 = Some(total_com_vx(sim, m));
            }
        },
    );
    let (u0, u1) = (u0.unwrap(), u1.unwrap());
    assert!(
        u0.abs() < 0.01,
        "the car is at rest when the throttle goes on: {u0} m/s"
    );
    (u1 - u0) / 0.5
}

/// 3.5: from rest on flat ground at full throttle, the mean acceleration over the first 0.5 s is
/// within 10% of F / M_eff (see `expected_launch`). The expected value comes from the car file and
/// statics only, never from the sim. It is checked twice: for the car as it is (the torque limit
/// binds) and with grip cut to mu 0.5 (the grip limit binds), so both branches of the min are
/// exercised and a sim that ignored either limit fails one of them.
#[test]
fn launch_matches_torque_or_grip_limit() {
    let body = car();
    let (expected, torque_limit, f_grip) = expected_launch(&body);
    assert!(
        torque_limit < f_grip,
        "car_basic is torque-limited at launch"
    );
    let accel = measured_launch(&body);
    println!("torque-limited car: measured {accel:.4} m/s^2, expected {expected:.4}");
    assert!(
        (accel - expected).abs() <= 0.10 * expected,
        "mean acceleration {accel} m/s^2 over the first 0.5 s is not within 10% of F/M_eff = {expected}"
    );

    let slippery = parse(&car_text().replace("mu 0.88", "mu 0.5")).unwrap();
    assert_eq!(slippery.tyres.iter().filter(|t| t.mu == 0.5).count(), 4);
    let (expected, torque_limit, f_grip) = expected_launch(&slippery);
    assert!(f_grip < torque_limit, "with mu 0.5 the grip limit binds");
    let accel = measured_launch(&slippery);
    println!("grip-limited car: measured {accel:.4} m/s^2, expected {expected:.4}");
    assert!(
        (accel - expected).abs() <= 0.10 * expected,
        "grip-limited: mean acceleration {accel} m/s^2 is not within 10% of F/M_eff = {expected}"
    );
}

/// Unwrapped yaw of the rear-to-front vector.
struct Yaw {
    last: f64,
    total: f64,
}

impl Yaw {
    fn new(angle: f64) -> Yaw {
        Yaw {
            last: angle,
            total: 0.0,
        }
    }
    fn update(&mut self, angle: f64) {
        let mut d = angle - self.last;
        while d > std::f64::consts::PI {
            d -= std::f64::consts::TAU;
        }
        while d <= -std::f64::consts::PI {
            d += std::f64::consts::TAU;
        }
        self.total += d;
        self.last = angle;
    }
}

fn group_mean(sim: &Sim, body: &Body, name: &str) -> Vec3 {
    let g = body.groups.iter().find(|g| g.name == name).unwrap();
    let mut c = Vec3::ZERO;
    for id in &g.nodes {
        c += sim.pos[body.node_index(*id).unwrap()];
    }
    c * (1.0 / g.nodes.len() as f64)
}

fn yaw_angle(sim: &Sim, body: &Body) -> f64 {
    let v = group_mean(sim, body, "front") - group_mean(sim, body, "rear");
    v.z.atan2(v.x)
}

/// The mean of the non-steerable wheels' centres (the rear axle midpoint), in the ground plane.
fn rear_axle_midpoint(sim: &Sim) -> (f64, f64) {
    let car = sim.car.as_ref().unwrap();
    let rear: Vec<_> = car.wheels.iter().filter(|w| !w.p.steerable).collect();
    let n = rear.len() as f64;
    (
        rear.iter().map(|w| w.pos.x).sum::<f64>() / n,
        rear.iter().map(|w| w.pos.z).sum::<f64>() / n,
    )
}

/// 3.5: at a steady 2 m/s with full steer, the path radius is within 10% of wheelbase /
/// tan(max steer angle) (low-speed kinematic steering; lateral acceleration is under 0.1 g, so
/// tyre slip is small), and over two laps it varies by at most 5%. The path radius of a lap is
/// the path length of the rear axle midpoint over one full turn of the car's yaw, divided by 2
/// pi. The speed is held at 2 m/s by a test driver that sets the throttle every substep.
#[test]
fn full_lock_circle_matches_kinematic_radius() {
    let body = car();
    let controls = body.controls.as_ref().unwrap();
    let steerable: Vec<_> = body.wheels.iter().filter(|w| w.steerable).collect();
    let fixed: Vec<_> = body.wheels.iter().filter(|w| !w.steerable).collect();
    let wheelbase = wheel_x(&body, steerable[0]) - wheel_x(&body, fixed[0]);
    assert!(wheelbase > 0.0);
    let expected = wheelbase / controls.max_steer_deg.to_radians().tan();
    println!("wheelbase {wheelbase} m, kinematic radius {expected:.4} m");

    let m: f64 = body.nodes.iter().map(|n| n.mass).sum::<f64>()
        + body.wheels.iter().map(|w| w.mass).sum::<f64>();
    // The car is built by the same entry point as the command line; the script holds full lock
    // from t = 14 s, after 4 s of straight running to 2 m/s. The throttle is the driver's.
    let setup = CarSetup {
        script: Some(script("0 0 0 0\n14 0 0 1\n")),
        until_script_end: false,
        grip_frac: None,
    };
    let mut sim = car_sim(substep_cfg(60.0), &body, &setup).unwrap();
    let s = setup.script.as_ref().unwrap();
    let mut yaw: Option<Yaw> = None;
    let mut lap_path = 0.0;
    let mut lap_start_total = 0.0;
    let mut last = rear_axle_midpoint(&sim);
    let mut laps: Vec<f64> = Vec::new();
    let mut speed_in_laps: Vec<f64> = Vec::new();
    let t_measure = 18.0; // 4 s of full lock before the laps are measured
    for i in 1..=60_000u64 {
        let t = (i - 1) as f64 * H;
        s.apply(&mut sim, t, H * 1e-6).unwrap();
        // the test driver: hold 2 m/s from t = 8 s (the car is settled by then)
        let v = (sim
            .vel
            .iter()
            .zip(&sim.mass)
            .map(|(v, w)| *v * *w)
            .fold(Vec3::ZERO, |a, b| a + b)
            * (1.0 / sim.mass.iter().sum::<f64>()))
        .length();
        let throttle = if t >= 8.0 {
            (3.0 * (2.0 - v)).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let (steer, brake) = (sim.car.as_ref().unwrap().steer, 0.0);
        sim.car
            .as_mut()
            .unwrap()
            .set_controls(throttle, brake, steer)
            .unwrap();
        sim.advance();
        sim.check().unwrap();
        let now = rear_axle_midpoint(&sim);
        let a = yaw_angle(&sim, &body);
        if t + H >= t_measure {
            if yaw.is_none() {
                yaw = Some(Yaw::new(a));
                last = now;
                lap_path = 0.0;
                lap_start_total = 0.0;
            }
            let y = yaw.as_mut().unwrap();
            y.update(a);
            lap_path += ((now.0 - last.0).powi(2) + (now.1 - last.1).powi(2)).sqrt();
            speed_in_laps.push(v);
            last = now;
            if y.total.abs() - lap_start_total >= std::f64::consts::TAU {
                laps.push(lap_path / std::f64::consts::TAU);
                lap_path = 0.0;
                lap_start_total += std::f64::consts::TAU;
                if laps.len() == 2 {
                    break;
                }
            }
        }
    }
    assert_eq!(laps.len(), 2, "two laps within 60 s");
    println!(
        "lap radii {laps:?}; speed in laps {:.3}..{:.3} m/s",
        speed_in_laps.iter().cloned().fold(f64::MAX, f64::min),
        speed_in_laps.iter().cloned().fold(f64::MIN, f64::max)
    );
    let _ = m;
    let (min_v, max_v) = (
        speed_in_laps.iter().cloned().fold(f64::MAX, f64::min),
        speed_in_laps.iter().cloned().fold(f64::MIN, f64::max),
    );
    assert!(
        min_v > 1.9 && max_v < 2.1,
        "the speed was held at 2 m/s: {min_v}..{max_v}"
    );
    for r in &laps {
        assert!(
            (r - expected).abs() <= 0.10 * expected,
            "path radius {r} m is not within 10% of wheelbase / tan(max steer) = {expected} m"
        );
    }
    assert!(
        (laps[0] - laps[1]).abs() <= 0.05 * laps[0].min(laps[1]),
        "the radius varies by more than 5% between laps: {laps:?}"
    );
}

// ---------------------------------------------------------------------------------------------
// the input file
// ---------------------------------------------------------------------------------------------

#[test]
fn input_grammar_accepts_what_it_describes() {
    let s = script("0 0 0 0\n\n  1.5   1  0.25 -1  \n2 0 1 1");
    assert_eq!(s.lines().len(), 2 + 1);
    let l = s.lines();
    assert_eq!(
        (l[1].number, l[1].t, l[1].throttle, l[1].brake, l[1].steer),
        (3, 1.5, 1.0, 0.25, -1.0),
        "line numbers count blank lines"
    );
    assert_eq!(s.t_last(), 2.0);
    // the line in force: held until the next line
    assert_eq!(s.line_at(0.0, 0.0).t, 0.0);
    assert_eq!(s.line_at(1.4999, 0.0).t, 0.0);
    assert_eq!(s.line_at(1.5, 0.0).t, 1.5);
    assert_eq!(s.line_at(99.0, 0.0).t, 2.0);
    // the real scenario file
    let fb = script(&scenario("full_brake.input"));
    assert_eq!((fb.lines().len(), fb.lines()[0].brake), (1, 1.0));
}

#[test]
fn input_errors_name_the_line() {
    let cases: &[(&str, &str)] = &[
        ("", "no line"),
        ("\n\n  \n", "no line"),
        ("0 0 0 0\r\n", "input line 1"),
        ("0 0 0 0\n1 0 0 0\r\n", "input line 2"),
        ("0 0 0\n", "input line 1"),
        ("0 0 0 0 0\n", "input line 1"),
        ("0 0 0 0\n1 0 x 0\n", "input line 2"),
        ("0 0 0 0\n\n1 NaN 0 0\n", "input line 3"),
        ("0 0 0 0\n1 0 inf 0\n", "input line 2"),
        ("0 0 0 0\n1 0 0 -inf\n", "input line 2"),
        ("# comment\n0 0 0 0\n", "input line 1"),
        ("0 -1.0001 0 0\n", "input line 1"), // below -1: reverse is -1..=0 (task t-20261120-220016-2)
        ("0 1.0001 0 0\n", "input line 1"),
        ("0 0 -0.5 0\n", "input line 1"),
        ("0 0 1.5 0\n", "input line 1"),
        ("0 0 0 1.01\n", "input line 1"),
        ("0 0 0 -1.01\n", "input line 1"),
        ("0.5 0 0 0\n", "input line 1"),
        ("-1 0 0 0\n", "input line 1"),
        ("0 0 0 0\n0 0 0 0\n", "input line 2"),
        ("0 0 0 0\n2 0 0 0\n1 0 0 0\n", "input line 3"),
        ("0 0 0 0\n1 0 0 0\n1 1 0 0\n", "input line 3"),
    ];
    for (text, want) in cases {
        let e = Script::parse(text).expect_err(&format!("{text:?} must be refused"));
        assert!(
            e.contains(want),
            "{text:?}: the error `{e}` must contain `{want}`"
        );
    }
}

#[test]
fn input_cli_errors_exit_2_and_name_the_line() {
    let path = car_path();
    let bad = tmp_file("bad_steer.input", "0 0 0 0\n1 0 0 0\n2 0 0 1.5\n");
    let (code, out, err) = cli(&["--body", &path, "--input", &bad, "--duration", "1"]);
    assert_eq!(code, Some(2));
    assert!(
        out.is_empty(),
        "nothing on stdout for a refused input: {out}"
    );
    assert!(
        err.contains("input line 3") && err.contains("steer"),
        "{err}"
    );
    let crlf = tmp_file("crlf.input", "0 0 0 0\r\n");
    let (code, _, err) = cli(&["--body", &path, "--input", &crlf, "--duration", "1"]);
    assert_eq!(
        (code, err.contains("input line 1")),
        (Some(2), true),
        "{err}"
    );
    let empty = tmp_file("empty.input", "");
    let (code, _, err) = cli(&["--body", &path, "--input", &empty, "--duration", "1"]);
    assert_eq!((code, err.contains("no line")), (Some(2), true), "{err}");
    let (code, _, err) = cli(&["--body", &path, "--input", "no/such/file.input"]);
    assert_eq!(
        (code, err.contains("no/such/file.input")),
        (Some(2), true),
        "{err}"
    );
}

/// The engine's work enters the energy rule (docs/STAGE6.md, settle rule 4): at full throttle the
/// car gains far more than G x E(0) of energy, so the rule fails without W(t) and holds with it;
/// and W(t) is a real account of the work, not a slack number: the energy gained never exceeds W
/// (the drive is the only source; tyres, dampers and beams only lose energy).
#[test]
fn drive_work_enters_the_energy_rule() {
    let body = car();
    let setup = CarSetup {
        script: Some(script(
            "0 0 0 0
8 1 0 0
",
        )),
        until_script_end: false,
        grip_frac: None,
    };
    let mut sim = car_sim(substep_cfg(14.0), &body, &setup).unwrap();
    let s = setup.script.as_ref().unwrap();
    let e0 = sim.total_energy();
    let (mut raw_gain, mut gain, mut worst_excess) = (0.0_f64, 0.0_f64, f64::MIN);
    for i in 1..=14_000u64 {
        s.apply(&mut sim, (i - 1) as f64 * H, H * 1e-6).unwrap();
        sim.advance();
        let (e, w) = (sim.total_energy(), sim.car.as_ref().unwrap().drive_work);
        raw_gain = raw_gain.max((e - e0) / e0);
        gain = gain.max((e - e0 - w) / (e0 + w));
        worst_excess = worst_excess.max(e - e0 - w);
    }
    let w = sim.car.as_ref().unwrap().drive_work;
    println!("E(0) {e0:.1} J, W(14 s) {w:.1} J, raw gain {raw_gain:.3}, corrected gain {gain:.2e}, worst E - E0 - W {worst_excess:.2} J");
    assert!(
        w > 1e4,
        "6 s of full throttle does tens of kilojoules of work: {w} J"
    );
    assert!(
        raw_gain > 0.01,
        "without W the rule would fail (gain {raw_gain})"
    );
    assert!(gain <= 0.01, "with W it holds: {gain}");
    assert!(
        worst_excess <= 0.01 * (e0 + w),
        "energy gained exceeds the engine's work: {worst_excess} J"
    );
    // and the settle report of the library applies the same rule
    let params = crash_sim::settle::SettleParams {
        by: 8.0,
        hold: 4.0,
        speed: 0.001,
        energy_rise: 0.01,
        energy_gain: 0.01,
    };
    let r = crash_sim::settle::measure_car(&substep_cfg(14.0), &body, &setup, &params);
    assert!(!r.aborted && r.max_energy_gain_frac <= 0.01, "{r:?}");
    assert!(
        (r.max_energy_gain_frac - gain).abs() < 1e-9,
        "{} vs {gain}",
        r.max_energy_gain_frac
    );
}

/// A line the car cannot obey is refused by name before the run: no driven wheel, no throttle.
#[test]
fn a_control_the_car_cannot_obey_is_refused() {
    // The file format already refuses a car with no driven, braked or steerable wheel (so no
    // file can reach this), but a car built in code can lose a wheel's role: take it away here.
    let mut sim = Sim::from_car(substep_cfg(1.0), &car()).unwrap();
    let car = sim.car.as_mut().unwrap();
    assert!(car.set_controls(0.5, 1.0, -1.0).is_ok());
    for w in car.wheels.iter_mut() {
        w.p.driven = false;
    }
    let e = car
        .set_controls(0.5, 0.0, 0.0)
        .expect_err("throttle with no driven wheel");
    assert!(e.contains("driven"), "{e}");
    assert!(
        car.set_controls(0.0, 1.0, 1.0).is_ok(),
        "brake and steer are still allowed"
    );
    // the script is checked line by line, before the run
    let e = script(
        "0 0 0 0
1 0.5 0 0
",
    )
    .check_for(car)
    .expect_err("line 2 asks for throttle");
    assert!(e.contains("input line 2") && e.contains("driven"), "{e}");
    assert!(script(
        "0 0 1 1
"
    )
    .check_for(car)
    .is_ok());
    for w in car.wheels.iter_mut() {
        w.p.steerable = false;
    }
    let e = script(
        "0 0 0 0.1
",
    )
    .check_for(car)
    .expect_err("steer with no steerable wheel");
    assert!(e.contains("input line 1") && e.contains("steerable"), "{e}");
}

// ---------------------------------------------------------------------------------------------
// the run
// ---------------------------------------------------------------------------------------------

fn final_line(out: &str) -> &str {
    out.lines()
        .last()
        .filter(|l| l.starts_with("final t="))
        .unwrap_or_else(|| panic!("no final line at the end of {out:?}"))
}

/// "a run ends at the last input line when there is no --duration", at a substep, not an output
/// step; a longer --duration runs on with the last line's values; a shorter one ends sooner.
#[test]
fn a_run_ends_at_the_last_input_line() {
    let path = car_path();
    let input = tmp_file("ends.input", "0 0 0 0\n1.2345 1 0 0\n");
    let (code, out, _) = cli(&["--body", &path, "--input", &input]);
    assert_eq!(code, Some(0));
    let fin = final_line(&out);
    assert!(
        fin.starts_with("final t=1.2340 "),
        "1.2345 s is not a whole 1 ms substep, the run ends at the last whole one: {fin}"
    );
    let input = tmp_file("ends2.input", "0 0 0 0\n1.235 1 0 0\n");
    let (_, out, _) = cli(&["--body", &path, "--input", &input]);
    assert!(
        final_line(&out).starts_with("final t=1.2350 "),
        "{}",
        final_line(&out)
    );
    // rows at t = 0 and every output step (0.01 s), up to the last whole step
    let rows: Vec<&str> = out
        .lines()
        .skip(1)
        .filter(|l| !l.starts_with("final"))
        .collect();
    assert_eq!(rows.len(), 124, "t = 0 and 123 output steps up to 1.23 s");
    assert!(rows[0].starts_with("0.0000,") && rows[123].starts_with("1.2300,"));
    // a longer duration: the last line (full throttle) holds, so the car keeps gaining speed
    let (_, short, _) = cli(&["--body", &path, "--input", &input, "--duration", "3"]);
    let (_, long, _) = cli(&["--body", &path, "--input", &input, "--duration", "5"]);
    assert!(
        final_line(&short).starts_with("final t=3.0000 ")
            && final_line(&long).starts_with("final t=5.0000 ")
    );
    let x = |l: &str| -> f64 {
        l.split(" x=")
            .nth(1)
            .unwrap()
            .split(' ')
            .next()
            .unwrap()
            .parse()
            .unwrap()
    };
    assert!(
        x(final_line(&long)) > x(final_line(&short)) + 1.0,
        "the car keeps driving on the last line: {} vs {}",
        final_line(&short),
        final_line(&long)
    );
    // a shorter duration than the script: ends at the duration
    let (_, cut, _) = cli(&["--body", &path, "--input", &input, "--duration", "0.5"]);
    assert!(final_line(&cut).starts_with("final t=0.5000 "));
}

/// `steer` > 0 turns toward +z, and the heading and position columns say so.
#[test]
fn positive_steer_turns_toward_plus_z() {
    let body = car();
    let mut yaw: Option<Yaw> = None;
    let mut z_end = 0.0;
    drive(&body, "0 0 0 0\n8 0.15 0 0.5\n", 14.0, |sim, t| {
        if t >= 8.0 {
            let a = yaw_angle(sim, &body);
            yaw.get_or_insert_with(|| Yaw::new(a)).update(a);
        }
        z_end = group_mean(sim, &body, "front").z;
    });
    assert!(yaw.unwrap().total > 0.2, "steer +0.5 raises the heading");
    assert!(z_end > 0.3, "and moves the car toward +z: z = {z_end}");
}

#[test]
fn rows_and_final_line_from_the_command_line() {
    let path = car_path();
    let input = tmp_file("rows.input", "0 0 0 0\n1 0.5 0 0.3\n");
    let (code, out, _) = cli(&["--body", &path, "--input", &input, "--duration", "2"]);
    assert_eq!(code, Some(0));
    let mut lines = out.lines();
    assert_eq!(lines.next(), Some("t,speed_mps,x,y,z,heading_deg"));
    let rows: Vec<&str> = lines.by_ref().take(201).collect();
    assert_eq!(rows.len(), 201);
    let last_row: Vec<f64> = rows[200].split(',').map(|v| v.parse().unwrap()).collect();
    assert_eq!(last_row.len(), 6);
    let fin = lines.next().unwrap();
    assert_eq!(lines.next(), None);
    // the final line is the last row's t, position and heading, in its own format
    assert_eq!(
        fin,
        format!(
            "final t={:.4} x={:.4} z={:.4} heading_deg={:.2}",
            last_row[0], last_row[2], last_row[4], last_row[5]
        )
    );
    assert!(
        last_row[0] == 2.0 && last_row[1] > 0.1,
        "the car moves: {rows:?}"
    );
}

// ---------------------------------------------------------------------------------------------
// the drive report
// ---------------------------------------------------------------------------------------------

fn params() -> DriveParams {
    DriveParams {
        lap_min_heading: 360.0,
        lap_min_path: 50.0,
        lap_close: 5.0,
        impact_min_kmh: 45.0,
        impact_max_kmh: 55.0,
    }
}

fn good() -> DriveMeasure {
    DriveMeasure {
        lap_heading_deg: 400.0,
        lap_path_m: Some(60.0),
        lap_close_m: Some(2.0),
        chassis_ground_contacts_before_wall: 0,
        wall_first_contact_s: Some(30.0),
        wall_impact_speed_kmh: Some(50.0),
    }
}

/// `drive: ok` iff every rule holds; each rule alone breaks it, and the FAIL line names the rule.
#[test]
fn drive_verdict_follows_each_rule() {
    let p = params();
    assert_eq!(good().verdict(&p), Ok(()));
    let fin = FinalState {
        t: 31.0,
        x: 1.0,
        z: 2.0,
        heading_deg: -10.0,
    };
    let text = good().render(&p, &fin);
    assert_eq!(text.lines().last(), Some("drive: ok"));
    assert_eq!(
        text,
        "lap_heading_deg=400.0\nlap_path_m=60.00\nlap_close_m=2.000\nchassis_ground_contacts_before_wall=0\nwall_first_contact_s=30.0000\nwall_impact_speed_kmh=50.0\nfinal t=31.0000 x=1.0000 z=2.0000 heading_deg=-10.00\ndrive: ok\n"
    );
    let breaks: Vec<(DriveMeasure, &str)> = vec![
        (
            DriveMeasure {
                lap_heading_deg: 359.9,
                ..good()
            },
            "lap_heading_deg",
        ),
        (
            DriveMeasure {
                lap_path_m: Some(49.99),
                ..good()
            },
            "lap_path_m",
        ),
        (
            DriveMeasure {
                lap_path_m: None,
                ..good()
            },
            "lap_path_m",
        ),
        (
            DriveMeasure {
                lap_close_m: Some(5.001),
                ..good()
            },
            "lap_close_m",
        ),
        (
            DriveMeasure {
                lap_close_m: None,
                ..good()
            },
            "lap_close_m",
        ),
        (
            DriveMeasure {
                chassis_ground_contacts_before_wall: 1,
                ..good()
            },
            "chassis_ground_contacts_before_wall",
        ),
        (
            DriveMeasure {
                wall_impact_speed_kmh: Some(44.9),
                ..good()
            },
            "wall_impact_speed_kmh",
        ),
        (
            DriveMeasure {
                wall_impact_speed_kmh: Some(55.1),
                ..good()
            },
            "wall_impact_speed_kmh",
        ),
        (
            DriveMeasure {
                wall_impact_speed_kmh: None,
                wall_first_contact_s: None,
                ..good()
            },
            "no wall contact",
        ),
    ];
    for (m, rule) in breaks {
        let e = m.verdict(&p).expect_err(rule);
        assert!(e.contains(rule), "{e}");
        let t = m.render(&p, &fin);
        assert!(
            t.lines().last().unwrap().starts_with("drive: FAIL ") && t.contains(rule),
            "{t}"
        );
    }
    // edges are inclusive
    for (m, name) in [
        (
            DriveMeasure {
                lap_heading_deg: 360.0,
                ..good()
            },
            "heading",
        ),
        (
            DriveMeasure {
                lap_path_m: Some(50.0),
                ..good()
            },
            "path",
        ),
        (
            DriveMeasure {
                lap_close_m: Some(5.0),
                ..good()
            },
            "close",
        ),
        (
            DriveMeasure {
                wall_impact_speed_kmh: Some(45.0),
                ..good()
            },
            "impact min",
        ),
        (
            DriveMeasure {
                wall_impact_speed_kmh: Some(55.0),
                ..good()
            },
            "impact max",
        ),
    ] {
        assert_eq!(m.verdict(&p), Ok(()), "{name}");
    }
    assert!(DriveParams {
        lap_min_heading: f64::NAN,
        ..p
    }
    .validate()
    .is_err());
    assert!(DriveParams {
        impact_min_kmh: 60.0,
        ..p
    }
    .validate()
    .is_err());
    assert!(DriveParams {
        lap_close: 0.0,
        ..p
    }
    .validate()
    .is_err());
    assert!(p.validate().is_ok());
}

/// A drive with a wall: the car is `car_basic.bsn` and one wall plane across its path.
fn wall_car_text() -> String {
    format!("{}plane 25 0 0 -1 0 0\n", car_text())
}

/// The drive report from the command line: the lines in order, the `final` line before the
/// verdict, the settle report after it; a wall in the way gives a contact time and an impact
/// speed that agree with the speed in the rows (an independent reading of the same run).
#[test]
fn drive_report_measures_the_wall_hit() {
    let body = tmp_file("wall_car.bsn", &wall_car_text());
    let input = tmp_file("wall.input", "0 0 0 0\n8 1 0 0\n14 0 0 0\n");
    let rows_args = [
        "--body",
        body.as_str(),
        "--input",
        input.as_str(),
        "--duration",
        "20",
    ];
    let (code, rows, _) = cli(&rows_args);
    assert_eq!(code, Some(0));
    let report_args = [
        "--body",
        body.as_str(),
        "--input",
        input.as_str(),
        "--duration",
        "20",
        "--drive-report",
        "--lap-min-heading",
        "360",
        "--lap-min-path",
        "50",
        "--lap-close",
        "5",
        "--impact-min-kmh",
        "1",
        "--impact-max-kmh",
        "200",
        "--settle-report",
        "--settle-by",
        "1",
        "--settle-hold",
        "1",
        "--settle-speed",
        "0.001",
        "--energy-rise",
        "0.01",
        "--energy-gain",
        "0.01",
    ];
    let (code, out, _) = cli(&report_args);
    // no lap: the heading rule fails, so the verdict is FAIL (exit 1), but every line is there
    assert_eq!(code, Some(1));
    let names: Vec<&str> = out
        .lines()
        .take(7)
        .map(|l| l.split(['=', ' ']).next().unwrap())
        .collect();
    assert_eq!(
        names,
        [
            "lap_heading_deg",
            "lap_path_m",
            "lap_close_m",
            "chassis_ground_contacts_before_wall",
            "wall_first_contact_s",
            "wall_impact_speed_kmh",
            "final"
        ]
    );
    assert!(
        out.lines()
            .nth(7)
            .unwrap()
            .starts_with("drive: FAIL lap_heading_deg "),
        "{out}"
    );
    assert!(
        out.lines().nth(8).unwrap().starts_with("last_impact_s="),
        "the settle report follows the drive report: {out}"
    );
    assert!(
        out.contains(
            "lap_path_m=never
lap_close_m=never
"
        ),
        "no loop was driven: {out}"
    );
    assert_eq!(
        out.lines().filter(|l| l.starts_with("final t=")).count(),
        1,
        "one final line: {out}"
    );

    // the wall hit, against the rows of the same drive
    let contact = value(&out, "wall_first_contact_s");
    let impact_kmh = value(&out, "wall_impact_speed_kmh");
    assert!(
        contact > 8.0 && contact < 14.0,
        "the car hits the wall under power: {contact} s"
    );
    assert_eq!(value(&out, "chassis_ground_contacts_before_wall"), 0.0);
    let before: f64 = rows
        .lines()
        .skip(1)
        .filter_map(|l| {
            let f: Vec<f64> = l.split(',').filter_map(|v| v.parse().ok()).collect();
            (f.len() == 6 && f[0] < contact).then(|| f[1])
        })
        .last()
        .unwrap();
    assert!(
        (impact_kmh - before * 3.6).abs() <= 0.05 * before * 3.6,
        "impact {impact_kmh} km/h vs {} km/h in the last row before the wall",
        before * 3.6
    );
    // a wall that is never reached gives `none` and the rule that needs a number fails by name
    let no_wall = tmp_file(
        "no_wall.input",
        "0 0 0 0
",
    );
    let (code, out, _) = cli(&[
        "--body",
        &body,
        "--input",
        &no_wall,
        "--duration",
        "2",
        "--drive-report",
        "--lap-min-heading",
        "360",
        "--lap-min-path",
        "50",
        "--lap-close",
        "5",
        "--impact-min-kmh",
        "1",
        "--impact-max-kmh",
        "200",
    ]);
    assert_eq!(code, Some(1));
    assert!(
        out.contains(
            "wall_first_contact_s=none
wall_impact_speed_kmh=none
"
        ),
        "{out}"
    );
}

/// The drive report's flags: all five are required with --drive-report (no defaults), and it
/// needs an --input.
#[test]
fn drive_report_flags_are_required() {
    let path = car_path();
    let input = tmp_file(
        "flags.input",
        "0 0 0 0
",
    );
    let all = [
        "--lap-min-heading",
        "360",
        "--lap-min-path",
        "50",
        "--lap-close",
        "5",
        "--impact-min-kmh",
        "45",
        "--impact-max-kmh",
        "55",
    ];
    for skip in 0..5 {
        let mut args = vec![
            "--body",
            path.as_str(),
            "--input",
            input.as_str(),
            "--duration",
            "1",
            "--drive-report",
        ];
        for (i, pair) in all.chunks(2).enumerate() {
            if i != skip {
                args.extend(pair);
            }
        }
        let (code, out, _) = cli(&args);
        assert_eq!(
            (code, out.is_empty()),
            (Some(2), true),
            "missing flag {skip}"
        );
    }
    let mut no_input = vec!["--body", path.as_str(), "--duration", "1", "--drive-report"];
    no_input.extend(all);
    assert_eq!(cli(&no_input).0, Some(2));
    let mut no_report = vec![
        "--body",
        path.as_str(),
        "--input",
        input.as_str(),
        "--duration",
        "1",
    ];
    no_report.extend(&all[..2]);
    assert_eq!(cli(&no_report).0, Some(2));
    let mut nan = vec![
        "--body",
        path.as_str(),
        "--input",
        input.as_str(),
        "--duration",
        "1",
        "--drive-report",
    ];
    nan.extend([
        "--lap-min-heading",
        "NaN",
        "--lap-min-path",
        "50",
        "--lap-close",
        "5",
        "--impact-min-kmh",
        "45",
        "--impact-max-kmh",
        "55",
    ]);
    assert_eq!(cli(&nan).0, Some(2));
}

/// A drive needs the `front` and `rear` groups for its heading: a body without them is refused.
#[test]
fn a_drive_without_front_and_rear_groups_is_refused() {
    let text = car_text()
        .replace("group front", "group nose")
        .replace("group rear", "group tail");
    let body = tmp_file("no_groups.bsn", &text);
    let input = tmp_file(
        "no_groups.input",
        "0 0 0 0
",
    );
    let (code, out, err) = cli(&["--body", &body, "--input", &input, "--duration", "1"]);
    assert_eq!((code, out.is_empty()), (Some(2), true));
    assert!(err.contains("front") || err.contains("rear"), "{err}");
}

const SETTLE_ARGS: [&str; 10] = [
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
];

/// A one-line script with no --duration would run 0 substeps: refused (exit 2) with the cause,
/// for the rows and the settle report; with --duration, or in a slope run (which has its own
/// length), it runs.
#[test]
fn a_script_that_ends_at_zero_needs_a_duration() {
    let path = car_path();
    let input = tmp_file(
        "zero.input",
        "0 0 1 0
",
    );
    let mut settle = vec!["--body", path.as_str(), "--input", input.as_str()];
    settle.extend(SETTLE_ARGS);
    settle.push("0.01");
    for args in [
        vec!["--body", path.as_str(), "--input", input.as_str()],
        settle.clone(),
    ] {
        let (code, out, err) = cli(&args);
        assert_eq!((code, out.is_empty()), (Some(2), true), "{args:?}");
        assert!(
            err.contains("0 substeps") && err.contains("--duration"),
            "{err}"
        );
    }
    settle.extend(["--duration", "12"]);
    assert_eq!(cli(&settle).0, Some(0));
    let (code, out, _) = cli(&[
        "--body",
        &path,
        "--input",
        &input,
        "--slope-grip-frac",
        "1.2",
        "--slope-report",
        "--slope-from",
        "1",
        "--slope-to",
        "2",
    ]);
    assert_eq!(code, Some(0), "{out}");
}

/// The settle report of a car with --input needs no front/rear groups (only the rows and the
/// drive report need a heading): it runs, and prints no `final` line.
#[test]
fn settle_report_with_input_needs_no_heading_groups() {
    let text = car_text()
        .replace("group front", "group nose")
        .replace("group rear", "group tail");
    let body = tmp_file("no_groups_settle.bsn", &text);
    let input = tmp_file(
        "no_groups_settle.input",
        "0 0 1 0
",
    );
    let mut args = vec![
        "--body",
        body.as_str(),
        "--input",
        input.as_str(),
        "--duration",
        "12",
    ];
    args.extend(SETTLE_ARGS);
    args.push("0.01");
    let (code, out, _) = cli(&args);
    assert_eq!(code, Some(0), "{out}");
    assert!(
        out.lines().any(|l| l == "settle: ok") && !out.contains("final t="),
        "{out}"
    );
}

/// --events belongs to a plain run: a car run refuses it, as it refuses --summary.
#[test]
fn car_runs_refuse_plain_run_flags() {
    let path = car_path();
    let input = tmp_file(
        "events.input",
        "0 0 0 0
",
    );
    for flag in ["--events", "--summary"] {
        let (code, out, err) = cli(&["--body", &path, "--input", &input, "--duration", "1", flag]);
        assert_eq!((code, out.is_empty()), (Some(2), true), "{flag}");
        assert!(err.contains(flag), "{err}");
    }
}

// ---------------------------------------------------------------------------------------------
// --trajectory-v2 (docs/STAGE6.md, "The live protocol")
// ---------------------------------------------------------------------------------------------

const V2_INPUT: &str = "0 0 0 0\n0.5 0.6 0 0\n1 0.3 0 -0.5\n";

/// Without `--trajectory-v2` an --input run prints exactly the drive rows and the `final` line of
/// the library's own `drive` (the default output is unchanged); with it the file is trajectory v2,
/// every node per frame, and has neither the drive header nor a `final` line.
#[test]
fn trajectory_v2_flag_replaces_rows_and_default_is_unchanged() {
    use crash_sim::input::{drive, render_rows};
    let path = car_path();
    let input = tmp_file("v2_flag.input", V2_INPUT);
    let body = car();

    // The default: the library's rows and final line, byte for byte.
    let cfg = Config::default();
    let setup = CarSetup {
        script: Some(script(V2_INPUT)),
        until_script_end: true,
        grip_frac: None,
    };
    let run = drive(&cfg, &body, &setup, f64::INFINITY).unwrap();
    let expected = format!("{}{}", render_rows(&run.rows), run.final_state.render());
    let (code, rows, _) = cli(&["--body", &path, "--input", &input]);
    assert_eq!(code, Some(0));
    assert_eq!(
        rows, expected,
        "the default output must stay the drive rows"
    );

    // The flag: trajectory v2.
    let (code, v2, err) = cli(&["--body", &path, "--input", &input, "--trajectory-v2"]);
    assert_eq!(code, Some(0), "{err}");
    assert!(v2.starts_with("trajectory v2\nnodes,"), "{v2}");
    assert!(!v2.contains("final "), "a v2 file has no final line");
    assert!(!v2.contains("speed_mps"), "a v2 file has no drive header");
    let lines: Vec<&str> = v2.lines().collect();
    let n: usize = lines[1].strip_prefix("nodes,").unwrap().parse().unwrap();
    assert_eq!(n, body.nodes.len());
    let columns = lines.iter().position(|l| *l == "t,node,x,y,z").unwrap();
    let beams = lines[2..columns]
        .iter()
        .filter(|l| l.starts_with("beam,"))
        .count();
    assert_eq!(beams, body.beams.len());
    let data: Vec<&str> = lines[columns + 1..]
        .iter()
        .copied()
        .filter(|l| !l.starts_with("break,"))
        .collect();
    // one frame at t = 0 and one per output step: the rows of the default run, one for one.
    assert_eq!(data.len(), n * run.rows.len());
    for (k, row) in run.rows.iter().enumerate() {
        let frame = &data[k * n..(k + 1) * n];
        for (i, l) in frame.iter().enumerate() {
            assert!(l.starts_with(&format!("{:.4},{i},", row.t)), "{l}");
        }
    }

    // A report flag, or no input, is refused: the flag replaces the rows, nothing else prints.
    let (code, _, err) = cli(&["--body", &path, "--trajectory-v2"]);
    assert_eq!(code, Some(2), "{err}");
    let (code, _, err) = cli(&[
        "--body",
        &path,
        "--input",
        &input,
        "--trajectory-v2",
        "--slope-report",
    ]);
    assert_eq!(code, Some(2), "{err}");
}
