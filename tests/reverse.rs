//! Reverse via a negative throttle (task t-20261120-220016-2, docs/STAGE6.md "Reverse"): the
//! range of the throttle in the input grammar (`--input` and `--live` share `LineParser`), the
//! rule that reverse torque applies only below `vehicle::REVERSE_MAX_FORWARD_SPEED`, and that a
//! throttle of 0 or more is not changed by it.

use crash_sim::format::{parse, Body};
use crash_sim::input::{LineParser, Script};
use crash_sim::vec3::Vec3;
use crash_sim::vehicle::REVERSE_MAX_FORWARD_SPEED;
use crash_sim::{Config, Sim};
use std::io::Write;
use std::process::{Command, Stdio};

fn car_path() -> String {
    format!("{}/scenarios/car_basic.bsn", env!("CARGO_MANIFEST_DIR"))
}

fn car() -> Body {
    let text = std::fs::read_to_string(car_path()).unwrap();
    parse(&text).unwrap()
}

/// One `advance` is one 1 ms substep.
fn cfg(duration: f64) -> Config {
    Config {
        duration,
        dt: 0.001,
        substeps: 1,
        ..Config::default()
    }
}

fn com(sim: &Sim) -> Vec3 {
    let mut p = Vec3::ZERO;
    let mut m = 0.0;
    for (x, mi) in sim.pos.iter().zip(&sim.mass) {
        p += *x * *mi;
        m += *mi;
    }
    p * (1.0 / m)
}

/// A sim of the car at rest, with the controls held at (`throttle`, 0, 0) for `seconds`.
fn run_at_rest(throttle: f64, seconds: f64) -> Sim {
    let mut sim = Sim::from_car(cfg(seconds), &car()).unwrap();
    sim.car
        .as_mut()
        .unwrap()
        .set_controls(throttle, 0.0, 0.0)
        .unwrap();
    for _ in 0..(seconds * 1000.0).round() as u64 {
        sim.advance();
        sim.check().unwrap();
    }
    sim
}

/// From rest, full reverse for 3 s moves the car backwards: its centre of mass goes along
/// -forward (the car faces +x), by metres, and the same run at throttle +1 goes the other way.
#[test]
fn reverse_from_rest_moves_the_car_backwards() {
    let start = com(&Sim::from_car(cfg(3.0), &car()).unwrap());
    let back = run_at_rest(-1.0, 3.0);
    let fwd = run_at_rest(1.0, 3.0);
    let along_minus_heading = -(com(&back) - start).x;
    let along_plus_heading = (com(&fwd) - start).x;
    println!(
        "reverse 3 s: {along_minus_heading:.3} m along -heading; forward 3 s: {along_plus_heading:.3} m along +heading"
    );
    assert!(
        along_minus_heading > 1.0,
        "reverse moved {along_minus_heading} m along -heading"
    );
    assert!(
        along_plus_heading > 1.0,
        "forward moved {along_plus_heading} m along +heading"
    );
    // Reverse is the same torque scale as forward (the distances differ with the weight
    // transfer and the tyre, not by a factor of 2), not a token push.
    let ratio = along_minus_heading / along_plus_heading;
    assert!((0.5..2.0).contains(&ratio), "reverse/forward distance {ratio}");
}

/// A sim of the car with every node and wheel moving forward at `speed` (m/s), wheels spinning
/// at the rolling rate, and the controls held at (`throttle`, 0, 0).
fn moving_sim(speed: f64, throttle: f64) -> Sim {
    let mut sim = Sim::from_car(cfg(1.0), &car()).unwrap();
    for v in sim.vel.iter_mut() {
        *v = Vec3::new(speed, 0.0, 0.0);
    }
    let car = sim.car.as_mut().unwrap();
    for w in car.wheels.iter_mut() {
        w.vel = Vec3::new(speed, 0.0, 0.0);
        w.spin = speed / w.p.radius;
    }
    car.set_controls(throttle, 0.0, 0.0).unwrap();
    sim
}

/// At 20 m/s forward a negative throttle adds no drive torque: the drive work is exactly 0 and
/// the run is bit for bit the run at throttle 0. The control: at 0.5 m/s (below the limit) the
/// same throttle does work, so the zero above is the rule and not a dead input.
#[test]
fn reverse_throttle_at_speed_adds_no_drive_torque() {
    assert!(20.0 > REVERSE_MAX_FORWARD_SPEED && 0.5 < REVERSE_MAX_FORWARD_SPEED);
    let mut rev = moving_sim(20.0, -1.0);
    let mut idle = moving_sim(20.0, 0.0);
    for _ in 0..500 {
        rev.advance();
        idle.advance();
        rev.check().unwrap();
        idle.check().unwrap();
    }
    let work = rev.car.as_ref().unwrap().drive_work;
    println!("20 m/s, throttle -1: drive work {work:e} J");
    assert!(work == 0.0, "drive work {work} at 20 m/s with throttle -1");
    assert!(
        rev.pos == idle.pos && rev.vel == idle.vel,
        "throttle -1 at 20 m/s must equal throttle 0"
    );
    let mut slow = moving_sim(0.5, -1.0);
    for _ in 0..50 {
        slow.advance();
    }
    let slow_work = slow.car.as_ref().unwrap().drive_work;
    println!("0.5 m/s, throttle -1: drive work {slow_work:e} J");
    assert!(slow_work != 0.0, "no drive work below the speed limit");
}

/// -1 is accepted and -1.01 is refused, by the parser, a script and the car, each with a named
/// error; a NaN is still refused.
#[test]
fn throttle_range_is_minus_one_to_one() {
    let mut p = LineParser::new();
    let l = p.feed("0 -1 0 0").unwrap().unwrap();
    assert_eq!(l.throttle, -1.0);
    let mut p = LineParser::new();
    let e = p.feed("0 -1.01 0 0").unwrap_err();
    assert!(
        e.contains("input line 1: throttle -1.01 must be from -1 to 1"),
        "{e}"
    );
    assert!(Script::parse("0 -1 0 0\n").is_ok());
    let e = Script::parse("0 0 0 0\n1 -1.01 0 0\n").unwrap_err();
    assert!(e.contains("input line 2: throttle -1.01"), "{e}");
    assert!(LineParser::new().feed("0 NaN 0 0").is_err());
    assert!(LineParser::new().feed("0 1.01 0 0").is_err());
    let mut sim = Sim::from_car(cfg(1.0), &car()).unwrap();
    let c = sim.car.as_mut().unwrap();
    assert!(c.set_controls(-1.0, 0.0, 0.0).is_ok());
    let e = c.set_controls(-1.01, 0.0, 0.0).unwrap_err();
    assert!(e.contains("throttle -1.01 must be a number from -1 to 1"), "{e}");
    assert_eq!(c.throttle, -1.0, "a refused control changes nothing");
    assert!(c.set_controls(f64::NAN, 0.0, 0.0).is_err());
}

fn crash_sim_bin() -> std::path::PathBuf {
    if let Some(p) = std::env::var_os("CARGO_BIN_EXE_crash-sim") {
        return std::path::PathBuf::from(p);
    }
    let exe = std::env::current_exe().expect("current_exe of the test executable");
    let name = format!("crash-sim{}", std::env::consts::EXE_SUFFIX);
    let dir = exe.parent().and_then(|p| p.parent()).unwrap_or_else(|| {
        panic!(
            "CARGO_BIN_EXE_crash-sim is not set and {} has no grandparent folder",
            exe.display()
        )
    });
    let bin = dir.join(name);
    assert!(bin.is_file(), "{} does not exist", bin.display());
    bin
}

/// The same lines through `--input` and `--live`: -1 is accepted by both (exit 0, the same
/// bytes, the car ends behind where it started) and -1.01 is refused by both with exit 2.
#[test]
fn input_and_live_accept_minus_one_and_refuse_below() {
    let dir = std::path::PathBuf::from(env!("CARGO_TARGET_TMPDIR"));
    std::fs::create_dir_all(&dir).unwrap();
    let live = |text: &str| {
        let mut child = Command::new(crash_sim_bin())
            .args(["--body", &car_path(), "--live"])
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .unwrap();
        child
            .stdin
            .take()
            .unwrap()
            .write_all(text.as_bytes())
            .unwrap();
        child.wait_with_output().unwrap()
    };
    let file = |name: &str, text: &str| {
        let path = dir.join(name);
        std::fs::write(&path, text).unwrap();
        Command::new(crash_sim_bin())
            .args(["--body", &car_path(), "--input"])
            .arg(&path)
            .output()
            .unwrap()
    };
    let good = "0 -1 0 0\n2 0 0 0\n";
    let (f, l) = (file("reverse_ok.input", good), live(good));
    let (fo, lo) = (
        String::from_utf8_lossy(&f.stdout).into_owned(),
        String::from_utf8_lossy(&l.stdout).into_owned(),
    );
    assert_eq!(f.status.code(), Some(0), "{}", String::from_utf8_lossy(&f.stderr));
    assert_eq!(l.status.code(), Some(0), "{}", String::from_utf8_lossy(&l.stderr));
    assert_eq!(fo, lo, "--input and --live give the same bytes");
    let last = fo.lines().last().unwrap();
    let x: f64 = last
        .split("x=")
        .nth(1)
        .and_then(|s| s.split_whitespace().next())
        .unwrap_or_else(|| panic!("no x= in {last:?}"))
        .parse()
        .unwrap();
    println!("final line: {last}");
    assert!(x < -0.3, "the car reversed, final x {x}");

    let bad = "0 -1.01 0 0\n";
    let (f, l) = (file("reverse_bad.input", bad), live(bad));
    for (what, o) in [("--input", f), ("--live", l)] {
        let err = String::from_utf8_lossy(&o.stderr).into_owned();
        assert_eq!(o.status.code(), Some(2), "{what}: {err}");
        assert!(
            err.contains("input line 1: throttle -1.01 must be from -1 to 1"),
            "{what}: {err}"
        );
    }
}
