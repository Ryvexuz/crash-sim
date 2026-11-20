//! Stage 6 milestone 3 done check, crash-sim half (nb-41, docs/STAGE6.md check 3.1 and 3.6):
//! drive a car around the pad and crash it into a wall.
//!
//! `pad_and_wall_car_is_car_basic`: the car that crashes is `car_basic.bsn` byte for byte, plus
//! one wall plane, so it is the car whose statics (3.3), grip (3.4), launch and steering (3.5)
//! are checked.
//!
//! `pad_lap_closes_and_hits_wall`: the 3.6 run, in-process, with the numbers of 3.6 and the same
//! rules: the drive report ok (loop of at least 360 degrees and 50 m, back within 5 m, no chassis
//! node on the ground before the wall, impact 45 to 55 km/h), the first wall contact before
//! t_last = 60 s, the settle report ok from 68 s to 72 s (no abort, so no NaN), and the crumple
//! between `rear` and `front` at least 0.05 m, at most 1.0 m, drifting at most 0.001 m.
//!
//! How `scenarios/pad_lap.input` is aimed (by hand, from the physics; nothing in the sim is
//! tuned). Wheelbase L = 2.5 m (mounts at x = -1.25 and 1.25), max steer 30 degrees, launch
//! acceleration at full throttle 5.00 m/s^2 (torque-limited, check 3.5), so a throttle u gives
//! about 5.00 u m/s^2 below the grip limit.
//! - `0 0 0 0` to 2 s: the car drops 0.2 m onto its tyres and the body motion dies down.
//! - `2 0.25 0 0.47`: steer 0.47 is 14.1 degrees, a kinematic radius L / tan(14.1 deg) = 9.96 m
//!   (10 m chosen: a 63 m circle, above the 50 m path floor, at under 0.25 g); throttle 0.25 gives
//!   1.25 m/s^2, so about 5 m/s after 4 s. Accelerating on the circle keeps the radius (low
//!   lateral acceleration), so the loop still closes at the start.
//! - `6 0 0 0.47`: coast round at about 4.7 m/s (cornering drag slows it to about 4.1 m/s): the
//!   rest of the 2 pi x 10 m circle takes about 12 s, so the heading passes 360 degrees at about
//!   18.4 s, back at the start pointing +x at the wall.
//! - `18.5 0.5 0 0`: straight; throttle 0.5 gives 2.5 m/s^2, from about 4.1 m/s to 50 km/h
//!   (13.89 m/s) in (13.89 - 4.1) / 2.5 = 3.9 s, about 35 m.
//! - `22.4 0 0 0`: coast at about 50 km/h (no drag term, check 3.5) the last metres to the wall
//!   at x = 50 (the front face starts at x = 2, 48 m away): first contact at about 23.3 s.
//! - `24 0 1 0`: full brake once the crash is over, so the car stays by the wall instead of
//!   rolling back from its rebound for the 36 s to t_last (the car has no rolling drag).
//! - `60 0 1 0`: t_last, the line 3.6 fixes; the brake holds to the end of the 72 s run.

use crash_sim::collide::{self, CrumpleParams};
use crash_sim::format::{parse, Body};
use crash_sim::input::{self, DriveParams, Script};
use crash_sim::settle::{CarSetup, SettleParams};
use crash_sim::Config;

fn scenario(name: &str) -> String {
    let path = format!("{}/scenarios/{name}", env!("CARGO_MANIFEST_DIR"));
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"))
}

fn body(name: &str) -> Body {
    parse(&scenario(name)).unwrap_or_else(|e| panic!("{name}: {e}"))
}

/// Path of the crash-sim binary, read at run time (as `tests/live.rs`): the copy a broadsword
/// test step points `CARGO_BIN_EXE_crash-sim` at, else the cargo layout next to the test
/// executable.
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
    assert!(bin.is_file(), "no crash-sim binary at {}", bin.display());
    bin
}

/// The arguments of the 3.6 command after `--`, exactly as docs/STAGE6.md writes them; the
/// program runs in the project folder, as `broadsword run` runs it.
const CHECK_3_6: &[&str] = &[
    "--body",
    "scenarios/pad_and_wall.bsn",
    "--input",
    "scenarios/pad_lap.input",
    "--duration",
    "72",
    "--drive-report",
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
    "--settle-report",
    "--settle-by",
    "68",
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
];

/// Run the binary in the project folder; (exit code, stdout, stderr).
fn cli(args: &[&str]) -> (Option<i32>, String, String) {
    let o = std::process::Command::new(crash_sim_bin())
        .args(args)
        .current_dir(env!("CARGO_MANIFEST_DIR"))
        .output()
        .unwrap();
    let (out, err) = (
        String::from_utf8_lossy(&o.stdout).into_owned(),
        String::from_utf8_lossy(&o.stderr).into_owned(),
    );
    println!("{args:?}: exit {:?}\n{out}{err}", o.status.code());
    (o.status.code(), out, err)
}

#[test]
fn pad_and_wall_car_is_car_basic() {
    let car = scenario("car_basic.bsn");
    let pad = scenario("pad_and_wall.bsn");
    // Byte for byte: car_basic.bsn is the start of pad_and_wall.bsn.
    assert!(
        pad.as_bytes().starts_with(car.as_bytes()),
        "pad_and_wall.bsn does not start with car_basic.bsn byte for byte"
    );
    assert!(car.ends_with('\n'), "car_basic.bsn must end with a newline");
    // What follows is comments and exactly one plane line, nothing else.
    let rest = &pad[car.len()..];
    let lines: Vec<&str> = rest
        .lines()
        .filter(|l| !l.trim().is_empty() && !l.starts_with('#'))
        .collect();
    assert_eq!(
        lines.len(),
        1,
        "after the car: exactly one non-comment line, got {lines:?}"
    );
    assert!(
        lines[0].starts_with("plane "),
        "the added line is not a plane: {}",
        lines[0]
    );

    // Parsed, the two bodies differ only in that plane, and car_basic.bsn has none.
    let (c, p) = (body("car_basic.bsn"), body("pad_and_wall.bsn"));
    assert!(c.planes.is_empty(), "car_basic.bsn has planes");
    assert_eq!(p.planes.len(), 1);
    let mut stripped = p.clone();
    stripped.planes.clear();
    assert_eq!(stripped, c, "the parsed cars differ");
    // The wall faces the car (normal -x, toward its start), ahead of its front face (x = 2).
    let wall = collide::Plane::from_format(&p.planes[0]).unwrap();
    assert_eq!(wall.normal.x, -1.0);
    assert_eq!((wall.normal.y, wall.normal.z), (0.0, 0.0));
    assert!(wall.point.x > 2.0, "the wall is not ahead of the car");
}

#[test]
fn pad_lap_closes_and_hits_wall() {
    // The 3.6 command: --body scenarios/pad_and_wall.bsn --input scenarios/pad_lap.input
    // --duration 72 (dt and substeps as the command line's defaults).
    let text = scenario("pad_lap.input");
    assert_eq!(text.lines().next(), Some("0 0 0 0"), "first line");
    assert_eq!(text.lines().last(), Some("60 0 1 0"), "last line");
    let script = Script::parse(&text).unwrap();
    assert_eq!(script.t_last(), 60.0);
    let car = body("pad_and_wall.bsn");
    let cfg = Config {
        duration: 72.0,
        ..Config::default()
    };
    let setup = CarSetup {
        script: Some(script),
        until_script_end: false,
        grip_frac: None,
    };

    // --drive-report --lap-min-heading 360 --lap-min-path 50 --lap-close 5
    // --impact-min-kmh 45 --impact-max-kmh 55
    let dp = DriveParams {
        lap_min_heading: 360.0,
        lap_min_path: 50.0,
        lap_close: 5.0,
        impact_min_kmh: 45.0,
        impact_max_kmh: 55.0,
    };
    let run = input::drive(&cfg, &car, &setup, dp.lap_min_heading).unwrap();
    assert_eq!(run.abort, None, "the drive aborted");
    let m = &run.measure;
    print!("{}", m.render(&dp, &run.final_state));
    assert_eq!(m.verdict(&dp), Ok(()), "drive: FAIL");
    assert!(m.lap_heading_deg >= 360.0);
    assert!(m.lap_path_m.is_some_and(|p| p >= 50.0));
    assert!(m.lap_close_m.is_some_and(|c| c <= 5.0));
    assert_eq!(m.chassis_ground_contacts_before_wall, 0);
    let v = m.wall_impact_speed_kmh.expect("no impact speed");
    assert!((45.0..=55.0).contains(&v), "impact {v} km/h");
    // The settle window must start after the crash.
    let hit = m
        .wall_first_contact_s
        .expect("the car never touched the wall");
    assert!(
        hit < 60.0,
        "first wall contact at {hit} s, not before t_last = 60 s"
    );
    assert!(
        (run.final_state.t - 72.0).abs() < 1e-9,
        "the run ends at {}",
        run.final_state.t
    );

    // --settle-report --settle-by 68 --settle-hold 4 --settle-speed 0.001 --energy-rise 0.01
    // --energy-gain 0.01 --crumple-report --crumple-groups rear,front --crumple-min 0.05
    // --crumple-max 1.0 --crumple-drift 0.001
    let sp = SettleParams {
        by: 68.0,
        hold: 4.0,
        speed: 0.001,
        energy_rise: 0.01,
        energy_gain: 0.01,
    };
    let cp = CrumpleParams {
        from: "rear".into(),
        to: "front".into(),
        min: 0.05,
        max: 1.0,
        drift: 0.001,
    };
    let (settle, crumple) = collide::measure_crumple_car(&cfg, &car, &setup, &sp, &cp).unwrap();
    print!("{}", settle.render());
    assert!(
        !settle.aborted,
        "the settle run aborted (a NaN or a strain limit)"
    );
    assert_eq!(settle.verdict, Ok(()), "settle: FAIL");
    assert!(settle.final_center_y.is_finite());
    let c = crumple.expect("no crumple report");
    print!("{}", c.render());
    assert_eq!(c.verdict, Ok(()), "crumple: FAIL");
    // rear and front of car_basic.bsn are 4 m apart (x = -2 and x = 2).
    assert!(
        (c.start_m - 4.0).abs() < 1e-12,
        "crumple start {}",
        c.start_m
    );
    assert!(c.delta_min_after.is_some_and(|d| d >= 0.05));
    assert!(c.delta_max <= 1.0);
    assert!(c.delta_range_after.is_some_and(|r| r <= 0.001));
}

/// The 3.6 command line itself (the real binary, the exact flags): exit 0, `drive: ok` and
/// `settle: ok` in the output, the last line `crumple: ok`, and the reports in the order drive,
/// settle, crumple, with the same numbers as the in-process run.
#[test]
fn pad_lap_command_line_passes_check_3_6() {
    let (code, out, err) = cli(CHECK_3_6);
    assert_eq!(code, Some(0), "exit code; stderr: {err}");
    assert!(err.is_empty(), "stderr: {err}");
    let lines: Vec<&str> = out.lines().collect();
    assert_eq!(lines.last(), Some(&"crumple: ok"), "last line");
    let at = |l: &str| {
        lines
            .iter()
            .position(|x| *x == l)
            .unwrap_or_else(|| panic!("no line `{l}`"))
    };
    let (drive, settle, crumple) = (at("drive: ok"), at("settle: ok"), at("crumple: ok"));
    assert!(drive < settle && settle < crumple, "report order");
    assert!(!out.contains("FAIL"), "a FAIL line");
    let wall: f64 = lines
        .iter()
        .find_map(|l| l.strip_prefix("wall_first_contact_s="))
        .expect("no wall_first_contact_s line")
        .parse()
        .expect("wall_first_contact_s is not a number");
    assert!(wall < 60.0, "wall_first_contact_s={wall}");
    assert!(lines.contains(&"crumple_start_m=4.000000"));
}

/// The crumple report runs with a car's --input drive, and nowhere it would not be measured on
/// the car's own run: with a slope flag it is refused (exit 2, nothing on stdout), and a missing
/// crumple group is a usage error (exit 2) before any output.
#[test]
fn crumple_with_input_refusals_exit_2() {
    let mut slope: Vec<&str> = CHECK_3_6.to_vec();
    slope.extend(["--slope-grip-frac", "0.5"]);
    let (code, out, err) = cli(&slope);
    assert_eq!(code, Some(2), "slope + crumple");
    assert!(out.is_empty(), "stdout: {out}");
    assert!(
        err.contains("--crumple-report cannot be used with --slope-grip-frac"),
        "{err}"
    );

    let groups: Vec<&str> = CHECK_3_6
        .iter()
        .map(|a| if *a == "rear,front" { "rear,nose" } else { *a })
        .collect();
    let (code, out, err) = cli(&groups);
    assert_eq!(code, Some(2), "missing group");
    assert!(out.is_empty(), "stdout: {out}");
    assert!(err.contains("no group `nose`"), "{err}");
}
