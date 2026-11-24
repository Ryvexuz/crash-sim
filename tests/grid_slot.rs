//! Single-car grid selection, checked against the library race and live input.

use crash_sim::ai::DriverParams;
use crash_sim::input::FinalState;
use crash_sim::race::{self, ProfileParams, RaceSetup};
use crash_sim::track::{LapEvent, Track};
use crash_sim::vec3::Vec3;
use crash_sim::{Body, Config};
use std::fmt::Write as _;
use std::io::Write as _;
use std::process::{Command, Stdio};

/// Path of the crash-sim binary, read at run time: the copy a broadsword test step points
/// `CARGO_BIN_EXE_crash-sim` at, else the cargo layout next to the test executable.
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

struct Run {
    code: i32,
    stdout: String,
    stderr: String,
}

fn run(args: &[&str]) -> Run {
    let out = Command::new(crash_sim_bin())
        .args(args)
        .stdin(Stdio::null())
        .output()
        .expect("run crash-sim");
    Run {
        code: out.status.code().expect("exit code"),
        stdout: String::from_utf8(out.stdout).expect("stdout is UTF-8"),
        stderr: String::from_utf8(out.stderr).expect("stderr is UTF-8"),
    }
}

fn body() -> Body {
    crash_sim::format::parse(include_str!("../scenarios/car_basic.bsn")).unwrap()
}

fn track() -> Track {
    Track::parse(include_str!("../scenarios/tracks/oval.track")).unwrap()
}

fn setup<'a>(cfg: &'a Config, body: &'a Body, track: &'a Track) -> RaceSetup<'a> {
    RaceSetup {
        cfg,
        body,
        track,
        cars: 1,
        laps: 1,
        time_limit_s: 70.0,
        grid_gap_m: 10.0,
        driver: DriverParams {
            lookahead_k: 0.6,
            lookahead_min: 3.0,
            lookahead_max: 12.0,
            kp: 0.8,
            ki: 0.3,
            integral_max: 2.0,
        },
        profile: ProfileParams {
            mu: 0.6,
            g: 9.81,
            a_accel: 2.0,
            a_brake: 4.0,
            v_max: 15.0,
        },
    }
}

fn manifest(path: &str) -> String {
    format!("{}/{path}", env!("CARGO_MANIFEST_DIR"))
}

fn file_run(input: &str, extra: &[&str]) -> Run {
    let car = manifest("scenarios/car_basic.bsn");
    let oval = manifest("scenarios/tracks/oval.track");
    let mut args = vec!["--body", &car, "--track", &oval, "--input", input];
    args.extend_from_slice(extra);
    run(&args)
}

fn final_line(text: &str) -> &str {
    let lines: Vec<_> = text.lines().filter(|l| l.starts_with("final ")).collect();
    assert_eq!(lines.len(), 1, "{text}");
    lines[0]
}

#[test]
fn grid_slot_one_equals_the_milestone_one_run() {
    let input = manifest("scenarios/tracks/oval_lap.input");
    let baseline = file_run(&input, &["--duration", "3"]);
    assert_eq!(baseline.code, 0, "{}", baseline.stderr);
    final_line(&baseline.stdout);
    for gap in ["10", "7.5"] {
        let r = file_run(
            &input,
            &["--duration", "3", "--grid-slot", "1", "--grid-gap", gap],
        );
        assert_eq!(r.code, 0, "{}", r.stderr);
        assert_eq!(r.stdout.as_bytes(), baseline.stdout.as_bytes());
    }
}

#[test]
fn grid_slot_three_matches_the_race_library() {
    let (cfg, body, track) = (Config::default(), body(), track());
    let mut s = setup(&cfg, &body, &track);
    s.cars = 3;
    s.time_limit_s = 10.0;
    let r = race::run(&s).unwrap();
    assert_eq!(r.contacts, 0, "contact invalidates independent replay");
    assert_eq!(r.h.to_bits(), 0.001_f64.to_bits());
    let mut script = String::new();
    for (j, c) in r.cars[2].inputs.iter().enumerate() {
        let ms = j * 20;
        writeln!(
            script,
            "{}.{:03} {} {} {}",
            ms / 1000,
            ms % 1000,
            c.throttle,
            c.brake,
            c.steer
        )
        .unwrap();
    }
    let ms = r.substeps;
    let c = r.cars[2].inputs.last().unwrap();
    writeln!(
        script,
        "{}.{:03} {} {} {}",
        ms / 1000,
        ms % 1000,
        c.throttle,
        c.brake,
        c.steer
    )
    .unwrap();
    let input = tmp_file("grid_slot_three.input", &script);
    let actual = file_run(&input, &["--grid-slot", "3", "--grid-gap", "10"]);
    assert_eq!(actual.code, 0, "{}", actual.stderr);
    let events: Vec<_> = actual
        .stdout
        .lines()
        .filter(|l| l.starts_with("start ") || l.starts_with("checkpoint "))
        .collect();
    let expected: Vec<_> = r.cars[2]
        .events
        .iter()
        .map(|event| match event {
            LapEvent::Start { t } => format!("start t={t:.4}"),
            LapEvent::Checkpoint { lap, k, t } => {
                format!("checkpoint lap={lap} k={k} t={t:.4}")
            }
            LapEvent::Lap(_) => panic!("unexpected lap within the ten-second replay"),
        })
        .collect();
    assert_eq!(events, expected);
    let sim = r.multi.sim(2);
    let mut centroid = Vec3::ZERO;
    for p in &sim.pos {
        centroid += *p;
    }
    centroid = centroid * (1.0 / sim.pos.len() as f64);
    let mean = |name: &str| {
        let group = body.groups.iter().find(|g| g.name == name).unwrap();
        let mut sum = Vec3::ZERO;
        for id in &group.nodes {
            sum += sim.pos[body.node_index(*id).unwrap()];
        }
        sum * (1.0 / group.nodes.len() as f64)
    };
    let d = mean("front") - mean("rear");
    let expected_final = FinalState {
        t: r.substeps as f64 * r.h,
        x: centroid.x,
        z: centroid.z,
        heading_deg: d.z.atan2(d.x).to_degrees(),
    }
    .render();
    assert_eq!(final_line(&actual.stdout), expected_final.trim_end());
    eprintln!(
        "grid_slot compared_events={} final={}",
        events.len(),
        expected_final.trim_end()
    );
    let other = file_run(&input, &["--grid-slot", "1", "--grid-gap", "10"]);
    assert_eq!(other.code, 0, "{}", other.stderr);
    assert_ne!(final_line(&actual.stdout), final_line(&other.stdout));
}

#[test]
fn grid_slot_refusals_exit_2_with_empty_stdout() {
    let input = manifest("scenarios/tracks/oval_lap.input");
    let cases: &[(&[&str], &str)] = &[
        (&["--grid-slot", "0", "--grid-gap", "10"], "expected 1 to 3"),
        (&["--grid-slot", "4", "--grid-gap", "10"], "expected 1 to 3"),
        (
            &["--grid-slot", "1.5", "--grid-gap", "10"],
            "--grid-slot needs a numeric value",
        ),
        (&["--grid-slot", "2"], "needs --grid-gap"),
        (&["--grid-gap", "10"], "needs --grid-slot"),
        (
            &["--grid-slot", "2", "--grid-gap", "0"],
            "must be finite and above zero",
        ),
        (
            &["--grid-slot", "2", "--grid-gap", "-1"],
            "must be finite and above zero",
        ),
        (
            &["--grid-slot", "2", "--grid-gap", "nan"],
            "must be finite and above zero",
        ),
        (
            &["--grid-slot", "2", "--grid-gap", "inf"],
            "must be finite and above zero",
        ),
        (
            &["--grid-slot", "3", "--grid-gap", "200"],
            "beyond the centreline loop",
        ),
    ];
    for (flags, needle) in cases {
        let r = file_run(&input, flags);
        assert_eq!(r.code, 2, "{flags:?}: {}", r.stderr);
        assert!(r.stdout.is_empty(), "{flags:?}: {}", r.stdout);
        assert!(r.stderr.contains(needle), "{flags:?}: {}", r.stderr);
    }
    let r = run(&[
        "--body",
        &manifest("scenarios/car_basic.bsn"),
        "--input",
        &input,
        "--grid-slot",
        "2",
        "--grid-gap",
        "10",
    ]);
    assert_eq!(r.code, 2, "{}", r.stderr);
    assert!(r.stdout.is_empty());
    assert!(r.stderr.contains("need --track"), "{}", r.stderr);
}

#[test]
fn grid_slot_live_mode_places_the_car() {
    let live = |slot: &str| {
        let mut child = Command::new(crash_sim_bin())
            .args([
                "--body",
                &manifest("scenarios/car_basic.bsn"),
                "--track",
                &manifest("scenarios/tracks/oval.track"),
                "--live",
                "--grid-slot",
                slot,
                "--grid-gap",
                "10",
            ])
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .unwrap();
        child
            .stdin
            .take()
            .unwrap()
            .write_all(b"0.000 0 1 0\n1.000 0 1 0\n")
            .unwrap();
        let out = child.wait_with_output().unwrap();
        assert!(
            out.status.success(),
            "{}",
            String::from_utf8_lossy(&out.stderr)
        );
        String::from_utf8(out.stdout).unwrap()
    };
    let (one, two) = (live("1"), live("2"));
    let coordinate = |text: &str, key: &str| -> f64 {
        final_line(text)
            .split_whitespace()
            .find_map(|f| f.strip_prefix(key))
            .unwrap()
            .parse()
            .unwrap()
    };
    let dx = coordinate(&two, "x=") - coordinate(&one, "x=");
    let dz = coordinate(&two, "z=") - coordinate(&one, "z=");
    eprintln!("grid_slot live x2-x1={dx} z2-z1={dz}");
    assert!((-10.01..=-9.99).contains(&dx), "{dx}");
    assert!(dz.abs() <= 0.01, "{dz}");
}
