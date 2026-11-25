//! Check 2.1's ai_race tests; the CLI slice appends the other six.

use crash_sim::ai::DriverParams;
use crash_sim::race::{self, ProfileParams, RaceEnd, RaceSetup};
use crash_sim::track::Track;
use crash_sim::{Body, Config};
mod run_tmp;

fn body() -> Body {
    crash_sim::format::parse(include_str!("../scenarios/car_basic.bsn")).unwrap()
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

#[test]
fn oval_tight_differs_only_in_width() {
    let oval_text = include_str!("../scenarios/tracks/oval.track");
    let tight_text = include_str!("../scenarios/tracks/oval_tight.track");
    assert_eq!(oval_text.matches("\nwidth ").count(), 1);
    assert_eq!(oval_text.matches("\nwidth 12\n").count(), 1);
    assert_eq!(
        tight_text,
        oval_text.replacen("\nwidth 12\n", "\nwidth 1.6\n", 1)
    );
    assert!(!tight_text.contains('\r'));
    assert!(tight_text.ends_with('\n'));
    let oval = Track::parse(oval_text).unwrap();
    let tight = Track::parse(tight_text).unwrap();
    assert_eq!(tight.points, oval.points);
    assert_eq!(tight.start, oval.start);
    assert_eq!(tight.checkpoints, oval.checkpoints);
    assert_eq!(tight.grid, oval.grid);
    assert!(!tight.points.is_empty());
    assert_eq!(tight.widths.len(), tight.points.len());
    assert_eq!(oval.widths.len(), oval.points.len());
    for width in &tight.widths {
        assert_eq!(width.to_bits(), 1.6_f64.to_bits());
    }
    for width in &oval.widths {
        assert_eq!(width.to_bits(), 12.0_f64.to_bits());
    }
    let body = body();
    assert!(!body.nodes.is_empty());
    let min_z = body
        .nodes
        .iter()
        .map(|n| n.pos.z)
        .fold(f64::INFINITY, f64::min);
    let max_z = body
        .nodes
        .iter()
        .map(|n| n.pos.z)
        .fold(f64::NEG_INFINITY, f64::max);
    let span = max_z - min_z;
    assert!((span - 1.8).abs() <= 1e-12, "node z span: {span}");
    assert!(1.6 < span, "tight width must be below node z span {span}");
}

#[test]
fn wall_rule_counts_tyre_outer_edges() {
    let oval_text = include_str!("../scenarios/tracks/oval.track");
    assert_eq!(oval_text.matches("\nwidth 12\n").count(), 1);
    let track = Track::parse(&oval_text.replace("\nwidth 12\n", "\nwidth 2\n")).unwrap();
    let cfg = Config::default();
    let original = body();
    let mut wide = original.clone();
    assert!(!wide.wheels.is_empty());
    for wheel in &mut wide.wheels {
        wheel.width = 0.6;
    }
    let mut s = setup(&cfg, &wide, &track);
    s.time_limit_s = 0.02;
    let result = race::run(&s).unwrap();
    assert!(matches!(result.end, RaceEnd::TimeLimit), "{:?}", result.end);
    assert_eq!(result.substeps, 20);
    assert_eq!(result.cars.len(), 1);
    let centre = race::car_position(result.multi.sim(0));
    let nodes = result.multi.chassis_positions(0);
    assert!(!nodes.is_empty());
    assert!(
        !race::wall_contact(&track, centre, 0.0, nodes, &[], &[]),
        "chassis outside: centre={centre:?}, nodes={nodes:?}"
    );
    assert_eq!(result.cars[0].wall_substeps, 20);
    assert_eq!(
        result.cars[0].first_wall_t.unwrap().to_bits(),
        result.h.to_bits()
    );

    let mut s = setup(&cfg, &original, &track);
    s.time_limit_s = 0.02;
    let control = race::run(&s).unwrap();
    assert!(
        matches!(control.end, RaceEnd::TimeLimit),
        "{:?}",
        control.end
    );
    assert_eq!(control.substeps, 20);
    assert_eq!(control.cars.len(), 1);
    assert_eq!(
        control.cars[0].wall_substeps,
        0,
        "control centre={:?}, nodes={:?}",
        race::car_position(control.multi.sim(0)),
        control.multi.chassis_positions(0)
    );
    assert_eq!(control.cars[0].first_wall_t, None);
}

use std::process::{Command, Stdio};

// Set to the calibrated scenario path when its params/evidence follow-up lands.
const PARAMS: Option<&str> = None;

fn exe() -> std::path::PathBuf {
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
    assert!(bin.is_file(), "missing binary {}", bin.display());
    bin
}

fn temp_file(name: &str, text: &str) -> String {
    let dir = run_tmp::run_tmp();
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
    let out = Command::new(exe())
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

fn manifest(path: &str) -> String {
    format!("{}/{path}", env!("CARGO_MANIFEST_DIR"))
}

fn params_text() -> String {
    let (cfg, car, track) = (
        Config::default(),
        body(),
        Track::parse(include_str!("../scenarios/tracks/oval.track")).unwrap(),
    );
    let s = setup(&cfg, &car, &track);
    let d = s.driver;
    let p = s.profile;
    let values = [
        d.lookahead_k,
        d.lookahead_min,
        d.lookahead_max,
        d.kp,
        d.ki,
        d.integral_max,
        p.mu,
        p.g,
        p.a_accel,
        p.a_brake,
        p.v_max,
    ];
    let mut text = String::from("ai 1\n");
    for (key, value) in crash_sim::ai_params::KEYS.iter().zip(values) {
        text.push_str(&format!("{key} {value}\n"));
    }
    text
}

fn flags(name: &str, cars: &str, seconds: &str) -> Vec<String> {
    let params = match PARAMS {
        Some(path) => manifest(path),
        None => temp_file(&format!("{name}.params"), &params_text()),
    };
    [
        "--body".into(),
        manifest("scenarios/car_basic.bsn"),
        "--track".into(),
        manifest("scenarios/tracks/oval.track"),
        "--ai".into(),
        cars.into(),
        "--laps".into(),
        "1".into(),
        "--time-limit".into(),
        seconds.into(),
        "--grid-gap".into(),
        "10".into(),
        "--ai-params".into(),
        params,
    ]
    .into()
}

fn cli(args: &[String]) -> Run {
    run(&args.iter().map(String::as_str).collect::<Vec<_>>())
}

fn set(args: &mut [String], flag: &str, value: &str) {
    let i = args.iter().position(|a| a == flag).unwrap();
    args[i + 1] = value.into();
}

fn field<'a>(line: &'a str, key: &str) -> &'a str {
    let prefix = format!("{key}=");
    line.split_whitespace()
        .find_map(|f| f.strip_prefix(prefix.as_str()))
        .unwrap_or_else(|| panic!("missing {key} in {line}"))
}

fn number(line: &str, key: &str) -> f64 {
    let n: f64 = field(line, key).parse().unwrap();
    assert!(n.is_finite(), "{line}");
    n
}

fn only_line<'a>(text: &'a str, prefix: &str) -> &'a str {
    let lines: Vec<_> = text.lines().filter(|l| l.starts_with(prefix)).collect();
    assert_eq!(lines.len(), 1, "{prefix}: {text}");
    lines[0]
}

fn refused(r: &Run, needle: &str) {
    assert_eq!(r.code, 2, "{}", r.stderr);
    assert!(r.stdout.is_empty(), "{}", r.stdout);
    assert!(r.stderr.contains(needle), "{needle}: {}", r.stderr);
}

#[test]
fn tight_track_reports_wall_contacts() {
    let mut args = flags("ai_tight", "1", "160");
    let control = cli(&args);
    assert_eq!(control.code, 0, "{}{}", control.stdout, control.stderr);
    assert_eq!(
        field(only_line(&control.stdout, "car 1 walls "), "substeps"),
        "0"
    );
    let verdict = control.stdout.lines().last().unwrap();
    assert!(verdict.starts_with("race: ok "));
    set(
        &mut args,
        "--track",
        &manifest("scenarios/tracks/oval_tight.track"),
    );
    let prefix = temp_file("ai_tight_script", "");
    args.extend(["--save-inputs".into(), prefix.clone()]);
    let r = cli(&args);
    assert_eq!(r.code, 1, "{}{}", r.stdout, r.stderr);
    let walls = only_line(&r.stdout, "car 1 walls ");
    let contacts: u64 = field(walls, "substeps").parse().unwrap();
    assert!(contacts >= 1, "{walls}");
    number(walls, "first_t");
    let verdict = r.stdout.lines().last().unwrap();
    assert!(verdict.starts_with("race: FAIL "), "{verdict}");
    assert!(verdict.contains("wall_substeps="), "{verdict}");
    assert!(!verdict.contains("unstable"), "{verdict}");
    assert_eq!(
        only_line(&r.stdout, "cars "),
        "cars gap_min_m=none gap_min_t=none near_substeps=0"
    );
    assert!(!std::fs::read(format!("{prefix}1.input")).unwrap().is_empty());
}

#[test]
fn near_rule_counts_centres_under_4_4_m() {
    let mut args = flags("ai_near", "2", "6");
    for (gap, near) in [("4.3", true), ("10", false)] {
        set(&mut args, "--grid-gap", gap);
        let r = cli(&args);
        assert_eq!(r.code, 1, "{}{}", r.stdout, r.stderr);
        let line = only_line(&r.stdout, "cars ");
        let minimum = number(line, "gap_min_m");
        let count: u64 = field(line, "near_substeps").parse().unwrap();
        let verdict = r.stdout.lines().last().unwrap();
        eprintln!("near control_gap={gap} gap_min_m={minimum} near_substeps={count}");
        assert!(verdict.starts_with("race: FAIL finished=0/2"), "{verdict}");
        assert!(!verdict.contains("wall_substeps="), "{verdict}");
        if near {
            // Check 2.1 defines the 4.30 m upper bound and the 4.40 m floor.
            assert!(minimum <= 4.30, "{line}");
            assert!(count >= 1, "{line}");
            assert_eq!(field(verdict, "near_substeps"), count.to_string());
        } else {
            assert!(minimum >= 4.40, "{line}");
            assert_eq!(count, 0, "{line}");
            assert!(!verdict.contains("near_substeps="), "{verdict}");
        }
        for slot in 1..=2 {
            assert_eq!(
                only_line(&r.stdout, &format!("car {slot} finish ")),
                format!("car {slot} finish t=none place=none")
            );
        }
    }
}

#[test]
fn ai_race_refuses_bad_flags() {
    let base = flags("ai_refusals", "2", "0.02");
    for (flag, bad) in [
        ("--ai", "0"),
        ("--ai", "4"),
        ("--ai", "-1"),
        ("--ai", "1.5"),
        ("--ai", "999999999999999999999999"),
        ("--laps", "0"),
        ("--laps", "11"),
        ("--time-limit", "0"),
        ("--time-limit", "-1"),
        ("--time-limit", "NaN"),
        ("--time-limit", "inf"),
        ("--time-limit", "1e309"),
        ("--grid-gap", "0"),
        ("--grid-gap", "-1"),
        ("--grid-gap", "NaN"),
        ("--grid-gap", "inf"),
        ("--grid-gap", ""),
        ("--grid-gap", "1\r"),
        ("--grid-gap", "1\n"),
        ("--grid-gap", "1\u{2028}"),
    ] {
        let mut args = base.clone();
        set(&mut args, flag, bad);
        let r = cli(&args);
        refused(&r, flag);
        assert!(r.stderr.contains("usage: crash-sim"), "{}", r.stderr);
    }
    for flag in [
        "--body",
        "--track",
        "--laps",
        "--time-limit",
        "--grid-gap",
        "--ai-params",
    ] {
        let mut args = base.clone();
        let i = args.iter().position(|a| a == flag).unwrap();
        args.drain(i..i + 2);
        refused(&cli(&args), flag);
    }
    for flag in [
        "--input",
        "--live",
        "--duration",
        "--dt",
        "--gravity",
        "--grid-slot",
        "--summary",
        "--events",
        "--drive-report",
        "--trajectory-v2",
        "--unknown",
    ] {
        let mut args = base.clone();
        args.extend([flag.into(), "x".into()]);
        refused(&cli(&args), flag);
    }
    for flag in [
        "--body",
        "--track",
        "--ai",
        "--laps",
        "--time-limit",
        "--grid-gap",
        "--ai-params",
        "--substeps",
        "--save-inputs",
    ] {
        // Use a complete baseline with this flag last and no value.
        let mut missing = base.clone();
        if let Some(i) = missing.iter().position(|a| a == flag) {
            missing.drain(i..i + 2);
        }
        missing.push(flag.into());
        refused(&cli(&missing), flag);
        let mut repeated = base.clone();
        if !repeated.iter().any(|a| a == flag) {
            repeated.extend([flag.into(), "1".into()]);
        }
        repeated.extend([flag.into(), "1".into()]);
        refused(&cli(&repeated), "repeated");
    }
    for value in ["0", "-1", "1.5", "999999999999999999999999"] {
        let mut args = base.clone();
        args.extend(["--substeps".into(), value.into()]);
        refused(&cli(&args), "--substeps");
    }
    let missing = format!("{}.missing", temp_file("ai_missing", ""));
    for flag in ["--body", "--track", "--ai-params"] {
        let mut args = base.clone();
        set(&mut args, flag, &missing);
        refused(&cli(&args), &missing);
    }
    for (name, text, needle) in [
        ("syntax", params_text().replace("kp 0.8", "kp bad"), "line 5"),
        (
            "driver",
            params_text().replace("lookahead_min 3", "lookahead_min 0"),
            "line 3",
        ),
        ("profile", params_text().replace("mu 0.6", "mu 0"), "line 8"),
        ("duplicate", format!("{}kp 0.8\n", params_text()), "line 13"),
        ("cr", params_text().replace("kp 0.8\n", "kp 0.8\r\n"), "line 5"),
        (
            "huge",
            params_text().replace("kp 0.8", &format!("kp {}", "9".repeat(100_000))),
            "line 5",
        ),
        (
            "subnormal",
            params_text().replace("v_max 15", "v_max 5e-324"),
            "profile:",
        ),
    ] {
        let path = temp_file(&format!("ai_refuse_{name}.params"), &text);
        let mut args = base.clone();
        set(&mut args, "--ai-params", &path);
        refused(&cli(&args), needle);
    }
    // A write failure after car 1 must still leave stdout empty and name car 2's path.
    let prefix = temp_file("ai_write_failure", "");
    let blocked = format!("{prefix}2.input");
    std::fs::create_dir_all(&blocked).unwrap();
    let mut args = base;
    args.extend(["--save-inputs".into(), prefix.clone()]);
    refused(&cli(&args), &format!("--save-inputs {blocked}"));
    assert!(std::path::Path::new(&blocked).is_dir());
    let first = std::fs::read_to_string(format!("{prefix}1.input")).unwrap();
    crash_sim::input::Script::parse(&first).unwrap();
}

fn three_car_run(name: &str, prefix: &str) -> Run {
    let mut args = flags(name, "3", "70");
    args.extend(["--save-inputs".into(), prefix.into()]);
    let r = cli(&args);
    assert!(r.code == 0 || r.code == 1, "{}{}", r.stdout, r.stderr);
    assert!(r.stderr.is_empty(), "{}", r.stderr);
    assert!(!r.stdout.contains("unstable"), "{}", r.stdout);
    r
}

fn check_scripts(prefix: &str, stdout: &str) {
    let end = number(stdout.lines().last().unwrap(), "end_t");
    for slot in 1..=3 {
        let text = std::fs::read_to_string(format!("{prefix}{slot}.input")).unwrap();
        let lines: Vec<_> = text.lines().collect();
        assert!(lines.len() >= 2, "{text}");
        assert_eq!(lines[0].split_whitespace().next(), Some("0.000"));
        for (j, line) in lines.iter().enumerate() {
            let f: Vec<_> = line.split_whitespace().collect();
            assert_eq!(f.len(), 4, "{line}");
            let ms = j * 20;
            assert_eq!(f[0], format!("{}.{:03}", ms / 1000, ms % 1000));
            let values: Vec<f64> = f.iter().map(|v| v.parse().unwrap()).collect();
            assert!(values.iter().all(|v| v.is_finite()), "{line}");
            assert!((0.0..=1.0).contains(&values[1]), "{line}");
            assert!((0.0..=1.0).contains(&values[2]), "{line}");
            assert!(values[1] == 0.0 || values[2] == 0.0, "{line}");
            assert!((-1.0..=1.0).contains(&values[3]), "{line}");
        }
        let script = crash_sim::input::Script::parse(&text).unwrap();
        assert_eq!(script.lines().last().unwrap().t, end);
    }
}

#[test]
fn ai_race_same_input_same_bytes() {
    let a = temp_file("ai_same_a", "");
    let b = temp_file("ai_same_b", "");
    let first = three_car_run("ai_same", &a);
    let second = three_car_run("ai_same", &b);
    assert_eq!(first.code, second.code);
    assert_eq!(first.stdout.as_bytes(), second.stdout.as_bytes());
    for slot in 1..=3 {
        assert_eq!(
            std::fs::read(format!("{a}{slot}.input")).unwrap(),
            std::fs::read(format!("{b}{slot}.input")).unwrap()
        );
    }
    let lines: Vec<_> = first.stdout.lines().collect();
    assert_eq!(lines.len(), 28, "{}", first.stdout);
    assert!(lines[0].starts_with(
        "race track=oval cars=3 laps=1 time_limit_s=70.000 grid_gap_m=10.00 ai_params="
    ));
    assert!(lines[1].starts_with("profile lap_s="));
    for slot in 1..=3 {
        let offset = 2 + (slot - 1) * 8;
        for (i, kind) in [
            "start ",
            "checkpoint lap=1 k=1 ",
            "checkpoint lap=1 k=2 ",
            "checkpoint lap=1 k=3 ",
            "lap n=1 ",
            "finish t=",
            "walls substeps=",
            "final ",
        ]
        .iter()
        .enumerate()
        {
            assert!(
                lines[offset + i].starts_with(&format!("car {slot} {kind}")),
                "{}",
                first.stdout
            );
        }
        assert_ne!(field(lines[offset + 5], "t"), "none");
        assert_ne!(field(lines[offset + 5], "place"), "none");
    }
    assert!(lines[26].starts_with("cars gap_min_m="));
    assert!(lines[27].starts_with("race: "));
    assert_eq!(first.code == 0, lines[27].starts_with("race: ok "));
    check_scripts(&a, &first.stdout);
    eprintln!("same_bytes {} end_t={}", lines[1], field(lines[27], "end_t"));
}

#[test]
fn ai_inputs_replay_one_car_alone() {
    let prefix = temp_file("ai_replay_a", "");
    let race = three_car_run("ai_replay", &prefix);
    check_scripts(&prefix, &race.stdout);
    let replay = run(&[
        "--body",
        &manifest("scenarios/car_basic.bsn"),
        "--track",
        &manifest("scenarios/tracks/oval.track"),
        "--input",
        &format!("{prefix}2.input"),
        "--grid-slot",
        "2",
        "--grid-gap",
        "10",
    ]);
    assert_eq!(replay.code, 0, "{}{}", replay.stdout, replay.stderr);
    let relevant = |line: &&str| {
        ["start ", "checkpoint ", "lap ", "final "]
            .iter()
            .any(|p| line.starts_with(*p))
    };
    let expected: Vec<_> = race
        .stdout
        .lines()
        .filter_map(|l| l.strip_prefix("car 2 "))
        .filter(relevant)
        .collect();
    let actual: Vec<_> = replay.stdout.lines().filter(relevant).collect();
    assert!(expected.len() >= 6, "{}", race.stdout);
    assert_eq!(actual, expected);
    eprintln!(
        "replay matched_lines={} end_t={}",
        actual.len(),
        field(race.stdout.lines().last().unwrap(), "end_t")
    );
}
