use crash_sim::ai::DriverParams;
use crash_sim::race::{ProfileParams, RaceSetup};
use crash_sim::track::Track;
use crash_sim::{Body, Config};
use std::process::{Command, Stdio};
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

fn cli(args: &[String]) -> Run {
    run(&args.iter().map(String::as_str).collect::<Vec<_>>())
}

fn folder(name: &str) -> std::path::PathBuf {
    let root = run_tmp::run_tmp();
    std::fs::create_dir_all(&root).unwrap();
    let root = root.canonicalize().unwrap();
    let dir = root.join(format!("save_guard_{name}"));
    assert_eq!(dir.parent(), Some(root.as_path()));
    if dir.exists() {
        std::fs::remove_dir_all(&dir).unwrap();
    }
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn args(params: &str, prefix: &str, cars: &str, seconds: &str) -> Vec<String> {
    vec![
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
        params.into(),
        "--save-inputs".into(),
        prefix.into(),
    ]
}

fn refused(args: &[String], flag: &str, input: &std::path::Path, before: &[u8]) {
    let r = cli(args);
    assert_eq!(r.code, 2, "{}{}", r.stdout, r.stderr);
    assert!(r.stdout.is_empty(), "{}", r.stdout);
    assert!(r.stderr.contains("--save-inputs "), "{}", r.stderr);
    assert!(
        r.stderr.contains(&format!("is the {flag} file")),
        "{}",
        r.stderr
    );
    assert_eq!(std::fs::read(input).unwrap(), before);
}

#[test]
fn save_inputs_refuses_to_overwrite_ai_params() {
    let dir = folder("save_inputs_refuses_to_overwrite_ai_params");
    let input = dir.join("run1.input");
    let text = params_text();
    std::fs::write(&input, &text).unwrap();
    refused(
        &args(
            input.to_str().unwrap(),
            dir.join("run").to_str().unwrap(),
            "1",
            "70",
        ),
        "--ai-params",
        &input,
        text.as_bytes(),
    );
}

#[test]
fn save_inputs_refuses_every_input_flag_and_spelling() {
    let dir = folder("save_inputs_refuses_every_input_flag_and_spelling");
    let params = dir.join("race.params");
    std::fs::write(&params, params_text()).unwrap();
    let mut cases = vec![
        (
            "--track",
            "t2.input",
            "t",
            "2",
            include_str!("../scenarios/tracks/oval.track").to_owned(),
        ),
        (
            "--body",
            "b1.input",
            "b",
            "1",
            include_str!("../scenarios/car_basic.bsn").to_owned(),
        ),
        ("--ai-params", "run1.input", "./run", "1", params_text()),
    ];
    if cfg!(windows) {
        cases.push(("--ai-params", "run1.input", "RUN", "1", params_text()));
    }
    for (flag, file, prefix, cars, text) in cases {
        let input = dir.join(file);
        std::fs::write(&input, &text).unwrap();
        let mut argv = args(
            params.to_str().unwrap(),
            dir.join(prefix).to_str().unwrap(),
            cars,
            "70",
        );
        let i = argv.iter().position(|a| a == flag).unwrap();
        argv[i + 1] = input.to_str().unwrap().into();
        refused(&argv, flag, &input, text.as_bytes());
        assert!(!dir.join("t1.input").exists());
    }
}

#[test]
fn save_inputs_still_writes_fresh_files() {
    let dir = folder("save_inputs_still_writes_fresh_files");
    let params = dir.join("race.params");
    let text = params_text();
    std::fs::write(&params, &text).unwrap();
    let argv = args(
        params.to_str().unwrap(),
        dir.join("fresh").to_str().unwrap(),
        "1",
        "1",
    );
    for _ in 0..2 {
        let r = cli(&argv);
        assert!(matches!(r.code, 0 | 1), "{}{}", r.stdout, r.stderr);
        assert!(r.stderr.is_empty(), "{}", r.stderr);
        let saved = std::fs::read_to_string(dir.join("fresh1.input")).unwrap();
        crash_sim::input::Script::parse(&saved).unwrap();
        assert_eq!(std::fs::read(&params).unwrap(), text.as_bytes());
    }
}
