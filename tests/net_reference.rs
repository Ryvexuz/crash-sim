//! Reference-mode output, independent single-car replay, and strict usage diagnostics.

mod run_tmp;

use std::process::{Command, Stdio};

const USAGE: &str = "usage: crash-sim --body <car.bsn> --track <file> --grid-gap <m> --duration <s> --input <file> --input2 <file> --net-delay <ticks> --net-hash-every <ticks>";

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
    assert!(
        bin.is_file(),
        "CARGO_BIN_EXE_crash-sim is not set and {} does not exist",
        bin.display()
    );
    bin
}

fn tmp_file(name: &str, text: &str) -> String {
    let dir = run_tmp::run_tmp();
    let path = dir.join(format!("net_reference_{}_{name}.input", std::process::id()));
    std::fs::write(&path, text).unwrap();
    path.to_string_lossy().into_owned()
}

struct Run {
    code: i32,
    stdout: String,
    stderr: String,
}

fn run(args: &[String]) -> Run {
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

fn reference_args() -> Vec<String> {
    [
        "--body".into(),
        manifest("scenarios/car_basic.bsn"),
        "--track".into(),
        manifest("scenarios/tracks/oval.track"),
        "--grid-gap".into(),
        "10".into(),
        "--duration".into(),
        "20".into(),
        "--input".into(),
        manifest("scenarios/tracks/oval_lap.input"),
        "--input2".into(),
        manifest("scenarios/tracks/oval_lap.input"),
        "--net-delay".into(),
        "6".into(),
        "--net-hash-every".into(),
        "100".into(),
    ]
    .into()
}

fn replace(args: &mut [String], flag: &str, value: &str) {
    let i = args.iter().position(|a| a == flag).expect("known flag");
    args[i + 1] = value.into();
}

fn success(args: &[String]) -> String {
    let r = run(args);
    assert_eq!(r.code, 0, "args={args:?}\n{}\n{}", r.stderr, r.stdout);
    assert!(r.stderr.is_empty(), "{}", r.stderr);
    assert!(!r.stdout.is_empty(), "empty reference output");
    r.stdout
}

fn car_starts(text: &str, car: u32) -> usize {
    let prefix = format!("car {car} start ");
    text.lines()
        .filter(|line| line.starts_with(prefix.as_str()))
        .count()
}

#[test]
fn reference_run_lines_and_order() {
    let text = success(&reference_args());
    let lines: Vec<_> = text.lines().collect();
    assert!(lines.len() >= 3, "{text}");
    let tail = lines.len() - 3;
    assert!(lines[tail].starts_with("car 1 final "), "{text}");
    assert!(lines[tail + 1].starts_with("car 2 final "), "{text}");
    assert!(lines[tail + 2].starts_with("cars "), "{text}");
    let mut hashes = Vec::new();
    let mut previous: Option<(f64, u32)> = None;
    for line in &lines[..tail] {
        assert!(
            !["net", "track", "race"]
                .iter()
                .any(|p| line.starts_with(*p)),
            "{line}"
        );
        let fields: Vec<_> = line.split(' ').collect();
        if line.starts_with("hash ") {
            assert_eq!(fields.len(), 3, "{line}");
            let tick = fields[1].strip_prefix("tick=").expect("hash tick");
            assert!(!tick.is_empty() && tick.bytes().all(|b| b.is_ascii_digit()));
            hashes.push(tick.parse::<u64>().unwrap());
            let value = fields[2].strip_prefix("value=").expect("hash value");
            assert_eq!(value.len(), 16, "{line}");
            assert!(
                value
                    .bytes()
                    .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b)),
                "{line}"
            );
            previous = None;
        } else {
            assert!(fields.len() >= 4, "{line}");
            assert_eq!(fields[0], "car", "{line}");
            assert!(matches!(fields[1], "1" | "2"), "{line}");
            let car = fields[1].parse::<u32>().unwrap();
            let key = match fields[2] {
                "start" | "checkpoint" => "t=",
                "lap" => "end_t=",
                _ => panic!("unexpected output: {line}"),
            };
            let t = fields
                .iter()
                .find_map(|f| f.strip_prefix(key))
                .expect("event time")
                .parse::<f64>()
                .unwrap();
            assert!(t.is_finite(), "{line}");
            if let Some((prev_t, prev_car)) = previous {
                assert!(
                    prev_t < t || (prev_t == t && prev_car <= car),
                    "out of order: previous=({prev_t}, {prev_car}), next={line}"
                );
            }
            previous = Some((t, car));
        }
    }
    assert!(previous.is_none(), "events after the last hash");
    assert_eq!(hashes, (1..=20).map(|i| i * 100 - 1).collect::<Vec<u64>>());
    assert!(car_starts(&text, 1) > 0, "{text}");
    assert!(car_starts(&text, 2) > 0, "{text}");
    eprintln!(
        "reference hash_lines={} last_tick={}",
        hashes.len(),
        hashes.last().unwrap()
    );
}

#[test]
fn reference_run_is_repeatable() {
    let args = reference_args();
    assert_eq!(success(&args).as_bytes(), success(&args).as_bytes());
}

#[test]
fn reference_car1_matches_single_car_run() {
    let still = tmp_file("single_still", "0 0 1 0\n");
    let mut args = reference_args();
    replace(&mut args, "--input2", &still);
    replace(&mut args, "--net-delay", "0");
    let reference = success(&args);
    let single_args = [
        "--body".into(),
        manifest("scenarios/car_basic.bsn"),
        "--track".into(),
        manifest("scenarios/tracks/oval.track"),
        "--grid-slot".into(),
        "1".into(),
        "--grid-gap".into(),
        "10".into(),
        "--input".into(),
        manifest("scenarios/tracks/oval_lap.input"),
        "--duration".into(),
        "20".into(),
    ];
    let single = success(&single_args);
    let is_event = |line: &&str| line.starts_with("start ") || line.starts_with("checkpoint ");
    let actual: Vec<_> = reference
        .lines()
        .filter_map(|l| l.strip_prefix("car 1 "))
        .filter(is_event)
        .collect();
    let expected: Vec<_> = single.lines().filter(is_event).collect();
    assert!(!expected.is_empty(), "single-car comparison is empty");
    for i in 0..actual.len().max(expected.len()) {
        assert_eq!(
            actual.get(i),
            expected.get(i),
            "first differing pair at index {i}"
        );
    }
    eprintln!("reference_car1 compared_lines={}", expected.len());
}

#[test]
fn reference_inputs_drive_their_own_cars() {
    let still = tmp_file("own_cars_still", "0 0 1 0\n");
    let mut args = reference_args();
    replace(&mut args, "--input2", &still);
    let one = success(&args);
    assert!(car_starts(&one, 1) > 0, "{one}");
    assert_eq!(car_starts(&one, 2), 0, "{one}");
    let both = success(&reference_args());
    assert!(car_starts(&both, 1) > 0, "{both}");
    assert!(car_starts(&both, 2) > 0, "{both}");
}

fn usage_error(args: &[String], needle: &str) {
    let r = run(args);
    assert_eq!(r.code, 2, "args={args:?}\n{}\n{}", r.stderr, r.stdout);
    assert!(r.stdout.is_empty(), "{}", r.stdout);
    assert!(r.stderr.starts_with("crash-sim: "), "{}", r.stderr);
    assert!(r.stderr.contains(USAGE), "{}", r.stderr);
    assert!(
        r.stderr.contains(needle),
        "expected {needle:?}: {}",
        r.stderr
    );
}

#[test]
fn reference_usage_errors() {
    let base = reference_args();
    for i in (0..base.len()).step_by(2) {
        let mut args = base.clone();
        args.drain(i..i + 2);
        usage_error(&args, &format!("{}: required flag is missing", base[i]));
    }
    for (flag, value, needle) in [
        ("--duration", "20.005", "whole number of hash intervals"),
        ("--net-hash-every", "300", "whole number of hash intervals"),
        ("--net-hash-every", "0", "--net-hash-every:"),
        ("--net-delay", "1001", "--net-delay:"),
        ("--net-delay", "-1", "--net-delay:"),
        ("--grid-gap", "0", "--grid-gap:"),
        ("--grid-gap", "-1", "--grid-gap:"),
        ("--grid-gap", "NaN", "--grid-gap:"),
        ("--grid-gap", "inf", "--grid-gap:"),
        ("--duration", "NaN", "--duration:"),
        ("--duration", "inf", "--duration:"),
        ("--duration", "0", "--duration:"),
        ("--duration", "5e-324", "--duration:"),
        ("--duration", "1.7976931348623157e308", "--duration:"),
        ("--duration", "184467440737095516.16", "--duration:"),
        ("--net-delay", "+1", "canonical decimal integer"),
        ("--net-delay", "01", "canonical decimal integer"),
        ("--net-hash-every", "01", "canonical decimal integer"),
        ("--net-hash-every", "18446744073709551616", "out of range"),
        ("--input", "", "--input:"),
        ("--input", "bad\rpath", "--input:"),
        ("--input2", "bad\u{2028}path", "--input2:"),
    ] {
        let mut args = base.clone();
        replace(&mut args, flag, value);
        usage_error(&args, needle);
    }
    for (flag, value) in [
        ("--host", "127.0.0.1:0"),
        ("--join", "127.0.0.1:0"),
        ("--ai", "3"),
        ("--substeps", "5"),
        ("--dt", "0.01"),
        ("--live", ""),
    ] {
        let mut args = base.clone();
        args.extend([flag.into(), value.into()]);
        usage_error(&args, &format!("{flag}: unknown or incompatible flag"));
    }
    let mut repeated = base.clone();
    repeated.extend(["--input".into(), base[9].clone()]);
    usage_error(&repeated, "--input: repeated flag");
    let mut missing_value = base.clone();
    missing_value.pop();
    usage_error(&missing_value, "--net-hash-every: needs a value");
    let mut huge_integer = base.clone();
    replace(&mut huge_integer, "--net-delay", &"9".repeat(4096));
    usage_error(&huge_integer, "--net-delay: integer is out of range");
    for (i, text) in [
        "0 0.1234 0 0\n",
        "0 0.0000000000001 0 0\n",
        "0 0 0 0\r\n",
        "0 NaN 0 0\n",
        "0 0 0 0\n0 1 0 0\n",
        "0 2 0 0\n",
        "",
    ]
    .iter()
    .enumerate()
    {
        let path = tmp_file(&format!("bad_{i}"), text);
        for flag in ["--input", "--input2"] {
            let mut args = base.clone();
            replace(&mut args, flag, &path);
            usage_error(&args, &path);
            if i < 2 {
                usage_error(&args, "more than 3 decimals");
            }
        }
    }
}
