use std::process::{Command, Output};

fn run(args: &[&str]) -> Output {
    Command::new(crash_sim_bin()).args(args).output().unwrap()
}

fn summary(args: &[&str]) -> String {
    let mut a = args.to_vec();
    a.push("--summary");
    let o = run(&a);
    assert!(o.status.success());
    String::from_utf8(o.stdout).unwrap()
}

/// ((t, min_y) of every CSV row, peak strain, final center_y) from the CSV of the same options.
fn from_csv(args: &[&str]) -> (Vec<(f64, f64)>, String, String) {
    let o = run(args);
    assert!(o.status.success());
    let text = String::from_utf8(o.stdout).unwrap();
    let rows: Vec<Vec<&str>> = text.lines().skip(1).map(|l| l.split(',').collect()).collect();
    let samples = rows.iter().map(|r| (r[0].parse::<f64>().unwrap(), r[1].parse::<f64>().unwrap())).collect();
    let peak = rows.iter().map(|r| r[3].parse::<f64>().unwrap()).fold(f64::MIN, f64::max);
    (samples, format!("{peak:.6}"), format!("{:.4}", rows.last().unwrap()[2].parse::<f64>().unwrap()))
}

fn parse(line: &str) -> (String, String, String) {
    assert!(line.ends_with('\n'));
    assert_eq!(line.matches('\n').count(), 1, "exactly one line: {line:?}");
    let f: Vec<&str> = line.trim_end().split(',').collect();
    assert_eq!(f.len(), 3);
    let v = |i: usize, k: &str| f[i].strip_prefix(k).unwrap_or_else(|| panic!("{} missing {k}", f[i])).to_string();
    (v(0, "first_contact_s="), v(1, "peak_strain="), v(2, "final_center_y="))
}

fn decimals(s: &str) -> usize {
    s.split('.').nth(1).unwrap().len()
}

#[test]
fn default_run_has_contact_and_exact_format() {
    let (c, p, y) = parse(&summary(&[]));
    assert_eq!(decimals(&c), 4);
    assert_eq!(decimals(&p), 6);
    assert_eq!(decimals(&y), 4);
    assert!(c.parse::<f64>().unwrap() > 0.0);
}

#[test]
fn short_run_never_touches_ground() {
    let (c, _, _) = parse(&summary(&["--duration", "0.1"]));
    assert_eq!(c, "none");
}

#[test]
fn values_match_the_csv_run() {
    for args in [vec![], vec!["--duration", "0.1"], vec!["--duration", "2", "--drop-height", "3", "--no-tilt"]] {
        let (c, p, y) = parse(&summary(&args));
        let (samples, cp, cy) = from_csv(&args);
        // The summary time comes from the substep loop. Every CSV sample strictly before it
        // must be above ground, and it must fall inside the output step that follows the last
        // such sample (dt = 0.01 in all these runs). So a contact time that is too early, too
        // late or invented fails. It can still be much earlier than the first CSV sample at
        // or below ground: a node can touch and bounce clear between samples (default run:
        // 0.639 s against 0.65 s; flat 3 m drop: 0.782 s against 1.26 s).
        let dt = 0.01;
        if c == "none" {
            assert!(samples.iter().all(|s| s.1 > 0.0), "{args:?}: summary says none but the CSV touches ground");
        } else {
            let c: f64 = c.parse().unwrap();
            let before: Vec<&(f64, f64)> = samples.iter().filter(|s| s.0 < c - 1e-9).collect();
            assert!(before.iter().all(|s| s.1 > 0.0), "{args:?}: CSV touches ground before summary time {c}");
            let last = before.last().expect("contact at t = 0 is not expected here").0;
            assert!(c >= last && c <= last + dt + 1e-9, "{args:?}: summary {c} not within one step after sample {last}");
            assert!(c <= samples.last().unwrap().0 + 1e-9, "{args:?}: summary {c} is after the last row");
        }
        assert_eq!(p, cp, "{args:?}");
        assert_eq!(y, cy, "{args:?}");
    }
}

#[test]
fn invalid_option_prints_nothing() {
    for args in [vec!["--summary", "--dt", "-1"], vec!["--summary", "--substeps", "0"], vec!["--summary", "--bogus"], vec!["--summary", "--dt"]] {
        let o = run(&args);
        assert!(!o.status.success(), "{args:?}");
        assert!(o.stdout.is_empty(), "{args:?}");
    }
}

#[test]
fn default_output_is_still_csv() {
    let o = run(&["--duration", "0.1"]);
    assert!(String::from_utf8(o.stdout).unwrap().starts_with("t,min_y,center_y,max_beam_strain\n"));
}

#[test]
fn contact_between_two_samples_is_reported() {
    // At 0.64 s the cube touches down inside the last output step and its strain is non-zero,
    // yet the last CSV sample is still above ground.
    let (c, p, _) = parse(&summary(&["--duration", "0.64"]));
    assert_ne!(c, "none");
    assert_eq!(decimals(&c), 4);
    assert!(c.parse::<f64>().unwrap() <= 0.64);
    assert!(p.parse::<f64>().unwrap() > 0.0);
}

#[test]
fn flat_drop_contact_time_matches_free_fall() {
    // sqrt(2 * 3 / 9.81) = 0.7820 s; symplectic Euler is late by about h/2 (0.5 ms).
    let (c, _, _) = parse(&summary(&["--duration", "2", "--drop-height", "3", "--no-tilt"]));
    let c: f64 = c.parse().unwrap();
    assert!((c - 0.7820).abs() < 0.002, "first contact {c}");
}

/// Path of the crash-sim binary, read at run time: the copy a broadsword test step points
/// `CARGO_BIN_EXE_crash-sim` at, else the cargo layout next to the test executable.
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
