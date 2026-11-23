//! n20-09: `--pulse-report` (docs/STAGE6-M4-M5.md, "The pulse report") and the gravity rule it needs.
//!
//! Every test runs the real binary on a tiny crash: a 12-node box of `scenarios/box_wall.bsn`'s
//! shape, lifted off the ground, flying at 10 m/s into a wall 0.1 m ahead, with no gravity. The
//! reference pulses are synthetic half-sine pulses written by the test (no real test data, and no
//! file of validation/ is named).

use crash_sim::Config;
use std::path::PathBuf;
use std::process::{Command, Output};

/// Path of the crash-sim binary, read at run time: the copy a broadsword test step points
/// `CARGO_BIN_EXE_crash-sim` at, else the cargo layout next to the test executable.
fn crash_sim_bin() -> PathBuf {
    if let Some(p) = std::env::var_os("CARGO_BIN_EXE_crash-sim") {
        return PathBuf::from(p);
    }
    let exe = std::env::current_exe().expect("current_exe of the test executable");
    let name = format!("crash-sim{}", std::env::consts::EXE_SUFFIX);
    let dir = exe
        .parent()
        .and_then(|p| p.parent())
        .expect("the test executable has a grandparent folder");
    let bin = dir.join(name);
    assert!(bin.is_file(), "no crash-sim binary at {}", bin.display());
    bin
}

/// A folder of this test's own files under the target folder.
fn dir(name: &str) -> PathBuf {
    let d = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join(format!("pulse_report_{name}"));
    std::fs::create_dir_all(&d).unwrap();
    d
}

/// The box body: 12 nodes of mass 1 kg, 50 beams, start velocity `v` along x, `rear` = back
/// face, `front` = front face (x = 2), `sensor` = back face, `all`; a wall plane at x = `wall_x`
/// (normal -x); the lowest node at height `y0`.
fn box_bsn(v: f64, wall_x: f64, y0: f64) -> String {
    let mut s =
        String::from("bsn 1\nmaterial box_beam stiffness 20000 damping 20 yield_strain 0.02\n");
    let mut id = 0;
    for x in 0..3 {
        for y in 0..2 {
            for z in 0..2 {
                s.push_str(&format!("node {id} {x} {} {z} mass 1\n", f64::from(y) + y0));
                id += 1;
            }
        }
    }
    // Every node pair of the same or the next x layer, as box_wall.bsn does (layers 0-1 and 1-2).
    for a in 0..12 {
        for b in (a + 1)..12 {
            if (a / 4 == b / 4) || (b / 4 == a / 4 + 1) {
                s.push_str(&format!("beam {a} {b} box_beam\n"));
            }
        }
    }
    s.push_str("group rear 0 1 2 3\ngroup sensor 0 1 2 3\ngroup front 8 9 10 11\n");
    s.push_str("group all 0 1 2 3 4 5 6 7 8 9 10 11\n");
    s.push_str(&format!(
        "velocity all {v} 0 0\nplane {wall_x} 0.5 0.5 -1 0 0\n"
    ));
    s
}

/// A synthetic reference pulse: from -0.02 s to 0.2 s every 0.1 ms, zero except for a half sine
/// of `peak` m/s^2 (negative) over the first 30 ms. Returns the `.pulse.csv` text.
fn ref_csv(peak: f64) -> String {
    ref_csv_from(peak, -0.02)
}

/// `ref_csv` with the record starting at `t0` s instead of -0.02 s (still 2201 samples).
fn ref_csv_from(peak: f64, t0: f64) -> String {
    let mut s = String::from("t_s,accel_mps2\n");
    for i in 0..2201 {
        let t = t0 + f64::from(i) * 1e-4;
        let a = if (0.0..=0.03).contains(&t) {
            -peak * (std::f64::consts::PI * t / 0.03).sin()
        } else {
            0.0
        };
        s.push_str(&format!("{t:.6},{a:.6}\n"));
    }
    s
}

fn meta_text(mass: f64, speed: f64, static_crush: Option<f64>) -> String {
    let mut s = format!(
        "test = synthetic-1\nmass_kg = {mass}\nspeed_kmh = {speed}\nchannel = synthetic x\nsample_rate_hz = 10000\n"
    );
    if let Some(c) = static_crush {
        s.push_str(&format!("static_crush_m = {c}\n"));
    }
    s
}

/// A set of files for one run.
struct Case {
    dir: PathBuf,
    body: PathBuf,
    refp: PathBuf,
}

/// Write `body`, a reference pulse text and its meta text into a folder of this test's own.
fn case(name: &str, body: &str, refcsv: &str, meta: &str) -> Case {
    let d = dir(name);
    let c = Case {
        body: d.join("body.bsn"),
        refp: d.join("ref.pulse.csv"),
        dir: d,
    };
    std::fs::write(&c.body, body).unwrap();
    std::fs::write(&c.refp, refcsv).unwrap();
    std::fs::write(c.dir.join("ref.meta"), meta).unwrap();
    c
}

fn default_case(name: &str) -> Case {
    case(
        name,
        &box_bsn(10.0, 2.1, 1.0),
        &ref_csv(400.0),
        &meta_text(12.0, 36.0, Some(0.05)),
    )
}

/// The report's own flags, in a fixed order; `(flag, value)`.
fn report_flags(c: &Case) -> Vec<(String, String)> {
    let f = |a: &str, b: &str| (a.to_string(), b.to_string());
    vec![
        f("--body", c.body.to_str().unwrap()),
        f("--pulse-report", c.refp.to_str().unwrap()),
        f("--gravity", "0"),
        f("--duration", "0.15"),
        f("--dt", "0.001"),
        f("--substeps", "10"),
        f("--pulse-sensor", "sensor"),
        f("--pulse-crush-groups", "rear,front"),
        f("--pulse-cfc-shape", "60"),
        f("--pulse-cfc-int", "180"),
        f("--pulse-window", "0.1"),
        f("--pulse-threshold", "20"),
        f("--pulse-crush-tail", "0.05"),
        f("--setup-mass-tol", "0.01"),
        f("--setup-speed-tol", "0.005"),
        f("--margin-dv", "0.05"),
        f("--margin-peak", "0.15"),
        f("--margin-peak-time", "0.010"),
        f("--margin-duration", "0.10"),
        f("--margin-olc", "0.15"),
        f("--margin-static-crush", "0.20"),
    ]
}

fn run_flags(flags: &[(String, String)], extra: &[&str]) -> Output {
    let mut cmd = Command::new(crash_sim_bin());
    for (k, v) in flags {
        cmd.arg(k).arg(v);
    }
    cmd.args(extra);
    cmd.output().unwrap()
}

/// Replace the value of `flag` in `flags`.
fn with(flags: &[(String, String)], flag: &str, value: &str) -> Vec<(String, String)> {
    let mut out = flags.to_vec();
    let slot = out
        .iter_mut()
        .find(|(k, _)| k == flag)
        .expect("flag is in the list");
    slot.1 = value.to_string();
    out
}

fn stdout(o: &Output) -> String {
    String::from_utf8(o.stdout.clone()).unwrap()
}

fn stderr(o: &Output) -> String {
    String::from_utf8_lossy(&o.stderr).into_owned()
}

/// Assert `token` is `key=<number with exactly `dec` decimals>` and return the number's text.
/// `signed` means a `+` or `-` sign is required.
fn field<'a>(token: &'a str, key: &str, dec: usize, signed: bool) -> &'a str {
    let v = token
        .strip_prefix(&format!("{key}="))
        .unwrap_or_else(|| panic!("token `{token}` does not start with `{key}=`"));
    let digits = v.strip_prefix(['+', '-']).unwrap_or(v);
    if signed {
        assert!(
            v.starts_with(['+', '-']),
            "`{token}`: a signed number is required"
        );
    }
    let (int, frac) = digits
        .split_once('.')
        .unwrap_or_else(|| panic!("`{token}`: no decimal point"));
    assert!(
        !int.is_empty() && int.chars().all(|c| c.is_ascii_digit()),
        "`{token}`: integer part"
    );
    assert!(
        frac.len() == dec && frac.chars().all(|c| c.is_ascii_digit()),
        "`{token}`: exactly {dec} decimals required"
    );
    v
}

/// Check the `metric <name> sim= ref= err= norm= ok|FAIL` line format and return (norm, ok).
fn check_metric_line(line: &str, name: &str, dec: usize) -> (f64, bool) {
    let t: Vec<&str> = line.split(' ').collect();
    assert_eq!(t.len(), 7, "metric line: {line}");
    assert_eq!(&t[..2], &["metric", name], "{line}");
    field(t[2], "sim", dec, false);
    field(t[3], "ref", dec, false);
    field(t[4], "err", 4, true);
    let norm: f64 = field(t[5], "norm", 4, false)[0..].parse().unwrap();
    assert!(t[6] == "ok" || t[6] == "FAIL", "{line}");
    assert_eq!(
        t[6] == "ok",
        norm <= 1.0,
        "ok must follow norm <= 1: {line}"
    );
    (norm, t[6] == "ok")
}

#[test]
fn pulse_report_lines_and_format() {
    let c = default_case("format");
    let o = run_flags(&report_flags(&c), &[]);
    let out = stdout(&o);
    let lines: Vec<&str> = out.lines().collect();
    assert!(
        out.ends_with('\n') && !out.contains('\r'),
        "LF line ends only"
    );
    assert_eq!(
        lines.len(),
        16,
        "one line per item of the docs' output list:\n{out}"
    );
    assert_eq!(lines[0], "pulse_ref=synthetic-1");
    assert_eq!(lines[1], "pulse_channel=synthetic x");
    // setup lines
    let t: Vec<&str> = lines[2].split(' ').collect();
    assert_eq!(t.len(), 7, "{}", lines[2]);
    assert_eq!(&t[..2], &["setup", "mass_kg"]);
    assert_eq!(field(t[2], "sim", 1, false), "12.0");
    assert_eq!(field(t[3], "ref", 1, false), "12.0");
    field(t[4], "err", 4, true);
    assert_eq!(field(t[5], "tol", 4, false), "0.0100");
    assert_eq!(t[6], "ok");
    let t: Vec<&str> = lines[3].split(' ').collect();
    assert_eq!(t.len(), 7, "{}", lines[3]);
    assert_eq!(&t[..2], &["setup", "speed_kmh"]);
    assert_eq!(field(t[2], "sim", 2, false), "36.00");
    assert_eq!(field(t[3], "ref", 2, false), "36.00");
    field(t[4], "err", 4, true);
    assert_eq!(field(t[5], "tol", 4, false), "0.0050");
    assert_eq!(t[6], "ok");
    // contact: 0.1 m at 10 m/s is 10 ms, to within one substep (0.1 ms)
    let contact: f64 = field(lines[4], "contact_s", 4, false)[0..].parse().unwrap();
    assert!((0.0099..=0.0101).contains(&contact), "contact_s {contact}");
    assert_eq!(lines[5], "ground_contacts_in_window=0");
    // metrics, in the printed order, with the docs' decimals
    let mut norms = Vec::new();
    let mut all_ok = true;
    for (i, (name, dec)) in [
        ("dv_mps", 4),
        ("peak_mps2", 1),
        ("peak_time_s", 4),
        ("duration_s", 4),
        ("olc_mps2", 1),
        ("static_crush_m", 4),
    ]
    .iter()
    .enumerate()
    {
        let (norm, ok) = check_metric_line(lines[6 + i], name, *dec);
        norms.push(norm);
        all_ok &= ok;
    }
    // The old static crush rule (centroid distance), printed for comparison, not gated.
    let t: Vec<&str> = lines[12].split(' ').collect();
    assert_eq!(
        &t[..2],
        &["info", "static_crush_centroid_m"],
        "{}",
        lines[12]
    );
    assert_eq!(t.len(), 3, "{}", lines[12]);
    field(t[2], "sim", 4, false);
    let t: Vec<&str> = lines[13].split(' ').collect();
    assert_eq!(&t[..2], &["info", "dyn_crush_m"], "{}", lines[13]);
    assert_eq!(t.len(), 4);
    field(t[2], "sim", 4, false);
    field(t[3], "ref", 4, false);
    let score: f64 = field(lines[14], "pulse_score", 4, false)[0..]
        .parse()
        .unwrap();
    let mean = norms.iter().sum::<f64>() / norms.len() as f64;
    assert!(
        (score - mean).abs() < 1e-3,
        "pulse_score {score} is the mean norm {mean}"
    );
    // the verdict line and the exit code agree
    let last = lines[15];
    if all_ok {
        assert_eq!(last, "pulse: ok");
        assert_eq!(o.status.code(), Some(0));
    } else {
        assert!(last.starts_with("pulse: FAIL "), "{last}");
        assert_eq!(o.status.code(), Some(1));
    }
    assert_eq!(stderr(&o), "");
}

#[test]
fn pulse_report_without_static_crush_prints_none() {
    let c = case(
        "nostatic",
        &box_bsn(10.0, 2.1, 1.0),
        &ref_csv(400.0),
        &meta_text(12.0, 36.0, None),
    );
    let o = run_flags(&report_flags(&c), &[]);
    let out = stdout(&o);
    let line = out
        .lines()
        .find(|l| l.starts_with("metric static_crush_m"))
        .expect("static crush line");
    let t: Vec<&str> = line.split(' ').collect();
    assert_eq!(t.len(), 4, "{line}");
    field(t[2], "sim", 4, false);
    assert_eq!(t[3], "ref=none");
    let gated = out
        .lines()
        .filter(|l| l.starts_with("metric ") && !l.ends_with("ref=none"))
        .count();
    assert_eq!(
        gated, 5,
        "five gated metric lines, the static crush line is not gated:
{out}"
    );
    let score = out.lines().find(|l| l.starts_with("pulse_score=")).unwrap();
    field(score, "pulse_score", 4, false);
    let old = out
        .lines()
        .find(|l| l.starts_with("info static_crush_centroid_m "))
        .expect("the old static crush line is printed without a meta value too");
    let t: Vec<&str> = old.split(' ').collect();
    assert_eq!(t.len(), 3, "{old}");
    field(t[2], "sim", 4, false);
}

/// The two static crush values of one run: (six-point `metric static_crush_m`, old
/// `info static_crush_centroid_m`).
fn static_pair(out: &str) -> (f64, f64) {
    let old = out
        .lines()
        .find_map(|l| l.strip_prefix("info static_crush_centroid_m sim="))
        .unwrap_or_else(|| panic!("no info static_crush_centroid_m line in:\n{out}"));
    (
        metric_field(out, "static_crush_m", "sim"),
        old.parse().unwrap(),
    )
}

/// docs/STAGE6-M4-M5.md, `static_crush_m` (rule change of 2026-11-22): the gated value is the
/// mean of six points equally spaced across the front group's node columns (z), each column the
/// plain mean of its nodes, not the distance between the group centroids. The box hits a wall
/// tilted about y, so its z = 1 side crushes more than its z = 0 side. With front group
/// {8 (z 0), 9 (z 1), 10 (z 0)} the six points weight the columns 1/2 and 1/2, so nodes
/// 8, 9, 10 get 1/4, 1/2, 1/4, which is exactly the mean of the values of the groups {8, 9}
/// and {10, 9} (one node per column, 1/2 each). A centroid weights 8, 9, 10 by 1/3 each and does
/// not meet that identity when the sides crush differently.
#[test]
fn static_crush_is_the_six_point_profile_mean() {
    let run = |name: &str, front: &str| {
        // Normal (-1, 0, -0.3) normalised: the wall meets the z = 1 side first.
        let body = box_bsn(10.0, 2.1, 1.0)
            .replace("group front 8 9 10 11\n", &format!("group front {front}\n"))
            .replace(
                "plane 2.1 0.5 0.5 -1 0 0\n",
                "plane 2.1 0.5 0.5 -0.9578262852211514 0 -0.28734788556634544\n",
            );
        assert!(body.contains(&format!("group front {front}\n")), "{body}");
        let c = case(
            name,
            &body,
            &ref_csv(400.0),
            &meta_text(12.0, 36.0, Some(0.05)),
        );
        let o = run_flags(&report_flags(&c), &[]);
        let out = stdout(&o);
        assert!(
            o.status.code() == Some(0) || o.status.code() == Some(1),
            "{out}{}",
            stderr(&o)
        );
        static_pair(&out)
    };
    let (six_3, old_3) = run("six_point_3", "8 9 10");
    let (six_89, old_89) = run("six_point_89", "8 9");
    let (six_109, old_109) = run("six_point_109", "10 9");
    // The front crushes unevenly, so the identity below is not a trivial one.
    assert!(
        (six_89 - six_109).abs() < 0.01 && six_89 > 0.002,
        "the two one-per-column groups crush alike and visibly: {six_89} {six_109}"
    );
    let want = 0.5 * (six_89 + six_109);
    // Each value is printed to 4 decimals: 1.5e-4 covers their rounding.
    assert!(
        (six_3 - want).abs() <= 1.5e-4,
        "six-point value {six_3} is not the mean of {six_89} and {six_109} ({want})"
    );
    // The old centroid rule weights the three nodes 1/3 each and so does not give that mean.
    let old_want = 0.5 * (old_89 + old_109);
    assert!(
        (old_3 - old_want).abs() > 5e-4,
        "the centroid value {old_3} also matches {old_want}: the test cannot tell the rules apart"
    );
    // No check that the two rules agree for one node per column: on this tilted wall they do not
    // (measured: 0.1434 against 0.0385 for {8, 9}); the likely cause, not measured here, is that
    // the box turns, which the profile's world-x distances read as crush and the centroid's 3-D
    // distance does not.
}

#[test]
fn pulse_report_same_run_same_bytes() {
    let c = default_case("bytes");
    let flags = report_flags(&c);
    let csv_a = c.dir.join("a.csv");
    let csv_b = c.dir.join("b.csv");
    let a = run_flags(&flags, &["--pulse-csv", csv_a.to_str().unwrap()]);
    let b = run_flags(
        &flags,
        &["--pulse-csv", csv_b.to_str().unwrap(), "--timing"],
    );
    assert!(!a.stdout.is_empty(), "the report must print something");
    assert_eq!(
        a.stdout, b.stdout,
        "stdout is the same bytes, with or without --timing"
    );
    assert_eq!(a.status.code(), b.status.code());
    let (ca, cb) = (
        std::fs::read(&csv_a).unwrap(),
        std::fs::read(&csv_b).unwrap(),
    );
    assert!(ca.len() > 100, "the CSV is not empty");
    assert_eq!(ca, cb, "the pulse CSV is the same bytes");
    // --timing: one line on stderr in the docs' format, nothing on stdout
    let e = stderr(&b);
    let lines: Vec<&str> = e.lines().collect();
    assert_eq!(lines.len(), 1, "stderr: {e}");
    let t: Vec<&str> = lines[0].split(' ').collect();
    assert_eq!(t.len(), 4, "{}", lines[0]);
    assert_eq!(t[0], "timing:");
    assert_eq!(field(t[1], "sim_s", 4, false), "0.1500");
    field(t[2], "wall_s", 4, false);
    field(t[3], "rtf", 4, false);
    assert_eq!(stderr(&a), "", "no --timing: nothing on stderr");
}

#[test]
fn pulse_csv_has_the_four_columns_from_contact() {
    let c = default_case("csv");
    let path = c.dir.join("out.csv");
    let o = run_flags(&report_flags(&c), &["--pulse-csv", path.to_str().unwrap()]);
    assert!(o.status.code().is_some_and(|c| c <= 1));
    let text = std::fs::read_to_string(&path).unwrap();
    let lines: Vec<&str> = text.lines().collect();
    assert_eq!(
        lines[0],
        "t_s,accel_mps2,accel_cfc60_mps2,accel_cfc180_mps2"
    );
    // from contact (t = 0) to the end of the run: 0.15 s run, contact at about 0.01 s
    assert!(
        lines.len() > 1300 && lines.len() < 1500,
        "{} rows",
        lines.len()
    );
    assert!(
        lines[1].starts_with("0.000000,"),
        "first row is at contact: {}",
        lines[1]
    );
    for l in &lines[1..] {
        let f: Vec<&str> = l.split(',').collect();
        assert_eq!(f.len(), 4, "{l}");
        for v in &f {
            assert!(v.parse::<f64>().is_ok_and(f64::is_finite), "{l}");
        }
        assert_eq!(f[0].split_once('.').unwrap().1.len(), 6, "{l}");
    }
    // the crash decelerates the car past the run's own --pulse-threshold (20 m/s^2), so the
    // duration metric of the report is not vacuous: some unfiltered sample is at or below -20
    let min = lines[1..]
        .iter()
        .map(|l| l.split(',').nth(1).unwrap().parse::<f64>().unwrap())
        .fold(f64::INFINITY, f64::min);
    assert!(min <= -20.0, "min unfiltered accel {min}");
}

/// The value of `name` in the `metric <name> ...` line, `key=` field.
fn metric_field(out: &str, name: &str, key: &str) -> f64 {
    let line = out
        .lines()
        .find(|l| l.starts_with(&format!("metric {name} ")))
        .unwrap_or_else(|| panic!("no metric {name} line in:\n{out}"));
    let tok = line
        .split(' ')
        .find(|t| t.starts_with(&format!("{key}=")))
        .unwrap();
    tok[key.len() + 1..].parse().unwrap()
}

#[test]
fn pulse_report_fails_on_scaled_reference() {
    // Pass 1: a synthetic reference; its CSV gives a pulse the sim reproduces by construction.
    let c = default_case("scaled");
    let csv = c.dir.join("sim.csv");
    let first = run_flags(&report_flags(&c), &["--pulse-csv", csv.to_str().unwrap()]);
    let first_out = stdout(&first);
    let rows: Vec<(f64, f64)> = std::fs::read_to_string(&csv)
        .unwrap()
        .lines()
        .skip(1)
        .map(|l| {
            let f: Vec<&str> = l.split(',').collect();
            (f[0].parse().unwrap(), f[1].parse().unwrap())
        })
        .collect();
    // Pass 2: that pulse as the reference (zero before contact, 0.02 s of it), scaled by `k`.
    // Only the peak margin is tight, so a scaled reference fails on the peak and nothing earlier.
    let make = |name: &str, k: f64| {
        let mut s = String::from("t_s,accel_mps2\n");
        for i in 0..200 {
            s.push_str(&format!("{:.6},0.000000\n", -0.02 + f64::from(i) * 1e-4));
        }
        for (t, a) in &rows {
            s.push_str(&format!("{t:.6},{:.6}\n", a * k));
        }
        let sim_static = metric_field(&first_out, "static_crush_m", "sim");
        case(
            name,
            &box_bsn(10.0, 2.1, 1.0),
            &s,
            &meta_text(12.0, 36.0, Some(sim_static)),
        )
    };
    let loose = |c: &Case| {
        let mut f = report_flags(c);
        for flag in [
            "--margin-dv",
            "--margin-duration",
            "--margin-olc",
            "--margin-static-crush",
        ] {
            f = with(&f, flag, "1.0");
        }
        with(&f, "--margin-peak", "0.15")
    };
    let same = make("scaled_1", 1.0);
    let o1 = run_flags(&loose(&same), &[]);
    let out1 = stdout(&o1);
    assert!(
        out1.lines().last() == Some("pulse: ok"),
        "an unscaled copy of the sim's own pulse passes (this baseline only proves the scaling is what fails):\n{out1}"
    );
    assert_eq!(o1.status.code(), Some(0));
    let scaled = make("scaled_13", 1.3);
    let o2 = run_flags(&loose(&scaled), &[]);
    let out2 = stdout(&o2);
    let last = out2.lines().last().unwrap();
    assert!(last.starts_with("pulse: FAIL peak_mps2 norm="), "{out2}");
    assert_eq!(o2.status.code(), Some(1));
    assert!(
        metric_field(&out2, "peak_mps2", "err") < -0.15,
        "the sim peak is well under 1.3 x"
    );
}

#[test]
fn pulse_report_missing_flag_is_usage_error() {
    let c = default_case("missing");
    let flags = report_flags(&c);
    let base = run_flags(&flags, &[]);
    assert!(
        base.status.code().is_some_and(|c| c <= 1),
        "baseline: {}",
        stderr(&base)
    );
    assert!(
        stdout(&base).contains("pulse: "),
        "the complete flag set must run"
    );
    for i in 0..flags.len() {
        let mut f = flags.clone();
        let (flag, _) = f.remove(i);
        let o = run_flags(&f, &[]);
        assert_eq!(o.status.code(), Some(2), "without {flag}: {}", stderr(&o));
        assert_eq!(stdout(&o), "", "without {flag}: stdout must be empty");
        assert!(
            stderr(&o).contains(&flag),
            "without {flag}: stderr must name it, got: {}",
            stderr(&o)
        );
    }
}

#[test]
fn pulse_report_refusals_are_exit_2_with_empty_stdout() {
    let c = default_case("refuse");
    let flags = report_flags(&c);
    // The unchanged flags run (so a refusal below is caused by its one change, not by a broken
    // setup or by a binary that does not know the flags at all).
    let base = run_flags(&flags, &[]);
    assert!(
        base.status.code().is_some_and(|c| c <= 1),
        "baseline: {}",
        stderr(&base)
    );
    assert!(
        stdout(&base).ends_with('\n') && stdout(&base).contains("pulse: "),
        "baseline"
    );
    // Each refusal: exit 2, nothing on stdout, and the stderr text names the reason.
    let refuse = |what: &str, f: &[(String, String)], extra: &[&str], reason: &str| {
        let o = run_flags(f, extra);
        assert_eq!(o.status.code(), Some(2), "{what}: stderr {}", stderr(&o));
        assert_eq!(stdout(&o), "", "{what}: stdout");
        assert!(
            stderr(&o).contains(reason),
            "{what}: stderr must contain `{reason}`, got: {}",
            stderr(&o)
        );
    };
    let refp = |name: &str| c.dir.join(name).to_str().unwrap().to_string();
    refuse(
        "path suffix",
        &with(&flags, "--pulse-report", &refp("ref.csv")),
        &[],
        "ending in .pulse.csv",
    );
    refuse(
        "missing file",
        &with(&flags, "--pulse-report", &refp("nope.pulse.csv")),
        &[],
        "cannot read file",
    );
    // a sample rate under 10 x CFC: h = 0.1 ms is 10 kHz, so CFC 1001 needs 10010 Hz
    refuse(
        "rate under 10 x CFC",
        &with(&flags, "--pulse-cfc-int", "1001"),
        &[],
        "J211 minimum",
    );
    refuse(
        "rate under 10 x CFC (shape)",
        &with(&flags, "--pulse-cfc-shape", "1001"),
        &[],
        "J211 minimum",
    );
    // a window longer than the sim record after contact, and longer than the reference (0.2 s,
    // from -0.02 s, so it ends at 0.18 s) with a sim record that is long enough
    refuse(
        "window over the sim record",
        &with(&flags, "--pulse-window", "0.14"),
        &[],
        "longer than the sim pulse",
    );
    let long = with(
        &with(&flags, "--duration", "0.35"),
        "--pulse-window",
        "0.25",
    );
    refuse(
        "window over the reference",
        &long,
        &[],
        "does not fit the reference",
    );
    // limit input: a window or a run so long that its sample count overflows
    refuse(
        "huge window",
        &with(&flags, "--pulse-window", "1e300"),
        &[],
        "longer than the sim pulse",
    );
    refuse(
        "huge duration",
        &with(&flags, "--duration", "1e300"),
        &[],
        "substeps, over the limit",
    );
    refuse(
        "huge substeps",
        &with(&flags, "--substeps", "4294967295"),
        &[],
        "substeps, over the limit",
    );
    refuse(
        "negative margin",
        &with(&flags, "--margin-dv", "-0.05"),
        &[],
        "--margin-dv must be",
    );
    refuse(
        "zero margin",
        &with(&flags, "--margin-olc", "0"),
        &[],
        "--margin-olc must be",
    );
    refuse(
        "NaN window",
        &with(&flags, "--pulse-window", "NaN"),
        &[],
        "--pulse-window must be",
    );
    refuse(
        "inf cfc",
        &with(&flags, "--pulse-cfc-shape", "inf"),
        &[],
        "--pulse-cfc-shape must be",
    );
    refuse(
        "unknown group",
        &with(&flags, "--pulse-sensor", "nonesuch"),
        &[],
        "no group `nonesuch`",
    );
    refuse(
        "crush groups need two",
        &with(&flags, "--pulse-crush-groups", "rear"),
        &[],
        "--pulse-crush-groups needs",
    );
    refuse(
        "tail longer than the run",
        &with(&flags, "--pulse-crush-tail", "0.2"),
        &[],
        "--pulse-crush-tail",
    );
    refuse(
        "negative gravity",
        &with(&flags, "--gravity", "-1"),
        &[],
        "gravity >= 0",
    );
    refuse(
        "solver other than node-beam and m5",
        &flags,
        &["--solver", "implicit"],
        "crash-sim: --solver implicit: accepted values are node-beam (default) and m5",
    );
    refuse(
        "--summary",
        &flags,
        &["--summary"],
        "--pulse-report takes only",
    );
    refuse("--live", &flags, &["--live"], "--pulse-report takes only");
    let no_report: Vec<(String, String)> = flags
        .iter()
        .filter(|(k, _)| k != "--pulse-report")
        .cloned()
        .collect();
    refuse(
        "flags without --pulse-report",
        &no_report,
        &[],
        "need --pulse-report",
    );
    // a meta file missing a required key; a malformed reference pulse; a record that starts after
    // contact, so it cannot supply the window from t = 0; a file that is not a body
    std::fs::write(
        c.dir.join("ref.meta"),
        "test = x\nmass_kg = 12\nspeed_kmh = 36\nchannel = c\n",
    )
    .unwrap();
    refuse("meta lacks sample_rate_hz", &flags, &[], "sample_rate_hz");
    std::fs::write(c.dir.join("ref.meta"), meta_text(12.0, 36.0, None)).unwrap();
    std::fs::write(&c.refp, "t_s,accel_mps2\n0,1\n").unwrap();
    refuse("too few reference samples", &flags, &[], "samples");
    std::fs::write(&c.refp, ref_csv_from(400.0, 0.001)).unwrap();
    refuse(
        "reference starts after contact",
        &flags,
        &[],
        "does not fit the reference",
    );
    std::fs::write(&c.refp, ref_csv(400.0)).unwrap();
    std::fs::write(&c.body, "not a body\n").unwrap();
    refuse("bad body", &flags, &[], "body.bsn");
}

#[test]
fn no_contact_is_a_fail_with_exit_1() {
    let c = case(
        "nocontact",
        &box_bsn(10.0, 100.0, 1.0),
        &ref_csv(400.0),
        &meta_text(12.0, 36.0, None),
    );
    let o = run_flags(&report_flags(&c), &[]);
    let out = stdout(&o);
    assert_eq!(o.status.code(), Some(1), "{out}");
    let lines: Vec<&str> = out.lines().collect();
    assert_eq!(lines[lines.len() - 2], "contact_s=none");
    assert_eq!(lines[lines.len() - 1], "pulse: FAIL no contact");
    assert!(
        !out.contains("metric "),
        "no metric without contact:\n{out}"
    );
}

#[test]
fn setup_mass_mismatch_is_named_in_the_fail_line() {
    let c = case(
        "badmass",
        &box_bsn(10.0, 2.1, 1.0),
        &ref_csv(400.0),
        &meta_text(13.0, 36.0, None),
    );
    let o = run_flags(&report_flags(&c), &[]);
    let out = stdout(&o);
    assert_eq!(o.status.code(), Some(1));
    let last = out.lines().last().unwrap();
    assert!(
        last.starts_with("pulse: FAIL setup mass_kg err=-0.0769 tol=0.0100"),
        "{last}"
    );
    let setup = out
        .lines()
        .find(|l| l.starts_with("setup mass_kg"))
        .unwrap();
    assert!(setup.ends_with(" FAIL"), "{setup}");
}

#[test]
fn setup_speed_mismatch_is_named_in_the_fail_line() {
    let c = case(
        "badspeed",
        &box_bsn(10.0, 2.1, 1.0),
        &ref_csv(400.0),
        &meta_text(12.0, 40.0, None),
    );
    let o = run_flags(&report_flags(&c), &[]);
    let out = stdout(&o);
    assert_eq!(o.status.code(), Some(1));
    let last = out.lines().last().unwrap();
    assert!(
        last.starts_with("pulse: FAIL setup speed_kmh err=-0.1000 tol=0.0050"),
        "{last}"
    );
}

#[test]
fn gravity_zero_run_has_no_ground_contact() {
    // Level at 1 m with no gravity: the nodes never reach the ground, so the count is 0 and the
    // verdict is not a ground FAIL.
    let c = default_case("g0");
    let out = stdout(&run_flags(&report_flags(&c), &[]));
    assert!(
        out.lines().any(|l| l == "ground_contacts_in_window=0"),
        "{out}"
    );
    assert!(!out.contains("FAIL ground contact"), "{out}");
    // The same run with the box resting on the ground counts contacts in the window and fails on
    // the first one, at contact (t=0.0000): the rule can fail.
    let low = case(
        "g0_low",
        &box_bsn(10.0, 2.1, 0.0),
        &ref_csv(400.0),
        &meta_text(12.0, 36.0, None),
    );
    let o = run_flags(&report_flags(&low), &[]);
    let out = stdout(&o);
    let count: usize = out
        .lines()
        .find_map(|l| l.strip_prefix("ground_contacts_in_window="))
        .expect("the count line")
        .parse()
        .unwrap();
    assert!(count > 0, "{out}");
    assert_eq!(o.status.code(), Some(1));
    assert_eq!(
        out.lines().last().unwrap(),
        "pulse: FAIL ground contact at t=0.0000"
    );
}

#[test]
fn an_unstable_run_is_a_fail_line_with_exit_1() {
    let c = case(
        "unstable",
        // elastic beams (no yield strain), so the crush blows past the strain limit
        &box_bsn(100.0, 2.1, 1.0).replace(" yield_strain 0.02", ""),
        &ref_csv(400.0),
        &meta_text(12.0, 360.0, None),
    );
    let o = run_flags(&report_flags(&c), &[]);
    let out = stdout(&o);
    assert_eq!(o.status.code(), Some(1), "{out}");
    assert!(out.starts_with("pulse: FAIL unstable at t = "), "{out}");
    assert_eq!(out.lines().count(), 1);
}

#[test]
fn a_car_with_wheels_counts_the_wheels_in_its_mass() {
    // car_basic.bsn with the groups and wall the report needs and a start speed, no gravity.
    let text = std::fs::read_to_string(format!(
        "{}/scenarios/car_basic.bsn",
        env!("CARGO_MANIFEST_DIR")
    ))
    .unwrap();
    let body = crash_sim::format::parse(&text).unwrap();
    let built = body.build();
    let nodes_mass: f64 = built.mass.iter().sum();
    let wheels_mass: f64 = body.wheels.iter().map(|w| w.mass).sum();
    assert!(wheels_mass > 0.0);
    let front = built
        .pos
        .iter()
        .map(|p| p.x)
        .fold(f64::NEG_INFINITY, f64::max);
    let all: Vec<String> = (0..built.pos.len()).map(|i| i.to_string()).collect();
    // The file's sections come in a fixed order: the new groups and the velocity go before the
    // first `wheel` line, the plane at the end.
    let at = text.find("\nwheel ").expect("the car has wheels") + 1;
    let car = format!(
        "{}group sensor {}\ngroup all {}\nvelocity all 10 0 0\n{}plane {} 0 0 -1 0 0\n",
        &text[..at],
        all[..4].join(" "),
        all.join(" "),
        &text[at..],
        front + 0.1
    );
    let total = nodes_mass + wheels_mass;
    let c = case("car", &car, &ref_csv(400.0), &meta_text(total, 36.0, None));
    let o = run_flags(&report_flags(&c), &[]);
    let out = stdout(&o);
    assert!(
        o.status.code().is_some_and(|c| c <= 1),
        "stderr {}\n{out}",
        stderr(&o)
    );
    let line = out
        .lines()
        .find(|l| l.starts_with("setup mass_kg"))
        .unwrap_or_else(|| panic!("no setup line:\n{out}\n{}", stderr(&o)));
    assert!(
        line.starts_with(&format!("setup mass_kg sim={total:.1} ref={total:.1} err=")),
        "{line}"
    );
}

#[test]
fn gravity_zero_is_accepted_and_bad_gravity_is_refused() {
    let zero = Config {
        gravity: 0.0,
        ..Config::default()
    };
    assert_eq!(zero.validate(), Ok(()), "gravity 0 must be accepted");
    assert_eq!(zero.validate_body(&[], &[]), Ok(()));
    for bad in [-1.0, -1e-300, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let cfg = Config {
            gravity: bad,
            ..Config::default()
        };
        let e = cfg
            .validate()
            .expect_err(&format!("gravity {bad} must be refused"));
        assert!(e.contains("gravity >= 0"), "message names the rule: {e}");
        assert!(
            cfg.validate_body(&[], &[]).is_err(),
            "validate_body, gravity {bad}"
        );
    }
}

// ---------------------------------------------------------------------------------------
// `--solver` (task t-20261123-202940): `node-beam` (default) or `m5`, the implicit solver.

/// The first stderr line of a refused `--solver` value.
fn solver_refusal(value: &str) -> String {
    format!("crash-sim: --solver {value}: accepted values are node-beam (default) and m5\n")
}

/// The box body with beams 5000 times stiffer (k = 1e8 N/m): the explicit step limit is then
/// about 43 us, so the default h = 0.1 ms is refused for node-beam and h = 10 us is accepted.
fn stiff_box() -> String {
    let text = box_bsn(10.0, 2.1, 1.0);
    let stiff = text.replacen("stiffness 20000 damping", "stiffness 100000000 damping", 1);
    assert_ne!(stiff, text);
    stiff
}

#[test]
fn pulse_solver_values() {
    let c = default_case("solver_values");
    let flags = report_flags(&c);
    // The complete rule: exact, case-sensitive; anything else is a usage error.
    for v in ["M5", "implicit", "", "node-beam,m5", "Node-Beam", " m5", "m5 "] {
        let o = run_flags(&flags, &["--solver", v]);
        assert_eq!(o.status.code(), Some(2), "--solver {v:?}: {}", stderr(&o));
        assert_eq!(stdout(&o), "", "--solver {v:?}: stdout");
        assert!(
            stderr(&o).starts_with(&solver_refusal(v)),
            "--solver {v:?}: stderr {}",
            stderr(&o)
        );
    }
    // Repeated: the last one wins.
    let o = run_flags(&flags, &["--solver", "M5", "--solver", "m5"]);
    assert!(
        o.status.code().is_some_and(|c| c <= 1) && stdout(&o).contains("pulse_score="),
        "last --solver m5 must run: {}",
        stderr(&o)
    );
    let o = run_flags(&flags, &["--solver", "m5", "--solver", "M5"]);
    assert_eq!(o.status.code(), Some(2));
    assert_eq!(stdout(&o), "");
    assert!(stderr(&o).starts_with(&solver_refusal("M5")), "{}", stderr(&o));
    // Outside --pulse-report, --solver is refused as before.
    let body = vec![("--body".to_string(), c.body.to_str().unwrap().to_string())];
    let o = run_flags(&body, &["--solver", "m5"]);
    assert_eq!(o.status.code(), Some(2));
    assert_eq!(stdout(&o), "");
    assert!(
        stderr(&o).starts_with("--pulse-csv, --timing, --solver and the --pulse-*, --setup-* and --margin-* flags need --pulse-report\n"),
        "{}",
        stderr(&o)
    );
    // A body with tets: m5 is a node-beam solver, so the existing tet rule refuses it, with the
    // same message as node-beam.
    let tets = box_bsn(10.0, 2.1, 1.0)
        .replacen("bsn 1\n", "bsn 3\n", 1)
        .replacen(
            "yield_strain 0.02\n",
            "yield_strain 0.02\nsolid s plateau_stress 1000000 yield_strain 0.02 damping_ratio 0.01\n",
            1,
        )
        .replacen("group rear", "tet 0 4 2 1 s\ngroup rear", 1);
    let ct = case("solver_tets", &tets, &ref_csv(400.0), &meta_text(12.0, 36.0, None));
    let tflags = report_flags(&ct);
    let m5 = run_flags(&tflags, &["--solver", "m5"]);
    let nb = run_flags(&tflags, &["--solver", "node-beam"]);
    for (name, o) in [("m5", &m5), ("node-beam", &nb)] {
        assert_eq!(o.status.code(), Some(2), "{name}: {}", stderr(o));
        assert_eq!(stdout(o), "", "{name}");
        assert_eq!(
            stderr(o),
            "crash-sim: the body has 1 tets, which this solver does not simulate (they would be dropped): tets are simulated by `solver_m5::SolverM5` (docs/PHYSICS.md, \"Solids\")\n",
            "{name}"
        );
    }
}

#[test]
fn pulse_solver_node_beam_is_the_default() {
    let c = default_case("solver_default");
    let flags = report_flags(&c);
    let plain = run_flags(&flags, &[]);
    let named = run_flags(&flags, &["--solver", "node-beam"]);
    assert!(plain.status.code().is_some_and(|c| c <= 1), "{}", stderr(&plain));
    assert!(stdout(&plain).contains("pulse_score="), "{}", stdout(&plain));
    assert_eq!(named.status.code(), plain.status.code());
    assert_eq!(named.stdout, plain.stdout, "stdout bytes");
    assert_eq!(named.stderr, plain.stderr, "stderr bytes");
}

#[test]
fn pulse_solver_m5_runs_above_explicit_limit() {
    let c = case(
        "solver_m5_above",
        &stiff_box(),
        &ref_csv(400.0),
        &meta_text(12.0, 36.0, Some(0.05)),
    );
    let flags = report_flags(&c); // --dt 0.001 --substeps 10: h = 0.1 ms
    // node-beam (named or by default) is refused by the explicit limit, same bytes either way.
    let nb = run_flags(&flags, &["--solver", "node-beam"]);
    let plain = run_flags(&flags, &[]);
    assert_eq!(nb.status.code(), Some(2), "{}", stderr(&nb));
    assert_eq!(stdout(&nb), "");
    assert!(
        stderr(&nb).starts_with("crash-sim: physics step h = dt/substeps = 0.000100 s reaches the body's stability limit "),
        "{}",
        stderr(&nb)
    );
    assert_eq!(plain.status.code(), Some(2));
    assert_eq!(plain.stdout, nb.stdout);
    assert_eq!(plain.stderr, nb.stderr);
    // m5 runs at the same step: a report, exit 0 or 1, with a score.
    let m5 = run_flags(&flags, &["--solver", "m5"]);
    let out = stdout(&m5);
    assert!(
        m5.status.code().is_some_and(|c| c <= 1),
        "m5: exit {:?}, stderr {}\n{out}",
        m5.status.code(),
        stderr(&m5)
    );
    assert!(
        out.lines().any(|l| l.starts_with("pulse_score=")),
        "m5: no pulse_score line:\n{out}"
    );
    // At a step both accept (h = 10 us), both run and m5 is a different integrator.
    let fine = with(&flags, "--substeps", "100");
    let nb = run_flags(&fine, &["--solver", "node-beam"]);
    let m5 = run_flags(&fine, &["--solver", "m5"]);
    for (name, o) in [("node-beam", &nb), ("m5", &m5)] {
        assert!(
            o.status.code().is_some_and(|c| c <= 1) && stdout(o).contains("pulse_score="),
            "{name} at h = 10 us: exit {:?}, stderr {}",
            o.status.code(),
            stderr(o)
        );
    }
    assert_ne!(nb.stdout, m5.stdout, "m5 gave node-beam's report: not the implicit solver");
}
