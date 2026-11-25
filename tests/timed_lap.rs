//! The timed lap (`--track`, docs/STAGE7.md "The race program"): these tests run the real binary on
//! `scenarios/tracks/oval.track` with `scenarios/tracks/oval_lap.input`, headless and with `--live`.

use std::io::{BufRead, BufReader, Write};
use std::process::{Command, Stdio};
use std::sync::mpsc;
use std::time::Duration;
mod run_tmp;

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

fn manifest(path: &str) -> String {
    format!("{}/{path}", env!("CARGO_MANIFEST_DIR"))
}

fn car_path() -> String {
    manifest("scenarios/car_basic.bsn")
}

fn oval_path() -> String {
    manifest("scenarios/tracks/oval.track")
}

fn lap_input_path() -> String {
    manifest("scenarios/tracks/oval_lap.input")
}

/// A file in the build's own temp folder (the test binary's, not the repo).
fn tmp_file(name: &str, text: &str) -> String {
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

fn lap_run(track: &str, input: &str) -> Run {
    run(&["--body", &car_path(), "--track", track, "--input", input])
}

/// The value of `key=` on a race line.
fn field<'a>(line: &'a str, key: &str) -> &'a str {
    let pat = format!("{key}=");
    line.split(' ')
        .find_map(|w| w.strip_prefix(pat.as_str()))
        .unwrap_or_else(|| panic!("no `{key}=` on `{line}`"))
}

fn num(line: &str, key: &str) -> f64 {
    field(line, key)
        .parse()
        .unwrap_or_else(|_| panic!("`{key}` is not a number on `{line}`"))
}

fn lines_starting<'a>(text: &'a str, prefix: &str) -> Vec<&'a str> {
    text.lines().filter(|l| l.starts_with(prefix)).collect()
}

/// The race lines of a run, in order: everything that is not a row or a `track_line`.
fn race_lines(text: &str) -> Vec<&str> {
    text.lines()
        .filter(|l| {
            [
                "track ",
                "start ",
                "checkpoint ",
                "lap ",
                "final ",
                "race: ",
            ]
            .iter()
            .any(|p| l.starts_with(p))
        })
        .collect()
}

const OVAL_TRACK_LINE: &str =
    "track name=oval points=64 checkpoints=3 centre_length_m=388.41 width_min_m=12.00 width_max_m=12.00";

#[test]
fn oval_script_gives_counted_laps() {
    let r = lap_run(&oval_path(), &lap_input_path());
    assert_eq!(r.code, 0, "stderr: {}", r.stderr);
    let laps = lines_starting(&r.stdout, "lap ");
    assert_eq!(laps.len(), 2, "two counted laps, got:\n{}", r.stdout);
    for (i, l) in laps.iter().enumerate() {
        assert_eq!(field(l, "n"), (i + 1).to_string(), "{l}");
        assert_eq!(field(l, "valid"), "yes", "{l}");
        assert_eq!(field(l, "reason"), "none", "{l}");
    }
    // Lap 2 starts where lap 1 ends, character for character.
    assert_eq!(field(laps[0], "end_t"), field(laps[1], "start_t"));
    assert_eq!(
        r.stdout.lines().last().unwrap(),
        format!(
            "race: laps_valid=2 laps_invalid=0 best_lap_s={}",
            [field(laps[0], "time_s"), field(laps[1], "time_s")]
                .iter()
                .min_by(|a, b| a
                    .parse::<f64>()
                    .unwrap()
                    .total_cmp(&b.parse::<f64>().unwrap()))
                .unwrap()
        )
    );
    // Three checkpoints per lap, in order, each between the lap's two line crossings.
    for lap in 1..=2 {
        let prefix = format!("checkpoint lap={lap} ");
        let cps = lines_starting(&r.stdout, &prefix);
        assert_eq!(cps.len(), 3, "{prefix}: {cps:?}");
        let mut last = num(laps[lap - 1], "start_t");
        for (k, c) in cps.iter().enumerate() {
            assert_eq!(field(c, "k"), (k + 1).to_string());
            let t = num(c, "t");
            assert!(t > last, "checkpoint times must increase: {c}");
            last = t;
        }
        assert!(last < num(laps[lap - 1], "end_t"));
    }
}

#[test]
fn timed_lap_same_input_same_bytes() {
    let a = lap_run(&oval_path(), &lap_input_path());
    let b = lap_run(&oval_path(), &lap_input_path());
    assert_eq!(a.code, 0);
    assert_eq!(b.code, 0);
    assert!(a.stdout.lines().count() >= 10, "{}", a.stdout);
    assert_eq!(a.stdout.as_bytes(), b.stdout.as_bytes());
}

/// docs/STAGE7.md check 1.4 and its floors, for each lap of the oval script.
#[test]
fn oval_lap_is_valid_and_above_floors() {
    let r = lap_run(&oval_path(), &lap_input_path());
    assert_eq!(r.code, 0, "stderr: {}", r.stderr);
    assert_eq!(r.stdout.lines().next().unwrap(), OVAL_TRACK_LINE);
    let starts = lines_starting(&r.stdout, "start ");
    assert_eq!(starts.len(), 1, "exactly one start line:\n{}", r.stdout);
    let laps = lines_starting(&r.stdout, "lap ");
    assert!(!laps.is_empty());
    assert_eq!(field(laps[0], "start_t"), field(starts[0], "t"));
    for l in &laps {
        assert_eq!(field(l, "off_track_s"), "0.000", "{l}");
        let path = num(l, "path_m");
        assert!(path >= 338.67, "path floor: {l}");
        let (time, vmax) = (num(l, "time_s"), num(l, "max_speed_kmh"));
        assert!(time * vmax / 3.6 >= path - 2.0, "timing consistency: {l}");
        let span = num(l, "end_t") - num(l, "start_t");
        assert!(
            (time - span).abs() <= 0.0006,
            "time_s against end_t - start_t: {l}"
        );
        // time_s has 3 decimals: the whole millisecond is the integer round(span * 1000).
        let ms = field(l, "time_s").replace('.', "").parse::<i64>().unwrap();
        assert!((ms as f64 - span * 1000.0).abs() <= 0.6, "{l}");
    }
}

/// A circle of constant steer: it crosses the start line forward again after a full turn, with
/// no checkpoint crossed in order, so the lap ends but is not valid.
#[test]
fn script_that_cuts_a_checkpoint_gives_no_counted_lap() {
    let script = tmp_file(
        "timed_lap_circle.input",
        "0 0.3 0 0\n5.2 0.04 0 0.16\n40 0.04 0 0.16\n",
    );
    let r = lap_run(&oval_path(), &script);
    assert_eq!(r.code, 0, "stderr: {}", r.stderr);
    let laps = lines_starting(&r.stdout, "lap ");
    assert_eq!(laps.len(), 1, "{}", r.stdout);
    assert_eq!(field(laps[0], "valid"), "no");
    assert_eq!(field(laps[0], "reason"), "missed_checkpoint");
    assert!(
        lines_starting(&r.stdout, "checkpoint ").is_empty(),
        "{}",
        r.stdout
    );
    assert_eq!(
        r.stdout.lines().last().unwrap(),
        "race: laps_valid=0 laps_invalid=1 best_lap_s=none"
    );
}

/// Mass-weighted centroid and rear-to-front heading from the t = 0 frame of a v2 live run.
fn grid_state(track_text: &str, name: &str) -> ((f64, f64), f64) {
    let track = tmp_file(name, track_text);
    let mut child = Command::new(crash_sim_bin())
        .args([
            "--body",
            &car_path(),
            "--track",
            &track,
            "--live",
            "--trajectory-v2",
        ])
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn");
    child
        .stdin
        .take()
        .unwrap()
        .write_all(b"0 0 0 0\n0.01 0 0 0\n")
        .unwrap();
    let out = child.wait_with_output().unwrap();
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let text = String::from_utf8(out.stdout).unwrap();
    let body = crash_sim::format::parse(&std::fs::read_to_string(car_path()).unwrap()).unwrap();
    let mut pos = vec![(0.0, 0.0); body.nodes.len()];
    let mut seen = 0;
    for l in text.lines() {
        let f: Vec<&str> = l.split(',').collect();
        if f.len() == 5 && f[0] == "0.0000" {
            let n: usize = f[1].parse().unwrap();
            pos[n] = (f[2].parse().unwrap(), f[4].parse().unwrap());
            seen += 1;
        }
    }
    assert_eq!(seen, body.nodes.len(), "one t = 0 frame:\n{text}");
    let total: f64 = body.nodes.iter().map(|n| n.mass).sum();
    let c = (
        pos.iter()
            .zip(&body.nodes)
            .map(|(p, n)| p.0 * n.mass)
            .sum::<f64>()
            / total,
        pos.iter()
            .zip(&body.nodes)
            .map(|(p, n)| p.1 * n.mass)
            .sum::<f64>()
            / total,
    );
    let mean = |group: &str| {
        let g = body.groups.iter().find(|g| g.name == group).unwrap();
        let ix: Vec<usize> = g
            .nodes
            .iter()
            .map(|id| body.node_index(*id).unwrap())
            .collect();
        (
            ix.iter().map(|&i| pos[i].0).sum::<f64>() / ix.len() as f64,
            ix.iter().map(|&i| pos[i].1).sum::<f64>() / ix.len() as f64,
        )
    };
    let (f, r) = (mean("front"), mean("rear"));
    (c, (f.1 - r.1).atan2(f.0 - r.0).to_degrees())
}

/// The frame text has 4 decimals, so a position is read to 1e-4 m and a heading (a ratio of two
/// such values over a 2 m wheelbase-scale baseline) to well under 0.01 degree.
const GRID_POS_TOL_M: f64 = 1e-3;
const GRID_HEADING_TOL_DEG: f64 = 0.01;

#[test]
fn car_on_grid_faces_the_track() {
    // The oval: start tangent +x, 20 m before the line at (0, 0).
    let oval = std::fs::read_to_string(oval_path()).unwrap();
    let ((x, z), h) = grid_state(&oval, "grid_oval.track");
    assert!(
        (x + 20.0).abs() <= GRID_POS_TOL_M && z.abs() <= GRID_POS_TOL_M,
        "{x} {z}"
    );
    assert!(h.abs() <= GRID_HEADING_TOL_DEG, "{h}");
    // A track whose start tangent is (1, 1) / sqrt 2 turns the car 45 degrees; the grid point is
    // 20 m back along the last segment, (-20, 0).
    let diag = "track 1\nwidth 12\nstart 0\ngrid 20\ncheckpoint 2\npoint 0 0\npoint 0 100\npoint -100 100\npoint -100 0\n";
    let ((x, z), h) = grid_state(diag, "grid_diag.track");
    assert!(
        (x + 20.0).abs() <= GRID_POS_TOL_M && z.abs() <= GRID_POS_TOL_M,
        "{x} {z}"
    );
    assert!((h - 45.0).abs() <= GRID_HEADING_TOL_DEG, "{h}");
}

/// Spawn a live run on the oval; stdout lines arrive on the returned channel.
fn live_oval(v2: bool) -> (std::process::Child, mpsc::Receiver<String>) {
    let mut c = Command::new(crash_sim_bin());
    c.args(["--body", &car_path(), "--track", &oval_path(), "--live"]);
    if v2 {
        c.arg("--trajectory-v2");
    }
    let mut child = c
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn");
    let out = child.stdout.take().unwrap();
    let (tx, rx) = mpsc::channel();
    std::thread::spawn(move || {
        for l in BufReader::new(out).lines() {
            match l {
                Ok(l) => {
                    if tx.send(l).is_err() {
                        break;
                    }
                }
                Err(_) => break,
            }
        }
    });
    (child, rx)
}

#[test]
fn live_and_file_input_give_same_race_lines() {
    let file = lap_run(&oval_path(), &lap_input_path());
    assert_eq!(file.code, 0);
    // Not vacuous: the lines being compared hold the two laps.
    assert_eq!(
        lines_starting(&file.stdout, "lap ").len(),
        2,
        "{}",
        file.stdout
    );
    let input = std::fs::read_to_string(lap_input_path()).unwrap();
    for v2 in [false, true] {
        let (mut child, rx) = live_oval(v2);
        child
            .stdin
            .take()
            .unwrap()
            .write_all(input.as_bytes())
            .unwrap();
        let mut live = String::new();
        while let Ok(l) = rx.recv_timeout(Duration::from_secs(120)) {
            live.push_str(&l);
            live.push('\n');
        }
        assert!(child.wait().unwrap().success());
        assert_eq!(
            race_lines(&live),
            race_lines(&file.stdout),
            "v2={v2}: live and file race lines differ"
        );
        // The draw lines come before the first row: 64 x 2 edges, 1 start, 3 checkpoints.
        let first_row = live
            .lines()
            .position(|l| l.starts_with("t,"))
            .expect("a column header");
        let draw: Vec<&str> = live
            .lines()
            .take(first_row)
            .filter(|l| l.starts_with("track_line "))
            .collect();
        assert_eq!(draw.len(), 128 + 1 + 3, "{}", draw.len());
        assert_eq!(
            draw.iter()
                .filter(|l| l.starts_with("track_line start "))
                .count(),
            1
        );
        assert_eq!(
            draw.iter()
                .filter(|l| l.starts_with("track_line checkpoint "))
                .count(),
            3
        );
        assert_eq!(
            lines_starting(&live, "track_line ").len(),
            draw.len(),
            "no draw line after the first row"
        );
    }
}

/// A lap line is printed as soon as the lap ends, not at end of input.
#[test]
fn live_prints_lap_lines_as_they_happen() {
    let input = std::fs::read_to_string(lap_input_path()).unwrap();
    // The lines up to t = 40 s: lap 1 ends at about 39.9 s, lap 2 not before the end of the file.
    let head: String = input
        .lines()
        .filter(|l| l.split(' ').next().unwrap().parse::<f64>().unwrap() <= 40.0)
        .map(|l| format!("{l}\n"))
        .collect();
    let (mut child, rx) = live_oval(false);
    let mut stdin = child.stdin.take().unwrap();
    stdin.write_all(head.as_bytes()).unwrap();
    stdin.flush().unwrap();
    let mut got = None;
    while let Ok(l) = rx.recv_timeout(Duration::from_secs(120)) {
        if l.starts_with("lap n=1 ") {
            got = Some(l);
            break;
        }
    }
    let lap = got.expect("a `lap n=1` line before the input ended");
    assert_eq!(field(&lap, "valid"), "yes");
    // The run has not ended: the `final` line comes only after end of input.
    assert!(rx.try_recv().map_or(true, |l| !l.starts_with("final ")));
    drop(stdin);
    let rest: Vec<String> = rx.iter().collect();
    assert!(rest.iter().any(|l| l.starts_with("final ")), "{rest:?}");
    assert!(child.wait().unwrap().success());
}

#[test]
fn track_refusals_exit_2_with_empty_stdout() {
    let missing = tmp_file("timed_lap_missing_input.txt", "");
    let _ = std::fs::remove_file(&missing);
    let bad_track = tmp_file("timed_lap_bad.track", "track 1\nwidth 12\nbogus\n");
    let cases: Vec<(Vec<String>, &str)> = vec![
        (
            vec![
                "--track".into(),
                manifest("scenarios/tracks/nope.track"),
                "--input".into(),
                lap_input_path(),
            ],
            "nope.track",
        ),
        (
            vec![
                "--track".into(),
                bad_track.clone(),
                "--input".into(),
                lap_input_path(),
            ],
            "line 3, column 1",
        ),
        (vec!["--track".into(), oval_path()], "--input"),
        (
            vec![
                "--track".into(),
                oval_path(),
                "--live".into(),
                "--input".into(),
                lap_input_path(),
            ],
            "--live",
        ),
        (
            vec!["--track".into(), oval_path(), "--input".into(), missing],
            "--input",
        ),
        (
            vec![
                "--track".into(),
                oval_path(),
                "--input".into(),
                lap_input_path(),
                "--summary".into(),
            ],
            "--track takes only",
        ),
    ];
    for (args, needle) in cases {
        let mut all = vec!["--body".to_string(), car_path()];
        all.extend(args.clone());
        let refs: Vec<&str> = all.iter().map(String::as_str).collect();
        let r = run(&refs);
        assert_eq!(r.code, 2, "{args:?}: {}", r.stderr);
        assert!(r.stdout.is_empty(), "{args:?}: stdout {}", r.stdout);
        assert!(
            r.stderr.contains(needle),
            "{args:?}: stderr `{}` lacks `{needle}`",
            r.stderr
        );
    }
}
