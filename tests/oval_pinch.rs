//! `scenarios/tracks/oval_pinch.track` (docs/STAGE7.md check 1.5): it is `oval.track` with the
//! widths of two stretches narrowed to 0.2 m, and nothing else. The first test reads the two files
//! as text; the second runs the real binary on both and checks the 1.5 rules.
//!
//! Why two stretches: the open-loop `oval_lap.input` drives lap 1 and lap 2 on slightly different
//! lines. Points 5 to 10 (bend 1, the stretch STAGE7 names) make lap 1 leave the 0.1 m half-width
//! band but lap 2 stays inside it (measured: lap 2 `off_track_s=0.000`), so 1.5's
//! `race: laps_valid=0` needs a second stretch that lap 2 leaves: points 33 to 36 (the start of
//! the back straight past gate 32 and the start of bend 2; measured lap 2 `off_track_s=0.131`).
//! The gates at points 0, 16, 32 and 48 keep the file's width 12.

use std::process::{Command, Stdio};

/// Points whose width the pinch narrows: bend 1 (points 5 to 10) and the back straight / bend 2
/// entry (points 33 to 36). Point indexes count the `point` lines from 0.
const PINCHED: [std::ops::RangeInclusive<usize>; 2] = [5..=10, 33..=36];
const PINCH_WIDTH: &str = "0.2";

fn is_pinched(index: usize) -> bool {
    PINCHED.iter().any(|r| r.contains(&index))
}

fn manifest(path: &str) -> String {
    format!("{}/{path}", env!("CARGO_MANIFEST_DIR"))
}

fn read(path: &str) -> String {
    std::fs::read_to_string(manifest(path)).unwrap_or_else(|e| panic!("read {path}: {e}"))
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

#[test]
fn oval_pinch_differs_only_in_width() {
    let oval = read("scenarios/tracks/oval.track");
    let pinch = read("scenarios/tracks/oval_pinch.track");
    assert!(
        !oval.contains('\r') && !pinch.contains('\r'),
        "no CR in track files"
    );
    let a: Vec<&str> = oval.lines().collect();
    let b: Vec<&str> = pinch.lines().collect();
    assert!(a.len() > 60, "oval.track read as {} lines", a.len());
    assert_eq!(a.len(), b.len(), "same number of lines");
    assert_eq!(
        oval.ends_with('\n'),
        pinch.ends_with('\n'),
        "same final line ending"
    );

    let mut point_index = 0usize;
    let mut changed = Vec::new();
    for (n, (x, y)) in a.iter().zip(&b).enumerate() {
        let is_point = x.starts_with("point ");
        let index = point_index;
        if is_point {
            point_index += 1;
        }
        if x == y {
            // An unchanged line must not be a pinched point (that would mean the pinch is missing).
            assert!(
                !(is_point && is_pinched(index)),
                "line {}: point {index} should be pinched but is unchanged: {y}",
                n + 1
            );
            continue;
        }
        assert!(
            is_point,
            "line {} differs but is not a point: {x} / {y}",
            n + 1
        );
        assert!(
            is_pinched(index),
            "line {}: point {index} differs but is not pinched: {y}",
            n + 1
        );
        // The only change is the per-point width suffix, appended after the same x and z text.
        assert_eq!(
            **y,
            format!("{x} width {PINCH_WIDTH}"),
            "line {}: only a `width {PINCH_WIDTH}` suffix may be added",
            n + 1
        );
        assert_eq!(x.split(' ').count(), 3, "oval point has x and z only: {x}");
        changed.push(index);
    }
    assert_eq!(
        changed,
        PINCHED.iter().cloned().flatten().collect::<Vec<_>>(),
        "exactly points 5 to 10 and 33 to 36 changed"
    );
    assert_eq!(point_index, 64, "oval.track has 64 points");
    assert!(
        a.contains(&"width 12") && b.contains(&"width 12"),
        "the file's own width stays 12"
    );
}

struct Run {
    code: i32,
    stdout: String,
    stderr: String,
}

fn lap_run(track: &str) -> Run {
    let out = Command::new(crash_sim_bin())
        .args([
            "--body",
            &manifest("scenarios/car_basic.bsn"),
            "--track",
            &manifest(track),
            "--input",
            &manifest("scenarios/tracks/oval_lap.input"),
        ])
        .stdin(Stdio::null())
        .output()
        .expect("run crash-sim");
    Run {
        code: out.status.code().expect("exit code"),
        stdout: String::from_utf8(out.stdout).expect("stdout is UTF-8"),
        stderr: String::from_utf8(out.stderr).expect("stderr is UTF-8"),
    }
}

/// The value of `key=` on a race line.
fn field<'a>(line: &'a str, key: &str) -> &'a str {
    let pat = format!("{key}=");
    line.split(' ')
        .find_map(|w| w.strip_prefix(pat.as_str()))
        .unwrap_or_else(|| panic!("no `{key}=` on `{line}`"))
}

fn lines_starting<'a>(text: &'a str, prefix: &str) -> Vec<&'a str> {
    text.lines().filter(|l| l.starts_with(prefix)).collect()
}

/// docs/STAGE7.md check 1.5: the lap is invalid because it leaves the narrowed stretch, and
/// nothing else about the run (times, grid, checkpoints, final state) changes.
#[test]
fn oval_pinch_lap_is_off_track_and_nothing_else_changes() {
    let base = lap_run("scenarios/tracks/oval.track");
    let pinch = lap_run("scenarios/tracks/oval_pinch.track");
    assert_eq!(base.code, 0, "oval stderr: {}", base.stderr);
    assert_eq!(pinch.code, 0, "pinch stderr: {}", pinch.stderr);

    let track = pinch
        .stdout
        .lines()
        .next()
        .expect("pinch run printed nothing");
    assert!(
        track.starts_with("track ") && track.ends_with(" width_min_m=0.20 width_max_m=12.00"),
        "track line: {track}"
    );

    let base_laps = lines_starting(&base.stdout, "lap n=1 ");
    let pinch_laps = lines_starting(&pinch.stdout, "lap n=1 ");
    assert_eq!(base_laps.len(), 1, "oval run lap 1:\n{}", base.stdout);
    assert_eq!(pinch_laps.len(), 1, "pinch run lap 1:\n{}", pinch.stdout);
    let (bl, pl) = (base_laps[0], pinch_laps[0]);
    assert_eq!(field(bl, "valid"), "yes", "oval lap 1 is valid: {bl}");
    assert_eq!(field(pl, "valid"), "no", "{pl}");
    assert_eq!(field(pl, "reason"), "off_track", "{pl}");
    let off: f64 = field(pl, "off_track_s")
        .parse()
        .unwrap_or_else(|_| panic!("off_track_s is not a number: {pl}"));
    assert!(off > 0.0, "off_track_s must be above 0.000: {pl}");
    for key in ["start_t", "end_t", "time_s"] {
        assert_eq!(field(pl, key), field(bl, key), "{key}: {pl} / {bl}");
    }

    // Start, checkpoint and final lines are the same text in both runs.
    for prefix in ["start ", "checkpoint ", "final "] {
        let (x, y) = (
            lines_starting(&base.stdout, prefix),
            lines_starting(&pinch.stdout, prefix),
        );
        assert!(!x.is_empty(), "oval run has no `{prefix}` line");
        assert_eq!(x, y, "`{prefix}` lines differ");
    }

    let last = pinch.stdout.lines().last().unwrap();
    assert!(last.starts_with("race: laps_valid=0"), "last line: {last}");
}
