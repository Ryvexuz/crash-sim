//! The live protocol (docs/STAGE6.md, "The live protocol"): `crash-sim --body <car> --live` reads
//! input lines from stdin as they arrive and writes the rows of a drive to stdout, flushed per
//! output step. These tests run the real binary.

use std::io::{BufRead, BufReader, Write};
use std::process::{Command, Stdio};
use std::sync::mpsc;
use std::time::Duration;

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

fn car_path() -> String {
    format!("{}/scenarios/car_basic.bsn", env!("CARGO_MANIFEST_DIR"))
}

/// A file in the build's own temp folder (the test binary's, not the repo).
fn tmp_file(name: &str, text: &str) -> String {
    let dir = std::path::PathBuf::from(env!("CARGO_TARGET_TMPDIR"));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join(name);
    std::fs::write(&path, text).unwrap();
    path.to_string_lossy().into_owned()
}

fn live_command() -> Command {
    let mut c = Command::new(crash_sim_bin());
    c.args(["--body", &car_path(), "--live"])
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    c
}

const LINES: &str = "0 0 0 0\n1 0.6 0 0\n2.5 0.3 0 -0.7\n4 0 0.5 0.2\n";

/// The same lines piped live (one line at a time, flushed) and given as `--input` (no
/// `--duration`, so the file run ends at its last line) give the same bytes.
#[test]
fn live_matches_file_input_bytes() {
    let path = tmp_file("live_same.input", LINES);
    let file = Command::new(crash_sim_bin())
        .args(["--body", &car_path(), "--input", &path])
        .output()
        .unwrap();
    assert_eq!(file.status.code(), Some(0));
    assert!(
        file.stdout.iter().filter(|b| **b == b'\n').count() > 100,
        "a drive prints its rows"
    );

    let mut child = live_command().spawn().unwrap();
    let mut stdin = child.stdin.take().unwrap();
    for line in LINES.split_inclusive('\n') {
        stdin.write_all(line.as_bytes()).unwrap();
        stdin.flush().unwrap();
    }
    drop(stdin); // end of input ends the run at the last line
    let live = child.wait_with_output().unwrap();
    assert_eq!(live.status.code(), Some(0));
    assert!(
        live.stderr.is_empty(),
        "{:?}",
        String::from_utf8_lossy(&live.stderr)
    );
    assert_eq!(
        String::from_utf8_lossy(&live.stdout),
        String::from_utf8_lossy(&file.stdout),
        "the same lines live and as --input must give the same bytes"
    );
    assert!(live.stdout == file.stdout);
    let text = String::from_utf8(live.stdout).unwrap();
    assert!(text.starts_with("t,speed_mps,x,y,z,heading_deg\n0.0000,"));
    assert!(
        text.lines().last().unwrap().starts_with("final t=4.0000 "),
        "{text}"
    );
}

/// Rows arrive while stdin is still open, and a row is not written before the line that lets the
/// sim reach it: the reader gets the rows up to the last time sent, and only after the next line
/// the rows after it.
#[test]
fn live_rows_arrive_before_end_of_input() {
    let mut child = live_command().spawn().unwrap();
    let mut stdin = child.stdin.take().unwrap();
    let stdout = child.stdout.take().unwrap();
    let (tx, rx) = mpsc::channel::<String>();
    std::thread::spawn(move || {
        for line in BufReader::new(stdout).lines() {
            if tx.send(line.unwrap()).is_err() {
                break;
            }
        }
    });
    let next = |rx: &mpsc::Receiver<String>| {
        rx.recv_timeout(Duration::from_secs(60))
            .expect("a row must arrive while stdin is open (flushed per output step)")
    };
    let time = |row: &str| -> f64 { row.split(',').next().unwrap().parse().unwrap() };

    stdin.write_all(b"0 0 0 0\n").unwrap();
    stdin.flush().unwrap();
    assert_eq!(next(&rx), "t,speed_mps,x,y,z,heading_deg");
    assert_eq!(time(&next(&rx)), 0.0);
    // Nothing more may be there: the sim has no time beyond 0.
    assert!(
        rx.recv_timeout(Duration::from_millis(500)).is_err(),
        "the sim ran past the last time received"
    );

    stdin.write_all(b"0.5 0.2 0 0\n").unwrap();
    stdin.flush().unwrap();
    let mut last = 0.0;
    while last < 0.5 - 1e-9 {
        last = time(&next(&rx));
    }
    assert!((last - 0.5).abs() < 1e-9, "last row at {last}");
    assert!(
        rx.recv_timeout(Duration::from_millis(500)).is_err(),
        "the sim ran past the last time received"
    );

    stdin.write_all(b"0.52 0.2 0 0\n").unwrap();
    stdin.flush().unwrap();
    assert!((time(&next(&rx)) - 0.51).abs() < 1e-9);
    assert!((time(&next(&rx)) - 0.52).abs() < 1e-9);

    drop(stdin);
    let rest: Vec<String> = rx.iter().collect();
    assert!(
        rest.last().unwrap().starts_with("final t=0.5200 "),
        "{rest:?}"
    );
    assert_eq!(child.wait().unwrap().code(), Some(0));
}

/// A bad line is an error that names the line, exit 2; the rows before it were already written.
#[test]
fn live_bad_line_exits_2_naming_the_line() {
    let mut child = live_command().spawn().unwrap();
    {
        let mut stdin = child.stdin.take().unwrap();
        // Line 3 is blank (counts), line 4 has a brake of 2.
        stdin
            .write_all(b"0 0 0 0\n0.5 0.2 0 0\n\n1 0 2 0\n")
            .unwrap();
    }
    let o = child.wait_with_output().unwrap();
    let err = String::from_utf8_lossy(&o.stderr).into_owned();
    assert_eq!(o.status.code(), Some(2), "{err}");
    assert!(
        err.contains("input line 4: brake 2 must be from 0 to 1"),
        "stderr must name the line: {err}"
    );
    let out = String::from_utf8_lossy(&o.stdout);
    assert!(out.starts_with("t,speed_mps,x,y,z,heading_deg\n"));
    assert!(
        !out.contains("final "),
        "a refused run has no final line: {out}"
    );
}

/// A closed stdout ends the run quietly: exit 0 and nothing on stderr.
#[test]
fn live_closed_stdout_ends_quietly() {
    let mut child = live_command().spawn().unwrap();
    drop(child.stdout.take().unwrap()); // the reader goes away
    let mut stdin = child.stdin.take().unwrap();
    // Many rows to write; ignore a write error, the child may already have gone.
    let _ = stdin.write_all(b"0 0 0 0\n3 0.5 0 0\n");
    let _ = stdin.flush();
    let o = {
        drop(stdin);
        child.wait_with_output().unwrap()
    };
    assert_eq!(o.status.code(), Some(0));
    assert!(
        o.stderr.is_empty(),
        "{:?}",
        String::from_utf8_lossy(&o.stderr)
    );
}
