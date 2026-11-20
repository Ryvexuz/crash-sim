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

fn bin() -> Command {
    Command::new(crash_sim_bin())
}

/// Spawn with stdout piped, drop the read end at once, and return the exit code and stderr.
fn run_with_closed_stdout(args: &[&str]) -> (Option<i32>, String) {
    let mut child = bin().args(args).stdout(Stdio::piped()).stderr(Stdio::piped()).spawn().unwrap();
    drop(child.stdout.take());
    let out = child.wait_with_output().unwrap();
    (out.status.code(), String::from_utf8_lossy(&out.stderr).into_owned())
}

#[test]
fn default_run_prints_the_csv() {
    let o = bin().output().unwrap();
    assert!(o.status.success());
    let text = String::from_utf8(o.stdout).unwrap();
    let mut lines = text.lines();
    let header = lines.next().unwrap();
    assert!(header.starts_with("t,"), "header: {header}");
    assert!(lines.count() > 1);
    assert!(text.ends_with('\n'));
    assert!(o.stderr.is_empty());
}

#[test]
fn default_run_is_deterministic_byte_for_byte() {
    let a = bin().output().unwrap().stdout;
    let b = bin().output().unwrap().stdout;
    assert_eq!(a, b);
}

#[test]
fn closed_pipe_csv_exits_nonzero_with_message() {
    let (code, err) = run_with_closed_stdout(&[]);
    assert_eq!(code, Some(1), "stderr: {err}");
    assert!(err.contains("cannot write output"), "stderr: {err}");
    assert!(!err.contains("panicked"), "stderr: {err}");
}

#[test]
fn closed_pipe_summary_exits_nonzero_without_panic() {
    let (code, err) = run_with_closed_stdout(&["--summary"]);
    assert_eq!(code, Some(1), "stderr: {err}");
    assert!(err.contains("cannot write output"), "stderr: {err}");
    assert!(!err.contains("panicked"), "stderr: {err}");
}

/// A run that goes unstable after writing samples (a very high drop) AND a closed stdout:
/// the run's own error must still reach stderr, and the exit code is 1.
#[test]
fn closed_pipe_after_failed_run_still_reports_the_run_error() {
    let (code, err) = run_with_closed_stdout(&["--drop-height", "1000", "--duration", "20"]);
    assert_eq!(code, Some(1), "stderr: {err}");
    assert!(err.contains("crash-sim: unstable at t ="), "stderr: {err}");
    assert!(!err.contains("panicked"), "stderr: {err}");
}

#[test]
fn closed_pipe_summary_after_failed_run_reports_the_run_error() {
    let (code, err) = run_with_closed_stdout(&["--drop-height", "1000", "--duration", "20", "--summary"]);
    assert_eq!(code, Some(1), "stderr: {err}");
    assert!(err.contains("crash-sim: unstable at t ="), "stderr: {err}");
}
