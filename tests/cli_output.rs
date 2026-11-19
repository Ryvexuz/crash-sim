use std::process::{Command, Stdio};

fn bin() -> Command {
    Command::new(env!("CARGO_BIN_EXE_crash-sim"))
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
