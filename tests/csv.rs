use crash_sim::{run, to_csv, Config, CSV_HEADER};
use std::process::Command;

#[test]
fn csv_header_and_rows_are_well_formed() {
    let cfg = Config::default();
    let samples = run(&cfg);
    let csv = to_csv(&samples);
    let mut lines = csv.lines();
    assert_eq!(lines.next(), Some(CSV_HEADER));
    let rows: Vec<&str> = lines.collect();
    assert_eq!(rows.len(), samples.len(), "one line per sample");
    assert!(!rows.is_empty());
    for (i, row) in rows.iter().enumerate() {
        let fields: Vec<&str> = row.split(',').collect();
        assert_eq!(fields.len(), 4, "row {i} has {} fields: {row}", fields.len());
        for f in fields {
            let v: f64 = f.parse().unwrap_or_else(|_| panic!("row {i}: field {f:?} not a number"));
            assert!(v.is_finite(), "row {i}: field {f:?} not finite");
        }
    }
}

#[test]
fn csv_time_column_steps_by_dt() {
    let cfg = Config::default();
    let csv = to_csv(&run(&cfg));
    let ts: Vec<f64> = csv
        .lines()
        .skip(1)
        .map(|l| l.split(',').next().unwrap().parse().unwrap())
        .collect();
    assert!(ts.len() > 1);
    for w in ts.windows(2) {
        assert!(w[1] > w[0], "t not strictly increasing: {} then {}", w[0], w[1]);
        assert!(((w[1] - w[0]) - cfg.dt).abs() < 1e-9, "step {} != dt {}", w[1] - w[0], cfg.dt);
    }
}

#[test]
fn binary_stdout_matches_library_csv() {
    let out = Command::new(crash_sim_bin()).output().expect("run binary");
    assert!(out.status.success(), "binary failed: {:?}", out.status);
    let expected = to_csv(&run(&Config::default()));
    assert_eq!(String::from_utf8(out.stdout).expect("utf8 stdout"), expected);
}

#[test]
fn binary_rejects_invalid_argument_with_empty_stdout() {
    let out = Command::new(crash_sim_bin())
        .args(["--gravity", "0"])
        .output()
        .expect("run binary");
    assert!(!out.status.success(), "expected non-zero exit");
    assert!(out.stdout.is_empty(), "stdout must be empty, got {:?}", String::from_utf8_lossy(&out.stdout));
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
