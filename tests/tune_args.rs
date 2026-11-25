mod run_tmp;

use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::atomic::{AtomicUsize, Ordering};

/// Use broadsword's runtime binary path, falling back to the cargo layout.
fn crash_sim_bin() -> PathBuf {
    if let Some(p) = std::env::var_os("CARGO_BIN_EXE_crash-sim") {
        return PathBuf::from(p);
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

struct Scratch {
    root: PathBuf,
    path: PathBuf,
}

impl Scratch {
    fn new() -> Self {
        static NEXT: AtomicUsize = AtomicUsize::new(0);
        let root = run_tmp::run_tmp().canonicalize().unwrap();
        let path = root.join(format!("tune-args-{}", NEXT.fetch_add(1, Ordering::Relaxed)));
        fs::create_dir(&path).unwrap();
        Self { root, path }
    }
}

impl Drop for Scratch {
    fn drop(&mut self) {
        let resolved = self.path.canonicalize().expect("resolve tune args scratch");
        assert_eq!(resolved.parent(), Some(self.root.as_path()));
        assert_ne!(resolved, self.root);
        fs::remove_dir_all(&resolved).expect("remove only this test's tune args directory");
    }
}

#[test]
fn dash_is_never_a_tune_flag_value() {
    let body = Path::new(env!("CARGO_MANIFEST_DIR")).join("scenarios/box_wall.bsn");
    for flag in ["--body", "--out", "--tune", "--set", "--jobs", "--show"] {
        let scratch = Scratch::new();
        let out = scratch.path.join("out");
        let mut command = Command::new(crash_sim_bin());
        command.current_dir(&scratch.path).arg("tune");
        match flag {
            "--body" => {
                command.arg("--out").arg(&out);
            }
            "--out" => {
                command.arg("--body").arg(&body);
            }
            _ => {
                command.arg("--body").arg(&body).arg("--out").arg(&out);
            }
        }
        let output = command.arg(flag).arg("--").output().unwrap();
        let stdout = String::from_utf8_lossy(&output.stdout);
        let stderr = String::from_utf8_lossy(&output.stderr);
        let diagnostic = format!(
            "flag={flag}, exit={:?}, stdout={stdout:?}, stderr={stderr:?}",
            output.status.code()
        );
        assert_eq!(output.status.code(), Some(2), "{diagnostic}");
        assert!(output.stdout.is_empty(), "{diagnostic}");
        let expected = format!("crash-sim tune: missing or invalid value for {flag}");
        assert_eq!(stderr.lines().next(), Some(expected.as_str()), "{diagnostic}");
        assert!(!out.exists(), "{diagnostic}");
        assert_eq!(fs::read_dir(&scratch.path).unwrap().count(), 0, "{diagnostic}");
    }
}
