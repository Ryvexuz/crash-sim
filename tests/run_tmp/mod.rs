//! The compile-time temporary folder is shared by every worktree and run.
//! Isolate scratch files by process so concurrent runs cannot delete each other's files.

pub fn run_tmp() -> std::path::PathBuf {
    let dir = std::path::PathBuf::from(env!("CARGO_TARGET_TMPDIR"))
        .join(format!("run-{}", std::process::id()));
    std::fs::create_dir_all(&dir)
        .unwrap_or_else(|e| panic!("cannot create test folder {}: {e}", dir.display()));
    dir
}
