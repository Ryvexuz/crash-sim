use std::path::Path;

mod run_tmp;

#[test]
fn run_tmp_is_per_process() {
    let d = run_tmp::run_tmp();
    assert!(d.is_dir(), "missing test folder {}", d.display());
    assert_eq!(
        d.file_name().unwrap(),
        format!("run-{}", std::process::id()).as_str()
    );
    assert_eq!(
        d.parent(),
        Some(Path::new(env!(concat!("CARGO_TARGET_", "TMPDIR"))))
    );
}

#[test]
fn no_test_names_the_shared_tmp_folder() {
    let word = ["CARGO_TARGET_", "TMPDIR"].concat();
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests");
    let helper = root.join("run_tmp").join("mod.rs");
    let mut pending = vec![root];
    let mut read = 0;
    let mut offenders = Vec::new();
    while let Some(dir) = pending.pop() {
        for entry in std::fs::read_dir(&dir)
            .unwrap_or_else(|e| panic!("cannot read directory {}: {e}", dir.display()))
        {
            let entry = entry.unwrap_or_else(|e| panic!("entry in {}: {e}", dir.display()));
            let path = entry.path();
            let kind = entry
                .file_type()
                .unwrap_or_else(|e| panic!("cannot inspect {}: {e}", path.display()));
            assert!(!kind.is_symlink(), "unexpected link {}", path.display());
            if kind.is_dir() {
                pending.push(path);
            } else if path.extension().is_some_and(|ext| ext == "rs") {
                let text = std::fs::read_to_string(&path)
                    .unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()));
                read += 1;
                if path != helper && text.contains(&word) {
                    offenders.push(path.display().to_string());
                }
            }
        }
    }
    offenders.sort();
    assert!(
        offenders.is_empty(),
        "tests naming the shared temporary folder:\n{}",
        offenders.join("\n")
    );
    assert!(read >= 18, "expected at least 18 Rust test files, read {read}");
}
