//! The racing save file: v1 -> v2 migration, v2 round trip, every refusal by line and words,
//! `record_lap`, and the atomic write (a failed write leaves the old file unchanged).

#[path = "../src/save.rs"]
mod save;

use save::{write_atomic, Best, Save, SaveError, UNKNOWN};
use std::path::PathBuf;

/// A fresh empty folder under cargo's per-target temp dir (shared build folder, not the repo).
fn dir(name: &str) -> PathBuf {
    let d = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join(format!("save-{name}"));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).unwrap();
    d
}

/// The committed v1 file. A Windows checkout may have turned LF into CRLF (there is no
/// .gitattributes rule for `*.save`), which the strict parser refuses by design: undo only that.
fn fixture() -> String {
    let path = format!("{}/tests/data/save_v1.save", env!("CARGO_MANIFEST_DIR"));
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}")).replace("\r\n", "\n")
}

fn refused(text: &str) -> (usize, String) {
    match Save::parse(text) {
        Err(SaveError::Parse { line, message }) => (line, message),
        other => panic!("expected a parse refusal for {text:?}, got {other:?}"),
    }
}

fn refusal(text: &str, line: usize, words: &str) {
    let (l, m) = refused(text);
    assert_eq!(l, line, "line for {text:?}: {m}");
    assert!(m.contains(words), "message {m:?} should contain {words:?}");
}

// ---------------------------------------------------------------------------------------
// Migration and round trip

#[test]
fn v1_fixture_loads_and_resaves_as_v2() {
    let s = Save::parse(&fixture()).expect("fixture parses");
    assert_eq!(s.best.len(), 3);
    let m = &s.best["monza"];
    assert_eq!(*m, Best { seconds: 83.412, car: UNKNOWN.into(), date: UNKNOWN.into() });
    assert_eq!(s.best["alpine"].seconds, 71.5);
    assert_eq!(s.best["zandvoort"].seconds, 90.0);
    assert_eq!(s.best["zandvoort"].car, "unknown");

    let d = dir("migrate");
    let p = d.join("a.save");
    s.save(&p).unwrap();
    let written = std::fs::read_to_string(&p).unwrap();
    assert_eq!(
        written,
        "save 2\nbest alpine 71.5 unknown unknown\nbest monza 83.412 unknown unknown\nbest zandvoort 90 unknown unknown\n"
    );
    assert_eq!(Save::load(&p).unwrap(), s);
    assert!(!save::temp_path(&p).exists(), "temp file left behind");
}

#[test]
fn v2_round_trip_is_byte_identical() {
    let text = "save 2\nbest alpine 71.5 gt-3 2026-11-19\nbest monza 83.412 unknown unknown\nbest oval 0.0000001 kart 2024-02-29\n";
    let s = Save::parse(text).unwrap();
    assert_eq!(s.to_text(), text);
    assert_eq!(Save::parse(&s.to_text()).unwrap(), s);

    let d = dir("roundtrip");
    let p = d.join("r.save");
    s.save(&p).unwrap();
    assert_eq!(std::fs::read(&p).unwrap(), text.as_bytes());
    // A header-only file (no laps yet) is valid and round-trips.
    let empty = Save::parse("save 2\n").unwrap();
    assert!(empty.best.is_empty());
    assert_eq!(empty.to_text(), "save 2\n");
}

// ---------------------------------------------------------------------------------------
// Refusals

#[test]
fn refuses_bad_headers_and_endings() {
    refusal("", 1, "empty file");
    refusal("best a 1\n", 1, "first line must be");
    refusal("save 3\n", 1, "unsupported version");
    refusal("save 0\n", 1, "unsupported version");
    refusal("save 2", 1, "newline");
    refusal("save 2\nbest a 1 c 2026-01-01", 2, "newline");
    refusal("save 2\r\n", 1, "carriage return");
    refusal("save 1\nbest a 1\r\n", 2, "carriage return");
    refusal("save 2\n\n", 2, "unknown line");
}

#[test]
fn refuses_bad_lines() {
    refusal("save 2\nlap a 1 c 2026-01-01\n", 2, "expected `best");
    refusal("save 2\nbest a 1\n", 2, "found 3 fields");
    refusal("save 1\nbest a 1 c 2026-01-01\n", 2, "found 5 fields");
    refusal("save 2\nbest a 1 c 2026-01-01 extra\n", 2, "found 6 fields");
    refusal("save 2\nbest  a 1 c 2026-01-01\n", 2, "found 6 fields");
    refusal("save 2\nbest a 1 c 2026-01-01 \n", 2, "found 6 fields");
    refusal("save 2\nbest a 1 c 2026-01-01\nbest a 2 c 2026-01-01\n", 3, "duplicate track");
    refusal("save 1\nbest a 1\nbest b 2\nbest a 3\n", 4, "duplicate track");
    refusal("save 2\nbest a\u{e9} 1 c 2026-01-01\n", 2, "printable ASCII");
}

#[test]
fn refuses_bad_seconds() {
    for bad in ["0", "0.0", "-1", "-0.5", "inf", "NaN", "nan", "infinity", "+5", "1e3", "1.", ".5", "1,5", "abc", ""] {
        let text = format!("save 2\nbest a {bad} c 2026-01-01\n");
        let (line, m) = refused(&text);
        assert_eq!(line, 2, "{bad:?}: {m}");
        assert!(m.contains("seconds") || m.contains("fields"), "{bad:?}: {m}");
    }
    refusal("save 1\nbest a 5\nbest b 0\n", 3, "must be positive");
    refusal("save 1\nbest a 1\nbest b 1e999\n", 3, "plain decimal");
    // A decimal so large it overflows f64 to infinity.
    let huge = "9".repeat(400);
    refusal(&format!("save 1\nbest a {huge}\n"), 2, "must be finite");
}

#[test]
fn refuses_bad_dates() {
    for bad in ["2026-13-01", "2026-00-10", "2026-02-30", "2025-02-29", "2026-1-1", "26-01-01", "2026/01/01", "2026-01-00", "0000-01-01", "Unknown", "2026-01-0a", "2026-04-31"] {
        refusal(&format!("save 2\nbest a 1 c {bad}\n"), 2, "date");
    }
    // Calendar edge that is valid.
    Save::parse("save 2\nbest a 1 c 2024-02-29\n").unwrap();
}

// ---------------------------------------------------------------------------------------
// record_lap

#[test]
fn record_lap_keeps_the_best() {
    let mut s = Save::new();
    assert!(s.record_lap("monza", 90.0, "gt", "2026-11-01").unwrap());
    assert!(!s.record_lap("monza", 95.0, "kart", "2026-11-02").unwrap(), "slower must not replace");
    assert!(!s.record_lap("monza", 90.0, "kart", "2026-11-02").unwrap(), "a tie keeps the old entry");
    assert_eq!(s.best["monza"], Best { seconds: 90.0, car: "gt".into(), date: "2026-11-01".into() });
    assert!(s.record_lap("monza", 89.25, "kart", "2026-11-03").unwrap());
    assert_eq!(s.best["monza"], Best { seconds: 89.25, car: "kart".into(), date: "2026-11-03".into() });
    assert!(s.record_lap("oval", 20.0, "kart", "unknown").unwrap());
    assert_eq!(s.best.len(), 2);
    // A migrated v1 entry is beaten like any other.
    let mut old = Save::parse(&fixture()).unwrap();
    assert!(old.record_lap("alpine", 70.0, "gt", "2026-11-19").unwrap());
    assert!(!old.record_lap("monza", 83.412, "gt", "2026-11-19").unwrap());
    assert_eq!(old.best["monza"].car, "unknown");
    assert_eq!(Save::parse(&old.to_text()).unwrap(), old);
}

#[test]
fn record_lap_refuses_bad_values_and_changes_nothing() {
    let mut s = Save::new();
    s.record_lap("monza", 90.0, "gt", "2026-11-01").unwrap();
    let before = s.clone();
    for secs in [0.0, -1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(matches!(s.record_lap("monza", secs, "gt", "2026-11-01"), Err(SaveError::Invalid(_))), "{secs}");
        assert!(matches!(s.record_lap("new", secs, "gt", "2026-11-01"), Err(SaveError::Invalid(_))), "{secs}");
    }
    assert!(s.record_lap("", 1.0, "gt", "2026-11-01").is_err());
    assert!(s.record_lap("a b", 1.0, "gt", "2026-11-01").is_err());
    assert!(s.record_lap("a", 1.0, "", "2026-11-01").is_err());
    assert!(s.record_lap("a", 1.0, "g t", "2026-11-01").is_err());
    assert!(s.record_lap("a", 1.0, "gt", "2026-02-30").is_err());
    assert!(s.record_lap("a", 1.0, "gt", "").is_err());
    assert_eq!(s, before);
}

// ---------------------------------------------------------------------------------------
// Atomic write

#[test]
fn failed_write_leaves_old_file_unchanged() {
    let d = dir("failed");
    let p = d.join("game.save");
    let mut s = Save::new();
    s.record_lap("monza", 90.0, "gt", "2026-11-01").unwrap();
    s.save(&p).unwrap();
    let old = std::fs::read(&p).unwrap();

    // Injected error: a directory sits where the temp file must go, so creating it fails.
    let tmp = save::temp_path(&p);
    std::fs::create_dir(&tmp).unwrap();
    s.record_lap("monza", 80.0, "kart", "2026-11-02").unwrap();
    match s.save(&p) {
        Err(SaveError::Io { path, .. }) => assert_eq!(path, tmp),
        other => panic!("expected an Io error, got {other:?}"),
    }
    assert_eq!(std::fs::read(&p).unwrap(), old, "old file must be untouched");
    assert!(tmp.is_dir(), "the blocker must not be removed");
    std::fs::remove_dir(&tmp).unwrap();

    // The same write succeeds once the blocker is gone, and replaces the old file.
    s.save(&p).unwrap();
    assert_eq!(Save::load(&p).unwrap().best["monza"].seconds, 80.0);
    assert!(!tmp.exists());
}

#[test]
fn failed_rename_leaves_target_and_no_temp() {
    let d = dir("rename");
    // The target is a non-empty directory: the data file is written, but the rename cannot
    // replace a directory, and the temp file must not be left behind.
    let target = d.join("game.save");
    std::fs::create_dir(&target).unwrap();
    std::fs::write(target.join("keep"), b"x").unwrap();
    let err = write_atomic(&target, b"save 2\n").unwrap_err();
    assert!(matches!(err, SaveError::Io { .. }), "{err:?}");
    assert!(target.join("keep").exists());
    assert!(!save::temp_path(&target).exists(), "temp file left behind after a failed rename");
}

#[test]
fn leftover_temp_from_a_crash_is_replaced() {
    let d = dir("leftover");
    let p = d.join("game.save");
    std::fs::write(save::temp_path(&p), b"half a wri").unwrap();
    let mut s = Save::new();
    s.record_lap("a", 1.5, "c", "2026-01-01").unwrap();
    s.save(&p).unwrap();
    assert_eq!(std::fs::read_to_string(&p).unwrap(), "save 2\nbest a 1.5 c 2026-01-01\n");
    assert!(!save::temp_path(&p).exists());
}

#[test]
fn load_missing_file_is_an_io_error() {
    let d = dir("missing");
    assert!(matches!(Save::load(&d.join("nope.save")), Err(SaveError::Io { .. })));
}
