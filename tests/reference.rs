//! Tests for the reference module (strict pulse and meta reader, resample, comparison). The module
//! is included by path because `src/lib.rs` does not register it yet.
#![allow(dead_code)]

#[path = "../src/reference.rs"]
mod reference;

use reference::*;
use std::path::PathBuf;

const GOOD_PULSE: &str = "tests/data/reference/good.pulse.csv";
const GOOD_META: &str = "tests/data/reference/good.meta";

fn root(rel: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(rel)
}

/// A pulse text with `n` samples at 1 ms, acceleration `a(i)`.
fn text(n: usize, a: impl Fn(usize) -> f64) -> String {
    let mut s = String::from("t_s,accel_mps2\n");
    for i in 0..n {
        s += &format!("{:.6},{:.6}\n", i as f64 * 0.001, a(i));
    }
    s
}

/// Replace one 1-based line of a text.
fn with_line(t: &str, line: usize, new: &str) -> String {
    t.lines()
        .enumerate()
        .map(|(k, l)| if k + 1 == line { new } else { l })
        .collect::<Vec<_>>()
        .join("\n")
        + "\n"
}

#[test]
fn good_files_load() {
    let p = load_pulse(root(GOOD_PULSE)).unwrap();
    assert_eq!(p.accel.len(), 12);
    assert!((p.t0 + 0.002).abs() < 1e-12);
    assert!((p.dt - 0.001).abs() < 1e-12);
    assert_eq!(p.accel[6], -400.0);
    let m = load_meta(root(GOOD_META)).unwrap();
    assert_eq!(m.test, "0000");
    assert_eq!(m.mass_kg, 1300.0);
    assert_eq!(m.speed_kmh, 56.0);
    assert_eq!(m.sample_rate_hz, 1000.0);
    assert_eq!(m.static_crush_m, Some(0.35));
    assert_eq!(m.unknown_keys, vec!["note".to_string()]);
}

#[test]
fn missing_file_is_io_error() {
    assert!(matches!(
        load_pulse(root("tests/data/reference/nope.csv")),
        Err(RefError::Io(_))
    ));
    assert!(matches!(
        load_meta(root("tests/data/reference/nope.meta")),
        Err(RefError::Io(_))
    ));
}

#[test]
fn wrong_header_refused_on_line_1() {
    let t = text(12, |_| 0.0).replacen("t_s,accel_mps2", "t,a", 1);
    let e = parse_pulse(&t).unwrap_err();
    assert!(matches!(e, RefError::BadHeader { .. }), "{e:?}");
    assert!(e.to_string().starts_with("line 1:"), "{e}");
    assert!(matches!(parse_pulse(""), Err(RefError::BadHeader { .. })));
}

#[test]
fn bad_row_refused_with_line() {
    let base = text(12, |i| i as f64);
    let e = parse_pulse(&with_line(&base, 5, "0.003000,abc")).unwrap_err();
    assert!(matches!(e, RefError::BadRow { line: 5, .. }), "{e:?}");
    let e = parse_pulse(&with_line(&base, 7, "0.005000")).unwrap_err();
    assert!(matches!(e, RefError::BadRow { line: 7, .. }), "{e:?}");
    let e = parse_pulse(&with_line(&base, 4, "")).unwrap_err();
    assert!(matches!(e, RefError::BadRow { line: 4, .. }), "{e:?}");
}

#[test]
fn non_finite_refused_with_line() {
    let base = text(12, |i| i as f64);
    for bad in ["NaN", "inf", "-inf"] {
        let e = parse_pulse(&with_line(&base, 6, &format!("0.004000,{bad}"))).unwrap_err();
        assert_eq!(e, RefError::NonFinite { line: 6 }, "{bad}");
        let e = parse_pulse(&with_line(&base, 3, &format!("{bad},1.0"))).unwrap_err();
        assert_eq!(e, RefError::NonFinite { line: 3 }, "{bad}");
    }
}

#[test]
fn non_increasing_time_refused_with_line() {
    let base = text(12, |i| i as f64);
    // Equal to the previous time (line 5 is sample 3 at 0.003).
    let e = parse_pulse(&with_line(&base, 6, "0.003000,1.0")).unwrap_err();
    assert_eq!(e, RefError::NonIncreasing { line: 6 });
    // Going back.
    let e = parse_pulse(&with_line(&base, 9, "0.001000,1.0")).unwrap_err();
    assert_eq!(e, RefError::NonIncreasing { line: 9 });
}

#[test]
fn non_uniform_time_refused_with_line_and_tolerance() {
    let base = text(12, |i| i as f64);
    // Sample 5 (line 7) moved by 0.5 ms of a 1 ms step: still increasing, not uniform.
    let e = parse_pulse(&with_line(&base, 7, "0.005500,1.0")).unwrap_err();
    match &e {
        RefError::NonUniform {
            line,
            deviation_s,
            tolerance_s,
        } => {
            assert_eq!(*line, 7);
            assert!((deviation_s - 0.0005).abs() < 1e-9, "{deviation_s}");
            assert!(
                (tolerance_s - UNIFORM_TOL_FRAC * 0.001).abs() < 1e-9,
                "{tolerance_s}"
            );
        }
        other => panic!("{other:?}"),
    }
    assert!(e.to_string().contains("tolerance"), "{e}");
    // A deviation inside the tolerance (1% of the step) is accepted.
    assert!(parse_pulse(&with_line(&base, 7, "0.004990,1.0")).is_ok());
    // Just outside it (3%) is not.
    assert!(matches!(
        parse_pulse(&with_line(&base, 7, "0.004970,1.0")),
        Err(RefError::NonUniform { line: 7, .. })
    ));
}

#[test]
fn rounded_six_decimal_times_at_10khz_are_uniform() {
    // 3200 samples at 0.1 ms written with 6 decimals, as n20-02 writes them.
    let mut s = String::from("t_s,accel_mps2\n");
    for i in 0..3200 {
        s += &format!("{:.6},{:.6}\n", -0.02 + i as f64 * 1e-4, 0.0);
    }
    let p = parse_pulse(&s).unwrap();
    assert_eq!(p.accel.len(), 3200);
    assert!((p.dt - 1e-4).abs() < 1e-9);
}

#[test]
fn too_few_samples_refused() {
    let e = parse_pulse(&text(9, |_| 0.0)).unwrap_err();
    assert_eq!(
        e,
        RefError::TooFewSamples {
            found: 9,
            last_line: 10
        }
    );
    assert!(parse_pulse(&text(10, |_| 0.0)).is_ok());
    assert!(matches!(
        parse_pulse("t_s,accel_mps2\n"),
        Err(RefError::TooFewSamples { found: 0, .. })
    ));
}

#[test]
fn crlf_files_load() {
    let t = text(12, |i| i as f64).replace('\n', "\r\n");
    assert_eq!(parse_pulse(&t).unwrap().accel.len(), 12);
}

#[test]
fn load_pulse_reads_a_temp_file() {
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR"));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("reference_test_pulse.csv");
    std::fs::write(&path, with_line(&text(12, |i| i as f64), 9, "0.001000,1.0")).unwrap();
    let e = load_pulse(&path).unwrap_err();
    assert_eq!(e, RefError::NonIncreasing { line: 9 });
    std::fs::write(&path, text(12, |i| i as f64)).unwrap();
    assert_eq!(load_pulse(&path).unwrap().accel[11], 11.0);
}

const META_OK: &str =
    "test = 1\nmass_kg = 1300\nspeed_kmh = 56\nchannel = c\nsample_rate_hz = 10000\n";

#[test]
fn meta_minimal_ok_no_unknown_keys() {
    let m = parse_meta(META_OK).unwrap();
    assert!(m.unknown_keys.is_empty());
    assert_eq!(m.static_crush_m, None);
}

#[test]
fn meta_missing_required_key_refused_each() {
    for key in REQUIRED_KEYS {
        let t: String = META_OK
            .lines()
            .filter(|l| !l.starts_with(key))
            .map(|l| format!("{l}\n"))
            .collect();
        assert_eq!(
            parse_meta(&t).unwrap_err(),
            RefError::MissingKey(key.to_string()),
            "{key}"
        );
    }
}

#[test]
fn meta_bad_lines_refused_with_line() {
    assert_eq!(
        parse_meta(&format!("{META_OK}junk\n")).unwrap_err(),
        RefError::BadMetaLine { line: 6 }
    );
    assert_eq!(
        parse_meta(&format!("{META_OK}= 3\n")).unwrap_err(),
        RefError::BadMetaLine { line: 6 }
    );
    assert!(matches!(
        parse_meta(&format!("{META_OK}extra =\n")).unwrap_err(),
        RefError::BadValue { line: 6, .. }
    ));
    assert_eq!(
        parse_meta(&format!("{META_OK}mass_kg = 5\n")).unwrap_err(),
        RefError::DuplicateKey {
            line: 6,
            key: "mass_kg".into()
        }
    );
}

#[test]
fn meta_bad_numbers_refused_with_line() {
    for (k, bad) in [
        ("mass_kg", "heavy"),
        ("mass_kg", "0"),
        ("speed_kmh", "-3"),
        ("sample_rate_hz", "inf"),
        ("speed_kmh", "NaN"),
    ] {
        let t = META_OK
            .lines()
            .map(|l| {
                if l.starts_with(k) {
                    format!("{k} = {bad}\n")
                } else {
                    format!("{l}\n")
                }
            })
            .collect::<String>();
        let e = parse_meta(&t).unwrap_err();
        assert!(
            matches!(&e, RefError::BadValue { key, .. } if key == k),
            "{k}={bad}: {e:?}"
        );
    }
    let e = parse_meta(&format!("{META_OK}static_crush_m = -1\n")).unwrap_err();
    assert!(matches!(e, RefError::BadValue { line: 6, .. }), "{e:?}");
}

#[test]
fn meta_comments_blank_lines_and_line_numbers() {
    let t = format!("# a comment\n\n{META_OK}bogus line\n");
    assert_eq!(
        parse_meta(&t).unwrap_err(),
        RefError::BadMetaLine { line: 8 }
    );
    assert!(parse_meta(&format!("# c\n\n{META_OK}")).is_ok());
}

#[test]
fn meta_unknown_keys_listed_sorted() {
    let m = parse_meta(&format!("{META_OK}zeta = 1\nalpha = 2\nurl = x\n")).unwrap();
    assert_eq!(
        m.unknown_keys,
        vec!["alpha".to_string(), "zeta".to_string()]
    );
}

fn linear_pulse(n: usize) -> Pulse {
    // a(t) = 3 + 200 t on t0 = -0.01, dt = 0.001
    let (t0, dt) = (-0.01, 0.001);
    Pulse {
        t0,
        dt,
        accel: (0..n).map(|i| 3.0 + 200.0 * (t0 + i as f64 * dt)).collect(),
    }
}

#[test]
fn resample_is_exact_on_a_linear_signal() {
    let p = linear_pulse(101);
    for dt in [0.0004, 0.0001, 0.00073, 0.002] {
        let r = resample(&p, dt).unwrap();
        for (k, a) in r.accel.iter().enumerate() {
            let t = r.time(k);
            assert!(
                (a - (3.0 + 200.0 * t)).abs() < 1e-9,
                "dt {dt} k {k}: {a} vs {}",
                3.0 + 200.0 * t
            );
        }
        assert!(r.t_end() <= p.t_end() + 1e-9);
        assert!(p.t_end() - r.t_end() < dt + 1e-9, "uses the whole record");
    }
}

#[test]
fn resample_at_a_window_and_at_own_step_is_identity() {
    let p = parse_pulse(&std::fs::read_to_string(root(GOOD_PULSE)).unwrap()).unwrap();
    let same = resample(&p, p.dt).unwrap();
    assert_eq!(same.accel.len(), p.accel.len());
    for (a, b) in same.accel.iter().zip(&p.accel) {
        assert!((a - b).abs() < 1e-9);
    }
    // Window [0, 0.006] at 0.5 ms: sample 2 of the data is t = 0.
    let w = resample_at(&p, 0.0, 0.0005, 13).unwrap();
    assert_eq!(w.accel[0], 0.0);
    assert!((w.accel[1] - (-5.0)).abs() < 1e-9, "{}", w.accel[1]); // halfway 0 .. -10
    assert!((w.accel[2] - (-10.0)).abs() < 1e-9);
    assert!((w.accel[12] - (-100.0)).abs() < 1e-9);
    assert_eq!(w.t0, 0.0);
    assert_eq!(w.dt, 0.0005);
}

#[test]
fn resample_refuses_outside_the_data_and_bad_steps() {
    let p = linear_pulse(101); // [-0.01, 0.09]
    assert!(matches!(
        resample_at(&p, -0.0101, 0.001, 5),
        Err(RefError::BadArgument(_))
    ));
    assert!(matches!(
        resample_at(&p, 0.0, 0.001, 92),
        Err(RefError::BadArgument(_))
    )); // ends at 0.091
    assert!(resample_at(&p, 0.0, 0.001, 91).is_ok()); // ends at 0.09 exactly
    assert!(resample_at(&p, -0.01, 0.001, 101).is_ok());
    for dt in [0.0, -0.001, f64::NAN, f64::INFINITY] {
        assert!(
            matches!(resample(&p, dt), Err(RefError::BadArgument(_))),
            "{dt}"
        );
        assert!(
            matches!(resample_at(&p, 0.0, dt, 3), Err(RefError::BadArgument(_))),
            "{dt}"
        );
    }
    // A step longer than the record leaves one sample only (the first), never an extrapolated one.
    let r = resample(&p, 1.0).unwrap();
    assert_eq!(r.accel.len(), 1);
    assert!(matches!(
        resample_at(&p, 0.0, 0.001, 0),
        Err(RefError::BadArgument(_))
    ));
    assert!(matches!(
        resample_at(&p, f64::NAN, 0.001, 3),
        Err(RefError::BadArgument(_))
    ));
}

// Metric functions local to this test (n20-09 passes the real ones).
fn dv(a: &[f64], dt: f64) -> Result<f64, String> {
    Ok(-a.windows(2).map(|w| 0.5 * (w[0] + w[1]) * dt).sum::<f64>())
}
fn pk(a: &[f64], dt: f64) -> Result<(f64, f64), String> {
    let (i, v) = a
        .iter()
        .enumerate()
        .fold((0, 0.0_f64), |m, (i, &v)| if v < m.1 { (i, v) } else { m });
    Ok((-v, i as f64 * dt))
}
fn du(a: &[f64], dt: f64) -> Result<Option<f64>, String> {
    let hits: Vec<usize> = (0..a.len()).filter(|&i| a[i] <= -20.0).collect();
    Ok(match (hits.first(), hits.last()) {
        (Some(f), Some(l)) => Some((l - f) as f64 * dt),
        _ => None,
    })
}
fn fns() -> MetricFns<'static> {
    MetricFns {
        delta_v: &dv,
        peak: &pk,
        duration: &du,
    }
}

fn window_of(p: &Pulse) -> Vec<f64> {
    resample_at(p, 0.0, 0.0005, 17).unwrap().accel
}

#[test]
fn a_pulse_compared_with_itself_has_zero_error() {
    let p = load_pulse(root(GOOD_PULSE)).unwrap();
    let w = window_of(&p);
    let c = compare(&w, &w, 0.0005, &fns()).unwrap();
    for m in [c.delta_v, c.peak, c.peak_time, c.duration.unwrap()] {
        assert_eq!(m.abs_err, 0.0);
        assert_eq!(m.rel_err, Some(0.0));
        assert_eq!(m.sim, m.reference);
    }
    assert!(c.peak.sim > 0.0 && c.delta_v.sim > 0.0 && c.duration.unwrap().sim > 0.0);
}

#[test]
fn compare_reports_known_errors() {
    let p = load_pulse(root(GOOD_PULSE)).unwrap();
    let reference = window_of(&p);
    let sim: Vec<f64> = reference.iter().map(|a| a * 1.1).collect();
    let c = compare(&sim, &reference, 0.0005, &fns()).unwrap();
    assert!((c.delta_v.rel_err.unwrap() - 0.1).abs() < 1e-9);
    assert!((c.peak.rel_err.unwrap() - 0.1).abs() < 1e-9);
    assert!((c.peak.abs_err - 40.0).abs() < 1e-9);
    assert_eq!(c.peak_time.abs_err, 0.0);
    // Peak delayed by 3 samples: time error is 1.5 ms absolute.
    let mut late = vec![0.0; 3];
    late.extend_from_slice(&reference[..reference.len() - 3]);
    let c = compare(&late, &reference, 0.0005, &fns()).unwrap();
    assert!(
        (c.peak_time.abs_err - 0.0015).abs() < 1e-12,
        "{}",
        c.peak_time.abs_err
    );
    assert!(c.peak.abs_err.abs() < 1e-12);
}

#[test]
fn compare_duration_undefined_and_zero_reference() {
    let p = load_pulse(root(GOOD_PULSE)).unwrap();
    let reference = window_of(&p);
    let flat = vec![0.0; reference.len()];
    let c = compare(&flat, &reference, 0.0005, &fns()).unwrap();
    assert!(c.duration.is_none());
    assert_eq!(c.duration_sim, None);
    assert!(c.duration_ref.is_some());
    // Zero reference, non-zero sim: relative error undefined, absolute still given.
    let c = compare(&reference, &flat, 0.0005, &fns()).unwrap();
    assert_eq!(c.peak.rel_err, None);
    assert_eq!(c.peak.abs_err, c.peak.sim);
}

#[test]
fn compare_refuses_bad_input_and_failing_metrics() {
    let a = [1.0, 2.0, 3.0];
    assert!(matches!(
        compare(&a, &a[..2], 0.1, &fns()),
        Err(RefError::BadArgument(_))
    ));
    assert!(matches!(
        compare(&[], &[], 0.1, &fns()),
        Err(RefError::BadArgument(_))
    ));
    assert!(matches!(
        compare(&a, &a, 0.0, &fns()),
        Err(RefError::BadArgument(_))
    ));
    let bad = |_: &[f64], _: f64| -> Result<f64, String> { Err("boom".into()) };
    let f = MetricFns {
        delta_v: &bad,
        peak: &pk,
        duration: &du,
    };
    let e = compare(&a, &a, 0.1, &f).unwrap_err();
    assert_eq!(
        e,
        RefError::Metric {
            which: "delta_v",
            reason: "boom".into()
        }
    );
}
