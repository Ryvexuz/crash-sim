//! Tests for the pulse module (CFC filter and metrics). The module is included by path
//! because `src/lib.rs` does not register it yet.
#![allow(dead_code)]

#[path = "../src/pulse.rs"]
mod pulse;

use pulse::*;
use std::f64::consts::PI;

fn sine(f: f64, dt: f64, n: usize) -> Vec<f64> {
    (0..n).map(|i| (2.0 * PI * f * i as f64 * dt).sin()).collect()
}

/// Largest magnitude in the middle half, away from the ends.
fn mid_amp(x: &[f64]) -> f64 {
    let n = x.len();
    x[n / 4..3 * n / 4].iter().fold(0.0, |m, v| f64::max(m, v.abs()))
}

#[test]
fn dc_input_unchanged() {
    let x = vec![123.456; 2000];
    let y = cfc_filter(&x, 1e-4, 60.0).unwrap();
    assert_eq!(y.len(), x.len());
    for (i, v) in y.iter().enumerate() {
        assert!((v - 123.456).abs() <= 1e-9 * 123.456, "sample {i}: {v}");
    }
}

#[test]
fn sine_at_tenth_of_cfc_passes() {
    let (dt, cfc) = (1e-4, 180.0);
    let x = sine(0.1 * cfc, dt, 20000);
    let y = cfc_filter(&x, dt, cfc).unwrap();
    let gain = mid_amp(&y) / mid_amp(&x);
    assert!((gain - 1.0).abs() < 0.005, "gain {gain}");
}

#[test]
fn sine_at_ten_times_cfc_attenuated_40_db() {
    let (dt, cfc) = (1e-4, 60.0);
    let x = sine(10.0 * cfc, dt, 20000);
    let y = cfc_filter(&x, dt, cfc).unwrap();
    let db = 20.0 * (mid_amp(&y) / mid_amp(&x)).log10();
    assert!(db <= -40.0, "gain {db} dB");
}

#[test]
fn symmetric_pulse_keeps_peak_time() {
    let (dt, cfc) = (1e-4, 180.0);
    let n = 4000;
    let mid = 2000usize;
    // Negative Gaussian, centre at sample 2000.
    let x: Vec<f64> = (0..n)
        .map(|i| {
            let t = (i as f64 - mid as f64) * dt;
            -300.0 * (-t * t / (2.0 * 0.005f64 * 0.005)).exp()
        })
        .collect();
    let y = cfc_filter(&x, dt, cfc).unwrap();
    let (v, t) = peak(&y, dt).unwrap();
    assert!(v < -100.0, "peak value {v}");
    let shift = (t / dt - mid as f64).abs();
    assert!(shift <= 1.0, "peak moved by {shift} samples");
}

#[test]
fn filter_refusals() {
    let x = vec![0.0, 1.0, 2.0, 3.0];
    assert!(matches!(cfc_filter(&x, 0.0, 60.0), Err(PulseError::BadDt(_))));
    assert!(matches!(cfc_filter(&x, -1e-4, 60.0), Err(PulseError::BadDt(_))));
    assert!(matches!(cfc_filter(&x, f64::NAN, 60.0), Err(PulseError::BadDt(_))));
    assert!(matches!(cfc_filter(&x, 1e-4, 0.0), Err(PulseError::BadCfc(_))));
    assert!(matches!(cfc_filter(&x, 1e-4, -5.0), Err(PulseError::BadCfc(_))));
    assert!(matches!(cfc_filter(&x, 1e-4, f64::INFINITY), Err(PulseError::BadCfc(_))));
    // 1 kHz sampling, CFC 180: 1000 < 1800.
    assert!(matches!(
        cfc_filter(&x, 1e-3, 180.0),
        Err(PulseError::SampleRateTooLow { .. })
    ));
    // Exactly 10 x CFC is allowed.
    assert!(cfc_filter(&x, 1e-3, 100.0).is_ok());
    let bad = vec![0.0, f64::NAN, 1.0];
    assert_eq!(cfc_filter(&bad, 1e-4, 60.0), Err(PulseError::NonFinite { index: 1 }));
    let inf = vec![0.0, 1.0, f64::NEG_INFINITY];
    assert_eq!(cfc_filter(&inf, 1e-4, 60.0), Err(PulseError::NonFinite { index: 2 }));
}

#[test]
fn filter_short_inputs_do_not_panic() {
    assert_eq!(cfc_filter(&[], 1e-4, 60.0).unwrap(), Vec::<f64>::new());
    assert_eq!(cfc_filter(&[5.0], 1e-4, 60.0).unwrap(), vec![5.0]);
    let y = cfc_filter(&[1.0, 1.0, 1.0], 1e-4, 60.0).unwrap();
    assert_eq!(y.len(), 3);
}

#[test]
fn delta_v_of_constant_acceleration() {
    // 100 m/s^2 for 0.1 s: 1001 samples spanning 0.1 s at dt = 1e-4.
    let x = vec![100.0; 1001];
    let dv = delta_v(&x, 1e-4).unwrap();
    assert!((dv - 10.0).abs() < 1e-9, "dv {dv}");
    assert_eq!(delta_v(&[], 1e-4).unwrap(), 0.0);
    assert!(delta_v(&x, 0.0).is_err());
    assert!(delta_v(&[1.0, f64::NAN], 1e-4).is_err());
}

#[test]
fn peak_is_most_negative_with_time() {
    let x = [1.0, -2.0, -7.5, 3.0, -7.5];
    assert_eq!(peak(&x, 0.01).unwrap(), (-7.5, 0.02));
    assert_eq!(peak(&[], 0.01), Err(PulseError::Empty));
    assert!(peak(&x, -1.0).is_err());
}

#[test]
fn duration_between_first_and_last_over_threshold() {
    let mut x = vec![0.0; 100];
    x[10] = -50.0;
    x[20] = 60.0;
    x[40] = -55.0;
    // Samples 10 to 40 at |a| >= 50: 30 samples.
    let d = duration(&x, 0.001, 50.0).unwrap().unwrap();
    assert!((d - 0.030).abs() < 1e-12, "d {d}");
    assert_eq!(duration(&x, 0.001, 100.0).unwrap(), None);
    assert!(duration(&x, 0.001, -1.0).is_err());
    assert!(duration(&x, 0.001, f64::NAN).is_err());
}

/// Analytic OLC case. Vehicle decelerates at a constant A = 200 m/s^2, v0 = 50 m/s (the
/// vehicle is still moving at 50 - 200 * 0.12 = 26 m/s, so it has not stopped by t2).
/// Free flight: s = A t^2 / 2 = 0.065 gives t1 = sqrt(0.13 / 200) = 0.02550 s; relative
/// speed then A t1 = 5.099 m/s. Over the restraint the relative deceleration is a_o - A and
/// the relative speed goes to zero over 0.235 m: (A t1)^2 / (2 (a_o - A)) = 0.235, so
/// a_o = A + (A t1)^2 / 0.47 = A + A * 0.13 / 0.47 = 200 * 1.276596 = 255.32 m/s^2.
#[test]
fn olc_constant_deceleration_matches_hand_calculation() {
    let dt = 1e-4;
    let x = vec![-200.0; 3001]; // 0.3 s
    let got = olc(&x, dt, 50.0).unwrap().unwrap();
    let want = 200.0 * (1.0 + 0.13 / 0.47);
    assert!((got - want).abs() / want < 0.01, "got {got}, want {want}");
}

#[test]
fn olc_none_when_occupant_never_moves_enough() {
    // Mild pulse over a short time: relative travel stays under 65 mm.
    let x = vec![-10.0; 101];
    assert_eq!(olc(&x, 1e-4, 15.0).unwrap(), None);
    // Positive acceleration: occupant never moves forward.
    let y = vec![50.0; 1000];
    assert_eq!(olc(&y, 1e-4, 15.0).unwrap(), None);
}

#[test]
fn olc_refusals() {
    let x = vec![-100.0; 10];
    assert!(olc(&x, 0.0, 15.0).is_err());
    assert!(olc(&x, 1e-4, 0.0).is_err());
    assert!(olc(&x, 1e-4, f64::NAN).is_err());
    assert!(olc(&[-1.0, f64::INFINITY], 1e-4, 15.0).is_err());
    assert_eq!(olc(&[], 1e-4, 15.0).unwrap(), None);
}
