//! n20-09: `--pulse-report` (docs/STAGE6-M4-M5.md, "The pulse report") and the gravity rule it needs.

use crash_sim::Config;

/// n20-07's rule: gravity 0 is a valid setting (a car flying level into a wall), while a negative,
/// NaN or infinite gravity is still refused, by both `validate` and `validate_body`.
#[test]
fn gravity_zero_is_accepted_and_bad_gravity_is_refused() {
    let zero = Config {
        gravity: 0.0,
        ..Config::default()
    };
    assert_eq!(zero.validate(), Ok(()), "gravity 0 must be accepted");
    assert_eq!(zero.validate_body(&[], &[]), Ok(()));
    for bad in [-1.0, -1e-300, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let cfg = Config {
            gravity: bad,
            ..Config::default()
        };
        let e = cfg
            .validate()
            .expect_err(&format!("gravity {bad} must be refused"));
        assert!(e.contains("gravity >= 0"), "message names the rule: {e}");
        assert!(cfg.validate_body(&[], &[]).is_err(), "validate_body, gravity {bad}");
    }
}
