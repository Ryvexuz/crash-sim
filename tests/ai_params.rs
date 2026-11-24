use crash_sim::ai_params::{parse, AiParams, ErrorKind, KEYS};
use crash_sim::race::{self, RaceEnd, RaceError, RaceSetup};
use crash_sim::speed_profile::ProfileError;
use crash_sim::track::Track;
use crash_sim::Config;

const GOOD: &str = "ai 1\n\
# driver\n\
lookahead_k 0.6\n\
lookahead_min 3.0\n\
lookahead_max 12.0\n\
kp 0.8\n\
ki 0.3\n\
integral_max 2.0\n\
\n\
# profile\n\
mu 0.6\n\
g 9.81\n\
a_accel 2.0\n\
a_brake 4.0\n\
v_max 15.0\n";

fn values(p: &AiParams) -> [u64; 11] {
    [
        p.driver.lookahead_k,
        p.driver.lookahead_min,
        p.driver.lookahead_max,
        p.driver.kp,
        p.driver.ki,
        p.driver.integral_max,
        p.profile.mu,
        p.profile.g,
        p.profile.a_accel,
        p.profile.a_brake,
        p.profile.v_max,
    ]
    .map(f64::to_bits)
}

#[test]
fn ai_params_reads_every_key() {
    let expected = [0.6_f64, 3.0, 12.0, 0.8, 0.3, 2.0, 0.6, 9.81, 2.0, 4.0, 15.0].map(f64::to_bits);
    let p = parse(GOOD).unwrap();
    assert_eq!(values(&p), expected);
    assert_eq!(p.lines, [3, 4, 5, 6, 7, 8, 11, 12, 13, 14, 15]);
    assert_eq!(p.line_of("v_max"), Some(15));
    assert_eq!(p.line_of("lookahead"), None);
    let key_lines: Vec<_> = GOOD
        .lines()
        .filter(|s| !s.is_empty() && !s.starts_with('#') && *s != "ai 1")
        .collect();
    let reverse = format!(
        "ai 1\n# driver\n# profile\n{}\n",
        key_lines.into_iter().rev().collect::<Vec<_>>().join("\n")
    );
    let p = parse(&reverse).unwrap();
    assert_eq!(values(&p), expected);
    for (i, key) in KEYS.iter().enumerate() {
        assert_eq!(p.lines[i], 14 - i);
        assert_eq!(p.line_of(key), Some(14 - i));
    }
    for (token, expected) in [
        ("0.1", 0.1_f64),
        ("1e-3", 0.001),
        ("+2", 2.0),
        ("-0", -0.0),
        ("5e-324", f64::from_bits(1)),
        ("1.7976931348623157e308", f64::MAX),
    ] {
        let p = parse(&GOOD.replace("kp 0.8", &format!("kp {token}"))).unwrap();
        assert_eq!(p.driver.kp.to_bits(), expected.to_bits());
    }
}

#[test]
fn ai_params_refuses_with_kind_and_line() {
    use ErrorKind::*;
    let mut cases = vec![
        (GOOD.replace('\n', "\r\n"), CarriageReturn, 1, Some(5)),
        (String::new(), Header, 1, None),
        (
            GOOD.trim_end_matches('\n').to_owned(),
            MissingFinalNewline,
            15,
            None,
        ),
        (GOOD.replace("ai 1", "ai 2"), UnknownVersion, 1, Some(4)),
        (GOOD.replace("ai 1", "ai one"), Header, 1, None),
        (GOOD.replace("ai 1", "track 1"), Header, 1, None),
        (format!("{GOOD}speed 1.0\n"), UnknownKey, 16, Some(1)),
        (format!("{GOOD}kp 0.9\n"), RepeatedKey, 16, Some(1)),
        (GOOD.replace("a_brake 4.0\n", ""), MissingKey, 14, None),
        (
            GOOD.replace("lookahead_k 0.6\n", "")
                .replace("v_max 15.0\n", ""),
            MissingKey,
            13,
            None,
        ),
    ];
    for (replacement, kind, column) in [
        ("kp\t0.8", Tab, Some(3)),
        ("kp  0.8", Spacing, None),
        (" kp 0.8", Spacing, None),
        ("kp 0.8 ", Spacing, None),
        ("   ", Spacing, None),
        ("kp 0.8 # reason", Syntax, None),
        ("kp", Syntax, None),
        ("kp fast", BadNumber, Some(4)),
        ("kp nan", NonFinite, Some(4)),
        ("kp inf", NonFinite, Some(4)),
        ("kp -inf", NonFinite, Some(4)),
        ("kp 1e999", NonFinite, Some(4)),
    ] {
        cases.push((GOOD.replace("kp 0.8", replacement), kind, 6, column));
    }
    for (text, kind, line, column) in cases {
        let e = parse(&text).unwrap_err();
        assert_eq!((e.kind, e.line, e.column), (kind, line, column), "{text}");
        assert!(e.to_string().starts_with(&format!("line {line}")));
        if kind == RepeatedKey {
            assert!(e.message.contains("line 6"));
        }
        if kind == MissingKey {
            let key = if line == 14 { "a_brake" } else { "lookahead_k" };
            assert!(e.message.contains(key));
        }
    }
    // Global checks precede even a malformed header; tabs precede comment skipping.
    assert_eq!(parse("bad\n# \t\r").unwrap_err().kind, CarriageReturn);
    assert_eq!(parse("bad").unwrap_err().kind, MissingFinalNewline);
    assert_eq!(parse("ai 2\t\n").unwrap_err().kind, Tab);
    assert_eq!(parse("ai 1\n# \t\n").unwrap_err().kind, Tab);
    assert_eq!(parse("ai 18446744073709551616\n").unwrap_err().kind, Header);
    assert_eq!(parse("ai 01\n").unwrap_err().kind, Header);
    let huge = format!("ai 1\n# {}\r\n", "é".repeat(100_000));
    let e = parse(&huge).unwrap_err();
    assert_eq!(
        (e.kind, e.line, e.column),
        (CarriageReturn, 2, Some(100_003))
    );
}

fn run(p: AiParams) -> Result<race::RaceResult, RaceError> {
    let cfg = Config::default();
    let body = crash_sim::format::parse(include_str!("../scenarios/car_basic.bsn")).unwrap();
    let track = Track::parse(include_str!("../scenarios/tracks/oval.track")).unwrap();
    race::run(&RaceSetup {
        cfg: &cfg,
        body: &body,
        track: &track,
        cars: 1,
        laps: 1,
        time_limit_s: 1.0,
        grid_gap_m: 10.0,
        driver: p.driver,
        profile: p.profile,
    })
}

#[test]
fn ai_params_never_checks_ranges() {
    let p = parse(&GOOD.replace("lookahead_min 3.0", "lookahead_min 0")).unwrap();
    assert_eq!(p.driver.lookahead_min.to_bits(), 0.0_f64.to_bits());
    assert_eq!(p.line_of("lookahead_min"), Some(4));
    assert!(matches!(run(p), Err(RaceError::Driver("lookahead_min"))));
    let p = parse(&GOOD.replace("mu 0.6", "mu -1")).unwrap();
    assert_eq!(p.profile.mu.to_bits(), (-1.0_f64).to_bits());
    assert_eq!(p.line_of("mu"), Some(11));
    assert!(matches!(
        run(p),
        Err(RaceError::Profile(ProfileError::BadParameter("mu")))
    ));
    let p = parse(&GOOD.replace("kp 0.8", "kp -5")).unwrap();
    assert_eq!(p.driver.kp.to_bits(), (-5.0_f64).to_bits());
}

#[test]
fn ai_params_good_file_races() {
    let result = run(parse(GOOD).unwrap()).unwrap();
    assert!(matches!(result.end, RaceEnd::TimeLimit));
    assert_eq!(result.substeps, 1000);
}
