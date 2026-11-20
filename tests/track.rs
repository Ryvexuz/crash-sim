//! Tests for src/track.rs (nb-43, docs/STAGE7.md check 1.1). The module is included by path
//! because `src/lib.rs` registers it in a later one-line task.

use crash_sim::track;
use crash_sim::vec3;

use track::{
    format_clock, format_seconds, ErrorKind, InvalidReason, LapEvent, LapTimer, ParseError,
    TimerError, Track,
};
use vec3::Vec3;

fn scenario(name: &str) -> String {
    let path = format!("{}/scenarios/tracks/{name}", env!("CARGO_MANIFEST_DIR"));
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"))
}

fn v(x: f64, z: f64) -> Vec3 {
    Vec3::new(x, 0.0, z)
}

fn file(lines: &[&str]) -> String {
    lines.join("\n") + "\n"
}

fn refused(text: &str) -> ParseError {
    match Track::parse(text) {
        Ok(_) => panic!("expected a refusal for:\n{text}"),
        Err(e) => e,
    }
}

/// Line 5 is the checkpoint, lines 6 to 9 the points.
const SQUARE: [&str; 9] = [
    "track 1",
    "width 10",
    "start 0",
    "grid 5",
    "checkpoint 2",
    "point 0 0",
    "point 100 0",
    "point 100 100",
    "point 0 100",
];

/// A loop of 8 points (200 m x 100 m, start gate at (0, 0) facing +x), width 40 m except point 1,
/// whose own width is 2 m; checkpoints at point 1 (the narrow gate at x = 100, z = 0) and 4
/// (x = 100, z = 100, driven toward -x).
fn wide_loop() -> Track {
    Track::parse(&file(&[
        "track 1",
        "width 40",
        "start 0",
        "grid 20",
        "checkpoint 1",
        "checkpoint 4",
        "point 0 0",
        "point 100 0 width 2",
        "point 200 0",
        "point 200 100",
        "point 100 100",
        "point 0 100",
        "point -100 100",
        "point -100 0",
    ]))
    .unwrap()
}

/// One lap of `wide_loop` along its centreline, one step per second: out-lap to the start line,
/// across it (step 0, crossing at x = 0), checkpoint 1 (step 1), checkpoint 4 (step 5), and back
/// across the start line (step 9, crossing at t = 9.5).
const LAP: [(f64, f64); 11] = [
    (-50.0, 0.0),
    (50.0, 0.0),
    (150.0, 0.0),
    (200.0, 0.0),
    (200.0, 100.0),
    (150.0, 100.0),
    (50.0, 100.0),
    (-100.0, 100.0),
    (-100.0, 0.0),
    (-50.0, 0.0),
    (50.0, 0.0),
];

/// Feed a list of positions, one step of `dt` each from time `t0`; all the events, in order.
fn run(timer: &mut LapTimer, route: &[(f64, f64)], t0: f64, dt: f64) -> Vec<LapEvent> {
    let mut events = Vec::new();
    for k in 1..route.len() {
        let (a, b) = (route[k - 1], route[k]);
        let t_prev = t0 + (k - 1) as f64 * dt;
        events.extend(
            timer
                .update(v(a.0, a.1), v(b.0, b.1), t_prev, t_prev + dt)
                .unwrap(),
        );
    }
    events
}

/// The centreline driven `loops` times from the grid position, resampled every `h` metres.
fn centre_route(track: &Track, loops: usize, h: f64) -> Vec<(f64, f64)> {
    let n = track.points.len();
    let g = track.grid_point();
    let mut verts = vec![(g[0], g[1])];
    for k in 0..(loops * n + 2) {
        let p = track.points[(track.start + k) % n];
        verts.push((p[0], p[1]));
    }
    let mut cum = vec![0.0];
    for w in verts.windows(2) {
        let d = ((w[1].0 - w[0].0).powi(2) + (w[1].1 - w[0].1).powi(2)).sqrt();
        cum.push(cum.last().unwrap() + d);
    }
    let total = *cum.last().unwrap();
    let mut out = Vec::new();
    let mut k = 0;
    while (k as f64) * h < total {
        let a = k as f64 * h;
        let i = (0..verts.len() - 1).rev().find(|&i| cum[i] <= a).unwrap();
        let f = (a - cum[i]) / (cum[i + 1] - cum[i]);
        out.push((
            verts[i].0 + (verts[i + 1].0 - verts[i].0) * f,
            verts[i].1 + (verts[i + 1].1 - verts[i].1) * f,
        ));
        k += 1;
    }
    out.push(*verts.last().unwrap());
    out
}

fn laps_of(events: &[LapEvent]) -> Vec<&track::LapRecord> {
    events
        .iter()
        .filter_map(|e| match e {
            LapEvent::Lap(r) => Some(r),
            _ => None,
        })
        .collect()
}

// ---------------------------------------------------------------- the track files

#[test]
fn oval_track_parses_as_specified() {
    let t = Track::parse(&scenario("oval.track")).unwrap();
    assert_eq!(t.points.len(), 64);
    let mut want: Vec<[f64; 2]> = vec![[0.0, 0.0], [50.0, 0.0]];
    for k in 1..=29 {
        let p = (6.0 * k as f64).to_radians();
        want.push([50.0 + 30.0 * p.sin(), 30.0 - 30.0 * p.cos()]);
    }
    want.extend([[50.0, 60.0], [0.0, 60.0], [-50.0, 60.0]]);
    for k in 1..=29 {
        let p = (6.0 * k as f64).to_radians();
        want.push([-50.0 - 30.0 * p.sin(), 30.0 + 30.0 * p.cos()]);
    }
    want.push([-50.0, 0.0]);
    assert_eq!(want.len(), 64);
    for (i, (got, w)) in t.points.iter().zip(&want).enumerate() {
        assert!(
            (got[0] - w[0]).abs() < 1e-9 && (got[1] - w[1]).abs() < 1e-9,
            "point {i}: {got:?} vs {w:?}"
        );
    }
    // The apexes the checkpoints sit on (STAGE7.md).
    for (i, apex) in [
        (16usize, [80.0, 30.0]),
        (32, [0.0, 60.0]),
        (48, [-80.0, 30.0]),
    ] {
        assert!((t.points[i][0] - apex[0]).abs() < 1e-9 && (t.points[i][1] - apex[1]).abs() < 1e-9);
    }
    assert_eq!(t.start, 0);
    assert_eq!(t.checkpoints, vec![16, 32, 48]);
    assert!(t.widths.iter().all(|&w| w == 12.0));
    assert_eq!(t.grid, 20.0);
    // 388.4094 m: two 100 m straights and 60 chords of 2 x 30 x sin 3 degrees.
    assert!(
        (t.centre_length() - 388.4094).abs() < 1e-3,
        "{}",
        t.centre_length()
    );
    // The start tangent is +x, and the grid is 20 m back along the front straight's lead-in.
    let tan = t.tangent(0);
    assert!((tan[0] - 1.0).abs() < 1e-12 && tan[1].abs() < 1e-12);
    let g = t.grid_point();
    assert!((g[0] + 20.0).abs() < 1e-9 && g[1].abs() < 1e-9, "{g:?}");
    assert_eq!(t.width_range(), (12.0, 12.0));
    let gate = t.gate(0);
    assert_eq!((gate.a, gate.b), ([0.0, -6.0], [0.0, 6.0]));
    assert_eq!(track::VERSION, 1);
}

#[test]
fn chicane_track_parses_and_has_a_narrow_point() {
    let t = Track::parse(&scenario("chicane.track")).unwrap();
    assert_eq!(t.points.len(), 12);
    assert_eq!(t.checkpoints, vec![4, 7, 9]);
    assert_eq!(t.widths[3], 8.0);
    assert_eq!(t.widths.iter().filter(|&&w| w == 10.0).count(), 11);
    assert_eq!(t.width_range(), (8.0, 10.0));
    assert_eq!(t.grid, 15.0);
}

#[test]
fn track_files_have_no_carriage_return() {
    let dir = format!("{}/scenarios/tracks", env!("CARGO_MANIFEST_DIR"));
    let mut seen = 0;
    for entry in std::fs::read_dir(&dir).unwrap() {
        let path = entry.unwrap().path();
        if path.extension().and_then(|e| e.to_str()) != Some("track") {
            continue;
        }
        let bytes = std::fs::read(&path).unwrap();
        assert!(
            !bytes.contains(&b'\r'),
            "{} has a carriage return",
            path.display()
        );
        assert!(
            Track::parse(std::str::from_utf8(&bytes).unwrap()).is_ok(),
            "{}",
            path.display()
        );
        seen += 1;
    }
    assert!(seen >= 2, "found {seen} track files");
}

#[test]
fn a_whole_lap_of_each_test_track_is_valid_with_every_checkpoint() {
    for (name, h, v_mps) in [("oval.track", 0.5, 20.0), ("chicane.track", 0.5, 10.0)] {
        let t = Track::parse(&scenario(name)).unwrap();
        let mut timer = LapTimer::new(&t);
        let route = centre_route(&t, 2, h);
        let events = run(&mut timer, &route, 0.0, h / v_mps);
        let laps = laps_of(&events);
        assert_eq!(laps.len(), 2, "{name}");
        for (i, lap) in laps.iter().enumerate() {
            assert_eq!(lap.n, i as u32 + 1);
            assert!(lap.valid(), "{name} lap {}: {:?}", lap.n, lap.invalid);
            assert_eq!(lap.off_track_s, 0.0);
            // L / v, to within one step.
            let want = t.centre_length() / v_mps;
            let got = lap.end_t - lap.start_t;
            assert!((got - want).abs() <= h / v_mps, "{name}: {got} vs {want}");
        }
        assert_eq!(
            laps[1].start_t, laps[0].end_t,
            "lap 2 starts where lap 1 ends"
        );
        let cps = events
            .iter()
            .filter(|e| matches!(e, LapEvent::Checkpoint { .. }))
            .count();
        assert_eq!(cps, 2 * t.checkpoints.len(), "{name}");
        assert!(matches!(events[0], LapEvent::Start { .. }));
        assert_eq!(
            timer.best_valid_ms(),
            Some(laps[0].time_ms.min(laps[1].time_ms))
        );
    }
}

// ---------------------------------------------------------------- refusals

#[test]
fn refuses_fewer_than_three_points() {
    let two = file(&[
        "track 1",
        "width 10",
        "start 0",
        "grid 1",
        "checkpoint 1",
        "point 0 0",
        "point 10 0",
    ]);
    let e = refused(&two);
    assert_eq!((e.kind, e.line), (ErrorKind::TooFewPoints, 7), "{e}");
    assert!(
        e.message.contains("at least 3 points") && e.message.contains("found 2"),
        "{e}"
    );
    let none = file(&["track 1", "width 10", "start 0", "grid 1", "checkpoint 1"]);
    let e = refused(&none);
    assert_eq!((e.kind, e.line), (ErrorKind::TooFewPoints, 5), "{e}");
    // Exactly 3 points is enough.
    assert!(Track::parse(&file(&[
        "track 1",
        "width 10",
        "start 0",
        "grid 1",
        "checkpoint 1",
        "point 0 0",
        "point 10 0",
        "point 10 10"
    ]))
    .is_ok());
}

#[test]
fn refuses_self_intersecting_centreline() {
    // A bow tie: the segment from point 2 crosses the one from point 0.
    let mut l = SQUARE.to_vec();
    l[6..9].copy_from_slice(&["point 10 10", "point 10 0", "point 0 10"]);
    let e = refused(&file(&l));
    assert_eq!((e.kind, e.line), (ErrorKind::SelfIntersecting, 8), "{e}");
    assert!(
        e.message.contains("meets itself")
            && e.message.contains("point 2")
            && e.message.contains("line 6"),
        "{e}"
    );

    // A vertex that touches an earlier segment is refused too.
    let touch = file(&[
        "track 1",
        "width 10",
        "start 0",
        "grid 1",
        "checkpoint 2",
        "point 0 0",
        "point 20 0",
        "point 20 10",
        "point 10 10",
        "point 10 0",
    ]);
    let e = refused(&touch);
    assert_eq!((e.kind, e.line), (ErrorKind::SelfIntersecting, 9), "{e}");

    // So is a point that doubles back over the segment before it (a spike).
    let spike = file(&[
        "track 1",
        "width 10",
        "start 0",
        "grid 1",
        "checkpoint 1",
        "point 0 0",
        "point 10 0",
        "point 5 0",
        "point 5 10",
    ]);
    assert_eq!(refused(&spike).kind, ErrorKind::SelfIntersecting);

    // Three points on one line fold the loop flat.
    let flat = file(&[
        "track 1",
        "width 10",
        "start 0",
        "grid 1",
        "checkpoint 1",
        "point 0 0",
        "point 10 0",
        "point 20 0",
    ]);
    assert_eq!(refused(&flat).kind, ErrorKind::SelfIntersecting);

    // Two neighbouring points at the same place are a zero-length segment.
    let mut l = SQUARE.to_vec();
    l[6] = "point 0 0";
    let e = refused(&file(&l));
    assert_eq!((e.kind, e.line), (ErrorKind::ZeroLengthSegment, 7), "{e}");
}

#[test]
fn refuses_checkpoint_out_of_range() {
    let mut l = SQUARE.to_vec();
    l[4] = "checkpoint 4";
    let e = refused(&file(&l));
    assert_eq!(
        (e.kind, e.line),
        (ErrorKind::CheckpointOutOfRange, 5),
        "{e}"
    );
    assert!(
        e.message.contains("checkpoint index 4") && e.message.contains("4 points"),
        "{e}"
    );
    let mut l = SQUARE.to_vec();
    l[2] = "start 4";
    let e = refused(&file(&l));
    assert_eq!((e.kind, e.line), (ErrorKind::StartOutOfRange, 3), "{e}");
}

#[test]
fn refuses_checkpoints_out_of_order() {
    let mut l = SQUARE.to_vec();
    l[4] = "checkpoint 3";
    l.insert(5, "checkpoint 2");
    let e = refused(&file(&l));
    assert_eq!(
        (e.kind, e.line),
        (ErrorKind::CheckpointsOutOfOrder, 6),
        "{e}"
    );
    assert!(
        e.message.contains("checkpoint 2") && e.message.contains("start index 0"),
        "{e}"
    );
    // The same checkpoint twice is out of order too.
    let mut l = SQUARE.to_vec();
    l[4] = "checkpoint 2";
    l.insert(5, "checkpoint 2");
    assert_eq!(refused(&file(&l)).kind, ErrorKind::CheckpointsOutOfOrder);
    // Order counts cyclically from the start index: start 2, then 3, 0, 1 is in order...
    let mut l = SQUARE.to_vec();
    l[2] = "start 2";
    l[4] = "checkpoint 3";
    l.insert(5, "checkpoint 0");
    l.insert(6, "checkpoint 1");
    let t = Track::parse(&file(&l)).unwrap();
    assert_eq!((t.start, t.checkpoints.clone()), (2, vec![3, 0, 1]));
    // ... and 0 then 3 is not.
    let mut l = SQUARE.to_vec();
    l[2] = "start 2";
    l[4] = "checkpoint 0";
    l.insert(5, "checkpoint 3");
    let e = refused(&file(&l));
    assert_eq!(
        (e.kind, e.line),
        (ErrorKind::CheckpointsOutOfOrder, 6),
        "{e}"
    );
    // A checkpoint on the start index.
    let mut l = SQUARE.to_vec();
    l[4] = "checkpoint 0";
    let e = refused(&file(&l));
    assert_eq!((e.kind, e.line), (ErrorKind::CheckpointIsStart, 5), "{e}");
}

#[test]
fn refuses_carriage_returns_with_their_line_and_column() {
    let crlf = file(&SQUARE).replace('\n', "\r\n");
    let e = refused(&crlf);
    assert_eq!(
        (e.kind, e.line, e.column),
        (ErrorKind::CarriageReturn, 1, Some(8)),
        "{e}"
    );
    assert_eq!(e.to_string(), format!("line 1, column 8: {}", e.message));
    // A bare CR (old Mac line endings) and one in the middle of a line are refused as well.
    let e = refused(&file(&SQUARE).replace("start 0\n", "start\r0\n"));
    assert_eq!(
        (e.kind, e.line, e.column),
        (ErrorKind::CarriageReturn, 3, Some(6)),
        "{e}"
    );
    let e = refused(&file(&SQUARE).replace('\n', "\r"));
    assert_eq!((e.kind, e.line), (ErrorKind::CarriageReturn, 1), "{e}");
}

#[test]
fn refuses_non_finite_numbers() {
    for (line_no, to) in [
        (2usize, "width nan"),
        (2, "width inf"),
        (4, "grid -inf"),
        (6, "point nan 0"),
        (7, "point 100 1e999"),
        (8, "point Infinity 100"),
    ] {
        let mut l = SQUARE.to_vec();
        l[line_no - 1] = to;
        let e = refused(&file(&l));
        assert_eq!(
            (e.kind, e.line),
            (ErrorKind::NonFinite, line_no),
            "{to}: {e}"
        );
        assert!(
            e.column.is_some() && e.message.contains("not finite"),
            "{to}: {e}"
        );
    }
    let mut l = SQUARE.to_vec();
    l[5] = "point 1.2.3 0";
    let e = refused(&file(&l));
    assert_eq!(
        (e.kind, e.line, e.column),
        (ErrorKind::BadNumber, 6, Some(7)),
        "{e}"
    );
}

#[test]
fn refuses_zero_and_negative_widths() {
    for (line_no, text) in [
        (2usize, "width 0"),
        (2, "width -3"),
        (7, "point 100 0 width 0"),
        (7, "point 100 0 width -1.5"),
    ] {
        let mut l = SQUARE.to_vec();
        l[line_no - 1] = text;
        let e = refused(&file(&l));
        assert_eq!(
            (e.kind, e.line),
            (ErrorKind::NonPositiveWidth, line_no),
            "{text}: {e}"
        );
        assert!(e.message.contains("greater than 0"), "{e}");
    }
}

#[test]
fn per_point_width_overrides_the_one_width() {
    let mut l = SQUARE.to_vec();
    l[7] = "point 100 100 width 3.5";
    let t = Track::parse(&file(&l)).unwrap();
    assert_eq!(t.widths, vec![10.0, 10.0, 3.5, 10.0]);
    assert_eq!(t.width_range(), (3.5, 10.0));
    // The gate at an overridden point has that point's width.
    let g = t.gate(2);
    let len = ((g.b[0] - g.a[0]).powi(2) + (g.b[1] - g.a[1]).powi(2)).sqrt();
    assert!((len - 3.5).abs() < 1e-12, "{len}");
}

#[test]
fn grid_is_read_and_checked() {
    let mut l = SQUARE.to_vec();
    l[3] = "grid 0";
    assert_eq!(Track::parse(&file(&l)).unwrap().grid_point(), [0.0, 0.0]);
    // 30 m back from (0, 0) is the loop's closing segment, from (0, 100): 70 m up it.
    l[3] = "grid 30";
    let g = Track::parse(&file(&l)).unwrap().grid_point();
    assert!((g[0]).abs() < 1e-12 && (g[1] - 30.0).abs() < 1e-12, "{g:?}");
    // 130 m back goes round the corner onto the segment before it.
    l[3] = "grid 130";
    let g = Track::parse(&file(&l)).unwrap().grid_point();
    assert!(
        (g[0] - 30.0).abs() < 1e-12 && (g[1] - 100.0).abs() < 1e-12,
        "{g:?}"
    );
    l[3] = "grid -1";
    let e = refused(&file(&l));
    assert_eq!((e.kind, e.line), (ErrorKind::BadGrid, 4), "{e}");
    l[3] = "grid 401";
    let e = refused(&file(&l));
    assert_eq!((e.kind, e.line), (ErrorKind::BadGrid, 4), "{e}");
    assert!(e.message.contains("longer than the centreline loop"), "{e}");
}

#[test]
fn refuses_malformed_files_with_their_line() {
    let ok = file(&SQUARE);
    assert!(Track::parse(&ok).is_ok());
    // header
    assert_eq!(refused("").kind, ErrorKind::Header);
    assert_eq!(refused("bsn 1\n").kind, ErrorKind::Header);
    let e = refused(&ok.replacen("track 1", "track 2", 1));
    assert_eq!((e.kind, e.line), (ErrorKind::UnknownVersion, 1), "{e}");
    // line ending and spacing
    let e = refused(ok.trim_end_matches('\n'));
    assert_eq!((e.kind, e.line), (ErrorKind::MissingFinalNewline, 9), "{e}");
    let e = refused(&ok.replacen("width 10", "width  10", 1));
    assert_eq!((e.kind, e.line), (ErrorKind::Spacing, 2), "{e}");
    let e = refused(&ok.replacen("start 0", "start 0 ", 1));
    assert_eq!((e.kind, e.line), (ErrorKind::Spacing, 3), "{e}");
    let e = refused(&ok.replacen("grid 5", "  ", 1));
    assert_eq!((e.kind, e.line), (ErrorKind::Spacing, 4), "{e}");
    let e = refused(&ok.replacen("grid 5", "grid\t5", 1));
    assert_eq!(
        (e.kind, e.line, e.column),
        (ErrorKind::Tab, 4, Some(5)),
        "{e}"
    );
    // keywords
    let e = refused(&ok.replacen("start 0", "begin 0", 1));
    assert_eq!((e.kind, e.line), (ErrorKind::UnknownKeyword, 3), "{e}");
    let e = refused(&ok.replacen("point 0 0", "width 12\npoint 0 0", 1));
    assert_eq!((e.kind, e.line), (ErrorKind::OutOfOrder, 6), "{e}");
    let e = refused(&ok.replacen("width 10\nstart 0", "start 0\nwidth 10", 1));
    assert_eq!((e.kind, e.line), (ErrorKind::OutOfOrder, 3), "{e}");
    let e = refused(&ok.replacen("start 0", "start 0\nstart 1", 1));
    assert_eq!((e.kind, e.line), (ErrorKind::OutOfOrder, 4), "{e}");
    // tokens
    let e = refused(&ok.replacen("point 0 100", "point 0", 1));
    assert_eq!((e.kind, e.line), (ErrorKind::Syntax, 9), "{e}");
    let e = refused(&ok.replacen("point 0 100", "point 0 100 depth 3", 1));
    assert_eq!(
        (e.kind, e.line, e.column),
        (ErrorKind::Syntax, 9, Some(13)),
        "{e}"
    );
    let e = refused(&ok.replacen("start 0", "start 00", 1));
    assert_eq!((e.kind, e.line), (ErrorKind::BadNumber, 3), "{e}");
    let e = refused(&ok.replacen("start 0", "start -1", 1));
    assert_eq!((e.kind, e.line), (ErrorKind::BadNumber, 3), "{e}");
    // missing lines
    let e = refused(&ok.replacen("checkpoint 2\n", "", 1));
    assert_eq!((e.kind, e.line), (ErrorKind::MissingLine, 8), "{e}");
    assert!(e.message.contains("checkpoint"), "{e}");
    let e = refused(&ok.replacen("width 10\n", "", 1));
    assert_eq!(e.kind, ErrorKind::MissingLine);
    // comments and blank lines after line 1 are fine
    let commented = ok.replacen("width 10", "\n# a comment\nwidth 10", 1);
    assert!(Track::parse(&commented).is_ok());
}

// ---------------------------------------------------------------- crossings

#[test]
fn crossing_time_is_exact_for_a_straight_pass() {
    let t = Track::parse(&scenario("oval.track")).unwrap();
    // The start gate of the oval is the segment x = 0, z from -6 to 6, forward = +x.
    // Worked by hand: the car moves from x = -0.3 at t = 0 to x = +0.7 at t = 0.001 along z = 0.
    // It meets x = 0 at the fraction s = 0.3 / 1.0 = 0.3 of the step, so the crossing time is
    // 0 + 0.3 x 0.001 = 0.0003 s, not the step's end (0.001) and not its start (0).
    let mut timer = LapTimer::new(&t);
    let ev = timer.update(v(-0.3, 0.0), v(0.7, 0.0), 0.0, 0.001).unwrap();
    assert_eq!(ev.len(), 1);
    let LapEvent::Start { t: tc } = ev[0] else {
        panic!("{ev:?}")
    };
    assert!((tc - 0.0003).abs() < 1e-12, "{tc}");

    // A slanted pass at a later time: (-0.2, -1) at t = 12.5 to (0.4, 1) at t = 12.502.
    // x(s) = -0.2 + 0.6 s = 0 at s = 1/3, so tc = 12.5 + 0.002 / 3 = 12.500666666...
    let mut timer = LapTimer::new(&t);
    let ev = timer
        .update(v(-0.2, -1.0), v(0.4, 1.0), 12.5, 12.502)
        .unwrap();
    let LapEvent::Start { t: tc } = ev[0] else {
        panic!("{ev:?}")
    };
    assert!((tc - (12.5 + 0.002 / 3.0)).abs() < 1e-12, "{tc}");

    // The same fraction through the gate itself.
    let c = t.gate(0).crossing(v(-0.3, 0.0), v(0.7, 0.0)).unwrap();
    assert!((c.s - 0.3).abs() < 1e-12 && c.forward);
    let c = t.gate(0).crossing(v(0.7, 0.0), v(-0.3, 0.0)).unwrap();
    assert!((c.s - 0.7).abs() < 1e-12 && !c.forward);
    // Beyond the gate's end (|z| > 6) or short of the line: no crossing.
    assert!(t.gate(0).crossing(v(-1.0, 6.5), v(1.0, 6.5)).is_none());
    assert!(t.gate(0).crossing(v(-2.0, 0.0), v(-0.5, 0.0)).is_none());
    // A move along the gate's own line is parallel: never a crossing.
    assert!(t.gate(0).crossing(v(0.0, -3.0), v(0.0, 3.0)).is_none());
}

#[test]
fn endpoint_on_the_line_counts_once() {
    let t = Track::parse(&scenario("oval.track")).unwrap();
    let mut timer = LapTimer::new(&t);
    // Step 1 ends exactly on the line (s = 1): that is the crossing, at the step's end time.
    let a = timer.update(v(-0.5, 0.0), v(0.0, 0.0), 1.0, 1.25).unwrap();
    assert_eq!(a, vec![LapEvent::Start { t: 1.25 }]);
    // Step 2 starts exactly on the line (s = 0): not a second crossing.
    let b = timer.update(v(0.0, 0.0), v(0.5, 0.0), 1.25, 1.5).unwrap();
    assert!(b.is_empty(), "{b:?}");
    // The car sitting on the line is no crossing either.
    let c = timer.update(v(0.0, 0.0), v(0.0, 0.0), 1.5, 1.75).unwrap();
    assert!(c.is_empty(), "{c:?}");
    assert_eq!(timer.current_lap(), Some((1, 1.25)));
    assert!(timer.laps().is_empty());
}

#[test]
fn crossings_inside_one_step_come_in_time_order() {
    // One long step crosses the start line (x = 0) and then checkpoint 1 (x = 100) on the wide
    // loop: Start at s = 0.25 (t = 11), Checkpoint at s = 0.75 (t = 13) of a 200 m step taking 4 s from t = 10.
    let t = wide_loop();
    let mut timer = LapTimer::new(&t);
    let ev = timer
        .update(v(-50.0, 0.0), v(150.0, 0.0), 10.0, 14.0)
        .unwrap();
    assert_eq!(
        ev,
        vec![
            LapEvent::Start { t: 11.0 },
            LapEvent::Checkpoint {
                lap: 1,
                k: 1,
                t: 13.0
            }
        ]
    );
}

// ---------------------------------------------------------------- the lap rule

#[test]
fn a_clean_lap_counts() {
    let t = wide_loop();
    let mut timer = LapTimer::new(&t);
    let ev = run(&mut timer, &LAP, 0.0, 1.0);
    let laps = laps_of(&ev);
    assert_eq!(laps.len(), 1, "{ev:?}");
    let lap = laps[0];
    assert!(lap.valid(), "{lap:?}");
    // The start line is crossed at step 0 (t = 0.5) and again at step 9 (t = 9.5): 9.000 s.
    assert_eq!(
        (lap.n, lap.start_t, lap.end_t, lap.time_ms),
        (1, 0.5, 9.5, 9000)
    );
    assert_eq!(lap.off_track_s, 0.0);
    assert_eq!(ev[0], LapEvent::Start { t: 0.5 });
    assert_eq!(
        ev[1],
        LapEvent::Checkpoint {
            lap: 1,
            k: 1,
            t: 1.5
        }
    );
    assert_eq!(timer.best_valid_ms(), Some(9000));
    assert_eq!(timer.current_lap(), Some((2, 9.5)));
    assert_eq!(format_seconds(lap.time_ms), "9.000");
}

#[test]
fn lap_with_a_skipped_checkpoint_is_not_counted() {
    let t = wide_loop();
    // Step 1 of the lap goes round checkpoint 1 (its gate is 2 m wide at x = 100): from (80, 3)
    // to (120, 3), 3 m beside it. Every sampled position is on track (the half-width there is
    // 4.8 m), so the only fault is the missed gate.
    let mut route = LAP.to_vec();
    route.splice(2..2, [(80.0, 3.0), (120.0, 3.0)]);
    let mut timer = LapTimer::new(&t);
    let ev = run(&mut timer, &route, 0.0, 1.0);
    let laps = laps_of(&ev);
    assert_eq!(laps.len(), 1, "{ev:?}");
    assert_eq!(
        laps[0].invalid,
        Some(InvalidReason::MissedCheckpoint),
        "{:?}",
        laps[0]
    );
    assert_eq!(
        laps[0].off_track_s, 0.0,
        "the fault is only the missed gate"
    );
    assert_eq!(
        timer.best_valid_ms(),
        None,
        "a lap that skipped a checkpoint never counts"
    );
    assert!(timer.laps().iter().all(|l| !l.valid()));
    // Checkpoint 4 was crossed, but not in its turn: only checkpoint 1 can advance the lap.
    assert!(
        ev.iter().all(|e| !matches!(e, LapEvent::Checkpoint { .. })),
        "{ev:?}"
    );
    // The timer then starts lap 2 normally.
    assert_eq!(timer.current_lap().map(|c| c.0), Some(2));
}

#[test]
fn reversing_over_the_line_never_counts() {
    let t = wide_loop();
    // Never crossed forward: reversing over the line starts nothing and times nothing.
    let mut timer = LapTimer::new(&t);
    let ev = run(
        &mut timer,
        &[(50.0, 0.0), (-50.0, 0.0), (50.0, 0.0), (-50.0, 0.0)],
        0.0,
        1.0,
    );
    // Step 1 (-50 -> 50, forward) starts lap 1; steps 0 and 2 are backward and do nothing.
    assert_eq!(ev, vec![LapEvent::Start { t: 1.5 }]);

    // Forward over the line, back over it, forward again: the lap ended by the second forward
    // pass has no checkpoints, so there is no valid lap, and the backward pass counted for nothing.
    let mut timer = LapTimer::new(&t);
    let ev = run(
        &mut timer,
        &[(-50.0, 0.0), (50.0, 0.0), (-50.0, 0.0), (50.0, 0.0)],
        0.0,
        1.0,
    );
    assert_eq!(ev[0], LapEvent::Start { t: 0.5 });
    let laps = laps_of(&ev);
    assert_eq!(laps.len(), 1, "{ev:?}");
    assert_eq!((laps[0].start_t, laps[0].end_t), (0.5, 2.5));
    assert_eq!(laps[0].invalid, Some(InvalidReason::MissedCheckpoint));
    assert_eq!(timer.best_valid_ms(), None);

    // After a valid lap, reversing back over the line adds nothing.
    let mut timer = LapTimer::new(&t);
    run(&mut timer, &LAP, 0.0, 1.0);
    assert_eq!(timer.laps().len(), 1);
    let ev = timer
        .update(v(50.0, 0.0), v(-50.0, 0.0), 10.0, 11.0)
        .unwrap();
    assert!(ev.is_empty(), "{ev:?}");
    assert_eq!(timer.laps().len(), 1);
    assert_eq!(timer.current_lap(), Some((2, 9.5)));
}

#[test]
fn driving_the_track_backwards_never_counts() {
    // The wide loop's lap, reversed.
    let t = wide_loop();
    let mut timer = LapTimer::new(&t);
    let mut route = LAP.to_vec();
    route.reverse();
    let ev = run(&mut timer, &route, 0.0, 1.0);
    assert!(ev.is_empty(), "{ev:?}");
    assert_eq!(timer.current_lap(), None);
    assert!(timer.laps().is_empty());

    // Two whole loops of the oval against the direction of travel, at 0.5 m resolution.
    let oval = Track::parse(&scenario("oval.track")).unwrap();
    let mut timer = LapTimer::new(&oval);
    let mut route = centre_route(&oval, 2, 0.5);
    route.reverse();
    let ev = run(&mut timer, &route, 0.0, 0.025);
    assert!(ev.is_empty(), "no start, no checkpoint, no lap: {ev:?}");
    assert_eq!(timer.current_lap(), None);
}

#[test]
fn lap_off_track_is_invalid() {
    let t = wide_loop();
    // Detour out to (260, 50), 60 m from the centreline (half-width 20), and back: one substep
    // (the one ending at (260, 50)) is off track, of 1 s.
    let mut route = LAP.to_vec();
    route.insert(5, (260.0, 50.0));
    let mut timer = LapTimer::new(&t);
    let ev = run(&mut timer, &route, 0.0, 1.0);
    let laps = laps_of(&ev);
    assert_eq!(laps.len(), 1, "{ev:?}");
    assert_eq!(laps[0].invalid, Some(InvalidReason::OffTrack));
    assert_eq!(laps[0].off_track_s, 1.0);
    assert_eq!(timer.best_valid_ms(), None);
    assert_eq!(InvalidReason::OffTrack.as_str(), "off_track");
    assert_eq!(
        InvalidReason::MissedCheckpoint.as_str(),
        "missed_checkpoint"
    );

    // The next clean lap is valid again: the fault does not carry over.
    let ev = run(&mut timer, &LAP[1..], 20.0, 1.0);
    let laps = laps_of(&ev);
    assert_eq!(laps.len(), 1, "{ev:?}");
    assert!(laps[0].valid(), "{:?}", laps[0]);
    assert_eq!(laps[0].n, 2);
    assert_eq!(timer.best_valid_ms(), Some(laps[0].time_ms));
}

#[test]
fn the_out_lap_is_not_timed_so_off_track_before_the_line_is_fine() {
    let t = wide_loop();
    let mut route = vec![(-50.0, 0.0), (-50.0, -100.0)];
    route.extend_from_slice(&LAP);
    let mut timer = LapTimer::new(&t);
    let ev = run(&mut timer, &route, 0.0, 1.0);
    let laps = laps_of(&ev);
    assert_eq!(laps.len(), 1, "{ev:?}");
    assert!(laps[0].valid(), "{:?}", laps[0]);
}

#[test]
fn a_missed_checkpoint_is_reported_before_off_track() {
    // Cut the loop's far corner (off track, and checkpoint 4's gate is never crossed).
    let t = wide_loop();
    let route = [
        (-50.0, 0.0),
        (50.0, 0.0),
        (150.0, 0.0),
        (200.0, 0.0),
        (300.0, 200.0),
        (50.0, 300.0),
        (-50.0, 0.0),
        (50.0, 0.0),
    ];
    let mut timer = LapTimer::new(&t);
    let ev = run(&mut timer, &route, 0.0, 1.0);
    let laps = laps_of(&ev);
    assert_eq!(laps.len(), 1, "{ev:?}");
    assert!(laps[0].off_track_s > 0.0);
    assert_eq!(laps[0].invalid, Some(InvalidReason::MissedCheckpoint));
}

#[test]
fn on_track_uses_half_width() {
    let oval = Track::parse(&scenario("oval.track")).unwrap();
    // Width 12: on the straight at x = 25 the edge is at z = +-6, and the edge itself is on track.
    assert!(oval.on_track(v(25.0, 0.0)));
    assert!(oval.on_track(v(25.0, 5.99)));
    assert!(oval.on_track(v(25.0, -6.0)));
    assert!(!oval.on_track(v(25.0, 6.01)));
    assert!(!oval.on_track(v(25.0, -6.01)));
    // Height does not matter: the car position is projected onto the ground plane.
    assert!(oval.on_track(Vec3::new(25.0, 3.0, 5.99)));
    // On the bend the distance is to the nearest piece of the centreline: the apex (80, 30).
    assert!(oval.on_track(v(85.9, 30.0)));
    assert!(!oval.on_track(v(86.1, 30.0)));
    assert!(oval.on_track(v(74.1, 30.0)));
    assert!(!oval.on_track(v(73.9, 30.0)));
    // The width of a segment with different ends is interpolated at the nearest point:
    // point 0 is 40 wide and point 1 is 2 wide, so at x = 80 (f = 0.8) it is 9.6 (half 4.8).
    let t = wide_loop();
    assert!(t.on_track(v(80.0, 4.79)));
    assert!(!t.on_track(v(80.0, 4.81)));
    // Past an end of the centreline polyline the nearest point is the end point itself.
    assert!(t.on_track(v(-100.0, -19.9)));
    assert!(!t.on_track(v(-100.0, -20.1)));
    // A position that is not a number is never on track.
    assert!(!t.on_track(v(f64::NAN, 0.0)));
}

#[test]
fn update_refuses_bad_steps_and_leaves_the_timer_alone() {
    let t = wide_loop();
    let mut timer = LapTimer::new(&t);
    assert_eq!(
        timer.update(v(f64::NAN, 0.0), v(0.0, 0.0), 0.0, 1.0),
        Err(TimerError::NonFinite)
    );
    assert_eq!(
        timer.update(v(0.0, 0.0), v(1.0, f64::INFINITY), 0.0, 1.0),
        Err(TimerError::NonFinite)
    );
    assert_eq!(
        timer.update(v(0.0, 0.0), v(1.0, 0.0), 0.0, f64::NAN),
        Err(TimerError::NonFinite)
    );
    assert_eq!(
        timer.update(v(-1.0, 0.0), v(1.0, 0.0), 2.0, 1.0),
        Err(TimerError::TimeWentBackwards)
    );
    assert_eq!(timer.current_lap(), None);
    assert!(TimerError::NonFinite.to_string().contains("not finite"));
    assert!(TimerError::TimeWentBackwards
        .to_string()
        .contains("earlier"));
    // A zero-length step in time is fine.
    assert!(timer.update(v(-1.0, 0.0), v(1.0, 0.0), 1.0, 1.0).is_ok());
}

#[test]
fn lap_times_are_printed_from_the_integer_millisecond() {
    assert_eq!(format_seconds(83456), "83.456");
    assert_eq!(format_seconds(5), "0.005");
    assert_eq!(format_seconds(60000), "60.000");
    assert_eq!(format_clock(83456), "1:23.456");
    assert_eq!(format_clock(5), "0:00.005");
    assert_eq!(format_clock(600999), "10:00.999");
}
