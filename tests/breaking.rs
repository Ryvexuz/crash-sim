//! Breaking beams (nb-35): a beam whose material has a `break_strain` breaks for good when its
//! |strain| exceeds it. A broken beam applies no force, is left out of the strain checks, and is
//! reported (`broken_beams` column, `--events` lines). The default cube is unchanged.

use crash_sim::format::parse;
use crash_sim::{
    events_text, run_body_report, run_checked_with_contact, to_csv, BreakEvent, Config, Sim,
    GRAVITY,
};
use std::process::{Command, Output};
mod run_tmp;

const K: f64 = 20_000.0;
const C: f64 = 20.0;

/// Two 1 kg nodes 1 m apart at height `y`, joined by one beam of stiffness K, damping C and the
/// given `break_strain` (rest length 1 m). The left node starts at (-v, 0, 0), the right at (v, 0, 0).
fn pair_text(y: f64, v: f64, extra: &str) -> String {
    format!(
        "bsn 1\nmaterial m stiffness {K} damping {C}{extra}\nnode 0 0 {y} 0 mass 1\nnode 1 1 {y} 0 mass 1\nbeam 0 1 m\ngroup left 0\ngroup right 1\nvelocity left {} 0 0\nvelocity right {v} 0 0\n",
        -v
    )
}

fn pair(y: f64, v: f64, extra: &str) -> Sim {
    Sim::from_body(Config::default(), &parse(&pair_text(y, v, extra)).unwrap()).unwrap()
}

/// The first substep (1-based) at which the pair's strain exceeds `limit`, by an independent
/// hand-written symplectic Euler on the one relative coordinate (stretch x, stretch rate u, two
/// 1 kg nodes, so x' acceleration is -2 (k x + c u)). Gravity acts equally on both nodes and
/// does not change it.
fn expected_break_substep(v_rel: f64, limit: f64, h: f64) -> u32 {
    let (mut x, mut u) = (0.0_f64, v_rel);
    for n in 1..100_000u32 {
        u -= 2.0 * (K * x + C * u) * h;
        x += u * h;
        if x.abs() / 1.0 > limit {
            return n;
        }
    }
    panic!("never breaks");
}

#[test]
fn beam_breaks_at_expected_substep() {
    let h = 0.001;
    let expect = expected_break_substep(5.0, 0.01, h);
    // The scenario's closed form puts the peak at 0.0074 s, so the break is near substep 5 to 7.
    assert!((3..=8).contains(&expect), "hand model gave {expect}");
    let mut sim = pair(100.0, 2.5, " break_strain 0.01");
    sim.advance(); // one output step = 10 substeps of 1 ms
    assert_eq!(sim.breaks.len(), 1, "breaks: {:?}", sim.breaks);
    let b = sim.breaks[0];
    assert_eq!(b.beam, 0);
    assert!(
        (b.t - expect as f64 * h).abs() < 1e-9,
        "break at t = {} but the hand model says substep {expect} (t = {})",
        b.t,
        expect as f64 * h
    );
    assert!(b.strain > 0.01 && b.strain < 0.02, "strain {}", b.strain);
    assert!(sim.broken[0]);
}

#[test]
fn below_the_break_strain_nothing_breaks() {
    // v_rel = 1.5 m/s peaks at about 0.0065 strain, under 0.01.
    let mut sim = pair(100.0, 0.75, " break_strain 0.01");
    for _ in 0..100 {
        sim.advance();
        sim.check().unwrap();
    }
    assert!(sim.breaks.is_empty());
    assert!(!sim.broken[0]);
}

#[test]
fn a_broken_beam_applies_no_force_and_never_comes_back() {
    let mut sim = pair(100.0, 2.5, " break_strain 0.01");
    sim.advance();
    assert!(sim.broken[0]);
    let rel = |s: &Sim| s.vel[1].x - s.vel[0].x;
    let u = rel(&sim);
    let d = (sim.pos[1] - sim.pos[0]).length();
    for _ in 0..50 {
        sim.advance();
        // No spring and no damper: the relative speed is exactly constant (gravity is the same on both).
        assert!(
            (rel(&sim) - u).abs() < 1e-9,
            "relative speed changed: {} -> {}",
            u,
            rel(&sim)
        );
        assert!(sim.broken[0], "a broken beam came back");
        assert_eq!(sim.breaks.len(), 1);
    }
    // 0.5 s at the relative speed, plus the length at the break.
    let moved = (sim.pos[1] - sim.pos[0]).length() - d;
    assert!(
        (moved - u * 0.5).abs() < 1e-6,
        "moved {moved}, expected {}",
        u * 0.5
    );
}

#[test]
fn a_broken_beam_does_not_come_back_when_the_nodes_return_to_its_length() {
    // Pull the nodes apart, break, then send them back to the rest distance: still broken, still no force.
    let mut sim = pair(100.0, 2.5, " break_strain 0.01");
    sim.advance();
    assert!(sim.broken[0]);
    sim.vel[0].x = 1.0;
    sim.vel[1].x = -1.0;
    for _ in 0..300 {
        sim.advance();
    }
    assert_eq!(sim.breaks.len(), 1);
    assert!(sim.broken[0]);
    // The nodes passed through each other without the beam pushing back: they coast at +-1 m/s.
    assert!(
        (sim.vel[0].x - 1.0).abs() < 1e-9 && (sim.vel[1].x + 1.0).abs() < 1e-9,
        "{:?} {:?}",
        sim.vel[0],
        sim.vel[1]
    );
}

#[test]
fn disconnected_node_falls_freely() {
    let y0 = 2.0;
    let mut sim = pair(y0, 2.5, " break_strain 0.01");
    for _ in 0..10 {
        sim.advance();
    }
    assert_eq!(sim.breaks.len(), 1);
    // t = 0.1 s, n = 100 substeps of h = 1 ms: symplectic Euler from rest gives y0 - g h^2 n (n + 1) / 2.
    let n = 100.0_f64;
    let expect = y0 - GRAVITY * 1e-6 * n * (n + 1.0) / 2.0;
    for (i, p) in sim.pos.iter().enumerate() {
        assert!(
            (p.y - expect).abs() < 1e-9,
            "node {i}: y = {}, free fall says {expect}",
            p.y
        );
    }
    // And it still hits the ground and stays there: nothing is left holding it up.
    let mut touched = false;
    for _ in 0..500 {
        sim.advance();
        sim.check().unwrap();
        touched |= sim.first_contact_s.is_some();
    }
    assert!(touched);
    for (i, (p, v)) in sim.pos.iter().zip(&sim.vel).enumerate() {
        assert_eq!(p.y, 0.0, "node {i} rests on the ground");
        assert_eq!(v.y, 0.0);
    }
}

#[test]
fn a_node_with_no_beams_from_the_start_still_falls() {
    let body = parse("bsn 1\nmaterial m stiffness 20000 damping 20 break_strain 0.01\nnode 0 0 2 0 mass 1\nnode 1 5 2 0 mass 1\n").unwrap();
    let r = run_body_report(
        &Config {
            duration: 2.0,
            ..Config::default()
        },
        &body,
    );
    r.result.unwrap();
    assert!(r.first_contact_s.is_some());
    assert_eq!(r.samples.last().unwrap().min_y, 0.0);
}

#[test]
fn broken_beam_does_not_trip_strain_limit() {
    // 5 m/s pulled for 1 s: unbroken, the strain would pass the 0.5 limit within 0.1 s and abort.
    let body = parse(&pair_text(100.0, 2.5, " break_strain 0.01")).unwrap();
    let r = run_body_report(
        &Config {
            duration: 1.0,
            ..Config::default()
        },
        &body,
    );
    r.result.as_ref().unwrap();
    assert_eq!(r.breaks.len(), 1);
    let last = r.samples.last().unwrap();
    assert_eq!(last.broken_beams, 1);
    assert_eq!(
        last.max_beam_strain, 0.0,
        "the only beam is broken, so no beam has a strain"
    );
}

#[test]
fn only_unbroken_beams_count_in_the_strain() {
    // Three nodes in a line; beam 0 (soft limit) breaks, beam 1 (no limit) stays and keeps its strain.
    let text = format!(
        "bsn 1\nmaterial weak stiffness {K} damping {C} break_strain 0.01\nmaterial tough stiffness {K} damping {C}\nnode 0 -1 100 0 mass 1\nnode 1 0 100 0 mass 1\nnode 2 1 100 0 mass 1\nbeam 0 1 weak\nbeam 1 2 tough\ngroup l 0\nvelocity l -2.5 0 0\n"
    );
    let body = parse(&text).unwrap();
    let r = run_body_report(
        &Config {
            duration: 0.5,
            ..Config::default()
        },
        &body,
    );
    r.result.as_ref().unwrap();
    assert_eq!(r.breaks.len(), 1);
    assert_eq!(r.breaks[0].beam, 0);
    let last = r.samples.last().unwrap();
    assert_eq!(last.broken_beams, 1);
    // The tough beam is not at its rest length, so it must still be counted (not skipped too).
    assert!(last.max_beam_strain < 0.5);
}

#[test]
fn breaks_in_one_substep_come_in_beam_index_order() {
    // Node 1 in the middle, nodes 0 and 2 pulled outwards at the same speed: both beams reach their limit in the same substep.
    let text = format!(
        "bsn 1\nmaterial m stiffness {K} damping {C} break_strain 0.01\nnode 0 -1 100 0 mass 1\nnode 1 0 100 0 mass 1\nnode 2 1 100 0 mass 1\nbeam 0 1 m\nbeam 1 2 m\ngroup l 0\ngroup r 2\nvelocity l -2.5 0 0\nvelocity r 2.5 0 0\n"
    );
    let body = parse(&text).unwrap();
    let r = run_body_report(
        &Config {
            duration: 0.1,
            ..Config::default()
        },
        &body,
    );
    r.result.as_ref().unwrap();
    let ev: Vec<(u32, usize)> = r
        .breaks
        .iter()
        .map(|b| ((b.t * 1e6).round() as u32, b.beam))
        .collect();
    assert_eq!(ev.len(), 2, "{:?}", r.breaks);
    assert_eq!(ev[0].0, ev[1].0, "both break in the same substep: {ev:?}");
    assert_eq!((ev[0].1, ev[1].1), (0, 1));
}

#[test]
fn a_beam_that_both_yields_and_breaks_can_break() {
    // yield 0.005, break 0.01, pulled at 10 m/s (0.01 of strain per substep): if plastic flow ran first and held the strain at 0.005, the next substep would reach only about 0.0099 and the beam would never break. Breaking is judged first, so it breaks.
    let mut sim = pair(100.0, 5.0, " yield_strain 0.005 break_strain 0.01");
    for _ in 0..5 {
        sim.advance();
    }
    assert_eq!(sim.breaks.len(), 1, "{:?}", sim.breaks);
}

#[test]
fn break_strain_not_above_yield_strain_is_refused() {
    // The parser refuses it ...
    let text = pair_text(100.0, 0.0, " yield_strain 0.02 break_strain 0.01");
    assert!(parse(&text)
        .unwrap_err()
        .to_string()
        .contains("break_strain"));
    // ... and so does the sim for a Body built another way.
    let mut body = parse(&pair_text(
        100.0,
        0.0,
        " yield_strain 0.01 break_strain 0.02",
    ))
    .unwrap();
    body.materials[0].break_strain = Some(0.01);
    let e = Sim::from_body(Config::default(), &body).err().unwrap();
    assert!(e.contains("break_strain"), "{e}");
}

#[test]
fn same_input_gives_the_same_bytes() {
    let body = parse(&pair_text(2.0, 2.5, " break_strain 0.01")).unwrap();
    let cfg = Config {
        duration: 2.0,
        ..Config::default()
    };
    let a = run_body_report(&cfg, &body);
    let b = run_body_report(&cfg, &body);
    assert_eq!(a.breaks, b.breaks);
    assert_eq!(
        crash_sim::to_csv_with_broken(&a.samples),
        crash_sim::to_csv_with_broken(&b.samples)
    );
}

#[test]
fn events_text_format() {
    let t = events_text(&[
        BreakEvent {
            t: 0.0074,
            beam: 3,
            strain: 0.0101234567,
        },
        BreakEvent {
            t: 1.5,
            beam: 12,
            strain: 0.2,
        },
    ]);
    assert_eq!(
        t,
        "break t=0.0074 beam=3 strain=0.010123\nbreak t=1.5000 beam=12 strain=0.200000\n"
    );
    assert_eq!(events_text(&[]), "");
}

/// FNV-1a 64 of a byte string (the golden below was taken from the program before nb-35).
fn fnv(data: &[u8]) -> u64 {
    data.iter().fold(0xcbf29ce484222325u64, |h, &b| {
        (h ^ b as u64).wrapping_mul(0x100000001b3)
    })
}

/// The default cube's CSV is byte-for-byte what it was before breaking existed: 502 lines,
/// 17067 bytes, FNV-1a 0x76ac6a81de76619f (measured on the nb-34 program).
#[test]
fn default_cube_output_unchanged() {
    let o = run(&[]);
    assert!(o.status.success());
    assert!(
        o.stdout.starts_with(b"t,min_y,center_y,max_beam_strain\n"),
        "cube header changed"
    );
    assert_eq!(o.stdout.len(), 17067);
    assert_eq!(fnv(&o.stdout), 0x76ac6a81de76619f);
    // The library path agrees with the program.
    let (samples, _, res) = run_checked_with_contact(&Config::default());
    res.unwrap();
    assert_eq!(to_csv(&samples).as_bytes(), &o.stdout[..]);
    // --events on the cube prints nothing extra (it has no breakable beams).
    assert_eq!(run(&["--events"]).stdout, o.stdout);
}

fn run(args: &[&str]) -> Output {
    Command::new(crash_sim_bin()).args(args).output().unwrap()
}

fn body_file(name: &str, text: &str) -> String {
    let path = run_tmp::run_tmp().join(name).to_string_lossy().into_owned();
    std::fs::write(&path, text).unwrap();
    path
}

#[test]
fn body_csv_has_broken_beams_and_events_match_it() {
    let path = body_file(
        "breaking_pull.bsn",
        &pair_text(100.0, 2.5, " break_strain 0.01"),
    );
    let o = run(&["--body", &path, "--duration", "1", "--events"]);
    assert!(o.status.success(), "{}", String::from_utf8_lossy(&o.stderr));
    let text = String::from_utf8(o.stdout).unwrap();
    let mut lines = text.lines();
    assert_eq!(
        lines.next().unwrap(),
        "t,min_y,center_y,max_beam_strain,broken_beams"
    );
    let rows: Vec<&str> = text
        .lines()
        .skip(1)
        .filter(|l| !l.starts_with("break "))
        .collect();
    assert_eq!(rows.len(), 101);
    assert!(
        rows.iter().all(|r| r.split(',').count() == 5),
        "every row has 5 columns"
    );
    assert!(rows[0].ends_with(",0"), "{}", rows[0]);
    assert!(
        rows[100].starts_with("1.0000,") && rows[100].ends_with(",1"),
        "{}",
        rows[100]
    );
    let ev: Vec<&str> = text.lines().filter(|l| l.starts_with("break ")).collect();
    assert_eq!(ev.len(), 1, "{text}");
    let t: f64 = ev[0]
        .strip_prefix("break t=")
        .unwrap()
        .split(' ')
        .next()
        .unwrap()
        .parse()
        .unwrap();
    assert!(t > 0.0 && t <= 0.0074 + 1e-9, "{}", ev[0]);
    assert!(ev[0].contains(" beam=0 strain=0.01"), "{}", ev[0]);
    // The same run is the same bytes, and without --events there are no break lines.
    let again = run(&["--body", &path, "--duration", "1", "--events"]);
    assert_eq!(again.stdout, text.as_bytes());
    let quiet = run(&["--body", &path, "--duration", "1"]);
    let quiet = String::from_utf8(quiet.stdout).unwrap();
    assert!(!quiet.contains("break "));
    assert!(quiet.starts_with("t,min_y,center_y,max_beam_strain,broken_beams\n"));
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
