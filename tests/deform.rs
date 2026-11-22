//! Plastic beams (nb-34): a beam whose material has a `yield_strain` changes its rest length
//! permanently when it is stretched or squeezed past that strain, and the body comes from a
//! `.bsn` file. Plastic flow only removes energy. A material without `yield_strain`, and the
//! default cube, behave exactly as before.

use crash_sim::format::parse;
use crash_sim::settle::total_energy;
use crash_sim::vec3::Vec3;
use crash_sim::{run_body_checked_with_contact, to_csv, Config, Sim};
use std::process::{Command, Output};

const K: f64 = 20_000.0;

/// Two 1 kg nodes 1 m apart, 100 m above the ground (so nothing touches it in 3 s of free fall),
/// joined by one beam of the given material tail, for example ` yield_strain 0.05`.
fn pair(material_tail: &str) -> Sim {
    let text = format!(
        "bsn 1\nmaterial m stiffness {K} damping 20{material_tail}\nnode 0 0 100 0 mass 1\nnode 1 1 100 0 mass 1\nbeam 0 1 m\n"
    );
    let body = parse(&text).unwrap();
    Sim::from_body(Config::default(), &body).unwrap()
}

fn len(sim: &Sim) -> f64 {
    (sim.pos[0] - sim.pos[1]).length()
}

/// Release the beam from rest at length `x` and run 3 s (default dt 0.01, 10 substeps each).
/// The damping makes the pair settle at its rest length within about a second (decay rate
/// 20 per second), so after 3 s the length equals the rest length.
fn release_from(sim: &mut Sim, x: f64) {
    sim.pos[1].x = x;
    for _ in 0..300 {
        sim.advance();
        sim.check().unwrap();
    }
}

#[test]
fn beam_past_yield_keeps_longer_rest() {
    let mut sim = pair(" yield_strain 0.05");
    assert_eq!(sim.beams[0].rest, 1.0);
    release_from(&mut sim, 1.2);
    let rest = sim.beams[0].rest;
    // The rule is rest' = len / (1 + y) at the yield, with len at most 1.2: at most 1.2 / 1.05.
    assert!(rest > 1.05, "rest length {rest} did not grow past the 1.05 m that 5% strain allows");
    assert!(rest <= 1.2 / 1.05 + 1e-12, "rest length {rest} is more than 1.2 / 1.05");
    // Released and settled: the beam rests at its new, longer rest length, not at 1 m.
    assert!((len(&sim) - rest).abs() < 1e-6, "settled length {} is not the new rest {rest}", len(&sim));
    assert!(len(&sim) > 1.05);
}

#[test]
fn beam_squeezed_past_yield_keeps_a_shorter_rest() {
    let mut sim = pair(" yield_strain 0.05");
    release_from(&mut sim, 0.8);
    let rest = sim.beams[0].rest;
    assert!(rest < 0.95, "rest length {rest} did not shrink past the 0.95 m that 5% strain allows");
    assert!(rest >= 0.8 / 0.95 - 1e-12);
    assert!((len(&sim) - rest).abs() < 1e-6);
}

#[test]
fn below_yield_the_rest_length_is_untouched() {
    let mut sim = pair(" yield_strain 0.05");
    release_from(&mut sim, 1.03);
    assert_eq!(sim.beams[0].rest, 1.0, "a 3% stretch must not move the rest length of a 5% beam");
    assert!((len(&sim) - 1.0).abs() < 1e-6, "an elastic return to 1 m was expected");
}

#[test]
fn material_without_yield_strain_never_moves_its_rest_length() {
    let mut sim = pair("");
    release_from(&mut sim, 1.2);
    assert_eq!(sim.beams[0].rest, 1.0);
    assert!((len(&sim) - 1.0).abs() < 1e-6);
    assert!(sim.yield_refused.is_empty());
}

/// The yield rule alone, at the instant it fires: positions and velocities are not touched, the
/// beam is left at the yield limit (stretched: strain +y; crushed: len - rest = -y rest0, the
/// constant yield force, task t-20261122-013607), and the stored elastic energy goes down.
#[test]
fn yield_returns_the_strain_to_the_limit_and_lowers_the_energy() {
    // limit: the wanted strain after the yield (stretched), or the wanted (len - rest) / rest0
    // (crushed; rest0 = 1 m, so it is len - rest in metres).
    for (x, limit) in [(1.2, 0.05), (0.8, -0.05)] {
        let mut sim = pair(" yield_strain 0.05");
        sim.pos[1].x = x;
        let (pos, vel) = (sim.pos.clone(), sim.vel.clone());
        let before = total_energy(&sim.pos, &sim.vel, &sim.beams, &sim.mass, sim.cfg.gravity);
        assert_eq!(sim.apply_yield(), 1);
        let after = total_energy(&sim.pos, &sim.vel, &sim.beams, &sim.mass, sim.cfg.gravity);
        assert_eq!(sim.pos, pos);
        assert_eq!(sim.vel, vel);
        let rest = sim.beams[0].rest;
        let strain = if x > 1.0 { (len(&sim) - rest) / rest } else { (len(&sim) - rest) / sim.rest0[0] };
        assert!((strain - limit).abs() < 1e-12, "strain after yield {strain}, wanted {limit}");
        assert!(after < before, "energy went from {before} to {after} through the yield");
        // A second pass has nothing left to do.
        assert_eq!(sim.apply_yield(), 0);
    }
}

/// Energy never rises through a yield event, in a whole dynamic run: a triangle thrown outwards
/// (so its beams yield) that falls on the ground. Each substep is split in its two halves, the
/// solver step and the yield pass, and the energy is compared across the yield pass only, so
/// the integrator's own energy error cannot hide or fake a rise.
#[test]
fn energy_never_rises_through_yield() {
    let text = "bsn 1\nmaterial m stiffness 20000 damping 20 yield_strain 0.01\n\
node 0 0 0.5 0 mass 1\nnode 1 1 0.5 0 mass 1\nnode 2 0.5 1.3660254037844386 0 mass 1\n\
beam 0 1 m\nbeam 0 2 m\nbeam 1 2 m\n\
group left 0\ngroup right 1\ngroup top 2\n\
velocity left -3 -1 0\nvelocity right 3 -1 0\nvelocity top 0 2 0\n";
    let body = parse(text).unwrap();
    let cfg = Config { dt: 0.001, substeps: 1, ..Config::default() };
    let mut sim = Sim::from_body(cfg, &body).unwrap();
    assert_eq!(sim.vel[0], Vec3::new(-3.0, -1.0, 0.0), "the start velocities come from the file");
    let rest0: Vec<f64> = sim.beams.iter().map(|b| b.rest).collect();
    let mut events = 0;
    let mut removed = 0.0;
    for _ in 0..3000 {
        sim.integrate(0.001);
        let before = total_energy(&sim.pos, &sim.vel, &sim.beams, &sim.mass, sim.cfg.gravity);
        events += sim.apply_yield();
        let after = total_energy(&sim.pos, &sim.vel, &sim.beams, &sim.mass, sim.cfg.gravity);
        assert!(after <= before + 1e-9 * (1.0 + before.abs()), "energy rose through a yield: {before} -> {after}");
        removed += before - after;
    }
    assert!(events > 0, "no beam ever yielded: the test would pass for any rule");
    assert!(removed > 0.0, "yielding removed no energy");
    assert!(sim.beams.iter().zip(&rest0).any(|(b, r)| b.rest != *r), "no rest length changed");
    assert!(sim.beams.iter().all(|b| b.rest > 0.0 && b.rest.is_finite()));
    assert!(sim.yield_refused.is_empty());
}

#[test]
fn zero_length_beam_keeps_its_rest_and_is_reported() {
    let mut sim = pair(" yield_strain 0.05");
    sim.pos[1] = sim.pos[0];
    assert_eq!(sim.apply_yield(), 0);
    assert_eq!(sim.beams[0].rest, 1.0, "a refused yield must not touch the rest length");
    assert_eq!(sim.yield_refused, vec![0]);
    let e = sim.check().unwrap_err();
    assert!(e.contains("beam 0") && e.contains("zero or negative"), "{e}");
    // Squeezed very hard but not to zero: the new rest length is small and positive.
    let mut sim = pair(" yield_strain 0.05");
    sim.pos[1].x = 0.01;
    assert_eq!(sim.apply_yield(), 1);
    assert!(sim.beams[0].rest > 0.0 && sim.beams[0].rest.is_finite());
}

#[test]
fn same_input_gives_the_same_bytes() {
    let text = "bsn 1\nmaterial m stiffness 20000 damping 20 yield_strain 0.01\n\
node 0 0 0.5 0 mass 1\nnode 1 1 0.5 0 mass 1\nnode 2 0.5 1.3660254037844386 0 mass 1\n\
beam 0 1 m\nbeam 0 2 m\nbeam 1 2 m\n\
group left 0\ngroup right 1\nvelocity left -3 0 0\nvelocity right 3 0 0\n";
    let body = parse(text).unwrap();
    let cfg = Config { duration: 3.0, ..Config::default() };
    let (a, _, ra) = run_body_checked_with_contact(&cfg, &body);
    let (b, _, rb) = run_body_checked_with_contact(&cfg, &body);
    ra.unwrap();
    rb.unwrap();
    assert_eq!(to_csv(&a), to_csv(&b));
    assert!(a.len() > 100);
    let (mut s1, mut s2) = (Sim::from_body(cfg.clone(), &body).unwrap(), Sim::from_body(cfg, &body).unwrap());
    for _ in 0..300 {
        s1.advance();
        s2.advance();
    }
    for i in 0..3 {
        assert_eq!(s1.beams[i].rest.to_bits(), s2.beams[i].rest.to_bits());
        assert_eq!(s1.pos[i], s2.pos[i]);
    }
    // The test is only meaningful if something yielded: beam 0 (the pair thrown apart) is
    // not at its start rest length of 1 m.
    assert!((s1.beams[0].rest - 1.0).abs() > 1e-9, "no beam yielded");
}

#[test]
fn a_body_with_a_collider_plane_gets_it_after_the_ground() {
    // nb-36: planes are wired. The sim's planes are the ground, then the body's, in file order.
    let text = "bsn 1
material m stiffness 20000 damping 20
node 0 0 1 0 mass 1
plane 5 0 0 -1 0 0
";
    let body = parse(text).unwrap();
    let sim = Sim::from_body(Config::default(), &body).unwrap();
    assert_eq!(sim.planes.len(), 2);
    assert_eq!(sim.planes[1].normal, crash_sim::vec3::Vec3::new(-1.0, 0.0, 0.0));
}

#[test]
fn a_step_over_the_body_limit_is_refused() {
    let body = parse(&format!("bsn 1\nmaterial m stiffness {K} damping 20\nnode 0 0 1 0 mass 1\nnode 1 1 1 0 mass 1\nbeam 0 1 m\n")).unwrap();
    // w2 = 2 k = 40000, so h = 0.01 gives h^2 w2 = 4: not below the limit.
    let cfg = Config { dt: 0.01, substeps: 1, ..Config::default() };
    let e = Sim::from_body(cfg, &body).err().unwrap();
    assert!(e.contains("stability limit"), "{e}");
}

fn run(args: &[&str]) -> Output {
    Command::new(crash_sim_bin()).args(args).output().unwrap()
}

fn scenario(name: &str) -> String {
    format!("{}/scenarios/{name}", env!("CARGO_MANIFEST_DIR"))
}

/// The default cube's output must not change, and `--body scenarios/cube.bsn` (the same cube as
/// a file) must print the same bytes.
#[test]
fn cube_bsn_output_matches_default() {
    let cube = scenario("cube.bsn");
    for extra in [&[][..], &["--summary"][..], &["--duration", "2", "--dt", "0.02", "--substeps", "20"][..]] {
        let plain = run(extra);
        let mut args = vec!["--body", cube.as_str()];
        args.extend_from_slice(extra);
        let body = run(&args);
        assert!(plain.status.success() && body.status.success(), "{extra:?}");
        assert!(!plain.stdout.is_empty());
        // A body run's CSV has one more column, `broken_beams` (nb-35); drop it and the rest is the cube's output.
        let body_out = String::from_utf8(body.stdout).unwrap();
        let without_column: String = if extra.contains(&"--summary") {
            body_out.clone()
        } else {
            body_out.lines().map(|l| format!("{}
", l.rsplit_once(',').unwrap().0)).collect()
        };
        assert_eq!(String::from_utf8(plain.stdout).unwrap(), without_column, "--body cube.bsn differs from the default cube for {extra:?}");
    }
}

#[test]
fn box_bsn_runs_from_the_command_line() {
    let path = scenario("box.bsn");
    let o = run(&["--body", path.as_str(), "--duration", "2"]);
    assert!(o.status.success(), "{}", String::from_utf8_lossy(&o.stderr));
    let text = String::from_utf8(o.stdout).unwrap();
    assert!(text.starts_with("t,min_y,center_y,max_beam_strain,broken_beams\n"));
    assert_eq!(text.lines().count(), 1 + 201);
}

#[test]
fn body_flag_errors_exit_2_with_empty_stdout() {
    let cube = scenario("cube.bsn");
    let missing = scenario("no_such_file.bsn");
    let cases: Vec<Vec<&str>> = vec![
        vec!["--body", missing.as_str()],
        vec!["--body"],
        vec!["--body", cube.as_str(), "--no-tilt"],
        vec!["--body", cube.as_str(), "--drop-height", "1"],
    ];
    for args in cases {
        let o = run(&args);
        assert_eq!(o.status.code(), Some(2), "{args:?}");
        assert!(o.stdout.is_empty(), "{args:?}");
        assert!(!o.stderr.is_empty(), "{args:?}");
    }
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
        panic!("CARGO_BIN_EXE_crash-sim is not set and {} has no grandparent folder", exe.display())
    });
    let bin = dir.join(name);
    assert!(
        bin.is_file(),
        "CARGO_BIN_EXE_crash-sim is not set and the cargo layout path {} does not exist",
        bin.display()
    );
    bin
}

#[test]
fn nan_strain_beam_is_skipped_not_refused() {
    let mut sim = pair(" yield_strain 0.05");
    sim.pos[1].x = f64::NAN;
    assert_eq!(sim.apply_yield(), 0);
    assert!(sim.yield_refused.is_empty(), "a NaN strain must skip the beam, not refuse it");
    assert_eq!(sim.beams[0].rest, 1.0);
}
