use crash_sim::format::{parse, write};
use crash_sim::vec3::Vec3;
use crash_sim::{Config, Sim};

// Copied from scenarios/box_wall.bsn; no scenario file is changed by these tests.
const BOX_WALL: &str = "bsn 1
material box_beam stiffness 20000 damping 20 yield_strain 0.02
node 0 0 0 0 mass 1
node 1 0 0 1 mass 1
node 2 0 1 0 mass 1
node 3 0 1 1 mass 1
node 4 1 0 0 mass 1
node 5 1 0 1 mass 1
node 6 1 1 0 mass 1
node 7 1 1 1 mass 1
node 8 2 0 0 mass 1
node 9 2 0 1 mass 1
node 10 2 1 0 mass 1
node 11 2 1 1 mass 1
beam 0 1 box_beam
beam 0 2 box_beam
beam 0 3 box_beam
beam 0 4 box_beam
beam 0 5 box_beam
beam 0 6 box_beam
beam 0 7 box_beam
beam 1 2 box_beam
beam 1 3 box_beam
beam 1 4 box_beam
beam 1 5 box_beam
beam 1 6 box_beam
beam 1 7 box_beam
beam 2 3 box_beam
beam 2 4 box_beam
beam 2 5 box_beam
beam 2 6 box_beam
beam 2 7 box_beam
beam 3 4 box_beam
beam 3 5 box_beam
beam 3 6 box_beam
beam 3 7 box_beam
beam 4 5 box_beam
beam 4 6 box_beam
beam 4 7 box_beam
beam 4 8 box_beam
beam 4 9 box_beam
beam 4 10 box_beam
beam 4 11 box_beam
beam 5 6 box_beam
beam 5 7 box_beam
beam 5 8 box_beam
beam 5 9 box_beam
beam 5 10 box_beam
beam 5 11 box_beam
beam 6 7 box_beam
beam 6 8 box_beam
beam 6 9 box_beam
beam 6 10 box_beam
beam 6 11 box_beam
beam 7 8 box_beam
beam 7 9 box_beam
beam 7 10 box_beam
beam 7 11 box_beam
beam 8 9 box_beam
beam 8 10 box_beam
beam 8 11 box_beam
beam 9 10 box_beam
beam 9 11 box_beam
beam 10 11 box_beam
group back 0 1 2 3
group front 8 9 10 11
group all 0 1 2 3 4 5 6 7 8 9 10 11
velocity all 10 0 0
plane 2.1 0.5 0.5 -1 0 0
";

fn config() -> Config {
    Config {
        duration: 1.0,
        dt: 0.001,
        substeps: 100,
        gravity: 0.0,
        ..Config::default()
    }
}

fn pair_text(tail: &str, rest: &str) -> String {
    format!(
        "bsn 1\nmaterial m stiffness 1000 damping 0{tail}\n\
         node 0 0 5 0 mass 1\nnode 1 1 5 0 mass 1\nbeam 0 1 m{rest}\n"
    )
}

fn sim(text: &str) -> Sim {
    Sim::from_body(config(), &parse(text).unwrap()).unwrap()
}

fn ke(sim: &Sim) -> f64 {
    sim.mass
        .iter()
        .zip(&sim.vel)
        .map(|(m, v)| 0.5 * m * v.dot(*v))
        .sum()
}

fn assert_same_bits(a: &Sim, b: &Sim) {
    assert_eq!(a.pos.len(), b.pos.len());
    assert_eq!(a.vel.len(), b.vel.len());
    assert_eq!(a.beams.len(), b.beams.len());
    for (av, bv) in a.pos.iter().chain(&a.vel).zip(b.pos.iter().chain(&b.vel)) {
        for (x, y) in [(av.x, bv.x), (av.y, bv.y), (av.z, bv.z)] {
            assert_eq!(x.to_bits(), y.to_bits(), "{x} != {y}");
        }
    }
    for (ab, bb) in a.beams.iter().zip(&b.beams) {
        assert_eq!(ab.rest.to_bits(), bb.rest.to_bits());
    }
}

#[test]
fn unload_return_parses_writes_and_refuses() {
    for tail in [
        " yield_strain 0.02 break_strain 0.05 unload_return 0.5",
        "",
        " unload_return 1",
    ] {
        let text = pair_text(tail, "");
        let body = parse(&text).unwrap();
        let expected = match tail {
            "" => None,
            " unload_return 1" => Some(1.0),
            _ => Some(0.5),
        };
        assert_eq!(body.materials[0].unload_return, expected);
        assert_eq!(write(&body), text);
        for version in [2, 3] {
            let text = text.replacen("bsn 1", &format!("bsn {version}"), 1);
            assert_eq!(parse(&text).unwrap(), body);
        }
    }
    for value in ["0", "-0.1", "1.5", "nan", "inf", "-inf", "1e999", "C:\\bad"] {
        let text = pair_text(&format!(" unload_return {value}"), "");
        let e = parse(&text).unwrap_err();
        assert_eq!(e.line, 2);
        assert!(e.message.contains("unload_return"), "{e}");
        let line = text.lines().nth(1).unwrap();
        assert_eq!(e.column, Some(line.rfind(value).unwrap() + 1));
        if ["0", "-0.1", "1.5"].contains(&value) {
            assert_eq!(
                e.message,
                format!("unload_return {value} must be > 0 and <= 1")
            );
        }
    }
    for tail in [
        " unload_return",
        " unload_return 0.5 unload_return 0.5",
        " unload_return 0.5 yield_strain 0.02",
        " unload_return 0.5 break_strain 0.05",
    ] {
        assert_eq!(parse(&pair_text(tail, "")).unwrap_err().line, 2);
    }
    let huge = pair_text(&format!(" unload_return {}", "9".repeat(65_536)), "");
    let e = parse(&huge).unwrap_err();
    assert_eq!(e.line, 2);
    assert!(e.message.contains("unload_return"), "{e}");
    for u in [
        f64::from_bits(1),
        f64::MIN_POSITIVE,
        f64::from_bits(1.0_f64.to_bits() - 1),
    ] {
        let text = pair_text(&format!(" unload_return {u}"), "");
        let body = parse(&text).unwrap();
        assert_eq!(body.materials[0].unload_return, Some(u));
        assert_eq!(sim(&text).unload_return, vec![Some(u)]);
        assert_eq!(write(&body), text);
    }
}

#[test]
fn unload_return_one_is_bit_identical_to_absent() {
    let mut absent = sim(BOX_WALL);
    let mut one = sim(&BOX_WALL.replace("yield_strain 0.02", "yield_strain 0.02 unload_return 1"));
    for _ in 0..400 {
        absent.advance();
        one.advance();
    }
    assert_same_bits(&absent, &one);
}

#[test]
fn unload_leaves_the_loading_stroke_bit_identical() {
    let velocities = "group left 0\ngroup right 1\nvelocity left 1 0 0\nvelocity right -1 0 0\n";
    let mut absent = sim(&(pair_text("", "") + velocities));
    let mut lossy = sim(&(pair_text(" unload_return 0.3", "") + velocities));
    for _ in 0..30 {
        absent.advance();
        lossy.advance();
        assert_same_bits(&absent, &lossy);
    }
    for _ in 30..100 {
        absent.advance();
        lossy.advance();
    }
    assert!(lossy.beams[0].rest < 1.0, "no compression set");
    assert_ne!(lossy.beams[0].rest.to_bits(), absent.beams[0].rest.to_bits());
}

#[test]
fn released_beam_returns_fraction_u() {
    let e0 = 0.5 * 1000.0 * (1.0_f64 - 1.1).powi(2);
    for (tail, fraction) in [
        (" unload_return 0.25", 0.25),
        (" unload_return 0.5", 0.5),
        ("", 1.0),
    ] {
        let mut released = sim(&pair_text(tail, " rest 1.1"));
        let mut peak_ke = 0.0_f64;
        for _ in 0..200 {
            released.advance();
            let kinetic = ke(&released);
            assert!(kinetic.is_finite(), "{tail:?}: non-finite KE={kinetic}");
            peak_ke = peak_ke.max(kinetic);
        }
        let expected = fraction * e0;
        assert!(
            (peak_ke - expected).abs() <= 0.01 * e0,
            "{tail:?}: peak KE={peak_ke}, expected={expected}, E0={e0}"
        );
    }
}

#[test]
fn tiny_return_stops_at_zero_deflection() {
    let e0 = 0.5 * 1000.0 * (1.0_f64 - 1.1).powi(2);
    let mut tiny = sim(&pair_text(" unload_return 0.000001", " rest 1.1"));
    for step in 1..=200 {
        tiny.advance();
        let kinetic = ke(&tiny);
        let total = tiny.total_energy();
        let rest = tiny.beams[0].rest;
        assert!(kinetic <= 0.001 * e0, "step={step}, KE={kinetic}, E0={e0}");
        assert!(
            total <= e0 * (1.0 + 1e-9),
            "step={step}, E={total}, E0={e0}"
        );
        assert!(
            rest.is_finite() && (0.99..=1.1).contains(&rest),
            "rest={rest}"
        );
    }

    // Direct entry point, with exact rational references, independent of the solver:
    // len=1, delta=+/-1/16; beta=1 or 3 gives rest'=19/16, 13/16 or 17/16.
    for (rest, speed, u, expected) in [
        (1.25, 0.125, 0.5, 1.1875_f64),
        (0.75, -0.125, 0.5, 0.8125),
        (1.25, 0.125, 0.25, 1.0625),
        (1.25, 1.0, 0.5, 1.0),
        (0.75, -1.0, 0.5, 1.0),
    ] {
        let mut direct = sim(&pair_text(&format!(" unload_return {u}"), ""));
        direct.beams[0].rest = rest;
        direct.vel[1].x = speed;
        let (pos, vel, rest0) = (
            direct.pos.clone(),
            direct.vel.clone(),
            direct.rest0.clone(),
        );
        let before = 0.5 * speed * speed + 0.5 * 1000.0 * (1.0_f64 - rest).powi(2);
        assert_eq!(direct.apply_unload(0.5), 1);
        assert_eq!(direct.beams[0].rest.to_bits(), expected.to_bits());
        assert!(direct.total_energy() <= before);
        assert_eq!(direct.pos, pos);
        assert_eq!(direct.vel, vel);
        assert_eq!(direct.rest0, rest0);
        assert_eq!(direct.beams[0].stiffness, 1000.0);
        assert_eq!(direct.beams[0].damping, 0.0);
    }
    // Small accepted u values: even a reciprocal/shift overflow must cap at len=1.
    for u in [f64::from_bits(1), f64::MIN_POSITIVE, 0.5] {
        let mut direct = sim(&pair_text("", ""));
        direct.unload_return[0] = Some(u);
        direct.beams[0].rest = f64::MAX;
        direct.vel[1].x = f64::MAX;
        assert_eq!(direct.apply_unload(2.0), 1);
        assert_eq!(direct.beams[0].rest.to_bits(), 1.0_f64.to_bits());
    }
    // Exact powers of two, near the length cutoff and near the squared-length
    // overflow band: x=-len, delta=len, so the exact capped reference is len.
    for exponent in [-39, 500] {
        let length = 2.0_f64.powi(exponent);
        let mut direct = sim(&pair_text(" unload_return 0.5", ""));
        direct.pos[1].x = length;
        direct.beams[0].rest = 2.0 * length;
        direct.vel[1].x = length;
        assert_eq!(direct.apply_unload(1.0), 1);
        assert_eq!(direct.beams[0].rest.to_bits(), length.to_bits());
    }
    // At u=1 and its predecessor, the exact shift is zero or too small to move
    // rest=5/4: delta=1/16, beta=0 or 2^-52, shift=0 or 2^-56.
    for u in [1.0, f64::from_bits(1.0_f64.to_bits() - 1)] {
        let mut direct = sim(&pair_text(&format!(" unload_return {u}"), ""));
        direct.beams[0].rest = 1.25;
        direct.vel[1].x = 0.125;
        assert_eq!(direct.apply_unload(0.5), 0);
        assert_eq!(direct.beams[0].rest.to_bits(), 1.25_f64.to_bits());
    }
    // Exact dyadic near-cancellation: one ulp of compression, capped to zero.
    let mut direct = sim(&pair_text(" unload_return 0.5", ""));
    direct.beams[0].rest = f64::from_bits(1.0_f64.to_bits() + 1);
    direct.vel[1].x = 1.0;
    assert_eq!(direct.apply_unload(1.0), 1);
    assert_eq!(direct.beams[0].rest.to_bits(), 1.0_f64.to_bits());
    // Subnormal delta and underflow round to no representable rest change.
    for h in [1.0, 0.5] {
        direct.beams[0].rest = 1.25;
        direct.vel[1].x = f64::from_bits(1);
        assert_eq!(direct.apply_unload(h), 0);
        assert_eq!(direct.beams[0].rest.to_bits(), 1.25_f64.to_bits());
    }
    // Non-finite/collapsed geometry and NaN rate are skipped at the pass itself.
    for length in [
        0.0,
        f64::from_bits(1),
        f64::MAX,
        f64::INFINITY,
        f64::NAN,
    ] {
        direct.pos[1].x = length;
        assert_eq!(direct.apply_unload(1.0), 0);
        assert_eq!(direct.beams[0].rest.to_bits(), 1.25_f64.to_bits());
    }
    direct.pos[1].x = 1.0;
    direct.vel[1] = Vec3::new(f64::NAN, 0.0, 0.0);
    assert_eq!(direct.apply_unload(1.0), 0);
    assert_eq!(direct.beams[0].rest.to_bits(), 1.25_f64.to_bits());
}

#[test]
fn both_constructors_map_unload_return_per_beam() {
    let text = BOX_WALL.replace("yield_strain 0.02", "yield_strain 0.02 unload_return 0.4");
    let mapped = sim(&text);
    assert_eq!(mapped.unload_return, vec![Some(0.4); 50]);
    let mixed = sim(
        "bsn 1\nmaterial a stiffness 1000 damping 0 unload_return 0.5\n\
         material b stiffness 1000 damping 0\n\
         material c stiffness 1000 damping 0 unload_return 0.25\n\
         node 0 0 5 0 mass 1\nnode 1 1 5 0 mass 1\nnode 2 2 5 0 mass 1\n\
         beam 0 1 c\nbeam 1 2 b\nbeam 0 2 a\n",
    );
    assert_eq!(mixed.unload_return, vec![Some(0.25), None, Some(0.5)]);
    let text = std::fs::read_to_string(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/scenarios/car_basic.bsn"
    ))
    .unwrap()
    .replace(
        "material chassis stiffness 300000 damping 200 yield_strain 0.02",
        "material chassis stiffness 300000 damping 200 yield_strain 0.02 unload_return 0.4",
    );
    let body = parse(&text).unwrap();
    assert_eq!(body.materials.len(), 1);
    let car = Sim::from_car(Config::default(), &body).unwrap();
    assert!(!car.beams.is_empty());
    assert_eq!(car.unload_return, vec![Some(0.4); car.beams.len()]);
    let cube = Sim::new(Config::default());
    assert!(!cube.beams.is_empty());
    assert_eq!(cube.unload_return, vec![None; cube.beams.len()]);
    for x in [0.0, -0.1, 1.5, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let mut body = parse(&pair_text("", "")).unwrap();
        body.materials[0].unload_return = Some(x);
        for with_car in [false, true] {
            let result = if with_car {
                Sim::from_car(config(), &body)
            } else {
                Sim::from_body(config(), &body)
            };
            let e = result.err().expect("invalid unload_return accepted");
            assert_eq!(
                e,
                format!("beam 0: unload_return {x} must be a number > 0 and <= 1")
            );
        }
    }
}

#[test]
fn broken_beam_is_left_alone() {
    let mut broken = sim(&pair_text(" break_strain 0.05 unload_return 0.5", " rest 1.2"));
    // One solver step supplies outward velocity, then the break precedes unloading.
    broken.integrate(0.00001);
    // Exercise the lower-level pass's broken guard without waiting for advance.
    broken.broken[0] = true;
    assert_eq!(broken.apply_unload(0.00001), 0);
    assert_eq!(broken.beams[0].rest.to_bits(), 1.2_f64.to_bits());
    let mut broken = sim(&pair_text(" break_strain 0.05 unload_return 0.5", " rest 1.2"));
    for step in 1..=200 {
        broken.advance();
        assert!(broken.broken[0], "step={step}");
        assert_eq!(broken.breaks.len(), 1);
        assert_eq!(broken.breaks[0].t.to_bits(), 0.00001_f64.to_bits());
        assert_eq!(broken.beams[0].rest.to_bits(), 1.2_f64.to_bits());
        assert!(broken.pos[1].x - broken.pos[0].x > 1.0);
    }
}
