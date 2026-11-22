//! Compressive yield at constant force (task t-20261122-013607, calibration redesign step 4):
//! a beam crushed past its yield strain y flows at the force k * y * rest0 (rest0 = its rest
//! length when the sim was built), however far it is crushed. The old rule, rest' = len / (1 - y),
//! let that force fall with the length (k * y * len / (1 - y)), which made the crumple zone of the
//! validation car go soft as it crushed (scratch/m4-dur/FINDINGS.md). Tensile yield is unchanged.

use crash_sim::format::parse;
use crash_sim::Sim;
use crash_sim::Config;

const K: f64 = 20_000.0;
const Y: f64 = 0.05;

/// Two 1 kg nodes 1 m apart (rest0 = 1 m), 100 m above the ground, one beam of stiffness K with
/// yield strain Y.
fn pair() -> Sim {
    let text = format!(
        "bsn 1\nmaterial m stiffness {K} damping 20 yield_strain {Y}\nnode 0 0 100 0 mass 1\nnode 1 1 100 0 mass 1\nbeam 0 1 m\n"
    );
    Sim::from_body(Config::default(), &parse(&text).unwrap()).unwrap()
}

/// The beam's elastic force k (len - rest), N (negative = compression).
fn spring_force(sim: &Sim) -> f64 {
    let len = (sim.pos[0] - sim.pos[1]).length();
    sim.beams[0].stiffness * (len - sim.beams[0].rest)
}

/// Crush the beam quasi-statically from 1 m to 0.5 m (50%) in 1 mm steps, running the yield pass
/// after each. Once past the yield, the force after the pass must stay within 1% of k y rest0 = 1000 N.
#[test]
fn crushed_beam_keeps_a_constant_yield_force() {
    let mut sim = pair();
    let yield_force = K * Y * 1.0;
    let mut checked = 0;
    let mut worst: f64 = 0.0;
    for step in 1..=500 {
        sim.pos[1].x = 1.0 - step as f64 * 0.001;
        sim.apply_yield();
        let f = spring_force(&sim);
        assert!(f <= 0.0, "a crushed beam pushes: force {f} N at step {step}");
        if step > 60 {
            // Past the 5% yield (50 mm) with margin: the beam is flowing.
            let err = (f.abs() - yield_force).abs() / yield_force;
            worst = worst.max(err);
            assert!(
                err < 0.01,
                "at {:.3} m crush the force is {:.1} N, not within 1% of k y rest0 = {yield_force} N",
                1.0 - sim.pos[1].x,
                f.abs()
            );
            checked += 1;
        }
    }
    assert_eq!(checked, 440);
    eprintln!("worst relative force error over 6%..50% crush: {worst:e}");
}

/// The new rest length is len + y rest0, and a second pass at the same length does nothing.
#[test]
fn compressive_yield_rest_is_len_plus_y_rest0() {
    let mut sim = pair();
    sim.pos[1].x = 0.5;
    assert_eq!(sim.apply_yield(), 1);
    let want = 0.5 + Y * 1.0;
    assert!((sim.beams[0].rest - want).abs() < 1e-12, "rest {} wanted {want}", sim.beams[0].rest);
    assert_eq!(sim.apply_yield(), 0);
}

/// Tensile yield keeps today's rule, rest' = len / (1 + y).
#[test]
fn tensile_yield_is_unchanged() {
    let mut sim = pair();
    sim.pos[1].x = 1.5;
    assert_eq!(sim.apply_yield(), 1);
    let want = 1.5 / (1.0 + Y);
    assert_eq!(sim.beams[0].rest.to_bits(), want.to_bits(), "rest {} wanted {want}", sim.beams[0].rest);
}
