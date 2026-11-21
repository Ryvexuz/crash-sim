//! Property tests for vec3 and the solver, std only (no proptest: ADR 0003 is still Proposed).
//!
//! A small splitmix64 PRNG, a fixed seed list and `CASES` random cases per property. Every
//! failure message starts with the property name, the seed and the case number, so the case
//! can be rebuilt: `Rng::new(seed)` then draw the cases in order (each case draws only from
//! the one `Rng` seeded with `seed ^ case`, see `case_rng`, so case N alone is reproducible).
//!
//! Public API only. Vec3 has no cross product or normalise, so the vec3 properties are the ones
//! that exist: addition commutes, dot(a,a) >= 0, the Cauchy-Schwarz bound |a.b| <= |a||b|, and
//! scaling by 1/|a| gives length 1.

use crash_sim::solver::{body_step_error, gershgorin_omega_max, Solver, State, SymplecticEuler};
use crash_sim::vec3::Vec3;
use crash_sim::Beam;

/// Cases per property per seed.
const CASES: u64 = 200;
/// Fixed seeds (4 seeds x 200 cases = 800 cases per property).
const SEEDS: [u64; 4] = [1, 0xC0FFEE, 0xDEAD_BEEF_1234_5678, 20_261_120];

/// splitmix64 (Vigna): a full-period 64-bit generator with one u64 of state.
struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Rng {
        Rng(seed)
    }
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    /// Uniform in [0, 1) with 53 random bits.
    fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }
    fn below(&mut self, n: usize) -> usize {
        (self.next_u64() % n as u64) as usize
    }
    /// A vector with each component uniform in [-scale, scale].
    fn vec3(&mut self, scale: f64) -> Vec3 {
        Vec3::new(
            self.range(-scale, scale),
            self.range(-scale, scale),
            self.range(-scale, scale),
        )
    }
}

/// The generator for one case; the seed is mixed with the case number so a failing case can
/// be rebuilt on its own.
fn case_rng(seed: u64, case: u64) -> Rng {
    let mut r = Rng::new(seed ^ case.wrapping_mul(0xA24B_AED4_963E_E407));
    r.next_u64(); // spread nearby case numbers apart
    r
}

/// A random node-beam body, far above the y = 0 ground (so no contact), for a gravity-free
/// solver run.
#[derive(Clone)]
struct Body {
    pos: Vec<Vec3>,
    vel: Vec<Vec3>,
    beams: Vec<Beam>,
    mass: Vec<f64>,
    h: f64,
}

/// Height of the body's centre above the ground: far more than the body can travel in a test.
const HEIGHT: f64 = 1.0e6;

fn random_body(r: &mut Rng) -> Body {
    let n = 3 + r.below(6); // 3..=8 nodes
    let mut pos: Vec<Vec3> = Vec::new();
    while pos.len() < n {
        let p = r.vec3(1.0) + Vec3::new(0.0, HEIGHT, 0.0);
        // keep nodes at least 0.2 m apart so every beam has a sensible rest length
        if pos.iter().all(|q| (p - *q).length() > 0.2) {
            pos.push(p);
        }
    }
    let vel: Vec<Vec3> = (0..n).map(|_| r.vec3(1.0)).collect();
    let mass: Vec<f64> = (0..n).map(|_| r.range(0.5, 2.0)).collect();
    let mut beams = Vec::new();
    for a in 0..n {
        for b in (a + 1)..n {
            // a chain a -> a+1 always, so the body is connected; other pairs at random
            if b == a + 1 || r.unit() < 0.5 {
                let len = (pos[a] - pos[b]).length();
                let rest = len * r.range(0.9, 1.1);
                beams.push(Beam {
                    a,
                    b,
                    rest,
                    stiffness: r.range(1_000.0, 20_000.0),
                    damping: r.range(1.0, 20.0),
                });
            }
        }
    }
    // Largest step in a ladder that the crate itself says is stable for this body.
    let mut h = 1.0e-3;
    while body_step_error(h, &beams, &mass).is_some() {
        h *= 0.5;
        assert!(h > 1e-9, "no stable step found for a random body");
    }
    Body {
        pos,
        vel,
        beams,
        mass,
        h,
    }
}

fn step(solver: &mut SymplecticEuler, b: &mut Body, h: f64) {
    solver.step(
        State {
            pos: &mut b.pos,
            vel: &mut b.vel,
            beams: &b.beams,
            mass: &b.mass,
            gravity: 0.0,
        },
        h,
    );
}

fn kinetic(b: &Body) -> f64 {
    b.vel
        .iter()
        .zip(&b.mass)
        .map(|(v, m)| 0.5 * m * v.dot(*v))
        .sum()
}

fn elastic(b: &Body) -> f64 {
    b.beams
        .iter()
        .map(|bm| {
            let ext = (b.pos[bm.b] - b.pos[bm.a]).length() - bm.rest;
            0.5 * bm.stiffness * ext * ext
        })
        .sum()
}

fn momentum(b: &Body) -> Vec3 {
    let mut p = Vec3::ZERO;
    for (v, m) in b.vel.iter().zip(&b.mass) {
        p += *v * *m;
    }
    p
}

// ---------------------------------------------------------------- vec3

#[test]
fn vec3_add_commutes() {
    // Floating-point addition is exactly commutative, so the tolerance is 0 (bit equality).
    for &seed in &SEEDS {
        for case in 0..CASES {
            let mut r = case_rng(seed, case);
            let (a, b) = (r.vec3(1e6), r.vec3(1e6));
            let (ab, ba) = (a + b, b + a);
            assert!(
                ab.x.to_bits() == ba.x.to_bits() && ab.y.to_bits() == ba.y.to_bits() && ab.z.to_bits() == ba.z.to_bits(),
                "vec3_add_commutes: seed {seed} case {case}: a = {a:?}, b = {b:?}, a+b = {ab:?}, b+a = {ba:?}"
            );
        }
    }
}

#[test]
fn vec3_dot_self_is_not_negative() {
    // A sum of three squares: no tolerance needed.
    for &seed in &SEEDS {
        for case in 0..CASES {
            let mut r = case_rng(seed, case);
            let a = r.vec3(1e6);
            let d = a.dot(a);
            assert!(
                d >= 0.0,
                "vec3_dot_self_is_not_negative: seed {seed} case {case}: a = {a:?}, dot(a,a) = {d}"
            );
        }
    }
}

#[test]
fn vec3_cauchy_schwarz() {
    // |a.b| <= |a||b|. Each side is computed with a few roundings (relative error about 1e-15
    // each), so allow 1e-12 relative to |a||b|: 1000 times the rounding, far below any real
    // violation.
    for &seed in &SEEDS {
        for case in 0..CASES {
            let mut r = case_rng(seed, case);
            let (a, b) = (r.vec3(100.0), r.vec3(100.0));
            let lhs = a.dot(b).abs();
            let rhs = a.length() * b.length();
            assert!(
                lhs <= rhs * (1.0 + 1e-12),
                "vec3_cauchy_schwarz: seed {seed} case {case}: a = {a:?}, b = {b:?}, |a.b| = {lhs}, |a||b| = {rhs}"
            );
        }
    }
}

#[test]
fn vec3_unit_vector_has_length_one() {
    // a * (1/|a|) has length 1 within 1e-12 (the brief's tolerance: a few roundings are about
    // 1e-15). Components are kept at least 0.01 so |a| is not near zero.
    for &seed in &SEEDS {
        for case in 0..CASES {
            let mut r = case_rng(seed, case);
            let a = r.vec3(100.0) + Vec3::new(0.01, 0.01, 0.01);
            if a.length() < 1e-3 {
                continue;
            }
            let u = a * (1.0 / a.length());
            let len = u.length();
            assert!((len - 1.0).abs() < 1e-12, "vec3_unit_vector_has_length_one: seed {seed} case {case}: a = {a:?}, |a/|a|| = {len}");
        }
    }
}

// ---------------------------------------------------------------- solver

/// Steps per solver case.
const STEPS: usize = 300;

/// Energy tolerance, as a multiple of (h * omega_max * start energy), where omega_max is the
/// Gershgorin bound on the body's highest angular frequency. Why a tolerance at all: symplectic
/// Euler does not conserve the plain energy KE + PE step to step, it conserves a nearby
/// "shadow" energy, so the plain energy wobbles by a first-order amount, about h * omega * E,
/// even with no damping. With a strict 0 tolerance this very test fails at seed 1 case 3 step
/// 24 (energy 39.2887 -> 39.3734, a 0.16 % rise of the start energy, h = 1e-3), which is the
/// integrator's known behaviour and not a bug. Over all 800 cases x 300 steps the largest
/// rise seen was 0.085 * h * omega_max * start; 0.25 is about 3 times that, and still tight:
/// a solver that injects energy breaks it at once (see the mutation in NOTES).
const ENERGY_TOLERANCE_FACTOR: f64 = 0.25;

#[test]
fn solver_energy_never_rises() {
    // Total mechanical energy = kinetic + beam elastic; damping > 0 on every beam, no gravity,
    // no ground. Per step: e <= previous + tol. Overall: e <= start + tol (the wobble does not
    // pile up, because damping removes energy).
    for &seed in &SEEDS {
        for case in 0..CASES {
            let mut r = case_rng(seed, case);
            let mut b = random_body(&mut r);
            let h = b.h;
            let omega = gershgorin_omega_max(b.pos.len(), &b.beams, &b.mass);
            let mut s = SymplecticEuler::default();
            let mut prev = kinetic(&b) + elastic(&b);
            let start = prev;
            let tol = ENERGY_TOLERANCE_FACTOR * h * omega * start;
            for k in 0..STEPS {
                step(&mut s, &mut b, h);
                let e = kinetic(&b) + elastic(&b);
                assert!(
                    e <= prev + tol && e <= start + tol,
                    "solver_energy_never_rises: seed {seed} case {case} step {k}: energy {prev} -> {e} (rise {}), start {start}, h {h}, tol {tol}",
                    e - prev
                );
                prev = e;
            }
        }
    }
}

#[test]
fn solver_conserves_momentum_without_external_force() {
    // Beam forces come in equal and opposite pairs, so the sum of m*v changes only by
    // rounding. Each step adds a few roundings of size 1e-16 times the largest term, so
    // 1e-9 relative to the sum of |m v| over 300 steps is generous but still far below a
    // real leak (a dropped or doubled force pair changes momentum by about 1e-2 per step).
    for &seed in &SEEDS {
        for case in 0..CASES {
            let mut r = case_rng(seed, case);
            let mut b = random_body(&mut r);
            let h = b.h;
            let mut s = SymplecticEuler::default();
            let p0 = momentum(&b);
            let scale: f64 = b
                .vel
                .iter()
                .zip(&b.mass)
                .map(|(v, m)| m * v.length())
                .sum::<f64>()
                + 1.0;
            for k in 0..STEPS {
                step(&mut s, &mut b, h);
                let drift = (momentum(&b) - p0).length();
                assert!(
                    drift <= 1e-9 * scale,
                    "solver_conserves_momentum: seed {seed} case {case} step {k}: drift {drift}, scale {scale}, p0 {p0:?}"
                );
            }
        }
    }
}

#[test]
fn solver_is_deterministic() {
    // Same body, same steps: the final state must match bit for bit.
    for &seed in &SEEDS {
        for case in 0..CASES {
            let mut r = case_rng(seed, case);
            let start = random_body(&mut r);
            let (mut x, mut y) = (start.clone(), start);
            let h = x.h;
            let (mut sx, mut sy) = (SymplecticEuler::default(), SymplecticEuler::default());
            for _ in 0..STEPS {
                step(&mut sx, &mut x, h);
                step(&mut sy, &mut y, h);
            }
            for i in 0..x.pos.len() {
                let same = |u: Vec3, v: Vec3| {
                    u.x.to_bits() == v.x.to_bits()
                        && u.y.to_bits() == v.y.to_bits()
                        && u.z.to_bits() == v.z.to_bits()
                };
                assert!(
                    same(x.pos[i], y.pos[i]) && same(x.vel[i], y.vel[i]),
                    "solver_is_deterministic: seed {seed} case {case} node {i}: pos {:?} vs {:?}, vel {:?} vs {:?}",
                    x.pos[i], y.pos[i], x.vel[i], y.vel[i]
                );
            }
        }
    }
}

/// Averaging window for the decay property, in steps.
const WINDOW: usize = 50;
/// Horizons (total steps) at which the decay property is checked on the same run.
/// Fraction of the lone-beam damped rate that the energy must at least show (see the test).
const DECAY_FACTOR: f64 = 0.05;
const DECAY_HORIZONS: [usize; 2] = [100, 300];

#[test]
fn solver_energy_decays_with_damping() {
    // Complements the per-step bound above, which is weak (a 1 % per-step velocity injection
    // slips under it for ~290 steps). With damping > 0 on every beam the vibration energy must
    // fall: the mean energy over the last WINDOW steps of a run must be below the mean over the
    // first WINDOW steps, ratio <= exp(-DECAY_FACTOR * gamma * dt), where gamma = min beam
    // damping / max node mass (the damped energy rate of a lone beam, per second), and
    // dt = (horizon - WINDOW) * h is the time between the window centres. Why a factor and not
    // 1: damping acts only along each beam, so some modes of a random body (transverse ones,
    // coupled ones) are damped far more slowly than gamma. Measured over all 800 cases, the
    // smallest -ln(ratio) / (gamma * dt) was 0.42 at horizon 100 and 0.18 at horizon 300
    // (seed 1 case 25), and a first try with 0.5 failed on exactly that case. DECAY_FACTOR is
    // 0.05, about 3.5 times below the smallest value seen, so the bound is "must fall by at
    // least what the weakest damping and heaviest mass force, scaled by 0.05": weak for slowly
    // damped modes, but it scales with the damping and a solver that keeps or adds energy
    // (ratio about 1 or more) breaks it. The body starts at rest (its beams are already
    // stretched or compressed by up to 10 %), so linear and angular momentum are zero:
    // rigid-body motion has no damping and would never decay.
    for &seed in &SEEDS {
        for case in 0..CASES {
            let mut r = case_rng(seed, case);
            let mut b = random_body(&mut r);
            for v in b.vel.iter_mut() {
                *v = Vec3::ZERO;
            }
            let h = b.h;
            let c_min = b
                .beams
                .iter()
                .map(|bm| bm.damping)
                .fold(f64::INFINITY, f64::min);
            let m_max = b.mass.iter().cloned().fold(0.0, f64::max);
            let gamma = c_min / m_max;
            let mut s = SymplecticEuler::default();
            let mut energy = Vec::with_capacity(STEPS);
            for _ in 0..STEPS {
                step(&mut s, &mut b, h);
                energy.push(kinetic(&b) + elastic(&b));
            }
            for &horizon in &DECAY_HORIZONS {
                let mean = |w: &[f64]| w.iter().sum::<f64>() / w.len() as f64;
                let first = mean(&energy[..WINDOW]);
                let last = mean(&energy[horizon - WINDOW..horizon]);
                let bound = (-DECAY_FACTOR * gamma * (horizon - WINDOW) as f64 * h).exp();
                assert!(
                    first > 0.0 && last <= bound * first,
                    "solver_energy_decays_with_damping: seed {seed} case {case} horizon {horizon}: first-window mean {first}, last-window mean {last}, ratio {}, bound {bound}, gamma {gamma}, h {h}",
                    last / first
                );
            }
        }
    }
}
