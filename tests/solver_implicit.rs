//! The implicit backward-Euler node-beam solver (src/solver_implicit.rs, docs/M5-SOLVER.md
//! option (1)): it matches the explicit step at small h, stays stable above the explicit step
//! limit (H2), plugs in through the `Solver` trait, is deterministic and fails closed.

use crash_sim::solver::{body_step_error, gershgorin_step_limit, Solver, State, SymplecticEuler};
use crash_sim::solver_implicit::{ImplicitEuler, Tolerances};
use crash_sim::vec3::Vec3;
use crash_sim::*;

const TOL: Tolerances = Tolerances { newton_rtol: 1e-10, newton_max_iter: 50, cg_rtol: 1e-12, cg_max_iter: 2_000 };

/// A free body: a regular tetrahedron of four 1 kg nodes, all six pairs joined by beams of
/// stiffness k, no damping, gravity 0. It starts strained (node 3 pulled out by 2 % of the
/// edge) and moving (a spin about y plus a drift), so the beams vibrate while it rotates.
struct Body {
    pos: Vec<Vec3>,
    vel: Vec<Vec3>,
    beams: Vec<Beam>,
    mass: Vec<f64>,
}

fn tetra(k: f64, damping: f64) -> Body {
    let rest_pos = [
        Vec3::new(1.0, 0.0, -1.0 / 2f64.sqrt()),
        Vec3::new(-1.0, 0.0, -1.0 / 2f64.sqrt()),
        Vec3::new(0.0, 1.0, 1.0 / 2f64.sqrt()),
        Vec3::new(0.0, -1.0, 1.0 / 2f64.sqrt()),
    ];
    let mut beams = Vec::new();
    for a in 0..4 {
        for b in a + 1..4 {
            beams.push(Beam { a, b, rest: (rest_pos[b] - rest_pos[a]).length(), stiffness: k, damping });
        }
    }
    let mut pos = rest_pos.to_vec();
    pos[3] = pos[3] * 1.02;
    let vel = pos.iter().map(|p| Vec3::new(p.z * 0.5, 0.1, -p.x * 0.5)).collect();
    Body { pos, vel, beams, mass: vec![1.0; 4] }
}

/// A straight chain of `n` 1 kg nodes, spacing 1 m, every rest length 1 m, squeezed to 80 %.
fn compressed_chain(n: usize, k: f64) -> Body {
    let pos = (0..n).map(|i| Vec3::new(0.8 * i as f64, 0.0, 0.0)).collect();
    let vel = vec![Vec3::ZERO; n];
    let beams = (0..n - 1).map(|i| Beam { a: i, b: i + 1, rest: 1.0, stiffness: k, damping: 0.0 }).collect();
    Body { pos, vel, beams, mass: vec![1.0; n] }
}

fn step(s: &mut dyn Solver, b: &mut Body, gravity: f64, h: f64) {
    s.step(State { pos: &mut b.pos, vel: &mut b.vel, beams: &b.beams, mass: &b.mass, gravity }, h);
}

fn energy(b: &Body) -> f64 {
    let ke: f64 = (0..b.pos.len()).map(|i| 0.5 * b.mass[i] * b.vel[i].dot(b.vel[i])).sum();
    let pe: f64 = b
        .beams
        .iter()
        .map(|bm| {
            let e = (b.pos[bm.b] - b.pos[bm.a]).length() - bm.rest;
            0.5 * bm.stiffness * e * e
        })
        .sum();
    ke + pe
}

fn implicit(tol: Tolerances) -> ImplicitEuler {
    ImplicitEuler::new(tol).expect("valid tolerances")
}

/// Largest node distance between implicit and symplectic Euler after `t_end` at step h.
fn gap(h: f64, t_end: f64) -> (f64, f64) {
    let (mut a, mut b) = (tetra(1.0e4, 0.0), tetra(1.0e4, 0.0));
    let start = a.pos.clone();
    let (mut ie, mut se) = (implicit(TOL), SymplecticEuler::default());
    let steps = (t_end / h).round() as usize;
    let mut worst: f64 = 0.0;
    let mut moved: f64 = 0.0;
    for _ in 0..steps {
        step(&mut ie, &mut a, 0.0, h);
        step(&mut se, &mut b, 0.0, h);
        for i in 0..4 {
            worst = worst.max((a.pos[i] - b.pos[i]).length());
            moved = moved.max((a.pos[i] - start[i]).length());
        }
    }
    assert_eq!(ie.check(), None);
    assert_eq!(se.check(), None);
    (worst, moved)
}

#[test]
fn implicit_matches_explicit_at_small_h() {
    // Gershgorin omega_max of this body is sqrt(2 x 3k / m) = 245 rad/s; T = 10 ms is about
    // 0.4 of a period. Both integrators are first order, so per unit time they differ by
    // O(omega h) of the vibration: the stated tolerance is omega h (omega T) d0, d0 = the
    // initial pull of node 3 (2 % of |p3| = 0.0245 m), i.e. 2.9e-4 m at h = 20 us and 1.5e-4 m
    // at 10 us; and the gap must halve with h.
    let probe = tetra(1.0e4, 0.0);
    let omega = 2.0 / gershgorin_step_limit(4, &probe.beams, &probe.mass);
    let d0 = 0.02 * probe.pos[3].length() / 1.02;
    let t_end = 0.01;
    let (g1, moved) = gap(2.0e-5, t_end);
    let (g2, _) = gap(1.0e-5, t_end);
    assert!(moved > 1e-3, "the body barely moved: {moved}");
    for (g, h) in [(g1, 2.0e-5), (g2, 1.0e-5)] {
        let tol = omega * h * omega * t_end * d0;
        assert!(g < tol, "implicit vs explicit at h = {h}: gap {g} >= tolerance {tol}");
        assert!(g > 0.0, "implicit and explicit gave the same positions: not two integrators");
    }
    let ratio = g1 / g2;
    assert!((1.6..=2.4).contains(&ratio), "gap is not first order in h: {g1} / {g2} = {ratio}");
}

#[test]
fn implicit_free_fall_is_exact_backward_euler() {
    let mut b = Body {
        pos: vec![Vec3::new(0.0, 10.0, 0.0), Vec3::new(1.0, 5.0, 2.0)],
        vel: vec![Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 3.0, -1.0)],
        beams: vec![],
        mass: vec![2.0, 0.5],
    };
    let mut s = implicit(TOL);
    let h = 1e-3;
    for _ in 0..100 {
        let (p0, v0) = (b.pos.clone(), b.vel.clone());
        step(&mut s, &mut b, 9.81, h);
        for i in 0..2 {
            let v = v0[i] + Vec3::new(0.0, -9.81 * h, 0.0);
            let p = p0[i] + v * h;
            assert!((b.vel[i] - v).length() < 1e-12, "{:?} vs {v:?}", b.vel[i]);
            assert!((b.pos[i] - p).length() < 1e-12, "{:?} vs {p:?}", b.pos[i]);
        }
    }
    assert_eq!(s.check(), None);
}

#[test]
fn implicit_is_stable_above_the_explicit_step_limit() {
    // H2 of docs/M5-SOLVER.md: a stiff body at 5 times the explicit Gershgorin limit.
    let k = 1.0e6;
    let probe = tetra(k, 0.0);
    let limit = gershgorin_step_limit(4, &probe.beams, &probe.mass);
    let h = 5.0 * limit;
    assert!(body_step_error(h, &probe.beams, &probe.mass).is_some(), "the explicit check must refuse h = {h}");

    let e0 = energy(&probe);
    let mut se_body = tetra(k, 0.0);
    let mut se = SymplecticEuler::default();
    let mut blew_up = false;
    for _ in 0..2_000 {
        step(&mut se, &mut se_body, 0.0, h);
        let e = energy(&se_body);
        if se.check().is_some() || e.is_nan() || e >= 1e6 * e0 {
            blew_up = true;
            break;
        }
    }
    assert!(blew_up, "symplectic Euler stayed bounded at 5x its limit: the test body is not stiff enough");

    let mut b = tetra(k, 0.0);
    let mut s = implicit(TOL);
    for n in 0..2_000 {
        step(&mut s, &mut b, 0.0, h);
        assert_eq!(s.check(), None, "implicit step {n} failed");
        let e = energy(&b);
        assert!(e <= e0 * (1.0 + 1e-9), "energy rose at step {n}: {e} > {e0}");
    }
    let st = s.stats();
    assert_eq!(st.steps, 2_000);
    assert!(st.newton_iters >= 2_000 && st.cg_iters > 0);
}

#[test]
fn implicit_solver_plugs_in() {
    // 0.5 s of the default drop (still in the air), through `Sim`, the way `--solver` will.
    let cfg = Config { duration: 0.5, ..Config::default() };
    let steps = (cfg.duration / cfg.dt).round() as u64;
    let mut sim = Sim::with_solver(cfg.clone(), Box::new(implicit(TOL)));
    let mut reference = Sim::new(cfg);
    let (mut a, mut b) = (vec![sim.sample()], vec![reference.sample()]);
    for _ in 0..steps {
        sim.advance();
        reference.advance();
        sim.check().expect("implicit solver stays valid");
        a.push(sim.sample());
        b.push(reference.sample());
    }
    assert_ne!(to_csv(&a), to_csv(&b), "same CSV as symplectic Euler: the implicit solver is not being used");
    // Under a constant force backward Euler's update is symplectic Euler's (v' = v - g h,
    // x' = x + h v'), so the two differ only through the beams' tiny vibration: well under
    // the 2.5 mm (g h t / 2 at t = 0.5 s, h = 1 ms) that either lags the exact fall by.
    let worst = a.iter().zip(&b).map(|(x, y)| (x.center_y - y.center_y).abs()).fold(0.0, f64::max);
    assert!(worst < 5e-3, "implicit solver diverges from symplectic Euler: {worst}");
    let fell = a[0].center_y - a.last().unwrap().center_y;
    assert!((fell - 9.81 * 0.25 / 2.0).abs() < 1e-2, "fell {fell}");
}

#[test]
fn implicit_same_input_same_bytes() {
    let run = || {
        let mut b = tetra(1.0e6, 3.0);
        let mut s = implicit(TOL);
        let h = 4e-3;
        for _ in 0..300 {
            step(&mut s, &mut b, 9.81, h);
        }
        assert_eq!(s.check(), None);
        let bits: Vec<u64> = b.pos.iter().chain(&b.vel).flat_map(|v| [v.x, v.y, v.z]).map(f64::to_bits).collect();
        (bits, s.stats())
    };
    let (a, sa) = run();
    let (b, sb) = run();
    assert_eq!(a, b);
    assert_eq!(sa, sb);
}

/// Runs one step expected to fail; returns the error and checks nothing was written, and that
/// a later step applies nothing either.
fn fails_closed(s: &mut ImplicitEuler, b: &mut Body, h: f64) -> String {
    let (p0, v0) = (b.pos.clone(), b.vel.clone());
    step(s, b, 0.0, h);
    let err = s.check().expect("the step must fail");
    let same = |x: &[Vec3], y: &[Vec3]| x.iter().zip(y).all(|(p, q)| p.x.to_bits() == q.x.to_bits() && p.y.to_bits() == q.y.to_bits() && p.z.to_bits() == q.z.to_bits());
    assert!(same(&b.pos, &p0) && same(&b.vel, &v0), "a failed step wrote the state");
    step(s, b, 0.0, h);
    assert!(same(&b.pos, &p0) && same(&b.vel, &v0), "a step after an error wrote the state");
    assert_eq!(s.check().as_deref(), Some(err.as_str()), "the first error is kept");
    assert_eq!(s.stats().steps, 0);
    err
}

#[test]
fn implicit_newton_cap_fails_closed() {
    // One Newton iteration cannot solve a nonlinear step at 5x the explicit limit to 1e-10.
    let mut b = tetra(1.0e6, 0.0);
    let h = 5.0 * gershgorin_step_limit(4, &b.beams, &b.mass);
    let mut s = implicit(Tolerances { newton_max_iter: 1, ..TOL });
    let err = fails_closed(&mut s, &mut b, h);
    assert!(err.contains("Newton did not converge in 1 iterations") && err.contains("at substep 1"), "{err}");
}

#[test]
fn implicit_cg_cap_fails_closed() {
    let mut b = tetra(1.0e6, 0.0);
    let h = 5.0 * gershgorin_step_limit(4, &b.beams, &b.mass);
    let mut s = implicit(Tolerances { cg_max_iter: 1, ..TOL });
    let err = fails_closed(&mut s, &mut b, h);
    assert!(err.contains("linear solve of Newton iteration 1 failed: pcg not converged after 1 iterations"), "{err}");
}

#[test]
fn implicit_nan_node_fails_closed() {
    let mut b = tetra(1.0e4, 0.0);
    b.pos[2].y = f64::NAN;
    let mut s = implicit(TOL);
    let (p0, v0) = (b.pos.clone(), b.vel.clone());
    step(&mut s, &mut b, 0.0, 1e-3);
    let err = s.check().expect("a NaN node must fail");
    assert_eq!(err, "implicit solver: node 2 is not finite at substep 1");
    for i in 0..4 {
        assert_eq!(b.vel[i], v0[i]);
        if i != 2 {
            assert_eq!(b.pos[i], p0[i]);
        }
    }
    assert!(b.pos[2].y.is_nan());
}

#[test]
fn implicit_refuses_bad_tolerances() {
    let bad = [
        (Tolerances { newton_rtol: 0.0, ..TOL }, "newton_rtol"),
        (Tolerances { newton_rtol: -1e-9, ..TOL }, "newton_rtol"),
        (Tolerances { newton_rtol: f64::NAN, ..TOL }, "newton_rtol"),
        (Tolerances { newton_rtol: f64::INFINITY, ..TOL }, "newton_rtol"),
        (Tolerances { cg_rtol: 0.0, ..TOL }, "cg_rtol"),
        (Tolerances { cg_rtol: f64::NAN, ..TOL }, "cg_rtol"),
        (Tolerances { newton_max_iter: 0, ..TOL }, "newton_max_iter"),
        (Tolerances { cg_max_iter: 0, ..TOL }, "cg_max_iter"),
    ];
    for (t, name) in bad {
        let err = ImplicitEuler::new(t).expect_err(name);
        assert!(err.contains(name), "{err}");
    }
    assert_eq!(implicit(TOL).tolerances(), TOL);
}

#[test]
fn implicit_keeps_matrix_positive_under_compression() {
    // Every beam is compressed 20 %: its geometric term k (len - rest) / len is negative. Kept,
    // it would make A indefinite at this h (h^2 k / 4 = 6.4 > m = 1 per transverse direction),
    // and pcg would refuse it (a non-positive diagonal or Domain).
    let mut b = compressed_chain(20, 1.0e7);
    let h = 5.0 * gershgorin_step_limit(20, &b.beams, &b.mass);
    let e0 = energy(&b);
    let mut s = implicit(TOL);
    for n in 0..200 {
        step(&mut s, &mut b, 0.0, h);
        assert_eq!(s.check(), None, "step {n}");
        assert!(energy(&b) <= e0 * (1.0 + 1e-9), "energy rose at step {n}");
    }
    // The chain springs back toward its rest length and stays on its line.
    let len = b.pos[19].x - b.pos[0].x;
    assert!(len > 0.8 * 19.0, "chain did not expand: {len}");
    assert!(b.pos.iter().all(|p| p.y == 0.0 && p.z == 0.0));
}
