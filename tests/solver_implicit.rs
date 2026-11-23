//! The implicit backward-Euler node-beam solver (src/solver_implicit.rs, docs/M5-SOLVER.md
//! option (1)): it matches the explicit step at small h, stays stable above the explicit step
//! limit (H2), plugs in through the `Solver` trait, is deterministic and fails closed.

use crash_sim::solver::{
    body_step_error, gershgorin_step_limit, step_error_with, Solver, State, SymplecticEuler,
};
use crash_sim::solver_implicit::{ImplicitEuler, Tolerances, M5_TOLERANCES};
use crash_sim::solver_m5::SolverM5;
use crash_sim::vec3::Vec3;
use crash_sim::*;

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
            beams.push(Beam {
                a,
                b,
                rest: (rest_pos[b] - rest_pos[a]).length(),
                stiffness: k,
                damping,
            });
        }
    }
    let mut pos = rest_pos.to_vec();
    pos[3] = pos[3] * 1.02;
    let vel = pos
        .iter()
        .map(|p| Vec3::new(p.z * 0.5, 0.1, -p.x * 0.5))
        .collect();
    Body {
        pos,
        vel,
        beams,
        mass: vec![1.0; 4],
    }
}

/// A straight chain of `n` 1 kg nodes, spacing 1 m, every rest length 1 m, squeezed to 80 %.
fn compressed_chain(n: usize, k: f64) -> Body {
    let pos = (0..n)
        .map(|i| Vec3::new(0.8 * i as f64, 0.0, 0.0))
        .collect();
    let vel = vec![Vec3::ZERO; n];
    let beams = (0..n - 1)
        .map(|i| Beam {
            a: i,
            b: i + 1,
            rest: 1.0,
            stiffness: k,
            damping: 0.0,
        })
        .collect();
    Body {
        pos,
        vel,
        beams,
        mass: vec![1.0; n],
    }
}

fn step(s: &mut dyn Solver, b: &mut Body, gravity: f64, h: f64) {
    s.step(
        State {
            pos: &mut b.pos,
            vel: &mut b.vel,
            beams: &b.beams,
            mass: &b.mass,
            gravity,
        },
        h,
    );
}

fn energy(b: &Body) -> f64 {
    let ke: f64 = (0..b.pos.len())
        .map(|i| 0.5 * b.mass[i] * b.vel[i].dot(b.vel[i]))
        .sum();
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
    let (mut ie, mut se) = (implicit(M5_TOLERANCES), SymplecticEuler::default());
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
        assert!(
            g < tol,
            "implicit vs explicit at h = {h}: gap {g} >= tolerance {tol}"
        );
        assert!(
            g > 0.0,
            "implicit and explicit gave the same positions: not two integrators"
        );
    }
    let ratio = g1 / g2;
    assert!(
        (1.6..=2.4).contains(&ratio),
        "gap is not first order in h: {g1} / {g2} = {ratio}"
    );
}

#[test]
fn implicit_free_fall_is_exact_backward_euler() {
    let mut b = Body {
        pos: vec![Vec3::new(0.0, 10.0, 0.0), Vec3::new(1.0, 5.0, 2.0)],
        vel: vec![Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 3.0, -1.0)],
        beams: vec![],
        mass: vec![2.0, 0.5],
    };
    let mut s = implicit(M5_TOLERANCES);
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
    assert!(
        body_step_error(h, &probe.beams, &probe.mass).is_some(),
        "the explicit check must refuse h = {h}"
    );

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
    assert!(
        blew_up,
        "symplectic Euler stayed bounded at 5x its limit: the test body is not stiff enough"
    );

    let mut b = tetra(k, 0.0);
    let mut s = implicit(M5_TOLERANCES);
    for n in 0..2_000 {
        step(&mut s, &mut b, 0.0, h);
        assert_eq!(s.check(), None, "implicit step {n} failed");
        let e = energy(&b);
        assert!(
            e <= e0 * (1.0 + 1e-9),
            "energy rose at step {n}: {e} > {e0}"
        );
    }
    let st = s.stats();
    assert_eq!(st.steps, 2_000);
    assert!(st.newton_iters >= 2_000 && st.cg_iters > 0);
}

#[test]
fn implicit_solver_plugs_in() {
    // 0.5 s of the default drop (still in the air), through `Sim`, the way `--solver` will.
    let cfg = Config {
        duration: 0.5,
        ..Config::default()
    };
    let steps = (cfg.duration / cfg.dt).round() as u64;
    let mut sim = Sim::with_solver(cfg.clone(), Box::new(implicit(M5_TOLERANCES)));
    let mut reference = Sim::new(cfg);
    let (mut a, mut b) = (vec![sim.sample()], vec![reference.sample()]);
    for _ in 0..steps {
        sim.advance();
        reference.advance();
        sim.check().expect("implicit solver stays valid");
        a.push(sim.sample());
        b.push(reference.sample());
    }
    assert_ne!(
        to_csv(&a),
        to_csv(&b),
        "same CSV as symplectic Euler: the implicit solver is not being used"
    );
    // Under a constant force backward Euler's update is symplectic Euler's (v' = v - g h,
    // x' = x + h v'), so the two differ only through the beams' tiny vibration: well under
    // the 2.5 mm (g h t / 2 at t = 0.5 s, h = 1 ms) that either lags the exact fall by.
    let worst = a
        .iter()
        .zip(&b)
        .map(|(x, y)| (x.center_y - y.center_y).abs())
        .fold(0.0, f64::max);
    assert!(
        worst < 5e-3,
        "implicit solver diverges from symplectic Euler: {worst}"
    );
    let fell = a[0].center_y - a.last().unwrap().center_y;
    assert!((fell - 9.81 * 0.25 / 2.0).abs() < 1e-2, "fell {fell}");
}

#[test]
fn implicit_same_input_same_bytes() {
    let run = || {
        let mut b = tetra(1.0e6, 3.0);
        let mut s = implicit(M5_TOLERANCES);
        let h = 4e-3;
        for _ in 0..300 {
            step(&mut s, &mut b, 9.81, h);
        }
        assert_eq!(s.check(), None);
        let bits: Vec<u64> = b
            .pos
            .iter()
            .chain(&b.vel)
            .flat_map(|v| [v.x, v.y, v.z])
            .map(f64::to_bits)
            .collect();
        (bits, s.stats())
    };
    let (a, sa) = run();
    let (b, sb) = run();
    assert_eq!(a, b);
    assert_eq!(sa, sb);
}

#[test]
fn implicit_creates_no_energy() {
    // A free, undamped, gravity-free tetrahedron, strained 1e-3 (node 3 stretched, node 2
    // sheared) and carrying a rigid velocity. Backward Euler is dissipative, so total energy
    // may never rise above its start by more than 1e-6 relative (the allowance for Newton's
    // 1e-10 tolerance and the beam energy being non-convex at strain 1e-3), and the internal
    // forces sum to zero, so total linear momentum stays within 1e-9 of its scale. Angular
    // momentum is not exactly conserved by backward Euler and is not asserted.
    let rigid = Vec3::new(0.3, 0.1, -0.2);
    let start = || {
        let mut b = tetra(1.0e4, 0.0);
        let rest = tetra(1.0e4, 0.0);
        b.pos[3] = rest.pos[3] * (1.001 / 1.02);
        b.pos[2] = rest.pos[2] + Vec3::new(1.0e-3, 0.0, 0.0);
        b.vel = vec![rigid; 4];
        b
    };
    let probe = start();
    let limit = gershgorin_step_limit(4, &probe.beams, &probe.mass);
    let e0 = energy(&probe);
    assert!(e0 > 0.0);
    let scale = probe.mass.iter().sum::<f64>() * rigid.length();
    let momentum = |b: &Body| (0..4).fold(Vec3::ZERO, |p, i| p + b.vel[i] * b.mass[i]);
    let p0 = momentum(&probe);
    for factor in [0.1, 5.0] {
        let h = factor * limit;
        let mut b = start();
        let mut s = implicit(M5_TOLERANCES);
        for n in 0..2_000 {
            step(&mut s, &mut b, 0.0, h);
            assert_eq!(s.check(), None, "h = {factor} x limit, substep {n} failed");
            let e = energy(&b);
            assert!(
                e <= e0 * (1.0 + 1e-6),
                "h = {factor} x limit: energy rose at substep {n}: {e} > {e0}"
            );
            let dp = (momentum(&b) - p0).length();
            assert!(
                dp <= 1e-9 * scale,
                "h = {factor} x limit: momentum drifted {dp} at substep {n}"
            );
        }
    }
}

/// Runs one step expected to fail; returns the error and checks nothing was written, and that
/// a later step applies nothing either.
fn fails_closed(s: &mut ImplicitEuler, b: &mut Body, h: f64) -> String {
    let (p0, v0) = (b.pos.clone(), b.vel.clone());
    step(s, b, 0.0, h);
    let err = s.check().expect("the step must fail");
    let same = |x: &[Vec3], y: &[Vec3]| {
        x.iter().zip(y).all(|(p, q)| {
            p.x.to_bits() == q.x.to_bits()
                && p.y.to_bits() == q.y.to_bits()
                && p.z.to_bits() == q.z.to_bits()
        })
    };
    assert!(
        same(&b.pos, &p0) && same(&b.vel, &v0),
        "a failed step wrote the state"
    );
    step(s, b, 0.0, h);
    assert!(
        same(&b.pos, &p0) && same(&b.vel, &v0),
        "a step after an error wrote the state"
    );
    assert_eq!(
        s.check().as_deref(),
        Some(err.as_str()),
        "the first error is kept"
    );
    assert_eq!(s.stats().steps, 0);
    err
}

#[test]
fn implicit_newton_cap_fails_closed() {
    // One Newton iteration cannot solve a nonlinear step at 5x the explicit limit to 1e-10.
    let mut b = tetra(1.0e6, 0.0);
    let h = 5.0 * gershgorin_step_limit(4, &b.beams, &b.mass);
    let mut s = implicit(Tolerances {
        newton_max_iter: 1,
        ..M5_TOLERANCES
    });
    let err = fails_closed(&mut s, &mut b, h);
    assert!(
        err.contains("Newton did not converge in 1 iterations") && err.contains("at substep 1"),
        "{err}"
    );
}

#[test]
fn implicit_cg_cap_fails_closed() {
    let mut b = tetra(1.0e6, 0.0);
    let h = 5.0 * gershgorin_step_limit(4, &b.beams, &b.mass);
    let mut s = implicit(Tolerances {
        cg_max_iter: 1,
        ..M5_TOLERANCES
    });
    let err = fails_closed(&mut s, &mut b, h);
    assert!(
        err.contains(
            "linear solve of Newton iteration 1 failed: pcg not converged after 1 iterations"
        ),
        "{err}"
    );
}

#[test]
fn implicit_nan_node_fails_closed() {
    let mut b = tetra(1.0e4, 0.0);
    b.pos[2].y = f64::NAN;
    let mut s = implicit(M5_TOLERANCES);
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
        (
            Tolerances {
                newton_rtol: 0.0,
                ..M5_TOLERANCES
            },
            "newton_rtol",
        ),
        (
            Tolerances {
                newton_rtol: -1e-9,
                ..M5_TOLERANCES
            },
            "newton_rtol",
        ),
        (
            Tolerances {
                newton_rtol: f64::NAN,
                ..M5_TOLERANCES
            },
            "newton_rtol",
        ),
        (
            Tolerances {
                newton_rtol: f64::INFINITY,
                ..M5_TOLERANCES
            },
            "newton_rtol",
        ),
        (
            Tolerances {
                cg_rtol: 0.0,
                ..M5_TOLERANCES
            },
            "cg_rtol",
        ),
        (
            Tolerances {
                cg_rtol: f64::NAN,
                ..M5_TOLERANCES
            },
            "cg_rtol",
        ),
        (
            Tolerances {
                newton_max_iter: 0,
                ..M5_TOLERANCES
            },
            "newton_max_iter",
        ),
        (
            Tolerances {
                cg_max_iter: 0,
                ..M5_TOLERANCES
            },
            "cg_max_iter",
        ),
    ];
    for (t, name) in bad {
        let err = ImplicitEuler::new(t).expect_err(name);
        assert!(err.contains(name), "{err}");
    }
    assert_eq!(implicit(M5_TOLERANCES).tolerances(), M5_TOLERANCES);
}

#[test]
fn implicit_keeps_matrix_positive_under_compression() {
    // Every beam is compressed 20 %: its geometric term k (len - rest) / len is negative. Kept,
    // it would make A indefinite at this h (h^2 k / 4 = 6.4 > m = 1 per transverse direction),
    // and pcg would refuse it (a non-positive diagonal or Domain).
    let mut b = compressed_chain(20, 1.0e7);
    let h = 5.0 * gershgorin_step_limit(20, &b.beams, &b.mass);
    let e0 = energy(&b);
    let mut s = implicit(M5_TOLERANCES);
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

#[test]
fn implicit_solves_a_slightly_strained_stiff_beam() {
    // Review r1 (solver_implicit.rs:18): two 1 kg nodes at rest, one stiff beam stretched by a
    // strain of 1e-11. The whole residual is the small beam force; the step must still solve
    // backward Euler, not return v' = v. Exact answer (1D, reduced mass mu = 0.5, w = relative
    // velocity, u = stretch): mu w' = -h k (u + h w'), so w' = -h k u / (mu + h^2 k).
    let (k, u, h) = (1.0e9, 1.0e-11, 1.0e-3);
    let mut b = Body {
        pos: vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0 + u, 0.0, 0.0)],
        vel: vec![Vec3::ZERO; 2],
        beams: vec![Beam {
            a: 0,
            b: 1,
            rest: 1.0,
            stiffness: k,
            damping: 0.0,
        }],
        mass: vec![1.0; 2],
    };
    let u = b.pos[1].x - 1.0; // the stretch as stored
    let w = -h * k * u / (0.5 + h * h * k);
    let mut s = implicit(M5_TOLERANCES);
    step(&mut s, &mut b, 0.0, h);
    assert_eq!(s.check(), None);
    let got = b.vel[1].x - b.vel[0].x;
    assert!(got != 0.0, "the step returned the start velocity unsolved");
    assert!(
        (got - w).abs() <= 1e-6 * w.abs(),
        "relative velocity {got} vs backward Euler {w}"
    );
    assert!(s.stats().newton_iters >= 1);
}

// ---------------------------------------------------------------------------------------
// The solver-aware step limit of `Sim` (task t-20261123-202940, R3): `Solver::explicit_beams`.

/// The suffix `Sim` adds to the step-limit error of a body with wheels.
const CAR_SUFFIX: &str = " (the chassis with its wheels, suspension and tyres)";

/// A `.bsn` body (`crash_sim::Body`; this file's own `Body` is the bare solver state).
fn bsn(text: &str) -> crash_sim::Body {
    crash_sim::format::parse(text).expect("a valid body")
}

/// A 2 x 2 x 2 cube of 1 kg nodes, lowest face at y = 0.5 m, every node pair joined by a stiff
/// beam (k = 1e6 N/m, c = 1 N s/m). No wheels and no tets.
fn stiff_cube() -> crash_sim::Body {
    let mut s = String::from("bsn 1\nmaterial m stiffness 1000000 damping 1\n");
    for i in 0..8 {
        let (x, y, z) = (i & 1, (i >> 1) & 1, (i >> 2) & 1);
        s.push_str(&format!("node {i} {x} {} {z} mass 1\n", f64::from(y) + 0.5));
    }
    for a in 0..8 {
        for b in a + 1..8 {
            s.push_str(&format!("beam {a} {b} m\n"));
        }
    }
    bsn(&s)
}

/// scenarios/car_basic.bsn: a body with wheels.
fn car_basic() -> crash_sim::Body {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/scenarios/car_basic.bsn");
    bsn(&std::fs::read_to_string(path).expect("scenarios/car_basic.bsn"))
}

/// A config whose physics step dt / substeps is `h` (to rounding), with dt >= `MIN_DT`, and the
/// step it really gives.
fn cfg_at(h: f64, gravity: f64) -> (Config, f64) {
    let substeps = (MIN_DT / h).ceil().max(1.0) as u32;
    let cfg = Config {
        duration: 1.0,
        dt: h * f64::from(substeps),
        substeps,
        gravity,
        ..Config::default()
    };
    let h_eff = cfg.dt / cfg.substeps as f64;
    assert!(
        cfg.dt >= MIN_DT && (h_eff - h).abs() <= 1e-12 * h,
        "cfg for h = {h}"
    );
    (cfg, h_eff)
}

/// The step at which `refused` turns true, by bisection (the bounds are monotone in h).
fn limit_of(refused: impl Fn(f64) -> bool) -> f64 {
    let (mut lo, mut hi) = (1e-9, 1.0);
    assert!(
        !refused(lo) && refused(hi),
        "the limit is not between {lo} and {hi}"
    );
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if refused(mid) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    hi
}

/// The car's chassis beams and node masses and its wheel extras, as `Sim::from_car` builds them.
struct CarLimit {
    beams: Vec<Beam>,
    mass: Vec<f64>,
    extra_mass: Vec<f64>,
    elements: Vec<crash_sim::solver::LimitElement>,
}

fn car_limit(body: &crash_sim::Body) -> CarLimit {
    let built = body.build();
    let car = crash_sim::vehicle::Car::new(
        body,
        &built.pos,
        &built.vel,
        &built.mass,
        1 + body.planes.len(),
        9.81,
    )
    .expect("the car is valid");
    let (extra_mass, elements) = car.limit_elements(built.mass.len());
    CarLimit {
        beams: built.beams,
        mass: built.mass,
        extra_mass,
        elements,
    }
}

/// A solver that does not override `explicit_beams`: it must get the default, the strict side.
struct DefaultSolver(SymplecticEuler);

impl Solver for DefaultSolver {
    fn step(&mut self, s: State<'_>, h: f64) {
        self.0.step(s, h);
    }
}

fn err_of(r: Result<Sim, String>) -> String {
    match r {
        Ok(_) => panic!("the sim was built; a refusal was expected"),
        Err(e) => e,
    }
}

#[test]
fn implicit_sim_runs_above_explicit_beam_limit() {
    let body = stiff_cube();
    let built = body.build();
    let limit = gershgorin_step_limit(built.mass.len(), &built.beams, &built.mass);
    let (cfg, h) = cfg_at(5.0 * limit, 9.81);
    let expected = body_step_error(h, &built.beams, &built.mass)
        .expect("the explicit check refuses 5 x its limit");

    // Explicit: refused with exactly `body_step_error`'s text, by both body constructors.
    let se = Sim::from_body_with_solver(cfg.clone(), &body, Box::new(SymplecticEuler::default()));
    assert_eq!(err_of(se), expected);
    assert_eq!(err_of(Sim::from_body(cfg.clone(), &body)), expected);

    // Implicit: built, and 200 substeps stay finite and valid (it falls 0.5 m onto the ground).
    let ie = Box::new(implicit(M5_TOLERANCES));
    let mut sim = Sim::from_body_with_solver(cfg.clone(), &body, ie).expect("implicit is accepted");
    let start = sim.pos.clone();
    let mut done = 0;
    while done < 200 {
        sim.advance();
        done += cfg.substeps;
        sim.check().expect("the implicit run stays valid");
    }
    assert!(sim
        .pos
        .iter()
        .chain(&sim.vel)
        .all(|v| v.x.is_finite() && v.y.is_finite() && v.z.is_finite()));
    assert!(
        sim.first_contact_s.is_some(),
        "the body never reached the ground"
    );
    let moved = sim
        .pos
        .iter()
        .zip(&start)
        .map(|(p, q)| (*p - *q).length())
        .fold(0.0, f64::max);
    assert!(moved > 0.1, "the body barely moved: {moved}");
    // The same through `from_car_with_solver` (a body without wheels is `from_body`'s sim).
    let ie = Box::new(implicit(M5_TOLERANCES));
    assert!(Sim::from_car_with_solver(cfg, &body, ie).is_ok());
}

#[test]
fn explicit_step_limit_unchanged() {
    // Body path: SymplecticEuler, SolverM5 and a solver with the default are all refused with
    // exactly `body_step_error`'s text, by `from_body_with_solver` and `from_car_with_solver`.
    let body = stiff_cube();
    let built = body.build();
    let limit = gershgorin_step_limit(built.mass.len(), &built.beams, &built.mass);
    let (cfg, h) = cfg_at(5.0 * limit, 9.81);
    let expected = body_step_error(h, &built.beams, &built.mass).expect("refused");
    let solvers = |b: &crash_sim::Body, small_h: f64| -> Vec<(&str, Box<dyn Solver>)> {
        vec![
            ("SymplecticEuler", Box::new(SymplecticEuler::default())),
            (
                "SolverM5",
                Box::new(SolverM5::from_body(b, small_h).expect("m5 at a small h")),
            ),
            (
                "default",
                Box::new(DefaultSolver(SymplecticEuler::default())),
            ),
        ]
    };
    for (name, s) in solvers(&body, 0.1 * limit) {
        let explicit = s.explicit_beams();
        let got = err_of(Sim::from_body_with_solver(cfg.clone(), &body, s));
        assert_eq!(got, expected, "{name}, from_body_with_solver");
        assert!(explicit, "{name}");
    }
    for (name, s) in solvers(&body, 0.1 * limit) {
        let got = err_of(Sim::from_car_with_solver(cfg.clone(), &body, s));
        assert_eq!(got, expected, "{name}, from_car_with_solver");
    }
    // Just below the limit all three are accepted (the refusal above is the limit, not a solver).
    let (below, _) = cfg_at(0.9 * limit, 9.81);
    for (name, s) in solvers(&body, 0.1 * limit) {
        assert!(
            Sim::from_body_with_solver(below.clone(), &body, s).is_ok(),
            "{name}"
        );
    }

    // Car path: refused with `step_error_with` over the beams and the wheels, plus the suffix.
    let car = car_basic();
    let cl = car_limit(&car);
    let all = |h: f64| step_error_with(h, &cl.beams, &cl.mass, &cl.extra_mass, &cl.elements);
    let l_all = limit_of(|h| all(h).is_some());
    let (cfg, h) = cfg_at(2.0 * l_all, 9.81);
    let expected = format!("{}{CAR_SUFFIX}", all(h).expect("refused"));
    for (name, s) in solvers(&car, 0.1 * l_all) {
        let got = err_of(Sim::from_car_with_solver(cfg.clone(), &car, s));
        assert_eq!(got, expected, "{name}, car");
    }
    assert_eq!(err_of(Sim::from_car(cfg, &car)), expected);
}

#[test]
fn implicit_car_keeps_wheel_limit() {
    let car = car_basic();
    let cl = car_limit(&car);
    let all = |h: f64| step_error_with(h, &cl.beams, &cl.mass, &cl.extra_mass, &cl.elements);
    let wheels = |h: f64| step_error_with(h, &[], &cl.mass, &cl.extra_mass, &cl.elements);
    let l_all = limit_of(|h| all(h).is_some());
    let l_wheels = limit_of(|h| wheels(h).is_some());
    assert!(
        l_wheels > 2.0 * l_all,
        "car_basic's beams must bind the explicit limit: with beams {l_all} s, wheels only {l_wheels} s"
    );

    // Between the two limits: implicit accepted, explicit refused.
    let (cfg, h) = cfg_at((l_all * l_wheels).sqrt(), 9.81);
    assert!(all(h).is_some() && wheels(h).is_none());
    let ie = Box::new(implicit(M5_TOLERANCES));
    assert!(
        Sim::from_car_with_solver(cfg.clone(), &car, ie).is_ok(),
        "implicit at {h} s"
    );
    let expected = format!("{}{CAR_SUFFIX}", all(h).unwrap());
    assert_eq!(err_of(Sim::from_car(cfg, &car)), expected);

    // Above the wheel limit: implicit refused with today's wheel message (no beams in it).
    let (cfg, h) = cfg_at(1.5 * l_wheels, 9.81);
    let expected = format!("{}{CAR_SUFFIX}", wheels(h).expect("refused"));
    let ie = Box::new(implicit(M5_TOLERANCES));
    assert_eq!(err_of(Sim::from_car_with_solver(cfg, &car, ie)), expected);
}

// ---------------------------------------------------------------------------------------
// Part 3a: option (1)'s frozen lines in docs/M5-SOLVER.md (section 8)

/// Lazlo's pre-approved test A ranges, 2026-11-22 04:44Z: front crush area (m^2), front yield
/// strain, crush damping ratio. A frozen range may be narrower, never wider.
const APPROVED_AREA_M2: (f64, f64) = (7e-5, 0.004);
const APPROVED_YIELD_STRAIN: (f64, f64) = (0.0010, 0.0114);
const APPROVED_DAMPING_RATIO: (f64, f64) = (0.0, 0.15);
/// The grid cap of docs/STAGE6-M4-M5.md (n20-06 bullet): at most 140 points.
const GRID_MAX_POINTS: usize = 140;
/// The approved `m5_method` value (brief t-20261123-210157, R1), whole.
const M5_METHOD: &str = "implicit backward Euler on the node-beam model (the same beams, yield rule and contact as node-beam); per substep Newton on the backward-Euler equations with a sparse Jacobi-PCG solve of M + h C + h^2 K, a compressed beam's negative geometric term dropped; a Newton or CG cap ends the run with a named error";
/// The approved N5: the same h as m4, so the integrator is the only difference.
const M5_SUBSTEPS: usize = 10;

/// The one line of `text` that starts `frozen: <key> = `, whole, with exactly one such line.
fn frozen_line(text: &str, doc: &str, key: &str) -> String {
    let want = format!("frozen: {key} = ");
    let lines: Vec<&str> = text.lines().filter(|l| l.starts_with(&want)).collect();
    assert_eq!(lines.len(), 1, "{doc}: exactly one `{want}` line");
    lines[0].to_string()
}

/// The two numbers of a `range` line value, checked against `approved` and `lo < hi`.
fn checked_range(value: &str, key: &str, approved: (f64, f64)) -> (f64, f64) {
    let nums: Vec<f64> = value
        .split_whitespace()
        .map(|w| {
            w.parse::<f64>()
                .unwrap_or_else(|_| panic!("{key}: `{w}` is not a number"))
        })
        .collect();
    assert_eq!(
        nums.len(),
        2,
        "{key}: a range is two numbers, got `{value}`"
    );
    let (lo, hi) = (nums[0], nums[1]);
    assert!(
        lo.is_finite() && hi.is_finite() && lo < hi,
        "{key}: `{value}`"
    );
    assert!(
        lo >= approved.0 && hi <= approved.1,
        "{key}: range [{lo}, {hi}] is outside the approved [{}, {}]",
        approved.0,
        approved.1
    );
    (lo, hi)
}

/// The point count of a `grid` line value: `<n> log|lin`, or a list of explicit values that
/// all lie inside `range`.
fn grid_points(value: &str, key: &str, range: (f64, f64)) -> usize {
    let words: Vec<&str> = value.split_whitespace().collect();
    if words.len() == 2 && (words[1] == "log" || words[1] == "lin") {
        let n = words[0]
            .parse::<usize>()
            .unwrap_or_else(|_| panic!("{key}: `{value}`"));
        assert!(
            n >= 2,
            "{key}: a spaced grid needs at least 2 points, got {n}"
        );
        return n;
    }
    assert!(!words.is_empty(), "{key}: empty grid");
    for w in &words {
        let v = w
            .parse::<f64>()
            .unwrap_or_else(|_| panic!("{key}: `{w}` is not a number"));
        assert!(
            v >= range.0 && v <= range.1,
            "{key}: grid value {v} is outside {range:?}"
        );
    }
    words.len()
}

/// Part 3a: option (1)'s block of docs/M5-SOLVER.md is pinned to the code (`M5_TOLERANCES`),
/// to the same-key lines of docs/STAGE6-M4-M5.md, and to Lazlo's pre-approved test A ranges.
#[test]
fn m5_option1_frozen_lines() {
    let m5 = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/docs/M5-SOLVER.md"))
        .expect("docs/M5-SOLVER.md");
    let st6 = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/docs/STAGE6-M4-M5.md"))
        .expect("docs/STAGE6-M4-M5.md");
    let value = |key: &str| -> String {
        let line = frozen_line(&m5, "docs/M5-SOLVER.md", key);
        line[format!("frozen: {key} = ").len()..].to_string()
    };

    // m5_method is the approved text of the brief, whole.
    assert_eq!(value("m5_method"), M5_METHOD);

    // m5_tolerances equals M5_TOLERANCES field by field, exact ==.
    let tol = value("m5_tolerances");
    let words: Vec<&str> = tol.split_whitespace().collect();
    assert_eq!(
        words.len(),
        8,
        "m5_tolerances: four `name value` pairs, got `{tol}`"
    );
    let names = ["newton_rtol", "newton_max_iter", "cg_rtol", "cg_max_iter"];
    for (i, name) in names.iter().enumerate() {
        assert_eq!(words[2 * i], *name, "m5_tolerances: pair {i} of `{tol}`");
    }
    let f = |w: &str| {
        w.parse::<f64>()
            .unwrap_or_else(|_| panic!("m5_tolerances: `{w}`"))
    };
    let u = |w: &str| {
        w.parse::<usize>()
            .unwrap_or_else(|_| panic!("m5_tolerances: `{w}`"))
    };
    assert_eq!(f(words[1]), M5_TOLERANCES.newton_rtol);
    assert_eq!(u(words[3]), M5_TOLERANCES.newton_max_iter);
    assert_eq!(f(words[5]), M5_TOLERANCES.cg_rtol);
    assert_eq!(u(words[7]), M5_TOLERANCES.cg_max_iter);

    // Each range and grid line equals the same-key line of STAGE6, character for character,
    // and each range lies inside the pre-approved test A range.
    let quantities = [
        ("crush.area_m2", APPROVED_AREA_M2),
        ("crush.yield_strain", APPROVED_YIELD_STRAIN),
        ("crush.damping_ratio", APPROVED_DAMPING_RATIO),
    ];
    let mut points = 1usize;
    for (q, approved) in quantities {
        let range_key = format!("range {q}");
        let grid_key = format!("grid {q}");
        for key in [&range_key, &grid_key] {
            assert_eq!(
                frozen_line(&m5, "docs/M5-SOLVER.md", key),
                frozen_line(&st6, "docs/STAGE6-M4-M5.md", key),
                "{key}: M5-SOLVER differs from STAGE6"
            );
        }
        let range = checked_range(&value(&range_key), &range_key, approved);
        points *= grid_points(&value(&grid_key), &grid_key, range);
    }
    assert!(
        points <= GRID_MAX_POINTS,
        "grid has {points} points, the cap is {GRID_MAX_POINTS}"
    );

    // m5_substeps is an integer of at least 2 (the m5-step paragraph, N5 >= 2).
    let n5 = value("m5_substeps");
    let n5 = n5
        .parse::<usize>()
        .unwrap_or_else(|_| panic!("m5_substeps: `{n5}` not an integer"));
    assert!(n5 >= 2, "m5_substeps = {n5}, N5 must be at least 2");
    assert_eq!(
        n5, M5_SUBSTEPS,
        "m5_substeps must be exactly the approved N5"
    );
}

// ---------------------------------------------------------------------------------------
// n20-11c: the `_m5` wall files (docs/STAGE6-M4-M5.md 5.1, option (1))

const CRUSH_LINES: [&str; 2] = ["material crush_front ", "material crush_rear "];

fn repo_text(rel: &str) -> String {
    let path = format!("{}/{rel}", env!("CARGO_MANIFEST_DIR"));
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"))
}

/// P1 (area, m^2), P2 (yield strain) and P3 (damping ratio) of one crush stage of a body text,
/// computed back as docs/STAGE6-M4-M5.md ("What is calibrated") defines them: P1 = k L_c / E,
/// L_c the mean rest length of the stage's beams, P3 = c / (2 sqrt(k m_c)), m_c the mean mass of
/// the distinct nodes at their ends.
fn crush_p123(text: &str, stage: &str, steel_e: f64) -> Result<[f64; 3], String> {
    let body = crash_sim::format::parse(text).map_err(|e| format!("does not parse: {e}"))?;
    let mat = body
        .materials
        .iter()
        .find(|m| m.name == stage)
        .ok_or(format!("no material `{stage}`"))?;
    let yield_strain = mat.yield_strain.ok_or(format!("`{stage}` has no yield_strain"))?;
    let beams: Vec<_> = body.beams.iter().filter(|b| b.material == stage).collect();
    if beams.is_empty() {
        return Err(format!("no beam uses `{stage}`"));
    }
    let mut ends: Vec<u32> = beams.iter().flat_map(|b| [b.a, b.b]).collect();
    ends.sort_unstable();
    ends.dedup();
    let mut mass = 0.0;
    for id in &ends {
        mass += body
            .nodes
            .iter()
            .find(|n| n.id == *id)
            .ok_or("a crush beam names a missing node")?
            .mass;
    }
    let mc = mass / ends.len() as f64;
    let lc = beams.iter().map(|b| b.rest).sum::<f64>() / beams.len() as f64;
    Ok([
        mat.stiffness * lc / steel_e,
        yield_strain,
        mat.damping / (2.0 * (mat.stiffness * mc).sqrt()),
    ])
}

/// The `_m5` text equals its node-beam text line for line (bytes, line ends included) except the
/// `material crush_front` and `material crush_rear` lines, and has exactly one of each.
fn check_differs_only_in_crush_lines(nb: &str, m5: &str) -> Result<(), String> {
    let (a, b): (Vec<&str>, Vec<&str>) = (nb.split('\n').collect(), m5.split('\n').collect());
    if a.len() != b.len() {
        return Err(format!("{} lines against {}", a.len(), b.len()));
    }
    for prefix in CRUSH_LINES {
        for (name, lines) in [("node-beam", &a), ("m5", &b)] {
            let n = lines.iter().filter(|l| l.starts_with(prefix)).count();
            if n != 1 {
                return Err(format!("the {name} file has {n} `{}` lines, not 1", prefix.trim()));
            }
        }
    }
    for (i, (x, y)) in a.iter().zip(&b).enumerate() {
        if x != y && !CRUSH_LINES.iter().any(|p| x.starts_with(p) && y.starts_with(p)) {
            return Err(format!("line {}: `{x}` against `{y}`", i + 1));
        }
    }
    Ok(())
}

/// P1 to P3 of both stages of the two `_m5` files agree within 1e-9 relative and lie inside the
/// frozen ranges of docs/M5-SOLVER.md.
fn check_m5_physics(a_m5: &str, b_m5: &str, steel_e: f64, ranges: &[(f64, f64); 3]) -> Result<(), String> {
    for stage in ["crush_front", "crush_rear"] {
        let a = crush_p123(a_m5, stage, steel_e)?;
        let b = crush_p123(b_m5, stage, steel_e)?;
        for (q, (x, y)) in ["area_m2", "yield_strain", "damping_ratio"].iter().zip(a.iter().zip(&b)) {
            // Exactly equal values (0 against 0 included) agree.
            let rel = if x == y { 0.0 } else { (x - y).abs() / x.abs().max(y.abs()) };
            if rel.is_nan() || rel > 1e-9 {
                return Err(format!("{stage} {q}: {x} against {y}, relative difference {rel}"));
            }
        }
        for (q, (v, (lo, hi))) in ["area_m2", "yield_strain", "damping_ratio"]
            .iter()
            .zip(a.iter().zip(ranges))
        {
            if v.is_nan() || v < lo || v > hi {
                return Err(format!("{stage} {q} = {v} is outside [{lo}, {hi}]"));
            }
        }
    }
    Ok(())
}

#[test]
fn m5_files_differ_only_in_crush_lines() {
    let st6 = repo_text("docs/STAGE6-M4-M5.md");
    let m5doc = repo_text("docs/M5-SOLVER.md");
    let e = frozen_line(&st6, "docs/STAGE6-M4-M5.md", "steel_E_pa");
    let steel_e: f64 = e["frozen: steel_E_pa = ".len()..].trim().parse().expect("steel_E_pa");
    let mut ranges = [(0.0, 0.0); 3];
    for (slot, q) in ranges.iter_mut().zip(["crush.area_m2", "crush.yield_strain", "crush.damping_ratio"]) {
        let l = frozen_line(&m5doc, "docs/M5-SOLVER.md", &format!("range {q}"));
        let v: Vec<f64> = l[format!("frozen: range {q} = ").len()..]
            .split_whitespace()
            .map(|w| w.parse().expect("a number"))
            .collect();
        assert_eq!(v.len(), 2, "{q}");
        *slot = (v[0], v[1]);
    }
    let pairs = [
        ("scenarios/validation_wall.bsn", "scenarios/validation_wall_m5.bsn"),
        ("scenarios/validation_wall_b.bsn", "scenarios/validation_wall_b_m5.bsn"),
    ];
    for (nb, m5) in pairs {
        let (nb_text, m5_text) = (repo_text(nb), repo_text(m5));
        check_differs_only_in_crush_lines(&nb_text, &m5_text).unwrap_or_else(|e| panic!("{m5}: {e}"));
        // No solid or tet in an option (1) file.
        assert!(
            !m5_text.lines().any(|l| l.starts_with("solid") || l.starts_with("tet") || l.starts_with("bsn 3")),
            "{m5}: a solid, tet or bsn 3 line"
        );
    }
    let (a_m5, b_m5) = (repo_text(pairs[0].1), repo_text(pairs[1].1));
    check_m5_physics(&a_m5, &b_m5, steel_e, &ranges).unwrap_or_else(|e| panic!("{e}"));
}
