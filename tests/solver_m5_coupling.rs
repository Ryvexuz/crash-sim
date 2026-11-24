//! m5 solver: stability, plug-in, same-input-same-bytes, coupling energy and the mesh rule (moved from tests/solver_m5.rs; see its header for the tolerance rules).
use crash_sim::format::{self, Material};
use crash_sim::solver::{Solver, State};
use crash_sim::solver_m5::{self, SolverM5, M5_MAX_TETS};
use crash_sim::vec3::Vec3;
use crash_sim::{to_csv_with_broken, Config, Sim};
use std::cell::Cell;
use std::rc::Rc;

mod m5_support;
use m5_support::*;

// R5: stability

/// R5: the closed-form tangent bound is at least the largest eigenvalue (in size) of a
/// finite-difference tangent of this file's own stress, for random F (stretch 0.3 to 3, shear,
/// rotation), and it is tight at rest (2 mu + 3 lambda).
#[test]
fn tangent_bound_covers_the_finite_difference_tangent() {
    let (plateau, ys) = (2e6, 0.02);
    let (mu, lam) = lame(plateau, ys);
    let law = law_of(plateau, ys);
    let fd_max = |f: &M| -> f64 {
        let eps = 1e-6;
        let mut a = vec![vec![0.0; 9]; 9];
        for k in 0..9 {
            let (mut fp, mut fm) = (*f, *f);
            fp[k / 3][k % 3] += eps;
            fm[k / 3][k % 3] -= eps;
            let (pp, pm) = (nh_p(&fp, mu, lam), nh_p(&fm, mu, lam));
            for i in 0..9 {
                a[i][k] = (pp[i / 3][i % 3] - pm[i / 3][i % 3]) / (2.0 * eps);
            }
        }
        for i in 0..9 {
            for k in 0..i {
                let m = 0.5 * (a[i][k] + a[k][i]);
                a[i][k] = m;
                a[k][i] = m;
            }
        }
        eig_sym(a).into_iter().map(f64::abs).fold(0.0, f64::max)
    };
    let at_rest = fd_max(&eye());
    let bound_rest = solver_m5::tangent_bound(&eye(), &law);
    assert!((at_rest - (2.0 * mu + 3.0 * lam)).abs() <= FD * at_rest);
    assert!(
        bound_rest >= at_rest * (1.0 - FD) && bound_rest <= at_rest * (1.0 + FD),
        "bound at rest {bound_rest} vs {at_rest}"
    );
    let mut rng = Rng(0x5eed_0004);
    let mut worst = f64::INFINITY;
    for _ in 0..200 {
        let s = [
            rng.range(0.3f64.ln(), 3f64.ln()).exp(),
            rng.range(0.3f64.ln(), 3f64.ln()).exp(),
            rng.range(0.3f64.ln(), 3f64.ln()).exp(),
        ];
        let mut shear = eye();
        shear[0][2] = rng.range(-1.0, 1.0);
        let d: M = [[s[0], 0.0, 0.0], [0.0, s[1], 0.0], [0.0, 0.0, s[2]]];
        let f = mmul(&mmul(&rng.rotation(), &mmul(&shear, &d)), &rng.rotation());
        let exact = fd_max(&f);
        let bound = solver_m5::tangent_bound(&f, &law);
        assert!(
            bound >= exact * (1.0 - FD),
            "bound {bound} below the tangent's {exact} at F = {f:?}"
        );
        worst = worst.min(bound / exact);
    }
    assert!(worst.is_finite() && worst >= 1.0 - FD);
}

/// R5a, closed form for one tet on the unit right corner (rest gradients -(1,1,1), e1, e2, e3):
/// node 0 has the largest Gershgorin sum, V A |g_0| S with V = 1/6, A = 2 (2 mu + 3 lambda),
/// |g_0| = sqrt 3, S = 3 + sqrt 3, so w2 = that / m. Just below the limit the solver is built
/// and runs 10 000 substeps bounded; at and just above it, it is refused.
#[test]
fn step_limit_closed_form_below_runs_above_is_refused() {
    let (plateau, ys, m) = (1e6, 0.02, 2.0);
    let (mu, lam) = lame(plateau, ys);
    let at = Vec3::new(0.0, 10.0, 0.0);
    let b = one_tet(plateau, ys, 0.0, 1.0, m, at);
    let probe = SolverM5::from_body(&b, 1e-9).unwrap();
    let s3 = 3f64.sqrt();
    let w2 = (1.0 / 6.0) * 2.0 * (2.0 * mu + 3.0 * lam) * s3 * (3.0 + s3) / m;
    assert!((probe.omega_max() * probe.omega_max() - w2).abs() <= ROUND * w2);
    let limit = 2.0 / w2.sqrt();
    assert!((probe.step_limit() - limit).abs() <= ROUND * limit);

    let e = refused(&b, limit * 1.001);
    assert!(
        e.starts_with("physics step h = dt/substeps = ")
            && e.contains("reaches the m5 stability limit")
            && e.ends_with("raise --substeps or lower --dt"),
        "{e}"
    );
    assert!(
        SolverM5::from_body(&b, limit).is_err(),
        "h = limit itself is refused (h omega < 2 is strict)"
    );

    let h = limit * 0.999;
    let mut s = SolverM5::from_body(&b, h).unwrap();
    let mut pos = positions(&b);
    let mut vel = vec![Vec3::ZERO; 4];
    // A small kick: 1e-3 strain rate times the step.
    vel[1] = Vec3::new(1e-3 / (100.0 * h), 0.0, 0.0);
    let mass = vec![m; 4];
    let e0 = 0.5 * m * vel[1].dot(vel[1]);
    for k in 0..10_000 {
        s.step(
            State {
                pos: &mut pos,
                vel: &mut vel,
                beams: &[],
                mass: &mass,
                gravity: 0.0,
            },
            h,
        );
        if k % 100 == 0 {
            assert_eq!(s.check(), None, "substep {k}");
        }
    }
    assert_eq!(s.check(), None);
    // Bounded by the shadow-energy bound of docs/M5-SOLVER.md section 5 with the Gershgorin
    // frequency of the tangent at rest without the headroom (still an upper bound on the true
    // one): h omega = 0.999 * 2 / sqrt(2), q = h omega / 2 = 0.706, (1 + q)/(1 - q) = 5.8, with
    // R9's allowance for the nonlinear terms.
    let q = h * (w2 / solver_m5::STIFFNESS_HEADROOM).sqrt() / 2.0;
    let cap = e0 * (1.0 + q) / (1.0 - q) * (1.0 + R9_NONLINEAR);
    let ke: f64 = vel.iter().map(|v| 0.5 * m * v.dot(*v)).sum();
    let eel = s.elastic_energy(&pos);
    assert!(
        ke + eel <= cap,
        "energy {} after 10 000 substeps above {cap} (start {e0})",
        ke + eel
    );
}

/// R5b: a tet sheared during the run past the per-substep bound ends with exactly
/// `stiffness bound exceeded at substep <n>: h omega = <value>`, value >= 2, at the substep the
/// closed form predicts. The shear is imposed (the nodes are put on `F = I + g e2 e1^T`, g =
/// 0.01 k, before substep k + 1), at h = 0.9 times the construction limit. With no plastic flow
/// (plateau / E = 0.69, far above the shear stresses here), the bound at node 0 is
/// V A(g) |g_0| S / m with A(g) = mu + (3 lambda + mu) / s_min(g)^2 (Je = 1),
/// s_min(g) = sqrt(1 + g^2/4) - g/2, and it trips at the first g with h^2 w2 >= 4.
#[test]
fn a_tet_stretched_past_the_bound_ends_with_the_named_error() {
    let (plateau, ys) = (690.0, 0.69);
    let (mu, lam) = lame(plateau, ys);
    let at = Vec3::new(0.0, 10.0, 0.0);
    let b = one_tet(plateau, ys, 0.0, 1.0, 1.0, at);
    let probe = SolverM5::from_body(&b, 1e-9).unwrap();
    let h = 0.9 * probe.step_limit();
    let mut s = SolverM5::from_body(&b, h).unwrap();
    let rest = positions(&b);
    let mass = vec![1.0; 4];
    let s3 = 3f64.sqrt();
    let trips = |g: f64| {
        let smin = (1.0 + g * g / 4.0).sqrt() - g / 2.0;
        let a = mu + (3.0 * lam + mu) / (smin * smin);
        h * h * (a * s3 * (3.0 + s3) / 6.0) >= 4.0
    };
    let k_pred = (0..1000u64)
        .find(|&k| trips(0.01 * k as f64))
        .expect("the closed form never trips");
    let mut msg = None;
    for k in 0..1000u64 {
        let g = 0.01 * k as f64;
        let mut pos: Vec<Vec3> = rest
            .iter()
            .map(|x| {
                let r = *x - at;
                at + Vec3::new(r.x, r.y + g * r.x, r.z)
            })
            .collect();
        let mut vel = vec![Vec3::ZERO; 4];
        s.step(
            State {
                pos: &mut pos,
                vel: &mut vel,
                beams: &[],
                mass: &mass,
                gravity: 0.0,
            },
            h,
        );
        if let Some(e) = s.check() {
            msg = Some((k, e));
            break;
        }
    }
    let (k, e) = msg.expect("the bound never tripped");
    let prefix = "stiffness bound exceeded at substep ";
    assert!(e.starts_with(prefix), "{e}");
    let (n, x) = e[prefix.len()..].split_once(": h omega = ").unwrap();
    let n: u64 = n.parse().unwrap();
    let x: f64 = x.parse().unwrap();
    assert_eq!(n, k + 1, "the error names the substep it happened in");
    assert_eq!(
        k,
        k_pred,
        "tripped at shear {} instead of the closed form's {}",
        0.01 * k as f64,
        0.01 * k_pred as f64
    );
    assert!(x >= 2.0, "h omega {x}");
    assert!(
        k > 10,
        "it tripped at shear {}, before the shear had grown",
        0.01 * k as f64
    );
    assert_eq!(s.plastic_strain(0), 0.0);
}

// ---------------------------------------------------------------------------------------
// R6, R7

struct Counting {
    inner: SolverM5,
    calls: Rc<Cell<u64>>,
}

impl Solver for Counting {
    fn step(&mut self, s: State<'_>, h: f64) {
        self.calls.set(self.calls.get() + 1);
        self.inner.step(s, h);
    }
    fn handles_solids(&self) -> bool {
        self.inner.handles_solids()
    }
    fn check(&self) -> Option<String> {
        self.inner.check()
    }
}

/// R6: through `Sim::from_body_with_solver`, the sim calls the m5 solver once per substep and
/// its tets act: a block stretched along x at the start contracts.
#[test]
fn m5_solver_plugs_in() {
    let b = block_2x2x2(6.9e8, 0.69, 0.05);
    let cfg = Config {
        dt: 1e-4,
        substeps: 100,
        gravity: 0.0,
        duration: 1.0,
        ..Config::default()
    };
    let h = cfg.dt / cfg.substeps as f64;
    let calls = Rc::new(Cell::new(0));
    let solver = Counting {
        inner: SolverM5::from_body(&b, h).unwrap(),
        calls: calls.clone(),
    };
    let mut sim = Sim::from_body_with_solver(cfg, &b, Box::new(solver)).unwrap();
    for p in sim.pos.iter_mut() {
        p.x *= 1.001;
    }
    let extent = |s: &Sim| {
        s.pos.iter().map(|p| p.x).fold(f64::MIN, f64::max)
            - s.pos.iter().map(|p| p.x).fold(f64::MAX, f64::min)
    };
    let start = extent(&sim);
    for _ in 0..2 {
        sim.advance();
        sim.check().unwrap();
    }
    assert_eq!(calls.get(), 200);
    assert!(
        extent(&sim) < start,
        "the stretched block did not contract: {} vs {start}",
        extent(&sim)
    );
}

/// The CSV of a run, and the final positions and velocities as raw bits.
fn run_csv(mut sim: Sim, steps: u64) -> (String, Vec<u64>) {
    let mut out = vec![sim.sample()];
    for _ in 0..steps {
        sim.advance();
        sim.check().unwrap();
        out.push(sim.sample());
    }
    let bits = sim
        .pos
        .iter()
        .chain(&sim.vel)
        .flat_map(|v| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()])
        .collect();
    (to_csv_with_broken(&out), bits)
}

/// R7: two runs of the same file give the same bytes; and a body with no tets under the m5
/// solver gives exactly the default solver's CSV on the cube scenario (the same integrator on
/// the same beams, in the same order: exact, not within a tolerance).
#[test]
fn m5_same_input_same_bytes() {
    let mut b = block_2x2x2(1e6, 0.02, 0.05);
    b.materials.push(Material {
        name: "m".into(),
        stiffness: 1e5,
        damping: 1.0,
        yield_strain: None,
        break_strain: None,
    });
    add_grid(&mut b, 3, 2, 2, 0.2, Vec3::new(0.0, 100.0, 0.0), 1.0);
    b.nodes.sort_by_key(|n| n.id);
    b.nodes.dedup_by_key(|n| n.id);
    add_beam_cells(&mut b, 2..3, 2, 2, "m");
    let text = format::write(&b);
    let cfg = Config {
        dt: 1e-4,
        substeps: 10,
        gravity: 9.81,
        duration: 1.0,
        ..Config::default()
    };
    let h = cfg.dt / cfg.substeps as f64;
    let run = || {
        let body = format::parse(&text).unwrap();
        let s = SolverM5::from_body(&body, h).unwrap();
        let mut sim = Sim::from_body_with_solver(cfg.clone(), &body, Box::new(s)).unwrap();
        for v in sim.vel.iter_mut() {
            *v = Vec3::new(0.3, -2.0, 0.1);
        }
        sim.vel[0] = Vec3::new(-3.0, 1.0, 2.0);
        run_csv(sim, 50)
    };
    let (a, c) = (run(), run());
    assert!(a.0.lines().count() == 52);
    assert!(a.0 == c.0, "two runs of the same file differ");
    assert!(
        a.1 == c.1,
        "two runs of the same file end in different bits"
    );

    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/scenarios/cube.bsn");
    let cube = format::parse(&std::fs::read_to_string(path).unwrap()).unwrap();
    let cfg = Config::default();
    let h = cfg.dt / cfg.substeps as f64;
    let steps = 500;
    let default = run_csv(Sim::from_body(cfg.clone(), &cube).unwrap(), steps);
    let m5 = run_csv(
        Sim::from_body_with_solver(
            cfg.clone(),
            &cube,
            Box::new(SolverM5::from_body(&cube, h).unwrap()),
        )
        .unwrap(),
        steps,
    );
    assert_eq!(default.0.lines().count(), 502);
    assert!(
        default.0 == m5.0,
        "the m5 solver with no tets differs from the default on the cube"
    );
    assert!(default.1 == m5.1, "the final state differs in its bits");
}

// ---------------------------------------------------------------------------------------
// R9: the coupling creates no energy

/// docs/M5-SOLVER.md section 5: a block of tets (2 x 2 x 2 cells, 48 tets) and a block of beams
/// (2 x 2 x 2 cells, all node pairs of each cell, each pair once) joined at the 9 nodes of one
/// face; no yield (plateau 1e12 Pa), no damping, no gravity, 100 m up. Start: a uniform shear and
/// stretch of 1e-3 plus a rigid velocity and spin. h = (2 / omega_max) / 10 with the solver's own
/// omega_max, 20 000 substeps. At every substep E <= E0 (1 + q)/(1 - q)(1 + 1e-2), q = 0.1, with
/// E = kinetic + beam + tet energy in this file's own formulas; linear and angular momentum to
/// 1e-9 of their scale.
#[test]
fn m5_coupling_creates_no_energy() {
    let (plateau, ys) = (1e12, 0.5);
    let (mu, lam) = lame(plateau, ys);
    let c = 0.2;
    let mut b = empty_body();
    b.solids.push(solid("s", plateau, ys, 0.0));
    b.materials.push(Material {
        name: "m".into(),
        stiffness: 1e11,
        damping: 0.0,
        yield_strain: None,
        break_strain: None,
    });
    add_grid(&mut b, 4, 2, 2, c, Vec3::new(0.0, 100.0, 0.0), 1.0);
    add_tet_cells(&mut b, 0..2, 2, 2, "s");
    add_beam_cells(&mut b, 2..4, 2, 2, "m");
    assert_eq!(b.tets.len(), 48);
    let rest = positions(&b);
    let idx = |id: u32| b.nodes.iter().position(|n| n.id == id).unwrap();
    let shared: Vec<usize> = (0..b.nodes.len())
        .filter(|&i| {
            let id = b.nodes[i].id;
            b.tets.iter().any(|t| t.nodes.contains(&id))
                && b.beams.iter().any(|bm| bm.a == id || bm.b == id)
        })
        .collect();
    assert_eq!(shared.len(), 9);

    let probe = SolverM5::from_body(&b, 1e-15).unwrap();
    let omega = probe.omega_max();
    let h = (2.0 / omega) / 10.0;
    let q = h * omega / 2.0;
    let mut s = SolverM5::from_body(&b, h).unwrap();
    let mass: Vec<f64> = b.nodes.iter().map(|n| n.mass).collect();
    let n = rest.len();
    let xc: Vec3 = rest.iter().fold(Vec3::ZERO, |a, x| a + *x) * (1.0 / n as f64);
    let g: M = [[1.0, 0.5, 0.0], [0.3, -0.5, 0.2], [0.0, 0.4, 0.8]];
    let mut pos: Vec<Vec3> = rest.iter().map(|x| *x + mvec(&g, *x - xc) * 1e-3).collect();
    let spin = Vec3::new(30.0, -60.0, 45.0);
    let v0 = Vec3::new(10.0, -5.0, 8.0);
    let mut vel: Vec<Vec3> = pos.iter().map(|x| v0 + spin.cross(*x - xc)).collect();
    let beams = crash_sim::format::Body::build(&b).beams;

    // This file's own energy: tets on their rest shape (no plastic flow at these strains).
    let tet_rest: Vec<([usize; 4], M, f64)> = b
        .tets
        .iter()
        .map(|t| {
            let ns = t.nodes.map(idx);
            let dm = [
                rest[ns[1]] - rest[ns[0]],
                rest[ns[2]] - rest[ns[0]],
                rest[ns[3]] - rest[ns[0]],
            ];
            let dmm: M = [
                [dm[0].x, dm[1].x, dm[2].x],
                [dm[0].y, dm[1].y, dm[2].y],
                [dm[0].z, dm[1].z, dm[2].z],
            ];
            (ns, minv(&dmm), mdet(&dmm) / 6.0)
        })
        .collect();
    let tet_energy = |pos: &[Vec3]| -> f64 {
        tet_rest
            .iter()
            .map(|(ns, dmi, v)| {
                let d = [
                    pos[ns[1]] - pos[ns[0]],
                    pos[ns[2]] - pos[ns[0]],
                    pos[ns[3]] - pos[ns[0]],
                ];
                let ds: M = [
                    [d[0].x, d[1].x, d[2].x],
                    [d[0].y, d[1].y, d[2].y],
                    [d[0].z, d[1].z, d[2].z],
                ];
                v * nh_w(&mmul(&ds, dmi), mu, lam)
            })
            .sum()
    };
    let energy = |pos: &[Vec3], vel: &[Vec3]| -> (f64, f64) {
        let ke: f64 = vel
            .iter()
            .zip(&mass)
            .map(|(v, m)| 0.5 * m * v.dot(*v))
            .sum();
        let be: f64 = beams
            .iter()
            .map(|bm| {
                let st = (pos[bm.a] - pos[bm.b]).length() - bm.rest;
                0.5 * bm.stiffness * st * st
            })
            .sum();
        let te = tet_energy(pos);
        (ke + be + te, te)
    };
    let momentum = |pos: &[Vec3], vel: &[Vec3]| -> (Vec3, Vec3, f64, f64) {
        let mut p = Vec3::ZERO;
        let mut l = Vec3::ZERO;
        let (mut ps, mut ls) = (0.0, 0.0);
        for i in 0..pos.len() {
            p += vel[i] * mass[i];
            l += (pos[i] - xc).cross(vel[i]) * mass[i];
            ps += mass[i] * vel[i].length();
            ls += mass[i] * (pos[i] - xc).length() * vel[i].length();
        }
        (p, l, ps, ls)
    };
    let (e0, te0) = energy(&pos, &vel);
    let (p0, l0, ps, ls) = momentum(&pos, &vel);
    // Energy sits in the motion, the beams and the tets at the start.
    let ke0: f64 = vel
        .iter()
        .zip(&mass)
        .map(|(v, m)| 0.5 * m * v.dot(*v))
        .sum();
    let be0 = e0 - ke0 - te0;
    assert!(
        ke0 > R9_SHARE * e0 && be0 > R9_SHARE * e0 && te0 > R9_SHARE * e0,
        "start energy: kinetic {ke0}, beams {be0}, tets {te0}"
    );
    let cap = e0 * (1.0 + q) / (1.0 - q) * (1.0 + R9_NONLINEAR);
    let mut te_swing = 0.0f64;
    let mut e_max_seen = e0;
    for k in 0..20_000 {
        s.step(
            State {
                pos: &mut pos,
                vel: &mut vel,
                beams: &beams,
                mass: &mass,
                gravity: 0.0,
            },
            h,
        );
        let (e, te) = energy(&pos, &vel);
        assert!(
            e <= cap,
            "substep {}: energy {e} above the bound {cap} (E0 {e0})",
            k + 1
        );
        e_max_seen = e_max_seen.max(e);
        te_swing = te_swing.max((te - te0).abs());
        let (p, l, _, _) = momentum(&pos, &vel);
        assert!(
            (p - p0).length() <= R9_MOMENTUM * ps,
            "substep {}: linear momentum moved by {}",
            k + 1,
            (p - p0).length()
        );
        assert!(
            (l - l0).length() <= R9_MOMENTUM * ls,
            "substep {}: angular momentum moved by {}",
            k + 1,
            (l - l0).length()
        );
    }
    assert_eq!(s.check(), None);
    assert_eq!(s.plastic_strain(0), 0.0);
    // Not vacuous: the tets' energy changed by much more than the allowance (a wave crossed).
    assert!(
        te_swing > R9_WAVE * te0,
        "the tet energy barely moved: {te_swing} of {te0}"
    );
    println!(
        "m5_coupling: h = {h:e} s, q = {q:.6}, E0 = {e0:e} J (tets {te0:e} J), max E / E0 = {:.9}, bound / E0 = {:.6}, largest tet energy swing / tet E0 = {:.4}",
        e_max_seen / e0,
        cap / e0,
        te_swing / te0
    );
}

// ---------------------------------------------------------------------------------------
// R12: the mesh rule

/// R12: `cut_cell` gives 6 tets of positive volume that fill the cell, all along the main
/// diagonal 0-7, and two neighbouring cells triangulate their shared face the same way (the
/// mesh is conforming), along x, y and z.
#[test]
fn cut_cell_is_six_positive_tets_and_conforming() {
    let corner = |k: u32| Vec3::new((k & 1) as f64, ((k >> 1) & 1) as f64, ((k >> 2) & 1) as f64);
    let tets = solver_m5::cut_cell([0, 1, 2, 3, 4, 5, 6, 7]);
    let mut total = 0.0;
    for (i, t) in tets.iter().enumerate() {
        let p = t.map(corner);
        let v = (p[1] - p[0]).dot((p[2] - p[0]).cross(p[3] - p[0])) / 6.0;
        assert_eq!(v, 1.0 / 6.0, "tet {i} {t:?}");
        assert!(t.contains(&0) && t.contains(&7));
        total += v;
        for u in &tets[..i] {
            let (mut a, mut c) = (*t, *u);
            a.sort();
            c.sort();
            assert_ne!(a, c);
        }
    }
    assert!(
        (total - 1.0).abs() <= ROUND,
        "the tets fill {total} of the cell"
    );
    // Faces on the plane `axis = side` of a cell at offset `off`, as sorted corner positions.
    let faces_on = |off: [usize; 3], axis: usize, side: f64| -> Vec<[(i64, i64, i64); 3]> {
        let mut out = Vec::new();
        for t in solver_m5::cut_cell(corners(off[0], off[1], off[2])) {
            let pts: Vec<(i64, i64, i64)> = t
                .iter()
                .map(|&id| ((id / 100) as i64, ((id / 10) % 10) as i64, (id % 10) as i64))
                .collect();
            for skip in 0..4 {
                let mut f: Vec<(i64, i64, i64)> =
                    (0..4).filter(|&k| k != skip).map(|k| pts[k]).collect();
                let coord = |p: &(i64, i64, i64)| [p.0, p.1, p.2][axis] as f64;
                if f.iter().all(|p| coord(p) == side) {
                    f.sort();
                    out.push([f[0], f[1], f[2]]);
                }
            }
        }
        out.sort();
        out
    };
    for axis in 0..3 {
        let mut next = [0usize; 3];
        next[axis] = 1;
        let a = faces_on([0, 0, 0], axis, 1.0);
        let b = faces_on(next, axis, 1.0);
        assert_eq!(a.len(), 2, "a cell face is cut into two triangles");
        assert_eq!(
            a, b,
            "the cells disagree on their shared face along axis {axis}"
        );
    }
}

/// R12 with the cap: a 5 x 5 x 2 zone is exactly `m5_max_tets` tets and is taken; one more cell
/// row is refused by the cap.
#[test]
fn a_zone_of_m5_max_tets_is_taken_and_one_more_cell_is_not() {
    let mut b = empty_body();
    b.solids.push(solid("s", 1e6, 0.02, 0.0));
    add_grid(&mut b, 5, 5, 3, 0.2, Vec3::new(0.0, 100.0, 0.0), 1.0);
    add_tet_cells(&mut b, 0..5, 5, 2, "s");
    assert_eq!(b.tets.len(), M5_MAX_TETS);
    let s = SolverM5::from_body(&b, 1e-6).unwrap();
    assert_eq!(s.tet_count(), 300);
    for t in solver_m5::cut_cell(corners(0, 0, 2)) {
        add_tet(&mut b, t, "s");
    }
    assert!(refused(&b, 1e-6).starts_with("the body has 306 tets, more than the m5 solver takes"));
}
