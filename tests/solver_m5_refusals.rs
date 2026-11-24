//! m5 solver: the inverted-tet rule, NaN and force checks, and constructor refusals (moved from tests/solver_m5.rs; see its header for the tolerance rules).
use crash_sim::format::{self, Body, BodyBeam, Material, Tet};
use crash_sim::solver::{Solver, State};
use crash_sim::solver_m5::{self, Mat3, SolverM5, J_MIN_FAIL, M5_MAX_TETS};
use crash_sim::vec3::Vec3;
use crash_sim::{Config, Sim};

mod m5_support;
use m5_support::*;

// ---------------------------------------------------------------------------------------
// R4e, R10: the inverted-tet rule

/// Two tets sharing the face 0 1 2; node 4 (apex of tet 1) is driven through the face by its
/// start velocity. The run ends with exactly `tet 1 inverted or collapsed at the updated
/// positions of substep <n>: Je = <value>`, no later than the free-flight substep plus 2, and no
/// NaN enters the state.
#[test]
fn a_tet_driven_through_collapse_ends_the_run_with_the_named_error() {
    let mut b = empty_body();
    // A very soft solid (E = 20 Pa), so the tet's resistance hardly slows the node.
    b.solids.push(solid("s", 10.0, 0.5, 0.0));
    let at = Vec3::new(0.0, 100.0, 0.0);
    let p = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, -1.0),
        Vec3::new(0.0, 0.0, 1.0),
    ];
    for (i, q) in p.iter().enumerate() {
        add_node(&mut b, i as u32, *q + at, 1.0);
    }
    add_tet(&mut b, [0, 2, 1, 3], "s"); // below the face z = 0
    add_tet(&mut b, [0, 1, 2, 4], "s"); // above it
    let cfg = Config {
        dt: 0.001,
        substeps: 10,
        gravity: 0.0,
        duration: 1.0,
        ..Config::default()
    };
    let h = cfg.dt / cfg.substeps as f64;
    let m5 = SolverM5::from_body(&b, h).unwrap();
    let mut sim = Sim::from_body_with_solver(cfg.clone(), &b, Box::new(m5)).unwrap();
    let v = 100.0;
    sim.vel[4] = Vec3::new(0.0, 0.0, -v);
    // Free flight: node 4's height z is 1 - n v h. The tet yields and locks up on the way
    // (e_max = 0.7 - sqrt(0.7 * 0.5)), after which Fp = diag(1, 1, e^-e_max), so the trial
    // Je = z e^e_max is at or below 0.05 from substep ceil((1 - 0.05 e^-e_max) / (v h)). The
    // tet's own resistance can only delay it, by a little at E = 20 Pa.
    let e_max = 0.7 - (0.7f64 * 0.5).sqrt();
    let n_free = ((1.0 - J_MIN_FAIL * (-e_max).exp()) / (v * h)).ceil() as u64;
    let mut err = None;
    for step in 0..1000 {
        sim.advance();
        match sim.check() {
            Ok(()) => assert!(
                ((step + 1) * 10) < n_free + 10,
                "no error long after the free-flight substep"
            ),
            Err(e) => {
                err = Some(e);
                break;
            }
        }
    }
    let e = err.expect("the run never failed");
    // Since round 5 the rule is applied to the positions a substep is about to write, so the
    // collapse is found there, in the substep that would have made it.
    let prefix = "tet 1 inverted or collapsed at the updated positions of substep ";
    assert!(e.starts_with(prefix), "{e}");
    let rest = &e[prefix.len()..];
    let (n, je) = rest.split_once(": Je = ").expect("`: Je = ` in the error");
    let n: u64 = n.parse().unwrap();
    let je: f64 = je.parse().unwrap();
    assert!(
        n >= n_free - 1 && n <= n_free + 2,
        "substep {n}, free flight {n_free}"
    );
    assert!(je <= J_MIN_FAIL && je.is_finite(), "Je = {je}");
    assert!(
        sim.pos
            .iter()
            .chain(&sim.vel)
            .all(|v| v.x.is_finite() && v.y.is_finite() && v.z.is_finite()),
        "a NaN entered the state"
    );
}

/// R4e: a NaN node ends the run with a named error (through the solver and through
/// `Sim::check`, which reports the solver's error first), and no NaN enters the force array.
/// `step` stops on the node itself (round 2); the tet guard behind it gives the R4e error
/// `tet <i> inverted or collapsed ... Je = NaN` where it is reached first, in `tet_forces`.
#[test]
fn a_nan_node_ends_the_run_with_the_named_error() {
    let at = Vec3::new(0.0, 10.0, 0.0);
    let b = one_tet(1e6, 0.02, 0.05, 1.0, 1.0, at);
    let mut s = SolverM5::from_body(&b, 1e-5).unwrap();
    let mut bad = positions(&b);
    bad[2].y = f64::NAN;
    let f = s.tet_forces(&bad, &[Vec3::ZERO; 4]);
    assert_eq!(
        s.check(),
        Some("tet 0 inverted or collapsed at substep 0: Je = NaN".to_string())
    );
    assert!(
        f.iter()
            .all(|v| v.x.is_finite() && v.y.is_finite() && v.z.is_finite()),
        "the tet guard let a NaN into the forces"
    );

    let mut s = SolverM5::from_body(&b, 1e-5).unwrap();
    let mut pos = positions(&b);
    let mut vel = vec![Vec3::ZERO; 4];
    pos[2].y = f64::NAN;
    let mass = vec![1.0; 4];
    s.step(
        State {
            pos: &mut pos,
            vel: &mut vel,
            beams: &[],
            mass: &mass,
            gravity: 0.0,
        },
        1e-5,
    );
    assert_eq!(
        s.check(),
        Some("node 2 is not finite at substep 1".to_string())
    );
    assert!(
        vel.iter()
            .all(|v| v.x.is_finite() && v.y.is_finite() && v.z.is_finite()),
        "a NaN entered the force array"
    );

    let cfg = Config {
        dt: 1e-4,
        substeps: 10,
        gravity: 0.0,
        duration: 1.0,
        ..Config::default()
    };
    let m5 = SolverM5::from_body(&b, cfg.dt / cfg.substeps as f64).unwrap();
    let mut sim = Sim::from_body_with_solver(cfg, &b, Box::new(m5)).unwrap();
    sim.pos[2].x = f64::NAN;
    sim.advance();
    assert_eq!(
        sim.check(),
        Err("node 2 is not finite at substep 1".to_string())
    );
}

/// Round 2 of review q-20261121-031356: a NaN node shared by a beam and a tet. The step must
/// stop before any force is summed: the run ends with `node <id> is not finite at substep <n>`
/// (the file's node id), and no other node gets a NaN or moves in that substep, through the
/// solver and through `Sim::check`. Before the fix the beam loop spread the NaN to node 14.
#[test]
fn a_nan_node_shared_by_a_beam_and_a_tet_stops_the_step_before_any_force() {
    let at = Vec3::new(0.0, 10.0, 0.0);
    let mut b = one_tet(1e6, 0.02, 0.0, 1.0, 1.0, at);
    for (k, n) in b.nodes.iter_mut().enumerate() {
        n.id = 10 + k as u32;
    }
    for t in b.tets.iter_mut() {
        t.nodes = t.nodes.map(|i| i + 10);
    }
    add_node(&mut b, 14, at + Vec3::new(-1.0, 0.0, 0.0), 1.0);
    b.materials.push(Material {
        name: "m".into(),
        stiffness: 1e3,
        damping: 1.0,
        yield_strain: None,
        break_strain: None,
        unload_return: None,
    });
    add_beam(&mut b, 10, 14, "m"); // node 10 is in the tet and in the beam
    let h = 1e-5;
    let built = Body::build(&b);
    let mass: Vec<f64> = built.mass.clone();
    for (field, bad) in [("pos", f64::NAN), ("vel", f64::INFINITY)] {
        let mut s = SolverM5::from_body(&b, h).unwrap();
        let mut pos = built.pos.clone();
        let mut vel = vec![Vec3::new(0.1, 0.0, 0.0); 5];
        if field == "pos" {
            pos[0].x = bad;
        } else {
            vel[0].y = bad;
        }
        let (pos0, vel0) = (pos.clone(), vel.clone());
        s.step(
            State {
                pos: &mut pos,
                vel: &mut vel,
                beams: &built.beams,
                mass: &mass,
                gravity: 9.81,
            },
            h,
        );
        let ok = |v: Vec3| v.x.is_finite() && v.y.is_finite() && v.z.is_finite();
        for i in 1..5 {
            assert!(
                ok(pos[i]) && ok(vel[i]),
                "{field}: node {i} got a non-finite value"
            );
        }
        for i in 1..5 {
            assert!(
                pos[i] == pos0[i] && vel[i] == vel0[i],
                "{field}: node {i} moved in the stopped substep"
            );
        }
        assert_eq!(
            s.check(),
            Some("node 10 is not finite at substep 1".to_string()),
            "{field}"
        );
    }
    let cfg = Config {
        dt: 1e-4,
        substeps: 10,
        gravity: 9.81,
        duration: 1.0,
        ..Config::default()
    };
    let m5 = SolverM5::from_body(&b, cfg.dt / cfg.substeps as f64).unwrap();
    let mut sim = Sim::from_body_with_solver(cfg, &b, Box::new(m5)).unwrap();
    sim.pos[0].y = f64::NAN;
    sim.advance();
    assert_eq!(
        sim.check(),
        Err("node 10 is not finite at substep 1".to_string())
    );
    assert!(
        sim.pos[1..]
            .iter()
            .chain(&sim.vel)
            .all(|v| v.x.is_finite() && v.y.is_finite() && v.z.is_finite()),
        "a NaN spread to another node"
    );
}

// ---------------------------------------------------------------------------------------
// Round 3 (q-20261121-032354): the force array is checked before it is applied

fn state_bits(pos: &[Vec3], vel: &[Vec3]) -> Vec<u64> {
    pos.iter()
        .chain(vel)
        .flat_map(|v| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()])
        .collect()
}

/// Two nodes and one beam between them, `gap` apart, no tets.
fn two_node_beam(gap: f64, mass: f64, k: f64, rest: f64) -> Body {
    let mut b = empty_body();
    b.materials.push(Material {
        name: "m".into(),
        stiffness: k,
        damping: 0.0,
        yield_strain: None,
        break_strain: None,
        unload_return: None,
    });
    add_node(&mut b, 0, Vec3::new(-gap / 2.0, 10.0, 0.0), mass);
    add_node(&mut b, 1, Vec3::new(gap / 2.0, 10.0, 0.0), mass);
    b.beams.push(BodyBeam {
        a: 0,
        b: 1,
        material: "m".into(),
        rest,
    });
    b
}

/// The reviewer's body (a beam between finite coordinates of 1e200 m, whose length and force
/// overflow) is refused at construction by `MAX_COORD_M`, and a start speed above
/// `MAX_SPEED_MPS` is refused too; both bounds are inclusive.
#[test]
fn a_body_beyond_the_coordinate_or_speed_bound_is_refused() {
    let e = refused(&two_node_beam(2e200, 1.0, 1e3, 1.0), 1e-5);
    assert!(
        e.starts_with("node 0: its position (-1e200, 1e1, 0e0) is beyond MAX_COORD_M = 1000000 m"),
        "{e}"
    );
    // At the bound: taken; just past it: refused.
    let mut at = two_node_beam(2.0, 1.0, 1e3, 2.0);
    at.nodes[1].pos.x = solver_m5::MAX_COORD_M;
    assert!(SolverM5::from_body(&at, 1e-5).is_ok());
    at.nodes[1].pos.z = -solver_m5::MAX_COORD_M * (1.0 + f64::EPSILON);
    assert!(refused(&at, 1e-5).starts_with("node 1: its position"));
    let mut fast = two_node_beam(2.0, 1.0, 1e3, 2.0);
    fast.groups.push(format::Group {
        name: "all".into(),
        nodes: vec![0, 1],
    });
    fast.velocities.push(format::Velocity {
        group: "all".into(),
        velocity: Vec3::new(6e3, 8e3, 0.0), // 1e4 m/s: at the bound
    });
    assert!(SolverM5::from_body(&fast, 1e-5).is_ok());
    fast.velocities[0].velocity.z = 1.0;
    assert!(
        refused(&fast, 1e-5).starts_with("velocity of group `all`: speed 1.00000000"),
        "{}",
        refused(&fast, 1e-5)
    );
    fast.velocities[0].velocity.z = f64::NAN;
    assert!(refused(&fast, 1e-5).contains("is above MAX_SPEED_MPS = 10000 m/s"));
}

/// A body that passes construction but whose force overflows during the run: nodes of 1e300 kg
/// and a beam of 5e307 N/m (stable: h^2 2k/m = 1 < 4 at h = 1e-4) with an explicit rest length
/// of 0.5 m between nodes 5 m apart, so k (len - rest) = 2.25e308 overflows in the first
/// substep. Through `Sim`: the run ends with the named error and the state never changes.
#[test]
fn a_beam_force_that_overflows_during_the_run_is_stopped_before_it_is_applied() {
    let b = two_node_beam(5.0, 1e300, 5e307, 0.5);
    let cfg = Config {
        dt: 1e-3,
        substeps: 10,
        gravity: 9.81,
        duration: 1.0,
        ..Config::default()
    };
    let m5 = SolverM5::from_body(&b, cfg.dt / cfg.substeps as f64).unwrap();
    let mut sim = Sim::from_body_with_solver(cfg, &b, Box::new(m5)).unwrap();
    let start = state_bits(&sim.pos, &sim.vel);
    sim.advance();
    assert!(
        state_bits(&sim.pos, &sim.vel) == start,
        "a substep changed the state"
    );
    assert_eq!(
        sim.check(),
        Err("force on node 0 is not finite at substep 1 (from beams)".to_string())
    );
}

/// The force check names each source of a non-finite force (gravity, beams, tets, the extra
/// forces of `step_with_forces`) and the update check an overflow of the new state; in every
/// case nothing of the substep is applied, the tets' plastic state included.
#[test]
fn the_force_check_catches_every_source_before_anything_is_applied() {
    let h = 1e-5;
    let run = |b: &Body,
               pos: &mut Vec<Vec3>,
               vel: &mut Vec<Vec3>,
               gravity: f64,
               extra: Option<&[Vec3]>|
     -> (Option<String>, SolverM5) {
        let mut s = SolverM5::from_body(b, h).unwrap();
        let mass: Vec<f64> = b.nodes.iter().map(|n| n.mass).collect();
        let beams = Body::build(b).beams;
        let before = state_bits(pos, vel);
        let st = State {
            pos,
            vel,
            beams: &beams,
            mass: &mass,
            gravity,
        };
        match extra {
            Some(e) => s.step_with_forces(st, e, h),
            None => s.step(st, h),
        }
        assert!(
            state_bits(pos, vel) == before,
            "the failed substep changed the state"
        );
        (s.check(), s)
    };
    // Gravity: 1e308 m/s^2 on 10 kg overflows.
    let b = one_tet(1e6, 0.02, 0.0, 1.0, 10.0, Vec3::new(0.0, 10.0, 0.0));
    let (mut pos, mut vel) = (positions(&b), vec![Vec3::ZERO; 4]);
    let (e, _) = run(&b, &mut pos, &mut vel, 1e308, None);
    assert_eq!(
        e.as_deref(),
        Some("force on node 0 is not finite at substep 1 (from gravity)")
    );
    // Beams: a node 1e200 m away (finite, past the node check) gives an infinite length.
    let b = two_node_beam(2.0, 1.0, 1e3, 2.0);
    let (mut pos, mut vel) = (positions(&b), vec![Vec3::ZERO; 2]);
    pos[1].x = 1e200;
    let (e, _) = run(&b, &mut pos, &mut vel, 9.81, None);
    assert_eq!(
        e.as_deref(),
        Some("force on node 0 is not finite at substep 1 (from beams)")
    );
    // Tets: two tets; tet 0 compressed past yield, tet 1 with a node 1e305 m out (Je finite,
    // mu Fe overflows). The flow of tet 0 must not be kept either.
    let mut b = one_tet(1e6, 0.02, 0.0, 1.0, 1.0, Vec3::new(0.0, 10.0, 0.0));
    for (i, q) in [
        Vec3::new(5.0, 10.0, 0.0),
        Vec3::new(6.0, 10.0, 0.0),
        Vec3::new(5.0, 11.0, 0.0),
        Vec3::new(5.0, 10.0, 1.0),
    ]
    .iter()
    .enumerate()
    {
        add_node(&mut b, 4 + i as u32, *q, 1.0);
    }
    add_tet(&mut b, [4, 5, 6, 7], "s");
    let squeezed = |far: bool| {
        let mut p = positions(&b);
        p[1].x = 0.5;
        if far {
            p[5].x = 1e305;
        }
        p
    };
    let (mut pos, mut vel) = (squeezed(false), vec![Vec3::ZERO; 8]);
    let mut s_ok = SolverM5::from_body(&b, h).unwrap();
    let mass = vec![1.0; 8];
    s_ok.step(
        State {
            pos: &mut pos,
            vel: &mut vel,
            beams: &[],
            mass: &mass,
            gravity: 0.0,
        },
        h,
    );
    assert_eq!(s_ok.check(), None);
    assert!(
        s_ok.plastic_strain(0) > 0.0,
        "control: the squeezed tet yields"
    );
    let (mut pos, mut vel) = (squeezed(true), vec![Vec3::ZERO; 8]);
    let (e, s) = run(&b, &mut pos, &mut vel, 0.0, None);
    assert_eq!(
        e.as_deref(),
        Some("force on node 4 is not finite at substep 1 (from tets)")
    );
    assert_eq!(
        s.plastic_strain(0),
        0.0,
        "the flow of a failed substep was kept"
    );
    // Extra forces of step_with_forces.
    let b = two_node_beam(2.0, 1.0, 1e3, 2.0);
    let (mut pos, mut vel) = (positions(&b), vec![Vec3::ZERO; 2]);
    let extra = [Vec3::ZERO, Vec3::new(0.0, f64::INFINITY, 0.0)];
    let (e, _) = run(&b, &mut pos, &mut vel, 9.81, Some(&extra));
    assert_eq!(
        e.as_deref(),
        Some("force on node 1 is not finite at substep 1 (from extra forces)")
    );
    // The update: finite force, finite state, but x + v h overflows (one free node, h = 1 s).
    let mut b = empty_body();
    add_node(&mut b, 0, Vec3::ZERO, 1.0);
    let mut s = SolverM5::from_body(&b, 1.0).unwrap();
    let (mut pos, mut vel) = (
        vec![Vec3::new(1.7e308, 0.0, 0.0)],
        vec![Vec3::new(1.7e308, 0.0, 0.0)],
    );
    let before = state_bits(&pos, &vel);
    s.step(
        State {
            pos: &mut pos,
            vel: &mut vel,
            beams: &[],
            mass: &[1.0],
            gravity: 0.0,
        },
        1.0,
    );
    assert!(
        state_bits(&pos, &vel) == before,
        "the failed update changed the state"
    );
    assert_eq!(
        s.check().as_deref(),
        Some("update of node 0 is not finite at substep 1")
    );
}

/// Round 4 (q-20261121-034426): every error path of a substep applies nothing. Body: tet 0
/// (nodes 0-3) squeezed past yield, so a committed substep would change its plastic state;
/// tet 1 (nodes 4-7); a beam between nodes 8 and 9. Each case adds one fault and checks the
/// named error, that the state is bit-identical, and that neither tet's plastic state changed.
/// The controls show tet 0 really flows at both steps when there is no fault. (The update
/// overflow path needs h = 1 s, which a tet refuses; it is checked on a free node above.)
#[test]
fn every_error_path_of_a_substep_applies_nothing() {
    let mut b = one_tet(1e6, 0.02, 0.0, 1.0, 2.0, Vec3::new(0.0, 10.0, 0.0));
    for (i, q) in [
        Vec3::new(5.0, 10.0, 0.0),
        Vec3::new(6.0, 10.0, 0.0),
        Vec3::new(5.0, 11.0, 0.0),
        Vec3::new(5.0, 10.0, 1.0),
        Vec3::new(10.0, 10.0, 0.0),
        Vec3::new(11.0, 10.0, 0.0),
    ]
    .iter()
    .enumerate()
    {
        add_node(&mut b, 4 + i as u32, *q, 2.0);
    }
    add_tet(&mut b, [4, 5, 6, 7], "s");
    b.materials.push(Material {
        name: "m".into(),
        stiffness: 1e3,
        damping: 0.0,
        yield_strain: None,
        break_strain: None,
        unload_return: None,
    });
    add_beam(&mut b, 8, 9, "m");
    let limit = SolverM5::from_body(&b, 1e-12).unwrap().step_limit();
    let beams = Body::build(&b).beams;
    let mass = vec![2.0; 10];
    let eye_bits = |m: Mat3| m.iter().flatten().map(|x| x.to_bits()).collect::<Vec<_>>();
    let identity = eye_bits([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);
    let squeezed = || {
        let mut p = positions(&b);
        p[1].x = 0.9; // tet 0: F = diag(0.9, 1, 1), past yield
        p
    };
    // One substep with a fault; returns the error and the solver.
    let run = |h: f64,
               pos: &mut Vec<Vec3>,
               gravity: f64,
               extra: Option<&[Vec3]>|
     -> (Option<String>, SolverM5) {
        let mut s = SolverM5::from_body(&b, h).unwrap();
        let mut vel = vec![Vec3::ZERO; 10];
        let before = state_bits(pos, &vel);
        let st = State {
            pos,
            vel: &mut vel,
            beams: &beams,
            mass: &mass,
            gravity,
        };
        match extra {
            Some(e) => s.step_with_forces(st, e, h),
            None => s.step(st, h),
        }
        let err = s.check();
        if err.is_some() {
            assert!(
                state_bits(pos, &vel) == before,
                "{err:?}: the failed substep changed the state"
            );
            for t in 0..2 {
                assert_eq!(
                    s.plastic_strain(t),
                    0.0,
                    "{err:?}: tet {t} kept a plastic flow"
                );
                assert!(
                    eye_bits(s.fp(t)) == identity,
                    "{err:?}: tet {t} kept a new Fp"
                );
            }
        }
        (err, s)
    };
    let (small, big) = (0.1 * limit, 0.9 * limit);
    for h in [small, big] {
        let (e, s) = run(h, &mut squeezed(), 9.81, None);
        assert_eq!(e, None, "control at h = {h}");
        assert!(s.plastic_strain(0) > 0.0, "control: tet 0 flows at h = {h}");
    }
    let mut p = squeezed();
    p[8].x = f64::NAN;
    assert_eq!(
        run(small, &mut p, 9.81, None).0.as_deref(),
        Some("node 8 is not finite at substep 1")
    );
    assert_eq!(
        run(small, &mut squeezed(), 1e308, None).0.as_deref(),
        Some("force on node 0 is not finite at substep 1 (from gravity)")
    );
    let mut p = squeezed();
    p[9].x = 1e200;
    assert_eq!(
        run(small, &mut p, 9.81, None).0.as_deref(),
        Some("force on node 8 is not finite at substep 1 (from beams)")
    );
    let mut p = squeezed();
    p[5].x = 1e305;
    assert_eq!(
        run(small, &mut p, 9.81, None).0.as_deref(),
        Some("force on node 4 is not finite at substep 1 (from tets)")
    );
    let mut extra = vec![Vec3::ZERO; 10];
    extra[8].x = f64::INFINITY;
    assert_eq!(
        run(small, &mut squeezed(), 9.81, Some(&extra)).0.as_deref(),
        Some("force on node 8 is not finite at substep 1 (from extra forces)")
    );
    // The Je rule: tet 1's apex brought to 1 cm above its base.
    let mut p = squeezed();
    p[7].z = 0.01;
    let e = run(small, &mut p, 9.81, None).0.unwrap();
    assert!(
        e.starts_with("tet 1 inverted or collapsed at substep 1: Je = 0.0"),
        "{e}"
    );
    // The stiffness verdict: tet 1 sheared by 1.5 at h = 0.9 of the construction limit.
    let mut p = squeezed();
    p[5].y += 1.5;
    let e = run(big, &mut p, 9.81, None).0.unwrap();
    assert!(
        e.starts_with("stiffness bound exceeded at substep 1: h omega = "),
        "{e}"
    );
}

/// Round 4 (q-20261121-034426): construction refuses, by name, every number its stability
/// bound is built from that is not finite, instead of replacing it (the old code set a
/// non-finite w2 or g to 0, which accepted any step).
#[test]
fn construction_refuses_every_non_finite_bound_input() {
    let at = Vec3::new(0.0, 10.0, 0.0);
    // The reviewer's case: plateau 1e308 Pa at yield 0.02 gives E = 5e309 Pa.
    let e = refused(&one_tet(1e308, 0.02, 0.0, 1.0, 1.0, at), 1e-5);
    assert_eq!(
        e,
        "solid `s`: its elastic modulus E = plateau_stress / yield_strain = inf Pa is not finite"
    );
    // E finite (1.7e308 Pa) but the tangent at rest times the headroom, 2 (2 mu + 3 lambda),
    // overflows: refused at the node sums.
    let e = refused(&one_tet(1.7e306, 0.01, 0.0, 1.0, 1.0, at), 1e-5);
    assert!(
        e.starts_with("node 0: its m5 stability sums (stiffness inf"),
        "{e}"
    );
    // A beam whose doubled stiffness overflows (2 x 1e308).
    let e = refused(&two_node_beam(2.0, 1.0, 1e308, 2.0), 1e-5);
    assert!(
        e.starts_with("node 0: its m5 stability sums (stiffness inf"),
        "{e}"
    );
    // Finite sums over a node of 1e-300 kg: w2 = 2e10 / 1e-300 overflows.
    let e = refused(&two_node_beam(2.0, 1e-300, 1e10, 2.0), 1e-5);
    assert!(
        e.starts_with("the m5 stability bound is not finite: w2 = inf"),
        "{e}"
    );
    // The damping coefficient: E = 1e306 Pa and a zone density of 4800 kg/m^3 (200 kg nodes on
    // a 1/6 m^3 tet; eps_D = 0.144, above the yield 0.01) make (lambda + 2 mu) rho overflow.
    let e = refused(&one_tet(1e304, 0.01, 0.05, 1.0, 200.0, at), 1e-5);
    assert!(
        e.starts_with(
            "tet 0: its damping coefficient eta = 2 zeta L sqrt((lambda + 2 mu) rho) = inf"
        ),
        "{e}"
    );
    // A material built in code with a NaN or a negative number (the format refuses both).
    for (k, c) in [(f64::NAN, 0.0), (-1e3, 0.0), (1e3, f64::NAN), (1e3, -1.0)] {
        let mut b = two_node_beam(2.0, 1.0, 1e3, 2.0);
        b.materials[0].stiffness = k;
        b.materials[0].damping = c;
        let e = refused(&b, 1e-5);
        assert!(
            e.starts_with("beam 0: its material `m` has stiffness"),
            "{e}"
        );
    }
}

/// Round 5 (q-20261121-040727), point 1: an apex that crosses its opposite face within one
/// substep. One substep per output step, so the substep that inverts the tet is the last one
/// before `Sim::check`: the new positions' Je is checked before they are written, the run ends
/// with the named error, and the state is bit-identical to the state before the substep.
#[test]
fn a_tet_inverted_within_one_substep_is_caught_on_the_updated_positions() {
    let b = one_tet(10.0, 0.5, 0.0, 1.0, 1.0, Vec3::new(0.0, 100.0, 0.0));
    let cfg = Config {
        dt: 2e-4,
        substeps: 1,
        gravity: 0.0,
        duration: 1.0,
        ..Config::default()
    };
    let h = cfg.dt / cfg.substeps as f64;
    let m5 = SolverM5::from_body(&b, h).unwrap();
    let mut sim = Sim::from_body_with_solver(cfg, &b, Box::new(m5)).unwrap();
    // 1e4 m/s down (MAX_SPEED_MPS): the apex moves 2 m in the substep, from z = 1 to z = -1.
    sim.vel[3] = Vec3::new(0.0, 0.0, -solver_m5::MAX_SPEED_MPS);
    let before = state_bits(&sim.pos, &sim.vel);
    sim.advance();
    let e = sim.check().expect_err("the inverted tet passed Sim::check");
    assert!(
        e.starts_with("tet 0 inverted or collapsed at the updated positions of substep 1: Je = -"),
        "{e}"
    );
    assert!(
        state_bits(&sim.pos, &sim.vel) == before,
        "the failed substep changed the state"
    );
}

/// Round 5, point 2: no non-finite value is ever written into the working force array. Each
/// contribution, and the sum it would make, is checked before it is added; the solver's
/// `debug_assert!` after each source (tests build in debug) checks the whole array, so a
/// violation panics here. Two finite beam forces of 1.2e308 N on one node (4e307 N/m, 3 m
/// stretch each, 1e300 kg nodes, stable at h = 1e-4) whose sum overflows; and a beam whose
/// length overflows (a node 1e200 m away in the state).
#[test]
fn an_overflowing_beam_never_reaches_the_force_array() {
    let mut b = empty_body();
    b.materials.push(Material {
        name: "m".into(),
        stiffness: 4e307,
        damping: 0.0,
        yield_strain: None,
        break_strain: None,
        unload_return: None,
    });
    add_node(&mut b, 0, Vec3::new(0.0, 10.0, 0.0), 1e300);
    add_node(&mut b, 1, Vec3::new(3.5, 10.0, 0.0), 1e300);
    add_node(&mut b, 2, Vec3::new(3.5, 10.0, 1e-3), 1e300);
    for c in [1, 2] {
        b.beams.push(BodyBeam {
            a: 0,
            b: c,
            material: "m".into(),
            rest: 0.5,
        });
    }
    let cfg = Config {
        dt: 1e-3,
        substeps: 10,
        gravity: 9.81,
        duration: 1.0,
        ..Config::default()
    };
    let m5 = SolverM5::from_body(&b, cfg.dt / cfg.substeps as f64).unwrap();
    let mut sim = Sim::from_body_with_solver(cfg, &b, Box::new(m5)).unwrap();
    let before = state_bits(&sim.pos, &sim.vel);
    sim.advance();
    assert_eq!(
        sim.check(),
        Err("force on node 0 is not finite at substep 1 (from beams)".to_string())
    );
    assert!(state_bits(&sim.pos, &sim.vel) == before);

    let b = two_node_beam(2.0, 1.0, 1e3, 2.0);
    let mut s = SolverM5::from_body(&b, 1e-5).unwrap();
    let (mut pos, mut vel) = (positions(&b), vec![Vec3::ZERO; 2]);
    pos[1].x = 1e200;
    let beams = Body::build(&b).beams;
    s.step(
        State {
            pos: &mut pos,
            vel: &mut vel,
            beams: &beams,
            mass: &[1.0, 1.0],
            gravity: 9.81,
        },
        1e-5,
    );
    assert_eq!(
        s.check().as_deref(),
        Some("force on node 0 is not finite at substep 1 (from beams)")
    );
}

// ---------------------------------------------------------------------------------------
// R1, R10: every refusal of the constructor

#[test]
fn hostile_and_bad_bodies_are_refused_by_name() {
    let at = Vec3::new(0.0, 10.0, 0.0);
    let good = || one_tet(1e6, 0.02, 0.0, 1.0, 1.0, at);
    assert!(SolverM5::from_body(&good(), 1e-5).is_ok());

    // 1e6 tets: refused by the cap before anything per tet is built.
    let mut huge = good();
    let t = huge.tets[0].clone();
    huge.tets = vec![t; 1_000_000];
    assert_eq!(refused(&huge, 1e-5), "the body has 1000000 tets, more than the m5 solver takes (M5_MAX_TETS = 300, docs/M5-SOLVER.md `frozen: m5_max_tets`)");
    let mut over = good();
    over.tets = vec![over.tets[0].clone(); M5_MAX_TETS + 1];
    assert!(refused(&over, 1e-5).starts_with("the body has 301 tets"));

    for h in [0.0, -1e-5, f64::NAN, f64::INFINITY] {
        assert!(
            refused(&good(), h).ends_with("s must be a positive finite number"),
            "h = {h}"
        );
    }

    let mut nan = good();
    nan.nodes[3].pos.z = f64::NAN;
    assert_eq!(refused(&nan, 1e-5), "node 3: its position is not finite");
    let mut heavy = good();
    heavy.nodes[1].mass = 0.0;
    assert_eq!(
        refused(&heavy, 1e-5),
        "node 1: its mass 0 must be a positive finite number"
    );
    let mut twice = good();
    twice.nodes[2].id = 1;
    assert_eq!(refused(&twice, 1e-5), "node id 1 is given twice");

    let mut beamy = good();
    beamy.materials.push(Material {
        name: "m".into(),
        stiffness: 1e3,
        damping: 0.0,
        yield_strain: None,
        break_strain: None,
        unload_return: None,
    });
    beamy.beams.push(BodyBeam {
        a: 0,
        b: 9,
        material: "m".into(),
        rest: 1.0,
    });
    assert_eq!(refused(&beamy, 1e-5), "beam 0 is on a missing node");
    beamy.beams[0] = BodyBeam {
        a: 0,
        b: 1,
        material: "nope".into(),
        rest: 1.0,
    };
    assert_eq!(
        refused(&beamy, 1e-5),
        "beam 0 names a missing material `nope`"
    );

    let mut dup = good();
    dup.solids.push(solid("s", 1e6, 0.02, 0.0));
    assert_eq!(refused(&dup, 1e-5), "solid `s` is given twice");
    for (p, y, z, want) in [
        (
            0.0,
            0.02,
            0.0,
            "solid `s`: plateau_stress 0 must be a positive finite number",
        ),
        (
            f64::NAN,
            0.02,
            0.0,
            "solid `s`: plateau_stress NaN must be a positive finite number",
        ),
        (
            1e6,
            0.0,
            0.0,
            "solid `s`: yield_strain 0 must be > 0 and < 1",
        ),
        (
            1e6,
            1.0,
            0.0,
            "solid `s`: yield_strain 1 must be > 0 and < 1",
        ),
        (
            1e6,
            f64::NAN,
            0.0,
            "solid `s`: yield_strain NaN must be > 0 and < 1",
        ),
        (
            1e6,
            0.02,
            -0.1,
            "solid `s`: damping_ratio -0.1 must be >= 0 and <= 1",
        ),
        (
            1e6,
            0.02,
            1.5,
            "solid `s`: damping_ratio 1.5 must be >= 0 and <= 1",
        ),
        (
            1e6,
            0.02,
            f64::NAN,
            "solid `s`: damping_ratio NaN must be >= 0 and <= 1",
        ),
    ] {
        let mut b = good();
        b.solids[0] = solid("s", p, y, z);
        assert_eq!(refused(&b, 1e-5), want);
    }

    let mut unk = good();
    unk.tets[0].solid = "t".into();
    assert_eq!(refused(&unk, 1e-5), "tet 0 (0 1 2 3): unknown solid `t`");
    let mut miss = good();
    miss.tets[0].nodes[2] = 7;
    assert_eq!(refused(&miss, 1e-5), "tet 0 (0 1 7 3): node 7 is missing");
    let mut rep = good();
    rep.tets[0].nodes[3] = 1;
    assert_eq!(refused(&rep, 1e-5), "tet 0 (0 1 2 1): lists node 1 twice");

    // Zero volume (four coplanar nodes) and collinear nodes.
    let mut flat = good();
    flat.nodes[3].pos = at + Vec3::new(1.0, 1.0, 0.0);
    assert!(
        refused(&flat, 1e-5).starts_with("tet 0 (0 1 2 3): its rest volume 0 is below the minimum"),
        "{}",
        refused(&flat, 1e-5)
    );
    let mut line = good();
    line.nodes[2].pos = at + Vec3::new(2.0, 0.0, 0.0);
    line.nodes[3].pos = at + Vec3::new(3.0, 0.0, 0.0);
    assert!(
        refused(&line, 1e-5).starts_with("tet 0 (0 1 2 3): its rest volume 0 is below the minimum")
    );
    // A sliver: volume above the minimum only because it is large.
    let mut sliver = good();
    for (i, q) in [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1000.0, 0.0, 0.0),
        Vec3::new(0.0, 1000.0, 0.0),
        Vec3::new(0.0, 0.0, 1e-4),
    ]
    .iter()
    .enumerate()
    {
        sliver.nodes[i].pos = *q;
    }
    assert!(refused(&sliver, 1e-5).starts_with("tet 0 (0 1 2 3) is too flat"));
    let mut inv = good();
    inv.tets[0].nodes = [0, 2, 1, 3];
    assert!(
        refused(&inv, 1e-5).starts_with("tet 0 (0 2 1 3) is inverted: its rest volume -0.16666")
    );
    let mut far = good();
    far.nodes[1].pos.x = 1e300;
    far.nodes[2].pos.y = 1e300;
    far.nodes[3].pos.z = 1e300;
    // Since round 3 the coordinate bound refuses this first (the tet's own size check stays
    // behind it, for a bound that is ever raised).
    assert_eq!(
        refused(&far, 1e-5),
        "node 1: its position (1e300, 1e1, 0e0) is beyond MAX_COORD_M = 1000000 m in a coordinate"
    );
    let mut same = good();
    same.tets.push(Tet {
        nodes: [0, 2, 3, 1],
        solid: "s".into(),
    });
    assert_eq!(
        refused(&same, 1e-5),
        "tet 1 (0 2 3 1) is on the same four nodes as tet 0"
    );

    // A node shared by tets of two solids.
    let mut two = good();
    two.solids.push(solid("u", 1e6, 0.02, 0.0));
    add_node(&mut two, 4, at + Vec3::new(1.0, 1.0, 1.0), 1.0);
    add_tet(&mut two, [1, 2, 3, 4], "u");
    let e = refused(&two, 1e-5);
    assert_eq!(
        e,
        "tet 1 (1 2 3 4): node 1 is shared by tets of two solids, `s` and `u`"
    );

    // A zone so dense its lock-up strain is not above the yield strain: phi = 4 m / (V 7850).
    let mut dense = good();
    for n in dense.nodes.iter_mut() {
        n.mass = 200.0; // phi = 800 / (7850 / 6) = 0.611, eps_D = 1 - 1.4 phi = 0.144
    }
    dense.solids[0].yield_strain = 0.2;
    assert!(refused(&dense, 1e-5).starts_with("solid `s`: the zone's lock-up strain eps_D = 0.14"));
}

// ---------------------------------------------------------------------------------------
