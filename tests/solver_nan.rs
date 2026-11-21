//! The default solver (`SymplecticEuler`) stops a non-finite node, or a non-finite force, before
//! anything is written, and reports it by name through `Solver::check` and `Sim::check`. All
//! expected values are exact `==` or exact error text.

use crash_sim::solver::{Solver, State, SymplecticEuler};
use crash_sim::vec3::Vec3;
use crash_sim::{Beam, Config, Sim};

/// A chain of four nodes 1 m apart with a beam between neighbours, at rest, no gravity.
fn chain() -> (Vec<Vec3>, Vec<Vec3>, Vec<Beam>, Vec<f64>) {
    let pos: Vec<Vec3> = (0..4).map(|i| Vec3::new(i as f64, 5.0, 0.0)).collect();
    let beams = (0..3)
        .map(|i| Beam { a: i, b: i + 1, rest: 1.0, stiffness: 1000.0, damping: 1.0 })
        .collect();
    (pos, vec![Vec3::ZERO; 4], beams, vec![1.0; 4])
}

fn all_finite(v: &[Vec3]) -> bool {
    v.iter().all(|p| p.x.is_finite() && p.y.is_finite() && p.z.is_finite())
}

#[test]
fn a_nan_node_ends_with_the_named_error_and_spreads_to_no_other_node() {
    let (mut pos, mut vel, beams, mass) = chain();
    pos[2].y = f64::NAN;
    let (pos0, vel0) = (pos.clone(), vel.clone());
    let mut s = SymplecticEuler::default();
    assert_eq!(s.check(), None);
    s.step(State { pos: &mut pos, vel: &mut vel, beams: &beams, mass: &mass, gravity: 9.81 }, 0.001);
    assert_eq!(s.check(), Some("node 2 is not finite at substep 1".to_string()));
    // Nothing was applied: every node but the NaN one is finite and unchanged, the velocities too.
    for i in [0usize, 1, 3] {
        assert!(pos[i] == pos0[i], "node {i} position moved");
    }
    assert!(pos[2].x == pos0[2].x && pos[2].y.is_nan());
    assert!(vel == vel0);
    assert!(all_finite(&vel));
    // The first error stays, and a later substep applies nothing.
    s.step(State { pos: &mut pos, vel: &mut vel, beams: &beams, mass: &mass, gravity: 9.81 }, 0.001);
    assert_eq!(s.check(), Some("node 2 is not finite at substep 1".to_string()));
    assert!(vel == vel0);
}

#[test]
fn a_nan_velocity_is_named_too() {
    let (mut pos, mut vel, beams, mass) = chain();
    vel[3].x = f64::INFINITY;
    let mut s = SymplecticEuler::default();
    s.step(State { pos: &mut pos, vel: &mut vel, beams: &beams, mass: &mass, gravity: 0.0 }, 0.001);
    assert_eq!(s.check(), Some("node 3 is not finite at substep 1".to_string()));
    assert!(all_finite(&pos));
}

#[test]
fn an_overflowing_beam_force_ends_with_the_named_error_and_applies_nothing() {
    let (mut pos, mut vel, mut beams, mass) = chain();
    // Finite stretch (2 m), but stiffness * stretch overflows f64 to infinity.
    beams[1].stiffness = f64::MAX;
    pos[2].x = 4.0;
    let (pos0, vel0) = (pos.clone(), vel.clone());
    let mut s = SymplecticEuler::default();
    s.step(State { pos: &mut pos, vel: &mut vel, beams: &beams, mass: &mass, gravity: 9.81 }, 0.001);
    assert_eq!(
        s.check(),
        Some("force on node 1 is not finite at substep 1 (from beams)".to_string())
    );
    assert!(pos == pos0 && vel == vel0, "a failed substep wrote something");
}

#[test]
fn an_overflowing_extra_force_ends_with_the_named_error() {
    let (mut pos, mut vel, beams, mass) = chain();
    let mut extra = vec![Vec3::ZERO; 4];
    extra[1] = Vec3::new(f64::INFINITY, 0.0, 0.0);
    let (pos0, vel0) = (pos.clone(), vel.clone());
    let mut s = SymplecticEuler::default();
    s.step_with_forces(
        State { pos: &mut pos, vel: &mut vel, beams: &beams, mass: &mass, gravity: 0.0 },
        &extra,
        0.001,
    );
    assert_eq!(
        s.check(),
        Some("force on node 1 is not finite at substep 1 (from extra forces)".to_string())
    );
    assert!(pos == pos0 && vel == vel0);
}

#[test]
fn sim_check_reports_the_solver_error_with_the_node_named() {
    let mut sim = Sim::new(Config::default());
    sim.pos[3].x = f64::NAN;
    sim.integrate(0.001);
    // Sim::check reads the solver's error first, so the substep is named, not just the index.
    assert_eq!(sim.check(), Err("node 3 is not finite at substep 1".to_string()));
}
