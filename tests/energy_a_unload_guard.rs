#![allow(dead_code)]

#[path = "../examples/crash_energy_a.rs"]
mod measurement;

use crash_sim::solver::SymplecticEuler;
use crash_sim::Sim;
use measurement::*;

#[test]
fn energy_a_refuses_an_unload_body() {
    let (_, mut body) = inputs().unwrap();
    validate_body(&body).unwrap();
    body.materials[0].unload_return = Some(0.5);
    assert!(validate_body(&body).unwrap_err().contains("unload_return"));
}

#[test]
fn energy_a_split_step_refuses_an_unload_sim() {
    let (cfg, body) = inputs().unwrap();
    let h = cfg.dt / f64::from(cfg.substeps);
    let mut sim = Sim::from_car(cfg, &body).unwrap();
    sim.unload_return[0] = Some(0.5);
    let materials = vec![0; sim.beams.len()];
    for enabled in [false, true] {
        let original_pos = sim.pos.clone();
        let original_vel = sim.vel.clone();
        let mut totals = Totals::default();
        assert!(
            split_step(
                &mut sim,
                &mut SymplecticEuler::default(),
                h,
                &materials,
                enabled.then_some(&mut totals),
            )
            .unwrap_err()
            .contains("unload_return")
        );
        assert_eq!(sim.pos, original_pos, "refuse before position update");
        assert_eq!(sim.vel, original_vel, "refuse before velocity update");
    }
}
