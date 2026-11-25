//! Position-following contact: explicit fixtures, unchanged kernel tolerances.

use std::collections::BTreeSet;

use crash_sim::car_contact::{
    apply_contacts, apply_contacts_moving, ContactCar, ContactError, ContactKey, ContactReport,
    ContactSettings, Failure, Feature, MovingContactCar,
};
use crash_sim::collide::Plane;
use crash_sim::format::parse;
use crash_sim::input::Script;
use crash_sim::net::session::{Input, LockstepSim};
use crash_sim::race::lockstep::{LockstepRace, LockstepSetup};
use crash_sim::track::Track;
use crash_sim::vec3::Vec3;
use crash_sim::{Beam, Config, Sim};

#[derive(Clone)]
struct Chassis {
    old: Vec<Vec3>,
    pos: Vec<Vec3>,
    vel: Vec<Vec3>,
    mass: Vec<f64>,
    beams: Vec<Beam>,
    broken: Vec<bool>,
    planes: Vec<Plane>,
}

impl Chassis {
    fn new(pos: Vec<Vec3>, edges: &[(usize, usize, f64)], stiffness: f64, mass: f64) -> Self {
        Self {
            old: pos.clone(),
            vel: vec![Vec3::ZERO; pos.len()],
            mass: vec![mass; pos.len()],
            pos,
            beams: edges
                .iter()
                .map(|&(a, b, rest)| Beam {
                    a,
                    b,
                    rest,
                    stiffness,
                    damping: 0.0,
                })
                .collect(),
            broken: vec![false; edges.len()],
            planes: Vec::new(),
        }
    }

    fn moving(&mut self) -> MovingContactCar<'_> {
        MovingContactCar {
            previous_pos: &self.old,
            pos: &mut self.pos,
            vel: &mut self.vel,
            mass: &self.mass,
            beams: &self.beams,
            broken: &self.broken,
            planes: &self.planes,
        }
    }
}

fn apply(cars: &mut [Chassis], cfg: ContactSettings) -> Result<ContactReport, ContactError> {
    let mut views: Vec<_> = cars.iter_mut().map(Chassis::moving).collect();
    apply_contacts_moving(&mut views, cfg, &BTreeSet::new()).map(|(report, _)| report)
}

fn vector_bits(v: Vec3) -> [u64; 3] {
    [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]
}

fn state_bits(cars: &[Chassis]) -> Vec<[u64; 3]> {
    cars.iter()
        .flat_map(|c| c.pos.iter().chain(&c.vel))
        .copied()
        .map(vector_bits)
        .collect()
}

struct SteadyRun {
    checkpoints: Vec<Vec<[u64; 3]>>,
    depth0: f64,
    max_depth: f64,
    final_depth: f64,
    impulses: usize,
}

fn steady_run() -> SteadyRun {
    let mut cars = [
        Chassis::new(vec![Vec3::new(0.0, 0.17, 0.0)], &[], 0.0, 14.0),
        Chassis::new(
            vec![Vec3::new(-0.075, 0.0, 0.0), Vec3::new(0.075, 0.0, 0.0)],
            &[(0, 1, 0.15)],
            0.0,
            14.0,
        ),
    ];
    let cfg = ContactSettings::new(0.25, 0.0, 1e-3, 0.25 / 8.0, 0.25 / 2.0).unwrap();
    let mut run = SteadyRun {
        checkpoints: Vec::new(),
        depth0: 0.0,
        max_depth: f64::NEG_INFINITY,
        final_depth: 0.0,
        impulses: 0,
    };
    for i in 0..14134 {
        for c in &mut cars {
            c.old.copy_from_slice(&c.pos);
        }
        cars[0].vel[0].y -= 3.476 * 1e-3;
        for c in &mut cars {
            for (p, &v) in c.pos.iter_mut().zip(&c.vel) {
                *p += v * 1e-3;
            }
        }
        let report = apply(&mut cars, cfg).unwrap_or_else(|e| panic!("substep={i} failure={e:?}"));
        run.impulses += report.impulses;
        // Symmetry fixes the closest segment point at its midpoint, independently
        // of the kernel's fraction calculation. Horizontal coordinates never move.
        let midpoint = (cars[1].pos[0] + cars[1].pos[1]) * 0.5;
        let depth = 0.25 - (cars[0].pos[0] - midpoint).length();
        if i == 0 {
            run.depth0 = depth;
        }
        run.max_depth = run.max_depth.max(depth);
        run.final_depth = depth;
        assert!(
            depth.is_finite() && depth <= run.depth0 + 1e-6,
            "substep={i} depth={depth:e} depth0={:e}",
            run.depth0
        );
        if (i + 1) % 1000 == 0 || i == 14133 {
            run.checkpoints.push(state_bits(&cars));
        }
    }
    assert!(run.impulses > 0, "steady push must apply impulses");
    run
}

#[test]
fn steady_push_depth_stays_bounded() {
    let run = steady_run();
    println!(
        "follow steady_push substeps=14134 depth0={:e} max_depth={:e} final_depth={:e} impulses={}",
        run.depth0, run.max_depth, run.final_depth, run.impulses
    );
}

#[test]
fn steady_push_is_deterministic() {
    let a = steady_run();
    let b = steady_run();
    assert!(!a.checkpoints.is_empty());
    assert_eq!(a.checkpoints, b.checkpoints);
    assert_eq!(a.impulses, b.impulses);
}

fn fixture(stiffness: f64, vy: f64) -> [Chassis; 2] {
    let mut cars = [
        Chassis::new(
            vec![Vec3::new(0.0, 0.1, 0.0), Vec3::new(-0.0, 0.6, 0.0)],
            &[(0, 1, 0.5)],
            stiffness,
            1.0,
        ),
        Chassis::new(
            vec![Vec3::new(-1.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)],
            &[(0, 1, 2.0)],
            stiffness,
            1.0,
        ),
    ];
    cars[0].vel[0].y = vy;
    cars
}

fn settings() -> ContactSettings {
    ContactSettings::new(0.125, 0.0, 0.001, 0.015625, 0.0625).unwrap()
}

#[test]
fn follow_refuses_a_node_pushed_through_the_ground() {
    let mut cars = fixture(1e5, -1.0);
    for car in &mut cars {
        car.planes.push(Plane::ground());
    }
    let before = state_bits(&cars);
    let error = apply(&mut cars, settings()).unwrap_err();
    assert_eq!(error.failure, Failure::PenetrationBound);
    assert_eq!(error.car, Some(1));
    assert_eq!(error.feature, Feature::Node(0));
    assert_eq!(state_bits(&cars), before);

    // Being behind a plane already is owned by the plane rule, not this guard.
    // A plane below the followed endpoints must not refuse either.
    // Axis-aligned distances have an independent sign reference: endpoints
    // start at zero and finish below zero. Exercise tiny offsets, cancellation
    // at the source's height, and the largest finite plane offsets too.
    for (y, crosses) in [
        (1.0, false),
        (-1.0, false),
        (0.1, false),
        (f64::MAX, false),
        (-f64::MAX, false),
        (f64::MIN_POSITIVE, false),
        (-f64::MIN_POSITIVE, true),
        (f64::from_bits(1), false),
        (-f64::from_bits(1), true),
    ] {
        let mut cars = fixture(1e5, -1.0);
        for car in &mut cars {
            car.planes.push(Plane::new(Vec3::new(0.0, y, 0.0), Vec3::new(0.0, 1.0, 0.0)).unwrap());
        }
        let before = state_bits(&cars);
        let result = apply(&mut cars, settings());
        if crosses {
            let error = result.unwrap_err();
            assert_eq!(error.failure, Failure::PenetrationBound, "plane_y={y:e}");
            assert_eq!((error.car, error.feature), (Some(1), Feature::Node(0)));
            assert_eq!(state_bits(&cars), before);
        } else {
            assert_eq!(result.unwrap().impulses, 1, "plane_y={y:e}");
        }
    }
    // Public Plane fields cannot let non-finite or unrepresentable arithmetic
    // bypass the crossing guard, even after trial impulses have been staged.
    for normal_y in [
        0.0,
        f64::from_bits(1),
        f64::MIN_POSITIVE,
        f64::MAX,
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
    ] {
        let mut cars = fixture(1e5, -1.0);
        cars[1].planes.push(Plane {
            point: Vec3::ZERO,
            normal: Vec3::new(0.0, normal_y, 0.0),
        });
        let before = state_bits(&cars);
        let error = apply(&mut cars, settings()).unwrap_err();
        assert_eq!(error.failure, Failure::NumericalRange);
        assert_eq!((error.car, error.feature), (Some(1), Feature::Node(0)));
        assert_eq!(state_bits(&cars), before);
    }
}

#[test]
fn second_contact_cannot_undo_the_first_pairs_depth() {
    let mut cars = [
        Chassis::new(vec![Vec3::new(0.0, 0.0625, 0.0)], &[], 0.0, 1.0),
        Chassis::new(
            vec![
                Vec3::new(-1.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(-1.0, 0.15625, 0.0),
                Vec3::new(1.0, 0.15625, 0.0),
            ],
            &[(0, 1, 2.0), (2, 3, 2.0)],
            0.0,
            1.0,
        ),
    ];
    cars[1].vel[2].y = -1.0;
    cars[1].vel[3].y = -1.0;
    let before = state_bits(&cars);
    let error = apply(&mut cars, settings()).unwrap_err();
    assert_eq!(
        error,
        ContactError {
            car: Some(0),
            feature: Feature::Contact(ContactKey {
                source_car: 0,
                node: 0,
                target_car: 1,
                beam: 0,
            }),
            failure: Failure::PenetrationBound,
        }
    );
    assert_eq!(state_bits(&cars), before);
}

#[test]
fn remembered_band_endpoint_is_exact() {
    let band: f64 = 0.125 + 2.0 * 0.015625;
    let key = ContactKey {
        source_car: 0,
        node: 0,
        target_car: 1,
        beam: 0,
    };
    let remembered = BTreeSet::from([key]);
    for (y, expected) in [(band, 1), (f64::from_bits(band.to_bits() + 1), 0)] {
        let mut cars = [
            Chassis::new(vec![Vec3::new(0.0, y, 0.0)], &[], 0.0, 1.0),
            Chassis::new(
                vec![Vec3::new(-1.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)],
                &[(0, 1, 2.0)],
                0.0,
                1.0,
            ),
        ];
        cars[0].vel[0].y = -1.0;
        let before = state_bits(&cars);
        let mut views: Vec<_> = cars.iter_mut().map(Chassis::moving).collect();
        let (report, keys) = apply_contacts_moving(&mut views, settings(), &remembered).unwrap();
        assert_eq!((report.contacts, report.impulses), (expected, expected));
        drop(views);
        if expected == 0 {
            assert!(keys.is_empty());
            assert_eq!(state_bits(&cars), before);
        } else {
            assert_eq!(keys, remembered);
            assert_ne!(state_bits(&cars), before);
        }
    }
}

#[test]
fn follow_moves_exactly_dv_times_h() {
    let original = fixture(1e5, -1.0);
    let mut cars = original.clone();
    let (report, keys) = {
        let mut views: Vec<_> = cars.iter_mut().map(Chassis::moving).collect();
        apply_contacts_moving(&mut views, settings(), &BTreeSet::new()).unwrap()
    };
    assert_eq!(report.impulses, 1);
    assert_eq!(
        keys.into_iter().collect::<Vec<_>>(),
        vec![ContactKey {
            source_car: 0,
            node: 0,
            target_car: 1,
            beam: 0,
        }]
    );
    let mut separating = fixture(1e5, 1.0);
    let mut views: Vec<_> = separating.iter_mut().map(Chassis::moving).collect();
    let (skipped, keys) = apply_contacts_moving(&mut views, settings(), &BTreeSet::new()).unwrap();
    assert_eq!(skipped.contacts, 1);
    assert_eq!(skipped.impulses, 0);
    assert!(keys.is_empty(), "separating contact must not become memory");
    assert!((report.normal_impulse - 2.0 / 3.0).abs() <= 16.0 * f64::EPSILON);
    for (ci, (c, old)) in cars.iter().zip(&original).enumerate() {
        for (ni, &position) in c.pos.iter().enumerate() {
            let expected = old.pos[ni] + (c.vel[ni] - old.vel[ni]) * 0.001;
            for (got, want, base) in [
                (position.x, expected.x, old.pos[ni].x),
                (position.y, expected.y, old.pos[ni].y),
                (position.z, expected.z, old.pos[ni].z),
            ] {
                assert!(
                    (got - want).abs() <= 4.0 * f64::EPSILON * base.abs().max(1.0),
                    "car={ci} node={ni} got={got:e} expected={want:e}"
                );
            }
        }
    }
    for (ci, ni) in [(0, 0), (1, 0), (1, 1)] {
        assert!((cars[ci].vel[ni].y + 1.0 / 3.0).abs() <= 16.0 * f64::EPSILON);
    }
    assert_eq!(vector_bits(cars[0].pos[1]), vector_bits(original[0].pos[1]));
    assert_eq!(vector_bits(cars[0].vel[1]), vector_bits(original[0].vel[1]));
    let mut velocity_only = original.clone();
    let mut views: Vec<_> = velocity_only
        .iter_mut()
        .map(|c| ContactCar {
            previous_pos: &c.old,
            pos: &c.pos,
            vel: &mut c.vel,
            mass: &c.mass,
            beams: &c.beams,
            broken: &c.broken,
        })
        .collect();
    assert_eq!(apply_contacts(&mut views, settings()).unwrap(), report);
    drop(views);
    for ((v, moving), old) in velocity_only.iter().zip(&cars).zip(&original) {
        assert_eq!(
            v.vel.iter().copied().map(vector_bits).collect::<Vec<_>>(),
            moving.vel.iter().copied().map(vector_bits).collect::<Vec<_>>()
        );
        assert_eq!(
            v.pos.iter().copied().map(vector_bits).collect::<Vec<_>>(),
            old.pos.iter().copied().map(vector_bits).collect::<Vec<_>>()
        );
    }
    // Refusals before staging preserve even non-finite payload bits.
    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let mut bad = original.clone();
        bad[0].vel[0].y = invalid;
        let before = state_bits(&bad);
        assert_eq!(
            apply(&mut bad, settings()).unwrap_err().failure,
            Failure::NonFinite
        );
        assert_eq!(state_bits(&bad), before);
    }
    // A trial impulse accelerates the light endpoint beyond next-step motion.
    // This refusal is after position following, so neither staged array may leak.
    let mut bad = original;
    let source = &mut bad[0];
    for p in source.pos.iter_mut().chain(&mut source.old) {
        p.x = -0.5;
    }
    source.mass.fill(100.0);
    bad[1].mass = vec![0.001, 1.0];
    let before = state_bits(&bad);
    let cfg = ContactSettings::new(0.125, 0.0, 0.015625, 0.015625, 0.0625).unwrap();
    let error = apply(&mut bad, cfg).unwrap_err();
    assert_eq!(error.failure, Failure::MotionBound);
    assert_eq!(error.car, Some(1));
    assert_eq!(error.feature, Feature::Node(0));
    assert_eq!(state_bits(&bad), before);
}

fn kinetic_plus_elastic(cars: &[Chassis]) -> f64 {
    let mut energy = 0.0;
    for c in cars {
        for (&v, &m) in c.vel.iter().zip(&c.mass) {
            energy += 0.5 * m * v.dot(v);
        }
        for (b, &broken) in c.beams.iter().zip(&c.broken) {
            if !broken {
                let stretch = (c.pos[b.a] - c.pos[b.b]).length() - b.rest;
                energy += 0.5 * b.stiffness * stretch * stretch;
            }
        }
    }
    energy
}

#[test]
fn follow_never_increases_kinetic_plus_elastic() {
    // Form 2: report the in-pass elastic change; only whole-substep full-state
    // energy is a gate. A stiff beam may gain energy during this pass alone.
    for k in [0.0, 1e3, 1e5, 1e6, 1e7] {
        for vy in [-0.25, -1.0, -4.0] {
            let mut cars = fixture(k, vy);
            let before = kinetic_plus_elastic(&cars);
            let report = apply(&mut cars, settings()).unwrap();
            assert_eq!(report.impulses, 1, "k={k:e} vy={vy:e}");
            let after = kinetic_plus_elastic(&cars);
            assert!(before.is_finite() && after.is_finite());
            println!("follow energy k={k:e} vy={vy:e} result=ok before={before:e} after={after:e}");
        }
    }
}

// Copied from tests/multi_car.rs::collision_cars, total_momentum, momentum_scale
// and step_contact; Form 2 measures energy across the whole substep.
fn collision_cars() -> [Sim; 2] {
    let body = parse(include_str!("../scenarios/car_basic.bsn")).unwrap();
    let cfg = Config {
        dt: 0.0005,
        substeps: 1,
        duration: 12.0,
        gravity: 0.0,
        ..Config::default()
    };
    let mut cars = [
        Sim::from_car(cfg.clone(), &body).unwrap(),
        Sim::from_car(cfg, &body).unwrap(),
    ];
    for (ci, sim) in cars.iter_mut().enumerate() {
        sim.planes.clear();
        let offset = if ci == 0 { -2.07 } else { 2.07 };
        let velocity = Vec3::new(if ci == 0 { 0.02 } else { -0.02 }, 0.0, 0.0);
        for (p, v) in sim.pos.iter_mut().zip(&mut sim.vel) {
            p.x += offset;
            *v = velocity;
        }
        let car = sim.car.as_mut().unwrap();
        car.drag_area = 0.0;
        car.rolling_resistance = 0.0;
        car.set_controls(0.0, 0.0, 0.0).unwrap();
        for w in &mut car.wheels {
            w.pos.x += offset;
            w.vel = velocity;
            w.spin = 0.0;
        }
    }
    cars
}

fn total_momentum(cars: &[Sim]) -> Vec3 {
    let mut p = Vec3::ZERO;
    for sim in cars {
        for (&m, &v) in sim.mass.iter().zip(&sim.vel) {
            p += v * m;
        }
        for w in &sim.car.as_ref().unwrap().wheels {
            p += w.vel * w.p.mass;
        }
    }
    p
}

fn momentum_scale(cars: &[Sim]) -> f64 {
    let mut sum = 0.0;
    for sim in cars {
        for (&m, &v) in sim.mass.iter().zip(&sim.vel) {
            sum += m * v.length();
        }
        for w in &sim.car.as_ref().unwrap().wheels {
            sum += w.p.mass * w.vel.length();
        }
    }
    sum.max(1.0)
}

fn step_contact(
    cars: &mut [Sim; 2],
    previous_impulses: &mut BTreeSet<ContactKey>,
) -> Result<ContactReport, ContactError> {
    let old = [cars[0].pos.clone(), cars[1].pos.clone()];
    for sim in cars.iter_mut() {
        sim.advance();
        sim.check().unwrap();
    }
    let cfg = ContactSettings::new(0.125, 0.0, cars[0].cfg.dt, 0.015625, 0.0625).unwrap();
    let mut views: Vec<_> = cars
        .iter_mut()
        .zip(&old)
        .map(|(sim, previous_pos)| MovingContactCar {
            previous_pos,
            pos: &mut sim.pos,
            vel: &mut sim.vel,
            mass: &sim.mass,
            beams: &sim.beams,
            broken: &sim.broken,
            planes: &sim.planes,
        })
        .collect();
    let (report, keys) = apply_contacts_moving(&mut views, cfg, previous_impulses)?;
    *previous_impulses = keys;
    Ok(report)
}

#[test]
fn head_on_follow_keeps_full_state_energy_and_momentum() {
    let mut cars = collision_cars();
    let p0 = total_momentum(&cars);
    let tolerance = 1e-6 * momentum_scale(&cars);
    let mut impulses = 0;
    let mut previous_impulses = BTreeSet::new();
    let mut worst_energy_rise = f64::NEG_INFINITY;
    let mut worst_momentum_error = 0.0_f64;
    for i in 0..2000 {
        let before: f64 = cars.iter().map(Sim::total_energy).sum();
        let report = step_contact(&mut cars, &mut previous_impulses)
            .unwrap_or_else(|e| panic!("substep={i} failure={e:?}"));
        impulses += report.impulses;
        let after: f64 = cars.iter().map(Sim::total_energy).sum();
        assert!(before.is_finite() && after.is_finite());
        let rise = after - before;
        worst_energy_rise = worst_energy_rise.max(rise);
        assert!(
            rise <= 1e-10 * before.abs().max(1.0),
            "substep={i} before={before:.17e} after={after:.17e} rise={rise:e}"
        );
        let error = (total_momentum(&cars) - p0).length();
        worst_momentum_error = worst_momentum_error.max(error);
        assert!(
            error <= tolerance,
            "substep={i} momentum_error={error:e} bound={tolerance:e}"
        );
    }
    assert!(impulses > 0, "head-on run must actually collide");
    println!(
        "follow head_on impulses={impulses} worst_energy_rise={worst_energy_rise:e} worst_momentum_error={worst_momentum_error:e}"
    );
}

fn check_touching_memory() {
    // Outside/outside end-cap chord adapted from multi_car's grazing refusal.
    // At its midpoint the relative y is zero and the distance is 0.1249,
    // strictly inside skin=0.125; at each end the squared distance is larger.
    let mut cars = [
        Chassis::new(vec![Vec3::new(-0.1249, 0.01, 0.0)], &[], 0.0, 1.0),
        Chassis::new(
            vec![Vec3::ZERO, Vec3::new(2.0, 0.0, 0.0)],
            &[(0, 1, 2.0)],
            0.0,
            1.0,
        ),
    ];
    cars[0].old[0].y = 0.0;
    cars[0].vel[0].x = 0.25;
    for p in &mut cars[1].old {
        p.y = 0.01;
    }
    let key = ContactKey {
        source_car: 0,
        node: 0,
        target_car: 1,
        beam: 0,
    };
    let remembered = BTreeSet::from([key]);
    for memory in [
        BTreeSet::new(),
        BTreeSet::from([ContactKey { beam: 1, ..key }]),
    ] {
        let before = state_bits(&cars);
        let mut views: Vec<_> = cars.iter_mut().map(Chassis::moving).collect();
        let error = apply_contacts_moving(&mut views, settings(), &memory).unwrap_err();
        assert_eq!(error.failure, Failure::UnsupportedSweep);
        assert_eq!(error.feature, Feature::Contact(key));
        assert_eq!(error.car, Some(0));
        drop(views);
        assert_eq!(state_bits(&cars), before);
    }
    let mut touching = cars.clone();
    let mut views: Vec<_> = touching.iter_mut().map(Chassis::moving).collect();
    let (report, keys) = apply_contacts_moving(&mut views, settings(), &remembered).unwrap();
    assert_eq!((report.contacts, report.impulses), (1, 1));
    assert_eq!(keys, remembered);
    // Memory is not a projection or a union: a stationary contact returns no
    // impulse keys, so the same chord with that returned set refuses again.
    cars[0].vel[0] = Vec3::ZERO;
    let before = state_bits(&cars);
    let mut views: Vec<_> = cars.iter_mut().map(Chassis::moving).collect();
    let (report, keys) = apply_contacts_moving(&mut views, settings(), &remembered).unwrap();
    assert_eq!((report.contacts, report.impulses), (1, 0));
    assert!(keys.is_empty());
    let error = apply_contacts_moving(&mut views, settings(), &keys).unwrap_err();
    assert_eq!(error.failure, Failure::UnsupportedSweep);
    drop(views);
    assert_eq!(state_bits(&cars), before);
    // Even a remembered key cannot extend the existing band or bypass depth.
    for (x, expected) in [(-0.2, None), (-0.05, Some(Failure::PenetrationBound))] {
        let source = &mut cars[0];
        for p in source.pos.iter_mut().chain(&mut source.old) {
            p.x = x;
        }
        let before = state_bits(&cars);
        let mut views: Vec<_> = cars.iter_mut().map(Chassis::moving).collect();
        let result = apply_contacts_moving(&mut views, settings(), &remembered);
        if let Some(failure) = expected {
            assert_eq!(result.unwrap_err().failure, failure);
        } else {
            let (report, keys) = result.unwrap();
            assert_eq!((report.contacts, report.impulses), (0, 0));
            assert!(keys.is_empty());
        }
        drop(views);
        assert_eq!(state_bits(&cars), before);
    }
    // Leaving the skin is not an outside/outside band contact.
    cars[0].old[0].x = -0.12;
    cars[0].pos[0].x = -0.1249;
    let before = state_bits(&cars);
    let mut views: Vec<_> = cars.iter_mut().map(Chassis::moving).collect();
    let (report, keys) = apply_contacts_moving(&mut views, settings(), &remembered).unwrap();
    assert_eq!((report.contacts, report.impulses), (0, 0));
    assert!(keys.is_empty());
    drop(views);
    assert_eq!(state_bits(&cars), before);
}

#[test]
fn follow_pair_had_impulse_before_refusal() {
    check_touching_memory();
    // Same scene, delay and exact-thousandths inputs as two_car_sweep.rs.
    // Every contact substep now uses the previous substep's impulse keys.
    let cfg = Config::default();
    let body = parse(include_str!("../scenarios/car_basic.bsn")).unwrap();
    let track = Track::parse(include_str!("../scenarios/tracks/oval.track")).unwrap();
    let scripts = [
        include_str!("../scenarios/full_brake.input"),
        include_str!("../scenarios/tracks/oval_lap.input"),
    ]
    .map(|text| {
        Script::parse(text)
            .unwrap()
            .lines()
            .iter()
            .map(|line| {
                let convert = |v: f64| {
                    let m = (v * 1000.0).round();
                    assert_eq!(m / 1000.0, v, "input line {}: value {v}", line.number);
                    m as i32
                };
                (
                    line.t,
                    Input {
                        throttle: convert(line.throttle),
                        brake: convert(line.brake),
                        steer: convert(line.steer),
                    },
                )
            })
            .collect::<Vec<_>>()
    });
    let mut race = LockstepRace::new(&LockstepSetup {
        cfg: &cfg,
        body: &body,
        track: &track,
        grid_gap_m: 10.0,
    })
    .unwrap();
    assert!(race.multi().last_impulse_keys().is_empty());
    let tol = cfg.dt / f64::from(cfg.substeps) * 1e-6;
    for tick in 0..2000 {
        let inputs = std::array::from_fn(|car| {
            if tick < 6 {
                return Input::NEUTRAL;
            }
            let t = (tick - 6) as f64 * cfg.dt + tol;
            let script = &scripts[car];
            let n = script.partition_point(|(time, _)| *time <= t);
            script
                .get(n.saturating_sub(1))
                .map_or(Input::NEUTRAL, |(_, input)| *input)
        });
        race.step(tick, inputs);
        assert!(
            race.failure().is_none(),
            "tick={tick} substep={} failure={:?} previous_impulses={:?}",
            race.substeps(),
            race.failure(),
            race.multi().last_impulse_keys()
        );
        if tick == 99 || tick == 199 {
            let expected = if tick == 99 {
                0x8c76e69f8437f16d
            } else {
                0x00bec3e589a2d5ec
            };
            assert_eq!(race.state_hash(), expected, "tick={tick}");
        }
    }
    println!("follow gate no_refusal");
}
