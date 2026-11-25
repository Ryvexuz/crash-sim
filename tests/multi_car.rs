//! Contact-kernel checks only. Ordered race stepping/replay remains n20-19b.

use crash_sim::car_contact::{
    apply_contacts, detect_contacts, ContactCar, ContactError, ContactKey, ContactReport,
    ContactSettings, Failure, Feature,
};
use crash_sim::format::parse;
use crash_sim::vec3::Vec3;
use crash_sim::{Beam, Config, Sim};
use std::collections::BTreeSet;

#[derive(Clone)]
struct Chassis {
    old: Vec<Vec3>,
    pos: Vec<Vec3>,
    vel: Vec<Vec3>,
    mass: Vec<f64>,
    beams: Vec<Beam>,
    broken: Vec<bool>,
}

impl Chassis {
    fn new(pos: Vec<Vec3>, edges: &[(usize, usize)]) -> Self {
        let beams = edges
            .iter()
            .map(|&(a, b)| Beam {
                a,
                b,
                rest: (pos[b] - pos[a]).length(),
                stiffness: 100.0,
                damping: 1.0,
            })
            .collect();
        Self {
            old: pos.clone(),
            vel: vec![Vec3::ZERO; pos.len()],
            mass: vec![1.0; pos.len()],
            pos,
            beams,
            broken: vec![false; edges.len()],
        }
    }

    fn view(&mut self) -> ContactCar<'_> {
        ContactCar {
            previous_pos: &self.old,
            pos: &self.pos,
            vel: &mut self.vel,
            mass: &self.mass,
            beams: &self.beams,
            broken: &self.broken,
        }
    }
}

fn settings(restitution: f64) -> ContactSettings {
    ContactSettings::new(0.125, restitution, 0.001, 0.015625, 0.0625).unwrap()
}

fn pair() -> Vec<Chassis> {
    let mut cars = vec![
        Chassis::new(vec![Vec3::new(0.5, 0.125, 0.0)], &[]),
        Chassis::new(vec![Vec3::ZERO, Vec3::new(2.0, 0.0, 0.0)], &[(0, 1)]),
    ];
    cars[0].mass[0] = 2.0;
    cars[0].vel[0].y = -1.0;
    cars[1].mass = vec![3.0, 5.0];
    cars
}

fn apply(cars: &mut [Chassis], cfg: ContactSettings) -> Result<ContactReport, ContactError> {
    let mut views: Vec<_> = cars.iter_mut().map(Chassis::view).collect();
    apply_contacts(&mut views, cfg)
}

fn keys(cars: &mut [Chassis], cfg: ContactSettings) -> Result<Vec<ContactKey>, ContactError> {
    let views: Vec<_> = cars.iter_mut().map(Chassis::view).collect();
    Ok(detect_contacts(&views, cfg)?
        .into_iter()
        .map(|c| c.key)
        .collect())
}

fn velocity_bits(cars: &[Chassis]) -> Vec<[u64; 3]> {
    cars.iter()
        .flat_map(|c| &c.vel)
        .map(|v| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()])
        .collect()
}

/// Independent exhaustive geometry: endpoint spheres or cross-product distance
/// to an interior line, rather than the kernel's clamped closest-point vector.
fn exhaustive(cars: &[Chassis], skin: f64) -> Vec<ContactKey> {
    let mut hits = Vec::new();
    for (sc, source) in cars.iter().enumerate() {
        for (node, &p) in source.pos.iter().enumerate() {
            for (tc, target) in cars.iter().enumerate() {
                if sc == tc {
                    continue;
                }
                for (bi, b) in target.beams.iter().enumerate() {
                    if target.broken[bi] {
                        continue;
                    }
                    let a = target.pos[b.a];
                    let end = target.pos[b.b];
                    let ab = end - a;
                    let ap = p - a;
                    let bp = p - end;
                    let interior = ap.dot(ab) >= 0.0 && bp.dot(ab) <= 0.0;
                    let hit = if interior {
                        ap.cross(ab).length() / ab.length() <= skin
                    } else {
                        ap.length().min(bp.length()) <= skin
                    };
                    if hit {
                        hits.push(ContactKey {
                            source_car: sc,
                            node,
                            target_car: tc,
                            beam: bi,
                        });
                    }
                }
            }
        }
    }
    hits
}

#[test]
fn candidate_completeness_matches_exhaustive_node_beam_search() {
    // Beam interiors are far from both endpoints; some nodes lie in the query
    // radius but outside the capsule. Exact binary coordinates avoid ambiguity.
    let target = Chassis::new(
        vec![
            Vec3::ZERO,
            Vec3::new(8.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 2.0),
            Vec3::new(8.0, 0.0, 2.0),
        ],
        &[(0, 1), (2, 3)],
    );
    let mut points = Vec::new();
    for ix in -1..=33 {
        for y in [0.09375, 0.125, 0.15625, 0.25] {
            for z in [0.0, 2.0] {
                points.push(Vec3::new(f64::from(ix) / 4.0, y, z));
            }
        }
    }
    for rotation in [false, true] {
        let mut cars = vec![Chassis::new(points.clone(), &[]), target.clone()];
        cars[1].broken[1] = true;
        if rotation {
            // Rigid quarter turn plus offset, applied to both cars.
            for c in &mut cars {
                for p in &mut c.pos {
                    *p = Vec3::new(16.0 - p.y, p.x - 8.0, p.z + 4.0);
                }
                c.old.clone_from(&c.pos);
            }
        }
        let expected = exhaustive(&cars, 0.125);
        assert!(!expected.is_empty(), "fixture has actual interior contacts");
        let actual = keys(&mut cars, settings(0.0)).unwrap();
        println!(
            "quarter_turn={rotation} exhaustive_contacts={} kernel_contacts={}",
            expected.len(),
            actual.len()
        );
        assert_eq!(actual, expected);
        assert!(actual.windows(2).all(|p| p[0] < p[1]));
        assert!(actual.iter().all(|k| k.beam == 0));
        // Exercise the opposite mapping direction, not only the earlier car's
        // nodes against the later car's beams.
        cars.reverse();
        let expected = exhaustive(&cars, 0.125);
        assert_eq!(keys(&mut cars, settings(0.0)).unwrap(), expected);
    }
}

#[test]
fn interiors_endpoints_tangency_and_nearby_noncontacts() {
    let positions = [
        (Vec3::new(1.0, 0.09375, 0.0), true),
        (Vec3::new(-0.09375, 0.0, 0.0), true),
        (Vec3::new(2.09375, 0.0, 0.0), true),
        (Vec3::new(1.0, 0.125, 0.0), true),
        (
            Vec3::new(1.0, f64::from_bits(0.125_f64.to_bits() + 1), 0.0),
            false,
        ),
        (Vec3::new(-0.125, -0.125, 0.0), false),
    ];
    for (p, hit) in positions {
        let mut cars = pair();
        cars[0].pos[0] = p;
        cars[0].old[0] = p;
        assert_eq!(!keys(&mut cars, settings(0.0)).unwrap().is_empty(), hit);
    }
}

#[test]
fn interpolation_impulses_match_exact_rational_reference() {
    let mut cars = pair();
    let initial_positions: Vec<_> = cars.iter().map(|c| c.pos.clone()).collect();
    let report = apply(&mut cars, settings(0.0)).unwrap();
    assert_eq!((report.contacts, report.impulses), (1, 1));
    // Exact rational derivation, independent of kernel output:
    // t=1/4, inverse effective mass=1/2+9/(16*3)+1/(16*5)=7/10.
    // j=10/7; final vy=(-2/7, -5/14, -1/14).
    for (got, numerator, denominator) in [
        (report.normal_impulse, 10_i128, 7_i128),
        (cars[0].vel[0].y, -2, 7),
        (cars[1].vel[0].y, -5, 14),
        (cars[1].vel[1].y, -1, 14),
    ] {
        let reference = numerator as f64 / denominator as f64;
        assert!((got - reference).abs() <= 16.0 * f64::EPSILON * reference.abs().max(1.0));
    }
    for (car, pos) in cars.iter().zip(initial_positions) {
        assert_eq!(
            car.pos, pos,
            "no penetration projection or spring-energy change"
        );
    }
}

#[test]
fn oblique_offset_beam_uses_its_geometric_normal() {
    let mut cars = pair();
    let normal = Vec3::new(-0.8, 0.6, 0.0);
    let offset = Vec3::new(4.0, -2.0, 3.0);
    cars[1].pos = vec![offset, offset + Vec3::new(1.2, 1.6, 0.0)];
    cars[1].old = cars[1].pos.clone();
    cars[0].pos[0] = offset + Vec3::new(0.3, 0.4, 0.0) + normal * 0.09375;
    cars[0].old = cars[0].pos.clone();
    cars[0].vel[0] = -normal;
    assert_eq!(apply(&mut cars, settings(0.0)).unwrap().impulses, 1);
    for (got, reference) in [
        (cars[0].vel[0], normal * (-2.0 / 7.0)),
        (cars[1].vel[0], normal * (-5.0 / 14.0)),
        (cars[1].vel[1], normal * (-1.0 / 14.0)),
    ] {
        assert!((got - reference).length() <= 128.0 * f64::EPSILON);
    }
}

#[test]
fn restitution_endpoints_and_partial_bounce_match_exact_energy() {
    for (restitution, numerator, denominator) in
        [(0.0, 2_i128, 7_i128), (0.5, 13, 28), (1.0, 1, 1)]
    {
        let mut cars = pair();
        assert_eq!(apply(&mut cars, settings(restitution)).unwrap().impulses, 1);
        let kinetic: f64 = cars
            .iter()
            .flat_map(|c| c.vel.iter().zip(&c.mass))
            .map(|(v, m)| 0.5 * m * v.dot(*v))
            .sum();
        // Initial K=1 J; analytic loss=(1-e^2)/(2*(7/10)).
        let reference = numerator as f64 / denominator as f64;
        assert!((kinetic - reference).abs() <= 32.0 * f64::EPSILON);
    }
}

#[test]
fn separating_broken_and_same_car_contacts_never_respond() {
    let mut cars = pair();
    cars[0].vel[0].y = 1.0;
    let before = velocity_bits(&cars);
    let r = apply(&mut cars, settings(1.0)).unwrap();
    assert_eq!((r.contacts, r.impulses), (1, 0));
    assert_eq!(velocity_bits(&cars), before);
    cars[1].broken[0] = true;
    assert_eq!(apply(&mut cars, settings(1.0)).unwrap().contacts, 0);
    // A node exactly on its own beam is an undefined inter-car normal, but
    // self-contact must not even reach narrow phase.
    let mut own = Chassis::new(
        vec![
            Vec3::ZERO,
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        ],
        &[(0, 1)],
    );
    own.vel[2].y = -1.0;
    let mut cars = vec![own];
    let before = velocity_bits(&cars);
    assert_eq!(apply(&mut cars, settings(1.0)).unwrap().contacts, 0);
    assert_eq!(velocity_bits(&cars), before);
}

#[test]
fn invalid_settings_are_named_refusals() {
    let good = [0.125, 0.5, 0.001, 0.015625, 0.0625];
    for (index, bad, expected) in [
        (0, 0.0, Failure::InvalidSkin),
        (0, -1.0, Failure::InvalidSkin),
        (0, f64::from_bits(1), Failure::InvalidSkin),
        (0, f64::MAX, Failure::InvalidSkin),
        (1, -f64::EPSILON, Failure::InvalidRestitution),
        (1, 1.0 + f64::EPSILON, Failure::InvalidRestitution),
        (2, 0.0, Failure::InvalidTimestep),
        (2, -1.0, Failure::InvalidTimestep),
        (2, f64::from_bits(1), Failure::InvalidTimestep),
        (3, 0.0, Failure::InvalidBounds),
        (3, 0.03125, Failure::InvalidBounds),
        (4, 0.0, Failure::InvalidBounds),
        (4, 0.125, Failure::InvalidBounds),
    ] {
        let mut args = good;
        args[index] = bad;
        let e = ContactSettings::new(args[0], args[1], args[2], args[3], args[4]).unwrap_err();
        assert_eq!(e.failure, expected);
        assert_eq!(e.feature, Feature::Settings);
    }
    for index in 0..good.len() {
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut args = good;
            args[index] = bad;
            assert!(ContactSettings::new(args[0], args[1], args[2], args[3], args[4]).is_err());
        }
    }
}

#[test]
fn malformed_state_refuses_without_writing_any_velocity() {
    for case in 0..24 {
        let mut cars = pair();
        let expected = match case {
            0 => {
                let _ = cars[1].vel.pop();
                Failure::LengthMismatch
            }
            1 => {
                let _ = cars[1].mass.pop();
                Failure::LengthMismatch
            }
            2 => {
                let _ = cars[1].old.pop();
                Failure::LengthMismatch
            }
            3 => {
                cars[1].broken.clear();
                Failure::LengthMismatch
            }
            4 => {
                cars[1].beams[0].b = usize::MAX;
                Failure::InvalidIndex
            }
            5 => {
                cars[0].mass[0] = 0.0;
                Failure::InvalidMass
            }
            6 => {
                cars[0].mass[0] = -1.0;
                Failure::InvalidMass
            }
            7 => {
                cars[0].mass[0] = f64::from_bits(1);
                Failure::InvalidMass
            }
            8 => {
                cars[0].pos[0].x = f64::NAN;
                Failure::NonFinite
            }
            9 => {
                cars[0].old[0].z = f64::INFINITY;
                Failure::NonFinite
            }
            10 => {
                cars[0].vel[0].x = f64::NEG_INFINITY;
                Failure::NonFinite
            }
            11 => {
                cars[1].beams[0].rest = f64::NAN;
                Failure::NonFinite
            }
            12 => {
                cars[1].pos[1] = cars[1].pos[0];
                cars[1].old = cars[1].pos.clone();
                Failure::DegenerateBeam
            }
            13 => {
                cars[0].pos[0].y = 0.0;
                cars[0].old = cars[0].pos.clone();
                Failure::UndefinedNormal
            }
            14 => {
                cars[0].pos[0].y = 0.03125;
                cars[0].old = cars[0].pos.clone();
                Failure::PenetrationBound
            }
            15 => {
                cars[0].pos[0].x += 0.03125;
                Failure::MotionBound
            }
            16 => {
                cars[0].vel[0].y = -32.0;
                Failure::MotionBound
            }
            17 => {
                cars[0].pos[0].x = f64::MAX;
                Failure::NumericalRange
            }
            18 => {
                cars[1].beams[0].rest = 0.0;
                Failure::NumericalRange
            }
            19 => {
                cars[1].beams[0].stiffness = -1.0;
                Failure::NumericalRange
            }
            20 => {
                cars[1].beams[0].damping = f64::NAN;
                Failure::NonFinite
            }
            21 => {
                cars[0].mass[0] = f64::MAX;
                Failure::InvalidMass
            }
            22 => {
                cars[1].pos[1].x = 0.01;
                cars[1].old[1] = Vec3::ZERO;
                Failure::DegenerateBeam
            }
            _ => {
                cars[0].pos[0].y = 0.0625;
                cars[0].old[0].y = 0.055;
                Failure::PenetrationBound
            }
        };
        let before = velocity_bits(&cars);
        let e = apply(&mut cars, settings(0.0)).unwrap_err();
        println!("case={case} refusal={e}");
        assert_eq!(e.failure, expected);
        assert!(e.car.is_some(), "state refusals identify a car");
        assert_eq!(velocity_bits(&cars), before);
        // The read-only lower-level entry point must not bypass validation.
        assert_eq!(keys(&mut cars, settings(0.0)).unwrap_err(), e);
    }
}

#[test]
fn excessive_response_speed_rolls_back_trial_impulses() {
    let mut cars = pair();
    cars[0].mass[0] = 100.0;
    let before = velocity_bits(&cars);
    let cfg = ContactSettings::new(0.125, 1.0, 0.001, 0.001, 0.0625).unwrap();
    let e = apply(&mut cars, cfg).unwrap_err();
    assert_eq!(e.failure, Failure::MotionBound);
    assert_eq!(velocity_bits(&cars), before);
}

#[test]
fn grazing_sweep_is_refused_instead_of_successful_zero_contacts() {
    let mut cars = pair();
    // The endpoint sphere is entered only at the midpoint of this short chord.
    cars[0].old[0] = Vec3::new(-0.1249, 0.0, 0.0);
    cars[0].pos[0] = Vec3::new(-0.1249, 0.01, 0.0);
    cars[0].vel[0] = Vec3::ZERO;
    // Split the allowed chord across moving source and beam to stay within
    // each node's skin/8 motion bound.
    // A very short target makes the permitted-motion query expansion
    // necessary: neither final endpoint lies within beam_length + skin.
    cars[1].pos[1].x = 0.125 / 1024.0;
    cars[1].old = cars[1].pos.clone();
    cars[1].beams[0].rest = 0.125 / 1024.0;
    for p in &mut cars[1].old {
        p.y = 0.01;
    }
    let before = velocity_bits(&cars);
    let e = apply(&mut cars, settings(0.0)).unwrap_err();
    assert_eq!(e.failure, Failure::UnsupportedSweep);
    assert_eq!(velocity_bits(&cars), before);
}

#[test]
fn crossing_the_entire_capsule_is_a_motion_refusal() {
    let mut cars = pair();
    cars[0].pos[0].y = -0.125;
    let before = velocity_bits(&cars);
    let e = apply(&mut cars, settings(0.0)).unwrap_err();
    assert_eq!(e.failure, Failure::MotionBound);
    assert_eq!(velocity_bits(&cars), before);
}

#[test]
fn empty_and_single_car_passes_do_not_bypass_validation() {
    assert_eq!(apply(&mut [], settings(0.0)).unwrap(), ContactReport::default());
    let mut one = vec![Chassis::new(vec![Vec3::ZERO], &[])];
    assert_eq!(apply(&mut one, settings(0.0)).unwrap().contacts, 0);
    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        one[0].mass[0] = invalid;
        assert_eq!(
            apply(&mut one, settings(0.0)).unwrap_err().failure,
            Failure::NonFinite
        );
    }
    one[0].mass[0] = 1.0;
    one[0].beams.push(Beam {
        a: 0,
        b: usize::MAX,
        rest: 1.0,
        stiffness: 100.0,
        damping: 1.0,
    });
    one[0].broken.push(true);
    assert_eq!(
        apply(&mut one, settings(0.0)).unwrap_err().failure,
        Failure::InvalidIndex
    );
}

#[test]
fn numeric_scale_and_near_cancellation_match_exact_rationals() {
    // Powers of two preserve the exact geometric fraction 1/4 and rational
    // effective mass. Reference fractions are derived above, not kernel output.
    // These scales exercise very small/large normal squares and mass magnitudes.
    for exponent in [-480, 0, 480] {
        for mass_exponent in [-400, 0, 400] {
            let scale = 2.0_f64.powi(exponent);
            let mass_scale = 2.0_f64.powi(mass_exponent);
            let mut cars = pair();
            for c in &mut cars {
                for p in &mut c.pos {
                    *p = *p * scale;
                }
                c.old.clone_from(&c.pos);
                for b in &mut c.beams {
                    b.rest *= scale;
                }
                for m in &mut c.mass {
                    *m *= mass_scale;
                }
            }
            let cfg = ContactSettings::new(
                0.125 * scale,
                0.0,
                0.001 * scale,
                0.015625 * scale,
                0.0625 * scale,
            )
            .unwrap();
            let r = apply(&mut cars, cfg).unwrap();
            assert_eq!(r.impulses, 1);
            for (got, numerator, denominator) in [
                (cars[0].vel[0].y, -2_i128, 7_i128),
                (cars[1].vel[0].y, -5, 14),
                (cars[1].vel[1].y, -1, 14),
            ] {
                let reference = numerator as f64 / denominator as f64;
                assert!((got - reference).abs() <= 32.0 * f64::EPSILON);
            }
        }
    }
    let mut cars = pair();
    let common = 2.0_f64.powi(40);
    for c in &mut cars {
        for v in &mut c.vel {
            v.y += common;
        }
    }
    let cfg = ContactSettings::new(0.125, 0.0, 2.0_f64.powi(-50), 0.015625, 0.0625).unwrap();
    assert_eq!(apply(&mut cars, cfg).unwrap().impulses, 1);
    for (got, numerator, denominator) in [
        (cars[0].vel[0].y, -2_i128, 7_i128),
        (cars[1].vel[0].y, -5, 14),
        (cars[1].vel[1].y, -1, 14),
    ] {
        let reference = common + numerator as f64 / denominator as f64;
        assert!((got - reference).abs() <= 4.0 * f64::EPSILON * common);
    }
}

#[test]
fn numerical_band_boundaries_and_unrepresentable_energy() {
    // At the skin band's exact endpoints, use a short beam so coordinates also
    // satisfy the independent coordinate-product bound. t=1/2 gives
    // inverse effective mass=19/30 and final vy=(-4/19, -5/19, -3/19).
    for skin in [f64::MIN_POSITIVE.sqrt() * 64.0, f64::MAX.sqrt() / 64.0] {
        for mass_scale in [f64::MIN_POSITIVE, 1.0, f64::MAX / 64.0] {
            let mut cars = pair();
            cars[0].pos[0] = Vec3::new(0.0, skin * 0.75, 0.0);
            cars[0].old = cars[0].pos.clone();
            cars[1].pos = vec![
                Vec3::new(-skin / 4.0, 0.0, 0.0),
                Vec3::new(skin / 4.0, 0.0, 0.0),
            ];
            cars[1].old = cars[1].pos.clone();
            cars[1].beams[0].rest = skin / 2.0;
            for car in &mut cars {
                for mass in &mut car.mass {
                    *mass *= mass_scale;
                }
            }
            let cfg =
                ContactSettings::new(skin, 0.0, skin / 128.0, skin / 8.0, skin / 2.0).unwrap();
            assert_eq!(apply(&mut cars, cfg).unwrap().impulses, 1);
            for (got, numerator) in [
                (cars[0].vel[0].y, -4_i128),
                (cars[1].vel[0].y, -5),
                (cars[1].vel[1].y, -3),
            ] {
                let reference = numerator as f64 / 19.0;
                assert!((got - reference).abs() <= 64.0 * f64::EPSILON);
            }
        }
    }
    let mut cars = pair();
    cars[0].mass[0] = 1e300;
    cars[0].vel[0].y = -1e6;
    let before = velocity_bits(&cars);
    let cfg = ContactSettings::new(0.125, 0.0, 1e-9, 0.015625, 0.0625).unwrap();
    let e = apply(&mut cars, cfg).unwrap_err();
    assert_eq!(e.failure, Failure::NumericalRange);
    assert_eq!(e.car, Some(0));
    assert_eq!(e.feature, Feature::Node(0));
    assert_eq!(velocity_bits(&cars), before);
}

#[test]
fn extreme_mass_ratio_matches_integer_rational_reference() {
    let mut cars = pair();
    let heavy = 1_i128 << 100;
    cars[0].mass[0] = heavy as f64;
    cars[1].mass = vec![1.0, 1.0];
    // Exact effective inverse mass = (5*2^97+1)/2^100.
    let denominator = 5 * (1_i128 << 97) + 1;
    let report = apply(&mut cars, settings(0.0)).unwrap();
    for (got, numerator) in [
        (report.normal_impulse, heavy),
        (cars[0].vel[0].y, 1 - denominator),
        (cars[1].vel[0].y, -3 * (1_i128 << 98)),
        (cars[1].vel[1].y, -(1_i128 << 98)),
    ] {
        let reference = numerator as f64 / denominator as f64;
        assert!((got - reference).abs() <= 32.0 * f64::EPSILON * reference.abs().max(1.0));
    }
}

fn sim_view<'a>(sim: &'a mut Sim, old: &'a [Vec3]) -> crash_sim::car_contact::MovingContactCar<'a> {
    crash_sim::car_contact::MovingContactCar {
        previous_pos: old,
        pos: &mut sim.pos,
        vel: &mut sim.vel,
        mass: &sim.mass,
        beams: &sim.beams,
        broken: &sim.broken,
    }
}

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
        // Free flight: no gravity, walls, tyres, aero, rolling resistance,
        // throttle or brake. Internal suspension/carrier forces remain real.
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

fn total_energy(cars: &[Sim]) -> f64 {
    cars.iter().map(Sim::total_energy).sum()
}

fn step_contact(cars: &mut [Sim; 2], memory: &mut BTreeSet<ContactKey>) -> ContactReport {
    let old = [cars[0].pos.clone(), cars[1].pos.clone()];
    for sim in cars.iter_mut() {
        sim.advance(); // exactly one existing single-car physics substep
        sim.check().unwrap();
    }
    let kinetic = |cars: &[Sim]| {
        cars.iter()
            .flat_map(|s| s.vel.iter().zip(&s.mass))
            .map(|(v, m)| 0.5 * m * v.dot(*v))
            .sum::<f64>()
    };
    let before = kinetic(cars);
    let cfg = ContactSettings::new(0.125, 0.0, cars[0].cfg.dt, 0.015625, 0.0625).unwrap();
    let mut views: Vec<_> = cars
        .iter_mut()
        .zip(&old)
        .map(|(s, p)| sim_view(s, p))
        .collect();
    let (r, keys) = crash_sim::car_contact::apply_contacts_moving(&mut views, cfg, memory).unwrap();
    *memory = keys;
    drop(views);
    let after = kinetic(cars);
    assert!(before.is_finite() && after.is_finite());
    assert!(
        after - before <= 1e-10 * before.abs().max(1.0),
        "contact energy before_j={before:.17e} after_j={after:.17e}"
    );
    r
}

#[test]
fn head_on_at_low_speed_conserves_momentum() {
    let mut cars = collision_cars();
    let e0 = total_energy(&cars);
    let mut memory = BTreeSet::new();
    let p0 = total_momentum(&cars);
    let tolerance = 1e-6 * momentum_scale(&cars);
    let initial_velocities = [cars[0].vel.clone(), cars[1].vel.clone()];
    let mut impulses = 0;
    let mut impulse_sum = 0.0;
    let mut worst_error = 0.0_f64;
    // Contact begins after closing the skin gap, not in the initial state.
    for _ in 0..2000 {
        let report = step_contact(&mut cars, &mut memory);
        impulses += report.impulses;
        impulse_sum += report.normal_impulse;
        let error = (total_momentum(&cars) - p0).length();
        worst_error = worst_error.max(error);
        assert!(
            error <= tolerance,
            "momentum_error={error:e} bound={tolerance:e}"
        );
    }
    println!(
        "impulses={impulses} impulse_sum_kg_m_s={impulse_sum:e} momentum_error={worst_error:e} bound={tolerance:e}"
    );
    assert!(impulses > 0 && impulse_sum > 0.0, "must actually collide");
    for (sim, initial) in cars.iter().zip(initial_velocities) {
        assert_ne!(sim.vel, initial, "both cars must respond");
    }
    let final_j = total_energy(&cars);
    println!("head_on initial_j={e0:e} final_j={final_j:e}");
    assert!(final_j <= e0 + 1e-10 * e0.abs().max(1.0));
}

#[test]
fn car_contact_never_creates_energy() {
    // Immediate hand-computable collision makes this check non-vacuous even if
    // a full-run trajectory is changed. Reversing the impulse must fail here.
    let mut simple = pair();
    assert_eq!(apply(&mut simple, settings(0.0)).unwrap().impulses, 1);
    let final_kinetic: f64 = simple
        .iter()
        .flat_map(|c| c.vel.iter().zip(&c.mass))
        .map(|(v, m)| 0.5 * m * v.dot(*v))
        .sum();
    assert!((final_kinetic - 2.0 / 7.0).abs() <= 32.0 * f64::EPSILON);
    let mut cars = collision_cars();
    let mut memory = BTreeSet::new();
    let e0 = total_energy(&cars);
    assert!(e0.is_finite() && e0 > 0.0);
    let mut gain = 0.0_f64;
    let mut min_after = f64::INFINITY;
    let mut rise = 0.0_f64;
    let mut impulses = 0;
    for step in 1..=24000 {
        impulses += step_contact(&mut cars, &mut memory).impulses;
        let energy = total_energy(&cars);
        assert!(energy.is_finite());
        gain = gain.max(energy - e0);
        if step >= 16000 {
            min_after = min_after.min(energy);
            rise = rise.max(energy - min_after);
        }
        assert!(gain <= 0.01 * e0, "energy_gain_j={gain:e} initial_j={e0:e}");
        assert!(rise <= 0.01, "energy_rise_after_8s_j={rise:e}");
    }
    println!("initial_j={e0:e} gain_j={gain:e} rise_after_8s_j={rise:e} impulses={impulses}");
    assert!(impulses > 0, "the complete 12 s run must actually collide");
    assert!(min_after.is_finite(), "the post-8 s window must be populated");
    assert!(cars.iter().all(|s| s.car.as_ref().unwrap().drive_work == 0.0));
    let final_j = total_energy(&cars);
    println!("car_contact initial_j={e0:e} final_j={final_j:e}");
    assert!(final_j <= e0 + 1e-10 * e0.abs().max(1.0));
}

use crash_sim::multi_car::{CarControls, CarPlacement, MultiError, MultiSim};

const ZERO_CONTROLS: CarControls = CarControls {
    throttle: 0.0,
    brake: 0.0,
    steer: 0.0,
};
const ZERO_PLACEMENT: CarPlacement = CarPlacement { offset: Vec3::ZERO };

fn x_placement(x: f64) -> CarPlacement {
    CarPlacement {
        offset: Vec3::new(x, 0.0, 0.0),
    }
}

fn road_car() -> Sim {
    let body = parse(include_str!("../scenarios/car_basic.bsn")).unwrap();
    Sim::from_car(
        Config {
            dt: 0.02,
            // h=0.001 s stays below this body's stability limit.
            substeps: 20,
            duration: 6.0,
            ..Config::default()
        },
        &body,
    )
    .unwrap()
}

fn contact_settings(h: f64) -> ContactSettings {
    ContactSettings::new(0.125, 0.0, h, 0.015625, 0.0625).unwrap()
}

// Independent writer: scalar lists, not the production vector writer.
fn car_words(sim: &Sim) -> Vec<u64> {
    let wheels = &sim.car.as_ref().unwrap().wheels;
    let mut words = vec![
        sim.pos.len() as u64,
        wheels.len() as u64,
        sim.beams.len() as u64,
    ];
    for i in 0..sim.pos.len() {
        words.extend(
            [
                sim.pos[i].x,
                sim.pos[i].y,
                sim.pos[i].z,
                sim.vel[i].x,
                sim.vel[i].y,
                sim.vel[i].z,
            ]
            .map(f64::to_bits),
        );
    }
    for w in wheels {
        words.extend(
            [w.pos.x, w.pos.y, w.pos.z, w.vel.x, w.vel.y, w.vel.z, w.spin].map(f64::to_bits),
        );
        for d in &w.deflection {
            words.extend([d.x, d.y, d.z].map(f64::to_bits));
        }
        for &load in &w.load {
            words.push(load.to_bits());
        }
        words.push(w.length.to_bits());
    }
    for i in 0..sim.beams.len() {
        words.push(sim.beams[i].rest.to_bits());
        words.push(if sim.broken[i] { 1 } else { 0 });
    }
    words
}

fn assert_same_car(actual: &Sim, expected: &Sim, step: u64) {
    assert_eq!(car_words(actual), car_words(expected), "step={step}");
    assert_eq!(
        actual.first_contact_s.map(f64::to_bits),
        expected.first_contact_s.map(f64::to_bits),
        "first contact at step={step}"
    );
    let time = (step as f64 * expected.cfg.dt).to_bits();
    assert_eq!(actual.sample().t.to_bits(), time);
    assert_eq!(expected.sample().t.to_bits(), time);
}

fn alone_step(sim: &mut Sim, c: CarControls) {
    sim.car
        .as_mut()
        .unwrap()
        .set_controls(c.throttle, c.brake, c.steer)
        .unwrap();
    sim.advance();
    sim.check().unwrap();
}

#[test]
fn one_car_matches_the_single_car_run_byte_for_byte() {
    let mut plain = road_car();
    let mut other = road_car();
    // Explicit signed zero makes the zero-placement constructor check non-vacuous.
    plain.pos[16].x = -0.0;
    other.pos[16].x = -0.0;
    let mut multi = MultiSim::new(
        vec![other],
        &[ZERO_PLACEMENT],
        contact_settings(0.001),
        0.001,
    )
    .unwrap();
    assert_same_car(multi.sim(0), &plain, 0);
    let mut initial_words = vec![1, 0];
    initial_words.extend(car_words(&plain));
    assert_eq!(multi.state_words(), initial_words);
    for step in 1..=300 {
        let c = CarControls {
            throttle: if step <= 100 {
                step as f64 / 100.0
            } else if step <= 220 {
                0.5
            } else {
                0.0
            },
            brake: if step > 220 { 0.75 } else { 0.0 },
            steer: if (120..=200).contains(&step) {
                0.125
            } else {
                0.0
            },
        };
        alone_step(&mut plain, c);
        let report = multi.advance(&[c]).unwrap();
        assert_eq!(report.substeps, 20);
        assert_eq!(
            (report.candidates, report.contacts, report.impulses),
            (0, 0, 0)
        );
        assert_eq!(multi.step_index(), step);
        assert_same_car(multi.sim(0), &plain, step);
    }
    let mut expected = vec![1, 300];
    expected.extend(car_words(&plain));
    assert_eq!(multi.state_words(), expected);

    // A road car's tyres can keep every chassis node off the ground forever.
    // Use a tangent front wall in a short free-flight variant to guarantee the
    // first_contact_s update is exercised, not merely None == None.
    let [mut plain, _] = collision_cars();
    let [mut other, _] = collision_cars();
    let wall = crash_sim::collide::Plane::new(plain.pos[32], Vec3::new(-1.0, 0.0, 0.0)).unwrap();
    plain.planes.push(wall);
    other.planes.push(wall);
    let mut multi = MultiSim::new(
        vec![other],
        &[ZERO_PLACEMENT],
        contact_settings(0.0005),
        0.0005,
    )
    .unwrap();
    assert!(plain.first_contact_s.is_none());
    alone_step(&mut plain, ZERO_CONTROLS);
    multi.advance(&[ZERO_CONTROLS]).unwrap();
    assert_eq!(
        plain.first_contact_s.map(f64::to_bits),
        Some(0.0005_f64.to_bits())
    );
    assert_same_car(multi.sim(0), &plain, 1);
}

#[test]
fn two_cars_driving_apart_never_interact() {
    let offset = Vec3::new(-12.0, 0.0, 0.0);
    let mut alone = [road_car(), road_car()];
    for p in &mut alone[1].pos {
        *p += offset;
    }
    for w in &mut alone[1].car.as_mut().unwrap().wheels {
        w.pos += offset;
    }
    // Higher-speed road driving needs a larger caller-selected skin/motion
    // band than the low-speed contact fixture (see PHYSICS.md).
    let settings = ContactSettings::new(1.0, 0.0, 0.001, 0.125, 0.5).unwrap();
    let mut multi = MultiSim::new(
        vec![road_car(), road_car()],
        &[ZERO_PLACEMENT, CarPlacement { offset }],
        settings,
        0.001,
    )
    .unwrap();
    let controls = [
        CarControls {
            throttle: 1.0,
            ..ZERO_CONTROLS
        },
        CarControls {
            brake: 1.0,
            ..ZERO_CONTROLS
        },
    ];
    for step in 1..=300 {
        let r = multi.advance(&controls).unwrap();
        assert_eq!(
            (r.candidates, r.contacts, r.impulses),
            (0, 0, 0),
            "step={step}"
        );
        for (sim, &control) in alone.iter_mut().zip(&controls) {
            alone_step(sim, control);
        }
    }
    // n20-20's ai_inputs_replay_one_car_alone relies on this in-process property.
    for (i, sim) in alone.iter().enumerate() {
        assert_same_car(multi.sim(i), sim, 300);
    }
    assert_eq!(multi.step_index(), 300);
}

fn short_collision_cars() -> [Sim; 2] {
    let mut cars = collision_cars();
    // The original gap is 0.14 m: closing at 0.04 m/s for 400*0.0005 s
    // does not reach skin=0.125 m. Close it by 0.01001 m once, so this bounded
    // fixture actually collides, retaining the original velocities and h.
    // Avoid 0.01 m, which targets tangency exactly at a substep boundary.
    let offset = Vec3::new(-0.01001, 0.0, 0.0);
    for p in &mut cars[1].pos {
        *p += offset;
    }
    for w in &mut cars[1].car.as_mut().unwrap().wheels {
        w.pos += offset;
    }
    cars
}

fn ordered_collision(substeps: u32, reverse: bool) -> MultiSim {
    let mut cars = short_collision_cars();
    for sim in &mut cars {
        sim.cfg.substeps = substeps;
        sim.cfg.dt = 0.0005 * substeps as f64;
    }
    if reverse {
        cars.reverse();
    }
    MultiSim::new(
        Vec::from(cars),
        &[ZERO_PLACEMENT; 2],
        contact_settings(0.0005),
        0.0005,
    )
    .unwrap()
}

#[test]
fn multi_car_same_inputs_same_bytes() {
    let mut first = ordered_collision(1, false);
    let mut second = ordered_collision(1, false);
    let mut reversed = ordered_collision(1, true);
    let mut contacts = [0; 2];
    for step in 1..=400 {
        contacts[0] += first.advance(&[ZERO_CONTROLS; 2]).unwrap().contacts;
        contacts[1] += second.advance(&[ZERO_CONTROLS; 2]).unwrap().contacts;
        reversed.advance(&[ZERO_CONTROLS; 2]).unwrap();
        if step % 50 == 0 || step == 400 {
            assert_eq!(first.state_words(), second.state_words(), "step={step}");
        }
    }
    println!("ordered_collision contacts={contacts:?}");
    assert!(contacts.iter().all(|&n| n > 0));
    assert_ne!(first.state_words(), reversed.state_words());

    let mut a = ordered_collision(2, false);
    let mut b = ordered_collision(2, false);
    let mut reference = short_collision_cars();
    let mut memory = BTreeSet::new();
    let mut variant_contacts = 0;
    // Two h-sized reference output steps are exactly one two-substep output
    // step here: no planes, controls or gravity; yielding is independent of
    // output time and this body has no break limits. The independent reference
    // uses Sim::advance plus the kernel, not MultiSim. Compare car words without
    // step_index, which intentionally differs. Counts prove both passes ran.
    for step in 1..=200 {
        let ra = a.advance(&[ZERO_CONTROLS; 2]).unwrap();
        let rb = b.advance(&[ZERO_CONTROLS; 2]).unwrap();
        let r0 = step_contact(&mut reference, &mut memory);
        let r1 = step_contact(&mut reference, &mut memory);
        assert_eq!((ra.substeps, rb.substeps), (2, 2));
        assert_eq!(ra, rb);
        assert_eq!(ra.candidates, r0.candidates + r1.candidates, "step={step}");
        assert_eq!(ra.contacts, r0.contacts + r1.contacts, "step={step}");
        assert_eq!(ra.impulses, r0.impulses + r1.impulses, "step={step}");
        assert_eq!(a.state_words(), b.state_words());
        for (i, sim) in reference.iter().enumerate() {
            assert_eq!(car_words(a.sim(i)), car_words(sim));
        }
        variant_contacts += ra.contacts;
    }
    assert!(variant_contacts > 0);
    // Full words cannot equal the first run: 200 versus 400 output steps.
    assert_eq!(&a.state_words()[2..], &first.state_words()[2..]);
}

#[test]
fn same_car_overlap_never_responds_through_multi_sim() {
    let [mut own, far] = collision_cars();
    // Put node 2 one quarter along beam 0--8. All beams retain their real
    // rest lengths; this overlap stays inside the existing strain limit.
    // Self-pairs must be skipped even when they would have an undefined normal.
    let beam = own.beams.iter().find(|b| b.a == 0 && b.b == 8).unwrap();
    own.pos[2] = own.pos[beam.a] + (own.pos[beam.b] - own.pos[beam.a]) * 0.25;
    own.check().unwrap();
    let mut multi = MultiSim::new(
        vec![own, far],
        &[ZERO_PLACEMENT, x_placement(50.0)],
        contact_settings(0.0005),
        0.0005,
    )
    .unwrap();
    let r = multi.advance(&[ZERO_CONTROLS; 2]).unwrap();
    assert_eq!((r.candidates, r.contacts, r.impulses), (0, 0, 0));
}

#[test]
fn multi_sim_refuses_bad_construction_and_controls() {
    let build = |cars, placements: &[CarPlacement], h| {
        MultiSim::new(cars, placements, contact_settings(0.001), h)
    };
    assert!(matches!(
        build(vec![], &[], 0.001),
        Err(MultiError::BadCars(_))
    ));
    assert!(matches!(
        build(vec![Sim::new(Config::default())], &[ZERO_PLACEMENT], 0.001),
        Err(MultiError::BadCars(_))
    ));
    for change_dt in [true, false] {
        let mut other = road_car();
        if change_dt {
            other.cfg.dt = 0.01;
        } else {
            other.cfg.substeps = 40;
        }
        assert!(matches!(
            build(vec![road_car(), other], &[ZERO_PLACEMENT; 2], 0.001),
            Err(MultiError::MismatchedConfig { car: 1 })
        ));
    }
    for dt in [0.0, -0.02, f64::NAN, f64::INFINITY, f64::from_bits(1)] {
        let mut car = road_car();
        car.cfg.dt = dt;
        assert!(matches!(
            build(vec![car], &[ZERO_PLACEMENT], 0.001),
            Err(MultiError::BadCars(_))
        ));
    }
    let mut car = road_car();
    car.cfg.substeps = 0;
    assert!(matches!(
        build(vec![car], &[ZERO_PLACEMENT], 0.001),
        Err(MultiError::BadCars(_))
    ));
    let mut car = road_car();
    car.car.as_mut().unwrap().wheels.clear();
    assert!(matches!(
        build(vec![car], &[ZERO_PLACEMENT], 0.001),
        Err(MultiError::BadCars(_))
    ));
    for h in [
        0.0,
        -0.001,
        f64::NAN,
        f64::INFINITY,
        f64::from_bits(1),
        f64::from_bits(0.001_f64.to_bits() + 1),
    ] {
        assert!(matches!(
            build(vec![road_car()], &[ZERO_PLACEMENT], h),
            Err(MultiError::BadCars(_))
        ));
    }
    assert!(matches!(
        build(vec![road_car()], &[], 0.001),
        Err(MultiError::BadCars(_))
    ));
    for offset in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(matches!(
            build(vec![road_car()], &[x_placement(offset)], 0.001),
            Err(MultiError::BadCars(_))
        ));
    }
    // Exact binary addition references include subnormals, the largest finite
    // values, and cancellation of adjacent huge floats. No simulation follows
    // these deliberately nonphysical constructor-only coordinate fixtures.
    for (p, offset, expected) in [
        (-0.0, -0.0, -0.0),
        (f64::from_bits(1), f64::from_bits(1), f64::from_bits(2)),
        (-f64::MAX, f64::MAX, 0.0),
        (
            -2.0_f64.powi(500),
            f64::from_bits(2.0_f64.powi(500).to_bits() + 1),
            2.0_f64.powi(448),
        ),
    ] {
        let mut car = road_car();
        for node in &mut car.pos {
            node.x = p;
        }
        for wheel in &mut car.car.as_mut().unwrap().wheels {
            wheel.pos.x = p;
        }
        let multi = build(vec![car], &[x_placement(offset)], 0.001).unwrap();
        assert_eq!(multi.sim(0).pos[0].x.to_bits(), expected.to_bits());
        assert_eq!(
            multi.sim(0).car.as_ref().unwrap().wheels[0].pos.x.to_bits(),
            expected.to_bits()
        );
    }
    let mut car = road_car();
    car.pos[0].x = f64::MAX;
    assert!(matches!(
        build(vec![car], &[x_placement(f64::MAX)], 0.001),
        Err(MultiError::BadCars(_))
    ));
    let mut multi = build(vec![road_car(), road_car()], &[ZERO_PLACEMENT; 2], 0.001).unwrap();
    let before = multi.state_words();
    assert!(matches!(
        multi.advance(&[]),
        Err(MultiError::ControlCount {
            expected: 2,
            got: 0
        })
    ));
    assert_eq!(multi.state_words(), before);
    for (controls, car) in [
        (
            [
                ZERO_CONTROLS,
                CarControls {
                    throttle: 2.0,
                    ..ZERO_CONTROLS
                },
            ],
            1,
        ),
        (
            [CarControls {
                throttle: 2.0,
                ..ZERO_CONTROLS
            }; 2],
            0,
        ),
        (
            [
                ZERO_CONTROLS,
                CarControls {
                    brake: f64::NAN,
                    ..ZERO_CONTROLS
                },
            ],
            1,
        ),
        (
            [
                ZERO_CONTROLS,
                CarControls {
                    steer: f64::INFINITY,
                    ..ZERO_CONTROLS
                },
            ],
            1,
        ),
    ] {
        let mut multi = build(vec![road_car(), road_car()], &[ZERO_PLACEMENT; 2], 0.001).unwrap();
        assert!(
            matches!(multi.advance(&controls), Err(MultiError::Controls { car: i, .. }) if i == car)
        );
        assert_eq!(multi.state_words(), before, "no car moved on refusal");
    }
    let mut car = road_car();
    car.vel[0].x = f64::NAN;
    let mut multi = build(vec![car], &[ZERO_PLACEMENT], 0.001).unwrap();
    assert!(matches!(
        multi.advance(&[ZERO_CONTROLS]),
        Err(MultiError::Unstable { car: 0, .. })
    ));
    let mut cars = collision_cars();
    for sim in &mut cars {
        for v in &mut sim.vel {
            v.x = 1.0;
        }
        for w in &mut sim.car.as_mut().unwrap().wheels {
            w.vel.x = 1.0;
        }
    }
    let tiny = ContactSettings::new(0.125, 0.0, 0.0005, 2.0_f64.powi(-40), 0.0625).unwrap();
    let mut multi = MultiSim::new(
        Vec::from(cars),
        &[ZERO_PLACEMENT, x_placement(-4.14)],
        tiny,
        0.0005,
    )
    .unwrap();
    let error = multi.advance(&[ZERO_CONTROLS; 2]);
    assert!(matches!(
        error,
        Err(MultiError::Contact(ContactError {
            car: Some(0),
            feature: Feature::Node(0),
            failure: Failure::MotionBound,
        }))
    ));
}

fn car_word_count(sim: &Sim) -> usize {
    3 + 6 * sim.pos.len()
        + sim
            .car
            .as_ref()
            .unwrap()
            .wheels
            .iter()
            .map(|w| 8 + 3 * w.deflection.len() + w.load.len())
            .sum::<usize>()
        + 2 * sim.beams.len()
}

#[test]
fn geometry_and_full_state_access_follow_live_state() {
    let mut multi = MultiSim::new(
        vec![road_car(), road_car()],
        &[ZERO_PLACEMENT, x_placement(-12.0)],
        contact_settings(0.001),
        0.001,
    )
    .unwrap();
    let before: Vec<_> = (0..2)
        .map(|i| multi.chassis_positions(i).to_vec())
        .collect();
    multi.advance(&[ZERO_CONTROLS; 2]).unwrap();
    assert_eq!(multi.car_count(), 2);
    for (i, old) in before.iter().enumerate() {
        let sim = multi.sim(i);
        assert_ne!(multi.chassis_positions(i), old.as_slice());
        assert_eq!(multi.chassis_positions(i), sim.pos.as_slice());
        let mounts = multi.wheel_mount_positions(i);
        let wheels = &sim.car.as_ref().unwrap().wheels;
        assert_eq!(mounts.len(), wheels.len());
        for (positions, wheel) in mounts.iter().zip(wheels) {
            assert_eq!(positions.len(), wheel.p.mount.len());
            for (&p, &node) in positions.iter().zip(&wheel.p.mount) {
                assert_eq!(p, sim.pos[node]);
            }
        }
    }
    let words = multi.state_words();
    assert_eq!(&words[..2], &[2, 1]);
    let split = 2 + car_word_count(multi.sim(0));
    assert_eq!(words.len(), split + car_word_count(multi.sim(1)));
    assert_eq!(&words[2..split], car_words(multi.sim(0)));
    assert_eq!(&words[split..], car_words(multi.sim(1)));
}
