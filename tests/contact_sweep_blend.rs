//! Blended separating planes must not certify sweeps that enter the skin.

use crash_sim::car_contact::{
    apply_contacts, detect_contacts, ContactCar, ContactError, ContactKey, ContactSettings, Failure,
    Feature,
};
use crash_sim::vec3::Vec3;
use crash_sim::Beam;

// Copy of car_contact.rs::closest for these fixed, non-degenerate fixtures.
fn closest(p: Vec3, a: Vec3, b: Vec3) -> f64 {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.dot(ab)).clamp(0.0, 1.0);
    ((p - a) - ab * t).length()
}

fn velocity_bits(cars: &[ContactCar<'_>]) -> Vec<[u64; 3]> {
    cars.iter()
        .flat_map(|car| car.vel.iter())
        .map(|v| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()])
        .collect()
}

fn assert_crossing_refused(old_beam: [Vec3; 2], beam: [Vec3; 2], old_node: Vec3, node: Vec3) {
    for (name, p, a, b) in [
        ("old", old_node, old_beam[0], old_beam[1]),
        ("final", node, beam[0], beam[1]),
    ] {
        let distance = closest(p, a, b);
        eprintln!("crossing {name} distance={distance:.9}");
        assert!(
            distance > 0.25 && distance <= 0.3125,
            "{name} distance={distance:?} must be outside skin within sweep band"
        );
    }
    let midpoint_distance = closest(
        old_node + (node - old_node) * 0.5,
        old_beam[0] + (beam[0] - old_beam[0]) * 0.5,
        old_beam[1] + (beam[1] - old_beam[1]) * 0.5,
    );
    eprintln!("crossing midpoint distance={midpoint_distance:.9}");
    assert!(
        midpoint_distance < 0.25,
        "midpoint distance={midpoint_distance:?} must enter skin"
    );

    let beams = [Beam {
        a: 0,
        b: 1,
        rest: 1.0,
        stiffness: 100.0,
        damping: 1.0,
    }];
    let mut beam_vel = [Vec3::ZERO; 2];
    let mut node_vel = [Vec3::ZERO];
    let old_node = [old_node];
    let node = [node];
    let mut views = [
        ContactCar {
            previous_pos: &old_beam,
            pos: &beam,
            vel: &mut beam_vel,
            mass: &[1.0; 2],
            beams: &beams,
            broken: &[false],
        },
        ContactCar {
            previous_pos: &old_node,
            pos: &node,
            vel: &mut node_vel,
            mass: &[1.0],
            beams: &[],
            broken: &[],
        },
    ];
    let settings = ContactSettings::new(0.25, 0.0, 0.001, 0.25 / 8.0, 0.25 / 2.0).unwrap();
    let expected = ContactError {
        car: Some(1),
        feature: Feature::Contact(ContactKey {
            source_car: 1,
            node: 0,
            target_car: 0,
            beam: 0,
        }),
        failure: Failure::UnsupportedSweep,
    };
    let before = velocity_bits(&views);
    assert_eq!(detect_contacts(&views, settings), Err(expected.clone()));
    assert_eq!(apply_contacts(&mut views, settings), Err(expected));
    assert_eq!(
        velocity_bits(&views),
        before,
        "refused crossing must leave every velocity bit unchanged"
    );
}

#[test]
fn crossing_sweep_past_an_end_cap_is_still_refused() {
    let beam = [Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0)];
    assert_crossing_refused(
        beam,
        beam,
        Vec3::new(-0.2497, -0.015, 0.0),
        Vec3::new(-0.2497, 0.015, 0.0),
    );
}

#[test]
fn crossing_sweep_of_a_moving_beam_is_still_refused() {
    assert_crossing_refused(
        [Vec3::new(0.0, 0.015, 0.0), Vec3::new(1.0, 0.015, 0.0)],
        [Vec3::new(0.0, -0.015, 0.0), Vec3::new(1.0, -0.015, 0.0)],
        Vec3::new(-0.2497, 0.0, 0.0),
        Vec3::new(-0.2497, 0.0, 0.0),
    );
}
