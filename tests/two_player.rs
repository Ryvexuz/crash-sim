use crash_sim::car_contact::ContactSettings;
use crash_sim::multi_car::{CarControls, CarPlacement, MultiSim};
use crash_sim::race;
use crash_sim::race_hash::{multi_state_hash, state_hash, RaceHashError};
use crash_sim::settle::{self, CarSetup};
use crash_sim::statehash::Hasher;
use crash_sim::track::Track;
use crash_sim::vec3::Vec3;
use crash_sim::{Body, Config, Sim};
use std::collections::HashSet;

fn body() -> Body {
    crash_sim::format::parse(include_str!("../scenarios/car_basic.bsn")).unwrap()
}

fn track() -> Track {
    Track::parse(include_str!("../scenarios/tracks/oval.track")).unwrap()
}

fn placed_cars() -> (Sim, Sim) {
    let track = track();
    let make = |slot| {
        let placed =
            race::place_on_grid(&body(), &track, race::slot_distance(&track, slot, 10.0)).unwrap();
        settle::car_sim(Config::default(), &placed, &CarSetup::default()).unwrap()
    };
    (make(1), make(2))
}

fn two_cars() -> (Sim, Sim) {
    let (mut a, mut b) = placed_cars();
    a.car.as_mut().unwrap().set_controls(0.5, 0.0, 0.3).unwrap();
    b.car
        .as_mut()
        .unwrap()
        .set_controls(0.4, 0.2, -0.2)
        .unwrap();
    for _ in 0..20 {
        a.advance();
        b.advance();
    }
    (a, b)
}

fn flip(x: &mut f64) {
    *x = f64::from_bits(x.to_bits() ^ 1);
}

#[test]
fn state_hash_covers_every_state_field() {
    let (a, b) = two_cars();
    let base = state_hash(599, &[&a, &b]).unwrap();
    let (a, b) = two_cars();
    assert_eq!(state_hash(599, &[&a, &b]).unwrap(), base);
    let mut seen = HashSet::new();
    seen.insert(base);
    for car_index in 0..2 {
        for case in 0..22 {
            let (mut a, mut b) = two_cars();
            let sim = if car_index == 0 { &mut a } else { &mut b };
            let last_node = sim.pos.len() - 1;
            let last_beam = sim.beams.len() - 1;
            let car = sim.car.as_mut().unwrap();
            let (field, name) = match case {
                0 => (&mut sim.pos[0].x, "node 0 pos.x"),
                1 => (&mut sim.pos[last_node].z, "last node pos.z"),
                2 => (&mut sim.vel[0].y, "node 0 vel.y"),
                3 => (&mut sim.vel[last_node].x, "last node vel.x"),
                4 => (&mut car.wheels[0].pos.y, "wheel 0 pos.y"),
                5 => (&mut car.wheels[3].vel.z, "wheel 3 vel.z"),
                6 => (&mut car.wheels[0].spin, "wheel 0 spin"),
                7 => (&mut car.wheels[3].spin, "wheel 3 spin"),
                8 => (&mut car.wheels[0].deflection[0].x, "wheel 0 deflection"),
                9 => (&mut car.wheels[0].load[0], "wheel 0 load"),
                10 => (&mut car.wheels[2].length, "wheel 2 length"),
                11 => (&mut car.throttle, "throttle"),
                12 => (&mut car.brake, "brake"),
                13 => (&mut car.steer, "steer"),
                14 => (&mut car.frame.up.y, "frame up"),
                15 => (&mut car.frame.forward.x, "frame forward"),
                16 => (&mut car.frame.axle.z, "frame axle"),
                17 => (&mut car.carrier_torque.x, "carrier torque"),
                18 => (&mut car.drive_work, "drive work"),
                19 => (&mut sim.beams[0].rest, "beam 0 rest"),
                20 => (&mut sim.beams[last_beam].rest, "last beam rest"),
                21 => {
                    sim.broken[0] = !sim.broken[0];
                    // No float changes in this case.
                    (&mut sim.beams[0].rest, "broken")
                }
                _ => unreachable!(),
            };
            if case != 21 {
                flip(field);
            }
            let hash = state_hash(599, &[&a, &b]).unwrap();
            assert_ne!(hash, base, "car {} {name}", car_index + 1);
            assert!(seen.insert(hash), "duplicate: car {} {name}", car_index + 1);
        }
    }
    let (mut a, b) = two_cars();
    for (name, hash) in [
        ("tick", state_hash(600, &[&a, &b]).unwrap()),
        ("swapped", state_hash(599, &[&b, &a]).unwrap()),
    ] {
        assert_ne!(hash, base, "{name}");
        assert!(seen.insert(hash), "{name}");
    }
    a.beams.last_mut().unwrap().rest *= 0.99;
    let hash = state_hash(599, &[&a, &b]).unwrap();
    assert_ne!(hash, base, "yield");
    assert!(seen.insert(hash), "yield");
}

#[test]
fn state_hash_order_is_documented() {
    let (a, b) = two_cars();
    let mut h = Hasher::new();
    h.write_u64(599);
    h.write_u64(2);
    for sim in [&a, &b] {
        let car = sim.car.as_ref().unwrap();
        h.write_u64(sim.pos.len() as u64);
        h.write_u64(car.wheels.len() as u64);
        h.write_u64(sim.beams.len() as u64);
        for i in 0..sim.pos.len() {
            let p = sim.pos[i];
            let v = sim.vel[i];
            for x in [p.x, p.y, p.z, v.x, v.y, v.z] {
                h.write_f64(x).unwrap();
            }
        }
        for w in &car.wheels {
            for x in [w.pos.x, w.pos.y, w.pos.z, w.vel.x, w.vel.y, w.vel.z, w.spin] {
                h.write_f64(x).unwrap();
            }
            for d in &w.deflection {
                h.write_f64(d.x).unwrap();
                h.write_f64(d.y).unwrap();
                h.write_f64(d.z).unwrap();
            }
            for &load in &w.load {
                h.write_f64(load).unwrap();
            }
            h.write_f64(w.length).unwrap();
        }
        h.write_f64(car.throttle).unwrap();
        h.write_f64(car.brake).unwrap();
        h.write_f64(car.steer).unwrap();
        for v in [
            car.frame.up,
            car.frame.forward,
            car.frame.axle,
            car.carrier_torque,
        ] {
            h.write_f64(v.x).unwrap();
            h.write_f64(v.y).unwrap();
            h.write_f64(v.z).unwrap();
        }
        h.write_f64(car.drive_work).unwrap();
        h.write_f64(sim.gravity_tilt()).unwrap();
        h.write_u64(u64::from(car.frame_error.is_some()));
        for i in 0..sim.beams.len() {
            h.write_f64(sim.beams[i].rest).unwrap();
            h.write_u64(u64::from(sim.broken[i]));
        }
    }
    assert_eq!(h.finish(), state_hash(599, &[&a, &b]).unwrap());
    let (a, b) = placed_cars();
    let settings = ContactSettings::new(0.125, 0.0, 0.001, 0.015625, 0.0625).unwrap();
    let mut multi = MultiSim::new(
        vec![a, b],
        &[CarPlacement { offset: Vec3::ZERO }; 2],
        settings,
        0.001,
    )
    .unwrap();
    multi
        .advance(
            &[CarControls {
                throttle: 0.0,
                brake: 0.0,
                steer: 0.0,
            }; 2],
        )
        .unwrap();
    let tick = multi.step_index();
    assert_eq!(
        multi_state_hash(tick, &multi).unwrap(),
        state_hash(tick, &[multi.sim(0), multi.sim(1)]).unwrap()
    );
}

#[test]
fn state_hash_refuses_nan_and_bad_shapes() {
    let (a, mut b) = two_cars();
    b.vel[0].x = f64::NAN;
    assert!(matches!(
        state_hash(0, &[&a, &b]),
        Err(RaceHashError::Failed { car: 1, .. })
    ));
    let (mut a, b) = two_cars();
    a.car.as_mut().unwrap().wheels[0].load[0] = f64::NAN;
    assert_eq!(state_hash(0, &[&a, &b]), Err(RaceHashError::Nan { car: 0 }));
    assert_eq!(state_hash(0, &[]), Err(RaceHashError::NoCars));
    let (mut a, b) = two_cars();
    a.broken.pop();
    assert_eq!(
        state_hash(0, &[&a, &b]),
        Err(RaceHashError::Shape { car: 0 })
    );
    let (mut a, _) = two_cars();
    let cube = Sim::new(Config::default());
    assert_eq!(
        state_hash(0, &[&a, &cube]),
        Err(RaceHashError::NoCar { car: 1 })
    );
    a.vel[0].x = 0.0;
    let positive = state_hash(0, &[&a]).unwrap();
    a.vel[0].x = -0.0;
    assert_ne!(positive, state_hash(0, &[&a]).unwrap());
}

#[test]
fn state_hash_same_input_same_hash() {
    let (mut a, mut b) = two_cars();
    let (mut c, mut d) = two_cars();
    let before = state_hash(4999, &[&a, &b]).unwrap();
    for _ in 0..50 {
        a.advance();
        b.advance();
        c.advance();
        d.advance();
    }
    let after = state_hash(4999, &[&a, &b]).unwrap();
    assert_eq!(after, state_hash(4999, &[&c, &d]).unwrap());
    assert_ne!(before, after);
}

#[test]
fn state_hash_refuses_failed_car() {
    let (mut a, b) = two_cars();
    a.pos[0].x = f64::INFINITY;
    assert!(matches!(
        state_hash(0, &[&a, &b]),
        Err(RaceHashError::Failed { car: 0, reason }) if !reason.is_empty()
    ));
}
