//! Near-rule contracts, with independent replay of every completed substep.

use crash_sim::ai::DriverParams;
use crash_sim::race::near::{ground_distance, min_pair_distance, NearTracker, NEAR_M};
use crash_sim::race::{self, ProfileParams, RaceEnd, RaceResult, RaceSetup};
use crash_sim::settle::{self, CarSetup};
use crash_sim::track::Track;
use crash_sim::vec3::Vec3;
use crash_sim::{Body, Config};

fn body() -> Body {
    crash_sim::format::parse(include_str!("../scenarios/car_basic.bsn")).unwrap()
}

fn track() -> Track {
    Track::parse(include_str!("../scenarios/tracks/oval.track")).unwrap()
}

fn setup<'a>(cfg: &'a Config, body: &'a Body, track: &'a Track) -> RaceSetup<'a> {
    RaceSetup {
        cfg,
        body,
        track,
        cars: 1,
        laps: 1,
        time_limit_s: 70.0,
        grid_gap_m: 10.0,
        driver: DriverParams {
            lookahead_k: 0.6,
            lookahead_min: 3.0,
            lookahead_max: 12.0,
            kp: 0.8,
            ki: 0.3,
            integral_max: 2.0,
        },
        profile: ProfileParams {
            mu: 0.6,
            g: 9.81,
            a_accel: 2.0,
            a_brake: 4.0,
            v_max: 15.0,
        },
    }
}

/// Independent reference: no near-module calls or race statistics as inputs.
fn replay(cfg: &Config, body: &Body, track: &Track, r: &RaceResult, gap: f64) -> (f64, f64, u64) {
    assert_eq!(r.contacts, 0, "isolated replay requires zero contacts");
    assert!(r.cars.len() >= 2);
    assert!(r.substeps > 0);
    let mut sims = Vec::with_capacity(r.cars.len());
    for k in 0..r.cars.len() {
        let placed =
            race::place_on_grid(body, track, race::slot_distance(track, k + 1, gap)).unwrap();
        let mut one = cfg.clone();
        one.dt = r.h;
        one.substeps = 1;
        sims.push(settle::car_sim(one, &placed, &CarSetup::default()).unwrap());
    }
    let mut minimum: Option<(f64, f64)> = None;
    let mut near_substeps = 0;
    for i in 0..r.substeps {
        let mut centres = Vec::with_capacity(sims.len());
        for (k, sim) in sims.iter_mut().enumerate() {
            let c = r.cars[k].inputs[(i / r.control_substeps) as usize];
            sim.car
                .as_mut()
                .unwrap()
                .set_controls(c.throttle, c.brake, c.steer)
                .unwrap();
            sim.advance();
            sim.check().unwrap();
            centres.push(race::car_position(sim));
        }
        let mut step_minimum = None;
        for (a, first) in centres.iter().enumerate() {
            for second in &centres[a + 1..] {
                let dx = first.x - second.x;
                let dz = first.z - second.z;
                let distance = (dx * dx + dz * dz).sqrt();
                if step_minimum.is_none_or(|m| distance < m) {
                    step_minimum = Some(distance);
                }
            }
        }
        let m = step_minimum.unwrap();
        let t1 = (i + 1) as f64 * r.h;
        if minimum.is_none_or(|(distance, _)| m < distance) {
            minimum = Some((m, t1));
        }
        // Independent copy of STAGE7's strict 4.40 m definition.
        if m < 4.40 {
            near_substeps += 1;
        }
    }
    let (distance, time) = minimum.unwrap();
    (distance, time, near_substeps)
}

#[test]
fn near_tracker_rules_by_hand() {
    assert_eq!(NEAR_M.to_bits(), 4.40_f64.to_bits());
    assert_eq!(
        ground_distance(Vec3::new(0.0, 5.0, 0.0), Vec3::new(3.0, -2.0, 4.0)),
        5.0
    );
    // Analytic IEEE-754 cases for the required unscaled formula: squaring
    // the smallest subnormal underflows; extreme finite coordinates overflow.
    assert_eq!(
        ground_distance(Vec3::ZERO, Vec3::new(f64::from_bits(1), 0.0, 0.0)),
        0.0
    );
    assert_eq!(
        ground_distance(
            Vec3::new(f64::MAX, 0.0, 0.0),
            Vec3::new(-f64::MAX, 0.0, 0.0)
        ),
        f64::INFINITY
    );
    // Adjacent values at 1 cancel to exactly 2^-52, independently known.
    assert_eq!(
        ground_distance(
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(1.0 + f64::EPSILON, 0.0, 0.0)
        ),
        f64::EPSILON
    );
    assert_eq!(min_pair_distance(&[]), None);
    assert_eq!(min_pair_distance(&[Vec3::ZERO]), None);
    assert_eq!(
        min_pair_distance(&[
            Vec3::ZERO,
            Vec3::new(10.0, 0.0, 0.0),
            Vec3::new(13.0, 0.0, 0.0),
        ]),
        Some(3.0)
    );
    let mut tracker = NearTracker::new();
    assert_eq!(tracker.stats(), None);
    tracker.observe(&[Vec3::ZERO, Vec3::new(4.40, 0.0, 0.0)], 0.001);
    let stats = tracker.stats().unwrap();
    assert_eq!(stats.near_substeps, 0);
    assert_eq!(stats.gap_min_m.to_bits(), 4.40_f64.to_bits());
    assert_eq!(stats.gap_min_t.to_bits(), 0.001_f64.to_bits());
    tracker.observe(&[Vec3::ZERO, Vec3::new(4.399, 0.0, 0.0)], 0.002);
    let stats = tracker.stats().unwrap();
    assert_eq!(stats.near_substeps, 1);
    assert_eq!(stats.gap_min_m.to_bits(), 4.399_f64.to_bits());
    assert_eq!(stats.gap_min_t.to_bits(), 0.002_f64.to_bits());
    tracker.observe(&[Vec3::ZERO, Vec3::new(4.399, 0.0, 0.0)], 0.003);
    let stats = tracker.stats().unwrap();
    assert_eq!(stats.near_substeps, 2);
    assert_eq!(stats.gap_min_t.to_bits(), 0.002_f64.to_bits());
    tracker.observe(&[Vec3::ZERO, Vec3::new(9.0, 0.0, 0.0)], 0.004);
    let stats = tracker.stats().unwrap();
    assert_eq!(stats.near_substeps, 2);
    assert_eq!(stats.gap_min_m.to_bits(), 4.399_f64.to_bits());
    assert_eq!(stats.gap_min_t.to_bits(), 0.002_f64.to_bits());
    tracker.observe(&[], 0.005);
    tracker.observe(&[Vec3::ZERO], 0.006);
    assert_eq!(tracker.stats(), Some(stats));

    let mut single = NearTracker::default();
    single.observe(&[Vec3::ZERO], 0.001);
    assert_eq!(single.stats(), None);

    let vertical = [Vec3::ZERO, Vec3::new(0.0, 100.0, 0.0)];
    assert_eq!(ground_distance(vertical[0], vertical[1]), 0.0);
    let mut heights = NearTracker::new();
    heights.observe(&vertical, 0.001);
    assert_eq!(heights.stats().unwrap().gap_min_m, 0.0);
    assert_eq!(heights.stats().unwrap().near_substeps, 1);
    // More than one near pair still counts just one substep.
    heights.observe(&[Vec3::ZERO, Vec3::ZERO, Vec3::ZERO], 0.002);
    assert_eq!(heights.stats().unwrap().near_substeps, 2);
    assert_eq!(
        heights.stats().unwrap().gap_min_t.to_bits(),
        0.001_f64.to_bits()
    );

    // First-pair NaN stays first; a later NaN cannot displace a finite pair.
    let nan = Vec3::new(f64::NAN, 0.0, 0.0);
    assert!(min_pair_distance(&[nan, Vec3::ZERO, Vec3::ZERO])
        .unwrap()
        .is_nan());
    assert_eq!(min_pair_distance(&[Vec3::ZERO, Vec3::ZERO, nan]), Some(0.0));
    let mut nonfinite = NearTracker::new();
    nonfinite.observe(&[nan, Vec3::ZERO], 0.001);
    nonfinite.observe(&[Vec3::ZERO, Vec3::ZERO], 0.002);
    let stats = nonfinite.stats().unwrap();
    assert!(stats.gap_min_m.is_nan());
    assert_eq!(stats.gap_min_t.to_bits(), 0.001_f64.to_bits());
    assert_eq!(stats.near_substeps, 1);
    tracker.observe(&[nan, Vec3::ZERO], 0.007);
    assert_eq!(tracker.stats().unwrap().gap_min_m, 4.399);
    assert_eq!(tracker.stats().unwrap().near_substeps, 2);
}

#[test]
fn one_car_race_has_no_gap() {
    let (cfg, body, track) = (Config::default(), body(), track());
    let mut s = setup(&cfg, &body, &track);
    s.time_limit_s = 1.0;
    let r = race::run(&s).unwrap();
    assert!(matches!(r.end, RaceEnd::TimeLimit), "{:?}", r.end);
    assert_eq!(r.substeps, 1000);
    assert_eq!(r.gap, None);
}

#[test]
fn three_car_race_gap_matches_independent_replay() {
    let (cfg, body, track) = (Config::default(), body(), track());
    let mut s = setup(&cfg, &body, &track);
    s.cars = 3;
    s.time_limit_s = 6.0;
    let r = race::run(&s).unwrap();
    assert!(matches!(r.end, RaceEnd::TimeLimit), "{:?}", r.end);
    assert_eq!(r.contacts, 0);
    assert_eq!(r.substeps, 6000);
    let stats = r.gap.expect("three cars must report a gap");
    let (distance, time, count) = replay(&cfg, &body, &track, &r, s.grid_gap_m);
    assert_eq!(stats.gap_min_m.to_bits(), distance.to_bits());
    assert_eq!(stats.gap_min_t.to_bits(), time.to_bits());
    assert_eq!(stats.near_substeps, count);
    assert_eq!(stats.near_substeps, 0);
    assert!(stats.gap_min_m > 4.40);
    assert!(stats.gap_min_m <= 10.0 + 1e-9);
    eprintln!(
        "three_car_race_gap_matches_independent_replay gap_min_m={} gap_min_t={}",
        stats.gap_min_m, stats.gap_min_t
    );
}

#[test]
fn close_grid_counts_every_near_substep() {
    let (cfg, body, track) = (Config::default(), body(), track());
    let mut s = setup(&cfg, &body, &track);
    s.cars = 2;
    s.grid_gap_m = 4.39;
    s.time_limit_s = 0.2;
    let r = race::run(&s).unwrap();
    assert_eq!(r.contacts, 0, "close grid must remain contact-free");
    assert!(matches!(r.end, RaceEnd::TimeLimit), "{:?}", r.end);
    assert_eq!(r.substeps, 200);
    let stats = r.gap.expect("two cars must report a gap");
    let (distance, time, count) = replay(&cfg, &body, &track, &r, s.grid_gap_m);
    assert_eq!(stats.gap_min_m.to_bits(), distance.to_bits());
    assert_eq!(stats.gap_min_t.to_bits(), time.to_bits());
    assert_eq!(stats.near_substeps, count);
    assert!(stats.near_substeps >= 1);
    assert!(stats.gap_min_m < 4.40);
    eprintln!(
        "close_grid_counts_every_near_substep near_substeps={} substeps={}",
        stats.near_substeps, r.substeps
    );
}
