//! Ready-104 fixes the tick-1714 refusal by position follow; this file pins the
//! fixed state rather than accepting the former `PenetrationBound` failure.

use crash_sim::car_contact::ContactSettings;
use crash_sim::input::Script;
use crash_sim::multi_car::{CarPlacement, MultiSim};
use crash_sim::net::session::{Input, LockstepSim};
use crash_sim::race::lockstep::{controls_of, LockstepRace, LockstepSetup};
use crash_sim::race::{place_on_grid, slot_distance};
use crash_sim::race_hash::multi_state_hash;
use crash_sim::settle::{self, CarSetup};
use crash_sim::track::Track;
use crash_sim::vec3::Vec3;
use crash_sim::{Body, Config};

const LAST_SUBSTEP: u64 = 17144;
const SKIN: f64 = 0.25;
// The unchanged skin - max_penetration bound used by the kernel.
const BOUND: f64 = 0.25 - 0.125;

// Copy of tests/two_car_sweep.rs::scene.
fn scene() -> (Config, Body, Track) {
    (
        Config::default(),
        crash_sim::format::parse(include_str!("../scenarios/car_basic.bsn")).unwrap(),
        Track::parse(include_str!("../scenarios/tracks/oval.track")).unwrap(),
    )
}

// Copy of tests/two_car_sweep.rs::scripts.
fn scripts() -> [Vec<(f64, Input)>; 2] {
    [
        include_str!("../scenarios/full_brake.input"),
        include_str!("../scenarios/tracks/oval_lap.input"),
    ]
    .map(|text| {
        Script::parse(text)
            .unwrap()
            .lines()
            .iter()
            .map(|line| {
                // cli/net_race.rs::thousandths: require the exact round trip.
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
            .collect()
    })
}

// Copy of tests/two_car_sweep.rs::inputs_at, itself cli/net_race.rs::input_at:
// reference delay 6 and execute's tolerance.
fn inputs_at(scripts: &[Vec<(f64, Input)>; 2], tick: u64, cfg: &Config) -> [Input; 2] {
    let tol = cfg.dt / f64::from(cfg.substeps) * 1e-6;
    std::array::from_fn(|car| {
        if tick < 6 {
            return Input::NEUTRAL;
        }
        let t = (tick - 6) as f64 * cfg.dt + tol;
        let script = &scripts[car];
        let n = script.partition_point(|(time, _)| *time <= t);
        script
            .get(n.saturating_sub(1))
            .map_or(Input::NEUTRAL, |(_, input)| *input)
    })
}

// Copy of tests/two_car_sweep.rs::check_network_hash.
fn check_network_hash(tick: u64, found: u64) {
    let expected = match tick {
        99 => 0x8c76e69f8437f16d,
        199 => 0x00bec3e589a2d5ec,
        _ => panic!("no network hash reference for tick={tick}, found={found:016x}"),
    };
    assert_eq!(found, expected, "tick={tick} found hash={found:016x}");
}

// Copy of tests/two_car_sweep.rs::sweep_scene.
fn sweep_scene() -> (Config, MultiSim) {
    let (cfg, body, track) = scene();
    let h = cfg.dt / f64::from(cfg.substeps);
    assert_eq!(cfg.substeps, 10, "found substeps={}", cfg.substeps);
    let mut sims = Vec::new();
    for slot in 1..=2 {
        let placed = place_on_grid(&body, &track, slot_distance(&track, slot, 10.0)).unwrap();
        let mut one = cfg.clone();
        one.dt = h;
        one.substeps = 1;
        sims.push(settle::car_sim(one, &placed, &CarSetup::default()).unwrap());
    }
    let settings = ContactSettings::new(0.25, 0.0, h, 0.25 / 8.0, 0.25 / 2.0).unwrap();
    let multi =
        MultiSim::new(sims, &[CarPlacement { offset: Vec3::ZERO }; 2], settings, h).unwrap();
    (cfg, multi)
}

// Copy of tests/two_car_sweep.rs::closest, itself car_contact.rs:187-192.
// Only fixed-scene snapshots are measured, not arbitrary caller floats.
fn closest(p: Vec3, a: Vec3, b: Vec3) -> (f64, Vec3, f64) {
    let ab = b - a;
    let squared = ab.dot(ab);
    assert!(
        squared.is_finite() && squared > 0.0,
        "diagnostic beam squared length={squared:?}"
    );
    let t = ((p - a).dot(ab) / squared).clamp(0.0, 1.0);
    let delta = (p - a) - ab * t;
    let d = delta.length();
    assert!(
        t.is_finite() && d.is_finite(),
        "diagnostic closest t={t:?} d={d:?}"
    );
    (t, delta, d)
}

#[test]
fn penetration_replay_passes_1714() {
    let (cfg, body, track) = scene();
    let scripts = scripts();
    let mut race = LockstepRace::new(&LockstepSetup {
        cfg: &cfg,
        body: &body,
        track: &track,
        grid_gap_m: 10.0,
    })
    .unwrap();
    for k in 0..2000 {
        race.step(k, inputs_at(&scripts, k, &cfg));
        assert!(
            race.failure().is_none(),
            "tick={k} failure={:?}",
            race.failure()
        );
        if k == 99 || k == 199 {
            check_network_hash(k, race.state_hash());
        }
        if k % 100 == 99 {
            eprintln!("penetration hash tick={k} value={:016x}", race.state_hash());
        }
    }
}

#[test]
fn pair_0_7_1_238_stays_above_the_bound() {
    let (cfg, mut multi) = sweep_scene();
    let scripts = scripts();
    assert_eq!(multi.sim(1).beams[238].a, 37, "beam238 endpoint a");
    assert_eq!(multi.sim(1).beams[238].b, 39, "beam238 endpoint b");
    let mut first_skin = None;
    let mut min_d = f64::INFINITY;
    let mut d = f64::NAN;
    for i in 0..=LAST_SUBSTEP {
        let k = i / u64::from(cfg.substeps);
        let inputs = inputs_at(&scripts, k, &cfg);
        let controls = [controls_of(inputs[0]), controls_of(inputs[1])];
        multi
            .advance(&controls)
            .unwrap_or_else(|error| panic!("substep={i} failure={error:?}"));
        if i % u64::from(cfg.substeps) == u64::from(cfg.substeps) - 1 && (k == 99 || k == 199) {
            check_network_hash(k, multi_state_hash(k, &multi).unwrap());
        }
        let (car0, car1) = (multi.sim(0), multi.sim(1));
        let b = car1.beams[238];
        assert!(!car1.broken[238], "substep={i} beam238 broken");
        d = closest(car0.pos[7], car1.pos[b.a], car1.pos[b.b]).2;
        assert!(d >= BOUND, "substep={i} d={d:.17e} bound={BOUND:.17e}");
        min_d = min_d.min(d);
        if d <= SKIN {
            first_skin.get_or_insert(i);
        }
    }
    eprintln!("pair 0/7/1/238 first_skin={first_skin:?} min_d={min_d:.9}");
    eprintln!(
        "penetration pair substep={LAST_SUBSTEP} d={d:.9} min_d={min_d:.9} first_skin={first_skin:?}"
    );
}
