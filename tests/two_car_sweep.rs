//! Ready-101 and ready-102 fixed the substep-2780 and substep-2848 refusals.
//! Pins both certificates, early hashes and the next known failure at tick 1714.

use crash_sim::car_contact::{detect_contacts, ContactCar, ContactSettings};
use crash_sim::input::Script;
use crash_sim::multi_car::{CarPlacement, MultiSim};
use crash_sim::net::session::{Input, LockstepSim};
use crash_sim::race::lockstep::{controls_of, LockstepRace, LockstepSetup};
use crash_sim::race::{place_on_grid, slot_distance};
use crash_sim::settle::{self, CarSetup};
use crash_sim::track::Track;
use crash_sim::vec3::Vec3;
use crash_sim::{Body, Config};

fn scene() -> (Config, Body, Track) {
    (
        Config::default(),
        crash_sim::format::parse(include_str!("../scenarios/car_basic.bsn")).unwrap(),
        Track::parse(include_str!("../scenarios/tracks/oval.track")).unwrap(),
    )
}

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
                // cli/net_race.rs::thousandths: parsed controls are finite and
                // in range; require the same exact thousandth round trip.
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

fn inputs_at(scripts: &[Vec<(f64, Input)>; 2], tick: u64, cfg: &Config) -> [Input; 2] {
    // Copy of cli/net_race.rs::input_at (pub(crate)), with the reference
    // run's delay and execute's tolerance.
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

fn check_network_hash(tick: u64, found: u64) {
    let expected = match tick {
        99 => 0x8c76e69f8437f16d,
        199 => 0x00bec3e589a2d5ec,
        _ => panic!("no network hash reference for tick={tick}, found={found:016x}"),
    };
    assert_eq!(found, expected, "tick={tick} found hash={found:016x}");
}

#[test]
fn rear_end_replay_matches_the_network_run() {
    let (cfg, body, track) = scene();
    let scripts = scripts();
    let mut race = LockstepRace::new(&LockstepSetup {
        cfg: &cfg,
        body: &body,
        track: &track,
        grid_gap_m: 10.0,
    })
    .unwrap();
    // Ready-104 fixed the tick-1714 refusal by making positions follow impulses.
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
    }
}

// Copy of car_contact.rs::closest. Only used on this fixed scene's snapshots
// and their linear interpolants, not as a general-purpose geometry API.
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

// Copy of car_contact.rs::separated's projection minus threshold, using
// Vec3::length in place of the kernel's hypot-based norm, as the brief requires.
fn slack(p: Vec3, q: Vec3, n: Vec3) -> f64 {
    let value = (p - q).dot(n)
        - (0.25 * n.length()
            + 64.0 * f64::EPSILON * p.length().max(q.length()).max(0.25) * n.length());
    assert!(value.is_finite(), "diagnostic slack={value:?}");
    value
}

fn sweep_scene() -> (Config, MultiSim, ContactSettings) {
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
    // race.rs's unchanged contact settings, with the same substep timestep.
    let settings = ContactSettings::new(0.25, 0.0, h, 0.25 / 8.0, 0.25 / 2.0).unwrap();
    let multi =
        MultiSim::new(sims, &[CarPlacement { offset: Vec3::ZERO }; 2], settings, h).unwrap();
    (cfg, multi, settings)
}

fn sweep_views<'a>(
    multi: &'a MultiSim,
    prev: &'a [Vec<Vec3>; 2],
    fin: &'a [Vec<Vec3>; 2],
    vel: &'a mut [Vec<Vec3>; 2],
) -> Vec<ContactCar<'a>> {
    vel.iter_mut()
        .enumerate()
        .map(|(c, v)| ContactCar {
            previous_pos: &prev[c],
            pos: &fin[c],
            vel: v,
            mass: &multi.sim(c).mass,
            beams: &multi.sim(c).beams,
            broken: &multi.sim(c).broken,
        })
        .collect()
}

// Copy of car_contact.rs::separated_blend: crossing weights only select
// candidates; the four strict slacks decide whether a plane certifies.
fn blend_candidates(endpoints: [(Vec3, Vec3); 4], n0: Vec3, n1: Vec3) -> Vec<(f64, [f64; 4])> {
    let lines = endpoints.map(|(p, q)| {
        let w = p - q;
        let a = w.dot(n0) - 0.25;
        (a, w.dot(n1) - 0.25 - a)
    });
    let mut candidates = Vec::new();
    for j in 0..4 {
        for k in j + 1..4 {
            let lambda = (lines[k].0 - lines[j].0) / (lines[j].1 - lines[k].1);
            if lambda.is_finite() && lambda > 0.0 && lambda < 1.0 {
                let n = n0 + (n1 - n0) * lambda;
                candidates.push((lambda, endpoints.map(|(p, q)| slack(p, q, n))));
            }
        }
    }
    candidates
}

fn print_swept_min([p0, a0, b0, p1, a1, b1]: [Vec3; 6]) {
    // Sample the kernel's linear capsule model for diagnosis only, not proof.
    // Both diagnostics use this classifier, with no assertion on its class.
    let (_, _, d0) = closest(p0, a0, b0);
    let (mut m, mut min_s) = (d0, 0.0);
    for j in 0..=10000 {
        let s = f64::from(j) / 10000.0;
        let (_, _, d) = closest(
            p0 + (p1 - p0) * s,
            a0 + (a1 - a0) * s,
            b0 + (b1 - b0) * s,
        );
        if d < m {
            m = d;
            min_s = s;
        }
    }
    eprintln!("sweep min_swept d={m:.9} s={min_s:.4}");
    if m > 0.25 {
        eprintln!("sweep class=false_refusal");
    } else {
        eprintln!("sweep class=entered_skin_between_samples");
    }
}

// The pre-follow separation assertions now live in tests/pre_follow_snapshots.rs,
// using frozen geometry rather than the race's position-following trajectory.

#[test]
fn next_refusal_sweep_geometry() {
    let (cfg, mut multi, settings) = sweep_scene();
    let scripts = scripts();
    let mut capture = None;
    'ticks: for k in 0..2000 {
        let inputs = inputs_at(&scripts, k, &cfg);
        let controls = [controls_of(inputs[0]), controls_of(inputs[1])];
        for substep in 0..cfg.substeps {
            let i = k * u64::from(cfg.substeps) + u64::from(substep);
            let prev: [Vec<Vec3>; 2] = std::array::from_fn(|c| multi.sim(c).pos.clone());
            if let Err(error) = multi.advance(&controls) {
                // Discover the refusing substep; never guess it from the tick.
                let fin: [Vec<Vec3>; 2] = std::array::from_fn(|c| multi.sim(c).pos.clone());
                capture = Some((k, i, prev, fin, error));
                break 'ticks;
            }
        }
    }
    let Some((tick, i, prev, fin, error)) = capture else {
        eprintln!("sweep next_refusal=none");
        return;
    };
    let crash_sim::multi_car::MultiError::Contact(crash_sim::car_contact::ContactError {
        feature: crash_sim::car_contact::Feature::Contact(key),
        failure: crash_sim::car_contact::Failure::UnsupportedSweep,
        ..
    }) = &error
    else {
        if let crash_sim::multi_car::MultiError::Contact(crash_sim::car_contact::ContactError {
            feature: crash_sim::car_contact::Feature::Contact(key),
            failure: crash_sim::car_contact::Failure::PenetrationBound,
            ..
        }) = &error
        {
            // ContactError exposes neither penetration depth nor its bound.
            eprintln!(
                "sweep next_failure tick={tick} kind=PenetrationBound key={}/{}/{}/{} depth=not_exposed",
                key.source_car, key.node, key.target_car, key.beam
            );
            return;
        }
        eprintln!("sweep next_refusal={error:?}");
        return;
    };
    let key = *key;
    let mut vel: [Vec<Vec3>; 2] = std::array::from_fn(|c| multi.sim(c).vel.clone());
    let views = sweep_views(&multi, &prev, &fin, &mut vel);
    let refusal = detect_contacts(&views, settings)
        .expect_err("captured refusing sweep unexpectedly accepted by kernel");
    assert_eq!(
        format!("{error:?}"),
        format!("Contact({refusal:?})"),
        "tick={tick} substep={i} captured refusal must reproduce"
    );
    eprintln!(
        "sweep refused tick={tick} substep={i} key={}/{}/{}/{} error={error:?}",
        key.source_car, key.node, key.target_car, key.beam
    );
    let beam = multi.sim(key.target_car).beams[key.beam];
    let (sc, tc) = (key.source_car, key.target_car);
    let (p0, a0, b0) = (prev[sc][key.node], prev[tc][beam.a], prev[tc][beam.b]);
    let (p1, a1, b1) = (fin[sc][key.node], fin[tc][beam.a], fin[tc][beam.b]);
    let (t0, delta0, d0) = closest(p0, a0, b0);
    let (t1, delta1, d1) = closest(p1, a1, b1);
    let n0 = delta0 * (1.0 / d0);
    let n1 = delta1 * (1.0 / d1);
    eprintln!("sweep old t={t0:.9} d={d0:.9}");
    eprintln!("sweep final t={t1:.9} d={d1:.9} d_minus_skin={:e}", d1 - 0.25);
    eprintln!("sweep normals dot={:.12}", n0.dot(n1));
    for (name, n) in [("final", n1), ("old", n0)] {
        for (pos, p, a, b) in [("old", p0, a0, b0), ("final", p1, a1, b1)] {
            for (end, q) in [("a", a), ("b", b)] {
                let value = slack(p, q, n);
                eprintln!("sweep slack n={name} pos={pos} end={end} value={value:e}");
            }
        }
    }
    let candidates = blend_candidates([(p0, a0), (p0, b0), (p1, a1), (p1, b1)], n0, n1);
    let found = candidates
        .iter()
        .any(|(_, slacks)| slacks.iter().all(|&value| value > 0.0));
    let best = candidates.iter().max_by(|(_, a), (_, b)| {
        let a = a.iter().copied().fold(f64::INFINITY, f64::min);
        let b = b.iter().copied().fold(f64::INFINITY, f64::min);
        a.total_cmp(&b)
    });
    eprintln!("sweep blend found={found} best_lambda_slacks={best:?}");
    let r = (p1 - p0) - ((a1 - a0) * (1.0 - t1) + (b1 - b0) * t1);
    eprintln!(
        "sweep motion node={:.9} a={:.9} b={:.9} rel_normal={:.9} rel_tangent={:.9}",
        (p1 - p0).length(),
        (a1 - a0).length(),
        (b1 - b0).length(),
        r.dot(n1),
        (r - n1 * r.dot(n1)).length()
    );
    print_swept_min([p0, a0, b0, p1, a1, b1]);
}
