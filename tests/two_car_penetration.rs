//! This file pins the tick-1714 `PenetrationBound` refusal so its numbers can be
//! read; the fix task replaces the expected failure with the fixed behaviour.

use std::collections::VecDeque;

use crash_sim::car_contact::{
    detect_contacts, ContactCar, ContactError, ContactKey, ContactSettings, Failure, Feature,
};
use crash_sim::input::Script;
use crash_sim::multi_car::{CarPlacement, MultiSim, StepReport};
use crash_sim::net::session::{Input, LockstepSim};
use crash_sim::race::lockstep::{controls_of, LockstepFailure, LockstepRace, LockstepSetup};
use crash_sim::race::{car_position, place_on_grid, slot_distance};
use crash_sim::race_hash::multi_state_hash;
use crash_sim::settle::{self, CarSetup};
use crash_sim::track::Track;
use crash_sim::vec3::Vec3;
use crash_sim::{Body, Config};

const FAIL_TICK: u64 = 1714;
const FAIL_SUBSTEP: u64 = FAIL_TICK * 10 + 4;
const SKIN: f64 = 0.25;
const MAX_MOTION: f64 = 0.03125;
// car_contact.rs:461-465 checks both distances against skin - max_penetration;
// race.rs:36-39 supplies these unchanged settings (also documented at :9-13).
// ContactSettings::new (:45-52) already caps max_penetration at skin/2.
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
    let settings = ContactSettings::new(0.25, 0.0, h, 0.25 / 8.0, 0.25 / 2.0).unwrap();
    let multi =
        MultiSim::new(sims, &[CarPlacement { offset: Vec3::ZERO }; 2], settings, h).unwrap();
    (cfg, multi, settings)
}

// Copy of tests/two_car_sweep.rs::closest, itself car_contact.rs:187-192.
// Only fixed-scene snapshots validated by the kernel are diagnosed, not arbitrary
// caller floats; this is not a general-purpose geometry API.
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

fn expected() -> ContactError {
    ContactError {
        car: Some(0),
        feature: Feature::Contact(ContactKey {
            source_car: 0,
            node: 7,
            target_car: 1,
            beam: 238,
        }),
        failure: Failure::PenetrationBound,
    }
}

fn pinned_debug() -> String {
    format!(
        "Sim {{ tick: {FAIL_TICK}, t: 17.145, error: Contact({:?}) }}",
        expected()
    )
}

#[test]
fn penetration_replay_matches_the_pin() {
    let (cfg, body, track) = scene();
    let scripts = scripts();
    let mut race = LockstepRace::new(&LockstepSetup {
        cfg: &cfg,
        body: &body,
        track: &track,
        grid_gap_m: 10.0,
    })
    .unwrap();
    for k in 0..=FAIL_TICK {
        race.step(k, inputs_at(&scripts, k, &cfg));
        if k < FAIL_TICK {
            assert!(
                race.failure().is_none(),
                "tick={k} failure={:?}",
                race.failure()
            );
        } else {
            let failure = race
                .failure()
                .unwrap_or_else(|| panic!("tick={k} expected failure, found=None"));
            assert_eq!(format!("{failure:?}"), pinned_debug(), "tick={k}");
        }
        if k == 99 || k == 199 {
            check_network_hash(k, race.state_hash());
        }
        if k % 100 == 99 {
            eprintln!("penetration hash tick={k} value={:016x}", race.state_hash());
        }
    }
}

#[derive(Debug)]
struct Entry {
    i: u64,
    t: f64,
    d: f64,
    n: Vec3,
    rel_n: f64,
    rel_t: f64,
    contacts: usize,
    impulses: usize,
    broken: bool,
}

/// First matching rule wins (skin 0.25, max_motion 0.03125):
/// 1. `run <= 4 || (0.25 - d_final) / run as f64 >= 0.03125 / 2.0`:
///    `tunnelling_fast_approach`, crossing at more than half the motion bound.
/// 2. `g_last >= 0.03125 / 2.0 || normals_dot < 0.9`:
///    `jump_in_last_substep`, a depth jump or a turn round the beam end/face.
/// 3. `rel_normal_start < 0.0 && -rel_normal_start * h >= 0.5 * g_last`:
///    `response_too_weak`, residual closing explains at least half the growth.
/// 4. Otherwise `steady_push_overlap`: closing was removed and substep
///    acceleration grows overlap that velocity-only response cannot undo.
fn classify(run: u64, d_final: f64, d_old: f64, dot: f64, start: f64, h: f64) -> &'static str {
    let g_last = d_old - d_final;
    if run <= 4 || (SKIN - d_final) / run as f64 >= MAX_MOTION / 2.0 {
        "tunnelling_fast_approach"
    } else if g_last >= MAX_MOTION / 2.0 || dot < 0.9 {
        "jump_in_last_substep"
    } else if start < 0.0 && -start * h >= 0.5 * g_last {
        "response_too_weak"
    } else {
        "steady_push_overlap"
    }
}

// Copy of tests/two_car_sweep.rs::print_nearest's traversal and ordering,
// adding full-scene counts before printing the five nearest live pairs.
fn print_nearest(multi: &MultiSim, fin: &[Vec<Vec3>; 2]) {
    let mut nearest = Vec::new();
    for (source_car, nodes) in fin.iter().enumerate() {
        let target_car = 1 - source_car;
        let target = multi.sim(target_car);
        for (node, &p) in nodes.iter().enumerate() {
            for (beam, b) in target.beams.iter().enumerate() {
                if target.broken[beam] {
                    continue;
                }
                let (_, _, d) = closest(p, fin[target_car][b.a], fin[target_car][b.b]);
                nearest.push((
                    d,
                    ContactKey {
                        source_car,
                        node,
                        target_car,
                        beam,
                    },
                ));
            }
        }
    }
    nearest.sort_by(|a, b| a.0.total_cmp(&b.0).then_with(|| a.1.cmp(&b.1)));
    assert!(nearest.len() >= 5, "nearest pair count={}", nearest.len());
    eprintln!(
        "penetration others within_skin={} below_bound={}",
        nearest.iter().filter(|(d, _)| *d <= SKIN).count(),
        nearest.iter().filter(|(d, _)| *d < BOUND).count()
    );
    for (rank, (d, key)) in nearest.iter().take(5).enumerate() {
        eprintln!(
            "penetration nearest rank={} source={}/{} target={}/{} d={d:.9}",
            rank + 1,
            key.source_car,
            key.node,
            key.target_car,
            key.beam
        );
    }
}

fn print_deform(multi: &MultiSim) {
    for c in 0..2 {
        let sim = multi.sim(c);
        let yielded = sim
            .beams
            .iter()
            .zip(&sim.rest0)
            .filter(|(b, rest)| b.rest.to_bits() != rest.to_bits())
            .count();
        let broken = sim.broken.iter().filter(|&&b| b).count();
        eprintln!("penetration deform car={c} yielded={yielded} broken={broken}");
    }
    eprintln!(
        "penetration deform beam238 rest0={:.9} rest={:.9}",
        multi.sim(1).rest0[238],
        multi.sim(1).beams[238].rest
    );
    let sim = multi.sim(0);
    let (mut incident, mut max_change) = (0, 0.0_f64);
    for (i, b) in sim.beams.iter().enumerate() {
        if !sim.broken[i] && (b.a == 7 || b.b == 7) {
            incident += 1;
            max_change = max_change.max((b.rest - sim.rest0[i]).abs() / sim.rest0[i]);
        }
    }
    eprintln!("penetration deform node7 incident={incident} max_rest_change={max_change:.6}");
}

#[test]
fn penetration_1714_contact_inputs() {
    let (cfg, mut multi, settings) = sweep_scene();
    let h = cfg.dt / f64::from(cfg.substeps);
    let scripts = scripts();
    assert_eq!(multi.sim(1).beams[238].a, 37, "beam238 endpoint a");
    assert_eq!(multi.sim(1).beams[238].b, 39, "beam238 endpoint b");
    let mut history = VecDeque::with_capacity(51);
    let (mut first_skin, mut first_contact) = (None, None);
    let (mut in_skin_total, mut run) = (0_u64, 0_u64);
    let mut totals = StepReport::default();
    let mut capture = None;
    'ticks: for k in 0..=FAIL_TICK {
        let inputs = inputs_at(&scripts, k, &cfg);
        let controls = [controls_of(inputs[0]), controls_of(inputs[1])];
        for j in 0..10 {
            let i = k * 10 + j;
            let prev: [Vec<Vec3>; 2] = std::array::from_fn(|c| multi.sim(c).pos.clone());
            let result = multi.advance(&controls);
            let report = match result {
                Ok(report) => {
                    assert!(i < FAIL_SUBSTEP, "substep={i} expected Err, found={report:?}");
                    report
                }
                Err(error) => {
                    assert_eq!(i, FAIL_SUBSTEP, "substep={i} found={error:?}");
                    assert_eq!(
                        format!("{error:?}"),
                        format!("Contact({:?})", expected()),
                        "substep={i}"
                    );
                    let failure = LockstepFailure::Sim {
                        tick: k,
                        t: (i + 1) as f64 * h,
                        error,
                    };
                    assert_eq!(format!("{failure:?}"), pinned_debug(), "substep={i}");
                    StepReport::default()
                }
            };
            totals.contacts += report.contacts;
            totals.impulses += report.impulses;
            if report.contacts > 0 && first_contact.is_none() {
                first_contact = Some(i);
            }
            if j == 9 && (k == 99 || k == 199) {
                check_network_hash(k, multi_state_hash(k, &multi).unwrap());
            }
            let (car0, car1) = (multi.sim(0), multi.sim(1));
            let (t, delta, d) = closest(car0.pos[7], car1.pos[37], car1.pos[39]);
            assert!(d > 0.0, "substep={i} undefined normal d={d:?}");
            let n = delta * (1.0 / d);
            let relative = car0.vel[7] - (car1.vel[37] * (1.0 - t) + car1.vel[39] * t);
            let rel_n = relative.dot(n);
            let rel_t = (relative - n * rel_n).length();
            assert!(
                rel_n.is_finite() && rel_t.is_finite(),
                "substep={i} rel_n={rel_n:?} rel_t={rel_t:?}"
            );
            if d <= SKIN {
                first_skin.get_or_insert(i);
                in_skin_total += 1;
                run += 1;
            } else {
                run = 0;
            }
            if history.len() == 51 {
                let _ = history.pop_front();
            }
            history.push_back(Entry {
                i,
                t,
                d,
                n,
                rel_n,
                rel_t,
                contacts: report.contacts,
                impulses: report.impulses,
                broken: car1.broken[238],
            });
            if i == FAIL_SUBSTEP {
                capture = Some((prev, inputs));
                break 'ticks;
            }
        }
    }
    let (prev, inputs) = capture.expect("missing failing-substep capture");
    let fin: [Vec<Vec3>; 2] = std::array::from_fn(|c| multi.sim(c).pos.clone());
    let mut vel: [Vec<Vec3>; 2] = std::array::from_fn(|c| multi.sim(c).vel.clone());
    // MultiSim::advance snapshots, substeps, checks, then applies contacts.
    // car_contact.rs:517-591 stages one fixed-order velocity-only pass; refusals
    // write nothing. Successful samples are post-response; the last is pre-response.
    let views: Vec<_> = vel
        .iter_mut()
        .enumerate()
        .map(|(c, v)| ContactCar {
            previous_pos: &prev[c],
            pos: &fin[c],
            vel: v,
            mass: &multi.sim(c).mass,
            beams: &multi.sim(c).beams,
            broken: &multi.sim(c).broken,
        })
        .collect();
    assert_eq!(
        detect_contacts(&views, settings),
        Err(expected()),
        "captured kernel inputs"
    );
    drop(views);
    assert_eq!(history.len(), 51, "history length={}", history.len());
    let last = &history[50];
    let before = &history[49];
    let (p0, a0, b0) = (prev[0][7], prev[1][37], prev[1][39]);
    let (p1, a1, b1) = (fin[0][7], fin[1][37], fin[1][39]);
    let (t, _, d_final) = closest(p1, a1, b1);
    let (_, _, d_old) = closest(p0, a0, b0);
    // prepare (:427-465) checks final/old geometry in BTreeSet key order.
    assert!(d_final < BOUND, "d_final={d_final:?} bound={BOUND:?}");
    assert!(d_old >= BOUND, "d_old={d_old:?} bound={BOUND:?}");
    assert!(!last.broken, "beam238 broken={}", last.broken);
    assert_eq!(
        d_old.to_bits(),
        before.d.to_bits(),
        "d_old={d_old:?} previous={:?}",
        before.d
    );
    let normals_dot = before.n.dot(last.n);
    let r = (p1 - p0) - ((a1 - a0) * (1.0 - t) + (b1 - b0) * t);
    let disp_normal = r.dot(last.n);
    eprintln!("penetration failure tick={FAIL_TICK} substep={FAIL_SUBSTEP} key=0/7/1/238");
    let (depth, excess) = (SKIN - d_final, BOUND - d_final);
    eprintln!("penetration bound skin=0.25 max_penetration=0.125 bound={BOUND:.9} d_final={d_final:.9} d_old={d_old:.9} depth={depth:.9} excess={excess:e} failed=final");
    let (t, nx, ny, nz) = (last.t, last.n.x, last.n.y, last.n.z);
    eprintln!("penetration geometry t={t:.9} n=({nx:.9},{ny:.9},{nz:.9}) normals_dot={normals_dot:.12}");
    let (rel_n, rel_t, start) = (last.rel_n, last.rel_t, before.rel_n);
    let disp_tangent = (r - last.n * disp_normal).length();
    eprintln!("penetration closing rel_normal={rel_n:.6} rel_tangent={rel_t:.6} rel_normal_start={start:.6} disp_normal={disp_normal:.6} disp_tangent={disp_tangent:.6}");
    let first_skin = first_skin.map_or_else(|| "none".to_owned(), |i| i.to_string());
    let first_contact = first_contact.map_or_else(|| "none".to_owned(), |i| i.to_string());
    let (contacts_total, impulses_total) = (totals.contacts, totals.impulses);
    eprintln!("penetration contact run={run} in_skin_total={in_skin_total} first_skin={first_skin} contacts_total={contacts_total} impulses_total={impulses_total} first_contact={first_contact}");
    for e in &history {
        eprintln!(
            "penetration history substep={} d={:.9} depth={:.9} rel_n={:.6} rel_t={:.6} contacts={} impulses={}",
            e.i,
            e.d,
            SKIN - e.d,
            e.rel_n,
            e.rel_t,
            e.contacts,
            e.impulses
        );
    }
    let (mut max_node_motion, mut max_speed_h) = (0.0_f64, 0.0_f64);
    for (c, positions) in fin.iter().enumerate() {
        for ((&p, &old), &v) in positions.iter().zip(&prev[c]).zip(&vel[c]) {
            max_node_motion = max_node_motion.max((p - old).length());
            max_speed_h = max_speed_h.max(v.length() * h);
        }
    }
    // No adaptive retry; validate (:223-230) bounds displacement and speed*h.
    let motion_cap_used = max_node_motion.max(max_speed_h) / MAX_MOTION;
    eprintln!("penetration step substeps_per_tick=10 h={h:e} adaptive=no max_node_motion={max_node_motion:.9} max_speed_h={max_speed_h:.9} max_motion=0.03125 motion_cap_used={motion_cap_used:.3}");
    let masses: [f64; 2] = std::array::from_fn(|c| multi.sim(c).mass.iter().sum());
    let (car0, car1) = (multi.sim(0), multi.sim(1));
    let (m_node, m_a, m_b) = (car0.mass[7], car1.mass[37], car1.mass[39]);
    // car_contact.rs:541: inverse mass weighted by the squared beam fractions.
    let u = 1.0 - t;
    let effective_inverse = 1.0 / m_node + u * u / m_a + t * t / m_b;
    let [mass0, mass1] = masses;
    eprintln!("penetration mass node={m_node:.3} a={m_a:.3} b={m_b:.3} effective_inverse={effective_inverse:.6} car0={mass0:.3} car1={mass1:.3}");
    // MultiSim forwards counts, not ContactReport::normal_impulse; history
    // exposes the response's counts and relative-speed effect, not its impulse.
    eprintln!("penetration impulse exposed=no");
    print_deform(&multi);
    print_nearest(&multi, &fin);
    let [speed0, speed1]: [f64; 2] = std::array::from_fn(|c| {
        let momentum = vel[c]
            .iter()
            .zip(&multi.sim(c).mass)
            .fold(Vec3::ZERO, |p, (&v, &m)| p + v * m);
        (momentum * (1.0 / masses[c])).length()
    });
    let centre_gap = (car_position(multi.sim(1)) - car_position(multi.sim(0))).length();
    eprintln!("penetration speed car0={speed0:.6} car1={speed1:.6} centre_gap={centre_gap:.6}");
    eprintln!(
        "penetration inputs car0={}/{}/{} car1={}/{}/{}",
        inputs[0].throttle,
        inputs[0].brake,
        inputs[0].steer,
        inputs[1].throttle,
        inputs[1].brake,
        inputs[1].steer
    );
    eprintln!(
        "penetration class={}",
        classify(run, d_final, d_old, normals_dot, before.rel_n, h)
    );
}
