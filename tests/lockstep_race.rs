use std::thread;
use std::time::Duration;

use crash_sim::net::session::{
    channel_pair, ChannelTransport, Input, LockstepSim, Role, Session, SessionConfig, SessionError,
};
use crash_sim::race::lap_stats::LapStatsTracker;
use crash_sim::race::lockstep::{
    controls_of, LockstepFailure, LockstepRace, LockstepSetup, TickEvent, FAILED_HASH,
};
use crash_sim::race::near::{ground_distance, NEAR_M};
use crash_sim::race::{car_position, place_on_grid, slot_distance, RaceError};
use crash_sim::race_hash::multi_state_hash;
use crash_sim::settle::{self, CarSetup};
use crash_sim::track::{LapEvent, LapTimer, Track};
use crash_sim::{Body, Config};

fn body() -> Body {
    crash_sim::format::parse(include_str!("../scenarios/car_basic.bsn")).unwrap()
}

fn track() -> Track {
    Track::parse(include_str!("../scenarios/tracks/oval.track")).unwrap()
}

fn setup<'a>(cfg: &'a Config, body: &'a Body, track: &'a Track) -> LockstepSetup<'a> {
    LockstepSetup {
        cfg,
        body,
        track,
        grid_gap_m: 10.0,
    }
}

fn drive(t: u64) -> [Input; 2] {
    [
        Input {
            throttle: if t < 350 { 500 } else { 0 },
            ..Input::NEUTRAL
        },
        Input {
            throttle: if (20..370).contains(&t) { 450 } else { 0 },
            ..Input::NEUTRAL
        },
    ]
}

fn healthy(race: &LockstepRace) {
    assert!(race.failure().is_none(), "{:?}", race.failure());
}

#[test]
fn lockstep_same_inputs_same_hashes() {
    let (cfg, body, track) = (Config::default(), body(), track());
    let s = setup(&cfg, &body, &track);
    let mut first = LockstepRace::new(&s).unwrap();
    let mut second = LockstepRace::new(&s).unwrap();
    let mut changed = LockstepRace::new(&s).unwrap();
    for t in 0..400 {
        first.step(t, drive(t));
        second.step(t, drive(t));
        let mut inputs = drive(t);
        if t == 150 {
            inputs[1].steer = 300;
        }
        changed.step(t, inputs);
        for race in [&first, &second, &changed] {
            healthy(race);
        }
        assert_eq!(first.state_hash(), second.state_hash(), "tick {t}");
        assert_eq!(
            first.state_hash(),
            multi_state_hash(t, first.multi()).unwrap(),
            "stored hash must describe this tick"
        );
        if t < 150 {
            assert_eq!(first.state_hash(), changed.state_hash(), "tick {t}");
        } else if t == 150 || t == 399 {
            assert_ne!(first.state_hash(), changed.state_hash(), "tick {t}");
        }
    }
}

#[test]
fn host_input_drives_car_1_guest_input_car_2() {
    let (cfg, body, track) = (Config::default(), body(), track());
    let s = setup(&cfg, &body, &track);
    for driven in 0..2 {
        let mut race = LockstepRace::new(&s).unwrap();
        let start = [
            car_position(race.multi().sim(0)),
            car_position(race.multi().sim(1)),
        ];
        let mut inputs = [Input::NEUTRAL; 2];
        inputs[driven].throttle = 1000;
        for t in 0..100 {
            race.step(t, inputs);
            healthy(&race);
        }
        for (k, initial) in start.into_iter().enumerate() {
            let distance = ground_distance(initial, car_position(race.multi().sim(k)));
            eprintln!("driven={} car={} distance_m={distance}", driven + 1, k + 1);
            if k == driven {
                assert!(distance > 1.0, "driven car distance {distance}");
            } else {
                assert!(distance < 0.1, "neutral car distance {distance}");
            }
        }
    }
}

#[test]
fn ticks_substeps_and_order_refusal() {
    let (cfg, body, track) = (Config::default(), body(), track());
    let mut s = setup(&cfg, &body, &track);
    let mut race = LockstepRace::new(&s).unwrap();
    assert_eq!(race.gap(), None);
    assert!(race.laps(0).is_empty());
    assert!(race.laps(u32::MAX).is_empty());
    for t in 0..7 {
        race.step(t, drive(t));
        healthy(&race);
    }
    assert_eq!(race.ticks(), 7);
    assert_eq!(race.substeps(), 70);
    assert_eq!(race.substeps_per_tick(), 10);
    assert_eq!(race.h().to_bits(), (0.01_f64 / 10.0).to_bits());
    let before = multi_state_hash(6, race.multi()).unwrap();
    let gap = race.gap();
    race.step(9, drive(9));
    assert!(matches!(
        race.failure(),
        Some(LockstepFailure::TickOrder {
            expected: 7,
            got: 9
        })
    ));
    assert_eq!(race.state_hash(), FAILED_HASH);
    race.step(7, drive(7));
    assert_eq!(race.ticks(), 7);
    assert_eq!(race.substeps(), 70);
    assert_eq!(multi_state_hash(6, race.multi()).unwrap(), before);
    assert_eq!(race.gap(), gap);
    assert!(matches!(
        race.failure(),
        Some(LockstepFailure::TickOrder { got: 9, .. })
    ));
    for bad in [0.0, -1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        s.grid_gap_m = bad;
        assert!(matches!(LockstepRace::new(&s), Err(RaceError::GridGap(_))));
    }
    s.grid_gap_m = f64::MAX;
    assert!(matches!(
        LockstepRace::new(&s),
        Err(RaceError::Grid { slot: 2 })
    ));
    s.grid_gap_m = 10.0;
    let mut bad_cfg = cfg.clone();
    bad_cfg.substeps = 0;
    assert!(matches!(
        LockstepRace::new(&setup(&bad_cfg, &body, &track)),
        Err(RaceError::Config(_))
    ));
    // All i32 values convert without overflow or clamping, including reverse.
    for value in [i32::MIN, -1000, -500, 0, 250, 1000, i32::MAX] {
        let c = controls_of(Input {
            throttle: value,
            brake: value,
            steer: value,
        });
        let expected = f64::from(value) / 1000.0;
        assert_eq!(c.throttle.to_bits(), expected.to_bits());
        assert_eq!(c.brake.to_bits(), expected.to_bits());
        assert_eq!(c.steer.to_bits(), expected.to_bits());
    }
    let mut refused = LockstepRace::new(&s).unwrap();
    let bad = Input {
        brake: -1,
        ..Input::NEUTRAL
    };
    refused.step(0, [bad; 2]);
    assert!(matches!(
        refused.failure(),
        Some(LockstepFailure::Sim { tick: 0, .. })
    ));
    assert_eq!(refused.state_hash(), FAILED_HASH);
    let before = multi_state_hash(0, refused.multi()).unwrap();
    refused.step(0, [Input::NEUTRAL; 2]);
    assert_eq!(refused.ticks(), 0);
    assert_eq!(refused.substeps(), 0);
    assert_eq!(multi_state_hash(0, refused.multi()).unwrap(), before);
}

type PeerOutput = (Vec<(u64, u64)>, u64, Vec<TickEvent>);

fn run_peer(
    transport: ChannelTransport,
    role: Role,
    cfg: Config,
    body: Body,
    track: Track,
) -> Result<PeerOutput, SessionError> {
    // Construct here: neither the race nor its non-Send solver leaves this thread.
    let race = LockstepRace::new(&setup(&cfg, &body, &track)).unwrap();
    let config = SessionConfig {
        seed: 1,
        tick_rate: 100,
        input_delay: 6,
        hash_every: 100,
        timeout: Duration::from_secs(5),
    };
    let mut session = Session::connect(transport, race, config, role)?;
    let mut hashes = Vec::new();
    for t in 0..400 {
        let input = drive(t)[if role == Role::Host { 0 } else { 1 }];
        let step = session.advance(input)?;
        healthy(session.sim());
        assert_eq!(step.tick, t);
        assert_eq!(step.checked_hash.is_some(), (t + 1) % 100 == 0);
        if let Some(hash) = step.checked_hash {
            hashes.push((t, hash));
        }
    }
    let mut race = session.close();
    healthy(&race);
    Ok((hashes, race.state_hash(), race.take_events()))
}

#[test]
fn in_process_peers_match_the_local_run() {
    let (a, b) = channel_pair();
    let (host_cfg, host_body, host_track) = (Config::default(), body(), track());
    let (guest_cfg, guest_body, guest_track) = (Config::default(), body(), track());
    let host = thread::spawn(move || run_peer(a, Role::Host, host_cfg, host_body, host_track));
    let guest = thread::spawn(move || run_peer(b, Role::Guest, guest_cfg, guest_body, guest_track));
    let host = host.join().unwrap().expect("host session ok");
    let guest = guest.join().unwrap().expect("guest session ok");
    assert_eq!(
        host.0.iter().map(|&(t, _)| t).collect::<Vec<_>>(),
        [99, 199, 299, 399]
    );
    assert_eq!(host, guest);
    let (cfg, body, track) = (Config::default(), body(), track());
    let mut local = LockstepRace::new(&setup(&cfg, &body, &track)).unwrap();
    let mut hashes = Vec::new();
    for t in 0..400 {
        let pick = |k: usize| {
            if t < 6 {
                Input::NEUTRAL
            } else {
                drive(t - 6)[k]
            }
        };
        local.step(t, [pick(0), pick(1)]);
        healthy(&local);
        if (t + 1) % 100 == 0 {
            hashes.push((t, local.state_hash()));
        }
    }
    assert_eq!(hashes, host.0);
    assert_eq!(local.state_hash(), host.1);
    assert_eq!(local.take_events(), host.2);
}

fn event_time(event: &LapEvent) -> f64 {
    match event {
        LapEvent::Start { t } | LapEvent::Checkpoint { t, .. } => *t,
        LapEvent::Lap(lap) => lap.end_t,
    }
}

#[test]
fn events_carry_the_tick_they_happened_in() {
    let (cfg, body, track) = (Config::default(), body(), track());
    let mut race = LockstepRace::new(&setup(&cfg, &body, &track)).unwrap();
    let mut actual = Vec::new();
    for t in 0..600 {
        race.step(t, drive(t));
        healthy(&race);
        // Deliberately read much later than the event's production tick.
        if (t + 1) % 200 == 0 {
            actual.extend(race.take_events());
        }
    }
    assert!(race.take_events().is_empty());
    let first = actual
        .iter()
        .find(|e| e.car == 1 && matches!(e.event, LapEvent::Start { .. }))
        .expect("car 1 crosses the start");
    eprintln!("car_1_first_start_tick={}", first.tick);
    for event in &actual {
        let t = event_time(&event.event);
        assert!(
            event.tick as f64 * 0.01 - 1e-9 <= t && t <= (event.tick + 1) as f64 * 0.01 + 1e-9,
            "{event:?}"
        );
        assert_eq!(
            event.lap_stats.is_some(),
            matches!(event.event, LapEvent::Lap(_))
        );
    }
    // Independent isolated-car replay, as in race_near: no LockstepRace
    // stepping or controls_of in the reference. Compare the full ordered
    // sequence assembled substep-first, then car 1 before car 2.
    assert!(race.gap().unwrap().gap_min_m > NEAR_M);
    let h = cfg.dt / f64::from(cfg.substeps);
    let mut one = cfg.clone();
    one.dt = h;
    one.substeps = 1;
    let mut sims: Vec<_> = (1..=2)
        .map(|slot| {
            let placed = place_on_grid(&body, &track, slot_distance(&track, slot, 10.0)).unwrap();
            settle::car_sim(one.clone(), &placed, &CarSetup::default()).unwrap()
        })
        .collect();
    let mut previous: Vec<_> = sims.iter().map(car_position).collect();
    let mut timers = [LapTimer::new(&track), LapTimer::new(&track)];
    let mut stats = [LapStatsTracker::new(), LapStatsTracker::new()];
    let mut expected = Vec::new();
    for i in 0..6000_u64 {
        let t0 = i as f64 * h;
        let t1 = (i + 1) as f64 * h;
        for (k, sim) in sims.iter_mut().enumerate() {
            let input = drive(i / 10)[k];
            sim.car
                .as_mut()
                .unwrap()
                .set_controls(f64::from(input.throttle) / 1000.0, 0.0, 0.0)
                .unwrap();
            sim.advance();
            sim.check().unwrap();
            let pos = car_position(sim);
            let events = timers[k].update(previous[k], pos, t0, t1).unwrap();
            let mut completed = stats[k]
                .observe(previous[k], pos, t0, t1, h, &events)
                .into_iter();
            for event in events {
                let lap_stats = if matches!(event, LapEvent::Lap(_)) {
                    completed.next()
                } else {
                    None
                };
                expected.push(TickEvent {
                    tick: i / 10,
                    car: k as u32 + 1,
                    event,
                    lap_stats,
                });
            }
            previous[k] = pos;
        }
        assert!(
            ground_distance(previous[0], previous[1]) > NEAR_M,
            "isolated reference requires separated cars at substep {i}"
        );
    }
    assert!(!expected.is_empty());
    assert_eq!(actual, expected);
}
