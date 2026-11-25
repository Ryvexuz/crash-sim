//! Library race contracts, including independent Sim/CLI replay of recorded inputs.

use crash_sim::ai::DriverParams;
use crash_sim::multi_car::MultiError;
use crash_sim::race::{
    self, ProfileParams, RaceEnd, RaceError, RaceFailure, RaceResult, RaceSetup,
};
use crash_sim::settle::{self, CarSetup};
use crash_sim::track::{format_seconds, LapEvent, LapTimer, Track};
use crash_sim::vec3::Vec3;
use crash_sim::{Body, Config};
use std::fmt::Write;
use std::process::{Command, Stdio};
use std::time::Instant;
mod run_tmp;

struct WallTime {
    name: &'static str,
    start: Instant,
}

impl WallTime {
    fn new(name: &'static str) -> Self {
        Self {
            name,
            start: Instant::now(),
        }
    }
}

impl Drop for WallTime {
    fn drop(&mut self) {
        eprintln!(
            "race_core wall_time {}: {:?}",
            self.name,
            self.start.elapsed()
        );
    }
}

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

fn vec_bits(v: Vec3) -> [u64; 3] {
    [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]
}

/// Rebuild alone from the recorded controls, without calling the race driver.
fn replay(cfg: &Config, body: &Body, track: &Track, result: &RaceResult, k: usize) {
    let placed = race::place_on_grid(body, track, race::slot_distance(track, k + 1, 10.0)).unwrap();
    let mut one = cfg.clone();
    one.dt = result.h;
    one.substeps = 1;
    let mut sim = settle::car_sim(one, &placed, &CarSetup::default()).unwrap();
    let mut timer = LapTimer::new(track);
    let mut prev = race::car_position(&sim);
    for i in 0..result.substeps {
        let c = result.cars[k].inputs[(i / result.control_substeps) as usize];
        sim.car
            .as_mut()
            .unwrap()
            .set_controls(c.throttle, c.brake, c.steer)
            .unwrap();
        sim.advance();
        sim.check().unwrap();
        let pos = race::car_position(&sim);
        timer
            .update(prev, pos, i as f64 * result.h, (i + 1) as f64 * result.h)
            .unwrap();
        prev = pos;
    }
    let raced = result.multi.sim(k);
    assert_eq!(sim.pos.len(), raced.pos.len());
    assert_eq!(sim.vel.len(), raced.vel.len());
    for (i, (&a, &b)) in sim.pos.iter().zip(&raced.pos).enumerate() {
        assert_eq!(vec_bits(a), vec_bits(b), "car {k} node {i} pos");
    }
    for (i, (&a, &b)) in sim.vel.iter().zip(&raced.vel).enumerate() {
        assert_eq!(vec_bits(a), vec_bits(b), "car {k} node {i} vel");
    }
    let a = &sim.car.as_ref().unwrap().wheels;
    let b = &raced.car.as_ref().unwrap().wheels;
    assert_eq!(a.len(), b.len());
    for (w, (a, b)) in a.iter().zip(b).enumerate() {
        assert_eq!(vec_bits(a.pos), vec_bits(b.pos), "car {k} wheel {w} pos");
        assert_eq!(vec_bits(a.vel), vec_bits(b.vel), "car {k} wheel {w} vel");
        assert_eq!(a.spin.to_bits(), b.spin.to_bits(), "car {k} wheel {w} spin");
    }
    assert_eq!(timer.laps(), result.cars[k].laps.as_slice(), "car {k} laps");
}

/// Copied from timed_lap: Broadsword's runtime override, else Cargo's layout.
fn crash_sim_bin() -> std::path::PathBuf {
    if let Some(p) = std::env::var_os("CARGO_BIN_EXE_crash-sim") {
        return std::path::PathBuf::from(p);
    }
    let exe = std::env::current_exe().expect("current_exe of the test executable");
    let name = format!("crash-sim{}", std::env::consts::EXE_SUFFIX);
    let dir = exe.parent().and_then(|p| p.parent()).unwrap_or_else(|| {
        panic!(
            "CARGO_BIN_EXE_crash-sim is not set and {} has no grandparent folder",
            exe.display()
        )
    });
    let bin = dir.join(name);
    assert!(
        bin.is_file(),
        "CARGO_BIN_EXE_crash-sim is not set and the cargo layout path {} does not exist",
        bin.display()
    );
    bin
}

fn event_line(event: &LapEvent) -> String {
    match event {
        LapEvent::Start { t } => format!("start t={t:.4}"),
        LapEvent::Checkpoint { lap, k, t } => format!("checkpoint lap={lap} k={k} t={t:.4}"),
        LapEvent::Lap(rec) => format!(
            "lap n={} start_t={:.4} end_t={:.4} time_s={} valid={} reason={} off_track_s={:.3}",
            rec.n,
            rec.start_t,
            rec.end_t,
            format_seconds(rec.time_ms),
            if rec.valid() { "yes" } else { "no" },
            rec.invalid.map_or("none", |r| r.as_str()),
            rec.off_track_s,
        ),
    }
}

#[test]
fn one_car_race_matches_the_timed_lap_program() {
    let _wall = WallTime::new("one_car_race_matches_the_timed_lap_program");
    let (cfg, body, track) = (Config::default(), body(), track());
    let s = setup(&cfg, &body, &track);
    let r = race::run(&s).unwrap();
    assert!(matches!(r.end, RaceEnd::AllFinished), "{:?}", r.end);
    assert!(!r.cars[0].laps.is_empty());
    assert_eq!(r.h.to_bits(), 0.001_f64.to_bits());
    let mut script = String::new();
    for (j, c) in r.cars[0].inputs.iter().enumerate() {
        let ms = j * 20;
        writeln!(
            script,
            "{}.{:03} {} {} {}",
            ms / 1000,
            ms % 1000,
            c.throttle,
            c.brake,
            c.steer
        )
        .unwrap();
    }
    let ms = r.substeps; // h = 1 ms, including the final script endpoint.
    let c = r.cars[0].inputs.last().unwrap();
    writeln!(
        script,
        "{}.{:03} {} {} {}",
        ms / 1000,
        ms % 1000,
        c.throttle,
        c.brake,
        c.steer
    )
    .unwrap();
    let dir = run_tmp::run_tmp();
    std::fs::create_dir_all(&dir).unwrap();
    let input = dir.join("race_core_one_car.input");
    std::fs::write(&input, script).unwrap();
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let out = Command::new(crash_sim_bin())
        .arg("--body")
        .arg(root.join("scenarios/car_basic.bsn"))
        .arg("--track")
        .arg(root.join("scenarios/tracks/oval.track"))
        .arg("--input")
        .arg(&input)
        .stdin(Stdio::null())
        .output()
        .expect("run timed-lap program");
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let stdout = String::from_utf8(out.stdout).unwrap();
    let actual: Vec<_> = stdout
        .lines()
        .filter_map(|line| {
            if line.starts_with("start ") || line.starts_with("checkpoint ") {
                Some(line.to_string())
            } else if line.starts_with("lap ") {
                Some(
                    line.split(' ')
                        .filter(|field| {
                            !field.starts_with("path_m=") && !field.starts_with("max_speed_kmh=")
                        })
                        .collect::<Vec<_>>()
                        .join(" "),
                )
            } else {
                None
            }
        })
        .collect();
    let expected: Vec<_> = r.cars[0].events.iter().map(event_line).collect();
    assert!(expected.iter().any(|l| l.starts_with("start ")));
    assert!(expected.iter().any(|l| l.starts_with("checkpoint ")));
    assert!(expected.iter().any(|l| l.starts_with("lap ")));
    assert_eq!(actual, expected, "timed-lap stdout:\n{stdout}");
    replay(&cfg, &body, &track, &r, 0);
}

#[test]
fn three_ai_cars_finish_one_lap_and_each_replays_alone() {
    let _wall = WallTime::new("three_ai_cars_finish_one_lap_and_each_replays_alone");
    let (cfg, body, track) = (Config::default(), body(), track());
    let mut s = setup(&cfg, &body, &track);
    s.cars = 3;
    // One lap, not three, bounds runtime. This is the in-process 2.4(b) proof:
    // ai_race's ai_inputs_replay_one_car_alone relies on it.
    s.time_limit_s = 80.0;
    let r = race::run(&s).unwrap();
    assert!(matches!(r.end, RaceEnd::AllFinished), "{:?}", r.end);
    assert_eq!(r.contacts, 0);
    assert_eq!(r.impulses, 0);
    assert_eq!(r.cars.len(), 3);
    for (k, car) in r.cars.iter().enumerate() {
        assert_eq!(car.slot, k + 1);
        assert!(car.laps[0].valid(), "car {k}: {:?}", car.laps);
        assert!(car.finish_t.unwrap() <= 80.0);
        assert_eq!(car.finish_t, Some(car.laps[0].end_t));
        assert_eq!(car.wall_substeps, 0, "car {k}: {:?}", car.first_wall_t);
        assert_eq!(car.first_wall_t, None);
        assert_eq!(car.inputs.len() as u64, r.substeps / r.control_substeps);
        eprintln!(
            "race_core car {} lap_time_s={} finish_t={}",
            car.slot,
            format_seconds(car.laps[0].time_ms),
            car.finish_t.unwrap()
        );
        replay(&cfg, &body, &track, &r, k);
    }
}

#[test]
fn race_same_setup_same_bytes() {
    let _wall = WallTime::new("race_same_setup_same_bytes");
    let fresh = || {
        let (cfg, body, track) = (Config::default(), body(), track());
        let mut s = setup(&cfg, &body, &track);
        s.cars = 3;
        s.time_limit_s = 6.0;
        race::run(&s).unwrap()
    };
    let a = fresh();
    let b = fresh();
    assert!(matches!(a.end, RaceEnd::TimeLimit), "{:?}", a.end);
    assert!(matches!(b.end, RaceEnd::TimeLimit), "{:?}", b.end);
    assert_eq!(a.substeps, 6000);
    assert_eq!(a.substeps, b.substeps);
    assert_eq!(a.multi.state_words(), b.multi.state_words());
    assert_eq!(a.cars.len(), b.cars.len());
    for (a, b) in a.cars.iter().zip(&b.cars) {
        assert_eq!(a.inputs.len(), b.inputs.len());
        assert!(!a.inputs.is_empty());
        for (a, b) in a.inputs.iter().zip(&b.inputs) {
            assert_eq!(a.throttle.to_bits(), b.throttle.to_bits());
            assert_eq!(a.brake.to_bits(), b.brake.to_bits());
            assert_eq!(a.steer.to_bits(), b.steer.to_bits());
        }
        assert_eq!(a.events, b.events);
        assert_eq!(a.wall_substeps, b.wall_substeps);
    }
    let body = body();
    let track = track();
    let cfg = Config::default();
    let placed = race::place_on_grid(&body, &track, track.grid).unwrap();
    let sim = settle::car_sim(cfg, &placed, &CarSetup::default()).unwrap();
    let moved = race::car_position(a.multi.sim(0)) - race::car_position(&sim);
    assert!(moved.length() > 20.0, "displacement: {moved:?}");
    assert!(a.cars[0].inputs.iter().any(|c| c.throttle > 0.0));

    // The same control-boundary contract also holds at h = 0.5 ms.
    let cfg = Config {
        substeps: 20,
        ..Config::default()
    };
    let mut s = setup(&cfg, &body, &track);
    s.time_limit_s = 0.04;
    let r = race::run(&s).unwrap();
    assert!(matches!(r.end, RaceEnd::TimeLimit));
    assert_eq!(r.control_substeps, 40);
    assert_eq!(r.substeps, 80);
    assert_eq!((r.substeps as f64 * r.h).to_bits(), 0.04_f64.to_bits());
    replay(&cfg, &body, &track, &r, 0);
}

#[test]
fn wall_rule_counts_nodes_and_tyre_outer_edges() {
    let _wall = WallTime::new("wall_rule_counts_nodes_and_tyre_outer_edges");
    let track = track();
    let centre = Vec3::new(-10.0, 0.0, 0.0);
    let chassis = [Vec3::new(-10.0, 0.0, -0.9), Vec3::new(-10.0, 0.0, 0.9)];
    let inside = vec![vec![
        Vec3::new(-10.0, 0.0, -0.75),
        Vec3::new(-10.0, 0.0, 0.75),
    ]];
    let half = [0.0975];
    assert!(!race::wall_contact(
        &track, centre, 0.0, &chassis, &inside, &half
    ));
    let outside = [chassis[0], Vec3::new(-10.0, 0.0, 6.01)];
    assert!(race::wall_contact(
        &track, centre, 0.0, &outside, &inside, &half
    ));
    for (z, expected) in [(5.95, true), (5.90, false), (-5.95, true)] {
        let mounts = vec![vec![Vec3::new(-10.0, 0.0, z)]];
        assert_eq!(
            race::wall_contact(&track, centre, 0.0, &chassis, &mounts, &half),
            expected,
            "mount z={z}"
        );
    }
    let mounts = vec![vec![Vec3::new(-10.0, 0.0, 5.95)]];
    assert!(race::wall_contact(
        &track,
        centre,
        std::f64::consts::PI,
        &chassis,
        &mounts,
        &half
    ));
    // Exactly centreline-mounted tyres must test both edges, including the left.
    let centre_left = Vec3::new(-10.0, 0.0, -5.95);
    let mounts = vec![vec![centre_left]];
    assert!(race::wall_contact(
        &track,
        centre_left,
        0.0,
        &chassis,
        &mounts,
        &half
    ));
    assert!(race::wall_contact(
        &track,
        centre,
        0.0,
        &chassis,
        &mounts,
        &[]
    ));
    assert!(race::wall_contact(
        &track,
        centre,
        f64::NAN,
        &chassis,
        &inside,
        &half
    ));
}

#[test]
fn narrow_track_counts_every_wall_substep() {
    let _wall = WallTime::new("narrow_track_counts_every_wall_substep");
    let text = include_str!("../scenarios/tracks/oval.track");
    assert_eq!(text.lines().filter(|&line| line == "width 12").count(), 1);
    let narrow = Track::parse(&text.replace("width 12\n", "width 1.6\n")).unwrap();
    let (cfg, body) = (Config::default(), body());
    let mut s = setup(&cfg, &body, &narrow);
    s.time_limit_s = 1.0;
    let r = race::run(&s).unwrap();
    assert!(matches!(r.end, RaceEnd::TimeLimit), "{:?}", r.end);
    assert_eq!(r.substeps, 1000);
    assert_eq!(r.cars[0].wall_substeps, 1000);
    assert_eq!(r.cars[0].first_wall_t.unwrap().to_bits(), r.h.to_bits());
}

#[test]
fn overlapping_grid_surfaces_the_contact_refusal() {
    let _wall = WallTime::new("overlapping_grid_surfaces_the_contact_refusal");
    let (cfg, body, track) = (Config::default(), body(), track());
    let mut s = setup(&cfg, &body, &track);
    s.cars = 2;
    s.grid_gap_m = 0.5;
    s.time_limit_s = 1.0;
    let r = race::run(&s).unwrap();
    match &r.end {
        RaceEnd::Failed(RaceFailure::Sim {
            error: MultiError::Contact(_),
            t,
        }) => {
            assert_eq!(t.to_bits(), r.h.to_bits());
        }
        end => panic!("overlap must refuse on the first substep: {end:?}"),
    }
    assert_eq!(r.substeps, 0);
    assert_eq!(r.cars.len(), 2);
    for car in &r.cars {
        assert_eq!(car.inputs.len(), 1);
        assert!(car.events.is_empty());
    }
}

#[test]
fn race_refuses_bad_setup() {
    let _wall = WallTime::new("race_refuses_bad_setup");
    let (cfg, body, track) = (Config::default(), body(), track());
    macro_rules! refuses {
        ($field:ident, $value:expr, $pattern:pat) => {{
            let mut s = setup(&cfg, &body, &track);
            s.$field = $value;
            assert!(
                matches!(race::run(&s), Err($pattern)),
                "field {}",
                stringify!($field)
            );
        }};
    }
    for cars in [0, 4] {
        let mut s = setup(&cfg, &body, &track);
        s.cars = cars;
        // Removing the upper guard must fail behaviourally, without running
        // a whole lap of four cars just to demonstrate the refusal is missing.
        s.time_limit_s = race::CONTROL_DT;
        assert!(matches!(race::run(&s), Err(RaceError::CarCount(_))));
    }
    for laps in [0, 11] {
        refuses!(laps, laps, RaceError::Laps(_));
    }
    for time in [0.0, -1.0, f64::NAN, f64::INFINITY, f64::MAX] {
        refuses!(time_limit_s, time, RaceError::TimeLimit(_));
    }
    for gap in [0.0, -1.0, f64::INFINITY, f64::NAN] {
        refuses!(grid_gap_m, gap, RaceError::GridGap(_));
    }
    let p = setup(&cfg, &body, &track).driver;
    for (driver, field) in [
        (
            DriverParams {
                lookahead_min: 0.0,
                ..p
            },
            "lookahead_min",
        ),
        (
            DriverParams {
                lookahead_max: 2.0,
                ..p
            },
            "lookahead_max",
        ),
        (
            DriverParams {
                integral_max: -1.0,
                ..p
            },
            "integral_max",
        ),
        (DriverParams { kp: f64::NAN, ..p }, "kp"),
        (
            DriverParams {
                ki: f64::INFINITY,
                ..p
            },
            "ki",
        ),
        (
            DriverParams {
                lookahead_k: f64::NAN,
                ..p
            },
            "lookahead_k",
        ),
        (
            DriverParams {
                lookahead_min: f64::NAN,
                ..p
            },
            "lookahead_min",
        ),
        (
            DriverParams {
                lookahead_max: f64::INFINITY,
                ..p
            },
            "lookahead_max",
        ),
        (
            DriverParams {
                integral_max: f64::NAN,
                ..p
            },
            "integral_max",
        ),
    ] {
        let mut s = setup(&cfg, &body, &track);
        s.driver = driver;
        assert!(matches!(race::run(&s), Err(RaceError::Driver(name)) if name == field));
    }
    let p = setup(&cfg, &body, &track).profile;
    refuses!(
        profile,
        ProfileParams { mu: 0.0, ..p },
        RaceError::Profile(_)
    );
    refuses!(
        profile,
        ProfileParams { v_max: 20.0, ..p },
        RaceError::SpeedAboveContactBound { .. }
    );
    // h = 0.75 ms cannot divide the 20 ms control interval. With dt = 0.01,
    // substeps = 3 still divides it (2 * substeps), so that setup instead
    // reaches the later contact-speed bound.
    let fractional = Config {
        dt: 0.0075,
        substeps: 10,
        ..cfg.clone()
    };
    refuses!(cfg, &fractional, RaceError::Config(_));
    let coarse = Config {
        substeps: 3,
        ..cfg.clone()
    };
    refuses!(cfg, &coarse, RaceError::SpeedAboveContactBound { .. });
    let zero = Config {
        substeps: 0,
        ..cfg.clone()
    };
    refuses!(cfg, &zero, RaceError::Config(_));
    let bad_body = crash_sim::format::parse(include_str!("../scenarios/box.bsn")).unwrap();
    refuses!(body, &bad_body, RaceError::Body(_));
    let mut s = setup(&cfg, &body, &track);
    s.cars = 3;
    s.grid_gap_m = 200.0;
    assert!(matches!(race::run(&s), Err(RaceError::Grid { slot: 3 })));
    // Refusal ordering is observable: an invalid later field cannot hide the first.
    s.cars = 0;
    s.laps = 0;
    s.time_limit_s = f64::NAN;
    assert!(matches!(race::run(&s), Err(RaceError::CarCount(0))));
    s.cars = 1;
    assert!(matches!(race::run(&s), Err(RaceError::Laps(0))));
    s.laps = 1;
    assert!(matches!(race::run(&s), Err(RaceError::TimeLimit(_))));
    assert!(race::slot_distance(&track, 0, 10.0).is_nan());
    for distance in [-1.0, f64::NAN, f64::INFINITY, track.centre_length() + 1.0] {
        assert!(matches!(
            race::place_on_grid(&body, &track, distance),
            Err(RaceError::Grid { slot: 1 })
        ));
    }
    assert_eq!(race::slot_distance(&track, 1, 10.0), 20.0);
    assert_eq!(race::slot_distance(&track, 3, 10.0), 40.0);
    // A positive subnormal time limit still runs a complete control interval;
    // neither the float-to-integer conversion nor ceil may silently make it zero.
    let mut s = setup(&cfg, &body, &track);
    s.time_limit_s = f64::from_bits(1);
    let r = race::run(&s).unwrap();
    assert!(matches!(r.end, RaceEnd::TimeLimit));
    assert_eq!(r.substeps, 20);
    assert_eq!(r.cars[0].inputs.len(), 1);
}
