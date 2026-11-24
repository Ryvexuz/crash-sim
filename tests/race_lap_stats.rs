//! Shared accumulation rule and real single-car replay contracts.
use crash_sim::ai::DriverParams;
use crash_sim::race::lap_stats::LapStatsTracker;
use crash_sim::race::{self, ProfileParams, RaceSetup};
use crash_sim::track::{LapEvent, LapRecord, Track};
use crash_sim::vec3::Vec3;
use crash_sim::{Body, Config};
use std::fmt::Write as _;
use std::process::{Command, Stdio};

fn lap(t: f64) -> LapEvent {
    LapEvent::Lap(LapRecord {
        n: 1,
        start_t: 0.0,
        end_t: t,
        time_ms: 0,
        invalid: None,
        off_track_s: 0.0,
    })
}

fn observe(
    tracker: &mut LapStatsTracker,
    len: f64,
    events: &[LapEvent],
) -> Vec<race::lap_stats::LapStats> {
    tracker.observe(
        Vec3::ZERO,
        Vec3::new(len, 0.0, 0.0),
        0.0,
        0.001,
        0.001,
        events,
    )
}

#[test]
fn lap_stats_hand_cases() {
    let mut tracker = LapStatsTracker::new();
    assert!(observe(&mut tracker, 0.01, &[LapEvent::Start { t: 0.00025 }]).is_empty());
    for len in [0.01, 0.01, 0.02] {
        assert!(observe(&mut tracker, len, &[]).is_empty());
    }
    let checkpoint = LapEvent::Checkpoint {
        lap: 1,
        k: 1,
        t: 0.00075,
    };
    let done = observe(&mut tracker, 0.01, &[lap(0.0005), checkpoint.clone()]);
    assert_eq!(done.len(), 1);
    assert!((done[0].path_m - 0.0525).abs() <= 1e-12);
    assert!((done[0].max_speed_mps - 20.0).abs() <= 1e-12);
    assert!(observe(&mut tracker, 0.01, &[checkpoint]).is_empty());
    let done = observe(&mut tracker, 0.01, &[lap(0.0005)]);
    assert!((done[0].path_m - 0.02).abs() <= 1e-12);
    let done = observe(&mut LapStatsTracker::new(), 0.01, &[lap(0.0005)]);
    assert!((done[0].path_m - 0.005).abs() <= 1e-12);
    assert!((done[0].max_speed_mps - 10.0).abs() <= 1e-12);
}

fn fixtures() -> (Config, Body, Track) {
    (
        Config::default(),
        crash_sim::format::parse(include_str!("../scenarios/car_basic.bsn")).unwrap(),
        Track::parse(include_str!("../scenarios/tracks/oval.track")).unwrap(),
    )
}

fn setup<'a>(cfg: &'a Config, body: &'a Body, track: &'a Track) -> RaceSetup<'a> {
    RaceSetup {
        cfg,
        body,
        track,
        cars: 1,
        laps: 2,
        time_limit_s: 120.0,
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
    assert!(bin.is_file(), "binary missing: {}", bin.display());
    bin
}

fn tmp_file(name: &str, text: &str) -> String {
    let dir = std::path::PathBuf::from(env!("CARGO_TARGET_TMPDIR"));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join(name);
    std::fs::write(&path, text).unwrap();
    path.to_string_lossy().into_owned()
}

#[test]
fn library_race_lap_stats_match_single_car_run() {
    let (cfg, body, track) = fixtures();
    let r = race::run(&setup(&cfg, &body, &track)).unwrap();
    assert!(matches!(r.end, race::RaceEnd::AllFinished));
    assert_eq!(r.contacts, 0);
    assert_eq!(r.h, 0.001);
    let car = &r.cars[0];
    let mut script = String::new();
    for (j, c) in car.inputs.iter().enumerate() {
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
    let last_ms = (car.inputs.len() - 1) as u64 * 20;
    let end_ms = r.substeps;
    if end_ms != last_ms {
        let c = car.inputs.last().unwrap();
        writeln!(
            script,
            "{}.{:03} {} {} {}",
            end_ms / 1000,
            end_ms % 1000,
            c.throttle,
            c.brake,
            c.steer
        )
        .unwrap();
    }
    let input = tmp_file("n20-20f-replay.input", &script);
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let out = Command::new(crash_sim_bin())
        .current_dir(root)
        .args([
            "--body",
            "scenarios/car_basic.bsn",
            "--track",
            "scenarios/tracks/oval.track",
            "--input",
            &input,
            "--grid-slot",
            "1",
            "--grid-gap",
            "10",
            "--dt",
            &cfg.dt.to_string(),
            "--substeps",
            &cfg.substeps.to_string(),
        ])
        .stdin(Stdio::null())
        .output()
        .expect("run crash-sim");
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let stdout = String::from_utf8(out.stdout).unwrap();
    let lines: Vec<_> = stdout.lines().filter(|l| l.starts_with("lap ")).collect();
    assert_eq!(lines.len(), car.lap_stats.len(), "{stdout}");
    assert!(lines.len() >= 2, "{stdout}");
    for (line, s) in lines.iter().zip(&car.lap_stats) {
        let field = |name: &str| {
            line.split_whitespace()
                .find_map(|f| f.strip_prefix(name))
                .unwrap()
        };
        assert_eq!(field("path_m="), format!("{:.2}", s.path_m));
        assert_eq!(
            field("max_speed_kmh="),
            format!("{:.1}", s.max_speed_mps * 3.6)
        );
        println!("{line}");
    }
}

#[test]
fn lap_stats_parallel_to_laps() {
    let (cfg, body, track) = fixtures();
    for (cars, limit) in [(1, 120.0), (1, 6.0), (3, 120.0)] {
        let mut s = setup(&cfg, &body, &track);
        s.cars = cars;
        s.time_limit_s = limit;
        let r = race::run(&s).unwrap();
        for car in &r.cars {
            assert_eq!(car.lap_stats.len(), car.laps.len());
            if limit == 6.0 {
                assert!(car.lap_stats.is_empty());
            }
        }
    }
}
