//! Capture on main 1e56aae before position follow; ordinary tests only load files.

use std::fmt::Write as _;
use std::path::PathBuf;

use crash_sim::car_contact::ContactSettings;
use crash_sim::input::Script;
use crash_sim::multi_car::{CarPlacement, MultiSim};
use crash_sim::net::session::Input;
use crash_sim::race::lockstep::controls_of;
use crash_sim::race::{place_on_grid, slot_distance};
use crash_sim::settle::{self, CarSetup};
use crash_sim::track::Track;
use crash_sim::vec3::Vec3;
use crash_sim::{Body, Config};

// Copied from two_car_sweep.rs, not shared with the changing replay tests.
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
                // Same exact thousandth round trip as the reference replay.
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

struct Snapshot {
    substep: u64,
    tick: u64,
    prev: [Vec<Vec3>; 2],
    fin: [Vec<Vec3>; 2],
    vel: [Vec<Vec3>; 2],
    broken: [Vec<bool>; 2],
}

fn fixture_path(substep: u64) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/pre_follow")
        .join(format!("substep_{substep}.txt"))
}

fn encode(snapshot: &Snapshot) -> String {
    let mut text = format!(
        "pre_follow 1 substep {} tick {} main 1e56aae\n",
        snapshot.substep, snapshot.tick
    );
    for c in 0..2 {
        for i in 0..snapshot.prev[c].len() {
            let (p, f, v) = (snapshot.prev[c][i], snapshot.fin[c][i], snapshot.vel[c][i]);
            writeln!(
                text,
                "node {c} {i} {:?} {:?} {:?} {:?} {:?} {:?} {:?} {:?} {:?}",
                p.x, p.y, p.z, f.x, f.y, f.z, v.x, v.y, v.z
            )
            .unwrap();
        }
    }
    for c in 0..2 {
        let flags: String = snapshot.broken[c]
            .iter()
            .map(|&broken| if broken { '1' } else { '0' })
            .collect();
        writeln!(text, "broken {c} {flags}").unwrap();
    }
    text
}

// Strict, scene-sized parser: no allocation depends on counts in the input.
fn decode(text: &str, substep: u64, body: &Body, cfg: &Config) -> Result<Snapshot, String> {
    if ![2780, 2848].contains(&substep) || cfg.substeps == 0 {
        return Err("unsupported capture substep or zero scene substeps".into());
    }
    let tick = substep / u64::from(cfg.substeps);
    let header = format!("pre_follow 1 substep {substep} tick {tick} main 1e56aae");
    let mut lines = text.lines();
    if lines.next() != Some(header.as_str()) {
        return Err("invalid pre_follow header".into());
    }
    let mut snapshot = Snapshot {
        substep,
        tick,
        prev: std::array::from_fn(|_| Vec::new()),
        fin: std::array::from_fn(|_| Vec::new()),
        vel: std::array::from_fn(|_| Vec::new()),
        broken: std::array::from_fn(|_| Vec::new()),
    };
    for c in 0..2 {
        for i in 0..body.nodes.len() {
            let line = lines.next().ok_or("missing node line")?;
            let mut fields = line.split(' ');
            let prefix = format!("node {c} {i} ");
            if !line.starts_with(&prefix) {
                return Err(format!("invalid node order: car={c} node={i}"));
            }
            for _ in 0..3 {
                fields.next();
            }
            let mut values = [0.0; 9];
            for value in &mut values {
                *value = fields
                    .next()
                    .ok_or("missing node coordinate")?
                    .parse::<f64>()
                    .map_err(|e| format!("invalid node coordinate: {e}"))?;
                if !value.is_finite() {
                    return Err(format!("non-finite node coordinate: car={c} node={i}"));
                }
            }
            if fields.next().is_some() {
                return Err("extra node coordinate".into());
            }
            snapshot.prev[c].push(Vec3::new(values[0], values[1], values[2]));
            snapshot.fin[c].push(Vec3::new(values[3], values[4], values[5]));
            snapshot.vel[c].push(Vec3::new(values[6], values[7], values[8]));
        }
    }
    for c in 0..2 {
        let prefix = format!("broken {c} ");
        let flags = lines
            .next()
            .and_then(|line| line.strip_prefix(&prefix))
            .ok_or("missing or invalid broken line")?;
        if flags.len() != body.beams.len() || !flags.bytes().all(|b| b == b'0' || b == b'1') {
            return Err(format!("invalid broken flags: car={c}"));
        }
        snapshot.broken[c] = flags.bytes().map(|b| b == b'1').collect();
    }
    if lines.next().is_some() || encode(&snapshot) != text {
        return Err("extra lines or non-canonical fixture encoding".into());
    }
    Ok(snapshot)
}

fn load(substep: u64, body: &Body, cfg: &Config) -> (String, Snapshot) {
    let path = fixture_path(substep);
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("fixture {}: {e}; run ignored capture first", path.display()));
    let snapshot = decode(&text, substep, body, cfg)
        .unwrap_or_else(|e| panic!("fixture {}: {e}", path.display()));
    (text, snapshot)
}

#[test]
#[ignore]
fn capture_pre_follow_fixtures() {
    let (cfg, mut multi, _) = sweep_scene();
    let scripts = scripts();
    // Copied from tick_284_sweep_is_proved_separated_by_pieces. Its flattened
    // loop takes prev/fin at the same boundaries as the tick/substep 2780 loop.
    for i in 0..=2848 {
        let inputs = inputs_at(&scripts, i / u64::from(cfg.substeps), &cfg);
        let prev: [Vec<Vec3>; 2] = std::array::from_fn(|c| multi.sim(c).pos.clone());
        multi
            .advance(&[controls_of(inputs[0]), controls_of(inputs[1])])
            .unwrap_or_else(|e| panic!("substep={i} expected Ok, found={e:?}"));
        if i != 2780 && i != 2848 {
            continue;
        }
        // Velocities and broken flags are read immediately after advance, just
        // where the original tests build sweep_views, before any further step.
        let snapshot = Snapshot {
            substep: i,
            tick: i / u64::from(cfg.substeps),
            prev,
            fin: std::array::from_fn(|c| multi.sim(c).pos.clone()),
            vel: std::array::from_fn(|c| multi.sim(c).vel.clone()),
            broken: std::array::from_fn(|c| multi.sim(c).broken.clone()),
        };
        let text = encode(&snapshot);
        let (_, body, _) = scene();
        decode(&text, i, &body, &cfg).expect("capture must satisfy fixture format");
        let path = fixture_path(i);
        std::fs::create_dir_all(path.parent().unwrap()).expect("create fixture directory");
        std::fs::write(&path, text)
            .unwrap_or_else(|e| panic!("write fixture {}: {e}", path.display()));
        eprintln!("captured substep={i} path={}", path.display());
    }
}

#[test]
fn pre_follow_fixtures_load_and_round_trip() {
    let (cfg, body, _) = scene();
    assert!(!body.nodes.is_empty() && !body.beams.is_empty());
    for substep in [2780, 2848] {
        let (text, snapshot) = load(substep, &body, &cfg);
        for c in 0..2 {
            assert_eq!(snapshot.prev[c].len(), body.nodes.len());
            assert_eq!(snapshot.fin[c].len(), body.nodes.len());
            assert_eq!(snapshot.vel[c].len(), body.nodes.len());
            assert_eq!(snapshot.broken[c].len(), body.beams.len());
        }
        assert_eq!(encode(&snapshot).as_bytes(), text.as_bytes());
        eprintln!("round trip substep={substep} bytes={}", text.len());

        // Behavioural negative cases: disabling the finite check would accept
        // canonical NaN, and disabling structural checks would accept damage.
        let first = text.lines().nth(1).unwrap();
        let fields: Vec<_> = first.split(' ').collect();
        for bad in ["NaN", "inf", "-inf", "not-a-number"] {
            let mut changed = fields.clone();
            changed[3] = bad;
            let corrupt = text.replacen(first, &changed.join(" "), 1);
            assert!(decode(&corrupt, substep, &body, &cfg).is_err(), "{bad}");
        }
        for corrupt in [
            String::new(),
            text.replacen("main 1e56aae", "main wrong", 1),
            text.replacen("node 0 0 ", "node 0 1 ", 1),
            text.replacen(first, "", 1),
            text.replacen("broken 0 ", "broken 0 x", 1),
            text.replace('\n', "\r\n"),
            format!("{text}{text}"),
            text.trim_end().to_owned(),
        ] {
            assert!(decode(&corrupt, substep, &body, &cfg).is_err());
        }
    }
}

// Same closest-point calculation as two_car_sweep.rs, only on fixed geometry.
fn distance(p: Vec3, a: Vec3, b: Vec3) -> f64 {
    let ab = b - a;
    let squared = ab.dot(ab);
    assert!(squared.is_finite() && squared > 0.0);
    let t = ((p - a).dot(ab) / squared).clamp(0.0, 1.0);
    let d = ((p - a) - ab * t).length();
    assert!(t.is_finite() && d.is_finite());
    d
}

#[test]
fn pre_follow_fixture_reproduces_capture_distances() {
    let (cfg, body, _) = scene();
    let (_, multi, _) = sweep_scene();
    for (substep, source, node, target, beam_index) in
        [(2780, 1, 32, 0, 0), (2848, 0, 2, 1, 225)]
    {
        let (_, snapshot) = load(substep, &body, &cfg);
        let beam = multi.sim(target).beams[beam_index];
        if substep == 2780 {
            assert_eq!((beam.a, beam.b), (0, 1));
        }
        assert!(!snapshot.broken[target][beam_index]);
        let [old, fin] = [&snapshot.prev, &snapshot.fin].map(|pos| {
            distance(pos[source][node], pos[target][beam.a], pos[target][beam.b])
        });
        eprintln!("substep={substep} prev_distance={old:?} fin_distance={fin:?}");
        // Exact UnsupportedSweep band from the diagnosis tests, not a tolerance.
        assert!(old > 0.25, "substep={substep} old distance={old:?}");
        assert!(
            0.25 < fin && fin <= 0.25 + 2.0 * 0.03125,
            "substep={substep} final distance={fin:?}"
        );
    }
}
