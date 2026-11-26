//! Frozen pre-follow geometry: no race replay is needed for these proofs.

use std::fmt::Write as _;
use std::path::PathBuf;

use crash_sim::car_contact::{detect_contacts, ContactCar, ContactKey, ContactSettings};
use crash_sim::multi_car::{CarPlacement, MultiSim};
use crash_sim::race::{place_on_grid, slot_distance};
use crash_sim::settle::{self, CarSetup};
use crash_sim::track::Track;
use crash_sim::vec3::Vec3;
use crash_sim::{Body, Config};

// Private copies of the capture scene and loader; do not include its tests.
fn scene() -> (Config, Body, Track) {
    (
        Config::default(),
        crash_sim::format::parse(include_str!("../scenarios/car_basic.bsn")).unwrap(),
        Track::parse(include_str!("../scenarios/tracks/oval.track")).unwrap(),
    )
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

#[derive(Debug)]
struct Snapshot {
    substep: u64,
    tick: u64,
    prev: [Vec<Vec3>; 2],
    fin: [Vec<Vec3>; 2],
    vel: [Vec<Vec3>; 2],
    broken: [Vec<bool>; 2],
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
    for (c, car) in snapshot.broken.iter().enumerate() {
        let flags: String = car
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
    let found = lines.next().ok_or("missing pre_follow header")?;
    let found_substep = found
        .split(' ')
        .nth(3)
        .ok_or("missing header substep")?
        .parse::<u64>()
        .map_err(|e| format!("invalid header substep: {e}"))?;
    if found_substep != substep {
        return Err(format!(
            "fixture substep mismatch: expected {substep}, found {found_substep}"
        ));
    }
    if found != header {
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
    for (c, slot) in snapshot.broken.iter_mut().enumerate() {
        let prefix = format!("broken {c} ");
        let flags = lines
            .next()
            .and_then(|line| line.strip_prefix(&prefix))
            .ok_or("missing or invalid broken line")?;
        if flags.len() != body.beams.len() || !flags.bytes().all(|b| b == b'0' || b == b'1') {
            return Err(format!("invalid broken flags: car={c}"));
        }
        *slot = flags.bytes().map(|b| b == b'1').collect();
    }
    if lines.next().is_some() || encode(&snapshot) != text {
        return Err("extra lines or non-canonical fixture encoding".into());
    }
    Ok(snapshot)
}

fn load(file: &str, substep: u64, body: &Body, cfg: &Config) -> Snapshot {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/pre_follow")
        .join(file);
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("fixture {}: {e}", path.display()));
    decode(&text, substep, body, cfg)
        .unwrap_or_else(|e| panic!("fixture {}: {e}", path.display()))
}

#[test]
fn snapshot_loader_rejects_mismatches_and_corruption() {
    let (cfg, body, _) = scene();
    let text = include_str!("fixtures/pre_follow/substep_2780.txt");
    assert!(decode(text, 2780, &body, &cfg).is_ok());
    let wrong = include_str!("fixtures/pre_follow/substep_2848.txt");
    let error = decode(wrong, 2780, &body, &cfg)
        .expect_err("wrong fixture must fail before geometry checks");
    assert_eq!(error, "fixture substep mismatch: expected 2780, found 2848");

    // Without the finite guard, canonical NaN would survive the round trip.
    let first = text.lines().nth(1).unwrap();
    let fields: Vec<_> = first.split(' ').collect();
    for bad in ["NaN", "inf", "-inf", "not-a-number"] {
        let mut changed = fields.clone();
        changed[3] = bad;
        let corrupt = text.replacen(first, &changed.join(" "), 1);
        assert!(decode(&corrupt, 2780, &body, &cfg).is_err(), "{bad}");
    }
    for corrupt in [
        String::new(),
        text.replacen("substep 2780", "substep -1", 1),
        text.replacen("substep 2780", "substep 18446744073709551616", 1),
        text.replacen("tick 278", "tick 284", 1),
        text.replacen("main 1e56aae", "main wrong", 1),
        text.replacen("node 0 0 ", "node 0 1 ", 1),
        text.replacen(first, "", 1),
        text.replacen("broken 0 ", "broken 0 x", 1),
        text.replace('\n', "\r\n"),
        text.replace('\n', "\r"),
        text.trim_end().to_owned(),
        text.repeat(body.nodes.len()),
    ] {
        assert!(decode(&corrupt, 2780, &body, &cfg).is_err());
    }
}

fn sweep_views<'a>(
    multi: &'a MultiSim,
    prev: &'a [Vec<Vec3>; 2],
    fin: &'a [Vec<Vec3>; 2],
    vel: &'a mut [Vec<Vec3>; 2],
    broken: &'a [Vec<bool>; 2],
) -> Vec<ContactCar<'a>> {
    vel.iter_mut()
        .enumerate()
        .map(|(c, v)| ContactCar {
            previous_pos: &prev[c],
            pos: &fin[c],
            vel: v,
            mass: &multi.sim(c).mass,
            beams: &multi.sim(c).beams,
            broken: &broken[c],
        })
        .collect()
}

// Diagnostic helpers copied from two_car_sweep.rs, for fixed geometry only.
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

fn slack(p: Vec3, q: Vec3, n: Vec3) -> f64 {
    let value = (p - q).dot(n)
        - (0.25 * n.length()
            + 64.0 * f64::EPSILON * p.length().max(q.length()).max(0.25) * n.length());
    assert!(value.is_finite(), "diagnostic slack={value:?}");
    value
}

fn print_nearest(multi: &MultiSim, fin: &[Vec<Vec3>; 2], broken: &[Vec<bool>; 2]) {
    let mut nearest = Vec::new();
    for (source_car, nodes) in fin.iter().enumerate() {
        let target_car = 1 - source_car;
        let target = multi.sim(target_car);
        for (node, &p) in nodes.iter().enumerate() {
            for (beam, b) in target.beams.iter().enumerate() {
                if broken[target_car][beam] {
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
    assert!(
        nearest.len() >= 5,
        "diagnostic nearest pair count={}",
        nearest.len()
    );
    for (rank, (d, key)) in nearest.iter().take(5).enumerate() {
        eprintln!(
            "sweep nearest rank={} source={}/{} target={}/{} d={d:.9}",
            rank + 1,
            key.source_car,
            key.node,
            key.target_car,
            key.beam
        );
    }
}

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
    // Sampling is diagnostic only, not a certificate or asserted classification.
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

#[test]
fn rear_end_snapshot_is_proved_separated() {
    let (cfg, multi, settings) = sweep_scene();
    let (_, body, _) = scene();
    let Snapshot {
        prev,
        fin,
        mut vel,
        broken,
        ..
    } = load("substep_2780.txt", 2780, &body, &cfg);
    // Omitted replay-only checks/prints: early network hashes, history length
    // and last 30 distances, accumulated contacts/impulses and first contact.
    {
        let views = sweep_views(&multi, &prev, &fin, &mut vel, &broken);
        let contacts = detect_contacts(&views, settings)
            .unwrap_or_else(|e| panic!("substep=2780 captured kernel inputs found={e:?}"));
        let key = ContactKey {
            source_car: 1,
            node: 32,
            target_car: 0,
            beam: 0,
        };
        assert!(
            contacts.iter().all(|contact| contact.key != key),
            "substep=2780 separated key={key:?} found contacts={contacts:?}"
        );
    }
    assert_eq!(multi.sim(0).beams[0].a, 0, "target beam 0 endpoint a");
    assert_eq!(multi.sim(0).beams[0].b, 1, "target beam 0 endpoint b");
    let (p0, a0, b0) = (prev[1][32], prev[0][0], prev[0][1]);
    let (p1, a1, b1) = (fin[1][32], fin[0][0], fin[0][1]);
    let (t0, delta0, d0) = closest(p0, a0, b0);
    let (t1, delta1, d1) = closest(p1, a1, b1);
    // car_contact.rs::prepare's UnsupportedSweep distance band.
    assert!(d0 > 0.25, "old distance={d0:?}");
    assert!(
        0.25 < d1 && d1 <= 0.25 + 2.0 * 0.03125,
        "final distance={d1:?}"
    );
    assert!(
        p0 != p1 || a0 != a1 || b0 != b1,
        "substep=2780 sweep must have motion"
    );
    let n1 = delta1 * (1.0 / d1);
    let n0 = delta0 * (1.0 / d0);
    eprintln!("sweep accepted tick=278 substep=2780 key=1/32/0/0");
    eprintln!("sweep old t={t0:.9} d={d0:.9}");
    eprintln!("sweep final t={t1:.9} d={d1:.9} d_minus_skin={:e}", d1 - 0.25);
    eprintln!("sweep normals dot={:.12}", n0.dot(n1));
    for (name, n) in [("final", n1), ("old", n0)] {
        let mut slacks = Vec::new();
        for (pos, p, a, b) in [("old", p0, a0, b0), ("final", p1, a1, b1)] {
            for (end, q) in [("a", a), ("b", b)] {
                let value = slack(p, q, n);
                eprintln!("sweep slack n={name} pos={pos} end={end} value={value:e}");
                slacks.push(value);
            }
        }
        assert!(
            slacks.iter().any(|&v| v <= 0.0),
            "substep=2780 normal={name} unexpectedly separated, slacks={slacks:?}"
        );
    }
    let endpoints = [(p0, a0), (p0, b0), (p1, a1), (p1, b1)];
    let (lambda, slacks) = blend_candidates(endpoints, n0, n1)
        .into_iter()
        .find(|(_, slacks)| slacks.iter().all(|&value| value > 0.0))
        .expect("substep=2780 missing blended separating plane");
    let min_slack = slacks.into_iter().fold(f64::INFINITY, f64::min);
    eprintln!("sweep blend lambda=<{lambda:.6}> min_slack=<{min_slack:e}>");
    let r = (p1 - p0) - ((a1 - a0) * (1.0 - t1) + (b1 - b0) * t1);
    eprintln!(
        "sweep motion node={:.9} a={:.9} b={:.9} rel_normal={:.9} rel_tangent={:.9}",
        (p1 - p0).length(),
        (a1 - a0).length(),
        (b1 - b0).length(),
        r.dot(n1),
        (r - n1 * r.dot(n1)).length()
    );
    print_nearest(&multi, &fin, &broken);
    print_swept_min([p0, a0, b0, p1, a1, b1]);
}

type Sample = (f64, [Vec3; 3], Vec3);
type Sweep = [[Vec3; 3]; 2];
type Certificate = (u32, usize, f64);

// Copy of separated_pieces: return deepest level, leaf count, minimum slack.
fn split_pieces(ends: &Sweep, lo: Sample, hi: Sample, depth: u32) -> Option<Certificate> {
    fn piece(ends: &Sweep, lo: Sample, hi: Sample, depth: u32) -> Option<Certificate> {
        let c = [
            (lo.1[0], lo.1[1]),
            (lo.1[0], lo.1[2]),
            (hi.1[0], hi.1[1]),
            (hi.1[0], hi.1[2]),
        ];
        for values in [hi.2, lo.2]
            .map(|n| c.map(|(p, q)| slack(p, q, n)))
            .into_iter()
            .chain(blend_candidates(c, lo.2, hi.2).into_iter().map(|(_, v)| v))
        {
            let min = values.into_iter().fold(f64::INFINITY, f64::min);
            if min > 0.0 {
                return Some((6 - depth, 1, min));
            }
        }
        split_pieces(ends, lo, hi, depth)
    }
    if depth == 0 {
        return None;
    }
    let s = (lo.0 + hi.0) * 0.5;
    let at = std::array::from_fn(|i| ends[0][i] + (ends[1][i] - ends[0][i]) * s);
    let (_, delta, d) = closest(at[0], at[1], at[2]);
    if !(d.is_finite() && d > 0.25) {
        return None;
    }
    let mid = (s, at, delta * (1.0 / d));
    let left = piece(ends, lo, mid, depth - 1)?;
    let right = piece(ends, mid, hi, depth - 1)?;
    Some((left.0.max(right.0), left.1 + right.1, left.2.min(right.2)))
}

#[test]
fn tick_284_snapshot_is_proved_separated_by_pieces() {
    let (cfg, multi, settings) = sweep_scene();
    let (_, body, _) = scene();
    let Snapshot {
        prev,
        fin,
        mut vel,
        broken,
        ..
    } = load("substep_2848.txt", 2848, &body, &cfg);
    let i = 2848;
    let views = sweep_views(&multi, &prev, &fin, &mut vel, &broken);
    let contacts = detect_contacts(&views, settings)
        .unwrap_or_else(|e| panic!("substep={i} captured kernel inputs found={e:?}"));
    let key = ContactKey {
        source_car: 0,
        node: 2,
        target_car: 1,
        beam: 225,
    };
    assert!(
        contacts.iter().all(|c| c.key != key),
        "key={key:?} contacts={contacts:?}"
    );
    assert_eq!(detect_contacts(&views, settings), Ok(contacts));
    let beam = multi.sim(1).beams[225];
    let old = [prev[0][2], prev[1][beam.a], prev[1][beam.b]];
    let new = [fin[0][2], fin[1][beam.a], fin[1][beam.b]];
    let (_, delta0, d0) = closest(old[0], old[1], old[2]);
    let (_, delta1, d1) = closest(new[0], new[1], new[2]);
    assert!(d0 > 0.25, "old distance={d0:?}");
    assert!(d1 > 0.25 && d1 <= 0.3125, "final distance={d1:?}");
    let (n0, n1) = (delta0 * (1.0 / d0), delta1 * (1.0 / d1));
    let c = [
        (old[0], old[1]),
        (old[0], old[2]),
        (new[0], new[1]),
        (new[0], new[2]),
    ];
    for n in [n1, n0] {
        assert!(
            c.iter().any(|&(p, q)| slack(p, q, n) <= 0.0),
            "normal={n:?}"
        );
    }
    assert!(!blend_candidates(c, n0, n1)
        .iter()
        .any(|(_, v)| v.iter().all(|&s| s > 0.0)));
    let (depth, leaves, min_slack) =
        split_pieces(&[old, new], (0.0, old, n0), (1.0, new, n1), 6)
            .expect("substep=2848 missing piece certificate");
    eprintln!(
        "sweep pieces tick=284 substep=2848 key=0/2/1/225 depth={depth} leaves={leaves} min_slack={min_slack:e}"
    );
}
