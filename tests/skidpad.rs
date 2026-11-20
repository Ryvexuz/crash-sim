//! The skidpad (nb-44, docs/STAGE7.md "The skidpad" and check 1.3): steady cornering on a 30 m
//! circle, the lateral acceleration at the grip limit, and how it moves with grip.
//!
//! `scenarios/skidpad.bsn` is `car_basic.bsn` byte for byte plus comments (flat ground, no
//! walls); `scenarios/skidpad.input` holds one steer value (kinematic radius 30 m) and rising
//! throttle levels, each held the same time of at least 20 s, brake 0. The method is fixed in
//! STAGE7.md before any tuning, and is measured from the motion of the car position only (the
//! chassis centre of mass), never from the tyre model's own forces:
//! - every substep: v, the horizontal speed of the car position, and psi, the direction of its
//!   horizontal velocity (unwrapped);
//! - 0.1 s blocks: v_b the mean of v over the block's substeps, r_b = (psi at the block's end -
//!   psi at its start) / 0.1 s, rho_b = v_b / r_b;
//! - a level's steady window is its last 5 s (50 blocks); it is steady iff max - min <= 2% of the
//!   mean for each of v_b, r_b and rho_b, and the body slip angle (chassis heading, `rear` to
//!   `front` centroid, against the velocity direction) stays below 10 degrees at every substep;
//! - a_g = mean(v_b) x mean(r_b) / g; `limit_g` is the largest a_g of the steady levels.
//!
//! The same run is repeated on two in-memory copies of the car with every tyre's mu x0.8 and
//! x1.2 (no extra files, so the copies cannot be tuned separately).

use crash_sim::format::{parse, Body};
use crash_sim::input::Script;
use crash_sim::settle::{car_sim, CarSetup};
use crash_sim::vec3::Vec3;
use crash_sim::{Config, Sim};
use std::sync::OnceLock;

/// One `advance` is one 1 ms substep, as in tests/drive.rs.
const H: f64 = 0.001;
/// Substeps per 0.1 s block.
const BLOCK: usize = 100;
/// Blocks in a level's steady window (5 s).
const WINDOW_BLOCKS: usize = 50;
/// Each throttle level is held at least this long (s).
const MIN_HOLD_S: f64 = 20.0;
/// Steadiness: max - min of v_b, r_b and rho_b at most this fraction of their mean.
const STEADY_TOL: f64 = 0.02;
/// Steadiness: the body slip angle stays below this (degrees); above it after the limit = spin.
const MAX_SLIP_DEG: f64 = 10.0;
/// The skidpad radius (m) and the tolerance of the kinematic radius.
const PAD_RADIUS: f64 = 30.0;
const RADIUS_TOL: f64 = 0.01;
/// docs/STAGE7.md, "Target `limit_g` 0.83 to 0.93 g" (*sourced*), check 1.3 (a).
const LIMIT_MIN_G: f64 = 0.830;
const LIMIT_MAX_G: f64 = 0.930;
/// Check 1.3 (b): the friction ceiling, mu plus the 2% steadiness tolerance.
const CEILING_FACTOR: f64 = 1.02;
/// Check 1.3 (c): the grip variants and the bands of their ratios to the x1.0 limit.
const GRIP_LOW: f64 = 0.8;
const GRIP_HIGH: f64 = 1.2;
const LOW_RATIO: (f64, f64) = (0.72, 0.88);
const HIGH_RATIO: (f64, f64) = (1.08, 1.32);
/// Check 1.3 (d): steady levels at least, and levels after the limit level at least.
const MIN_STEADY_LEVELS: usize = 5;
const LEVELS_AFTER_LIMIT: usize = 2;

fn scenario(name: &str) -> String {
    let path = format!("{}/scenarios/{name}", env!("CARGO_MANIFEST_DIR"));
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"))
}

fn body(name: &str) -> Body {
    parse(&scenario(name)).unwrap_or_else(|e| panic!("{name}: {e}"))
}

fn script() -> Script {
    Script::parse(&scenario("skidpad.input")).unwrap_or_else(|e| panic!("skidpad.input: {e}"))
}

/// Round to 3 decimals, as the `skidpad` lines print and check 1.3 reads them.
fn r3(x: f64) -> f64 {
    (x * 1000.0).round() / 1000.0
}

fn wrap(a: f64) -> f64 {
    let tau = std::f64::consts::TAU;
    let mut x = a % tau;
    if x > std::f64::consts::PI {
        x -= tau;
    } else if x <= -std::f64::consts::PI {
        x += tau;
    }
    x
}

fn node_indices(b: &Body, ids: &[u32]) -> Vec<usize> {
    ids.iter()
        .map(|id| {
            b.node_index(*id)
                .unwrap_or_else(|| panic!("node {id} is missing"))
        })
        .collect()
}

fn group(b: &Body, name: &str) -> Vec<usize> {
    let g = b
        .groups
        .iter()
        .find(|g| g.name == name)
        .unwrap_or_else(|| panic!("the car has no group `{name}`"));
    node_indices(b, &g.nodes)
}

fn centroid(pos: &[Vec3], ix: &[usize]) -> Vec3 {
    let mut c = Vec3::ZERO;
    for &i in ix {
        c += pos[i];
    }
    c * (1.0 / ix.len() as f64)
}

/// Wheelbase (m): horizontal distance between the mean mount centroid of the steerable wheels
/// and that of the others, from the file's node positions.
fn wheelbase(b: &Body) -> f64 {
    let pos: Vec<Vec3> = b.nodes.iter().map(|n| n.pos).collect();
    let axle = |steerable: bool| {
        let cs: Vec<Vec3> = b
            .wheels
            .iter()
            .filter(|w| w.steerable == steerable)
            .map(|w| centroid(&pos, &node_indices(b, &w.mount)))
            .collect();
        assert!(
            !cs.is_empty(),
            "the car needs steerable and non-steerable wheels"
        );
        let mut c = Vec3::ZERO;
        for x in &cs {
            c += *x;
        }
        c * (1.0 / cs.len() as f64)
    };
    let d = axle(true) - axle(false);
    (d.x * d.x + d.z * d.z).sqrt()
}

/// The one mu of every tyre of `b` (the ceiling of check 1.3 (b) needs a single value).
fn tyre_mu(b: &Body) -> f64 {
    let mu = b.tyres[0].mu;
    assert!(
        b.tyres.iter().all(|t| t.mu == mu),
        "the tyres have different mu"
    );
    mu
}

/// The script's levels: (throttle, steer) per line, the hold time D shared by every level, and
/// the steer value. Panics unless the rules of `skidpad_input_holds_a_30_m_kinematic_circle`
/// hold (one steer, brake 0, throttle rising, uniform holds of at least 20 s from t = 0).
struct Levels {
    throttle: Vec<f64>,
    hold_s: f64,
    steer: f64,
}

fn levels(s: &Script) -> Levels {
    let lines = s.lines();
    assert!(lines.len() >= 2, "skidpad.input needs at least two levels");
    assert_eq!(lines[0].t, 0.0, "the first level starts at t = 0");
    let steer = lines[0].steer;
    let hold_s = lines[1].t - lines[0].t;
    assert!(
        hold_s >= MIN_HOLD_S,
        "levels are held {hold_s} s, under {MIN_HOLD_S} s"
    );
    for (k, l) in lines.iter().enumerate() {
        assert_eq!(
            l.steer, steer,
            "line {}: one steer value for the whole file",
            l.number
        );
        assert_eq!(l.brake, 0.0, "line {}: brake 0 throughout", l.number);
        assert!(
            (l.t - k as f64 * hold_s).abs() <= 1e-9,
            "line {}: levels are held the same {hold_s} s each (t = {} expected)",
            l.number,
            k as f64 * hold_s
        );
        if k > 0 {
            assert!(
                l.throttle > lines[k - 1].throttle,
                "line {}: throttle levels rise",
                l.number
            );
        }
    }
    Levels {
        throttle: lines.iter().map(|l| l.throttle).collect(),
        hold_s,
        steer,
    }
}

/// What one level measured over its steady window.
#[derive(Clone, Debug)]
struct Level {
    throttle: f64,
    steady: bool,
    speed_kmh: f64,
    radius_m: f64,
    a_g: f64,
}

/// One run of the skidpad on the car with every tyre's mu x `scale`.
#[derive(Clone, Debug)]
struct Run {
    scale: f64,
    levels: Vec<Level>,
    /// 1-based level of the largest steady a_g, None with no steady level.
    limit_level: Option<usize>,
    limit_g: f64,
    steady_levels: usize,
    oversteer: bool,
}

fn spread_ok(x: &[f64]) -> bool {
    let mean = x.iter().sum::<f64>() / x.len() as f64;
    let max = x.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let min = x.iter().cloned().fold(f64::INFINITY, f64::min);
    mean.is_finite() && mean != 0.0 && max - min <= STEADY_TOL * mean.abs()
}

fn mean(x: &[f64]) -> f64 {
    x.iter().sum::<f64>() / x.len() as f64
}

fn run(base: &Body, scale: f64) -> Run {
    let mut car = base.clone();
    for t in &mut car.tyres {
        t.mu *= scale;
    }
    let s = script();
    let lv = levels(&s);
    let k_levels = lv.throttle.len();
    let per_level = (lv.hold_s / H).round() as usize;
    let total = per_level * k_levels;
    let cfg = Config {
        duration: total as f64 * H,
        dt: H,
        substeps: 1,
        ..Config::default()
    };
    let setup = CarSetup {
        script: Some(s.clone()),
        until_script_end: false,
        grip_frac: None,
    };
    let mut sim: Sim = car_sim(cfg, &car, &setup).unwrap_or_else(|e| panic!("car_sim: {e}"));
    let g = sim.gravity_vector().length();
    let (front, rear) = (group(&car, "front"), group(&car, "rear"));

    // State j is the state at time j x H (j = 0 the start): v, unwrapped psi and |slip|.
    let mut v = Vec::with_capacity(total + 1);
    let mut psi = Vec::with_capacity(total + 1);
    let mut slip = Vec::with_capacity(total + 1);
    let mut last_psi: Option<f64> = None;
    let mut record = |sim: &Sim, v: &mut Vec<f64>, psi: &mut Vec<f64>, slip: &mut Vec<f64>| {
        let m: f64 = sim.mass.iter().sum();
        let mut u = Vec3::ZERO;
        for (vi, mi) in sim.vel.iter().zip(&sim.mass) {
            u += *vi * *mi;
        }
        let u = u * (1.0 / m);
        let dir = u.z.atan2(u.x);
        let p = match last_psi {
            None => dir,
            Some(prev) => prev + wrap(dir - prev),
        };
        last_psi = Some(p);
        let hd = centroid(&sim.pos, &front) - centroid(&sim.pos, &rear);
        let heading = hd.z.atan2(hd.x);
        v.push((u.x * u.x + u.z * u.z).sqrt());
        psi.push(p);
        slip.push(wrap(heading - dir).abs().to_degrees());
    };
    record(&sim, &mut v, &mut psi, &mut slip);
    for i in 1..=total {
        s.apply(&mut sim, (i - 1) as f64 * H, H * 1e-6).unwrap();
        sim.advance();
        sim.check()
            .unwrap_or_else(|e| panic!("grip x{scale}: unstable at t={}: {e}", i as f64 * H));
        record(&sim, &mut v, &mut psi, &mut slip);
    }

    let mut levels = Vec::with_capacity(k_levels);
    for (k, &throttle) in lv.throttle.iter().enumerate() {
        let end = (k + 1) * per_level;
        let start = end - WINDOW_BLOCKS * BLOCK;
        let mut vb = Vec::with_capacity(WINDOW_BLOCKS);
        let mut rb = Vec::with_capacity(WINDOW_BLOCKS);
        let mut rhob = Vec::with_capacity(WINDOW_BLOCKS);
        for b in 0..WINDOW_BLOCKS {
            let (j0, j1) = (start + b * BLOCK, start + (b + 1) * BLOCK);
            let vm = mean(&v[j0 + 1..=j1]);
            let r = (psi[j1] - psi[j0]) / (BLOCK as f64 * H);
            vb.push(vm);
            rb.push(r);
            rhob.push(vm / r);
        }
        let slip_ok = slip[start + 1..=end].iter().all(|a| *a < MAX_SLIP_DEG);
        let steady = slip_ok && spread_ok(&vb) && spread_ok(&rb) && spread_ok(&rhob);
        levels.push(Level {
            throttle,
            steady,
            speed_kmh: mean(&vb) * 3.6,
            radius_m: mean(&rhob),
            a_g: mean(&vb) * mean(&rb) / g,
        });
    }
    let mut limit_level = None;
    let mut limit_g = f64::NEG_INFINITY;
    for (k, l) in levels.iter().enumerate() {
        if l.steady && l.a_g > limit_g {
            limit_g = l.a_g;
            limit_level = Some(k + 1);
        }
    }
    let steady_levels = levels.iter().filter(|l| l.steady).count();
    let oversteer = match limit_level {
        Some(n) => slip[n * per_level + 1..].iter().any(|a| *a > MAX_SLIP_DEG),
        None => false,
    };
    Run {
        scale,
        levels,
        limit_level,
        limit_g,
        steady_levels,
        oversteer,
    }
}

fn render(r: &Run) -> String {
    let mut out = String::new();
    for (k, l) in r.levels.iter().enumerate() {
        out.push_str(&format!(
            "skidpad grip_scale={:.1} level={} throttle={:.4} steady={} speed_kmh={:.1} radius_m={:.2} a_g={:.3}\n",
            r.scale,
            k + 1,
            l.throttle,
            if l.steady { "yes" } else { "no" },
            l.speed_kmh,
            l.radius_m,
            l.a_g
        ));
    }
    out.push_str(&format!(
        "skidpad grip_scale={:.1} limit_g={:.3} limit_level={} steady_levels={} limit_balance={}\n",
        r.scale,
        r.limit_g,
        r.limit_level.map_or("none".to_string(), |n| n.to_string()),
        r.steady_levels,
        if r.oversteer {
            "oversteer"
        } else {
            "understeer"
        }
    ));
    out
}

struct Results {
    mu: f64,
    runs: [Run; 3],
}

/// The three runs, measured once and printed in the order of STAGE7.md (header, x1.0, x0.8,
/// x1.2) by whichever test asks first.
fn results() -> &'static Results {
    static R: OnceLock<Results> = OnceLock::new();
    R.get_or_init(|| {
        let car = body("skidpad.bsn");
        let mu = tyre_mu(&car);
        let lv = levels(&script());
        let max_steer = car
            .controls
            .as_ref()
            .expect("controls")
            .max_steer_deg
            .to_radians();
        let radius = wheelbase(&car) / (lv.steer * max_steer).tan();
        println!(
            "skidpad mu={mu:.4} steer_frac={:.4} radius_kinematic_m={radius:.2} levels={}",
            lv.steer,
            lv.throttle.len()
        );
        // The three runs are independent (each builds its own sim from its own copy of the
        // car), so they run on three threads; they are printed afterwards in the fixed order.
        let runs: Vec<Run> = std::thread::scope(|s| {
            let handles: Vec<_> = [1.0, GRIP_LOW, GRIP_HIGH]
                .into_iter()
                .map(|scale| {
                    let car = &car;
                    s.spawn(move || run(car, scale))
                })
                .collect();
            handles
                .into_iter()
                .map(|h| {
                    h.join()
                        .unwrap_or_else(|_| panic!("a skidpad run panicked"))
                })
                .collect()
        });
        for r in &runs {
            print!("{}", render(r));
        }
        let runs: [Run; 3] = runs.try_into().unwrap();
        Results { mu, runs }
    })
}

/// Check 1.3 (d) for one run.
fn assert_run_shape(r: &Run) {
    let n = r
        .limit_level
        .unwrap_or_else(|| panic!("grip x{}: no steady level", r.scale));
    assert!(
        r.levels[n - 1].steady,
        "grip x{}: the limit level is not steady",
        r.scale
    );
    assert!(
        n + LEVELS_AFTER_LIMIT <= r.levels.len(),
        "grip x{}: limit_level {n} is not at least {LEVELS_AFTER_LIMIT} levels before the last ({})",
        r.scale,
        r.levels.len()
    );
    assert!(
        r.steady_levels >= MIN_STEADY_LEVELS,
        "grip x{}: {} steady levels, under {MIN_STEADY_LEVELS}",
        r.scale,
        r.steady_levels
    );
}

#[test]
fn skidpad_car_is_car_basic() {
    let car = scenario("car_basic.bsn");
    let pad = scenario("skidpad.bsn");
    assert!(car.ends_with('\n'), "car_basic.bsn must end with a newline");
    assert!(
        pad.starts_with(&car),
        "skidpad.bsn does not start with car_basic.bsn byte for byte"
    );
    let rest = &pad[car.len()..];
    assert!(
        !rest.is_empty(),
        "skidpad.bsn adds its comment after the car"
    );
    for line in rest.lines() {
        assert!(
            line.starts_with('#'),
            "skidpad.bsn adds a line that is not a comment: {line:?}"
        );
    }
    let (c, p) = (body("car_basic.bsn"), body("skidpad.bsn"));
    assert!(p.planes.is_empty(), "skidpad.bsn has a wall");
    assert_eq!(c, p, "skidpad.bsn parses to a different car");
}

#[test]
fn skidpad_input_holds_a_30_m_kinematic_circle() {
    let car = body("skidpad.bsn");
    let lv = levels(&script());
    let max_steer = car
        .controls
        .as_ref()
        .expect("controls")
        .max_steer_deg
        .to_radians();
    let radius = wheelbase(&car) / (lv.steer * max_steer).tan();
    assert!(
        (radius - PAD_RADIUS).abs() <= RADIUS_TOL * PAD_RADIUS,
        "kinematic radius {radius} m is not within 1% of {PAD_RADIUS} m"
    );
}

#[test]
fn skidpad_limit_in_road_tyre_range() {
    let res = results();
    let r = &res.runs[0];
    assert_run_shape(r);
    let lim = r3(r.limit_g);
    assert!(
        (LIMIT_MIN_G..=LIMIT_MAX_G).contains(&lim),
        "limit_g {lim} is outside {LIMIT_MIN_G} to {LIMIT_MAX_G} g"
    );
    assert!(
        lim <= CEILING_FACTOR * res.mu,
        "limit_g {lim} is above 1.02 x mu = {}",
        CEILING_FACTOR * res.mu
    );
}

#[test]
fn skidpad_limit_follows_grip() {
    let res = results();
    for r in &res.runs {
        assert_run_shape(r);
        let lim = r3(r.limit_g);
        let ceiling = CEILING_FACTOR * r.scale * res.mu;
        assert!(
            lim <= ceiling,
            "grip x{}: limit {lim} is above the ceiling {ceiling}",
            r.scale
        );
    }
    let (base, low, high) = (
        r3(res.runs[0].limit_g),
        r3(res.runs[1].limit_g),
        r3(res.runs[2].limit_g),
    );
    assert!(
        low < base && base < high,
        "limits are not ordered: {low} < {base} < {high}"
    );
    let (ql, qh) = (low / base, high / base);
    assert!(
        (LOW_RATIO.0..=LOW_RATIO.1).contains(&ql),
        "limit(0.8) / limit_g = {ql} is outside {LOW_RATIO:?}"
    );
    assert!(
        (HIGH_RATIO.0..=HIGH_RATIO.1).contains(&qh),
        "limit(1.2) / limit_g = {qh} is outside {HIGH_RATIO:?}"
    );
}
