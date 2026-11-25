//! Report-only normal barrier force for the fixed wall scenario.
//! Physics inputs are the repository body and frozen flags, not caller overrides.
use crash_sim::collide::{self, Plane};
use crash_sim::format::{self, Body};
use crash_sim::solver::{Solver, State, SymplecticEuler};
use crash_sim::statehash::Hasher;
use crash_sim::vec3::Vec3;
use crash_sim::vehicle::{Substep, AIR_DENSITY};
use crash_sim::{pulse, step_count, Config, Sim};
use std::io::Write;
use std::sync::OnceLock;

#[derive(Clone, Debug)]
pub struct WallForce {
    pub h: f64,
    pub contact_k: usize,
    pub node_n: Vec<f64>,
    pub tyre_n: Vec<f64>,
    pub raw_n: Vec<f64>,
    pub cfc60_n: Vec<f64>,
    pub crush_m: Vec<f64>,
    pub momentum: Momentum,
    pub ground_node_substeps: usize,
    pub ground_tyre_substeps: usize,
    pub state_hashes: Vec<u64>,
}

#[derive(Clone, Debug)]
pub struct Momentum {
    pub p0_x: f64,
    pub p_end_x: f64,
    pub wall_x: f64,
    pub ground_node_x: f64,
    pub aero_x: f64,
    pub residual: f64,
    pub tolerance: f64,
}

#[derive(Debug)]
pub struct Summary {
    pub peak_raw: (f64, usize),
    pub peak_cfc60: (f64, usize),
    pub node_ns: f64,
    pub tyre_ns: f64,
    pub total_ns: f64,
    pub cfc60_ns: f64,
}

#[derive(Clone, Debug)]
pub enum WallError {
    Usage(String),
    Input(String),
    Unstable(String),
    Mismatch(String),
    NoContact,
    Momentum(String),
    Output(String),
}

impl std::fmt::Display for WallError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Usage(s) => write!(f, "{s}"),
            Self::Input(s) => write!(f, "input: {s}"),
            Self::Unstable(s) => write!(f, "unstable: {s}"),
            Self::Mismatch(s) => write!(f, "mismatch: {s}"),
            Self::NoContact => write!(f, "no wall contact"),
            Self::Momentum(s) => write!(f, "momentum: {s}"),
            Self::Output(s) => write!(f, "output: {s}"),
        }
    }
}

impl std::error::Error for WallError {}

pub struct RunInput {
    pub cfg: Config,
    pub crush_groups: (String, String),
    pub h: f64,
    pub n: usize,
}

/// Parse only the authoritative frozen line. Other frozen flags remain untouched.
pub fn parse_run_flags(text: &str) -> Result<RunInput, WallError> {
    let bad = |s: &str| WallError::Input(s.into());
    let mut lines = text
        .lines()
        .filter_map(|l| l.strip_prefix("frozen: run_flags = "));
    let line = lines.next().ok_or_else(|| bad("missing frozen run_flags"))?;
    if lines.next().is_some() {
        return Err(bad("duplicate frozen run_flags"));
    }
    if line
        .chars()
        .any(|c| c.is_control() || matches!(c, '\u{2028}' | '\u{2029}'))
    {
        return Err(bad("control character in frozen run_flags"));
    }
    let words: Vec<_> = line.split_whitespace().collect();
    let value = |flag: &str| -> Result<&str, WallError> {
        let mut matches = words.iter().enumerate().filter(|(_, w)| **w == flag);
        let (i, _) = matches
            .next()
            .ok_or_else(|| bad(&format!("missing {flag}")))?;
        if matches.next().is_some() {
            return Err(bad(&format!("duplicate {flag}")));
        }
        words
            .get(i + 1)
            .copied()
            .filter(|w| !w.starts_with("--"))
            .ok_or_else(|| bad(&format!("missing value for {flag}")))
    };
    let number = |flag: &str| -> Result<f64, WallError> {
        let token = value(flag)?;
        let v: f64 = token.parse().map_err(|_| bad(&format!("invalid {flag}")))?;
        if !v.is_finite() {
            return Err(bad(&format!("non-finite {flag}")));
        }
        let nonzero_mantissa = token
            .bytes()
            .take_while(|&b| b != b'e' && b != b'E')
            .any(|b| matches!(b, b'1'..=b'9'));
        if v == 0.0 && nonzero_mantissa {
            return Err(bad(&format!("underflow in {flag}")));
        }
        Ok(v)
    };
    let cfg = Config {
        gravity: number("--gravity")?,
        duration: number("--duration")?,
        dt: number("--dt")?,
        substeps: value("--substeps")?
            .parse()
            .map_err(|_| bad("invalid --substeps"))?,
        ..Config::default()
    };
    if cfg.gravity != 0.0 {
        return Err(bad("the momentum identity requires zero gravity"));
    }
    if cfg.duration <= 0.0 || cfg.dt <= 0.0 || cfg.substeps == 0 {
        return Err(bad("duration, dt and substeps must be positive"));
    }
    let h = cfg.dt / cfg.substeps as f64;
    let count = cfg.duration / cfg.dt * (1.0 + 1e-9);
    if !(h.is_finite() && h > 0.0 && (1.0 / h).is_finite())
        || !count.is_finite()
        || count < 1.0
        || count >= u64::MAX as f64
    {
        return Err(bad("unrepresentable substep or step count"));
    }
    let n = step_count(&cfg)
        .checked_mul(u64::from(cfg.substeps))
        .and_then(|n| usize::try_from(n).ok())
        .filter(|&n| n <= isize::MAX as usize / std::mem::size_of::<f64>())
        .ok_or_else(|| bad("substep count overflow"))?;
    let groups = value("--pulse-crush-groups")?;
    let (rear, front) = groups
        .split_once(',')
        .filter(|(a, b)| !a.is_empty() && !b.is_empty() && !b.contains(','))
        .ok_or_else(|| bad("invalid --pulse-crush-groups"))?;
    pulse::cfc_filter(&[], h, 60.0).map_err(|e| bad(&e.to_string()))?;
    Ok(RunInput {
        cfg,
        crush_groups: (rear.into(), front.into()),
        h,
        n,
    })
}

fn load_input() -> Result<(RunInput, Body), WallError> {
    let read = |path: &str| {
        std::fs::read_to_string(path).map_err(|e| WallError::Input(format!("{path}: {e}")))
    };
    let input = parse_run_flags(&read("docs/STAGE6-M4-M5.md")?)?;
    let body = format::parse(&read("scenarios/validation_wall.bsn")?)
        .map_err(|e| WallError::Input(e.to_string()))?;
    validate_body(&body)?;
    Ok((input, body))
}

pub fn validate_body(body: &Body) -> Result<(), WallError> {
    if let Some(m) = body.materials.iter().find(|m| m.break_strain.is_some()) {
        return Err(WallError::Input(format!(
            "material {} has break_strain; the split run has no break pass",
            m.name
        )));
    }
    if !body.tets.is_empty() {
        return Err(WallError::Input(
            "the split run does not simulate tets".into(),
        ));
    }
    if body.wheels.is_empty() {
        return Err(WallError::Input("the measurement requires wheels".into()));
    }
    if body.planes.len() != 1 {
        return Err(WallError::Input("expected ground plus one wall".into()));
    }
    Ok(())
}

fn group(body: &Body, name: &str) -> Result<Vec<usize>, WallError> {
    let g = body
        .groups
        .iter()
        .find(|g| g.name == name)
        .filter(|g| !g.nodes.is_empty())
        .ok_or_else(|| WallError::Input(format!("missing or empty crush group {name}")))?;
    g.nodes
        .iter()
        .map(|&id| {
            body.node_index(id)
                .ok_or_else(|| WallError::Input(format!("group {name}: missing node {id}")))
        })
        .collect()
}

// Copied from src/main.rs:1062,1075 (profile constants and profile_weights).
const PROFILE_POINTS: usize = 6;
const PROFILE_COLUMN_TOL_M: f64 = 1e-9;

fn profile_weights(z: &[f64]) -> Vec<f64> {
    let mut cols: Vec<f64> = Vec::new();
    for &v in z {
        if !cols.iter().any(|c| (c - v).abs() <= PROFILE_COLUMN_TOL_M) {
            cols.push(v);
        }
    }
    cols.sort_by(f64::total_cmp);
    let col_of = |v: f64| {
        cols.iter()
            .position(|c| (c - v).abs() <= PROFILE_COLUMN_TOL_M)
            .expect("every node is in a column")
    };
    let mut col_w = vec![0.0; cols.len()];
    if cols.len() == 1 {
        col_w[0] = 1.0;
    } else {
        let (lo, hi) = (cols[0], cols[cols.len() - 1]);
        let share = 1.0 / PROFILE_POINTS as f64;
        for i in 0..PROFILE_POINTS {
            let zi = lo + (hi - lo) * i as f64 / (PROFILE_POINTS - 1) as f64;
            let j = (0..cols.len() - 1)
                .find(|&j| zi <= cols[j + 1])
                .unwrap_or(cols.len() - 2);
            let t = ((zi - cols[j]) / (cols[j + 1] - cols[j])).clamp(0.0, 1.0);
            col_w[j] += share * (1.0 - t);
            col_w[j + 1] += share * t;
        }
    }
    let mut count = vec![0usize; cols.len()];
    for &v in z {
        count[col_of(v)] += 1;
    }
    z.iter()
        .map(|&v| {
            let c = col_of(v);
            col_w[c] / count[c] as f64
        })
        .collect()
}

// Copied from src/main.rs:1122 (profile_depth).
fn profile_depth(sim: &Sim, a: &[usize], b: &[usize], w: &[f64]) -> f64 {
    let xa = a.iter().map(|&i| sim.pos[i].x).sum::<f64>() / a.len() as f64;
    b.iter()
        .zip(w)
        .map(|(&i, wi)| wi * (sim.pos[i].x - xa))
        .sum()
}

// Copied from src/main.rs:1132 (touches).
fn touches(sim: &Sim, range: std::ops::Range<usize>) -> bool {
    let planes = &sim.planes[range];
    if collide::any_at_or_behind(&sim.pos, planes) {
        return true;
    }
    sim.car.as_ref().is_some_and(|c| {
        c.wheels
            .iter()
            .any(|w| planes.iter().any(|pl| pl.distance(w.pos) <= w.p.radius))
    })
}

fn xyz(v: Vec3) -> [f64; 3] {
    [v.x, v.y, v.z]
}

// One ordered traversal is used for both the direct bit comparison and FNV.
fn state_fields(sim: &Sim) -> Vec<(String, usize, f64)> {
    let mut fields = Vec::new();
    for (i, (&p, &v)) in sim.pos.iter().zip(&sim.vel).enumerate() {
        for (name, vector) in [("node.pos", p), ("node.vel", v)] {
            for (axis, value) in xyz(vector).into_iter().enumerate() {
                fields.push((format!("{name}.{axis}"), i, value));
            }
        }
    }
    for (i, b) in sim.beams.iter().enumerate() {
        fields.push(("beam.rest".into(), i, b.rest));
    }
    if let Some(car) = &sim.car {
        for (i, w) in car.wheels.iter().enumerate() {
            for (name, vector) in [("wheel.pos", w.pos), ("wheel.vel", w.vel)] {
                for (axis, value) in xyz(vector).into_iter().enumerate() {
                    fields.push((format!("{name}.{axis}"), i, value));
                }
            }
            fields.push(("wheel.spin".into(), i, w.spin));
            for (j, &v) in w.deflection.iter().enumerate() {
                for (axis, value) in xyz(v).into_iter().enumerate() {
                    fields.push((format!("wheel.deflection.{j}.{axis}"), i, value));
                }
            }
            for (j, &value) in w.load.iter().enumerate() {
                fields.push((format!("wheel.load.{j}"), i, value));
            }
        }
    }
    fields
}

fn compare(a: &Sim, b: &Sim, k: usize) -> Result<(), WallError> {
    let (a, b) = (state_fields(a), state_fields(b));
    if a.len() != b.len() {
        return Err(WallError::Mismatch(format!("{k} state length 0")));
    }
    for ((field, index, a), (_, _, b)) in a.iter().zip(&b) {
        if a.to_bits() != b.to_bits() {
            return Err(WallError::Mismatch(format!("{k} {field} {index}")));
        }
    }
    Ok(())
}

fn state_hash(sim: &Sim, k: usize) -> Result<u64, WallError> {
    let mut hash = Hasher::new();
    for (field, index, value) in state_fields(sim) {
        if !value.is_finite() {
            return Err(WallError::Unstable(format!(
                "{k} {field} {index} not finite"
            )));
        }
        hash.write_f64(value)
            .map_err(|e| WallError::Unstable(format!("{k}: {e}")))?;
    }
    Ok(hash.finish())
}

fn momentum_x(sim: &Sim) -> f64 {
    let mut p = 0.0;
    for (&m, v) in sim.mass.iter().zip(&sim.vel) {
        p += m * v.x;
    }
    if let Some(car) = &sim.car {
        for w in &car.wheels {
            p += w.p.mass * w.vel.x;
        }
    }
    p
}

fn finite(values: &[f64], context: &str) -> Result<(), WallError> {
    if values.iter().all(|v| v.is_finite()) {
        Ok(())
    } else {
        Err(WallError::Unstable(format!(
            "{context}: non-finite measurement"
        )))
    }
}

/// Cached so the CLI contract test and the measurement tests share one physics run.
pub fn measure_test_a() -> Result<WallForce, WallError> {
    static RESULT: OnceLock<Result<WallForce, WallError>> = OnceLock::new();
    RESULT.get_or_init(measure).clone()
}

fn measure() -> Result<WallForce, WallError> {
    let (input, body) = load_input()?;
    let RunInput {
        cfg,
        crush_groups,
        h,
        n,
    } = input;
    let rear = group(&body, &crush_groups.0)?;
    let front = group(&body, &crush_groups.1)?;
    let mut split = Sim::from_car(cfg.clone(), &body).map_err(WallError::Input)?;
    let mut real = Sim::from_car(cfg.clone(), &body).map_err(WallError::Input)?;
    let mut frozen = Sim::from_car(cfg.clone(), &body).map_err(WallError::Input)?;
    real.cfg.dt = h;
    real.cfg.substeps = 1;
    if split.planes.len() != 2 || split.car.as_ref().is_none_or(|c| c.wheels.is_empty()) {
        return Err(WallError::Input(
            "expected wheels, ground and one wall".into(),
        ));
    }
    let (ground, wall) = (split.planes[0], split.planes[1]);
    if ground != Plane::ground() || wall.normal != Vec3::new(-1.0, 0.0, 0.0) {
        return Err(WallError::Input(
            "momentum requires ground +y and wall -x".into(),
        ));
    }
    split.check().map_err(WallError::Input)?;
    let weights = profile_weights(&front.iter().map(|&i| split.pos[i].z).collect::<Vec<_>>());
    let depth0 = profile_depth(&split, &rear, &front, &weights);
    let p0_x = momentum_x(&split);
    finite(&[depth0, p0_x], "initial state")?;
    let mut result = WallForce {
        h,
        contact_k: 0,
        node_n: Vec::new(),
        tyre_n: Vec::new(),
        raw_n: Vec::new(),
        cfc60_n: Vec::new(),
        crush_m: Vec::new(),
        momentum: Momentum {
            p0_x,
            p_end_x: p0_x,
            wall_x: 0.0,
            ground_node_x: 0.0,
            aero_x: 0.0,
            residual: 0.0,
            tolerance: 1e-6 * p0_x.abs(),
        },
        ground_node_substeps: 0,
        ground_tyre_substeps: 0,
        state_hashes: Vec::new(),
    };
    for series in [
        &mut result.node_n,
        &mut result.tyre_n,
        &mut result.raw_n,
        &mut result.crush_m,
    ] {
        series
            .try_reserve_exact(n)
            .map_err(|e| WallError::Input(format!("history allocation: {e}")))?;
    }
    result
        .state_hashes
        .try_reserve_exact(n)
        .map_err(|e| WallError::Input(format!("hash allocation: {e}")))?;
    let gv = split.gravity_vector();
    let mut solver = SymplecticEuler::default();
    let mut extra = Vec::new();
    for k in 1..=n {
        extra.clear();
        extra.extend(split.mass.iter().map(|&m| Vec3::new(m * gv.x, 0.0, 0.0)));
        // src/vehicle.rs:978 add_drag: same node-order mass-weighted velocity.
        let mut p = Vec3::ZERO;
        let mut mc = 0.0;
        for (v, &mi) in split.vel.iter().zip(&split.mass) {
            p += *v * mi;
            mc += mi;
        }
        let vc = p * (1.0 / mc);
        let car = split
            .car
            .as_mut()
            .ok_or_else(|| WallError::Input("missing car".into()))?;
        let aero = if car.drag_area == 0.0 {
            0.0
        } else {
            let rate = (0.5 * AIR_DENSITY * car.drag_area * vc.length() / mc).min(1.0 / h);
            -rate * vc.x * mc * h
        };
        finite(&[mc, vc.x, vc.y, vc.z, aero], &format!("substep {k} aero"))?;
        car.pre_step(
            &split.pos,
            &split.vel,
            &split.mass,
            &split.planes,
            Substep { gravity: gv, h },
            &mut extra,
        );
        let mut tyre = 0.0;
        let mut ground_tyre = false;
        for w in &car.wheels {
            finite(&w.load, &format!("substep {k} tyre load"))?;
            tyre += w.load[1].max(0.0);
            ground_tyre |= w.load[0] > 0.0;
        }
        if ground_tyre {
            result.ground_tyre_substeps += 1;
            return Err(WallError::Momentum(format!(
                "ground tyre contact at substep {k}: the identity has an unmeasured term"
            )));
        }
        finite(&[tyre], &format!("substep {k} tyre sum"))?;
        solver.step_with_forces(
            State {
                pos: &mut split.pos,
                vel: &mut split.vel,
                beams: &split.beams,
                mass: &split.mass,
                gravity: -gv.y,
            },
            &extra,
            h,
        );
        car.post_step(
            &mut split.pos,
            &mut split.vel,
            &split.mass,
            &split.planes,
            h,
        );
        let vel_g = split.vel.clone();
        collide::apply(&mut split.pos, &mut split.vel, &[ground], h);
        let vel_w = split.vel.clone();
        collide::apply(&mut split.pos, &mut split.vel, &[wall], h);
        split.apply_yield();
        split.apply_unload(h);
        if let Some(e) = solver.check() {
            return Err(WallError::Unstable(format!("substep {k}: {e}")));
        }
        split
            .check()
            .map_err(|e| WallError::Unstable(format!("substep {k}: {e}")))?;
        real.advance();
        real.check()
            .map_err(|e| WallError::Unstable(format!("real substep {k}: {e}")))?;
        compare(&split, &real, k)?;
        if k.is_multiple_of(cfg.substeps as usize) {
            frozen.advance();
            frozen
                .check()
                .map_err(|e| WallError::Unstable(format!("frozen substep {k}: {e}")))?;
            compare(&split, &frozen, k)?;
        }
        let mut node_j = 0.0;
        let mut wall_node_x = 0.0;
        let mut ground_x = 0.0;
        let mut ground_changed = collide::any_at_or_behind(&split.pos, &[ground]);
        for (i, &mass) in split.mass.iter().enumerate() {
            node_j += mass * (split.vel[i] - vel_w[i]).dot(wall.normal);
            wall_node_x += mass * (split.vel[i].x - vel_w[i].x);
            ground_x += mass * (vel_w[i].x - vel_g[i].x);
            ground_changed |= xyz(vel_w[i]).map(f64::to_bits) != xyz(vel_g[i]).map(f64::to_bits);
        }
        if ground_changed {
            result.ground_node_substeps += 1;
        }
        let node = node_j / h;
        let raw = node + tyre;
        let crush = depth0 - profile_depth(&split, &rear, &front, &weights);
        finite(
            &[node, tyre, raw, crush, wall_node_x, ground_x],
            &format!("substep {k} force"),
        )?;
        if node < 0.0 {
            return Err(WallError::Mismatch(format!(
                "{k} negative node wall impulse"
            )));
        }
        if result.contact_k == 0 && touches(&split, 1..2) {
            result.contact_k = k;
        }
        if result.contact_k == 0 && (node != 0.0 || tyre != 0.0) {
            return Err(WallError::Mismatch(format!("{k} force before contact")));
        }
        let m = &mut result.momentum;
        let next = [
            m.wall_x + (wall_node_x + wall.normal.x * tyre * h),
            m.ground_node_x + ground_x,
            m.aero_x + aero,
        ];
        finite(&next, &format!("substep {k} impulse totals"))?;
        m.wall_x = next[0];
        m.ground_node_x = next[1];
        m.aero_x = next[2];
        result.node_n.push(node);
        result.tyre_n.push(tyre);
        result.raw_n.push(raw);
        result.crush_m.push(crush);
        result.state_hashes.push(state_hash(&split, k)?);
    }
    if result.contact_k == 0 {
        return Err(WallError::NoContact);
    }
    let m = &mut result.momentum;
    m.p_end_x = momentum_x(&split);
    m.residual = (m.p_end_x - m.p0_x) - (m.wall_x + m.ground_node_x + m.aero_x);
    finite(
        &[
            m.p_end_x,
            m.wall_x,
            m.ground_node_x,
            m.aero_x,
            m.residual,
            m.tolerance,
        ],
        "momentum totals",
    )?;
    if m.residual.abs() > m.tolerance {
        return Err(WallError::Momentum(format!(
            "p0_x={} p_end_x={} wall_x={} ground_node_x={} aero_x={} residual={} tolerance={}",
            m.p0_x, m.p_end_x, m.wall_x, m.ground_node_x, m.aero_x, m.residual, m.tolerance
        )));
    }
    result.cfc60_n = pulse::cfc_filter(&result.raw_n, h, 60.0)
        .map_err(|e| WallError::Unstable(format!("CFC60: {e}")))?;
    finite(&result.cfc60_n, "CFC60 output")?;
    let s = summarize(
        &result.raw_n,
        &result.cfc60_n,
        &result.node_n,
        &result.tyre_n,
        h,
    );
    check_summary(&s)?;
    Ok(result)
}

fn peak(values: &[f64]) -> (f64, usize) {
    let mut best = (f64::NEG_INFINITY, 0);
    for (i, &v) in values.iter().enumerate() {
        if v > best.0 {
            best = (v, i + 1);
        }
    }
    best
}

/// Valid domain: equal-length, nonempty finite series and positive finite h;
/// products must not underflow to zero and ordered sums must stay finite.
/// Invalid input gives NaN summaries (substep zero), rejected by the report's
/// named summary check. Tiny nonzero products may retain subnormal precision.
pub fn summarize(
    raw_n: &[f64],
    cfc60_n: &[f64],
    node_n: &[f64],
    tyre_n: &[f64],
    h: f64,
) -> Summary {
    let invalid = || Summary {
        peak_raw: (f64::NAN, 0),
        peak_cfc60: (f64::NAN, 0),
        node_ns: f64::NAN,
        tyre_ns: f64::NAN,
        total_ns: f64::NAN,
        cfc60_ns: f64::NAN,
    };
    let valid = h.is_finite()
        && h > 0.0
        && !raw_n.is_empty()
        && [cfc60_n, node_n, tyre_n]
            .iter()
            .all(|s| s.len() == raw_n.len())
        && [raw_n, cfc60_n, node_n, tyre_n]
            .iter()
            .all(|s| s.iter().all(|v| v.is_finite()));
    if !valid {
        return invalid();
    }
    let impulse = |v: &[f64]| {
        v.iter()
            .try_fold(0.0_f64, |sum, &f| {
                let j = f * h;
                let next = sum + j;
                if next.is_finite() && (f == 0.0 || j != 0.0) {
                    Some(next)
                } else {
                    None
                }
            })
            .unwrap_or(f64::NAN)
    };
    let s = Summary {
        peak_raw: peak(raw_n),
        peak_cfc60: peak(cfc60_n),
        node_ns: impulse(node_n),
        tyre_ns: impulse(tyre_n),
        total_ns: impulse(raw_n),
        cfc60_ns: impulse(cfc60_n),
    };
    if [s.node_ns, s.tyre_ns, s.total_ns, s.cfc60_ns]
        .iter()
        .all(|v| v.is_finite())
    {
        s
    } else {
        invalid()
    }
}

fn check_summary(s: &Summary) -> Result<(), WallError> {
    finite(
        &[
            s.peak_raw.0,
            s.peak_cfc60.0,
            s.node_ns,
            s.tyre_ns,
            s.total_ns,
            s.cfc60_ns,
        ],
        "summary",
    )?;
    if (s.total_ns - (s.node_ns + s.tyre_ns)).abs() > 1e-9 * s.total_ns {
        return Err(WallError::Mismatch("node + tyre impulse != total".into()));
    }
    Ok(())
}

/// J211 coefficients, not a second filter. Domain: positive finite parameters,
/// finite reciprocal dt and sample rate >= 10*CFC, with representable wd and
/// positive wa squared. Invalid input returns NaNs.
pub fn cfc_coefficients(cfc: f64, dt: f64) -> [f64; 5] {
    if !(cfc.is_finite()
        && cfc > 0.0
        && dt.is_finite()
        && dt > 0.0
        && (1.0 / dt).is_finite()
        && (10.0 * cfc).is_finite()
        && 1.0 / dt >= 10.0 * cfc)
    {
        return [f64::NAN; 5];
    }
    let wd = 2.0 * std::f64::consts::PI * cfc * 2.0775;
    if !wd.is_finite() {
        return [f64::NAN; 5];
    }
    let wa = (wd * dt / 2.0).sin() / (wd * dt / 2.0).cos();
    if wa * wa == 0.0 {
        return [f64::NAN; 5];
    }
    let s2 = std::f64::consts::SQRT_2;
    let den = 1.0 + s2 * wa + wa * wa;
    let a0 = wa * wa / den;
    [
        a0,
        2.0 * a0,
        a0,
        -2.0 * (wa * wa - 1.0) / den,
        (-1.0 + s2 * wa - wa * wa) / den,
    ]
}

/// Finite equal-length series, positive finite step, representable target count.
/// Invalid arguments or allocation failure give [(NaN, 0)], not a valid crossing.
/// Targets use multiplication, not cumulative addition; nonmonotonic crush still
/// selects the first crossing. The report refuses the sentinel before writing.
pub fn force_vs_crush(
    crush_m: &[f64],
    raw_n: &[f64],
    cfc60_n: &[f64],
    step_m: f64,
) -> Vec<(f64, usize)> {
    let invalid = || vec![(f64::NAN, 0)];
    if !(step_m.is_finite() && step_m > 0.0)
        || crush_m.len() != raw_n.len()
        || crush_m.len() != cfc60_n.len()
        || [crush_m, raw_n, cfc60_n]
            .iter()
            .any(|s| s.iter().any(|v| !v.is_finite()))
    {
        return invalid();
    }
    let max = crush_m.iter().copied().fold(0.0, f64::max);
    let count = max / step_m;
    // Target indices must remain exact integers in f64 and fit a Vec allocation.
    if !count.is_finite()
        || count >= (1_u64 << f64::MANTISSA_DIGITS) as f64
        || count.ceil() >= (isize::MAX as usize / std::mem::size_of::<(f64, usize)>()) as f64
    {
        return invalid();
    }
    let mut rows = Vec::new();
    let upper = count.ceil() as usize;
    if rows.try_reserve_exact(upper).is_err() {
        return invalid();
    }
    // ceil avoids dropping an exact multiplication target when division rounds
    // just below an integer. The actual inclusion rule is target <= max.
    let mut j = 1;
    for (i, &crush) in crush_m.iter().enumerate() {
        while j <= upper {
            let target = step_m * j as f64;
            if target > crush {
                break;
            }
            rows.push((target, i + 1));
            j += 1;
        }
    }
    rows
}

fn report(w: &WallForce, out: &mut dyn Write) -> Result<(), WallError> {
    let write_error = |e: std::io::Error| WallError::Output(e.to_string());
    let s = summarize(&w.raw_n, &w.cfc60_n, &w.node_n, &w.tyre_n, w.h);
    check_summary(&s)?;
    let coeff = cfc_coefficients(60.0, w.h);
    finite(&coeff, "CFC coefficients")?;
    let crossings = force_vs_crush(&w.crush_m, &w.raw_n, &w.cfc60_n, 0.01);
    if crossings
        .iter()
        .any(|&(target, k)| !target.is_finite() || k == 0)
    {
        return Err(WallError::Mismatch(
            "invalid force_vs_crush arguments or allocation".into(),
        ));
    }
    let rel = |k: usize| (k as f64 - w.contact_k as f64) * w.h;
    for (i, &raw) in w.raw_n.iter().enumerate() {
        writeln!(
            out,
            "force t_s={:.6} raw_n={:.6} node_n={:.6} tyre_n={:.6} cfc60_n={:.6} crush_m={:.6}",
            (i + 1) as f64 * w.h, raw, w.node_n[i], w.tyre_n[i], w.cfc60_n[i], w.crush_m[i]
        )
        .map_err(write_error)?;
    }
    for (target, k) in crossings {
        writeln!(
            out,
            "fvc crush_target_m={:.6} t_rel_s={:.6} crush_m={:.6} raw_n={:.6} cfc60_n={:.6}",
            target, rel(k), w.crush_m[k - 1], w.raw_n[k - 1], w.cfc60_n[k - 1]
        )
        .map_err(write_error)?;
    }
    writeln!(
        out,
        "cfc cfc=60 dt_s={:.6} a0={:.15e} a1={:.15e} a2={:.15e} b1={:.15e} b2={:.15e}",
        w.h, coeff[0], coeff[1], coeff[2], coeff[3], coeff[4]
    )
    .map_err(write_error)?;
    writeln!(
        out,
        "contact contact_s={:.6} substep={}",
        w.contact_k as f64 * w.h,
        w.contact_k
    )
    .map_err(write_error)?;
    for (series, p) in [("raw", s.peak_raw), ("cfc60", s.peak_cfc60)] {
        writeln!(
            out,
            "peak series={series} force_n={:.6} t_rel_s={:.6}",
            p.0,
            rel(p.1)
        )
        .map_err(write_error)?;
    }
    writeln!(
        out,
        "impulse node_ns={:.6} tyre_ns={:.6} total_ns={:.6} cfc60_ns={:.6}",
        s.node_ns, s.tyre_ns, s.total_ns, s.cfc60_ns
    )
    .map_err(write_error)?;
    let crush = peak(&w.crush_m);
    writeln!(out, "crush max_m={:.6} t_rel_s={:.6}", crush.0, rel(crush.1))
        .map_err(write_error)?;
    writeln!(
        out,
        "ground node_substeps={} tyre_substeps={}",
        w.ground_node_substeps, w.ground_tyre_substeps
    )
    .map_err(write_error)?;
    let m = &w.momentum;
    writeln!(
        out,
        "momentum p0_x_ns={:.6} p_end_x_ns={:.6} dp_x_ns={:.6} wall_x_ns={:.6} ground_node_x_ns={:.6} aero_x_ns={:.6} residual_ns={:.15e} tolerance_ns={:.15e}",
        m.p0_x,
        m.p_end_x,
        m.p_end_x - m.p0_x,
        m.wall_x,
        m.ground_node_x,
        m.aero_x,
        m.residual,
        m.tolerance
    )
    .map_err(write_error)?;
    // Flush report data before the success marker, so a flush error cannot follow it.
    out.flush().map_err(write_error)?;
    writeln!(out, "wall_force_a: ok").map_err(write_error)
}

pub fn run(args: &[String], out: &mut dyn Write, err: &mut dyn Write) -> i32 {
    let outcome = if args.len() != 1 || args[0] != "--test-a" {
        Err(WallError::Usage("usage: wall_force_a --test-a".into()))
    } else {
        measure_test_a().and_then(|w| report(&w, out))
    };
    match outcome {
        Ok(()) => 0,
        Err(e) => {
            let input = matches!(&e, WallError::Usage(_) | WallError::Input(_));
            let message = e.to_string().replace(
                |c: char| c.is_control() || matches!(c, '\u{2028}' | '\u{2029}'),
                " ",
            );
            let written = if input {
                writeln!(err, "wall_force_a: {message}")
            } else {
                writeln!(err, "wall_force_a: FAIL {message}")
            };
            if written.and_then(|()| err.flush()).is_err() {
                return 1;
            }
            if input {
                2
            } else {
                1
            }
        }
    }
}

#[cfg(not(test))]
fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    std::process::exit(run(&args, &mut std::io::stdout(), &mut std::io::stderr()));
}
