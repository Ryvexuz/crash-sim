//! Test A only; no reference trace, body override, or model edits.
//!
//! Work is positive when energy leaves the car. The ledger uses force-start,
//! average-velocity quadrature (see `budget`). Radial tyre work uses the *clipped*
//! normal force, so an opening tyre cannot supply a fictitious tensile dashpot force.
//! Rolling resistance uses the operation-local spin-energy decrease. Carrier force work
//! is minus the power of its node reactions and wheel reactions; its post-step projection
//! is measured by the local mechanical-energy change of an unconstrained shadow state.
//! Node collision loss is kinetic loss, and node_projection_j is the signed stored-energy
//! decrease across position projection (it can be negative). Carrier work can also be
//! signed. No loss is inferred from the budget residual: integration/quadrature error stays
//! in that signed residual. Suspension stores use current geometry, not the lagged length
//! cached by Car::pre_step. At the frozen zero gravity, reference_load is zero: tangential
//! patch forces/stores are exactly zero, NOT radial loads or rolling resistance.
//!
//! Round 3 bounds the integrator residual at 2% of initial kinetic energy, not 1%.
//! This diagnostic compares material shares of tens of kJ against roughly 169 kJ
//! initially; a 2% integration-error bound cannot flip that material-share decision.
//! The signed residual is reported at every sample and is never booked as dissipation.
//! A budget breach is recorded without truncating the run; all summaries precede
//! the final FAIL verdict and nonzero exit status.
//!
//! The orchestrator invokes this example through the task's Broadsword run route.
//! Plastic flow and the excluded explicit-yield overshoot are documented in `budget`.

use crash_sim::collide;
use crash_sim::solver::{Solver, State, SymplecticEuler};
use crash_sim::vec3::Vec3;
use crash_sim::vehicle::{
    rolling_resistance_moment, Car, Substep, AIR_DENSITY, STOP_STIFFNESS_FACTOR,
};
use crash_sim::{Body, Config, Sim};
use std::io::{self, Write};

#[path = "energy_a_support/budget.rs"]
pub mod budget;

pub const MATERIALS: [&str; 4] = ["chassis", "cabin", "crush_front", "crush_rear"];
pub const BUDGET_FRACTION: f64 = budget::BUDGET_FRACTION;
type Result<T> = std::result::Result<T, String>;

fn finite(name: &str, values: &[f64]) -> Result<()> {
    if values.iter().all(|x| x.is_finite()) {
        Ok(())
    } else {
        Err(format!("non-finite {name}"))
    }
}

/// Embedded, reviewed inputs bound this tool's domain; there is no user numeric/path input.
/// Read the one frozen line, rather than maintaining another set of run parameters.
pub fn frozen(doc: &str) -> Result<Config> {
    let mut lines = doc
        .lines()
        .filter_map(|s| s.strip_prefix("frozen: run_flags = "));
    let line = lines.next().ok_or("missing frozen run_flags")?;
    if lines.next().is_some() {
        return Err("duplicate frozen run_flags".into());
    }
    let words: Vec<_> = line.split_ascii_whitespace().collect();
    if !words.chunks_exact(2).remainder().is_empty() || words.is_empty() {
        return Err("invalid frozen flag/value pairs".into());
    }
    let mut flags = std::collections::BTreeMap::new();
    for pair in words.chunks_exact(2) {
        if !pair[0].starts_with("--") || flags.insert(pair[0], pair[1]).is_some() {
            return Err("invalid or duplicate frozen flag".into());
        }
    }
    let get = |key| {
        flags
            .get(key)
            .copied()
            .ok_or_else(|| format!("missing frozen {key}"))
    };
    let number = |key| -> Result<f64> {
        let v = get(key)?
            .parse::<f64>()
            .map_err(|_| format!("invalid frozen {key}"))?;
        finite(key, &[v])?;
        if v < 0.0 {
            return Err(format!("negative frozen {key}"));
        }
        Ok(v)
    };
    // Validate the entire line, even the comparison flags this measurement does not use.
    for (&key, &value) in &flags {
        match key {
            "--pulse-sensor" | "--pulse-crush-groups" => {
                if value.is_empty()
                    || !value
                        .bytes()
                        .all(|c| c.is_ascii_alphanumeric() || b"_,".contains(&c))
                {
                    return Err(format!("invalid frozen {key}"));
                }
            }
            "--gravity"
            | "--duration"
            | "--dt"
            | "--substeps"
            | "--pulse-cfc-shape"
            | "--pulse-cfc-int"
            | "--pulse-window"
            | "--pulse-threshold"
            | "--pulse-crush-tail"
            | "--setup-mass-tol"
            | "--setup-speed-tol"
            | "--margin-dv"
            | "--margin-peak"
            | "--margin-peak-time"
            | "--margin-duration"
            | "--margin-olc"
            | "--margin-static-crush" => {
                number(key)?;
            }
            _ => return Err(format!("unknown frozen {key}")),
        }
    }
    let substeps = get("--substeps")?
        .parse::<u32>()
        .map_err(|_| "invalid frozen substeps")?;
    let cfg = Config {
        gravity: number("--gravity")?,
        duration: number("--duration")?,
        dt: number("--dt")?,
        substeps,
        ..Config::default()
    };
    let groups: Vec<_> = get("--pulse-crush-groups")?.split(',').collect();
    let sensor = get("--pulse-sensor")?;
    let window = number("--pulse-window")?;
    if groups != ["rear", "front"] || sensor != "sensor" || cfg.gravity != 0.0 {
        return Err("Test A requires zero gravity and sensor/rear/front groups".into());
    }
    if cfg.dt <= 0.0
        || cfg.duration <= 0.0
        || substeps == 0
        || window <= 0.0
        || window > cfg.duration
    {
        return Err("invalid frozen time configuration".into());
    }
    Ok(cfg)
}

pub fn inputs() -> Result<(Config, Body)> {
    let cfg = frozen(include_str!("../docs/STAGE6-M4-M5.md"))?;
    let body = crash_sim::format::parse(include_str!("../scenarios/validation_wall.bsn"))
        .map_err(|e| format!("validation_wall: {e}"))?;
    validate_body(&body)?;
    // This also checks the coupled stability limit with wheels, suspension and tyres.
    Sim::from_car(cfg.clone(), &body)?;
    Ok((cfg, body))
}

pub fn validate_body(body: &Body) -> Result<()> {
    if !body.tets.is_empty() || body.materials.iter().any(|m| m.break_strain.is_some()) {
        return Err("measurement has no tet or broken-beam pass".into());
    }
    // The scenario is authoritative; these are main's required material invariants.
    for (name, k, c, y) in [
        ("chassis", 300000.0, 200.0, 0.02),
        ("cabin", 1000000.0, 6666.67, 0.02),
        ("crush_front", 35897435.898, 724.569, 0.0024),
        ("crush_rear", 143589743.59, 1982.618, 0.0024),
    ] {
        let m = body
            .materials
            .iter()
            .find(|m| m.name == name)
            .ok_or("missing material")?;
        if m.stiffness != k || m.damping != c || m.yield_strain != Some(y) {
            return Err(format!("material {name} differs from main"));
        }
    }
    if body.materials.len() != MATERIALS.len() || body.wheels.is_empty() {
        return Err("unexpected Test A materials or missing wheels".into());
    }
    Ok(())
}

pub fn group(body: &Body, name: &str) -> Result<Vec<usize>> {
    let g = body
        .groups
        .iter()
        .find(|g| g.name == name)
        .ok_or("missing group")?;
    if g.nodes.is_empty() {
        return Err(format!("empty group {name}"));
    }
    g.nodes
        .iter()
        .map(|id| body.node_index(*id).ok_or("missing group node".into()))
        .collect()
}

pub fn velocities(sim: &Sim, sensor: &[usize]) -> Result<[f64; 2]> {
    let (mut p, mut m) = (0.0, 0.0);
    for &i in sensor {
        let mi = *sim.mass.get(i).ok_or("sensor index out of range")?;
        p += mi * sim.vel.get(i).ok_or("sensor velocity missing")?.x;
        m += mi;
    }
    if m <= 0.0 || sensor.is_empty() {
        return Err("sensor mass must be positive".into());
    }
    let sensor_v = p / m;
    p = 0.0;
    m = 0.0;
    for (v, mi) in sim.vel.iter().zip(&sim.mass) {
        p += mi * v.x;
        m += mi;
    }
    if let Some(car) = &sim.car {
        for w in &car.wheels {
            p += w.p.mass * w.vel.x;
            m += w.p.mass;
        }
    }
    if m <= 0.0 {
        return Err("whole-car mass must be positive".into());
    }
    let result = [sensor_v, p / m];
    finite("velocity", &result)?;
    Ok(result)
}

pub fn touches_wall(sim: &Sim) -> bool {
    let planes = &sim.planes[1..];
    collide::any_at_or_behind(&sim.pos, planes)
        || sim.car.as_ref().is_some_and(|c| {
            c.wheels
                .iter()
                .any(|w| planes.iter().any(|p| p.distance(w.pos) <= w.p.radius))
        })
}

/// main.rs::profile_weights: six equally spaced lateral points, equal weight per point,
/// linear interpolation between columns, equal weight among a column's nodes.
pub fn profile_weights(z: &[f64]) -> Result<Vec<f64>> {
    finite("profile", z)?;
    if z.is_empty() {
        return Err("empty profile".into());
    }
    let mut cols: Vec<f64> = Vec::new();
    for &v in z {
        if !cols.iter().any(|c| (c - v).abs() <= 1e-9) {
            cols.push(v);
        }
    }
    cols.sort_by(f64::total_cmp);
    let col_of = |v: f64| cols.iter().position(|c| (c - v).abs() <= 1e-9);
    let mut weight = vec![0.0; cols.len()];
    if cols.len() == 1 {
        weight[0] = 1.0;
    } else {
        for i in 0..6 {
            let zi = cols[0] + (cols[cols.len() - 1] - cols[0]) * i as f64 / 5.0;
            let j = (0..cols.len() - 1)
                .find(|&j| zi <= cols[j + 1])
                .unwrap_or(cols.len() - 2);
            let t = ((zi - cols[j]) / (cols[j + 1] - cols[j])).clamp(0.0, 1.0);
            weight[j] += (1.0 / 6.0) * (1.0 - t);
            weight[j + 1] += (1.0 / 6.0) * t;
        }
    }
    let mut count = vec![0; cols.len()];
    for &v in z {
        count[col_of(v).ok_or("profile column missing")?] += 1;
    }
    let weights = z
        .iter()
        .map(|&v| {
            let c = col_of(v).ok_or("profile column missing")?;
            Ok(weight[c] / count[c] as f64)
        })
        .collect::<Result<Vec<_>>>()?;
    finite("profile weights", &weights)?;
    Ok(weights)
}

fn profile_depth(sim: &Sim, rear: &[usize], front: &[usize], weights: &[f64]) -> f64 {
    let x = rear.iter().map(|&i| sim.pos[i].x).sum::<f64>() / rear.len() as f64;
    front
        .iter()
        .zip(weights)
        .map(|(&i, w)| w * (sim.pos[i].x - x))
        .sum()
}

pub fn beam_elastic(sim: &Sim) -> Vec<f64> {
    sim.beams
        .iter()
        .map(|b| {
            let d = (sim.pos[b.b] - sim.pos[b.a]).length() - b.rest;
            0.5 * b.stiffness * d * d
        })
        .collect()
}

pub fn beam_damper(sim: &Sim, h: f64) -> Result<Vec<f64>> {
    if !h.is_finite() || h <= 0.0 {
        return Err("invalid work timestep".into());
    }
    // Validate before the solver's degenerate-length skip: a coincident pair must not
    // conceal a non-finite velocity or coefficient.
    for b in &sim.beams {
        let a = sim.pos.get(b.a).ok_or("invalid beam position index")?;
        let z = sim.pos.get(b.b).ok_or("invalid beam position index")?;
        let va = sim.vel.get(b.a).ok_or("invalid beam velocity index")?;
        let vz = sim.vel.get(b.b).ok_or("invalid beam velocity index")?;
        finite(
            "beam force inputs",
            &[
                a.x,
                a.y,
                a.z,
                z.x,
                z.y,
                z.z,
                va.x,
                va.y,
                va.z,
                vz.x,
                vz.y,
                vz.z,
                b.rest,
                b.stiffness,
                b.damping,
            ],
        )?;
        finite("beam force length", &[(*z - *a).length()])?;
        if b.rest <= 0.0 || b.stiffness < 0.0 || b.damping < 0.0 {
            return Err("invalid beam work coefficients".into());
        }
    }
    let work: Vec<_> = sim
        .beams
        .iter()
        .map(|b| {
            let d = sim.pos[b.b] - sim.pos[b.a];
            let len = d.length();
            if len < 1e-12 {
                0.0 // Same degenerate-length skip as the force pass.
            } else {
                let v = (sim.vel[b.b] - sim.vel[b.a]).dot(d * (1.0 / len));
                b.damping * v * v * h
            }
        })
        .collect();
    finite("beam damper work", &work)?;
    Ok(work)
}

pub fn yield_work(sim: &mut Sim) -> Result<Vec<f64>> {
    Ok(budget::yield_diagnostics(sim)?.plastic)
}

fn node_kinetic(vel: &[Vec3], mass: &[f64]) -> f64 {
    vel.iter().zip(mass).map(|(v, m)| 0.5 * m * v.dot(*v)).sum()
}

pub fn wheel_kinetic(car: &Car) -> f64 {
    car.wheels
        .iter()
        .map(|w| 0.5 * w.p.mass * w.vel.dot(w.vel) + 0.5 * w.p.inertia * w.spin * w.spin)
        .sum()
}

fn kinetic(sim: &Sim) -> f64 {
    node_kinetic(&sim.vel, &sim.mass) + sim.car.as_ref().map_or(0.0, wheel_kinetic)
}

fn vehicle_elastic(car: &Car, pos: &[Vec3], planes: &[collide::Plane]) -> Result<f64> {
    if car.reference_load != 0.0 || car.throttle != 0.0 || car.brake != 0.0 || car.steer != 0.0 {
        return Err("measurement supports only the frozen zero-gravity coasting car".into());
    }
    let up = car.fit_frame(pos)?.up;
    let mut e = 0.0;
    for w in &car.wheels {
        let c = w.p.mount.iter().fold(Vec3::ZERO, |s, &i| s + pos[i])
            * (1.0 / w.p.mount.len() as f64);
        let s = (c - w.pos).dot(up);
        let ds = w.p.rest - s;
        let over = if s < w.p.travel_min {
            w.p.travel_min - s
        } else if s > w.p.travel_max {
            s - w.p.travel_max
        } else {
            0.0
        };
        e += 0.5 * w.p.spring * ds * ds + 0.5 * STOP_STIFFNESS_FACTOR * w.p.spring * over * over;
        for pl in planes {
            let pen = (w.p.radius - pl.distance(w.pos)).max(0.0);
            e += 0.5 * w.p.radial_stiffness * pen * pen;
        }
    }
    finite("vehicle elastic energy", &[e])?;
    Ok(e)
}

fn stored(sim: &Sim) -> Result<f64> {
    Ok(beam_elastic(sim).iter().sum::<f64>()
        + vehicle_elastic(sim.car.as_ref().ok_or("missing car")?, &sim.pos, &sim.planes)?)
}

#[derive(Clone, Debug, Default)]
pub struct Totals {
    pub plastic: [f64; 4],
    pub yield_at_cap: [f64; 4],
    pub yield_overshoot: [f64; 4],
    pub damper: [f64; 4],
    pub suspension: f64,
    pub tyre_damper: f64,
    pub aero: f64,
    pub collision: f64,
    pub projection: f64,
    pub rolling: f64,
    pub carrier: f64,
    pub node_impulse: f64,
    pub tyre_impulse: f64,
}

impl Totals {
    pub fn plastic_sum(&self) -> f64 {
        self.plastic.iter().sum()
    }
    pub fn damper_sum(&self) -> f64 {
        self.damper.iter().sum::<f64>() + self.suspension + self.tyre_damper + self.aero
    }
    pub fn contact_sum(&self) -> f64 {
        self.collision + self.projection + self.rolling + self.carrier
    }
    fn check(&self) -> Result<()> {
        budget::check_yield_overshoot(self)?;
        finite(
            "cumulative work/impulse",
            &[
                self.plastic_sum(),
                self.damper_sum(),
                self.contact_sum(),
                self.node_impulse,
                self.tyre_impulse,
                self.suspension,
                self.tyre_damper,
                self.aero,
                self.collision,
                self.projection,
                self.rolling,
                self.carrier,
            ],
        )?;
        finite("material plastic work", &self.plastic)?;
        finite("material damper work", &self.damper)
    }
}

/// Left-end analytic cross-check; the force-work ledger uses `budget::BeamWork`.
pub fn record_damper(sim: &Sim, h: f64, materials: &[usize], totals: &mut Totals) -> Result<()> {
    if materials.len() != sim.beams.len() || materials.iter().any(|m| *m >= MATERIALS.len()) {
        return Err("invalid beam material map".into());
    }
    for (work, &material) in beam_damper(sim, h)?.iter().zip(materials) {
        totals.damper[material] += work;
    }
    totals.check()
}

/// Force-local vehicle accounting. The returned shadow has unconstrained end positions and
/// velocities; spin is filled from post_step (which only copies pre_step's spin_next).
fn vehicle_work(sim: &Sim, extra: &[Vec3], h: f64, totals: &mut Totals) -> Result<Car> {
    let car = sim.car.as_ref().ok_or("missing car")?;
    vehicle_elastic(car, &sim.pos, &sim.planes)?;
    let mut shadow = car.clone();
    let up = car.frame.up;
    let mut known = vec![Vec3::ZERO; sim.pos.len()];
    let mut wheel_reaction_power = 0.0;
    for (w, next) in car.wheels.iter().zip(&mut shadow.wheels) {
        let share = 1.0 / w.p.mount.len() as f64;
        let vc = w.p.mount.iter().fold(Vec3::ZERO, |s, &i| s + sim.vel[i]) * share;
        let sd = (vc - w.vel).dot(up);
        totals.suspension += w.p.damper * sd * sd * h;
        let s = w.length;
        let stop = if s < w.p.travel_min {
            STOP_STIFFNESS_FACTOR * w.p.spring * (w.p.travel_min - s)
        } else if s > w.p.travel_max {
            -STOP_STIFFNESS_FACTOR * w.p.spring * (s - w.p.travel_max)
        } else {
            0.0
        };
        let fs = w.p.spring * (w.p.rest - s) - w.p.damper * sd + stop;
        for &i in &w.p.mount {
            known[i] += up * (fs * share);
        }
        let mut force = up * (-fs); // Frozen gravity is zero; patch force is zero.
        for (k, pl) in sim.planes.iter().enumerate() {
            let pen = w.p.radius - pl.distance(w.pos);
            let vn = w.vel.dot(pl.normal);
            if pen > 0.0 {
                totals.tyre_damper += (w.p.radial_stiffness * pen - w.load[k]) * vn * h;
            }
            if w.load[k] > 0.0 {
                force += pl.normal * w.load[k] + Vec3::ZERO;
            }
        }
        totals.tyre_impulse += w.load[1] * h; // No ground-contact condition.
        // Tangential torque is zero at reference_load=0. Isolate the actual rolling
        // operation, before pre_step's brake/chassis-spin arithmetic (brake is zero).
        let stop = rolling_resistance_moment(
            car.rolling_resistance,
            w.load.iter().sum(),
            w.p.radius,
            w.spin,
        ) * h
            / w.p.inertia;
        let rolled = if stop <= 0.0 {
            w.spin
        } else if w.spin.abs() <= stop {
            0.0
        } else {
            w.spin - w.spin.signum() * stop
        };
        finite("rolling resistance work", &[stop, rolled])?;
        totals.rolling += 0.5 * w.p.inertia * (w.spin * w.spin - rolled * rolled);
        let across = force - up * force.dot(up);
        wheel_reaction_power += (-across).dot(w.vel);
        let axial = force - across;
        next.vel += axial * (h / w.p.mass);
        next.pos += next.vel * h;
    }
    let mass: f64 = sim.mass.iter().sum();
    let v = sim
        .vel
        .iter()
        .zip(&sim.mass)
        .fold(Vec3::ZERO, |p, (v, m)| p + *v * *m)
        * (1.0 / mass);
    let rate = (0.5 * AIR_DENSITY * car.drag_area * v.length() / mass).min(1.0 / h);
    let a = v * (-rate);
    totals.aero += -a.dot(v) * mass * h;
    for ((f, &mi), vi) in known.iter_mut().zip(&sim.mass).zip(&sim.vel) {
        *f += a * mi;
        finite("force-evaluation velocity", &[vi.x, vi.y, vi.z])?;
    }
    let node_power: f64 = extra
        .iter()
        .zip(&known)
        .zip(&sim.vel)
        .map(|((all, known), v)| (*all - *known).dot(*v))
        .sum();
    totals.carrier -= (node_power + wheel_reaction_power) * h;
    Ok(shadow)
}

pub fn node_wall_impulse(before: &[Vec3], sim: &Sim) -> Result<f64> {
    if before.len() != sim.vel.len() || sim.planes.len() != 2 {
        return Err("invalid wall impulse state".into());
    }
    let j = before
        .iter()
        .zip(&sim.vel)
        .zip(&sim.mass)
        .map(|((a, b), m)| m * (*b - *a).dot(sim.planes[1].normal))
        .sum();
    finite("node wall impulse", &[j])?;
    Ok(j)
}

/// One authoritative split path, with observation optionally disabled. Observations never
/// supply a force or write physical state; only the real solver, vehicle and collision passes do.
pub fn split_step(
    sim: &mut Sim,
    solver: &mut SymplecticEuler,
    h: f64,
    materials: &[usize],
    mut observation: Option<&mut Totals>,
) -> Result<()> {
    if !h.is_finite()
        || h <= 0.0
        || sim.cfg.gravity != 0.0
        || sim.planes.len() != 2
        || sim.break_strain.iter().any(Option::is_some)
        || sim.broken.iter().any(|b| *b)
        || !sim.breaks.is_empty()
        || materials.len() != sim.beams.len()
        || materials.iter().any(|m| *m >= MATERIALS.len())
    {
        return Err("unsupported split configuration".into());
    }
    sim.check()?;
    vehicle_elastic(
        sim.car.as_ref().ok_or("missing car")?,
        &sim.pos,
        &sim.planes,
    )?;
    let gv = sim.gravity_vector();
    let mut extra: Vec<_> = sim
        .mass
        .iter()
        .map(|&m| Vec3::new(m * gv.x, 0.0, 0.0))
        .collect();
    sim.car.as_mut().ok_or("missing car")?.pre_step(
        &sim.pos,
        &sim.vel,
        &sim.mass,
        &sim.planes,
        Substep { gravity: gv, h },
        &mut extra,
    );
    let shadow = match observation.as_deref_mut() {
        Some(t) => Some(budget::ForceWork::start(sim, &extra, h, t)?),
        None => None,
    };
    solver.step_with_forces(
        State {
            pos: &mut sim.pos,
            vel: &mut sim.vel,
            beams: &sim.beams,
            mass: &sim.mass,
            gravity: -gv.y,
        },
        &extra,
        h,
    );
    if let Some(e) = solver.check() {
        return Err(e);
    }
    let shadow = match (shadow, observation.as_deref_mut()) {
        (Some(work), Some(t)) => Some(work.finish(sim, materials, t)?),
        _ => None,
    };
    let unconstrained_nodes = shadow.as_ref().map(|_| (sim.pos.clone(), sim.vel.clone()));
    sim.car.as_mut().ok_or("missing car")?.post_step(
        &mut sim.pos,
        &mut sim.vel,
        &sim.mass,
        &sim.planes,
        h,
    );
    if let (Some(mut shadow), Some((pos, vel)), Some(t)) =
        (shadow, unconstrained_nodes, observation.as_deref_mut())
    {
        let car = sim.car.as_ref().ok_or("missing car")?;
        for (a, b) in shadow.wheels.iter_mut().zip(&car.wheels) {
            a.spin = b.spin;
        }
        let beam_before: f64 = sim
            .beams
            .iter()
            .map(|b| {
                let d = (pos[b.b] - pos[b.a]).length() - b.rest;
                0.5 * b.stiffness * d * d
            })
            .sum();
        let before = node_kinetic(&vel, &sim.mass)
            + wheel_kinetic(&shadow)
            + beam_before
            + vehicle_elastic(&shadow, &pos, &sim.planes)?;
        t.carrier += before - (kinetic(sim) + stored(sim)?);
    }
    let before_contact = if observation.is_some() {
        Some((kinetic(sim), stored(sim)?))
    } else {
        None
    };
    let ground = sim.planes[0];
    let wall = sim.planes[1];
    collide::apply(&mut sim.pos, &mut sim.vel, &[ground], h);
    let before_wall = observation.as_ref().map(|_| sim.vel.clone());
    collide::apply(&mut sim.pos, &mut sim.vel, &[wall], h);
    if let Some(t) = observation {
        t.node_impulse += node_wall_impulse(&before_wall.ok_or("missing wall observation")?, sim)?;
        let (k, e) = before_contact.ok_or("missing contact observation")?;
        t.collision += k - kinetic(sim);
        t.projection += e - stored(sim)?;
        budget::record_exact_yield(sim, materials, t)?;
        t.check()?;
    } else {
        sim.apply_yield();
    }
    sim.check()
}

fn bits3(a: Vec3, b: Vec3) -> bool {
    a.x.to_bits() == b.x.to_bits()
        && a.y.to_bits() == b.y.to_bits()
        && a.z.to_bits() == b.z.to_bits()
}

/// Compare physical state, not the different output clocks/configs or allocator addresses.
/// Debug on derived Car/Wheel includes PRIVATE force, axle, spin_next and frame-fit weights.
/// Its finite-f64 round-trip representations distinguish signed zero, unlike PartialEq.
/// Public dynamic floats additionally receive explicit to_bits comparisons.
pub fn same_state(a: &Sim, b: &Sim) -> Result<()> {
    let vectors = a.pos.len() == b.pos.len()
        && a.vel.len() == b.vel.len()
        && a.pos.iter().zip(&b.pos).all(|(a, b)| bits3(*a, *b))
        && a.vel.iter().zip(&b.vel).all(|(a, b)| bits3(*a, *b));
    let beams = a.beams.len() == b.beams.len()
        && a.beams.iter().zip(&b.beams).all(|(a, b)| {
            a.a == b.a
                && a.b == b.b
                && a.rest.to_bits() == b.rest.to_bits()
                && a.stiffness.to_bits() == b.stiffness.to_bits()
                && a.damping.to_bits() == b.damping.to_bits()
        });
    let wheels = match (&a.car, &b.car) {
        (Some(a), Some(b)) => {
            a == b
                && format!("{a:?}") == format!("{b:?}")
                && a.wheels.len() == b.wheels.len()
                && a.wheels.iter().zip(&b.wheels).all(|(a, b)| {
                    bits3(a.pos, b.pos)
                        && bits3(a.vel, b.vel)
                        && a.spin.to_bits() == b.spin.to_bits()
                        && a.length.to_bits() == b.length.to_bits()
                        && a.deflection.len() == b.deflection.len()
                        && a.deflection
                            .iter()
                            .zip(&b.deflection)
                            .all(|(a, b)| bits3(*a, *b))
                        && a.load.len() == b.load.len()
                        && a.load
                            .iter()
                            .zip(&b.load)
                            .all(|(a, b)| a.to_bits() == b.to_bits())
                })
        }
        (None, None) => true,
        _ => false,
    };
    if !(vectors
        && beams
        && wheels
        && a.yield_strain == b.yield_strain
        && a.rest0 == b.rest0
        && a.yield_refused == b.yield_refused
        && a.break_strain == b.break_strain
        && a.broken == b.broken
        && a.breaks == b.breaks
        && a.mass == b.mass
        && a.planes == b.planes
        && a.check() == b.check())
    {
        return Err("passive observation bit mismatch".into());
    }
    Ok(())
}

#[derive(Clone, Debug)]
pub struct Row {
    pub elastic: [f64; 4],
    pub totals: Totals,
    pub velocity: [f64; 2],
    pub crush: f64,
    pub kinetic: f64,
    pub vehicle_elastic: f64,
    pub wheel_kinetic: f64,
    pub residual: f64,
}

pub fn check_budget(initial: f64, k: f64, e: f64, p: f64, d: f64, c: f64) -> Result<f64> {
    budget::check_budget(initial, k, e, p, d, c)
}

#[derive(Clone, Debug)]
pub struct Measurement {
    pub rows: Vec<Row>,
    pub h: f64,
    pub contact: usize,
    pub split: usize,
    pub initial: f64,
    pub max_residual: f64,
    pub budget: budget::Budget,
}

pub fn earliest_max(values: &[f64]) -> Result<usize> {
    finite("phase profile", values)?;
    if values.is_empty() {
        return Err("empty phase profile".into());
    }
    let mut best = 0;
    for i in 1..values.len() {
        if values[i] > values[best] {
            best = i;
        }
    }
    Ok(best)
}

pub fn rebound(values: &[f64], contact: usize, h: f64) -> Result<(Option<f64>, f64, f64)> {
    finite("rebound velocity", values)?;
    if contact == 0
        || contact >= values.len()
        || !h.is_finite()
        || h <= 0.0
        || values[contact - 1] <= 0.0
    {
        return Err("invalid rebound contact, timestep or pre-contact speed".into());
    }
    let mut best = None;
    let mut min = 0.0;
    for (i, &v) in values.iter().enumerate().skip(contact) {
        if v < min {
            best = Some(i);
            min = v;
        }
    }
    let speed = if best.is_some() { -min } else { 0.0 };
    let fraction = speed / values[contact - 1];
    finite("rebound result", &[speed, fraction])?;
    let time = best.map(|i| (i - contact) as f64 * h);
    if let Some(t) = time {
        finite("rebound time", &[t])?;
    }
    Ok((time, speed, fraction))
}

pub fn measure() -> Result<Measurement> {
    let (cfg, body) = inputs()?;
    let mut measured = Sim::from_car(cfg.clone(), &body)?;
    let mut disabled = Sim::from_car(cfg.clone(), &body)?;
    let mut ordinary = Sim::from_car(cfg.clone(), &body)?;
    let mut boundaries = Sim::from_car(cfg.clone(), &body)?;
    let h = cfg.dt / f64::from(cfg.substeps);
    ordinary.cfg.dt = h;
    ordinary.cfg.substeps = 1;
    let materials: Vec<_> = body
        .beams
        .iter()
        .map(|b| {
            MATERIALS
                .iter()
                .position(|m| *m == b.material)
                .ok_or("unknown beam material".into())
        })
        .collect::<Result<_>>()?;
    let sensor = group(&body, "sensor")?;
    let expected_sensor: Vec<_> = [12, 14]
        .iter()
        .map(|id| {
            body.node_index(*id)
                .ok_or("missing rear-floor sensor node".into())
        })
        .collect::<Result<_>>()?;
    if sensor != expected_sensor {
        return Err("sensor is not rear-floor nodes 12,14".into());
    }
    let rear = group(&body, "rear")?;
    let front = group(&body, "front")?;
    let weights = profile_weights(&front.iter().map(|&i| measured.pos[i].z).collect::<Vec<_>>())?;
    let depth0 = profile_depth(&measured, &rear, &front, &weights);
    let initial = kinetic(&measured);
    let initial_stored = stored(&measured)?;
    finite("initial stores", &[initial_stored])?;
    if initial_stored.abs() > 1e-8 {
        return Err(format!("initial stored energy_j={initial_stored:.12}"));
    }
    let n = crash_sim::step_count(&cfg)
        .checked_mul(u64::from(cfg.substeps))
        .and_then(|n| usize::try_from(n).ok())
        .ok_or("substep count overflow")?;
    if n == 0 {
        return Err("empty measurement".into());
    }
    let mut result = Measurement {
        rows: Vec::new(),
        h,
        contact: 0,
        split: 0,
        initial,
        max_residual: 0.0,
        budget: budget::Budget::new(initial)?,
    };
    result
        .rows
        .try_reserve(n.checked_add(1).ok_or("sample count overflow")?)
        .map_err(|e| format!("sample allocation: {e}"))?;
    let mut totals = Totals::default();
    let mut solver = SymplecticEuler::default();
    let mut off_solver = SymplecticEuler::default();
    let mut contact = if touches_wall(&measured) {
        Some(0)
    } else {
        None
    };
    for i in 0..=n {
        if i > 0 {
            ordinary.advance();
            split_step(&mut measured, &mut solver, h, &materials, Some(&mut totals))?;
            split_step(&mut disabled, &mut off_solver, h, &materials, None)?;
            same_state(&measured, &ordinary).map_err(|e| format!("{e} substep={i} ordinary"))?;
            same_state(&measured, &disabled).map_err(|e| format!("{e} substep={i} disabled"))?;
            if solver.check() != off_solver.check() {
                return Err(format!("solver error mismatch substep={i}"));
            }
            if i.checked_rem(cfg.substeps as usize) == Some(0) {
                boundaries.advance();
                same_state(&measured, &boundaries)
                    .map_err(|e| format!("{e} substep={i} output boundary"))?;
            }
            ordinary.check()?;
            if contact.is_none() && touches_wall(&measured) {
                contact = Some(i);
            }
        }
        let mut elastic = [0.0; 4];
        for (e, &m) in beam_elastic(&measured).iter().zip(&materials) {
            elastic[m] += e;
        }
        finite("material elastic energy", &elastic)?;
        let ve = vehicle_elastic(
            measured.car.as_ref().ok_or("missing car")?,
            &measured.pos,
            &measured.planes,
        )?;
        let k = kinetic(&measured);
        let residual = budget::residual(
            initial,
            k,
            elastic.iter().sum::<f64>() + ve,
            totals.plastic_sum(),
            totals.damper_sum(),
            totals.contact_sum(),
        )
        .map_err(|e| {
            format!(
                "{e} substep={i} kinetic_j={k:.6} beam_elastic_j={:.6} vehicle_elastic_j={ve:.6} plastic_j={:.6} beam_damper_j={:.6} suspension_damper_j={:.6} tyre_damper_j={:.6} aero_j={:.6} node_collision_j={:.6} node_projection_j={:.6} tyre_friction_rolling_j={:.6} carrier_constraint_j={:.6} previous_residual_j={:.6}",
                elastic.iter().sum::<f64>(),
                totals.plastic_sum(),
                totals.damper.iter().sum::<f64>(),
                totals.suspension,
                totals.tyre_damper,
                totals.aero,
                totals.collision,
                totals.projection,
                totals.rolling,
                totals.carrier,
                result.rows.last().map_or(0.0, |row| row.residual)
            )
        })?;
        result.budget.record(i, residual)?;
        result.max_residual = result.budget.max_abs_residual;
        let crush = depth0 - profile_depth(&measured, &rear, &front, &weights);
        finite("crush depth", &[crush])?;
        result.rows.push(Row {
            elastic,
            totals: totals.clone(),
            velocity: velocities(&measured, &sensor)?,
            crush,
            kinetic: k,
            vehicle_elastic: ve,
            wheel_kinetic: measured.car.as_ref().map_or(0.0, wheel_kinetic),
            residual,
        });
    }
    result.contact = contact.ok_or("no wall contact")?;
    result.split = earliest_max(&result.rows.iter().map(|r| r.crush).collect::<Vec<_>>())?;
    if result.contact == 0 || result.split < result.contact || result.split >= n {
        return Err("missing run-up, compression or unloading intervals".into());
    }
    Ok(result)
}

impl Measurement {
    pub fn validate(&self) -> Result<()> {
        if self.rows.len() < 2
            || self.contact == 0
            || self.contact > self.split
            || self.split >= self.rows.len() - 1
            || !self.h.is_finite()
            || self.h <= 0.0
        {
            return Err("invalid measurement samples or split".into());
        }
        finite("measurement times", &[self.h * self.rows.len() as f64])?;
        for row in &self.rows {
            finite(
                "sample",
                &[
                    row.kinetic,
                    row.vehicle_elastic,
                    row.wheel_kinetic,
                    row.crush,
                    row.residual,
                    row.velocity[0],
                    row.velocity[1],
                    row.velocity[0] - row.velocity[1],
                ],
            )?;
            finite("sample elastic", &row.elastic)?;
            row.totals.check()?;
            let r = budget::residual(
                self.initial,
                row.kinetic,
                row.elastic.iter().sum::<f64>() + row.vehicle_elastic,
                row.totals.plastic_sum(),
                row.totals.damper_sum(),
                row.totals.contact_sum(),
            )?;
            if r.to_bits() != row.residual.to_bits() {
                return Err("recorded budget residual mismatch".into());
            }
        }
        self.budget
            .validate(self.initial, &self.rows, self.max_residual)?;
        // Cumulative columns start at initialization, not at contact.
        let first = &self.rows[0].totals;
        if first.node_impulse != 0.0
            || first.tyre_impulse != 0.0
            || first.plastic_sum() != 0.0
            || first.damper_sum() != 0.0
        {
            return Err("nonzero initial work or impulse".into());
        }
        let middle = &self.rows[self.split].totals;
        let last = &self.rows[self.rows.len() - 1].totals;
        for (a, b) in [(first, middle), (middle, last), (first, last)] {
            finite(
                "phase wall impulses",
                &[
                    b.node_impulse - a.node_impulse,
                    b.tyre_impulse - a.tyre_impulse,
                    (b.node_impulse - a.node_impulse) + (b.tyre_impulse - a.tyre_impulse),
                ],
            )?;
            for m in 0..MATERIALS.len() {
                finite(
                    "phase material work",
                    &[b.plastic[m] - a.plastic[m], b.damper[m] - a.damper[m]],
                )?;
            }
        }
        for (start, split, end) in [
            (first.node_impulse, middle.node_impulse, last.node_impulse),
            (first.tyre_impulse, middle.tyre_impulse, last.tyre_impulse),
        ] {
            let phase_sum = (split - start) + (end - split);
            let run = end - start;
            // Only floating summation roundoff is allowed in this identity.
            if (phase_sum - run).abs() > 4.0 * f64::EPSILON * run.abs().max(1.0) {
                return Err("wall impulse phase sum mismatch".into());
            }
            finite("wall impulse total", &[run])?;
        }
        finite(
            "combined wall impulse",
            &[last.node_impulse + last.tyre_impulse],
        )?;
        for c in 0..2 {
            rebound(
                &self.rows.iter().map(|r| r.velocity[c]).collect::<Vec<_>>(),
                self.contact,
                self.h,
            )?;
        }
        Ok(())
    }

    /// A phase contains intervals ending after its start sample and through its end sample.
    /// The shared split sample supplies unloading's starting elastic store, not duplicate work.
    pub fn render(&self, out: &mut impl Write) -> Result<()> {
        self.validate()?;
        self.render_inner(out).map_err(|e| format!("output write: {e}"))
    }

    fn render_inner(&self, out: &mut impl Write) -> io::Result<()> {
        let rel = |i: usize| (i as f64 - self.contact as f64) * self.h;
        let peak = self.budget.at_substep;
        for (i, row) in self.rows.iter().enumerate() {
            let phase = if i <= self.split {
                "compression"
            } else {
                "unloading"
            };
            for (m, name) in MATERIALS.iter().enumerate() {
                writeln!(
                    out,
                    "energy t_s={:.6} phase={phase} material={name} elastic_j={:.6} plastic_j={:.6} damper_j={:.6}",
                    rel(i),
                    row.elastic[m],
                    row.totals.plastic[m],
                    row.totals.damper[m]
                )?;
            }
            writeln!(
                out,
                "velocity t_s={:.6} sensor_mps={:.6} com_mps={:.6} sensor_minus_com_mps={:.6}",
                rel(i),
                row.velocity[0],
                row.velocity[1],
                row.velocity[0] - row.velocity[1]
            )?;
            writeln!(
                out,
                "residual t_s={:.6} residual_j={:.6}",
                rel(i),
                row.residual
            )?;
        }
        writeln!(
            out,
            "split contact_s={:.6} compression_end_s={:.6} end_s={:.6}",
            self.contact as f64 * self.h,
            rel(self.split),
            rel(self.rows.len() - 1)
        )?;
        for (phase, start, end) in [
            ("compression", 0, self.split),
            ("unloading", self.split, self.rows.len() - 1),
        ] {
            for m in 0..MATERIALS.len() {
                let peak = self.rows[start..=end]
                    .iter()
                    .map(|r| r.elastic[m])
                    .fold(0.0, f64::max);
                budget::render_phase(out, self, phase, start, end, m, peak)?;
            }
        }
        for (phase, start, end) in [
            ("compression", 0, self.split),
            ("unloading", self.split, self.rows.len() - 1),
            ("total", 0, self.rows.len() - 1),
        ] {
            let a = &self.rows[start].totals;
            let b = &self.rows[end].totals;
            let node = b.node_impulse - a.node_impulse;
            let tyre = b.tyre_impulse - a.tyre_impulse;
            writeln!(
                out,
                "wall phase={phase} node_ns={node:.6} tyre_ns={tyre:.6} total_ns={:.6}",
                node + tyre
            )?;
        }
        for (c, channel) in ["sensor", "com"].iter().enumerate() {
            let values: Vec<_> = self.rows.iter().map(|r| r.velocity[c]).collect();
            let (time, speed, fraction) = rebound(&values, self.contact, self.h)
                .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
            let time = time.map_or_else(|| "none".into(), |t| format!("{t:.6}"));
            writeln!(
                out,
                "rebound channel={channel} t_s={time} speed_mps={speed:.6} fraction_v0={fraction:.6}"
            )?;
        }
        let row = self
            .rows
            .last()
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "no samples"))?;
        let t = &row.totals;
        budget::render_aux(out, row)?;
        writeln!(
            out,
            "budget initial_kinetic_j={:.6} kinetic_j={:.6} elastic_j={:.6} plastic_j={:.6} damper_j={:.6} contact_loss_j={:.6} residual_j={:.6} max_abs_residual_j={:.6} tolerance_j={:.6}",
            self.initial,
            row.kinetic,
            row.elastic.iter().sum::<f64>() + row.vehicle_elastic,
            t.plastic_sum(),
            t.damper_sum(),
            t.contact_sum(),
            row.residual,
            self.max_residual,
            BUDGET_FRACTION * self.initial
        )?;
        writeln!(
            out,
            "integrator_residual max_abs_j={:.6} at_substep={peak} fraction_of_initial_ke={:.6}",
            self.max_residual,
            self.max_residual / self.initial
        )?;
        self.budget.finish(out)
    }
}

/// Stream-injected CLI also makes refusal and write failures testable without child processes.
pub fn cli(args: &[String], out: &mut impl Write, err: &mut impl Write) -> i32 {
    if args != ["--test-a"] {
        if writeln!(err, "usage: crash_energy_a --test-a").is_err() {
            return 1;
        }
        return 2;
    }
    if let Err(e) = inputs() {
        if writeln!(err, "measurement_a: FAIL input {e}").is_err() {
            return 1;
        }
        return 2;
    }
    match budget::report(measure(), out) {
        Ok(code) => code,
        Err(e) => {
            let _ = writeln!(err, "measurement_a: FAIL {e}");
            1
        }
    }
}

#[cfg(not(test))]
fn main() {
    let args: Vec<_> = std::env::args_os()
        .skip(1)
        .map(|arg| arg.to_string_lossy().into_owned())
        .collect();
    let code = cli(&args, &mut io::stdout().lock(), &mut io::stderr().lock());
    std::process::exit(code);
}
