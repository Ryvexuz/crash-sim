//! Library AI race, using the timed-lap program's time base: each Sim has dt = h,
//! substeps = 1, and timers see [i*h, (i+1)*h] after every MultiSim advance.
//! Drivers update every CONTROL_DT (0.02 s), holding recorded controls throughout
//! the interval. End checks occur only at whole control boundaries; finished cars
//! keep driving until all finish or the interval-count time limit is reached.
//! The read-only wall rule checks every chassis node and each tyre's outward edge
//! every substep; there are no physical track walls.
//!
//! Contact uses skin 0.25 m, restitution 0, max_motion = skin/8 = 0.03125 m,
//! max_penetration = skin/2 = 0.125 m, and timestep h. The motion bound covers
//! 31.25 m/s at h = 1 ms, twice the 15.6 m/s bend speed; the skin is below the
//! 0.4 m body clearance of centres 4.40 m apart. Profiles must leave half the
//! motion bound as headroom. Kernel refusals stop the race, without retry/clamp.
//! `place_on_grid` copies cli::race::place_on_grid operation for operation until
//! the CLI slice replaces that copy with a call here.

use crate::ai::{CarState, Driver, DriverParams};
use crate::car_contact::{ContactError, ContactSettings};
use crate::multi_car::{CarControls, CarPlacement, MultiError, MultiSim};
use crate::settle::{self, CarSetup};
use crate::speed_profile::{self, ProfileError};
use crate::track::{LapEvent, LapRecord, LapTimer, TimerError, Track};
use crate::vec3::Vec3;
use crate::{Body, Config, Sim};

pub const CONTROL_DT: f64 = 0.02;
pub const MAX_CARS: usize = 3;
pub const MAX_LAPS: u32 = 10;
const CONTACT_SKIN: f64 = 0.25;
const CONTACT_RESTITUTION: f64 = 0.0;
const MAX_MOTION: f64 = CONTACT_SKIN / 8.0;
const MAX_PENETRATION: f64 = CONTACT_SKIN / 2.0;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ProfileParams {
    pub mu: f64,
    pub g: f64,
    pub a_accel: f64,
    pub a_brake: f64,
    pub v_max: f64,
}

pub struct RaceSetup<'a> {
    /// dt, substeps and gravity are used; duration is ignored (the race has its own end).
    pub cfg: &'a Config,
    pub body: &'a Body,
    pub track: &'a Track,
    pub cars: usize,
    pub laps: u32,
    pub time_limit_s: f64,
    pub grid_gap_m: f64,
    pub driver: DriverParams,
    pub profile: ProfileParams,
}

#[derive(Debug)]
pub enum RaceError {
    CarCount(usize),
    Laps(u32),
    TimeLimit(f64),
    GridGap(f64),
    /// A DriverParams field fails Driver's own validity rule.
    Driver(&'static str),
    Profile(ProfileError),
    /// Invalid config, unrepresentable counts, or a fractional control interval.
    Config(String),
    Body(String),
    /// A 1-based slot lies farther back than the centreline loop.
    Grid {
        slot: usize,
    },
    SpeedAboveContactBound {
        v_max: f64,
    },
    Contact(ContactError),
    Multi(MultiError),
}

impl std::fmt::Display for RaceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::CarCount(n) => write!(f, "cars {n}: expected 1..={MAX_CARS}"),
            Self::Laps(n) => write!(f, "laps {n}: expected 1..={MAX_LAPS}"),
            Self::TimeLimit(t) => write!(f, "time_limit_s {t}: invalid or unrepresentable"),
            Self::GridGap(g) => write!(f, "grid_gap_m {g}: must be finite and positive"),
            Self::Driver(field) => write!(f, "invalid driver field {field}"),
            Self::Profile(e) => write!(f, "profile: {e}"),
            Self::Config(e) => write!(f, "config: {e}"),
            Self::Body(e) => write!(f, "body: {e}"),
            Self::Grid { slot } => write!(f, "grid slot {slot}: distance outside centreline loop"),
            Self::SpeedAboveContactBound { v_max } => {
                write!(f, "profile v_max {v_max}: exceeds contact motion headroom")
            }
            Self::Contact(e) => write!(f, "contact: {e}"),
            Self::Multi(e) => write!(f, "multi: {e}"),
        }
    }
}

impl std::error::Error for RaceError {}

/// A runtime failure; the result retains the race so far and the state at refusal.
#[derive(Debug)]
pub enum RaceFailure {
    Sim {
        t: f64,
        error: MultiError,
    },
    Timer {
        car: usize,
        t: f64,
        error: TimerError,
    },
}

#[derive(Debug)]
pub enum RaceEnd {
    AllFinished,
    TimeLimit,
    Failed(RaceFailure),
}

pub struct CarResult {
    /// 1-based grid slot (= car index + 1).
    pub slot: usize,
    /// inputs[j] is held for interval j, starting at j * control_substeps.
    /// A failed interval retains its input, even if its first advance refused.
    pub inputs: Vec<CarControls>,
    /// Every LapTimer event, in order.
    pub events: Vec<LapEvent>,
    pub laps: Vec<LapRecord>,
    /// end_t of the setup's requested lap count, valid or not.
    pub finish_t: Option<f64>,
    pub wall_substeps: u64,
    /// t1 of the first substep with a wall contact.
    pub first_wall_t: Option<f64>,
}

pub struct RaceResult {
    pub cars: Vec<CarResult>,
    /// Fully observed substeps; end_t = substeps as f64 * h.
    /// On failure, multi retains its state at refusal, not a rolled-back state.
    pub substeps: u64,
    pub h: f64,
    pub control_substeps: u64,
    /// Sums of successful MultiSim StepReports.
    pub contacts: usize,
    pub impulses: usize,
    pub end: RaceEnd,
    /// Final state, available read-only through MultiSim.
    pub multi: MultiSim,
}

/// Grid distance of 1-based slot k. Slot zero returns NaN, not an integer underflow.
/// run validates the gap and the resulting distance before placement.
pub fn slot_distance(track: &Track, slot: usize, grid_gap_m: f64) -> f64 {
    let Some(index) = slot.checked_sub(1) else {
        return f64::NAN;
    };
    track.grid + index as f64 * grid_gap_m
}

/// Place a parsed body at a finite distance in [0, centre_length] before the start.
/// Invalid distances use Grid { slot: 1 }; run checks the actual slot first.
pub fn place_on_grid(body: &Body, track: &Track, distance_m: f64) -> Result<Body, RaceError> {
    if !body.planes.is_empty() {
        return Err(RaceError::Body("wall planes cannot be placed".into()));
    }
    if !(distance_m.is_finite() && distance_m >= 0.0 && distance_m <= track.centre_length()) {
        return Err(RaceError::Grid { slot: 1 });
    }
    let t = track.tangent(track.start);
    let mut grid_track = track.clone();
    grid_track.grid = distance_m;
    let g = grid_track.grid_point();
    let turn = |x: f64, z: f64| (x * t[0] - z * t[1], x * t[1] + z * t[0]);
    let built = body.build();
    let total: f64 = built.mass.iter().sum();
    let (mut cx, mut cz) = (0.0, 0.0);
    for (p, m) in built.pos.iter().zip(&built.mass) {
        cx += p.x * m / total;
        cz += p.z * m / total;
    }
    let (rx, rz) = turn(cx, cz);
    let (dx, dz) = (g[0] - rx, g[1] - rz);
    let mut placed = body.clone();
    for n in &mut placed.nodes {
        let (x, z) = turn(n.pos.x, n.pos.z);
        n.pos = Vec3::new(x + dx, n.pos.y, z + dz);
    }
    for v in &mut placed.velocities {
        let (x, z) = turn(v.velocity.x, v.velocity.z);
        v.velocity = Vec3::new(x, v.velocity.y, z);
    }
    if placed.nodes.iter().any(|n| !finite_vec(n.pos))
        || placed.velocities.iter().any(|v| !finite_vec(v.velocity))
    {
        return Err(RaceError::Body("non-finite grid placement".into()));
    }
    Ok(placed)
}

/// Mass-weighted chassis mean, in node order; exactly cli::race::chassis_centroid.
pub fn car_position(sim: &Sim) -> Vec3 {
    let m: f64 = sim.mass.iter().sum();
    let mut c = Vec3::ZERO;
    for (p, mi) in sim.pos.iter().zip(&sim.mass) {
        c += *p * *mi;
    }
    c * (1.0 / m)
}

/// The single read-only wall rule: chassis nodes and outward tyre edges.
/// Malformed wall geometry is treated as contact, never silently ignored.
pub fn wall_contact(
    track: &Track,
    centre: Vec3,
    heading: f64,
    chassis: &[Vec3],
    mounts: &[Vec<Vec3>],
    half_widths: &[f64],
) -> bool {
    if !finite_vec(centre)
        || !heading.is_finite()
        || mounts.len() != half_widths.len()
        || chassis.iter().any(|&p| !track.on_track(p))
    {
        return true;
    }
    let right = Vec3::new(-heading.sin(), 0.0, heading.cos());
    for (wheel, &half) in mounts.iter().zip(half_widths) {
        if wheel.is_empty() || !half.is_finite() || half < 0.0 {
            return true;
        }
        for &p in wheel {
            let side = (p.x - centre.x) * right.x + (p.z - centre.z) * right.z;
            if !side.is_finite()
                || (side >= 0.0 && !track.on_track(p + right * half))
                || (side <= 0.0 && !track.on_track(p - right * half))
            {
                return true;
            }
        }
    }
    false
}

fn finite_vec(v: Vec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}

fn validate_driver(p: DriverParams) -> Result<(), RaceError> {
    for (name, value) in [
        ("lookahead_k", p.lookahead_k),
        ("lookahead_min", p.lookahead_min),
        ("lookahead_max", p.lookahead_max),
        ("kp", p.kp),
        ("ki", p.ki),
        ("integral_max", p.integral_max),
    ] {
        if !value.is_finite() {
            return Err(RaceError::Driver(name));
        }
    }
    if p.lookahead_min <= 0.0 {
        return Err(RaceError::Driver("lookahead_min"));
    }
    if p.lookahead_max < p.lookahead_min {
        return Err(RaceError::Driver("lookahead_max"));
    }
    if p.integral_max < 0.0 {
        return Err(RaceError::Driver("integral_max"));
    }
    Ok(())
}

struct Geometry {
    front: Vec<usize>,
    rear: Vec<usize>,
    wheelbase: f64,
    half_widths: Vec<f64>,
}

fn mean(pos: &[Vec3], indices: &[usize]) -> Vec3 {
    let mut c = Vec3::ZERO;
    for &i in indices {
        c += pos[i];
    }
    c * (1.0 / indices.len() as f64)
}

impl Geometry {
    fn of(body: &Body) -> Result<Self, RaceError> {
        let bad = |s: &str| RaceError::Body(s.into());
        if body.wheels.is_empty() || !body.planes.is_empty() {
            return Err(bad("need wheels and no wall planes"));
        }
        let group = |name: &str| -> Result<Vec<usize>, RaceError> {
            let g = body.groups.iter().find(|g| g.name == name);
            let g = g.filter(|g| !g.nodes.is_empty()).ok_or_else(|| bad(name))?;
            g.nodes
                .iter()
                .map(|id| body.node_index(*id).ok_or_else(|| bad(name)))
                .collect()
        };
        let front = group("front")?;
        let rear = group("rear")?;
        let built = body.build();
        let mut steered = Vec::new();
        let mut unsteered = Vec::new();
        let mut half_widths = Vec::new();
        for w in &body.wheels {
            if w.mount.is_empty() || !w.width.is_finite() || w.width <= 0.0 {
                return Err(bad("wheel mounts or width invalid"));
            }
            let indices = w
                .mount
                .iter()
                .map(|id| body.node_index(*id).ok_or_else(|| bad("wheel mount id")))
                .collect::<Result<Vec<_>, _>>()?;
            let axle = if w.steerable {
                &mut steered
            } else {
                &mut unsteered
            };
            axle.extend(indices);
            half_widths.push(w.width / 2.0);
        }
        if steered.is_empty() || unsteered.is_empty() {
            return Err(bad("need steerable and unsteered wheels"));
        }
        let d = mean(&built.pos, &steered) - mean(&built.pos, &unsteered);
        let wheelbase = (d.x * d.x + d.z * d.z).sqrt();
        let heading = mean(&built.pos, &front) - mean(&built.pos, &rear);
        if !wheelbase.is_finite() || wheelbase <= 0.0 {
            return Err(bad("wheelbase must be finite and positive"));
        }
        if !finite_vec(heading) || (heading.x == 0.0 && heading.z == 0.0) {
            return Err(bad("front/rear direction must be finite and nonzero"));
        }
        Ok(Self {
            front,
            rear,
            wheelbase,
            half_widths,
        })
    }

    fn heading(&self, sim: &Sim) -> f64 {
        let d = mean(&sim.pos, &self.front) - mean(&sim.pos, &self.rear);
        d.z.atan2(d.x)
    }

    fn state(&self, sim: &Sim) -> CarState {
        let pos = car_position(sim);
        let heading = self.heading(sim);
        let mut v = Vec3::ZERO;
        for (vi, mi) in sim.vel.iter().zip(&sim.mass) {
            v += *vi * *mi;
        }
        let v = v * (1.0 / sim.mass.iter().sum::<f64>());
        CarState {
            x: pos.x,
            z: pos.z,
            heading,
            speed: v.dot(Vec3::new(heading.cos(), 0.0, heading.sin())),
            wheelbase: self.wheelbase,
            max_steer: sim.car.as_ref().unwrap().max_steer,
        }
    }
}

/// Race parsed Body/Track values, with refusals before physics and fixed car order.
/// Counts must fit the exact-integer range of f64 so substep timestamps cannot
/// silently lose an integer before conversion; this is a numerical, not scene, limit.
pub fn run(setup: &RaceSetup<'_>) -> Result<RaceResult, RaceError> {
    if !(1..=MAX_CARS).contains(&setup.cars) {
        return Err(RaceError::CarCount(setup.cars));
    }
    if !(1..=MAX_LAPS).contains(&setup.laps) {
        return Err(RaceError::Laps(setup.laps));
    }
    let interval_count = (setup.time_limit_s / CONTROL_DT * (1.0 - 1e-9)).ceil();
    let exact_integer_limit = 1_u64 << f64::MANTISSA_DIGITS;
    if !setup.time_limit_s.is_finite()
        || setup.time_limit_s <= 0.0
        || !interval_count.is_finite()
        || interval_count > exact_integer_limit as f64
    {
        return Err(RaceError::TimeLimit(setup.time_limit_s));
    }
    if !setup.grid_gap_m.is_finite() || setup.grid_gap_m <= 0.0 {
        return Err(RaceError::GridGap(setup.grid_gap_m));
    }
    validate_driver(setup.driver)?;
    setup.cfg.validate_numbers().map_err(RaceError::Config)?;
    let h = setup.cfg.dt / setup.cfg.substeps as f64;
    let control_count = (CONTROL_DT / h).round();
    if !h.is_finite()
        || h <= 0.0
        || !control_count.is_finite()
        || control_count < 1.0
        || control_count > exact_integer_limit as f64
        || (control_count * h - CONTROL_DT).abs() > CONTROL_DT * 1e-9
    {
        return Err(RaceError::Config(format!(
            "h={h}: {CONTROL_DT} s must be a whole positive number of substeps"
        )));
    }
    let control_substeps = control_count as u64;
    let max_intervals = interval_count as u64;
    if max_intervals
        .checked_mul(control_substeps)
        .is_none_or(|n| n > exact_integer_limit || !(n as f64 * h).is_finite())
    {
        return Err(RaceError::Config(
            "race substep count is unrepresentable".into(),
        ));
    }
    let geometry = Geometry::of(setup.body)?;
    let path: Vec<_> = setup.track.points.iter().map(|p| (p[0], p[1])).collect();
    let points: Vec<_> = path.iter().map(|&(x, z)| Vec3::new(x, 0.0, z)).collect();
    let p = setup.profile;
    let target = speed_profile::profile(&points, p.mu, p.g, p.a_accel, p.a_brake, p.v_max)
        .map_err(RaceError::Profile)?;
    if p.v_max * h > MAX_MOTION / 2.0 {
        return Err(RaceError::SpeedAboveContactBound { v_max: p.v_max });
    }
    let mut placed = Vec::new();
    for slot in 1..=setup.cars {
        let distance = slot_distance(setup.track, slot, setup.grid_gap_m);
        if !distance.is_finite() || distance < 0.0 || distance > setup.track.centre_length() {
            return Err(RaceError::Grid { slot });
        }
        let body = place_on_grid(setup.body, setup.track, distance)?;
        let built = body.build();
        setup
            .cfg
            .validate_body(&built.beams, &built.mass)
            .map_err(RaceError::Config)?;
        placed.push(body);
    }
    let mut one = setup.cfg.clone();
    one.dt = h;
    one.substeps = 1;
    let mut sims = Vec::new();
    for body in &placed {
        let sim =
            settle::car_sim(one.clone(), body, &CarSetup::default()).map_err(RaceError::Config)?;
        let car = sim
            .car
            .as_ref()
            .ok_or_else(|| RaceError::Body("no simulated car".into()))?;
        if car.wheels.len() != setup.body.wheels.len()
            || car
                .wheels
                .iter()
                .zip(&setup.body.wheels)
                .any(|(a, b)| a.p.name != b.name)
        {
            return Err(RaceError::Body(
                "wheel order differs between body and sim".into(),
            ));
        }
        sims.push(sim);
    }
    let settings = ContactSettings::new(
        CONTACT_SKIN,
        CONTACT_RESTITUTION,
        h,
        MAX_MOTION,
        MAX_PENETRATION,
    )
    .map_err(RaceError::Contact)?;
    let placements = vec![CarPlacement { offset: Vec3::ZERO }; setup.cars];
    let multi = MultiSim::new(sims, &placements, settings, h).map_err(RaceError::Multi)?;
    let mut prev: Vec<_> = (0..setup.cars)
        .map(|k| car_position(multi.sim(k)))
        .collect();
    let mut timers: Vec<_> = (0..setup.cars)
        .map(|_| LapTimer::new(setup.track))
        .collect();
    let mut drivers: Vec<_> = (0..setup.cars).map(|_| Driver::new(setup.driver)).collect();
    let cars = (1..=setup.cars)
        .map(|slot| CarResult {
            slot,
            inputs: Vec::new(),
            events: Vec::new(),
            laps: Vec::new(),
            finish_t: None,
            wall_substeps: 0,
            first_wall_t: None,
        })
        .collect();
    let mut result = RaceResult {
        cars,
        substeps: 0,
        h,
        control_substeps,
        contacts: 0,
        impulses: 0,
        end: RaceEnd::TimeLimit,
        multi,
    };
    for _ in 0..max_intervals {
        let mut controls = Vec::with_capacity(setup.cars);
        for (k, driver) in drivers.iter_mut().enumerate() {
            let state = geometry.state(result.multi.sim(k));
            let c = driver.update(&state, &path, &target, CONTROL_DT);
            let c = CarControls {
                throttle: c.throttle,
                brake: c.brake,
                steer: c.steer,
            };
            result.cars[k].inputs.push(c);
            controls.push(c);
        }
        for _ in 0..control_substeps {
            let i = result.substeps;
            let t0 = i as f64 * h;
            let t1 = (i + 1) as f64 * h;
            match result.multi.advance(&controls) {
                Ok(report) => {
                    result.contacts += report.contacts;
                    result.impulses += report.impulses;
                }
                Err(error) => {
                    result.end = RaceEnd::Failed(RaceFailure::Sim { t: t1, error });
                    return Ok(result);
                }
            }
            for (k, (timer, previous)) in timers.iter_mut().zip(&mut prev).enumerate() {
                let sim = result.multi.sim(k);
                let pos = car_position(sim);
                let events = match timer.update(*previous, pos, t0, t1) {
                    Ok(events) => events,
                    Err(error) => {
                        result.end = RaceEnd::Failed(RaceFailure::Timer {
                            car: k,
                            t: t1,
                            error,
                        });
                        return Ok(result);
                    }
                };
                let car = &mut result.cars[k];
                for event in events {
                    if let LapEvent::Lap(lap) = &event {
                        car.laps.push(lap.clone());
                        if car.finish_t.is_none() && car.laps.len() >= setup.laps as usize {
                            car.finish_t = Some(car.laps[setup.laps as usize - 1].end_t);
                        }
                    }
                    car.events.push(event);
                }
                *previous = pos;
                if wall_contact(
                    setup.track,
                    pos,
                    geometry.heading(sim),
                    result.multi.chassis_positions(k),
                    &result.multi.wheel_mount_positions(k),
                    &geometry.half_widths,
                ) {
                    car.wall_substeps += 1;
                    car.first_wall_t.get_or_insert(t1);
                }
            }
            result.substeps += 1;
        }
        if result
            .cars
            .iter()
            .all(|car| car.laps.len() >= setup.laps as usize)
        {
            result.end = RaceEnd::AllFinished;
            break;
        }
    }
    Ok(result)
}
