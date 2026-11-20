//! The settle report (docs/STAGE6.md, "The settle report"): runs a sim and measures, at every
//! physics substep (never only at output steps), whether the body comes to rest and stays there.
//!
//! This is a library module so that other bodies (walls, wheels) can reuse the energy function,
//! the impact detection and the verdict rules. It uses only the public `Sim` API.
//!
//! There are no default thresholds here: every limit is a field of `SettleParams` that the
//! caller must give.

use crate::collide;
use crate::vec3::Vec3;
use crate::{max_strain_unbroken, vehicle, Beam, Body, Config, Sim};

/// Total energy in joules: kinetic plus gravitational (potential measured from y = 0) plus
/// elastic beam energy.
///
///   E = sum over nodes of (1/2) m_i |v_i|^2 + m_i g y_i
///       + sum over beams of (1/2) k_b (len_b - rest_b)^2
///
/// Beams use their own `stiffness` and current `rest` length. The caller passes only the beams
/// that have not broken (`Sim::live_beams`): a broken beam stores no energy. A body with wheels
/// is measured with `Sim::total_energy` instead, which adds the wheels' terms and the tilted
/// gravity (this function knows neither).
pub fn total_energy(pos: &[Vec3], vel: &[Vec3], beams: &[Beam], mass: &[f64], gravity: f64) -> f64 {
    let mut e = 0.0;
    for i in 0..pos.len() {
        let v = vel[i];
        e += 0.5 * mass[i] * (v.x * v.x + v.y * v.y + v.z * v.z) + mass[i] * gravity * pos[i].y;
    }
    for b in beams {
        let stretch = (pos[b.a] - pos[b.b]).length() - b.rest;
        e += 0.5 * b.stiffness * stretch * stretch;
    }
    e
}

/// An impact happened in the substep that just ended: a node sits on a collider plane (the
/// ground or a wall, `collide::impact_this_substep`) and moves away from it. The contact rule
/// only leaves a node on a plane moving away when it reversed a contact faster than `REST_SPEED`.
fn impact_this_substep(sim: &Sim) -> bool {
    collide::impact_this_substep(&sim.pos, &sim.vel, &sim.planes)
}

/// The five thresholds of `--settle-report`. No defaults.
#[derive(Clone, Copy, Debug)]
pub struct SettleParams {
    /// T: from this time on, the body must be at rest (s).
    pub by: f64,
    /// S: the run must go on at least this long after T (s).
    pub hold: f64,
    /// V: "at rest" means max node speed below this (m/s).
    pub speed: f64,
    /// E: after T, total energy may rise by at most this (J).
    pub energy_rise: f64,
    /// G: over the whole run, E(t) may exceed E(0) by at most G * E(0).
    pub energy_gain: f64,
}

impl SettleParams {
    pub fn validate(&self) -> Result<(), String> {
        let f = |x: f64| x.is_finite();
        if f(self.by) && self.by >= 0.0 && f(self.hold) && self.hold >= 0.0 && f(self.speed) && self.speed > 0.0
            && f(self.energy_rise) && self.energy_rise >= 0.0 && f(self.energy_gain) && self.energy_gain >= 0.0
        {
            Ok(())
        } else {
            Err("settle thresholds must be finite: settle-by >= 0, settle-hold >= 0, settle-speed > 0, energy-rise >= 0, energy-gain >= 0".into())
        }
    }
}

/// What the run measured, and the verdict.
#[derive(Clone, Debug)]
pub struct SettleReport {
    /// Time of the last impact (substep time, s), or None.
    pub last_impact_s: Option<f64>,
    /// Earliest substep from which max node speed stays below V to the end; None = never.
    pub settled_from_s: Option<f64>,
    /// Largest max node speed at t >= T.
    pub max_speed_after_by: f64,
    /// Largest rise E(t2) - E(t1), t1 <= t2, over the substeps with t >= T.
    pub energy_rise_after_by_j: f64,
    /// Largest (E(t) - E(0)) / E(0) over the run.
    pub max_energy_gain_frac: f64,
    /// Largest beam strain over every substep (and t = 0).
    pub peak_strain: f64,
    /// Mean y of all nodes at the last substep.
    pub final_center_y: f64,
    /// Ok, or the FAIL reason (without the `settle: FAIL ` prefix).
    pub verdict: Result<(), String>,
    /// True when the run aborted (non-finite state, strain limit or a refused config): the
    /// measured fields are then partial and `render` prints only the FAIL line.
    pub aborted: bool,
}

impl SettleReport {
    /// The report text: the key=value lines then the verdict line, each ending in a newline.
    pub fn render(&self) -> String {
        if self.aborted {
            return format!("settle: FAIL {}\n", self.verdict.clone().unwrap_err());
        }
        let impact = match self.last_impact_s {
            Some(t) => format!("{t:.4}"),
            None => "none".to_string(),
        };
        let settled = match self.settled_from_s {
            Some(t) => format!("{t:.4}"),
            None => "never".to_string(),
        };
        let verdict = match &self.verdict {
            Ok(()) => "settle: ok".to_string(),
            Err(r) => format!("settle: FAIL {r}"),
        };
        format!(
            "last_impact_s={impact}\nsettled_from_s={settled}\nmax_speed_after_settle_by={:.3e}\nenergy_rise_after_settle_by_j={:.3e}\nmax_energy_gain_frac={:.3e}\npeak_strain={:.6}\nfinal_center_y={:.10}\n{verdict}\n",
            self.max_speed_after_by, self.energy_rise_after_by_j, self.max_energy_gain_frac, self.peak_strain, self.final_center_y
        )
    }

    pub fn is_ok(&self) -> bool {
        self.verdict.is_ok()
    }
}

fn aborted(reason: String) -> SettleReport {
    SettleReport {
        last_impact_s: None,
        settled_from_s: None,
        max_speed_after_by: 0.0,
        energy_rise_after_by_j: 0.0,
        max_energy_gain_frac: 0.0,
        peak_strain: 0.0,
        final_center_y: 0.0,
        verdict: Err(reason),
        aborted: true,
    }
}

/// How a body with wheels is set up for a run (`Sim::from_car`): which controls are on and
/// whether gravity is tilted into a slope. The default is a free-rolling car on flat ground.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CarSetup {
    /// Full brake from t = 0 (the one input nb-39 simulates; throttle and steering are nb-40).
    pub brake: bool,
    /// `--slope-grip-frac f`: tilt gravity so that tan theta = f x mu (`vehicle::slope_angle`).
    pub grip_frac: Option<f64>,
}

/// The one `--input` file nb-39 can run: a single line `0 0 <brake> 0` (t = 0, no throttle,
/// brake 0 or 1, no steering), which is what `scenarios/full_brake.input` holds. Returns the
/// brake flag. Anything else is refused by name, never read as something near it: timed
/// scripts, throttle and steering belong to nb-40.
pub fn parse_brake_input(text: &str) -> Result<bool, String> {
    if text.contains('\r') {
        return Err("input: carriage returns are not allowed (use \\n line ends)".into());
    }
    let lines: Vec<&str> = text.lines().filter(|l| !l.trim().is_empty()).collect();
    let [line] = lines.as_slice() else {
        return Err(format!(
            "input: this run reads exactly one line `0 0 <brake> 0`, found {} (timed input scripts are not simulated yet)",
            lines.len()
        ));
    };
    let f: Vec<&str> = line.split_whitespace().collect();
    if f.len() != 4 {
        return Err(format!("input: a line is `t throttle brake steer`, found {} fields in `{line}`", f.len()));
    }
    let mut v = [0.0_f64; 4];
    for (i, x) in f.iter().enumerate() {
        v[i] = x
            .parse::<f64>()
            .ok()
            .filter(|n| n.is_finite())
            .ok_or_else(|| format!("input: `{x}` is not a finite number"))?;
    }
    if v[0] != 0.0 {
        return Err(format!("input: the line must start at t = 0, found t = {}", v[0]));
    }
    if v[1] != 0.0 || v[3] != 0.0 {
        return Err("input: throttle and steer must be 0 (they are not simulated yet)".into());
    }
    if v[2] == 0.0 {
        Ok(false)
    } else if v[2] == 1.0 {
        Ok(true)
    } else {
        Err(format!("input: brake must be 0 or 1 (a partial brake is not simulated yet), found {}", v[2]))
    }
}

/// The car's sim for `body`: `Sim::from_car`, the slope tilt and the brake flag. One place for
/// every run of a car (settle report, slope report), so none can skip a refusal.
pub fn car_sim(cfg: Config, body: &Body, setup: &CarSetup) -> Result<Sim, String> {
    if body.wheels.is_empty() {
        return Err("the body has no wheels: a slope or an input only applies to a body with wheels".into());
    }
    let tilt = match setup.grip_frac {
        Some(f) => Some(vehicle::slope_angle(body, f)?),
        None => None,
    };
    let mut sim = Sim::from_car(cfg, body)?;
    if let Some(theta) = tilt {
        sim.set_gravity_tilt(theta)?;
    }
    match sim.car.as_mut() {
        Some(car) => car.brake = setup.brake,
        None => return Err("internal: Sim::from_car built no car for a body with wheels".into()),
    }
    Ok(sim)
}

/// The slope report of `--slope-report` (docs/STAGE6.md check 3.4).
#[derive(Clone, Copy, Debug)]
pub struct SlopeReport {
    /// The tyre grip coefficient mu.
    pub mu: f64,
    /// tan theta = f x mu.
    pub tan: f64,
    /// (u(to) - u(from)) / (to - from), u = the chassis centre-of-mass velocity along +x.
    pub accel: f64,
    /// g (sin theta - mu cos theta).
    pub predicted: f64,
}

impl SlopeReport {
    pub fn render(&self) -> String {
        format!(
            "slope_mu={:.4}\nslope_tan={:.6}\nslope_accel_mps2={:.4}\nslope_predicted_mps2={:.4}\n",
            self.mu, self.tan, self.accel, self.predicted
        )
    }
}

/// Chassis centre-of-mass velocity along +x (nodes only, as the check defines u).
fn chassis_com_vx(sim: &Sim) -> f64 {
    let m: f64 = sim.mass.iter().sum();
    sim.vel.iter().zip(&sim.mass).map(|(v, w)| v.x * w).sum::<f64>() / m
}

/// Run the car of `body` on the slope `setup` gives, for `to` seconds of substeps (`cfg.dt /
/// cfg.substeps` each), and report the slide acceleration between `from` and `to`. Both times
/// must be whole substeps with 0 <= from < to, and the slope is required.
pub fn measure_slope(cfg: &Config, body: &Body, setup: &CarSetup, from: f64, to: f64) -> Result<SlopeReport, String> {
    cfg.validate_numbers()?;
    let grip_frac = setup.grip_frac.ok_or("a slope report needs a slope (--slope-grip-frac)")?;
    let h = cfg.dt / cfg.substeps as f64;
    let steps_of = |t: f64, what: &str| -> Result<u64, String> {
        if !(t.is_finite() && t >= 0.0) {
            return Err(format!("{what} = {t} must be a finite number >= 0"));
        }
        let n = (t / h).round();
        if (t / h - n).abs() > 1e-6 {
            return Err(format!("{what} = {t} s is not a whole number of {h} s substeps"));
        }
        Ok(n as u64)
    };
    let (n_from, n_to) = (steps_of(from, "slope-from")?, steps_of(to, "slope-to")?);
    if n_from >= n_to {
        return Err(format!("slope-from = {from} must be below slope-to = {to}"));
    }
    let mut one = cfg.clone();
    one.dt = h;
    one.substeps = 1;
    let mut sim = car_sim(one, body, setup)?;
    let (mut u_from, mut u_to) = (None, None);
    if n_from == 0 {
        u_from = Some(chassis_com_vx(&sim));
    }
    for i in 1..=n_to {
        sim.advance();
        sim.check().map_err(|e| format!("unstable at t = {:.4} s: {e}", i as f64 * h))?;
        if i == n_from {
            u_from = Some(chassis_com_vx(&sim));
        }
        if i == n_to {
            u_to = Some(chassis_com_vx(&sim));
        }
    }
    let (Some(u0), Some(u1)) = (u_from, u_to) else {
        return Err("slope report: no velocity sampled".into());
    };
    let accel = (u1 - u0) / (to - from);
    if !accel.is_finite() {
        return Err(format!("slope acceleration is not finite (u {u0} to {u1})"));
    }
    let theta = vehicle::slope_angle(body, grip_frac)?;
    let mu = body.tyres[0].mu;
    Ok(SlopeReport { mu, tan: theta.tan(), accel, predicted: cfg.gravity * (theta.sin() - mu * theta.cos()) })
}

/// Run `cfg` and measure the settle report at every substep.
pub fn measure(cfg: &Config, p: &SettleParams) -> SettleReport {
    measure_with(cfg, p, |_, _| {})
}

/// Like `measure`, for a body loaded from a `.bsn` file (`None`: the built-in cube).
pub fn measure_body(cfg: &Config, body: Option<&Body>, p: &SettleParams) -> SettleReport {
    measure_body_with(cfg, body, p, |_, _| {})
}

/// Like `measure`, with a hook called after every substep (and before it is measured) with the
/// sim and the 1-based substep number. For tests that plant a fault (an energy injection, a
/// NaN) to prove the report can say FAIL; the production path passes a no-op.
pub fn measure_with(cfg: &Config, p: &SettleParams, hook: impl FnMut(&mut Sim, u64)) -> SettleReport {
    measure_body_with(cfg, None, p, hook)
}

/// `measure_with` for a cube (`body` None) or a body loaded from a `.bsn` file. A refused body
/// or config comes back as an aborted report (the sim's own validation runs for a body). A body
/// with wheels is refused here (`Sim::from_body`): wheels are simulated only by `measure_car`,
/// so every other caller of this function (the crumple report) still refuses them.
pub fn measure_body_with(cfg: &Config, body: Option<&Body>, p: &SettleParams, hook: impl FnMut(&mut Sim, u64)) -> SettleReport {
    measure_inner(cfg, body, None, p, hook)
}

/// The settle report of a car: `Sim::from_car` with `setup` (brake, slope), and the energy of
/// `Sim::total_energy` (wheels, suspension, tyres and the tilted gravity included).
pub fn measure_car(cfg: &Config, body: &Body, setup: &CarSetup, p: &SettleParams) -> SettleReport {
    measure_car_with(cfg, body, setup, p, |_, _| {})
}

/// `measure_car` with a per-substep hook, as `measure_with`.
pub fn measure_car_with(cfg: &Config, body: &Body, setup: &CarSetup, p: &SettleParams, hook: impl FnMut(&mut Sim, u64)) -> SettleReport {
    measure_inner(cfg, Some(body), Some(setup), p, hook)
}

fn measure_inner(cfg: &Config, body: Option<&Body>, car: Option<&CarSetup>, p: &SettleParams, mut hook: impl FnMut(&mut Sim, u64)) -> SettleReport {
    if let Err(e) = p.validate() {
        return aborted(e);
    }
    // The cube gets the full check; a body gets the finite-number rules on the original dt and
    // substeps here (its step limit is checked by `Sim::from_body` on h, which is the same h).
    let checked = if body.is_none() { cfg.validate() } else { cfg.validate_numbers() };
    if let Err(e) = checked {
        return aborted(e);
    }
    let h = cfg.dt / cfg.substeps as f64;
    let outer = crate::step_count(cfg);
    let total = outer * cfg.substeps as u64;
    // One `advance` of this config is exactly one substep, so the substep loop is here, where
    // it can be observed. Same h, same order of operations as the real run.
    let mut one = cfg.clone();
    one.dt = h;
    one.substeps = 1;
    let mut sim = match (body, car) {
        (Some(b), Some(setup)) => match car_sim(one, b, setup) {
            Ok(s) => s,
            Err(e) => return aborted(e),
        },
        (None, _) => Sim::new(one),
        (Some(b), None) => match Sim::from_body(one, b) {
            Ok(s) => s,
            Err(e) => return aborted(e),
        },
    };
    let g = cfg.gravity;
    let is_car = car.is_some();
    let energy = |s: &Sim| if is_car { s.total_energy() } else { total_energy(&s.pos, &s.vel, &s.live_beams(), &s.mass, g) };

    let e0 = energy(&sim);
    if !(e0.is_finite() && e0 > 0.0) {
        return aborted(format!("energy at t = 0 is {e0}, not a positive finite number"));
    }
    // Compare times with a tolerance far below one substep: t = i * h is not exact.
    let tol = h * 1e-6;
    let time = |i: u64| i as f64 * h;

    let mut last_impact: Option<u64> = None;
    let mut last_fast: Option<u64> = None; // last substep with max speed >= V
    let mut max_speed_after = 0.0_f64;
    // The reference for the rise is the energy at T itself. The loop starts at substep 1, so
    // when T = 0 the state at T is the initial one: seed with E(0), or a rise in the first
    // substep followed by a fall would read as zero.
    let mut min_e_after = if p.by - tol <= 0.0 { e0 } else { f64::INFINITY };
    let mut rise_after = 0.0_f64;
    let mut gain = 0.0_f64;
    let mut peak = max_strain_unbroken(&sim.beams, &sim.pos, &sim.broken);
    let mut first_speed_fail: Option<(f64, f64)> = None; // (speed, t) of the first substep at or after T
    for i in 1..=total {
        sim.advance();
        hook(&mut sim, i);
        let t = time(i);
        if let Err(e) = sim.check() {
            return aborted(format!("unstable at t = {t:.4} s: {e}"));
        }
        if impact_this_substep(&sim) {
            last_impact = Some(i);
        }
        let speed = sim.max_speed();
        if speed >= p.speed {
            last_fast = Some(i);
        }
        let e = energy(&sim);
        if !e.is_finite() {
            return aborted(format!("unstable at t = {t:.4} s: total energy is not finite"));
        }
        peak = peak.max(max_strain_unbroken(&sim.beams, &sim.pos, &sim.broken));
        gain = gain.max((e - e0) / e0);
        if t >= p.by - tol {
            max_speed_after = max_speed_after.max(speed);
            if speed >= p.speed && first_speed_fail.is_none() {
                first_speed_fail = Some((speed, t));
            }
            min_e_after = min_e_after.min(e);
            rise_after = rise_after.max(e - min_e_after);
        }
    }
    let n = sim.pos.len() as f64;
    let final_center_y = sim.pos.iter().map(|q| q.y).sum::<f64>() / n;
    let end_s = time(total);
    let settled_from_s = match last_fast {
        None if total > 0 => Some(time(1)),
        Some(i) if i < total => Some(time(i + 1)),
        _ => None,
    };

    let verdict = if end_s + tol < p.by + p.hold {
        Err(format!("hold: the run ends at t={end_s:.4} s, before settle-by + settle-hold = {:.4} s", p.by + p.hold))
    } else if let Some((s, t)) = first_speed_fail {
        Err(format!("speed {s:.4} m/s >= {} at t={t:.4}", p.speed))
    } else if rise_after > p.energy_rise {
        Err(format!("energy rise {rise_after:.3e} J > {} J after t={:.4}", p.energy_rise, p.by))
    } else if gain > p.energy_gain {
        Err(format!("energy gain {gain:.3e} > {} of E(0) = {e0:.3} J", p.energy_gain))
    } else {
        Ok(())
    };
    SettleReport {
        last_impact_s: last_impact.map(time),
        settled_from_s,
        max_speed_after_by: max_speed_after,
        energy_rise_after_by_j: rise_after,
        max_energy_gain_frac: gain,
        peak_strain: peak,
        final_center_y,
        verdict,
        aborted: false,
    }
}
