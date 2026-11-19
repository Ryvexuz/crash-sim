//! The settle report (docs/STAGE6.md, "The settle report"): runs a sim and measures, at every
//! physics substep (never only at output steps), whether the body comes to rest and stays there.
//!
//! This is a library module so that other bodies (walls, wheels) can reuse the energy function,
//! the impact detection and the verdict rules. It uses only the public `Sim` API.
//!
//! There are no default thresholds here: every limit is a field of `SettleParams` that the
//! caller must give.

use crate::vec3::Vec3;
use crate::{max_strain, Beam, Config, Sim};

/// Total energy in joules: kinetic plus gravitational (potential measured from y = 0) plus
/// elastic beam energy.
///
///   E = sum over nodes of (1/2) m_i |v_i|^2 + m_i g y_i
///       + sum over beams of (1/2) k_b (len_b - rest_b)^2
///
/// Beams use their own `stiffness` and current `rest` length. (Broken beams, nb-34, and wheels,
/// nb-39, add their terms here when those exist.)
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

/// An impact happened in the substep that just ended: a node sits exactly on the ground plane
/// and moves up. The contact rule only leaves a node at y = 0 moving up when it reversed a
/// contact faster than `REST_SPEED` (a slower contact is set to vy = 0).
fn impact_this_substep(sim: &Sim) -> bool {
    sim.pos.iter().zip(sim.vel.iter()).any(|(p, v)| p.y == 0.0 && v.y > 0.0)
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

/// Run `cfg` and measure the settle report at every substep.
pub fn measure(cfg: &Config, p: &SettleParams) -> SettleReport {
    measure_with(cfg, p, |_, _| {})
}

/// Like `measure`, with a hook called after every substep (and before it is measured) with the
/// sim and the 1-based substep number. For tests that plant a fault (an energy injection, a
/// NaN) to prove the report can say FAIL; the production path passes a no-op.
pub fn measure_with(cfg: &Config, p: &SettleParams, mut hook: impl FnMut(&mut Sim, u64)) -> SettleReport {
    if let Err(e) = cfg.validate().and_then(|_| p.validate()) {
        return aborted(e);
    }
    let h = cfg.dt / cfg.substeps as f64;
    let outer = (cfg.duration / cfg.dt).round() as u64;
    let total = outer * cfg.substeps as u64;
    // One `advance` of this config is exactly one substep, so the substep loop is here, where
    // it can be observed. Same h, same order of operations as the real run.
    let mut one = cfg.clone();
    one.dt = h;
    one.substeps = 1;
    let mut sim = Sim::new(one);
    let g = cfg.gravity;
    let energy = |s: &Sim| total_energy(&s.pos, &s.vel, &s.beams, &s.mass, g);

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
    let mut peak = max_strain(&sim.beams, &sim.pos);
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
        peak = peak.max(max_strain(&sim.beams, &sim.pos));
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
