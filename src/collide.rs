//! Collider planes (walls) and the crumple report.
//!
//! A collider is an infinite plane: a point and a unit normal, the side the normal points to
//! is free space. The ground (y = 0, normal +y) is one of them, built by `Plane::ground()`,
//! and its contact rule is today's ground rule applied along the plane normal
//! (docs/PHYSICS.md, "Plane contact"). The rule is applied by `Sim` after the solver's step,
//! not inside the solver, so every solver gets the same contact.
//!
//! `measure_crumple` is the `--crumple-report` of docs/STAGE6.md, milestone 2: it runs the
//! settle report of `settle::measure_with` and measures, at every substep, how much shorter
//! the body is between two named groups.

use crate::format::{self, Body};
use crate::settle::{measure_body_with, SettleParams, SettleReport};
use crate::vec3::Vec3;
use crate::{Config, Sim, FRICTION_RATE, REST_SPEED, RESTITUTION};

/// An infinite collider plane. `normal` has length 1 (to rounding) and points into free space.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plane {
    pub point: Vec3,
    pub normal: Vec3,
}

impl Plane {
    /// The ground: y = 0, free space above. Its contact reduces to the old ground rule with
    /// exactly the same arithmetic on y (the normal is exactly (0, 1, 0)).
    pub fn ground() -> Plane {
        Plane { point: Vec3::ZERO, normal: Vec3::new(0.0, 1.0, 0.0) }
    }

    /// A plane through `point` with the given normal, which need not be unit length. It is
    /// scaled by its largest component before its length is taken (so a huge or tiny normal
    /// cannot overflow or underflow), then divided by that length. A zero or non-finite
    /// normal is an error.
    pub fn new(point: Vec3, normal: Vec3) -> Result<Plane, String> {
        let finite = |v: Vec3| v.x.is_finite() && v.y.is_finite() && v.z.is_finite();
        if !finite(point) {
            return Err(format!("plane point ({}, {}, {}) is not finite", point.x, point.y, point.z));
        }
        // f64::max skips a NaN, so finiteness is checked on every component first.
        let m = normal.x.abs().max(normal.y.abs()).max(normal.z.abs());
        if !(finite(normal) && m > 0.0) {
            return Err(format!("plane normal ({}, {}, {}) is zero or not finite", normal.x, normal.y, normal.z));
        }
        let s = Vec3::new(normal.x / m, normal.y / m, normal.z / m);
        let len = s.length();
        Ok(Plane { point, normal: s * (1.0 / len) })
    }

    /// The collider of a `plane` line of a `.bsn` file.
    pub fn from_format(p: &format::Plane) -> Result<Plane, String> {
        Plane::new(p.point, p.normal)
    }

    /// Signed distance of `p` from the plane (m): negative means behind it, inside the wall.
    pub fn distance(&self, p: Vec3) -> f64 {
        (p - self.point).dot(self.normal)
    }
}

/// Today's ground contact rule, for every node and every plane, in plane order.
///
/// After a substep's position update, a node with signed distance d < 0 from a plane:
/// 1. is moved back onto the plane along the normal (pos += n * (-d));
/// 2. has its normal velocity vn = v . n reversed and scaled by `RESTITUTION` when it moves
///    in (vn < 0) faster than `REST_SPEED` (-vn > REST_SPEED), or set to 0 when it moves in
///    slower; a node moving out keeps its normal velocity;
/// 3. has its velocity along the plane multiplied by max(0, 1 - FRICTION_RATE * h).
///
/// For the ground this is the old rule operation for operation; `tests/box_wall.rs` compares
/// it bit for bit with a copy of the old in-solver code.
pub fn apply(pos: &mut [Vec3], vel: &mut [Vec3], planes: &[Plane], h: f64) {
    let keep = (1.0 - FRICTION_RATE * h).max(0.0);
    for i in 0..pos.len() {
        for pl in planes {
            let d = pl.distance(pos[i]);
            if d < 0.0 {
                pos[i] += pl.normal * (-d);
                let vn = vel[i].dot(pl.normal);
                let tangent = vel[i] - pl.normal * vn;
                let new_vn = if vn < 0.0 {
                    if -vn > REST_SPEED {
                        -vn * RESTITUTION
                    } else {
                        0.0
                    }
                } else {
                    vn
                };
                vel[i] = tangent * keep + pl.normal * new_vn;
            }
        }
    }
}

/// True if any node is on or behind any plane (signed distance <= 0). For the ground only
/// this is today's `pos.y <= 0` test (`Sim::first_contact_s`).
pub fn any_at_or_behind(pos: &[Vec3], planes: &[Plane]) -> bool {
    pos.iter().any(|p| planes.iter().any(|pl| pl.distance(*p) <= 0.0))
}

/// An impact happened in the substep that just ended: some node sits on a plane (distance
/// within a few rounding errors of zero, because moving a node back onto an oblique plane
/// leaves rounding dust; for the ground the distance is exactly 0) and moves away from it.
/// The contact rule only leaves a node on a plane moving away when it reversed a contact
/// faster than `REST_SPEED`. Generalises `settle`'s ground-only test (settle.rs uses it too).
pub fn impact_this_substep(pos: &[Vec3], vel: &[Vec3], planes: &[Plane]) -> bool {
    for (p, v) in pos.iter().zip(vel.iter()) {
        for pl in planes {
            let scale = 1.0
                + p.x.abs() + p.y.abs() + p.z.abs()
                + pl.point.x.abs() + pl.point.y.abs() + pl.point.z.abs();
            if pl.distance(*p).abs() <= 8.0 * f64::EPSILON * scale && v.dot(pl.normal) > 0.0 {
                return true;
            }
        }
    }
    false
}

// ---------------------------------------------------------------------------------------
// The crumple report

/// The four thresholds of `--crumple-report` and the two groups it measures between.
/// No defaults.
#[derive(Clone, Debug)]
pub struct CrumpleParams {
    /// Group the length is measured from (its centroid).
    pub from: String,
    /// Group the length is measured to.
    pub to: String,
    /// crumple-min: delta(t) must be at least this at every substep with t >= settle-by (m).
    pub min: f64,
    /// crumple-max: delta(t) must be at most this at every substep of the run (m).
    pub max: f64,
    /// crumple-drift: max minus min of delta(t) over t >= settle-by is at most this (m).
    pub drift: f64,
}

impl CrumpleParams {
    pub fn validate(&self) -> Result<(), String> {
        let ok = self.min.is_finite() && self.max.is_finite() && self.drift.is_finite()
            && self.min >= 0.0 && self.max > 0.0 && self.drift >= 0.0 && self.min <= self.max;
        if ok {
            Ok(())
        } else {
            Err("crumple thresholds must be finite: crumple-min >= 0, crumple-max > 0, crumple-min <= crumple-max, crumple-drift >= 0".into())
        }
    }
}

/// What the run measured. L(t) is the distance between the centroids of the two groups and
/// delta(t) = L(0) - L(t): positive means shorter.
#[derive(Clone, Debug)]
pub struct CrumpleReport {
    pub from: String,
    pub to: String,
    pub start_m: f64,
    pub end_m: f64,
    /// delta at the last substep.
    pub delta_m: f64,
    /// Smallest delta over the substeps with t >= settle-by; None if there were none.
    pub delta_min_after: Option<f64>,
    /// Largest minus smallest delta over those substeps; None if there were none.
    pub delta_range_after: Option<f64>,
    /// Largest delta over the whole run (0 at t = 0 counts).
    pub delta_max: f64,
    /// Ok, or the FAIL reason (without the `crumple: FAIL ` prefix).
    pub verdict: Result<(), String>,
}

impl CrumpleReport {
    pub fn is_ok(&self) -> bool {
        self.verdict.is_ok()
    }

    /// The key=value lines then the verdict line, each ending in a newline.
    pub fn render(&self) -> String {
        let opt = |v: Option<f64>| match v {
            Some(x) => format!("{x:.6}"),
            None => "none".to_string(),
        };
        let verdict = match &self.verdict {
            Ok(()) => "crumple: ok".to_string(),
            Err(r) => format!("crumple: FAIL {r}"),
        };
        format!(
            "crumple_groups={},{}\ncrumple_start_m={:.6}\ncrumple_end_m={:.6}\ncrumple_delta_m={:.6}\ncrumple_delta_min_after_settle_by_m={}\ncrumple_delta_range_after_settle_by_m={}\ncrumple_delta_max_m={:.6}\n{verdict}\n",
            self.from, self.to, self.start_m, self.end_m, self.delta_m,
            opt(self.delta_min_after), opt(self.delta_range_after), self.delta_max
        )
    }
}

/// Run `cfg` on `body` (which must have the two groups) and measure the settle report and
/// the crumple at every substep. Returns the settle report and, unless the run aborted, the
/// crumple report. `Err` is a usage problem (no body, a missing group, bad thresholds), not a
/// FAIL verdict.
pub fn measure_crumple(
    cfg: &Config,
    body: Option<&Body>,
    sp: &SettleParams,
    cp: &CrumpleParams,
) -> Result<(SettleReport, Option<CrumpleReport>), String> {
    cp.validate()?;
    let body = body.ok_or_else(|| "--crumple-report needs --body (the groups are named in the body file)".to_string())?;
    let group = |name: &str| -> Result<Vec<usize>, String> {
        let g = body.groups.iter().find(|g| g.name == name).ok_or_else(|| format!("the body has no group `{name}`"))?;
        Ok(g.nodes.iter().map(|id| body.node_index(*id).expect("group names declared nodes")).collect())
    };
    let from = group(&cp.from)?;
    let to = group(&cp.to)?;
    let centroid = |pos: &[Vec3], ix: &[usize]| {
        let mut s = Vec3::ZERO;
        for &i in ix {
            s += pos[i];
        }
        s * (1.0 / ix.len() as f64)
    };
    let length = |pos: &[Vec3]| (centroid(pos, &to) - centroid(pos, &from)).length();
    let start = length(&body.build().pos);

    let h = cfg.dt / cfg.substeps as f64;
    let tol = h * 1e-6;
    let mut last_len = start;
    let mut delta_max = 0.0_f64;
    let mut min_after = f64::INFINITY;
    let mut max_after = f64::NEG_INFINITY;
    let settle = measure_body_with(cfg, Some(body), sp, |sim: &mut Sim, i: u64| {
        let t = i as f64 * h;
        let l = length(&sim.pos);
        let d = start - l;
        last_len = l;
        delta_max = delta_max.max(d);
        if t >= sp.by - tol {
            min_after = min_after.min(d);
            max_after = max_after.max(d);
        }
    });
    if settle.aborted {
        return Ok((settle, None));
    }
    let any_after = min_after.is_finite();
    let verdict = if !any_after {
        Err(format!("rule 1: no substep at or after settle-by = {:.4} s", sp.by))
    } else if min_after < cp.min {
        Err(format!("rule 1: crumple {min_after:.6} m < crumple-min {} m at t >= {:.4} s", cp.min, sp.by))
    } else if delta_max > cp.max {
        Err(format!("rule 2: crumple {delta_max:.6} m > crumple-max {} m", cp.max))
    } else if max_after - min_after > cp.drift {
        Err(format!("rule 3: crumple drifts {:.6} m > crumple-drift {} m after t = {:.4} s", max_after - min_after, cp.drift, sp.by))
    } else {
        Ok(())
    };
    let report = CrumpleReport {
        from: cp.from.clone(),
        to: cp.to.clone(),
        start_m: start,
        end_m: last_len,
        delta_m: start - last_len,
        delta_min_after: if any_after { Some(min_after) } else { None },
        delta_range_after: if any_after { Some(max_after - min_after) } else { None },
        delta_max,
        verdict,
    };
    Ok((settle, Some(report)))
}
