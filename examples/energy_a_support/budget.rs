//! Budget judgment is separate from arithmetic validity: a finite breach must not
//! truncate Test A's observations or summaries. The round-3 bound remains exactly
//! 2% of initial kinetic energy, with no floor and no residual booked as a loss.
//!
//! The yield cross-check is yield force times plastic flow:
//! `k * |length - rest_new| * |rest_old - rest_new|`, for tension and compression.
//! The quadratic remainder `0.5 * k * (rest_old - rest_new)^2` is reported separately
//! as `yield_overshoot_j`. It diagnoses integrator overshoot of the explicit spring's
//! accounting, not physical dissipation; it belongs to NO loss term or budget sum.
//! We observe the real `apply_yield` pass without adding a force cap or changing the
//! solver. Round 7's ledger instead uses each actual start force dotted with the
//! average of its old and solver-end velocities, times h. Summing these independent
//! works gives the symplectic Euler kinetic change, including cross-force terms.
//! Beam plastic_j is minus spring work minus the stored-energy change, including
//! the actual yield pass. This discrete quantity is signed, even on an elastic
//! interval; it includes the spring's integration defect, not the global residual.
//! yield_at_cap_j and yield_overshoot_j remain printed cross-checks only. In
//! particular, overshoot is not added to the force-work plastic ledger.
//! Suspension/stop and tyre spring work are evaluated too; their difference from
//! geometric stored-energy changes remains in the signed residual. No residual is
//! folded into a physical term. The unchanged 2% bound still judges the whole run.

use super::{beam_elastic, finite, Measurement, Result, Row, Totals, MATERIALS};
use crash_sim::vec3::Vec3;
use crash_sim::vehicle::{Car, AIR_DENSITY, STOP_STIFFNESS_FACTOR};
use crash_sim::Sim;
use std::io::{self, Write};

pub const BUDGET_FRACTION: f64 = 0.02;

/// Work done ON a degree of freedom. The midpoint avoids overflow at equal large
/// velocities and retains equal subnormals. The supported binary64 band requires
/// finite vector differences, dot products and final work; underflow retains IEEE rounding.
/// Invalid arithmetic is an error, never a zero work contribution.
pub fn force_work(force: Vec3, old: Vec3, new: Vec3, h: f64) -> Result<f64> {
    finite(
        "force work inputs",
        &[
            force.x, force.y, force.z, old.x, old.y, old.z, new.x, new.y, new.z, h,
        ],
    )?;
    if h <= 0.0 {
        return Err("invalid force work timestep".into());
    }
    let midpoint = |a: f64, b: f64| {
        if a.is_sign_negative() == b.is_sign_negative() {
            a + (b - a) * 0.5
        } else {
            (a + b) * 0.5
        }
    };
    let average = Vec3::new(
        midpoint(old.x, new.x),
        midpoint(old.y, new.y),
        midpoint(old.z, new.z),
    );
    let work = force.dot(average) * h;
    finite("force work", &[work])?;
    Ok(work)
}

/// Independent beam forces and stores from the force-evaluation state. The actual
/// solver-end velocities must be supplied BEFORE carrier/contact projections.
pub struct BeamWork {
    old_vel: Vec<Vec3>,
    elastic: Vec<f64>,
    spring: Vec<Vec3>,
    damper: Vec<Vec3>,
    ends: Vec<(usize, usize)>,
    h: f64,
}

impl BeamWork {
    pub fn start(sim: &Sim, h: f64) -> Result<Self> {
        if sim.pos.len() != sim.vel.len() {
            return Err("invalid force-start beam dimensions".into());
        }
        // Reuse the existing beam input/degenerate-length validation, NOT its
        // left-end work as ledger data. Test A's finite intermediate band applies.
        super::beam_damper(sim, h)?;
        let elastic = beam_elastic(sim);
        finite("force-start elastic energy", &elastic)?;
        let mut spring = Vec::with_capacity(sim.beams.len());
        let mut damper = Vec::with_capacity(sim.beams.len());
        let mut ends = Vec::with_capacity(sim.beams.len());
        for b in &sim.beams {
            let d = sim.pos[b.b] - sim.pos[b.a];
            let length = d.length();
            let dir = if length < 1e-12 {
                Vec3::ZERO
            } else {
                d * (1.0 / length)
            };
            let fs = dir * (b.stiffness * (length - b.rest));
            let fd = dir * (b.damping * (sim.vel[b.b] - sim.vel[b.a]).dot(dir));
            finite("beam work forces", &[fs.x, fs.y, fs.z, fd.x, fd.y, fd.z])?;
            spring.push(fs);
            damper.push(fd);
            ends.push((b.a, b.b));
        }
        Ok(Self {
            old_vel: sim.vel.clone(),
            elastic,
            spring,
            damper,
            ends,
            h,
        })
    }

    pub fn finish(self, sim: &Sim, materials: &[usize], totals: &mut Totals) -> Result<()> {
        if sim.vel.len() != self.old_vel.len()
            || sim.pos.len() != self.old_vel.len()
            || sim.beams.len() != self.ends.len()
            || materials.len() != self.ends.len()
            || materials.iter().any(|m| *m >= MATERIALS.len())
            || sim
                .beams
                .iter()
                .zip(&self.ends)
                .any(|(b, &(a, z))| b.a != a || b.b != z)
        {
            return Err("invalid force-end beam state".into());
        }
        let end_elastic = beam_elastic(sim);
        finite("force-end elastic energy", &end_elastic)?;
        let mut next = totals.clone();
        for (i, (&m, &(a, b))) in materials.iter().zip(&self.ends).enumerate() {
            // Equal/opposite forces at BOTH nodes, with velocities after ALL
            // forces: no per-force virtual velocity update can lose cross terms.
            let pair = |f: Vec3| -> Result<f64> {
                Ok(force_work(f, self.old_vel[a], sim.vel[a], self.h)?
                    + force_work(-f, self.old_vel[b], sim.vel[b], self.h)?)
            };
            let spring = pair(self.spring[i])?;
            let damper = pair(self.damper[i])?;
            next.plastic[m] += -spring - (end_elastic[i] - self.elastic[i]);
            next.damper[m] -= damper;
        }
        next.check()?;
        *totals = next;
        Ok(())
    }
}

/// Force components on nodes and on unconstrained wheels. Carrier force on the
/// wheels removes the transverse part; its node counterpart is the actual extra
/// force minus the independently reconstructed suspension and aero forces.
#[derive(Clone)]
struct Forces {
    nodes: Vec<Vec3>,
    wheels: Vec<Vec3>,
}

impl Forces {
    fn zero(nodes: usize, wheels: usize) -> Self {
        Self {
            nodes: vec![Vec3::ZERO; nodes],
            wheels: vec![Vec3::ZERO; wheels],
        }
    }

    fn work(
        &self,
        old_nodes: &[Vec3],
        old_wheels: &[Vec3],
        end: &Sim,
        shadow: &Car,
        h: f64,
    ) -> Result<f64> {
        if self.nodes.len() != old_nodes.len()
            || self.nodes.len() != end.vel.len()
            || self.wheels.len() != old_wheels.len()
            || self.wheels.len() != shadow.wheels.len()
        {
            return Err("invalid component force dimensions".into());
        }
        let mut work = 0.0;
        for ((f, a), b) in self.nodes.iter().zip(old_nodes).zip(&end.vel) {
            work += force_work(*f, *a, *b, h)?;
        }
        for ((f, a), b) in self.wheels.iter().zip(old_wheels).zip(&shadow.wheels) {
            work += force_work(*f, *a, b.vel, h)?;
        }
        finite("component force work", &[work])?;
        Ok(work)
    }

    fn check(&self) -> Result<()> {
        for f in self.nodes.iter().chain(&self.wheels) {
            finite("auxiliary work force", &[f.x, f.y, f.z])?;
        }
        Ok(())
    }
}

pub(super) struct ForceWork {
    old_nodes: Vec<Vec3>,
    old_wheels: Vec<Vec3>,
    beams: BeamWork,
    shadow: Car,
    suspension_spring: Forces,
    suspension_damper: Forces,
    tyre_spring: Forces,
    tyre_damper: Forces,
    aero: Forces,
    carrier: Forces,
    h: f64,
}

impl ForceWork {
    pub(super) fn start(sim: &Sim, extra: &[Vec3], h: f64, totals: &mut Totals) -> Result<Self> {
        let beams = BeamWork::start(sim, h)?;
        let car = sim.car.as_ref().ok_or("missing force-start car")?;
        if extra.len() != sim.vel.len() || sim.cfg.gravity != 0.0 {
            return Err("invalid auxiliary force state".into());
        }
        // The legacy helper constructs the unconstrained wheel trajectory and
        // isolates rolling spin loss and actual radial impulse. Discard ALL its
        // left-end force work; only these operation-local quantities are copied.
        let mut local = Totals::default();
        let shadow = super::vehicle_work(sim, extra, h, &mut local)?;
        let zero = Forces::zero(sim.vel.len(), car.wheels.len());
        let mut work = Self {
            old_nodes: sim.vel.clone(),
            old_wheels: car.wheels.iter().map(|w| w.vel).collect(),
            beams,
            shadow,
            suspension_spring: zero.clone(),
            suspension_damper: zero.clone(),
            tyre_spring: zero.clone(),
            tyre_damper: zero.clone(),
            aero: zero.clone(),
            carrier: zero,
            h,
        };
        let up = car.frame.up;
        for (j, w) in car.wheels.iter().enumerate() {
            let share = 1.0 / w.p.mount.len() as f64;
            let vc = w.p.mount.iter().fold(Vec3::ZERO, |s, &i| s + sim.vel[i]) * share;
            let sd = (vc - w.vel).dot(up);
            let stop = if w.length < w.p.travel_min {
                STOP_STIFFNESS_FACTOR * w.p.spring * (w.p.travel_min - w.length)
            } else if w.length > w.p.travel_max {
                -STOP_STIFFNESS_FACTOR * w.p.spring * (w.length - w.p.travel_max)
            } else {
                0.0
            };
            let spring = up * (w.p.spring * (w.p.rest - w.length) + stop);
            let damper = up * (-w.p.damper * sd);
            for &i in &w.p.mount {
                work.suspension_spring.nodes[i] += spring * share;
                work.suspension_damper.nodes[i] += damper * share;
            }
            work.suspension_spring.wheels[j] = -spring;
            work.suspension_damper.wheels[j] = -damper;
            let mut raw = -(spring + damper);
            for (k, pl) in sim.planes.iter().enumerate() {
                let pen = w.p.radius - pl.distance(w.pos);
                if pen > 0.0 {
                    let elastic = pl.normal * (w.p.radial_stiffness * pen);
                    work.tyre_spring.wheels[j] += elastic;
                    work.tyre_damper.wheels[j] += pl.normal * w.load[k] - elastic;
                }
                if w.load[k] > 0.0 {
                    raw += pl.normal * w.load[k];
                }
            }
            work.carrier.wheels[j] = -(raw - up * raw.dot(up));
        }
        let mass: f64 = sim.mass.iter().sum();
        let v = sim
            .vel
            .iter()
            .zip(&sim.mass)
            .fold(Vec3::ZERO, |p, (v, m)| p + *v * *m)
            * (1.0 / mass);
        let rate = (0.5 * AIR_DENSITY * car.drag_area * v.length() / mass).min(1.0 / h);
        let acceleration = v * (-rate);
        for (i, &m) in sim.mass.iter().enumerate() {
            work.aero.nodes[i] = acceleration * m;
            work.carrier.nodes[i] = extra[i]
                - (work.suspension_spring.nodes[i]
                    + work.suspension_damper.nodes[i]
                    + work.aero.nodes[i]);
        }
        for f in [
            &work.suspension_spring,
            &work.suspension_damper,
            &work.tyre_spring,
            &work.tyre_damper,
            &work.aero,
            &work.carrier,
        ] {
            f.check()?;
        }
        let mut next = totals.clone();
        next.rolling += local.rolling;
        next.tyre_impulse += local.tyre_impulse;
        next.check()?;
        *totals = next;
        Ok(work)
    }

    pub(super) fn finish(self, sim: &Sim, materials: &[usize], totals: &mut Totals) -> Result<Car> {
        let mut next = totals.clone();
        let eval = |f: &Forces| f.work(&self.old_nodes, &self.old_wheels, sim, &self.shadow, self.h);
        // Conservative work is evaluated independently, not inferred from K.
        // Its geometric storage defect is deliberately left in the residual.
        eval(&self.suspension_spring)?;
        eval(&self.tyre_spring)?;
        next.suspension -= eval(&self.suspension_damper)?;
        next.tyre_damper -= eval(&self.tyre_damper)?;
        next.aero -= eval(&self.aero)?;
        next.carrier -= eval(&self.carrier)?;
        self.beams.finish(sim, materials, &mut next)?;
        next.check()?;
        *totals = next;
        Ok(self.shadow)
    }
}

#[derive(Debug)]
pub struct YieldWork {
    pub plastic: Vec<f64>,
    pub overshoot: Vec<f64>,
}

/// Pair the smallest and largest nonnegative factors first. Unlike a fixed
/// left-associated product, this cannot underflow to zero or overflow before a
/// third factor that would have brought it back into range. Rounding of nonzero
/// subnormal intermediate products remains binary64 rounding.
fn product3(mut factors: [f64; 3]) -> f64 {
    factors.sort_by(f64::total_cmp);
    (factors[0] * factors[2]) * factors[1]
}

/// Scalar accounting entry point. Inputs are finite, nonnegative stiffness and
/// length, and positive rest lengths. Reject overflow in either contribution,
/// including their sum. Binary64 subnormal work and underflow to zero are retained;
/// the powers-of-two fixtures cover that representable band independently.
pub fn yield_components(k: f64, length: f64, old: f64, new: f64) -> Result<(f64, f64)> {
    finite("yield work inputs", &[k, length, old, new])?;
    if k < 0.0 || length < 0.0 || old <= 0.0 || new <= 0.0 {
        return Err("invalid yield work inputs".into());
    }
    let flow = (old - new).abs();
    let extension = (length - new).abs();
    if new < old.min(length) || new > old.max(length) {
        return Err("invalid plastic flow direction".into());
    }
    let plastic = product3([k, extension, flow]);
    // Halving flow can round to zero only at the smallest subnormal flow. Even
    // maximum finite k cannot make its squared-flow work representable.
    let overshoot = product3([k, flow, 0.5 * flow]);
    finite(
        "yield work components",
        &[plastic, overshoot, plastic + overshoot],
    )?;
    Ok((plastic, overshoot))
}

/// Snapshot every old rest length, then measure the actual production yield pass
/// at fixed positions. No energy-drop subtraction is used to compute plastic work.
pub fn yield_diagnostics(sim: &mut Sim) -> Result<YieldWork> {
    let n = sim.beams.len();
    if sim.rest0.len() != n
        || sim.yield_strain.len() != n
        || sim.broken.len() != n
        || sim.break_strain.len() != n
        || sim.break_strain.iter().any(Option::is_some)
        || sim.broken.iter().any(|b| *b)
        || !sim.breaks.is_empty()
        || !sim.yield_refused.is_empty()
    {
        return Err("unsupported yield configuration".into());
    }
    let mut old = Vec::with_capacity(n);
    for (i, b) in sim.beams.iter().enumerate() {
        let a = sim.pos.get(b.a).ok_or("invalid yield beam index")?;
        let z = sim.pos.get(b.b).ok_or("invalid yield beam index")?;
        finite("yield positions", &[a.x, a.y, a.z, z.x, z.y, z.z])?;
        let length = (*z - *a).length();
        yield_components(b.stiffness, length, b.rest, b.rest)?;
        finite("yield reference length", &[sim.rest0[i]])?;
        if sim.rest0[i] <= 0.0 {
            return Err("invalid yield reference length".into());
        }
        if let Some(y) = sim.yield_strain[i] {
            finite("yield strain", &[y])?;
            if y < 0.0 {
                return Err("negative yield strain".into());
            }
        }
        old.push(b.rest);
    }
    // Preserve the old fail-closed pre-yield energy check before the real pass.
    finite("pre-yield elastic energy", &beam_elastic(sim))?;
    sim.apply_yield();
    if !sim.yield_refused.is_empty() {
        return Err("yield refused".into());
    }
    let mut work = YieldWork {
        plastic: Vec::with_capacity(n),
        overshoot: Vec::with_capacity(n),
    };
    for (b, rest_old) in sim.beams.iter().zip(old) {
        let length = (sim.pos[b.b] - sim.pos[b.a]).length();
        let (plastic, overshoot) = yield_components(b.stiffness, length, rest_old, b.rest)?;
        work.plastic.push(plastic);
        work.overshoot.push(overshoot);
    }
    Ok(work)
}

pub(super) fn check_yield_overshoot(totals: &Totals) -> Result<()> {
    finite("material yield overshoot", &totals.yield_overshoot)?;
    finite("material yield at cap", &totals.yield_at_cap)?;
    finite("total yield overshoot", &[totals.yield_overshoot.iter().sum()])?;
    finite("total yield at cap", &[totals.yield_at_cap.iter().sum()])?;
    if totals.yield_at_cap.iter().any(|w| *w < 0.0) {
        return Err("negative yield at cap".into());
    }
    if totals.yield_overshoot.iter().any(|w| *w < 0.0) {
        return Err("negative yield overshoot".into());
    }
    Ok(())
}

/// Analytic cross-check entry point: records yield-force times flow as plastic
/// for the round-6 diagnostic fixtures, not the round-7 force-work ledger.
pub fn record_yield(sim: &mut Sim, materials: &[usize], totals: &mut Totals) -> Result<()> {
    if materials.len() != sim.beams.len() || materials.iter().any(|m| *m >= MATERIALS.len()) {
        return Err("invalid beam material map".into());
    }
    totals.check()?;
    let work = yield_diagnostics(sim)?;
    let mut next = totals.clone();
    for ((plastic, overshoot), &m) in work.plastic.iter().zip(&work.overshoot).zip(materials) {
        next.plastic[m] += plastic;
        next.yield_at_cap[m] += plastic;
        next.yield_overshoot[m] += overshoot;
    }
    next.check()?;
    *totals = next;
    Ok(())
}

/// Complete the beam force-work ledger across the real fixed-position yield
/// operation. Combined with BeamWork, plastic is minus spring work minus the
/// full stored-energy change. Cap work and overshoot are separate cross-checks.
pub fn record_exact_yield(sim: &mut Sim, materials: &[usize], totals: &mut Totals) -> Result<()> {
    if materials.len() != sim.beams.len() || materials.iter().any(|m| *m >= MATERIALS.len()) {
        return Err("invalid beam material map".into());
    }
    totals.check()?;
    // yield_diagnostics validates before changing rest lengths.
    let old: Vec<_> = sim.beams.iter().map(|b| b.rest).collect();
    let work = yield_diagnostics(sim)?;
    let after = beam_elastic(sim);
    let mut next = totals.clone();
    for (i, &m) in materials.iter().enumerate() {
        let b = &sim.beams[i];
        let extension = (sim.pos[b.b] - sim.pos[b.a]).length() - old[i];
        let before = 0.5 * b.stiffness * extension * extension;
        next.plastic[m] += before - after[i];
        next.yield_at_cap[m] += work.plastic[i];
        next.yield_overshoot[m] += work.overshoot[i];
    }
    next.check()?;
    *totals = next;
    Ok(())
}

pub(super) fn render_phase(
    out: &mut impl Write,
    measurement: &Measurement,
    phase: &str,
    start: usize,
    end: usize,
    material: usize,
    peak: f64,
) -> io::Result<()> {
    let a = &measurement.rows[start];
    let b = &measurement.rows[end];
    let name = MATERIALS[material];
    writeln!(
        out,
        "phase phase={phase} material={name} elastic_start_j={:.6} elastic_end_j={:.6} elastic_peak_j={peak:.6} plastic_j={:.6} damper_j={:.6} yield_at_cap_j={:.6} yield_overshoot_j={:.6}",
        a.elastic[material],
        b.elastic[material],
        b.totals.plastic[material] - a.totals.plastic[material],
        b.totals.damper[material] - a.totals.damper[material],
        b.totals.yield_at_cap[material] - a.totals.yield_at_cap[material],
        b.totals.yield_overshoot[material] - a.totals.yield_overshoot[material]
    )
}

pub(super) fn render_aux(out: &mut impl Write, row: &Row) -> io::Result<()> {
    let t = &row.totals;
    writeln!(
        out,
        "aux suspension_damper_j={:.6} tyre_damper_j={:.6} aero_j={:.6} node_collision_j={:.6} node_projection_j={:.6} tyre_friction_rolling_j={:.6} carrier_constraint_j={:.6} vehicle_elastic_j={:.6} wheel_kinetic_j={:.6} yield_at_cap_j={:.6} yield_overshoot_j={:.6}",
        t.suspension,
        t.tyre_damper,
        t.aero,
        t.collision,
        t.projection,
        t.rolling,
        t.carrier,
        row.vehicle_elastic,
        row.wheel_kinetic,
        t.yield_at_cap.iter().sum::<f64>(),
        t.yield_overshoot.iter().sum::<f64>()
    )
}

/// Evaluate the unchanged signed ledger equation, rejecting invalid arithmetic,
/// but leaving the bound judgment to `Budget::record`.
pub fn residual(initial: f64, k: f64, e: f64, p: f64, d: f64, c: f64) -> Result<f64> {
    finite("budget", &[initial, k, e, p, d, c])?;
    if initial <= 0.0 || k < 0.0 || e < 0.0 {
        return Err("invalid budget energy".into());
    }
    let residual = initial - (k + e + p + d + c);
    finite("budget residual", &[residual])?;
    Ok(residual)
}

/// Strict single-sample entry point retained for the analytic closure fixtures.
pub fn check_budget(initial: f64, k: f64, e: f64, p: f64, d: f64, c: f64) -> Result<f64> {
    let residual = residual(initial, k, e, p, d, c)?;
    let mut budget = Budget::new(initial)?;
    budget.record(0, residual)?;
    match budget.first_breach {
        Some(_) => Err(format!(
            "budget residual_j={residual:.6} tolerance_j={:.6}",
            budget.tolerance
        )),
        None => Ok(residual),
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Breach {
    pub substep: usize,
    pub residual: f64,
}

/// The initial energy must be finite and positive. Work may be signed; only the
/// final residual's magnitude is judged. Binary64 underflow is not replaced with
/// a tolerance floor. Non-finite sums or output fractions remain immediate errors.
#[derive(Clone, Debug, PartialEq)]
pub struct Budget {
    pub budget_ok: bool,
    pub first_breach: Option<Breach>,
    pub max_abs_residual: f64,
    pub at_substep: usize,
    initial: f64,
    tolerance: f64,
    samples: usize,
}

impl Budget {
    pub fn new(initial: f64) -> Result<Self> {
        finite("initial budget energy", &[initial])?;
        if initial <= 0.0 {
            return Err("invalid initial budget energy".into());
        }
        let tolerance = BUDGET_FRACTION * initial;
        finite("budget tolerance", &[tolerance])?;
        Ok(Self {
            budget_ok: true,
            first_breach: None,
            max_abs_residual: 0.0,
            at_substep: 0,
            initial,
            tolerance,
            samples: 0,
        })
    }

    /// Samples include initialization (index zero). Strict improvement preserves
    /// the earliest maximum on ties. Validate all arithmetic before updating state.
    pub fn record(&mut self, substep: usize, residual: f64) -> Result<()> {
        finite(
            "recorded budget residual",
            &[residual, residual.abs() / self.initial],
        )?;
        if substep != self.samples {
            return Err("nonsequential budget substep".into());
        }
        let samples = self.samples.checked_add(1).ok_or("budget sample overflow")?;
        if residual.abs() > self.tolerance {
            self.budget_ok = false;
            if self.first_breach.is_none() {
                self.first_breach = Some(Breach { substep, residual });
            }
        }
        if residual.abs() > self.max_abs_residual {
            self.max_abs_residual = residual.abs();
            self.at_substep = substep;
        }
        self.samples = samples;
        Ok(())
    }

    pub fn validate(&self, initial: f64, rows: &[Row], max_residual: f64) -> Result<()> {
        if rows.is_empty() {
            return Err("empty budget samples".into());
        }
        let mut expected = Self::new(initial)?;
        for (i, row) in rows.iter().enumerate() {
            expected.record(i, row.residual)?;
        }
        if *self != expected || max_residual.to_bits() != expected.max_abs_residual.to_bits() {
            return Err("recorded budget judgment mismatch".into());
        }
        Ok(())
    }

    /// Called only after measurement validation and every data/summary row.
    /// A budget FAIL is a complete report, not an output error. Flush failures
    /// still propagate; a failure flushing data cannot emit a success marker.
    pub(super) fn finish(&self, out: &mut impl Write) -> io::Result<()> {
        out.flush()?;
        if let Some(first) = self.first_breach {
            writeln!(
                out,
                "measurement_a: FAIL budget first_substep={} residual_j={:.6} tolerance_j={:.6} max_abs_residual_j={:.6} at_substep={}",
                first.substep,
                first.residual,
                self.tolerance,
                self.max_abs_residual,
                self.at_substep
            )?;
        } else {
            writeln!(out, "measurement_a: ok")?;
        }
        out.flush()
    }
}

/// Shared by the CLI and stream-injected fixtures: a fully rendered budget
/// failure must exit 1, even though collecting and writing its data succeeded.
pub fn report(measurement: Result<Measurement>, out: &mut impl Write) -> Result<i32> {
    let measurement = measurement?;
    measurement.render(out)?;
    Ok(if measurement.budget.budget_ok { 0 } else { 1 })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crash_sim::solver::{Solver, State, SymplecticEuler};
    use crash_sim::vec3::Vec3;
    use crash_sim::{Beam, Config};

    fn solver_step(sim: &mut Sim, h: f64) {
        let mut solver = SymplecticEuler::default();
        solver.step(
            State {
                pos: &mut sim.pos,
                vel: &mut sim.vel,
                beams: &sim.beams,
                mass: &sim.mass,
                gravity: 0.0,
            },
            h,
        );
        assert_eq!(solver.check(), None);
    }

    #[test]
    fn exact_force_quadrature_closes_beam_budget_not_left_end() {
        let h = 1.0 / 128.0;
        let mut sim = yielding_beam(1.0, 0.125);
        sim.yield_strain[0] = None;
        sim.mass.fill(32.0);
        sim.beams[0].stiffness = 2048.0;
        sim.beams[0].damping = 256.0;
        sim.vel[0].x = -2.0;
        sim.vel[1].x = 2.0;
        let work = BeamWork::start(&sim, h).unwrap();
        solver_step(&mut sim, h);
        let mut totals = Totals::default();
        work.finish(&sim, &[2], &mut totals).unwrap();
        record_exact_yield(&mut sim, &[2], &mut totals).unwrap();
        // Independent dyadic solution: F=1024 N, each speed falls by 1/4,
        // K1=32*(7/4)^2, extension=7/256, E1=1024*(7/256)^2.
        // Wd=1024*(2+7/4)/128=30 J (both endpoints included).
        let k = super::super::node_kinetic(&sim.vel, &sim.mass);
        let e = beam_elastic(&sim)[0];
        assert_eq!(k, 98.0);
        assert_eq!(e, 49.0 / 64.0);
        assert_eq!(totals.damper[2], 30.0);
        assert_eq!(totals.plastic[2], -49.0 / 64.0);
        let residual = residual(128.0, k, e, totals.plastic_sum(), totals.damper_sum(), 0.0).unwrap();
        assert!(residual.abs() <= 1e-9 * 128.0, "exact residual_j={residual}");
        // Mutation target: use old velocity in force_work. Then Wd=32, not 30,
        // and the closure assertion fails by 2 J even with the same spring ledger.
        let left_end_damper = 256.0 * 4.0 * 4.0 * h;
        let left_end_residual = 128.0 - (98.0 + e + totals.plastic_sum() + left_end_damper);
        assert_eq!(left_end_residual, -2.0);
        assert!(left_end_residual.abs() > 1e-9 * 128.0);

        // Nonzero spring AND damper forces share the same actual end velocities.
        // Fspring=16, Fdamper=32; dv=48/(128*2)=3/16 at each end.
        let mut sim = yielding_beam(9.0 / 8.0, 0.125);
        sim.yield_strain[0] = None;
        sim.beams[0].damping = 16.0;
        sim.vel[0].x = -1.0;
        sim.vel[1].x = 1.0;
        let work = BeamWork::start(&sim, h).unwrap();
        solver_step(&mut sim, h);
        let mut totals = Totals::default();
        work.finish(&sim, &[0], &mut totals).unwrap();
        assert_eq!(sim.vel[1].x, 13.0 / 16.0);
        assert_eq!(totals.damper[0], 29.0 / 64.0);
        assert_eq!(totals.plastic[0], 215.0 / 16384.0);
        let end = super::super::node_kinetic(&sim.vel, &sim.mass)
            + beam_elastic(&sim)[0]
            + totals.plastic_sum()
            + totals.damper_sum();
        assert!((3.0 - end).abs() <= 1e-9 * 3.0, "initial K+E=3, end={end}");
    }

    #[test]
    fn exact_yield_ledger_keeps_cap_and_overshoot_as_cross_checks() {
        let mut sim = yielding_beam(7.0 / 8.0, 1.0 / 8.0);
        let work = BeamWork::start(&sim, 1.0 / 128.0).unwrap();
        solver_step(&mut sim, 1.0 / 128.0);
        let mut totals = Totals::default();
        work.finish(&sim, &[3], &mut totals).unwrap();
        record_exact_yield(&mut sim, &[3], &mut totals).unwrap();
        // K0=2, E0=1, K1=2*(15/16)^2=225/128, E1=1.
        // Thus discrete plastic=31/128; cap flow=16*15/1024=15/64.
        assert_eq!(totals.plastic[3], 31.0 / 128.0);
        assert_eq!(totals.yield_at_cap[3], 15.0 / 64.0);
        assert_eq!(totals.yield_overshoot[3], 225.0 / 16384.0);
        assert_ne!(
            totals.plastic[3],
            4065.0 / 16384.0,
            "old energy-drop definition"
        );
        assert_eq!(
            super::super::node_kinetic(&sim.vel, &sim.mass)
                + beam_elastic(&sim)[0]
                + totals.plastic_sum(),
            3.0
        );
        for material in 0..3 {
            assert_eq!(totals.plastic[material], 0.0);
            assert_eq!(totals.yield_at_cap[material], 0.0);
            assert_eq!(totals.yield_overshoot[material], 0.0);
        }
    }

    #[test]
    fn auxiliary_force_components_match_unconstrained_kinetic_change() {
        use crash_sim::vehicle::Substep;

        let (cfg, body) = super::super::inputs().unwrap();
        let h = cfg.dt / f64::from(cfg.substeps);
        let mut sim = Sim::from_car(cfg, &body).unwrap();
        for (i, v) in sim.vel.iter_mut().enumerate() {
            *v = Vec3::new(2.0 + i as f64 / 32.0, -0.25, 0.125);
        }
        let wall = sim.planes[1];
        let car = sim.car.as_mut().unwrap();
        for w in &mut car.wheels {
            w.vel = Vec3::new(1.0, 0.5, 0.0);
            w.spin = 0.0; // Isolate translational force work, not rolling spin.
        }
        // Positive radial wall load and suspension damping, with unequal wheel
        // and carrier velocities. No projection/contact operation runs here.
        let w = &mut car.wheels[0];
        w.pos += wall.normal * (w.p.radius - 0.001 - wall.distance(w.pos));
        let initial_k = super::super::kinetic(&sim);
        let initial_e: f64 = beam_elastic(&sim).iter().sum();
        let mut extra = vec![Vec3::ZERO; sim.vel.len()];
        sim.car.as_mut().unwrap().pre_step(
            &sim.pos,
            &sim.vel,
            &sim.mass,
            &sim.planes,
            Substep {
                gravity: Vec3::ZERO,
                h,
            },
            &mut extra,
        );
        let car = sim.car.as_ref().unwrap();
        assert!(
            (car.frame.up - Vec3::new(0.0, 1.0, 0.0)).length() <= 1e-12,
            "flat frame fit tolerance"
        );
        assert_eq!(car.wheels.len(), 4);
        for w in &car.wheels {
            assert_eq!(w.p.mass, 30.0);
            assert_eq!(w.p.damper, 1272.0);
            assert!(
                (w.length - w.p.rest).abs() <= 1e-12,
                "initial spring geometry tolerance"
            );
        }
        let mut totals = Totals::default();
        let work = ForceWork::start(&sim, &extra, h, &mut totals).unwrap();
        assert!(totals.tyre_impulse > 0.0, "fixture must load the tyre");
        let mut solver = SymplecticEuler::default();
        solver.step_with_forces(
            State {
                pos: &mut sim.pos,
                vel: &mut sim.vel,
                beams: &sim.beams,
                mass: &sim.mass,
                gravity: 0.0,
            },
            &extra,
            h,
        );
        assert_eq!(solver.check(), None);
        let spring_work = work
            .suspension_spring
            .work(&work.old_nodes, &work.old_wheels, &sim, &work.shadow, h)
            .unwrap()
            + work
                .tyre_spring
                .work(&work.old_nodes, &work.old_wheels, &sim, &work.shadow, h)
                .unwrap();
        let materials = vec![0; sim.beams.len()];
        let shadow = work.finish(&sim, &materials, &mut totals).unwrap();
        // Independent wheel solution (up to frame-fit roundoff): the axis is +y,
        // wall force is transverse,
        // suspension extension is zero, and mount-minus-wheel vy=-.75 m/s.
        // Each wheel has Fy=-1272*.75=-954 N and m=30 kg. Zero spin stays zero.
        let wheel_vy = 0.5 - 954.0 * h / 30.0;
        let wheel_k = 4.0 * 0.5 * 30.0 * (1.0 + wheel_vy * wheel_vy);
        assert!(
            (super::super::wheel_kinetic(&shadow) - wheel_k).abs() <= 1e-9 * initial_k,
            "unconstrained wheel analytic reference"
        );
        // Node velocities come from the REAL solver, independent of the ledger.
        let expected = super::super::node_kinetic(&sim.vel, &sim.mass) + wheel_k - initial_k;
        let delta_e = beam_elastic(&sim).iter().sum::<f64>() - initial_e;
        let actual = spring_work
            - delta_e
            - totals.plastic_sum()
            - totals.damper_sum()
            - totals.contact_sum();
        assert!(
            (actual - expected).abs() <= 1e-9 * initial_k,
            "auxiliary work_j={actual} kinetic_change_j={expected}"
        );
        assert_ne!(totals.suspension, 0.0, "fixture must exercise suspension");
        assert_ne!(totals.tyre_damper, 0.0, "fixture must exercise tyre damping");
        assert_ne!(totals.aero, 0.0, "fixture must exercise aero");
        assert_ne!(totals.carrier, 0.0, "fixture must exercise carrier reactions");
    }

    #[test]
    fn force_quadrature_numeric_edges_and_refusals() {
        let x = |v| Vec3::new(v, 0.0, 0.0);
        // Independent exact powers of two cover subnormal, underflow, largest
        // finite output, and cancellation, without using measured output as reference.
        let tiny = f64::from_bits(1);
        assert_eq!(force_work(x(tiny), x(1.0), x(1.0), 1.0).unwrap(), tiny);
        assert_eq!(force_work(x(1.0), x(tiny), x(tiny), 1.0).unwrap(), tiny);
        assert_eq!(force_work(x(tiny), x(0.25), x(0.25), 1.0).unwrap(), 0.0);
        assert_eq!(
            force_work(x(1.0), x(f64::MAX), x(f64::MAX), 1.0).unwrap(),
            f64::MAX
        );
        assert_eq!(
            force_work(x(1.0), x(f64::MAX), x(-f64::MAX), 1.0).unwrap(),
            0.0
        );
        assert_eq!(
            force_work(x(1.0), x(1.0), x(-1.0 + f64::EPSILON), 1.0).unwrap(),
            2.0_f64.powi(-53)
        );
        assert!(force_work(x(f64::MAX), x(2.0), x(2.0), 1.0).is_err());
        for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(force_work(x(1.0), x(1.0), x(1.0), bad).is_err());
        }
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(force_work(x(bad), x(1.0), x(1.0), 1.0).is_err());
            assert!(force_work(x(1.0), x(bad), x(1.0), 1.0).is_err());
            assert!(force_work(x(1.0), x(1.0), x(bad), 1.0).is_err());
        }
        let mut sim = yielding_beam(1.0, 0.125);
        let work = BeamWork::start(&sim, 1.0 / 128.0).unwrap();
        let mut totals = Totals::default();
        assert!(work.finish(&sim, &[4], &mut totals).is_err());
        assert_eq!(totals.plastic_sum(), 0.0);
        sim.beams[0].a = usize::MAX;
        assert!(BeamWork::start(&sim, 1.0 / 128.0).is_err());
        assert!(record_exact_yield(&mut sim, &[0], &mut totals).is_err());
        assert_eq!(totals.plastic_sum(), 0.0);
    }

    fn yielding_beam(length: f64, strain: f64) -> Sim {
        let mut sim = Sim::new(Config {
            gravity: 0.0,
            ..Config::default()
        });
        sim.pos = vec![Vec3::new(0.0, 2.0, 0.0), Vec3::new(length, 2.0, 0.0)];
        sim.vel = vec![Vec3::new(1.0, 0.0, 0.0), Vec3::new(-1.0, 0.0, 0.0)];
        sim.mass = vec![2.0, 2.0];
        sim.beams = vec![Beam {
            a: 0,
            b: 1,
            rest: 1.0,
            stiffness: 128.0,
            damping: 0.0,
        }];
        sim.rest0 = vec![1.0];
        sim.yield_strain = vec![Some(strain)];
        sim.break_strain = vec![None];
        sim.broken = vec![false];
        sim
    }

    #[test]
    fn yielding_beam_overshoot_is_not_plastic_work() {
        let mut sim = yielding_beam(7.0 / 8.0, 1.0 / 8.0);
        let mut solver = SymplecticEuler::default();
        let h = 1.0 / 128.0;
        solver.step(
            State {
                pos: &mut sim.pos,
                vel: &mut sim.vel,
                beams: &sim.beams,
                mass: &sim.mass,
                gravity: 0.0,
            },
            h,
        );
        assert_eq!(solver.check(), None);
        // Spring force -16 N changes each 2 kg node speed by 1/16 m/s.
        // v_rel*h = -(15/8)/128 = -15/1024 m beyond the yield surface.
        assert_eq!(sim.vel[0].x, 15.0 / 16.0);
        assert_eq!(sim.vel[1].x, -15.0 / 16.0);
        let positions = sim.pos.clone();
        let velocities = sim.vel.clone();
        let energy_before: f64 = beam_elastic(&sim).iter().sum();
        let mut totals = Totals::default();
        record_yield(&mut sim, &[2], &mut totals).unwrap();
        assert_eq!(sim.beams[0].rest, 1009.0 / 1024.0);
        // Independent exact dyadic references: F_yield=16 N; flow=15/1024 m.
        // Plastic=16*15/1024; overshoot=(128/2)*(15/1024)^2.
        assert_eq!(totals.plastic[2], 15.0 / 64.0);
        assert_eq!(totals.yield_overshoot[2], 225.0 / 16384.0);
        assert_eq!(energy_before - beam_elastic(&sim)[0], 4065.0 / 16384.0);
        assert_eq!(totals.plastic[2] + totals.yield_overshoot[2], 4065.0 / 16384.0);
        assert_eq!(totals.plastic_sum(), 15.0 / 64.0);
        assert_eq!(totals.damper_sum(), 0.0);
        assert_eq!(totals.contact_sum(), 0.0);
        for (a, b) in sim.pos.iter().zip(positions) {
            assert_eq!(a.x.to_bits(), b.x.to_bits());
            assert_eq!(a.y.to_bits(), b.y.to_bits());
            assert_eq!(a.z.to_bits(), b.z.to_bits());
        }
        for (a, b) in sim.vel.iter().zip(velocities) {
            assert_eq!(a.x.to_bits(), b.x.to_bits());
            assert_eq!(a.y.to_bits(), b.y.to_bits());
            assert_eq!(a.z.to_bits(), b.z.to_bits());
        }
        // Behavioural mutation: the old definition adds 225/16384 J and violates
        // the exact 15/64 J assertion above, rather than merely failing to compile.
        assert_ne!(energy_before - beam_elastic(&sim)[0], totals.plastic_sum());
    }

    #[test]
    fn yield_components_cover_tension_compression_and_numeric_edges() {
        assert_eq!(yield_components(128.0, 0.25, 1.0, 0.375).unwrap(), (10.0, 25.0));
        assert_eq!(yield_components(128.0, 2.25, 1.0, 1.5).unwrap(), (48.0, 16.0));
        assert_eq!(yield_components(128.0, 1.0, 1.0, 1.0).unwrap(), (0.0, 0.0));
        // Near cancellation: flow is 2^-52, new extension is 1-2^-52.
        // With k=2^52, plastic=1-2^-52 and overshoot=2^-53, exactly.
        let ulp = f64::EPSILON;
        assert_eq!(
            yield_components(2.0_f64.powi(52), 2.0, 1.0, 1.0 + ulp).unwrap(),
            (1.0 - ulp, 2.0_f64.powi(-53))
        );
        // For length=q, old=4q, new=2q and k=2:
        // plastic=4q^2, overshoot=4q^2. Exact powers of two independently
        // specify underflow, subnormal and large finite expectations.
        for (exponent, expected) in [
            (-539, 0.0),
            (-538, f64::from_bits(1)),
            (-537, f64::from_bits(4)),
            (-512, f64::MIN_POSITIVE),
            (0, 4.0),
            (509, 2.0_f64.powi(1020)),
        ] {
            let q = 2.0_f64.powi(exponent);
            assert_eq!(
                yield_components(2.0, q, 4.0 * q, 2.0 * q).unwrap(),
                (expected, expected),
                "exponent={exponent}"
            );
        }
        assert_eq!(yield_components(f64::MAX, 1.0, 1.0, 1.0).unwrap(), (0.0, 0.0));
        assert_eq!(
            yield_components(f64::from_bits(1), 1.0, 4.0, 2.0).unwrap(),
            (f64::from_bits(2), f64::from_bits(2))
        );
        let smallest = f64::from_bits(1);
        assert_eq!(
            yield_components(2.0, smallest, 4.0 * smallest, 2.0 * smallest).unwrap(),
            (0.0, 0.0)
        );
        let max_eighth = f64::from_bits(f64::MAX.to_bits() - (3_u64 << 52));
        assert_eq!(
            yield_components(f64::MAX, 0.25, 1.0, 0.5).unwrap(),
            (max_eighth, max_eighth)
        );
        // Extreme finite k times very small extensions/flow must not lose a
        // representable product to an intermediate overflow or underflow.
        let q = 2.0_f64.powi(-537);
        assert_eq!(
            yield_components(2.0_f64.powi(1023), q, 4.0 * q, 2.0 * q).unwrap(),
            (2.0_f64.powi(-50), 2.0_f64.powi(-50))
        );
        for field in 0..4 {
            for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                let mut args = [128.0, 0.25, 1.0, 0.375];
                args[field] = bad;
                let [k, length, old, new] = args;
                assert!(yield_components(k, length, old, new).is_err());
            }
        }
        for args in [
            [-1.0, 0.25, 1.0, 0.375],
            [128.0, -0.25, 1.0, 0.375],
            [128.0, 0.25, 0.0, 0.375],
            [128.0, 0.25, 1.0, 0.0],
            [128.0, 0.25, 1.0, 0.125],
            [f64::MAX, 1.0, 4.0, 2.0],
            [2.0, 0.0, f64::MAX, f64::MAX / 2.0],
        ] {
            let [k, length, old, new] = args;
            assert!(yield_components(k, length, old, new).is_err());
        }
    }

    #[test]
    fn yield_overshoot_is_material_local_and_report_only() {
        for material in 0..MATERIALS.len() {
            let mut sim = yielding_beam(0.25, 0.125);
            let mut totals = Totals::default();
            record_yield(&mut sim, &[material], &mut totals).unwrap();
            for m in 0..MATERIALS.len() {
                assert_eq!(totals.plastic[m], if m == material { 10.0 } else { 0.0 });
                assert_eq!(totals.yield_overshoot[m], if m == material { 25.0 } else { 0.0 });
            }
            let row = Row {
                elastic: [0.0; 4],
                totals,
                velocity: [1.0; 2],
                crush: 0.0,
                kinetic: 990.0,
                vehicle_elastic: 0.0,
                wheel_kinetic: 0.0,
                residual: 0.0,
            };
            let mut end = row.clone();
            end.totals.plastic[material] = 15.0;
            end.totals.yield_overshoot[material] = 32.0;
            end.kinetic = 985.0;
            let mut initial = row.clone();
            initial.totals = Totals::default();
            initial.kinetic = 1000.0;
            let mut budget = Budget::new(1000.0).unwrap();
            for i in 0..3 {
                budget.record(i, 0.0).unwrap();
            }
            let measurement = Measurement {
                rows: vec![initial, row, end],
                h: 0.125,
                contact: 1,
                split: 1,
                initial: 1000.0,
                max_residual: 0.0,
                budget,
            };
            let mut out = Vec::new();
            measurement.render(&mut out).unwrap();
            let text = String::from_utf8(out).unwrap();
            for (phase, expected) in [("compression", "25.000000"), ("unloading", "7.000000")] {
                let prefix = format!("phase phase={phase} material={} ", MATERIALS[material]);
                let line = text.lines().find(|l| l.starts_with(&prefix)).unwrap();
                assert!(line.ends_with(&format!("yield_overshoot_j={expected}")));
            }
            let aux = text.lines().find(|l| l.starts_with("aux ")).unwrap();
            assert!(aux.ends_with("yield_overshoot_j=32.000000"));
            assert!(text.ends_with("measurement_a: ok\n"));
            // This rendered ledger has zero residual while nonzero overshoot is
            // reported. Including that diagnostic in the budget would invalidate it.
            assert_eq!(measurement.rows[2].totals.plastic_sum(), 15.0);
            assert_eq!(measurement.rows[2].totals.damper_sum(), 0.0);
            assert_eq!(measurement.rows[2].totals.contact_sum(), 0.0);
            let mut invalid = measurement.clone();
            invalid.rows[2].totals.yield_overshoot[material] = f64::NAN;
            let mut out = Vec::new();
            assert!(invalid
                .render(&mut out)
                .unwrap_err()
                .contains("yield overshoot"));
            assert!(out.is_empty(), "validate diagnostics before output");
        }
    }

    #[test]
    fn yield_input_refusals_do_not_change_rest_or_ledger() {
        for bad_map in [vec![], vec![4], vec![usize::MAX]] {
            let mut sim = yielding_beam(0.25, 0.125);
            let mut totals = Totals::default();
            assert!(record_yield(&mut sim, &bad_map, &mut totals).is_err());
            assert_eq!(sim.beams[0].rest, 1.0);
            assert_eq!(totals.plastic_sum(), 0.0);
            assert_eq!(totals.yield_overshoot, [0.0; 4]);
        }
        for field in 0..5 {
            let mut sim = yielding_beam(0.25, 0.125);
            match field {
                0 => sim.beams[0].a = usize::MAX,
                1 => sim.rest0.clear(),
                2 => sim.yield_strain[0] = Some(f64::NAN),
                3 => sim.pos[0].x = f64::INFINITY,
                _ => sim.break_strain[0] = Some(0.5),
            }
            assert!(yield_diagnostics(&mut sim).is_err());
            assert_eq!(sim.beams[0].rest, 1.0);
        }
    }
}
