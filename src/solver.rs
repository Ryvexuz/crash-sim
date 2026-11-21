//! The solver interface: one physics substep on a node-beam state, behind a trait so the
//! integrator can be swapped. `SymplecticEuler` is today's integrator, moved here unchanged
//! in maths and order of operations (docs/PHYSICS.md, "Integrator").

use crate::vec3::Vec3;
use crate::Beam;

/// The state a solver advances: node positions and velocities (mutable), and the beams and
/// per-node masses it reads, plus the gravitational acceleration. Beams carry their own
/// stiffness and damping.
pub struct State<'a> {
    pub pos: &'a mut [Vec3],
    pub vel: &'a mut [Vec3],
    pub beams: &'a [Beam],
    /// Mass of each node (kg), one per node.
    pub mass: &'a [f64],
    /// Gravitational acceleration, m/s^2, pointing down (-y).
    pub gravity: f64,
}

/// Advances a state by one physics substep of length `h` seconds. A solver may keep its own
/// working data between calls but must not need anything else from the caller.
pub trait Solver {
    fn step(&mut self, state: State<'_>, h: f64);

    /// Like `step`, with an extra force (N) on every node on top of gravity and the beams
    /// (`extra[i]` acts on node i; the wheels' suspension reactions and a tilted gravity's
    /// horizontal part come in this way, docs/PHYSICS.md "Wheels, suspension and tyres").
    /// The default is operator splitting: each node's velocity first gets `extra * h / m`, then
    /// `step` runs, so any solver takes the forces; `SymplecticEuler` instead adds them into its
    /// own force sum. `extra` must have one entry per node.
    fn step_with_forces(&mut self, state: State<'_>, extra: &[Vec3], h: f64) {
        assert_eq!(extra.len(), state.pos.len(), "one extra force per node");
        for i in 0..state.vel.len() {
            state.vel[i] += extra[i] * (h / state.mass[i]);
        }
        self.step(state, h);
    }

    /// True if this solver simulates a body's solids and tets (docs/PHYSICS.md "Solids"). A
    /// solver that does not is refused a body with tets by `Sim`, because the tets would be
    /// dropped. The default is false: a node-beam solver knows nothing of tets.
    fn handles_solids(&self) -> bool {
        false
    }

    /// The first error this solver found in its own working data (for the m5 solver: an
    /// inverted or collapsed tet, or an exceeded stiffness bound), or None. `Sim::check` returns
    /// it, so the run ends like it does on a beam strain. The default is None: a solver that
    /// keeps no working data has nothing to report.
    fn check(&self) -> Option<String> {
        None
    }
}

/// Semi-implicit (symplectic) Euler. Contact with the ground and the walls is not here:
/// `Sim` applies `collide::apply` after every solver step.
///
/// A non-finite node, force or update stops the substep before anything is written: the
/// first such error is kept (`check` returns it, so `Sim::check` ends the run with it) and no
/// later substep applies anything. Build it with `SymplecticEuler::default()`.
///
/// The happy path is the old arithmetic plus one fused finiteness test per node, on the new
/// position (a non-finite contribution makes the force sum non-finite, which makes the new
/// velocity and position non-finite; so does a non-finite input node). The new values go to
/// working buffers and are copied over `pos` and `vel` only if every node passed. Only on a
/// failure does `diagnose` run, recomputing the forces contribution by contribution to name the
/// node and the source.
#[derive(Clone, Debug, Default)]
pub struct SymplecticEuler {
    /// Substeps taken so far, counting the one in progress.
    substep: u64,
    /// The first error found, if any.
    error: Option<String>,
    /// Working buffers, kept between substeps so the step allocates nothing once warm.
    force: Vec<Vec3>,
    new_vel: Vec<Vec3>,
    new_pos: Vec<Vec3>,
}

fn finite(v: Vec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}

/// The unchanged inputs of one failed substep, for `SymplecticEuler::diagnose`.
struct Frame<'a> {
    pos: &'a [Vec3],
    vel: &'a [Vec3],
    beams: &'a [Beam],
    mass: &'a [f64],
    gravity: f64,
    extra: Option<&'a [Vec3]>,
    h: f64,
}

/// Branch-light finiteness of a vector (no short circuit).
#[inline(always)]
fn finite_fused(v: Vec3) -> bool {
    v.x.is_finite() & v.y.is_finite() & v.z.is_finite()
}

impl SymplecticEuler {
    fn fail(&mut self, msg: String) {
        if self.error.is_none() {
            self.error = Some(msg);
        }
    }

    /// The one symplectic Euler step. With `extra` None the arithmetic is exactly the old `step`
    /// (so the cube's output is byte-for-byte unchanged); with Some, each node's extra force is
    /// added after its beam forces. On a failure `pos` and `vel` are untouched.
    fn step_inner(&mut self, s: State<'_>, extra: Option<&[Vec3]>, h: f64) {
        let State { pos, vel, beams, mass, gravity } = s;
        self.substep += 1;
        if self.error.is_some() {
            return;
        }
        let mut force = std::mem::take(&mut self.force);
        force.clear();
        force.extend(mass.iter().map(|&m| Vec3::new(0.0, -gravity * m, 0.0)));
        for bm in beams {
            let d = pos[bm.b] - pos[bm.a];
            let len = d.length();
            if len < 1e-12 {
                continue;
            }
            let dir = d * (1.0 / len);
            let rel_v = (vel[bm.b] - vel[bm.a]).dot(dir);
            let mag = bm.stiffness * (len - bm.rest) + bm.damping * rel_v;
            let f = dir * mag;
            force[bm.a] += f;
            force[bm.b] += -f;
        }
        if let Some(extra) = extra {
            for (f, e) in force.iter_mut().zip(extra) {
                *f += *e;
            }
        }
        let (mut nv, mut np) = (std::mem::take(&mut self.new_vel), std::mem::take(&mut self.new_pos));
        nv.resize(pos.len(), Vec3::ZERO);
        np.resize(pos.len(), Vec3::ZERO);
        let mut ok = true;
        for i in 0..pos.len() {
            // semi-implicit Euler: velocity first, then position with the new velocity
            let v = vel[i] + force[i] * (h / mass[i]);
            let p = pos[i] + v * h;
            // One test on the new position covers the rest: a non-finite input node, force sum
            // or new velocity makes it non-finite too (h is finite and > 0, checked by `Config`).
            ok &= finite_fused(p);
            nv[i] = v;
            np[i] = p;
        }
        if ok {
            vel.copy_from_slice(&nv);
            pos.copy_from_slice(&np);
        } else {
            self.diagnose(Frame { pos, vel, beams, mass, gravity, extra, h });
        }
        self.force = force;
        self.new_vel = nv;
        self.new_pos = np;
    }

    /// Cold path: the substep failed, nothing was written. Recomputes from the unchanged state,
    /// checking each contribution, and records the first error with its node and source.
    fn diagnose(&mut self, fr: Frame<'_>) {
        let Frame { pos, vel, beams, mass, gravity, extra, h } = fr;
        let n = self.substep;
        if let Some(i) = (0..pos.len()).find(|&i| !(finite(pos[i]) && finite(vel[i]))) {
            self.fail(format!("node {i} is not finite at substep {n}"));
            return;
        }
        let mut force: Vec<Vec3> = mass.iter().map(|&m| Vec3::new(0.0, -gravity * m, 0.0)).collect();
        if let Some(i) = force.iter().position(|&f| !finite(f)) {
            self.fail(format!("force on node {i} is not finite at substep {n} (from gravity)"));
            return;
        }
        for bm in beams {
            let d = pos[bm.b] - pos[bm.a];
            let len = d.length();
            if len < 1e-12 {
                continue;
            }
            let dir = d * (1.0 / len);
            let rel_v = (vel[bm.b] - vel[bm.a]).dot(dir);
            let mag = bm.stiffness * (len - bm.rest) + bm.damping * rel_v;
            let f = dir * mag;
            let (fa, fb) = (force[bm.a] + f, force[bm.b] + -f);
            if !(finite(f) && finite(fa) && finite(fb)) {
                let at = if finite(f) && finite(fa) { bm.b } else { bm.a };
                self.fail(format!("force on node {at} is not finite at substep {n} (from beams)"));
                return;
            }
            force[bm.a] = fa;
            force[bm.b] = fb;
        }
        if let Some(extra) = extra {
            for (i, (f, e)) in force.iter_mut().zip(extra).enumerate() {
                let sum = *f + *e;
                if !finite(sum) {
                    self.fail(format!(
                        "force on node {i} is not finite at substep {n} (from extra forces)"
                    ));
                    return;
                }
                *f = sum;
            }
        }
        let i = (0..pos.len())
            .find(|&i| {
                let nv = vel[i] + force[i] * (h / mass[i]);
                !(finite(nv) && finite(pos[i] + nv * h))
            })
            .unwrap_or(0);
        self.fail(format!("update of node {i} is not finite at substep {n}"));
    }
}

impl Solver for SymplecticEuler {
    fn step(&mut self, s: State<'_>, h: f64) {
        self.step_inner(s, None, h);
    }

    fn step_with_forces(&mut self, s: State<'_>, extra: &[Vec3], h: f64) {
        assert_eq!(extra.len(), s.pos.len(), "one extra force per node");
        self.step_inner(s, Some(extra), h);
    }

    fn check(&self) -> Option<String> {
        self.error.clone()
    }
}

/// The largest beam stiffness, the largest beam damping and the lightest node mass of a
/// state: the (k, c, m) that the step-limit checks use. With no beams, k and c are 0.
pub fn step_limit_params(beams: &[Beam], mass: &[f64]) -> (f64, f64, f64) {
    let k = beams.iter().map(|b| b.stiffness).fold(0.0, f64::max);
    let c = beams.iter().map(|b| b.damping).fold(0.0, f64::max);
    let m = mass.iter().cloned().fold(f64::INFINITY, f64::min);
    (k, c, m)
}

/// Gershgorin bound on the highest angular frequency (rad/s) of a state, ignoring damping:
/// omega_max^2 <= 2 * max over nodes of (sum of the stiffnesses of the beams at that node)
/// / lightest mass. Valid for any body, unlike `STIFFEST_MODE_EIGENVALUE`, which is true
/// for the cube only.
pub fn gershgorin_omega_max(n_nodes: usize, beams: &[Beam], mass: &[f64]) -> f64 {
    let mut per_node = vec![0.0_f64; n_nodes];
    for b in beams {
        per_node[b.a] += b.stiffness;
        per_node[b.b] += b.stiffness;
    }
    let ksum = per_node.iter().cloned().fold(0.0, f64::max);
    let m = mass.iter().cloned().fold(f64::INFINITY, f64::min);
    (2.0 * ksum / m).sqrt()
}

/// Largest step (s) for which symplectic Euler is stable for this state, ignoring damping:
/// 2 / `gershgorin_omega_max` (the step must keep omega_max * h below 2).
pub fn gershgorin_step_limit(n_nodes: usize, beams: &[Beam], mass: &[f64]) -> f64 {
    2.0 / gershgorin_omega_max(n_nodes, beams, mass)
}

/// Error text if step `h` is not safe for a body of any shape, else None. The cube's
/// `STIFFEST_MODE_EIGENVALUE` = 8 does not hold for another body, so both the spring rate and
/// the damping rate of the stiffest mode are replaced by their Gershgorin bounds:
///     w2 = 2 * (largest sum of beam stiffnesses at one node) / (lightest mass)
///     g  = 2 * (largest sum of beam dampings at one node)    / (lightest mass)
/// and the step must meet the cube's coupled condition with them, h^2 w2 + 2 h g < 4 (see
/// `stable_step_limit_with_cause`). The spring part is a proven bound for any body. Pairing it
/// with the damping bound is conservative but not proven when beams of different materials
/// make the damping matrix not proportional to the stiffness matrix; for a body of one
/// material it is the cube's argument with 8 replaced by the larger bound.
pub fn body_step_error(h: f64, beams: &[Beam], mass: &[f64]) -> Option<String> {
    step_error_with(h, beams, mass, &[], &[])
}

/// One spring-damper of the step-limit check that is not a beam: its force is k (or c) times
/// the weighted sum of the coordinates of its ends, `ends` = (degree of freedom, weight).
/// A spring to the fixed ground has one end; a suspension pulls a wheel against the centroid of
/// n mount nodes (weight 1/n each, and -1 on the wheel).
#[derive(Clone, Debug, PartialEq)]
pub struct LimitElement {
    pub ends: Vec<(usize, f64)>,
    /// Stiffness (N/m).
    pub k: f64,
    /// Damping (N s/m).
    pub c: f64,
}

/// `body_step_error` for a body with extra degrees of freedom and elements (the wheels of
/// docs/PHYSICS.md "Wheels, suspension and tyres"). Degrees of freedom 0..mass.len() are the
/// nodes; mass.len() + j has mass `extra_mass[j]`. An element with stiffness matrix
/// k w w^T has Gershgorin row sum k |w_i| sum_j |w_j| at degree of freedom i, which is what a
/// beam contributes with k_at = k |w_i| sum_j |w_j| / 2 (for a beam, w = (1, -1): k). So each
/// element adds k |w_i| sum|w| / 2 to k_at[i] (and the same with c to c_at[i]), the lightest
/// mass is taken over nodes and extra masses, and the condition is the same
/// h^2 w2 + 2 h g < 4. With no extras the numbers are exactly those of `body_step_error`.
pub fn step_error_with(
    h: f64,
    beams: &[Beam],
    mass: &[f64],
    extra_mass: &[f64],
    elements: &[LimitElement],
) -> Option<String> {
    let n = mass.len() + extra_mass.len();
    let mut k_at = vec![0.0_f64; n];
    let mut c_at = vec![0.0_f64; n];
    for b in beams {
        k_at[b.a] += b.stiffness;
        k_at[b.b] += b.stiffness;
        c_at[b.a] += b.damping;
        c_at[b.b] += b.damping;
    }
    for e in elements {
        let wsum: f64 = e.ends.iter().map(|(_, w)| w.abs()).sum();
        for &(i, w) in &e.ends {
            k_at[i] += e.k * w.abs() * wsum / 2.0;
            c_at[i] += e.c * w.abs() * wsum / 2.0;
        }
    }
    let m = mass
        .iter()
        .chain(extra_mass)
        .cloned()
        .fold(f64::INFINITY, f64::min);
    let w2 = 2.0 * k_at.iter().cloned().fold(0.0, f64::max) / m;
    let g = 2.0 * c_at.iter().cloned().fold(0.0, f64::max) / m;
    if h * h * w2 + 2.0 * h * g < 4.0 {
        return None;
    }
    // The positive root of h^2 w2 + 2 h g = 4, written without cancellation (as in
    // `stable_step_limit_with_cause`); with w2 = 0 the limit is 2/g.
    let limit = 4.0 / (g + (g * g + 4.0 * w2).sqrt());
    Some(format!(
        "physics step h = dt/substeps = {h:.6} s reaches the body's stability limit {limit:.6} s (h^2*w2 + 2*h*g < 4 with the Gershgorin bounds w2 = {w2}, g = {g}); raise --substeps or lower --dt"
    ))
}
