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
}

/// Semi-implicit (symplectic) Euler. Contact with the ground and the walls is not here:
/// `Sim` applies `collide::apply` after every solver step.
#[derive(Clone, Copy, Debug, Default)]
pub struct SymplecticEuler;

impl Solver for SymplecticEuler {
    fn step(&mut self, s: State<'_>, h: f64) {
        symplectic_euler(s, None, h);
    }

    fn step_with_forces(&mut self, s: State<'_>, extra: &[Vec3], h: f64) {
        assert_eq!(extra.len(), s.pos.len(), "one extra force per node");
        symplectic_euler(s, Some(extra), h);
    }
}

/// The one symplectic Euler step. With `extra` None the arithmetic is exactly the old `step`
/// (so the cube's output is byte-for-byte unchanged); with Some, each node's extra force is
/// added after its beam forces.
fn symplectic_euler(s: State<'_>, extra: Option<&[Vec3]>, h: f64) {
    let State { pos, vel, beams, mass, gravity } = s;
    let mut force: Vec<Vec3> = mass.iter().map(|&m| Vec3::new(0.0, -gravity * m, 0.0)).collect();
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
    for i in 0..pos.len() {
        // semi-implicit Euler: velocity first, then position with the new velocity
        vel[i] += force[i] * (h / mass[i]);
        pos[i] += vel[i] * h;
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
