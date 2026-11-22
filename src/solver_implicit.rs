//! The implicit node-beam solver of docs/M5-SOLVER.md option (1): backward Euler on the same
//! beams, yield rule and contact as `solver::SymplecticEuler` (only the time integration
//! changes), solved by Newton with a Jacobi-preconditioned CG (`sparse::pcg`) per iteration.
//!
//! **The step.** With `x`, `v` at the start of the step and `f(x, v)` = gravity + beams +
//! extra forces (the extra forces are held constant over the step), backward Euler is
//!     `M (v' - v) = h f(x + h v', v')`,   `x' = x + h v'`.
//! Newton solves `g(v') = M (v' - v) - h f(x + h v', v') = 0` from `v' = v`. Each iteration
//! assembles `A = M + h C + h^2 K` from per-beam 3x3 blocks as CSR triplets (duplicates summed
//! in a fixed order), with
//!     `C_e = c d d^T`,   `K_e = k d d^T + max(0, k (len - rest) / len) (I - d d^T)`,
//! (`d` the beam's unit direction at the current guess) and solves `A dv = -g` by `pcg`.
//! The negative geometric term of a compressed beam is dropped, and the derivative of the
//! damping force with respect to positions is left out, so `A` is symmetric positive definite
//! (M is diagonal and positive). Both only slow Newton: `g` is always the exact residual, so a
//! converged step solves the exact backward-Euler equations.
//!
//! **Convergence (the one rule, in `ImplicitEuler::solve`).** A step is converged when
//!     `||g(v')||_2 <= max(newton_rtol * ||g(v)||_2, ROUNDOFF * size(v'))`.
//! The first term is relative to the residual at the START of the step (`v' = v`), so the
//! start velocity is accepted only if `||g(v)|| <= ROUNDOFF * size(v)`, i.e. only if the whole
//! residual is within round-off of the terms it is made of: then no `f64` computation of `g`
//! can tell it from zero. `size` is the 2-norm, over all components, of the sizes of the terms
//! `g` sums, in units of momentum (M v): `m (|v'| + |v|) + h (|W| + sum over the node's beams of
//! |d| (k (|x_a| + |x_b| + rest) + c (|v'_a| + |v'_b|)))`, with `W` the gravity and extra
//! forces and `x_a`, `x_b` the guessed end positions (the beam's direction is a difference of
//! them, so its round-off is about 1e-16 of `|x_a| + |x_b|`). `ROUNDOFF` = 64 f64 epsilons.
//! Review r1 of task t-20261122-195241 showed that a tolerance relative to `size` alone
//! accepts the start velocity of a resting beam with a strain below `newton_rtol`.
//!
//! **Fails closed.** A non-finite input node, residual or new state, a `pcg` error (not
//! converged, matrix not positive definite, ...) and Newton reaching `newton_max_iter` are
//! errors. Every error goes through one function, the step then applies nothing, the first
//! error is kept, `check` returns it (so `Sim::check` ends the run) and no later step applies
//! anything. Nothing is clamped.
//!
//! **Determinism.** No threads, no hash maps; beams in index order, nodes in index order and
//! `pcg`'s fixed order. The same input gives the same bytes.

use crate::solver::{Solver, State};
use crate::sparse::{norm2, pcg, Csr};
use crate::vec3::Vec3;
use crate::Beam;

/// The solve's tolerances and caps. Every field is required; `ImplicitEuler::new` refuses a
/// value that is not finite and positive (tolerances) or is zero (caps).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tolerances {
    /// Newton stops when `||g||_2 <= newton_rtol * scale` (module docs).
    pub newton_rtol: f64,
    /// Newton iterations allowed per step; reaching it without convergence is an error.
    pub newton_max_iter: usize,
    /// Relative residual of each linear solve (`pcg`'s `rtol`).
    pub cg_rtol: f64,
    /// CG iterations allowed per linear solve (`pcg`'s `max_iter`).
    pub cg_max_iter: usize,
}

/// Counts of the work done, for timing and for the tests.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stats {
    /// Steps that converged and were applied.
    pub steps: u64,
    /// Newton iterations over all applied steps.
    pub newton_iters: u64,
    /// CG iterations over all applied steps.
    pub cg_iters: u64,
    /// The most Newton iterations one applied step took.
    pub max_newton_iters: usize,
}

/// Backward-Euler node-beam solver. Build it with `ImplicitEuler::new`.
#[derive(Clone, Debug)]
pub struct ImplicitEuler {
    tol: Tolerances,
    /// Steps taken so far, counting the one in progress.
    substep: u64,
    error: Option<String>,
    stats: Stats,
}

/// The round-off floor of the convergence rule (module docs): 64 f64 epsilons of the size of
/// the terms the residual sums. A numerical constant of the method, not a tuning knob.
const ROUNDOFF: f64 = 64.0 * f64::EPSILON;

fn finite(v: Vec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}

fn comp(v: Vec3, k: usize) -> f64 {
    match k {
        0 => v.x,
        1 => v.y,
        _ => v.z,
    }
}

impl ImplicitEuler {
    /// A solver with these tolerances. Refuses, by name, a tolerance that is not finite and
    /// positive and a cap of zero.
    pub fn new(tol: Tolerances) -> Result<ImplicitEuler, String> {
        if !(tol.newton_rtol.is_finite() && tol.newton_rtol > 0.0) {
            return Err(format!(
                "implicit solver: newton_rtol = {} must be finite and > 0",
                tol.newton_rtol
            ));
        }
        if !(tol.cg_rtol.is_finite() && tol.cg_rtol > 0.0) {
            return Err(format!(
                "implicit solver: cg_rtol = {} must be finite and > 0",
                tol.cg_rtol
            ));
        }
        if tol.newton_max_iter == 0 {
            return Err("implicit solver: newton_max_iter must be at least 1".to_string());
        }
        if tol.cg_max_iter == 0 {
            return Err("implicit solver: cg_max_iter must be at least 1".to_string());
        }
        Ok(ImplicitEuler {
            tol,
            substep: 0,
            error: None,
            stats: Stats::default(),
        })
    }

    pub fn tolerances(&self) -> Tolerances {
        self.tol
    }

    pub fn stats(&self) -> Stats {
        self.stats
    }

    fn fail(&mut self, msg: String) {
        if self.error.is_none() {
            self.error = Some(msg);
        }
    }

    /// One backward-Euler step; on any error nothing is written.
    fn step_inner(&mut self, s: State<'_>, extra: Option<&[Vec3]>, h: f64) {
        self.substep += 1;
        if self.error.is_some() {
            return;
        }
        let n = self.substep;
        match self.solve(&s, extra, h) {
            Ok((new_vel, newton, cg)) => {
                let new_pos: Vec<Vec3> = (0..s.pos.len())
                    .map(|i| s.pos[i] + new_vel[i] * h)
                    .collect();
                if let Some(i) =
                    (0..new_pos.len()).find(|&i| !(finite(new_pos[i]) && finite(new_vel[i])))
                {
                    self.fail(format!(
                        "implicit solver: update of node {i} is not finite at substep {n}"
                    ));
                    return;
                }
                s.vel.copy_from_slice(&new_vel);
                s.pos.copy_from_slice(&new_pos);
                self.stats.steps += 1;
                self.stats.newton_iters += newton as u64;
                self.stats.cg_iters += cg as u64;
                self.stats.max_newton_iters = self.stats.max_newton_iters.max(newton);
            }
            Err(msg) => self.fail(format!("implicit solver: {msg} at substep {n}")),
        }
    }

    /// Newton for v'. Returns (v', Newton iterations, CG iterations) or the error text.
    fn solve(
        &self,
        s: &State<'_>,
        extra: Option<&[Vec3]>,
        h: f64,
    ) -> Result<(Vec<Vec3>, usize, usize), String> {
        let (pos, vel, beams, mass) = (&*s.pos, &*s.vel, s.beams, s.mass);
        let nn = pos.len();
        if !(h.is_finite() && h > 0.0) {
            return Err(format!("step h = {h} must be finite and > 0"));
        }
        if let Some(e) = extra {
            if e.len() != nn {
                return Err(format!("{} extra forces for {nn} nodes", e.len()));
            }
            if let Some(i) = (0..nn).find(|&i| !finite(e[i])) {
                return Err(format!("extra force on node {i} is not finite"));
            }
        }
        if let Some(i) = (0..nn).find(|&i| !(finite(pos[i]) && finite(vel[i]))) {
            return Err(format!("node {i} is not finite"));
        }
        if let Some(i) = (0..nn).find(|&i| !(mass[i].is_finite() && mass[i] > 0.0)) {
            return Err(format!(
                "mass of node {i} = {} must be finite and > 0",
                mass[i]
            ));
        }
        if !s.gravity.is_finite() {
            return Err("gravity is not finite".to_string());
        }
        // The forces that do not depend on the state: gravity and the extra forces.
        let ext: Vec<Vec3> = (0..nn)
            .map(|i| Vec3::new(0.0, -s.gravity * mass[i], 0.0) + extra.map_or(Vec3::ZERO, |e| e[i]))
            .collect();
        let mut vn: Vec<Vec3> = vel.to_vec();
        let (mut g, mut size) = residual(pos, vel, &vn, beams, mass, &ext, h)?;
        let g0 = norm2(&g);
        let (mut iters, mut cg_total) = (0usize, 0usize);
        loop {
            let gn = norm2(&g);
            if !(gn.is_finite() && size.is_finite() && g0.is_finite()) {
                return Err(format!(
                    "Newton residual is not finite after {iters} iterations"
                ));
            }
            // The convergence rule (module docs): relative to the start residual, with a floor
            // at the round-off of the terms g is made of.
            let goal = (self.tol.newton_rtol * g0).max(ROUNDOFF * size);
            if gn <= goal {
                return Ok((vn, iters, cg_total));
            }
            if iters >= self.tol.newton_max_iter {
                return Err(format!(
                    "Newton did not converge in {iters} iterations (residual {gn:e} > {goal:e} = max(newton_rtol {} x start residual {g0:e}, round-off floor {:e}))",
                    self.tol.newton_rtol,
                    ROUNDOFF * size
                ));
            }
            let guess: Vec<Vec3> = (0..nn).map(|i| pos[i] + vn[i] * h).collect();
            let a = assemble(&guess, beams, mass, h)?;
            let rhs: Vec<f64> = g.iter().map(|v| -v).collect();
            let (dv, cg, _) =
                pcg(&a, &rhs, None, self.tol.cg_rtol, self.tol.cg_max_iter).map_err(|e| {
                    format!("linear solve of Newton iteration {} failed: {e}", iters + 1)
                })?;
            cg_total += cg;
            for i in 0..nn {
                vn[i] += Vec3::new(dv[3 * i], dv[3 * i + 1], dv[3 * i + 2]);
            }
            iters += 1;
            (g, size) = residual(pos, vel, &vn, beams, mass, &ext, h)?;
        }
    }
}

/// `g(v') = M (v' - v) - h f(x + h v', v')`, flattened (x, y, z per node), and the 2-norm of
/// the sizes of the terms it sums (module docs, "Convergence").
fn residual(
    pos: &[Vec3],
    vel: &[Vec3],
    vn: &[Vec3],
    beams: &[Beam],
    mass: &[f64],
    ext: &[Vec3],
    h: f64,
) -> Result<(Vec<f64>, f64), String> {
    let nn = pos.len();
    let mut force = ext.to_vec();
    let abs = |v: Vec3| Vec3::new(v.x.abs(), v.y.abs(), v.z.abs());
    let mut size: Vec<Vec3> = ext.iter().map(|&e| abs(e)).collect();
    for bm in beams {
        let pa = pos[bm.a] + vn[bm.a] * h;
        let pb = pos[bm.b] + vn[bm.b] * h;
        let d = pb - pa;
        let len = d.length();
        if len < 1e-12 {
            continue;
        }
        let dir = d * (1.0 / len);
        let rel_v = (vn[bm.b] - vn[bm.a]).dot(dir);
        let mag = bm.stiffness * (len - bm.rest) + bm.damping * rel_v;
        let f = dir * mag;
        force[bm.a] += f;
        force[bm.b] += -f;
        // The sizes of the terms `mag` sums: d is a difference of the two end positions, k len
        // and k rest cancel at rest, as do the two velocities in rel_v.
        let mag_size = bm.stiffness.abs() * (pa.length() + pb.length() + bm.rest.abs())
            + bm.damping.abs() * (vn[bm.a].length() + vn[bm.b].length());
        let s = abs(dir) * mag_size;
        size[bm.a] += s;
        size[bm.b] += s;
    }
    let mut g = Vec::with_capacity(3 * nn);
    let mut terms = Vec::with_capacity(3 * nn);
    for i in 0..nn {
        let r = (vn[i] - vel[i]) * mass[i] - force[i] * h;
        if !finite(r) {
            return Err(format!("residual on node {i} is not finite"));
        }
        g.extend([r.x, r.y, r.z]);
        let t = (abs(vn[i]) + abs(vel[i])) * mass[i] + size[i] * h;
        terms.extend([t.x, t.y, t.z]);
    }
    Ok((g, norm2(&terms)))
}

/// `A = M + h C + h^2 K` at positions `x` as a CSR matrix (module docs for the blocks).
fn assemble(x: &[Vec3], beams: &[Beam], mass: &[f64], h: f64) -> Result<Csr, String> {
    let nn = x.len();
    let mut t: Vec<(usize, usize, f64)> = Vec::with_capacity(3 * nn + 36 * beams.len());
    for (i, &m) in mass.iter().enumerate() {
        for k in 0..3 {
            t.push((3 * i + k, 3 * i + k, m));
        }
    }
    for bm in beams {
        let d = x[bm.b] - x[bm.a];
        let len = d.length();
        if len < 1e-12 {
            continue;
        }
        let dir = d * (1.0 / len);
        let axial = h * bm.damping + h * h * bm.stiffness;
        // Geometric (tension) term only: the compressed beam's negative one is dropped so A
        // stays positive definite.
        let geo = h * h * (bm.stiffness * (len - bm.rest) / len).max(0.0);
        for r in 0..3 {
            for c in 0..3 {
                let ddt = comp(dir, r) * comp(dir, c);
                let id = if r == c { 1.0 } else { 0.0 };
                let v = axial * ddt + geo * (id - ddt);
                t.push((3 * bm.a + r, 3 * bm.a + c, v));
                t.push((3 * bm.b + r, 3 * bm.b + c, v));
                t.push((3 * bm.a + r, 3 * bm.b + c, -v));
                t.push((3 * bm.b + r, 3 * bm.a + c, -v));
            }
        }
    }
    Csr::from_triplets(3 * nn, 3 * nn, &t).map_err(|e| format!("assembly failed: {e}"))
}

impl Solver for ImplicitEuler {
    fn step(&mut self, s: State<'_>, h: f64) {
        self.step_inner(s, None, h);
    }

    fn step_with_forces(&mut self, s: State<'_>, extra: &[Vec3], h: f64) {
        self.step_inner(s, Some(extra), h);
    }

    fn check(&self) -> Option<String> {
        self.error.clone()
    }
}
