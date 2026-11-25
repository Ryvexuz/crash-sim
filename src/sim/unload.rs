use super::Sim;

/// Unloading return, after every physics substep of length h, after breaks and yield.
/// For each unbroken beam with `unload_return` u:
/// 1. d = pos[b] - pos[a], len = |d|. Skip non-finite len or len < 1e-12 m
///    (the force-step cutoff); `check` reports a non-finite state.
/// 2. x = len - rest, r = d . (vel[b] - vel[a]) / len (after solver and contacts),
///    delta = r * h.
/// 3. Unloading means (x < 0 && delta > 0) || (x > 0 && delta < 0).
///    Loading, zero x or delta, and NaN leave the beam alone.
/// 4. beta = 1/u - 1. If beta == 0 (u == 1), change nothing, bit for bit.
/// 5. shift = beta * delta. If |shift| >= |x|, rest' = len; otherwise
///    rest' = rest - shift, so x' = x + shift moves toward zero without crossing it.
///    When 1/u overflows, use delta / u - delta (an infinite delta / u stays
///    infinite), the same value without the reciprocal overflow.
/// 6. Keep rest unless rest' is positive and finite (the fail-closed guard).
///
/// Only rest changes: positions, velocities, stiffness, damping, rest0 and all other
/// fields are untouched. Returns the number of beams whose rest length changed.
/// This loop allocates nothing.
///
/// While unloading, x shrinks by (1 + beta) per unit of lengthening, so the spring
/// returns k*x0^2 / (2*(1 + beta)) = u*(1/2)*k*x0^2. Each change has |x'| <= |x|,
/// so stored elastic energy never rises. Loading, stiffness and the step limit are
/// untouched. Rest may move below rest0 (compression set) or above it (tension set);
/// apply_yield already takes its exact path outside [0.5*rest0, rest0].
pub(super) fn apply(sim: &mut Sim, h: f64) -> usize {
    let mut changed = 0;
    for (i, beam) in sim.beams.iter_mut().enumerate() {
        let Some(u) = sim.unload_return[i] else {
            continue;
        };
        if sim.broken[i] {
            continue;
        }
        let d = sim.pos[beam.b] - sim.pos[beam.a];
        let len = d.length();
        if !len.is_finite() || len < 1e-12 {
            continue;
        }
        let x = len - beam.rest;
        let r = d.dot(sim.vel[beam.b] - sim.vel[beam.a]) / len;
        let delta = r * h;
        if !((x < 0.0 && delta > 0.0) || (x > 0.0 && delta < 0.0)) {
            continue;
        }
        let beta = 1.0 / u - 1.0;
        if beta == 0.0 {
            continue;
        }
        let shift = if beta.is_finite() {
            beta * delta
        } else {
            let q = delta / u;
            if q.is_finite() {
                q - delta
            } else {
                q
            }
        };
        let rest = if shift.abs() >= x.abs() {
            len
        } else {
            beam.rest - shift
        };
        if rest.is_finite() && rest > 0.0 && rest != beam.rest {
            beam.rest = rest;
            changed += 1;
        }
    }
    changed
}
