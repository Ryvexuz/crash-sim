use crate::vec3::Vec3;
use crate::Beam;

pub const GRAVITY: f64 = 9.81;
pub const NODE_MASS: f64 = 1.0;
/// Beam stiffness (N/m). The highest cube mode is sqrt(8 k / m) = 400 rad/s
/// (see max_stable_step), so at the default 1 ms substep omega*dt is about 0.40, well under
/// the symplectic Euler stability limit of 2.
pub const BEAM_STIFFNESS: f64 = 20_000.0;
/// Axial damping (N*s/m). Damps impact ringing within the 5 s run. It lowers the stable
/// step: the coupled limit (see `stable_step_limit`) is 0.0041 s at 20, against 0.005 s
/// undamped, and `validate` enforces it.
pub const BEAM_DAMPING: f64 = 20.0;
/// Fraction of downward speed returned on ground contact.
pub const RESTITUTION: f64 = 0.5;
/// Horizontal speed loss rate (1/s) for a node touching the ground.
pub const FRICTION_RATE: f64 = 5.0;
/// Impact speeds below this (m/s) are treated as resting contact (no bounce),
/// otherwise gravity's per-substep velocity would keep re-bouncing forever.
pub const REST_SPEED: f64 = 0.2;
/// Strain above this means the run has blown up (or the cube is crushed); the sim aborts.
/// A beam yields only when |strain| exceeds its yield strain by more than this, so a strain that
/// rounding leaves one ulp past the limit right after a yield does not yield again.
pub const YIELD_EPS: f64 = 1e-12;
pub const MAX_STRAIN_LIMIT: f64 = 0.5;
/// Initial tilt in degrees about x and z when tilt is on.
pub const TILT_DEG: f64 = 5.0;

/// Largest physics step (s) that keeps symplectic Euler stable for the constants above,
/// ignoring damping (a conservative stiffness-only bound). `validate` still checks it
/// as well as `stable_step_limit`: Gershgorin holds for any deformed shape, whereas
/// `stable_step_limit` uses the stiffest mode of the rest shape.
///
/// Each node has 7 beams, so by Gershgorin the stiffness matrix has largest
/// eigenvalue <= 2 * 7 * k / m, i.e. omega_max <= sqrt(14 k / m) (about 529 rad/s;
/// the real cube mode is exactly 400). Symplectic Euler is stable for omega * h < 2.
/// This is a conservative bound, no extra margin needed.
pub fn max_stable_step() -> f64 {
    2.0 / (14.0 * BEAM_STIFFNESS / NODE_MASS).sqrt()
}

/// Largest eigenvalue of the beam matrix of the cube, in units of the beam constant: the
/// stiffest mode has omega^2 = 8 k / m (omega = 400 rad/s) and, because each beam's
/// damper acts along the same direction as its spring, the same mode has damping rate
/// gamma = 8 c / m. (Checked numerically in tests/stability.rs.)
pub const STIFFEST_MODE_EIGENVALUE: f64 = 8.0;

/// Which condition of `stable_step_limit` sets the limit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Binding {
    /// h^2 w2 + 2 h g < 4: spring and damper together (the stiffness limit is this
    /// condition with g = 0).
    Coupled,
    /// 0 < h g < 2: the damper alone. For w2 > 0 the coupled condition is always the
    /// tighter one, so this binds only when there is no spring.
    DamperAlone,
}

/// Largest step h (s), exclusive, for which the stiffest damped mode of the cube is stable
/// under this code's integrator, for beam stiffness `k`, damping `c` and node mass `m`.
/// Returns the limit and the condition that sets it.
///
/// Derivation. In a mode with spring rate w2 = omega^2 and damping rate g = gamma, one
/// symplectic Euler step (velocity first, then position with the new velocity) is
///     v' = (1 - h g) v - h w2 x,      x' = x + h v'
/// a 2x2 map with determinant 1 - h g and trace 2 - h g - h^2 w2. By the Jury criterion it
/// is stable iff  |det| < 1,  tr < 1 + det  and  -tr < 1 + det, that is
///     (A) 0 < h g < 2     and     (B) h^2 w2 + 2 h g < 4.
/// For the stiffest mode w2 = 8 k / m and g = 8 c / m. The largest h meeting (B) is the
/// positive root h = (-g + sqrt(g^2 + 4 w2)) / w2. At h = 2/g, (B) reads 4 w2/g^2 + 4 < 4,
/// which is false for w2 > 0, so the root is always below 2/g: (B) binds whenever there is
/// a spring, and (A) only when w2 = 0 (then the limit is 2/g, or unbounded if g = 0 too).
///
/// Damping makes the limit tighter than the undamped 2/omega. With today's constants
/// (k = 20000, m = 1) the limit is 0.005 s at c = 0 and 0.0041 s at c = 20; it falls below
/// the Gershgorin bound `max_stable_step()` (0.00378 s) once c is above about 28.3, so
/// the damping limit is real: a c between 28 and 66 would pass a separate c*h/m < 0.25 test
/// and still blow up. `Config::validate` therefore enforces this coupled condition itself.
pub fn stable_step_limit_with_cause(k: f64, c: f64, m: f64) -> (f64, Binding) {
    let w2 = STIFFEST_MODE_EIGENVALUE * k / m;
    let g = STIFFEST_MODE_EIGENVALUE * c / m;
    let damper_alone = if g > 0.0 { 2.0 / g } else { f64::INFINITY };
    // Same root as (-g + sqrt(g^2 + 4 w2)) / w2, written without cancellation.
    let coupled = if w2 > 0.0 { 4.0 / (g + (g * g + 4.0 * w2).sqrt()) } else { f64::INFINITY };
    if damper_alone < coupled {
        (damper_alone, Binding::DamperAlone)
    } else {
        (coupled, Binding::Coupled)
    }
}

/// The limit of `stable_step_limit_with_cause`, without the cause.
pub fn stable_step_limit(k: f64, c: f64, m: f64) -> f64 {
    stable_step_limit_with_cause(k, c, m).0
}

/// Error text if step `h` is not below `stable_step_limit(k, c, m)`, else None. One message
/// naming h, the limit and the condition that binds. `Config::validate` calls it with the
/// real constants; it takes them as arguments so any constants can be tested.
pub fn damped_step_error(h: f64, k: f64, c: f64, m: f64) -> Option<String> {
    let (limit, binding) = stable_step_limit_with_cause(k, c, m);
    if h < limit {
        return None;
    }
    let which = match binding {
        Binding::Coupled => "h^2*w2 + 2*h*g < 4 (spring and damper together)",
        Binding::DamperAlone => "0 < h*g < 2 (damper alone)",
    };
    Some(format!(
        "physics step h = dt/substeps = {h:.6} s reaches the damped stability limit {limit:.6} s, set by {which} with w2 = 8k/m, g = 8c/m (k = {k}, c = {c}, m = {m}); raise --substeps or lower --dt"
    ))
}

/// Relative safety margin of `clearly_within` on the squared bounds.
const CLEAR_MARGIN: f64 = 1e-9;

/// True only if a beam of squared length `d2` and rest length `rest` is certainly inside the
/// strain band, |len - rest| / rest <= `t`, as the float arithmetic `((len - rest) / rest).abs()
/// <= t` with len = sqrt(d2) would judge it. It never takes a sqrt, and false means "unknown":
/// the caller then runs the exact original arithmetic, so no result changes.
///
/// Derivation. For rest > 0 and 0 <= t < 1 the band is rest (1 - t) <= len <= rest (1 + t), and
/// since len >= 0 that is the same as rest^2 (1 - t)^2 <= d2 <= rest^2 (1 + t)^2. Float rounding
/// (d2, the sqrt, the subtraction and the division are each within a few 1e-16 relative) means
/// the squared form is not exactly the same test at the boundary, so the bounds are pulled in by
/// a relative `CLEAR_MARGIN` = 1e-9, which is at least 1e5 times the rounding error: a d2 strictly
/// between the pulled-in bounds has a computed strain at least about 4e-10 inside the band, so
/// the original test is false there. Everything near the boundary (inside the margin) returns
/// false and takes the sqrt. `t <= 0.9` keeps the lower margin, 5e-10 (1 - t), well above the
/// rounding error. `rest` in [1e-100, 1e100] keeps the squares far from overflow and underflow.
/// NaN in d2, rest or t, rest outside that range, and t outside [0, 0.9] make a comparison
/// false, so they return false and the caller's exact path keeps its NaN semantics (`!(a > b)`
/// is true for NaN there).
pub(crate) fn clearly_within(d2: f64, rest: f64, t: f64) -> bool {
    if !((1e-100..=1e100).contains(&rest) && (0.0..=0.9).contains(&t)) {
        return false;
    }
    let hi = rest * (1.0 + t);
    let lo = rest * (1.0 - t);
    d2 < hi * hi * (1.0 - CLEAR_MARGIN) && d2 > lo * lo * (1.0 + CLEAR_MARGIN)
}

/// Largest |length - rest| / rest. NaN-propagating: a blown-up state never reads as 0.
pub fn max_strain(beams: &[Beam], pos: &[Vec3]) -> f64 {
    max_strain_of(beams.iter(), pos)
}

/// The one strain fold behind `max_strain` and `max_strain_unbroken`, over beams in the order
/// given.
fn max_strain_of<'a>(beams: impl Iterator<Item = &'a Beam>, pos: &[Vec3]) -> f64 {
    let mut m = 0.0_f64;
    for b in beams {
        let s = ((pos[b.a] - pos[b.b]).length() - b.rest).abs() / b.rest;
        if s.is_nan() {
            return f64::NAN;
        }
        m = m.max(s);
    }
    m
}

/// `max_strain` over the beams not marked broken (`broken[i]` for `beams[i]`): the same
/// arithmetic in the same beam order, with the broken beams skipped in place rather than
/// copied out (`Sim::check` runs this every substep).
pub fn max_strain_unbroken(beams: &[Beam], pos: &[Vec3], broken: &[bool]) -> f64 {
    max_strain_of(
        beams
            .iter()
            .zip(broken)
            .filter(|(_, &br)| !br)
            .map(|(b, _)| b),
        pos,
    )
}
