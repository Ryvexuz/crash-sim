//! Starter crash sim: a cube of 8 nodes and 28 beams falling onto a ground plane.
//!
//! Simple on purpose: spring-damper beams, symplectic Euler, y = 0 ground with
//! restitution and friction. Std only, deterministic (no RNG, fixed step order).

pub mod format;
pub mod settle;
pub mod solver;
pub mod vec3;

pub use format::{Body, ParseError};

use solver::{Solver, State, SymplecticEuler};
use vec3::Vec3;

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

#[derive(Clone, Debug)]
pub struct Config {
    pub duration: f64,
    /// Output step in seconds.
    pub dt: f64,
    /// Physics substeps per output step.
    pub substeps: u32,
    /// Height of the lowest corner above the ground, metres.
    pub drop_height: f64,
    pub tilt: bool,
    pub edge: f64,
    /// Gravitational acceleration in m/s^2 (Earth 9.81, Moon 1.62, Mars 3.71).
    pub gravity: f64,
}

impl Default for Config {
    fn default() -> Self {
        Config { duration: 5.0, dt: 0.01, substeps: 10, drop_height: 2.0, tilt: true, edge: 1.0, gravity: GRAVITY }
    }
}

impl Config {
    /// Reject settings that cannot give a valid run, including unstable physics steps.
    pub fn validate(&self) -> Result<(), String> {
        let ok = self.dt.is_finite() && self.dt > 0.0;
        if !(ok && self.duration.is_finite() && self.duration >= 0.0 && self.substeps >= 1
            && self.drop_height.is_finite() && self.drop_height >= 0.0
            && self.gravity.is_finite() && self.gravity > 0.0)
        {
            return Err("dt must be > 0, duration >= 0, substeps >= 1, drop-height >= 0, gravity > 0 (all finite)".into());
        }
        let h = self.dt / self.substeps as f64;
        // The limits come from the stiffest beam and the lightest node of the body this
        // config builds (today always the cube, whose numbers are the constants above).
        let (_, beams, mass) = build_cube(self);
        let (k, c, m) = solver::step_limit_params(&beams, &mass);
        let omega_max = solver::gershgorin_omega_max(mass.len(), &beams, &mass);
        let limit = solver::gershgorin_step_limit(mass.len(), &beams, &mass);
        if h > limit {
            return Err(format!(
 "physics step h = dt/substeps = {h:.6} s exceeds the stable limit {limit:.6} s (symplectic Euler needs omega_max*h < 2 with omega_max <= sqrt(14*k/m) = {:.0} rad/s, k = {k}); raise --substeps or lower --dt",
                omega_max
            ));
        }
        if let Some(e) = damped_step_error(h, k, c, m) {
            return Err(e);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Beam {
    pub a: usize,
    pub b: usize,
    pub rest: f64,
    /// Spring stiffness (N/m).
    pub stiffness: f64,
    /// Axial damping (N*s/m).
    pub damping: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct Sample {
    pub t: f64,
    pub min_y: f64,
    pub center_y: f64,
    pub max_beam_strain: f64,
}

pub struct Sim {
    pub cfg: Config,
    pub pos: Vec<Vec3>,
    pub vel: Vec<Vec3>,
    pub beams: Vec<Beam>,
    /// Mass of each node (kg).
    pub mass: Vec<f64>,
    /// Time (s) of the first physics substep at which any node was at or below the
    /// ground (0.0 if the cube starts there); None until then. Set inside the substep
    /// loop, so it cannot miss a contact that starts and ends between two output samples.
    pub first_contact_s: Option<f64>,
    solver: Box<dyn Solver>,
    step: u64,
}

/// The cube's start positions, beams and node masses for `cfg`.
fn build_cube(cfg: &Config) -> (Vec<Vec3>, Vec<Beam>, Vec<f64>) {
    let e = cfg.edge;
    let mut pos: Vec<Vec3> = (0..8)
        .map(|i| {
            Vec3::new(
                if i & 1 != 0 { e } else { 0.0 } - e / 2.0,
                if i & 2 != 0 { e } else { 0.0 } - e / 2.0,
                if i & 4 != 0 { e } else { 0.0 } - e / 2.0,
            )
        })
        .collect();
    if cfg.tilt {
        let (s, c) = TILT_DEG.to_radians().sin_cos();
        for p in pos.iter_mut() {
            // rotate about x, then about z (cube is centred on the origin)
            let (y, z) = (p.y * c - p.z * s, p.y * s + p.z * c);
            let (x, y) = (p.x * c - y * s, p.x * s + y * c);
            *p = Vec3::new(x, y, z);
        }
    }
    let lowest = pos.iter().map(|p| p.y).fold(f64::INFINITY, f64::min);
    let shift = cfg.drop_height - lowest;
    for p in pos.iter_mut() {
        p.y += shift;
    }
    // Every node pair is a beam: 12 edges + 12 face diagonals + 4 body diagonals = 28.
    let mut beams = Vec::new();
    for a in 0..8 {
        for b in (a + 1)..8 {
            beams.push(Beam { a, b, rest: (pos[a] - pos[b]).length(), stiffness: BEAM_STIFFNESS, damping: BEAM_DAMPING });
        }
    }
    let mass = vec![NODE_MASS; pos.len()];
    (pos, beams, mass)
}

impl Sim {
    /// A cube sim with the default solver, `SymplecticEuler`.
    pub fn new(cfg: Config) -> Sim {
        Sim::with_solver(cfg, Box::new(SymplecticEuler))
    }

    /// A cube sim advanced by the given solver.
    pub fn with_solver(cfg: Config, solver: Box<dyn Solver>) -> Sim {
        let (pos, beams, mass) = build_cube(&cfg);
        let first_contact_s = if pos.iter().any(|p| p.y <= 0.0) { Some(0.0) } else { None };
        let n = pos.len();
        Sim { cfg, pos, vel: vec![Vec3::ZERO; n], beams, mass, first_contact_s, solver, step: 0 }
    }

    fn substep(&mut self, h: f64) {
        self.solver.step(
            State { pos: &mut self.pos, vel: &mut self.vel, beams: &self.beams, mass: &self.mass, gravity: self.cfg.gravity },
            h,
        );
    }

    pub fn sample(&self) -> Sample {
        let min_y = self.pos.iter().map(|p| p.y).fold(f64::INFINITY, f64::min);
        let center_y = self.pos.iter().map(|p| p.y).sum::<f64>() / self.pos.len() as f64;
        let max_beam_strain = max_strain(&self.beams, &self.pos);
        Sample { t: self.step as f64 * self.cfg.dt, min_y, center_y, max_beam_strain }
    }

    /// Fail if any node position/velocity is non-finite or strain is NaN or above the limit.
    pub fn check(&self) -> Result<(), String> {
        let finite = |v: &Vec3| v.x.is_finite() && v.y.is_finite() && v.z.is_finite();
        if let Some(i) = self.pos.iter().position(|p| !finite(p)) {
            return Err(format!("node {i} position is not finite"));
        }
        if let Some(i) = self.vel.iter().position(|v| !finite(v)) {
            return Err(format!("node {i} velocity is not finite"));
        }
        let strain = max_strain(&self.beams, &self.pos);
        if strain.is_nan() || strain > MAX_STRAIN_LIMIT {
            return Err(format!("max beam strain {strain} exceeds limit {MAX_STRAIN_LIMIT}"));
        }
        Ok(())
    }

    /// Largest node speed, m/s.
    pub fn max_speed(&self) -> f64 {
        self.vel.iter().map(|v| v.length()).fold(0.0, f64::max)
    }

    /// Advance one output step (`substeps` physics substeps).
    pub fn advance(&mut self) {
        let h = self.cfg.dt / self.cfg.substeps as f64;
        for k in 0..self.cfg.substeps {
            self.substep(h);
            if self.first_contact_s.is_none() && self.pos.iter().any(|p| p.y <= 0.0) {
                self.first_contact_s = Some(self.step as f64 * self.cfg.dt + (k + 1) as f64 * h);
            }
        }
        self.step += 1;
    }
}

/// Largest |length - rest| / rest. NaN-propagating: a blown-up state never reads as 0.
pub fn max_strain(beams: &[Beam], pos: &[Vec3]) -> f64 {
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

/// Run the sim. Returns the samples of all good steps (t = 0, dt, 2dt, ...) and,
/// if the config was refused or the state blew up, the reason (with the time).
/// The step count is `round(duration / dt)`, so the last row can differ from
/// `duration` by up to dt/2 when dt does not divide it.
pub fn run_checked(cfg: &Config) -> (Vec<Sample>, Result<(), String>) {
    let (samples, _, res) = run_checked_with_contact(cfg);
    (samples, res)
}

/// Like `run_checked`, but also returns `Sim::first_contact_s` as it stood when the run
/// ended (at the abort if the run blew up).
pub fn run_checked_with_contact(cfg: &Config) -> (Vec<Sample>, Option<f64>, Result<(), String>) {
    if let Err(e) = cfg.validate() {
        return (Vec::new(), None, Err(e));
    }
    let steps = (cfg.duration / cfg.dt).round() as u64;
    let mut sim = Sim::new(cfg.clone());
    let mut out = Vec::with_capacity(steps as usize + 1);
    out.push(sim.sample());
    for _ in 0..steps {
        sim.advance();
        if let Err(e) = sim.check() {
            return (out, sim.first_contact_s, Err(format!("unstable at t = {:.4} s: {e}", sim.sample().t)));
        }
        out.push(sim.sample());
    }
    (out, sim.first_contact_s, Ok(()))
}

/// Run the whole sim; panics if the config is refused or the sim blows up.
pub fn run(cfg: &Config) -> Vec<Sample> {
    let (samples, res) = run_checked(cfg);
    res.expect("simulation failed");
    samples
}

pub const CSV_HEADER: &str = "t,min_y,center_y,max_beam_strain";

pub fn to_csv(samples: &[Sample]) -> String {
    let mut s = String::from(CSV_HEADER);
    s.push('\n');
    for r in samples {
        s.push_str(&format!("{:.4},{:.6},{:.6},{:.6}\n", r.t, r.min_y, r.center_y, r.max_beam_strain));
    }
    s
}
