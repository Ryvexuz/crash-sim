//! Starter crash sim: a cube of 8 nodes and 28 beams falling onto a ground plane.
//!
//! Simple on purpose: spring-damper beams, symplectic Euler, y = 0 ground with
//! restitution and friction. Std only, deterministic (no RNG, fixed step order).

pub mod ai;
pub mod collide;
pub mod format;
pub mod input;
pub mod menu;
pub mod net;
pub mod pulse;
pub mod reference;
pub mod save;
pub mod settings;
pub mod settle;
pub mod sparse;
pub mod solver;
pub mod solver_implicit;
pub mod solver_m5;
pub mod speed_profile;
pub mod statehash;
pub mod track;
pub mod trajectory;
pub mod vec3;
pub mod vehicle;

pub use format::{Body, ParseError};

use collide::Plane;
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
        self.validate_numbers()?;
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

    /// The finite-number checks shared by `validate` and `validate_body` (same message).
    fn validate_numbers(&self) -> Result<(), String> {
        let ok = self.dt.is_finite() && self.dt > 0.0;
        if !(ok && self.duration.is_finite() && self.duration >= 0.0 && self.substeps >= 1
            && self.drop_height.is_finite() && self.drop_height >= 0.0
            && self.gravity.is_finite() && self.gravity >= 0.0)
        {
            return Err("dt must be > 0, duration >= 0, substeps >= 1, drop-height >= 0, gravity >= 0 (all finite)".into());
        }
        if self.dt < MIN_DT {
            return Err(format!(
                "dt = {} s is below {MIN_DT} s: the CSV time column has 4 decimals, so rows would repeat or merge; raise --dt",
                self.dt
            ));
        }
        Ok(())
    }

    /// Like `validate`, for a body loaded from a `.bsn` file. The cube's stiffest-mode
    /// numbers do not hold for another body, so the step limit is `solver::body_step_error`
    /// (a Gershgorin bound that is valid for any body). `drop_height`, `tilt` and `edge`
    /// do not apply to a loaded body: its positions come from the file.
    pub fn validate_body(&self, beams: &[Beam], mass: &[f64]) -> Result<(), String> {
        self.validate_numbers()?;
        let h = self.dt / self.substeps as f64;
        match solver::body_step_error(h, beams, mass) {
            Some(e) => Err(e),
            None => Ok(()),
        }
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
    /// How many beams have broken so far (always 0 for a body with no `break_strain`).
    pub broken_beams: usize,
}

/// One beam break: when (s, the end of the substep where the strain was found past the limit),
/// which beam (index into `Sim::beams`, which never shrinks) and its |strain| then.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BreakEvent {
    pub t: f64,
    pub beam: usize,
    pub strain: f64,
}

pub struct Sim {
    pub cfg: Config,
    pub pos: Vec<Vec3>,
    pub vel: Vec<Vec3>,
    pub beams: Vec<Beam>,
    /// Mass of each node (kg).
    pub mass: Vec<f64>,
    /// Time (s) of the first physics substep at which any node was at or behind any
    /// collider plane, ground or wall (0.0 if the body starts there); None until then. Set inside the substep
    /// loop, so it cannot miss a contact that starts and ends between two output samples.
    pub first_contact_s: Option<f64>,
    /// Collider planes: the ground first, then the body's own, in file order. `integrate`
    /// applies them with `collide::apply` after every solver step.
    pub planes: Vec<Plane>,
    /// Per beam: the strain beyond which it yields (its rest length moves, see
    /// `apply_yield`), or None for an elastic beam. One entry per beam, same order.
    pub yield_strain: Vec<Option<f64>>,
    /// Per beam: its rest length when the sim was built (rest0), same order as `beams`. Never
    /// changes. Compressive yield flows at the force k * y * rest0 (`apply_yield`).
    pub rest0: Vec<f64>,
    /// Indices of beams that exceeded their yield strain while their two nodes were (almost)
    /// at the same place, where the new rest length would be zero: their rest length was left
    /// alone and `check` reports them.
    pub yield_refused: Vec<usize>,
    /// Per beam: the strain beyond which it breaks for good, or None for an unbreakable beam.
    pub break_strain: Vec<Option<f64>>,
    /// Per beam: true once it has broken. A broken beam stays in `beams` (so indices never move)
    /// but is left out of the solve, the strain checks and the yield pass.
    pub broken: Vec<bool>,
    /// Every break so far, in the order they happened (within one substep, by beam index).
    pub breaks: Vec<BreakEvent>,
    /// The wheels, for a sim built by `Sim::from_car` from a body with wheels; None otherwise.
    pub car: Option<vehicle::Car>,
    /// Gravity tilt theta (rad, `set_gravity_tilt`): gravity is g (sin theta, -cos theta, 0).
    gravity_tilt: f64,
    solver: Box<dyn Solver>,
    step: u64,
    /// The per-node extra force of the tilted-gravity and car path of `integrate`, kept between
    /// substeps so the path does not allocate one Vec per substep (like `vehicle::StepScratch`).
    extra_scratch: Vec<Vec3>,
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
        Sim::with_solver(cfg, Box::new(SymplecticEuler::default()))
    }

    /// A cube sim advanced by the given solver.
    pub fn with_solver(cfg: Config, solver: Box<dyn Solver>) -> Sim {
        let (pos, beams, mass) = build_cube(&cfg);
        let planes = vec![Plane::ground()];
        let first_contact_s = if collide::any_at_or_behind(&pos, &planes) { Some(0.0) } else { None };
        let n = pos.len();
        let yield_strain = vec![None; beams.len()];
        let n_beams = beams.len();
        let rest0 = beams.iter().map(|b| b.rest).collect();
        Sim { cfg, pos, vel: vec![Vec3::ZERO; n], beams, mass, first_contact_s, planes, yield_strain, rest0, yield_refused: Vec::new(), break_strain: vec![None; n_beams], broken: vec![false; n_beams], breaks: Vec::new(), car: None, gravity_tilt: 0.0, solver, step: 0, extra_scratch: Vec::new() }
    }

    /// A sim of a body loaded from a `.bsn` file, with the default solver.
    pub fn from_body(cfg: Config, body: &Body) -> Result<Sim, String> {
        Sim::from_body_with_solver(cfg, body, Box::new(SymplecticEuler::default()))
    }

    /// A sim of a body loaded from a `.bsn` file, advanced by the given solver. Refused (Err)
    /// when the config is invalid, the step is not below the body's stability bound, or the
    /// body has a collider plane that is not valid (zero or non-finite normal). The sim's planes
    /// are the y = 0 ground, then the body's planes in file order.
    /// Positions, start velocities, masses and per-beam stiffness and damping come from the
    /// body; a material's `yield_strain` makes its beams plastic (`apply_yield`).
    ///
    /// A body with wheels is refused: this constructor does not simulate them, and every
    /// command that builds its sim here (the CSV run, the settle and crumple reports, the
    /// trajectory writer) measures nodes only, so the wheels would be dropped or not counted.
    /// `Sim::from_car` simulates them.
    pub fn from_body_with_solver(cfg: Config, body: &Body, solver: Box<dyn Solver>) -> Result<Sim, String> {
        if !body.wheels.is_empty() {
            return Err(format!(
                "the body has {} wheels, which this run does not simulate (they would be dropped): wheels are simulated by `Sim::from_car` (docs/PHYSICS.md, \"Wheels, suspension and tyres\"), which the command-line runs and reports do not use yet",
                body.wheels.len()
            ));
        }
        Sim::build_from_body(cfg, body, solver, false)
    }

    /// A sim of a body with wheels (or without: then it is `from_body`'s sim), with the default
    /// solver. Refused for everything `from_body` refuses except the wheels, and for wheel
    /// numbers that are not valid, a chassis whose up axis cannot be found, or a step that is not
    /// below the stability bound of the chassis together with the wheels
    /// (`vehicle::Car::limit_elements`).
    pub fn from_car(cfg: Config, body: &Body) -> Result<Sim, String> {
        Sim::from_car_with_solver(cfg, body, Box::new(SymplecticEuler::default()))
    }

    /// `from_car` with the given solver.
    pub fn from_car_with_solver(
        cfg: Config,
        body: &Body,
        solver: Box<dyn Solver>,
    ) -> Result<Sim, String> {
        Sim::build_from_body(cfg, body, solver, true)
    }

    fn build_from_body(
        cfg: Config,
        body: &Body,
        solver: Box<dyn Solver>,
        with_wheels: bool,
    ) -> Result<Sim, String> {
        // The one point every body constructor passes through: a solver that does not simulate
        // tets is refused a body that has them, as `from_body_with_solver` refuses wheels.
        if !body.tets.is_empty() && !solver.handles_solids() {
            return Err(format!(
                "the body has {} tets, which this solver does not simulate (they would be dropped): tets are simulated by `solver_m5::SolverM5` (docs/PHYSICS.md, \"Solids\")",
                body.tets.len()
            ));
        }
        let mut planes = vec![Plane::ground()];
        for p in &body.planes {
            planes.push(Plane::from_format(p)?);
        }
        let built = body.build();
        let car = if with_wheels && !body.wheels.is_empty() {
            cfg.validate_numbers()?;
            let car = vehicle::Car::new(
                body,
                &built.pos,
                &built.vel,
                &built.mass,
                planes.len(),
                cfg.gravity,
            )?;
            let (extra_mass, elements) = car.limit_elements(built.mass.len());
            let h = cfg.dt / cfg.substeps as f64;
            let (beams, mass) = (&built.beams, &built.mass);
            if let Some(e) = solver::step_error_with(h, beams, mass, &extra_mass, &elements) {
                return Err(format!(
                    "{e} (the chassis with its wheels, suspension and tyres)"
                ));
            }
            Some(car)
        } else {
            cfg.validate_body(&built.beams, &built.mass)?;
            None
        };
        let yield_strain: Vec<Option<f64>> = body
            .beams
            .iter()
            .map(|b| body.materials.iter().find(|m| m.name == b.material).expect("beam names an existing material").yield_strain)
            .collect();
        let break_strain: Vec<Option<f64>> = body
            .beams
            .iter()
            .map(|b| body.materials.iter().find(|m| m.name == b.material).expect("beam names an existing material").break_strain)
            .collect();
        for (i, (y, b)) in yield_strain.iter().zip(&break_strain).enumerate() {
            if let Some(b) = b {
                if !(b.is_finite() && *b > 0.0) || y.is_some_and(|y| *b <= y) {
                    return Err(format!("beam {i}: break_strain {b} must be a positive number above its yield_strain"));
                }
            }
        }
        let n_beams = built.beams.len();
        let rest0 = built.beams.iter().map(|b| b.rest).collect();
        let first_contact_s = if collide::any_at_or_behind(&built.pos, &planes) { Some(0.0) } else { None };
        Ok(Sim {
            cfg,
            pos: built.pos,
            vel: built.vel,
            beams: built.beams,
            mass: built.mass,
            first_contact_s,
            planes,
            yield_strain,
            rest0,
            yield_refused: Vec::new(),
            break_strain,
            broken: vec![false; n_beams],
            breaks: Vec::new(),
            car,
            gravity_tilt: 0.0,
            solver,
            step: 0,
            extra_scratch: Vec::new(),
        })
    }

    /// Tilt gravity by theta (rad) about z: it becomes g (sin theta, -cos theta, 0), so on the
    /// flat ground the body stands on a slope of angle theta, downhill +x (docs/STAGE6.md check
    /// 3.4; `vehicle::slope_angle` gives theta from a grip fraction). Refused unless theta is
    /// finite and |theta| < pi/2.
    pub fn set_gravity_tilt(&mut self, theta: f64) -> Result<(), String> {
        if !(theta.is_finite() && theta.abs() < std::f64::consts::FRAC_PI_2) {
            return Err(format!(
                "gravity tilt {theta} rad must be finite and below pi/2 in size"
            ));
        }
        self.gravity_tilt = theta;
        Ok(())
    }

    /// The gravity tilt theta (rad); 0 unless `set_gravity_tilt` changed it.
    pub fn gravity_tilt(&self) -> f64 {
        self.gravity_tilt
    }

    /// The gravitational acceleration vector (m/s^2).
    pub fn gravity_vector(&self) -> Vec3 {
        let (s, c) = self.gravity_tilt.sin_cos();
        Vec3::new(self.cfg.gravity * s, -self.cfg.gravity * c, 0.0)
    }

    /// Total energy (J): kinetic, gravitational (potential -m g . r, zero at the origin; for an
    /// untilted gravity this is m g y) and elastic over the nodes and unbroken beams, plus the
    /// wheels' terms (`vehicle::Car::energy`) for a car. For a body without wheels and without
    /// tilt it equals `settle::total_energy`.
    pub fn total_energy(&self) -> f64 {
        let gv = self.gravity_vector();
        let mut e = 0.0;
        for i in 0..self.pos.len() {
            let v = self.vel[i];
            e += 0.5 * self.mass[i] * v.dot(v) - self.mass[i] * gv.dot(self.pos[i]);
        }
        // The unbroken beams in index order, read in place (no copy: this runs every substep of
        // a car's settle report).
        for (b, _) in self
            .beams
            .iter()
            .zip(&self.broken)
            .filter(|(_, &broken)| !broken)
        {
            let stretch = (self.pos[b.a] - self.pos[b.b]).length() - b.rest;
            e += 0.5 * b.stiffness * stretch * stretch;
        }
        if let Some(car) = &self.car {
            e += car.energy(&self.planes, gv);
        }
        e
    }

    /// One solver substep of length `h` (no yield pass). `advance` does this and then
    /// `apply_yield`, once per substep; the two are public so a test can measure the energy
    /// between them.
    pub fn integrate(&mut self, h: f64) {
        // Broken beams are not given to the solver at all, so they cannot apply force.
        let live: Vec<Beam>;
        let beams: &[Beam] = if self.breaks.is_empty() {
            &self.beams
        } else {
            live = self.live_beams();
            &live
        };
        if self.car.is_none() && self.gravity_tilt == 0.0 {
            // The plain path, unchanged: the cube's output stays byte-for-byte what it was.
            self.solver.step(State { pos: &mut self.pos, vel: &mut self.vel, beams, mass: &self.mass, gravity: self.cfg.gravity }, h);
            collide::apply(&mut self.pos, &mut self.vel, &self.planes, h);
            return;
        }
        // Tilted gravity: its y part goes to the solver as `gravity`, its x part as an extra
        // force on every node. The wheels add their suspension reactions on the mount nodes.
        let gv = self.gravity_vector();
        // The buffer is taken out of `self` for the substep and put back at the end, so it is
        // refilled in place (same values, same order as the Vec it replaces) and only grows on
        // the first substep.
        let mut extra = std::mem::take(&mut self.extra_scratch);
        extra.clear();
        extra.extend(self.mass.iter().map(|&m| Vec3::new(m * gv.x, 0.0, 0.0)));
        if let Some(car) = self.car.as_mut() {
            let step = vehicle::Substep { gravity: gv, h };
            let (pos, vel, mass) = (&self.pos, &self.vel, &self.mass);
            car.pre_step(pos, vel, mass, &self.planes, step, &mut extra);
        }
        let state = State {
            pos: &mut self.pos,
            vel: &mut self.vel,
            beams,
            mass: &self.mass,
            gravity: -gv.y,
        };
        self.solver.step_with_forces(state, &extra, h);
        if let Some(car) = self.car.as_mut() {
            car.post_step(&mut self.pos, &mut self.vel, &self.mass, &self.planes, h);
        }
        collide::apply(&mut self.pos, &mut self.vel, &self.planes, h);
        self.extra_scratch = extra;
    }

    /// The beams that have not broken, in index order.
    pub fn live_beams(&self) -> Vec<Beam> {
        self.beams.iter().zip(&self.broken).filter(|(_, &b)| !b).map(|(bm, _)| *bm).collect()
    }

    /// Break every unbroken beam whose |strain| exceeds its `break_strain`, in beam index order.
    /// The time of an event is the end of substep number `substep` (1-based) of the current
    /// output step. A broken beam never comes back. Returns how many broke. NaN never breaks a
    /// beam (it stays for `check` to report).
    fn apply_breaks(&mut self, substep: u32, h: f64) -> usize {
        let t = self.step as f64 * self.cfg.dt + substep as f64 * h;
        let mut n = 0;
        for (i, bm) in self.beams.iter().enumerate() {
            let Some(limit) = self.break_strain[i] else { continue };
            if self.broken[i] {
                continue;
            }
            let strain = ((self.pos[bm.a] - self.pos[bm.b]).length() - bm.rest).abs() / bm.rest;
            if strain > limit {
                self.broken[i] = true;
                self.breaks.push(BreakEvent { t, beam: i, strain });
                n += 1;
            }
        }
        n
    }

    /// Plastic flow, after a substep. For every beam with a yield strain y (t = y + YIELD_EPS),
    /// strain s = (len - rest) / rest and rest0 its rest length when the sim was built:
    ///     stretched (s > t):                 rest' = len / (1 + y)    so that (len - rest') / rest' = +y
    ///     crushed ((len - rest) / rest0 < -t): rest' = len + y rest0    so that k (len - rest') = -k y rest0
    /// Tension returns the strain to the yield limit (perfectly plastic, as before). Compression
    /// flows at the constant force k y rest0 (task t-20261122-013607): the old compressive rule,
    /// rest' = len / (1 - y), let the force k y len / (1 - y) fall as the beam crushed. A beam
    /// whose strain s is NaN never yields. Positions and velocities are untouched, and
    /// |len - rest'| < |len - rest| (in compression rest - len > t rest0 > y rest0 = rest' - len),
    /// so the stored elastic energy only goes down. A beam whose new rest length would not be a
    /// positive finite number, or whose length is below 1e-12 m (where the force step skips it
    /// too), keeps its rest length and its index goes into `yield_refused`. Returns how many
    /// beams yielded. Breaking (`apply_breaks`) still measures strain against the current rest.
    pub fn apply_yield(&mut self) -> usize {
        let mut yielded = 0;
        for (i, bm) in self.beams.iter_mut().enumerate() {
            let Some(y) = self.yield_strain[i] else { continue };
            if self.broken[i] {
                continue;
            }
            let t = y + YIELD_EPS;
            let rest0 = self.rest0[i];
            let d = self.pos[bm.a] - self.pos[bm.b];
            // Cheap exit with no sqrt. `clearly_within` certifies computed s inside [-t, t], so
            // no tensile flow. With rest0 / 2 <= rest <= rest0 it also certifies no compressive
            // flow: it puts len above rest (1 - t) (1 + 5e-10), so len - rest > -t rest0 +
            // 5e-11 rest >= -t rest0 + 2.5e-11 rest0 (t <= 0.9), far beyond the few 1e-16 rest0
            // rounding of (len - rest) / rest0. Anything else, NaN included, takes the exact path.
            if bm.rest <= rest0 && bm.rest >= 0.5 * rest0 && clearly_within(d.dot(d), bm.rest, t) {
                continue;
            }
            let len = d.length();
            let strain = (len - bm.rest) / bm.rest;
            // A NaN strain skips the beam, as before; a NaN y fails both comparisons below.
            if strain.is_nan() {
                continue;
            }
            let new_rest = if strain > t {
                len / (1.0 + y)
            } else if (len - bm.rest) / rest0 < -t {
                len + y * rest0
            } else {
                continue;
            };
            if len < 1e-12 || !(new_rest.is_finite() && new_rest > 0.0) {
                if !self.yield_refused.contains(&i) {
                    self.yield_refused.push(i);
                }
                continue;
            }
            bm.rest = new_rest;
            yielded += 1;
        }
        yielded
    }

    fn substep(&mut self, substep: u32, h: f64) {
        self.integrate(h);
        // Breaking is judged before yielding: a beam with both limits must be able to reach
        // its break strain, which plastic flow would otherwise hold at the yield strain.
        self.apply_breaks(substep, h);
        self.apply_yield();
    }

    pub fn sample(&self) -> Sample {
        let min_y = self.pos.iter().map(|p| p.y).fold(f64::INFINITY, f64::min);
        let center_y = self.pos.iter().map(|p| p.y).sum::<f64>() / self.pos.len() as f64;
        let max_beam_strain = max_strain_unbroken(&self.beams, &self.pos, &self.broken);
        Sample { t: self.step as f64 * self.cfg.dt, min_y, center_y, max_beam_strain, broken_beams: self.breaks.len() }
    }

    /// Fail if the solver reports an error of its own (`Solver::check`, first: an m5 tet that
    /// inverted or met a NaN node is named as such), or any node position/velocity is
    /// non-finite or strain is NaN or above the limit.
    pub fn check(&self) -> Result<(), String> {
        if let Some(e) = self.solver.check() {
            return Err(e);
        }
        let finite = |v: &Vec3| v.x.is_finite() && v.y.is_finite() && v.z.is_finite();
        if let Some(i) = self.pos.iter().position(|p| !finite(p)) {
            return Err(format!("node {i} position is not finite"));
        }
        if let Some(i) = self.vel.iter().position(|v| !finite(v)) {
            return Err(format!("node {i} velocity is not finite"));
        }
        if let Some(&i) = self.yield_refused.first() {
            return Err(format!(
                "beam {i} would yield to a rest length of zero or negative (its nodes are at the same place)"
            ));
        }
        // Strain: when every unbroken beam is clearly inside the limit by its squared length (no
        // sqrt, see `clearly_within`) the result is Ok. Otherwise (a beam near or past the
        // limit, or NaN anywhere) the exact fold below decides and words the error, as before.
        let all_clear = self.beams.iter().zip(&self.broken).all(|(bm, &broken)| {
            broken || {
                let d = self.pos[bm.a] - self.pos[bm.b];
                clearly_within(d.dot(d), bm.rest, MAX_STRAIN_LIMIT)
            }
        });
        if !all_clear {
            let strain = max_strain_unbroken(&self.beams, &self.pos, &self.broken);
            if strain.is_nan() || strain > MAX_STRAIN_LIMIT {
                return Err(format!("max beam strain {strain} exceeds limit {MAX_STRAIN_LIMIT}"));
            }
        }
        if let Some(e) = self.car.as_ref().and_then(|c| c.check()) {
            return Err(e);
        }
        Ok(())
    }

    /// Largest node speed, m/s. For a car, also every wheel's centre speed and rim speed
    /// (|spin| x radius), so a spinning or bouncing wheel is never "at rest".
    pub fn max_speed(&self) -> f64 {
        let nodes = self.vel.iter().map(|v| v.length()).fold(0.0, f64::max);
        match &self.car {
            Some(c) => nodes.max(c.max_speed()),
            None => nodes,
        }
    }

    /// Advance one output step (`substeps` physics substeps).
    pub fn advance(&mut self) {
        let h = self.cfg.dt / self.cfg.substeps as f64;
        for k in 0..self.cfg.substeps {
            self.substep(k + 1, h);
            if self.first_contact_s.is_none() && collide::any_at_or_behind(&self.pos, &self.planes) {
                self.first_contact_s = Some(self.step as f64 * self.cfg.dt + (k + 1) as f64 * h);
            }
        }
        self.step += 1;
    }
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
fn clearly_within(d2: f64, rest: f64, t: f64) -> bool {
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

/// Everything a body run produces: the samples of the good steps, the first contact time, every
/// beam break, and the run's verdict.
pub struct Report {
    pub samples: Vec<Sample>,
    pub first_contact_s: Option<f64>,
    pub breaks: Vec<BreakEvent>,
    pub result: Result<(), String>,
}

/// Smallest dt the run accepts: the CSV time column has 4 decimals, so a smaller dt would
/// print repeated or merged t values.
pub const MIN_DT: f64 = 1e-4;

/// Output steps of a run: `floor(duration / dt)`, with a relative epsilon of 1e-9 so that a
/// duration that is a whole number of steps up to rounding (5.0 / 0.01, 0.3 / 0.01) is not
/// cut short. The last row therefore never lies after `duration` (it can lie up to dt before it).
pub fn step_count(cfg: &Config) -> u64 {
    (cfg.duration / cfg.dt * (1.0 + 1e-9)).floor() as u64
}

/// Run the sim. Returns the samples of all good steps (t = 0, dt, 2dt, ...) and,
/// if the config was refused or the state blew up, the reason (with the time).
/// The step count is `step_count(cfg)`: `floor(duration / dt)` with a tiny relative epsilon,
/// so the last row never lies after `duration`.
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
    let r = run_sim(Sim::new(cfg.clone()));
    (r.samples, r.first_contact_s, r.result)
}

/// Like `run_checked_with_contact`, for a body loaded from a `.bsn` file. A refused body or
/// config comes back as the error with no samples.
pub fn run_body_checked_with_contact(cfg: &Config, body: &Body) -> (Vec<Sample>, Option<f64>, Result<(), String>) {
    let r = run_body_report(cfg, body);
    (r.samples, r.first_contact_s, r.result)
}

/// Like `run_body_checked_with_contact`, and also returns the beam breaks.
pub fn run_body_report(cfg: &Config, body: &Body) -> Report {
    match Sim::from_body(cfg.clone(), body) {
        Ok(sim) => run_sim(sim),
        Err(e) => Report { samples: Vec::new(), first_contact_s: None, breaks: Vec::new(), result: Err(e) },
    }
}

/// The run loop shared by both entry points, on a sim whose config is already validated.
fn run_sim(mut sim: Sim) -> Report {
    let cfg = sim.cfg.clone();
    let steps = step_count(&cfg);
    let mut out = Vec::with_capacity(steps as usize + 1);
    out.push(sim.sample());
    for _ in 0..steps {
        sim.advance();
        if let Err(e) = sim.check() {
            let result = Err(format!("unstable at t = {:.4} s: {e}", sim.sample().t));
            return Report { samples: out, first_contact_s: sim.first_contact_s, breaks: sim.breaks, result };
        }
        out.push(sim.sample());
    }
    Report { samples: out, first_contact_s: sim.first_contact_s, breaks: sim.breaks, result: Ok(()) }
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

/// The CSV header of a body run: the cube header plus the `broken_beams` count.
pub const CSV_HEADER_BROKEN: &str = "t,min_y,center_y,max_beam_strain,broken_beams";

/// Like `to_csv`, with a final `broken_beams` column (used for `--body` runs only, so the cube's
/// output stays as it was).
pub fn to_csv_with_broken(samples: &[Sample]) -> String {
    let mut s = String::from(CSV_HEADER_BROKEN);
    s.push('\n');
    for r in samples {
        s.push_str(&format!("{:.4},{:.6},{:.6},{:.6},{}\n", r.t, r.min_y, r.center_y, r.max_beam_strain, r.broken_beams));
    }
    s
}

/// One line per break, `break t=<4 decimals> beam=<index> strain=<6 decimals>` (strain is the
/// absolute value), each ending in a newline; empty when nothing broke.
pub fn events_text(breaks: &[BreakEvent]) -> String {
    breaks.iter().map(|b| format!("break t={:.4} beam={} strain={:.6}\n", b.t, b.beam, b.strain)).collect()
}
