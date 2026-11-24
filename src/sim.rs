use super::*;

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
    pub(super) fn validate_numbers(&self) -> Result<(), String> {
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
    pub(super) gravity_tilt: f64,
    pub(super) solver: Box<dyn Solver>,
    pub(super) step: u64,
    /// The per-node extra force of the tilted-gravity and car path of `integrate`, kept between
    /// substeps so the path does not allocate one Vec per substep (like `vehicle::StepScratch`).
    pub(super) extra_scratch: Vec<Vec3>,
}

/// The step-limit refusal of every body sim (`Sim::build_from_body`, with and without wheels):
/// the one place the explicit limit is applied or relaxed. For a solver that integrates the
/// beams explicitly (`Solver::explicit_beams`, true by default) it is exactly
/// `solver::step_error_with` with the beams (with no extra masses and elements that is
/// `solver::body_step_error`, which `Config::validate_body` applies). For an implicit beam
/// solver the beams leave the bound; the extra masses and elements (the wheels, suspension and
/// tyres, integrated explicitly by the vehicle code) stay in it with the same node masses.
fn step_limit_error(
    solver: &dyn Solver,
    h: f64,
    beams: &[Beam],
    mass: &[f64],
    extra_mass: &[f64],
    elements: &[solver::LimitElement],
) -> Option<String> {
    let beams = if solver.explicit_beams() { beams } else { &[] };
    solver::step_error_with(h, beams, mass, extra_mass, elements)
}

/// The cube's start positions, beams and node masses for `cfg`.
pub(super) fn build_cube(cfg: &Config) -> (Vec<Vec3>, Vec<Beam>, Vec<f64>) {
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
        // The finite-number checks come first on every path, for every solver.
        cfg.validate_numbers()?;
        let h = cfg.dt / cfg.substeps as f64;
        let (beams, mass) = (&built.beams, &built.mass);
        let car = if with_wheels && !body.wheels.is_empty() {
            let car = vehicle::Car::new(
                body,
                &built.pos,
                &built.vel,
                &built.mass,
                planes.len(),
                cfg.gravity,
            )?;
            let (extra_mass, elements) = car.limit_elements(built.mass.len());
            let limit = step_limit_error(&*solver, h, beams, mass, &extra_mass, &elements);
            if let Some(e) = limit {
                return Err(format!(
                    "{e} (the chassis with its wheels, suspension and tyres)"
                ));
            }
            Some(car)
        } else {
            if let Some(e) = step_limit_error(&*solver, h, beams, mass, &[], &[]) {
                return Err(e);
            }
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

    pub(crate) fn substep(&mut self, substep: u32, h: f64) {
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

