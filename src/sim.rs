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
