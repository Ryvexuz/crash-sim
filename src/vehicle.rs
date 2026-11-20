//! Wheels on spring-damper suspension, with a simple tyre (nb-39, docs/STAGE6.md milestone 3).
//! The model, its units and what it leaves out are in docs/PHYSICS.md, "Wheels, suspension and
//! tyres"; this file is the source of truth.
//!
//! A wheel is a point mass with a spin. Its centre is held on its suspension axis, the line
//! through the centroid of its mount nodes along the chassis' local up axis: along the axis it
//! moves freely against the suspension spring, damper and travel stops; across the axis it is
//! held by a rigid constraint that moves it and its mount nodes together, so the chassis carries
//! the tyre's horizontal forces and the wheel's mass. Every force acts on the mount nodes, so the
//! chassis can still deform, yield and break. `Sim` calls `pre_step` before the solver's step
//! (it adds the reactions on the mount nodes) and `post_step` after it.

use crate::collide::Plane;
use crate::format::Body;
use crate::solver::LimitElement;
use crate::vec3::Vec3;

/// Relaxation length of the tyre, sigma (m). The tyre's contact patch carries a deflection d
/// that grows with the patch's slip velocity and relaxes over this rolling distance; its force is
/// load x stiffness x d / sigma. *Starting value, from memory, unverified:* passenger-tyre
/// relaxation lengths are quoted at about 0.2 to 0.6 m; 0.3 m gives a static patch stiffness of
/// 2943 x 15 / 0.3 = 147 kN/m per tyre of `car_basic.bsn` (longitudinal) at its static load.
pub const TYRE_RELAXATION_LENGTH: f64 = 0.3;
/// Damping time of the patch deflection, tau (s): the patch force is load x stiffness x
/// (d + tau d') / sigma. *Starting value, reasoned:* the car's horizontal mode on its tyres
/// (about 300 kg per corner on 147 kN/m, 22 rad/s) then has a damping ratio of about
/// tau x 22 / 2 = 0.11, and the wheel's spin mode (I / r^2 = 15 kg) about 1; without it the
/// patch spring would ring forever on a car at rest, where the relaxation term is zero.
pub const TYRE_DAMPING_TIME: f64 = 0.01;
/// Stiffness of a suspension travel stop, as a multiple of the spring rate. *Starting value,
/// reasoned:* a bump stop is roughly an order of magnitude stiffer than the spring (from memory,
/// unverified); at 20 x 16655 = 333 kN/m a corner load of one extra g past the stop
/// (2943 N) goes 9 mm into it, and the car stays inside the 1 ms step limit.
pub const STOP_STIFFNESS_FACTOR: f64 = 20.0;
/// The highest forward speed (m/s) at which a negative throttle (reverse) still gives drive
/// torque: reverse torque applies while the car's centre of mass moves forward slower than this,
/// or backward at any speed. Above it a negative throttle gives no drive torque at all (the
/// wheels coast, the brake is the control that slows the car): this model has one engine torque
/// and no gearbox, so a reverse torque at speed would be a hard engine-brake plus a push the
/// wrong way, not a controllable car, and there is no "reverse gear at 50 km/h". *Starting
/// value, reasoned:* 1 m/s (3.6 km/h) is a walking pace, so a car that has just been stopped
/// by the brake can be put into reverse without waiting for exact zero.
pub const REVERSE_MAX_FORWARD_SPEED: f64 = 1.0;

/// The numbers of one wheel, from its `wheel`, `suspension` and `tyre` lines.
#[derive(Clone, Debug, PartialEq)]
pub struct WheelParams {
    pub name: String,
    /// Mount node indices (file order of the nodes).
    pub mount: Vec<usize>,
    pub radius: f64,
    pub mass: f64,
    pub inertia: f64,
    pub braked: bool,
    /// Gets a share of the drive torque (`Car::throttle`).
    pub driven: bool,
    /// Turns with the steering (`Car::steer`).
    pub steerable: bool,
    pub spring: f64,
    pub damper: f64,
    pub rest: f64,
    pub travel_min: f64,
    pub travel_max: f64,
    pub mu: f64,
    pub radial_stiffness: f64,
    pub radial_damping: f64,
    pub long_stiffness: f64,
    pub lat_stiffness: f64,
}

/// A simulated wheel.
#[derive(Clone, Debug, PartialEq)]
pub struct Wheel {
    pub p: WheelParams,
    /// Centre (m) and its velocity (m/s).
    pub pos: Vec3,
    pub vel: Vec3,
    /// Spin about the axle (rad/s); positive rolls the car forward (+x of the chassis).
    pub spin: f64,
    /// Per collider plane (same order as `Sim::planes`): the contact patch deflection (m), in
    /// the plane; zero when the tyre does not touch that plane.
    pub deflection: Vec<Vec3>,
    /// Per collider plane: the normal load (N) of the last force pass.
    pub load: Vec<f64>,
    /// Suspension length (mount centroid to wheel centre along the up axis, m) at the last
    /// force pass.
    pub length: f64,
    force: Vec3,
    /// The axle of the last force pass: the chassis axle turned about the up axis by the steer
    /// angle for a steerable wheel. Spin is about it and the tyre's rolling direction follows it.
    axle: Vec3,
    /// The spin at the end of this substep, found in `pre_step` (tyre torque, then the brake).
    spin_next: f64,
}

impl Wheel {
    /// Total normal load over all planes (N), from the last force pass.
    pub fn total_load(&self) -> f64 {
        self.load.iter().sum()
    }
    /// How much shorter than its rest length the suspension is (m), at the last force pass.
    pub fn compression(&self) -> f64 {
        self.p.rest - self.length
    }
    /// Force of the travel stops along the axis (N, positive pushes the wheel away from the mount).
    fn stop_force(&self, s: f64) -> f64 {
        let k = STOP_STIFFNESS_FACTOR * self.p.spring;
        if s < self.p.travel_min {
            k * (self.p.travel_min - s)
        } else if s > self.p.travel_max {
            -k * (s - self.p.travel_max)
        } else {
            0.0
        }
    }
}

/// The chassis frame of one substep: up (the suspension axis direction, mount to wheel is -up),
/// forward (+x of the chassis) and the axle (up x forward; positive spin about it rolls forward).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Frame {
    pub up: Vec3,
    pub forward: Vec3,
    pub axle: Vec3,
}

/// What `Car::pre_step` needs to know about the substep: the gravity vector (m/s^2) and its
/// length h (s).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Substep {
    pub gravity: Vec3,
    pub h: f64,
}

/// Per-wheel buffers `pre_step` refills every substep, so the hot path never allocates once
/// their capacity is reached. They carry no state between substeps (each is cleared and refilled
/// before it is read), so two cars compare equal whatever their scratch holds.
#[derive(Clone, Debug, Default)]
struct StepScratch {
    brake: Vec<f64>,
    drive: Vec<f64>,
    axles: Vec<Vec3>,
}

impl PartialEq for StepScratch {
    fn eq(&self, _other: &Self) -> bool {
        true
    }
}

/// The wheels of a car and what they share.
#[derive(Clone, Debug, PartialEq)]
pub struct Car {
    pub wheels: Vec<Wheel>,
    /// Throttle -1..=1: every driven wheel gets an equal share of `max_drive_torque x throttle`;
    /// a negative value is reverse torque, applied only under the rule of
    /// `REVERSE_MAX_FORWARD_SPEED`.
    pub throttle: f64,
    /// Brake 0..=1: every braked wheel gets an equal share of `max_brake_torque x brake`.
    pub brake: f64,
    /// Steering -1..=1: every steerable wheel turns `steer x max_steer` about the up axis,
    /// positive toward the car's right (+z of the chassis for the file's +x forward, +y up).
    pub steer: f64,
    /// Total drive torque at full throttle (N m), from `controls`.
    pub max_drive_torque: f64,
    /// Total brake torque at full brake (N m), from `controls`.
    pub max_brake_torque: f64,
    /// Road-wheel angle of a steerable wheel at full steer (rad), from `controls`.
    pub max_steer: f64,
    /// W(t), the work (J) the drive torque has done so far: per wheel and substep the torque
    /// times the substep times the mean spin of the wheel relative to the chassis (the engine
    /// pushes the wheel and the chassis the opposite way). The energy rule of docs/STAGE6.md
    /// (settle rule 4) subtracts it.
    pub drive_work: f64,
    /// The reference load of every tyre (N): the static load per wheel, total weight / wheel
    /// count. The patch spring has the fixed rate C x reference load / sigma (see `tyre_force`).
    pub reference_load: f64,
    /// Frame fit weights: forward = sum_i wx_i p_i, and the image of +z is sum_i wz_i p_i
    /// (the least-squares linear map from the start shape to the current one, applied to x and z).
    wx: Vec<f64>,
    wz: Vec<f64>,
    /// The frame of the last force pass.
    pub frame: Frame,
    /// Set when the chassis frame could not be found (a collapsed chassis); `Sim::check` fails on it.
    pub frame_error: Option<String>,
    /// The torque (N m) the wheel carriers passed to the chassis in the last substep: the
    /// brakes' reaction and the moment of the axis constraint's force about the mount (the
    /// carrier is rigid, so that force reaches the chassis at the wheel centre). `pre_step`
    /// applies it to the chassis in the next substep, as forces in the solver's sum.
    pub carrier_torque: Vec3,
    /// Buffers for `pre_step`; see `StepScratch`.
    scratch: StepScratch,
}

fn finite3(v: Vec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}

/// Inverse of a symmetric 3x3 matrix (rows), or None when it is (nearly) singular relative to its
/// size.
fn inverse3(m: [[f64; 3]; 3]) -> Option<[[f64; 3]; 3]> {
    let c00 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
    let c01 = m[1][2] * m[2][0] - m[1][0] * m[2][2];
    let c02 = m[1][0] * m[2][1] - m[1][1] * m[2][0];
    let det = m[0][0] * c00 + m[0][1] * c01 + m[0][2] * c02;
    let tr = m[0][0] + m[1][1] + m[2][2];
    if !(det.is_finite() && tr > 0.0 && det > 1e-9 * (tr / 3.0).powi(3)) {
        return None;
    }
    let inv = 1.0 / det;
    Some([
        [
            c00 * inv,
            (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv,
            (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv,
        ],
        [
            c01 * inv,
            (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv,
            (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv,
        ],
        [
            c02 * inv,
            (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv,
            (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv,
        ],
    ])
}

/// Index of every mount node of `w`, or an error naming the missing one.
fn mount_indices(body: &Body, w: &crate::format::Wheel) -> Result<Vec<usize>, String> {
    if w.mount.is_empty() {
        return Err(format!("wheel `{}` has no mount node", w.name));
    }
    let mut out = Vec::new();
    for id in &w.mount {
        let i = body
            .node_index(*id)
            .ok_or_else(|| format!("wheel `{}` is mounted on missing node {id}", w.name))?;
        if out.contains(&i) {
            return Err(format!("wheel `{}`: mount lists node {id} twice", w.name));
        }
        out.push(i);
    }
    Ok(out)
}

/// The numbers of every wheel, checked again here (a `Body` need not come from `format::parse`).
fn wheel_params(body: &Body) -> Result<Vec<WheelParams>, String> {
    let mut out = Vec::new();
    for w in &body.wheels {
        let name = &w.name;
        if out.iter().any(|p: &WheelParams| &p.name == name) {
            return Err(format!("duplicate wheel `{name}`"));
        }
        let s = body
            .suspension(name)
            .ok_or_else(|| format!("wheel `{name}` has no suspension"))?;
        let t = body
            .tyre(name)
            .ok_or_else(|| format!("wheel `{name}` has no tyre"))?;
        let pos = |x: f64| x.is_finite() && x > 0.0;
        let nonneg = |x: f64| x.is_finite() && x >= 0.0;
        let checks = [
            (pos(w.radius), "radius must be > 0"),
            (pos(w.mass), "mass must be > 0"),
            (pos(w.inertia), "inertia must be > 0"),
            (pos(s.spring), "spring rate must be > 0"),
            (nonneg(s.damper), "damper rate must be >= 0"),
            (
                nonneg(s.travel_min)
                    && s.travel_max.is_finite()
                    && s.travel_min < s.rest
                    && s.rest < s.travel_max,
                "suspension needs 0 <= travel min < rest < travel max",
            ),
            (pos(t.mu), "mu must be > 0"),
            (pos(t.radial_stiffness), "radial_stiffness must be > 0"),
            (nonneg(t.radial_damping), "radial_damping must be >= 0"),
            (pos(t.long_stiffness), "long_stiffness must be > 0"),
            (pos(t.lat_stiffness), "lat_stiffness must be > 0"),
        ];
        for (ok, what) in checks {
            if !ok {
                return Err(format!("wheel `{name}`: {what} (all finite)"));
            }
        }
        out.push(WheelParams {
            name: name.clone(),
            mount: mount_indices(body, w)?,
            radius: w.radius,
            mass: w.mass,
            inertia: w.inertia,
            braked: w.braked,
            driven: w.driven,
            steerable: w.steerable,
            spring: s.spring,
            damper: s.damper,
            rest: s.rest,
            travel_min: s.travel_min,
            travel_max: s.travel_max,
            mu: t.mu,
            radial_stiffness: t.radial_stiffness,
            radial_damping: t.radial_damping,
            long_stiffness: t.long_stiffness,
            lat_stiffness: t.lat_stiffness,
        });
    }
    Ok(out)
}

/// The slope angle theta (rad) of `--slope-grip-frac f` (docs/STAGE6.md check 3.4):
/// tan theta = f x mu, with mu the body's tyre grip coefficient. Refused when the body has no
/// tyres, its tyres' mu differ, or f is not a finite number >= 0.
pub fn slope_angle(body: &Body, grip_frac: f64) -> Result<f64, String> {
    if !(grip_frac.is_finite() && grip_frac >= 0.0) {
        return Err(format!(
            "slope grip fraction {grip_frac} must be a finite number >= 0"
        ));
    }
    let first = body.tyres.first().ok_or_else(|| {
        "the body has no tyres, so a slope relative to tyre grip has no meaning".to_string()
    })?;
    if let Some(t) = body.tyres.iter().find(|t| t.mu != first.mu) {
        return Err(format!(
            "the tyres' mu differ ({} on `{}`, {} on `{}`): the slope needs one grip coefficient",
            first.mu, first.wheel, t.mu, t.wheel
        ));
    }
    Ok((grip_frac * first.mu).atan())
}

impl Car {
    /// The car of `body` (which must have wheels and `controls`), for chassis nodes at `pos` with
    /// velocities `vel` and masses `mass`, `planes` collider planes and gravity `g` (m/s^2).
    /// Each wheel starts at `Body::wheel_start_center`, with the mean velocity of its mount
    /// nodes and the spin that rolls it without slip along the chassis' forward axis.
    pub fn new(
        body: &Body,
        pos: &[Vec3],
        vel: &[Vec3],
        mass: &[f64],
        n_planes: usize,
        g: f64,
    ) -> Result<Car, String> {
        let params = wheel_params(body)?;
        if params.is_empty() {
            return Err("the body has no wheels".into());
        }
        let controls = body
            .controls
            .as_ref()
            .ok_or_else(|| "the body has wheels but no `controls`".to_string())?;
        if !(controls.max_brake_torque.is_finite() && controls.max_brake_torque > 0.0) {
            return Err(format!(
                "max_brake_torque {} must be a finite number > 0",
                controls.max_brake_torque
            ));
        }
        // The frame fit: r0 = start position minus start centroid, M0 = sum r0 r0^T.
        let n = pos.len() as f64;
        let mut c0 = Vec3::ZERO;
        for p in pos {
            c0 += *p;
        }
        let c0 = c0 * (1.0 / n);
        let mut m0 = [[0.0_f64; 3]; 3];
        for p in pos {
            let r = *p - c0;
            let a = [r.x, r.y, r.z];
            for (i, row) in m0.iter_mut().enumerate() {
                for (j, x) in row.iter_mut().enumerate() {
                    *x += a[i] * a[j];
                }
            }
        }
        let b = inverse3(m0).ok_or_else(|| "the chassis nodes are (nearly) in one plane or line, so its up axis cannot be found".to_string())?;
        // B e_x and B e_z (columns 0 and 2 of the symmetric inverse).
        let bx = Vec3::new(b[0][0], b[1][0], b[2][0]);
        let bz = Vec3::new(b[0][2], b[1][2], b[2][2]);
        let wx: Vec<f64> = pos.iter().map(|p| (*p - c0).dot(bx)).collect();
        let wz: Vec<f64> = pos.iter().map(|p| (*p - c0).dot(bz)).collect();
        let total_mass: f64 = mass.iter().sum::<f64>() + params.iter().map(|w| w.mass).sum::<f64>();
        let reference_load = total_mass * g / params.len() as f64;
        if !(controls.max_drive_torque.is_finite() && controls.max_drive_torque > 0.0) {
            return Err(format!(
                "max_drive_torque {} must be a finite number > 0",
                controls.max_drive_torque
            ));
        }
        if !(controls.max_steer_deg.is_finite()
            && controls.max_steer_deg > 0.0
            && controls.max_steer_deg < 90.0)
        {
            return Err(format!(
                "max_steer_deg {} must be a finite number between 0 and 90",
                controls.max_steer_deg
            ));
        }
        let mut car = Car {
            wheels: Vec::new(),
            throttle: 0.0,
            brake: 0.0,
            steer: 0.0,
            max_drive_torque: controls.max_drive_torque,
            max_brake_torque: controls.max_brake_torque,
            max_steer: controls.max_steer_deg.to_radians(),
            drive_work: 0.0,
            reference_load,
            wx,
            wz,
            frame: Frame {
                up: Vec3::new(0.0, 1.0, 0.0),
                forward: Vec3::new(1.0, 0.0, 0.0),
                axle: Vec3::new(0.0, 0.0, -1.0),
            },
            frame_error: None,
            carrier_torque: Vec3::ZERO,
            scratch: StepScratch::default(),
        };
        car.frame = car
            .fit_frame(pos)
            .map_err(|e| format!("at the start: {e}"))?;
        for p in params {
            let (c, vc) = centroid(&p.mount, pos, vel);
            let start = Vec3::new(c.x, c.y - p.rest, c.z);
            let spin = vc.dot(car.frame.forward) / p.radius;
            car.wheels.push(Wheel {
                pos: start,
                vel: vc,
                spin,
                deflection: vec![Vec3::ZERO; n_planes],
                load: vec![0.0; n_planes],
                length: p.rest,
                force: Vec3::ZERO,
                axle: car.frame.axle,
                spin_next: spin,
                p,
            });
        }
        Ok(car)
    }

    /// The chassis frame from the node positions: the least-squares linear map A from the start
    /// shape to the current one gives ax = A e_x and az = A e_z; up = unit(az x ax),
    /// forward = unit(ax minus its up part), axle = up x forward. For a rigid motion A is the
    /// rotation, so up is the rotated +y exactly.
    pub fn fit_frame(&self, pos: &[Vec3]) -> Result<Frame, String> {
        let mut ax = Vec3::ZERO;
        let mut az = Vec3::ZERO;
        for (i, p) in pos.iter().enumerate() {
            ax += *p * self.wx[i];
            az += *p * self.wz[i];
        }
        let up = az.cross(ax).normalized().ok_or_else(|| {
            "the chassis frame collapsed (its up axis has no direction)".to_string()
        })?;
        let forward = (ax - up * ax.dot(up)).normalized().ok_or_else(|| {
            "the chassis frame collapsed (its forward axis has no direction)".to_string()
        })?;
        Ok(Frame {
            up,
            forward,
            axle: up.cross(forward),
        })
    }

    /// The step-limit elements and extra masses of the wheels, for `solver::step_error_with`:
    /// wheel j has degree of freedom `n_nodes + 2j` (its centre, mass m) and `n_nodes + 2j + 1`
    /// (its spin as rim travel, mass I / r^2). Elements: the suspension at its stiffest (spring
    /// plus a travel stop) with its damper, between the wheel and its mount centroid; the tyre's
    /// radial spring and damper to the ground; and the tyre patch at its stiffest (the larger of
    /// the two slip stiffnesses at the reference load) with its damping, between the wheel centre and
    /// its rim.
    pub fn limit_elements(&self, n_nodes: usize) -> (Vec<f64>, Vec<LimitElement>) {
        let mut masses = Vec::new();
        let mut el = Vec::new();
        for (j, w) in self.wheels.iter().enumerate() {
            let p = &w.p;
            let (iw, is) = (n_nodes + 2 * j, n_nodes + 2 * j + 1);
            masses.push(p.mass);
            masses.push(p.inertia / (p.radius * p.radius));
            let share = 1.0 / p.mount.len() as f64;
            let mut ends: Vec<(usize, f64)> = p.mount.iter().map(|&i| (i, share)).collect();
            ends.push((iw, -1.0));
            el.push(LimitElement {
                ends,
                k: p.spring * (1.0 + STOP_STIFFNESS_FACTOR),
                c: p.damper,
            });
            el.push(LimitElement {
                ends: vec![(iw, 1.0)],
                k: p.radial_stiffness,
                c: p.radial_damping,
            });
            let kt = p.long_stiffness.max(p.lat_stiffness) * self.reference_load
                / TYRE_RELAXATION_LENGTH;
            el.push(LimitElement {
                ends: vec![(iw, 1.0), (is, -1.0)],
                k: kt,
                c: kt * TYRE_DAMPING_TIME,
            });
        }
        (masses, el)
    }

    /// Set the controls (throttle -1..=1, brake 0..=1, steer -1..=1). A value outside its range,
    /// or not finite, is refused and nothing changes; so is throttle on a car with no driven
    /// wheel, brake on one with no braked wheel and steer on one with no steerable wheel (the
    /// input would silently do nothing).
    pub fn set_controls(&mut self, throttle: f64, brake: f64, steer: f64) -> Result<(), String> {
        if !(throttle.is_finite() && (-1.0..=1.0).contains(&throttle)) {
            return Err(format!("throttle {throttle} must be a number from -1 to 1"));
        }
        if !(brake.is_finite() && (0.0..=1.0).contains(&brake)) {
            return Err(format!("brake {brake} must be a number from 0 to 1"));
        }
        if !(steer.is_finite() && (-1.0..=1.0).contains(&steer)) {
            return Err(format!("steer {steer} must be a number from -1 to 1"));
        }
        if throttle != 0.0 && !self.wheels.iter().any(|w| w.p.driven) {
            return Err("throttle needs a driven wheel, and the car has none".into());
        }
        if brake > 0.0 && !self.wheels.iter().any(|w| w.p.braked) {
            return Err("brake needs a braked wheel, and the car has none".into());
        }
        if steer != 0.0 && !self.wheels.iter().any(|w| w.p.steerable) {
            return Err("steer needs a steerable wheel, and the car has none".into());
        }
        self.throttle = throttle;
        self.brake = brake;
        self.steer = steer;
        Ok(())
    }

    /// The brake torque on wheel `w` now (N m): an equal share of `max_brake_torque x brake`
    /// for each braked wheel.
    fn brake_torque(&self, w: &Wheel) -> f64 {
        let n = self.wheels.iter().filter(|w| w.p.braked).count();
        if w.p.braked && n > 0 {
            self.max_brake_torque * self.brake / n as f64
        } else {
            0.0
        }
    }

    /// The forward speed (m/s) of the chassis' centre of mass along the frame of the last force
    /// pass; negative when the car moves backwards.
    fn forward_speed(&self, vel: &[Vec3], mass: &[f64]) -> f64 {
        let mut p = Vec3::ZERO;
        let mut m = 0.0;
        for (v, &mi) in vel.iter().zip(mass) {
            p += *v * mi;
            m += mi;
        }
        if m > 0.0 {
            (p * (1.0 / m)).dot(self.frame.forward)
        } else {
            0.0
        }
    }

    /// The drive torque on wheel `w` now (N m): an equal share of `max_drive_torque x throttle`
    /// for each driven wheel. A negative throttle (reverse) gives its torque only while
    /// `forward_speed` is at most `REVERSE_MAX_FORWARD_SPEED`; above that it gives none.
    fn drive_torque(&self, w: &Wheel, forward_speed: f64) -> f64 {
        let n = self.wheels.iter().filter(|w| w.p.driven).count();
        if self.throttle < 0.0 && forward_speed > REVERSE_MAX_FORWARD_SPEED {
            return 0.0;
        }
        if w.p.driven && n > 0 {
            self.max_drive_torque * self.throttle / n as f64
        } else {
            0.0
        }
    }

    /// The axle of wheel `w` for frame `fr`: the chassis axle, turned about the up axis by
    /// `steer x max_steer` (positive toward +z of the chassis, the car's right) when the wheel
    /// is steerable. The wheel's rolling direction is up x axle.
    fn wheel_axle(&self, w: &Wheel, fr: &Frame) -> Vec3 {
        if !w.p.steerable || self.steer == 0.0 {
            return fr.axle;
        }
        let (s, c) = (self.steer * self.max_steer).sin_cos();
        let right = fr.forward.cross(fr.up);
        let rolling = fr.forward * c + right * s;
        fr.up.cross(rolling)
    }

    /// Before the solver's step of length `h`: the frame, then every wheel's suspension, tyre
    /// and brake forces from the current state, all applied within this substep:
    /// - along the axis, the wheel keeps its force (gravity, suspension, tyre) and the mount
    ///   nodes get the suspension's reaction, an equal share on each;
    /// - across the axis the wheel is carried by its rigid carrier, so that part of the wheel's
    ///   force goes to the mount nodes (equal shares) together with its moment about the mount
    ///   centroid (the force acts at the wheel centre, not at the mount);
    /// - the spin gets the tyre's torque and then the brake, which can stop it but never reverse
    ///   it within a substep; the brake caliper is on the chassis, so the chassis gets the brake
    ///   torque back about the axle.
    ///
    /// The moments, and the last substep's `carrier_torque` (the moment of the axis constraint's
    /// residual impulse), go to `extra` as a rigid-body share over all chassis nodes.
    pub fn pre_step(
        &mut self,
        pos: &[Vec3],
        vel: &[Vec3],
        mass: &[f64],
        planes: &[Plane],
        step: Substep,
        extra: &mut [Vec3],
    ) {
        let Substep { gravity, h } = step;
        let mut chassis_torque = self.carrier_torque;
        // Take the buffers out of `self` (a move, no allocation) so the loops below can borrow
        // `self` as before; they go back at the end. Cleared and refilled, they keep their capacity.
        let mut scratch = std::mem::take(&mut self.scratch);
        scratch.brake.clear();
        scratch.drive.clear();
        scratch.axles.clear();
        // Only a reverse throttle needs the speed; for throttle >= 0 it is never read.
        let forward_speed = if self.throttle < 0.0 {
            self.forward_speed(vel, mass)
        } else {
            0.0
        };
        for w in &self.wheels {
            scratch.brake.push(self.brake_torque(w));
            scratch.drive.push(self.drive_torque(w, forward_speed));
        }
        let old = self.frame;
        match self.fit_frame(pos) {
            Ok(f) => self.frame = f,
            Err(e) => {
                if self.frame_error.is_none() {
                    self.frame_error = Some(e);
                }
            }
        }
        let fr = self.frame;
        for w in &self.wheels {
            scratch.axles.push(self.wheel_axle(w, &fr));
        }
        let mut drive_work = 0.0;
        // The chassis' rotation rate about the axle (rad/s), from how far its forward axis turned
        // since the last substep: d(forward)/dt = omega x forward, so omega . axle =
        // -(d forward/dt) . up. The brake acts on the wheel's spin relative to this.
        let chassis_spin = -((fr.forward - old.forward) * (1.0 / h)).dot(fr.up);
        let reference_load = self.reference_load;
        let per_wheel = scratch.brake.iter().zip(&scratch.drive).zip(&scratch.axles);
        for (w, ((&tb, &td), &wa)) in self.wheels.iter_mut().zip(per_wheel) {
            w.axle = wa;
            let (c, vc) = centroid(&w.p.mount, pos, vel);
            let s = (c - w.pos).dot(fr.up);
            let s_dot = (vc - w.vel).dot(fr.up);
            w.length = s;
            // Positive pushes the wheel away from the mount (down the axis) and the mount up.
            let f_s = w.p.spring * (w.p.rest - s) - w.p.damper * s_dot + w.stop_force(s);
            let share = 1.0 / w.p.mount.len() as f64;
            for &i in &w.p.mount {
                extra[i] += fr.up * (f_s * share);
            }
            let mut force = gravity * w.p.mass + fr.up * (-f_s);
            let mut torque = 0.0;
            for (k, pl) in planes.iter().enumerate() {
                let (fz, ft) = tyre_force(w, k, pl, wa, reference_load);
                w.load[k] = fz;
                if fz > 0.0 {
                    let rc = pl.normal * (-w.p.radius);
                    force += pl.normal * fz + ft;
                    torque += rc.cross(ft).dot(wa);
                }
            }
            // Across the axis the carrier takes the force: onto the mount nodes, with its moment.
            let across = force - fr.up * force.dot(fr.up);
            for &i in &w.p.mount {
                extra[i] += across * share;
            }
            chassis_torque += (w.pos - c).cross(across);
            w.force = force - across;
            // Spin: tyre torque and drive torque first, then the brake, which at most stops the
            // wheel turning relative to the chassis (the caliper turns with the chassis). The
            // chassis' rate about this wheel's axle is its rate about the chassis axle, scaled
            // by how far the steering turned the axle.
            let cs = chassis_spin * wa.dot(fr.axle);
            let free = w.spin + (torque + td) * (h / w.p.inertia);
            let stop = tb * h / w.p.inertia;
            let rel = free - cs;
            w.spin_next = cs
                + if rel.abs() <= stop {
                    0.0
                } else {
                    rel - rel.signum() * stop
                };
            // The brake's reaction, and the engine's: the engine is on the chassis, so the
            // chassis gets the drive torque back the other way about the axle.
            chassis_torque += wa * (-w.p.inertia * (w.spin_next - free) / h - td);
            drive_work += td * h * (0.5 * (w.spin + w.spin_next) - cs);
        }
        self.drive_work += drive_work;
        self.scratch = scratch;
        add_torque(pos, mass, chassis_torque, extra);
    }

    /// Capacities of `pre_step`'s three reuse buffers (brake, drive, axles): constant once the
    /// first substep has run. Exists so a test can show the buffers are reused, not reallocated.
    pub fn scratch_capacities(&self) -> [usize; 3] {
        [
            self.scratch.brake.capacity(),
            self.scratch.drive.capacity(),
            self.scratch.axles.capacity(),
        ]
    }

    /// Addresses of `pre_step`'s three reuse buffers, for the same test: they do not move while
    /// the buffers are reused.
    pub fn scratch_addresses(&self) -> [usize; 3] {
        [
            self.scratch.brake.as_ptr() as usize,
            self.scratch.drive.as_ptr() as usize,
            self.scratch.axles.as_ptr() as usize,
        ]
    }

    /// After the solver's step: each wheel's velocity and centre (symplectic Euler, velocity
    /// first), its spin with the brake as a torque that can stop it but never reverse it within
    /// a substep, the constraint that puts the wheel back on its axis, and the patch deflections.
    pub fn post_step(
        &mut self,
        pos: &mut [Vec3],
        vel: &mut [Vec3],
        mass: &[f64],
        planes: &[Plane],
        h: f64,
    ) {
        // The axis after the solver's step, and how fast it turned: a point at distance s down
        // the axis moves with the mount centroid's velocity minus s du/dt.
        let up_old = self.frame.up;
        let up = match self.fit_frame(pos) {
            Ok(f) => f.up,
            Err(e) => {
                if self.frame_error.is_none() {
                    self.frame_error = Some(e);
                }
                up_old
            }
        };
        let up_rate = (up - up_old) * (1.0 / h);
        let reference_load = self.reference_load;
        // Angular impulse (N m s) of the axis constraint's residual, for the next substep.
        let mut chassis_l = Vec3::ZERO;
        for w in self.wheels.iter_mut() {
            w.vel += w.force * (h / w.p.mass);
            w.pos += w.vel * h;
            w.spin = w.spin_next;

            // Across the axis the wheel moves with its mount: remove the wheel's offset from the
            // axis and its velocity across the axis relative to the axis point it sits at,
            // sharing each correction by mass so momentum and the centre of mass are unchanged.
            // The forces across the axis already went to the chassis in `pre_step`, so what is
            // left here is the wheel's own inertia (and rounding).
            let (c, vc) = centroid(&w.p.mount, pos, vel);
            let mm: f64 = w.p.mount.iter().map(|&i| mass[i]).sum();
            let (fw, fm) = (mm / (w.p.mass + mm), w.p.mass / (w.p.mass + mm));
            let s = (c - w.pos).dot(up);
            let e = w.pos - (c - up * s);
            let rel = w.vel - (vc - up_rate * s);
            let rel = rel - up * rel.dot(up);
            w.pos += e * (-fw);
            w.vel += rel * (-fw);
            for &i in &w.p.mount {
                pos[i] += e * fm;
                vel[i] += rel * fm;
            }
            // The carrier is rigid: the impulse it passes reaches the chassis at the wheel centre,
            // not at the mount, so the chassis also gets its moment about the mount centroid.
            let impulse = rel * (fm * mm);
            chassis_l += (w.pos - c).cross(impulse);

            for (k, pl) in planes.iter().enumerate() {
                let fz = w.load[k];
                if fz <= 0.0 {
                    w.deflection[k] = Vec3::ZERO;
                    continue;
                }
                let n = pl.normal;
                let (vs, speed) = patch_slip(w.vel, w.spin, w.p.radius, n, w.axle);
                let mut d =
                    (w.deflection[k] + vs * h) * (1.0 / (1.0 + h * speed / TYRE_RELAXATION_LENGTH));
                d = d - n * d.dot(n);
                // Sliding: the elastic force may not exceed the friction circle.
                let (ft, lt) = patch_axes(n, w.axle);
                let kz = reference_load / TYRE_RELAXATION_LENGTH;
                let fx = kz * w.p.long_stiffness * d.dot(ft);
                let fy = kz * w.p.lat_stiffness * d.dot(lt);
                let mag = (fx * fx + fy * fy).sqrt();
                let cap = w.p.mu * fz;
                if mag > cap {
                    d = d * (cap / mag);
                }
                w.deflection[k] = d;
            }
        }
        self.carrier_torque = chassis_l * (1.0 / h);
    }

    /// Fastest wheel point: the larger of a wheel's centre speed and its rim speed |spin| r.
    pub fn max_speed(&self) -> f64 {
        self.wheels
            .iter()
            .map(|w| w.vel.length().max(w.spin.abs() * w.p.radius))
            .fold(0.0, f64::max)
    }

    /// Error text when any wheel state is not finite or the frame was lost, else None.
    pub fn check(&self) -> Option<String> {
        if let Some(e) = &self.frame_error {
            return Some(e.clone());
        }
        if !(self.throttle.is_finite()
            && (-1.0..=1.0).contains(&self.throttle)
            && self.brake.is_finite()
            && (0.0..=1.0).contains(&self.brake)
            && self.steer.is_finite()
            && (-1.0..=1.0).contains(&self.steer)
            && self.drive_work.is_finite())
        {
            return Some(format!(
                "car controls out of range or not finite: throttle {}, brake {}, steer {}, drive work {}",
                self.throttle, self.brake, self.steer, self.drive_work
            ));
        }
        for w in &self.wheels {
            if !(finite3(w.pos)
                && finite3(w.vel)
                && w.spin.is_finite()
                && w.deflection.iter().all(|d| finite3(*d)))
            {
                return Some(format!("wheel `{}` state is not finite", w.p.name));
            }
        }
        None
    }

    /// Energy stored in and moving with the wheels (J), with gravity `gravity` (m/s^2 vector;
    /// potential zero at the origin): (1/2) m |v|^2 + (1/2) I spin^2 - m gravity . pos per wheel,
    /// the suspension spring (1/2) k (rest - s)^2 and a stop's (1/2) k_stop overrun^2 at the
    /// last force pass's length, the tyre radial spring (1/2) k_r pen^2 against every plane
    /// it presses into, and the patch spring (1/2) (load/sigma) (C_x d_x^2 + C_y d_y^2).
    pub fn energy(&self, planes: &[Plane], gravity: Vec3) -> f64 {
        let mut e = 0.0;
        for w in &self.wheels {
            let p = &w.p;
            e += 0.5 * p.mass * w.vel.dot(w.vel) + 0.5 * p.inertia * w.spin * w.spin
                - p.mass * gravity.dot(w.pos);
            let ds = p.rest - w.length;
            e += 0.5 * p.spring * ds * ds;
            let over = if w.length < p.travel_min {
                p.travel_min - w.length
            } else if w.length > p.travel_max {
                w.length - p.travel_max
            } else {
                0.0
            };
            e += 0.5 * STOP_STIFFNESS_FACTOR * p.spring * over * over;
            for (k, pl) in planes.iter().enumerate() {
                let pen = p.radius - pl.distance(w.pos);
                if pen > 0.0 {
                    e += 0.5 * p.radial_stiffness * pen * pen;
                }
                if w.load[k] > 0.0 {
                    let (ft, lt) = patch_axes(pl.normal, w.axle);
                    let kz = self.reference_load / TYRE_RELAXATION_LENGTH;
                    let (dx, dy) = (w.deflection[k].dot(ft), w.deflection[k].dot(lt));
                    e += 0.5 * kz * (p.long_stiffness * dx * dx + p.lat_stiffness * dy * dy);
                }
            }
        }
        e
    }
}

/// Add the torque `t` (N m) to `extra` as a rigid-body share over the chassis nodes: with J the
/// chassis inertia tensor about its centre of mass, node i gets m_i (J^-1 t) x r_i, which sums to
/// no force and exactly the torque t. Nothing happens when t is zero; a chassis whose J cannot be
/// inverted (all nodes on one line) gets nothing, and `Car::fit_frame` has already failed on
/// such a chassis.
fn add_torque(pos: &[Vec3], mass: &[f64], t: Vec3, extra: &mut [Vec3]) {
    if t == Vec3::ZERO {
        return;
    }
    let l = t;
    let m: f64 = mass.iter().sum();
    let mut com = Vec3::ZERO;
    for (p, mi) in pos.iter().zip(mass) {
        com += *p * *mi;
    }
    let com = com * (1.0 / m);
    let mut j = [[0.0_f64; 3]; 3];
    for (p, mi) in pos.iter().zip(mass) {
        let r = *p - com;
        let a = [r.x, r.y, r.z];
        let rr = r.dot(r);
        for (i, row) in j.iter_mut().enumerate() {
            for (k, x) in row.iter_mut().enumerate() {
                *x += mi * (if i == k { rr } else { 0.0 } - a[i] * a[k]);
            }
        }
    }
    let Some(inv) = inverse3(j) else { return };
    let w = Vec3::new(
        inv[0][0] * l.x + inv[0][1] * l.y + inv[0][2] * l.z,
        inv[1][0] * l.x + inv[1][1] * l.y + inv[1][2] * l.z,
        inv[2][0] * l.x + inv[2][1] * l.y + inv[2][2] * l.z,
    );
    for ((p, mi), e) in pos.iter().zip(mass).zip(extra.iter_mut()) {
        *e += w.cross(*p - com) * *mi;
    }
}

/// Mean position and mean velocity of the nodes `ix`.
fn centroid(ix: &[usize], pos: &[Vec3], vel: &[Vec3]) -> (Vec3, Vec3) {
    let mut c = Vec3::ZERO;
    let mut v = Vec3::ZERO;
    for &i in ix {
        c += pos[i];
        v += vel[i];
    }
    let s = 1.0 / ix.len() as f64;
    (c * s, v * s)
}

/// The tyre's rolling and lateral directions in a plane with normal `n`: rolling = unit(axle x n)
/// (forward when the plane is the ground under an upright car), lateral = n x rolling. When the
/// axle is along the normal (the tyre's side face on a wall) any direction in the plane is used.
fn patch_axes(n: Vec3, axle: Vec3) -> (Vec3, Vec3) {
    let ft = axle.cross(n).normalized().unwrap_or_else(|| {
        let t = if n.x.abs() < 0.9 {
            Vec3::new(1.0, 0.0, 0.0)
        } else {
            Vec3::new(0.0, 1.0, 0.0)
        };
        (t - n * t.dot(n))
            .normalized()
            .expect("a unit normal has a perpendicular")
    });
    (ft, n.cross(ft))
}

/// Slip velocity of the contact patch in the plane (m/s), and the tangential speed of the wheel
/// centre over the plane (the rolling speed that relaxes the deflection).
fn patch_slip(vel: Vec3, spin: f64, r: f64, n: Vec3, axle: Vec3) -> (Vec3, f64) {
    let vp = vel + axle.cross(n * (-r)) * spin;
    let vs = vp - n * vp.dot(n);
    let vt = vel - n * vel.dot(n);
    (vs, vt.length())
}

/// Normal load (N) and tangential force (N, in the plane) of wheel `w`'s tyre on plane `pl`
/// (index `k` in the sim's planes), from the state before the step.
fn tyre_force(w: &Wheel, k: usize, pl: &Plane, axle: Vec3, reference_load: f64) -> (f64, Vec3) {
    let (p, vel, n, d) = (&w.p, w.vel, pl.normal, w.deflection[k]);
    let pen = p.radius - pl.distance(w.pos);
    if pen <= 0.0 {
        return (0.0, Vec3::ZERO);
    }
    let fz = (p.radial_stiffness * pen - p.radial_damping * vel.dot(n)).max(0.0);
    if fz <= 0.0 {
        return (0.0, Vec3::ZERO);
    }
    let (vs, speed) = patch_slip(vel, w.spin, p.radius, n, axle);
    let d_rate = vs - d * (speed / TYRE_RELAXATION_LENGTH);
    let (ft, lt) = patch_axes(n, axle);
    let kz = reference_load / TYRE_RELAXATION_LENGTH;
    let along = |a: Vec3| d.dot(a) + TYRE_DAMPING_TIME * d_rate.dot(a);
    let mut f =
        -(ft * (kz * p.long_stiffness * along(ft)) + lt * (kz * p.lat_stiffness * along(lt)));
    let mag = f.length();
    let cap = p.mu * fz;
    if mag > cap {
        f = f * (cap / mag);
    }
    (fz, f)
}
