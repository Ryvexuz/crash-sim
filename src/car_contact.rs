//! Ordered, transactional node-versus-live-beam contact between different cars.
//!
//! Call after each single-car physics substep, with positions saved before that
//! substep. Moving contact follows impulses; see docs/PHYSICS.md for the rule.

use std::collections::BTreeSet;
use std::fmt;

use crate::broadphase::candidate_pairs;
use crate::vec3::Vec3;
use crate::Beam;

/// Explicit caller settings, in metres, seconds and dimensionless restitution.
/// There are no scene defaults. Construction enforces the discrete rule's band.
#[derive(Clone, Copy, Debug)]
pub struct ContactSettings {
    skin: f64,
    restitution: f64,
    timestep: f64,
    max_motion: f64,
    max_penetration: f64,
}

impl ContactSettings {
    pub fn new(
        skin: f64,
        restitution: f64,
        timestep: f64,
        max_motion: f64,
        max_penetration: f64,
    ) -> Result<Self, ContactError> {
        // Numerical (not scene) bounds: squared distances and reciprocals must
        // stay normal, with headroom for differences, dot products and padding.
        let lo = f64::MIN_POSITIVE.sqrt() * 64.0;
        let hi = f64::MAX.sqrt() / 64.0;
        if !(skin.is_finite() && (lo..=hi).contains(&skin)) {
            return Err(error(None, Feature::Settings, Failure::InvalidSkin));
        }
        if !(restitution.is_finite() && (0.0..=1.0).contains(&restitution)) {
            return Err(error(None, Feature::Settings, Failure::InvalidRestitution));
        }
        if !(timestep.is_finite() && timestep > 0.0 && (1.0 / timestep).is_finite()) {
            return Err(error(None, Feature::Settings, Failure::InvalidTimestep));
        }
        if !(max_motion.is_finite()
            && max_motion > 0.0
            && max_motion <= skin / 8.0
            && max_penetration.is_finite()
            && max_penetration > 0.0
            && max_penetration <= skin / 2.0)
        {
            return Err(error(None, Feature::Settings, Failure::InvalidBounds));
        }
        Ok(Self {
            skin,
            restitution,
            timestep,
            max_motion,
            max_penetration,
        })
    }
}

/// Slice order defines car, node and beam identities; broken beams keep their
/// indices. `previous_pos` is the snapshot immediately before the substep.
pub struct ContactCar<'a> {
    pub previous_pos: &'a [Vec3],
    pub pos: &'a [Vec3],
    pub vel: &'a mut [Vec3],
    pub mass: &'a [f64],
    pub beams: &'a [Beam],
    pub broken: &'a [bool],
}

/// Processing order is lexicographic in these fields, including both directions
/// of a car pair. Endpoint candidates never constitute contacts by themselves.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct ContactKey {
    pub source_car: usize,
    pub node: usize,
    pub target_car: usize,
    pub beam: usize,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Contact {
    pub key: ContactKey,
    /// Closest point = (1 - fraction) * a + fraction * b.
    pub fraction: f64,
    /// Unit vector from the beam's closest point to the source node.
    pub normal: Vec3,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ContactReport {
    pub candidates: usize,
    pub contacts: usize,
    pub impulses: usize,
    /// Sum of applied normal impulse magnitudes (kg m/s).
    pub normal_impulse: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Feature {
    Settings,
    Layout,
    Node(usize),
    Beam(usize),
    Contact(ContactKey),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Failure {
    InvalidSkin,
    InvalidRestitution,
    InvalidTimestep,
    InvalidBounds,
    LengthMismatch,
    InvalidIndex,
    InvalidMass,
    NonFinite,
    NumericalRange,
    MotionBound,
    DegenerateBeam,
    UndefinedNormal,
    PenetrationBound,
    /// A swept capsule may have been entered and exited between samples.
    /// Reduce the substep; no successful zero-contact result is returned.
    UnsupportedSweep,
    EnergyIncrease,
}

/// A refusal always names its car/feature (settings have no car). No state is
/// written on any refusal, including a failure after an earlier trial impulse.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ContactError {
    pub car: Option<usize>,
    pub feature: Feature,
    pub failure: Failure,
}

impl fmt::Display for ContactError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let reason = match self.failure {
            Failure::InvalidSkin => "skin must be finite and inside the documented numerical band",
            Failure::InvalidRestitution => "restitution must be finite in [0,1]",
            Failure::InvalidTimestep => "timestep must be positive with a finite reciprocal",
            Failure::InvalidBounds => "require 0 < motion <= skin/8 and 0 < penetration <= skin/2",
            Failure::LengthMismatch => "node arrays or beam/broken arrays have unequal lengths",
            Failure::InvalidIndex => "beam endpoint is outside the node arrays",
            Failure::InvalidMass => "mass must be positive with a normal finite reciprocal",
            Failure::NonFinite => "replace non-finite input or refuse the simulation",
            Failure::NumericalRange => "arithmetic is outside the documented representable band",
            Failure::MotionBound => "reduce timestep: displacement or speed*timestep exceeds motion",
            Failure::DegenerateBeam => "live beam squared length must be positive and normal",
            Failure::UndefinedNormal => "node lies on a beam axis; no normal can be chosen",
            Failure::PenetrationBound => "restore the pre-step state and reduce timestep or overlap",
            Failure::UnsupportedSweep => "reduce timestep or use continuous contact for this sweep",
            Failure::EnergyIncrease => "trial impulses increased energy; no response was committed",
        };
        write!(
            f,
            "car {:?}, {:?}: {:?}: {reason}",
            self.car, self.feature, self.failure
        )
    }
}

impl std::error::Error for ContactError {}

fn error(car: Option<usize>, feature: Feature, failure: Failure) -> ContactError {
    ContactError {
        car,
        feature,
        failure,
    }
}

fn finite(v: Vec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}

fn norm(v: Vec3) -> f64 {
    v.x.hypot(v.y).hypot(v.z)
}

fn closest(p: Vec3, a: Vec3, b: Vec3) -> (f64, Vec3, f64) {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.dot(ab)).clamp(0.0, 1.0);
    let delta = (p - a) - ab * t;
    (t, delta, delta.length())
}

fn validate(cars: &[ContactCar<'_>], cfg: ContactSettings) -> Result<f64, ContactError> {
    let mut longest = 0.0_f64;
    // World-coordinate precision must resolve the skin. This also bounds all
    // geometric products below overflow, independently of the number of cars.
    let coordinate_bound = (cfg.skin / (64.0 * f64::EPSILON)).min(f64::MAX.sqrt() / 64.0);
    for (ci, car) in cars.iter().enumerate() {
        if car.pos.len() != car.previous_pos.len()
            || car.pos.len() != car.vel.len()
            || car.pos.len() != car.mass.len()
            || car.beams.len() != car.broken.len()
        {
            return Err(error(Some(ci), Feature::Layout, Failure::LengthMismatch));
        }
        for (i, &p) in car.pos.iter().enumerate() {
            let feature = Feature::Node(i);
            let old = car.previous_pos[i];
            let v = car.vel[i];
            if !(finite(p) && finite(old) && finite(v) && car.mass[i].is_finite()) {
                return Err(error(Some(ci), feature, Failure::NonFinite));
            }
            let inverse = 1.0 / car.mass[i];
            if !(car.mass[i] > 0.0 && inverse.is_normal()) {
                return Err(error(Some(ci), feature, Failure::InvalidMass));
            }
            for q in [p, old] {
                if q.x.abs().max(q.y.abs()).max(q.z.abs()) > coordinate_bound {
                    return Err(error(Some(ci), feature, Failure::NumericalRange));
                }
            }
            let motion = norm(p - old);
            let predicted = norm(v) * cfg.timestep;
            if !(motion.is_finite() && predicted.is_finite()) {
                return Err(error(Some(ci), feature, Failure::NumericalRange));
            }
            if motion > cfg.max_motion || predicted > cfg.max_motion {
                return Err(error(Some(ci), feature, Failure::MotionBound));
            }
        }
        for (bi, beam) in car.beams.iter().enumerate() {
            let feature = Feature::Beam(bi);
            if beam.a >= car.pos.len() || beam.b >= car.pos.len() {
                return Err(error(Some(ci), feature, Failure::InvalidIndex));
            }
            if !(beam.rest.is_finite() && beam.stiffness.is_finite() && beam.damping.is_finite()) {
                return Err(error(Some(ci), feature, Failure::NonFinite));
            }
            if !(beam.rest > 0.0 && beam.stiffness >= 0.0 && beam.damping >= 0.0) {
                return Err(error(Some(ci), feature, Failure::NumericalRange));
            }
            if car.broken[bi] {
                continue;
            }
            for positions in [car.previous_pos, car.pos] {
                let ab = positions[beam.b] - positions[beam.a];
                let squared = ab.dot(ab);
                if !(squared.is_normal() && squared > 0.0) {
                    return Err(error(Some(ci), feature, Failure::DegenerateBeam));
                }
                longest = longest.max(squared.sqrt());
            }
        }
    }
    Ok(longest)
}

/// A separating plane proves that a node stays outside a linearly moving
/// capsule: all four source-minus-endpoint vectors project beyond the skin.
/// Their bilinear interpolation has the same lower bound for every time and
/// every point on the beam. False means unsupported, not "no collision".
fn separated(
    source: &ContactCar<'_>,
    target: &ContactCar<'_>,
    key: ContactKey,
    n: Vec3,
    skin: f64,
) -> bool {
    clears(&corners(source, target, key), n, skin)
}

/// Four strict projections certify every bilinear sweep point by convexity.
/// Any nonzero normal works, so extra candidates cannot certify a real crossing.
/// Projections along n0 + lambda * (n1 - n0) are linear in lambda; try their
/// at most six pairwise crossings, accepting only through separated's strict bound.
fn separated_blend(
    source: &ContactCar<'_>,
    target: &ContactCar<'_>,
    key: ContactKey,
    n1: Vec3,
    n0: Vec3,
    skin: f64,
) -> bool {
    blend_clears(&corners(source, target, key), n1, n0, skin)
}

type Corners = [(Vec3, Vec3); 4];

fn corners(source: &ContactCar<'_>, target: &ContactCar<'_>, key: ContactKey) -> Corners {
    let b = target.beams[key.beam];
    [
        (source.previous_pos[key.node], target.previous_pos[b.a]),
        (source.previous_pos[key.node], target.previous_pos[b.b]),
        (source.pos[key.node], target.pos[b.a]),
        (source.pos[key.node], target.pos[b.b]),
    ]
}

fn clears(c: &Corners, n: Vec3, skin: f64) -> bool {
    let normal_length = norm(n);
    c.iter().all(|&(p, q)| {
        // Bound subtraction, dot-product and normal-length roundoff.
        // This only makes the refusal more conservative; it never
        // widens the exact narrow-phase contact predicate.
        let scale = norm(p).max(norm(q)).max(skin);
        let margin = 64.0 * f64::EPSILON * scale * normal_length;
        (p - q).dot(n) > skin * normal_length + margin
    })
}

fn blend_clears(c: &Corners, n1: Vec3, n0: Vec3, skin: f64) -> bool {
    let mut lines = [(0.0, 0.0); 4];
    for (line, &(p, q)) in c.iter().enumerate() {
        let w = p - q;
        let a = w.dot(n0) - skin;
        lines[line] = (a, w.dot(n1) - skin - a);
    }
    for j in 0..4 {
        for k in j + 1..4 {
            let lambda = (lines[k].0 - lines[j].0) / (lines[j].1 - lines[k].1);
            if lambda.is_finite()
                && lambda > 0.0
                && lambda < 1.0
                && clears(c, n0 + (n1 - n0) * lambda, skin)
            {
                return true;
            }
        }
    }
    false
}

const SWEEP_DEPTH: u32 = 6;

/// Restricted linear motion is bilinear; certified neighbouring pieces cover it.
/// Dyadic interior samples come once from the snapshots, shared by both pieces;
/// their corner error stays inside the existing 64*EPSILON margin.
/// Depth six is a work cap (at most 127 pieces), not a precision parameter;
/// only uncertified pieces split, and an uncertified cover still refuses.
fn separated_pieces(
    source: &ContactCar<'_>,
    target: &ContactCar<'_>,
    key: ContactKey,
    n1: Vec3,
    n0: Vec3,
    skin: f64,
) -> bool {
    type Sample = (f64, [Vec3; 3], Vec3);
    fn split(
        old: &[Vec3; 3],
        new: &[Vec3; 3],
        lo: Sample,
        hi: Sample,
        depth: u32,
        skin: f64,
    ) -> bool {
        if depth == 0 {
            return false;
        }
        let s = (lo.0 + hi.0) * 0.5;
        let at = std::array::from_fn(|i| old[i] + (new[i] - old[i]) * s);
        let (_, delta, d) = closest(at[0], at[1], at[2]);
        if !(d.is_finite() && d > skin) {
            return false;
        }
        let mid = (s, at, delta * (1.0 / d));
        piece(old, new, lo, mid, depth - 1, skin) && piece(old, new, mid, hi, depth - 1, skin)
    }
    fn piece(
        old: &[Vec3; 3],
        new: &[Vec3; 3],
        lo: Sample,
        hi: Sample,
        depth: u32,
        skin: f64,
    ) -> bool {
        let c = [
            (lo.1[0], lo.1[1]),
            (lo.1[0], lo.1[2]),
            (hi.1[0], hi.1[1]),
            (hi.1[0], hi.1[2]),
        ];
        clears(&c, hi.2, skin)
            || clears(&c, lo.2, skin)
            || blend_clears(&c, hi.2, lo.2, skin)
            || split(old, new, lo, hi, depth, skin)
    }
    let b = target.beams[key.beam];
    let old = [
        source.previous_pos[key.node],
        target.previous_pos[b.a],
        target.previous_pos[b.b],
    ];
    let new = [source.pos[key.node], target.pos[b.a], target.pos[b.b]];
    split(&old, &new, (0.0, old, n0), (1.0, new, n1), SWEEP_DEPTH, skin)
}

fn prepare(
    cars: &[ContactCar<'_>],
    cfg: ContactSettings,
    previous_impulses: &BTreeSet<ContactKey>,
) -> Result<(usize, Vec<Contact>), ContactError> {
    let longest = validate(cars, cfg)?;
    let mut flat = Vec::new();
    let mut ids = Vec::new();
    let mut incident = Vec::new();
    for (ci, car) in cars.iter().enumerate() {
        let mut links = vec![Vec::new(); car.pos.len()];
        for (bi, b) in car.beams.iter().enumerate() {
            if !car.broken[bi] {
                links[b.a].push(bi);
                links[b.b].push(bi);
            }
        }
        for (ni, &p) in car.pos.iter().enumerate() {
            flat.push(p);
            ids.push((ci, ni));
        }
        incident.push(links);
    }
    // Any swept hit is at most skin + 2*max_motion from the final beam.
    // Either endpoint is at most longest farther away. Padding covers roundoff
    // in the endpoint query, NOT the exact narrow-phase contact predicate.
    let radius = (longest + cfg.skin + 2.0 * cfg.max_motion) * (1.0 + 64.0 * f64::EPSILON);
    let pairs = candidate_pairs(&flat, radius, &|i, j| ids[i].0 == ids[j].0)
        .map_err(|_| error(None, Feature::Layout, Failure::NumericalRange))?;
    let mut keys = BTreeSet::new();
    for (i, j) in pairs {
        for (source, endpoint) in [(i, j), (j, i)] {
            let (sc, node) = ids[source];
            let (tc, target_node) = ids[endpoint];
            for &beam in &incident[tc][target_node] {
                keys.insert(ContactKey {
                    source_car: sc,
                    node,
                    target_car: tc,
                    beam,
                });
            }
        }
    }
    let count = keys.len();
    let mut contacts = Vec::new();
    for key in keys {
        let source = &cars[key.source_car];
        let target = &cars[key.target_car];
        let b = target.beams[key.beam];
        let (t, delta, distance) = closest(source.pos[key.node], target.pos[b.a], target.pos[b.b]);
        let (_, old_delta, old_distance) = closest(
            source.previous_pos[key.node],
            target.previous_pos[b.a],
            target.previous_pos[b.b],
        );
        let refusal = |failure| error(Some(key.source_car), Feature::Contact(key), failure);
        if !(distance.is_finite() && old_distance.is_finite() && t.is_finite()) {
            return Err(refusal(Failure::NumericalRange));
        }
        if distance == 0.0 || old_distance == 0.0 {
            return Err(refusal(Failure::UndefinedNormal));
        }
        if distance < cfg.skin - cfg.max_penetration
            || old_distance < cfg.skin - cfg.max_penetration
        {
            return Err(refusal(Failure::PenetrationBound));
        }
        let normal = delta * (1.0 / distance);
        let old_normal = old_delta * (1.0 / old_distance);
        // Position following can park an impulsed pair just outside the skin.
        // Keep it touching for this substep only, within the existing band.
        if distance <= cfg.skin
            || (old_distance > cfg.skin
                && distance <= cfg.skin + 2.0 * cfg.max_motion
                && previous_impulses.contains(&key))
        {
            contacts.push(Contact {
                key,
                fraction: t,
                normal,
            });
        } else if old_distance > cfg.skin
            && distance <= cfg.skin + 2.0 * cfg.max_motion
            && (source.previous_pos[key.node] != source.pos[key.node]
                || target.previous_pos[b.a] != target.pos[b.a]
                || target.previous_pos[b.b] != target.pos[b.b])
            && !separated(source, target, key, normal, cfg.skin)
            && !separated(source, target, key, old_normal, cfg.skin)
            && !separated_blend(source, target, key, normal, old_normal, cfg.skin)
            && !separated_pieces(source, target, key, normal, old_normal, cfg.skin)
        {
            return Err(refusal(Failure::UnsupportedSweep));
        }
    }
    Ok((count, contacts))
}

/// Read-only narrow phase, sharing response's structural, motion and geometric
/// validation. Response additionally checks staged impulse/energy arithmetic.
pub fn detect_contacts(
    cars: &[ContactCar<'_>],
    cfg: ContactSettings,
) -> Result<Vec<Contact>, ContactError> {
    prepare(cars, cfg, &BTreeSet::new()).map(|(_, contacts)| contacts)
}

/// Same slice identities and pre-substep snapshot meaning as `ContactCar`;
/// positions follow each applied impulse's velocity change times the timestep.
pub struct MovingContactCar<'a> {
    pub previous_pos: &'a [Vec3],
    pub pos: &'a mut [Vec3],
    pub vel: &'a mut [Vec3],
    pub mass: &'a [f64],
    pub beams: &'a [Beam],
    pub broken: &'a [bool],
    /// This car's collider planes, in collider order (ground first).
    pub planes: &'a [crate::collide::Plane],
}

fn kinetic(cars: &[ContactCar<'_>], velocities: &[Vec<Vec3>]) -> Result<f64, ContactError> {
    let mut sum = 0.0;
    for (ci, (car, vel)) in cars.iter().zip(velocities).enumerate() {
        for (ni, (&v, &m)) in vel.iter().zip(car.mass).enumerate() {
            if !finite(v) {
                return Err(error(Some(ci), Feature::Node(ni), Failure::NonFinite));
            }
            sum += 0.5 * m * v.dot(v);
            if !sum.is_finite() {
                return Err(error(Some(ci), Feature::Node(ni), Failure::NumericalRange));
            }
        }
    }
    Ok(sum)
}

/// One fixed-order sequential normal-impulse pass. All work is staged; only a
/// finite, energy-safe result is copied back. The caller must propagate errors.
pub fn apply_contacts(
    cars: &mut [ContactCar<'_>],
    cfg: ContactSettings,
) -> Result<ContactReport, ContactError> {
    let staged = contact_pass(cars, cfg, false, &BTreeSet::new())?;
    for (car, vel) in cars.iter_mut().zip(staged.velocities) {
        car.vel.copy_from_slice(&vel);
    }
    Ok(staged.report)
}

/// One transactional sequential pass with `x += dv*h` for every impulse node.
/// Normals and fractions use incoming positions. Any refusal writes nothing.
/// Supply only the immediately preceding successful substep's impulse keys
/// (empty initially). Outside/outside pairs in the existing motion band with
/// those keys remain contacts; all other pairs keep the usual sweep checks.
/// Returns the report and the ordered set of keys that actually got impulses.
pub fn apply_contacts_moving(
    cars: &mut [MovingContactCar<'_>],
    cfg: ContactSettings,
    previous_impulses: &BTreeSet<ContactKey>,
) -> Result<(ContactReport, BTreeSet<ContactKey>), ContactError> {
    let staged = {
        let views: Vec<_> = cars
            .iter_mut()
            .map(|c| ContactCar {
                previous_pos: c.previous_pos,
                pos: &*c.pos,
                vel: &mut *c.vel,
                mass: c.mass,
                beams: c.beams,
                broken: c.broken,
            })
            .collect();
        contact_pass(&views, cfg, true, previous_impulses)?
    };
    for (ci, (car, positions)) in cars.iter().zip(&staged.positions).enumerate() {
        for (i, (&before, &after)) in car.pos.iter().zip(positions).enumerate() {
            for plane in car.planes {
                let old_distance = plane.distance(before);
                let distance = plane.distance(after);
                if !(old_distance.is_finite() && distance.is_finite())
                    || !plane.normal.dot(plane.normal).is_normal()
                {
                    return Err(error(Some(ci), Feature::Node(i), Failure::NumericalRange));
                }
                if old_distance >= 0.0 && distance < 0.0 {
                    return Err(error(Some(ci), Feature::Node(i), Failure::PenetrationBound));
                }
            }
        }
    }
    for ((car, vel), pos) in cars.iter_mut().zip(staged.velocities).zip(staged.positions) {
        car.vel.copy_from_slice(&vel);
        car.pos.copy_from_slice(&pos);
    }
    Ok((staged.report, staged.impulse_keys))
}

struct StagedContacts {
    report: ContactReport,
    impulse_keys: BTreeSet<ContactKey>,
    velocities: Vec<Vec<Vec3>>,
    positions: Vec<Vec<Vec3>>,
}

fn contact_pass(
    cars: &[ContactCar<'_>],
    cfg: ContactSettings,
    follow: bool,
    previous_impulses: &BTreeSet<ContactKey>,
) -> Result<StagedContacts, ContactError> {
    let (candidates, contacts) = prepare(cars, cfg, previous_impulses)?;
    let mut velocities: Vec<Vec<Vec3>> = cars.iter().map(|c| c.vel.to_vec()).collect();
    let mut positions: Vec<Vec<Vec3>> = cars.iter().map(|c| c.pos.to_vec()).collect();
    let mut moved: Vec<Vec<bool>> = cars.iter().map(|c| vec![false; c.pos.len()]).collect();
    let before = kinetic(cars, &velocities)?;
    let mut report = ContactReport {
        candidates,
        contacts: contacts.len(),
        ..ContactReport::default()
    };
    let mut last_impulse = None;
    let mut impulse_keys = BTreeSet::new();
    for contact in contacts {
        let key = contact.key;
        let b = cars[key.target_car].beams[key.beam];
        let t = contact.fraction;
        let u = 1.0 - t;
        let source_inverse = 1.0 / cars[key.source_car].mass[key.node];
        let a_inverse = 1.0 / cars[key.target_car].mass[b.a];
        let b_inverse = 1.0 / cars[key.target_car].mass[b.b];
        let target_velocity =
            velocities[key.target_car][b.a] * u + velocities[key.target_car][b.b] * t;
        let relative = (velocities[key.source_car][key.node] - target_velocity).dot(contact.normal);
        let effective_inverse = source_inverse + u * u * a_inverse + t * t * b_inverse;
        let refusal = |failure| error(Some(key.source_car), Feature::Contact(key), failure);
        if !(relative.is_finite() && effective_inverse.is_normal() && effective_inverse > 0.0) {
            return Err(refusal(Failure::NumericalRange));
        }
        if relative >= 0.0 {
            continue;
        }
        let impulse = -(1.0 + cfg.restitution) * relative / effective_inverse;
        let changes = [
            contact.normal * (impulse * source_inverse),
            contact.normal * (-impulse * u * a_inverse),
            contact.normal * (-impulse * t * b_inverse),
        ];
        if !(impulse.is_finite() && changes.iter().all(|&v| finite(v))) {
            return Err(refusal(Failure::NumericalRange));
        }
        velocities[key.source_car][key.node] += changes[0];
        velocities[key.target_car][b.a] += changes[1];
        velocities[key.target_car][b.b] += changes[2];
        if follow {
            moved[key.source_car][key.node] = true;
            moved[key.target_car][b.a] = true;
            moved[key.target_car][b.b] = true;
            positions[key.source_car][key.node] += changes[0] * cfg.timestep;
            positions[key.target_car][b.a] += changes[1] * cfg.timestep;
            positions[key.target_car][b.b] += changes[2] * cfg.timestep;
            if ![
                positions[key.source_car][key.node],
                positions[key.target_car][b.a],
                positions[key.target_car][b.b],
            ]
            .iter()
            .all(|&p| finite(p))
            {
                return Err(refusal(Failure::NumericalRange));
            }
        }
        report.impulses += 1;
        report.normal_impulse += impulse;
        impulse_keys.insert(key);
        last_impulse = Some(key);
        if !report.normal_impulse.is_finite() {
            return Err(refusal(Failure::NumericalRange));
        }
    }
    // Bound chassis kinetic energy in-pass. Position following can change beam
    // elastic energy; callers measure full-state energy over whole substeps,
    // including the integrator's work, rather than refusing a steady push here.
    let after = kinetic(cars, &velocities)?;
    if after - before > 1e-10 * before.abs().max(1.0) {
        let (car, feature) = last_impulse.map_or((None, Feature::Layout), |key| {
            (Some(key.source_car), Feature::Contact(key))
        });
        return Err(error(car, feature, Failure::EnergyIncrease));
    }
    // Finite speed and the NEXT substep's permitted motion are checked before
    // any write as well. An impulse can accelerate a very light endpoint.
    for (ci, vel) in velocities.iter().enumerate() {
        for (ni, &v) in vel.iter().enumerate() {
            if norm(v) * cfg.timestep > cfg.max_motion {
                return Err(error(Some(ci), Feature::Node(ni), Failure::MotionBound));
            }
        }
    }
    if follow && !impulse_keys.is_empty() {
        // Visit affected live pairs in ContactKey order, including pairs absent
        // from the incoming contacts: another impulse may have moved them inward.
        for (sc, source) in positions.iter().enumerate() {
            for (node, &p) in source.iter().enumerate() {
                for (tc, target) in cars.iter().enumerate() {
                    if sc == tc {
                        continue;
                    }
                    for (bi, b) in target.beams.iter().enumerate() {
                        if target.broken[bi]
                            || !(moved[sc][node] || moved[tc][b.a] || moved[tc][b.b])
                        {
                            continue;
                        }
                        let key = ContactKey {
                            source_car: sc,
                            node,
                            target_car: tc,
                            beam: bi,
                        };
                        let (_, _, distance) = closest(p, positions[tc][b.a], positions[tc][b.b]);
                        let failure = if !distance.is_finite() {
                            Failure::NumericalRange
                        } else if distance < cfg.skin - cfg.max_penetration {
                            Failure::PenetrationBound
                        } else {
                            continue;
                        };
                        return Err(error(Some(sc), Feature::Contact(key), failure));
                    }
                }
            }
        }
    }
    Ok(StagedContacts {
        report,
        impulse_keys,
        velocities,
        positions,
    })
}
