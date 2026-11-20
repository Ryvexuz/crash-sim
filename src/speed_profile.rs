//! Speed profile along a closed centreline: lateral-grip limit from curvature, then a forward
//! (acceleration) and a backward (braking) pass repeated around the loop until nothing changes.
//!
//! The path is a closed loop of points in the ground plane (x, z; y is up and ignored). The last
//! point connects back to the first. Self-contained, std only apart from `Vec3`.
use crate::vec3::Vec3;

/// Most sweeps (forward + backward pair) before the profile is declared not converged.
const MAX_SWEEPS: usize = 1000;

#[derive(Debug, Clone, PartialEq)]
pub enum ProfileError {
    TooFewPoints(usize),
    /// Point `index` equals the next point (cyclically).
    DuplicatePoint(usize),
    /// A parameter that must be finite and positive was not; carries its name.
    BadParameter(&'static str),
    /// The accel/brake passes did not settle within the sweep cap.
    NotConverged,
}

impl std::fmt::Display for ProfileError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ProfileError::TooFewPoints(n) => write!(f, "need at least 3 points, got {n}"),
            ProfileError::DuplicatePoint(i) => write!(f, "point {i} duplicates the next point"),
            ProfileError::BadParameter(p) => write!(f, "{p} must be finite and positive"),
            ProfileError::NotConverged => write!(f, "speed profile did not converge"),
        }
    }
}

impl std::error::Error for ProfileError {}

fn check_path(path: &[Vec3]) -> Result<(), ProfileError> {
    if path.len() < 3 {
        return Err(ProfileError::TooFewPoints(path.len()));
    }
    for i in 0..path.len() {
        let d = path[(i + 1) % path.len()] - path[i];
        if d.x == 0.0 && d.z == 0.0 {
            return Err(ProfileError::DuplicatePoint(i));
        }
    }
    Ok(())
}

fn seg_len(a: Vec3, b: Vec3) -> f64 {
    let (dx, dz) = (b.x - a.x, b.z - a.z);
    (dx * dx + dz * dz).sqrt()
}

/// Signed curvature (1 / circumradius) at every point from it and its two neighbours, closed loop.
/// Positive turns from +x towards +z. Collinear points give 0.
pub fn curvature(path: &[Vec3]) -> Result<Vec<f64>, ProfileError> {
    check_path(path)?;
    let n = path.len();
    Ok((0..n)
        .map(|i| {
            let (p0, p1, p2) = (path[(i + n - 1) % n], path[i], path[(i + 1) % n]);
            let cross = (p1.x - p0.x) * (p2.z - p0.z) - (p1.z - p0.z) * (p2.x - p0.x);
            let denom = seg_len(p0, p1) * seg_len(p1, p2) * seg_len(p0, p2);
            if denom == 0.0 { 0.0 } else { 2.0 * cross / denom }
        })
        .collect())
}

fn positive(name: &'static str, v: f64) -> Result<(), ProfileError> {
    if v.is_finite() && v > 0.0 { Ok(()) } else { Err(ProfileError::BadParameter(name)) }
}

/// Target speed at every point: sqrt(mu g / |k|) capped at `v_max`, then limited so speed never
/// rises faster than `a_accel` or falls faster than `a_brake` (both m/s^2, positive) per metre
/// travelled, around the closed loop.
pub fn profile(
    path: &[Vec3],
    mu: f64,
    g: f64,
    a_accel: f64,
    a_brake: f64,
    v_max: f64,
) -> Result<Vec<f64>, ProfileError> {
    positive("mu", mu)?;
    positive("g", g)?;
    positive("a_accel", a_accel)?;
    positive("a_brake", a_brake)?;
    positive("v_max", v_max)?;
    let k = curvature(path)?;
    let n = path.len();
    let ds: Vec<f64> = (0..n).map(|i| seg_len(path[i], path[(i + 1) % n])).collect();
    let mut v: Vec<f64> = k
        .iter()
        .map(|k| if *k == 0.0 { v_max } else { (mu * g / k.abs()).sqrt().min(v_max) })
        .collect();
    for _ in 0..MAX_SWEEPS {
        let mut changed = false;
        // Forward: speed at i limits speed at i+1 (start at the slowest point so one lap suffices).
        for i in 0..n {
            let j = (i + 1) % n;
            let lim = (v[i] * v[i] + 2.0 * a_accel * ds[i]).sqrt();
            if lim < v[j] {
                v[j] = lim;
                changed = true;
            }
        }
        // Backward: speed at i+1 limits speed at i given the braking available over ds[i].
        for i in (0..n).rev() {
            let j = (i + 1) % n;
            let lim = (v[j] * v[j] + 2.0 * a_brake * ds[i]).sqrt();
            if lim < v[i] {
                v[i] = lim;
                changed = true;
            }
        }
        if !changed {
            return Ok(v);
        }
    }
    Err(ProfileError::NotConverged)
}
