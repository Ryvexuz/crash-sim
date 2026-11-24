//! STAGE7's 4.40 m centre floor is 2 x 2.193 m rounded up.
//! This independent measure is not the contact rule and never changes the race.
//! Observe every completed substep, not just control/output boundaries.
//! Strict comparisons keep the first minimum and exclude exactly 4.40 m from
//! the near count. The later CLI slice prints these statistics as the `cars` line.
//!
//! Distances use the specified raw IEEE-754 formula, without clamping or
//! rescaling: extreme inputs can underflow, overflow, or yield NaN. A first NaN
//! minimum is retained and is not near; later NaNs do not replace a minimum.
//! The race supplies Sim-checked positions and finite substep end times.

use crate::vec3::Vec3;

/// STAGE7's floor: two centres closer than this may touch (2 x 2.193 m, rounded up).
pub const NEAR_M: f64 = 4.40;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GapStats {
    /// Smallest pair distance seen over all observed substeps, metres.
    pub gap_min_m: f64,
    /// t1 of the first substep at which gap_min_m was reached.
    pub gap_min_t: f64,
    /// Substeps at which at least one pair was strictly under NEAR_M.
    pub near_substeps: u64,
}

/// Ground-plane distance: dx = a.x - b.x, dz = a.z - b.z, (dx * dx + dz * dz).sqrt().
pub fn ground_distance(a: Vec3, b: Vec3) -> f64 {
    let dx = a.x - b.x;
    let dz = a.z - b.z;
    (dx * dx + dz * dz).sqrt()
}

/// Smallest ground_distance over pairs (i, j), i < j, visited i = 0.. then j = i + 1...
/// A later pair replaces the minimum only when strictly smaller.
/// None for fewer than two centres.
pub fn min_pair_distance(centres: &[Vec3]) -> Option<f64> {
    let mut minimum = None;
    for (i, &a) in centres.iter().enumerate() {
        for &b in centres.iter().skip(i + 1) {
            let distance = ground_distance(a, b);
            if minimum.is_none_or(|m| distance < m) {
                minimum = Some(distance);
            }
        }
    }
    minimum
}

#[derive(Clone, Debug, Default)]
pub struct NearTracker {
    stats: Option<GapStats>,
}

impl NearTracker {
    pub fn new() -> Self {
        Self::default()
    }

    /// One substep: centres in car order, t1 the substep's end time.
    pub fn observe(&mut self, centres: &[Vec3], t1: f64) {
        let Some(m) = min_pair_distance(centres) else {
            return;
        };
        let stats = self.stats.get_or_insert(GapStats {
            gap_min_m: m,
            gap_min_t: t1,
            near_substeps: 0,
        });
        if m < stats.gap_min_m {
            stats.gap_min_m = m;
            stats.gap_min_t = t1;
        }
        if m < NEAR_M {
            stats.near_substeps += 1;
        }
    }

    /// None until a substep with two or more centres has been observed.
    pub fn stats(&self) -> Option<GapStats> {
        self.stats
    }
}
