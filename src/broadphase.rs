//! Uniform-grid contact candidates for points, without contact response.
//!
//! Follows foundation's `DSpatialHash` design described in the task brief: a
//! padded cell search, occupied-cell fallback, and lexicographically sorted pairs.
//! No dependency on the foundation checkout (which is absent in task worktrees).

use std::collections::HashMap;

use crate::vec3::Vec3;

/// Invalid input to [`candidate_pairs`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BroadphaseError {
    /// The radius is negative or non-finite. Zero is a valid query radius.
    BadRadius,
    /// The first position with a non-finite component.
    NonFinite { index: usize },
}

/// Padding in cell units, not a distance tolerance: the final predicate is exact.
const CELL_PAD: f64 = 0.125;

/// Six guard bits keep cell arithmetic rounding smaller than the search pad.
const CELL_PRECISION_SCALE: f64 = 64.0 * f64::EPSILON;

type Cell = (i64, i64, i64);

fn cell_of(p: Vec3, size: f64) -> Cell {
    (
        (p.x / size).floor() as i64,
        (p.y / size).floor() as i64,
        (p.z / size).floor() as i64,
    )
}

fn cell_bounds(p: Vec3, radius: f64, size: f64) -> (Cell, Cell) {
    let inv = 1.0 / size;
    let lower = |v: f64| (((v - radius) * inv) - CELL_PAD).floor() as i64;
    let upper = |v: f64| (((v + radius) * inv) + CELL_PAD).floor() as i64;
    (
        (lower(p.x), lower(p.y), lower(p.z)),
        (upper(p.x), upper(p.y), upper(p.z)),
    )
}

/// All `(i, j)`, `i < j`, within `radius`, except pairs for which `skip(i, j)`
/// is true (for example, beam-connected nodes). Output is lexicographically sorted.
///
/// Radius must be finite and nonnegative; every position component must be finite.
/// Validation finishes before `skip` is called, including for empty/single inputs.
/// `skip` is called only on qualifying pairs, once each, in sorted order.
///
/// "Within" means exactly `dx*dx + dy*dy + dz*dz <= radius*radius`, with
/// products first, left-to-right addition, and no fused multiply-add. This includes
/// IEEE-754 underflow/overflow behavior, not a rescaled Euclidean norm.
pub fn candidate_pairs(
    pos: &[Vec3],
    radius: f64,
    skip: &dyn Fn(usize, usize) -> bool,
) -> Result<Vec<(usize, usize)>, BroadphaseError> {
    if !radius.is_finite() || radius < 0.0 {
        return Err(BroadphaseError::BadRadius);
    }
    let mut largest = 0.0_f64;
    for (index, p) in pos.iter().enumerate() {
        if !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite()) {
            return Err(BroadphaseError::NonFinite { index });
        }
        largest = largest.max(p.x.abs()).max(p.y.abs()).max(p.z.abs());
    }
    if pos.len() < 2 {
        return Ok(Vec::new());
    }

    // Bound normalized coordinates so subtraction, reciprocal multiplication
    // and division rounding stay inside the pad, even at enormous world
    // coordinates. The scale is below one, so this cannot overflow.
    // This is a floating-point precision bound, not a configurable scene limit.
    let precision_size = largest * CELL_PRECISION_SCALE;
    let size = radius.max(f64::MIN_POSITIVE.sqrt()).max(precision_size);
    let mut grid: HashMap<Cell, Vec<usize>> = HashMap::new();
    for (index, &p) in pos.iter().enumerate() {
        grid.entry(cell_of(p, size)).or_default().push(index);
    }

    let radius_squared = radius * radius;
    // An infinite square admits even infinite squared separations. Subnormal
    // squares (including zero) can round separations greater than radius inside.
    // Visit occupied cells in these cases rather than imposing geometric bounds
    // that could contradict the specified predicate.
    let unrestricted = !radius_squared.is_normal();
    let mut pairs = Vec::new();
    for (i, &p) in pos.iter().enumerate() {
        let mut visit = |indices: &[usize]| {
            for &j in indices {
                if j <= i {
                    continue;
                }
                let dx = p.x - pos[j].x;
                let dy = p.y - pos[j].y;
                let dz = p.z - pos[j].z;
                let xx = dx * dx;
                let yy = dy * dy;
                let zz = dz * dz;
                if (xx + yy) + zz <= radius_squared {
                    pairs.push((i, j));
                }
            }
        };
        if unrestricted {
            for indices in grid.values() {
                visit(indices);
            }
            continue;
        }

        let (lo, hi) = cell_bounds(p, radius, size);
        // Coordinates are bounded by the precision-based cell size above, so
        // these differences and their product fit (each range spans only a few
        // cells, since size >= radius).
        let cell_count =
            (hi.0 - lo.0 + 1) as u128 * (hi.1 - lo.1 + 1) as u128 * (hi.2 - lo.2 + 1) as u128;
        if cell_count > grid.len() as u128 {
            for indices in grid.values() {
                visit(indices);
            }
        } else {
            for x in lo.0..=hi.0 {
                for y in lo.1..=hi.1 {
                    for z in lo.2..=hi.2 {
                        if let Some(indices) = grid.get(&(x, y, z)) {
                            visit(indices);
                        }
                    }
                }
            }
        }
    }
    // Neither output order nor callback order may depend on HashMap iteration.
    pairs.sort_unstable();
    pairs.retain(|&(i, j)| !skip(i, j));
    Ok(pairs)
}
