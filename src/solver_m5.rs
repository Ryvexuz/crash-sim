//! The m5 crash-zone solver (docs/M5-SOLVER.md, method (2); the law is written out in
//! docs/PHYSICS.md, "Solids"): explicit linear tetrahedra with compressible Neo-Hookean
//! elasticity and a compressive crushable plasticity with lock-up, on the same nodes as the
//! beams, and one semi-implicit Euler update for every node.
//!
//! Each substep (`SolverM5::step`): gravity, then the beam forces (the arithmetic and order of
//! `solver::SymplecticEuler`, so a body with no tets gives the same bits), then the tet forces in
//! tet index order, all into one working force array, then `vel += force * h / m`,
//! `pos += vel * h`. No threads, no hash maps, a fixed order everywhere: the same input gives the
//! same bytes.
//!
//! The solver fails closed. No non-finite value is ever written into the working force array:
//! each contribution is checked, with the sum it would make, before it is written (the error
//! names its source), and the whole array is checked again after each source. The state before
//! the step and the new state after it are checked too, and every tet's `Je` on the new
//! positions. A tet whose `Je` is not finite or is at or below `J_MIN_FAIL`, and a tangent bound
//! that puts `h omega` at or above 2, are errors as well. Every error goes through one function,
//! and one gate before the commit then applies nothing of the substep. The first error is kept
//! and `Solver::check` returns it, so `Sim::check` ends the run. Nothing is clamped.

use crate::format::{Body, TET_MIN_SHAPE, TET_MIN_VOLUME};
use crate::solver::{Solver, State};
use crate::vec3::Vec3;

/// A 3 x 3 matrix, row major: `m[i][j]` is row i, column j.
pub type Mat3 = [[f64; 3]; 3];

/// The most tets the solver takes: `frozen: m5_max_tets = 300` (docs/M5-SOLVER.md section 8).
/// A body with more is refused before the solver allocates anything per tet.
pub const M5_MAX_TETS: usize = 300;
/// The largest node coordinate (m, in size, inclusive) `from_body` takes: 1 000 km. A car, a
/// crash zone or a whole track lies within a few km of its origin, so this is far beyond any
/// real body, and it keeps every length, squared length and beam or tet product of a built
/// body many orders of magnitude below f64 overflow (the source of round 3's finding).
pub const MAX_COORD_M: f64 = 1e6;
/// The largest start speed (m/s, inclusive) `from_body` takes: 10 km/s, about 640 times the
/// 56 km/h of the m4/m5 crash tests and above orbital speed, so it is far beyond any car.
pub const MAX_SPEED_MPS: f64 = 1e4;
/// `frozen: poisson_ratio = 0.3` (docs/M5-SOLVER.md section 8).
pub const POISSON_RATIO: f64 = 0.3;
/// Steel density (kg/m^3) of the lock-up line `phi_zone = zone_mass / (zone_volume * 7850)`.
pub const STEEL_DENSITY: f64 = 7850.0;
/// The cap 0.7 of the lock-up line `eps_D = min(0.7, 1 - 1.4 phi_zone)`.
pub const LOCKUP_CAP: f64 = 0.7;
/// The factor 1.4 of the lock-up line `eps_D = min(0.7, 1 - 1.4 phi_zone)`.
pub const LOCKUP_SLOPE: f64 = 1.4;
/// The inverted-tet rule (R4e): a trial `Je` at or below this, or not finite, ends the run.
pub const J_MIN_FAIL: f64 = 0.05;
/// The construction check (R5a) takes the tet tangent at rest times this factor, so that the
/// stiffness may double during the run before the per-substep check (R5b) trips.
pub const STIFFNESS_HEADROOM: f64 = 2.0;
/// Cyclic Jacobi sweeps of the 3 x 3 symmetric eigen-solve (a fixed count, no open loop).
pub const JACOBI_SWEEPS: usize = 6;
/// Bisection steps of the plastic return mapping (R4b: a fixed count, no open loop).
pub const BISECTION_STEPS: usize = 60;
/// Relative allowance (of the trace) for the rounding of the Jacobi rotations in the lower
/// eigenvalue bound of `sym_eigen`: about 450 times the f64 epsilon.
const EIGEN_ROUNDING: f64 = 1e-13;

// ---------------------------------------------------------------------------------------
// Small matrix helpers

fn det(m: &Mat3) -> f64 {
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
}

/// The cofactor matrix: `cof[i][j]` = d det / d m[i][j], so `m^-T = cof / det`.
fn cofactor(m: &Mat3) -> Mat3 {
    [
        [
            m[1][1] * m[2][2] - m[1][2] * m[2][1],
            m[1][2] * m[2][0] - m[1][0] * m[2][2],
            m[1][0] * m[2][1] - m[1][1] * m[2][0],
        ],
        [
            m[0][2] * m[2][1] - m[0][1] * m[2][2],
            m[0][0] * m[2][2] - m[0][2] * m[2][0],
            m[0][1] * m[2][0] - m[0][0] * m[2][1],
        ],
        [
            m[0][1] * m[1][2] - m[0][2] * m[1][1],
            m[0][2] * m[1][0] - m[0][0] * m[1][2],
            m[0][0] * m[1][1] - m[0][1] * m[1][0],
        ],
    ]
}

fn mul(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut c = [[0.0; 3]; 3];
    for (ci, ai) in c.iter_mut().zip(a) {
        for (j, cij) in ci.iter_mut().enumerate() {
            *cij = ai[0] * b[0][j] + ai[1] * b[1][j] + ai[2] * b[2][j];
        }
    }
    c
}

/// `m^T m`.
fn ata(m: &Mat3) -> Mat3 {
    let mut c = [[0.0; 3]; 3];
    for (i, ci) in c.iter_mut().enumerate() {
        for (j, cij) in ci.iter_mut().enumerate() {
            *cij = m[0][i] * m[0][j] + m[1][i] * m[1][j] + m[2][i] * m[2][j];
        }
    }
    c
}

fn mat_vec(m: &Mat3, v: Vec3) -> Vec3 {
    Vec3::new(
        m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
        m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
        m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z,
    )
}

/// `m + a u n^T`.
fn add_outer(m: &Mat3, a: f64, u: Vec3, n: Vec3) -> Mat3 {
    let (u, n) = ([u.x, u.y, u.z], [n.x, n.y, n.z]);
    let mut r = *m;
    for (i, ri) in r.iter_mut().enumerate() {
        for (j, rij) in ri.iter_mut().enumerate() {
            *rij += a * u[i] * n[j];
        }
    }
    r
}

fn row(m: &Mat3, k: usize) -> Vec3 {
    Vec3::new(m[k][0], m[k][1], m[k][2])
}

/// The matrix whose columns are `a`, `b`, `c`.
fn from_cols(a: Vec3, b: Vec3, c: Vec3) -> Mat3 {
    [[a.x, b.x, c.x], [a.y, b.y, c.y], [a.z, b.z, c.z]]
}

/// `sum_k d_k (x) r_k`, with `d_k` the columns of a matrix and `r_k` the rows of `b`: the
/// product `[d1 d2 d3] b`.
fn cols_times(d: [Vec3; 3], b: &Mat3) -> Mat3 {
    mul(&from_cols(d[0], d[1], d[2]), b)
}

fn finite(v: Vec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}

// ---------------------------------------------------------------------------------------
// The law (docs/PHYSICS.md "Solids")

/// The numbers of one solid's law. `mu` and `lambda` are the Lame constants of
/// `E = plateau_stress / yield_strain` and `POISSON_RATIO`; `eps_d` is the zone's lock-up strain
/// and `e_max = eps_d - sqrt(eps_d yield_strain)` the plastic compression at which flow stops.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Law {
    pub mu: f64,
    pub lambda: f64,
    pub plateau_stress: f64,
    pub yield_strain: f64,
    pub eps_d: f64,
    pub e_max: f64,
}

impl Law {
    /// The law of a solid with these numbers in a zone of lock-up strain `eps_d`. The caller
    /// (`SolverM5::from_body`) has checked `plateau_stress > 0`, `0 < yield_strain < eps_d`.
    pub fn new(plateau_stress: f64, yield_strain: f64, eps_d: f64) -> Law {
        let e = plateau_stress / yield_strain;
        let nu = POISSON_RATIO;
        Law {
            mu: e / (2.0 * (1.0 + nu)),
            lambda: e * nu / ((1.0 + nu) * (1.0 - 2.0 * nu)),
            plateau_stress,
            yield_strain,
            eps_d,
            e_max: eps_d - (eps_d * yield_strain).sqrt(),
        }
    }

    /// The yield stress (Pa, positive) at plastic compression `e_c` (a log strain):
    /// `plateau_stress / (1 - e_c / eps_d)`, for `0 <= e_c <= e_max`.
    pub fn sigma_y(&self, e_c: f64) -> f64 {
        self.plateau_stress / (1.0 - e_c / self.eps_d)
    }
}

/// The zone's lock-up strain `eps_D = min(0.7, 1 - 1.4 phi_zone)` with
/// `phi_zone = zone_mass / (zone_volume * 7850)` (the `frozen: lockup` line).
pub fn lockup_strain(zone_mass: f64, zone_volume: f64) -> f64 {
    let phi = zone_mass / (zone_volume * STEEL_DENSITY);
    (1.0 - LOCKUP_SLOPE * phi).min(LOCKUP_CAP)
}

/// The elastic energy per rest volume (J/m^3) of an elastic deformation gradient:
/// `W = mu/2 (tr(Fe^T Fe) - 3) - mu ln Je + lambda/2 (ln Je)^2`, infinite unless `Je > 0`.
pub fn energy_density(fe: &Mat3, law: &Law) -> f64 {
    let je = det(fe);
    if je.is_nan() || je <= 0.0 {
        return f64::INFINITY;
    }
    let c = ata(fe);
    let l = je.ln();
    0.5 * law.mu * (c[0][0] + c[1][1] + c[2][2] - 3.0) - law.mu * l + 0.5 * law.lambda * l * l
}

/// `P = dW/dFe = mu Fe + (lambda ln Je - mu) Fe^-T` (Pa), for `Je > 0`.
pub fn first_piola(fe: &Mat3, law: &Law) -> Mat3 {
    let je = det(fe);
    piola_from(fe, je, je.ln(), law)
}

fn piola_from(fe: &Mat3, je: f64, ln_je: f64, law: &Law) -> Mat3 {
    let cof = cofactor(fe);
    let a = (law.lambda * ln_je - law.mu) / je;
    let mut p = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            p[i][j] = law.mu * fe[i][j] + a * cof[i][j];
        }
    }
    p
}

/// The result of `sym_eigen`: eigenvalues, the matrix whose column k is the eigenvector of
/// `values[k]`, and a lower bound on the smallest eigenvalue that holds whatever the accuracy
/// of the sweeps: Gershgorin's bound on the rotated matrix (each diagonal minus its row's
/// off-diagonal magnitudes) minus `EIGEN_ROUNDING` times the trace's size for the rounding of
/// the rotations (Weyl: the rotated matrix is within that of a matrix similar to the input).
#[derive(Clone, Copy, Debug)]
pub struct Eigen {
    pub values: [f64; 3],
    pub vectors: Mat3,
    pub lower: f64,
}

/// Eigenvalues and eigenvectors of a symmetric 3 x 3 matrix by `JACOBI_SWEEPS` cyclic Jacobi
/// sweeps over the pairs (0,1), (0,2), (1,2) (Numerical Recipes' rotation). A pair whose
/// off-diagonal is exactly 0, or below 1e-3 epsilon of its diagonals (where a rotation cannot
/// change the result), is skipped; the count of sweeps never depends on the data.
pub fn sym_eigen(m: &Mat3) -> Eigen {
    let mut a = *m;
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..JACOBI_SWEEPS {
        for &(p, q, r) in &[(0usize, 1usize, 2usize), (0, 2, 1), (1, 2, 0)] {
            let apq = a[p][q];
            let (app, aqq) = (a[p][p], a[q][q]);
            if apq == 0.0 || apq.abs() <= 1e-3 * f64::EPSILON * (app.abs() + aqq.abs()) {
                a[p][q] = 0.0;
                a[q][p] = 0.0;
                continue;
            }
            let theta = (aqq - app) / (2.0 * apq);
            let t = if theta.abs() > 1e150 {
                0.5 / theta
            } else {
                theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt())
            };
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            a[p][p] = app - t * apq;
            a[q][q] = aqq + t * apq;
            a[p][q] = 0.0;
            a[q][p] = 0.0;
            let (arp, arq) = (a[r][p], a[r][q]);
            a[r][p] = c * arp - s * arq;
            a[p][r] = a[r][p];
            a[r][q] = s * arp + c * arq;
            a[q][r] = a[r][q];
            for vk in v.iter_mut() {
                let (vkp, vkq) = (vk[p], vk[q]);
                vk[p] = c * vkp - s * vkq;
                vk[q] = s * vkp + c * vkq;
            }
        }
    }
    let values = [a[0][0], a[1][1], a[2][2]];
    let size = values[0].abs() + values[1].abs() + values[2].abs();
    let mut lower = f64::INFINITY;
    for i in 0..3 {
        let off: f64 = (0..3).filter(|&j| j != i).map(|j| a[i][j].abs()).sum();
        lower = lower.min(values[i] - off);
    }
    Eigen {
        values,
        vectors: v,
        lower: lower - EIGEN_ROUNDING * size,
    }
}

/// The closed-form upper bound (Pa) on the size of the elastic tangent `d2W/dFe2` (as an
/// operator on 3 x 3 matrices with the Frobenius norm) at `Fe`, from the lower bound `lower`
/// on the smallest eigenvalue of `Fe^T Fe` and `ln Je`:
///     |dFe : A : dFe| <= (mu + (3 lambda + |mu - lambda ln Je|) / s_min^2) |dFe|^2
/// (docs/PHYSICS.md "Solids", R5). Infinite when `lower` is not a positive number.
fn bound_from(lower: f64, ln_je: f64, law: &Law) -> f64 {
    if lower.is_nan() || lower <= 0.0 {
        return f64::INFINITY;
    }
    law.mu + (3.0 * law.lambda + (law.mu - law.lambda * ln_je).abs()) / lower
}

/// `bound_from` for an elastic deformation gradient (its own eigen-solve). Infinite unless
/// `Je > 0`.
pub fn tangent_bound(fe: &Mat3, law: &Law) -> f64 {
    let je = det(fe);
    if !(je > 0.0 && je.is_finite()) {
        return f64::INFINITY;
    }
    bound_from(sym_eigen(&ata(fe)).lower, je.ln(), law)
}

/// One plastic update (R4b): the increment `d >= 0` of the plastic compression, the unit
/// direction `n` (the eigenvector of the smallest stretch `s1` of the trial `Fe`), the trial
/// principal Kirchhoff stress `tau1` (Pa), and the bracket of the solve: `g0 = g(0) < 0`,
/// `g_hi = g(d_hi)`. `capped` is true when `g(d_hi) < 0`, so `d = d_hi` and the rest of the
/// stress stays elastic (lock-up, or a flow that would take `s1` past 1).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Flow {
    pub d: f64,
    pub n: Vec3,
    pub s1: f64,
    pub tau1: f64,
    pub g0: f64,
    pub d_hi: f64,
    pub g_hi: f64,
    pub capped: bool,
}

/// The return mapping of a trial elastic `Fe` with plastic compression `e_c` so far, or None
/// when there is no flow (`tau1 + sigma_y(e_c) >= 0`, or `e_c` has reached `e_max`, or `Je` is
/// not a positive number). Its own eigen-solve; `SolverM5` runs the same `flow_from` on the
/// eigen-solve it also uses for the tangent bound.
pub fn return_map(fe: &Mat3, e_c: f64, law: &Law) -> Option<Flow> {
    let je = det(fe);
    if !(je > 0.0 && je.is_finite()) {
        return None;
    }
    flow_from(&sym_eigen(&ata(fe)), je.ln(), e_c, law)
}

fn flow_from(eig: &Eigen, ln_je: f64, e_c: f64, law: &Law) -> Option<Flow> {
    if e_c.is_nan() || e_c >= law.e_max {
        return None;
    }
    let mut k = 0;
    for i in 1..3 {
        if eig.values[i] < eig.values[k] {
            k = i;
        }
    }
    let l1 = eig.values[k];
    if l1.is_nan() || l1 <= 0.0 {
        return None;
    }
    let tau1 = law.mu * (l1 - 1.0) + law.lambda * ln_je;
    let g0 = tau1 + law.sigma_y(e_c);
    if g0.is_nan() || g0 >= 0.0 {
        return None;
    }
    let g = |d: f64| {
        law.mu * (l1 * (2.0 * d).exp() - 1.0) + law.lambda * (ln_je + d) + law.sigma_y(e_c + d)
    };
    // -ln s1 = -ln(l1) / 2 is the stretch back to 1; e_max - e_c the room left before lock-up.
    let d_hi = (-0.5 * l1.ln()).min(law.e_max - e_c);
    let g_hi = g(d_hi);
    let (d, capped) = if g_hi < 0.0 {
        (d_hi, true)
    } else {
        let (mut lo, mut hi) = (0.0, d_hi);
        for _ in 0..BISECTION_STEPS {
            let mid = 0.5 * (lo + hi);
            if g(mid) < 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        (hi, false)
    };
    let v = &eig.vectors;
    Some(Flow {
        d,
        n: Vec3::new(v[0][k], v[1][k], v[2][k]),
        s1: l1.sqrt(),
        tau1,
        g0,
        d_hi,
        g_hi,
        capped,
    })
}

/// `Fe' = Fe (I + (e^d - 1) n n^T)`: the elastic part after the flow (its smallest stretch
/// times `e^d`, `Je' = Je e^d`).
pub fn apply_flow(fe: &Mat3, flow: &Flow) -> Mat3 {
    add_outer(fe, flow.d.exp() - 1.0, mat_vec(fe, flow.n), flow.n)
}

// ---------------------------------------------------------------------------------------
// The mesh rule (R12)

/// The 6 tets of a hexahedral cell cut along its main diagonal, corner 0 to corner 7, the same
/// way for every cell, so a mesh of such cells is conforming across faces. `c[k]` is the node at
/// corner k, bit 0 of k the +x side, bit 1 the +y side, bit 2 the +z side. Tet j runs 0, then
/// one axis step, then two, then 7, for the axis orders xyz, xzy, yxz, yzx, zxy, zyx in that
/// order; for the odd orders the two middle nodes are swapped, so every tet of a cell whose
/// edges point along +x, +y and +z has a positive volume as the format requires.
pub fn cut_cell(c: [u32; 8]) -> [[u32; 4]; 6] {
    const ORDERS: [([usize; 3], bool); 6] = [
        ([1, 2, 4], true),
        ([1, 4, 2], false),
        ([2, 1, 4], false),
        ([2, 4, 1], true),
        ([4, 1, 2], true),
        ([4, 2, 1], false),
    ];
    let mut tets = [[0u32; 4]; 6];
    for (t, (axes, even)) in tets.iter_mut().zip(ORDERS) {
        let (p, q) = (axes[0], axes[0] | axes[1]);
        *t = if even {
            [c[0], c[p], c[q], c[7]]
        } else {
            [c[0], c[q], c[p], c[7]]
        };
    }
    tets
}

// ---------------------------------------------------------------------------------------
// The solver

/// One tet's constants and plastic state.
#[derive(Clone, Debug)]
struct TetEl {
    nodes: [usize; 4],
    solid: usize,
    /// Rest volume (m^3), > 0.
    volume: f64,
    /// Inverse of the rest edge matrix `[X1-X0, X2-X0, X3-X0]`: row k is the rest gradient of
    /// the shape function of node k + 1 (that of node 0 is minus their sum). `F = Ds dm_inv`.
    dm_inv: Mat3,
    /// `|grad N_a|` at rest, a = 0..3, and their sum (the damping part of the R5 bound).
    g_rest: [f64; 4],
    g_rest_sum: f64,
    /// `dm_inv Fp^-1`: `Fe = Ds b`, and row k is the gradient of node k + 1 in the plastic
    /// intermediate shape. Updated at every flow, `b' = b (I + (e^d - 1) n n^T)`.
    b: Mat3,
    /// The plastic part `Fp` (`F = Fe Fp`), `Fp' = (I + (e^-d - 1) n n^T) Fp`.
    fp: Mat3,
    /// Plastic compression so far (a log strain), 0 <= e_c <= e_max.
    e_c: f64,
    /// Viscous coefficient (Pa s) of the damping stress `eta D` (Q3).
    eta: f64,
}

/// The m5 solver (R1). Built by `SolverM5::from_body`; `Sim::from_body_with_solver` or
/// `Sim::from_car_with_solver` runs it on the same body.
pub struct SolverM5 {
    tets: Vec<TetEl>,
    laws: Vec<Law>,
    n_nodes: usize,
    /// The file's id of each node, in file order (for error messages).
    node_ids: Vec<u32>,
    h: f64,
    m_min: f64,
    eps_d: f64,
    /// Per node: twice the sum of the stiffnesses (and dampings) of the body's beams there.
    beam_k: Vec<f64>,
    beam_c: Vec<f64>,
    w2_rest: f64,
    g_rest: f64,
    /// The substep's working arrays: the summed forces, the velocities after the extra-force
    /// kick of `step_with_forces`, the new velocities and positions, and each tet's new plastic
    /// state `(b, Fp, e_c)`. They are checked before any of them reaches the state.
    force: Vec<Vec3>,
    kicked: Vec<Vec3>,
    new_vel: Vec<Vec3>,
    new_pos: Vec<Vec3>,
    pending: Vec<(Mat3, Mat3, f64)>,
    k_node: Vec<f64>,
    c_node: Vec<f64>,
    substep: u64,
    error: Option<String>,
}

/// The rest volume `(b - a) . ((c - a) x (d - a)) / 6` (docs/FORMAT.md), the longest of the six
/// edges, and the largest face area of four points.
fn tet_shape(p: [Vec3; 4]) -> (f64, f64, f64) {
    let vol = (p[1] - p[0]).dot((p[2] - p[0]).cross(p[3] - p[0])) / 6.0;
    let mut longest = 0.0f64;
    for (i, j) in [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)] {
        longest = longest.max((p[j] - p[i]).length());
    }
    let mut face = 0.0f64;
    for [i, j, k] in [[1, 2, 3], [0, 2, 3], [0, 1, 3], [0, 1, 2]] {
        face = face.max(0.5 * (p[j] - p[i]).cross(p[k] - p[i]).length());
    }
    (vol, longest, face)
}

impl SolverM5 {
    /// Build the solver for `body` at physics step `h` (s). Refused (Err, with a named reason)
    /// for, in this order: more tets than `M5_MAX_TETS` (before anything per tet is allocated);
    /// `h` not a positive finite number; a node whose position is not finite or has a
    /// coordinate beyond `MAX_COORD_M`, or whose mass is not a positive finite number; a node
    /// id given twice; a start velocity above `MAX_SPEED_MPS`; a beam on a missing node or
    /// naming a missing material; a solid with a name given twice or numbers outside the
    /// format's ranges; a tet naming a missing solid or node, listing a node twice, whose rest
    /// volume is not finite, below `TET_MIN_VOLUME`, flatter than `TET_MIN_SHAPE` or negative, or
    /// on the same four nodes as an earlier tet; a node shared by tets of two solids; a zone
    /// whose lock-up strain is not above a solid's yield strain; and a step at or above the
    /// stability bound of R5a.
    pub fn from_body(body: &Body, h: f64) -> Result<SolverM5, String> {
        if body.tets.len() > M5_MAX_TETS {
            return Err(format!(
                "the body has {} tets, more than the m5 solver takes (M5_MAX_TETS = {M5_MAX_TETS}, docs/M5-SOLVER.md `frozen: m5_max_tets`)",
                body.tets.len()
            ));
        }
        if !(h.is_finite() && h > 0.0) {
            return Err(format!(
                "physics step h = {h} s must be a positive finite number"
            ));
        }
        let n = body.nodes.len();
        for nd in &body.nodes {
            if !finite(nd.pos) {
                return Err(format!("node {}: its position is not finite", nd.id));
            }
            let p = nd.pos;
            if p.x.abs().max(p.y.abs()).max(p.z.abs()) > MAX_COORD_M {
                return Err(format!(
                    "node {}: its position ({:e}, {:e}, {:e}) is beyond MAX_COORD_M = {MAX_COORD_M} m in a coordinate",
                    nd.id, p.x, p.y, p.z
                ));
            }
            if !(nd.mass.is_finite() && nd.mass > 0.0) {
                return Err(format!(
                    "node {}: its mass {} must be a positive finite number",
                    nd.id, nd.mass
                ));
            }
        }
        // Node id -> index (file order) by a sorted list: no hash map, O(n log n).
        let mut ids: Vec<(u32, usize)> = body
            .nodes
            .iter()
            .enumerate()
            .map(|(i, nd)| (nd.id, i))
            .collect();
        ids.sort_unstable();
        if let Some(w) = ids.windows(2).find(|w| w[0].0 == w[1].0) {
            return Err(format!("node id {} is given twice", w[0].0));
        }
        for v in &body.velocities {
            let speed = v.velocity.length();
            if speed.is_nan() || speed > MAX_SPEED_MPS {
                return Err(format!(
                    "velocity of group `{}`: speed {speed:e} m/s is above MAX_SPEED_MPS = {MAX_SPEED_MPS} m/s (or not a number)",
                    v.group
                ));
            }
        }
        let index = |id: u32| {
            ids.binary_search_by_key(&id, |&(i, _)| i)
                .ok()
                .map(|k| ids[k].1)
        };

        let mut beam_k = vec![0.0f64; n];
        let mut beam_c = vec![0.0f64; n];
        for (i, b) in body.beams.iter().enumerate() {
            let (Some(a), Some(c)) = (index(b.a), index(b.b)) else {
                return Err(format!("beam {i} is on a missing node"));
            };
            let Some(m) = body.materials.iter().find(|m| m.name == b.material) else {
                return Err(format!(
                    "beam {i} names a missing material `{}`",
                    b.material
                ));
            };
            // The format's ranges, again for a body built in code: a NaN or negative number
            // here would silently weaken the stability bound.
            if !(m.stiffness.is_finite()
                && m.stiffness > 0.0
                && m.damping.is_finite()
                && m.damping >= 0.0)
            {
                return Err(format!(
                    "beam {i}: its material `{}` has stiffness {} and damping {}; they must be finite, stiffness > 0 and damping >= 0",
                    m.name, m.stiffness, m.damping
                ));
            }
            beam_k[a] += 2.0 * m.stiffness;
            beam_k[c] += 2.0 * m.stiffness;
            beam_c[a] += 2.0 * m.damping;
            beam_c[c] += 2.0 * m.damping;
        }

        for (i, s) in body.solids.iter().enumerate() {
            if body.solids[..i].iter().any(|o| o.name == s.name) {
                return Err(format!("solid `{}` is given twice", s.name));
            }
            if !(s.plateau_stress.is_finite() && s.plateau_stress > 0.0) {
                return Err(format!(
                    "solid `{}`: plateau_stress {} must be a positive finite number",
                    s.name, s.plateau_stress
                ));
            }
            if !(s.yield_strain > 0.0 && s.yield_strain < 1.0) {
                return Err(format!(
                    "solid `{}`: yield_strain {} must be > 0 and < 1",
                    s.name, s.yield_strain
                ));
            }
            if !(0.0..=1.0).contains(&s.damping_ratio) {
                return Err(format!(
                    "solid `{}`: damping_ratio {} must be >= 0 and <= 1",
                    s.name, s.damping_ratio
                ));
            }
            let e = s.plateau_stress / s.yield_strain;
            if !e.is_finite() {
                return Err(format!(
                    "solid `{}`: its elastic modulus E = plateau_stress / yield_strain = {e:e} Pa is not finite",
                    s.name
                ));
            }
        }

        // The tets: geometry and the checks of the format, again, for a body built in code.
        struct Raw {
            nodes: [usize; 4],
            solid: usize,
            volume: f64,
            face: f64,
            dm_inv: Mat3,
        }
        let mut raw: Vec<Raw> = Vec::with_capacity(body.tets.len());
        let mut sets: Vec<[usize; 4]> = Vec::with_capacity(body.tets.len());
        let mut node_solid: Vec<Option<usize>> = vec![None; n];
        for (t, tet) in body.tets.iter().enumerate() {
            let label = format!(
                "tet {t} ({} {} {} {})",
                tet.nodes[0], tet.nodes[1], tet.nodes[2], tet.nodes[3]
            );
            let Some(solid) = body.solids.iter().position(|s| s.name == tet.solid) else {
                return Err(format!("{label}: unknown solid `{}`", tet.solid));
            };
            let mut nodes = [0usize; 4];
            for k in 0..4 {
                let Some(i) = index(tet.nodes[k]) else {
                    return Err(format!("{label}: node {} is missing", tet.nodes[k]));
                };
                if nodes[..k].contains(&i) {
                    return Err(format!("{label}: lists node {} twice", tet.nodes[k]));
                }
                nodes[k] = i;
            }
            let p = nodes.map(|i| body.nodes[i].pos);
            let (volume, longest, face) = tet_shape(p);
            if !(volume.is_finite() && longest.is_finite() && face.is_finite()) {
                return Err(format!("{label}: its size is not finite"));
            }
            if volume.abs() < TET_MIN_VOLUME {
                return Err(format!("{label}: its rest volume {volume} is below the minimum {TET_MIN_VOLUME} m^3 (zero volume, or coplanar or collinear nodes)"));
            }
            if volume.abs() / (longest * longest * longest) < TET_MIN_SHAPE {
                return Err(format!("{label} is too flat: its rest volume {volume} is below {TET_MIN_SHAPE} times the cube of its longest edge {longest}"));
            }
            if volume < 0.0 {
                return Err(format!(
                    "{label} is inverted: its rest volume {volume} is negative"
                ));
            }
            let mut set = nodes;
            set.sort_unstable();
            if let Some(o) = sets.iter().position(|s| *s == set) {
                return Err(format!("{label} is on the same four nodes as tet {o}"));
            }
            sets.push(set);
            for &i in &nodes {
                match node_solid[i] {
                    Some(s) if s != solid => {
                        return Err(format!(
                            "{label}: node {} is shared by tets of two solids, `{}` and `{}`",
                            body.nodes[i].id, body.solids[s].name, body.solids[solid].name
                        ));
                    }
                    _ => node_solid[i] = Some(solid),
                }
            }
            let dm = from_cols(p[1] - p[0], p[2] - p[0], p[3] - p[0]);
            let cof = cofactor(&dm);
            let dd = det(&dm);
            let mut dm_inv = [[0.0; 3]; 3];
            for i in 0..3 {
                for j in 0..3 {
                    dm_inv[i][j] = cof[j][i] / dd;
                }
            }
            raw.push(Raw {
                nodes,
                solid,
                volume,
                face,
                dm_inv,
            });
        }

        // The zone (the frozen lock-up line): the nodes of at least one tet, and the tets.
        let zone_mass: f64 = (0..n)
            .filter(|&i| node_solid[i].is_some())
            .map(|i| body.nodes[i].mass)
            .sum();
        let zone_volume: f64 = raw.iter().map(|r| r.volume).sum();
        let eps_d = if raw.is_empty() {
            LOCKUP_CAP
        } else {
            lockup_strain(zone_mass, zone_volume)
        };
        let mut laws = Vec::with_capacity(body.solids.len());
        for s in &body.solids {
            if !raw.is_empty() && (eps_d.is_nan() || eps_d <= s.yield_strain) {
                return Err(format!(
                    "solid `{}`: the zone's lock-up strain eps_D = {eps_d} (zone mass {zone_mass} kg, zone volume {zone_volume} m^3) is not above its yield_strain {}, so it could never crush",
                    s.name, s.yield_strain
                ));
            }
            laws.push(Law::new(s.plateau_stress, s.yield_strain, eps_d));
        }
        let rho = if raw.is_empty() {
            0.0
        } else {
            zone_mass / zone_volume
        };

        let m_min = body
            .nodes
            .iter()
            .map(|nd| nd.mass)
            .fold(f64::INFINITY, f64::min);
        let mut k_at = beam_k.clone();
        let mut c_at = beam_c.clone();
        let mut tets = Vec::with_capacity(raw.len());
        for (ti, r) in raw.into_iter().enumerate() {
            let law = &laws[r.solid];
            let zeta = body.solids[r.solid].damping_ratio;
            // Q3: the critical damping of a bar of the tet's smallest altitude 3 V / (largest
            // face), with the dilatational wave speed of the zone's mean density.
            let alt = 3.0 * r.volume / r.face;
            let modulus = law.lambda + 2.0 * law.mu;
            let eta = if zeta > 0.0 {
                2.0 * zeta * alt * (modulus * rho).sqrt()
            } else {
                0.0
            };
            if !eta.is_finite() {
                return Err(format!(
                    "tet {ti}: its damping coefficient eta = 2 zeta L sqrt((lambda + 2 mu) rho) = {eta:e} Pa s is not finite"
                ));
            }
            let rows = [row(&r.dm_inv, 0), row(&r.dm_inv, 1), row(&r.dm_inv, 2)];
            let g0 = -(rows[0] + rows[1] + rows[2]);
            let g_rest = [
                g0.length(),
                rows[0].length(),
                rows[1].length(),
                rows[2].length(),
            ];
            let g_rest_sum = g_rest.iter().sum::<f64>();
            // R5a: the tangent at rest, 2 mu + 3 lambda (bound_from at Fe = I), times the headroom.
            let a_rest = STIFFNESS_HEADROOM * (2.0 * law.mu + 3.0 * law.lambda);
            for (a, &node) in r.nodes.iter().enumerate() {
                k_at[node] += r.volume * a_rest * g_rest[a] * g_rest_sum;
                c_at[node] += r.volume * eta * g_rest[a] * g_rest_sum;
            }
            let identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
            tets.push(TetEl {
                nodes: r.nodes,
                solid: r.solid,
                volume: r.volume,
                dm_inv: r.dm_inv,
                g_rest,
                g_rest_sum,
                b: r.dm_inv,
                fp: identity,
                e_c: 0.0,
                eta,
            });
        }
        // Every node's sums, not only the largest: `f64::max` would drop a NaN silently.
        if let Some(i) = (0..n).find(|&i| !(k_at[i].is_finite() && c_at[i].is_finite())) {
            return Err(format!(
                "node {}: its m5 stability sums (stiffness {:e}, damping {:e}) are not finite",
                body.nodes[i].id, k_at[i], c_at[i]
            ));
        }
        let w2 = k_at.iter().cloned().fold(0.0, f64::max) / m_min;
        let g = c_at.iter().cloned().fold(0.0, f64::max) / m_min;
        if !(w2.is_finite() && g.is_finite()) {
            return Err(format!(
                "the m5 stability bound is not finite: w2 = {w2:e}, g = {g:e} (the lightest node is {m_min:e} kg)"
            ));
        }
        let x = h * h * w2 + 2.0 * h * g;
        if x.is_nan() || x >= 4.0 {
            let limit = 4.0 / (g + (g * g + 4.0 * w2).sqrt());
            return Err(format!(
                "physics step h = dt/substeps = {h:.6} s reaches the m5 stability limit {limit:.6} s (h^2*w2 + 2*h*g < 4 with the Gershgorin bounds w2 = {w2}, g = {g}, the tet tangent at rest times {STIFFNESS_HEADROOM}); raise --substeps or lower --dt"
            ));
        }
        Ok(SolverM5 {
            tets,
            laws,
            n_nodes: n,
            node_ids: body.nodes.iter().map(|nd| nd.id).collect(),
            h,
            m_min,
            eps_d,
            beam_k,
            beam_c,
            w2_rest: w2,
            g_rest: g,
            force: Vec::with_capacity(n),
            kicked: Vec::with_capacity(n),
            new_vel: Vec::with_capacity(n),
            new_pos: Vec::with_capacity(n),
            pending: Vec::new(),
            k_node: vec![0.0; n],
            c_node: vec![0.0; n],
            substep: 0,
            error: None,
        })
    }

    /// The rest-state Gershgorin bound (rad/s) on the highest frequency, with the tet tangent
    /// times `STIFFNESS_HEADROOM`, ignoring damping: `sqrt(w2)` of R5a.
    pub fn omega_max(&self) -> f64 {
        self.w2_rest.sqrt()
    }

    /// The largest step (s, exclusive) R5a accepts: the positive root of `h^2 w2 + 2 h g = 4`.
    pub fn step_limit(&self) -> f64 {
        let (w2, g) = (self.w2_rest, self.g_rest);
        4.0 / (g + (g * g + 4.0 * w2).sqrt())
    }

    /// The zone's lock-up strain `eps_D`.
    pub fn eps_d(&self) -> f64 {
        self.eps_d
    }

    /// The law of solid `s` (index in `Body::solids`).
    pub fn law(&self, s: usize) -> Law {
        self.laws[s]
    }

    /// How many tets the solver holds.
    pub fn tet_count(&self) -> usize {
        self.tets.len()
    }

    /// The plastic compression `e_c` of tet `t`.
    pub fn plastic_strain(&self, t: usize) -> f64 {
        self.tets[t].e_c
    }

    /// The plastic part `Fp` of tet `t`.
    pub fn fp(&self, t: usize) -> Mat3 {
        self.tets[t].fp
    }

    /// How many substeps `step` has run.
    pub fn substeps(&self) -> u64 {
        self.substep
    }

    /// The elastic energy (J) of the tets at positions `pos`, `sum V W(Fe)`, with each tet's
    /// current plastic state; infinite if a tet's `Je` is not positive. Changes nothing.
    pub fn elastic_energy(&self, pos: &[Vec3]) -> f64 {
        let mut e = 0.0;
        for t in &self.tets {
            let fe = self.trial_fe(t, pos);
            e += t.volume * energy_density(&fe, &self.laws[t.solid]);
        }
        e
    }

    fn trial_fe(&self, t: &TetEl, pos: &[Vec3]) -> Mat3 {
        let x0 = pos[t.nodes[0]];
        let d = [
            pos[t.nodes[1]] - x0,
            pos[t.nodes[2]] - x0,
            pos[t.nodes[3]] - x0,
        ];
        cols_times(d, &t.b)
    }

    /// The tet forces (N) alone at `pos` and `vel`, one per node, as `step` computes them;
    /// errors are recorded as in `step` (at the current substep count) and returned by
    /// `Solver::check`. The plastic update is kept only when every force is finite (as in
    /// `step`, which also needs the rest of the substep to be finite). For quasi-static tests.
    pub fn tet_forces(&mut self, pos: &[Vec3], vel: &[Vec3]) -> Vec<Vec3> {
        let mut f = vec![Vec3::ZERO; pos.len()];
        let h = self.h;
        self.add_tet_forces(pos, vel, &mut f);
        if self.check_forces(&f, "tets") {
            self.stiffness_verdict(h);
        }
        // The same gate as `step_inner`.
        if !self.failed() {
            self.commit_tets();
        }
        f
    }

    /// Records the first error. Every error of a substep goes through here, and `failed`
    /// (the gate before anything is applied) reads what it sets.
    fn fail(&mut self, msg: String) {
        if self.error.is_none() {
            self.error = Some(msg);
        }
    }

    /// True once any error has been recorded: from then on no substep applies anything (the
    /// run ends at the next `Sim::check`).
    fn failed(&self) -> bool {
        self.error.is_some()
    }

    /// True if every entry of the working force array is finite; else records
    /// `force on node <id> is not finite at substep <n> (from <source>)` for the first one.
    fn check_forces(&mut self, force: &[Vec3], source: &str) -> bool {
        match force.iter().position(|&f| !finite(f)) {
            None => true,
            Some(i) => {
                let n = self.substep;
                self.fail(format!(
                    "force on node {} is not finite at substep {n} (from {source})",
                    self.node_ids[i]
                ));
                false
            }
        }
    }

    /// Keeps the plastic state `add_tet_forces` computed for this substep.
    fn commit_tets(&mut self) {
        for (t, &(b, fp, e_c)) in self.tets.iter_mut().zip(&self.pending) {
            t.b = b;
            t.fp = fp;
            t.e_c = e_c;
        }
    }

    /// R5b: the Gershgorin sums `add_tet_forces` left in `k_node` and `c_node`, at step `h`.
    fn stiffness_verdict(&mut self, h: f64) {
        let n = self.substep;
        let kmax = self.k_node.iter().cloned().fold(0.0, f64::max);
        let cmax = self.c_node.iter().cloned().fold(0.0, f64::max);
        let w2 = kmax / self.m_min;
        let g = cmax / self.m_min;
        let x = h * h * w2 + 2.0 * h * g;
        // NaN or infinity fails too.
        if x.is_nan() || x >= 4.0 {
            self.fail(format!(
                "stiffness bound exceeded at substep {n}: h omega = {}",
                x.sqrt()
            ));
        }
    }

    /// Adds every tet's force into `force`, in tet index order, and leaves each tet's new
    /// plastic state in `pending` and its share of the R5b sums in `k_node` and `c_node`;
    /// nothing of the tets' own state changes here (`commit_tets` keeps it).
    fn add_tet_forces(&mut self, pos: &[Vec3], vel: &[Vec3], force: &mut [Vec3]) {
        let n = self.substep;
        self.k_node.copy_from_slice(&self.beam_k);
        self.c_node.copy_from_slice(&self.beam_c);
        self.pending.clear();
        self.pending
            .extend(self.tets.iter().map(|t| (t.b, t.fp, t.e_c)));
        for ti in 0..self.tets.len() {
            let t = &self.tets[ti];
            let law = self.laws[t.solid];
            let x0 = pos[t.nodes[0]];
            let d = [
                pos[t.nodes[1]] - x0,
                pos[t.nodes[2]] - x0,
                pos[t.nodes[3]] - x0,
            ];
            let fe_trial = cols_times(d, &t.b);
            let je_trial = det(&fe_trial);
            // R4e: the inverted-tet rule, on the trial Fe, before anything else.
            if !(je_trial.is_finite() && je_trial > J_MIN_FAIL) {
                self.fail(format!(
                    "tet {ti} inverted or collapsed at substep {n}: Je = {je_trial}"
                ));
                continue;
            }
            let eig = sym_eigen(&ata(&fe_trial));
            let mut ln_je = je_trial.ln();
            let (mut fe, mut b, mut fp, mut e_c, mut je) = (fe_trial, t.b, t.fp, t.e_c, je_trial);
            if let Some(flow) = flow_from(&eig, ln_je, t.e_c, &law) {
                // Fe' = Fe (I + (e^d - 1) n n^T) and b' = b (I + (e^d - 1) n n^T): the same map.
                fe = apply_flow(&fe, &flow);
                b = apply_flow(&b, &flow);
                // (I + (e^-d - 1) n n^T) Fp = Fp + (e^-d - 1) n (Fp^T n)^T
                let fpt_n = Vec3::new(
                    fp[0][0] * flow.n.x + fp[1][0] * flow.n.y + fp[2][0] * flow.n.z,
                    fp[0][1] * flow.n.x + fp[1][1] * flow.n.y + fp[2][1] * flow.n.z,
                    fp[0][2] * flow.n.x + fp[1][2] * flow.n.y + fp[2][2] * flow.n.z,
                );
                fp = add_outer(&fp, (-flow.d).exp() - 1.0, flow.n, fpt_n);
                e_c += flow.d;
                ln_je += flow.d;
                je *= flow.d.exp();
            }
            let p = piola_from(&fe, je, ln_je, &law);
            let rows = [row(&b, 0), row(&b, 1), row(&b, 2)];
            let v = t.volume;
            let mut f = [Vec3::ZERO; 4];
            for k in 0..3 {
                f[k + 1] = mat_vec(&p, rows[k]) * (-v);
            }
            if t.eta > 0.0 {
                // Q3: sigma_v = eta D with D = sym(Fdot F^-1) of the total F; P_v = J sigma_v F^-T
                // = sigma_v cof(F); its rows act through the rest gradients (rows of dm_inv).
                let v0 = vel[t.nodes[0]];
                let dv = [
                    vel[t.nodes[1]] - v0,
                    vel[t.nodes[2]] - v0,
                    vel[t.nodes[3]] - v0,
                ];
                let ft = cols_times(d, &t.dm_inv);
                let fdot = cols_times(dv, &t.dm_inv);
                let cof = cofactor(&ft);
                let jt = det(&ft);
                let mut l = [[0.0; 3]; 3];
                for i in 0..3 {
                    for j in 0..3 {
                        // F^-1 = cof^T / J, so (Fdot F^-1)_ij = sum_k Fdot_ik cof_jk / J
                        l[i][j] = (fdot[i][0] * cof[j][0]
                            + fdot[i][1] * cof[j][1]
                            + fdot[i][2] * cof[j][2])
                            / jt;
                    }
                }
                let mut sig = [[0.0; 3]; 3];
                for i in 0..3 {
                    for j in 0..3 {
                        sig[i][j] = t.eta * 0.5 * (l[i][j] + l[j][i]);
                    }
                }
                let pv = mul(&sig, &cof);
                for k in 0..3 {
                    f[k + 1] += mat_vec(&pv, row(&t.dm_inv, k)) * (-v);
                }
            }
            f[0] = -(f[1] + f[2] + f[3]);
            // A non-finite force goes into the working array, where the caller's check of the
            // whole array finds it before anything is applied.
            // R5b: this tet's share of the Gershgorin sums, from the current state.
            let a_bound = bound_from(eig.lower, ln_je, &law);
            let gn = [
                (rows[0] + rows[1] + rows[2]).length(),
                rows[0].length(),
                rows[1].length(),
                rows[2].length(),
            ];
            let gsum: f64 = gn.iter().sum();
            // J |F^-1|^2 <= Je e^-e_c e^(2 e_c) / s_min(Fe)^2 (|Fp^-1| <= e^e_c).
            let c_scale = if t.eta > 0.0 {
                t.eta * je * e_c.exp() / eig.lower
            } else {
                0.0
            };
            let nodes = t.nodes;
            for a in 0..4 {
                self.k_node[nodes[a]] += v * a_bound * gn[a] * gsum;
                if t.eta > 0.0 {
                    self.c_node[nodes[a]] += v * c_scale * t.g_rest[a] * t.g_rest_sum;
                }
            }
            // Checked with the sums it would make before it is written (the four nodes of a
            // tet are distinct, so each sum reads the array as it is now).
            let sums = [0, 1, 2, 3].map(|a| force[nodes[a]] + f[a]);
            if let Some(a) = (0..4).find(|&a| !(finite(f[a]) && finite(sums[a]))) {
                self.fail(format!(
                    "force on node {} is not finite at substep {n} (from tets)",
                    self.node_ids[nodes[a]]
                ));
                return;
            }
            for a in 0..4 {
                force[nodes[a]] = sums[a];
            }
            self.pending[ti] = (b, fp, e_c);
        }
    }

    /// One substep, `extra` the per-node extra forces of `step_with_forces` (the trait's
    /// operator splitting: velocities first get `extra * h / m`, then the step runs on them).
    /// Everything is computed into the working arrays first: the kicked velocities, the force
    /// array (gravity, then the beams with the arithmetic and order of `SymplecticEuler`, then
    /// the tets in index order, damping included), the new velocities and positions, and the
    /// tets' new plastic state. Each is checked for finiteness before the next uses it, and the
    /// state and the tets are written only when all of them passed: on the first non-finite
    /// value a named error is recorded and nothing of the substep is applied. So no non-finite
    /// value from any source reaches the force that is applied, nor the state.
    fn step_inner(&mut self, s: State<'_>, extra: Option<&[Vec3]>, h: f64) {
        let State {
            pos,
            vel,
            beams,
            mass,
            gravity,
        } = s;
        self.substep += 1;
        let n = self.substep;
        let nn = self.n_nodes;
        if pos.len() != nn
            || vel.len() != nn
            || mass.len() != nn
            || extra.is_some_and(|e| e.len() != nn)
        {
            self.fail(format!(
                "the m5 solver was built for {nn} nodes but substep {n} has {} (a different body)",
                pos.len()
            ));
            return;
        }
        if h != self.h {
            self.fail(format!(
                "the m5 solver was built for h = {} s but substep {n} has h = {h} s",
                self.h
            ));
            return;
        }
        // The state itself first: a non-finite node would spread through every force.
        if let Some(i) = (0..pos.len()).find(|&i| !(finite(pos[i]) && finite(vel[i]))) {
            self.fail(format!(
                "node {} is not finite at substep {n}",
                self.node_ids[i]
            ));
            return;
        }
        let mut kicked = std::mem::take(&mut self.kicked);
        kicked.clear();
        if let Some(extra) = extra {
            if !self.check_forces(extra, "extra forces") {
                return;
            }
            kicked.extend((0..nn).map(|i| vel[i] + extra[i] * (h / mass[i])));
            if let Some(i) = kicked.iter().position(|&v| !finite(v)) {
                self.fail(format!(
                    "update of node {} is not finite at substep {n}",
                    self.node_ids[i]
                ));
                return;
            }
        }
        let v: &[Vec3] = if extra.is_some() { &kicked } else { vel };
        let mut force = std::mem::take(&mut self.force);
        force.clear();
        // Each contribution is computed in a local value and checked, with the sum it would
        // make, before it is written: the working array never holds a non-finite value (the
        // `debug_assert!`s check that; `check_forces` stays as the backstop).
        for (i, &m) in mass.iter().enumerate() {
            let g = Vec3::new(0.0, -gravity * m, 0.0);
            if !finite(g) {
                self.fail(format!(
                    "force on node {} is not finite at substep {n} (from gravity)",
                    self.node_ids[i]
                ));
                return;
            }
            force.push(g);
        }
        debug_assert!(
            force.iter().all(|&f| finite(f)),
            "a non-finite value is in the force array (gravity)"
        );
        if !self.check_forces(&force, "gravity") {
            return;
        }
        for bm in beams {
            let d = pos[bm.b] - pos[bm.a];
            let len = d.length();
            if len < 1e-12 {
                continue;
            }
            let dir = d * (1.0 / len);
            let rel_v = (v[bm.b] - v[bm.a]).dot(dir);
            let mag = bm.stiffness * (len - bm.rest) + bm.damping * rel_v;
            let f = dir * mag;
            let (fa, fb) = (force[bm.a] + f, force[bm.b] + -f);
            if !(finite(f) && finite(fa) && finite(fb)) {
                let at = if finite(f) && finite(fa) { bm.b } else { bm.a };
                self.fail(format!(
                    "force on node {} is not finite at substep {n} (from beams)",
                    self.node_ids[at]
                ));
                return;
            }
            force[bm.a] = fa;
            force[bm.b] = fb;
        }
        debug_assert!(
            force.iter().all(|&f| finite(f)),
            "a non-finite value is in the force array (beams)"
        );
        if !self.check_forces(&force, "beams") {
            return;
        }
        if !self.tets.is_empty() {
            self.add_tet_forces(pos, v, &mut force);
            debug_assert!(
                force.iter().all(|&f| finite(f)),
                "a non-finite value is in the force array (tets)"
            );
            if !self.check_forces(&force, "tets") {
                return;
            }
        }
        // semi-implicit Euler: velocity first, then position with the new velocity
        let (mut nv, mut np) = (
            std::mem::take(&mut self.new_vel),
            std::mem::take(&mut self.new_pos),
        );
        nv.clear();
        np.clear();
        for i in 0..nn {
            let vi = v[i] + force[i] * (h / mass[i]);
            nv.push(vi);
            np.push(pos[i] + vi * h);
        }
        if let Some(i) = (0..nn).find(|&i| !(finite(nv[i]) && finite(np[i]))) {
            self.fail(format!(
                "update of node {} is not finite at substep {n}",
                self.node_ids[i]
            ));
            return;
        }
        // The R4e rule again on the positions about to be written, with the plastic state about
        // to be kept: an apex that crosses its face within this substep is caught here, before
        // `Sim::check` could see an inverted tet.
        for ti in 0..self.tets.len() {
            let t = &self.tets[ti];
            let x0 = np[t.nodes[0]];
            let d = [
                np[t.nodes[1]] - x0,
                np[t.nodes[2]] - x0,
                np[t.nodes[3]] - x0,
            ];
            let je = det(&cols_times(d, &self.pending[ti].0));
            if !(je.is_finite() && je > J_MIN_FAIL) {
                self.fail(format!(
                    "tet {ti} inverted or collapsed at the updated positions of substep {n}: Je = {je}"
                ));
                break;
            }
        }
        if !self.tets.is_empty() {
            self.stiffness_verdict(h);
        }
        // The one gate: every error path above (and any later one) goes through `fail`, so if
        // an error is recorded, in this substep or before, nothing is applied.
        if self.failed() {
            return;
        }
        self.commit_tets();
        vel.copy_from_slice(&nv);
        pos.copy_from_slice(&np);
        self.force = force;
        self.kicked = kicked;
        self.new_vel = nv;
        self.new_pos = np;
    }
}

impl Solver for SolverM5 {
    /// One substep (`step_inner`): the working force array is checked before it is applied.
    fn step(&mut self, s: State<'_>, h: f64) {
        self.step_inner(s, None, h);
    }

    /// The trait's operator splitting, with the same checks: the extra forces and the kicked
    /// velocities are checked too, and nothing is applied unless the whole substep is finite.
    fn step_with_forces(&mut self, s: State<'_>, extra: &[Vec3], h: f64) {
        self.step_inner(s, Some(extra), h);
    }

    fn handles_solids(&self) -> bool {
        true
    }

    fn check(&self) -> Option<String> {
        self.error.clone()
    }
}
