//! CSR storage, SpMV and a Jacobi-preconditioned conjugate-gradient solve, for the implicit
//! node-beam solver (`solver_implicit`, docs/M5-SOLVER.md option (1)).
//!
//! **A copy, not new maths.** This is `projects/numerics/src/sparse.rs` (numerics main at
//! 8132cfa, reviewed as brief N6) copied into crash-sim, because a path dependency on
//! `../numerics` does not resolve inside a task worktree (docs/M5-SOLVER.md section 6) and
//! ADR 0001's copy policy applies (as with `Vec3`). Lazlo chose option (1) with this copy on
//! 2026-11-22 19:50:55Z. The differences from the numerics module are only at the edges:
//! triplets are one slice of `(row, column, value)` tuples, the errors are this module's
//! `SparseError` (numerics' `ShapeMismatch` is `Shape`, its `BadParam` on an index is
//! `BadIndex`), and the 2-norm helper is copied in. The algorithm, its order of operations,
//! its power-of-two scaling and its checks are unchanged.
//!
//! **Determinism.** No threads, no hash maps. Triplets are ordered by (row, column, input
//! position) with a stable sort, and duplicates are summed in that order. Every row sum, dot
//! product and norm runs in storage (index) order. The same input gives the same `f64` bits.
//!
//! **Bad input.** Everything that can fail returns a [`SparseError`]; nothing panics. [`Csr`]
//! keeps its fields private, so a `Csr` always has finite entries, columns strictly increasing
//! within a row, indices in range and a consistent `indptr`.
//!
//! **Range of PCG.** `pcg` scales `A` and `b` by exact powers of two before iterating (and
//! scales the answer back), so entries from the subnormals up to near `f64::MAX` do not
//! overflow or underflow inside the dot products. An answer that does not fit in `f64` is
//! `NonFinite { what: "solution" }`. Known limits (from the numerics module's own docs), which
//! are errors, not wrong answers: a diagonal that spans more than the `f64` range after scaling
//! (for example `diag(1e300, 1e-10)`) gives `NonFinite { what: "inverse diagonal" }`, and a
//! start vector whose scaled copy overflows gives `NonFinite { what: "x0" }`. `A` is not checked
//! for symmetry; an indefinite `A` is found only if `p^T A p <= 0` occurs (`Domain`).

use std::fmt;

/// Why a sparse operation refused its input or did not finish.
#[derive(Debug, Clone, PartialEq)]
pub enum SparseError {
    /// A square matrix was required.
    NotSquare,
    /// A vector (or the triplet list) has the wrong length.
    Shape { expected: usize, got: usize },
    /// Zero rows or zero columns.
    Empty,
    /// A NaN or infinity in the input or result named by `what`.
    NonFinite { what: &'static str },
    /// A triplet's row or column is out of range.
    BadIndex,
    /// A parameter named `name` is out of its allowed range (or a size cannot be allocated).
    BadParam { name: &'static str },
    /// The diagonal entry of row `row` is zero, negative or not stored.
    NonPositiveDiagonal { row: usize },
    /// `max_iter` iterations did not reach the tolerance; `residual` is the last recurrence
    /// residual. Never a result.
    NotConverged { iterations: usize, residual: f64 },
    /// `p^T A p <= 0`: the matrix is not positive definite.
    Domain,
}

impl fmt::Display for SparseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SparseError::NotSquare => write!(f, "matrix is not square"),
            SparseError::Shape { expected, got } => write!(f, "length mismatch: expected {expected}, got {got}"),
            SparseError::Empty => write!(f, "empty matrix"),
            SparseError::NonFinite { what } => write!(f, "non-finite value in {what}"),
            SparseError::BadIndex => write!(f, "triplet index out of range"),
            SparseError::BadParam { name } => write!(f, "bad parameter: {name}"),
            SparseError::NonPositiveDiagonal { row } => write!(f, "non-positive diagonal entry in row {row}"),
            SparseError::NotConverged { iterations, residual } => {
                write!(f, "pcg not converged after {iterations} iterations (residual {residual:e})")
            }
            SparseError::Domain => write!(f, "pcg: p^T A p is not positive (matrix not positive definite)"),
        }
    }
}

/// Compressed sparse row matrix of `f64`.
#[derive(Debug, Clone, PartialEq)]
pub struct Csr {
    nrows: usize,
    ncols: usize,
    indptr: Vec<usize>,
    indices: Vec<usize>,
    data: Vec<f64>,
}

fn all_finite(x: &[f64]) -> bool {
    x.iter().all(|v| v.is_finite())
}

fn norm_inf_vec(x: &[f64]) -> f64 {
    let mut m: f64 = 0.0;
    for &v in x {
        if v.is_nan() {
            return f64::NAN;
        }
        m = m.max(v.abs());
    }
    m
}

/// 2-norm, scaled by the largest entry so it does not overflow (numerics `tol::norm2_vec`).
fn norm2_vec(x: &[f64]) -> f64 {
    let m = norm_inf_vec(x);
    if m == 0.0 || !m.is_finite() {
        return m;
    }
    m * x.iter().map(|v| (v / m) * (v / m)).sum::<f64>().sqrt()
}

impl Csr {
    /// Builds a CSR matrix from `(row, column, value)` triplets. Duplicates are summed, in the
    /// order (row, column, input position). Explicit zeros are kept.
    /// Errors: `Empty` (zero rows or columns), `BadIndex` (an index out of range), `BadParam`
    /// (a size too large to allocate), `NonFinite` (a NaN or infinite value, or a duplicate sum
    /// that overflows).
    pub fn from_triplets(nrows: usize, ncols: usize, t: &[(usize, usize, f64)]) -> Result<Csr, SparseError> {
        if nrows == 0 || ncols == 0 {
            return Err(SparseError::Empty);
        }
        if t.iter().any(|&(r, c, _)| r >= nrows || c >= ncols) {
            return Err(SparseError::BadIndex);
        }
        if t.iter().any(|&(_, _, v)| !v.is_finite()) {
            return Err(SparseError::NonFinite { what: "triplet values" });
        }
        let ptr_len = nrows.checked_add(1).ok_or(SparseError::BadParam { name: "nrows" })?;
        let mut indptr: Vec<usize> = Vec::new();
        indptr.try_reserve_exact(ptr_len).map_err(|_| SparseError::BadParam { name: "nrows" })?;
        let mut order: Vec<usize> = (0..t.len()).collect();
        // Stable sort: equal (row, column) keep their input order, so the sums below have a fixed order.
        order.sort_by_key(|&k| (t[k].0, t[k].1));
        indptr.resize(ptr_len, 0);
        let mut indices = Vec::with_capacity(order.len());
        let mut data: Vec<f64> = Vec::with_capacity(order.len());
        let mut last: Option<(usize, usize)> = None;
        for &k in &order {
            let (r, c, v) = t[k];
            let key = (r, c);
            if last == Some(key) {
                if let Some(acc) = data.last_mut() {
                    *acc += v;
                    if !acc.is_finite() {
                        return Err(SparseError::NonFinite { what: "summed duplicate entry" });
                    }
                }
            } else {
                indices.push(c);
                data.push(v);
                indptr[r + 1] += 1;
                last = Some(key);
            }
        }
        for i in 0..nrows {
            indptr[i + 1] += indptr[i];
        }
        Ok(Csr { nrows, ncols, indptr, indices, data })
    }

    pub fn nrows(&self) -> usize {
        self.nrows
    }
    pub fn ncols(&self) -> usize {
        self.ncols
    }
    /// Number of stored entries (duplicates counted once).
    pub fn nnz(&self) -> usize {
        self.data.len()
    }

    fn mul_into(&self, x: &[f64], out: &mut [f64]) {
        for (i, o) in out.iter_mut().enumerate() {
            let mut s = 0.0;
            for k in self.indptr[i]..self.indptr[i + 1] {
                s += self.data[k] * x[self.indices[k]];
            }
            *o = s;
        }
    }

    /// `A x`. Errors: `Shape` if `x.len() != ncols`, `NonFinite` for a NaN or infinite `x`, or
    /// a product entry that overflows.
    pub fn mul_vec(&self, x: &[f64]) -> Result<Vec<f64>, SparseError> {
        if x.len() != self.ncols {
            return Err(SparseError::Shape { expected: self.ncols, got: x.len() });
        }
        if !all_finite(x) {
            return Err(SparseError::NonFinite { what: "x" });
        }
        let mut y = vec![0.0; self.nrows];
        self.mul_into(x, &mut y);
        if !all_finite(&y) {
            return Err(SparseError::NonFinite { what: "product" });
        }
        Ok(y)
    }

    /// The diagonal, length `min(nrows, ncols)`; an entry that is not stored is `0.0`.
    pub fn diag(&self) -> Vec<f64> {
        (0..self.nrows.min(self.ncols))
            .map(|i| {
                let cols = &self.indices[self.indptr[i]..self.indptr[i + 1]];
                match cols.binary_search(&i) {
                    Ok(p) => self.data[self.indptr[i] + p],
                    Err(_) => 0.0,
                }
            })
            .collect()
    }
}

/// `2^k` for `k` in `-1022..=1023`, exact.
fn pow2(k: i32) -> f64 {
    f64::from_bits(((1023 + k) as u64) << 52)
}

/// `x * 2^k` in steps that stay inside the exponent range.
fn mul_pow2(mut x: f64, mut k: i32) -> f64 {
    while k != 0 {
        let s = k.clamp(-500, 500);
        x *= pow2(s);
        k -= s;
    }
    x
}

/// Binary exponent of a finite `m > 0`: `2^e <= m < 2^(e+1)`.
fn exp2_of(m: f64) -> i32 {
    let (m, adj) = if m < f64::MIN_POSITIVE { (m * pow2(64), 64) } else { (m, 0) };
    ((m.to_bits() >> 52) & 0x7ff) as i32 - 1023 - adj
}

fn max_abs(x: &[f64]) -> f64 {
    x.iter().fold(0.0, |m, v| if v.abs() > m { v.abs() } else { m })
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    let mut s = 0.0;
    for i in 0..a.len() {
        s += a[i] * b[i];
    }
    s
}

/// Solves `A x = b` for symmetric positive definite `A` by conjugate gradients with the Jacobi
/// (diagonal) preconditioner. Stops when the true relative residual `||b - A x||_2 / ||b||_2 <=
/// rtol`; returns `(x, iterations, that residual)`. A zero `b` gives `(0, 0, 0.0)`. `x0` is the
/// start vector (zeros if `None`).
///
/// Errors: `NotSquare`; `Shape` (`b` or `x0` length); `BadParam` (`rtol` not finite or not
/// positive, `max_iter` 0); `NonFinite` (NaN or infinity in `b` or `x0`, an overflowing iterate
/// or solution); `NonPositiveDiagonal { row }`; `Domain` (`p^T A p <= 0`);
/// `NotConverged { iterations, residual }` when `max_iter` steps did not reach `rtol`.
pub fn pcg(
    a: &Csr,
    b: &[f64],
    x0: Option<&[f64]>,
    rtol: f64,
    max_iter: usize,
) -> Result<(Vec<f64>, usize, f64), SparseError> {
    if a.nrows != a.ncols {
        return Err(SparseError::NotSquare);
    }
    let n = a.nrows;
    if b.len() != n {
        return Err(SparseError::Shape { expected: n, got: b.len() });
    }
    if let Some(x0) = x0 {
        if x0.len() != n {
            return Err(SparseError::Shape { expected: n, got: x0.len() });
        }
        if !all_finite(x0) {
            return Err(SparseError::NonFinite { what: "x0" });
        }
    }
    if !all_finite(b) {
        return Err(SparseError::NonFinite { what: "b" });
    }
    if !(rtol.is_finite() && rtol > 0.0) {
        return Err(SparseError::BadParam { name: "rtol" });
    }
    if max_iter == 0 {
        return Err(SparseError::BadParam { name: "max_iter" });
    }
    if let Some(row) = a.diag().iter().position(|&d| d <= 0.0) {
        return Err(SparseError::NonPositiveDiagonal { row });
    }
    let bmax = max_abs(b);
    if bmax == 0.0 {
        return Ok((vec![0.0; n], 0, 0.0));
    }
    // Scaled problem A' y = b' with A' = A 2^-ea, b' = b 2^-eb; then x = y 2^(eb - ea).
    let ea = exp2_of(max_abs(&a.data));
    let eb = exp2_of(bmax);
    let sa = Csr { data: a.data.iter().map(|&v| mul_pow2(v, -ea)).collect(), ..a.clone() };
    let bs: Vec<f64> = b.iter().map(|&v| mul_pow2(v, -eb)).collect();
    let mut y: Vec<f64> = match x0 {
        Some(x0) => x0.iter().map(|&v| mul_pow2(v, ea - eb)).collect(),
        None => vec![0.0; n],
    };
    if !all_finite(&y) {
        return Err(SparseError::NonFinite { what: "x0" });
    }
    let dinv: Vec<f64> = sa.diag().iter().map(|&d| 1.0 / d).collect();
    if !all_finite(&dinv) {
        return Err(SparseError::NonFinite { what: "inverse diagonal" });
    }
    let nb = norm2_vec(&bs);
    let (mut r, mut ap) = (vec![0.0; n], vec![0.0; n]);
    let true_residual = |y: &[f64], r: &mut [f64], ap: &mut [f64]| -> f64 {
        sa.mul_into(y, ap);
        for i in 0..n {
            r[i] = bs[i] - ap[i];
        }
        norm2_vec(r) / nb
    };
    let mut rel = true_residual(&y, &mut r, &mut ap);
    let (mut z, mut p, mut rz) = (vec![0.0; n], vec![0.0; n], 0.0);
    let (mut iters, mut restart) = (0usize, true);
    loop {
        if !rel.is_finite() {
            return Err(SparseError::NonFinite { what: "pcg iterate" });
        }
        if rel <= rtol {
            let t = true_residual(&y, &mut r, &mut ap);
            if t <= rtol {
                let x: Vec<f64> = y.iter().map(|&v| mul_pow2(v, eb - ea)).collect();
                if !all_finite(&x) {
                    return Err(SparseError::NonFinite { what: "solution" });
                }
                return Ok((x, iters, t));
            }
            rel = t;
            restart = true;
        }
        if iters >= max_iter {
            return Err(SparseError::NotConverged { iterations: iters, residual: rel });
        }
        if restart {
            for i in 0..n {
                z[i] = dinv[i] * r[i];
            }
            p.copy_from_slice(&z);
            rz = dot(&r, &z);
            restart = false;
        }
        sa.mul_into(&p, &mut ap);
        let pap = dot(&p, &ap);
        if pap.is_nan() || pap <= 0.0 {
            return Err(SparseError::Domain);
        }
        let alpha = rz / pap;
        for i in 0..n {
            y[i] += alpha * p[i];
            r[i] -= alpha * ap[i];
        }
        iters += 1;
        for i in 0..n {
            z[i] = dinv[i] * r[i];
        }
        let rz_new = dot(&r, &z);
        let beta = rz_new / rz;
        for i in 0..n {
            p[i] = z[i] + beta * p[i];
        }
        rz = rz_new;
        rel = norm2_vec(&r) / nb;
    }
}
