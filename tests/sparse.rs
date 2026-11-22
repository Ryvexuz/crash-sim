//! crash-sim's copy of numerics' CSR / Jacobi-PCG module (src/sparse.rs): storage, SpMV, the
//! solve, its refusals and its determinism.

use crash_sim::sparse::{pcg, Csr, SparseError};

/// Stiffness chain of `n` unknowns: node i joined to i+1 by a spring of stiffness k, and node 0
/// to the ground, so the matrix is the 1D beam chain (SPD, tridiagonal).
fn chain(n: usize, k: f64) -> Vec<(usize, usize, f64)> {
    let mut t = vec![(0, 0, k)];
    for i in 0..n - 1 {
        t.push((i, i, k));
        t.push((i + 1, i + 1, k));
        t.push((i, i + 1, -k));
        t.push((i + 1, i, -k));
    }
    t
}

fn dense_mul(n: usize, t: &[(usize, usize, f64)], x: &[f64]) -> Vec<f64> {
    let mut a = vec![0.0; n * n];
    for &(r, c, v) in t {
        a[r * n + c] += v;
    }
    (0..n)
        .map(|i| (0..n).map(|j| a[i * n + j] * x[j]).sum())
        .collect()
}

fn norm(x: &[f64]) -> f64 {
    x.iter().map(|v| v * v).sum::<f64>().sqrt()
}

fn rel_residual(a: &Csr, x: &[f64], b: &[f64]) -> f64 {
    let ax = a.mul_vec(x).unwrap();
    let r: Vec<f64> = b.iter().zip(&ax).map(|(b, a)| b - a).collect();
    norm(&r) / norm(b)
}

/// Unpreconditioned CG, written here only to compare iteration counts with `pcg`'s Jacobi.
fn plain_cg(a: &Csr, b: &[f64], rtol: f64, max_iter: usize) -> usize {
    let n = b.len();
    let mut x = vec![0.0; n];
    let mut r = b.to_vec();
    let mut p = r.clone();
    let mut rr: f64 = r.iter().map(|v| v * v).sum();
    let nb = norm(b);
    for it in 0..max_iter {
        if rr.sqrt() / nb <= rtol {
            return it;
        }
        let ap = a.mul_vec(&p).unwrap();
        let alpha = rr / p.iter().zip(&ap).map(|(p, q)| p * q).sum::<f64>();
        for i in 0..n {
            x[i] += alpha * p[i];
            r[i] -= alpha * ap[i];
        }
        let rr_new: f64 = r.iter().map(|v| v * v).sum();
        for i in 0..n {
            p[i] = r[i] + (rr_new / rr) * p[i];
        }
        rr = rr_new;
    }
    max_iter
}

#[test]
fn csr_sums_duplicate_triplets() {
    let t = [
        (0, 0, 1.0),
        (1, 1, 2.0),
        (0, 0, 3.0),
        (0, 1, 0.5),
        (0, 0, -0.25),
        (1, 0, 4.0),
    ];
    let a = Csr::from_triplets(2, 2, &t).unwrap();
    assert_eq!(a.nnz(), 4, "the three (0, 0) triplets are one entry");
    assert_eq!(a.diag(), vec![3.75, 2.0]);
    assert_eq!(a.mul_vec(&[1.0, 0.0]).unwrap(), vec![3.75, 4.0]);
    assert_eq!(a.mul_vec(&[0.0, 1.0]).unwrap(), vec![0.5, 2.0]);
}

#[test]
fn csr_mul_vec_matches_dense() {
    let n = 7;
    let mut t = Vec::new();
    for i in 0..n {
        for j in 0..n {
            if (i * 3 + j * 5) % 4 != 0 {
                t.push((i, j, (i as f64) * 0.7 - (j as f64) * 1.3 + 0.1));
            }
        }
    }
    t.push((2, 3, 9.0)); // a duplicate of an existing entry
    let a = Csr::from_triplets(n, n, &t).unwrap();
    let x: Vec<f64> = (0..n).map(|i| 1.0 + i as f64 * 0.25).collect();
    let y = a.mul_vec(&x).unwrap();
    let d = dense_mul(n, &t, &x);
    for i in 0..n {
        assert!(
            (y[i] - d[i]).abs() <= 1e-12 * (1.0 + d[i].abs()),
            "row {i}: {} vs {}",
            y[i],
            d[i]
        );
    }
    // a rectangular matrix too
    let r = Csr::from_triplets(2, 3, &[(0, 2, 2.0), (1, 0, -1.0)]).unwrap();
    assert_eq!(r.mul_vec(&[1.0, 2.0, 3.0]).unwrap(), vec![6.0, -1.0]);
    assert_eq!(r.diag(), vec![0.0, 0.0]);
}

#[test]
fn csr_refuses_bad_input() {
    assert_eq!(Csr::from_triplets(0, 2, &[]), Err(SparseError::Empty));
    assert_eq!(
        Csr::from_triplets(2, 2, &[(2, 0, 1.0)]),
        Err(SparseError::BadIndex)
    );
    assert_eq!(
        Csr::from_triplets(2, 2, &[(0, 2, 1.0)]),
        Err(SparseError::BadIndex)
    );
    assert_eq!(
        Csr::from_triplets(2, 2, &[(0, 0, f64::NAN)]),
        Err(SparseError::NonFinite {
            what: "triplet values"
        })
    );
    assert_eq!(
        Csr::from_triplets(2, 2, &[(0, 0, f64::MAX), (0, 0, f64::MAX)]),
        Err(SparseError::NonFinite {
            what: "summed duplicate entry"
        })
    );
    let a = Csr::from_triplets(2, 2, &[(0, 0, 1.0)]).unwrap();
    assert_eq!(
        a.mul_vec(&[1.0]),
        Err(SparseError::Shape {
            expected: 2,
            got: 1
        })
    );
    assert_eq!(
        a.mul_vec(&[f64::INFINITY, 0.0]),
        Err(SparseError::NonFinite { what: "x" })
    );
    let big = Csr::from_triplets(1, 1, &[(0, 0, f64::MAX)]).unwrap();
    assert_eq!(
        big.mul_vec(&[2.0]),
        Err(SparseError::NonFinite { what: "product" })
    );
}

#[test]
fn pcg_solves_1d_beam_chain() {
    let n = 200;
    let a = Csr::from_triplets(n, n, &chain(n, 3.0e6)).unwrap();
    let b: Vec<f64> = (0..n).map(|i| ((i * 7) % 11) as f64 - 4.5).collect();
    let (x, iters, res) = pcg(&a, &b, None, 1e-10, 10_000).unwrap();
    assert!(res <= 1e-10, "residual {res}");
    assert!(iters > 0 && iters <= 10_000);
    let again = rel_residual(&a, &x, &b);
    assert!(
        (again - res).abs() <= 1e-12,
        "returned residual {res} vs recomputed {again}"
    );
}

#[test]
fn pcg_jacobi_needs_fewer_iterations_than_identity() {
    // SPD, strictly diagonally dominant, diagonal from 1e0 to 1e6: Jacobi removes the scale.
    let n = 100;
    let mut t = Vec::new();
    for i in 0..n {
        let d = 10f64.powf(6.0 * i as f64 / (n - 1) as f64);
        t.push((i, i, d));
        if i + 1 < n {
            let dn = 10f64.powf(6.0 * (i + 1) as f64 / (n - 1) as f64);
            let off = 0.1 * d.min(dn);
            t.push((i, i + 1, off));
            t.push((i + 1, i, off));
        }
    }
    let a = Csr::from_triplets(n, n, &t).unwrap();
    let b: Vec<f64> = (0..n).map(|i| 1.0 + (i % 3) as f64).collect();
    let (_, jacobi, _) = pcg(&a, &b, None, 1e-10, 1_000).unwrap();
    let identity = plain_cg(&a, &b, 1e-10, 1_000);
    assert!(jacobi < 20, "Jacobi-PCG took {jacobi} iterations");
    assert!(identity > 100, "plain CG took only {identity} iterations");
}

#[test]
fn pcg_same_input_same_bytes() {
    let n = 150;
    let a = Csr::from_triplets(n, n, &chain(n, 2.5e5)).unwrap();
    let b: Vec<f64> = (0..n).map(|i| (i as f64 * 0.37).sin()).collect();
    let bits = |v: &[f64]| v.iter().map(|x| x.to_bits()).collect::<Vec<u64>>();
    let (x1, i1, r1) = pcg(&a, &b, None, 1e-11, 10_000).unwrap();
    let a2 = Csr::from_triplets(n, n, &chain(n, 2.5e5)).unwrap();
    let (x2, i2, r2) = pcg(&a2, &b, None, 1e-11, 10_000).unwrap();
    assert_eq!(bits(&x1), bits(&x2));
    assert_eq!((i1, r1.to_bits()), (i2, r2.to_bits()));
}

#[test]
fn pcg_refuses_non_square_size_mismatch_and_nonpositive_diagonal() {
    let rect = Csr::from_triplets(2, 3, &[(0, 0, 1.0), (1, 1, 1.0)]).unwrap();
    assert_eq!(
        pcg(&rect, &[1.0, 1.0], None, 1e-8, 10),
        Err(SparseError::NotSquare)
    );
    let a = Csr::from_triplets(2, 2, &[(0, 0, 2.0), (1, 1, 2.0)]).unwrap();
    assert_eq!(
        pcg(&a, &[1.0], None, 1e-8, 10),
        Err(SparseError::Shape {
            expected: 2,
            got: 1
        })
    );
    assert_eq!(
        pcg(&a, &[1.0, 1.0], Some(&[0.0]), 1e-8, 10),
        Err(SparseError::Shape {
            expected: 2,
            got: 1
        })
    );
    assert_eq!(
        pcg(&a, &[f64::NAN, 1.0], None, 1e-8, 10),
        Err(SparseError::NonFinite { what: "b" })
    );
    assert_eq!(
        pcg(&a, &[1.0, 1.0], None, 0.0, 10),
        Err(SparseError::BadParam { name: "rtol" })
    );
    assert_eq!(
        pcg(&a, &[1.0, 1.0], None, f64::NAN, 10),
        Err(SparseError::BadParam { name: "rtol" })
    );
    assert_eq!(
        pcg(&a, &[1.0, 1.0], None, 1e-8, 0),
        Err(SparseError::BadParam { name: "max_iter" })
    );
    let zero_diag = Csr::from_triplets(2, 2, &[(0, 0, 2.0), (0, 1, 1.0), (1, 0, 1.0)]).unwrap();
    assert_eq!(
        pcg(&zero_diag, &[1.0, 1.0], None, 1e-8, 10),
        Err(SparseError::NonPositiveDiagonal { row: 1 })
    );
    let neg = Csr::from_triplets(2, 2, &[(0, 0, -2.0), (1, 1, 2.0)]).unwrap();
    assert_eq!(
        pcg(&neg, &[1.0, 1.0], None, 1e-8, 10),
        Err(SparseError::NonPositiveDiagonal { row: 0 })
    );
}

#[test]
fn pcg_nonconvergence_is_error() {
    let n = 200;
    let a = Csr::from_triplets(n, n, &chain(n, 1.0)).unwrap();
    let b: Vec<f64> = (0..n).map(|i| 1.0 + (i % 5) as f64).collect();
    match pcg(&a, &b, None, 1e-10, 1) {
        Err(SparseError::NotConverged {
            iterations,
            residual,
        }) => {
            assert_eq!(iterations, 1);
            assert!(residual > 1e-10 && residual.is_finite(), "{residual}");
        }
        other => panic!("expected NotConverged, got {other:?}"),
    }
}

#[test]
fn pcg_refuses_indefinite_matrix() {
    // Positive diagonal, but eigenvalues 3 and -1: CG meets p^T A p <= 0.
    let a =
        Csr::from_triplets(2, 2, &[(0, 0, 1.0), (1, 1, 1.0), (0, 1, 2.0), (1, 0, 2.0)]).unwrap();
    assert_eq!(
        pcg(&a, &[1.0, -1.0], None, 1e-10, 50),
        Err(SparseError::Domain)
    );
}

#[test]
fn pcg_zero_rhs_and_start_vector() {
    let n = 50;
    let a = Csr::from_triplets(n, n, &chain(n, 10.0)).unwrap();
    assert_eq!(
        pcg(&a, &vec![0.0; n], None, 1e-10, 100),
        Ok((vec![0.0; n], 0, 0.0))
    );
    let b: Vec<f64> = (0..n).map(|i| i as f64).collect();
    let (x, _, _) = pcg(&a, &b, None, 1e-12, 10_000).unwrap();
    // Starting at the answer converges with no iteration.
    let (x2, iters, res) = pcg(&a, &b, Some(&x), 1e-10, 10_000).unwrap();
    assert_eq!(iters, 0, "residual {res}");
    assert_eq!(x, x2);
}

#[test]
fn pcg_rechecks_the_returned_solution_after_rescaling() {
    // Review r1 (sparse.rs:376): x = b / a = 1e-616 underflows to 0 when scaled back; its true
    // residual against the unscaled system is 1, so this must be an error, never a result.
    let a = Csr::from_triplets(1, 1, &[(0, 0, 1e308)]).unwrap();
    let r = pcg(&a, &[1e-308], None, 1e-10, 10);
    assert!(
        r.is_err(),
        "an underflowed solution was returned as converged: {r:?}"
    );
    match r {
        Err(SparseError::Unrepresentable { residual }) => assert_eq!(residual, 1.0),
        other => panic!("expected Unrepresentable, got {other:?}"),
    }
    // A tiny but representable answer (a subnormal) is still solved, and its residual is the
    // true one of the returned vector.
    let a = Csr::from_triplets(1, 1, &[(0, 0, 1e300)]).unwrap();
    let (x, _, res) = pcg(&a, &[1e-10], None, 1e-10, 10).unwrap();
    assert!(x[0] > 0.0 && x[0] < f64::MIN_POSITIVE, "{x:?}");
    assert!(res <= 1e-10, "{res}");
    let back = (1e-10 - 1e300 * x[0]).abs() / 1e-10;
    assert!(
        (back - res).abs() <= 1e-12,
        "returned residual {res} vs recomputed {back}"
    );
}

/// The test's own true relative residual `||b - A x|| / ||b||` (dense, unscaled `A x`), with
/// both norms scaled by the same power of two (2^-e, e from max |b|) before squaring, so a
/// huge or tiny `b` neither overflows nor underflows. Independent of `src/sparse.rs`.
fn independent_rel_residual(n: usize, t: &[(usize, usize, f64)], x: &[f64], b: &[f64]) -> f64 {
    let ax = dense_mul(n, t, x);
    assert!(
        ax.iter().all(|v| v.is_finite()),
        "A x overflowed in the test: {ax:?}"
    );
    let mb = b.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
    let e = mb.log2().floor() as i32;
    let s = |v: f64| {
        // v * 2^-e in two exact steps (each factor is a normal power of two)
        let h = -e / 2;
        v * 2f64.powi(h) * 2f64.powi(-e - h)
    };
    let nr: f64 = b
        .iter()
        .zip(&ax)
        .map(|(b, a)| s(b - a).powi(2))
        .sum::<f64>()
        .sqrt();
    let nb: f64 = b.iter().map(|&v| s(v).powi(2)).sum::<f64>().sqrt();
    nr / nb
}

#[test]
fn pcg_extreme_magnitudes_table() {
    let i4: Vec<(usize, usize, f64)> = (0..4).map(|i| (i, i, 1.0)).collect();
    let pair = |s: f64| vec![(0, 0, 2.0 * s), (0, 1, s), (1, 0, s), (1, 1, 2.0 * s)];
    let near = (1.0 - 1e-8) * 1e308;
    // (name, n, A, b, x0, rtol, Ok expected)
    #[allow(clippy::type_complexity)]
    let cases: Vec<(
        &str,
        usize,
        Vec<(usize, usize, f64)>,
        Vec<f64>,
        Option<Vec<f64>>,
        f64,
        bool,
    )> = vec![
        (
            "review r2: ||b|| above f64::MAX, x0 near b",
            4,
            i4.clone(),
            vec![1e308; 4],
            Some(vec![near; 4]),
            1e-7,
            true,
        ),
        (
            "||b|| above f64::MAX, no x0",
            4,
            i4.clone(),
            vec![1e308; 4],
            None,
            1e-10,
            true,
        ),
        (
            "review r1: x underflows to 0",
            1,
            vec![(0, 0, 1e308)],
            vec![1e-308],
            None,
            1e-10,
            false,
        ),
        (
            "x subnormal",
            1,
            vec![(0, 0, 1e300)],
            vec![1e-10],
            None,
            1e-10,
            true,
        ),
        (
            "x overflows",
            1,
            vec![(0, 0, 1e-300)],
            vec![1e300],
            None,
            1e-10,
            false,
        ),
        (
            "A 1e300, b 1e300",
            2,
            pair(1e300),
            vec![1e300, -1e300],
            None,
            1e-12,
            true,
        ),
        (
            "A 1e-300, b 1e-300",
            2,
            pair(1e-300),
            vec![1e-300, 1e-300],
            None,
            1e-12,
            true,
        ),
        (
            "A and b subnormal",
            2,
            pair(1e-310),
            vec![1e-310, 2e-310],
            None,
            1e-6,
            true,
        ),
        (
            "b mixed 1e300 and 1e-300",
            2,
            pair(1.0),
            vec![1e300, 1e-300],
            None,
            1e-12,
            true,
        ),
        (
            "diagonal spans beyond f64",
            2,
            vec![(0, 0, 1e300), (1, 1, 1e-300)],
            vec![1.0, 1.0],
            None,
            1e-10,
            false,
        ),
        (
            "A 1e300, b 1e-300: x 1e-600 underflows",
            2,
            pair(1e300),
            vec![1e-300, 1e-300],
            None,
            1e-12,
            false,
        ),
    ];
    for (name, n, t, b, x0, rtol, want_ok) in cases {
        let a = Csr::from_triplets(n, n, &t).unwrap();
        let r = pcg(&a, &b, x0.as_deref(), rtol, 1_000);
        match (&r, want_ok) {
            (Ok((x, _, res)), true) => {
                let indep = independent_rel_residual(n, &t, x, &b);
                assert!(
                    indep <= rtol,
                    "{name}: Ok with true residual {indep} > rtol {rtol}"
                );
                assert!(
                    (res - indep).abs() <= 1e-3 * indep + 1e-300,
                    "{name}: returned residual {res} but the true one is {indep}"
                );
            }
            (Err(_), false) => {}
            _ => panic!(
                "{name}: expected {}, got {r:?}",
                if want_ok { "Ok" } else { "Err" }
            ),
        }
    }
}
