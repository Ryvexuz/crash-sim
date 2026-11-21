//! The m5 crash-zone solver (docs/M5-SOLVER.md section 9, R1 to R12 except R8; docs/PHYSICS.md
//! "Solids"). Every expected value here is a closed form, a formula written out in this file
//! independently of the crate (the Neo-Hookean energy and stress, the hardening law, the
//! lock-up line, the Gershgorin sums), an exact `==`, or a check definition the design note
//! names; no test takes the crate's own output as its reference (R11).
//!
//! Tolerance rules (R11). Each float in a comparison is one of these, an exact value, or a
//! number the design note names:

use crash_sim::format::{self, Body, BodyBeam, Material, Node, Solid, Tet};
use crash_sim::solver::{Solver, State, SymplecticEuler};
use crash_sim::solver_m5::{self, Law, Mat3, SolverM5, J_MIN_FAIL, M5_MAX_TETS};
use crash_sim::vec3::Vec3;
use crash_sim::{to_csv_with_broken, Config, Sim};
use std::cell::Cell;
use std::rc::Rc;

/// Rounding of a few dozen f64 operations on values of one size, relative.
const ROUND: f64 = 1e-12;
/// A sum of many forces of one size that should cancel: relative to the largest nodal force.
const CANCEL: f64 = 1e-10;
/// R4: "a plateau at plateau_stress within 1%".
const R4_PLATEAU: f64 = 0.01;
/// The law along a path, checked through forces: the solve's 60 bisection steps leave `d` to
/// about 1e-17 of its bracket, and this file's own root-finding for the elastic stretch to
/// 1e-15; the rest is the rounding of the stress reconstruction. Relative.
const LAW: f64 = 1e-9;
/// R9: the named allowance for the nonlinear terms at strain 1e-3.
const R9_NONLINEAR: f64 = 1e-2;
/// R9: momentum conserved to 1e-9 of its scale.
const R9_MOMENTUM: f64 = 1e-9;
/// R9's "energy sits in both blocks", as a check definition of this file (not a tolerance on
/// a computed value): the kinetic, beam and tet energy at the start are each over 1% of E0.
const R9_SHARE: f64 = 0.01;
/// R9's "a wave crosses the shared face", as a check definition of this file: the tet energy
/// moves by more than 10% of its start at some substep, ten times the R9 allowance.
const R9_WAVE: f64 = 0.1;
/// A central finite difference of the stress with step 1e-6: truncation about 1e-12 of the
/// tangent, rounding about 1e-10; relative to the largest eigenvalue.
const FD: f64 = 1e-6;
/// A strain of 1e-8 in `mu F + (lambda ln J - mu) F^-T` cancels about 8 of 16 digits.
const SMALL_STRAIN: f64 = 1e-6;

// ---------------------------------------------------------------------------------------
// Independent maths (not the crate's)

type M = [[f64; 3]; 3];

fn mdet(m: &M) -> f64 {
    m[0][0] * m[1][1] * m[2][2] + m[0][1] * m[1][2] * m[2][0] + m[0][2] * m[1][0] * m[2][1]
        - m[0][2] * m[1][1] * m[2][0]
        - m[0][0] * m[1][2] * m[2][1]
        - m[0][1] * m[1][0] * m[2][2]
}

/// Inverse by Gauss-Jordan elimination with partial pivoting.
fn minv(m: &M) -> M {
    let mut a = [[0.0; 6]; 3];
    for i in 0..3 {
        for j in 0..3 {
            a[i][j] = m[i][j];
        }
        a[i][3 + i] = 1.0;
    }
    for col in 0..3 {
        let p = (col..3)
            .max_by(|&x, &y| a[x][col].abs().total_cmp(&a[y][col].abs()))
            .unwrap();
        a.swap(col, p);
        let d = a[col][col];
        for x in a[col].iter_mut() {
            *x /= d;
        }
        for r in 0..3 {
            if r != col {
                let f = a[r][col];
                for j in 0..6 {
                    a[r][j] -= f * a[col][j];
                }
            }
        }
    }
    let mut out = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            out[i][j] = a[i][3 + j];
        }
    }
    out
}

fn mmul(a: &M, b: &M) -> M {
    let mut c = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    c
}

fn mt(a: &M) -> M {
    let mut c = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            c[i][j] = a[j][i];
        }
    }
    c
}

fn eye() -> M {
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
}

fn mvec(m: &M, v: Vec3) -> Vec3 {
    Vec3::new(
        m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
        m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
        m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z,
    )
}

fn fnorm(m: &M) -> f64 {
    m.iter().flatten().map(|x| x * x).sum::<f64>().sqrt()
}

/// Rotation by `angle` about the unit vector `axis` (Rodrigues).
fn rot(axis: Vec3, angle: f64) -> M {
    let a = axis * (1.0 / axis.length());
    let (s, c) = angle.sin_cos();
    let k: M = [[0.0, -a.z, a.y], [a.z, 0.0, -a.x], [-a.y, a.x, 0.0]];
    let k2 = mmul(&k, &k);
    let mut r = eye();
    for i in 0..3 {
        for j in 0..3 {
            r[i][j] += s * k[i][j] + (1.0 - c) * k2[i][j];
        }
    }
    r
}

/// The Lame constants of R4a: E = plateau / yield, Poisson ratio 0.3 (the frozen line).
fn lame(plateau: f64, yield_strain: f64) -> (f64, f64) {
    let e = plateau / yield_strain;
    (e / 2.6, e * 0.3 / (1.3 * 0.4))
}

/// W = mu/2 (tr C - 3) - mu ln J + lambda/2 (ln J)^2.
fn nh_w(f: &M, mu: f64, lam: f64) -> f64 {
    let j = mdet(f);
    let i1: f64 = f.iter().flatten().map(|x| x * x).sum();
    0.5 * mu * (i1 - 3.0) - mu * j.ln() + 0.5 * lam * j.ln() * j.ln()
}

/// P = mu F + (lambda ln J - mu) F^-T.
fn nh_p(f: &M, mu: f64, lam: f64) -> M {
    let j = mdet(f);
    let fit = mt(&minv(f));
    let mut p = [[0.0; 3]; 3];
    for i in 0..3 {
        for k in 0..3 {
            p[i][k] = mu * f[i][k] + (lam * j.ln() - mu) * fit[i][k];
        }
    }
    p
}

/// Eigenvalues of a symmetric n x n matrix, cyclic Jacobi until the off-diagonal is negligible.
fn eig_sym(mut a: Vec<Vec<f64>>) -> Vec<f64> {
    let n = a.len();
    for _ in 0..100 {
        let off: f64 = (0..n)
            .flat_map(|i| (0..n).map(move |j| (i, j)))
            .filter(|(i, j)| i != j)
            .map(|(i, j)| a[i][j] * a[i][j])
            .sum();
        let diag: f64 = (0..n).map(|i| a[i][i] * a[i][i]).sum();
        if off <= 1e-30 * diag {
            break;
        }
        for p in 0..n {
            for q in p + 1..n {
                if a[p][q] == 0.0 {
                    continue;
                }
                let th = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
                let t = th.signum() / (th.abs() + (th * th + 1.0).sqrt());
                let c = 1.0 / (t * t + 1.0).sqrt();
                let s = t * c;
                for ak in a.iter_mut() {
                    let (akp, akq) = (ak[p], ak[q]);
                    ak[p] = c * akp - s * akq;
                    ak[q] = s * akp + c * akq;
                }
                for k in 0..n {
                    let (apk, aqk) = (a[p][k], a[q][k]);
                    a[p][k] = c * apk - s * aqk;
                    a[q][k] = s * apk + c * aqk;
                }
            }
        }
    }
    (0..n).map(|i| a[i][i]).collect()
}

/// Smallest eigenvalue of F^T F and its eigenvector (this file's own solve).
fn smallest_stretch2(f: &M) -> f64 {
    let c = mmul(&mt(f), f);
    let v = eig_sym(c.iter().map(|r| r.to_vec()).collect());
    v.into_iter().fold(f64::INFINITY, f64::min)
}

/// A small deterministic generator (xorshift64*), uniform in [0, 1).
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        (self.0.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * self.next()
    }
    fn unit(&mut self) -> Vec3 {
        loop {
            let v = Vec3::new(
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
            );
            let l = v.length();
            if l > 0.1 && l < 1.0 {
                return v * (1.0 / l);
            }
        }
    }
    fn rotation(&mut self) -> M {
        let axis = self.unit();
        let angle = self.range(-3.0, 3.0);
        rot(axis, angle)
    }
}

// ---------------------------------------------------------------------------------------
// Bodies

fn empty_body() -> Body {
    Body {
        materials: Vec::new(),
        solids: Vec::new(),
        nodes: Vec::new(),
        beams: Vec::new(),
        tets: Vec::new(),
        groups: Vec::new(),
        velocities: Vec::new(),
        wheels: Vec::new(),
        suspensions: Vec::new(),
        tyres: Vec::new(),
        controls: None,
        planes: Vec::new(),
    }
}

fn solid(name: &str, plateau: f64, yield_strain: f64, damping: f64) -> Solid {
    Solid {
        name: name.into(),
        plateau_stress: plateau,
        yield_strain,
        damping_ratio: damping,
    }
}

fn add_node(b: &mut Body, id: u32, p: Vec3, mass: f64) {
    b.nodes.push(Node { id, pos: p, mass });
}

fn add_beam(b: &mut Body, a: u32, c: u32, material: &str) {
    let pa = b.nodes.iter().find(|n| n.id == a).unwrap().pos;
    let pc = b.nodes.iter().find(|n| n.id == c).unwrap().pos;
    b.beams.push(BodyBeam {
        a,
        b: c,
        material: material.into(),
        rest: (pa - pc).length(),
    });
}

fn add_tet(b: &mut Body, n: [u32; 4], s: &str) {
    b.tets.push(Tet {
        nodes: n,
        solid: s.into(),
    });
}

/// One tet on the unit right corner: nodes 0..3 at the origin and on the three axes, times
/// `size`, placed at `at`. Its rest edge matrix is `size` I, so `F = Ds / size`.
fn one_tet(plateau: f64, yield_strain: f64, damping: f64, size: f64, mass: f64, at: Vec3) -> Body {
    let mut b = empty_body();
    b.solids.push(solid("s", plateau, yield_strain, damping));
    let p = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(size, 0.0, 0.0),
        Vec3::new(0.0, size, 0.0),
        Vec3::new(0.0, 0.0, size),
    ];
    for (i, q) in p.iter().enumerate() {
        add_node(&mut b, i as u32, *q + at, mass);
    }
    add_tet(&mut b, [0, 1, 2, 3], "s");
    b
}

/// Grid node id of (ix, iy, iz) with 10 nodes per axis room.
fn gid(ix: usize, iy: usize, iz: usize) -> u32 {
    (ix * 100 + iy * 10 + iz) as u32
}

/// Nodes on the grid ix in 0..=nx, iy in 0..=ny, iz in 0..=nz, spacing `c`, from `at`.
fn add_grid(b: &mut Body, nx: usize, ny: usize, nz: usize, c: f64, at: Vec3, mass: f64) {
    for ix in 0..=nx {
        for iy in 0..=ny {
            for iz in 0..=nz {
                let p = at + Vec3::new(ix as f64 * c, iy as f64 * c, iz as f64 * c);
                add_node(b, gid(ix, iy, iz), p, mass);
            }
        }
    }
}

fn corners(ix: usize, iy: usize, iz: usize) -> [u32; 8] {
    let mut c = [0u32; 8];
    for (k, ck) in c.iter_mut().enumerate() {
        *ck = gid(ix + (k & 1), iy + ((k >> 1) & 1), iz + ((k >> 2) & 1));
    }
    c
}

/// The 6 tets of each cell (R12's rule, `solver_m5::cut_cell`) in the cell range.
fn add_tet_cells(b: &mut Body, xs: std::ops::Range<usize>, ny: usize, nz: usize, s: &str) {
    for ix in xs {
        for iy in 0..ny {
            for iz in 0..nz {
                for t in solver_m5::cut_cell(corners(ix, iy, iz)) {
                    add_tet(b, t, s);
                }
            }
        }
    }
}

/// All 28 node pairs of each cell in the range as beams, each pair once.
fn add_beam_cells(b: &mut Body, xs: std::ops::Range<usize>, ny: usize, nz: usize, material: &str) {
    let mut pairs: Vec<(u32, u32)> = Vec::new();
    for ix in xs {
        for iy in 0..ny {
            for iz in 0..nz {
                let c = corners(ix, iy, iz);
                for i in 0..8 {
                    for j in i + 1..8 {
                        let p = (c[i].min(c[j]), c[i].max(c[j]));
                        if !pairs.contains(&p) {
                            pairs.push(p);
                        }
                    }
                }
            }
        }
    }
    for (a, c) in pairs {
        add_beam(b, a, c, material);
    }
}

fn positions(b: &Body) -> Vec<Vec3> {
    b.nodes.iter().map(|n| n.pos).collect()
}

/// P from the nodal forces: `P = -(1/V) sum_a f_a (x) X_a` (since `sum_a X_a (x) grad N_a = I`).
fn stress_from_forces(f: &[Vec3], rest: &[Vec3], volume: f64) -> M {
    let mut p = [[0.0; 3]; 3];
    for (fa, xa) in f.iter().zip(rest) {
        let (fa, xa) = ([fa.x, fa.y, fa.z], [xa.x, xa.y, xa.z]);
        for i in 0..3 {
            for j in 0..3 {
                p[i][j] -= fa[i] * xa[j] / volume;
            }
        }
    }
    p
}

fn law_of(plateau: f64, yield_strain: f64) -> Law {
    // The lock-up strain does not enter the elastic law; 0.7 is the frozen cap.
    Law::new(plateau, yield_strain, 0.7)
}

fn as_m(m: &Mat3) -> M {
    *m
}

// ---------------------------------------------------------------------------------------
// The frozen numbers

/// The solver's constants are the `frozen:` lines of docs/M5-SOLVER.md section 8, read from
/// the file (never a copy).
#[test]
fn m5_constants_are_the_frozen_lines() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/docs/M5-SOLVER.md");
    let text = std::fs::read_to_string(path).expect("docs/M5-SOLVER.md");
    let line = |key: &str| -> String {
        let want = format!("frozen: {key} = ");
        let lines: Vec<&str> = text.lines().filter(|l| l.starts_with(&want)).collect();
        assert_eq!(lines.len(), 1, "exactly one `{want}` line");
        lines[0][want.len()..].to_string()
    };
    assert_eq!(line("m5_max_tets").parse::<usize>().unwrap(), M5_MAX_TETS);
    assert_eq!(
        line("poisson_ratio").parse::<f64>().unwrap(),
        solver_m5::POISSON_RATIO
    );
    let lockup = line("lockup");
    let want = format!(
        "eps_D = min({}, 1 - {} phi_zone); phi_zone = zone_mass / (zone_volume * {});",
        solver_m5::LOCKUP_CAP,
        solver_m5::LOCKUP_SLOPE,
        solver_m5::STEEL_DENSITY
    );
    assert!(
        lockup.starts_with(&want),
        "lock-up line `{lockup}` does not start `{want}`"
    );
    assert!(lockup.ends_with("flow stops at e_c = eps_D - sqrt(eps_D yield_strain)"));
    assert!(line("m5_method").contains(
        "an inverted tet or an exceeded stiffness bound ends the run with a named error"
    ));
    // R4e: the failure constant of decision 6 of the note.
    assert_eq!(J_MIN_FAIL, 0.05);
}

// ---------------------------------------------------------------------------------------
// R2, R3: the trait methods and the refusal

#[test]
fn default_solver_handles_no_solids_and_reports_nothing() {
    assert!(!SymplecticEuler.handles_solids());
    assert_eq!(SymplecticEuler.check(), None);
    let b = one_tet(1e6, 0.02, 0.0, 1.0, 1.0, Vec3::new(0.0, 10.0, 0.0));
    let s = SolverM5::from_body(&b, 1e-4).unwrap();
    assert!(s.handles_solids());
    assert_eq!(s.check(), None);
}

/// R3, at the lowest entry: `Sim::from_body_with_solver` with `SymplecticEuler` and a body with
/// tets, and the other two constructors that pass `build_from_body`.
#[test]
fn a_body_with_tets_is_refused_by_a_solver_without_solids() {
    let mut b = empty_body();
    b.solids.push(solid("s", 1e5, 0.02, 0.0));
    add_grid(&mut b, 1, 1, 1, 1.0, Vec3::new(0.0, 5.0, 0.0), 1.0);
    add_tet_cells(&mut b, 0..1, 1, 1, "s");
    let cfg = Config {
        dt: 0.001,
        substeps: 10,
        ..Config::default()
    };
    let want = "the body has 6 tets, which this solver does not simulate (they would be dropped): tets are simulated by `solver_m5::SolverM5` (docs/PHYSICS.md, \"Solids\")";
    let e = Sim::from_body_with_solver(cfg.clone(), &b, Box::new(SymplecticEuler))
        .err()
        .unwrap();
    assert_eq!(e, want);
    assert_eq!(Sim::from_body(cfg.clone(), &b).err().unwrap(), want);
    assert_eq!(
        Sim::from_car_with_solver(cfg.clone(), &b, Box::new(SymplecticEuler))
            .err()
            .unwrap(),
        want
    );
    // The m5 solver takes it.
    let m5 = SolverM5::from_body(&b, cfg.dt / cfg.substeps as f64).unwrap();
    assert!(Sim::from_body_with_solver(cfg, &b, Box::new(m5)).is_ok());
}

// ---------------------------------------------------------------------------------------
// R4a: the elastic law

#[test]
fn piola_and_energy_are_the_neo_hookean_formulas() {
    let law = law_of(2e6, 0.02);
    let (mu, lam) = lame(2e6, 0.02);
    assert!((law.mu - mu).abs() <= ROUND * mu && (law.lambda - lam).abs() <= ROUND * lam);
    let mut rng = Rng(0x5eed_0001);
    for _ in 0..300 {
        // stretches 0.3 to 3, a shear, two rotations
        let s = [
            rng.range(0.3f64.ln(), 3f64.ln()).exp(),
            rng.range(0.3f64.ln(), 3f64.ln()).exp(),
            rng.range(0.3f64.ln(), 3f64.ln()).exp(),
        ];
        let mut shear = eye();
        shear[0][1] = rng.range(-1.0, 1.0);
        let d: M = [[s[0], 0.0, 0.0], [0.0, s[1], 0.0], [0.0, 0.0, s[2]]];
        let f = mmul(&mmul(&rng.rotation(), &mmul(&d, &shear)), &rng.rotation());
        let p = solver_m5::first_piola(&f, &law);
        let q = nh_p(&f, mu, lam);
        let scale = fnorm(&q);
        for i in 0..3 {
            for j in 0..3 {
                assert!(
                    (p[i][j] - q[i][j]).abs() <= ROUND * scale,
                    "P differs at {i}{j}: {} vs {}",
                    p[i][j],
                    q[i][j]
                );
            }
        }
        let w = solver_m5::energy_density(&f, &law);
        let wt = nh_w(&f, mu, lam);
        // W is a sum of terms of size mu |F|^2: compare against that scale.
        let wscale =
            mu * fnorm(&f).powi(2) + lam * mdet(&f).ln().powi(2) + mu * mdet(&f).ln().abs();
        assert!((w - wt).abs() <= ROUND * wscale, "W {w} vs {wt}");
    }
    // Not a positive volume: infinite energy.
    let mut flat = eye();
    flat[2][2] = 0.0;
    assert_eq!(solver_m5::energy_density(&flat, &law), f64::INFINITY);
}

#[test]
fn piola_matches_linear_elasticity_at_small_strain() {
    let law = law_of(2e6, 0.02);
    let (mu, lam) = lame(2e6, 0.02);
    let e: M = [[1.0, 0.3, -0.2], [0.1, -0.5, 0.4], [0.0, 0.2, 0.7]];
    let eps = 1e-8;
    let mut f = eye();
    for i in 0..3 {
        for j in 0..3 {
            f[i][j] += eps * e[i][j];
        }
    }
    let p = solver_m5::first_piola(&f, &law);
    let tr = e[0][0] + e[1][1] + e[2][2];
    for i in 0..3 {
        for j in 0..3 {
            let lin = eps * (mu * (e[i][j] + e[j][i]) + if i == j { lam * tr } else { 0.0 });
            assert!(
                (p[i][j] - lin).abs() <= SMALL_STRAIN * eps * (mu + lam),
                "P{i}{j} {} vs linear {lin}",
                p[i][j]
            );
        }
    }
}

/// R4a: the force on a compressed tet grows without bound as Je -> 0 (Saint Venant-Kirchhoff's
/// falls). Through the solver's own tet forces down to just above J_MIN_FAIL, then the stress
/// function to Je = 1e-6. The solid locks up almost at once (yield_strain 0.69 with eps_D = 0.7
/// gives e_max = 0.7 - sqrt(0.7 * 0.69) = 0.005), so past that the tet is elastic with
/// Fp = diag(e^-e_max, 1, 1): Fe11 = j e^e_max = Je and P11 = Pe11 e^e_max.
#[test]
fn compressive_force_grows_without_bound_as_je_goes_to_zero() {
    let (plateau, ys) = (6.9e8, 0.69);
    let (mu, lam) = lame(plateau, ys);
    let at = Vec3::new(0.0, 10.0, 0.0);
    let b = one_tet(plateau, ys, 0.0, 1.0, 1.0, at);
    let e_max = 0.7 - (0.7f64 * ys).sqrt();
    let mut s = SolverM5::from_body(&b, 1e-6).unwrap();
    let mut last = 0.0;
    let mut j = 1.0;
    let mut checked = 0;
    while j > 0.0605 {
        j -= 0.005;
        let mut pos = positions(&b);
        pos[1] = at + Vec3::new(j, 0.0, 0.0); // F = diag(j, 1, 1)
        let f = s.tet_forces(&pos, &[Vec3::ZERO; 4]);
        assert_eq!(s.check(), None);
        assert!(
            f[1].x > last,
            "the force must keep growing as Je falls ({} after {last})",
            f[1].x
        );
        last = f[1].x;
        if (s.plastic_strain(0) - e_max).abs() <= LAW {
            // f_1 = -V P e1, V = 1/6.
            let je = j * e_max.exp();
            let p11 = (mu * je + (lam * je.ln() - mu) / je) * e_max.exp();
            let want = -p11 / 6.0;
            assert!(
                (f[1].x - want).abs() <= LAW * want.abs(),
                "Je {je}: f {} vs {want}",
                f[1].x
            );
            checked += 1;
        }
    }
    assert!(checked >= 80, "only {checked} locked points");
    // Below J_MIN_FAIL the solver stops, so the law itself: P11 follows the closed form, which
    // grows like (mu - lambda ln j) / j without bound, at every decade down to Je = 1e-6, while
    // Saint Venant-Kirchhoff's P11 = j (lambda/2 + mu) (j^2 - 1) falls toward zero.
    let law = law_of(plateau, ys);
    let mut last = 0.0;
    let mut svk_last = f64::INFINITY;
    for &j in &[1e-2, 1e-3, 1e-4, 1e-5, 1e-6] {
        let p = solver_m5::first_piola(&[[j, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], &law);
        let want = mu * j + (lam * j.ln() - mu) / j;
        assert!(
            (p[0][0] - want).abs() <= ROUND * want.abs(),
            "Je {j}: P11 {} vs {want}",
            p[0][0]
        );
        assert!(-p[0][0] > last);
        last = -p[0][0];
        let svk = (j * (0.5 * lam + mu) * (j * j - 1.0)).abs();
        assert!(svk < svk_last, "SVK falls toward zero");
        svk_last = svk;
    }
}

// ---------------------------------------------------------------------------------------
// R4b, R4c: plastic flow and lock-up

/// The elastic stretch `s_e` of confined uniaxial compression whose Kirchhoff stress is `tau`
/// (this file's own root-finding of tau = mu (s^2 - 1) + lambda ln s, increasing in s).
fn elastic_stretch(tau: f64, mu: f64, lam: f64) -> f64 {
    let (mut lo, mut hi) = (1e-6, 4.0);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if mu * (mid * mid - 1.0) + lam * mid.ln() < tau {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

/// R4: uniaxial confined compression of one tet gives a plateau at `plateau_stress` within 1%,
/// follows `sigma_y(e_c) = plateau / (1 - e_c / eps_D)`, unloads elastically, and locks up at
/// `e_max`. The plastic strain is read from the forces (the stress and this file's elastic law),
/// not from the solver.
#[test]
fn confined_compression_plateau_unload_and_lockup() {
    let (plateau, ys) = (1e6, 0.02);
    let (mu, lam) = lame(plateau, ys);
    let mass = 1e-3;
    let b = one_tet(plateau, ys, 0.0, 1.0, mass, Vec3::ZERO);
    let rest = positions(&b);
    let vol = 1.0 / 6.0;
    // The frozen lock-up line, by hand: phi = 4 mass / (V 7850), eps_D = min(0.7, 1 - 1.4 phi).
    let phi = 4.0 * mass / (vol * 7850.0);
    let eps_d = (1.0 - 1.4 * phi).min(0.7);
    let e_max = eps_d - (eps_d * ys).sqrt();
    let sigma_y = |e: f64| plateau / (1.0 - e / eps_d);
    let mut s = SolverM5::from_body(&b, 1e-6).unwrap();
    assert!((s.eps_d() - eps_d).abs() <= ROUND);

    let eval = |s: &mut SolverM5, x: f64| -> (f64, f64) {
        let mut pos = rest.clone();
        pos[1] = Vec3::new(x, 0.0, 0.0);
        let f = s.tet_forces(&pos, &[Vec3::ZERO; 4]);
        let p = stress_from_forces(&f, &rest, vol);
        let tau = p[0][0] * x; // Kirchhoff tau = P F^T, F = diag(x, 1, 1)
        let se = elastic_stretch(tau, mu, lam);
        (tau, se.ln() - x.ln()) // e_c = ln s_e - ln s
    };

    let mut x = 1.0;
    let mut first_yield = None;
    let mut plateau_points = 0;
    let mut last_ec = 0.0;
    let mut last_detfp = 1.0;
    let mut locked_at = None;
    while x > 0.47 {
        x -= 1e-3;
        let (tau, e_c) = eval(&mut s, x);
        assert_eq!(s.check(), None);
        assert!(
            e_c >= last_ec - LAW,
            "e_c must not fall while loading: {e_c} after {last_ec}"
        );
        let detfp = mdet(&as_m(&s.fp(0)));
        assert!(
            detfp <= last_detfp,
            "det Fp must never increase: {detfp} after {last_detfp}"
        );
        last_detfp = detfp;
        if e_c > LAW {
            first_yield.get_or_insert(x);
            // det Fp = e^-e_c (each flow takes e^-d off it).
            assert!(
                (detfp - (-e_c).exp()).abs() <= LAW,
                "det Fp {detfp} vs e^-e_c {}",
                (-e_c).exp()
            );
            if e_c < e_max - LAW {
                assert!(
                    (-tau - sigma_y(e_c)).abs() <= LAW * sigma_y(e_c),
                    "tau {tau} vs sigma_y {}",
                    sigma_y(e_c)
                );
                if sigma_y(e_c) <= plateau * (1.0 + R4_PLATEAU) {
                    assert!((-tau / plateau - 1.0).abs() <= R4_PLATEAU);
                    plateau_points += 1;
                }
            } else {
                // Locked: e_c stays at e_max and the stress climbs above the yield stress.
                assert!(
                    (e_c - e_max).abs() <= LAW,
                    "lock-up e_c {e_c} vs e_max {e_max}"
                );
                if locked_at.is_none() {
                    locked_at = Some(x);
                } else {
                    assert!(
                        -tau > sigma_y(e_max),
                        "past lock-up the stress {tau} is elastic, above sigma_y(e_max)"
                    );
                }
            }
        } else {
            assert!(-tau < plateau, "elastic below the plateau");
        }
        last_ec = e_c;
    }
    assert!(first_yield.is_some(), "the tet never yielded");
    assert!(
        plateau_points >= 3,
        "only {plateau_points} points on the plateau"
    );
    assert!(locked_at.is_some(), "the tet never locked up");
    assert!((s.plastic_strain(0) - e_max).abs() <= LAW);

    // Unloading: e_c stays, and the stress is the elastic law at s_e = s e^e_c.
    let e_fixed = last_ec;
    for _ in 0..50 {
        x += 2e-3;
        let (tau, e_c) = eval(&mut s, x);
        assert!(
            (e_c - e_fixed).abs() <= LAW,
            "unloading moved e_c: {e_c} vs {e_fixed}"
        );
        let se = x * e_fixed.exp();
        let want = mu * (se * se - 1.0) + lam * se.ln();
        assert!(
            (tau - want).abs() <= LAW * plateau,
            "unloading tau {tau} vs elastic {want}"
        );
    }
}

/// R4b: tension never yields: a confined stretch to 2 stays elastic and on the closed form.
#[test]
fn tension_never_yields() {
    let (plateau, ys) = (1e6, 0.02);
    let (mu, lam) = lame(plateau, ys);
    let b = one_tet(plateau, ys, 0.0, 1.0, 1e-3, Vec3::ZERO);
    let rest = positions(&b);
    let mut s = SolverM5::from_body(&b, 1e-6).unwrap();
    let mut x = 1.0;
    while x < 2.0 {
        x += 1e-2;
        let mut pos = rest.clone();
        pos[1] = Vec3::new(x, 0.0, 0.0);
        let f = s.tet_forces(&pos, &[Vec3::ZERO; 4]);
        assert_eq!(s.plastic_strain(0), 0.0);
        let tau = stress_from_forces(&f, &rest, 1.0 / 6.0)[0][0] * x;
        let want = mu * (x * x - 1.0) + lam * x.ln();
        assert!((tau - want).abs() <= ROUND * want.abs());
    }
    // And no tension state flows, whatever its rotation.
    let law = law_of(plateau, ys);
    let mut rng = Rng(0x5eed_0002);
    for _ in 0..300 {
        let d: M = [
            [rng.range(1.0, 3.0), 0.0, 0.0],
            [0.0, rng.range(1.0, 3.0), 0.0],
            [0.0, 0.0, rng.range(1.0, 3.0)],
        ];
        let f = mmul(&mmul(&rng.rotation(), &d), &rng.rotation());
        assert_eq!(solver_m5::return_map(&f, 0.0, &law), None);
    }
}

/// R4b: on compressed states the return mapping moves the smallest stretch toward 1 (s1 larger)
/// and makes the most compressive stress smaller in size; the bracket holds a sign change, or it
/// is the capped case; and every update lowers the elastic energy (dissipation >= 0).
#[test]
fn return_mapping_signs_bracket_and_dissipation() {
    let (plateau, ys) = (1e6, 0.02);
    let (mu, lam) = lame(plateau, ys);
    let law = law_of(plateau, ys);
    let (eps_d, e_max) = (0.7, 0.7 - (0.7f64 * ys).sqrt());
    let sigma_y = |e: f64| plateau / (1.0 - e / eps_d);
    let mut rng = Rng(0x5eed_0003);
    let (mut flowed, mut capped, mut solved) = (0, 0, 0);
    for _ in 0..2000 {
        // One compressed direction, the others at or above 1: the flow (which never takes s1
        // past 1) keeps it the smallest stretch, so this file's own eigen-solve finds it.
        let d: M = [
            [rng.range(0.6, 1.0), 0.0, 0.0],
            [0.0, rng.range(1.0, 1.05), 0.0],
            [0.0, 0.0, rng.range(1.0, 1.05)],
        ];
        let f = mmul(&mmul(&rng.rotation(), &d), &rng.rotation());
        let e_c = if rng.next() < 0.2 {
            rng.range(0.9 * e_max, e_max)
        } else {
            rng.range(0.0, e_max)
        };
        let Some(fl) = solver_m5::return_map(&f, e_c, &law) else {
            continue;
        };
        flowed += 1;
        let fe2 = solver_m5::apply_flow(&f, &fl);
        let s1 = smallest_stretch2(&f).sqrt();
        let s1b = smallest_stretch2(&fe2).sqrt();
        let tau = mu * (s1 * s1 - 1.0) + lam * mdet(&f).ln();
        let taub = mu * (s1b * s1b - 1.0) + lam * mdet(&fe2).ln();
        assert!(tau < 0.0, "a flow needs a compressive tau1: {tau}");
        assert!(fl.d > 0.0);
        assert!(s1b > s1, "s1 must grow: {s1} -> {s1b}");
        assert!(
            taub.abs() < tau.abs(),
            "|tau1| must shrink: {tau} -> {taub}"
        );
        assert!(fl.g0 < 0.0);
        if fl.capped {
            assert!(fl.g_hi < 0.0);
            assert_eq!(fl.d, fl.d_hi);
            capped += 1;
        } else {
            assert!(fl.g_hi >= 0.0);
            // The root: the stress sits on the yield surface at e_c + d.
            assert!(
                (taub + sigma_y(e_c + fl.d)).abs() <= LAW * sigma_y(e_c + fl.d),
                "tau1' {taub} vs -sigma_y {}",
                sigma_y(e_c + fl.d)
            );
            solved += 1;
        }
        // Dissipation: the elastic energy goes down.
        assert!(
            nh_w(&fe2, mu, lam) <= nh_w(&f, mu, lam),
            "a plastic update raised the energy"
        );
    }
    assert!(
        flowed >= 500 && capped >= 10 && solved >= 100,
        "too few cases: {flowed} flowed, {capped} capped, {solved} solved"
    );
}

// ---------------------------------------------------------------------------------------
// R4: patch test, frame invariance, damping

fn block_2x2x2(plateau: f64, ys: f64, damping: f64) -> Body {
    let mut b = empty_body();
    b.solids.push(solid("s", plateau, ys, damping));
    add_grid(&mut b, 2, 2, 2, 0.2, Vec3::new(0.0, 100.0, 0.0), 1.0);
    add_tet_cells(&mut b, 0..2, 2, 2, "s");
    b
}

#[test]
fn patch_test_uniform_strain_leaves_interior_node_unloaded() {
    let b = block_2x2x2(6.9e8, 0.69, 0.05);
    assert_eq!(b.tets.len(), 48);
    let (mu, lam) = lame(6.9e8, 0.69);
    let rest = positions(&b);
    let c = rest[b.nodes.iter().position(|n| n.id == gid(1, 1, 1)).unwrap()];
    let f: M = [[1.02, 0.01, 0.0], [0.005, 0.99, 0.02], [0.0, -0.01, 1.01]];
    let pos: Vec<Vec3> = rest.iter().map(|x| c + mvec(&f, *x - c)).collect();
    let mut s = SolverM5::from_body(&b, 1e-7).unwrap();
    // Elastic energy of a uniform F: total volume times W(F).
    let e = s.elastic_energy(&pos);
    let want = 0.4f64.powi(3) * nh_w(&f, mu, lam);
    assert!((e - want).abs() <= ROUND * want, "energy {e} vs {want}");
    let forces = s.tet_forces(&pos, &vec![Vec3::ZERO; pos.len()]);
    assert_eq!(s.check(), None);
    let fmax = forces.iter().map(|x| x.length()).fold(0.0, f64::max);
    let interior = b.nodes.iter().position(|n| n.id == gid(1, 1, 1)).unwrap();
    assert!(fmax > 0.0);
    assert!(
        forces[interior].length() <= CANCEL * fmax,
        "interior node force {:?}",
        forces[interior]
    );
    // A uniform velocity gradient (stretch rate) leaves it unloaded by the damping too.
    let l: M = [[3.0, 1.0, 0.0], [1.0, -2.0, 0.5], [0.0, 0.5, 1.0]];
    let vel: Vec<Vec3> = rest.iter().map(|x| mvec(&l, *x - c)).collect();
    let damped = s.tet_forces(&pos, &vel);
    let dmax = damped
        .iter()
        .zip(&forces)
        .map(|(a, b)| (*a - *b).length())
        .fold(0.0, f64::max);
    assert!(dmax > 0.0, "the damping did not act");
    assert!(damped[interior].length() <= CANCEL * fmax.max(dmax));
}

#[test]
fn frame_invariance_and_rigid_spin_has_no_damping_force() {
    let b = block_2x2x2(6.9e8, 0.69, 0.05);
    let rest = positions(&b);
    let c = rest[13];
    let f: M = [[1.03, 0.02, -0.01], [0.0, 0.98, 0.01], [0.01, 0.0, 1.02]];
    let pos: Vec<Vec3> = rest.iter().map(|x| c + mvec(&f, *x - c)).collect();
    let r = rot(Vec3::new(0.3, -1.0, 0.5), 1.1);
    let rpos: Vec<Vec3> = pos.iter().map(|x| c + mvec(&r, *x - c)).collect();
    let mut s = SolverM5::from_body(&b, 1e-7).unwrap();
    let (e0, e1) = (s.elastic_energy(&pos), s.elastic_energy(&rpos));
    assert!(
        (e0 - e1).abs() <= ROUND * e0,
        "rotation changed the energy: {e0} vs {e1}"
    );
    let zero = vec![Vec3::ZERO; pos.len()];
    let f0 = s.tet_forces(&pos, &zero);
    let f1 = s.tet_forces(&rpos, &zero);
    let fmax = f0.iter().map(|x| x.length()).fold(0.0, f64::max);
    for (a, b) in f0.iter().zip(&f1) {
        assert!(
            (mvec(&r, *a) - *b).length() <= CANCEL * fmax,
            "forces do not rotate with the body"
        );
    }
    // A rigid spin of the undeformed block: no elastic and no damping force.
    let w = Vec3::new(10.0, -20.0, 5.0);
    let vel: Vec<Vec3> = rest.iter().map(|x| w.cross(*x - c)).collect();
    let spin = s.tet_forces(&rest, &vel);
    let ref_scale = fmax;
    for x in &spin {
        assert!(
            x.length() <= CANCEL * ref_scale,
            "a rigid spin gave force {x:?}"
        );
    }
}

// ---------------------------------------------------------------------------------------
// R4e, R10: the inverted-tet rule

/// Two tets sharing the face 0 1 2; node 4 (apex of tet 1) is driven through the face by its
/// start velocity. The run ends with exactly `tet 1 inverted or collapsed at the updated
/// positions of substep <n>: Je = <value>`, no later than the free-flight substep plus 2, and no
/// NaN enters the state.
#[test]
fn a_tet_driven_through_collapse_ends_the_run_with_the_named_error() {
    let mut b = empty_body();
    // A very soft solid (E = 20 Pa), so the tet's resistance hardly slows the node.
    b.solids.push(solid("s", 10.0, 0.5, 0.0));
    let at = Vec3::new(0.0, 100.0, 0.0);
    let p = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, -1.0),
        Vec3::new(0.0, 0.0, 1.0),
    ];
    for (i, q) in p.iter().enumerate() {
        add_node(&mut b, i as u32, *q + at, 1.0);
    }
    add_tet(&mut b, [0, 2, 1, 3], "s"); // below the face z = 0
    add_tet(&mut b, [0, 1, 2, 4], "s"); // above it
    let cfg = Config {
        dt: 0.001,
        substeps: 10,
        gravity: 0.0,
        duration: 1.0,
        ..Config::default()
    };
    let h = cfg.dt / cfg.substeps as f64;
    let m5 = SolverM5::from_body(&b, h).unwrap();
    let mut sim = Sim::from_body_with_solver(cfg.clone(), &b, Box::new(m5)).unwrap();
    let v = 100.0;
    sim.vel[4] = Vec3::new(0.0, 0.0, -v);
    // Free flight: node 4's height z is 1 - n v h. The tet yields and locks up on the way
    // (e_max = 0.7 - sqrt(0.7 * 0.5)), after which Fp = diag(1, 1, e^-e_max), so the trial
    // Je = z e^e_max is at or below 0.05 from substep ceil((1 - 0.05 e^-e_max) / (v h)). The
    // tet's own resistance can only delay it, by a little at E = 20 Pa.
    let e_max = 0.7 - (0.7f64 * 0.5).sqrt();
    let n_free = ((1.0 - J_MIN_FAIL * (-e_max).exp()) / (v * h)).ceil() as u64;
    let mut err = None;
    for step in 0..1000 {
        sim.advance();
        match sim.check() {
            Ok(()) => assert!(
                ((step + 1) * 10) < n_free + 10,
                "no error long after the free-flight substep"
            ),
            Err(e) => {
                err = Some(e);
                break;
            }
        }
    }
    let e = err.expect("the run never failed");
    // Since round 5 the rule is applied to the positions a substep is about to write, so the
    // collapse is found there, in the substep that would have made it.
    let prefix = "tet 1 inverted or collapsed at the updated positions of substep ";
    assert!(e.starts_with(prefix), "{e}");
    let rest = &e[prefix.len()..];
    let (n, je) = rest.split_once(": Je = ").expect("`: Je = ` in the error");
    let n: u64 = n.parse().unwrap();
    let je: f64 = je.parse().unwrap();
    assert!(
        n >= n_free - 1 && n <= n_free + 2,
        "substep {n}, free flight {n_free}"
    );
    assert!(je <= J_MIN_FAIL && je.is_finite(), "Je = {je}");
    assert!(
        sim.pos
            .iter()
            .chain(&sim.vel)
            .all(|v| v.x.is_finite() && v.y.is_finite() && v.z.is_finite()),
        "a NaN entered the state"
    );
}

/// R4e: a NaN node ends the run with a named error (through the solver and through
/// `Sim::check`, which reports the solver's error first), and no NaN enters the force array.
/// `step` stops on the node itself (round 2); the tet guard behind it gives the R4e error
/// `tet <i> inverted or collapsed ... Je = NaN` where it is reached first, in `tet_forces`.
#[test]
fn a_nan_node_ends_the_run_with_the_named_error() {
    let at = Vec3::new(0.0, 10.0, 0.0);
    let b = one_tet(1e6, 0.02, 0.05, 1.0, 1.0, at);
    let mut s = SolverM5::from_body(&b, 1e-5).unwrap();
    let mut bad = positions(&b);
    bad[2].y = f64::NAN;
    let f = s.tet_forces(&bad, &[Vec3::ZERO; 4]);
    assert_eq!(
        s.check(),
        Some("tet 0 inverted or collapsed at substep 0: Je = NaN".to_string())
    );
    assert!(
        f.iter()
            .all(|v| v.x.is_finite() && v.y.is_finite() && v.z.is_finite()),
        "the tet guard let a NaN into the forces"
    );

    let mut s = SolverM5::from_body(&b, 1e-5).unwrap();
    let mut pos = positions(&b);
    let mut vel = vec![Vec3::ZERO; 4];
    pos[2].y = f64::NAN;
    let mass = vec![1.0; 4];
    s.step(
        State {
            pos: &mut pos,
            vel: &mut vel,
            beams: &[],
            mass: &mass,
            gravity: 0.0,
        },
        1e-5,
    );
    assert_eq!(
        s.check(),
        Some("node 2 is not finite at substep 1".to_string())
    );
    assert!(
        vel.iter()
            .all(|v| v.x.is_finite() && v.y.is_finite() && v.z.is_finite()),
        "a NaN entered the force array"
    );

    let cfg = Config {
        dt: 1e-4,
        substeps: 10,
        gravity: 0.0,
        duration: 1.0,
        ..Config::default()
    };
    let m5 = SolverM5::from_body(&b, cfg.dt / cfg.substeps as f64).unwrap();
    let mut sim = Sim::from_body_with_solver(cfg, &b, Box::new(m5)).unwrap();
    sim.pos[2].x = f64::NAN;
    sim.advance();
    assert_eq!(
        sim.check(),
        Err("node 2 is not finite at substep 1".to_string())
    );
}

/// Round 2 of review q-20261121-031356: a NaN node shared by a beam and a tet. The step must
/// stop before any force is summed: the run ends with `node <id> is not finite at substep <n>`
/// (the file's node id), and no other node gets a NaN or moves in that substep, through the
/// solver and through `Sim::check`. Before the fix the beam loop spread the NaN to node 14.
#[test]
fn a_nan_node_shared_by_a_beam_and_a_tet_stops_the_step_before_any_force() {
    let at = Vec3::new(0.0, 10.0, 0.0);
    let mut b = one_tet(1e6, 0.02, 0.0, 1.0, 1.0, at);
    for (k, n) in b.nodes.iter_mut().enumerate() {
        n.id = 10 + k as u32;
    }
    for t in b.tets.iter_mut() {
        t.nodes = t.nodes.map(|i| i + 10);
    }
    add_node(&mut b, 14, at + Vec3::new(-1.0, 0.0, 0.0), 1.0);
    b.materials.push(Material {
        name: "m".into(),
        stiffness: 1e3,
        damping: 1.0,
        yield_strain: None,
        break_strain: None,
    });
    add_beam(&mut b, 10, 14, "m"); // node 10 is in the tet and in the beam
    let h = 1e-5;
    let built = Body::build(&b);
    let mass: Vec<f64> = built.mass.clone();
    for (field, bad) in [("pos", f64::NAN), ("vel", f64::INFINITY)] {
        let mut s = SolverM5::from_body(&b, h).unwrap();
        let mut pos = built.pos.clone();
        let mut vel = vec![Vec3::new(0.1, 0.0, 0.0); 5];
        if field == "pos" {
            pos[0].x = bad;
        } else {
            vel[0].y = bad;
        }
        let (pos0, vel0) = (pos.clone(), vel.clone());
        s.step(
            State {
                pos: &mut pos,
                vel: &mut vel,
                beams: &built.beams,
                mass: &mass,
                gravity: 9.81,
            },
            h,
        );
        let ok = |v: Vec3| v.x.is_finite() && v.y.is_finite() && v.z.is_finite();
        for i in 1..5 {
            assert!(
                ok(pos[i]) && ok(vel[i]),
                "{field}: node {i} got a non-finite value"
            );
        }
        for i in 1..5 {
            assert!(
                pos[i] == pos0[i] && vel[i] == vel0[i],
                "{field}: node {i} moved in the stopped substep"
            );
        }
        assert_eq!(
            s.check(),
            Some("node 10 is not finite at substep 1".to_string()),
            "{field}"
        );
    }
    let cfg = Config {
        dt: 1e-4,
        substeps: 10,
        gravity: 9.81,
        duration: 1.0,
        ..Config::default()
    };
    let m5 = SolverM5::from_body(&b, cfg.dt / cfg.substeps as f64).unwrap();
    let mut sim = Sim::from_body_with_solver(cfg, &b, Box::new(m5)).unwrap();
    sim.pos[0].y = f64::NAN;
    sim.advance();
    assert_eq!(
        sim.check(),
        Err("node 10 is not finite at substep 1".to_string())
    );
    assert!(
        sim.pos[1..]
            .iter()
            .chain(&sim.vel)
            .all(|v| v.x.is_finite() && v.y.is_finite() && v.z.is_finite()),
        "a NaN spread to another node"
    );
}

// ---------------------------------------------------------------------------------------
// Round 3 (q-20261121-032354): the force array is checked before it is applied

fn state_bits(pos: &[Vec3], vel: &[Vec3]) -> Vec<u64> {
    pos.iter()
        .chain(vel)
        .flat_map(|v| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()])
        .collect()
}

/// Two nodes and one beam between them, `gap` apart, no tets.
fn two_node_beam(gap: f64, mass: f64, k: f64, rest: f64) -> Body {
    let mut b = empty_body();
    b.materials.push(Material {
        name: "m".into(),
        stiffness: k,
        damping: 0.0,
        yield_strain: None,
        break_strain: None,
    });
    add_node(&mut b, 0, Vec3::new(-gap / 2.0, 10.0, 0.0), mass);
    add_node(&mut b, 1, Vec3::new(gap / 2.0, 10.0, 0.0), mass);
    b.beams.push(BodyBeam {
        a: 0,
        b: 1,
        material: "m".into(),
        rest,
    });
    b
}

/// The reviewer's body (a beam between finite coordinates of 1e200 m, whose length and force
/// overflow) is refused at construction by `MAX_COORD_M`, and a start speed above
/// `MAX_SPEED_MPS` is refused too; both bounds are inclusive.
#[test]
fn a_body_beyond_the_coordinate_or_speed_bound_is_refused() {
    let e = refused(&two_node_beam(2e200, 1.0, 1e3, 1.0), 1e-5);
    assert!(
        e.starts_with("node 0: its position (-1e200, 1e1, 0e0) is beyond MAX_COORD_M = 1000000 m"),
        "{e}"
    );
    // At the bound: taken; just past it: refused.
    let mut at = two_node_beam(2.0, 1.0, 1e3, 2.0);
    at.nodes[1].pos.x = solver_m5::MAX_COORD_M;
    assert!(SolverM5::from_body(&at, 1e-5).is_ok());
    at.nodes[1].pos.z = -solver_m5::MAX_COORD_M * (1.0 + f64::EPSILON);
    assert!(refused(&at, 1e-5).starts_with("node 1: its position"));
    let mut fast = two_node_beam(2.0, 1.0, 1e3, 2.0);
    fast.groups.push(format::Group {
        name: "all".into(),
        nodes: vec![0, 1],
    });
    fast.velocities.push(format::Velocity {
        group: "all".into(),
        velocity: Vec3::new(6e3, 8e3, 0.0), // 1e4 m/s: at the bound
    });
    assert!(SolverM5::from_body(&fast, 1e-5).is_ok());
    fast.velocities[0].velocity.z = 1.0;
    assert!(
        refused(&fast, 1e-5).starts_with("velocity of group `all`: speed 1.00000000"),
        "{}",
        refused(&fast, 1e-5)
    );
    fast.velocities[0].velocity.z = f64::NAN;
    assert!(refused(&fast, 1e-5).contains("is above MAX_SPEED_MPS = 10000 m/s"));
}

/// A body that passes construction but whose force overflows during the run: nodes of 1e300 kg
/// and a beam of 5e307 N/m (stable: h^2 2k/m = 1 < 4 at h = 1e-4) with an explicit rest length
/// of 0.5 m between nodes 5 m apart, so k (len - rest) = 2.25e308 overflows in the first
/// substep. Through `Sim`: the run ends with the named error and the state never changes.
#[test]
fn a_beam_force_that_overflows_during_the_run_is_stopped_before_it_is_applied() {
    let b = two_node_beam(5.0, 1e300, 5e307, 0.5);
    let cfg = Config {
        dt: 1e-3,
        substeps: 10,
        gravity: 9.81,
        duration: 1.0,
        ..Config::default()
    };
    let m5 = SolverM5::from_body(&b, cfg.dt / cfg.substeps as f64).unwrap();
    let mut sim = Sim::from_body_with_solver(cfg, &b, Box::new(m5)).unwrap();
    let start = state_bits(&sim.pos, &sim.vel);
    sim.advance();
    assert!(
        state_bits(&sim.pos, &sim.vel) == start,
        "a substep changed the state"
    );
    assert_eq!(
        sim.check(),
        Err("force on node 0 is not finite at substep 1 (from beams)".to_string())
    );
}

/// The force check names each source of a non-finite force (gravity, beams, tets, the extra
/// forces of `step_with_forces`) and the update check an overflow of the new state; in every
/// case nothing of the substep is applied, the tets' plastic state included.
#[test]
fn the_force_check_catches_every_source_before_anything_is_applied() {
    let h = 1e-5;
    let run = |b: &Body,
               pos: &mut Vec<Vec3>,
               vel: &mut Vec<Vec3>,
               gravity: f64,
               extra: Option<&[Vec3]>|
     -> (Option<String>, SolverM5) {
        let mut s = SolverM5::from_body(b, h).unwrap();
        let mass: Vec<f64> = b.nodes.iter().map(|n| n.mass).collect();
        let beams = Body::build(b).beams;
        let before = state_bits(pos, vel);
        let st = State {
            pos,
            vel,
            beams: &beams,
            mass: &mass,
            gravity,
        };
        match extra {
            Some(e) => s.step_with_forces(st, e, h),
            None => s.step(st, h),
        }
        assert!(
            state_bits(pos, vel) == before,
            "the failed substep changed the state"
        );
        (s.check(), s)
    };
    // Gravity: 1e308 m/s^2 on 10 kg overflows.
    let b = one_tet(1e6, 0.02, 0.0, 1.0, 10.0, Vec3::new(0.0, 10.0, 0.0));
    let (mut pos, mut vel) = (positions(&b), vec![Vec3::ZERO; 4]);
    let (e, _) = run(&b, &mut pos, &mut vel, 1e308, None);
    assert_eq!(
        e.as_deref(),
        Some("force on node 0 is not finite at substep 1 (from gravity)")
    );
    // Beams: a node 1e200 m away (finite, past the node check) gives an infinite length.
    let b = two_node_beam(2.0, 1.0, 1e3, 2.0);
    let (mut pos, mut vel) = (positions(&b), vec![Vec3::ZERO; 2]);
    pos[1].x = 1e200;
    let (e, _) = run(&b, &mut pos, &mut vel, 9.81, None);
    assert_eq!(
        e.as_deref(),
        Some("force on node 0 is not finite at substep 1 (from beams)")
    );
    // Tets: two tets; tet 0 compressed past yield, tet 1 with a node 1e305 m out (Je finite,
    // mu Fe overflows). The flow of tet 0 must not be kept either.
    let mut b = one_tet(1e6, 0.02, 0.0, 1.0, 1.0, Vec3::new(0.0, 10.0, 0.0));
    for (i, q) in [
        Vec3::new(5.0, 10.0, 0.0),
        Vec3::new(6.0, 10.0, 0.0),
        Vec3::new(5.0, 11.0, 0.0),
        Vec3::new(5.0, 10.0, 1.0),
    ]
    .iter()
    .enumerate()
    {
        add_node(&mut b, 4 + i as u32, *q, 1.0);
    }
    add_tet(&mut b, [4, 5, 6, 7], "s");
    let squeezed = |far: bool| {
        let mut p = positions(&b);
        p[1].x = 0.5;
        if far {
            p[5].x = 1e305;
        }
        p
    };
    let (mut pos, mut vel) = (squeezed(false), vec![Vec3::ZERO; 8]);
    let mut s_ok = SolverM5::from_body(&b, h).unwrap();
    let mass = vec![1.0; 8];
    s_ok.step(
        State {
            pos: &mut pos,
            vel: &mut vel,
            beams: &[],
            mass: &mass,
            gravity: 0.0,
        },
        h,
    );
    assert_eq!(s_ok.check(), None);
    assert!(
        s_ok.plastic_strain(0) > 0.0,
        "control: the squeezed tet yields"
    );
    let (mut pos, mut vel) = (squeezed(true), vec![Vec3::ZERO; 8]);
    let (e, s) = run(&b, &mut pos, &mut vel, 0.0, None);
    assert_eq!(
        e.as_deref(),
        Some("force on node 4 is not finite at substep 1 (from tets)")
    );
    assert_eq!(
        s.plastic_strain(0),
        0.0,
        "the flow of a failed substep was kept"
    );
    // Extra forces of step_with_forces.
    let b = two_node_beam(2.0, 1.0, 1e3, 2.0);
    let (mut pos, mut vel) = (positions(&b), vec![Vec3::ZERO; 2]);
    let extra = [Vec3::ZERO, Vec3::new(0.0, f64::INFINITY, 0.0)];
    let (e, _) = run(&b, &mut pos, &mut vel, 9.81, Some(&extra));
    assert_eq!(
        e.as_deref(),
        Some("force on node 1 is not finite at substep 1 (from extra forces)")
    );
    // The update: finite force, finite state, but x + v h overflows (one free node, h = 1 s).
    let mut b = empty_body();
    add_node(&mut b, 0, Vec3::ZERO, 1.0);
    let mut s = SolverM5::from_body(&b, 1.0).unwrap();
    let (mut pos, mut vel) = (
        vec![Vec3::new(1.7e308, 0.0, 0.0)],
        vec![Vec3::new(1.7e308, 0.0, 0.0)],
    );
    let before = state_bits(&pos, &vel);
    s.step(
        State {
            pos: &mut pos,
            vel: &mut vel,
            beams: &[],
            mass: &[1.0],
            gravity: 0.0,
        },
        1.0,
    );
    assert!(
        state_bits(&pos, &vel) == before,
        "the failed update changed the state"
    );
    assert_eq!(
        s.check().as_deref(),
        Some("update of node 0 is not finite at substep 1")
    );
}

/// Round 4 (q-20261121-034426): every error path of a substep applies nothing. Body: tet 0
/// (nodes 0-3) squeezed past yield, so a committed substep would change its plastic state;
/// tet 1 (nodes 4-7); a beam between nodes 8 and 9. Each case adds one fault and checks the
/// named error, that the state is bit-identical, and that neither tet's plastic state changed.
/// The controls show tet 0 really flows at both steps when there is no fault. (The update
/// overflow path needs h = 1 s, which a tet refuses; it is checked on a free node above.)
#[test]
fn every_error_path_of_a_substep_applies_nothing() {
    let mut b = one_tet(1e6, 0.02, 0.0, 1.0, 2.0, Vec3::new(0.0, 10.0, 0.0));
    for (i, q) in [
        Vec3::new(5.0, 10.0, 0.0),
        Vec3::new(6.0, 10.0, 0.0),
        Vec3::new(5.0, 11.0, 0.0),
        Vec3::new(5.0, 10.0, 1.0),
        Vec3::new(10.0, 10.0, 0.0),
        Vec3::new(11.0, 10.0, 0.0),
    ]
    .iter()
    .enumerate()
    {
        add_node(&mut b, 4 + i as u32, *q, 2.0);
    }
    add_tet(&mut b, [4, 5, 6, 7], "s");
    b.materials.push(Material {
        name: "m".into(),
        stiffness: 1e3,
        damping: 0.0,
        yield_strain: None,
        break_strain: None,
    });
    add_beam(&mut b, 8, 9, "m");
    let limit = SolverM5::from_body(&b, 1e-12).unwrap().step_limit();
    let beams = Body::build(&b).beams;
    let mass = vec![2.0; 10];
    let eye_bits = |m: Mat3| m.iter().flatten().map(|x| x.to_bits()).collect::<Vec<_>>();
    let identity = eye_bits([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);
    let squeezed = || {
        let mut p = positions(&b);
        p[1].x = 0.9; // tet 0: F = diag(0.9, 1, 1), past yield
        p
    };
    // One substep with a fault; returns the error and the solver.
    let run = |h: f64,
               pos: &mut Vec<Vec3>,
               gravity: f64,
               extra: Option<&[Vec3]>|
     -> (Option<String>, SolverM5) {
        let mut s = SolverM5::from_body(&b, h).unwrap();
        let mut vel = vec![Vec3::ZERO; 10];
        let before = state_bits(pos, &vel);
        let st = State {
            pos,
            vel: &mut vel,
            beams: &beams,
            mass: &mass,
            gravity,
        };
        match extra {
            Some(e) => s.step_with_forces(st, e, h),
            None => s.step(st, h),
        }
        let err = s.check();
        if err.is_some() {
            assert!(
                state_bits(pos, &vel) == before,
                "{err:?}: the failed substep changed the state"
            );
            for t in 0..2 {
                assert_eq!(
                    s.plastic_strain(t),
                    0.0,
                    "{err:?}: tet {t} kept a plastic flow"
                );
                assert!(
                    eye_bits(s.fp(t)) == identity,
                    "{err:?}: tet {t} kept a new Fp"
                );
            }
        }
        (err, s)
    };
    let (small, big) = (0.1 * limit, 0.9 * limit);
    for h in [small, big] {
        let (e, s) = run(h, &mut squeezed(), 9.81, None);
        assert_eq!(e, None, "control at h = {h}");
        assert!(s.plastic_strain(0) > 0.0, "control: tet 0 flows at h = {h}");
    }
    let mut p = squeezed();
    p[8].x = f64::NAN;
    assert_eq!(
        run(small, &mut p, 9.81, None).0.as_deref(),
        Some("node 8 is not finite at substep 1")
    );
    assert_eq!(
        run(small, &mut squeezed(), 1e308, None).0.as_deref(),
        Some("force on node 0 is not finite at substep 1 (from gravity)")
    );
    let mut p = squeezed();
    p[9].x = 1e200;
    assert_eq!(
        run(small, &mut p, 9.81, None).0.as_deref(),
        Some("force on node 8 is not finite at substep 1 (from beams)")
    );
    let mut p = squeezed();
    p[5].x = 1e305;
    assert_eq!(
        run(small, &mut p, 9.81, None).0.as_deref(),
        Some("force on node 4 is not finite at substep 1 (from tets)")
    );
    let mut extra = vec![Vec3::ZERO; 10];
    extra[8].x = f64::INFINITY;
    assert_eq!(
        run(small, &mut squeezed(), 9.81, Some(&extra)).0.as_deref(),
        Some("force on node 8 is not finite at substep 1 (from extra forces)")
    );
    // The Je rule: tet 1's apex brought to 1 cm above its base.
    let mut p = squeezed();
    p[7].z = 0.01;
    let e = run(small, &mut p, 9.81, None).0.unwrap();
    assert!(
        e.starts_with("tet 1 inverted or collapsed at substep 1: Je = 0.0"),
        "{e}"
    );
    // The stiffness verdict: tet 1 sheared by 1.5 at h = 0.9 of the construction limit.
    let mut p = squeezed();
    p[5].y += 1.5;
    let e = run(big, &mut p, 9.81, None).0.unwrap();
    assert!(
        e.starts_with("stiffness bound exceeded at substep 1: h omega = "),
        "{e}"
    );
}

/// Round 4 (q-20261121-034426): construction refuses, by name, every number its stability
/// bound is built from that is not finite, instead of replacing it (the old code set a
/// non-finite w2 or g to 0, which accepted any step).
#[test]
fn construction_refuses_every_non_finite_bound_input() {
    let at = Vec3::new(0.0, 10.0, 0.0);
    // The reviewer's case: plateau 1e308 Pa at yield 0.02 gives E = 5e309 Pa.
    let e = refused(&one_tet(1e308, 0.02, 0.0, 1.0, 1.0, at), 1e-5);
    assert_eq!(
        e,
        "solid `s`: its elastic modulus E = plateau_stress / yield_strain = inf Pa is not finite"
    );
    // E finite (1.7e308 Pa) but the tangent at rest times the headroom, 2 (2 mu + 3 lambda),
    // overflows: refused at the node sums.
    let e = refused(&one_tet(1.7e306, 0.01, 0.0, 1.0, 1.0, at), 1e-5);
    assert!(
        e.starts_with("node 0: its m5 stability sums (stiffness inf"),
        "{e}"
    );
    // A beam whose doubled stiffness overflows (2 x 1e308).
    let e = refused(&two_node_beam(2.0, 1.0, 1e308, 2.0), 1e-5);
    assert!(
        e.starts_with("node 0: its m5 stability sums (stiffness inf"),
        "{e}"
    );
    // Finite sums over a node of 1e-300 kg: w2 = 2e10 / 1e-300 overflows.
    let e = refused(&two_node_beam(2.0, 1e-300, 1e10, 2.0), 1e-5);
    assert!(
        e.starts_with("the m5 stability bound is not finite: w2 = inf"),
        "{e}"
    );
    // The damping coefficient: E = 1e306 Pa and a zone density of 4800 kg/m^3 (200 kg nodes on
    // a 1/6 m^3 tet; eps_D = 0.144, above the yield 0.01) make (lambda + 2 mu) rho overflow.
    let e = refused(&one_tet(1e304, 0.01, 0.05, 1.0, 200.0, at), 1e-5);
    assert!(
        e.starts_with(
            "tet 0: its damping coefficient eta = 2 zeta L sqrt((lambda + 2 mu) rho) = inf"
        ),
        "{e}"
    );
    // A material built in code with a NaN or a negative number (the format refuses both).
    for (k, c) in [(f64::NAN, 0.0), (-1e3, 0.0), (1e3, f64::NAN), (1e3, -1.0)] {
        let mut b = two_node_beam(2.0, 1.0, 1e3, 2.0);
        b.materials[0].stiffness = k;
        b.materials[0].damping = c;
        let e = refused(&b, 1e-5);
        assert!(
            e.starts_with("beam 0: its material `m` has stiffness"),
            "{e}"
        );
    }
}

/// Round 5 (q-20261121-040727), point 1: an apex that crosses its opposite face within one
/// substep. One substep per output step, so the substep that inverts the tet is the last one
/// before `Sim::check`: the new positions' Je is checked before they are written, the run ends
/// with the named error, and the state is bit-identical to the state before the substep.
#[test]
fn a_tet_inverted_within_one_substep_is_caught_on_the_updated_positions() {
    let b = one_tet(10.0, 0.5, 0.0, 1.0, 1.0, Vec3::new(0.0, 100.0, 0.0));
    let cfg = Config {
        dt: 2e-4,
        substeps: 1,
        gravity: 0.0,
        duration: 1.0,
        ..Config::default()
    };
    let h = cfg.dt / cfg.substeps as f64;
    let m5 = SolverM5::from_body(&b, h).unwrap();
    let mut sim = Sim::from_body_with_solver(cfg, &b, Box::new(m5)).unwrap();
    // 1e4 m/s down (MAX_SPEED_MPS): the apex moves 2 m in the substep, from z = 1 to z = -1.
    sim.vel[3] = Vec3::new(0.0, 0.0, -solver_m5::MAX_SPEED_MPS);
    let before = state_bits(&sim.pos, &sim.vel);
    sim.advance();
    let e = sim.check().expect_err("the inverted tet passed Sim::check");
    assert!(
        e.starts_with("tet 0 inverted or collapsed at the updated positions of substep 1: Je = -"),
        "{e}"
    );
    assert!(
        state_bits(&sim.pos, &sim.vel) == before,
        "the failed substep changed the state"
    );
}

/// Round 5, point 2: no non-finite value is ever written into the working force array. Each
/// contribution, and the sum it would make, is checked before it is added; the solver's
/// `debug_assert!` after each source (tests build in debug) checks the whole array, so a
/// violation panics here. Two finite beam forces of 1.2e308 N on one node (4e307 N/m, 3 m
/// stretch each, 1e300 kg nodes, stable at h = 1e-4) whose sum overflows; and a beam whose
/// length overflows (a node 1e200 m away in the state).
#[test]
fn an_overflowing_beam_never_reaches_the_force_array() {
    let mut b = empty_body();
    b.materials.push(Material {
        name: "m".into(),
        stiffness: 4e307,
        damping: 0.0,
        yield_strain: None,
        break_strain: None,
    });
    add_node(&mut b, 0, Vec3::new(0.0, 10.0, 0.0), 1e300);
    add_node(&mut b, 1, Vec3::new(3.5, 10.0, 0.0), 1e300);
    add_node(&mut b, 2, Vec3::new(3.5, 10.0, 1e-3), 1e300);
    for c in [1, 2] {
        b.beams.push(BodyBeam {
            a: 0,
            b: c,
            material: "m".into(),
            rest: 0.5,
        });
    }
    let cfg = Config {
        dt: 1e-3,
        substeps: 10,
        gravity: 9.81,
        duration: 1.0,
        ..Config::default()
    };
    let m5 = SolverM5::from_body(&b, cfg.dt / cfg.substeps as f64).unwrap();
    let mut sim = Sim::from_body_with_solver(cfg, &b, Box::new(m5)).unwrap();
    let before = state_bits(&sim.pos, &sim.vel);
    sim.advance();
    assert_eq!(
        sim.check(),
        Err("force on node 0 is not finite at substep 1 (from beams)".to_string())
    );
    assert!(state_bits(&sim.pos, &sim.vel) == before);

    let b = two_node_beam(2.0, 1.0, 1e3, 2.0);
    let mut s = SolverM5::from_body(&b, 1e-5).unwrap();
    let (mut pos, mut vel) = (positions(&b), vec![Vec3::ZERO; 2]);
    pos[1].x = 1e200;
    let beams = Body::build(&b).beams;
    s.step(
        State {
            pos: &mut pos,
            vel: &mut vel,
            beams: &beams,
            mass: &[1.0, 1.0],
            gravity: 9.81,
        },
        1e-5,
    );
    assert_eq!(
        s.check().as_deref(),
        Some("force on node 0 is not finite at substep 1 (from beams)")
    );
}

// ---------------------------------------------------------------------------------------
// R1, R10: every refusal of the constructor

fn refused(b: &Body, h: f64) -> String {
    match SolverM5::from_body(b, h) {
        Ok(_) => panic!("the body was accepted"),
        Err(e) => e,
    }
}

#[test]
fn hostile_and_bad_bodies_are_refused_by_name() {
    let at = Vec3::new(0.0, 10.0, 0.0);
    let good = || one_tet(1e6, 0.02, 0.0, 1.0, 1.0, at);
    assert!(SolverM5::from_body(&good(), 1e-5).is_ok());

    // 1e6 tets: refused by the cap before anything per tet is built.
    let mut huge = good();
    let t = huge.tets[0].clone();
    huge.tets = vec![t; 1_000_000];
    assert_eq!(refused(&huge, 1e-5), "the body has 1000000 tets, more than the m5 solver takes (M5_MAX_TETS = 300, docs/M5-SOLVER.md `frozen: m5_max_tets`)");
    let mut over = good();
    over.tets = vec![over.tets[0].clone(); M5_MAX_TETS + 1];
    assert!(refused(&over, 1e-5).starts_with("the body has 301 tets"));

    for h in [0.0, -1e-5, f64::NAN, f64::INFINITY] {
        assert!(
            refused(&good(), h).ends_with("s must be a positive finite number"),
            "h = {h}"
        );
    }

    let mut nan = good();
    nan.nodes[3].pos.z = f64::NAN;
    assert_eq!(refused(&nan, 1e-5), "node 3: its position is not finite");
    let mut heavy = good();
    heavy.nodes[1].mass = 0.0;
    assert_eq!(
        refused(&heavy, 1e-5),
        "node 1: its mass 0 must be a positive finite number"
    );
    let mut twice = good();
    twice.nodes[2].id = 1;
    assert_eq!(refused(&twice, 1e-5), "node id 1 is given twice");

    let mut beamy = good();
    beamy.materials.push(Material {
        name: "m".into(),
        stiffness: 1e3,
        damping: 0.0,
        yield_strain: None,
        break_strain: None,
    });
    beamy.beams.push(BodyBeam {
        a: 0,
        b: 9,
        material: "m".into(),
        rest: 1.0,
    });
    assert_eq!(refused(&beamy, 1e-5), "beam 0 is on a missing node");
    beamy.beams[0] = BodyBeam {
        a: 0,
        b: 1,
        material: "nope".into(),
        rest: 1.0,
    };
    assert_eq!(
        refused(&beamy, 1e-5),
        "beam 0 names a missing material `nope`"
    );

    let mut dup = good();
    dup.solids.push(solid("s", 1e6, 0.02, 0.0));
    assert_eq!(refused(&dup, 1e-5), "solid `s` is given twice");
    for (p, y, z, want) in [
        (
            0.0,
            0.02,
            0.0,
            "solid `s`: plateau_stress 0 must be a positive finite number",
        ),
        (
            f64::NAN,
            0.02,
            0.0,
            "solid `s`: plateau_stress NaN must be a positive finite number",
        ),
        (
            1e6,
            0.0,
            0.0,
            "solid `s`: yield_strain 0 must be > 0 and < 1",
        ),
        (
            1e6,
            1.0,
            0.0,
            "solid `s`: yield_strain 1 must be > 0 and < 1",
        ),
        (
            1e6,
            f64::NAN,
            0.0,
            "solid `s`: yield_strain NaN must be > 0 and < 1",
        ),
        (
            1e6,
            0.02,
            -0.1,
            "solid `s`: damping_ratio -0.1 must be >= 0 and <= 1",
        ),
        (
            1e6,
            0.02,
            1.5,
            "solid `s`: damping_ratio 1.5 must be >= 0 and <= 1",
        ),
        (
            1e6,
            0.02,
            f64::NAN,
            "solid `s`: damping_ratio NaN must be >= 0 and <= 1",
        ),
    ] {
        let mut b = good();
        b.solids[0] = solid("s", p, y, z);
        assert_eq!(refused(&b, 1e-5), want);
    }

    let mut unk = good();
    unk.tets[0].solid = "t".into();
    assert_eq!(refused(&unk, 1e-5), "tet 0 (0 1 2 3): unknown solid `t`");
    let mut miss = good();
    miss.tets[0].nodes[2] = 7;
    assert_eq!(refused(&miss, 1e-5), "tet 0 (0 1 7 3): node 7 is missing");
    let mut rep = good();
    rep.tets[0].nodes[3] = 1;
    assert_eq!(refused(&rep, 1e-5), "tet 0 (0 1 2 1): lists node 1 twice");

    // Zero volume (four coplanar nodes) and collinear nodes.
    let mut flat = good();
    flat.nodes[3].pos = at + Vec3::new(1.0, 1.0, 0.0);
    assert!(
        refused(&flat, 1e-5).starts_with("tet 0 (0 1 2 3): its rest volume 0 is below the minimum"),
        "{}",
        refused(&flat, 1e-5)
    );
    let mut line = good();
    line.nodes[2].pos = at + Vec3::new(2.0, 0.0, 0.0);
    line.nodes[3].pos = at + Vec3::new(3.0, 0.0, 0.0);
    assert!(
        refused(&line, 1e-5).starts_with("tet 0 (0 1 2 3): its rest volume 0 is below the minimum")
    );
    // A sliver: volume above the minimum only because it is large.
    let mut sliver = good();
    for (i, q) in [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1000.0, 0.0, 0.0),
        Vec3::new(0.0, 1000.0, 0.0),
        Vec3::new(0.0, 0.0, 1e-4),
    ]
    .iter()
    .enumerate()
    {
        sliver.nodes[i].pos = *q;
    }
    assert!(refused(&sliver, 1e-5).starts_with("tet 0 (0 1 2 3) is too flat"));
    let mut inv = good();
    inv.tets[0].nodes = [0, 2, 1, 3];
    assert!(
        refused(&inv, 1e-5).starts_with("tet 0 (0 2 1 3) is inverted: its rest volume -0.16666")
    );
    let mut far = good();
    far.nodes[1].pos.x = 1e300;
    far.nodes[2].pos.y = 1e300;
    far.nodes[3].pos.z = 1e300;
    // Since round 3 the coordinate bound refuses this first (the tet's own size check stays
    // behind it, for a bound that is ever raised).
    assert_eq!(
        refused(&far, 1e-5),
        "node 1: its position (1e300, 1e1, 0e0) is beyond MAX_COORD_M = 1000000 m in a coordinate"
    );
    let mut same = good();
    same.tets.push(Tet {
        nodes: [0, 2, 3, 1],
        solid: "s".into(),
    });
    assert_eq!(
        refused(&same, 1e-5),
        "tet 1 (0 2 3 1) is on the same four nodes as tet 0"
    );

    // A node shared by tets of two solids.
    let mut two = good();
    two.solids.push(solid("u", 1e6, 0.02, 0.0));
    add_node(&mut two, 4, at + Vec3::new(1.0, 1.0, 1.0), 1.0);
    add_tet(&mut two, [1, 2, 3, 4], "u");
    let e = refused(&two, 1e-5);
    assert_eq!(
        e,
        "tet 1 (1 2 3 4): node 1 is shared by tets of two solids, `s` and `u`"
    );

    // A zone so dense its lock-up strain is not above the yield strain: phi = 4 m / (V 7850).
    let mut dense = good();
    for n in dense.nodes.iter_mut() {
        n.mass = 200.0; // phi = 800 / (7850 / 6) = 0.611, eps_D = 1 - 1.4 phi = 0.144
    }
    dense.solids[0].yield_strain = 0.2;
    assert!(refused(&dense, 1e-5).starts_with("solid `s`: the zone's lock-up strain eps_D = 0.14"));
}

// ---------------------------------------------------------------------------------------
// R5: stability

/// R5: the closed-form tangent bound is at least the largest eigenvalue (in size) of a
/// finite-difference tangent of this file's own stress, for random F (stretch 0.3 to 3, shear,
/// rotation), and it is tight at rest (2 mu + 3 lambda).
#[test]
fn tangent_bound_covers_the_finite_difference_tangent() {
    let (plateau, ys) = (2e6, 0.02);
    let (mu, lam) = lame(plateau, ys);
    let law = law_of(plateau, ys);
    let fd_max = |f: &M| -> f64 {
        let eps = 1e-6;
        let mut a = vec![vec![0.0; 9]; 9];
        for k in 0..9 {
            let (mut fp, mut fm) = (*f, *f);
            fp[k / 3][k % 3] += eps;
            fm[k / 3][k % 3] -= eps;
            let (pp, pm) = (nh_p(&fp, mu, lam), nh_p(&fm, mu, lam));
            for i in 0..9 {
                a[i][k] = (pp[i / 3][i % 3] - pm[i / 3][i % 3]) / (2.0 * eps);
            }
        }
        for i in 0..9 {
            for k in 0..i {
                let m = 0.5 * (a[i][k] + a[k][i]);
                a[i][k] = m;
                a[k][i] = m;
            }
        }
        eig_sym(a).into_iter().map(f64::abs).fold(0.0, f64::max)
    };
    let at_rest = fd_max(&eye());
    let bound_rest = solver_m5::tangent_bound(&eye(), &law);
    assert!((at_rest - (2.0 * mu + 3.0 * lam)).abs() <= FD * at_rest);
    assert!(
        bound_rest >= at_rest * (1.0 - FD) && bound_rest <= at_rest * (1.0 + FD),
        "bound at rest {bound_rest} vs {at_rest}"
    );
    let mut rng = Rng(0x5eed_0004);
    let mut worst = f64::INFINITY;
    for _ in 0..200 {
        let s = [
            rng.range(0.3f64.ln(), 3f64.ln()).exp(),
            rng.range(0.3f64.ln(), 3f64.ln()).exp(),
            rng.range(0.3f64.ln(), 3f64.ln()).exp(),
        ];
        let mut shear = eye();
        shear[0][2] = rng.range(-1.0, 1.0);
        let d: M = [[s[0], 0.0, 0.0], [0.0, s[1], 0.0], [0.0, 0.0, s[2]]];
        let f = mmul(&mmul(&rng.rotation(), &mmul(&shear, &d)), &rng.rotation());
        let exact = fd_max(&f);
        let bound = solver_m5::tangent_bound(&f, &law);
        assert!(
            bound >= exact * (1.0 - FD),
            "bound {bound} below the tangent's {exact} at F = {f:?}"
        );
        worst = worst.min(bound / exact);
    }
    assert!(worst.is_finite() && worst >= 1.0 - FD);
}

/// R5a, closed form for one tet on the unit right corner (rest gradients -(1,1,1), e1, e2, e3):
/// node 0 has the largest Gershgorin sum, V A |g_0| S with V = 1/6, A = 2 (2 mu + 3 lambda),
/// |g_0| = sqrt 3, S = 3 + sqrt 3, so w2 = that / m. Just below the limit the solver is built
/// and runs 10 000 substeps bounded; at and just above it, it is refused.
#[test]
fn step_limit_closed_form_below_runs_above_is_refused() {
    let (plateau, ys, m) = (1e6, 0.02, 2.0);
    let (mu, lam) = lame(plateau, ys);
    let at = Vec3::new(0.0, 10.0, 0.0);
    let b = one_tet(plateau, ys, 0.0, 1.0, m, at);
    let probe = SolverM5::from_body(&b, 1e-9).unwrap();
    let s3 = 3f64.sqrt();
    let w2 = (1.0 / 6.0) * 2.0 * (2.0 * mu + 3.0 * lam) * s3 * (3.0 + s3) / m;
    assert!((probe.omega_max() * probe.omega_max() - w2).abs() <= ROUND * w2);
    let limit = 2.0 / w2.sqrt();
    assert!((probe.step_limit() - limit).abs() <= ROUND * limit);

    let e = refused(&b, limit * 1.001);
    assert!(
        e.starts_with("physics step h = dt/substeps = ")
            && e.contains("reaches the m5 stability limit")
            && e.ends_with("raise --substeps or lower --dt"),
        "{e}"
    );
    assert!(
        SolverM5::from_body(&b, limit).is_err(),
        "h = limit itself is refused (h omega < 2 is strict)"
    );

    let h = limit * 0.999;
    let mut s = SolverM5::from_body(&b, h).unwrap();
    let mut pos = positions(&b);
    let mut vel = vec![Vec3::ZERO; 4];
    // A small kick: 1e-3 strain rate times the step.
    vel[1] = Vec3::new(1e-3 / (100.0 * h), 0.0, 0.0);
    let mass = vec![m; 4];
    let e0 = 0.5 * m * vel[1].dot(vel[1]);
    for k in 0..10_000 {
        s.step(
            State {
                pos: &mut pos,
                vel: &mut vel,
                beams: &[],
                mass: &mass,
                gravity: 0.0,
            },
            h,
        );
        if k % 100 == 0 {
            assert_eq!(s.check(), None, "substep {k}");
        }
    }
    assert_eq!(s.check(), None);
    // Bounded by the shadow-energy bound of docs/M5-SOLVER.md section 5 with the Gershgorin
    // frequency of the tangent at rest without the headroom (still an upper bound on the true
    // one): h omega = 0.999 * 2 / sqrt(2), q = h omega / 2 = 0.706, (1 + q)/(1 - q) = 5.8, with
    // R9's allowance for the nonlinear terms.
    let q = h * (w2 / solver_m5::STIFFNESS_HEADROOM).sqrt() / 2.0;
    let cap = e0 * (1.0 + q) / (1.0 - q) * (1.0 + R9_NONLINEAR);
    let ke: f64 = vel.iter().map(|v| 0.5 * m * v.dot(*v)).sum();
    let eel = s.elastic_energy(&pos);
    assert!(
        ke + eel <= cap,
        "energy {} after 10 000 substeps above {cap} (start {e0})",
        ke + eel
    );
}

/// R5b: a tet sheared during the run past the per-substep bound ends with exactly
/// `stiffness bound exceeded at substep <n>: h omega = <value>`, value >= 2, at the substep the
/// closed form predicts. The shear is imposed (the nodes are put on `F = I + g e2 e1^T`, g =
/// 0.01 k, before substep k + 1), at h = 0.9 times the construction limit. With no plastic flow
/// (plateau / E = 0.69, far above the shear stresses here), the bound at node 0 is
/// V A(g) |g_0| S / m with A(g) = mu + (3 lambda + mu) / s_min(g)^2 (Je = 1),
/// s_min(g) = sqrt(1 + g^2/4) - g/2, and it trips at the first g with h^2 w2 >= 4.
#[test]
fn a_tet_stretched_past_the_bound_ends_with_the_named_error() {
    let (plateau, ys) = (690.0, 0.69);
    let (mu, lam) = lame(plateau, ys);
    let at = Vec3::new(0.0, 10.0, 0.0);
    let b = one_tet(plateau, ys, 0.0, 1.0, 1.0, at);
    let probe = SolverM5::from_body(&b, 1e-9).unwrap();
    let h = 0.9 * probe.step_limit();
    let mut s = SolverM5::from_body(&b, h).unwrap();
    let rest = positions(&b);
    let mass = vec![1.0; 4];
    let s3 = 3f64.sqrt();
    let trips = |g: f64| {
        let smin = (1.0 + g * g / 4.0).sqrt() - g / 2.0;
        let a = mu + (3.0 * lam + mu) / (smin * smin);
        h * h * (a * s3 * (3.0 + s3) / 6.0) >= 4.0
    };
    let k_pred = (0..1000u64)
        .find(|&k| trips(0.01 * k as f64))
        .expect("the closed form never trips");
    let mut msg = None;
    for k in 0..1000u64 {
        let g = 0.01 * k as f64;
        let mut pos: Vec<Vec3> = rest
            .iter()
            .map(|x| {
                let r = *x - at;
                at + Vec3::new(r.x, r.y + g * r.x, r.z)
            })
            .collect();
        let mut vel = vec![Vec3::ZERO; 4];
        s.step(
            State {
                pos: &mut pos,
                vel: &mut vel,
                beams: &[],
                mass: &mass,
                gravity: 0.0,
            },
            h,
        );
        if let Some(e) = s.check() {
            msg = Some((k, e));
            break;
        }
    }
    let (k, e) = msg.expect("the bound never tripped");
    let prefix = "stiffness bound exceeded at substep ";
    assert!(e.starts_with(prefix), "{e}");
    let (n, x) = e[prefix.len()..].split_once(": h omega = ").unwrap();
    let n: u64 = n.parse().unwrap();
    let x: f64 = x.parse().unwrap();
    assert_eq!(n, k + 1, "the error names the substep it happened in");
    assert_eq!(
        k,
        k_pred,
        "tripped at shear {} instead of the closed form's {}",
        0.01 * k as f64,
        0.01 * k_pred as f64
    );
    assert!(x >= 2.0, "h omega {x}");
    assert!(
        k > 10,
        "it tripped at shear {}, before the shear had grown",
        0.01 * k as f64
    );
    assert_eq!(s.plastic_strain(0), 0.0);
}

// ---------------------------------------------------------------------------------------
// R6, R7

struct Counting {
    inner: SolverM5,
    calls: Rc<Cell<u64>>,
}

impl Solver for Counting {
    fn step(&mut self, s: State<'_>, h: f64) {
        self.calls.set(self.calls.get() + 1);
        self.inner.step(s, h);
    }
    fn handles_solids(&self) -> bool {
        self.inner.handles_solids()
    }
    fn check(&self) -> Option<String> {
        self.inner.check()
    }
}

/// R6: through `Sim::from_body_with_solver`, the sim calls the m5 solver once per substep and
/// its tets act: a block stretched along x at the start contracts.
#[test]
fn m5_solver_plugs_in() {
    let b = block_2x2x2(6.9e8, 0.69, 0.05);
    let cfg = Config {
        dt: 1e-4,
        substeps: 100,
        gravity: 0.0,
        duration: 1.0,
        ..Config::default()
    };
    let h = cfg.dt / cfg.substeps as f64;
    let calls = Rc::new(Cell::new(0));
    let solver = Counting {
        inner: SolverM5::from_body(&b, h).unwrap(),
        calls: calls.clone(),
    };
    let mut sim = Sim::from_body_with_solver(cfg, &b, Box::new(solver)).unwrap();
    for p in sim.pos.iter_mut() {
        p.x *= 1.001;
    }
    let extent = |s: &Sim| {
        s.pos.iter().map(|p| p.x).fold(f64::MIN, f64::max)
            - s.pos.iter().map(|p| p.x).fold(f64::MAX, f64::min)
    };
    let start = extent(&sim);
    for _ in 0..2 {
        sim.advance();
        sim.check().unwrap();
    }
    assert_eq!(calls.get(), 200);
    assert!(
        extent(&sim) < start,
        "the stretched block did not contract: {} vs {start}",
        extent(&sim)
    );
}

/// The CSV of a run, and the final positions and velocities as raw bits.
fn run_csv(mut sim: Sim, steps: u64) -> (String, Vec<u64>) {
    let mut out = vec![sim.sample()];
    for _ in 0..steps {
        sim.advance();
        sim.check().unwrap();
        out.push(sim.sample());
    }
    let bits = sim
        .pos
        .iter()
        .chain(&sim.vel)
        .flat_map(|v| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()])
        .collect();
    (to_csv_with_broken(&out), bits)
}

/// R7: two runs of the same file give the same bytes; and a body with no tets under the m5
/// solver gives exactly the default solver's CSV on the cube scenario (the same integrator on
/// the same beams, in the same order: exact, not within a tolerance).
#[test]
fn m5_same_input_same_bytes() {
    let mut b = block_2x2x2(1e6, 0.02, 0.05);
    b.materials.push(Material {
        name: "m".into(),
        stiffness: 1e5,
        damping: 1.0,
        yield_strain: None,
        break_strain: None,
    });
    add_grid(&mut b, 3, 2, 2, 0.2, Vec3::new(0.0, 100.0, 0.0), 1.0);
    b.nodes.sort_by_key(|n| n.id);
    b.nodes.dedup_by_key(|n| n.id);
    add_beam_cells(&mut b, 2..3, 2, 2, "m");
    let text = format::write(&b);
    let cfg = Config {
        dt: 1e-4,
        substeps: 10,
        gravity: 9.81,
        duration: 1.0,
        ..Config::default()
    };
    let h = cfg.dt / cfg.substeps as f64;
    let run = || {
        let body = format::parse(&text).unwrap();
        let s = SolverM5::from_body(&body, h).unwrap();
        let mut sim = Sim::from_body_with_solver(cfg.clone(), &body, Box::new(s)).unwrap();
        for v in sim.vel.iter_mut() {
            *v = Vec3::new(0.3, -2.0, 0.1);
        }
        sim.vel[0] = Vec3::new(-3.0, 1.0, 2.0);
        run_csv(sim, 50)
    };
    let (a, c) = (run(), run());
    assert!(a.0.lines().count() == 52);
    assert!(a.0 == c.0, "two runs of the same file differ");
    assert!(
        a.1 == c.1,
        "two runs of the same file end in different bits"
    );

    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/scenarios/cube.bsn");
    let cube = format::parse(&std::fs::read_to_string(path).unwrap()).unwrap();
    let cfg = Config::default();
    let h = cfg.dt / cfg.substeps as f64;
    let steps = 500;
    let default = run_csv(Sim::from_body(cfg.clone(), &cube).unwrap(), steps);
    let m5 = run_csv(
        Sim::from_body_with_solver(
            cfg.clone(),
            &cube,
            Box::new(SolverM5::from_body(&cube, h).unwrap()),
        )
        .unwrap(),
        steps,
    );
    assert_eq!(default.0.lines().count(), 502);
    assert!(
        default.0 == m5.0,
        "the m5 solver with no tets differs from the default on the cube"
    );
    assert!(default.1 == m5.1, "the final state differs in its bits");
}

// ---------------------------------------------------------------------------------------
// R9: the coupling creates no energy

/// docs/M5-SOLVER.md section 5: a block of tets (2 x 2 x 2 cells, 48 tets) and a block of beams
/// (2 x 2 x 2 cells, all node pairs of each cell, each pair once) joined at the 9 nodes of one
/// face; no yield (plateau 1e12 Pa), no damping, no gravity, 100 m up. Start: a uniform shear and
/// stretch of 1e-3 plus a rigid velocity and spin. h = (2 / omega_max) / 10 with the solver's own
/// omega_max, 20 000 substeps. At every substep E <= E0 (1 + q)/(1 - q)(1 + 1e-2), q = 0.1, with
/// E = kinetic + beam + tet energy in this file's own formulas; linear and angular momentum to
/// 1e-9 of their scale.
#[test]
fn m5_coupling_creates_no_energy() {
    let (plateau, ys) = (1e12, 0.5);
    let (mu, lam) = lame(plateau, ys);
    let c = 0.2;
    let mut b = empty_body();
    b.solids.push(solid("s", plateau, ys, 0.0));
    b.materials.push(Material {
        name: "m".into(),
        stiffness: 1e11,
        damping: 0.0,
        yield_strain: None,
        break_strain: None,
    });
    add_grid(&mut b, 4, 2, 2, c, Vec3::new(0.0, 100.0, 0.0), 1.0);
    add_tet_cells(&mut b, 0..2, 2, 2, "s");
    add_beam_cells(&mut b, 2..4, 2, 2, "m");
    assert_eq!(b.tets.len(), 48);
    let rest = positions(&b);
    let idx = |id: u32| b.nodes.iter().position(|n| n.id == id).unwrap();
    let shared: Vec<usize> = (0..b.nodes.len())
        .filter(|&i| {
            let id = b.nodes[i].id;
            b.tets.iter().any(|t| t.nodes.contains(&id))
                && b.beams.iter().any(|bm| bm.a == id || bm.b == id)
        })
        .collect();
    assert_eq!(shared.len(), 9);

    let probe = SolverM5::from_body(&b, 1e-15).unwrap();
    let omega = probe.omega_max();
    let h = (2.0 / omega) / 10.0;
    let q = h * omega / 2.0;
    let mut s = SolverM5::from_body(&b, h).unwrap();
    let mass: Vec<f64> = b.nodes.iter().map(|n| n.mass).collect();
    let n = rest.len();
    let xc: Vec3 = rest.iter().fold(Vec3::ZERO, |a, x| a + *x) * (1.0 / n as f64);
    let g: M = [[1.0, 0.5, 0.0], [0.3, -0.5, 0.2], [0.0, 0.4, 0.8]];
    let mut pos: Vec<Vec3> = rest.iter().map(|x| *x + mvec(&g, *x - xc) * 1e-3).collect();
    let spin = Vec3::new(30.0, -60.0, 45.0);
    let v0 = Vec3::new(10.0, -5.0, 8.0);
    let mut vel: Vec<Vec3> = pos.iter().map(|x| v0 + spin.cross(*x - xc)).collect();
    let beams = crash_sim::format::Body::build(&b).beams;

    // This file's own energy: tets on their rest shape (no plastic flow at these strains).
    let tet_rest: Vec<([usize; 4], M, f64)> = b
        .tets
        .iter()
        .map(|t| {
            let ns = t.nodes.map(idx);
            let dm = [
                rest[ns[1]] - rest[ns[0]],
                rest[ns[2]] - rest[ns[0]],
                rest[ns[3]] - rest[ns[0]],
            ];
            let dmm: M = [
                [dm[0].x, dm[1].x, dm[2].x],
                [dm[0].y, dm[1].y, dm[2].y],
                [dm[0].z, dm[1].z, dm[2].z],
            ];
            (ns, minv(&dmm), mdet(&dmm) / 6.0)
        })
        .collect();
    let tet_energy = |pos: &[Vec3]| -> f64 {
        tet_rest
            .iter()
            .map(|(ns, dmi, v)| {
                let d = [
                    pos[ns[1]] - pos[ns[0]],
                    pos[ns[2]] - pos[ns[0]],
                    pos[ns[3]] - pos[ns[0]],
                ];
                let ds: M = [
                    [d[0].x, d[1].x, d[2].x],
                    [d[0].y, d[1].y, d[2].y],
                    [d[0].z, d[1].z, d[2].z],
                ];
                v * nh_w(&mmul(&ds, dmi), mu, lam)
            })
            .sum()
    };
    let energy = |pos: &[Vec3], vel: &[Vec3]| -> (f64, f64) {
        let ke: f64 = vel
            .iter()
            .zip(&mass)
            .map(|(v, m)| 0.5 * m * v.dot(*v))
            .sum();
        let be: f64 = beams
            .iter()
            .map(|bm| {
                let st = (pos[bm.a] - pos[bm.b]).length() - bm.rest;
                0.5 * bm.stiffness * st * st
            })
            .sum();
        let te = tet_energy(pos);
        (ke + be + te, te)
    };
    let momentum = |pos: &[Vec3], vel: &[Vec3]| -> (Vec3, Vec3, f64, f64) {
        let mut p = Vec3::ZERO;
        let mut l = Vec3::ZERO;
        let (mut ps, mut ls) = (0.0, 0.0);
        for i in 0..pos.len() {
            p += vel[i] * mass[i];
            l += (pos[i] - xc).cross(vel[i]) * mass[i];
            ps += mass[i] * vel[i].length();
            ls += mass[i] * (pos[i] - xc).length() * vel[i].length();
        }
        (p, l, ps, ls)
    };
    let (e0, te0) = energy(&pos, &vel);
    let (p0, l0, ps, ls) = momentum(&pos, &vel);
    // Energy sits in the motion, the beams and the tets at the start.
    let ke0: f64 = vel
        .iter()
        .zip(&mass)
        .map(|(v, m)| 0.5 * m * v.dot(*v))
        .sum();
    let be0 = e0 - ke0 - te0;
    assert!(
        ke0 > R9_SHARE * e0 && be0 > R9_SHARE * e0 && te0 > R9_SHARE * e0,
        "start energy: kinetic {ke0}, beams {be0}, tets {te0}"
    );
    let cap = e0 * (1.0 + q) / (1.0 - q) * (1.0 + R9_NONLINEAR);
    let mut te_swing = 0.0f64;
    let mut e_max_seen = e0;
    for k in 0..20_000 {
        s.step(
            State {
                pos: &mut pos,
                vel: &mut vel,
                beams: &beams,
                mass: &mass,
                gravity: 0.0,
            },
            h,
        );
        let (e, te) = energy(&pos, &vel);
        assert!(
            e <= cap,
            "substep {}: energy {e} above the bound {cap} (E0 {e0})",
            k + 1
        );
        e_max_seen = e_max_seen.max(e);
        te_swing = te_swing.max((te - te0).abs());
        let (p, l, _, _) = momentum(&pos, &vel);
        assert!(
            (p - p0).length() <= R9_MOMENTUM * ps,
            "substep {}: linear momentum moved by {}",
            k + 1,
            (p - p0).length()
        );
        assert!(
            (l - l0).length() <= R9_MOMENTUM * ls,
            "substep {}: angular momentum moved by {}",
            k + 1,
            (l - l0).length()
        );
    }
    assert_eq!(s.check(), None);
    assert_eq!(s.plastic_strain(0), 0.0);
    // Not vacuous: the tets' energy changed by much more than the allowance (a wave crossed).
    assert!(
        te_swing > R9_WAVE * te0,
        "the tet energy barely moved: {te_swing} of {te0}"
    );
    println!(
        "m5_coupling: h = {h:e} s, q = {q:.6}, E0 = {e0:e} J (tets {te0:e} J), max E / E0 = {:.9}, bound / E0 = {:.6}, largest tet energy swing / tet E0 = {:.4}",
        e_max_seen / e0,
        cap / e0,
        te_swing / te0
    );
}

// ---------------------------------------------------------------------------------------
// R12: the mesh rule

/// R12: `cut_cell` gives 6 tets of positive volume that fill the cell, all along the main
/// diagonal 0-7, and two neighbouring cells triangulate their shared face the same way (the
/// mesh is conforming), along x, y and z.
#[test]
fn cut_cell_is_six_positive_tets_and_conforming() {
    let corner = |k: u32| Vec3::new((k & 1) as f64, ((k >> 1) & 1) as f64, ((k >> 2) & 1) as f64);
    let tets = solver_m5::cut_cell([0, 1, 2, 3, 4, 5, 6, 7]);
    let mut total = 0.0;
    for (i, t) in tets.iter().enumerate() {
        let p = t.map(corner);
        let v = (p[1] - p[0]).dot((p[2] - p[0]).cross(p[3] - p[0])) / 6.0;
        assert_eq!(v, 1.0 / 6.0, "tet {i} {t:?}");
        assert!(t.contains(&0) && t.contains(&7));
        total += v;
        for u in &tets[..i] {
            let (mut a, mut c) = (*t, *u);
            a.sort();
            c.sort();
            assert_ne!(a, c);
        }
    }
    assert!(
        (total - 1.0).abs() <= ROUND,
        "the tets fill {total} of the cell"
    );
    // Faces on the plane `axis = side` of a cell at offset `off`, as sorted corner positions.
    let faces_on = |off: [usize; 3], axis: usize, side: f64| -> Vec<[(i64, i64, i64); 3]> {
        let mut out = Vec::new();
        for t in solver_m5::cut_cell(corners(off[0], off[1], off[2])) {
            let pts: Vec<(i64, i64, i64)> = t
                .iter()
                .map(|&id| ((id / 100) as i64, ((id / 10) % 10) as i64, (id % 10) as i64))
                .collect();
            for skip in 0..4 {
                let mut f: Vec<(i64, i64, i64)> =
                    (0..4).filter(|&k| k != skip).map(|k| pts[k]).collect();
                let coord = |p: &(i64, i64, i64)| [p.0, p.1, p.2][axis] as f64;
                if f.iter().all(|p| coord(p) == side) {
                    f.sort();
                    out.push([f[0], f[1], f[2]]);
                }
            }
        }
        out.sort();
        out
    };
    for axis in 0..3 {
        let mut next = [0usize; 3];
        next[axis] = 1;
        let a = faces_on([0, 0, 0], axis, 1.0);
        let b = faces_on(next, axis, 1.0);
        assert_eq!(a.len(), 2, "a cell face is cut into two triangles");
        assert_eq!(
            a, b,
            "the cells disagree on their shared face along axis {axis}"
        );
    }
}

/// R12 with the cap: a 5 x 5 x 2 zone is exactly `m5_max_tets` tets and is taken; one more cell
/// row is refused by the cap.
#[test]
fn a_zone_of_m5_max_tets_is_taken_and_one_more_cell_is_not() {
    let mut b = empty_body();
    b.solids.push(solid("s", 1e6, 0.02, 0.0));
    add_grid(&mut b, 5, 5, 3, 0.2, Vec3::new(0.0, 100.0, 0.0), 1.0);
    add_tet_cells(&mut b, 0..5, 5, 2, "s");
    assert_eq!(b.tets.len(), M5_MAX_TETS);
    let s = SolverM5::from_body(&b, 1e-6).unwrap();
    assert_eq!(s.tet_count(), 300);
    for t in solver_m5::cut_cell(corners(0, 0, 2)) {
        add_tet(&mut b, t, "s");
    }
    assert!(refused(&b, 1e-6).starts_with("the body has 306 tets, more than the m5 solver takes"));
}
