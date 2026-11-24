//! Shared helpers of the m5 solver tests (moved from tests/solver_m5.rs; see its header for the tolerance rules).
#![allow(dead_code)]
use crash_sim::format::{Body, BodyBeam, Node, Solid, Tet};
use crash_sim::solver_m5::{self, Law, Mat3, SolverM5};
use crash_sim::vec3::Vec3;

/// Rounding of a few dozen f64 operations on values of one size, relative.
pub const ROUND: f64 = 1e-12;
/// A sum of many forces of one size that should cancel: relative to the largest nodal force.
pub const CANCEL: f64 = 1e-10;
/// R4: "a plateau at plateau_stress within 1%".
pub const R4_PLATEAU: f64 = 0.01;
/// The law along a path, checked through forces: the solve's 60 bisection steps leave `d` to
/// about 1e-17 of its bracket, and this file's own root-finding for the elastic stretch to
/// 1e-15; the rest is the rounding of the stress reconstruction. Relative.
pub const LAW: f64 = 1e-9;
/// R9: the named allowance for the nonlinear terms at strain 1e-3.
pub const R9_NONLINEAR: f64 = 1e-2;
/// R9: momentum conserved to 1e-9 of its scale.
pub const R9_MOMENTUM: f64 = 1e-9;
/// R9's "energy sits in both blocks", as a check definition of this file (not a tolerance on
/// a computed value): the kinetic, beam and tet energy at the start are each over 1% of E0.
pub const R9_SHARE: f64 = 0.01;
/// R9's "a wave crosses the shared face", as a check definition of this file: the tet energy
/// moves by more than 10% of its start at some substep, ten times the R9 allowance.
pub const R9_WAVE: f64 = 0.1;
/// A central finite difference of the stress with step 1e-6: truncation about 1e-12 of the
/// tangent, rounding about 1e-10; relative to the largest eigenvalue.
pub const FD: f64 = 1e-6;
/// A strain of 1e-8 in `mu F + (lambda ln J - mu) F^-T` cancels about 8 of 16 digits.
pub const SMALL_STRAIN: f64 = 1e-6;

// ---------------------------------------------------------------------------------------
// Independent maths (not the crate's)

pub type M = [[f64; 3]; 3];

pub fn mdet(m: &M) -> f64 {
    m[0][0] * m[1][1] * m[2][2] + m[0][1] * m[1][2] * m[2][0] + m[0][2] * m[1][0] * m[2][1]
        - m[0][2] * m[1][1] * m[2][0]
        - m[0][0] * m[1][2] * m[2][1]
        - m[0][1] * m[1][0] * m[2][2]
}

/// Inverse by Gauss-Jordan elimination with partial pivoting.
pub fn minv(m: &M) -> M {
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

pub fn mmul(a: &M, b: &M) -> M {
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

pub fn mt(a: &M) -> M {
    let mut c = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            c[i][j] = a[j][i];
        }
    }
    c
}

pub fn eye() -> M {
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
}

pub fn mvec(m: &M, v: Vec3) -> Vec3 {
    Vec3::new(
        m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
        m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
        m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z,
    )
}

pub fn fnorm(m: &M) -> f64 {
    m.iter().flatten().map(|x| x * x).sum::<f64>().sqrt()
}

/// Rotation by `angle` about the unit vector `axis` (Rodrigues).
pub fn rot(axis: Vec3, angle: f64) -> M {
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
pub fn lame(plateau: f64, yield_strain: f64) -> (f64, f64) {
    let e = plateau / yield_strain;
    (e / 2.6, e * 0.3 / (1.3 * 0.4))
}

/// W = mu/2 (tr C - 3) - mu ln J + lambda/2 (ln J)^2.
pub fn nh_w(f: &M, mu: f64, lam: f64) -> f64 {
    let j = mdet(f);
    let i1: f64 = f.iter().flatten().map(|x| x * x).sum();
    0.5 * mu * (i1 - 3.0) - mu * j.ln() + 0.5 * lam * j.ln() * j.ln()
}

/// P = mu F + (lambda ln J - mu) F^-T.
pub fn nh_p(f: &M, mu: f64, lam: f64) -> M {
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
pub fn eig_sym(mut a: Vec<Vec<f64>>) -> Vec<f64> {
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
pub fn smallest_stretch2(f: &M) -> f64 {
    let c = mmul(&mt(f), f);
    let v = eig_sym(c.iter().map(|r| r.to_vec()).collect());
    v.into_iter().fold(f64::INFINITY, f64::min)
}

/// A small deterministic generator (xorshift64*), uniform in [0, 1).
pub struct Rng(pub u64);
impl Rng {
    pub fn next(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        (self.0.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1u64 << 53) as f64
    }
    pub fn range(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * self.next()
    }
    pub fn unit(&mut self) -> Vec3 {
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
    pub fn rotation(&mut self) -> M {
        let axis = self.unit();
        let angle = self.range(-3.0, 3.0);
        rot(axis, angle)
    }
}

// ---------------------------------------------------------------------------------------
// Bodies

pub fn empty_body() -> Body {
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

pub fn solid(name: &str, plateau: f64, yield_strain: f64, damping: f64) -> Solid {
    Solid {
        name: name.into(),
        plateau_stress: plateau,
        yield_strain,
        damping_ratio: damping,
    }
}

pub fn add_node(b: &mut Body, id: u32, p: Vec3, mass: f64) {
    b.nodes.push(Node { id, pos: p, mass });
}

pub fn add_beam(b: &mut Body, a: u32, c: u32, material: &str) {
    let pa = b.nodes.iter().find(|n| n.id == a).unwrap().pos;
    let pc = b.nodes.iter().find(|n| n.id == c).unwrap().pos;
    b.beams.push(BodyBeam {
        a,
        b: c,
        material: material.into(),
        rest: (pa - pc).length(),
    });
}

pub fn add_tet(b: &mut Body, n: [u32; 4], s: &str) {
    b.tets.push(Tet {
        nodes: n,
        solid: s.into(),
    });
}

/// One tet on the unit right corner: nodes 0..3 at the origin and on the three axes, times
/// `size`, placed at `at`. Its rest edge matrix is `size` I, so `F = Ds / size`.
pub fn one_tet(plateau: f64, yield_strain: f64, damping: f64, size: f64, mass: f64, at: Vec3) -> Body {
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
pub fn gid(ix: usize, iy: usize, iz: usize) -> u32 {
    (ix * 100 + iy * 10 + iz) as u32
}

/// Nodes on the grid ix in 0..=nx, iy in 0..=ny, iz in 0..=nz, spacing `c`, from `at`.
pub fn add_grid(b: &mut Body, nx: usize, ny: usize, nz: usize, c: f64, at: Vec3, mass: f64) {
    for ix in 0..=nx {
        for iy in 0..=ny {
            for iz in 0..=nz {
                let p = at + Vec3::new(ix as f64 * c, iy as f64 * c, iz as f64 * c);
                add_node(b, gid(ix, iy, iz), p, mass);
            }
        }
    }
}

pub fn corners(ix: usize, iy: usize, iz: usize) -> [u32; 8] {
    let mut c = [0u32; 8];
    for (k, ck) in c.iter_mut().enumerate() {
        *ck = gid(ix + (k & 1), iy + ((k >> 1) & 1), iz + ((k >> 2) & 1));
    }
    c
}

/// The 6 tets of each cell (R12's rule, `solver_m5::cut_cell`) in the cell range.
pub fn add_tet_cells(b: &mut Body, xs: std::ops::Range<usize>, ny: usize, nz: usize, s: &str) {
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
pub fn add_beam_cells(b: &mut Body, xs: std::ops::Range<usize>, ny: usize, nz: usize, material: &str) {
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

pub fn positions(b: &Body) -> Vec<Vec3> {
    b.nodes.iter().map(|n| n.pos).collect()
}

/// P from the nodal forces: `P = -(1/V) sum_a f_a (x) X_a` (since `sum_a X_a (x) grad N_a = I`).
pub fn stress_from_forces(f: &[Vec3], rest: &[Vec3], volume: f64) -> M {
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

pub fn law_of(plateau: f64, yield_strain: f64) -> Law {
    // The lock-up strain does not enter the elastic law; 0.7 is the frozen cap.
    Law::new(plateau, yield_strain, 0.7)
}

pub fn as_m(m: &Mat3) -> M {
    *m
}

pub fn block_2x2x2(plateau: f64, ys: f64, damping: f64) -> Body {
    let mut b = empty_body();
    b.solids.push(solid("s", plateau, ys, damping));
    add_grid(&mut b, 2, 2, 2, 0.2, Vec3::new(0.0, 100.0, 0.0), 1.0);
    add_tet_cells(&mut b, 0..2, 2, 2, "s");
    b
}

pub fn refused(b: &Body, h: f64) -> String {
    match SolverM5::from_body(b, h) {
        Ok(_) => panic!("the body was accepted"),
        Err(e) => e,
    }
}
