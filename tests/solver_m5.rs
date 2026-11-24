//! The m5 crash-zone solver (docs/M5-SOLVER.md section 9, R1 to R12 except R8; docs/PHYSICS.md
//! "Solids"). Every expected value here is a closed form, a formula written out in this file
//! independently of the crate (the Neo-Hookean energy and stress, the hardening law, the
//! lock-up line, the Gershgorin sums), an exact `==`, or a check definition the design note
//! names; no test takes the crate's own output as its reference (R11).
//!
//! Tolerance rules (R11). Each float in a comparison is one of these, an exact value, or a
//! number the design note names:

use crash_sim::solver::{Solver, SymplecticEuler};
use crash_sim::solver_m5::{self, SolverM5, J_MIN_FAIL, M5_MAX_TETS};
use crash_sim::vec3::Vec3;
use crash_sim::{Config, Sim};

mod m5_support;
use m5_support::*;

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
    assert!(line("m5_option2_method").contains(
        "an inverted tet or an exceeded stiffness bound ends the run with a named error"
    ));
    // R4e: the failure constant of decision 6 of the note.
    assert_eq!(J_MIN_FAIL, 0.05);
}

// ---------------------------------------------------------------------------------------
// R2, R3: the trait methods and the refusal

#[test]
fn default_solver_handles_no_solids_and_reports_nothing() {
    assert!(!SymplecticEuler::default().handles_solids());
    assert_eq!(SymplecticEuler::default().check(), None);
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
    let e = Sim::from_body_with_solver(cfg.clone(), &b, Box::new(SymplecticEuler::default()))
        .err()
        .unwrap();
    assert_eq!(e, want);
    assert_eq!(Sim::from_body(cfg.clone(), &b).err().unwrap(), want);
    assert_eq!(
        Sim::from_car_with_solver(cfg.clone(), &b, Box::new(SymplecticEuler::default()))
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
