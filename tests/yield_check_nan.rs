//! `Sim::apply_yield` and `Sim::check` give the same results as the plain arithmetic (one sqrt
//! per beam, `max_strain_unbroken` then `is_nan() || > limit`), including every NaN path (task
//! t-20261120-193454-2). The reference below is the rule with no squared-length shortcut: since
//! task t-20261122-013607 it is the constant-force compressive rule (crushed when
//! `(len - rest) / rest0 < -(y + YIELD_EPS)`, then `rest' = len + y rest0`); tension is as before
//! (`strain > y + YIELD_EPS`, then `rest' = len / (1 + y)`). The tests compare the real methods with it.

use crash_sim::{max_strain_unbroken, Beam, Config, Sim, MAX_STRAIN_LIMIT, YIELD_EPS};

/// `apply_yield` with no shortcut, on copies of the beams: (new beams, yielded count, refused list).
fn reference_yield(sim: &Sim) -> (Vec<Beam>, usize, Vec<usize>) {
    let mut beams = sim.beams.clone();
    let mut refused = sim.yield_refused.clone();
    let mut yielded = 0;
    for (i, bm) in beams.iter_mut().enumerate() {
        let Some(y) = sim.yield_strain[i] else {
            continue;
        };
        if sim.broken[i] {
            continue;
        }
        let len = (sim.pos[bm.a] - sim.pos[bm.b]).length();
        let strain = (len - bm.rest) / bm.rest;
        if strain.is_nan() {
            continue;
        }
        let t = y + YIELD_EPS;
        let new_rest = if strain > t {
            len / (1.0 + y)
        } else if (len - bm.rest) / sim.rest0[i] < -t {
            len + y * sim.rest0[i]
        } else {
            continue;
        };
        if len < 1e-12 || !(new_rest.is_finite() && new_rest > 0.0) {
            if !refused.contains(&i) {
                refused.push(i);
            }
            continue;
        }
        bm.rest = new_rest;
        yielded += 1;
    }
    (beams, yielded, refused)
}

/// The original `check` tail for the strain (positions are finite in these tests).
fn reference_check(sim: &Sim) -> Result<(), String> {
    if let Some(&i) = sim.yield_refused.first() {
        return Err(format!(
            "beam {i} would yield to a rest length of zero or negative (its nodes are at the same place)"
        ));
    }
    let strain = max_strain_unbroken(&sim.beams, &sim.pos, &sim.broken);
    if strain.is_nan() || strain > MAX_STRAIN_LIMIT {
        return Err(format!(
            "max beam strain {strain} exceeds limit {MAX_STRAIN_LIMIT}"
        ));
    }
    Ok(())
}

fn cube() -> Sim {
    Sim::new(Config::default())
}

fn same_beams(a: &[Beam], b: &[Beam]) -> bool {
    a.len() == b.len()
        && a.iter()
            .zip(b)
            .all(|(x, y)| x.rest.to_bits() == y.rest.to_bits())
}

/// Run both methods on `sim` and fail on any difference from the reference.
fn assert_same(label: &str, sim: &mut Sim) {
    let (ref_beams, ref_yielded, ref_refused) = reference_yield(sim);
    let ref_check_before = reference_check(sim);
    assert_eq!(
        sim.check(),
        ref_check_before,
        "[{label}] check before apply_yield"
    );
    let yielded = sim.apply_yield();
    assert_eq!(yielded, ref_yielded, "[{label}] yielded count");
    assert!(
        same_beams(&sim.beams, &ref_beams),
        "[{label}] rest lengths differ"
    );
    assert_eq!(sim.yield_refused, ref_refused, "[{label}] refused list");
    let ref_check_after = reference_check(sim);
    assert_eq!(
        sim.check(),
        ref_check_after,
        "[{label}] check after apply_yield"
    );
}

#[test]
fn nan_beams_behave_as_before() {
    // 1: a NaN rest length with finite positions: strain NaN. apply_yield must skip it (NaN
    // never exceeds) and check must report it (NaN is an error).
    let mut sim = cube();
    sim.yield_strain = vec![Some(0.01); sim.beams.len()];
    sim.beams[3].rest = f64::NAN;
    let before = sim.check();
    assert!(before.is_err(), "a NaN strain must fail check: {before:?}");
    assert_same("nan rest", &mut sim);
    assert_eq!(sim.apply_yield(), 0);
    let err = sim.check().unwrap_err();
    assert!(err.contains("NaN"), "error names the NaN strain: {err}");

    // 2: a NaN yield strain: never exceeds, rest unchanged, check Ok.
    let mut sim = cube();
    sim.yield_strain = vec![Some(f64::NAN); sim.beams.len()];
    sim.beams[1].rest *= 0.9;
    assert_same("nan yield", &mut sim);
    assert_eq!(sim.apply_yield(), 0);

    // 3: a zero rest length: strain is inf or NaN.
    let mut sim = cube();
    sim.yield_strain = vec![Some(0.01); sim.beams.len()];
    sim.beams[0].rest = 0.0;
    assert_same("zero rest", &mut sim);

    // 4: a negative rest length and an infinite one.
    let mut sim = cube();
    sim.yield_strain = vec![Some(0.01); sim.beams.len()];
    sim.beams[0].rest = -1.0;
    sim.beams[1].rest = f64::INFINITY;
    assert_same("negative and infinite rest", &mut sim);

    // 5: a broken beam with a NaN rest is skipped by both, a live one is not.
    let mut sim = cube();
    sim.yield_strain = vec![Some(0.01); sim.beams.len()];
    sim.beams[2].rest = f64::NAN;
    sim.broken[2] = true;
    assert_same("broken nan beam", &mut sim);
    assert!(sim.check().is_ok(), "a broken NaN beam is not judged");

    // 6: coincident nodes (the refused path).
    let mut sim = cube();
    sim.yield_strain = vec![Some(0.01); sim.beams.len()];
    let (a, b) = (sim.beams[0].a, sim.beams[0].b);
    sim.pos[b] = sim.pos[a];
    assert_same("coincident nodes", &mut sim);
    assert!(sim.check().is_err());
}

/// Seeded sweep: rest lengths set so that beam strains land on both sides of the yield and
/// check limits, including within 1e-13 and 1e-9 of them. Every case must equal the reference.
#[test]
fn boundary_sweep_matches_reference() {
    let mut state = 0x9E37_79B9_7F4A_7C15_u64;
    let mut next = move || {
        // splitmix64
        state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    };
    let unit = |r: u64| (r >> 11) as f64 / (1u64 << 53) as f64;
    let mut cases = 0;
    for case in 0..400 {
        let mut sim = cube();
        let y = [0.0, 1e-6, 0.01, 0.05, 0.3, 0.5, 0.89, 0.95][(next() % 8) as usize];
        sim.yield_strain = vec![Some(y); sim.beams.len()];
        for i in 0..sim.beams.len() {
            let bm = sim.beams[i];
            let limit = if case % 2 == 0 {
                y + YIELD_EPS
            } else {
                MAX_STRAIN_LIMIT
            };
            // Target strain: a fraction of the limit, or the limit plus or minus a hair.
            let target = match next() % 4 {
                0 => limit * (2.0 * unit(next()) - 1.0),
                1 => limit * (1.0 + 1e-13 * (unit(next()) - 0.5)),
                2 => -limit * (1.0 + 1e-13 * (unit(next()) - 0.5)),
                _ => limit * (1.0 + 1e-9 * (unit(next()) - 0.5)),
            };
            let len = (sim.pos[bm.a] - sim.pos[bm.b]).length();
            // strain = (len - rest) / rest = target  =>  rest = len / (1 + target)
            sim.beams[i].rest = len / (1.0 + target);
        }
        assert_same(&format!("sweep case {case} y={y}"), &mut sim);
        cases += 1;
    }
    assert_eq!(cases, 400);
}

/// Seeded sweep near the compressive limit, where the shortcut (taken only for
/// rest0 / 2 <= rest <= rest0) must agree with the exact rule: the cube is scaled by f so every
/// beam has len = f rest0, and each rest is set so (len - rest) / rest0 lands at a fraction of
/// -(y + YIELD_EPS) or within 1e-13 or 1e-9 of it, with rest on both sides of rest0 / 2 and rest0.
#[test]
fn compressive_boundary_sweep_matches_reference() {
    let mut state = 0x2545_F491_4F6C_DD1D_u64;
    let mut next = move || {
        state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    };
    let unit = |r: u64| (r >> 11) as f64 / (1u64 << 53) as f64;
    let mut yielded_total = 0;
    let mut skipped_total = 0;
    for case in 0..400 {
        let mut sim = cube();
        let y = [0.0, 1e-6, 0.01, 0.05, 0.3, 0.5, 0.89, 0.95][(next() % 8) as usize];
        sim.yield_strain = vec![Some(y); sim.beams.len()];
        let f = 0.3 + 0.8 * unit(next());
        for p in sim.pos.iter_mut() {
            *p = *p * f;
        }
        let limit = y + YIELD_EPS;
        for i in 0..sim.beams.len() {
            let bm = sim.beams[i];
            let len = (sim.pos[bm.a] - sim.pos[bm.b]).length();
            let c = match next() % 4 {
                0 => -limit * unit(next()),
                1 => -limit * (1.0 + 1e-13 * (unit(next()) - 0.5)),
                2 => -limit * (1.0 + 1e-9 * (unit(next()) - 0.5)),
                _ => -limit * (1.0 + 0.2 * unit(next())),
            };
            // c = (len - rest) / rest0  =>  rest = len - c rest0
            sim.beams[i].rest = len - c * sim.rest0[i];
        }
        let (_, ref_yielded, _) = reference_yield(&sim);
        yielded_total += ref_yielded;
        skipped_total += sim.beams.len() - ref_yielded;
        assert_same(&format!("compressive case {case} y={y} f={f}"), &mut sim);
    }
    assert!(
        yielded_total > 0 && skipped_total > 0,
        "the sweep must hit both sides: {yielded_total} yielded, {skipped_total} not"
    );
}
