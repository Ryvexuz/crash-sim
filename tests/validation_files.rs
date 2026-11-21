//! The validation car files (docs/STAGE6-M4-M5.md, gate 4.2, n20-08 and n20-08b): the car of test A
//! (`validation_car.bsn`, `validation_wall.bsn`) and the car of the second test
//! (`validation_car_b.bsn`, `validation_wall_b.bsn`).
//!
//! The five tests named by gate 4.2:
//! - `validation_wall_embeds_validation_car`, `validation_wall_b_embeds_validation_car_b`: the wall
//!   file is the car file byte for byte plus exactly one `plane` line, and the setup rules hold
//!   (groups, start velocity, wall 0.05 m ahead with normal (-1, 0, 0)).
//! - `validation_files_share_crush_physics`: P1 to P3 computed back from each file agree between
//!   the two cars within 1e-9 relative.
//! - `crush_physics_inside_frozen_ranges`: P1 to P3 of both files lie inside the `frozen: range`
//!   lines of docs/STAGE6-M4-M5.md, read from that file at run time.
//! - `crush_beams_one_length`: the `crush` beams of each file have one rest length within 5%.
//!
//! `validation_checks_reject_wrong_copies` is not one of the five: it runs the same checks on
//! deliberately wrong copies and requires each to fail, so a check that cannot fail is caught.
//!
//! P1 to P3 are computed back from the file as docs/STAGE6-M4-M5.md ("What is calibrated") defines
//! them: P1 = k L_c / E, P2 = yield_strain, P3 = c / (2 sqrt(k m_c)), with L_c the mean rest length
//! of the `crush` beams, m_c the mean mass of the distinct nodes at their ends and E the
//! `frozen: steel_E_pa` line.

use crash_sim::format::{parse, Body};
use crash_sim::vec3::Vec3;

/// Gate 4.2: P1 to P3 agree between the two cars within this relative difference.
const SHARE_REL_TOL: f64 = 1e-9;
/// docs/STAGE6-M4-M5.md, "What is calibrated": the `crush` beams of one file have the same rest
/// length within 5%, read as (longest - shortest) / mean.
const ONE_LENGTH_SPREAD: f64 = 0.05;
/// docs/STAGE6-M4-M5.md, "The run": the wall plane is 0.05 m ahead of the front-most point.
const WALL_GAP_M: f64 = 0.05;
/// The speed is compared with `speed_kmh / 3.6` within this relative difference (the file prints
/// the shortest decimal of the f64 quotient, so the two are in practice equal).
const SPEED_REL_TOL: f64 = 1e-12;
/// The impact speed of the second test's car, km/h: the NHTSA report's closing speed (EV5 VEHSPD
/// 56.1), recorded in validation/CAR_B.md. It is a constant here because this file never opens
/// the second test's reference files.
const SECOND_SPEED_KMH: f64 = 56.1;

fn read(rel: &str) -> String {
    let path = format!("{}/{rel}", env!("CARGO_MANIFEST_DIR"));
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"))
}

fn body_of(text: &str) -> Body {
    parse(text).unwrap_or_else(|e| panic!("does not parse: {e}"))
}

/// The value of the first line `key = <value>` of a `key = value` file, as a number.
fn meta_number(text: &str, key: &str) -> f64 {
    let prefix = format!("{key} = ");
    let line = text
        .lines()
        .find_map(|l| l.strip_prefix(prefix.as_str()))
        .unwrap_or_else(|| panic!("no `{key} = ` line"));
    line.trim()
        .parse()
        .unwrap_or_else(|_| panic!("`{key}` is not a number: {line}"))
}

/// The numbers after `frozen: <name> =` in docs/STAGE6-M4-M5.md; refuses a missing line.
fn frozen(doc: &str, name: &str) -> Vec<f64> {
    let prefix = format!("frozen: {name} = ");
    let line = doc
        .lines()
        .find_map(|l| l.strip_prefix(prefix.as_str()))
        .unwrap_or_else(|| panic!("docs/STAGE6-M4-M5.md has no `{prefix}` line"));
    let v: Vec<f64> = line
        .split_whitespace()
        .map(|t| {
            t.parse()
                .unwrap_or_else(|_| panic!("`{prefix}{line}`: {t} is not a number"))
        })
        .collect();
    assert!(!v.is_empty(), "`{prefix}` has no numbers");
    v
}

#[derive(Debug, Clone, Copy)]
struct Crush {
    p1_area_m2: f64,
    p2_yield_strain: f64,
    p3_damping_ratio: f64,
    n_beams: usize,
    shortest: f64,
    longest: f64,
    mean_len: f64,
}

/// P1 to P3 and the beam lengths of the `crush` material of a car, computed back from its numbers.
fn crush_of(body: &Body, steel_e: f64) -> Result<Crush, String> {
    let mat = body
        .materials
        .iter()
        .find(|m| m.name == "crush")
        .ok_or("no material `crush`")?;
    let yield_strain = mat
        .yield_strain
        .ok_or("material `crush` has no yield_strain")?;
    if mat.break_strain.is_some() {
        return Err("material `crush` has a break_strain (docs/STAGE6-M4-M5.md: none)".into());
    }
    let beams: Vec<_> = body
        .beams
        .iter()
        .filter(|b| b.material == "crush")
        .collect();
    if beams.is_empty() {
        return Err("no beam uses the material `crush`".into());
    }
    let mut ends: Vec<u32> = beams.iter().flat_map(|b| [b.a, b.b]).collect();
    ends.sort_unstable();
    ends.dedup();
    let mass_of = |id: u32| body.nodes.iter().find(|n| n.id == id).map(|n| n.mass);
    let mut total = 0.0;
    for id in &ends {
        total += mass_of(*id).ok_or("a crush beam names a missing node")?;
    }
    let mc = total / ends.len() as f64;
    let lens: Vec<f64> = beams.iter().map(|b| b.rest).collect();
    let mean_len = lens.iter().sum::<f64>() / lens.len() as f64;
    let shortest = lens.iter().cloned().fold(f64::INFINITY, f64::min);
    let longest = lens.iter().cloned().fold(0.0, f64::max);
    Ok(Crush {
        p1_area_m2: mat.stiffness * mean_len / steel_e,
        p2_yield_strain: yield_strain,
        p3_damping_ratio: mat.damping / (2.0 * (mat.stiffness * mc).sqrt()),
        n_beams: beams.len(),
        shortest,
        longest,
        mean_len,
    })
}

/// The wall file is the car file plus one plane line, and the setup rules hold for the car.
fn check_wall_embeds_car(car_text: &str, wall_text: &str, speed_kmh: f64) -> Result<(), String> {
    if !car_text.ends_with('\n') {
        return Err("the car file does not end with a newline".into());
    }
    if !wall_text.as_bytes().starts_with(car_text.as_bytes()) {
        return Err("the wall file does not start with the car file byte for byte".into());
    }
    let rest = &wall_text[car_text.len()..];
    let lines: Vec<&str> = rest.lines().collect();
    if lines.len() != 1 || !lines[0].starts_with("plane ") || !rest.ends_with('\n') {
        return Err(format!(
            "after the car file the wall file must hold exactly one `plane` line, got {lines:?}"
        ));
    }
    let car = parse(car_text).map_err(|e| format!("car does not parse: {e}"))?;
    let wall = parse(wall_text).map_err(|e| format!("wall does not parse: {e}"))?;
    if !car.planes.is_empty() {
        return Err("the car file has a plane".into());
    }
    if wall.planes.len() != 1 {
        return Err(format!(
            "the wall file has {} planes, not 1",
            wall.planes.len()
        ));
    }
    let mut stripped = wall.clone();
    stripped.planes.clear();
    if stripped != car {
        return Err(
            "parsed, the wall file differs from the car file by more than the plane".into(),
        );
    }
    let plane = &wall.planes[0];
    if (plane.normal.x, plane.normal.y, plane.normal.z) != (-1.0, 0.0, 0.0) {
        return Err(format!(
            "the wall normal is {:?}, not (-1, 0, 0)",
            plane.normal
        ));
    }
    let front_x = car
        .nodes
        .iter()
        .map(|n| n.pos.x)
        .fold(f64::NEG_INFINITY, f64::max);
    let gap = plane.point.x - front_x;
    if (gap - WALL_GAP_M).abs() > 1e-9 {
        return Err(format!(
            "the wall is {gap} m ahead of the front-most node, not {WALL_GAP_M}"
        ));
    }

    // Groups: all (every node), sensor, front (the front-most nodes), rear (the rear-most nodes).
    let group = |name: &str| -> Result<&Vec<u32>, String> {
        car.groups
            .iter()
            .find(|g| g.name == name)
            .map(|g| &g.nodes)
            .ok_or(format!("no group `{name}`"))
    };
    let pos_of = |id: u32| {
        car.nodes
            .iter()
            .find(|n| n.id == id)
            .map(|n| n.pos)
            .ok_or(format!("no node {id}"))
    };
    let all = group("all")?;
    let mut all_sorted = all.clone();
    all_sorted.sort_unstable();
    let mut ids: Vec<u32> = car.nodes.iter().map(|n| n.id).collect();
    ids.sort_unstable();
    if all_sorted != ids {
        return Err("group `all` is not every node".into());
    }
    let rear_x = car
        .nodes
        .iter()
        .map(|n| n.pos.x)
        .fold(f64::INFINITY, f64::min);
    for id in group("front")? {
        if pos_of(*id)?.x != front_x {
            return Err(format!("front group node {id} is not at the front-most x"));
        }
    }
    for id in group("rear")? {
        if pos_of(*id)?.x != rear_x {
            return Err(format!("rear group node {id} is not at the rear-most x"));
        }
    }
    if group("front")?.is_empty() || group("rear")?.is_empty() || group("sensor")?.is_empty() {
        return Err("an empty front, rear or sensor group".into());
    }
    // The sensor is outside the crush zone: none of its nodes is the end of a crush beam.
    for b in car.beams.iter().filter(|b| b.material == "crush") {
        for end in [b.a, b.b] {
            if group("sensor")?.contains(&end) {
                return Err(format!("sensor node {end} is inside the crush zone"));
            }
        }
    }

    // Start velocity: every node at (speed / 3.6, 0, 0), by one line on `all`; front toward +x.
    if car.velocities.len() != 1 || car.velocities[0].group != "all" {
        return Err("the car needs exactly one velocity line, on group `all`".into());
    }
    let v0 = speed_kmh / 3.6;
    let v: Vec3 = car.velocities[0].velocity;
    if ((v.x - v0) / v0).abs() > SPEED_REL_TOL || v.y != 0.0 || v.z != 0.0 {
        return Err(format!("the start velocity is {v:?}, not ({v0}, 0, 0)"));
    }
    // Wheels: the vehicle code gives a wheel the mean velocity of its mount nodes (docs/FORMAT.md),
    // so the wheels start at v0 as long as every mount node is in `all`.
    if car.wheels.len() != 4 {
        return Err(format!("{} wheels, not 4", car.wheels.len()));
    }
    for w in &car.wheels {
        if w.mount.iter().any(|m| !all.contains(m)) {
            return Err(format!("wheel {} has a mount node outside `all`", w.name));
        }
    }
    Ok(())
}

fn check_one_length(c: &Crush) -> Result<(), String> {
    let spread = (c.longest - c.shortest) / c.mean_len;
    if spread > ONE_LENGTH_SPREAD {
        return Err(format!(
            "{} crush beams run from {} to {} m: spread {spread} of the mean, more than {ONE_LENGTH_SPREAD}",
            c.n_beams, c.shortest, c.longest
        ));
    }
    Ok(())
}

fn check_inside_ranges(c: &Crush, doc: &str) -> Result<(), String> {
    for (name, value) in [
        ("crush.area_m2", c.p1_area_m2),
        ("crush.yield_strain", c.p2_yield_strain),
        ("crush.damping_ratio", c.p3_damping_ratio),
    ] {
        let r = frozen(doc, &format!("range {name}"));
        if r.len() != 2 {
            return Err(format!(
                "`frozen: range {name}` has {} numbers, not 2",
                r.len()
            ));
        }
        if value.is_nan() || value < r[0] || value > r[1] {
            return Err(format!("{name} = {value} is outside [{}, {}]", r[0], r[1]));
        }
    }
    Ok(())
}

fn check_share(a: &Crush, b: &Crush) -> Result<(), String> {
    for (name, x, y) in [
        ("P1 area_m2", a.p1_area_m2, b.p1_area_m2),
        ("P2 yield_strain", a.p2_yield_strain, b.p2_yield_strain),
        ("P3 damping_ratio", a.p3_damping_ratio, b.p3_damping_ratio),
    ] {
        // Exactly equal values agree; this also keeps 0 against 0 from becoming 0 / 0 = NaN.
        let rel = if x == y {
            0.0
        } else {
            (x - y).abs() / x.abs().max(y.abs())
        };
        if rel.is_nan() || rel > SHARE_REL_TOL {
            return Err(format!(
                "{name}: {x} against {y}, relative difference {rel} is above {SHARE_REL_TOL}"
            ));
        }
    }
    Ok(())
}

struct Files {
    car_a: String,
    wall_a: String,
    car_b: String,
    wall_b: String,
    doc: String,
    steel_e: f64,
}

fn files() -> Files {
    let doc = read("docs/STAGE6-M4-M5.md");
    let e = frozen(&doc, "steel_E_pa");
    assert_eq!(e.len(), 1, "frozen: steel_E_pa");
    Files {
        car_a: read("scenarios/validation_car.bsn"),
        wall_a: read("scenarios/validation_wall.bsn"),
        car_b: read("scenarios/validation_car_b.bsn"),
        wall_b: read("scenarios/validation_wall_b.bsn"),
        steel_e: e[0],
        doc,
    }
}

fn speed_a() -> f64 {
    meta_number(&read("validation/test-a.meta"), "speed_kmh")
}

#[test]
fn validation_wall_embeds_validation_car() {
    let f = files();
    let speed = speed_a();
    assert_eq!(speed, 56.2, "speed_kmh of validation/test-a.meta");
    check_wall_embeds_car(&f.car_a, &f.wall_a, speed).unwrap();
}

#[test]
fn validation_wall_b_embeds_validation_car_b() {
    let f = files();
    check_wall_embeds_car(&f.car_b, &f.wall_b, SECOND_SPEED_KMH).unwrap();
}

#[test]
fn validation_files_share_crush_physics() {
    let f = files();
    let a = crush_of(&body_of(&f.car_a), f.steel_e).unwrap();
    let b = crush_of(&body_of(&f.car_b), f.steel_e).unwrap();
    println!("A: {a:?}\nB: {b:?}");
    check_share(&a, &b).unwrap();
    // The two cars have different lengths and node masses, so the shared physics is not shared
    // raw numbers: the check is not vacuous only if the raw stiffness differs.
    let (ka, kb) = (
        body_of(&f.car_a)
            .materials
            .iter()
            .find(|m| m.name == "crush")
            .unwrap()
            .stiffness,
        body_of(&f.car_b)
            .materials
            .iter()
            .find(|m| m.name == "crush")
            .unwrap()
            .stiffness,
    );
    assert_ne!(
        ka, kb,
        "the two files have the same crush stiffness, so L_c is not what differs"
    );
}

#[test]
fn crush_physics_inside_frozen_ranges() {
    let f = files();
    for (name, text) in [
        ("validation_car.bsn", &f.car_a),
        ("validation_car_b.bsn", &f.car_b),
    ] {
        let c = crush_of(&body_of(text), f.steel_e).unwrap();
        println!("{name}: {c:?}");
        check_inside_ranges(&c, &f.doc).unwrap_or_else(|e| panic!("{name}: {e}"));
    }
}

#[test]
fn crush_beams_one_length() {
    let f = files();
    for (name, text) in [
        ("validation_car.bsn", &f.car_a),
        ("validation_car_b.bsn", &f.car_b),
    ] {
        let c = crush_of(&body_of(text), f.steel_e).unwrap();
        println!(
            "{name}: {} crush beams, {} to {} m",
            c.n_beams, c.shortest, c.longest
        );
        assert!(c.n_beams >= 2, "{name}: fewer than 2 crush beams");
        check_one_length(&c).unwrap_or_else(|e| panic!("{name}: {e}"));
    }
}

/// Two exactly equal values agree, zero included: the sweep's grid has damping ratio 0, so both
/// cars can carry P3 = 0, and 0 against 0 must not become a NaN relative difference.
#[test]
fn check_share_accepts_equal_values_zero_included() {
    let f = files();
    let a = crush_of(&body_of(&f.car_a), f.steel_e).unwrap();
    let zero = Crush {
        p3_damping_ratio: 0.0,
        ..a
    };
    check_share(&zero, &zero).unwrap();
    check_share(&a, &a).unwrap();
    // Equal zeros still do not hide a real difference in another quantity.
    let off = Crush {
        p1_area_m2: zero.p1_area_m2 * 1.01,
        ..zero
    };
    let e = check_share(&zero, &off).unwrap_err();
    assert!(e.contains("P1 area_m2"), "{e}");
    // Zero against a non-zero value is a difference (relative difference 1).
    let e = check_share(&zero, &a).unwrap_err();
    assert!(e.contains("P3 damping_ratio"), "{e}");
}

/// What one run of the wall file measures at the wall (`wall_load_of`).
#[derive(Debug)]
struct WallLoad {
    /// First substep with a node or tyre on or behind the wall, s (the pulse report's contact_s).
    contact_s: f64,
    /// Last substep of the window, s (contact + `frozen: window_s`).
    window_end_s: f64,
    /// Sum over the window of the wall's normal impulse on the nodes, and on the tyres, N s.
    node_impulse_ns: f64,
    tyre_impulse_ns: f64,
    /// Height of the wall-force resultant over the window: sum of impulse x node height / impulse.
    resultant_y_m: f64,
    /// The sprung centre of gravity's height (mass-weighted node y), weighted the same way.
    cg_y_m: f64,
    /// Substeps in the window with a node or tyre on or behind the ground (the report's rule).
    ground_contacts: usize,
    /// Passenger-cell pitch (deg, + = nose up), its largest and smallest value at every substep
    /// from the start to the window end, and when.
    pitch_max: (f64, f64),
    pitch_min: (f64, f64),
}

/// Pitch of the passenger cell, deg: the angle of the least-squares line of y against x over the
/// nodes of stations 0 to 3 (ids 0 to 31, every node behind the crush beams).
fn cell_pitch_deg(sim: &crash_sim::Sim, body: &Body) -> f64 {
    let idx: Vec<usize> = (0..32u32)
        .map(|id| body.node_index(id).expect("cell node"))
        .collect();
    let n = idx.len() as f64;
    let mx = idx.iter().map(|&i| sim.pos[i].x).sum::<f64>() / n;
    let my = idx.iter().map(|&i| sim.pos[i].y).sum::<f64>() / n;
    let sxy: f64 = idx
        .iter()
        .map(|&i| (sim.pos[i].x - mx) * (sim.pos[i].y - my))
        .sum();
    let sxx: f64 = idx.iter().map(|&i| (sim.pos[i].x - mx).powi(2)).sum();
    (sxy / sxx).atan().to_degrees()
}

/// Runs a wall file as the 4.4 command does (gravity 0, h = `frozen: h_s`, `frozen: duration_s`,
/// stepped one substep at a time) and measures the wall's load on the car, nodes and tyres.
///
/// A node's wall contact is an impulse (`collide::apply` moves it back onto the plane and changes
/// its velocity), so its load is measured as impulse. A second sim (`split`) is stepped by the
/// same operations as `Sim::integrate`'s car path, written out here with the public parts
/// (`Car::pre_step`, the default solver `SymplecticEuler`, `Car::post_step`, `collide::apply`),
/// with one difference: the contact pass is applied with the ground alone and then with the wall
/// alone, which per node is the same operations in the same order (`collide::apply` loops planes
/// inside the node loop), and the momentum change of each node in the wall pass is the wall's
/// impulse on it. A tyre's wall load is a force (`Wheel::load[1]`, at the wheel centre's height),
/// taken times h. The split sim's node positions and velocities are compared bit for bit with the
/// real sim's (`Sim::advance`) after every substep, so the measured run is the real run.
fn wall_load_of(body: &Body, doc: &str) -> WallLoad {
    use crash_sim::collide;
    use crash_sim::solver::{Solver, State, SymplecticEuler};
    use crash_sim::vehicle::Substep;
    let one = |name: &str| {
        let v = frozen(doc, name);
        assert_eq!(v.len(), 1, "frozen: {name}");
        v[0]
    };
    let (h, duration, window) = (one("h_s"), one("duration_s"), one("window_s"));
    assert!(
        body.materials.iter().all(|m| m.break_strain.is_none()),
        "a material has a break_strain: the split run below has no break pass"
    );
    let cfg = crash_sim::Config {
        duration,
        dt: h,
        substeps: 1,
        gravity: 0.0,
        ..crash_sim::Config::default()
    };
    let mut real = crash_sim::Sim::from_car(cfg.clone(), body).unwrap_or_else(|e| panic!("{e}"));
    let mut split = crash_sim::Sim::from_car(cfg.clone(), body).unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(split.planes.len(), 2, "the wall file has one plane");
    let (ground_plane, wall) = (split.planes[0], split.planes[1]);
    let mut solver = SymplecticEuler::default();
    let gv = split.gravity_vector();
    let mut extra: Vec<Vec3> = Vec::new();
    let touches = |sim: &crash_sim::Sim, planes: &[collide::Plane]| {
        collide::any_at_or_behind(&sim.pos, planes)
            || sim.car.as_ref().is_some_and(|c| {
                c.wheels
                    .iter()
                    .any(|w| planes.iter().any(|pl| pl.distance(w.pos) <= w.p.radius))
            })
    };
    let n = crash_sim::step_count(&cfg) as usize;
    let window_steps = (window / h * (1.0 + 1e-9)).floor() as usize;
    let sprung: f64 = split.mass.iter().sum();
    let p0 = cell_pitch_deg(&split, body);
    let (mut contact, mut j_sum, mut jy_sum, mut jcg_sum, mut ground) = (None, 0.0, 0.0, 0.0, 0);
    let (mut node_j, mut tyre_j) = (0.0, 0.0);
    let (mut pmax, mut pmin) = ((f64::NEG_INFINITY, 0.0), (f64::INFINITY, 0.0));
    for k in 1..=n {
        real.advance();
        real.check()
            .unwrap_or_else(|e| panic!("unstable at substep {k}: {e}"));
        // `Sim::integrate`'s car path, then its yield pass (`Sim::advance` with one substep).
        extra.clear();
        extra.extend(split.mass.iter().map(|&m| Vec3::new(m * gv.x, 0.0, 0.0)));
        let car = split.car.as_mut().expect("the validation car has wheels");
        car.pre_step(
            &split.pos,
            &split.vel,
            &split.mass,
            &split.planes,
            Substep { gravity: gv, h },
            &mut extra,
        );
        let tyre: Vec<(f64, f64)> = car.wheels.iter().map(|w| (w.load[1], w.pos.y)).collect();
        let state = State {
            pos: &mut split.pos,
            vel: &mut split.vel,
            beams: &split.beams,
            mass: &split.mass,
            gravity: -gv.y,
        };
        solver.step_with_forces(state, &extra, h);
        car.post_step(
            &mut split.pos,
            &mut split.vel,
            &split.mass,
            &split.planes,
            h,
        );
        collide::apply(&mut split.pos, &mut split.vel, &[ground_plane], h);
        let before = split.vel.clone();
        collide::apply(&mut split.pos, &mut split.vel, &[wall], h);
        split.apply_yield();
        assert!(
            solver.check().is_none(),
            "the split run's solver failed at substep {k}"
        );
        for i in 0..split.pos.len() {
            for (a, b) in [(real.pos[i], split.pos[i]), (real.vel[i], split.vel[i])] {
                assert!(
                    a.x.to_bits() == b.x.to_bits()
                        && a.y.to_bits() == b.y.to_bits()
                        && a.z.to_bits() == b.z.to_bits(),
                    "substep {k}, node {i}: the split run left the real run ({a:?} against {b:?})"
                );
            }
        }
        if contact.is_none() && touches(&real, &real.planes[1..]) {
            contact = Some(k);
        }
        let Some(kc) = contact else { continue };
        if k > kc + window_steps {
            break;
        }
        let pitch = cell_pitch_deg(&split, body) - p0;
        if pitch > pmax.0 {
            pmax = (pitch, k as f64 * h);
        }
        if pitch < pmin.0 {
            pmin = (pitch, k as f64 * h);
        }
        if touches(&real, &real.planes[..1]) {
            ground += 1;
        }
        let cg_y = split
            .pos
            .iter()
            .zip(&split.mass)
            .map(|(p, m)| p.y * m)
            .sum::<f64>()
            / sprung;
        for i in 0..split.pos.len() {
            let j = split.mass[i] * (split.vel[i] - before[i]).dot(wall.normal);
            j_sum += j;
            jy_sum += j * split.pos[i].y;
            jcg_sum += j * cg_y;
            node_j += j;
        }
        for &(fz, y) in &tyre {
            if fz > 0.0 {
                j_sum += fz * h;
                jy_sum += fz * h * y;
                jcg_sum += fz * h * cg_y;
                tyre_j += fz * h;
            }
        }
    }
    let kc = contact.expect("the car never reached the wall");
    assert!(
        kc + window_steps <= n,
        "the window runs past the end of the run"
    );
    assert!(j_sum > 0.0, "no wall impulse in the window");
    WallLoad {
        contact_s: kc as f64 * h,
        window_end_s: (kc + window_steps) as f64 * h,
        node_impulse_ns: node_j,
        tyre_impulse_ns: tyre_j,
        resultant_y_m: jy_sum / j_sum,
        cg_y_m: jcg_sum / j_sum,
        ground_contacts: ground,
        pitch_max: pmax,
        pitch_min: pmin,
    }
}

/// The car of test A at the wall: the wall-force resultant, measured over the pulse window from
/// the run itself (`wall_load_of`), acts below the sprung centre of gravity, so the impact cannot
/// pitch the car nose up (validation/CAR.md, "Pitch at the wall"); and no node or tyre reaches the
/// ground in the window. Test A only: this file never opens the second test's files.
#[test]
fn validation_wall_load_acts_below_the_centre_of_gravity() {
    let f = files();
    let body = body_of(&f.wall_a);
    let w = wall_load_of(&body, &f.doc);
    print_wall_load("validation_wall.bsn", &w);
    check_wall_load(&w).unwrap();
}

fn print_wall_load(name: &str, w: &WallLoad) {
    println!(
        "{name}: resultant_y_m={:.4} cg_y_m={:.4} arm_m={:+.4} node_impulse_ns={:.0} tyre_impulse_ns={:.0} ground_contacts={} pitch_deg max {:+.3} at {:.4} s, min {:+.3} at {:.4} s (every substep), window {:.4} to {:.4} s",
        w.resultant_y_m,
        w.cg_y_m,
        w.resultant_y_m - w.cg_y_m,
        w.node_impulse_ns,
        w.tyre_impulse_ns,
        w.ground_contacts,
        w.pitch_max.0,
        w.pitch_max.1,
        w.pitch_min.0,
        w.pitch_min.1,
        w.contact_s,
        w.window_end_s
    );
}

/// The wall load acts below the centre of gravity and nothing reaches the ground in the window.
fn check_wall_load(w: &WallLoad) -> Result<(), String> {
    // NaN on either side fails too (partial_cmp is None).
    if w.resultant_y_m.partial_cmp(&w.cg_y_m) != Some(std::cmp::Ordering::Less) {
        return Err(format!(
            "the wall load acts at {} m, not below the centre of gravity at {} m",
            w.resultant_y_m, w.cg_y_m
        ));
    }
    if w.ground_contacts != 0 {
        return Err(format!(
            "{} substeps in the window with a node or tyre on the ground",
            w.ground_contacts
        ));
    }
    Ok(())
}

/// Every check, run on a deliberately wrong copy of the real files, must fail with its own words.
#[test]
fn validation_checks_reject_wrong_copies() {
    let f = files();
    let good_a = crush_of(&body_of(&f.car_a), f.steel_e).unwrap();
    let sp = speed_a();

    // The wall file: a wall too far ahead; a second plane; a changed car line; a wrong speed.
    let far = f.wall_a.replace("plane 2.153 ", "plane 2.253 ");
    assert_ne!(far, f.wall_a);
    let e = check_wall_embeds_car(&f.car_a, &far, sp).unwrap_err();
    assert!(e.contains("ahead of the front-most node"), "{e}");
    let two = format!("{}plane 9 0 0 -1 0 0\n", f.wall_a);
    let e = check_wall_embeds_car(&f.car_a, &two, sp).unwrap_err();
    assert!(e.contains("exactly one `plane` line"), "{e}");
    let changed = f.wall_a.replacen(
        "node 0 -2.261 0.55 -0.844 mass 18.25",
        "node 0 -2.261 0.55 -0.844 mass 18.5",
        1,
    );
    assert_ne!(changed, f.wall_a);
    let e = check_wall_embeds_car(&f.car_a, &changed, sp).unwrap_err();
    assert!(e.contains("byte for byte"), "{e}");
    let e = check_wall_embeds_car(&f.car_a, &f.wall_a, sp * 1.01).unwrap_err();
    assert!(e.contains("start velocity"), "{e}");
    let e = check_wall_embeds_car(&f.car_b, &f.wall_a, sp).unwrap_err();
    assert!(e.contains("byte for byte"), "{e}");

    // The crush physics: a stiffness 1% off, a beam 20% longer, a yield strain outside the range.
    let mut b = body_of(&f.car_b);
    b.materials
        .iter_mut()
        .find(|m| m.name == "crush")
        .unwrap()
        .stiffness *= 1.01;
    let off = crush_of(&b, f.steel_e).unwrap();
    let e = check_share(&good_a, &off).unwrap_err();
    assert!(e.contains("P1 area_m2"), "{e}");
    let mut b = body_of(&f.car_b);
    b.materials
        .iter_mut()
        .find(|m| m.name == "crush")
        .unwrap()
        .damping *= 1.01;
    let e = check_share(&good_a, &crush_of(&b, f.steel_e).unwrap()).unwrap_err();
    assert!(e.contains("P3 damping_ratio"), "{e}");
    let mut b = body_of(&f.car_a);
    b.materials
        .iter_mut()
        .find(|m| m.name == "crush")
        .unwrap()
        .yield_strain = Some(0.02);
    let e = check_inside_ranges(&crush_of(&b, f.steel_e).unwrap(), &f.doc).unwrap_err();
    assert!(e.contains("crush.yield_strain"), "{e}");
    let mut b = body_of(&f.car_a);
    b.beams
        .iter_mut()
        .find(|x| x.material == "crush")
        .unwrap()
        .rest *= 1.2;
    let e = check_one_length(&crush_of(&b, f.steel_e).unwrap()).unwrap_err();
    assert!(e.contains("spread"), "{e}");
    // The wall load: the layout before t-20261121-204929 (the four roof-level beams over the
    // wheel mounts `crush` again, the rest of the file unchanged) loads the wall above the
    // centre of gravity and pitches the car onto the ground.
    let mut b = body_of(&f.wall_a);
    let mut changed = 0;
    for beam in b.beams.iter_mut() {
        if [(27, 35), (29, 37), (35, 43), (37, 45)].contains(&(beam.a, beam.b)) {
            assert_eq!(beam.material, "chassis");
            beam.material = "crush".into();
            changed += 1;
        }
    }
    assert_eq!(changed, 4);
    let w = wall_load_of(&b, &f.doc);
    print_wall_load("16 crush beams", &w);
    let e = check_wall_load(&w).unwrap_err();
    assert!(e.contains("not below the centre of gravity"), "{e}");
    // A doc without the range lines is refused by name (no vacuous pass).
    let r = std::panic::catch_unwind(|| frozen("nothing here\n", "range crush.area_m2"));
    assert!(r.is_err(), "a missing frozen line was accepted");
}
