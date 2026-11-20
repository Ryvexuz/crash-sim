//! Stage 6 milestone 2: collider planes (the ground and walls), the scenario files of
//! docs/STAGE6.md ("Milestone 2" and check 2.9), and the crumple report.

use crash_sim::collide::{self, CrumpleParams, Plane};
use crash_sim::format::{parse, write};
use crash_sim::settle::SettleParams;
use crash_sim::solver::{Solver, State};
use crash_sim::vec3::Vec3;
use crash_sim::*;

fn scenario(name: &str) -> String {
    let path = format!("{}/scenarios/{name}.bsn", env!("CARGO_MANIFEST_DIR"));
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"))
}

fn body(name: &str) -> Body {
    parse(&scenario(name)).unwrap_or_else(|e| panic!("{name}.bsn: {e}"))
}

// ---------------------------------------------------------------------------------------
// The ground is a plane, and the cube does not change

/// A copy of `SymplecticEuler` as it was before contact moved out of the solver (a978cb8):
/// the same forces and update, and the y = 0 ground rule written out inline.
struct OldGroundSolver;

impl Solver for OldGroundSolver {
    fn step(&mut self, s: State<'_>, h: f64) {
        let State { pos, vel, beams, mass, gravity } = s;
        let mut force: Vec<Vec3> = mass.iter().map(|&m| Vec3::new(0.0, -gravity * m, 0.0)).collect();
        for bm in beams {
            let d = pos[bm.b] - pos[bm.a];
            let len = d.length();
            if len < 1e-12 {
                continue;
            }
            let dir = d * (1.0 / len);
            let rel_v = (vel[bm.b] - vel[bm.a]).dot(dir);
            let mag = bm.stiffness * (len - bm.rest) + bm.damping * rel_v;
            let f = dir * mag;
            force[bm.a] += f;
            force[bm.b] += -f;
        }
        for i in 0..pos.len() {
            vel[i] += force[i] * (h / mass[i]);
            pos[i] += vel[i] * h;
            if pos[i].y < 0.0 {
                pos[i].y = 0.0;
                if vel[i].y < 0.0 {
                    let vy = vel[i].y;
                    vel[i].y = if -vy > REST_SPEED { -vy * RESTITUTION } else { 0.0 };
                }
                let keep = (1.0 - FRICTION_RATE * h).max(0.0);
                vel[i].x *= keep;
                vel[i].z *= keep;
            }
        }
    }
}

#[test]
fn ground_plane_collider_keeps_cube_output() {
    // The new path: SymplecticEuler plus collide::apply with the ground plane. The reference:
    // the old solver with its own inline ground contact and no planes. Same numbers, every
    // node, every output step of the whole default run (equal as numbers; a zero may differ in sign).
    let cfg = Config::default();
    let mut new = Sim::new(cfg.clone());
    assert_eq!(new.planes, vec![Plane::ground()], "a cube has exactly the ground plane");
    let mut old = Sim::with_solver(cfg.clone(), Box::new(OldGroundSolver));
    old.planes.clear();
    let steps = (cfg.duration / cfg.dt).round() as u64;
    for step in 0..steps {
        new.advance();
        old.advance();
        for i in 0..new.pos.len() {
            assert!(new.pos[i] == old.pos[i], "pos {i} differs at output step {step}: {:?} vs {:?}", new.pos[i], old.pos[i]);
            assert!(new.vel[i] == old.vel[i], "vel {i} differs at output step {step}");
        }
    }
    assert!(new.first_contact_s.is_some(), "the default cube must reach the ground");
    // (`old` has no planes, so it never records a contact time; its numbers above are the reference.)
    assert!(new.sample().center_y == old.sample().center_y);
}

// ---------------------------------------------------------------------------------------
// The plane contact rule

fn wall_x() -> Plane {
    // x = 2, free space at smaller x.
    Plane::new(Vec3::new(2.0, 0.0, 0.0), Vec3::new(-1.0, 0.0, 0.0)).unwrap()
}

#[test]
fn wall_reverses_scales_and_drags() {
    let h = 0.001;
    let keep = (1.0 - FRICTION_RATE * h).max(0.0);
    // Past the wall (x = 2.01), moving in at 4 m/s with 1 m/s sideways: pushed back to x = 2,
    // vx = -4 * RESTITUTION, sideways speed times keep.
    let mut pos = vec![Vec3::new(2.01, 1.0, 1.0)];
    let mut vel = vec![Vec3::new(4.0, 1.0, -2.0)];
    collide::apply(&mut pos, &mut vel, &[wall_x()], h);
    assert!((pos[0].x - 2.0).abs() < 1e-12, "{:?}", pos[0]);
    assert_eq!((pos[0].y, pos[0].z), (1.0, 1.0));
    assert!((vel[0].x - (-4.0 * RESTITUTION)).abs() < 1e-12, "{:?}", vel[0]);
    assert!((vel[0].y - keep).abs() < 1e-12 && (vel[0].z - (-2.0 * keep)).abs() < 1e-12);
    // Slower than REST_SPEED: no bounce, normal velocity 0.
    let mut pos = vec![Vec3::new(2.0001, 0.5, 0.5)];
    let mut vel = vec![Vec3::new(REST_SPEED * 0.5, 0.0, 0.0)];
    collide::apply(&mut pos, &mut vel, &[wall_x()], h);
    assert!(vel[0].x == 0.0, "{:?}", vel[0]);
    // Moving out of the wall keeps its normal velocity (only the drag acts).
    let mut pos = vec![Vec3::new(2.01, 0.5, 0.5)];
    let mut vel = vec![Vec3::new(-3.0, 0.0, 0.0)];
    collide::apply(&mut pos, &mut vel, &[wall_x()], h);
    assert!((vel[0].x - (-3.0)).abs() < 1e-12, "{:?}", vel[0]);
    // A node in free space is not touched at all.
    let mut pos = vec![Vec3::new(1.99, 0.5, 0.5)];
    let mut vel = vec![Vec3::new(5.0, 6.0, 7.0)];
    collide::apply(&mut pos, &mut vel, &[wall_x()], h);
    assert_eq!(pos[0], Vec3::new(1.99, 0.5, 0.5));
    assert_eq!(vel[0], Vec3::new(5.0, 6.0, 7.0));
}

#[test]
fn plane_normals_are_made_unit_and_bad_ones_refused() {
    let p = Plane::new(Vec3::ZERO, Vec3::new(3.0, 3.0, 0.0)).unwrap();
    let r = std::f64::consts::FRAC_1_SQRT_2;
    assert!((p.normal.x - r).abs() < 1e-15 && (p.normal.y - r).abs() < 1e-15 && p.normal.z == 0.0);
    // A huge normal does not overflow its length.
    let big = Plane::new(Vec3::ZERO, Vec3::new(0.0, 1e200, 0.0)).unwrap();
    assert_eq!(big.normal, Vec3::new(0.0, 1.0, 0.0));
    assert!(Plane::new(Vec3::ZERO, Vec3::ZERO).is_err());
    assert!(Plane::new(Vec3::ZERO, Vec3::new(f64::NAN, 1.0, 0.0)).is_err());
    assert!(Plane::new(Vec3::ZERO, Vec3::new(1.0, f64::INFINITY, 0.0)).is_err());
    assert!(Plane::new(Vec3::new(f64::NAN, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).is_err());
    // Behind an oblique plane: distance is signed along the unit normal.
    assert!((p.distance(Vec3::new(-1.0, -1.0, 5.0)) + 2.0 * r).abs() < 1e-12);
}

#[test]
fn impact_is_a_node_on_a_plane_moving_away() {
    let h = 0.001;
    // An oblique plane: after contact the node is on the plane up to rounding, moving away.
    let tilted = Plane::new(Vec3::new(0.3, 0.7, 0.1), Vec3::new(-2.0, 1.0, 0.5)).unwrap();
    let mut pos = vec![Vec3::new(0.0, 0.0, 0.0)];
    let mut vel = vec![-tilted.normal * 3.0];
    pos[0] = tilted.point + tilted.normal * (-0.01);
    collide::apply(&mut pos, &mut vel, &[tilted], h);
    assert!(collide::impact_this_substep(&pos, &vel, &[tilted]), "a bounce is an impact");
    // The ground: exactly on it and moving up.
    let g = [Plane::ground()];
    assert!(collide::impact_this_substep(&[Vec3::new(1.0, 0.0, 2.0)], &[Vec3::new(0.0, 0.5, 0.0)], &g));
    // Resting on it (normal velocity 0), or above it: no impact.
    assert!(!collide::impact_this_substep(&[Vec3::new(1.0, 0.0, 2.0)], &[Vec3::new(0.0, 0.0, 0.0)], &g));
    assert!(!collide::impact_this_substep(&[Vec3::new(1.0, 0.1, 2.0)], &[Vec3::new(0.0, 0.5, 0.0)], &g));
}

// ---------------------------------------------------------------------------------------
// The scenario files

const WALL_FILES: [&str; 5] = ["box_wall", "box_wall_elastic", "box_wall_soft", "box_wall_stiff", "box_wall_slow"];

#[test]
fn scenario_files_are_canonical() {
    for name in WALL_FILES.iter().chain(["break_pull", "break_pull_slow"].iter()) {
        let text = scenario(name);
        let b = parse(&text).unwrap_or_else(|e| panic!("{name}.bsn: {e}"));
        assert_eq!(write(&b), text, "{name}.bsn is not in canonical form");
    }
}

/// (line number, line in a, line in b) for every line that differs; the files must have as
/// many lines as each other.
fn differing_lines(a: &str, b: &str) -> Vec<(usize, String, String)> {
    let (la, lb): (Vec<&str>, Vec<&str>) = (a.lines().collect(), b.lines().collect());
    assert_eq!(la.len(), lb.len(), "the two files have different numbers of lines");
    (0..la.len()).filter(|&i| la[i] != lb[i]).map(|i| (i + 1, la[i].to_string(), lb[i].to_string())).collect()
}

/// Positions (0-based) of the space-separated tokens that differ between two lines with the
/// same number of tokens.
fn differing_tokens(a: &str, b: &str) -> Vec<usize> {
    let (ta, tb): (Vec<&str>, Vec<&str>) = (a.split(' ').collect(), b.split(' ').collect());
    assert_eq!(ta.len(), tb.len(), "lines `{a}` and `{b}` have different numbers of tokens");
    (0..ta.len()).filter(|&i| ta[i] != tb[i]).collect()
}

#[test]
fn box_wall_variants_differ_in_one_number() {
    let nominal = scenario("box_wall");
    // The nominal file is the nb-33 box with yield strain, an `all` group, a velocity and a wall.
    let b = body("box_wall");
    assert_eq!((b.nodes.len(), b.beams.len(), b.planes.len()), (12, 50, 1));
    assert!(b.materials[0].yield_strain.is_some() && b.materials[0].break_strain.is_none());
    let k0 = b.materials[0].stiffness;

    // elastic: the one material line loses its yield_strain, nothing else.
    let d = differing_lines(&nominal, &scenario("box_wall_elastic"));
    assert_eq!(d.len(), 1, "{d:?}");
    assert!(d[0].1.starts_with("material ") && d[0].1.contains(" yield_strain "), "{d:?}");
    assert_eq!(d[0].1.replace(" yield_strain 0.02", ""), d[0].2, "{d:?}");
    // soft and stiff: one number, the stiffness, times 0.5 and 2.
    for (name, factor) in [("box_wall_soft", 0.5), ("box_wall_stiff", 2.0)] {
        let d = differing_lines(&nominal, &scenario(name));
        assert_eq!(d.len(), 1, "{name}: {d:?}");
        assert!(d[0].1.starts_with("material "), "{name}: {d:?}");
        assert_eq!(differing_tokens(&d[0].1, &d[0].2), vec![3], "{name}: only the stiffness value differs");
        let k = body(name).materials[0].stiffness;
        assert!((k - factor * k0).abs() < 1e-9, "{name}: stiffness {k} is not {factor} x {k0}");
    }
    // slow: one number, the speed, at half the kinetic energy.
    let d = differing_lines(&nominal, &scenario("box_wall_slow"));
    assert_eq!(d.len(), 1, "{d:?}");
    assert!(d[0].1.starts_with("velocity all "), "{d:?}");
    assert_eq!(differing_tokens(&d[0].1, &d[0].2), vec![2], "only the x speed differs");
    let (v, vs) = (b.velocities[0].velocity.x, body("box_wall_slow").velocities[0].velocity.x);
    assert_eq!(v, 10.0);
    assert!(((vs / v).powi(2) - 0.5).abs() < 1e-3, "kinetic energy ratio {}", (vs / v).powi(2));
}

#[test]
fn break_pull_files_differ_only_in_speed() {
    let (fast, slow) = (scenario("break_pull"), scenario("break_pull_slow"));
    let d = differing_lines(&fast, &slow);
    assert_eq!(d.len(), 2, "{d:?}");
    assert!(d[0].1.starts_with("velocity left ") && d[1].1.starts_with("velocity right "), "{d:?}");
    for (_, a, b) in &d {
        assert_eq!(differing_tokens(a, b), vec![2], "only the x speed differs in `{a}`");
    }
    let (f, s) = (body("break_pull"), body("break_pull_slow"));
    let speeds = |b: &Body| (b.velocities[0].velocity, b.velocities[1].velocity);
    assert_eq!(speeds(&f), (Vec3::new(-2.5, 0.0, 0.0), Vec3::new(2.5, 0.0, 0.0)));
    assert_eq!(speeds(&s), (Vec3::new(-0.75, 0.0, 0.0), Vec3::new(0.75, 0.0, 0.0)));
    // The closed form of docs/STAGE6.md 2.9: k = 20000, c = 20, two 1 kg nodes, break strain 0.010.
    let m = &f.materials[0];
    assert_eq!((m.stiffness, m.damping, m.break_strain, m.yield_strain), (20000.0, 20.0, Some(0.010), None));
    assert_eq!((f.nodes.len(), f.beams.len(), f.beams[0].rest), (2, 1, 1.0));
}

#[test]
fn box_wall_sim_has_the_ground_then_the_wall_and_the_start_velocity() {
    let b = body("box_wall");
    let sim = Sim::from_body(Config::default(), &b).expect("the box at 1 ms is a valid step");
    assert_eq!(sim.planes.len(), 2);
    assert_eq!(sim.planes[0], Plane::ground());
    assert_eq!(sim.planes[1].normal, Vec3::new(-1.0, 0.0, 0.0));
    assert!((sim.planes[1].distance(Vec3::new(2.0, 0.5, 0.5)) - 0.10).abs() < 1e-12, "the wall is 0.10 m beyond the front face");
    assert!(sim.vel.iter().all(|v| *v == Vec3::new(10.0, 0.0, 0.0)));
    assert_eq!(sim.pos.iter().map(|p| p.y).fold(f64::INFINITY, f64::min), 0.0, "the box rests on the ground");
}

// ---------------------------------------------------------------------------------------
// The crumple report (the numbers are those of docs/STAGE6.md check 2.3)

fn settle_params() -> SettleParams {
    SettleParams { by: 6.0, hold: 4.0, speed: 0.001, energy_rise: 0.001, energy_gain: 0.01 }
}

fn crumple_params() -> CrumpleParams {
    CrumpleParams { from: "back".into(), to: "front".into(), min: 0.02, max: 1.0, drift: 0.001 }
}

fn wall_cfg() -> Config {
    Config { duration: 10.0, ..Config::default() }
}

#[test]
fn crumple_report_refuses_missing_inputs() {
    let sp = settle_params();
    // No body.
    let e = collide::measure_crumple(&wall_cfg(), None, &sp, &crumple_params()).unwrap_err();
    assert!(e.contains("--body"), "{e}");
    // A group the body does not have.
    let bad = CrumpleParams { from: "back".into(), to: "nose".into(), ..crumple_params() };
    let e = collide::measure_crumple(&wall_cfg(), Some(&body("box_wall")), &sp, &bad).unwrap_err();
    assert!(e.contains("nose"), "{e}");
    // Thresholds that contradict each other.
    let bad = CrumpleParams { min: 2.0, max: 1.0, ..crumple_params() };
    assert!(collide::measure_crumple(&wall_cfg(), Some(&body("box_wall")), &sp, &bad).is_err());
}

#[test]
fn elastic_box_wall_has_no_lasting_crumple() {
    // Check 2.4: without plasticity the same metric fails, and the run is not an abort.
    let (settle, crumple) = collide::measure_crumple(&wall_cfg(), Some(&body("box_wall_elastic")), &settle_params(), &crumple_params()).unwrap();
    assert!(!settle.aborted && settle.is_ok(), "{:?}", settle.verdict);
    let c = crumple.expect("a crumple report");
    assert!((c.start_m - 2.0).abs() < 1e-9, "start length {}", c.start_m);
    assert!(c.verdict.is_err(), "an elastic box must not read as crumpled: {c:?}");
    assert!(c.delta_m.abs() <= 0.002, "elastic residue {} m", c.delta_m);
    assert!(c.render().lines().last().unwrap().starts_with("crumple: FAIL"));
}

#[test]
fn box_wall_crumples_and_stays_crumpled() {
    // Check 2.3. Needs plastic beams (task nb-34 / -32): without them this fails like the elastic box.
    let (settle, crumple) = collide::measure_crumple(&wall_cfg(), Some(&body("box_wall")), &settle_params(), &crumple_params()).unwrap();
    assert!(settle.is_ok(), "{:?}", settle.verdict);
    let c = crumple.expect("a crumple report");
    assert!(c.is_ok(), "{}", c.render());
    assert!((c.start_m - 2.0).abs() < 1e-9);
}

#[test]
fn crumple_moves_the_way_physics_says() {
    // Check 2.5: softer beams crumple further, stiffer less, slower impact less.
    let delta = |name: &str| {
        let (settle, crumple) = collide::measure_crumple(&wall_cfg(), Some(&body(name)), &settle_params(), &crumple_params()).unwrap();
        assert!(settle.is_ok(), "{name}: {:?}", settle.verdict);
        crumple.unwrap().delta_m
    };
    let (soft, nominal, stiff, slow) = (delta("box_wall_soft"), delta("box_wall"), delta("box_wall_stiff"), delta("box_wall_slow"));
    assert!(soft > nominal && nominal > stiff, "soft {soft}, nominal {nominal}, stiff {stiff}");
    assert!(slow < nominal, "slow {slow}, nominal {nominal}");
}
