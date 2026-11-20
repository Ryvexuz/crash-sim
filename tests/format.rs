//! The `.bsn` format (versions 1 and 2): round trips, the scenario files, and every refusal by name.
//! Each refusal test checks the line, the column where the token is at fault, and the words
//! of the message, so a parser that refuses for the wrong reason or at the wrong place fails.

use crash_sim::format::{parse, write, Body, BodyBeam, Controls, Group, Material, Node, ParseError, Plane, Suspension, Tyre, Velocity, Wheel};
use crash_sim::vec3::Vec3;
use crash_sim::{Config, Sim, BEAM_DAMPING, BEAM_STIFFNESS, NODE_MASS};

/// A scenario file, exactly as checked out (`.gitattributes` forces LF for `*.bsn`).
fn scenario(name: &str) -> String {
    let path = format!("{}/scenarios/{name}", env!("CARGO_MANIFEST_DIR"));
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"))
}

/// The files must be LF on disk: `.gitattributes` has to keep Windows checkouts from
/// turning them into CRLF, which the strict parser refuses.
#[test]
fn scenario_files_have_no_carriage_returns() {
    let dir = format!("{}/scenarios", env!("CARGO_MANIFEST_DIR"));
    let mut seen = 0;
    for e in std::fs::read_dir(&dir).unwrap() {
        let p = e.unwrap().path();
        if p.extension().map_or(false, |x| x == "bsn") {
            let bytes = std::fs::read(&p).unwrap();
            assert!(!bytes.contains(&b'\r'), "{} contains a carriage return", p.display());
            seen += 1;
        }
    }
    assert!(seen >= 2, "found {seen} .bsn files");
}

// ---------------------------------------------------------------------------------------
// Scenario files

#[test]
fn cube_bsn_matches_default_cube() {
    let built = parse(&scenario("cube.bsn")).expect("cube.bsn parses").build();
    let sim = Sim::new(Config::default());
    assert_eq!(built.pos.len(), sim.pos.len());
    for (i, (a, b)) in built.pos.iter().zip(&sim.pos).enumerate() {
        assert_eq!(a, b, "node {i} position (bit for bit)");
    }
    assert_eq!(built.mass, sim.mass, "masses");
    assert_eq!(built.mass, vec![NODE_MASS; 8]);
    assert!(built.vel.iter().all(|v| *v == Vec3::ZERO));
    assert_eq!(built.beams.len(), sim.beams.len());
    for (i, (a, b)) in built.beams.iter().zip(&sim.beams).enumerate() {
        assert_eq!((a.a, a.b), (b.a, b.b), "beam {i} nodes");
        assert_eq!(a.rest, b.rest, "beam {i} rest length (bit for bit)");
        assert_eq!(a.stiffness, b.stiffness, "beam {i} stiffness");
        assert_eq!(a.damping, b.damping, "beam {i} damping");
    }
    assert_eq!(built.beams[0].stiffness, BEAM_STIFFNESS);
    assert_eq!(built.beams[0].damping, BEAM_DAMPING);
}

#[test]
fn box_bsn_is_the_two_by_one_by_one_box() {
    let body = parse(&scenario("box.bsn")).expect("box.bsn parses");
    assert_eq!(body.nodes.len(), 12, "3 x 2 x 2 nodes");
    assert!(body.nodes.iter().all(|n| n.mass == 1.0), "every node is 1 kg");
    let xs: Vec<f64> = body.nodes.iter().map(|n| n.pos.x).collect();
    let (xmin, xmax) = (xs.iter().cloned().fold(f64::INFINITY, f64::min), xs.iter().cloned().fold(f64::NEG_INFINITY, f64::max));
    assert_eq!(xmax - xmin, 2.0, "long axis x is 2 m");
    for axis in [|n: &Node| n.pos.y, |n: &Node| n.pos.z] {
        let v: Vec<f64> = body.nodes.iter().map(axis).collect();
        let (lo, hi) = (v.iter().cloned().fold(f64::INFINITY, f64::min), v.iter().cloned().fold(f64::NEG_INFINITY, f64::max));
        assert_eq!(hi - lo, 1.0, "the other two axes are 1 m");
    }
    assert_eq!(body.nodes.iter().map(|n| n.pos.y).fold(f64::INFINITY, f64::min), 0.0, "resting on y = 0");
    let mut distinct: Vec<(u64, u64, u64)> = body.nodes.iter().map(|n| (n.pos.x.to_bits(), n.pos.y.to_bits(), n.pos.z.to_bits())).collect();
    distinct.sort();
    distinct.dedup();
    assert_eq!(distinct.len(), 12, "12 different positions");
    // groups: back = the 4 nodes at the smallest x, front = the 4 at the largest x
    let ids_where = |f: &dyn Fn(&Node) -> bool| -> Vec<u32> {
        let mut v: Vec<u32> = body.nodes.iter().filter(|n| f(n)).map(|n| n.id).collect();
        v.sort();
        v
    };
    let group = |name: &str| -> Vec<u32> {
        let mut v = body.groups.iter().find(|g| g.name == name).unwrap_or_else(|| panic!("group {name}")).nodes.clone();
        v.sort();
        v
    };
    assert_eq!(body.groups.len(), 2);
    assert_eq!(group("back"), ids_where(&|n| n.pos.x == xmin));
    assert_eq!(group("front"), ids_where(&|n| n.pos.x == xmax));
    assert_eq!(group("back").len(), 4);
    assert_eq!(group("front").len(), 4);
    // beams: every pair of nodes within one cell (edge, face or body diagonal), once each
    assert_eq!(body.beams.len(), 50);
    for b in &body.beams {
        let (pa, pb) = (body.nodes[body.node_index(b.a).unwrap()].pos, body.nodes[body.node_index(b.b).unwrap()].pos);
        let d = pb - pa;
        assert!(d.x.abs() <= 1.0 && d.y.abs() <= 1.0 && d.z.abs() <= 1.0, "beam {}-{} spans more than one cell", b.a, b.b);
        assert_eq!(b.rest, d.length());
    }
}

#[test]
fn canonical_file_round_trips_byte_identical() {
    for name in ["cube.bsn", "box.bsn"] {
        let text = scenario(name);
        let body = parse(&text).unwrap_or_else(|e| panic!("{name}: {e}"));
        let again = write(&body);
        assert_eq!(again, text, "write(parse({name})) differs from the file");
    }
    // a canonical file that uses every line type, a beam with its own rest length, and
    // numbers that are not round
    let text = "bsn 1\n\
material soft stiffness 1500.5 damping 0 yield_strain 0.01 break_strain 0.2\n\
material hard stiffness 20000 damping 20 break_strain 0.05\n\
node 0 0.1 0.30000000000000004 -0.0000001 mass 1.5\n\
node 7 1000000000000000000000 2 3 mass 0.001\n\
node 3 0 0 0.25 mass 2\n\
beam 0 7 soft\n\
beam 3 0 hard rest 0.3\n\
group a 0 3\n\
group b 7\n\
velocity a 1 -2.5 0\n\
velocity b 0 0 0.5\n\
plane 0 0 0 0 1 0\n";
    let body = parse(text).expect("parses");
    assert_eq!(write(&body), text);
}

// ---------------------------------------------------------------------------------------
// Round trip of bodies

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        self.0 >> 11
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
    fn float(&mut self) -> f64 {
        let scale = [1e-9, 1e-3, 1.0, 1e3, 1e9][self.below(5) as usize];
        ((self.next() as f64) / (1u64 << 53) as f64 - 0.5) * scale
    }
    fn positive(&mut self) -> f64 {
        self.float().abs() + 1e-12
    }
}

fn random_body(rng: &mut Rng) -> Body {
    let n_mat = 1 + rng.below(3) as usize;
    let materials: Vec<Material> = (0..n_mat)
        .map(|i| {
            let y = if rng.below(2) == 0 { Some(rng.positive()) } else { None };
            let b = if rng.below(2) == 0 { Some(y.unwrap_or(0.0) * 2.0 + rng.positive()) } else { None };
            Material { name: format!("m{i}"), stiffness: rng.positive(), damping: if rng.below(3) == 0 { 0.0 } else { rng.positive() }, yield_strain: y, break_strain: b }
        })
        .collect();
    let n_nodes = 1 + rng.below(9) as u32;
    let mut id = rng.below(5) as u32;
    let mut nodes = Vec::new();
    for _ in 0..n_nodes {
        nodes.push(Node { id, pos: Vec3::new(rng.float(), rng.float(), rng.float()), mass: rng.positive() });
        id += 1 + rng.below(1000) as u32;
    }
    let mut beams: Vec<BodyBeam> = Vec::new();
    for _ in 0..rng.below(12) {
        let (i, j) = (rng.below(n_nodes as u64) as usize, rng.below(n_nodes as u64) as usize);
        let (a, b) = (nodes[i].id, nodes[j].id);
        let dist = (nodes[j].pos - nodes[i].pos).length();
        if i == j || dist == 0.0 || beams.iter().any(|x| (x.a, x.b) == (a, b) || (x.a, x.b) == (b, a)) {
            continue;
        }
        let rest = if rng.below(3) == 0 { rng.positive() } else { dist };
        beams.push(BodyBeam { a, b, material: format!("m{}", rng.below(n_mat as u64)), rest });
    }
    let mut groups = Vec::new();
    let mut velocities = Vec::new();
    let mut velocity_taken: Vec<u32> = Vec::new();
    for g in 0..rng.below(4) {
        let members: Vec<u32> = nodes.iter().filter(|_| rng.below(2) == 0).map(|n| n.id).collect();
        if members.is_empty() {
            continue;
        }
        let name = format!("g{g}");
        if rng.below(2) == 0 && members.iter().all(|m| !velocity_taken.contains(m)) {
            velocity_taken.extend(&members);
            velocities.push(Velocity { group: name.clone(), velocity: Vec3::new(rng.float(), rng.float(), rng.float()) });
        }
        groups.push(Group { name, nodes: members });
    }
    let planes = (0..rng.below(3)).map(|_| Plane { point: Vec3::new(rng.float(), rng.float(), rng.float()), normal: Vec3::new(1.0, rng.float(), rng.float()) }).collect();
    let (mut wheels, mut suspensions, mut tyres, mut controls) = (Vec::new(), Vec::new(), Vec::new(), None);
    if rng.below(2) == 0 {
        for w in 0..1 + rng.below(4) {
            let mut mount: Vec<u32> = nodes.iter().filter(|_| rng.below(3) == 0).map(|n| n.id).collect();
            if mount.is_empty() {
                mount.push(nodes[rng.below(n_nodes as u64) as usize].id);
            }
            // wheel 0 has every flag, so the controls always have a wheel to act on
            let flag = |rng: &mut Rng| w == 0 || rng.below(2) == 0;
            wheels.push(Wheel {
                name: format!("w{w}"),
                mount,
                radius: rng.positive(),
                width: rng.positive(),
                mass: rng.positive(),
                inertia: rng.positive(),
                steerable: flag(rng),
                driven: flag(rng),
                braked: flag(rng),
            });
            // travel_min < rest < travel_max, strictly, even when a sum rounds to a summand
            let travel_min = if rng.below(4) == 0 { 0.0 } else { rng.positive() };
            let mut rest = travel_min + rng.positive();
            if rest <= travel_min {
                rest = travel_min * 2.0 + 1.0;
            }
            let mut travel_max = rest + rng.positive();
            if travel_max <= rest {
                travel_max = rest * 2.0 + 1.0;
            }
            suspensions.push(Suspension {
                wheel: format!("w{w}"),
                spring: rng.positive(),
                damper: if rng.below(3) == 0 { 0.0 } else { rng.positive() },
                rest,
                travel_min,
                travel_max,
            });
            tyres.push(Tyre {
                wheel: format!("w{w}"),
                mu: rng.positive(),
                radial_stiffness: rng.positive(),
                radial_damping: if rng.below(3) == 0 { 0.0 } else { rng.positive() },
                long_stiffness: rng.positive(),
                lat_stiffness: rng.positive(),
            });
        }
        // suspension and tyre lines need not follow the wheel order
        if rng.below(2) == 0 {
            suspensions.reverse();
        }
        controls = Some(Controls { max_steer_deg: 1e-6 + (rng.below(89_000) as f64) / 1000.0, max_drive_torque: rng.positive(), max_brake_torque: rng.positive() });
    }
    Body { materials, nodes, beams, groups, velocities, wheels, suspensions, tyres, controls, planes }
}

#[test]
fn parse_of_write_gives_the_same_body() {
    let mut rng = Rng(12345);
    let (mut with_rest, mut with_car) = (0, 0);
    for case in 0..500 {
        let body = random_body(&mut rng);
        let text = write(&body);
        let back = parse(&text).unwrap_or_else(|e| panic!("case {case}: {e}\n{text}"));
        assert_eq!(back, body, "case {case}\n{text}");
        assert_eq!(write(&back), text, "case {case}: write is not stable");
        with_rest += text.matches(" rest ").count() - text.matches("
suspension ").count();
        with_car += usize::from(text.starts_with("bsn 2
"));
    }
    assert!(with_rest > 50, "the generator should exercise explicit rest lengths ({with_rest})");
    assert!(with_car > 150, "the generator should exercise the car sections ({with_car})");
}

#[test]
fn explicit_rest_equal_to_the_distance_is_written_without_it() {
    let text = "bsn 1\nmaterial m stiffness 1 damping 0\nnode 0 0 0 0 mass 1\nnode 1 3 4 0 mass 1\nbeam 0 1 m rest 5\n";
    let body = parse(text).unwrap();
    assert_eq!(body.beams[0].rest, 5.0);
    assert_eq!(write(&body), "bsn 1\nmaterial m stiffness 1 damping 0\nnode 0 0 0 0 mass 1\nnode 1 3 4 0 mass 1\nbeam 0 1 m\n");
}

#[test]
fn build_applies_materials_group_velocities_and_rest() {
    let text = "bsn 1\nmaterial a stiffness 10 damping 1\nmaterial b stiffness 30 damping 3\n\
node 5 0 0 0 mass 2\nnode 9 3 4 0 mass 4\nnode 2 0 4 0 mass 1\n\
beam 5 9 a\nbeam 9 2 b rest 2.5\ngroup left 5 2\nvelocity left 1 2 3\n";
    let b = parse(text).unwrap().build();
    assert_eq!(b.mass, vec![2.0, 4.0, 1.0]);
    assert_eq!(b.vel, vec![Vec3::new(1.0, 2.0, 3.0), Vec3::ZERO, Vec3::new(1.0, 2.0, 3.0)]);
    assert_eq!((b.beams[0].a, b.beams[0].b, b.beams[0].rest, b.beams[0].stiffness, b.beams[0].damping), (0, 1, 5.0, 10.0, 1.0));
    assert_eq!((b.beams[1].a, b.beams[1].b, b.beams[1].rest, b.beams[1].stiffness, b.beams[1].damping), (1, 2, 2.5, 30.0, 3.0));
}

#[test]
fn comments_and_blank_lines_are_ignored_and_line_numbers_still_count_them() {
    let ok = "bsn 1\n# a comment\n\nmaterial m stiffness 1 damping 0\nnode 0 0 0 0 mass 1\n";
    assert_eq!(parse(ok).unwrap().nodes.len(), 1);
    let bad = "bsn 1\n# a comment\n\nmaterial m stiffness 1 damping 0\n# more\nnode 0 0 0 0 mass 0\n";
    assert_eq!(refuse(bad).line, 6);
}

// ---------------------------------------------------------------------------------------
// Refusals

fn refuse(text: &str) -> ParseError {
    match parse(text) {
        Ok(_) => panic!("accepted, but must be refused:\n{text}"),
        Err(e) => e,
    }
}

/// Assert a refusal at `line` and `column` whose message contains `words`.
fn expect(text: &str, line: usize, column: Option<usize>, words: &str) {
    let e = refuse(text);
    assert_eq!((e.line, e.column), (line, column), "wrong place for `{words}`: {e}");
    assert!(e.message.contains(words), "message `{}` lacks `{words}`", e.message);
    let shown = e.to_string();
    assert!(shown.starts_with(&format!("line {line}")), "Display must name the line: {shown}");
}

/// A valid two-node body with the given lines after the header. Line 1 is the header,
/// line 2 the material, lines 3 and 4 the nodes.
fn base(extra: &str) -> String {
    format!("bsn 1\nmaterial m stiffness 100 damping 1\nnode 0 0 0 0 mass 1\nnode 1 1 0 0 mass 1\n{extra}")
}

#[test]
fn the_base_body_is_accepted() {
    assert!(parse(&base("beam 0 1 m\n")).is_ok());
}

#[test]
fn refuse_empty_file() {
    expect("", 1, None, "empty file");
}

#[test]
fn refuse_missing_header() {
    expect("material m stiffness 1 damping 0\n", 1, None, "must be `bsn 1`");
    expect("\nbsn 1\n", 1, None, "must be `bsn 1`");
    expect("# comment\nbsn 1\n", 1, None, "must be `bsn 1`");
}

#[test]
fn refuse_unknown_version() {
    expect("bsn 3\nnode 0 0 0 0 mass 1\n", 1, Some(5), "unknown version 3");
    expect("bsn 0\nnode 0 0 0 0 mass 1\n", 1, Some(5), "unknown version 0");
    expect("bsn 99999999999\nnode 0 0 0 0 mass 1\n", 1, Some(5), "unknown version 99999999999");
}

#[test]
fn refuse_malformed_version() {
    expect("bsn one\n", 1, Some(5), "not a version number");
    expect("bsn 1.0\n", 1, Some(5), "not a version number");
    expect("bsn 1 \n", 1, Some(5), "not a version number");
    expect("bsn\n", 1, None, "must be `bsn 1`");
}

#[test]
fn refuse_missing_final_newline() {
    expect("bsn 1\nmaterial m stiffness 1 damping 0\nnode 0 0 0 0 mass 1", 3, None, "newline");
}

#[test]
fn refuse_duplicate_node_id() {
    expect(&base("node 1 5 5 5 mass 1\n"), 5, Some(6), "duplicate node id 1");
}

#[test]
fn refuse_beam_to_missing_node() {
    expect(&base("beam 0 7 m\n"), 5, Some(8), "beam to missing node 7");
    expect(&base("beam 7 0 m\n"), 5, Some(6), "beam to missing node 7");
}

#[test]
fn refuse_beam_before_its_node_is_declared() {
    // nodes must come before beams, so a beam can only name earlier nodes
    expect("bsn 1\nmaterial m stiffness 1 damping 0\nnode 0 0 0 0 mass 1\nbeam 0 1 m\nnode 1 1 0 0 mass 1\n", 4, Some(8), "missing node 1");
}

#[test]
fn refuse_zero_length_beam() {
    let t = "bsn 1\nmaterial m stiffness 1 damping 0\nnode 0 1 2 3 mass 1\nnode 1 1 2 3 mass 1\nbeam 0 1 m\n";
    expect(t, 5, Some(6), "zero-length beam");
    // a given rest length does not excuse coincident nodes
    expect(&t.replace("beam 0 1 m", "beam 0 1 m rest 1"), 5, Some(6), "zero-length beam");
}

#[test]
fn refuse_beam_from_a_node_to_itself() {
    expect(&base("beam 1 1 m\n"), 5, Some(8), "to itself");
}

#[test]
fn refuse_duplicate_beam() {
    expect(&base("beam 0 1 m\nbeam 1 0 m\n"), 6, Some(6), "duplicate beam");
}

#[test]
fn refuse_beam_with_unknown_material() {
    expect(&base("beam 0 1 steel\n"), 5, Some(10), "unknown material `steel`");
}

#[test]
fn refuse_nonpositive_rest() {
    expect(&base("beam 0 1 m rest 0\n"), 5, Some(17), "rest length 0 must be > 0");
    expect(&base("beam 0 1 m rest -1\n"), 5, Some(17), "must be > 0");
}

#[test]
fn refuse_zero_mass() {
    expect("bsn 1\nmaterial m stiffness 1 damping 0\nnode 0 0 0 0 mass 0\n", 3, Some(19), "mass 0 must be > 0");
}

#[test]
fn refuse_negative_mass() {
    expect("bsn 1\nmaterial m stiffness 1 damping 0\nnode 0 0 0 0 mass -2\n", 3, Some(19), "mass -2 must be > 0");
}

#[test]
fn refuse_non_finite_mass() {
    for bad in ["NaN", "nan", "inf", "-inf", "infinity"] {
        let t = format!("bsn 1\nmaterial m stiffness 1 damping 0\nnode 0 0 0 0 mass {bad}\n");
        expect(&t, 3, Some(19), "not finite");
    }
    // so large it overflows to infinity while parsing
    expect("bsn 1\nmaterial m stiffness 1 damping 0\nnode 0 0 0 0 mass 1e999\n", 3, Some(19), "not finite");
}

#[test]
fn refuse_non_finite_position() {
    expect("bsn 1\nmaterial m stiffness 1 damping 0\nnode 0 0 NaN 0 mass 1\n", 3, Some(10), "position y");
    expect("bsn 1\nmaterial m stiffness 1 damping 0\nnode 0 inf 0 0 mass 1\n", 3, Some(8), "position x");
}

#[test]
fn refuse_zero_stiffness() {
    expect("bsn 1\nmaterial m stiffness 0 damping 0\nnode 0 0 0 0 mass 1\n", 2, Some(22), "stiffness 0 must be > 0");
}

#[test]
fn refuse_negative_stiffness() {
    expect("bsn 1\nmaterial m stiffness -5 damping 0\nnode 0 0 0 0 mass 1\n", 2, Some(22), "stiffness -5 must be > 0");
}

#[test]
fn refuse_non_finite_stiffness_or_damping() {
    expect("bsn 1\nmaterial m stiffness inf damping 0\nnode 0 0 0 0 mass 1\n", 2, Some(22), "not finite");
    expect("bsn 1\nmaterial m stiffness 1 damping NaN\nnode 0 0 0 0 mass 1\n", 2, Some(32), "not finite");
}

#[test]
fn refuse_negative_damping() {
    expect("bsn 1\nmaterial m stiffness 1 damping -1\nnode 0 0 0 0 mass 1\n", 2, Some(32), "damping -1 must be >= 0");
}

#[test]
fn refuse_bad_strain_limits() {
    let m = |tail: &str| format!("bsn 1\nmaterial m stiffness 1 damping 0 {tail}\nnode 0 0 0 0 mass 1\n");
    expect(&m("yield_strain 0"), 2, Some(47), "yield_strain 0 must be > 0");
    expect(&m("break_strain -1"), 2, Some(47), "break_strain -1 must be > 0");
    expect(&m("yield_strain 0.2 break_strain 0.1"), 2, Some(64), "must be above yield_strain");
    expect(&m("yield_strain 0.1 break_strain 0.1"), 2, Some(64), "must be above yield_strain");
    // order is fixed: yield_strain before break_strain
    expect(&m("break_strain 0.2 yield_strain 0.1"), 2, Some(51), "unexpected `yield_strain`");
}

#[test]
fn refuse_empty_body() {
    expect("bsn 1\n", 1, None, "empty body");
    expect("bsn 1\nmaterial m stiffness 1 damping 0\n# nothing else\n", 3, None, "empty body");
}

#[test]
fn refuse_unknown_keyword() {
    expect(&base("spring 0 1 m\n"), 5, Some(1), "unknown keyword `spring`");
    expect(&base("Node 2 0 0 0 mass 1\n"), 5, Some(1), "unknown keyword `Node`");
}

#[test]
fn refuse_wrong_keyword_inside_a_line() {
    expect("bsn 1\nmaterial m stiffness 1 damp 0\nnode 0 0 0 0 mass 1\n", 2, Some(24), "expected `damping`");
    expect("bsn 1\nmaterial m stiffness 1 damping 0\nnode 0 0 0 0 weight 1\n", 3, Some(14), "expected `mass`");
}

#[test]
fn refuse_missing_token() {
    expect("bsn 1\nmaterial m stiffness 1 damping 0\nnode 0 0 0 mass 1\n", 3, Some(12), "position z");
    expect("bsn 1\nmaterial m stiffness 1 damping 0\nnode 0 0 0 0 mass\n", 3, Some(18), "missing mass");
    expect(&base("beam 0 1\n"), 5, Some(9), "missing beam material");
    expect(&base("group g\n"), 5, Some(8), "missing group member");
}

#[test]
fn refuse_extra_token() {
    expect("bsn 1\nmaterial m stiffness 1 damping 0\nnode 0 0 0 0 mass 1 2\n", 3, Some(21), "unexpected `2`");
    expect(&base("beam 0 1 m rest 1 2\n"), 5, Some(19), "unexpected `2`");
    expect(&base("plane 0 0 0 0 1 0 0\n"), 5, Some(19), "unexpected `0`");
}

#[test]
fn refuse_not_a_number() {
    expect("bsn 1\nmaterial m stiffness 1 damping 0\nnode 0 1,5 0 0 mass 1\n", 3, Some(8), "`1,5` is not a number");
    expect("bsn 1\nmaterial m stiffness x damping 0\nnode 0 0 0 0 mass 1\n", 2, Some(22), "is not a number");
}

#[test]
fn refuse_bad_node_ids() {
    for bad in ["-1", "+1", "01", "1.0", "x", "4294967296"] {
        let t = format!("bsn 1\nmaterial m stiffness 1 damping 0\nnode {bad} 0 0 0 mass 1\n");
        expect(&t, 3, Some(6), "is not a node id");
    }
    // the largest id is fine
    assert!(parse("bsn 1\nmaterial m stiffness 1 damping 0\nnode 4294967295 0 0 0 mass 1\n").is_ok());
}

#[test]
fn refuse_bad_names() {
    expect("bsn 1\nmaterial 1m stiffness 1 damping 0\nnode 0 0 0 0 mass 1\n", 2, Some(10), "is not a name");
    expect(&base("group a-b 0\n"), 5, Some(7), "is not a name");
}

#[test]
fn refuse_extra_spaces_and_tabs() {
    expect("bsn 1\nmaterial m stiffness  1 damping 0\nnode 0 0 0 0 mass 1\n", 2, Some(22), "extra space");
    expect("bsn 1\n material m stiffness 1 damping 0\nnode 0 0 0 0 mass 1\n", 2, Some(1), "extra space");
    expect("bsn 1\nmaterial m stiffness 1 damping 0 \nnode 0 0 0 0 mass 1\n", 2, Some(34), "extra space");
    expect("bsn 1\nmaterial m stiffness 1\tdamping 0\nnode 0 0 0 0 mass 1\n", 2, Some(23), "tab");
}

#[test]
fn refuse_carriage_returns() {
    expect("bsn 1\r\nmaterial m stiffness 1 damping 0\nnode 0 0 0 0 mass 1\n", 1, Some(5), "not a version number");
    expect("bsn 1\nmaterial m stiffness 1 damping 0\r\nnode 0 0 0 0 mass 1\n", 2, Some(33), "carriage return");
}

#[test]
fn refuse_sections_out_of_order() {
    expect("bsn 1\nnode 0 0 0 0 mass 1\nmaterial m stiffness 1 damping 0\n", 3, Some(1), "after a later section");
    expect(&base("group g 0\nbeam 0 1 m\n"), 6, Some(1), "after a later section");
    expect(&base("velocity g 0 0 0\n"), 5, Some(10), "unknown group");
}

#[test]
fn refuse_duplicate_material() {
    expect("bsn 1\nmaterial m stiffness 1 damping 0\nmaterial m stiffness 2 damping 0\nnode 0 0 0 0 mass 1\n", 3, Some(10), "duplicate material `m`");
}

#[test]
fn refuse_group_naming_a_missing_node() {
    expect(&base("group g 0 9\n"), 5, Some(11), "missing node 9");
}

#[test]
fn refuse_group_listing_a_node_twice() {
    expect(&base("group g 0 1 0\n"), 5, Some(13), "twice");
}

#[test]
fn refuse_duplicate_group() {
    expect(&base("group g 0\ngroup g 1\n"), 6, Some(7), "duplicate group `g`");
}

#[test]
fn refuse_velocity_for_an_unknown_group() {
    expect(&base("group g 0\nvelocity h 1 0 0\n"), 6, Some(10), "unknown group `h`");
}

#[test]
fn refuse_second_velocity_for_a_group() {
    expect(&base("group g 0\nvelocity g 1 0 0\nvelocity g 2 0 0\n"), 7, Some(10), "already has a velocity");
}

#[test]
fn refuse_velocity_groups_that_share_a_node() {
    expect(&base("group g 0 1\ngroup h 1\nvelocity g 1 0 0\nvelocity h 2 0 0\n"), 8, Some(10), "second velocity");
}

#[test]
fn refuse_non_finite_velocity() {
    expect(&base("group g 0\nvelocity g 1 NaN 0\n"), 6, Some(14), "velocity y");
}

#[test]
fn refuse_plane_with_zero_normal() {
    expect(&base("plane 0 0 0 0 0 0\n"), 5, None, "zero vector");
    expect(&base("plane 0 0 0 0 0 inf\n"), 5, Some(17), "plane normal z");
}

#[test]
fn refuse_beam_whose_length_overflows_unless_rest_is_given() {
    let nodes = "bsn 1\nmaterial m stiffness 100 damping 1\nnode 0 -1e308 0 0 mass 1\nnode 1 1e308 0 0 mass 1\n";
    expect(&format!("{nodes}beam 0 1 m\n"), 5, Some(6), "not finite");
    let ok = parse(&format!("{nodes}beam 0 1 m rest 5\n")).expect("explicit rest is accepted");
    assert_eq!(ok.beams[0].rest, 5.0);
}

#[test]
fn refuse_carriage_return_on_comment_and_blank_lines() {
    expect(&base("# comment\r\n"), 5, Some(10), "carriage return");
    expect(&base("\r\n"), 5, Some(1), "carriage return");
}

#[test]
fn tiny_plane_normal_is_not_the_zero_vector_and_round_trips() {
    let b = parse(&base("plane 0 0 0 1e-200 0 0\n")).expect("a tiny nonzero normal is a direction");
    assert_eq!(b.planes[0].normal, Vec3::new(1e-200, 0.0, 0.0));
    assert_eq!(parse(&write(&b)).unwrap(), b);
    expect(&base("plane 0 0 0 0 0 0\n"), 5, None, "zero vector");
}

// ---------------------------------------------------------------------------------------
// Version 2: the car sections (wheel, suspension, tyre, controls)

/// A valid one-wheel car. Line 5 is the wheel, 6 the suspension, 7 the tyre, 8 the controls.
const CAR: &str = "bsn 2\n\
material m stiffness 100 damping 1\n\
node 0 0 1 0 mass 1\n\
node 1 1 1 0 mass 1\n\
wheel w mount 0 1 radius 0.3 width 0.2 mass 10 inertia 0.45 steerable yes driven yes braked yes\n\
suspension w spring 1000 damper 10 rest 0.5 travel 0.3 0.6\n\
tyre w mu 0.9 radial_stiffness 100000 radial_damping 100 long_stiffness 15 lat_stiffness 10\n\
controls max_steer_deg 30 max_drive_torque 100 max_brake_torque 200\n";

/// `CAR` with the first `from` replaced by `to` (which must change it).
fn car(from: &str, to: &str) -> String {
    assert!(CAR.contains(from), "CAR has no `{from}`");
    CAR.replacen(from, to, 1)
}

/// The 1-based column just after the first `prefix` on line `line` of `text`.
fn col_after(text: &str, line: usize, prefix: &str) -> usize {
    let l = text.split('\n').nth(line - 1).unwrap_or_else(|| panic!("no line {line}"));
    l.find(prefix).unwrap_or_else(|| panic!("line {line} `{l}` has no `{prefix}`")) + prefix.len() + 1
}

/// Assert a refusal of `text` at `line`, at the column just after `prefix` on that line.
fn expect_after(text: &str, line: usize, prefix: &str, words: &str) {
    expect(text, line, Some(col_after(text, line, prefix)), words);
}

#[test]
fn the_base_car_is_accepted_and_round_trips() {
    let body = parse(CAR).unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(write(&body), CAR);
    // a collider plane still comes last, after the car
    let with_plane = format!("{CAR}plane 5 0 0 -1 0 0\n");
    assert_eq!(write(&parse(&with_plane).unwrap()), with_plane);
}

#[test]
fn version_1_files_still_load_and_are_written_as_version_1() {
    for name in ["cube.bsn", "box.bsn", "box_wall.bsn", "break_pull.bsn"] {
        let text = scenario(name);
        assert!(text.starts_with("bsn 1\n"), "{name} is a version 1 file");
        assert_eq!(write(&parse(&text).unwrap_or_else(|e| panic!("{name}: {e}"))), text, "{name}");
    }
}

#[test]
fn version_2_file_without_car_sections_loads_like_version_1() {
    let v1 = base("beam 0 1 m\n");
    let v2 = v1.replacen("bsn 1\n", "bsn 2\n", 1);
    assert_eq!(parse(&v2).unwrap(), parse(&v1).unwrap());
    // the canonical form is the lowest version that holds the body
    assert_eq!(write(&parse(&v2).unwrap()), v1);
}

#[test]
fn car_basic_round_trips() {
    let text = scenario("car_basic.bsn");
    let body = parse(&text).unwrap_or_else(|e| panic!("car_basic.bsn: {e}"));
    assert_eq!(write(&body), text, "write(parse(car_basic.bsn)) differs from the file");
    assert_eq!(parse(&write(&body)).unwrap(), body);
}

fn extent(v: impl Iterator<Item = f64>) -> (f64, f64) {
    v.fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), x| (lo.min(x), hi.max(x)))
}

/// The car of docs/STAGE6.md milestone 3 and docs/FORMAT.md "car_basic.bsn": the numbers the
/// later checks rely on.
#[test]
fn car_basic_is_the_stage6_car() {
    let body = parse(&scenario("car_basic.bsn")).unwrap_or_else(|e| panic!("car_basic.bsn: {e}"));
    let near = |a: f64, b: f64, what: &str| assert!((a - b).abs() <= 1e-12, "{what}: {a} != {b}");
    // a few dozen nodes, a 4.0 x 1.4 x 1.8 m chassis: x long (front toward +x), y up, z across
    assert_eq!(body.nodes.len(), 40);
    let (x0, x1) = extent(body.nodes.iter().map(|n| n.pos.x));
    let (y0, y1) = extent(body.nodes.iter().map(|n| n.pos.y));
    let (z0, z1) = extent(body.nodes.iter().map(|n| n.pos.z));
    near(x1 - x0, 4.0, "length");
    near(y1 - y0, 1.4, "height");
    near(z1 - z0, 1.8, "width");
    // about 1200 kg with the wheels
    let total: f64 = body.nodes.iter().map(|n| n.mass).sum::<f64>() + body.wheels.iter().map(|w| w.mass).sum::<f64>();
    near(total, 1200.0, "total mass");
    // groups front and rear: the front-most and rear-most chassis nodes
    let ids = |f: &dyn Fn(&Node) -> bool| -> Vec<u32> { body.nodes.iter().filter(|n| f(n)).map(|n| n.id).collect() };
    let group = |name: &str| body.groups.iter().find(|g| g.name == name).unwrap_or_else(|| panic!("group {name}")).nodes.clone();
    assert_eq!(group("front"), ids(&|n| n.pos.x == x1));
    assert_eq!(group("rear"), ids(&|n| n.pos.x == x0));
    // four wheels: front steer, rear drive, all braked; one grip coefficient
    assert_eq!(body.wheels.len(), 4);
    for w in &body.wheels {
        let c = body.wheel_start_center(&w.name).unwrap();
        let front = c.x > 0.0;
        assert_eq!((w.steerable, w.driven, w.braked), (front, !front, true), "wheel {}", w.name);
        // every tyre's lowest point 0.20 m above the ground, suspension at its rest length
        near(c.y - w.radius, 0.2, &format!("wheel {} tyre bottom", w.name));
        assert_eq!(body.tyre(&w.name).unwrap().mu, body.tyres[0].mu, "wheel {} mu", w.name);
    }
    assert_eq!(body.wheels.iter().filter(|w| body.wheel_start_center(&w.name).unwrap().x > 0.0).count(), 2, "two front wheels");
    assert!(body.controls.is_some());
    // mirror-symmetric left to right (z -> -z): nodes with their masses, beams, wheels
    let mirror_id = |id: u32| -> u32 {
        let n = &body.nodes[body.node_index(id).unwrap()];
        let m: Vec<&Node> = body.nodes.iter().filter(|o| o.pos == Vec3::new(n.pos.x, n.pos.y, -n.pos.z)).collect();
        assert_eq!(m.len(), 1, "node {id} has one mirror node");
        assert_eq!(m[0].mass, n.mass, "node {id} mirror mass");
        m[0].id
    };
    for b in &body.beams {
        let (a, c) = (mirror_id(b.a), mirror_id(b.b));
        let m = body.beams.iter().find(|o| (o.a, o.b) == (a, c) || (o.a, o.b) == (c, a));
        let m = m.unwrap_or_else(|| panic!("beam {}-{} has no mirror", b.a, b.b));
        assert_eq!((&m.material, m.rest), (&b.material, b.rest), "beam {}-{} mirror", b.a, b.b);
    }
    for w in &body.wheels {
        let mount: Vec<u32> = w.mount.iter().map(|id| mirror_id(*id)).collect();
        let m = body.wheels.iter().find(|o| o.mount == mount).unwrap_or_else(|| panic!("wheel {} has no mirror", w.name));
        assert_ne!(m.name, w.name, "a wheel on the centre line");
        assert_eq!(Wheel { name: w.name.clone(), mount: w.mount.clone(), ..m.clone() }, *w, "wheel {} mirror numbers", w.name);
        let (s, sm) = (body.suspension(&w.name).unwrap(), body.suspension(&m.name).unwrap());
        assert_eq!(Suspension { wheel: s.wheel.clone(), ..sm.clone() }, *s, "suspension {} mirror", w.name);
        let (t, tm) = (body.tyre(&w.name).unwrap(), body.tyre(&m.name).unwrap());
        assert_eq!(Tyre { wheel: t.wheel.clone(), ..tm.clone() }, *t, "tyre {} mirror", w.name);
    }
    // the chassis is stable at the default physics step (1 ms) by the any-body Gershgorin rule
    let cfg = Config::default();
    let built = body.build();
    let h = cfg.dt / cfg.substeps as f64;
    assert_eq!(h, 0.001);
    assert_eq!(crash_sim::solver::body_step_error(h, &built.beams, &built.mass), None);
}

#[test]
fn wheel_start_center_hangs_below_the_mount_centroid() {
    let body = parse(CAR).unwrap();
    // mount nodes (0, 1, 0) and (1, 1, 0), rest length 0.5
    assert_eq!(body.wheel_start_center("w"), Some(Vec3::new(0.5, 0.5, 0.0)));
    assert_eq!(body.wheel_start_center("v"), None);
    // build() is the chassis only
    assert_eq!(body.build().pos.len(), 2);
}

#[test]
fn refuse_car_section_in_a_version_1_file() {
    for (line, kw) in [(5, "wheel"), (6, "suspension"), (7, "tyre"), (8, "controls")] {
        // only the first car line is reached, so drop the lines above the one tested
        let mut lines: Vec<&str> = CAR.split('\n').collect();
        lines[0] = "bsn 1";
        let kept: Vec<&str> = lines.iter().enumerate().filter(|(i, _)| *i < 4 || *i + 1 >= line).map(|(_, l)| *l).collect();
        let text = kept.join("\n");
        expect(&text, 5, Some(1), &format!("`{kw}` needs version 2"));
    }
}

#[test]
fn refuse_wheel_on_a_missing_node() {
    expect_after(&car("mount 0 1", "mount 0 7"), 5, "mount 0 ", "wheel `w` is mounted on missing node 7");
    expect_after(&car("mount 0 1", "mount 9"), 5, "mount ", "missing node 9");
}

#[test]
fn refuse_wheel_without_a_mount_node() {
    expect_after(&car("mount 0 1 ", "mount "), 5, "mount ", "wheel `w` needs at least one mount node");
}

#[test]
fn refuse_wheel_mount_listing_a_node_twice() {
    expect_after(&car("mount 0 1", "mount 0 1 0"), 5, "mount 0 1 ", "mount lists node 0 twice");
}

#[test]
fn refuse_wheel_mount_that_is_not_a_node_id() {
    expect_after(&car("mount 0 1", "mount 0 x"), 5, "mount 0 ", "mount node `x` is not a node id");
}

#[test]
fn refuse_duplicate_wheel() {
    let text = car("tyre w", "wheel w mount 0 radius 1 width 1 mass 1 inertia 1 steerable no driven no braked no\ntyre w");
    // the second wheel line comes after the suspension, so it is out of order first
    expect(&text, 7, Some(1), "after a later section");
    let text = car("suspension w", "wheel w mount 0 radius 1 width 1 mass 1 inertia 1 steerable no driven no braked no\nsuspension w");
    expect_after(&text, 6, "wheel ", "duplicate wheel `w`");
}

#[test]
fn refuse_nonpositive_wheel_radius() {
    expect_after(&car("radius 0.3", "radius 0"), 5, "radius ", "radius 0 must be > 0");
    expect_after(&car("radius 0.3", "radius -0.3"), 5, "radius ", "radius -0.3 must be > 0");
}

#[test]
fn refuse_nonpositive_wheel_width() {
    expect_after(&car("width 0.2", "width 0"), 5, "width ", "width 0 must be > 0");
}

#[test]
fn refuse_nonpositive_wheel_mass() {
    expect_after(&car("mass 10", "mass -1"), 5, "mass ", "wheel mass -1 must be > 0");
}

#[test]
fn refuse_nonpositive_wheel_inertia() {
    expect_after(&car("inertia 0.45", "inertia 0"), 5, "inertia ", "inertia 0 must be > 0");
}

#[test]
fn refuse_non_finite_wheel_number() {
    expect_after(&car("radius 0.3", "radius NaN"), 5, "radius ", "radius `NaN` is not finite");
    expect_after(&car("width 0.2", "width inf"), 5, "width ", "width `inf` is not finite");
}

#[test]
fn refuse_wheel_flag_that_is_not_yes_or_no() {
    expect_after(&car("steerable yes", "steerable true"), 5, "steerable ", "steerable `true` must be `yes` or `no`");
    expect_after(&car("driven yes", "driven 1"), 5, "driven ", "driven `1` must be `yes` or `no`");
    expect_after(&car("braked yes", "braked Yes"), 5, "braked ", "braked `Yes` must be `yes` or `no`");
}

#[test]
fn refuse_wheel_line_with_a_wrong_missing_or_extra_token() {
    expect_after(&car("width 0.2", "wide 0.2"), 5, "radius 0.3 ", "expected `width`");
    expect_after(&car(" braked yes", ""), 5, "driven yes", "missing `braked`");
    expect_after(&car("braked yes", "braked yes 1"), 5, "braked yes ", "unexpected `1`");
}

#[test]
fn refuse_suspension_for_an_unknown_wheel() {
    expect_after(&car("suspension w", "suspension v"), 6, "suspension ", "suspension names unknown wheel `v`");
}

#[test]
fn refuse_second_suspension_for_a_wheel() {
    let text = car("tyre w", "suspension w spring 1 damper 0 rest 0.5 travel 0.3 0.6\ntyre w");
    expect_after(&text, 7, "suspension ", "wheel `w` already has a suspension");
}

#[test]
fn refuse_wheel_without_a_suspension() {
    expect_after(&car("suspension w spring 1000 damper 10 rest 0.5 travel 0.3 0.6\n", ""), 5, "wheel ", "wheel `w` has no `suspension` line");
}

#[test]
fn refuse_nonpositive_spring_rate() {
    expect_after(&car("spring 1000", "spring 0"), 6, "spring ", "spring rate 0 must be > 0");
}

#[test]
fn refuse_negative_damper_rate() {
    expect_after(&car("damper 10", "damper -1"), 6, "damper ", "damper rate -1 must be >= 0");
    // an undamped suspension is allowed
    assert!(parse(&car("damper 10", "damper 0")).is_ok());
}

#[test]
fn refuse_travel_min_above_max() {
    expect_after(&car("travel 0.3 0.6", "travel 0.6 0.3"), 6, "travel ", "travel min 0.6 must be below travel max 0.3");
    expect_after(&car("travel 0.3 0.6", "travel 0.6 0.6"), 6, "travel ", "travel min 0.6 must be below travel max 0.6");
}

#[test]
fn refuse_negative_travel_min() {
    expect_after(&car("rest 0.5 travel 0.3 0.6", "rest 0.5 travel -0.1 0.6"), 6, "travel ", "travel min -0.1 must be >= 0");
}

#[test]
fn refuse_nonpositive_suspension_rest_length() {
    expect_after(&car("rest 0.5", "rest 0"), 6, "rest ", "suspension rest length 0 must be > 0");
}

#[test]
fn refuse_suspension_rest_length_outside_its_travel() {
    expect_after(&car("rest 0.5", "rest 0.7"), 6, "rest ", "rest length 0.7 must be inside the travel (0.3, 0.6)");
    // on a stop is not inside: the car must start off its stops
    expect_after(&car("rest 0.5", "rest 0.3"), 6, "rest ", "rest length 0.3 must be inside the travel (0.3, 0.6)");
    expect_after(&car("rest 0.5", "rest 0.6"), 6, "rest ", "must be inside the travel");
}

#[test]
fn refuse_tyre_for_an_unknown_wheel() {
    expect_after(&car("tyre w", "tyre v"), 7, "tyre ", "tyre names unknown wheel `v`");
}

#[test]
fn refuse_second_tyre_for_a_wheel() {
    let text = car("controls", "tyre w mu 1 radial_stiffness 1 radial_damping 0 long_stiffness 1 lat_stiffness 1\ncontrols");
    expect_after(&text, 8, "tyre ", "wheel `w` already has a tyre");
}

#[test]
fn refuse_wheel_without_a_tyre() {
    let text = car("tyre w mu 0.9 radial_stiffness 100000 radial_damping 100 long_stiffness 15 lat_stiffness 10\n", "");
    expect_after(&text, 5, "wheel ", "wheel `w` has no `tyre` line");
}

#[test]
fn refuse_nonpositive_tyre_grip() {
    expect_after(&car("mu 0.9", "mu 0"), 7, "mu ", "mu 0 must be > 0");
    expect_after(&car("mu 0.9", "mu -0.9"), 7, "mu ", "mu -0.9 must be > 0");
}

#[test]
fn refuse_bad_tyre_stiffness_or_damping() {
    expect_after(&car("radial_stiffness 100000", "radial_stiffness 0"), 7, "radial_stiffness ", "radial_stiffness 0 must be > 0");
    expect_after(&car("radial_damping 100", "radial_damping -1"), 7, "radial_damping ", "radial_damping -1 must be >= 0");
    expect_after(&car("long_stiffness 15", "long_stiffness 0"), 7, "long_stiffness ", "long_stiffness 0 must be > 0");
    expect_after(&car("lat_stiffness 10", "lat_stiffness -10"), 7, "lat_stiffness ", "lat_stiffness -10 must be > 0");
}

#[test]
fn refuse_second_controls_line() {
    let text = format!("{CAR}controls max_steer_deg 30 max_drive_torque 100 max_brake_torque 200\n");
    expect(&text, 9, Some(1), "second `controls` line");
}

#[test]
fn refuse_wheels_without_controls() {
    let text = car("controls max_steer_deg 30 max_drive_torque 100 max_brake_torque 200\n", "");
    expect(&text, 5, Some(1), "the body has wheels but no `controls` line");
}

#[test]
fn refuse_controls_without_wheels() {
    let text = "bsn 2\nmaterial m stiffness 100 damping 1\nnode 0 0 1 0 mass 1\ncontrols max_steer_deg 30 max_drive_torque 100 max_brake_torque 200\n";
    expect(text, 4, Some(1), "`controls` without any wheel");
}

#[test]
fn refuse_steer_angle_outside_0_to_90_degrees() {
    for bad in ["0", "-5", "90", "120"] {
        let text = car("max_steer_deg 30", &format!("max_steer_deg {bad}"));
        expect_after(&text, 8, "max_steer_deg ", &format!("max_steer_deg {bad} must be > 0 and < 90"));
    }
}

#[test]
fn refuse_nonpositive_drive_or_brake_torque() {
    expect_after(&car("max_drive_torque 100", "max_drive_torque 0"), 8, "max_drive_torque ", "max_drive_torque 0 must be > 0");
    expect_after(&car("max_brake_torque 200", "max_brake_torque -1"), 8, "max_brake_torque ", "max_brake_torque -1 must be > 0");
}

#[test]
fn refuse_controls_with_no_wheel_to_act_on() {
    let none = |flag: &str| car(&format!("{flag} yes"), &format!("{flag} no"));
    expect_after(&none("steerable"), 8, "max_steer_deg ", "no wheel is steerable");
    expect_after(&none("driven"), 8, "max_drive_torque ", "no wheel is driven");
    expect_after(&none("braked"), 8, "max_brake_torque ", "no wheel is braked");
}

#[test]
fn refuse_car_sections_out_of_order() {
    // tyre before suspension
    let lines: Vec<&str> = CAR.split('\n').collect();
    let swapped = [lines[0], lines[1], lines[2], lines[3], lines[4], lines[6], lines[5], lines[7], ""].join("\n");
    expect(&swapped, 7, Some(1), "after a later section");
    // a plane comes after the car, not before it
    let text = car("wheel w", "plane 5 0 0 -1 0 0\nwheel w");
    expect(&text, 6, Some(1), "after a later section");
    // a wheel before the groups
    let text = car("suspension w", "group g 0\nsuspension w");
    expect(&text, 6, Some(1), "after a later section");
}
