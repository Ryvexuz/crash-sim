//! The `.bsn` v1 format: round trips, the two scenario files, and every refusal by name.
//! Each refusal test checks the line, the column where the token is at fault, and the words
//! of the message, so a parser that refuses for the wrong reason or at the wrong place fails.

use crash_sim::format::{parse, write, Body, BodyBeam, Group, Material, Node, ParseError, Plane, Velocity};
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
    Body { materials, nodes, beams, groups, velocities, planes }
}

#[test]
fn parse_of_write_gives_the_same_body() {
    let mut rng = Rng(12345);
    let mut with_rest = 0;
    for case in 0..500 {
        let body = random_body(&mut rng);
        let text = write(&body);
        let back = parse(&text).unwrap_or_else(|e| panic!("case {case}: {e}\n{text}"));
        assert_eq!(back, body, "case {case}\n{text}");
        assert_eq!(write(&back), text, "case {case}: write is not stable");
        with_rest += text.matches(" rest ").count();
    }
    assert!(with_rest > 50, "the generator should exercise explicit rest lengths ({with_rest})");
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
    expect("bsn 2\nnode 0 0 0 0 mass 1\n", 1, Some(5), "unknown version 2");
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
