use super::*;
use crash_sim::format::{MAX_SOLIDS, MAX_TETS};

#[test]
fn version_3_file_round_trips_byte_identical() {
    // the base, and a file with every line type: two solids, a car and a plane
    let base = v3("");
    let body = parse(&base).unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(write(&body), base);
    let text = "bsn 3\n\
material m stiffness 100 damping 1 yield_strain 0.02\n\
solid crush plateau_stress 250000.5 yield_strain 0.010000000000000002 damping_ratio 0\n\
solid hard plateau_stress 2500000 yield_strain 0.5 damping_ratio 1\n\
node 0 0 1 0 mass 1\n\
node 1 1 1 0 mass 1\n\
node 2 0 2 0 mass 1\n\
node 3 0 1 1 mass 1\n\
node 7 1 2 1 mass 0.25\n\
beam 0 1 m\n\
beam 1 7 m rest 2\n\
tet 0 1 2 3 crush\n\
tet 1 2 3 7 hard\n\
group g 0 7\n\
velocity g 1 0 0\n\
wheel w mount 0 1 radius 0.3 width 0.2 mass 10 inertia 0.45 steerable yes driven yes braked yes\n\
suspension w spring 1000 damper 10 rest 0.5 travel 0.3 0.6\n\
tyre w mu 0.9 radial_stiffness 100000 radial_damping 100 long_stiffness 15 lat_stiffness 10\n\
controls max_steer_deg 30 max_drive_torque 100 max_brake_torque 200 drag_area 0.65\n\
plane 5 0 0 -1 0 0\n";
    let body = parse(text).unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(
        write(&body),
        text,
        "write(parse(x)) differs for the full version 3 file"
    );
    assert_eq!(parse(&write(&body)).unwrap(), body);
}

#[test]
fn version_3_file_without_solids_or_tets_loads_like_version_1_and_2() {
    let v1 = base("beam 0 1 m\n");
    let as_v3 = v1.replacen("bsn 1\n", "bsn 3\n", 1);
    assert_eq!(parse(&as_v3).unwrap(), parse(&v1).unwrap());
    assert_eq!(
        write(&parse(&as_v3).unwrap()),
        v1,
        "the lowest version that holds the body"
    );
    let car_v3 = CAR.replacen("bsn 2\n", "bsn 3\n", 1);
    assert_eq!(write(&parse(&car_v3).unwrap()), CAR);
    // a solid alone (no tet) still needs version 3
    let solid_only = v3("").replace("tet 0 1 2 3 s\n", "");
    assert_eq!(write(&parse(&solid_only).unwrap()), solid_only);
}

#[test]
fn version_3_keeps_comments_out_and_the_order_of_tets() {
    let text =
        v3("tet 1 2 3 4 s\n").replace("tet 0 1 2 3 s\n", "# the first tet\n\ntet 0 1 2 3 s\n");
    let body = parse(&text).unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(write(&body), v3("tet 1 2 3 4 s\n"));
}

#[test]
fn refuse_solid_or_tet_before_version_3() {
    for version in [1, 2] {
        let text = v3("").replacen("bsn 3", &format!("bsn {version}"), 1);
        expect(
            &text,
            3,
            Some(1),
            &format!("`solid` needs version 3: this file is `bsn {version}`"),
        );
        let text = text.replace(
            "solid s plateau_stress 1000000 yield_strain 0.02 damping_ratio 0.01\n",
            "",
        );
        expect(
            &text,
            9,
            Some(1),
            &format!("`tet` needs version 3: this file is `bsn {version}`"),
        );
    }
}

#[test]
fn refuse_unknown_version_names_version_3_as_read() {
    expect(
        "bsn 4\nnode 0 0 0 0 mass 1\n",
        1,
        Some(5),
        "unknown version 4: this parser reads versions 1, 2 and 3",
    );
    expect(
        "bsn x\n",
        1,
        Some(5),
        "the first line must be `bsn 1`, `bsn 2` or `bsn 3`",
    );
}

#[test]
fn refuse_duplicate_solid() {
    let text = v3_with(
        "node 0 ",
        "solid s plateau_stress 1 yield_strain 0.5 damping_ratio 0\nnode 0 ",
    );
    expect_after(&text, 4, "solid ", "duplicate solid `s`");
}

#[test]
fn refuse_bad_solid_name() {
    expect_after(
        &v3_with("solid s ", "solid 1s "),
        3,
        "solid ",
        "solid name `1s` is not a name",
    );
}

#[test]
fn refuse_nonpositive_plateau_stress() {
    for bad in ["0", "-1", "-0"] {
        let text = v3_with("plateau_stress 1000000", &format!("plateau_stress {bad}"));
        expect_after(&text, 3, "plateau_stress ", "must be > 0");
    }
    expect_after(
        &v3_with("plateau_stress 1000000", "plateau_stress -2.5"),
        3,
        "plateau_stress ",
        "plateau_stress -2.5 must be > 0",
    );
}

#[test]
fn refuse_solid_yield_strain_outside_0_to_1() {
    for bad in ["0", "1", "-0.1", "1.5", "1e-400"] {
        let text = v3_with("yield_strain 0.02", &format!("yield_strain {bad}"));
        expect_after(&text, 3, "yield_strain ", "must be > 0 and < 1");
    }
    expect_after(
        &v3_with("yield_strain 0.02", "yield_strain 1"),
        3,
        "yield_strain ",
        "yield_strain 1 must be > 0 and < 1",
    );
    // just inside both ends is accepted
    for ok in ["0.000001", "0.999999"] {
        assert!(
            parse(&v3_with("yield_strain 0.02", &format!("yield_strain {ok}"))).is_ok(),
            "{ok}"
        );
    }
}

#[test]
fn refuse_damping_ratio_outside_0_to_1() {
    for bad in ["-0.01", "1.01", "2"] {
        let text = v3_with("damping_ratio 0.01", &format!("damping_ratio {bad}"));
        expect_after(
            &text,
            3,
            "damping_ratio ",
            &format!("damping_ratio {bad} must be >= 0 and <= 1"),
        );
    }
    // both ends are accepted
    for ok in ["0", "1"] {
        assert!(
            parse(&v3_with(
                "damping_ratio 0.01",
                &format!("damping_ratio {ok}")
            ))
            .is_ok(),
            "{ok}"
        );
    }
}

#[test]
fn refuse_non_finite_or_non_numeric_solid_numbers() {
    for (key, value) in [
        ("plateau_stress", "1000000"),
        ("yield_strain", "0.02"),
        ("damping_ratio", "0.01"),
    ] {
        for (bad, words) in [
            ("NaN", format!("{key} `NaN` is not finite")),
            ("nan", format!("{key} `nan` is not finite")),
            ("inf", format!("{key} `inf` is not finite")),
            ("-inf", format!("{key} `-inf` is not finite")),
            ("1e999", format!("{key} `1e999` is not finite")),
            ("x", format!("{key} `x` is not a number")),
            ("1,5", format!("{key} `1,5` is not a number")),
        ] {
            let text = v3_with(&format!("{key} {value}"), &format!("{key} {bad}"));
            expect_after(&text, 3, &format!("{key} "), &words);
        }
    }
}

#[test]
fn refuse_solid_line_with_a_wrong_missing_or_extra_token() {
    expect_after(
        &v3_with("plateau_stress", "plateau"),
        3,
        "solid s ",
        "expected `plateau_stress`, found `plateau`",
    );
    expect_after(
        &v3_with(" damping_ratio 0.01", " damping 0.01"),
        3,
        "0.02 ",
        "expected `damping_ratio`, found `damping`",
    );
    // the keys are in a fixed order
    let swapped = v3_with(
        "yield_strain 0.02 damping_ratio 0.01",
        "damping_ratio 0.01 yield_strain 0.02",
    );
    expect_after(
        &swapped,
        3,
        "1000000 ",
        "expected `yield_strain`, found `damping_ratio`",
    );
    let text = v3_with(" damping_ratio 0.01", "");
    expect(
        &text,
        3,
        Some(text.split('\n').nth(2).unwrap().chars().count() + 1),
        "missing `damping_ratio`",
    );
    let text = v3_with("damping_ratio 0.01", "damping_ratio");
    expect(
        &text,
        3,
        Some(text.split('\n').nth(2).unwrap().chars().count() + 1),
        "missing damping_ratio",
    );
    expect_after(
        &v3_with("damping_ratio 0.01", "damping_ratio 0.01 x"),
        3,
        "0.01 ",
        "unexpected `x` at the end of the line",
    );
}

#[test]
fn refuse_solid_out_of_order() {
    // before the materials
    let text = v3("").replacen(
        "material m stiffness 100 damping 1\nsolid s plateau_stress 1000000 yield_strain 0.02 damping_ratio 0.01\n",
        "solid s plateau_stress 1000000 yield_strain 0.02 damping_ratio 0.01\nmaterial m stiffness 100 damping 1\n",
        1,
    );
    expect(&text, 3, Some(1), "after a later section");
    // after a node
    let text = v3_with(
        "node 1 ",
        "solid t plateau_stress 1 yield_strain 0.5 damping_ratio 0\nnode 1 ",
    );
    expect(&text, 5, Some(1), "after a later section");
}

#[test]
fn refuse_tet_to_missing_node() {
    for (from, to, prefix) in [
        ("tet 0 1 2 3", "tet 9 1 2 3", "tet "),
        ("tet 0 1 2 3", "tet 0 1 2 9", "tet 0 1 2 "),
    ] {
        expect_after(&v3_with(from, to), 10, prefix, "tet to missing node 9");
    }
}

#[test]
fn refuse_tet_listing_a_node_twice() {
    expect_after(
        &v3_with("tet 0 1 2 3", "tet 0 1 2 0"),
        10,
        "tet 0 1 2 ",
        "tet lists node 0 twice",
    );
    expect_after(
        &v3_with("tet 0 1 2 3", "tet 0 1 1 3"),
        10,
        "tet 0 1 ",
        "tet lists node 1 twice",
    );
}

#[test]
fn refuse_tet_with_unknown_solid() {
    expect_after(
        &v3_with("tet 0 1 2 3 s", "tet 0 1 2 3 t"),
        10,
        "tet 0 1 2 3 ",
        "tet uses unknown solid `t`",
    );
    // a material is not a solid
    expect_after(
        &v3_with("tet 0 1 2 3 s", "tet 0 1 2 3 m"),
        10,
        "tet 0 1 2 3 ",
        "tet uses unknown solid `m`",
    );
}

#[test]
fn refuse_tet_line_with_a_bad_missing_or_extra_token() {
    expect_after(
        &v3_with("tet 0 1 2 3", "tet 0 1 x 3"),
        10,
        "tet 0 1 ",
        "tet node c `x` is not a node id",
    );
    expect_after(
        &v3_with("tet 0 1 2 3", "tet 0 01 2 3"),
        10,
        "tet 0 ",
        "tet node b `01` is not a node id",
    );
    let text = v3_with("tet 0 1 2 3 s", "tet 0 1 2 3");
    expect(&text, 10, Some(12), "missing tet solid");
    expect_after(
        &v3_with("tet 0 1 2 3 s", "tet 0 1 2 s"),
        10,
        "tet 0 1 2 ",
        "tet node d `s` is not a node id",
    );
    expect_after(
        &v3_with("tet 0 1 2 3 s", "tet 0 1 2 3 s 4"),
        10,
        "tet 0 1 2 3 s ",
        "unexpected `4` at the end of the line",
    );
}

#[test]
fn refuse_inverted_tet() {
    // an odd permutation of a positive tet turns it inside out
    for bad in ["tet 1 0 2 3", "tet 0 1 3 2", "tet 3 1 2 0"] {
        expect_after(
            &v3_with("tet 0 1 2 3", bad),
            10,
            "tet ",
            "is inverted: its rest volume -0.16666666666666666 is negative",
        );
    }
    // an even permutation is the same tet, positive
    assert!(parse(&v3_with("tet 0 1 2 3", "tet 1 2 0 3")).is_ok());
}

#[test]
fn refuse_tet_below_the_minimum_rest_volume() {
    // four coplanar nodes (z = 0), three collinear ones, and a tiny but well-shaped tet
    let flat = v3("").replacen("node 3 0 0 1 ", "node 3 1 1 0 ", 1);
    expect_after(
        &flat,
        10,
        "tet ",
        "tet 0 1 2 3: its rest volume 0 is below the minimum 0.000000001 m^3",
    );
    let collinear = v3("").replacen("node 2 0 1 0 ", "node 2 2 0 0 ", 1);
    expect_after(&collinear, 10, "tet ", "rest volume 0 is below the minimum");
    let tiny = v3("")
        .replacen("node 1 1 0 0 ", "node 1 0.001 0 0 ", 1)
        .replacen("node 2 0 1 0 ", "node 2 0 0.001 0 ", 1)
        .replacen("node 3 0 0 1 ", "node 3 0 0 0.001 ", 1);
    // its volume is 1e-9 / 6, so the beam 0-1 is fine but the tet is refused
    expect_after(&tiny, 10, "tet ", "is below the minimum 0.000000001 m^3");
    // twelve times larger (edges 0.002, 0.001 and 0.006 m: volume 2e-9 m^3) is accepted
    let above = v3("")
        .replacen("node 1 1 0 0 ", "node 1 0.002 0 0 ", 1)
        .replacen("node 2 0 1 0 ", "node 2 0 0.001 0 ", 1)
        .replacen("node 3 0 0 1 ", "node 3 0 0 0.006 ", 1);
    assert!(parse(&above).is_ok(), "{:?}", parse(&above).err());
}

#[test]
fn refuse_tet_too_flat_for_its_size() {
    // a sliver: a 1000 m square base with its fourth node 0.0001 m above a corner. Its volume
    // (1000 x 1000 x 0.0001 / 6 = 0.0167 m^3) is far above the absolute minimum, but over the
    // cube of its longest edge (1414 m) it is 5.9e-12, below the shape minimum 1e-6
    let sliver = v3("")
        .replacen("node 1 1 0 0 ", "node 1 1000 0 0 ", 1)
        .replacen("node 2 0 1 0 ", "node 2 0 1000 0 ", 1)
        .replacen("node 3 0 0 1 ", "node 3 1000 1000 0.0001 ", 1);
    expect_after(
        &sliver,
        10,
        "tet ",
        "tet 0 1 2 3 is too flat: its rest volume",
    );
    // a thin tet that is still usable: height 0.001 m on the unit corner, volume 1/6000 m^3,
    // longest edge sqrt(2) m, ratio 5.9e-5 (above 1e-6): accepted
    let thin = v3("").replacen("node 3 0 0 1 ", "node 3 0 0 0.001 ", 1);
    assert!(parse(&thin).is_ok(), "{:?}", parse(&thin).err());
}

/// The shape ratio is volume / L / L / L, never volume / L^3: a well-shaped tet whose edges
/// are 5e102 m has a finite volume (2.1e307 m^3) but L^3 (3.5e308) overflows, and it must
/// still be accepted, with the shape 1/(6 x 2^1.5) = 0.059 it has at any size.
#[test]
fn huge_well_shaped_tet_is_not_refused_by_an_overflowing_cube() {
    let big = v3("")
        .replacen("node 1 1 0 0 ", "node 1 5e102 0 0 ", 1)
        .replacen("node 2 0 1 0 ", "node 2 0 5e102 0 ", 1)
        .replacen("node 3 0 0 1 ", "node 3 0 0 5e102 ", 1);
    let body = parse(&big).unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(body.tets.len(), 1);
    assert_eq!(parse(&write(&body)).unwrap(), body);
}

/// The reviewer's tet (round 3): edges of 6e102 m on the axes. Its volume is
/// (6e102)^3 / 6 = 3.6e307 m^3, finite, but the triple product before the / 6 is 2.16e308,
/// above the largest f64 (1.8e308). It must be accepted, with that volume.
#[test]
fn huge_well_shaped_tet_whose_triple_product_overflows_is_accepted() {
    let big = v3("")
        .replacen("node 1 1 0 0 ", "node 1 6e102 0 0 ", 1)
        .replacen("node 2 0 1 0 ", "node 2 0 6e102 0 ", 1)
        .replacen("node 3 0 0 1 ", "node 3 0 0 6e102 ", 1);
    let body = parse(&big).unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(body.tets.len(), 1);
    // inverted, the same tet is refused as inverted (so its volume was computed, not skipped)
    let inverted = big.replacen("tet 0 1 2 3 s", "tet 1 0 2 3 s", 1);
    expect_after(&inverted, 10, "tet ", "is inverted: its rest volume -");
}

/// Tiny well-shaped tets on either side of the minimum volume 1e-9 m^3: edges of 1.9 mm on
/// the axes give 1.9e-3^3 / 6 = 1.143e-9 m^3 (accepted) and 1.8 mm give 0.972e-9 (refused).
#[test]
fn tiny_well_shaped_tets_either_side_of_the_minimum_volume() {
    let corner = |e: &str| {
        v3("")
            .replacen("node 1 1 0 0 ", &format!("node 1 {e} 0 0 "), 1)
            .replacen("node 2 0 1 0 ", &format!("node 2 0 {e} 0 "), 1)
            .replacen("node 3 0 0 1 ", &format!("node 3 0 0 {e} "), 1)
    };
    assert!(
        parse(&corner("0.0019")).is_ok(),
        "{:?}",
        parse(&corner("0.0019")).err()
    );
    expect_after(
        &corner("0.0018"),
        10,
        "tet ",
        "is below the minimum 0.000000001 m^3",
    );
}

/// The caps of round 3, written out so this test fails if they change without it.
const CAP_SOLIDS: usize = 1000;
const CAP_TETS: usize = 1_000_000;

#[test]
fn refuse_more_solids_than_the_cap() {
    assert_eq!(
        (MAX_SOLIDS, MAX_TETS),
        (CAP_SOLIDS, CAP_TETS),
        "the caps changed: change this test too"
    );
    let mut text = String::from("bsn 3\n");
    for i in 0..CAP_SOLIDS + 1 {
        text.push_str(&format!(
            "solid s{i} plateau_stress 1 yield_strain 0.5 damping_ratio 0\n"
        ));
    }
    text.push_str("node 0 0 0 0 mass 1\n");
    // the record after the cap, line 1 + cap + 1, at its keyword; every line above it passed
    expect(
        &text,
        CAP_SOLIDS + 2,
        Some(1),
        &format!("too many solids: a body has at most {CAP_SOLIDS}"),
    );
    // exactly the cap is accepted
    let at_cap = text.replacen(
        &format!("solid s{CAP_SOLIDS} plateau_stress 1 yield_strain 0.5 damping_ratio 0\n"),
        "",
        1,
    );
    assert_eq!(parse(&at_cap).unwrap().solids.len(), CAP_SOLIDS);
}

/// A conforming lattice of unit cubes, each cut into the 6 tets of its main diagonal (the
/// Kuhn cut: 0, e_i, e_i + e_j, (1, 1, 1) for each order (i, j, k) of the axes, two nodes
/// swapped for an odd order so every volume is +1/6), and the first `count` of them.
fn lattice(count: usize) -> String {
    let (nx, ny, nz) = (56usize, 56usize, 54usize);
    assert!(nx * ny * nz * 6 >= count);
    let id = |x: usize, y: usize, z: usize| x + (nx + 1) * (y + (ny + 1) * z);
    let mut text =
        String::from("bsn 3\nsolid s plateau_stress 1 yield_strain 0.5 damping_ratio 0\n");
    for z in 0..=nz {
        for y in 0..=ny {
            for x in 0..=nx {
                text.push_str(&format!("node {} {x} {y} {z} mass 1\n", id(x, y, z)));
            }
        }
    }
    let orders: [([usize; 3], bool); 6] = [
        ([0, 1, 2], true),
        ([1, 2, 0], true),
        ([2, 0, 1], true),
        ([0, 2, 1], false),
        ([2, 1, 0], false),
        ([1, 0, 2], false),
    ];
    let mut n = 0;
    'cells: for z in 0..nz {
        for y in 0..ny {
            for x in 0..nx {
                for (order, even) in orders {
                    let mut p = [x, y, z];
                    let a = id(p[0], p[1], p[2]);
                    p[order[0]] += 1;
                    let b = id(p[0], p[1], p[2]);
                    p[order[1]] += 1;
                    let c = id(p[0], p[1], p[2]);
                    let d = id(x + 1, y + 1, z + 1);
                    let (b, c) = if even { (b, c) } else { (c, b) };
                    text.push_str(&format!("tet {a} {b} {c} {d} s\n"));
                    n += 1;
                    if n == count {
                        break 'cells;
                    }
                }
            }
        }
    }
    text
}

#[test]
fn refuse_more_tets_than_the_cap() {
    // a small lattice is valid: every tet positive and distinct
    let small = lattice(6 * 56 * 2);
    assert_eq!(
        parse(&small).unwrap_or_else(|e| panic!("{e}")).tets.len(),
        6 * 56 * 2
    );
    // cap + 1 tets: the last one is refused at its keyword, so the cap itself passed
    let text = lattice(CAP_TETS + 1);
    let last = text.split_terminator('\n').count();
    expect(
        &text,
        last,
        Some(1),
        &format!("too many tets: a body has at most {CAP_TETS}"),
    );
}

#[test]
fn refuse_tet_whose_volume_is_not_finite() {
    let far = v3("")
        .replacen("node 1 1 0 0 ", "node 1 1e308 0 0 ", 1)
        .replacen("node 0 0 0 0 ", "node 0 -1e308 0 0 ", 1);
    // the beam 0-1 overflows first, so give it a rest length
    let far = far.replacen("beam 0 1 m\n", "beam 0 1 m rest 1\n", 1);
    expect_after(
        &far,
        10,
        "tet ",
        "tet 0 1 2 3: its size is not finite: an edge is longer than the largest f64",
    );
    // every edge finite (1.7e103 m at most), but the volume (1.2e103)^3 / 6 = 2.9e308 m^3 is
    // above the largest f64
    let vast = v3("")
        .replacen("node 1 1 0 0 ", "node 1 1.2e103 0 0 ", 1)
        .replacen("node 2 0 1 0 ", "node 2 0 1.2e103 0 ", 1)
        .replacen("node 3 0 0 1 ", "node 3 0 0 1.2e103 ", 1);
    expect_after(
        &vast,
        10,
        "tet ",
        "tet 0 1 2 3: its size is not finite: its rest volume overflows f64",
    );
}

#[test]
fn refuse_duplicate_tet() {
    // the same four nodes in any order, even or odd
    expect_after(
        &v3("tet 1 2 0 3 s\n"),
        11,
        "tet ",
        "duplicate tet: nodes 1 2 0 3 already form a tet",
    );
    let text = v3_with("solid s plateau_stress 1000000 yield_strain 0.02 damping_ratio 0.01\n", "solid s plateau_stress 1000000 yield_strain 0.02 damping_ratio 0.01\nsolid u plateau_stress 1 yield_strain 0.5 damping_ratio 0\n");
    // in another solid too: one set of nodes is one tet
    expect_after(&(text + "tet 3 1 2 0 u\n"), 12, "tet ", "duplicate tet");
}

#[test]
fn refuse_tet_out_of_order() {
    // before the beams
    let text = v3("").replacen(
        "beam 0 1 m\ntet 0 1 2 3 s\n",
        "tet 0 1 2 3 s\nbeam 0 1 m\n",
        1,
    );
    expect(&text, 10, Some(1), "after a later section");
    // after a group
    expect(
        &v3("group g 0\ntet 1 2 3 4 s\n"),
        12,
        Some(1),
        "after a later section",
    );
    // a node after a tet: nodes come first, so a tet can only name earlier nodes
    expect(
        &v3("node 5 2 2 2 mass 1\n"),
        11,
        Some(1),
        "after a later section",
    );
}

#[test]
fn hostile_solid_and_tet_input_is_refused_or_bounded() {
    // a 50 MB number is parsed in one pass and refused as not finite
    let huge = "9".repeat(50 * 1024 * 1024);
    let text = v3_with("plateau_stress 1000000", &format!("plateau_stress {huge}"));
    let e = refuse(&text);
    assert_eq!(
        (e.line, e.column),
        (3, Some(col_after(&text, 3, "plateau_stress ")))
    );
    assert!(
        e.message.contains("is not finite"),
        "{}",
        &e.message[..200.min(e.message.len())]
    );
    // a 50 MB trailing token on a tet line
    let text = v3_with("tet 0 1 2 3 s", &format!("tet 0 1 2 3 s {huge}"));
    let e = refuse(&text);
    assert_eq!(
        (e.line, e.column),
        (10, Some(col_after(&text, 10, "tet 0 1 2 3 s ")))
    );
    // a 50 MB solid name on a tet line is an unknown solid, refused by name
    let text = v3_with("tet 0 1 2 3 s", &format!("tet 0 1 2 3 s{huge}"));
    assert_eq!(refuse(&text).line, 10);
    // a flood: the most solids a body may have and many tets, each tet on fresh nodes, parses
    // (solid and tet lookups are by hash, so the time is linear in the lines)
    let n = 20_000;
    let mut text = String::from("bsn 3\n");
    for i in 0..CAP_SOLIDS {
        text.push_str(&format!(
            "solid s{i} plateau_stress 1 yield_strain 0.5 damping_ratio 0\n"
        ));
    }
    for i in 0..n {
        let x = (i * 2) as f64;
        text.push_str(&format!("node {} {x} 0 0 mass 1\nnode {} {} 0 0 mass 1\nnode {} {x} 1 0 mass 1\nnode {} {x} 0 1 mass 1\n", 4 * i, 4 * i + 1, x + 1.0, 4 * i + 2, 4 * i + 3));
    }
    for i in 0..n {
        text.push_str(&format!(
            "tet {} {} {} {} s{}\n",
            4 * i,
            4 * i + 1,
            4 * i + 2,
            4 * i + 3,
            (n - 1 - i) % CAP_SOLIDS
        ));
    }
    let body = parse(&text).unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(write(&body), text);
    // the same flood with one duplicate at the very end is refused on its line
    let last = 1 + CAP_SOLIDS + 4 * n + n + 1;
    let dup = format!("{text}tet 1 2 3 0 s0\n");
    let e = refuse(&dup);
    assert_eq!(e.line, last);
    assert!(e.message.contains("duplicate tet"), "{}", e.message);
}

