use super::*;

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
fn rolling_resistance_and_drag_area_are_optional_and_round_trip() {
    // missing keys: None (the vehicle code reads 0), and the file is written without them
    let body = parse(CAR).unwrap();
    let c = body.controls.as_ref().unwrap();
    assert_eq!((c.rolling_resistance, c.drag_area), (None, None));
    assert_eq!(write(&body), CAR);
    for (keys, rr, cda) in [
        (" rolling_resistance 0.012 drag_area 0.65", Some(0.012), Some(0.65)),
        (" rolling_resistance 0.012", Some(0.012), None),
        (" drag_area 0.65", None, Some(0.65)),
        (" rolling_resistance 0 drag_area 0", Some(0.0), Some(0.0)),
    ] {
        let text = car("max_brake_torque 200", &format!("max_brake_torque 200{keys}"));
        let body = parse(&text).unwrap_or_else(|e| panic!("{keys}: {e}"));
        let c = body.controls.as_ref().unwrap();
        assert_eq!((c.rolling_resistance, c.drag_area), (rr, cda), "{keys}");
        assert_eq!(write(&body), text, "{keys}: write(parse(x)) differs");
        assert_eq!(parse(&write(&body)).unwrap(), body, "{keys}");
    }
}

#[test]
fn refuse_bad_rolling_resistance_or_drag_area() {
    let with = |keys: &str| car("max_brake_torque 200", &format!("max_brake_torque 200 {keys}"));
    for (key, other) in [("rolling_resistance", "drag_area 0.65"), ("drag_area", "")] {
        let pre = format!("{key} ");
        let rest = if other.is_empty() { String::new() } else { format!(" {other}") };
        for (bad, words) in [
            ("-0.01", format!("{key} -0.01 must be >= 0")),
            ("-1e-300", format!("{key} -0.000")),
            ("NaN", format!("{key} `NaN` is not finite")),
            ("nan", format!("{key} `nan` is not finite")),
            ("inf", format!("{key} `inf` is not finite")),
            ("-inf", format!("{key} `-inf` is not finite")),
            ("1e999", format!("{key} `1e999` is not finite")),
            ("x", format!("{key} `x` is not a number")),
        ] {
            let text = with(&format!("{key} {bad}{rest}"));
            expect_after(&text, 8, &pre, &words);
        }
        // the key with no value
        let text = with(key);
        expect(&text, 8, Some(text.split('\n').nth(7).unwrap().chars().count() + 1), &format!("missing {key}"));
    }
    // the order is rolling_resistance, then drag_area; anything else after them is refused
    let text = with("drag_area 0.65 rolling_resistance 0.012");
    expect_after(&text, 8, "0.65 ", "unexpected `rolling_resistance` at the end of the line");
    let text = with("rolling_resistance 0.012 drag_area 0.65 drag_area 0.65");
    expect_after(&text, 8, "drag_area 0.65 ", "unexpected `drag_area` at the end of the line");
    let text = with("rolling_resistance 0.012 rolling_resistance 0.012");
    expect_after(&text, 8, "0.012 ", "unexpected `rolling_resistance` at the end of the line");
}

#[test]
fn refuse_controls_with_no_wheel_to_act_on() {
    let none = |flag: &str| car(&format!("{flag} yes"), &format!("{flag} no"));
    expect_after(&none("steerable"), 8, "max_steer_deg ", "no wheel is steerable");
    expect_after(&none("driven"), 8, "max_drive_torque ", "no wheel is driven");
    expect_after(&none("braked"), 8, "max_brake_torque ", "no wheel is braked");
}
