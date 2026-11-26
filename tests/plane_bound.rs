use crash_sim::collide::Plane;
use crash_sim::format::parse;
use crash_sim::solver_m5::MAX_COORD_M;
use crash_sim::vec3::Vec3;

#[test]
fn plane_point_at_limit_is_accepted() {
    for value in [MAX_COORD_M, -MAX_COORD_M, 0.0, f64::from_bits(1)] {
        for point in [
            Vec3::new(value, 0.0, 0.0),
            Vec3::new(0.0, value, 0.0),
            Vec3::new(0.0, 0.0, value),
            Vec3::new(value, value, value),
        ] {
            let plane = Plane::new(point, Vec3::new(0.0, 1.0, 0.0))
                .unwrap_or_else(|error| panic!("point {point:?}: {error}"));
            assert_eq!(plane.point, point);
            assert_eq!(plane.normal, Vec3::new(0.0, 1.0, 0.0));
        }
    }
}

#[test]
fn plane_point_past_limit_is_refused() {
    let next = f64::from_bits(MAX_COORD_M.to_bits() + 1);
    for value in [
        next,
        -next,
        2.0_f64.powi(42),
        -2.0_f64.powi(42),
        f64::MAX,
        -f64::MAX,
    ] {
        for point in [
            Vec3::new(value, 0.0, 0.0),
            Vec3::new(0.0, value, 0.0),
            Vec3::new(0.0, 0.0, value),
        ] {
            let error = Plane::new(point, Vec3::new(0.0, 1.0, 0.0))
                .expect_err("a point beyond the coordinate limit must be refused");
            let expected = format!(
                "plane point ({}, {}, {}) exceeds the coordinate limit of {} m",
                point.x, point.y, point.z, MAX_COORD_M
            );
            assert_eq!(error, expected);

            let text = format!(
                "bsn 1\nnode 0 0 0 0 mass 1\nplane {} {} {} 0 1 0\n",
                point.x, point.y, point.z
            );
            let body = parse(&text).expect("the plane line is syntactically valid");
            assert_eq!(body.planes.len(), 1);
            let parsed_error = Plane::from_format(&body.planes[0])
                .expect_err("a parsed plane must use the same coordinate bound");
            assert_eq!(parsed_error, expected);
        }
    }
}

#[test]
fn astra_far_plane_is_refused() {
    let far = 2.0_f64.powi(42);
    let normal = Vec3::new(0.0, 1.0, 1.0);
    let error = Plane::new(Vec3::new(0.0, far, -far), normal)
        .expect_err("the far point loses the node's offset in distance calculations");
    assert!(error.contains("plane point"), "{error}");
    assert!(error.contains(&format!("{MAX_COORD_M} m")), "{error}");

    let plane = Plane::new(Vec3::ZERO, normal).expect("the same plane through the origin builds");
    let distance = plane.distance(Vec3::new(0.0, -0.000333, 0.0));
    assert!(distance < 0.0, "node behind y + z = 0: distance={distance}");
}
