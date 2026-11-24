//! Check 2.1's ai_race tests; the CLI slice appends the other six.

use crash_sim::ai::DriverParams;
use crash_sim::race::{self, ProfileParams, RaceEnd, RaceSetup};
use crash_sim::track::Track;
use crash_sim::{Body, Config};

fn body() -> Body {
    crash_sim::format::parse(include_str!("../scenarios/car_basic.bsn")).unwrap()
}

fn setup<'a>(cfg: &'a Config, body: &'a Body, track: &'a Track) -> RaceSetup<'a> {
    RaceSetup {
        cfg,
        body,
        track,
        cars: 1,
        laps: 1,
        time_limit_s: 70.0,
        grid_gap_m: 10.0,
        driver: DriverParams {
            lookahead_k: 0.6,
            lookahead_min: 3.0,
            lookahead_max: 12.0,
            kp: 0.8,
            ki: 0.3,
            integral_max: 2.0,
        },
        profile: ProfileParams {
            mu: 0.6,
            g: 9.81,
            a_accel: 2.0,
            a_brake: 4.0,
            v_max: 15.0,
        },
    }
}

#[test]
fn oval_tight_differs_only_in_width() {
    let oval_text = include_str!("../scenarios/tracks/oval.track");
    let tight_text = include_str!("../scenarios/tracks/oval_tight.track");
    assert_eq!(oval_text.matches("\nwidth ").count(), 1);
    assert_eq!(oval_text.matches("\nwidth 12\n").count(), 1);
    assert_eq!(
        tight_text,
        oval_text.replacen("\nwidth 12\n", "\nwidth 1.6\n", 1)
    );
    assert!(!tight_text.contains('\r'));
    assert!(tight_text.ends_with('\n'));
    let oval = Track::parse(oval_text).unwrap();
    let tight = Track::parse(tight_text).unwrap();
    assert_eq!(tight.points, oval.points);
    assert_eq!(tight.start, oval.start);
    assert_eq!(tight.checkpoints, oval.checkpoints);
    assert_eq!(tight.grid, oval.grid);
    assert!(!tight.points.is_empty());
    assert_eq!(tight.widths.len(), tight.points.len());
    assert_eq!(oval.widths.len(), oval.points.len());
    for width in &tight.widths {
        assert_eq!(width.to_bits(), 1.6_f64.to_bits());
    }
    for width in &oval.widths {
        assert_eq!(width.to_bits(), 12.0_f64.to_bits());
    }
    let body = body();
    assert!(!body.nodes.is_empty());
    let min_z = body
        .nodes
        .iter()
        .map(|n| n.pos.z)
        .fold(f64::INFINITY, f64::min);
    let max_z = body
        .nodes
        .iter()
        .map(|n| n.pos.z)
        .fold(f64::NEG_INFINITY, f64::max);
    let span = max_z - min_z;
    assert!((span - 1.8).abs() <= 1e-12, "node z span: {span}");
    assert!(1.6 < span, "tight width must be below node z span {span}");
}

#[test]
fn wall_rule_counts_tyre_outer_edges() {
    let oval_text = include_str!("../scenarios/tracks/oval.track");
    assert_eq!(oval_text.matches("\nwidth 12\n").count(), 1);
    let track = Track::parse(&oval_text.replace("\nwidth 12\n", "\nwidth 2\n")).unwrap();
    let cfg = Config::default();
    let original = body();
    let mut wide = original.clone();
    assert!(!wide.wheels.is_empty());
    for wheel in &mut wide.wheels {
        wheel.width = 0.6;
    }
    let mut s = setup(&cfg, &wide, &track);
    s.time_limit_s = 0.02;
    let result = race::run(&s).unwrap();
    assert!(matches!(result.end, RaceEnd::TimeLimit), "{:?}", result.end);
    assert_eq!(result.substeps, 20);
    assert_eq!(result.cars.len(), 1);
    let centre = race::car_position(result.multi.sim(0));
    let nodes = result.multi.chassis_positions(0);
    assert!(!nodes.is_empty());
    assert!(
        !race::wall_contact(&track, centre, 0.0, nodes, &[], &[]),
        "chassis outside: centre={centre:?}, nodes={nodes:?}"
    );
    assert_eq!(result.cars[0].wall_substeps, 20);
    assert_eq!(
        result.cars[0].first_wall_t.unwrap().to_bits(),
        result.h.to_bits()
    );

    let mut s = setup(&cfg, &original, &track);
    s.time_limit_s = 0.02;
    let control = race::run(&s).unwrap();
    assert!(
        matches!(control.end, RaceEnd::TimeLimit),
        "{:?}",
        control.end
    );
    assert_eq!(control.substeps, 20);
    assert_eq!(control.cars.len(), 1);
    assert_eq!(
        control.cars[0].wall_substeps,
        0,
        "control centre={:?}, nodes={:?}",
        race::car_position(control.multi.sim(0)),
        control.multi.chassis_positions(0)
    );
    assert_eq!(control.cars[0].first_wall_t, None);
}
