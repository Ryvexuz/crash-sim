//! Tests of the settle report (docs/STAGE6.md, milestone 1).
//!
//! INTERIM: src/lib.rs is outside the claim of this task, so the module is compiled into this
//! test from its file. Once lib.rs has `pub mod settle;`, replace the next block with
//! `use crash_sim::settle::{...};`.

#[allow(unused_imports)]
use crash_sim::{max_strain, vec3, Beam, Config, Sim};
#[path = "../src/settle.rs"]
mod settle;

use crash_sim::vec3::Vec3;
use settle::{measure, measure_with, total_energy, SettleParams};
use std::process::{Command, Output};

/// The thresholds of docs/STAGE6.md check 1.3.
fn p13() -> SettleParams {
    SettleParams { by: 6.0, hold: 4.0, speed: 0.001, energy_rise: 0.001, energy_gain: 1e-5 }
}

fn cfg10() -> Config {
    Config { duration: 10.0, ..Config::default() }
}

#[test]
fn two_node_energy_matches_hand_value() {
    // Node a: m = 2 kg at (0, 1, 0), v = (3, 0, 0).  Node b: m = 3 kg at (0, 3, 0), v = (0, -2, 0).
    // One beam a-b: length 2 m, rest 1.5 m, k = 100 N/m. g = 9.81.
    // kinetic   1/2*2*3^2 + 1/2*3*2^2        = 9 + 6        = 15
    // potential 2*9.81*1 + 3*9.81*3          = 19.62 + 88.29 = 107.91
    // elastic   1/2*100*(2 - 1.5)^2          = 12.5
    // total                                                  = 135.41
    let pos = [Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 3.0, 0.0)];
    let vel = [Vec3::new(3.0, 0.0, 0.0), Vec3::new(0.0, -2.0, 0.0)];
    let beams = [Beam { a: 0, b: 1, rest: 1.5, stiffness: 100.0, damping: 7.0 }];
    let mass = [2.0, 3.0];
    let e = total_energy(&pos, &vel, &beams, &mass, 9.81);
    assert!((e - 135.41).abs() < 1e-9, "energy {e}");
    // Each term on its own, so a wrong term cannot be hidden by another.
    assert!((total_energy(&pos, &vel, &beams, &mass, 0.0) - 27.5).abs() < 1e-9);
    let still = [Vec3::ZERO, Vec3::ZERO];
    assert!((total_energy(&pos, &still, &[], &mass, 9.81) - 107.91).abs() < 1e-9);
}

#[test]
fn default_cube_settles_with_the_doc_numbers() {
    let r = measure(&cfg10(), &p13());
    assert!(r.is_ok(), "{}", r.render());
    assert!(r.peak_strain <= 0.1 && r.peak_strain > 0.0);
    let last = r.last_impact_s.expect("the cube bounces");
    assert!(last > 0.5 && last < 4.0, "last impact {last}");
    let settled = r.settled_from_s.expect("it settles");
    assert!(settled > last && settled < 6.0, "settled from {settled}");
    assert!((r.final_center_y - 0.4998705625).abs() <= 1.294375e-6, "{}", r.final_center_y);
}

#[test]
fn planted_energy_injection_fails_settle() {
    // Control: no injection, speed rule loose so only the energy rules could speak.
    let loose = SettleParams { speed: 100.0, ..p13() };
    assert!(measure(&cfg10(), &loose).is_ok());
    // Plant: at substep 7000 (t = 7 s) the cube is lying still; give one node 0.5 m/s upward.
    let plant = |sim: &mut Sim, i: u64| {
        if i == 7000 {
            sim.vel[7].y += 0.5;
        }
    };
    let r = measure_with(&cfg10(), &loose, plant);
    let msg = r.verdict.clone().expect_err("injection must fail");
    assert!(msg.starts_with("energy rise"), "{msg}");
    assert!(r.energy_rise_after_by_j > 0.001);
    assert!(r.render().ends_with(&format!("settle: FAIL {msg}\n")));
    // With the real speed threshold the same injection is caught by the speed rule first.
    let r = measure_with(&cfg10(), &p13(), plant);
    assert!(r.verdict.unwrap_err().starts_with("speed"), "speed rule comes first");
    // Energy gained before T is caught by the gain rule (rule 4) when the rise rule is off.
    let early = |sim: &mut Sim, i: u64| {
        if i == 3000 {
            sim.vel[7].y += 20.0;
        }
    };
    let gainy = SettleParams { speed: 100.0, energy_rise: 1e9, ..p13() };
    let r = measure_with(&cfg10(), &gainy, early);
    assert!(r.verdict.unwrap_err().starts_with("energy gain"));
}

#[test]
fn non_finite_state_is_an_abort_not_a_success() {
    let nan = |sim: &mut Sim, i: u64| {
        if i == 500 {
            sim.pos[0].x = f64::NAN;
        }
    };
    let r = measure_with(&cfg10(), &p13(), nan);
    assert!(r.aborted && !r.is_ok());
    let text = r.render();
    assert!(text.starts_with("settle: FAIL unstable at t = 0.5000 s: "), "{text}");
    assert_eq!(text.lines().count(), 1);
}

#[test]
fn hold_rule_needs_the_run_to_last() {
    let r = measure(&cfg10(), &SettleParams { by: 6.0, hold: 4.5, ..p13() });
    assert!(r.verdict.unwrap_err().starts_with("hold"));
    // Exactly T + S is long enough (the float time of the last substep is not exact).
    assert!(measure(&cfg10(), &SettleParams { by: 6.0, hold: 4.0, ..p13() }).is_ok());
}

#[test]
fn bad_thresholds_are_refused() {
    for p in [
        SettleParams { speed: 0.0, ..p13() },
        SettleParams { by: f64::NAN, ..p13() },
        SettleParams { energy_gain: -1.0, ..p13() },
    ] {
        let r = measure(&cfg10(), &p);
        assert!(r.aborted && !r.is_ok());
    }
}

/// Static sag of the resting cube: edge/2 - (19/72) m g / k (docs/STAGE6.md, "Static sag").
fn static_center_y(g: f64) -> f64 {
    0.5 - (19.0 / 72.0) * 1.0 * g / 20_000.0
}

#[test]
fn settle_holds_across_the_invariants_grid() {
    for &drop in &[0.5, 1.0, 2.0, 4.0] {
        for &tilt in &[false, true] {
            for &g in &[9.81, 1.62] {
                let cfg = Config { duration: 20.0, drop_height: drop, tilt, gravity: g, ..Config::default() };
                let p = SettleParams { by: 16.0, hold: 4.0, speed: 0.001, energy_rise: 0.001, energy_gain: 1e-5 };
                let r = measure(&cfg, &p);
                let case = format!("drop {drop} tilt {tilt} g {g}");
                assert!(r.is_ok(), "{case}: {}", r.render());
                let sag = (19.0 / 72.0) * g / 20_000.0;
                let want = static_center_y(g);
                assert!((r.final_center_y - want).abs() <= 0.01 * sag, "{case}: center_y {} want {want}", r.final_center_y);
            }
        }
    }
}

fn run(args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_crash-sim")).args(args).output().unwrap()
}

const FLAGS_13: [&str; 11] = [
    "--settle-report", "--settle-by", "6", "--settle-hold", "4", "--settle-speed", "0.001", "--energy-rise", "0.001", "--energy-gain", "0.00001",
];

#[test]
fn cli_prints_the_report_and_exits_by_verdict() {
    let mut a = vec!["--duration", "10"];
    a.extend(FLAGS_13);
    let o = run(&a);
    assert_eq!(o.status.code(), Some(0));
    let out = String::from_utf8(o.stdout).unwrap();
    let keys: Vec<&str> = out.lines().map(|l| l.split(['=', ':']).next().unwrap()).collect();
    assert_eq!(
        keys,
        ["last_impact_s", "settled_from_s", "max_speed_after_settle_by", "energy_rise_after_settle_by_j", "max_energy_gain_frac", "peak_strain", "final_center_y", "settle"]
    );
    assert_eq!(out.lines().last().unwrap(), "settle: ok");

    // Check 1.4: still bouncing at 2 s -> exit 1 and a speed FAIL on the last line.
    let o = run(&["--duration", "10", "--settle-report", "--settle-by", "2", "--settle-hold", "8", "--settle-speed", "0.001", "--energy-rise", "0.001", "--energy-gain", "0.00001"]);
    assert_eq!(o.status.code(), Some(1));
    let out = String::from_utf8(o.stdout).unwrap();
    assert!(out.lines().last().unwrap().starts_with("settle: FAIL speed "), "{out}");
}

#[test]
fn cli_usage_errors_exit_2_with_empty_stdout() {
    // Each threshold missing in turn, and thresholds without --settle-report.
    for skip in 0..5 {
        let mut a: Vec<&str> = vec!["--settle-report"];
        let pairs = [("--settle-by", "6"), ("--settle-hold", "4"), ("--settle-speed", "0.001"), ("--energy-rise", "0.001"), ("--energy-gain", "0.00001")];
        for (i, (f, v)) in pairs.iter().enumerate() {
            if i != skip {
                a.push(f);
                a.push(v);
            }
        }
        let o = run(&a);
        assert_eq!(o.status.code(), Some(2), "missing {}", pairs[skip].0);
        assert!(o.stdout.is_empty());
    }
    let o = run(&["--settle-by", "6"]);
    assert_eq!(o.status.code(), Some(2));
    assert!(o.stdout.is_empty());
    let mut a = vec!["--summary"];
    a.extend(FLAGS_13);
    assert_eq!(run(&a).status.code(), Some(2));
}
