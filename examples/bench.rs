//! Timing harness for the sim's hot paths. Std only. See docs/BENCH.md.
//!
//! usage: bench [--runs <n>] [--case box|cube|settle|car|break|m5]
//! One JSON line per case: name, runs, median_ms, min_ms, max_ms, mad_ms. The `break` and `m5`
//! cases then print one more line, the count they assert is above 0 (`beams broken`, `plastic tets`).
use crash_sim::collide::{self, Plane};
use crash_sim::format::{Body, Node, Solid, Tet};
use crash_sim::input::{self, Script};
use crash_sim::settle::{self, CarSetup, SettleParams};
use crash_sim::solver::{Solver, State};
use crash_sim::solver_m5::{self, SolverM5};
use crash_sim::vec3::Vec3;
use crash_sim::{format, Config, Sim};
use std::process::exit;
use std::time::Instant;

/// Solver steps (output steps of `Sim::advance`) per timed run of the box and cube cases.
const STEPS: u64 = 500;
/// Seconds of the scripted lap the car case runs: 10 s took about 50 ms per run, over the 30 ms
/// budget of a gate case, so it runs the first 5 s.
const CAR_SECONDS: f64 = 5.0;
/// `break` case: the car (front face at x = 2 m) drives straight at a wall `BREAK_WALL_X` m along
/// +x for `BREAK_SECONDS`, with a `break_strain` of `BREAK_STRAIN` on every material so the crash
/// tears beams. The script is full throttle, no brake, no steer, from t = 0.
const BREAK_WALL_X: f64 = 5.0;
const BREAK_SECONDS: f64 = 4.0;
const BREAK_STRAIN: f64 = 0.025;
const BREAK_SCRIPT: &str = "0 1 0 0\n";
/// `m5` case: a block of `M5_CELLS` cells (x, y, z; 6 tets each, 270 in all, under the solver's
/// 300-tet limit) of `M5_CELL_M` m with nodes of `M5_NODE_KG`, thrown at the floor (y = 0) at `M5_IMPACT_MPS` for `M5_STEPS` substeps of `M5_H` s. The solid crushes at
/// `M5_PLATEAU_PA` after an elastic strain of `M5_YIELD_STRAIN`.
const M5_CELLS: [usize; 3] = [5, 3, 3];
const M5_CELL_M: f64 = 0.1;
const M5_NODE_KG: f64 = 0.5;
const M5_IMPACT_MPS: f64 = 20.0;
const M5_STEPS: u64 = 500;
const M5_H: f64 = 1e-5;
const M5_PLATEAU_PA: f64 = 2e6;
const M5_YIELD_STRAIN: f64 = 0.02;
const CASES: [&str; 6] = ["box", "cube", "settle", "car", "break", "m5"];

fn fail(msg: &str) -> ! {
    eprintln!("bench: {msg}");
    eprintln!("usage: bench [--runs <n>] [--case box|cube|settle|car|break|m5]");
    exit(2)
}

/// Time `f` once, in milliseconds.
fn time_ms(f: &mut dyn FnMut()) -> f64 {
    let t = Instant::now();
    f();
    t.elapsed().as_secs_f64() * 1000.0
}

fn median(sorted: &[f64]) -> f64 {
    let n = sorted.len();
    if n % 2 == 1 {
        sorted[n / 2]
    } else {
        (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0
    }
}

fn run_case(name: &str, runs: usize, f: &mut dyn FnMut()) {
    let mut ms: Vec<f64> = (0..runs).map(|_| time_ms(f)).collect();
    ms.sort_by(|a, b| a.total_cmp(b));
    let med = median(&ms);
    let mut dev: Vec<f64> = ms.iter().map(|x| (x - med).abs()).collect();
    dev.sort_by(|a, b| a.total_cmp(b));
    let mad = median(&dev);
    println!(
        "{{\"name\":\"{name}\",\"runs\":{runs},\"median_ms\":{med:.3},\"min_ms\":{:.3},\"max_ms\":{:.3},\"mad_ms\":{mad:.3}}}",
        ms[0],
        ms[runs - 1]
    );
}

/// Load a scenario file (path relative to the crate root, where cargo runs examples).
fn load_body(path: &str) -> format::Body {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| fail(&format!("{path}: {e}")));
    format::parse(&text).unwrap_or_else(|e| fail(&format!("{path}: {e}")))
}

fn steps_case(name: &str, path: &str, runs: usize) {
    let body = load_body(path);
    let cfg = Config::default();
    let mut f = || {
        let mut sim =
            Sim::from_body(cfg.clone(), &body).unwrap_or_else(|e| fail(&format!("{path}: {e}")));
        for _ in 0..STEPS {
            sim.advance();
        }
        // A blown-up run would time garbage: fail instead of reporting it.
        if let Err(e) = sim.check() {
            fail(&format!("{name}: {e}"));
        }
    };
    run_case(name, runs, &mut f);
}

fn settle_case(runs: usize) {
    // The documented `crash-sim --duration 10 --settle-report ...` command (docs/STAGE6.md).
    let cfg = Config {
        duration: 10.0,
        ..Config::default()
    };
    let params = SettleParams {
        by: 6.0,
        hold: 4.0,
        speed: 0.001,
        energy_rise: 0.001,
        energy_gain: 0.00001,
    };
    let mut f = || {
        let report = settle::measure(&cfg, &params);
        if report.aborted {
            fail(&format!("settle: run aborted: {:?}", report.verdict));
        }
    };
    run_case("settle", runs, &mut f);
}

/// The first `CAR_SECONDS` of the pad lap, driven the way `crash-sim --input` drives it
/// (`input::drive` on a `CarSetup` with the parsed script and an explicit duration). Paths come
/// from the crate root, not the current directory.
fn car_case(runs: usize) {
    let root = env!("CARGO_MANIFEST_DIR");
    let body = load_body(&format!("{root}/scenarios/pad_and_wall.bsn"));
    let input_path = format!("{root}/scenarios/pad_lap.input");
    let text = std::fs::read_to_string(&input_path)
        .unwrap_or_else(|e| fail(&format!("{input_path}: {e}")));
    let script = Script::parse(&text).unwrap_or_else(|e| fail(&format!("{input_path}: {e}")));
    let cfg = Config {
        duration: CAR_SECONDS,
        ..Config::default()
    };
    let setup = CarSetup {
        script: Some(script),
        until_script_end: false,
        grip_frac: None,
    };
    let mut f = || {
        let run = input::drive(&cfg, &body, &setup, f64::INFINITY)
            .unwrap_or_else(|e| fail(&format!("car: {e}")));
        // A blown-up run would time garbage: fail instead of reporting it.
        if let Some(e) = run.abort {
            fail(&format!("car: {e}"));
        }
    };
    run_case("car", runs, &mut f);
}

/// The car body driven straight into a nearby wall with a `break_strain` on every material. The
/// run is built the way `input::drive` builds it (`settle::car_sim` on `dt = h` with one substep
/// per step, the script applied before each) but keeps the sim, to count the broken beams.
fn break_case(runs: usize) {
    let root = env!("CARGO_MANIFEST_DIR");
    let mut body = load_body(&format!("{root}/scenarios/pad_and_wall.bsn"));
    for m in &mut body.materials {
        m.break_strain = Some(BREAK_STRAIN);
    }
    if body.planes.len() != 1 {
        fail("break: pad_and_wall.bsn must have exactly one plane (the wall)");
    }
    body.planes[0].point = Vec3::new(BREAK_WALL_X, 0.0, 0.0);
    let script = Script::parse(BREAK_SCRIPT).unwrap_or_else(|e| fail(&format!("break: {e}")));
    let cfg = Config {
        duration: BREAK_SECONDS,
        ..Config::default()
    };
    let setup = CarSetup {
        script: Some(script.clone()),
        until_script_end: false,
        grip_frac: None,
    };
    let h = cfg.dt / cfg.substeps as f64;
    let total = settle::run_substeps(&cfg, &setup).unwrap_or_else(|e| fail(&format!("break: {e}")));
    let mut one = cfg.clone();
    one.dt = h;
    one.substeps = 1;
    let tol = h * 1e-6;
    let mut broken = 0usize;
    let mut f = || {
        let mut sim = settle::car_sim(one.clone(), &body, &setup)
            .unwrap_or_else(|e| fail(&format!("break: {e}")));
        for i in 1..=total {
            script
                .apply(&mut sim, (i - 1) as f64 * h, tol)
                .unwrap_or_else(|e| fail(&format!("break: {e}")));
            sim.advance();
            // A blown-up run would time garbage: fail instead of reporting it.
            if let Err(e) = sim.check() {
                fail(&format!("break: {e}"));
            }
        }
        broken = sim.breaks.len();
    };
    run_case("break", runs, &mut f);
    println!("break: beams broken = {broken}");
    if broken == 0 {
        fail("break: no beam broke (the case must break beams)");
    }
}

/// The block of the `m5` case: `M5_CELLS` cells, each cut into 6 tets by `solver_m5::cut_cell`,
/// one node per grid point, resting 1 cm above the floor.
fn m5_body() -> Body {
    let [cx, cy, cz] = M5_CELLS;
    let id =
        |ix: usize, iy: usize, iz: usize| (ix * (cy + 1) * (cz + 1) + iy * (cz + 1) + iz) as u32;
    let mut nodes = Vec::new();
    for ix in 0..=cx {
        for iy in 0..=cy {
            for iz in 0..=cz {
                nodes.push(Node {
                    id: id(ix, iy, iz),
                    pos: Vec3::new(
                        ix as f64 * M5_CELL_M,
                        0.01 + iy as f64 * M5_CELL_M,
                        iz as f64 * M5_CELL_M,
                    ),
                    mass: M5_NODE_KG,
                });
            }
        }
    }
    let mut tets = Vec::new();
    for ix in 0..cx {
        for iy in 0..cy {
            for iz in 0..cz {
                let mut c = [0u32; 8];
                for (k, ck) in c.iter_mut().enumerate() {
                    *ck = id(ix + (k & 1), iy + ((k >> 1) & 1), iz + ((k >> 2) & 1));
                }
                for t in solver_m5::cut_cell(c) {
                    tets.push(Tet {
                        nodes: t,
                        solid: "foam".into(),
                    });
                }
            }
        }
    }
    Body {
        materials: Vec::new(),
        solids: vec![Solid {
            name: "foam".into(),
            plateau_stress: M5_PLATEAU_PA,
            yield_strain: M5_YIELD_STRAIN,
            damping_ratio: 0.05,
        }],
        nodes,
        beams: Vec::new(),
        tets,
        groups: Vec::new(),
        velocities: Vec::new(),
        wheels: Vec::new(),
        suspensions: Vec::new(),
        tyres: Vec::new(),
        controls: None,
        planes: Vec::new(),
    }
}

/// `SolverM5` stepped `M5_STEPS` substeps on a block thrown at the floor, with `collide::apply`
/// after every step (the order `Sim::integrate` uses). `Sim` does not expose its solver, so the
/// solver is driven directly. Ends by counting the tets with plastic strain.
fn m5_case(runs: usize) {
    let body = m5_body();
    let built = body.build();
    let floor = [Plane {
        point: Vec3::ZERO,
        normal: Vec3::new(0.0, 1.0, 0.0),
    }];
    let mut plastic = 0usize;
    let mut f = || {
        let mut solver =
            SolverM5::from_body(&body, M5_H).unwrap_or_else(|e| fail(&format!("m5: {e}")));
        let mut pos = built.pos.clone();
        let mut vel = vec![Vec3::new(0.0, -M5_IMPACT_MPS, 0.0); pos.len()];
        for _ in 0..M5_STEPS {
            solver.step(
                State {
                    pos: &mut pos,
                    vel: &mut vel,
                    beams: &built.beams,
                    mass: &built.mass,
                    gravity: 9.81,
                },
                M5_H,
            );
            collide::apply(&mut pos, &mut vel, &floor, M5_H);
            // A blown-up run would time garbage: fail instead of reporting it.
            if let Some(e) = solver.check() {
                fail(&format!("m5: {e}"));
            }
        }
        plastic = (0..solver.tet_count())
            .filter(|&t| solver.plastic_strain(t) > 0.0)
            .count();
    };
    run_case("m5", runs, &mut f);
    println!("m5: plastic tets = {plastic}");
    if plastic == 0 {
        fail("m5: no tet yielded (the case must yield tets)");
    }
}

fn main() {
    let mut runs: usize = 10;
    let mut only: Option<String> = None;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--runs" => {
                runs = match args.next().and_then(|v| v.parse::<usize>().ok()) {
                    Some(n) if n >= 1 => n,
                    _ => fail("--runs needs an integer >= 1"),
                }
            }
            "--case" => match args.next() {
                Some(c) if CASES.contains(&c.as_str()) => only = Some(c),
                Some(c) => fail(&format!(
                    "unknown case '{c}' (cases: box, cube, settle, car, break, m5)"
                )),
                None => fail("--case needs a name (box, cube, settle, car, break, m5)"),
            },
            other => fail(&format!("unknown argument '{other}'")),
        }
    }
    let want = |c: &str| only.as_deref().is_none_or(|o| o == c);
    if want("box") {
        steps_case("box", "scenarios/box.bsn", runs);
    }
    if want("cube") {
        steps_case("cube", "scenarios/cube.bsn", runs);
    }
    if want("settle") {
        settle_case(runs);
    }
    if want("car") {
        car_case(runs);
    }
    if want("break") {
        break_case(runs);
    }
    if want("m5") {
        m5_case(runs);
    }
}
