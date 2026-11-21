//! Per-phase timing of the 3.6 run (scenarios/pad_and_wall.bsn with scenarios/pad_lap.input,
//! 72 s). Std only. Measure only: it changes no simulated number. See docs/PERF_HOTSPOTS.md.
//!
//! usage: profile_step [--runs <n>]   (default 5; the table shows the median of the runs per phase)
//!
//! A "step" here is one physics substep (h = dt / substeps = 1 ms): the unit `input::drive` loops
//! over (it runs a sim with dt = h and substeps = 1, so one `advance` is one substep).
//!
//! `Sim::integrate`'s car path is repeated here from public pieces (`Car::pre_step`,
//! `SymplecticEuler::step_with_forces`, `Car::post_step`, `collide::apply`) so each can be timed.
//! That copy is only trusted because each run ends by comparing its final node positions and
//! velocities, bit for bit, with a second run that uses the real `Sim::advance`; a difference
//! fails the program. `Sim::apply_breaks` is private and is not called: the scenario's beams have
//! no break strain (checked at start), so it would never break a beam.
use crash_sim::collide;
use crash_sim::input::Script;
use crash_sim::settle::{self, CarSetup};
use crash_sim::solver::{Solver, State, SymplecticEuler};
use crash_sim::vec3::Vec3;
use crash_sim::vehicle::Substep;
use crash_sim::{format, Config, Sim};
use std::process::exit;
use std::time::{Duration, Instant};

const BODY: &str = "scenarios/pad_and_wall.bsn";
const INPUT: &str = "scenarios/pad_lap.input";
const DURATION_S: f64 = 72.0;

/// The timed phases, in the order a substep runs them.
const PHASES: [&str; 8] = [
    "script.apply (controls for this substep)",
    "extra-force setup (alloc + gravity part)",
    "Car::pre_step (wheels, suspension, tyres)",
    "solver step (beam forces + node integration)",
    "Car::post_step (wheel update, axis constraint)",
    "collide::apply (ground and wall contacts)",
    "Sim::apply_yield (plastic flow)",
    "first-contact test + Sim::check",
];

fn fail(msg: &str) -> ! {
    eprintln!("profile_step: {msg}");
    eprintln!("usage: profile_step [--runs <n>]");
    exit(2)
}

fn median(sorted: &[f64]) -> f64 {
    let n = sorted.len();
    if n % 2 == 1 {
        sorted[n / 2]
    } else {
        (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0
    }
}

struct Setup {
    body: format::Body,
    setup: CarSetup,
    /// The sim's config as `input::drive` builds it: dt = h, one substep per step.
    one: Config,
    steps: u64,
}

fn load() -> Setup {
    let btext = std::fs::read_to_string(BODY).unwrap_or_else(|e| fail(&format!("{BODY}: {e}")));
    let body = format::parse(&btext).unwrap_or_else(|e| fail(&format!("{BODY}: {e}")));
    let itext = std::fs::read_to_string(INPUT).unwrap_or_else(|e| fail(&format!("{INPUT}: {e}")));
    let script = Script::parse(&itext).unwrap_or_else(|e| fail(&format!("{INPUT}: {e}")));
    let cfg = Config {
        duration: DURATION_S,
        ..Config::default()
    };
    // `--input` with `--duration`: the run is `duration` long, not the script's length.
    let setup = CarSetup {
        script: Some(script),
        until_script_end: false,
        grip_frac: None,
    };
    let steps = settle::run_substeps(&cfg, &setup).unwrap_or_else(|e| fail(&e));
    let mut one = cfg.clone();
    one.dt = cfg.dt / cfg.substeps as f64;
    one.substeps = 1;
    Setup {
        body,
        setup,
        one,
        steps,
    }
}

fn new_sim(s: &Setup) -> Sim {
    let sim = settle::car_sim(s.one.clone(), &s.body, &s.setup).unwrap_or_else(|e| fail(&e));
    if sim.break_strain.iter().any(|b| b.is_some()) || sim.gravity_tilt() != 0.0 {
        fail("the scenario has a break strain or a gravity tilt: this profile does not cover it");
    }
    sim
}

/// The untimed reference run: the real `Sim::advance`, as `input::drive` calls it.
fn run_real(s: &Setup) -> (Sim, Duration) {
    let mut sim = new_sim(s);
    let script = s.setup.script.as_ref().expect("script set in load");
    let h = s.one.dt;
    let tol = h * 1e-6;
    let t0 = Instant::now();
    for i in 1..=s.steps {
        let t_start = (i - 1) as f64 * h;
        script
            .apply(&mut sim, t_start, tol)
            .unwrap_or_else(|e| fail(&e));
        sim.advance();
        if let Err(e) = sim.check() {
            fail(&format!("real run unstable at step {i}: {e}"));
        }
    }
    (sim, t0.elapsed())
}

/// The timed run: the same steps with `Sim::integrate`'s car path split into its public pieces.
fn run_timed(s: &Setup) -> (Sim, [Duration; PHASES.len()], Duration) {
    let mut sim = new_sim(s);
    let script = s.setup.script.as_ref().expect("script set in load");
    let h = s.one.dt;
    let tol = h * 1e-6;
    let mut solver = SymplecticEuler::default();
    let mut acc = [Duration::ZERO; PHASES.len()];
    let t0 = Instant::now();
    for i in 1..=s.steps {
        let t_start = (i - 1) as f64 * h;
        let mut t = Instant::now();
        let mut lap = |k: usize, acc: &mut [Duration; PHASES.len()]| {
            let now = Instant::now();
            acc[k] += now - t;
            t = now;
        };
        script
            .apply(&mut sim, t_start, tol)
            .unwrap_or_else(|e| fail(&e));
        lap(0, &mut acc);
        // Sim::integrate, car path (no tilt, no broken beams, checked in new_sim).
        let gv: Vec3 = sim.gravity_vector();
        let mut extra: Vec<Vec3> = sim
            .mass
            .iter()
            .map(|&m| Vec3::new(m * gv.x, 0.0, 0.0))
            .collect();
        lap(1, &mut acc);
        {
            let car = sim.car.as_mut().expect("car sim");
            let step = Substep { gravity: gv, h };
            car.pre_step(&sim.pos, &sim.vel, &sim.mass, &sim.planes, step, &mut extra);
        }
        lap(2, &mut acc);
        let state = State {
            pos: &mut sim.pos,
            vel: &mut sim.vel,
            beams: &sim.beams,
            mass: &sim.mass,
            gravity: -gv.y,
        };
        solver.step_with_forces(state, &extra, h);
        lap(3, &mut acc);
        {
            let car = sim.car.as_mut().expect("car sim");
            car.post_step(&mut sim.pos, &mut sim.vel, &sim.mass, &sim.planes, h);
        }
        lap(4, &mut acc);
        collide::apply(&mut sim.pos, &mut sim.vel, &sim.planes, h);
        lap(5, &mut acc);
        sim.apply_yield();
        lap(6, &mut acc);
        // advance's first-contact test (its result is not needed here) and drive's check.
        let touching = collide::any_at_or_behind(&sim.pos, &sim.planes);
        if let Err(e) = sim.check() {
            fail(&format!("timed run unstable at step {i}: {e}"));
        }
        std::hint::black_box(touching);
        lap(7, &mut acc);
    }
    (sim, acc, t0.elapsed())
}

fn same_bits(a: &[Vec3], b: &[Vec3]) -> bool {
    a.len() == b.len()
        && a.iter().zip(b).all(|(p, q)| {
            p.x.to_bits() == q.x.to_bits()
                && p.y.to_bits() == q.y.to_bits()
                && p.z.to_bits() == q.z.to_bits()
        })
}

fn ms(d: Duration) -> f64 {
    d.as_secs_f64() * 1000.0
}

fn main() {
    let mut runs: usize = 5;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--runs" => {
                runs = match args.next().and_then(|v| v.parse::<usize>().ok()) {
                    Some(n) if n >= 1 => n,
                    _ => fail("--runs needs an integer >= 1"),
                }
            }
            other => fail(&format!("unknown argument '{other}'")),
        }
    }
    let s = load();
    println!(
        "profile_step: {BODY} + {INPUT}, {DURATION_S} s = {} steps of h = {} s, {runs} runs",
        s.steps, s.one.dt
    );
    let mut per_phase: Vec<Vec<f64>> = vec![Vec::new(); PHASES.len()];
    let mut timed_total: Vec<f64> = Vec::new();
    let mut real_total: Vec<f64> = Vec::new();
    for r in 0..runs {
        let (real, real_d) = run_real(&s);
        let (timed, acc, timed_d) = run_timed(&s);
        if !same_bits(&real.pos, &timed.pos) || !same_bits(&real.vel, &timed.vel) {
            fail("the timed run's final state differs from Sim::advance's: the phase split is not the real step");
        }
        for (k, d) in acc.iter().enumerate() {
            per_phase[k].push(ms(*d));
        }
        timed_total.push(ms(timed_d));
        real_total.push(ms(real_d));
        println!(
            "run {}: real advance loop {:.1} ms, timed loop {:.1} ms (final state identical)",
            r + 1,
            ms(real_d),
            ms(timed_d)
        );
    }
    let med = |v: &mut Vec<f64>| {
        v.sort_by(|a, b| a.total_cmp(b));
        median(v)
    };
    let meds: Vec<f64> = per_phase.iter_mut().map(med).collect();
    let phase_sum: f64 = meds.iter().sum();
    let steps = s.steps as f64;
    println!();
    println!("median of {runs} runs per phase");
    println!(
        "{:<48} {:>10} {:>12} {:>7}",
        "phase", "total ms", "ms per step", "share"
    );
    for (name, m) in PHASES.iter().zip(&meds) {
        println!(
            "{:<48} {:>10.2} {:>12.6} {:>6.1}%",
            name,
            m,
            m / steps,
            100.0 * m / phase_sum
        );
    }
    println!(
        "{:<48} {:>10.2} {:>12.6} {:>6.1}%",
        "sum of phases (step time)",
        phase_sum,
        phase_sum / steps,
        100.0
    );
    let tt = med(&mut timed_total);
    let rt = med(&mut real_total);
    println!();
    println!(
        "timed loop wall time (median) {tt:.2} ms = {:.6} ms per step (includes the timer calls)",
        tt / steps
    );
    println!(
        "real Sim::advance loop (median) {rt:.2} ms = {:.6} ms per step",
        rt / steps
    );
    println!();
    println!("needs instrumentation (follow-up): inside the solver step, beam forces vs node integration;");
    println!("inside Car::pre_step, the wheel loop vs tyre_force vs add_torque; inside post_step, the axis");
    println!(
        "constraint vs patch update; Sim::apply_breaks (private; no beam here has a break strain);"
    );
    println!("input.rs's per-step measurement (centroid, heading, wall_touched; private).");
}
