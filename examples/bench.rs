//! Timing harness for the sim's hot paths. Std only. See docs/BENCH.md.
//!
//! usage: bench [--runs <n>] [--case box|cube|settle]
//! One JSON line per case: name, runs, median_ms, min_ms, max_ms, mad_ms.
use crash_sim::settle::{self, SettleParams};
use crash_sim::{format, Config, Sim};
use std::process::exit;
use std::time::Instant;

/// Solver steps (output steps of `Sim::advance`) per timed run of the box and cube cases.
const STEPS: u64 = 500;
const CASES: [&str; 3] = ["box", "cube", "settle"];

fn fail(msg: &str) -> ! {
    eprintln!("bench: {msg}");
    eprintln!("usage: bench [--runs <n>] [--case box|cube|settle]");
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
                Some(c) => fail(&format!("unknown case '{c}' (cases: box, cube, settle)")),
                None => fail("--case needs a name (box, cube, settle)"),
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
}
