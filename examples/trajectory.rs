//! Writes a trajectory for the renderer to stdout.
//!
//! `trajectory [stride]` is the built-in cube, v1. `trajectory --body <file.bsn> [--stride <n>] [--duration <s>]`
//! is any body, v2 (see `crash_sim::trajectory`). Every text comes from the library, so the tests check the
//! same code this prints.

use crash_sim::trajectory::{body_text, cube_text};
use crash_sim::Config;

fn fail(msg: &str) -> ! {
    eprintln!("error: {msg}");
    std::process::exit(2);
}

fn whole(a: &str, what: &str) -> u64 {
    match a.parse::<u64>() {
        Ok(n) if n >= 1 => n,
        _ => fail(&format!("{what} must be a whole number >= 1, got '{a}'")),
    }
}

fn main() {
    let mut cfg = Config::default();
    let mut stride: u64 = 1;
    let mut body_path: Option<String> = None;
    let mut args = std::env::args().skip(1);
    let mut first = true;
    while let Some(a) = args.next() {
        match a.as_str() {
            "--body" => {
                body_path = Some(
                    args.next()
                        .unwrap_or_else(|| fail("--body needs a file path")),
                )
            }
            "--stride" => {
                stride = whole(
                    &args
                        .next()
                        .unwrap_or_else(|| fail("--stride needs a value")),
                    "stride",
                )
            }
            "--duration" => {
                let v = args
                    .next()
                    .unwrap_or_else(|| fail("--duration needs a value"));
                cfg.duration = match v.parse::<f64>() {
                    Ok(d) if d.is_finite() && d > 0.0 => d,
                    _ => fail(&format!("duration must be a positive number, got '{v}'")),
                };
            }
            // The original form: a bare first argument is the stride.
            _ if first && !a.starts_with("--") => stride = whole(&a, "stride"),
            _ => fail(&format!("unknown argument '{a}'")),
        }
        first = false;
    }
    let text = match &body_path {
        None => cube_text(&cfg, stride),
        Some(path) => {
            let source = std::fs::read_to_string(path)
                .unwrap_or_else(|e| fail(&format!("--body {path}: {e}")));
            let body =
                crash_sim::format::parse(&source).unwrap_or_else(|e| fail(&format!("{path}: {e}")));
            body_text(&cfg, &body, stride)
        }
    };
    match text {
        Ok(t) => print!("{t}"),
        Err(e) => fail(&e),
    }
}
