use crash_sim::{run_body_checked_with_contact, run_checked_with_contact, to_csv, Config};
use std::io::Write;
use std::process::exit;

use crash_sim::settle;

fn usage() -> ! {
    eprintln!("usage: crash-sim [--duration <s>] [--dt <s>] [--substeps <n>] [--drop-height <m>] [--gravity <m/s^2>] [--no-tilt] [--summary]");
    eprintln!("       crash-sim --body <file.bsn> [--duration <s>] [--dt <s>] [--substeps <n>] [--gravity <m/s^2>] [--summary]");
    eprintln!("       crash-sim ... --settle-report --settle-by <s> --settle-hold <s> --settle-speed <m/s> --energy-rise <J> --energy-gain <fraction>");
    exit(2)
}

fn num<T: std::str::FromStr>(args: &mut impl Iterator<Item = String>, flag: &str) -> T {
    match args.next().and_then(|v| v.parse().ok()) {
        Some(v) => v,
        None => {
            eprintln!("{flag} needs a numeric value");
            usage()
        }
    }
}

fn main() {
    let mut cfg = Config::default();
    let mut summary = false;
    let mut settle_report = false;
    let mut body_path: Option<String> = None;
    let mut cube_only = false;
    let (mut by, mut hold, mut speed, mut rise, mut gain): (Option<f64>, Option<f64>, Option<f64>, Option<f64>, Option<f64>) =
        (None, None, None, None, None);
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--duration" => cfg.duration = num(&mut args, "--duration"),
            "--dt" => cfg.dt = num(&mut args, "--dt"),
            "--substeps" => cfg.substeps = num(&mut args, "--substeps"),
            "--drop-height" => {
                cube_only = true;
                cfg.drop_height = num(&mut args, "--drop-height")
            }
            "--body" => match args.next() {
                Some(p) => body_path = Some(p),
                None => {
                    eprintln!("--body needs a file path");
                    usage()
                }
            },
            "--gravity" => cfg.gravity = num(&mut args, "--gravity"),
            "--no-tilt" => {
                cube_only = true;
                cfg.tilt = false
            }
            "--summary" => summary = true,
            "--settle-report" => settle_report = true,
            "--settle-by" => by = Some(num(&mut args, "--settle-by")),
            "--settle-hold" => hold = Some(num(&mut args, "--settle-hold")),
            "--settle-speed" => speed = Some(num(&mut args, "--settle-speed")),
            "--energy-rise" => rise = Some(num(&mut args, "--energy-rise")),
            "--energy-gain" => gain = Some(num(&mut args, "--energy-gain")),
            _ => usage(),
        }
    }
    let body = match &body_path {
        Some(path) => {
            if cube_only {
                eprintln!("--drop-height and --no-tilt set up the built-in cube and cannot be used with --body (the file gives the positions)");
                usage()
            }
            if settle_report {
                eprintln!("--settle-report does not work with --body yet (it builds the cube itself)");
                usage()
            }
            let text = match std::fs::read_to_string(path) {
                Ok(t) => t,
                Err(e) => {
                    eprintln!("crash-sim: --body {path}: {e}");
                    exit(2)
                }
            };
            match crash_sim::format::parse(&text) {
                Ok(b) => Some(b),
                Err(e) => {
                    eprintln!("crash-sim: {path}: {e}");
                    exit(2)
                }
            }
        }
        None => None,
    };
    let any_threshold = by.is_some() || hold.is_some() || speed.is_some() || rise.is_some() || gain.is_some();
    if settle_report {
        // No defaults: all five thresholds are required, and --summary is a different output.
        let (Some(by), Some(hold), Some(speed), Some(rise), Some(gain)) = (by, hold, speed, rise, gain) else {
            eprintln!("--settle-report needs --settle-by, --settle-hold, --settle-speed, --energy-rise and --energy-gain");
            usage()
        };
        if summary {
            eprintln!("--settle-report and --summary cannot be used together");
            usage()
        }
        let params = settle::SettleParams { by, hold, speed, energy_rise: rise, energy_gain: gain };
        if let Err(e) = params.validate() {
            eprintln!("{e}");
            usage()
        }
        let report = settle::measure(&cfg, &params);
        let _ = std::io::stdout().write_all(report.render().as_bytes());
        if !report.is_ok() {
            exit(1);
        }
        return;
    }
    if any_threshold {
        eprintln!("--settle-by, --settle-hold, --settle-speed, --energy-rise and --energy-gain need --settle-report");
        usage()
    }
    let (samples, first_contact, result) = match &body {
        Some(b) => run_body_checked_with_contact(&cfg, b),
        None => run_checked_with_contact(&cfg),
    };
    if summary {
        if result.is_ok() {
            if let (Some(last), true) = (samples.last(), !samples.is_empty()) {
                // From the substep loop, not from the output samples, so a contact
                // that starts and ends between two samples is still reported.
                let contact = match first_contact {
                    Some(t) => format!("{t:.4}"),
                    None => "none".to_string(),
                };
                let peak = samples.iter().map(|s| s.max_beam_strain).fold(f64::MIN, f64::max);
                println!(
                    "first_contact_s={contact},peak_strain={peak:.6},final_center_y={:.4}",
                    last.center_y
                );
            }
        }
    } else if !samples.is_empty() {
        let _ = std::io::stdout().write_all(to_csv(&samples).as_bytes());
    }
    if let Err(e) = result {
        eprintln!("crash-sim: {e}");
        exit(1);
    }
}
