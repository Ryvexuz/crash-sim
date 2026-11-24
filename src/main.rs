#[path = "cli/ai_race.rs"]
mod ai_race;
#[path = "cli/net_race.rs"]
mod net_race;
#[path = "cli/race.rs"]
mod race;
use race::{run_race, RaceMode};
#[path = "cli/pulse_run.rs"]
mod pulse_run;
use pulse_run::{pulse_params, run_pulse};

use crash_sim::{
    events_text, run_body_report, run_checked_with_contact, to_csv, to_csv_with_broken, Config,
};
use std::io::Write;
use std::process::exit;

use crash_sim::input::{self, DriveParams, DriveRun, Script};
use crash_sim::settle::{self, CarSetup, SettleParams};
use crash_sim::solver_implicit::{ImplicitEuler, M5_TOLERANCES};
use crash_sim::track::{format_seconds, LapEvent, LapTimer, Track};
use crash_sim::vec3::Vec3;
use crash_sim::{collide, pulse, reference, Body, Sim};

/// Five optional numeric flags parsed together.
type FiveOpt = (Option<f64>, Option<f64>, Option<f64>, Option<f64>, Option<f64>);

/// Write `text` and flush. The only path any stdout output takes, so a closed pipe or a full
/// disk is an error value, never a panic and never silently dropped.
fn write_all_flush(out: &mut impl Write, text: &str) -> std::io::Result<()> {
    out.write_all(text.as_bytes())?;
    out.flush()
}

/// The message printed to stderr when stdout cannot be written.
fn write_failure_message(e: &std::io::Error) -> String {
    format!("crash-sim: cannot write output: {e}")
}

/// Write to the locked stdout; on failure print one stderr line and exit 1.
fn emit(text: &str) {
    let stdout = std::io::stdout();
    let mut lock = stdout.lock();
    if let Err(e) = write_all_flush(&mut lock, text) {
        eprintln!("{}", write_failure_message(&e));
        exit(1);
    }
}

fn usage() -> ! {
    eprintln!("{}", ai_race::USAGE);
    eprintln!("       crash-sim --shell --data <dir> --shell-track <file> [--shell-track <file> ...] --shell-car <file> [--shell-car <file> ...] --today <YYYY-MM-DD>   (the game shell: docs/SHELL.md)");
    eprintln!("usage: crash-sim [--duration <s>] [--dt <s>] [--substeps <n>] [--drop-height <m>] [--gravity <m/s^2>] [--no-tilt] [--summary]");
    eprintln!("       crash-sim --body <file.bsn> [--duration <s>] [--dt <s>] [--substeps <n>] [--gravity <m/s^2>] [--summary] [--events]");
    eprintln!("       crash-sim ... --settle-report --settle-by <s> --settle-hold <s> --settle-speed <m/s> --energy-rise <J> --energy-gain <fraction>");
    eprintln!("       crash-sim --body <car.bsn> --input <file> [--duration <s>] [--slope-grip-frac <f>]   (per-step rows and a final line)");
    eprintln!("       crash-sim --body <car.bsn> (--input <file> | --live) --trajectory-v2   (trajectory v2 frames, every node, instead of the drive rows; for the renderer)");
    eprintln!("       crash-sim --body <car.bsn> --track <file.track> (--input <file> [--duration <s>] | --live [--trajectory-v2]) [--grid-slot <k> --grid-gap <m>] [--dt <s>] [--substeps <n>]   (a timed lap: docs/STAGE7.md, \"The race program\")");
    eprintln!("       crash-sim --body <car.bsn> --live [--dt <s>] [--substeps <n>] [--trajectory-v2]   (input lines on stdin, rows on stdout: docs/STAGE6.md, \"The live protocol\")");
    eprintln!("       crash-sim --body <car.bsn> [--input <file>] [--slope-grip-frac <f>] (--settle-report ... | --slope-report --slope-from <s> --slope-to <s>)");
    eprintln!("       crash-sim --body <car.bsn> --input <file> --drive-report --lap-min-heading <deg> --lap-min-path <m> --lap-close <m> --impact-min-kmh <km/h> --impact-max-kmh <km/h> [--settle-report ...]");
    eprintln!("       crash-sim --body <file.bsn> [--input <file> [--drive-report ...]] --settle-report ... --crumple-report --crumple-groups <from>,<to> --crumple-min <m> --crumple-max <m> --crumple-drift <m>");
    eprintln!("       crash-sim --body <file.bsn> --pulse-report <ref>.pulse.csv --duration <s> --dt <s> --substeps <n> --gravity <m/s^2> --pulse-sensor <group> --pulse-crush-groups <a>,<b> --pulse-cfc-shape <cfc> --pulse-cfc-int <cfc> --pulse-window <s> --pulse-threshold <m/s^2> --pulse-crush-tail <s> --setup-mass-tol <f> --setup-speed-tol <f> --margin-dv <f> --margin-peak <f> --margin-peak-time <s> --margin-duration <f> --margin-olc <f> --margin-static-crush <f> [--pulse-csv <path>] [--timing] [--solver node-beam|m5]   (docs/STAGE6-M4-M5.md, \"The pulse report\")");
    exit(2)
}

/// The flags of `--pulse-report`, as given. Every one but `csv`, `timing` and `solver` is
/// required (no default in code); `pulse_params` checks that and the values.
#[derive(Default)]
struct PulseFlags {
    report: Option<String>,
    csv: Option<String>,
    timing: bool,
    solver: Option<String>,
    sensor: Option<String>,
    crush: Option<(String, String)>,
    cfc_shape: Option<f64>,
    cfc_int: Option<f64>,
    window: Option<f64>,
    threshold: Option<f64>,
    tail: Option<f64>,
    mass_tol: Option<f64>,
    speed_tol: Option<f64>,
    margin_dv: Option<f64>,
    margin_peak: Option<f64>,
    margin_peak_time: Option<f64>,
    margin_duration: Option<f64>,
    margin_olc: Option<f64>,
    margin_static: Option<f64>,
}

impl PulseFlags {
    /// True if any pulse-report flag was given (so `--pulse-report` is then required).
    fn any(&self) -> bool {
        self.report.is_some()
            || self.csv.is_some()
            || self.timing
            || self.solver.is_some()
            || self.sensor.is_some()
            || self.crush.is_some()
            || [
                self.cfc_shape,
                self.cfc_int,
                self.window,
                self.threshold,
                self.tail,
                self.mass_tol,
                self.speed_tol,
                self.margin_dv,
                self.margin_peak,
                self.margin_peak_time,
                self.margin_duration,
                self.margin_olc,
                self.margin_static,
            ]
            .iter()
            .any(Option::is_some)
    }
}

/// The next argument as text, or a usage error naming the flag.
fn text_arg(args: &mut impl Iterator<Item = String>, flag: &str) -> String {
    match args.next() {
        Some(v) => v,
        None => {
            eprintln!("{flag} needs a value");
            usage()
        }
    }
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

/// The crumple thresholds from their flags. No defaults: the groups and the three thresholds
/// are all required, so a missing one is a usage error.
fn crumple_params(
    groups: Option<(String, String)>,
    min: Option<f64>,
    max: Option<f64>,
    drift: Option<f64>,
) -> collide::CrumpleParams {
    let (Some((from, to)), Some(min), Some(max), Some(drift)) = (groups, min, max, drift) else {
        eprintln!("--crumple-report needs --crumple-groups, --crumple-min, --crumple-max and --crumple-drift");
        usage()
    };
    collide::CrumpleParams {
        from,
        to,
        min,
        max,
        drift,
    }
}

fn main() {
    let ai_args: Vec<String> = std::env::args().skip(1).collect();
    if ai_args.iter().any(|a| a == "--shell") {
        exit(crash_sim::shell::run(
            &ai_args,
            &mut std::io::stdin().lock(),
            &mut std::io::stdout().lock(),
            &mut std::io::stderr(),
        ));
    }
    if ai_args.iter().any(|a| a == "--ai") {
        exit(ai_race::run_ai_cli(&ai_args));
    }
    if net_race::wants_reference(&ai_args) {
        exit(net_race::run_reference_cli(&ai_args));
    }
    let mut cfg = Config::default();
    let mut summary = false;
    let mut events = false;
    let mut settle_report = false;
    let mut body_path: Option<String> = None;
    let mut crumple_report = false;
    let mut crumple_groups: Option<(String, String)> = None;
    let (mut cmin, mut cmax, mut cdrift): (Option<f64>, Option<f64>, Option<f64>) =
        (None, None, None);
    let mut cube_only = false;
    let mut input_path: Option<String> = None;
    let mut live = false;
    let mut track_path: Option<String> = None;
    let (mut grid_slot, mut grid_gap) = (None, None);
    let mut trajectory_v2 = false;
    let mut duration_given = false;
    let mut drive_report = false;
    let (mut lap_heading, mut lap_path, mut lap_close): (Option<f64>, Option<f64>, Option<f64>) =
        (None, None, None);
    let (mut imp_min, mut imp_max): (Option<f64>, Option<f64>) = (None, None);
    let mut slope_grip: Option<f64> = None;
    let mut slope_report = false;
    let (mut slope_from, mut slope_to): (Option<f64>, Option<f64>) = (None, None);
    let (mut by, mut hold, mut speed, mut rise, mut gain): FiveOpt = (None, None, None, None, None);
    let mut pf = PulseFlags::default();
    let (mut dt_given, mut substeps_given, mut gravity_given) = (false, false, false);
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--duration" => {
                duration_given = true;
                cfg.duration = num(&mut args, "--duration")
            }
            "--dt" => {
                dt_given = true;
                cfg.dt = num(&mut args, "--dt")
            }
            "--substeps" => {
                substeps_given = true;
                cfg.substeps = num(&mut args, "--substeps")
            }
            "--pulse-report" => pf.report = Some(text_arg(&mut args, "--pulse-report")),
            "--pulse-csv" => pf.csv = Some(text_arg(&mut args, "--pulse-csv")),
            "--timing" => pf.timing = true,
            "--solver" => pf.solver = Some(text_arg(&mut args, "--solver")),
            "--pulse-sensor" => pf.sensor = Some(text_arg(&mut args, "--pulse-sensor")),
            "--pulse-crush-groups" => {
                let v = text_arg(&mut args, "--pulse-crush-groups");
                match v.split_once(',') {
                    Some((a, b)) if !a.is_empty() && !b.is_empty() && !b.contains(',') => {
                        pf.crush = Some((a.to_string(), b.to_string()))
                    }
                    _ => {
                        eprintln!("--pulse-crush-groups needs <a>,<b>");
                        usage()
                    }
                }
            }
            "--pulse-cfc-shape" => pf.cfc_shape = Some(num(&mut args, "--pulse-cfc-shape")),
            "--pulse-cfc-int" => pf.cfc_int = Some(num(&mut args, "--pulse-cfc-int")),
            "--pulse-window" => pf.window = Some(num(&mut args, "--pulse-window")),
            "--pulse-threshold" => pf.threshold = Some(num(&mut args, "--pulse-threshold")),
            "--pulse-crush-tail" => pf.tail = Some(num(&mut args, "--pulse-crush-tail")),
            "--setup-mass-tol" => pf.mass_tol = Some(num(&mut args, "--setup-mass-tol")),
            "--setup-speed-tol" => pf.speed_tol = Some(num(&mut args, "--setup-speed-tol")),
            "--margin-dv" => pf.margin_dv = Some(num(&mut args, "--margin-dv")),
            "--margin-peak" => pf.margin_peak = Some(num(&mut args, "--margin-peak")),
            "--margin-peak-time" => {
                pf.margin_peak_time = Some(num(&mut args, "--margin-peak-time"))
            }
            "--margin-duration" => pf.margin_duration = Some(num(&mut args, "--margin-duration")),
            "--margin-olc" => pf.margin_olc = Some(num(&mut args, "--margin-olc")),
            "--margin-static-crush" => {
                pf.margin_static = Some(num(&mut args, "--margin-static-crush"))
            }
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
            "--input" => match args.next() {
                Some(p) => input_path = Some(p),
                None => {
                    eprintln!("--input needs a file path");
                    usage()
                }
            },
            "--live" => live = true,
            "--track" => track_path = Some(text_arg(&mut args, "--track")),
            "--grid-slot" => grid_slot = Some(num::<usize>(&mut args, "--grid-slot")),
            "--grid-gap" => grid_gap = Some(num::<f64>(&mut args, "--grid-gap")),
            "--trajectory-v2" => trajectory_v2 = true,
            "--drive-report" => drive_report = true,
            "--lap-min-heading" => lap_heading = Some(num(&mut args, "--lap-min-heading")),
            "--lap-min-path" => lap_path = Some(num(&mut args, "--lap-min-path")),
            "--lap-close" => lap_close = Some(num(&mut args, "--lap-close")),
            "--impact-min-kmh" => imp_min = Some(num(&mut args, "--impact-min-kmh")),
            "--impact-max-kmh" => imp_max = Some(num(&mut args, "--impact-max-kmh")),
            "--slope-grip-frac" => slope_grip = Some(num(&mut args, "--slope-grip-frac")),
            "--slope-report" => slope_report = true,
            "--slope-from" => slope_from = Some(num(&mut args, "--slope-from")),
            "--slope-to" => slope_to = Some(num(&mut args, "--slope-to")),
            "--gravity" => {
                gravity_given = true;
                cfg.gravity = num(&mut args, "--gravity")
            }
            "--no-tilt" => {
                cube_only = true;
                cfg.tilt = false
            }
            "--summary" => summary = true,
            "--events" => events = true,
            "--settle-report" => settle_report = true,
            "--settle-by" => by = Some(num(&mut args, "--settle-by")),
            "--settle-hold" => hold = Some(num(&mut args, "--settle-hold")),
            "--settle-speed" => speed = Some(num(&mut args, "--settle-speed")),
            "--energy-rise" => rise = Some(num(&mut args, "--energy-rise")),
            "--energy-gain" => gain = Some(num(&mut args, "--energy-gain")),
            "--crumple-report" => crumple_report = true,
            "--crumple-groups" => {
                let v = args.next().unwrap_or_default();
                match v.split_once(',') {
                    Some((a, b)) if !a.is_empty() && !b.is_empty() && !b.contains(',') => {
                        crumple_groups = Some((a.to_string(), b.to_string()))
                    }
                    _ => {
                        eprintln!("--crumple-groups needs <from>,<to>");
                        usage()
                    }
                }
            }
            "--crumple-min" => cmin = Some(num(&mut args, "--crumple-min")),
            "--crumple-max" => cmax = Some(num(&mut args, "--crumple-max")),
            "--crumple-drift" => cdrift = Some(num(&mut args, "--crumple-drift")),
            _ => usage(),
        }
    }
    if (grid_slot.is_some() || grid_gap.is_some()) && track_path.is_none() {
        eprintln!("--grid-slot and --grid-gap need --track");
        usage()
    }
    let body = match &body_path {
        Some(path) => {
            if cube_only {
                eprintln!("--drop-height and --no-tilt set up the built-in cube and cannot be used with --body (the file gives the positions)");
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
    if pf.any() {
        // The pulse report has its own path and replaces the CSV: it is run alone.
        let Some(ref_path) = pf.report.clone() else {
            eprintln!("--pulse-csv, --timing, --solver and the --pulse-*, --setup-* and --margin-* flags need --pulse-report");
            usage()
        };
        let other = live
            || track_path.is_some()
            || input_path.is_some()
            || summary
            || events
            || settle_report
            || crumple_report
            || drive_report
            || trajectory_v2
            || cube_only
            || slope_grip.is_some()
            || slope_report
            || slope_from.is_some()
            || slope_to.is_some()
            || cmin.is_some()
            || cmax.is_some()
            || cdrift.is_some()
            || crumple_groups.is_some()
            || by.is_some()
            || hold.is_some()
            || speed.is_some()
            || rise.is_some()
            || gain.is_some()
            || lap_heading.is_some()
            || lap_path.is_some()
            || lap_close.is_some()
            || imp_min.is_some()
            || imp_max.is_some();
        if other {
            eprintln!("--pulse-report takes only --body, the run flags (--duration, --dt, --substeps, --gravity) and its own flags");
            usage()
        }
        let Some(b) = body.as_ref() else {
            eprintln!("--pulse-report needs --body");
            usage()
        };
        if !(duration_given && dt_given && substeps_given && gravity_given) {
            eprintln!(
                "--pulse-report needs --duration, --dt, --substeps and --gravity (no defaults)"
            );
            usage()
        }
        let params = match pulse_params(&pf, ref_path) {
            Ok(p) => p,
            Err(e) => {
                eprintln!("crash-sim: {e}");
                usage()
            }
        };
        exit(run_pulse(&cfg, b, &params));
    }
    if let Some(track_path) = track_path {
        // A timed lap (docs/STAGE7.md, "The race program"): its own path, with the lap timer.
        let other = summary
            || events
            || settle_report
            || crumple_report
            || drive_report
            || cube_only
            || slope_report
            || slope_grip.is_some()
            || slope_from.is_some()
            || slope_to.is_some()
            || cmin.is_some()
            || cmax.is_some()
            || cdrift.is_some()
            || crumple_groups.is_some()
            || by.is_some()
            || hold.is_some()
            || speed.is_some()
            || rise.is_some()
            || gain.is_some()
            || lap_heading.is_some()
            || lap_path.is_some()
            || lap_close.is_some()
            || imp_min.is_some()
            || imp_max.is_some();
        if other {
            eprintln!("--track takes only --body, --input (or --live), --duration, --dt, --substeps, --gravity and, with --live, --trajectory-v2");
            usage()
        }
        let grid = match (grid_slot, grid_gap) {
            (Some(_), None) => {
                eprintln!("--grid-slot needs --grid-gap");
                usage()
            }
            (None, Some(_)) => {
                eprintln!("--grid-gap in a single-car run needs --grid-slot");
                usage()
            }
            (Some(k), Some(_)) if !(1..=3).contains(&k) => {
                eprintln!("--grid-slot <k>: expected 1 to 3");
                usage()
            }
            (Some(_), Some(gap)) if !gap.is_finite() || gap <= 0.0 => {
                eprintln!("--grid-gap <m>: must be finite and above zero");
                usage()
            }
            pair => pair.0.zip(pair.1),
        };
        let Some(b) = body.as_ref() else {
            eprintln!("--track needs --body");
            usage()
        };
        let mode = if live {
            if input_path.is_some() || duration_given {
                eprintln!("--track --live takes the input from stdin and ends at end of input, so it cannot be used with --input or --duration");
                usage()
            }
            RaceMode::Live { v2: trajectory_v2 }
        } else {
            let Some(input) = input_path else {
                eprintln!("--track needs --input (or --live)");
                usage()
            };
            if trajectory_v2 {
                eprintln!("--trajectory-v2 with --track needs --live");
                usage()
            }
            RaceMode::File {
                input,
                duration_given,
            }
        };
        exit(run_race(&cfg, b, &track_path, mode, grid));
    }
    if live {
        // The live run has its own path: the input comes from stdin, the length from the input.
        let other = input_path.is_some()
            || duration_given
            || summary
            || events
            || settle_report
            || crumple_report
            || drive_report
            || slope_report
            || slope_grip.is_some()
            || slope_from.is_some()
            || slope_to.is_some()
            || cmin.is_some()
            || cmax.is_some()
            || cdrift.is_some()
            || crumple_groups.is_some()
            || by.is_some()
            || hold.is_some()
            || speed.is_some()
            || rise.is_some()
            || gain.is_some()
            || lap_heading.is_some()
            || lap_path.is_some()
            || lap_close.is_some()
            || imp_min.is_some()
            || imp_max.is_some();
        if other {
            eprintln!("--live takes only --body, --dt, --substeps and --trajectory-v2 (the input comes from stdin and the run ends at end of input)");
            usage()
        }
        let Some(b) = body.as_ref().filter(|b| !b.wheels.is_empty()) else {
            eprintln!("--live needs --body with a body that has wheels");
            usage()
        };
        exit(run_live(&cfg, b, trajectory_v2));
    }
    let any_crumple =
        crumple_groups.is_some() || cmin.is_some() || cmax.is_some() || cdrift.is_some();
    if (crumple_report || any_crumple) && !settle_report {
        eprintln!("--crumple-report and its flags need --settle-report (and its five thresholds)");
        usage()
    }
    if any_crumple && !crumple_report {
        eprintln!("--crumple-groups, --crumple-min, --crumple-max and --crumple-drift need --crumple-report");
        usage()
    }
    // The car flags: they set up a car (Sim::from_car), so they need a body with wheels, and
    // something to print: a report, or the per-step rows of an --input drive. A plain run still
    // refuses a body with wheels.
    let has_wheels = body.as_ref().is_some_and(|b| !b.wheels.is_empty());
    let any_lap = lap_heading.is_some()
        || lap_path.is_some()
        || lap_close.is_some()
        || imp_min.is_some()
        || imp_max.is_some();
    let car_flags = input_path.is_some()
        || slope_grip.is_some()
        || slope_report
        || slope_from.is_some()
        || slope_to.is_some()
        || drive_report
        || any_lap;
    if trajectory_v2 {
        // The frames replace the per-step rows of an --input drive, so nothing else may print.
        if input_path.is_none() {
            eprintln!("--trajectory-v2 needs --input (or --live)");
            usage()
        }
        if drive_report
            || any_lap
            || settle_report
            || crumple_report
            || slope_report
            || slope_grip.is_some()
        {
            eprintln!("--trajectory-v2 replaces the per-step rows, so it cannot be used with a report or a slope flag");
            usage()
        }
    }
    if car_flags {
        if !has_wheels {
            eprintln!("--input, --slope-grip-frac, --slope-report and --drive-report need --body with a body that has wheels");
            usage()
        }
        if input_path.is_none() && !settle_report && !slope_report {
            eprintln!("--slope-grip-frac needs --settle-report or --slope-report, or an --input drive to run");
            usage()
        }
        if crumple_report && (slope_grip.is_some() || slope_report) {
            eprintln!("--crumple-report cannot be used with --slope-grip-frac or --slope-report");
            usage()
        }
        if summary {
            eprintln!("--summary cannot be used with --input, --slope-grip-frac, --slope-report or --drive-report");
            usage()
        }
        if events {
            eprintln!("--events cannot be used with --input, --slope-grip-frac, --slope-report or --drive-report");
            usage()
        }
        if (drive_report || any_lap) && input_path.is_none() {
            eprintln!("--drive-report and its flags need --input");
            usage()
        }
        if any_lap && !drive_report {
            eprintln!("--lap-min-heading, --lap-min-path, --lap-close, --impact-min-kmh and --impact-max-kmh need --drive-report");
            usage()
        }
    }
    if slope_report != (slope_from.is_some() && slope_to.is_some())
        || (slope_from.is_some() != slope_to.is_some())
    {
        eprintln!("--slope-report needs --slope-from and --slope-to, and they need --slope-report");
        usage()
    }
    if slope_report && slope_grip.is_none() {
        eprintln!("--slope-report needs --slope-grip-frac");
        usage()
    }
    let script = match (&input_path, car_flags) {
        (Some(path), true) => {
            let text = match std::fs::read_to_string(path) {
                Ok(t) => t,
                Err(e) => {
                    eprintln!("crash-sim: --input {path}: {e}");
                    exit(2)
                }
            };
            match Script::parse(&text) {
                Ok(s) => Some(s),
                Err(e) => {
                    eprintln!("crash-sim: {path}: {e}");
                    exit(2)
                }
            }
        }
        _ => None,
    };
    let any_threshold =
        by.is_some() || hold.is_some() || speed.is_some() || rise.is_some() || gain.is_some();
    if car_flags {
        // A car run: its own path, which prints every report it was asked for.
        let Some(b) = &body else { usage() };
        let settle_params = if settle_report {
            // No defaults: all five thresholds are required.
            let (Some(by), Some(hold), Some(speed), Some(rise), Some(gain)) =
                (by, hold, speed, rise, gain)
            else {
                eprintln!("--settle-report needs --settle-by, --settle-hold, --settle-speed, --energy-rise and --energy-gain");
                usage()
            };
            let params = SettleParams {
                by,
                hold,
                speed,
                energy_rise: rise,
                energy_gain: gain,
            };
            if let Err(e) = params.validate() {
                eprintln!("{e}");
                usage()
            }
            Some(params)
        } else {
            if any_threshold {
                eprintln!("--settle-by, --settle-hold, --settle-speed, --energy-rise and --energy-gain need --settle-report");
                usage()
            }
            None
        };
        let drive_params = if drive_report {
            // No defaults: all five thresholds are required.
            let (
                Some(lap_min_heading),
                Some(lap_min_path),
                Some(lap_close),
                Some(impact_min_kmh),
                Some(impact_max_kmh),
            ) = (lap_heading, lap_path, lap_close, imp_min, imp_max)
            else {
                eprintln!("--drive-report needs --lap-min-heading, --lap-min-path, --lap-close, --impact-min-kmh and --impact-max-kmh");
                usage()
            };
            let p = DriveParams {
                lap_min_heading,
                lap_min_path,
                lap_close,
                impact_min_kmh,
                impact_max_kmh,
            };
            if let Err(e) = p.validate() {
                eprintln!("{e}");
                usage()
            }
            Some(p)
        } else {
            None
        };
        let crumple = if crumple_report {
            Some(crumple_params(crumple_groups.clone(), cmin, cmax, cdrift))
        } else {
            None
        };
        let setup = CarSetup {
            script,
            until_script_end: !duration_given,
            grip_frac: slope_grip,
        };
        let args = CarArgs {
            cfg: &cfg,
            body: b,
            setup,
            settle: settle_params,
            slope: slope_from.zip(slope_to).filter(|_| slope_report),
            drive: drive_params,
            crumple,
            trajectory_v2,
        };
        exit(run_car(&args));
    }
    if settle_report {
        // No defaults: all five thresholds are required, and --summary is a different output.
        let (Some(by), Some(hold), Some(speed), Some(rise), Some(gain)) =
            (by, hold, speed, rise, gain)
        else {
            eprintln!("--settle-report needs --settle-by, --settle-hold, --settle-speed, --energy-rise and --energy-gain");
            usage()
        };
        if summary {
            eprintln!("--settle-report and --summary cannot be used together");
            usage()
        }
        let params = SettleParams {
            by,
            hold,
            speed,
            energy_rise: rise,
            energy_gain: gain,
        };
        if let Err(e) = params.validate() {
            eprintln!("{e}");
            usage()
        }
        if crumple_report {
            let cp = crumple_params(crumple_groups, cmin, cmax, cdrift);
            // A body with wheels is refused here (its run aborts, as `measure_crumple` does not
            // simulate wheels); a car's crumple is measured on its --input run (`run_car`).
            let (report, crumple) =
                match collide::measure_crumple(&cfg, body.as_ref(), &params, &cp) {
                    Ok(r) => r,
                    Err(e) => {
                        eprintln!("{e}");
                        usage()
                    }
                };
            let mut out = report.render();
            if let Some(c) = &crumple {
                out.push_str(&c.render());
            }
            emit(&out);
            if !report.is_ok() || crumple.as_ref().is_some_and(|c| !c.is_ok()) {
                exit(1);
            }
            return;
        }
        let report = match &body {
            Some(b) if has_wheels => settle::measure_car(&cfg, b, &CarSetup::default(), &params),
            _ => settle::measure_body(&cfg, body.as_ref(), &params),
        };
        emit(&report.render());
        if !report.is_ok() {
            exit(1);
        }
        return;
    }
    if any_threshold {
        eprintln!("--settle-by, --settle-hold, --settle-speed, --energy-rise and --energy-gain need --settle-report");
        usage()
    }
    let (samples, first_contact, breaks, result) = match &body {
        Some(b) => {
            let r = run_body_report(&cfg, b);
            (r.samples, r.first_contact_s, r.breaks, r.result)
        }
        None => {
            let (s, f, r) = run_checked_with_contact(&cfg);
            (s, f, Vec::new(), r)
        }
    };
    // The run's own error is reported first, before any output write, so a failed write
    // (which exits 1 at once) can never hide it.
    if let Err(e) = &result {
        eprintln!("crash-sim: {e}");
    }
    if summary {
        if result.is_ok() {
            if let (Some(last), true) = (samples.last(), !samples.is_empty()) {
                // From the substep loop, not from the output samples, so a contact
                // that starts and ends between two samples is still reported.
                let contact = match first_contact {
                    Some(t) => format!("{t:.4}"),
                    None => "none".to_string(),
                };
                let peak = samples
                    .iter()
                    .map(|s| s.max_beam_strain)
                    .fold(f64::MIN, f64::max);
                emit(&format!(
                    "first_contact_s={contact},peak_strain={peak:.6},final_center_y={:.4}\n",
                    last.center_y
                ));
            }
        }
    } else if !samples.is_empty() {
        // The broken_beams column is for --body runs only: the cube's CSV stays as it was.
        emit(&if body.is_some() {
            to_csv_with_broken(&samples)
        } else {
            to_csv(&samples)
        });
    }
    if events && !breaks.is_empty() {
        emit(&events_text(&breaks));
    }
    if result.is_err() {
        exit(1);
    }
}

/// The live run (docs/STAGE6.md, "The live protocol"): lines on stdin, rows on stdout flushed per
/// output step. Exit 0 at end of input or when stdout is closed (quietly, no message), 1 when the
/// sim aborts or another write fails, 2 for a refused input (a bad line, the body does not suit).
fn run_live(cfg: &Config, body: &Body, v2: bool) -> i32 {
    let stdin = std::io::stdin();
    let mut input = stdin.lock();
    let stdout = std::io::stdout();
    let mut out = stdout.lock();
    let mut emit = |text: &str| write_all_flush(&mut out, text);
    match input::live(cfg, body, v2, &mut input, &mut emit) {
        Ok(()) => 0,
        Err(input::LiveError::Input(e)) => {
            eprintln!("crash-sim: --live: {e}");
            2
        }
        Err(input::LiveError::Abort(e)) => {
            eprintln!("crash-sim: {e}");
            1
        }
        Err(input::LiveError::Output(e)) if e.kind() == std::io::ErrorKind::BrokenPipe => 0,
        Err(input::LiveError::Output(e)) => {
            eprintln!("{}", write_failure_message(&e));
            1
        }
    }
}

/// Everything a car run needs, already validated.
struct CarArgs<'a> {
    cfg: &'a Config,
    body: &'a Body,
    setup: CarSetup,
    settle: Option<SettleParams>,
    /// `--slope-from`, `--slope-to` when `--slope-report` is on.
    slope: Option<(f64, f64)>,
    drive: Option<DriveParams>,
    /// `--crumple-report` (only with `--settle-report`): measured on the settle run.
    crumple: Option<collide::CrumpleParams>,
    /// `--trajectory-v2`: the run is written as trajectory v2 frames instead of the drive rows.
    trajectory_v2: bool,
}

/// A car run: the drive (when there is a script), then the settle (and crumple) and slope
/// reports, printed in this order: the drive report (which holds the `final` line), the settle
/// report, the crumple report, the slope report; a run with a script and no drive report ends
/// with the `final` line; a run with a script and no report prints the per-step rows then the
/// `final` line. Returns the exit code: 0 ok, 1 a FAIL or an abort, 2 a refused input (the script
/// does not suit the car, the body has no `front` or `rear` group or no crumple group).
fn run_car(a: &CarArgs) -> i32 {
    let mut out = String::new();
    let mut failed = false;
    // The drive run feeds the rows, the drive report and the `final` line. It runs first, so a
    // refusal is reported before anything is printed.
    let rows_mode = a.settle.is_none() && a.slope.is_none() && a.drive.is_none();
    // The settle and slope reports need no heading: a body without `front` and `rear` groups
    // still runs them (and prints no `final` line); the rows and the drive report need them.
    let heading = input::has_heading_groups(a.body);
    // A slope run has its own length, so a script that ends at 0 only costs it the `final` line.
    let has_length = settle::run_substeps(a.cfg, &a.setup).is_ok();
    let needs_run =
        a.setup.script.is_some() && has_length && (a.drive.is_some() || rows_mode || heading);
    if a.setup.script.is_some() && (a.settle.is_some() || a.drive.is_some() || rows_mode) {
        if let Err(e) = settle::run_substeps(a.cfg, &a.setup) {
            eprintln!("crash-sim: {e}");
            return 2;
        }
    }
    let mut frames = String::new();
    let run: Option<DriveRun> = match &a.setup.script {
        Some(_) if needs_run => match input::drive_frames(
            a.cfg,
            a.body,
            &a.setup,
            a.drive.map_or(f64::INFINITY, |p| p.lap_min_heading),
            a.trajectory_v2.then_some(&mut frames),
        ) {
            Ok(r) => Some(r),
            Err(e) => {
                eprintln!("crash-sim: {e}");
                return 2;
            }
        },
        _ => None,
    };
    let reports = a.settle.is_some() || a.slope.is_some() || a.drive.is_some();
    if let Some(r) = &run {
        if let Some(e) = &r.abort {
            if a.drive.is_some() {
                out.push_str(&format!("drive: FAIL {e}\n"));
            } else {
                eprintln!("crash-sim: {e}");
            }
            failed = true;
        }
        if a.trajectory_v2 {
            out.push_str(&frames);
        } else if !reports {
            out.push_str(&input::render_rows(&r.rows));
        }
        if let (Some(dp), None) = (&a.drive, &r.abort) {
            out.push_str(&r.measure.render(dp, &r.final_state));
            failed |= r.measure.verdict(dp).is_err();
        }
    }
    if let Some(params) = &a.settle {
        match &a.crumple {
            Some(cp) => match collide::measure_crumple_car(a.cfg, a.body, &a.setup, params, cp) {
                Ok((report, crumple)) => {
                    out.push_str(&report.render());
                    failed |= !report.is_ok();
                    if let Some(c) = &crumple {
                        out.push_str(&c.render());
                        failed |= !c.is_ok();
                    }
                }
                Err(e) => {
                    // A usage problem (a missing group, bad thresholds), found before any output.
                    eprintln!("crash-sim: {e}");
                    return 2;
                }
            },
            None => {
                let report = settle::measure_car(a.cfg, a.body, &a.setup, params);
                out.push_str(&report.render());
                failed |= !report.is_ok();
            }
        }
    }
    if let Some((from, to)) = a.slope {
        match settle::measure_slope(a.cfg, a.body, &a.setup, from, to) {
            Ok(r) => out.push_str(&r.render()),
            Err(e) => {
                eprintln!("crash-sim: {e}");
                failed = true;
            }
        }
    }
    if let Some(r) = &run {
        if a.drive.is_none() && r.abort.is_none() && !a.trajectory_v2 {
            out.push_str(&r.final_state.render());
        }
    }
    emit(&out);
    i32::from(failed)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{Error, ErrorKind};

    struct FailWrite;
    impl Write for FailWrite {
        fn write(&mut self, _: &[u8]) -> std::io::Result<usize> {
            Err(Error::new(ErrorKind::BrokenPipe, "pipe closed"))
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    struct FailFlush(Vec<u8>);
    impl Write for FailFlush {
        fn write(&mut self, b: &[u8]) -> std::io::Result<usize> {
            self.0.extend_from_slice(b);
            Ok(b.len())
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Err(Error::other("disk full"))
        }
    }

    #[test]
    fn write_error_is_returned() {
        let e = write_all_flush(&mut FailWrite, "x\n").unwrap_err();
        assert_eq!(e.kind(), ErrorKind::BrokenPipe);
    }

    #[test]
    fn flush_error_is_returned() {
        let mut w = FailFlush(Vec::new());
        let e = write_all_flush(&mut w, "x\n").unwrap_err();
        assert_eq!(e.to_string(), "disk full");
        assert_eq!(w.0, b"x\n");
    }

    #[test]
    fn success_writes_exact_bytes() {
        let mut v: Vec<u8> = Vec::new();
        write_all_flush(&mut v, "a,b\n1,2\n").unwrap();
        assert_eq!(v, b"a,b\n1,2\n");
    }

    #[test]
    fn failure_message_names_the_write() {
        let m = write_failure_message(&Error::new(ErrorKind::BrokenPipe, "pipe closed"));
        assert_eq!(m, "crash-sim: cannot write output: pipe closed");
    }
}
