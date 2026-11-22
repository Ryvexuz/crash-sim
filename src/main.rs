use crash_sim::{
    events_text, run_body_report, run_checked_with_contact, to_csv, to_csv_with_broken, Config,
};
use std::io::Write;
use std::process::exit;

use crash_sim::input::{self, DriveParams, DriveRun, Script};
use crash_sim::settle::{self, CarSetup, SettleParams};
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
    eprintln!("usage: crash-sim [--duration <s>] [--dt <s>] [--substeps <n>] [--drop-height <m>] [--gravity <m/s^2>] [--no-tilt] [--summary]");
    eprintln!("       crash-sim --body <file.bsn> [--duration <s>] [--dt <s>] [--substeps <n>] [--gravity <m/s^2>] [--summary] [--events]");
    eprintln!("       crash-sim ... --settle-report --settle-by <s> --settle-hold <s> --settle-speed <m/s> --energy-rise <J> --energy-gain <fraction>");
    eprintln!("       crash-sim --body <car.bsn> --input <file> [--duration <s>] [--slope-grip-frac <f>]   (per-step rows and a final line)");
    eprintln!("       crash-sim --body <car.bsn> (--input <file> | --live) --trajectory-v2   (trajectory v2 frames, every node, instead of the drive rows; for the renderer)");
    eprintln!("       crash-sim --body <car.bsn> --track <file.track> (--input <file> [--duration <s>] | --live [--trajectory-v2]) [--dt <s>] [--substeps <n>]   (a timed lap: docs/STAGE7.md, \"The race program\")");
    eprintln!("       crash-sim --body <car.bsn> --live [--dt <s>] [--substeps <n>] [--trajectory-v2]   (input lines on stdin, rows on stdout: docs/STAGE6.md, \"The live protocol\")");
    eprintln!("       crash-sim --body <car.bsn> [--input <file>] [--slope-grip-frac <f>] (--settle-report ... | --slope-report --slope-from <s> --slope-to <s>)");
    eprintln!("       crash-sim --body <car.bsn> --input <file> --drive-report --lap-min-heading <deg> --lap-min-path <m> --lap-close <m> --impact-min-kmh <km/h> --impact-max-kmh <km/h> [--settle-report ...]");
    eprintln!("       crash-sim --body <file.bsn> [--input <file> [--drive-report ...]] --settle-report ... --crumple-report --crumple-groups <from>,<to> --crumple-min <m> --crumple-max <m> --crumple-drift <m>");
    eprintln!("       crash-sim --body <file.bsn> --pulse-report <ref>.pulse.csv --duration <s> --dt <s> --substeps <n> --gravity <m/s^2> --pulse-sensor <group> --pulse-crush-groups <a>,<b> --pulse-cfc-shape <cfc> --pulse-cfc-int <cfc> --pulse-window <s> --pulse-threshold <m/s^2> --pulse-crush-tail <s> --setup-mass-tol <f> --setup-speed-tol <f> --margin-dv <f> --margin-peak <f> --margin-peak-time <s> --margin-duration <f> --margin-olc <f> --margin-static-crush <f> [--pulse-csv <path>] [--timing] [--solver node-beam]   (docs/STAGE6-M4-M5.md, \"The pulse report\")");
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
        exit(run_race(&cfg, b, &track_path, mode));
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

// ---------------------------------------------------------------------------------------
// The timed lap (docs/STAGE7.md, "The race program")

/// How a `--track` run gets its controls.
enum RaceMode {
    /// `--input <file>`: the run ends at the script's last line, or at `--duration`.
    File { input: String, duration_given: bool },
    /// `--live`: input lines on stdin; `v2` writes trajectory v2 frames, else the drive rows.
    Live { v2: bool },
}

/// What one lap has measured so far (the lap timer decides validity and time; these are the
/// numbers it does not keep).
#[derive(Clone, Copy)]
struct LapStats {
    /// Path of the car position since the lap's start crossing (m).
    path: f64,
    /// Largest horizontal speed of the car position over a substep of the lap (m/s).
    max_speed: f64,
}

/// The `front` and `rear` node indices a heading is read from.
struct HeadingGroups {
    front: Vec<usize>,
    rear: Vec<usize>,
}

impl HeadingGroups {
    fn of(body: &Body) -> Option<HeadingGroups> {
        let group = |name: &str| -> Option<Vec<usize>> {
            let g = body.groups.iter().find(|g| g.name == name)?;
            g.nodes.iter().map(|id| body.node_index(*id)).collect()
        };
        Some(HeadingGroups {
            front: group("front")?,
            rear: group("rear")?,
        })
    }

    fn mean(pos: &[Vec3], ix: &[usize]) -> Vec3 {
        let mut c = Vec3::ZERO;
        for &i in ix {
            c += pos[i];
        }
        c * (1.0 / ix.len() as f64)
    }

    /// Heading (degrees): the ground-plane direction from the `rear` to the `front` centroid.
    fn heading_deg(&self, sim: &Sim) -> f64 {
        let d = Self::mean(&sim.pos, &self.front) - Self::mean(&sim.pos, &self.rear);
        d.z.atan2(d.x).to_degrees()
    }

    /// The `final` state: the centroid of all nodes (docs/STAGE6.md, nb-40's rule).
    fn final_state(&self, sim: &Sim, t: f64) -> input::FinalState {
        let all: Vec<usize> = (0..sim.pos.len()).collect();
        let c = Self::mean(&sim.pos, &all);
        input::FinalState {
            t,
            x: c.x,
            z: c.z,
            heading_deg: self.heading_deg(sim),
        }
    }

    /// The drive row at time `t` (as `input::drive` prints it).
    fn row(&self, sim: &Sim, t: f64) -> input::DriveRow {
        let all: Vec<usize> = (0..sim.pos.len()).collect();
        let m: f64 = sim.mass.iter().sum();
        let mut v = Vec3::ZERO;
        for (vi, mi) in sim.vel.iter().zip(&sim.mass) {
            v += *vi * *mi;
        }
        input::DriveRow {
            t,
            speed: (v * (1.0 / m)).length(),
            pos: Self::mean(&sim.pos, &all),
            heading_deg: self.heading_deg(sim),
        }
    }
}

/// The car position the lap timer follows: the mass-weighted mean of the body's nodes (the
/// wheels are not nodes), docs/STAGE7.md "Car position".
fn chassis_centroid(sim: &Sim) -> Vec3 {
    let m: f64 = sim.mass.iter().sum();
    let mut c = Vec3::ZERO;
    for (p, mi) in sim.pos.iter().zip(&sim.mass) {
        c += *p * *mi;
    }
    c * (1.0 / m)
}

/// `body` turned about the vertical axis so that its +x points along the track's start tangent,
/// and moved so that its chassis centroid is on the grid point; heights are unchanged. The start
/// velocities turn with it. Refused when the body has wall planes: they would stay where the
/// file put them, and a track has no walls in milestone 1.
fn place_on_grid(body: &Body, track: &Track) -> Result<Body, String> {
    if !body.planes.is_empty() {
        return Err("--track: the body has wall planes, which a track run cannot place".into());
    }
    let t = track.tangent(track.start);
    let g = track.grid_point();
    let turn = |x: f64, z: f64| (x * t[0] - z * t[1], x * t[1] + z * t[0]);
    let built = body.build();
    let total: f64 = built.mass.iter().sum();
    let (mut cx, mut cz) = (0.0, 0.0);
    for (p, m) in built.pos.iter().zip(&built.mass) {
        cx += p.x * m / total;
        cz += p.z * m / total;
    }
    let (rx, rz) = turn(cx, cz);
    let (dx, dz) = (g[0] - rx, g[1] - rz);
    let mut placed = body.clone();
    for n in &mut placed.nodes {
        let (x, z) = turn(n.pos.x, n.pos.z);
        n.pos = Vec3::new(x + dx, n.pos.y, z + dz);
    }
    for v in &mut placed.velocities {
        let (x, z) = turn(v.velocity.x, v.velocity.z);
        v.velocity = Vec3::new(x, v.velocity.y, z);
    }
    Ok(placed)
}

/// A sim and its lap timer, advanced one substep at a time.
struct Race {
    sim: Sim,
    timer: LapTimer,
    h: f64,
    /// Substeps done.
    i: u64,
    prev: Vec3,
    stats: Option<LapStats>,
}

impl Race {
    fn new(sim: Sim, track: &Track, h: f64) -> Race {
        let prev = chassis_centroid(&sim);
        Race {
            sim,
            timer: LapTimer::new(track),
            h,
            i: 0,
            prev,
            stats: None,
        }
    }

    /// One substep, with the controls already set. Returns the race lines it caused (`start`,
    /// `checkpoint`, `lap`), or the reason the run aborts.
    fn step(&mut self) -> Result<String, (f64, String)> {
        let t0 = self.i as f64 * self.h;
        self.i += 1;
        let t1 = self.i as f64 * self.h;
        self.sim.advance();
        if let Err(e) = self.sim.check() {
            return Err((t1, e));
        }
        let pos = chassis_centroid(&self.sim);
        let events = self
            .timer
            .update(self.prev, pos, t0, t1)
            .map_err(|e| (t1, e.to_string()))?;
        let len = ((pos.x - self.prev.x).powi(2) + (pos.z - self.prev.z).powi(2)).sqrt();
        let speed = len / self.h;
        self.prev = pos;
        let mut out = String::new();
        let mut crossed = false;
        for ev in events {
            // The fraction of this substep before the crossing.
            let frac = |t: f64| ((t - t0) / (t1 - t0)).clamp(0.0, 1.0);
            match ev {
                LapEvent::Start { t } => {
                    out.push_str(&format!("start t={t:.4}\n"));
                    self.stats = Some(LapStats {
                        path: len * (1.0 - frac(t)),
                        max_speed: speed,
                    });
                    crossed = true;
                }
                LapEvent::Checkpoint { lap, k, t } => {
                    out.push_str(&format!("checkpoint lap={lap} k={k} t={t:.4}\n"));
                }
                LapEvent::Lap(rec) => {
                    let f = frac(rec.end_t);
                    let done = self.stats.take().unwrap_or(LapStats {
                        path: 0.0,
                        max_speed: 0.0,
                    });
                    out.push_str(&format!(
                        "lap n={} start_t={:.4} end_t={:.4} time_s={} valid={} reason={} path_m={:.2} max_speed_kmh={:.1} off_track_s={:.3}\n",
                        rec.n,
                        rec.start_t,
                        rec.end_t,
                        format_seconds(rec.time_ms),
                        if rec.valid() { "yes" } else { "no" },
                        rec.invalid.map_or("none", |r| r.as_str()),
                        done.path + len * f,
                        done.max_speed.max(speed) * 3.6,
                        rec.off_track_s,
                    ));
                    self.stats = Some(LapStats {
                        path: len * (1.0 - f),
                        max_speed: speed,
                    });
                    crossed = true;
                }
            }
        }
        if !crossed {
            if let Some(s) = self.stats.as_mut() {
                s.path += len;
                s.max_speed = s.max_speed.max(speed);
            }
        }
        Ok(out)
    }

    /// The last line of a finished run: `race: laps_valid=... laps_invalid=... best_lap_s=...`.
    fn summary(&self) -> String {
        let laps = self.timer.laps();
        let valid = laps.iter().filter(|l| l.valid()).count();
        let best = self
            .timer
            .best_valid_ms()
            .map_or("none".to_string(), format_seconds);
        format!(
            "race: laps_valid={valid} laps_invalid={} best_lap_s={best}\n",
            laps.len() - valid
        )
    }
}

/// `race: FAIL unstable at t=<4 decimals>: <reason>`, the last line of an aborted run.
fn race_fail_line(t: f64, reason: &str) -> String {
    format!("race: FAIL unstable at t={t:.4}: {reason}\n")
}

/// The `track` line.
fn track_line(track: &Track, name: &str) -> String {
    let (wmin, wmax) = track.width_range();
    format!(
        "track name={name} points={} checkpoints={} centre_length_m={:.2} width_min_m={wmin:.2} width_max_m={wmax:.2}\n",
        track.points.len(),
        track.checkpoints.len(),
        track.centre_length()
    )
}

/// The `track_line <kind> <x0> <z0> <x1> <z1>` lines a live run prints for the viewer: the two
/// edges (left and right of the direction of travel, +z being the right of a car facing +x), the
/// start line and each checkpoint.
fn track_draw_lines(track: &Track) -> String {
    let n = track.points.len();
    let mut s = String::new();
    let edge = |i: usize, side: f64| {
        let t = track.tangent(i);
        let h = track.widths[i] / 2.0;
        let p = track.points[i];
        [p[0] + -t[1] * h * side, p[1] + t[0] * h * side]
    };
    for i in 0..n {
        let j = (i + 1) % n;
        for (kind, side) in [("left", -1.0), ("right", 1.0)] {
            let (a, b) = (edge(i, side), edge(j, side));
            s.push_str(&format!(
                "track_line {kind} {:.4} {:.4} {:.4} {:.4}\n",
                a[0], a[1], b[0], b[1]
            ));
        }
    }
    let gate = |kind: &str, i: usize| {
        let g = track.gate(i);
        format!(
            "track_line {kind} {:.4} {:.4} {:.4} {:.4}\n",
            g.a[0], g.a[1], g.b[0], g.b[1]
        )
    };
    s.push_str(&gate("start", track.start));
    for &c in &track.checkpoints {
        s.push_str(&gate("checkpoint", c));
    }
    s
}

/// A timed lap (`--track`): returns the exit code: 0 the run reached its end (even with no valid
/// lap), 1 the sim aborted or an output write failed, 2 a refused file or flag value.
fn run_race(cfg: &Config, body: &Body, track_path: &str, mode: RaceMode) -> i32 {
    let text = match std::fs::read_to_string(track_path) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("crash-sim: --track {track_path}: {e}");
            return 2;
        }
    };
    let track = match Track::parse(&text) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("crash-sim: {track_path}: {e}");
            return 2;
        }
    };
    let name = std::path::Path::new(track_path)
        .file_stem()
        .map_or(String::new(), |s| s.to_string_lossy().into_owned());
    let Some(heading) = HeadingGroups::of(body).filter(|_| !body.wheels.is_empty()) else {
        eprintln!("crash-sim: --track needs --body with wheels and `front` and `rear` groups");
        return 2;
    };
    let placed = match place_on_grid(body, &track) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("crash-sim: {e}");
            return 2;
        }
    };
    let built = placed.build();
    if let Err(e) = cfg.validate_body(&built.beams, &built.mass) {
        eprintln!("crash-sim: {e}");
        return 2;
    }
    let ctx = RaceCtx {
        cfg,
        body: &placed,
        track: &track,
        name: &name,
        heading: &heading,
    };
    match mode {
        RaceMode::File {
            input,
            duration_given,
        } => race_file(&ctx, &input, duration_given),
        RaceMode::Live { v2 } => race_live(&ctx, v2),
    }
}

/// What both race runs need, already checked.
struct RaceCtx<'a> {
    cfg: &'a Config,
    /// The body, placed on the grid.
    body: &'a Body,
    track: &'a Track,
    /// The track file's stem.
    name: &'a str,
    heading: &'a HeadingGroups,
}

fn race_file(ctx: &RaceCtx, input_path: &str, duration_given: bool) -> i32 {
    let RaceCtx {
        cfg,
        body,
        track,
        name,
        heading,
    } = *ctx;
    let text = match std::fs::read_to_string(input_path) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("crash-sim: --input {input_path}: {e}");
            return 2;
        }
    };
    let script = match Script::parse(&text) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("crash-sim: {input_path}: {e}");
            return 2;
        }
    };
    let setup = CarSetup {
        script: Some(script.clone()),
        until_script_end: !duration_given,
        grip_frac: None,
    };
    let total = match settle::run_substeps(cfg, &setup) {
        Ok(n) => n,
        Err(e) => {
            eprintln!("crash-sim: {e}");
            return 2;
        }
    };
    let h = cfg.dt / cfg.substeps as f64;
    let mut one = cfg.clone();
    one.dt = h;
    one.substeps = 1;
    let sim = match settle::car_sim(one, body, &setup) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("crash-sim: {e}");
            return 2;
        }
    };
    let tol = h * 1e-6;
    let mut race = Race::new(sim, track, h);
    let mut out = track_line(track, name);
    for _ in 0..total {
        let t_start = race.i as f64 * h;
        if let Err(e) = script.apply(&mut race.sim, t_start, tol) {
            eprintln!("crash-sim: {e}");
            return 2;
        }
        match race.step() {
            Ok(lines) => out.push_str(&lines),
            Err((t, reason)) => {
                out.push_str(&race_fail_line(t, &reason));
                emit(&out);
                return 1;
            }
        }
    }
    out.push_str(&heading.final_state(&race.sim, race.i as f64 * h).render());
    out.push_str(&race.summary());
    emit(&out);
    0
}

/// Read the next non-blank line of the live input: `Ok(None)` at end of input. A last line with no
/// `\n` counts as a line, as in a file.
fn race_next_line(
    parser: &mut input::LineParser,
    fed: &mut usize,
    src: &mut impl std::io::BufRead,
) -> Result<Option<input::Line>, input::LiveError> {
    loop {
        let mut buf: Vec<u8> = Vec::new();
        let n = src
            .read_until(b'\n', &mut buf)
            .map_err(|e| input::LiveError::Input(format!("input: cannot read: {e}")))?;
        if n == 0 {
            return Ok(None);
        }
        *fed += 1;
        if buf.last() == Some(&b'\n') {
            buf.pop();
        }
        let text = String::from_utf8(buf)
            .map_err(|_| input::LiveError::Input(format!("input line {fed}: not valid UTF-8")))?;
        if let Some(l) = parser.feed(&text).map_err(input::LiveError::Input)? {
            return Ok(Some(l));
        }
    }
}

/// The live race: as `input::live` (same substep count rule, so a file of the same lines gives
/// the same bytes) with the lap timer, the `track` and `track_line` lines before the first row and
/// each race line as soon as it happens, then the `final` and `race:` lines at end of input.
fn race_live(ctx: &RaceCtx, v2: bool) -> i32 {
    let stdin = std::io::stdin();
    let mut src = stdin.lock();
    let stdout = std::io::stdout();
    let mut out = stdout.lock();
    let mut emit = |text: &str| write_all_flush(&mut out, text);
    match race_live_run(ctx, v2, &mut src, &mut emit) {
        Ok(()) => 0,
        Err(input::LiveError::Input(e)) => {
            eprintln!("crash-sim: --live: {e}");
            2
        }
        Err(input::LiveError::Abort(_)) => 1,
        Err(input::LiveError::Output(e)) if e.kind() == std::io::ErrorKind::BrokenPipe => 0,
        Err(input::LiveError::Output(e)) => {
            eprintln!("{}", write_failure_message(&e));
            1
        }
    }
}

fn race_live_run(
    ctx: &RaceCtx,
    v2: bool,
    src: &mut impl std::io::BufRead,
    emit: &mut impl FnMut(&str) -> std::io::Result<()>,
) -> Result<(), input::LiveError> {
    use input::LiveError;
    let RaceCtx {
        cfg,
        body,
        track,
        name,
        heading,
    } = *ctx;
    let h = cfg.dt / cfg.substeps as f64;
    let tol = h * 1e-6;
    let mut parser = input::LineParser::new();
    let mut fed = 0usize;
    let first = race_next_line(&mut parser, &mut fed, src)?.ok_or_else(|| {
        LiveError::Input(
            "input: end of input before any line (a run needs at least `0 <throttle> <brake> <steer>`)"
                .into(),
        )
    })?;
    let mut lines = vec![first];
    let mut one = cfg.clone();
    one.dt = h;
    one.substeps = 1;
    let sim = settle::car_sim(one, body, &CarSetup::default()).map_err(LiveError::Input)?;
    let mut probe = sim
        .car
        .clone()
        .ok_or_else(|| LiveError::Input("internal: a live run built no car".into()))?;
    probe
        .set_controls(first.throttle, first.brake, first.steer)
        .map_err(|e| LiveError::Input(format!("input line {}: {e}", first.number)))?;
    let mut writer = if v2 {
        Some(input::FrameWriter::new(&sim, body).map_err(LiveError::Input)?)
    } else {
        None
    };
    let mut text = track_line(track, name);
    text.push_str(&track_draw_lines(track));
    match writer.as_mut() {
        Some(w) => {
            text.push_str(w.header());
            text.push_str(&w.frame(&sim, 0.0));
        }
        None => {
            text.push_str(input::ROWS_HEADER);
            text.push('\n');
            text.push_str(&input::render_row(&heading.row(&sim, 0.0)));
        }
    }
    emit(&text).map_err(LiveError::Output)?;
    let mut race = Race::new(sim, track, h);
    let mut t_last = first.t;
    loop {
        let allowed = (t_last / h * (1.0 + 1e-9)).floor() as u64;
        while race.i < allowed {
            let t_start = race.i as f64 * h;
            let n = lines.partition_point(|l| l.t <= t_start + tol);
            let l = lines[n.saturating_sub(1)];
            match race.sim.car.as_mut() {
                Some(car) => car
                    .set_controls(l.throttle, l.brake, l.steer)
                    .map_err(|e| LiveError::Input(format!("input line {}: {e}", l.number)))?,
                None => return Err(LiveError::Input("internal: a live run built no car".into())),
            }
            match race.step() {
                Ok(lines_text) => {
                    if !lines_text.is_empty() {
                        emit(&lines_text).map_err(LiveError::Output)?;
                    }
                }
                Err((t, reason)) => {
                    emit(&race_fail_line(t, &reason)).map_err(LiveError::Output)?;
                    return Err(LiveError::Abort(reason));
                }
            }
            if race.i.is_multiple_of(cfg.substeps as u64) {
                let t = race.i as f64 * h;
                let text = match writer.as_mut() {
                    Some(w) => w.frame(&race.sim, t),
                    None => input::render_row(&heading.row(&race.sim, t)),
                };
                emit(&text).map_err(LiveError::Output)?;
            }
        }
        match race_next_line(&mut parser, &mut fed, src)? {
            Some(l) => {
                probe
                    .set_controls(l.throttle, l.brake, l.steer)
                    .map_err(|e| LiveError::Input(format!("input line {}: {e}", l.number)))?;
                t_last = l.t;
                lines.push(l);
            }
            None => break,
        }
    }
    if race.i == 0 {
        return Err(LiveError::Input(format!(
            "the input ends at t={t_last}, so the run has 0 substeps"
        )));
    }
    let mut tail = String::new();
    if let Some(w) = writer.as_mut() {
        tail.push_str(&w.tail(&race.sim));
    }
    tail.push_str(&heading.final_state(&race.sim, race.i as f64 * h).render());
    tail.push_str(&race.summary());
    emit(&tail).map_err(LiveError::Output)
}

// ---------------------------------------------------------------------------------------
// The pulse report (docs/STAGE6-M4-M5.md, "The pulse report")

/// Most substeps a pulse report run may take: the run keeps several f64 per substep, so a longer
/// run is refused (exit 2) instead of exhausting memory. 5 million is 500 s at h = 0.1 ms.
const PULSE_MAX_SUBSTEPS: u64 = 5_000_000;

/// The checked flags of a pulse report. Every number is finite; see `pulse_params`.
struct PulseParams {
    ref_path: String,
    csv: Option<String>,
    timing: bool,
    sensor: String,
    crush: (String, String),
    cfc_shape: f64,
    cfc_int: f64,
    window: f64,
    threshold: f64,
    tail: f64,
    mass_tol: f64,
    speed_tol: f64,
    margin_dv: f64,
    margin_peak: f64,
    margin_peak_time: f64,
    margin_duration: f64,
    margin_olc: f64,
    margin_static: f64,
}

/// Check the pulse flags: every required one is present, every number finite, and in its range.
/// The error text is for a usage error (exit 2).
fn pulse_params(f: &PulseFlags, ref_path: String) -> Result<PulseParams, String> {
    if !ref_path.ends_with(".pulse.csv") {
        return Err(format!(
            "--pulse-report {ref_path}: the reference must be a path ending in .pulse.csv (its .meta is read beside it)"
        ));
    }
    if let Some(s) = &f.solver {
        if s != "node-beam" {
            return Err(format!("--solver {s}: only `node-beam` is available here"));
        }
    }
    fn need<T: Clone>(v: &Option<T>, flag: &str) -> Result<T, String> {
        v.clone()
            .ok_or_else(|| format!("--pulse-report needs {flag}"))
    }
    // (value, flag, lowest allowed, whether the lowest itself is allowed)
    fn number(v: &Option<f64>, flag: &str, min: f64, min_ok: bool) -> Result<f64, String> {
        let x = need(v, flag)?;
        if x.is_finite() && (x > min || (min_ok && x == min)) {
            Ok(x)
        } else {
            let rel = if min_ok { ">=" } else { ">" };
            Err(format!("{flag} must be finite and {rel} {min}, got {x}"))
        }
    }
    Ok(PulseParams {
        ref_path,
        csv: f.csv.clone(),
        timing: f.timing,
        sensor: need(&f.sensor, "--pulse-sensor")?,
        crush: need(&f.crush, "--pulse-crush-groups")?,
        cfc_shape: number(&f.cfc_shape, "--pulse-cfc-shape", 0.0, false)?,
        cfc_int: number(&f.cfc_int, "--pulse-cfc-int", 0.0, false)?,
        window: number(&f.window, "--pulse-window", 0.0, false)?,
        threshold: number(&f.threshold, "--pulse-threshold", 0.0, true)?,
        tail: number(&f.tail, "--pulse-crush-tail", 0.0, false)?,
        mass_tol: number(&f.mass_tol, "--setup-mass-tol", 0.0, true)?,
        speed_tol: number(&f.speed_tol, "--setup-speed-tol", 0.0, true)?,
        margin_dv: number(&f.margin_dv, "--margin-dv", 0.0, false)?,
        margin_peak: number(&f.margin_peak, "--margin-peak", 0.0, false)?,
        margin_peak_time: number(&f.margin_peak_time, "--margin-peak-time", 0.0, false)?,
        margin_duration: number(&f.margin_duration, "--margin-duration", 0.0, false)?,
        margin_olc: number(&f.margin_olc, "--margin-olc", 0.0, false)?,
        margin_static: number(&f.margin_static, "--margin-static-crush", 0.0, false)?,
    })
}

/// What a pulse report produced.
struct PulseOutcome {
    /// Everything for stdout, ending with the `pulse:` line.
    text: String,
    /// The `--pulse-csv` file's text, when the car touched the wall.
    csv: Option<String>,
    /// `pulse: ok`.
    ok: bool,
    /// `(sim_s, wall_s)` of the solver loop, when it ran to the end.
    timing: Option<(f64, f64)>,
}

/// Run the pulse report; print it; return the exit code: 0 ok, 1 a FAIL or an abort, 2 a refusal
/// (usage, reference, setup), in which case nothing is printed on stdout.
fn run_pulse(cfg: &Config, body: &Body, p: &PulseParams) -> i32 {
    let out = match pulse_report(cfg, body, p) {
        Ok(o) => o,
        Err(e) => {
            eprintln!("crash-sim: {e}");
            return 2;
        }
    };
    if let (Some(path), Some(csv)) = (&p.csv, &out.csv) {
        if let Err(e) = std::fs::write(path, csv) {
            eprintln!("crash-sim: --pulse-csv {path}: {e}");
            return 1;
        }
    }
    if let (true, Some((sim_s, wall_s))) = (p.timing, out.timing) {
        eprintln!(
            "timing: sim_s={sim_s:.4} wall_s={wall_s:.4} rtf={:.4}",
            wall_s / sim_s
        );
    }
    emit(&out.text);
    i32::from(!out.ok)
}

/// The sensor group's mass-weighted mean x-velocity u (m/s).
fn group_u(sim: &crash_sim::Sim, ix: &[usize]) -> f64 {
    let (mut p, mut m) = (0.0, 0.0);
    for &i in ix {
        p += sim.mass[i] * sim.vel[i].x;
        m += sim.mass[i];
    }
    p / m
}

/// Distance between the centroids of two node groups (m), as the crumple report measures it.
fn group_len(sim: &crash_sim::Sim, a: &[usize], b: &[usize]) -> f64 {
    let centroid = |ix: &[usize]| {
        let mut s = crash_sim::vec3::Vec3::ZERO;
        for &i in ix {
            s += sim.pos[i];
        }
        s * (1.0 / ix.len() as f64)
    };
    (centroid(b) - centroid(a)).length()
}

/// Number of points of the crush profile: NHTSA's C1 to C6 (docs/STAGE6-M4-M5.md V8).
const PROFILE_POINTS: usize = 6;
/// Two front nodes whose start z differs by at most this (m) are one column of the profile.
const PROFILE_COLUMN_TOL_M: f64 = 1e-9;

/// Weights of the six-point crush profile (docs/STAGE6-M4-M5.md, "Metrics", `static_crush_m`),
/// one per node of the front group, from the nodes' start z (the lateral axis; x is the crash
/// axis, y is up). The six points are equally spaced across the front, from its leftmost to its
/// rightmost node column, both ends included, as C1 (left side) to C6 (right side) are across the
/// damage width. Nodes whose z agree within `PROFILE_COLUMN_TOL_M` are one column, its value the
/// plain mean of its nodes; a point between two columns takes the linear interpolation of their
/// values. The profile is the mean of the six points, so it is a fixed weighted mean of the nodes:
/// each column gets the sum of its interpolation shares over the six points, divided by six,
/// shared equally by its nodes. The weights sum to 1. One column takes the plain mean.
fn profile_weights(z: &[f64]) -> Vec<f64> {
    let mut cols: Vec<f64> = Vec::new();
    for &v in z {
        if !cols.iter().any(|c| (c - v).abs() <= PROFILE_COLUMN_TOL_M) {
            cols.push(v);
        }
    }
    cols.sort_by(f64::total_cmp);
    let col_of = |v: f64| {
        cols.iter()
            .position(|c| (c - v).abs() <= PROFILE_COLUMN_TOL_M)
            .expect("every node is in a column")
    };
    let mut col_w = vec![0.0; cols.len()];
    if cols.len() == 1 {
        col_w[0] = 1.0;
    } else {
        let (lo, hi) = (cols[0], cols[cols.len() - 1]);
        let share = 1.0 / PROFILE_POINTS as f64;
        for i in 0..PROFILE_POINTS {
            let zi = lo + (hi - lo) * i as f64 / (PROFILE_POINTS - 1) as f64;
            // The bracketing columns j, j + 1 (the last pair for the right end).
            let j = (0..cols.len() - 1)
                .find(|&j| zi <= cols[j + 1])
                .unwrap_or(cols.len() - 2);
            let t = ((zi - cols[j]) / (cols[j + 1] - cols[j])).clamp(0.0, 1.0);
            col_w[j] += share * (1.0 - t);
            col_w[j + 1] += share * t;
        }
    }
    let mut count = vec![0usize; cols.len()];
    for &v in z {
        count[col_of(v)] += 1;
    }
    z.iter()
        .map(|&v| {
            let c = col_of(v);
            col_w[c] / count[c] as f64
        })
        .collect()
}

/// The six-point profile depth (m): the weighted mean (`profile_weights`) over the front group
/// `b` of each node's x ahead of the plain mean x of the reference group `a`. Static crush is its
/// start value minus its mean over the tail. Known limit: it measures along the world x axis, so a
/// body that turns about y reads its rotation as crush; the validation runs hit a wall square to x
/// with a car symmetric in z.
fn profile_depth(sim: &crash_sim::Sim, a: &[usize], b: &[usize], w: &[f64]) -> f64 {
    let xa = a.iter().map(|&i| sim.pos[i].x).sum::<f64>() / a.len() as f64;
    b.iter()
        .zip(w)
        .map(|(&i, wi)| wi * (sim.pos[i].x - xa))
        .sum()
}

/// True if any node is on or behind one of `planes[range]`, or any wheel (a tyre) is within its
/// radius of one: the sim's contact test (`any_at_or_behind`) extended to the tyres.
fn touches(sim: &crash_sim::Sim, range: std::ops::Range<usize>) -> bool {
    let planes = &sim.planes[range];
    if collide::any_at_or_behind(&sim.pos, planes) {
        return true;
    }
    sim.car.as_ref().is_some_and(|c| {
        c.wheels
            .iter()
            .any(|w| planes.iter().any(|pl| pl.distance(w.pos) <= w.p.radius))
    })
}

/// The metrics of one filtered pulse window (docs: "Metrics"), as positive magnitudes.
struct PulseMetrics {
    dv: f64,
    peak: f64,
    peak_time: f64,
    /// None when the pulse never reaches the threshold.
    duration: Option<f64>,
    /// None when the OLC occupant never completes its travel.
    olc: Option<f64>,
    dyn_crush: f64,
}

/// Metrics of `f60` (CFC shape) and `f180` (CFC int) windows sampled every `h` s from contact,
/// with impact speed `v0` (m/s). The duration counts samples at or below -threshold, so the
/// samples are clamped at 0 from above first (`pulse::duration` itself counts |sample|).
fn pulse_metrics(
    f60: &[f64],
    f180: &[f64],
    h: f64,
    v0: f64,
    threshold: f64,
) -> Result<PulseMetrics, String> {
    let e = |e: pulse::PulseError| e.to_string();
    let dv = -pulse::delta_v(f180, h).map_err(e)?;
    let (pk, pk_t) = pulse::peak(f60, h).map_err(e)?;
    let below: Vec<f64> = f60.iter().map(|v| v.min(0.0)).collect();
    let duration = pulse::duration(&below, h, threshold).map_err(e)?;
    let olc = pulse::olc(f180, h, v0).map_err(e)?;
    // Displacement toward the wall: x(t) = integral of (v0 + integral of a), trapezoid.
    let (mut v, mut x, mut dyn_crush) = (v0, 0.0_f64, 0.0_f64);
    for w in f180.windows(2) {
        let vn = v + 0.5 * (w[0] + w[1]) * h;
        x += 0.5 * (v + vn) * h;
        v = vn;
        dyn_crush = dyn_crush.max(x);
    }
    Ok(PulseMetrics {
        dv,
        peak: pk.abs(),
        peak_time: pk_t,
        duration,
        olc,
        dyn_crush,
    })
}

/// The report. `Err` is a refusal (exit 2): a file the sim or reference cannot use, a rate under
/// 10 x CFC, a window longer than either pulse. A FAIL or an abort is an `Ok` outcome with
/// `ok == false` (exit 1).
///
/// Choices the docs leave open, fixed here:
/// - Substep k (1-based) gives the sample a_k = (u_k - u_(k-1)) / h at t = k h; contact is the
///   first substep k whose end has a node or tyre on or behind a body plane (the ground, the
///   first plane, is not a wall), and sample j of the window, t = j h from contact, is a_(k + j).
/// - The impact speed (setup line and OLC / dynamic crush start) is u just before the contact
///   substep; with no contact it is the start speed, and only `no contact` follows.
/// - `ground_contacts_in_window` counts substeps in the window with a node or tyre on or behind
///   the ground; the FAIL line's `t` is the first of them, in seconds from contact.
/// - Every line that can be computed is printed even when an earlier rule fails; the last line
///   names the first rule that failed. A sim OLC or duration that does not exist counts as 0.
/// - Static crush (the six-point profile depth, `profile_weights`, and the old centroid distance
///   L) uses the series' value at the start of the run, and the substeps with
///   t >= run end - tail (to 1e-6 of h).
fn pulse_report(cfg: &Config, body: &Body, p: &PulseParams) -> Result<PulseOutcome, String> {
    // The reference: the pulse and its meta, both strict (reference.rs).
    let meta_path = format!(
        "{}.meta",
        &p.ref_path[..p.ref_path.len() - ".pulse.csv".len()]
    );
    let refp = reference::load_pulse(&p.ref_path).map_err(|e| format!("reference: {e}"))?;
    let meta = reference::load_meta(&meta_path).map_err(|e| format!("reference meta: {e}"))?;

    // The sim's step, and the J211 rate rule for both pulses (n20-03's refusal).
    let h = cfg.dt / cfg.substeps as f64;
    for cfc in [p.cfc_shape, p.cfc_int] {
        pulse::cfc_filter(&[0.0, 0.0], h, cfc).map_err(|e| format!("sim pulse: {e}"))?;
        pulse::cfc_filter(&[0.0, 0.0], refp.dt, cfc).map_err(|e| format!("reference: {e}"))?;
    }
    // A huge --duration must be a refusal, not an overflow: the product saturates.
    let n_sub = crash_sim::step_count(cfg).saturating_mul(u64::from(cfg.substeps));
    if n_sub == 0 {
        return Err("the run has no substep: raise --duration".into());
    }
    if n_sub > PULSE_MAX_SUBSTEPS {
        return Err(format!(
            "the run has {n_sub} substeps, over the limit of {PULSE_MAX_SUBSTEPS}: lower --duration or raise --dt"
        ));
    }
    if p.tail > cfg.duration {
        return Err(format!(
            "--pulse-crush-tail {} s is longer than the run ({} s)",
            p.tail, cfg.duration
        ));
    }

    // Groups.
    let group = |name: &str| -> Result<Vec<usize>, String> {
        let g = body
            .groups
            .iter()
            .find(|g| g.name == name)
            .ok_or_else(|| format!("the body has no group `{name}`"))?;
        Ok(g.nodes
            .iter()
            .map(|id| body.node_index(*id).expect("group names declared nodes"))
            .collect())
    };
    let sensor = group(&p.sensor)?;
    let crush_a = group(&p.crush.0)?;
    let crush_b = group(&p.crush.1)?;

    // The sim, built with the real config (so every refusal is the usual one), then stepped one
    // substep at a time: with dt = h and one substep, `advance` is exactly one substep of the
    // same length h (`cfg` is public).
    let mut sim = crash_sim::Sim::from_car(cfg.clone(), body)?;
    sim.cfg.dt = h;
    sim.cfg.substeps = 1;
    let wheel_mass: f64 = sim
        .car
        .as_ref()
        .map_or(0.0, |c| c.wheels.iter().map(|w| w.p.mass).sum());
    let sim_mass = sim.mass.iter().sum::<f64>() + wheel_mass;

    let n = n_sub as usize;
    let mut u = Vec::with_capacity(n + 1);
    let mut len = Vec::with_capacity(n + 1);
    let mut depth = Vec::with_capacity(n + 1);
    let mut ground = Vec::with_capacity(n + 1);
    let z0: Vec<f64> = crush_b.iter().map(|&i| sim.pos[i].z).collect();
    let weights = profile_weights(&z0);
    u.push(group_u(&sim, &sensor));
    len.push(group_len(&sim, &crush_a, &crush_b));
    depth.push(profile_depth(&sim, &crush_a, &crush_b, &weights));
    ground.push(false);
    let mut contact: Option<usize> = None;
    let started = std::time::Instant::now();
    for k in 1..=n {
        sim.advance();
        if let Err(e) = sim.check() {
            return Ok(PulseOutcome {
                text: format!("pulse: FAIL unstable at t = {:.4} s: {e}\n", k as f64 * h),
                csv: None,
                ok: false,
                timing: None,
            });
        }
        u.push(group_u(&sim, &sensor));
        len.push(group_len(&sim, &crush_a, &crush_b));
        depth.push(profile_depth(&sim, &crush_a, &crush_b, &weights));
        ground.push(touches(&sim, 0..1));
        if contact.is_none() && touches(&sim, 1..sim.planes.len()) {
            contact = Some(k);
        }
    }
    let wall_s = started.elapsed().as_secs_f64();
    let timing = Some((n as f64 * h, wall_s));

    // The sim pulse: a_k = (u_k - u_(k-1)) / h at t = k h, k = 1..=n.
    let accel: Vec<f64> = u.windows(2).map(|w| (w[1] - w[0]) / h).collect();

    // Header and setup lines (rule 1).
    let mut text = String::new();
    text.push_str(&format!(
        "pulse_ref={}\npulse_channel={}\n",
        meta.test, meta.channel
    ));
    // The impact speed is the sensor group's u just before the contact substep (u_0 if there is
    // no contact, where only the no-contact FAIL follows).
    let v0 = u[contact.map_or(0, |k| k - 1)];
    let rel = |sim: f64, r: f64| (sim - r) / r;
    let mass_err = rel(sim_mass, meta.mass_kg);
    let speed_kmh = v0 * 3.6;
    let speed_err = rel(speed_kmh, meta.speed_kmh);
    let mass_ok = mass_err.abs() <= p.mass_tol;
    let speed_ok = speed_err.abs() <= p.speed_tol;
    let tag = |ok: bool| if ok { "ok" } else { "FAIL" };
    text.push_str(&format!(
        "setup mass_kg sim={:.1} ref={:.1} err={:+.4} tol={:.4} {}\n",
        sim_mass,
        meta.mass_kg,
        mass_err,
        p.mass_tol,
        tag(mass_ok)
    ));
    text.push_str(&format!(
        "setup speed_kmh sim={:.2} ref={:.2} err={:+.4} tol={:.4} {}\n",
        speed_kmh,
        meta.speed_kmh,
        speed_err,
        p.speed_tol,
        tag(speed_ok)
    ));
    let Some(kc) = contact else {
        text.push_str("contact_s=none\npulse: FAIL no contact\n");
        return Ok(PulseOutcome {
            text,
            csv: None,
            ok: false,
            timing,
        });
    };
    text.push_str(&format!("contact_s={:.4}\n", kc as f64 * h));

    // The window: samples j = 0..n_win at t = j h from contact, sim sample j being a_(kc + j).
    // The step count is compared as a float first, so a huge window cannot overflow the count.
    let window_steps = (p.window / h * (1.0 + 1e-9)).floor();
    let n_win = if window_steps <= (n - kc) as f64 {
        window_steps as usize + 1
    } else {
        usize::MAX
    };
    if n_win == usize::MAX || kc + n_win - 1 > n {
        return Err(format!(
            "--pulse-window {} s is longer than the sim pulse after contact (contact at {:.4} s, run {} s)",
            p.window,
            kc as f64 * h,
            cfg.duration
        ));
    }
    let ground_window = &ground[kc..kc + n_win];
    let ground_count = ground_window.iter().filter(|&&g| g).count();
    text.push_str(&format!("ground_contacts_in_window={ground_count}\n"));
    let ground_first = ground_window.iter().position(|&g| g);

    // Filters: each pulse over its whole record at its own rate, then the window.
    let sim_fail = |mut text: String, what: &str, e: String| {
        text.push_str(&format!("pulse: FAIL {what}: {e}\n"));
        PulseOutcome {
            text,
            csv: None,
            ok: false,
            timing,
        }
    };
    let f60 = match pulse::cfc_filter(&accel, h, p.cfc_shape) {
        Ok(v) => v,
        Err(e) => return Ok(sim_fail(text, "sim pulse", e.to_string())),
    };
    let f180 = match pulse::cfc_filter(&accel, h, p.cfc_int) {
        Ok(v) => v,
        Err(e) => return Ok(sim_fail(text, "sim pulse", e.to_string())),
    };
    let win = |v: &[f64]| v[kc - 1..kc - 1 + n_win].to_vec();
    let (s60, s180) = (win(&f60), win(&f180));
    let ref_window = |cfc: f64| -> Result<Vec<f64>, String> {
        let filtered =
            pulse::cfc_filter(&refp.accel, refp.dt, cfc).map_err(|e| format!("reference: {e}"))?;
        let rec = reference::Pulse {
            t0: refp.t0,
            dt: refp.dt,
            accel: filtered,
        };
        let r = reference::resample_at(&rec, 0.0, h, n_win).map_err(|e| {
            format!(
                "--pulse-window {} s does not fit the reference pulse: {e}",
                p.window
            )
        })?;
        Ok(r.accel)
    };
    let (r60, r180) = (ref_window(p.cfc_shape)?, ref_window(p.cfc_int)?);

    let ref_v0 = meta.speed_kmh / 3.6;
    let sm = match pulse_metrics(&s60, &s180, h, v0, p.threshold) {
        Ok(m) => m,
        Err(e) => return Ok(sim_fail(text, "sim metrics", e)),
    };
    let rm = pulse_metrics(&r60, &r180, h, ref_v0, p.threshold)
        .map_err(|e| format!("reference metrics: {e}"))?;
    let ref_dur = rm
        .duration
        .ok_or("the reference pulse never reaches --pulse-threshold")?;
    let ref_olc = rm
        .olc
        .ok_or("the reference pulse has no OLC (the occupant never completes its travel)")?;
    // A missing sim duration or OLC counts as 0 (a norm of 1 / margin, a FAIL).
    let (sim_dur, sim_olc) = (sm.duration.unwrap_or(0.0), sm.olc.unwrap_or(0.0));

    // Static crush: a series' start value minus its mean over the last `tail` seconds of the
    // run. The gated value is the six-point profile depth (the reference's mean of C1 to C6,
    // rule change of 2026-11-22); the old centroid distance L is printed as
    // `info static_crush_centroid_m` for comparison and not gated.
    let t_end = n as f64 * h;
    let tol = h * 1e-6;
    let settled = |series: &[f64]| {
        let tail: Vec<f64> = series
            .iter()
            .enumerate()
            .filter(|(k, _)| *k as f64 * h >= t_end - p.tail - tol)
            .map(|(_, l)| *l)
            .collect();
        series[0] - tail.iter().sum::<f64>() / tail.len() as f64
    };
    let sim_static = settled(&depth);
    let sim_static_centroid = settled(&len);

    // Gated metrics, in the printed order: (name, decimals, sim, ref, margin, absolute error?).
    struct Row {
        name: &'static str,
        dec: usize,
        sim: f64,
        refv: f64,
        margin: f64,
        absolute: bool,
    }
    let mut rows = vec![
        Row {
            name: "dv_mps",
            dec: 4,
            sim: sm.dv,
            refv: rm.dv,
            margin: p.margin_dv,
            absolute: false,
        },
        Row {
            name: "peak_mps2",
            dec: 1,
            sim: sm.peak,
            refv: rm.peak,
            margin: p.margin_peak,
            absolute: false,
        },
        Row {
            name: "peak_time_s",
            dec: 4,
            sim: sm.peak_time,
            refv: rm.peak_time,
            margin: p.margin_peak_time,
            absolute: true,
        },
        Row {
            name: "duration_s",
            dec: 4,
            sim: sim_dur,
            refv: ref_dur,
            margin: p.margin_duration,
            absolute: false,
        },
        Row {
            name: "olc_mps2",
            dec: 1,
            sim: sim_olc,
            refv: ref_olc,
            margin: p.margin_olc,
            absolute: false,
        },
    ];
    if let Some(s) = meta.static_crush_m {
        rows.push(Row {
            name: "static_crush_m",
            dec: 4,
            sim: sim_static,
            refv: s,
            margin: p.margin_static,
            absolute: false,
        });
    }
    let mut norms = Vec::new();
    let mut first_bad: Option<String> = None;
    for r in &rows {
        if !(r.refv.is_finite() && (r.absolute || r.refv > 0.0)) {
            return Err(format!(
                "the reference's {} is {}: a relative error needs a finite value above zero",
                r.name, r.refv
            ));
        }
        let err = if r.absolute {
            r.sim - r.refv
        } else {
            (r.sim - r.refv) / r.refv
        };
        let norm = err.abs() / r.margin;
        // A non-finite sim value is a FAIL, never an ok.
        let ok = r.sim.is_finite() && norm <= 1.0;
        if !ok && first_bad.is_none() {
            first_bad = Some(if r.sim.is_finite() {
                format!("{} norm={norm:.4}", r.name)
            } else {
                format!("{} is not finite", r.name)
            });
        }
        norms.push(norm);
        text.push_str(&format!(
            "metric {} sim={:.d$} ref={:.d$} err={:+.4} norm={:.4} {}\n",
            r.name,
            r.sim,
            r.refv,
            err,
            norm,
            tag(ok),
            d = r.dec
        ));
    }
    if meta.static_crush_m.is_none() {
        text.push_str(&format!(
            "metric static_crush_m sim={sim_static:.4} ref=none\n"
        ));
    }
    text.push_str(&format!(
        "info static_crush_centroid_m sim={sim_static_centroid:.4}\n"
    ));
    text.push_str(&format!(
        "info dyn_crush_m sim={:.4} ref={:.4}\n",
        sm.dyn_crush, rm.dyn_crush
    ));
    let score = norms.iter().sum::<f64>() / norms.len() as f64;
    text.push_str(&format!("pulse_score={score:.4}\n"));

    // The verdict: the first rule that fails, in the order of the docs.
    let verdict = if !mass_ok {
        Some(format!(
            "setup mass_kg err={mass_err:+.4} tol={:.4}",
            p.mass_tol
        ))
    } else if !speed_ok {
        Some(format!(
            "setup speed_kmh err={speed_err:+.4} tol={:.4}",
            p.speed_tol
        ))
    } else if let Some(j) = ground_first {
        Some(format!("ground contact at t={:.4}", j as f64 * h))
    } else {
        first_bad
    };
    let ok = verdict.is_none();
    match verdict {
        None => text.push_str("pulse: ok\n"),
        Some(v) => text.push_str(&format!("pulse: FAIL {v}\n")),
    }

    // The CSV: the sim pulse from contact to the end of the run.
    let mut csv = String::from("t_s,accel_mps2,accel_cfc60_mps2,accel_cfc180_mps2\n");
    for j in kc - 1..accel.len() {
        csv.push_str(&format!(
            "{:.6},{:.4},{:.4},{:.4}\n",
            (j + 1 - kc) as f64 * h,
            accel[j],
            f60[j],
            f180[j]
        ));
    }
    Ok(PulseOutcome {
        text,
        csv: Some(csv),
        ok,
        timing,
    })
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

    fn close(got: &[f64], want: &[f64]) {
        assert_eq!(got.len(), want.len(), "{got:?} against {want:?}");
        for (g, w) in got.iter().zip(want) {
            assert!((g - w).abs() < 1e-12, "{got:?} against {want:?}");
        }
    }

    /// The validation car's front: columns at z = -0.844, -0.75425, 0.75425, 0.844, two nodes
    /// each. Points 1 and 6 sit on the outer columns; points 2 to 5 fall between the inner two at
    /// t = 0.1643, 0.3881, 0.6119, 0.8357, whose shares sum to 2 for each inner column. So the
    /// column weights are 1/6, 2/6, 2/6, 1/6, halved for the two nodes of each column.
    #[test]
    fn profile_weights_of_the_validation_front() {
        let z = [
            -0.844, -0.844, -0.75425, -0.75425, 0.75425, 0.75425, 0.844, 0.844,
        ];
        let (a, b) = (1.0 / 12.0, 2.0 / 12.0);
        close(&profile_weights(&z), &[a, a, b, b, b, b, a, a]);
    }

    /// Six evenly spaced columns: each point is one column, 1/6 each, in any node order.
    #[test]
    fn profile_weights_six_even_columns() {
        let z = [0.5, 0.0, 0.3, 0.1, 0.4, 0.2];
        close(&profile_weights(&z), &[1.0 / 6.0; 6]);
    }

    /// Two columns, one with two nodes: 1/2 per column (not 1/3 per node, as a centroid has).
    #[test]
    fn profile_weights_two_columns_uneven_counts() {
        let z = [0.0, 1.0, 0.0];
        close(&profile_weights(&z), &[0.25, 0.5, 0.25]);
    }

    /// One column: the plain mean.
    #[test]
    fn profile_weights_one_column() {
        close(&profile_weights(&[2.0, 2.0, 2.0, 2.0]), &[0.25; 4]);
    }

    #[test]
    fn failure_message_names_the_write() {
        let m = write_failure_message(&Error::new(ErrorKind::BrokenPipe, "pipe closed"));
        assert_eq!(m, "crash-sim: cannot write output: pipe closed");
    }
}
