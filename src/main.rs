#[path = "cli/ai_race.rs"]
mod ai_race;
#[path = "cli/net_peer.rs"]
mod net_peer;
#[path = "cli/net_race.rs"]
mod net_race;
#[path = "cli/race.rs"]
mod race;
#[path = "cli/tune.rs"]
mod tune;
use race::{run_race, RaceMode};
#[path = "cli/pulse_run.rs"]
mod pulse_run;
use pulse_run::{pulse_params, run_pulse};
#[path = "cli/single_run.rs"]
mod single_run;

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

fn main() {
    let ai_args: Vec<String> = std::env::args().skip(1).collect();
    if ai_args.first().map(String::as_str) == Some("tune") {
        exit(tune::run_tune_cli(&ai_args[1..]));
    }
    if ai_args.iter().any(|a| a == "--shell") {
        exit(crash_sim::shell::run(
            &ai_args,
            &mut std::io::stdin().lock(),
            &mut std::io::stdout().lock(),
            &mut std::io::stderr(),
        ));
    }
    if ai_args.iter().any(|a| a == "--host" || a == "--join")
        && !ai_args.iter().any(|a| a == "--input2")
    {
        exit(net_peer::run_peer_cli(&ai_args));
    }
    if net_race::wants_reference(&ai_args) {
        exit(net_race::run_reference_cli(&ai_args));
    }
    if ai_args.iter().any(|a| a == "--ai") {
        exit(ai_race::run_ai_cli(&ai_args));
    }
    single_run::run(std::env::args().skip(1));
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
