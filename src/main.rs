use crash_sim::{events_text, run_body_report, run_checked_with_contact, to_csv, to_csv_with_broken, Config};
use std::io::Write;
use std::process::exit;

use crash_sim::{collide, settle};

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
    eprintln!("       crash-sim --body <file.bsn> ... --settle-report ... --crumple-report --crumple-groups <from>,<to> --crumple-min <m> --crumple-max <m> --crumple-drift <m>");
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
    let mut events = false;
    let mut settle_report = false;
    let mut body_path: Option<String> = None;
    let mut crumple_report = false;
    let mut crumple_groups: Option<(String, String)> = None;
    let (mut cmin, mut cmax, mut cdrift): (Option<f64>, Option<f64>, Option<f64>) = (None, None, None);
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
                    Some((a, b)) if !a.is_empty() && !b.is_empty() && !b.contains(',') => crumple_groups = Some((a.to_string(), b.to_string())),
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
    let any_crumple = crumple_groups.is_some() || cmin.is_some() || cmax.is_some() || cdrift.is_some();
    if (crumple_report || any_crumple) && !settle_report {
        eprintln!("--crumple-report and its flags need --settle-report (and its five thresholds)");
        usage()
    }
    if any_crumple && !crumple_report {
        eprintln!("--crumple-groups, --crumple-min, --crumple-max and --crumple-drift need --crumple-report");
        usage()
    }
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
        if crumple_report {
            // No defaults: the groups and the three thresholds are all required.
            let (Some((from, to)), Some(min), Some(max), Some(drift)) = (crumple_groups, cmin, cmax, cdrift) else {
                eprintln!("--crumple-report needs --crumple-groups, --crumple-min, --crumple-max and --crumple-drift");
                usage()
            };
            let cp = collide::CrumpleParams { from, to, min, max, drift };
            let (report, crumple) = match collide::measure_crumple(&cfg, body.as_ref(), &params, &cp) {
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
        let report = settle::measure_body(&cfg, body.as_ref(), &params);
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
                let peak = samples.iter().map(|s| s.max_beam_strain).fold(f64::MIN, f64::max);
                emit(&format!(
                    "first_contact_s={contact},peak_strain={peak:.6},final_center_y={:.4}\n",
                    last.center_y
                ));
            }
        }
    } else if !samples.is_empty() {
        // The broken_beams column is for --body runs only: the cube's CSV stays as it was.
        emit(&if body.is_some() { to_csv_with_broken(&samples) } else { to_csv(&samples) });
    }
    if events && !breaks.is_empty() {
        emit(&events_text(&breaks));
    }
    if result.is_err() {
        exit(1);
    }
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
            Err(Error::new(ErrorKind::Other, "disk full"))
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
