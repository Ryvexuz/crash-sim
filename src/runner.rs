use crate::{Body, BreakEvent, Config, Sample, Sim};

/// Everything a body run produces: the samples of the good steps, the first contact time, every
/// beam break, and the run's verdict.
pub struct Report {
    pub samples: Vec<Sample>,
    pub first_contact_s: Option<f64>,
    pub breaks: Vec<BreakEvent>,
    pub result: Result<(), String>,
}

/// Smallest dt the run accepts: the CSV time column has 4 decimals, so a smaller dt would
/// print repeated or merged t values.
pub const MIN_DT: f64 = 1e-4;

/// Output steps of a run: `floor(duration / dt)`, with a relative epsilon of 1e-9 so that a
/// duration that is a whole number of steps up to rounding (5.0 / 0.01, 0.3 / 0.01) is not
/// cut short. The last row therefore never lies after `duration` (it can lie up to dt before it).
pub fn step_count(cfg: &Config) -> u64 {
    (cfg.duration / cfg.dt * (1.0 + 1e-9)).floor() as u64
}

/// Run the sim. Returns the samples of all good steps (t = 0, dt, 2dt, ...) and,
/// if the config was refused or the state blew up, the reason (with the time).
/// The step count is `step_count(cfg)`: `floor(duration / dt)` with a tiny relative epsilon,
/// so the last row never lies after `duration`.
pub fn run_checked(cfg: &Config) -> (Vec<Sample>, Result<(), String>) {
    let (samples, _, res) = run_checked_with_contact(cfg);
    (samples, res)
}

/// Like `run_checked`, but also returns `Sim::first_contact_s` as it stood when the run
/// ended (at the abort if the run blew up).
pub fn run_checked_with_contact(cfg: &Config) -> (Vec<Sample>, Option<f64>, Result<(), String>) {
    if let Err(e) = cfg.validate() {
        return (Vec::new(), None, Err(e));
    }
    let r = run_sim(Sim::new(cfg.clone()));
    (r.samples, r.first_contact_s, r.result)
}

/// Like `run_checked_with_contact`, for a body loaded from a `.bsn` file. A refused body or
/// config comes back as the error with no samples.
pub fn run_body_checked_with_contact(cfg: &Config, body: &Body) -> (Vec<Sample>, Option<f64>, Result<(), String>) {
    let r = run_body_report(cfg, body);
    (r.samples, r.first_contact_s, r.result)
}

/// Like `run_body_checked_with_contact`, and also returns the beam breaks.
pub fn run_body_report(cfg: &Config, body: &Body) -> Report {
    match Sim::from_body(cfg.clone(), body) {
        Ok(sim) => run_sim(sim),
        Err(e) => Report { samples: Vec::new(), first_contact_s: None, breaks: Vec::new(), result: Err(e) },
    }
}

/// The run loop shared by both entry points, on a sim whose config is already validated.
fn run_sim(mut sim: Sim) -> Report {
    let cfg = sim.cfg.clone();
    let steps = step_count(&cfg);
    let mut out = Vec::with_capacity(steps as usize + 1);
    out.push(sim.sample());
    for _ in 0..steps {
        sim.advance();
        if let Err(e) = sim.check() {
            let result = Err(format!("unstable at t = {:.4} s: {e}", sim.sample().t));
            return Report { samples: out, first_contact_s: sim.first_contact_s, breaks: sim.breaks, result };
        }
        out.push(sim.sample());
    }
    Report { samples: out, first_contact_s: sim.first_contact_s, breaks: sim.breaks, result: Ok(()) }
}

/// Run the whole sim; panics if the config is refused or the sim blows up.
pub fn run(cfg: &Config) -> Vec<Sample> {
    let (samples, res) = run_checked(cfg);
    res.expect("simulation failed");
    samples
}

pub const CSV_HEADER: &str = "t,min_y,center_y,max_beam_strain";

pub fn to_csv(samples: &[Sample]) -> String {
    let mut s = String::from(CSV_HEADER);
    s.push('\n');
    for r in samples {
        s.push_str(&format!("{:.4},{:.6},{:.6},{:.6}\n", r.t, r.min_y, r.center_y, r.max_beam_strain));
    }
    s
}

/// The CSV header of a body run: the cube header plus the `broken_beams` count.
pub const CSV_HEADER_BROKEN: &str = "t,min_y,center_y,max_beam_strain,broken_beams";

/// Like `to_csv`, with a final `broken_beams` column (used for `--body` runs only, so the cube's
/// output stays as it was).
pub fn to_csv_with_broken(samples: &[Sample]) -> String {
    let mut s = String::from(CSV_HEADER_BROKEN);
    s.push('\n');
    for r in samples {
        s.push_str(&format!("{:.4},{:.6},{:.6},{:.6},{}\n", r.t, r.min_y, r.center_y, r.max_beam_strain, r.broken_beams));
    }
    s
}

/// One line per break, `break t=<4 decimals> beam=<index> strain=<6 decimals>` (strain is the
/// absolute value), each ending in a newline; empty when nothing broke.
pub fn events_text(breaks: &[BreakEvent]) -> String {
    breaks.iter().map(|b| format!("break t={:.4} beam={} strain={:.6}\n", b.t, b.beam, b.strain)).collect()
}
