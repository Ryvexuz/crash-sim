//! Scripted two-car reference run: the lockstep sim and confirmed output, without transport.

use super::race::{race_fail_line, HeadingGroups};
use crash_sim::input::Script;
use crash_sim::net::confirm::{ConfirmError, ConfirmedLines};
use crash_sim::net::session::{Input, LockstepSim, MAX_INPUT_DELAY};
use crash_sim::net::INPUT_SCALE;
use crash_sim::race::lockstep::{LockstepRace, LockstepSetup, TickEvent};
use crash_sim::track::{format_seconds, LapEvent, Track};
use crash_sim::Config;
use std::collections::BTreeMap;

const USAGE: &str = "usage: crash-sim --body <car.bsn> --track <file> --grid-gap <m> --duration <s> --input <file> --input2 <file> --net-delay <ticks> --net-hash-every <ticks>";
const REQUIRED: [&str; 8] = [
    "--body",
    "--track",
    "--grid-gap",
    "--duration",
    "--input",
    "--input2",
    "--net-delay",
    "--net-hash-every",
];

pub fn wants_reference(args: &[String]) -> bool {
    args.iter()
        .any(|a| matches!(a.as_str(), "--input2" | "--net-delay" | "--net-hash-every"))
}

struct Flags<'a> {
    body: &'a str,
    track: &'a str,
    inputs: [&'a str; 2],
    gap: f64,
    ticks: u64,
    delay: u64,
    hash_every: u64,
    cfg: Config,
}

fn positive(flag: &str, text: &str) -> Result<f64, String> {
    text.parse::<f64>()
        .ok()
        .filter(|v| v.is_finite() && *v > 0.0)
        .ok_or_else(|| format!("{flag}: must be finite and above zero"))
}

fn integer(flag: &str, text: &str) -> Result<u64, String> {
    if text.is_empty()
        || !text.bytes().all(|b| b.is_ascii_digit())
        || (text.len() > 1 && text.starts_with('0'))
    {
        return Err(format!("{flag}: expected a canonical decimal integer"));
    }
    text.parse()
        .map_err(|_| format!("{flag}: integer is out of range"))
}

impl<'a> Flags<'a> {
    fn parse(args: &'a [String]) -> Result<Self, String> {
        let mut values = BTreeMap::new();
        let mut iter = args.iter();
        while let Some(flag) = iter.next() {
            let flag = flag.as_str();
            if !REQUIRED.contains(&flag) {
                return Err(format!("{flag}: unknown or incompatible flag"));
            }
            if values.contains_key(flag) {
                return Err(format!("{flag}: repeated flag"));
            }
            let value = iter
                .next()
                .filter(|v| !v.starts_with("--"))
                .ok_or_else(|| format!("{flag}: needs a value"))?;
            if value.is_empty()
                || value
                    .chars()
                    .any(|c| c.is_control() || matches!(c, '\u{2028}' | '\u{2029}'))
            {
                return Err(format!(
                    "{flag}: empty value or forbidden line/control character"
                ));
            }
            values.insert(flag, value.as_str());
        }
        for flag in REQUIRED {
            if !values.contains_key(flag) {
                return Err(format!("{flag}: required flag is missing"));
            }
        }
        let cfg = Config::default();
        let gap = positive("--grid-gap", values["--grid-gap"])?;
        let duration = positive("--duration", values["--duration"])?;
        let delay = integer("--net-delay", values["--net-delay"])?;
        if delay > MAX_INPUT_DELAY {
            return Err(format!("--net-delay: expected 0 to {MAX_INPUT_DELAY}"));
        }
        let hash_every = integer("--net-hash-every", values["--net-hash-every"])?;
        if hash_every == 0 {
            return Err("--net-hash-every: expected at least 1".into());
        }
        let ticks = duration_ticks(duration, &cfg, hash_every)?;
        Ok(Self {
            body: values["--body"],
            track: values["--track"],
            inputs: [values["--input"], values["--input2"]],
            gap,
            ticks,
            delay,
            hash_every,
            cfg,
        })
    }
}

pub(crate) fn duration_ticks(duration: f64, cfg: &Config, hash_every: u64) -> Result<u64, String> {
    let count = duration / cfg.dt;
    let rounded = count.round();
    // The strict upper bound avoids saturating float-to-u64 conversion at 2^64.
    let duration_error = || {
        format!(
            "--duration: must be a whole number of hash intervals ({hash_every} ticks of {} s)",
            cfg.dt
        )
    };
    if !rounded.is_finite() || !(1.0..u64::MAX as f64).contains(&rounded) {
        return Err(duration_error());
    }
    let ticks = rounded as u64;
    if (count - ticks as f64).abs() > 1e-6 || !ticks.is_multiple_of(hash_every) {
        return Err(duration_error());
    }
    Ok(ticks)
}

pub(crate) enum Error {
    Usage(String),
    Setup(String),
    Internal(String),
}

impl From<ConfirmError> for Error {
    fn from(error: ConfirmError) -> Self {
        Self::Internal(error.to_string())
    }
}

pub fn run_reference_cli(args: &[String]) -> i32 {
    match Flags::parse(args)
        .map_err(Error::Usage)
        .and_then(|f| execute(&f))
    {
        Ok((text, code)) => {
            super::emit(&text);
            code
        }
        Err(Error::Usage(e)) => {
            eprintln!("crash-sim: {e}\n{USAGE}");
            2
        }
        Err(Error::Setup(e)) => {
            eprintln!("crash-sim: {e}");
            2
        }
        Err(Error::Internal(e)) => {
            eprintln!("crash-sim: internal: {e}");
            1
        }
    }
}

fn read(flag: &str, path: &str) -> Result<String, String> {
    std::fs::read_to_string(path).map_err(|e| format!("{flag} {path}: {e}"))
}

fn thousandths(v: f64) -> Result<i32, &'static str> {
    let scale = f64::from(INPUT_SCALE);
    if !v.is_finite() || !(-1.0..=1.0).contains(&v) {
        return Err("control out of range");
    }
    let scaled = v * scale;
    let m = scaled.round();
    // The tolerance alone can admit a different control, including tiny subnormals.
    if (scaled - m).abs() > 1e-6 || m / scale != v {
        return Err("more than 3 decimals");
    }
    Ok(m as i32)
}

pub(crate) fn script(flag: &str, path: &str) -> Result<Vec<(f64, Input)>, String> {
    let parsed = Script::parse(&read(flag, path)?).map_err(|e| format!("{flag} {path}: {e}"))?;
    parsed
        .lines()
        .iter()
        .map(|line| {
            let convert =
                |v| thousandths(v).map_err(|e| format!("{path}: input line {}: {e}", line.number));
            let input = Input {
                throttle: convert(line.throttle)?,
                brake: convert(line.brake)?,
                steer: convert(line.steer)?,
            };
            if !input.in_range() {
                return Err(format!(
                    "{path}: input line {}: control out of range",
                    line.number
                ));
            }
            Ok((line.t, input))
        })
        .collect()
}

pub(crate) fn input_at(script: &[(f64, Input)], tick: u64, delay: u64, dt: f64, tol: f64) -> Input {
    if tick < delay {
        return Input::NEUTRAL;
    }
    let t = (tick - delay) as f64 * dt + tol;
    let n = script.partition_point(|(time, _)| *time <= t);
    script
        .get(n.saturating_sub(1))
        .map_or(Input::NEUTRAL, |(_, input)| *input)
}

pub(crate) fn event_line(event: &TickEvent) -> Result<(f64, String), Error> {
    let (t, line) = match &event.event {
        LapEvent::Start { t } => (*t, format!("start t={t:.4}")),
        LapEvent::Checkpoint { lap, k, t } => (*t, format!("checkpoint lap={lap} k={k} t={t:.4}")),
        LapEvent::Lap(rec) => {
            let stats = event
                .lap_stats
                .as_ref()
                .ok_or_else(|| Error::Internal("missing lap statistics".into()))?;
            (
                rec.end_t,
                format!(
                    "lap n={} start_t={:.4} end_t={:.4} time_s={} valid={} reason={} path_m={:.2} max_speed_kmh={:.1} off_track_s={:.3}",
                    rec.n,
                    rec.start_t,
                    rec.end_t,
                    format_seconds(rec.time_ms),
                    if rec.valid() { "yes" } else { "no" },
                    rec.invalid.map_or("none", |r| r.as_str()),
                    stats.path_m,
                    stats.max_speed_mps * 3.6,
                    rec.off_track_s,
                ),
            )
        }
    };
    Ok((t, format!("car {} {line}", event.car)))
}

fn execute(flags: &Flags<'_>) -> Result<(String, i32), Error> {
    let body = crash_sim::format::parse(&read("--body", flags.body).map_err(Error::Setup)?)
        .map_err(|e| Error::Setup(format!("{}: {e}", flags.body)))?;
    let track = Track::parse(&read("--track", flags.track).map_err(Error::Setup)?)
        .map_err(|e| Error::Setup(format!("{}: {e}", flags.track)))?;
    let script1 = script("--input", flags.inputs[0]).map_err(Error::Usage)?;
    let script2 = script("--input2", flags.inputs[1]).map_err(Error::Usage)?;
    let heading = HeadingGroups::of(&body)
        .ok_or_else(|| Error::Setup("the body has no heading groups".into()))?;
    let mut race = LockstepRace::new(&LockstepSetup {
        cfg: &flags.cfg,
        body: &body,
        track: &track,
        grid_gap_m: flags.gap,
    })
    .map_err(|e| Error::Setup(e.to_string()))?;
    let mut confirmed = ConfirmedLines::new(flags.hash_every)?;
    let mut out = String::new();
    let dt = flags.cfg.dt;
    let tol = dt / f64::from(flags.cfg.substeps) * 1e-6;
    for tick in 0..flags.ticks {
        race.step(
            tick,
            [
                input_at(&script1, tick, flags.delay, dt, tol),
                input_at(&script2, tick, flags.delay, dt, tol),
            ],
        );
        if let Some(failure) = race.failure() {
            confirmed.discard();
            out.push_str(&race_fail_line(tick as f64 * dt, &format!("{failure:?}")));
            return Ok((out, 1));
        }
        for event in race.take_events() {
            let (t, line) = event_line(&event)?;
            confirmed.push(event.tick, t, event.car, line)?;
        }
        if (tick + 1).is_multiple_of(flags.hash_every) {
            for line in confirmed.confirm(tick, race.state_hash())? {
                out.push_str(&line);
                out.push('\n');
            }
        }
    }
    for car in 0..2 {
        out.push_str(&final_line(&heading, &race, car, flags.ticks as f64 * dt));
    }
    out.push_str(&cars_line(&race));
    Ok((out, 0))
}

pub(crate) fn final_line(heading: &HeadingGroups, race: &LockstepRace, car: usize, t: f64) -> String {
    format!(
        "car {} {}",
        car + 1,
        heading.final_state(race.multi().sim(car), t).render()
    )
}

pub(crate) fn cars_line(race: &LockstepRace) -> String {
    match race.gap() {
        Some(gap) => format!(
            "cars gap_min_m={:.2} gap_min_t={:.4} near_substeps={}\n",
            gap.gap_min_m, gap.gap_min_t, gap.near_substeps
        ),
        None => "cars gap_min_m=none gap_min_t=none near_substeps=0\n".into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn delayed_input_rule() {
        let neutral = Input::NEUTRAL;
        let drive = Input {
            throttle: 1000,
            ..neutral
        };
        let brake = Input {
            brake: 1000,
            steer: -36,
            ..neutral
        };
        let script = [(0.0, neutral), (0.05, drive), (0.1, brake)];
        for tick in 0..=16 {
            let expected = match tick {
                0..=10 => neutral,
                11..=15 => drive,
                _ => brake,
            };
            assert_eq!(
                input_at(&script, tick, 6, 0.01, 1e-9),
                expected,
                "tick={tick}"
            );
        }
        // A non-neutral first line proves the first D ticks are neutral, not script time zero.
        let immediate = [(0.0, drive)];
        for tick in 0..6 {
            assert_eq!(input_at(&immediate, tick, 6, 0.01, 1e-9), neutral);
        }
        assert_eq!(input_at(&immediate, 6, 6, 0.01, 1e-9), drive);
        for (tick, expected) in [(4, neutral), (5, drive), (10, brake)] {
            assert_eq!(input_at(&script, tick, 0, 0.01, 1e-9), expected);
        }
    }

    #[test]
    fn script_values_to_thousandths() {
        for (v, expected) in [
            (0.036, 36),
            (-0.036, -36),
            (1.0, 1000),
            (0.48, 480),
            (0.0, 0),
        ] {
            let m = thousandths(v).unwrap();
            assert_eq!(m, expected);
            assert_eq!(f64::from(m) / 1000.0, v);
        }
        for m in -INPUT_SCALE..=INPUT_SCALE {
            assert_eq!(thousandths(f64::from(m) / f64::from(INPUT_SCALE)), Ok(m));
        }
        for v in [0.0005, 0.1234, f64::from_bits(1), 0.036 + f64::EPSILON] {
            assert_eq!(thousandths(v), Err("more than 3 decimals"));
        }
        for v in [f64::NAN, f64::INFINITY, -f64::INFINITY, f64::MAX] {
            assert_eq!(thousandths(v), Err("control out of range"));
        }
    }
}
