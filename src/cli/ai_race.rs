//! Command-line presentation of the library AI race; physics and rules stay there.

use super::race::{race_fail_line, HeadingGroups};
use crash_sim::ai_params::{self, AiParams};
use crash_sim::multi_car::CarControls;
use crash_sim::race::{self, RaceEnd, RaceError, RaceFailure, RaceResult, RaceSetup};
use crash_sim::speed_profile::{self, ProfileError};
use crash_sim::track::{format_seconds, LapEvent, Track};
use crash_sim::vec3::Vec3;
use crash_sim::Config;
use std::collections::BTreeMap;
use std::path::Path;

pub(super) const USAGE: &str = "usage: crash-sim --body <car.bsn> --track <file> --ai <n> --laps <N> --time-limit <s> --grid-gap <m> --ai-params <file> [--save-inputs <prefix>] [--substeps <n>]";

struct Flags<'a> {
    body: &'a str,
    track: &'a str,
    params: &'a str,
    cars: usize,
    laps: u32,
    time_limit: f64,
    grid_gap: f64,
    save: Option<&'a str>,
    cfg: Config,
}

fn integer<T: std::str::FromStr>(flag: &str, text: &str) -> Result<T, String> {
    if text.is_empty() || !text.bytes().all(|b| b.is_ascii_digit()) {
        return Err(format!("{flag}: expected a positive integer"));
    }
    text.parse()
        .map_err(|_| format!("{flag}: integer is out of range"))
}

fn positive(flag: &str, text: &str) -> Result<f64, String> {
    text.parse::<f64>()
        .ok()
        .filter(|v| v.is_finite() && *v > 0.0)
        .ok_or_else(|| format!("{flag}: must be finite and above zero"))
}

impl<'a> Flags<'a> {
    fn parse(args: &'a [String]) -> Result<Self, String> {
        const REQUIRED: [&str; 7] = [
            "--body",
            "--track",
            "--ai",
            "--laps",
            "--time-limit",
            "--grid-gap",
            "--ai-params",
        ];
        let mut values = BTreeMap::new();
        let mut iter = args.iter();
        while let Some(flag) = iter.next() {
            let flag = flag.as_str();
            if !REQUIRED.contains(&flag) && !["--save-inputs", "--substeps"].contains(&flag) {
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
        // The required keys were checked immediately above.
        let cars = integer::<usize>("--ai", values["--ai"])?;
        let laps = integer::<u32>("--laps", values["--laps"])?;
        if !(1..=race::MAX_CARS).contains(&cars) {
            return Err(format!("--ai: expected 1 to {}", race::MAX_CARS));
        }
        if !(1..=race::MAX_LAPS).contains(&laps) {
            return Err(format!("--laps: expected 1 to {}", race::MAX_LAPS));
        }
        let mut cfg = Config::default();
        if let Some(text) = values.get("--substeps") {
            cfg.substeps = integer("--substeps", text)?;
            if cfg.substeps == 0 {
                return Err("--substeps: expected at least 1".into());
            }
        }
        Ok(Self {
            body: values["--body"],
            track: values["--track"],
            params: values["--ai-params"],
            cars,
            laps,
            time_limit: positive("--time-limit", values["--time-limit"])?,
            grid_gap: positive("--grid-gap", values["--grid-gap"])?,
            save: values.get("--save-inputs").copied(),
            cfg,
        })
    }
}

pub(super) fn run_ai_cli(args: &[String]) -> i32 {
    let flags = match Flags::parse(args) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("crash-sim: {e}\n{USAGE}");
            return 2;
        }
    };
    match execute(&flags) {
        Ok((text, code)) => {
            super::emit(&text);
            code
        }
        Err(e) => {
            eprintln!("crash-sim: {e}");
            2
        }
    }
}

fn read(flag: &str, path: &str) -> Result<String, String> {
    std::fs::read_to_string(path).map_err(|e| format!("{flag} {path}: {e}"))
}

fn race_error(path: &str, params: &AiParams, error: RaceError) -> String {
    let key = match &error {
        RaceError::Driver(key) | RaceError::Profile(ProfileError::BadParameter(key)) => Some(*key),
        _ => None,
    };
    match key.and_then(|k| params.line_of(k)) {
        Some(line) => format!("{path}: line {line}: {error}"),
        None => error.to_string(),
    }
}

fn stem(path: &str) -> String {
    Path::new(path)
        .file_stem()
        .map_or_else(String::new, |s| s.to_string_lossy().into_owned())
}

fn guard_save_inputs(flags: &Flags<'_>) -> Result<(), String> {
    let Some(prefix) = flags.save else {
        return Ok(());
    };
    // Canonical paths resolve aliases and symlinks, but not hard links (out of scope).
    for slot in 1..=flags.cars {
        let path = format!("{prefix}{slot}.input");
        let Ok(save) = std::fs::canonicalize(&path) else {
            continue;
        };
        for (flag, input) in [
            ("--body", flags.body),
            ("--track", flags.track),
            ("--ai-params", flags.params),
        ] {
            let source = std::fs::canonicalize(input).map_err(|e| {
                format!("--save-inputs {path}: cannot resolve {flag} {input}: {e}")
            })?;
            if save == source {
                return Err(format!(
                    "--save-inputs {path}: is the {flag} file {input}; refusing to overwrite an input"
                ));
            }
        }
    }
    Ok(())
}

fn execute(flags: &Flags<'_>) -> Result<(String, i32), String> {
    let body = crash_sim::format::parse(&read("--body", flags.body)?)
        .map_err(|e| format!("{}: {e}", flags.body))?;
    let heading = HeadingGroups::of(&body)
        .filter(|_| !body.wheels.is_empty())
        .ok_or("--ai needs --body with wheels and `front` and `rear` groups")?;
    let track = Track::parse(&read("--track", flags.track)?)
        .map_err(|e| format!("{}: {e}", flags.track))?;
    let params = ai_params::parse(&read("--ai-params", flags.params)?)
        .map_err(|e| format!("{}: {e}", flags.params))?;
    guard_save_inputs(flags)?;
    let setup = RaceSetup {
        cfg: &flags.cfg,
        body: &body,
        track: &track,
        cars: flags.cars,
        laps: flags.laps,
        time_limit_s: flags.time_limit,
        grid_gap_m: flags.grid_gap,
        driver: params.driver,
        profile: params.profile,
    };
    // Refuse an unrepresentable profile summary before starting physics or writing files.
    let profile = profile_line(&track, &params, flags.params)?;
    let result = race::run(&setup).map_err(|e| race_error(flags.params, &params, e))?;
    let mut text = format!(
        "race track={} cars={} laps={} time_limit_s={:.3} grid_gap_m={:.2} ai_params={}\n",
        stem(flags.track),
        flags.cars,
        flags.laps,
        flags.time_limit,
        flags.grid_gap,
        stem(flags.params),
    );
    text.push_str(&profile);
    let code = race_lines(&mut text, &result, &heading, flags.laps)?;
    if let Some(prefix) = flags.save {
        // Build every script before writing any file, and write before emitting stdout.
        let scripts = result
            .cars
            .iter()
            .map(|car| {
                Ok((
                    format!("{prefix}{}.input", car.slot),
                    script(&result, &car.inputs)?,
                ))
            })
            .collect::<Result<Vec<_>, String>>()?;
        for (path, contents) in scripts {
            std::fs::write(&path, contents).map_err(|e| format!("--save-inputs {path}: {e}"))?;
        }
    }
    Ok((text, code))
}

fn profile_line(track: &Track, params: &AiParams, path: &str) -> Result<String, String> {
    let points: Vec<_> = track
        .points
        .iter()
        .map(|p| Vec3::new(p[0], 0.0, p[1]))
        .collect();
    let p = params.profile;
    let target = speed_profile::profile(&points, p.mu, p.g, p.a_accel, p.a_brake, p.v_max)
        .map_err(|e| race_error(path, params, RaceError::Profile(e)))?;
    let mut lap = 0.0;
    let mut min = f64::INFINITY;
    let mut max = 0.0_f64;
    for (i, &v) in target.iter().enumerate() {
        if !v.is_finite() || v <= 0.0 {
            return Err("profile: target speed must be finite and above zero".into());
        }
        lap += (points[(i + 1) % points.len()] - points[i]).length() / v;
        min = min.min(v * 3.6);
        max = max.max(v * 3.6);
    }
    // File values can be finite yet overflow/underflow in the derived summary.
    if !lap.is_finite() || lap <= 0.0 || !min.is_finite() || !max.is_finite() {
        return Err("profile: non-finite or zero summary".into());
    }
    Ok(format!(
        "profile lap_s={lap:.3} v_min_kmh={min:.1} v_max_kmh={max:.1}\n"
    ))
}

fn race_lines(
    out: &mut String,
    result: &RaceResult,
    heading: &HeadingGroups,
    laps: u32,
) -> Result<i32, String> {
    let end_t = result.substeps as f64 * result.h;
    let mut places: Vec<_> = result
        .cars
        .iter()
        .filter_map(|car| car.finish_t.map(|t| (t, car.slot)))
        .collect();
    places.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
    for (k, car) in result.cars.iter().enumerate() {
        let slot = car.slot;
        let mut stats = car.lap_stats.iter();
        for event in &car.events {
            let line = match event {
                LapEvent::Start { t } => format!("start t={t:.4}\n"),
                LapEvent::Checkpoint { lap, k, t } => {
                    format!("checkpoint lap={lap} k={k} t={t:.4}\n")
                }
                LapEvent::Lap(rec) => {
                    let done = stats.next().ok_or("missing lap statistics")?;
                    format!(
                        "lap n={} start_t={:.4} end_t={:.4} time_s={} valid={} reason={} path_m={:.2} max_speed_kmh={:.1} off_track_s={:.3}\n",
                        rec.n,
                        rec.start_t,
                        rec.end_t,
                        format_seconds(rec.time_ms),
                        if rec.valid() { "yes" } else { "no" },
                        rec.invalid.map_or("none", |r| r.as_str()),
                        done.path_m,
                        done.max_speed_mps * 3.6,
                        rec.off_track_s,
                    )
                }
            };
            out.push_str(&format!("car {slot} {line}"));
        }
        match car.finish_t {
            Some(t) => {
                let place = places
                    .iter()
                    .position(|&(_, s)| s == slot)
                    .ok_or("missing finish place")?
                    + 1;
                out.push_str(&format!("car {slot} finish t={t:.4} place={place}\n"));
            }
            None => out.push_str(&format!("car {slot} finish t=none place=none\n")),
        }
        let first = car
            .first_wall_t
            .map_or_else(|| "none".into(), |t| format!("{t:.4}"));
        out.push_str(&format!(
            "car {slot} walls substeps={} first_t={first}\n",
            car.wall_substeps
        ));
        out.push_str(&format!(
            "car {slot} {}",
            heading.final_state(result.multi.sim(k), end_t).render()
        ));
    }
    let near = match result.gap {
        Some(gap) => {
            out.push_str(&format!(
                "cars gap_min_m={:.2} gap_min_t={:.4} near_substeps={}\n",
                gap.gap_min_m, gap.gap_min_t, gap.near_substeps
            ));
            gap.near_substeps
        }
        None => {
            out.push_str("cars gap_min_m=none gap_min_t=none near_substeps=0\n");
            0
        }
    };
    if let RaceEnd::Failed(failure) = &result.end {
        let (t, reason) = match failure {
            RaceFailure::Sim { t, error } => (*t, error.to_string()),
            RaceFailure::Timer { t, error, .. } => (*t, error.to_string()),
        };
        out.push_str(&race_fail_line(t, &reason));
        return Ok(1);
    }
    let cars = result.cars.len();
    let finished = result
        .cars
        .iter()
        .filter(|car| car.laps.len() >= laps as usize)
        .count();
    let walls: u64 = result.cars.iter().map(|car| car.wall_substeps).sum();
    if race_ok(finished, cars, walls, near) {
        out.push_str(&format!(
            "race: ok cars={cars} finished={finished} wall_substeps=0 near_substeps=0 end_t={end_t:.4}\n"
        ));
        return Ok(0);
    }
    out.push_str("race: FAIL");
    if finished != cars {
        out.push_str(&format!(" finished={finished}/{cars}"));
    }
    if walls != 0 {
        out.push_str(&format!(" wall_substeps={walls}"));
    }
    if near != 0 {
        out.push_str(&format!(" near_substeps={near}"));
    }
    out.push_str(&format!(" end_t={end_t:.4}\n"));
    Ok(1)
}

// Milliseconds per driver interval, fixed by the input protocol (CONTROL_DT = 0.02).
const CONTROL_MS: u64 = 20;

fn script(result: &RaceResult, inputs: &[CarControls]) -> Result<String, String> {
    let mut text = String::new();
    let mut last_ms = None;
    let line = |ms: u64, c: &CarControls| {
        format!(
            "{}.{:03} {} {} {}\n",
            ms / 1000,
            ms % 1000,
            c.throttle,
            c.brake,
            c.steer
        )
    };
    for (j, c) in inputs.iter().enumerate() {
        let ms = u64::try_from(j)
            .ok()
            .and_then(|j| j.checked_mul(CONTROL_MS))
            .ok_or("input script timestamp overflow")?;
        text.push_str(&line(ms, c));
        last_ms = Some(ms);
    }
    if result.substeps.is_multiple_of(result.control_substeps) {
        let end_ms = (result.substeps / result.control_substeps)
            .checked_mul(CONTROL_MS)
            .ok_or("input script end timestamp overflow")?;
        if last_ms != Some(end_ms) {
            if let Some(c) = inputs.last() {
                text.push_str(&line(end_ms, c));
            }
        }
    }
    Ok(text)
}

/// The `race: ok` rule: every car finished, no wall substep and no near substep. Every other result is a FAIL.
fn race_ok(finished: usize, cars: usize, walls: u64, near: u64) -> bool {
    finished == cars && walls == 0 && near == 0
}

#[cfg(test)]
mod tests {
    use super::race_ok;

    #[test]
    fn race_ok_needs_every_rule() {
        assert!(race_ok(2, 2, 0, 0));
        assert!(!race_ok(1, 2, 0, 0), "a car did not finish");
        assert!(!race_ok(2, 2, 1, 0), "a wall substep");
        assert!(!race_ok(2, 2, 0, 1), "all finished, no walls, a near substep");
    }
}
