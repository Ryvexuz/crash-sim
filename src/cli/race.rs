use super::*;

// ---------------------------------------------------------------------------------------
// The timed lap (docs/STAGE7.md, "The race program")

/// How a `--track` run gets its controls.
pub(super) enum RaceMode {
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
pub(super) struct HeadingGroups {
    front: Vec<usize>,
    rear: Vec<usize>,
}

impl HeadingGroups {
    pub(super) fn of(body: &Body) -> Option<HeadingGroups> {
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
    pub(super) fn final_state(&self, sim: &Sim, t: f64) -> input::FinalState {
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
    pub(super) fn row(&self, sim: &Sim, t: f64) -> input::DriveRow {
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
pub(super) fn place_on_grid(body: &Body, track: &Track) -> Result<Body, String> {
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
pub(super) struct Race {
    pub(super) sim: Sim,
    timer: LapTimer,
    h: f64,
    /// Substeps done.
    pub(super) i: u64,
    prev: Vec3,
    stats: Option<LapStats>,
}

impl Race {
    pub(super) fn new(sim: Sim, track: &Track, h: f64) -> Race {
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
    pub(super) fn step(&mut self) -> Result<String, (f64, String)> {
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
    pub(super) fn summary(&self) -> String {
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
pub(super) fn race_fail_line(t: f64, reason: &str) -> String {
    format!("race: FAIL unstable at t={t:.4}: {reason}\n")
}

/// The `track` line.
pub(super) fn track_line(track: &Track, name: &str) -> String {
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
pub(super) fn track_draw_lines(track: &Track) -> String {
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
pub(super) fn run_race(cfg: &Config, body: &Body, track_path: &str, mode: RaceMode) -> i32 {
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

