//! Driving a car from an input script (docs/STAGE6.md, milestone 3: "The drive report" and
//! checks 3.4 to 3.8), and the report of such a drive.
//!
//! The script is a text file of lines `t throttle brake steer`: from time `t` (seconds of
//! simulation) the three values hold, until the next line. Brake is 0 to 1, throttle is -1 to 1
//! (negative is reverse, see `vehicle::REVERSE_MAX_FORWARD_SPEED`), steer is -1 to 1 (positive
//! turns toward the car's right, +z of a car that faces +x); they are
//! fractions of the car file's `controls` (maximum drive torque, brake torque and steer angle).
//! The file rules, each broken one an error that names the line:
//!
//! - only `\n` line ends (a `\r` is an error, because a replay must read what the writer wrote);
//! - blank lines (spaces only) are skipped, and every other line has exactly four finite numbers;
//! - the first line is at t = 0 (nothing is defined before it) and times strictly increase;
//! - throttle in -1..=1, brake in 0..=1, steer in -1..=1;
//! - a file with no line is an error.
//!
//! A line takes effect at the first substep that starts at or after its time, so the same file
//! gives the same run whatever the output step is. Everything here is deterministic.

use crate::format::Body;
use crate::settle::{car_sim, run_substeps, CarSetup};
use crate::trajectory;
use crate::vec3::Vec3;
use crate::{Config, Sim};

/// One line of a script, with its line number in the file (blank lines count).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Line {
    pub number: usize,
    pub t: f64,
    pub throttle: f64,
    pub brake: f64,
    pub steer: f64,
}

/// A parsed input script: at least one line, times strictly increasing from 0.
#[derive(Clone, Debug, PartialEq)]
pub struct Script {
    lines: Vec<Line>,
}

/// The one parser of input lines, fed a line at a time: `Script::parse` (a file) and `live` (a
/// pipe) both use it, so a line is accepted or refused the same way in both. It holds what the
/// rules need from the lines before: the line number and the previous time.
#[derive(Clone, Debug, Default)]
pub struct LineParser {
    /// Lines fed so far (blank lines count): the number of the next line is this plus 1.
    fed: usize,
    last_t: Option<f64>,
}

impl LineParser {
    pub fn new() -> LineParser {
        LineParser::default()
    }

    /// Feed the next line, without its `\n`. `Ok(None)` for a blank line; the error names the line.
    pub fn feed(&mut self, raw: &str) -> Result<Option<Line>, String> {
        self.fed += 1;
        let number = self.fed;
        if raw.contains('\r') {
            return Err(format!(
                "input line {number}: a carriage return is not allowed (lines end with \\n only)"
            ));
        }
        if raw.trim().is_empty() {
            return Ok(None);
        }
        let f: Vec<&str> = raw.split_whitespace().collect();
        if f.len() != 4 {
            return Err(format!("input line {number}: a line is `t throttle brake steer`, found {} fields in `{raw}`", f.len()));
        }
        let mut v = [0.0_f64; 4];
        for (k, x) in f.iter().enumerate() {
            v[k] = x
                .parse::<f64>()
                .ok()
                .filter(|n| n.is_finite())
                .ok_or_else(|| format!("input line {number}: `{x}` is not a finite number"))?;
        }
        let [t, throttle, brake, steer] = v;
        if t < 0.0 {
            return Err(format!("input line {number}: time {t} is below 0"));
        }
        match self.last_t {
            None if t != 0.0 => {
                return Err(format!(
                    "input line {number}: the first line must be at t = 0, found t = {t}"
                ));
            }
            Some(prev) if t <= prev => {
                return Err(format!("input line {number}: time {t} is not above the previous line's time {prev} (times must strictly increase)"));
            }
            _ => {}
        }
        if !(-1.0..=1.0).contains(&throttle) {
            return Err(format!(
                "input line {number}: throttle {throttle} must be from -1 to 1"
            ));
        }
        if !(0.0..=1.0).contains(&brake) {
            return Err(format!(
                "input line {number}: brake {brake} must be from 0 to 1"
            ));
        }
        if !(-1.0..=1.0).contains(&steer) {
            return Err(format!(
                "input line {number}: steer {steer} must be from -1 to 1"
            ));
        }
        self.last_t = Some(t);
        Ok(Some(Line {
            number,
            t,
            throttle,
            brake,
            steer,
        }))
    }
}

impl Script {
    /// Parse `text`. The error names the line (`input line N: ...`).
    pub fn parse(text: &str) -> Result<Script, String> {
        let mut lines: Vec<Line> = Vec::new();
        let mut parser = LineParser::new();
        let mut pieces: Vec<&str> = text.split('\n').collect();
        // `a\nb\n` splits into a, b and an empty last piece that is not a line.
        if pieces.last() == Some(&"") {
            pieces.pop();
        }
        for raw in pieces {
            if let Some(line) = parser.feed(raw)? {
                lines.push(line);
            }
        }
        if lines.is_empty() {
            return Err("input: the file has no line (a script needs at least `0 <throttle> <brake> <steer>`)".into());
        }
        Ok(Script { lines })
    }

    /// The lines, in file order.
    pub fn lines(&self) -> &[Line] {
        &self.lines
    }

    /// The time of the last line (s): where a run with no `--duration` ends.
    pub fn t_last(&self) -> f64 {
        self.lines[self.lines.len() - 1].t
    }

    /// The line in force at time `t` (s): the last one with time <= t, within `tol`.
    pub fn line_at(&self, t: f64, tol: f64) -> &Line {
        let n = self.lines.partition_point(|l| l.t <= t + tol);
        &self.lines[n.saturating_sub(1)]
    }

    /// Check every line against the car (a throttle line needs a driven wheel, and so on), so a
    /// line the car cannot obey is an error naming it before the run, not a silent no-op.
    pub fn check_for(&self, car: &crate::vehicle::Car) -> Result<(), String> {
        let mut probe = car.clone();
        for l in &self.lines {
            probe
                .set_controls(l.throttle, l.brake, l.steer)
                .map_err(|e| format!("input line {}: {e}", l.number))?;
        }
        Ok(())
    }

    /// Put the line in force at `t` onto `sim`'s car.
    pub fn apply(&self, sim: &mut Sim, t: f64, tol: f64) -> Result<(), String> {
        let l = *self.line_at(t, tol);
        match sim.car.as_mut() {
            Some(car) => car
                .set_controls(l.throttle, l.brake, l.steer)
                .map_err(|e| format!("input line {}: {e}", l.number)),
            None => Err("internal: an input script was given to a sim with no car".into()),
        }
    }
}

/// The five thresholds of `--drive-report`. No defaults.
#[derive(Clone, Copy, Debug)]
pub struct DriveParams {
    /// `--lap-min-heading`: heading change that counts as a loop (degrees).
    pub lap_min_heading: f64,
    /// `--lap-min-path`: path length of the loop (m).
    pub lap_min_path: f64,
    /// `--lap-close`: how near the start the loop ends (m).
    pub lap_close: f64,
    /// `--impact-min-kmh`, `--impact-max-kmh`: the wall impact speed window.
    pub impact_min_kmh: f64,
    pub impact_max_kmh: f64,
}

impl DriveParams {
    pub fn validate(&self) -> Result<(), String> {
        let ok = |x: f64| x.is_finite() && x > 0.0;
        if ok(self.lap_min_heading)
            && ok(self.lap_min_path)
            && ok(self.lap_close)
            && ok(self.impact_min_kmh)
            && ok(self.impact_max_kmh)
            && self.impact_min_kmh <= self.impact_max_kmh
        {
            Ok(())
        } else {
            Err("drive thresholds must be finite and above 0, with impact-min-kmh <= impact-max-kmh".into())
        }
    }
}

/// One output-step row of a drive: simulation time, chassis centre-of-mass speed, chassis
/// centroid and heading.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DriveRow {
    pub t: f64,
    pub speed: f64,
    pub pos: Vec3,
    pub heading_deg: f64,
}

/// The header of `render_rows`.
pub const ROWS_HEADER: &str = "t,speed_mps,x,y,z,heading_deg";

/// The rows as text: the header, then one line per row, each ending in a newline.
pub fn render_rows(rows: &[DriveRow]) -> String {
    let mut s = String::from(ROWS_HEADER);
    s.push('\n');
    for r in rows {
        s.push_str(&render_row(r));
    }
    s
}

/// One row as text, ending in a newline: the only place a row is formatted, so a file run and a
/// live run print the same bytes.
pub fn render_row(r: &DriveRow) -> String {
    format!(
        "{:.4},{:.4},{:.4},{:.4},{:.4},{:.2}\n",
        r.t, r.speed, r.pos.x, r.pos.y, r.pos.z, r.heading_deg
    )
}

/// What a drive measured. The fields are the lines of the drive report.
#[derive(Clone, Debug, PartialEq)]
pub struct DriveMeasure {
    /// Largest |heading change| from the start, before the first wall contact (degrees).
    pub lap_heading_deg: f64,
    /// Path length of the chassis centroid up to the substep where |heading change| first
    /// reached `lap_min_heading`, or None.
    pub lap_path_m: Option<f64>,
    /// Horizontal distance of the chassis centroid from its start at that substep, or None.
    pub lap_close_m: Option<f64>,
    /// Substeps before the first wall contact in which a chassis node was at or below the ground.
    pub chassis_ground_contacts_before_wall: u64,
    /// First substep (its end time, s) in which a chassis node or a tyre touched a wall.
    pub wall_first_contact_s: Option<f64>,
    /// Chassis centre-of-mass speed along the wall normal, toward the wall, at the start of that
    /// substep (km/h): the approach speed, before the contact rule reverses any node.
    pub wall_impact_speed_kmh: Option<f64>,
}

/// The state at the last substep: the `final` line.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FinalState {
    pub t: f64,
    pub x: f64,
    pub z: f64,
    pub heading_deg: f64,
}

impl FinalState {
    /// `final t=<4 decimals> x=<4 decimals> z=<4 decimals> heading_deg=<2 decimals>`, with a newline.
    pub fn render(&self) -> String {
        format!(
            "final t={:.4} x={:.4} z={:.4} heading_deg={:.2}\n",
            self.t, self.x, self.z, self.heading_deg
        )
    }
}

/// A finished (or aborted) drive.
#[derive(Clone, Debug)]
pub struct DriveRun {
    /// One row at t = 0 and at every output step.
    pub rows: Vec<DriveRow>,
    pub measure: DriveMeasure,
    pub final_state: FinalState,
    /// Why the run stopped early (non-finite state, strain limit): `rows`, `measure` and
    /// `final_state` then describe the last good substep.
    pub abort: Option<String>,
}

impl DriveMeasure {
    /// The report of `p`: the lines then the verdict line, each ending in a newline, with the
    /// `final` line of `fin` before the verdict (docs/STAGE6.md, "The drive report").
    pub fn render(&self, p: &DriveParams, fin: &FinalState) -> String {
        let opt = |v: Option<f64>, d: usize| match v {
            Some(x) => format!("{x:.d$}"),
            None => "never".to_string(),
        };
        let wall = match self.wall_first_contact_s {
            Some(t) => format!("{t:.4}"),
            None => "none".to_string(),
        };
        let impact = match self.wall_impact_speed_kmh {
            Some(v) => format!("{v:.1}"),
            None => "none".to_string(),
        };
        let verdict = match self.verdict(p) {
            Ok(()) => "drive: ok".to_string(),
            Err(r) => format!("drive: FAIL {r}"),
        };
        format!(
            "lap_heading_deg={:.1}\nlap_path_m={}\nlap_close_m={}\nchassis_ground_contacts_before_wall={}\nwall_first_contact_s={wall}\nwall_impact_speed_kmh={impact}\n{}{verdict}\n",
            self.lap_heading_deg,
            opt(self.lap_path_m, 2),
            opt(self.lap_close_m, 3),
            self.chassis_ground_contacts_before_wall,
            fin.render(),
        )
    }

    /// `drive: ok` iff the heading, path, closing, ground-contact and impact rules all hold;
    /// otherwise the first rule that fails, with its numbers.
    pub fn verdict(&self, p: &DriveParams) -> Result<(), String> {
        if self.lap_heading_deg < p.lap_min_heading {
            return Err(format!(
                "lap_heading_deg {:.1} < lap-min-heading {}",
                self.lap_heading_deg, p.lap_min_heading
            ));
        }
        match self.lap_path_m {
            Some(x) if x >= p.lap_min_path => {}
            Some(x) => {
                return Err(format!(
                    "lap_path_m {x:.2} < lap-min-path {}",
                    p.lap_min_path
                ))
            }
            None => {
                return Err(format!(
                    "lap_path_m never: the heading never reached lap-min-heading {}",
                    p.lap_min_heading
                ))
            }
        }
        match self.lap_close_m {
            Some(x) if x <= p.lap_close => {}
            Some(x) => return Err(format!("lap_close_m {x:.3} > lap-close {}", p.lap_close)),
            None => return Err("lap_close_m never".into()),
        }
        if self.chassis_ground_contacts_before_wall != 0 {
            return Err(format!(
                "chassis_ground_contacts_before_wall {} != 0",
                self.chassis_ground_contacts_before_wall
            ));
        }
        match self.wall_impact_speed_kmh {
            Some(v) if v >= p.impact_min_kmh && v <= p.impact_max_kmh => Ok(()),
            Some(v) => Err(format!(
                "wall_impact_speed_kmh {v:.1} outside {} to {}",
                p.impact_min_kmh, p.impact_max_kmh
            )),
            None => Err("no wall contact, so no impact speed".into()),
        }
    }
}

/// Indices (into the sim's nodes) of the nodes of group `name`.
fn group_indices(body: &Body, name: &str) -> Result<Vec<usize>, String> {
    let g = body
        .groups
        .iter()
        .find(|g| g.name == name)
        .ok_or_else(|| format!("the body has no group `{name}`, which the heading of a drive needs (docs/STAGE6.md, \"The drive report\")"))?;
    g.nodes
        .iter()
        .map(|id| {
            body.node_index(*id)
                .ok_or_else(|| format!("group `{name}` names a missing node {id}"))
        })
        .collect()
}

fn mean(pos: &[Vec3], ix: &[usize]) -> Vec3 {
    let mut c = Vec3::ZERO;
    for &i in ix {
        c += pos[i];
    }
    c * (1.0 / ix.len() as f64)
}

/// The angle (radians, -pi to pi) of the ground-plane direction of `v`: atan2(z, x), so a turn
/// toward +z (the car's right when it faces +x) raises it.
fn ground_angle(v: Vec3) -> f64 {
    v.z.atan2(v.x)
}

/// `a` wrapped into (-pi, pi].
fn wrap(a: f64) -> f64 {
    let tau = std::f64::consts::TAU;
    let mut x = a % tau;
    if x > std::f64::consts::PI {
        x -= tau;
    } else if x <= -std::f64::consts::PI {
        x += tau;
    }
    x
}

/// Chassis centre-of-mass velocity (nodes only, mass weighted).
fn chassis_com_velocity(sim: &Sim) -> Vec3 {
    let m: f64 = sim.mass.iter().sum();
    let mut v = Vec3::ZERO;
    for (vi, mi) in sim.vel.iter().zip(&sim.mass) {
        v += *vi * *mi;
    }
    v * (1.0 / m)
}

/// Heading of the car (radians): the ground-plane direction from the `rear` to the `front` centroid.
fn heading_of(sim: &Sim, front: &[usize], rear: &[usize]) -> f64 {
    ground_angle(mean(&sim.pos, front) - mean(&sim.pos, rear))
}

/// Centroid of all the sim's nodes.
fn centroid_of(sim: &Sim) -> Vec3 {
    let all: Vec<usize> = (0..sim.pos.len()).collect();
    mean(&sim.pos, &all)
}

/// The output row at time `t`: used by `drive` and `live`, so both print the same numbers.
fn row_of(sim: &Sim, front: &[usize], rear: &[usize], t: f64) -> DriveRow {
    DriveRow {
        t,
        speed: chassis_com_velocity(sim).length(),
        pos: centroid_of(sim),
        heading_deg: heading_of(sim, front, rear).to_degrees(),
    }
}

/// The `final` state at time `t`.
fn final_of(sim: &Sim, front: &[usize], rear: &[usize], t: f64) -> FinalState {
    let c = centroid_of(sim);
    FinalState {
        t,
        x: c.x,
        z: c.z,
        heading_deg: heading_of(sim, front, rear).to_degrees(),
    }
}

/// Whether the car touches wall plane `k` (any plane after the ground): a chassis node at or
/// behind it, or a tyre pressed into it. Returns the first such plane index.
fn wall_touched(sim: &Sim) -> Option<usize> {
    let car = sim.car.as_ref();
    (1..sim.planes.len()).find(|&k| {
        let pl = &sim.planes[k];
        sim.pos.iter().any(|p| pl.distance(*p) <= 0.0)
            || car.is_some_and(|c| {
                c.wheels
                    .iter()
                    .any(|w| w.p.radius - pl.distance(w.pos) > 0.0)
            })
    })
}

/// Whether `body` has the `front` and `rear` groups the heading of a drive needs.
pub fn has_heading_groups(body: &Body) -> bool {
    group_indices(body, "rear").is_ok() && group_indices(body, "front").is_ok()
}

/// Writes a car run as trajectory v2 (`src/trajectory.rs`): the header, then one frame of every node
/// per output step, each frame preceded by the break events that happened since the last one. The
/// wheels are not nodes of the body and are not in the file (v2 has no wheel lines), and there is no
/// `final` line: the file is the trajectory only, so the renderer can read it as it is. Shared by
/// `drive` (a file run) and `live`, so both write the same bytes.
pub struct FrameWriter {
    header: String,
    written: usize,
}

impl FrameWriter {
    /// The writer for `sim` (a car sim built from `body`): refuses what `trajectory::header_v2` and
    /// `walls_from_planes` refuse (a wall not along an axis, a bad beam).
    pub fn new(sim: &Sim, body: &Body) -> Result<FrameWriter, String> {
        let walls = trajectory::walls_from_planes(&body.planes)?;
        let header = trajectory::header_v2(sim.pos.len(), &sim.beams, &walls)?;
        Ok(FrameWriter { header, written: 0 })
    }

    /// The header, through the column line.
    pub fn header(&self) -> &str {
        &self.header
    }

    /// The text of the frame at time `t`: the break events not yet written, then the node rows.
    pub fn frame(&mut self, sim: &Sim, t: f64) -> String {
        let mut s = self.tail(sim);
        s.push_str(&trajectory::frame_rows(t, &sim.pos));
        s
    }

    /// The break events not yet written (those after the last frame).
    pub fn tail(&mut self, sim: &Sim) -> String {
        let mut s = String::new();
        for b in &sim.breaks[self.written..] {
            s.push_str(&trajectory::break_row(b.t, b.beam));
        }
        self.written = sim.breaks.len();
        s
    }
}

/// Run the drive of `setup` (which must carry a script) on `body` and measure it at every
/// substep. `lap_min_heading` (degrees) is the heading change at which the lap path and closing
/// distance are read; pass `f64::INFINITY` when no lap report is wanted. The run is
/// `settle::run_substeps` substeps of `cfg.dt / cfg.substeps`, one row at t = 0 and one at the
/// end of every output step.
pub fn drive(
    cfg: &Config,
    body: &Body,
    setup: &CarSetup,
    lap_min_heading: f64,
) -> Result<DriveRun, String> {
    drive_frames(cfg, body, setup, lap_min_heading, None)
}

/// `drive`, and when `frames` is given, the trajectory v2 text of the run appended to it (the
/// header, a frame at t = 0 and one per output step, the break events after the last frame). A run
/// that aborts leaves the whole frames before the abort.
pub fn drive_frames(
    cfg: &Config,
    body: &Body,
    setup: &CarSetup,
    lap_min_heading: f64,
    mut frames: Option<&mut String>,
) -> Result<DriveRun, String> {
    cfg.validate_numbers()?;
    let script = setup
        .script
        .as_ref()
        .ok_or("a drive needs an input script (--input)")?;
    let rear = group_indices(body, "rear")?;
    let front = group_indices(body, "front")?;
    let h = cfg.dt / cfg.substeps as f64;
    let total = run_substeps(cfg, setup)?;
    let mut one = cfg.clone();
    one.dt = h;
    one.substeps = 1;
    let mut sim = car_sim(one, body, setup)?;
    let tol = h * 1e-6;

    let heading = |sim: &Sim| heading_of(sim, &front, &rear);
    let centroid = centroid_of;
    let row = |sim: &Sim, t: f64| row_of(sim, &front, &rear, t);
    let fin = |sim: &Sim, t: f64| final_of(sim, &front, &rear, t);

    let mut writer = match frames.as_deref_mut() {
        Some(f) => {
            let mut w = FrameWriter::new(&sim, body)?;
            f.push_str(w.header());
            f.push_str(&w.frame(&sim, 0.0));
            Some(w)
        }
        None => None,
    };
    let start = centroid(&sim);
    let mut prev_c = start;
    let mut prev_h = heading(&sim);
    let mut unwrapped = 0.0_f64; // radians
    let mut path = 0.0_f64;
    let mut m = DriveMeasure {
        lap_heading_deg: 0.0,
        lap_path_m: None,
        lap_close_m: None,
        chassis_ground_contacts_before_wall: 0,
        wall_first_contact_s: None,
        wall_impact_speed_kmh: None,
    };
    let mut rows = vec![row(&sim, 0.0)];
    let mut last = fin(&sim, 0.0);
    let mut abort = None;
    for i in 1..=total {
        let t_start = (i - 1) as f64 * h;
        let t = i as f64 * h;
        script.apply(&mut sim, t_start, tol)?;
        // The approach speed: the contact rule reverses the nodes that touch in this substep, so
        // the speed at its end is already cut by the bounce.
        let v_before = chassis_com_velocity(&sim);
        sim.advance();
        if let Err(e) = sim.check() {
            abort = Some(format!("unstable at t = {t:.4} s: {e}"));
            break;
        }
        let c = centroid(&sim);
        let hd = heading(&sim);
        unwrapped += wrap(hd - prev_h);
        path += ((c.x - prev_c.x).powi(2) + (c.z - prev_c.z).powi(2)).sqrt();
        prev_h = hd;
        prev_c = c;
        if m.wall_first_contact_s.is_none() {
            if let Some(k) = wall_touched(&sim) {
                m.wall_first_contact_s = Some(t);
                m.wall_impact_speed_kmh = Some(-v_before.dot(sim.planes[k].normal) * 3.6);
            } else {
                // A substep before the first wall contact: the lap and ground rules count it.
                let heading_deg = unwrapped.to_degrees().abs();
                m.lap_heading_deg = m.lap_heading_deg.max(heading_deg);
                if m.lap_path_m.is_none() && heading_deg >= lap_min_heading {
                    m.lap_path_m = Some(path);
                    m.lap_close_m =
                        Some(((c.x - start.x).powi(2) + (c.z - start.z).powi(2)).sqrt());
                }
                if sim.pos.iter().any(|p| sim.planes[0].distance(*p) <= 0.0) {
                    m.chassis_ground_contacts_before_wall += 1;
                }
            }
        }
        last = fin(&sim, t);
        if i % cfg.substeps as u64 == 0 {
            rows.push(row(&sim, t));
            if let (Some(w), Some(f)) = (writer.as_mut(), frames.as_deref_mut()) {
                f.push_str(&w.frame(&sim, t));
            }
        }
    }
    if let (Some(w), Some(f)) = (writer.as_mut(), frames) {
        f.push_str(&w.tail(&sim));
    }
    Ok(DriveRun {
        rows,
        measure: m,
        final_state: last,
        abort,
    })
}

/// Why a live run ended without its `final` line.
#[derive(Debug)]
pub enum LiveError {
    /// A refused input: a bad line, the body or the car does not suit, no line at all (exit 2).
    Input(String),
    /// The sim became unstable or broke a limit: the rows so far were written (exit 1).
    Abort(String),
    /// stdout could not be written.
    Output(std::io::Error),
}

/// Read the next non-blank line from `input`: `Ok(None)` at end of input. A last line with no
/// `\n` counts as a line, as in a file. Each line goes through `parser`, the same parser a file does.
fn next_line(
    parser: &mut LineParser,
    input: &mut impl std::io::BufRead,
) -> Result<Option<Line>, LiveError> {
    loop {
        let mut buf: Vec<u8> = Vec::new();
        let n = input
            .read_until(b'\n', &mut buf)
            .map_err(|e| LiveError::Input(format!("input: cannot read: {e}")))?;
        if n == 0 {
            return Ok(None);
        }
        if buf.last() == Some(&b'\n') {
            buf.pop();
        }
        let text = String::from_utf8(buf).map_err(|_| {
            LiveError::Input(format!("input line {}: not valid UTF-8", parser.fed + 1))
        })?;
        if let Some(l) = parser.feed(&text).map_err(LiveError::Input)? {
            return Ok(Some(l));
        }
    }
}

/// The live drive (`--live`, docs/STAGE6.md "The live protocol"): lines of the input grammar are
/// read from `input` as they arrive and `emit` is called with the output text, once for the header
/// and the row at t = 0 (after the first line) and once per output step after that, then once for
/// the `final` line at end of input. The sim runs substep i only when the last time received is at
/// or after the end of that substep, so it never runs ahead of the lines, and it runs exactly the
/// substeps a file of the same lines runs (the same count as `settle::run_substeps`), so the bytes
/// are the same. `emit` is where the caller writes and flushes: its error ends the run.
pub fn live(
    cfg: &Config,
    body: &Body,
    v2: bool,
    input: &mut impl std::io::BufRead,
    emit: &mut impl FnMut(&str) -> std::io::Result<()>,
) -> Result<(), LiveError> {
    cfg.validate_numbers().map_err(LiveError::Input)?;
    let rear = group_indices(body, "rear").map_err(LiveError::Input)?;
    let front = group_indices(body, "front").map_err(LiveError::Input)?;
    let h = cfg.dt / cfg.substeps as f64;
    let tol = h * 1e-6;
    let mut parser = LineParser::new();
    let first = next_line(&mut parser, input)?.ok_or_else(|| {
        LiveError::Input(
            "input: end of input before any line (a run needs at least `0 <throttle> <brake> <steer>`)"
                .into(),
        )
    })?;
    let mut script = Script { lines: vec![first] };
    let setup = CarSetup {
        script: Some(script.clone()),
        until_script_end: false,
        grip_frac: None,
    };
    let mut one = cfg.clone();
    one.dt = h;
    one.substeps = 1;
    let mut sim = car_sim(one, body, &setup).map_err(LiveError::Input)?;
    let mut probe = sim
        .car
        .clone()
        .ok_or_else(|| LiveError::Input("internal: a live run built no car".into()))?;
    let mut writer = if v2 {
        Some(FrameWriter::new(&sim, body).map_err(LiveError::Input)?)
    } else {
        None
    };
    let mut text = String::new();
    match writer.as_mut() {
        Some(w) => {
            text.push_str(w.header());
            text.push_str(&w.frame(&sim, 0.0));
        }
        None => {
            text.push_str(ROWS_HEADER);
            text.push('\n');
            text.push_str(&render_row(&row_of(&sim, &front, &rear, 0.0)));
        }
    }
    emit(&text).map_err(LiveError::Output)?;
    let mut done: u64 = 0;
    loop {
        // The substeps whose end is at or before the last time received (the same count as
        // `settle::run_substeps` gives a file whose last line is that time).
        let allowed = (script.t_last() / h * (1.0 + 1e-9)).floor() as u64;
        while done < allowed {
            done += 1;
            let t = done as f64 * h;
            script
                .apply(&mut sim, (done - 1) as f64 * h, tol)
                .map_err(LiveError::Input)?;
            sim.advance();
            if let Err(e) = sim.check() {
                return Err(LiveError::Abort(format!("unstable at t = {t:.4} s: {e}")));
            }
            if done.is_multiple_of(cfg.substeps as u64) {
                let text = match writer.as_mut() {
                    Some(w) => w.frame(&sim, t),
                    None => render_row(&row_of(&sim, &front, &rear, t)),
                };
                emit(&text).map_err(LiveError::Output)?;
            }
        }
        match next_line(&mut parser, input)? {
            Some(l) => {
                probe
                    .set_controls(l.throttle, l.brake, l.steer)
                    .map_err(|e| LiveError::Input(format!("input line {}: {e}", l.number)))?;
                script.lines.push(l);
            }
            None => break,
        }
    }
    if done == 0 {
        return Err(LiveError::Input(format!(
            "the input ends at t={}, so the run has 0 substeps",
            script.t_last()
        )));
    }
    match writer.as_mut() {
        // v2 is the trajectory only: the break events after the last frame, no `final` line.
        Some(w) => {
            let tail = w.tail(&sim);
            if tail.is_empty() {
                return Ok(());
            }
            emit(&tail).map_err(LiveError::Output)
        }
        None => {
            let fin = final_of(&sim, &front, &rear, done as f64 * h);
            emit(&fin.render()).map_err(LiveError::Output)
        }
    }
}
