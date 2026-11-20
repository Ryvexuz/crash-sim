//! Driving a car from an input script (docs/STAGE6.md, milestone 3: "The drive report" and
//! checks 3.4 to 3.8), and the report of such a drive.
//!
//! The script is a text file of lines `t throttle brake steer`: from time `t` (seconds of
//! simulation) the three values hold, until the next line. Throttle and brake are 0 to 1,
//! steer is -1 to 1 (positive turns toward the car's right, +z of a car that faces +x); they are
//! fractions of the car file's `controls` (maximum drive torque, brake torque and steer angle).
//! The file rules, each broken one an error that names the line:
//!
//! - only `\n` line ends (a `\r` is an error, because a replay must read what the writer wrote);
//! - blank lines (spaces only) are skipped, and every other line has exactly four finite numbers;
//! - the first line is at t = 0 (nothing is defined before it) and times strictly increase;
//! - throttle and brake in 0..=1, steer in -1..=1;
//! - a file with no line is an error.
//!
//! A line takes effect at the first substep that starts at or after its time, so the same file
//! gives the same run whatever the output step is. Everything here is deterministic.

use crate::format::Body;
use crate::settle::{car_sim, run_substeps, CarSetup};
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

impl Script {
    /// Parse `text`. The error names the line (`input line N: ...`).
    pub fn parse(text: &str) -> Result<Script, String> {
        let mut lines: Vec<Line> = Vec::new();
        let mut pieces: Vec<&str> = text.split('\n').collect();
        // `a\nb\n` splits into a, b and an empty last piece that is not a line.
        if pieces.last() == Some(&"") {
            pieces.pop();
        }
        for (i, raw) in pieces.iter().enumerate() {
            let number = i + 1;
            if raw.contains('\r') {
                return Err(format!("input line {number}: a carriage return is not allowed (lines end with \\n only)"));
            }
            if raw.trim().is_empty() {
                continue;
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
            match lines.last() {
                None if t != 0.0 => {
                    return Err(format!(
                        "input line {number}: the first line must be at t = 0, found t = {t}"
                    ));
                }
                Some(prev) if t <= prev.t => {
                    return Err(format!("input line {number}: time {t} is not above the previous line's time {} (times must strictly increase)", prev.t));
                }
                _ => {}
            }
            if !(0.0..=1.0).contains(&throttle) {
                return Err(format!(
                    "input line {number}: throttle {throttle} must be from 0 to 1"
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
            lines.push(Line {
                number,
                t,
                throttle,
                brake,
                steer,
            });
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
        s.push_str(&format!(
            "{:.4},{:.4},{:.4},{:.4},{:.4},{:.2}\n",
            r.t, r.speed, r.pos.x, r.pos.y, r.pos.z, r.heading_deg
        ));
    }
    s
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

    let all: Vec<usize> = (0..sim.pos.len()).collect();
    let heading = |sim: &Sim| ground_angle(mean(&sim.pos, &front) - mean(&sim.pos, &rear));
    let centroid = |sim: &Sim| mean(&sim.pos, &all);
    let row = |sim: &Sim, t: f64| DriveRow {
        t,
        speed: chassis_com_velocity(sim).length(),
        pos: centroid(sim),
        heading_deg: heading(sim).to_degrees(),
    };
    let fin = |sim: &Sim, t: f64| {
        let c = centroid(sim);
        FinalState {
            t,
            x: c.x,
            z: c.z,
            heading_deg: heading(sim).to_degrees(),
        }
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
        }
    }
    Ok(DriveRun {
        rows,
        measure: m,
        final_state: last,
        abort,
    })
}
