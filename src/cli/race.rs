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

