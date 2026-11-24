//! The only lap path/top-speed accumulation rule, shared by CLI and library races.
//! Horizontal substep length is split at start/lap crossings; speed is the whole
//! substep's length divided by h. Checkpoints do not split the accumulation.

use crate::track::LapEvent;
use crate::vec3::Vec3;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LapStats {
    pub path_m: f64,
    pub max_speed_mps: f64,
}

#[derive(Clone, Debug, Default)]
pub struct LapStatsTracker {
    open: Option<LapStats>,
}

impl LapStatsTracker {
    pub fn new() -> Self {
        Self::default()
    }

    /// One substep; returns one `LapStats` per `LapEvent::Lap`, in event order.
    /// Callers supply the same positions and time base as their lap timer.
    pub fn observe(
        &mut self,
        prev: Vec3,
        pos: Vec3,
        t0: f64,
        t1: f64,
        h: f64,
        events: &[LapEvent],
    ) -> Vec<LapStats> {
        let len = ((pos.x - prev.x).powi(2) + (pos.z - prev.z).powi(2)).sqrt();
        let speed = len / h;
        let mut completed = Vec::new();
        let mut crossed = false;
        for ev in events {
            let frac = |t: f64| ((t - t0) / (t1 - t0)).clamp(0.0, 1.0);
            match ev {
                LapEvent::Start { t } => {
                    self.open = Some(LapStats {
                        path_m: len * (1.0 - frac(*t)),
                        max_speed_mps: speed,
                    });
                    crossed = true;
                }
                LapEvent::Lap(rec) => {
                    let f = frac(rec.end_t);
                    let done = self.open.take().unwrap_or(LapStats {
                        path_m: 0.0,
                        max_speed_mps: 0.0,
                    });
                    completed.push(LapStats {
                        path_m: done.path_m + len * f,
                        max_speed_mps: done.max_speed_mps.max(speed),
                    });
                    self.open = Some(LapStats {
                        path_m: len * (1.0 - f),
                        max_speed_mps: speed,
                    });
                    crossed = true;
                }
                LapEvent::Checkpoint { .. } => {}
            }
        }
        if !crossed {
            if let Some(s) = self.open.as_mut() {
                s.path_m += len;
                s.max_speed_mps = s.max_speed_mps.max(speed);
            }
        }
        completed
    }
}
