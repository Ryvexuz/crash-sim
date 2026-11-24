//! Two-car lockstep: one tick is one output step of `cfg.substeps` substeps.
//! The host drives car 1 in slot 1; the guest drives car 2 in slot 2.
//! Inputs arrive already delayed by the session (or the reference-run caller).
//! Callers check `failure()` after every step, so `FAILED_HASH` is never compared
//! as a successful state hash. Before the first tick the stored hash is also zero.

use super::lap_stats::{LapStats, LapStatsTracker};
use super::near::{GapStats, NearTracker};
use super::{
    car_position, place_on_grid, slot_distance, RaceError, CONTACT_RESTITUTION, CONTACT_SKIN,
    MAX_MOTION, MAX_PENETRATION,
};
use crate::car_contact::ContactSettings;
use crate::multi_car::{CarControls, CarPlacement, MultiError, MultiSim};
use crate::net::session::{Input, LockstepSim};
use crate::net::INPUT_SCALE;
use crate::race_hash::{multi_state_hash, RaceHashError};
use crate::settle::{self, CarSetup};
use crate::track::{LapEvent, LapRecord, LapTimer, TimerError, Track};
use crate::vec3::Vec3;
use crate::{Body, Config};

pub struct LockstepSetup<'a> {
    pub cfg: &'a Config,
    pub body: &'a Body,
    pub track: &'a Track,
    pub grid_gap_m: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct TickEvent {
    pub tick: u64,
    pub car: u32,
    pub event: LapEvent,
    pub lap_stats: Option<LapStats>,
}

#[derive(Debug)]
pub enum LockstepFailure {
    TickOrder {
        expected: u64,
        got: u64,
    },
    Sim {
        tick: u64,
        t: f64,
        error: MultiError,
    },
    Timer {
        tick: u64,
        car: u32,
        t: f64,
        error: TimerError,
    },
    Hash {
        tick: u64,
        error: RaceHashError,
    },
}

/// Returned after failure; callers must check `failure()` after every step.
pub const FAILED_HASH: u64 = 0;

/// Convert thousandths to sim units without clamping, including reverse throttle.
pub fn controls_of(input: Input) -> CarControls {
    CarControls {
        throttle: f64::from(input.throttle) / f64::from(INPUT_SCALE),
        brake: f64::from(input.brake) / f64::from(INPUT_SCALE),
        steer: f64::from(input.steer) / f64::from(INPUT_SCALE),
    }
}

pub struct LockstepRace {
    multi: MultiSim,
    timers: [LapTimer; 2],
    lap_trackers: [LapStatsTracker; 2],
    prev: [Vec3; 2],
    near: NearTracker,
    events: Vec<TickEvent>,
    laps: [Vec<LapRecord>; 2],
    ticks: u64,
    substeps: u64,
    h: f64,
    substeps_per_tick: u64,
    last_hash: u64,
    failure: Option<LockstepFailure>,
}

impl LockstepRace {
    pub fn new(setup: &LockstepSetup<'_>) -> Result<Self, RaceError> {
        if !setup.grid_gap_m.is_finite() || setup.grid_gap_m <= 0.0 {
            return Err(RaceError::GridGap(setup.grid_gap_m));
        }
        setup.cfg.validate_numbers().map_err(RaceError::Config)?;
        let h = setup.cfg.dt / f64::from(setup.cfg.substeps);
        let mut placed = Vec::new();
        for slot in 1..=2 {
            let distance = slot_distance(setup.track, slot, setup.grid_gap_m);
            if !distance.is_finite() || distance < 0.0 || distance > setup.track.centre_length() {
                return Err(RaceError::Grid { slot });
            }
            let body = place_on_grid(setup.body, setup.track, distance)?;
            let built = body.build();
            setup
                .cfg
                .validate_body(&built.beams, &built.mass)
                .map_err(RaceError::Config)?;
            placed.push(body);
        }
        let mut one = setup.cfg.clone();
        one.dt = h;
        one.substeps = 1;
        let mut sims = Vec::new();
        for body in &placed {
            sims.push(
                settle::car_sim(one.clone(), body, &CarSetup::default())
                    .map_err(RaceError::Config)?,
            );
        }
        let settings = ContactSettings::new(
            CONTACT_SKIN,
            CONTACT_RESTITUTION,
            h,
            MAX_MOTION,
            MAX_PENETRATION,
        )
        .map_err(RaceError::Contact)?;
        let multi = MultiSim::new(sims, &[CarPlacement { offset: Vec3::ZERO }; 2], settings, h)
            .map_err(RaceError::Multi)?;
        let prev = [car_position(multi.sim(0)), car_position(multi.sim(1))];
        Ok(Self {
            multi,
            timers: std::array::from_fn(|_| LapTimer::new(setup.track)),
            lap_trackers: std::array::from_fn(|_| LapStatsTracker::new()),
            prev,
            near: NearTracker::new(),
            events: Vec::new(),
            laps: std::array::from_fn(|_| Vec::new()),
            ticks: 0,
            substeps: 0,
            h,
            substeps_per_tick: u64::from(setup.cfg.substeps),
            last_hash: FAILED_HASH,
            failure: None,
        })
    }

    pub fn ticks(&self) -> u64 {
        self.ticks
    }
    pub fn substeps(&self) -> u64 {
        self.substeps
    }
    pub fn h(&self) -> f64 {
        self.h
    }
    pub fn substeps_per_tick(&self) -> u64 {
        self.substeps_per_tick
    }
    pub fn take_events(&mut self) -> Vec<TickEvent> {
        std::mem::take(&mut self.events)
    }
    pub fn laps(&self, car: u32) -> &[LapRecord] {
        match car {
            1 => &self.laps[0],
            2 => &self.laps[1],
            _ => &[],
        }
    }
    pub fn gap(&self) -> Option<GapStats> {
        self.near.stats()
    }
    pub fn failure(&self) -> Option<&LockstepFailure> {
        self.failure.as_ref()
    }
    pub fn multi(&self) -> &MultiSim {
        &self.multi
    }
}

impl LockstepSim for LockstepRace {
    fn step(&mut self, tick: u64, inputs: [Input; 2]) {
        if self.failure.is_some() {
            return;
        }
        if tick != self.ticks {
            self.failure = Some(LockstepFailure::TickOrder {
                expected: self.ticks,
                got: tick,
            });
            return;
        }
        // Like race::run, require an exact integer substep clock and finite
        // timestamps before advancing. This is the f64 precision boundary,
        // not a configurable race duration cap.
        let end = self.substeps.checked_add(self.substeps_per_tick);
        if end
            .is_none_or(|n| n > (1_u64 << f64::MANTISSA_DIGITS) || !(n as f64 * self.h).is_finite())
        {
            self.failure = Some(LockstepFailure::Sim {
                tick,
                t: self.substeps as f64 * self.h,
                error: MultiError::BadCars("substep clock is unrepresentable".into()),
            });
            return;
        }
        let controls = [controls_of(inputs[0]), controls_of(inputs[1])];
        for _ in 0..self.substeps_per_tick {
            let i = self.substeps;
            let t0 = i as f64 * self.h;
            let t1 = (i + 1) as f64 * self.h;
            if let Err(error) = self.multi.advance(&controls) {
                self.failure = Some(LockstepFailure::Sim { tick, t: t1, error });
                return;
            }
            let mut centres = [Vec3::ZERO; 2];
            for (k, centre) in centres.iter_mut().enumerate() {
                let pos = car_position(self.multi.sim(k));
                *centre = pos;
                let events = match self.timers[k].update(self.prev[k], pos, t0, t1) {
                    Ok(events) => events,
                    Err(error) => {
                        self.failure = Some(LockstepFailure::Timer {
                            tick,
                            car: k as u32 + 1,
                            t: t1,
                            error,
                        });
                        return;
                    }
                };
                let mut stats = self.lap_trackers[k]
                    .observe(self.prev[k], pos, t0, t1, self.h, &events)
                    .into_iter();
                for event in events {
                    let lap_stats = if let LapEvent::Lap(lap) = &event {
                        self.laps[k].push(lap.clone());
                        stats.next()
                    } else {
                        None
                    };
                    self.events.push(TickEvent {
                        tick,
                        car: k as u32 + 1,
                        event,
                        lap_stats,
                    });
                }
                self.prev[k] = pos;
            }
            self.near.observe(&centres, t1);
            self.substeps += 1;
        }
        self.ticks = tick + 1;
        match multi_state_hash(tick, &self.multi) {
            Ok(hash) => self.last_hash = hash,
            Err(error) => self.failure = Some(LockstepFailure::Hash { tick, error }),
        }
    }

    fn state_hash(&self) -> u64 {
        if self.failure.is_some() {
            FAILED_HASH
        } else {
            self.last_hash
        }
    }
}
