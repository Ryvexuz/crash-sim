//! Fixed-order, lockstep stepping of cars with contact after every substep.

use crate::car_contact::{apply_contacts, ContactCar, ContactError, ContactSettings};
use crate::collide;
use crate::vec3::Vec3;
use crate::Sim;

/// Throttle, brake, steer for one car, held for one output step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CarControls {
    pub throttle: f64,
    pub brake: f64,
    pub steer: f64,
}

/// Offset in metres, added once to every node and wheel centre.
#[derive(Clone, Copy, Debug)]
pub struct CarPlacement {
    pub offset: Vec3,
}

#[derive(Debug)]
pub enum MultiError {
    /// Invalid cars, placements, or substep size.
    BadCars(String),
    /// Cars disagree on dt or substeps (bit-exact).
    MismatchedConfig {
        car: usize,
    },
    ControlCount {
        expected: usize,
        got: usize,
    },
    Controls {
        car: usize,
        message: String,
    },
    Unstable {
        car: usize,
        message: String,
    },
    Contact(ContactError),
}

impl std::fmt::Display for MultiError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::BadCars(message) => write!(f, "bad cars: {message}"),
            Self::MismatchedConfig { car } => write!(f, "car {car}: dt or substeps differs"),
            Self::ControlCount { expected, got } => {
                write!(f, "expected {expected} controls, got {got}")
            }
            Self::Controls { car, message } => write!(f, "car {car} controls: {message}"),
            Self::Unstable { car, message } => write!(f, "car {car} unstable: {message}"),
            Self::Contact(error) => write!(f, "{error}"),
        }
    }
}

impl std::error::Error for MultiError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Contact(error) => Some(error),
            _ => None,
        }
    }
}

/// Counts summed over all physics substeps of one output step.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct StepReport {
    pub substeps: u32,
    pub candidates: usize,
    pub contacts: usize,
    pub impulses: usize,
}

pub struct MultiSim {
    cars: Vec<Sim>,
    settings: ContactSettings,
    previous_pos: Vec<Vec<Vec3>>,
}

fn finite(v: Vec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}

impl MultiSim {
    /// Slice order is the car order, fixed for life. Placements are applied once.
    ///
    /// `substep_h` must equal dt/substeps bit for bit. The caller must supply
    /// the same timestep to `ContactSettings::new`, which has no timestep getter.
    /// Config numbers and finite placement sums are checked before any placement.
    pub fn new(
        mut sims: Vec<Sim>,
        placements: &[CarPlacement],
        settings: ContactSettings,
        substep_h: f64,
    ) -> Result<MultiSim, MultiError> {
        if sims.is_empty() || placements.len() != sims.len() {
            return Err(MultiError::BadCars(
                "need at least one car and exactly one placement per car".into(),
            ));
        }
        let cfg = &sims[0].cfg;
        for (i, sim) in sims.iter().enumerate() {
            if sim.cfg.dt.to_bits() != cfg.dt.to_bits() || sim.cfg.substeps != cfg.substeps {
                return Err(MultiError::MismatchedConfig { car: i });
            }
            sim.cfg
                .validate_numbers()
                .map_err(|e| MultiError::BadCars(format!("car {i}: {e}")))?;
            if sim.car.as_ref().is_none_or(|car| car.wheels.is_empty()) {
                return Err(MultiError::BadCars(format!("car {i}: no wheels")));
            }
            if sim.step != sims[0].step {
                return Err(MultiError::BadCars(format!("car {i}: output step differs")));
            }
            let offset = placements[i].offset;
            if !finite(offset)
                || sim
                    .pos
                    .iter()
                    .chain(sim.car.as_ref().unwrap().wheels.iter().map(|w| &w.pos))
                    .any(|&p| !finite(p + offset))
            {
                return Err(MultiError::BadCars(format!(
                    "car {i}: placement or placed position is not finite"
                )));
            }
        }
        let h = cfg.dt / cfg.substeps as f64;
        if !(substep_h.is_finite() && substep_h > 0.0 && substep_h.to_bits() == h.to_bits()) {
            return Err(MultiError::BadCars(
                "substep_h must equal dt/substeps bit for bit".into(),
            ));
        }
        for (sim, placement) in sims.iter_mut().zip(placements) {
            // Do not add even +0.0: that would erase a stored -0.0.
            if placement.offset != Vec3::ZERO {
                for p in &mut sim.pos {
                    *p += placement.offset;
                }
                for wheel in &mut sim.car.as_mut().unwrap().wheels {
                    wheel.pos += placement.offset;
                }
            }
        }
        let previous_pos = sims.iter().map(|sim| sim.pos.clone()).collect();
        Ok(Self {
            cars: sims,
            settings,
            previous_pos,
        })
    }

    pub fn car_count(&self) -> usize {
        self.cars.len()
    }

    /// Read only; panics on a bad index like slice indexing.
    pub fn sim(&self, car: usize) -> &Sim {
        &self.cars[car]
    }

    pub fn step_index(&self) -> u64 {
        self.cars[0].step
    }

    /// Apply controls, then snapshot/substep/check/contact/first-contact, in
    /// that order for each physics substep. Every slice traversal is in index
    /// order. One car bypasses contact entirely.
    ///
    /// Errors leave state at the point of failure, not rolled back. Stop using
    /// this sim after any error. A controls refusal moves no car, though earlier
    /// cars' controls may already have been set.
    pub fn advance(&mut self, controls: &[CarControls]) -> Result<StepReport, MultiError> {
        if controls.len() != self.cars.len() {
            return Err(MultiError::ControlCount {
                expected: self.cars.len(),
                got: controls.len(),
            });
        }
        for (i, (sim, control)) in self.cars.iter_mut().zip(controls).enumerate() {
            sim.car
                .as_mut()
                .unwrap()
                .set_controls(control.throttle, control.brake, control.steer)
                .map_err(|message| MultiError::Controls { car: i, message })?;
        }
        let dt = self.cars[0].cfg.dt;
        let substeps = self.cars[0].cfg.substeps;
        let h = dt / substeps as f64;
        let mut report = StepReport::default();
        for k in 0..substeps {
            for (sim, old) in self.cars.iter().zip(&mut self.previous_pos) {
                old.copy_from_slice(&sim.pos);
            }
            for sim in &mut self.cars {
                sim.substep(k + 1, h);
            }
            for (i, sim) in self.cars.iter().enumerate() {
                sim.check()
                    .map_err(|message| MultiError::Unstable { car: i, message })?;
            }
            if self.cars.len() >= 2 {
                let mut views: Vec<_> = self
                    .cars
                    .iter_mut()
                    .zip(&self.previous_pos)
                    .map(|(sim, old)| ContactCar {
                        previous_pos: old,
                        pos: &sim.pos,
                        vel: &mut sim.vel,
                        mass: &sim.mass,
                        beams: &sim.beams,
                        broken: &sim.broken,
                    })
                    .collect();
                let contact =
                    apply_contacts(&mut views, self.settings).map_err(MultiError::Contact)?;
                report.candidates += contact.candidates;
                report.contacts += contact.contacts;
                report.impulses += contact.impulses;
            }
            for sim in &mut self.cars {
                if sim.first_contact_s.is_none() && collide::any_at_or_behind(&sim.pos, &sim.planes)
                {
                    sim.first_contact_s = Some(sim.step as f64 * dt + (k + 1) as f64 * h);
                }
            }
            report.substeps += 1;
        }
        for sim in &mut self.cars {
            sim.step += 1;
        }
        Ok(report)
    }

    pub fn chassis_positions(&self, car: usize) -> &[Vec3] {
        &self.cars[car].pos
    }

    /// Mount nodes, not wheel centres or a tyre-edge point.
    pub fn wheel_mount_positions(&self, car: usize) -> Vec<Vec<Vec3>> {
        let sim = &self.cars[car];
        sim.car
            .as_ref()
            .unwrap()
            .wheels
            .iter()
            .map(|wheel| wheel.p.mount.iter().map(|&i| sim.pos[i]).collect())
            .collect()
    }

    /// Live state words in the order specified in docs/PHYSICS.md. No float
    /// canonicalisation: signed zero and all other bits are preserved.
    pub fn state_words(&self) -> Vec<u64> {
        let mut words = vec![self.car_count() as u64, self.step_index()];
        for sim in &self.cars {
            let car = sim.car.as_ref().unwrap();
            words.extend([
                sim.pos.len() as u64,
                car.wheels.len() as u64,
                sim.beams.len() as u64,
            ]);
            for (&pos, &vel) in sim.pos.iter().zip(&sim.vel) {
                push_vec(&mut words, pos);
                push_vec(&mut words, vel);
            }
            for wheel in &car.wheels {
                push_vec(&mut words, wheel.pos);
                push_vec(&mut words, wheel.vel);
                words.push(wheel.spin.to_bits());
                for &deflection in &wheel.deflection {
                    push_vec(&mut words, deflection);
                }
                words.extend(wheel.load.iter().map(|v| v.to_bits()));
                words.push(wheel.length.to_bits());
            }
            for (beam, &broken) in sim.beams.iter().zip(&sim.broken) {
                words.extend([beam.rest.to_bits(), u64::from(broken)]);
            }
        }
        words
    }
}

fn push_vec(words: &mut Vec<u64>, v: Vec3) {
    words.extend([v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]);
}
