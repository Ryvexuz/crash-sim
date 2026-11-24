//! Bit-exact race state hashing, defined only for cars whose `Sim::check()` is Ok.
//!
//! Not hashed: body constants shared from the same files (`cfg`, `mass`, beam
//! `a`, `b`, `stiffness`, `damping`, `planes`, `rest0`, `yield_strain`,
//! `break_strain`, wheel `p`, car `max_*`, `reference_load`,
//! `rolling_resistance`, `drag_area`); records the step never reads back
//! (`first_contact_s`, `breaks`, `yield_refused`, with failure checked first);
//! private per-substep fields (wheel `force`, `axle`, `spin_next`, car scratch,
//! and Sim solver working data, `step`, `extra_scratch`). Working buffers are
//! overwritten before use; car `wx` and `wz` are fixed frame-fit weights derived
//! from the body. The solver's substep counter is read only into error text;
//! its working buffers (`force`, `new_vel`, `new_pos`) are overwritten before
//! read every substep. A stored solver error is refused through `Sim::check`.

use crate::multi_car::MultiSim;
use crate::statehash::Hasher;
use crate::Sim;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RaceHashError {
    /// The slice of cars is empty.
    NoCars,
    /// The indexed sim has no car.
    NoCar { car: usize },
    /// Beam and broken lists have different lengths.
    Shape { car: usize },
    /// A NaN was found in the indexed car.
    Nan { car: usize },
    /// The indexed simulation has failed its check.
    Failed { car: usize, reason: String },
}

impl std::fmt::Display for RaceHashError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoCars => write!(f, "There are no cars to hash."),
            Self::NoCar { car } => write!(f, "Car {} has no simulated car.", car + 1),
            Self::Shape { car } => {
                write!(f, "Car {} has mismatched beam and broken lists.", car + 1)
            }
            Self::Nan { car } => write!(f, "Car {} has NaN state.", car + 1),
            Self::Failed { car, reason } => {
                write!(f, "Car {} failed: {reason}.", car + 1)
            }
        }
    }
}

impl std::error::Error for RaceHashError {}

/// Hash tick, car count, then each car's counts, nodes, wheels, carried state
/// and beams, in slice order. Floats retain their exact bits, including zero signs.
pub fn state_hash(tick: u64, cars: &[&Sim]) -> Result<u64, RaceHashError> {
    if cars.is_empty() {
        return Err(RaceHashError::NoCars);
    }
    let mut hash = Hasher::new();
    hash.write_u64(tick);
    hash.write_u64(cars.len() as u64);
    for (index, sim) in cars.iter().enumerate() {
        let car = sim
            .car
            .as_ref()
            .ok_or(RaceHashError::NoCar { car: index })?;
        if sim.beams.len() != sim.broken.len() {
            return Err(RaceHashError::Shape { car: index });
        }
        sim.check()
            .map_err(|reason| RaceHashError::Failed { car: index, reason })?;
        hash.write_u64(sim.pos.len() as u64);
        hash.write_u64(car.wheels.len() as u64);
        hash.write_u64(sim.beams.len() as u64);
        // Use a fallible feed for every float, including fields check() does not read.
        let feed = |hash: &mut Hasher, x| {
            hash.write_f64(x)
                .map_err(|_| RaceHashError::Nan { car: index })
        };
        for (p, v) in sim.pos.iter().zip(&sim.vel) {
            for x in [p.x, p.y, p.z, v.x, v.y, v.z] {
                feed(&mut hash, x)?;
            }
        }
        for w in &car.wheels {
            for x in [w.pos.x, w.pos.y, w.pos.z, w.vel.x, w.vel.y, w.vel.z, w.spin] {
                feed(&mut hash, x)?;
            }
            for d in &w.deflection {
                for x in [d.x, d.y, d.z] {
                    feed(&mut hash, x)?;
                }
            }
            for &x in &w.load {
                feed(&mut hash, x)?;
            }
            feed(&mut hash, w.length)?;
        }
        for x in [car.throttle, car.brake, car.steer] {
            feed(&mut hash, x)?;
        }
        for v in [
            car.frame.up,
            car.frame.forward,
            car.frame.axle,
            car.carrier_torque,
        ] {
            for x in [v.x, v.y, v.z] {
                feed(&mut hash, x)?;
            }
        }
        feed(&mut hash, car.drive_work)?;
        feed(&mut hash, sim.gravity_tilt())?;
        hash.write_u64(u64::from(car.frame_error.is_some()));
        for (beam, &broken) in sim.beams.iter().zip(&sim.broken) {
            feed(&mut hash, beam.rest)?;
            hash.write_u64(u64::from(broken));
        }
    }
    Ok(hash.finish())
}

/// Hash all cars of a MultiSim in its fixed car order.
pub fn multi_state_hash(tick: u64, multi: &MultiSim) -> Result<u64, RaceHashError> {
    let cars: Vec<_> = (0..multi.car_count()).map(|i| multi.sim(i)).collect();
    state_hash(tick, &cars)
}
