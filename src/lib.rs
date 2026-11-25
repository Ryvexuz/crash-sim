//! Starter crash sim: a cube of 8 nodes and 28 beams falling onto a ground plane.
//!
//! Simple on purpose: spring-damper beams, symplectic Euler, y = 0 ground with
//! restitution and friction. Std only, deterministic (no RNG, fixed step order).

pub mod ai;
pub mod ai_params;
pub mod broadphase;
pub mod car_contact;
pub mod collide;
pub mod format;
pub mod input;
pub mod menu;
pub mod multi_car;
pub mod net;
mod physics;
pub mod pulse;
pub mod race;
pub mod race_hash;
pub mod reference;
mod runner;
pub mod save;
pub mod settings;
pub mod settle;
pub mod shell;
mod sim;
pub mod sparse;
pub mod solver;
pub mod solver_implicit;
pub mod solver_m5;
pub mod speed_profile;
pub mod statehash;
pub mod track;
pub mod trajectory;
pub mod tune;
pub mod vec3;
pub mod vehicle;

pub use format::{Body, ParseError};
pub use physics::*;
pub use runner::*;
pub use sim::{Beam, BreakEvent, Config, Sample, Sim};

pub(crate) use collide::Plane;
pub(crate) use physics::clearly_within;
pub(crate) use solver::{Solver, State, SymplecticEuler};
pub(crate) use vec3::Vec3;
