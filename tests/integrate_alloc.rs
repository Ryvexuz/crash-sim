//! `Sim::integrate`'s tilted-gravity and car path does not allocate its per-node `extra` force
//! buffer every substep (task t-20261120-193454-2). A counting global allocator (std only) counts
//! the allocations made inside `integrate`. The solver here is a no-op that does not allocate, so
//! only `Sim`'s own allocations are counted (the real `SymplecticEuler` allocates its own force
//! Vec in src/solver.rs, which is outside this task). This file holds one test, so no other
//! thread allocates while it counts.

use crash_sim::format::parse;
use crash_sim::solver::{Solver, State};
use crash_sim::{Config, Sim};
use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering};

struct Counting;

static ALLOCS: AtomicUsize = AtomicUsize::new(0);

// SAFETY: every call is forwarded unchanged to the system allocator; only a counter is added.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        ALLOCS.fetch_add(1, Ordering::SeqCst);
        System.alloc(layout)
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        System.dealloc(ptr, layout)
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        ALLOCS.fetch_add(1, Ordering::SeqCst);
        System.realloc(ptr, layout, new_size)
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

fn allocs() -> usize {
    ALLOCS.load(Ordering::SeqCst)
}

/// Does nothing and allocates nothing: the default `step_with_forces` then only loops.
struct NoAlloc;

impl Solver for NoAlloc {
    fn step(&mut self, _state: State<'_>, _h: f64) {}
}

#[test]
fn integrate_reuses_its_extra_buffer() {
    let path = format!("{}/scenarios/car_basic.bsn", env!("CARGO_MANIFEST_DIR"));
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let body = parse(&text).unwrap_or_else(|e| panic!("car_basic.bsn: {e}"));
    let cfg = Config {
        duration: 1.0,
        dt: 0.001,
        substeps: 1,
        ..Config::default()
    };

    // The car path, and the tilted-gravity path of a car-less cube.
    let mut car_sim = Sim::from_car_with_solver(cfg.clone(), &body, Box::new(NoAlloc)).unwrap();
    assert!(car_sim.car.is_some(), "car_basic.bsn has wheels");
    let mut tilt_sim = Sim::with_solver(cfg, Box::new(NoAlloc));
    tilt_sim.set_gravity_tilt(0.1).unwrap();

    for (label, sim) in [("car", &mut car_sim), ("tilt", &mut tilt_sim)] {
        // The first substep may allocate: the buffer grows to the node count.
        sim.integrate(0.001);
        let mut total = 0;
        for _ in 0..2000 {
            let a = allocs();
            sim.integrate(0.001);
            total += allocs() - a;
        }
        println!("[{label}] allocations in 2000 integrate calls: {total}");
        assert_eq!(total, 0, "[{label}] integrate allocated");
    }
}
