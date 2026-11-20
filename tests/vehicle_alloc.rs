//! `Car::pre_step` and `Car::post_step` do not allocate once the car has run one substep
//! (task t-20261120-193454). A counting global allocator (std only) counts the allocations made
//! inside each call. This file holds one test, so no other thread allocates while it counts.

use crash_sim::format::parse;
use crash_sim::vec3::Vec3;
use crash_sim::{vehicle, Config, Sim};
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

#[test]
fn car_substep_reuses_its_buffers() {
    let path = format!("{}/scenarios/car_basic.bsn", env!("CARGO_MANIFEST_DIR"));
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let body = parse(&text).unwrap_or_else(|e| panic!("car_basic.bsn: {e}"));
    let cfg = Config {
        duration: 1.0,
        dt: 0.001,
        substeps: 1,
        ..Config::default()
    };
    let sim = Sim::from_car(cfg, &body).unwrap();
    let mut car = sim.car.clone().expect("car_basic.bsn has wheels");
    assert!(!car.wheels.is_empty());
    let mut pos = sim.pos.clone();
    let mut vel = sim.vel.clone();
    let mut extra = vec![Vec3::ZERO; pos.len()];
    let step = vehicle::Substep {
        gravity: Vec3::new(0.0, -9.81, 0.0),
        h: 0.001,
    };
    // Some throttle and brake and a steer, so every torque and axle path runs.
    car.set_controls(0.5, 0.25, 0.3).unwrap();

    // The first substep may allocate: the buffers grow to the wheel count.
    car.pre_step(&pos, &vel, &sim.mass, &sim.planes, step, &mut extra);
    let caps = car.scratch_capacities();
    let addrs = car.scratch_addresses();
    assert!(
        caps.iter().all(|&c| c >= car.wheels.len()),
        "capacities {caps:?} for {} wheels",
        car.wheels.len()
    );

    let mut pre_allocs = 0;
    let mut post_allocs = 0;
    for i in 0..2000 {
        // Vary the controls so a path that only runs on some substeps is covered too.
        let t = (i % 7) as f64 / 7.0;
        car.set_controls(t, 1.0 - t, t - 0.5).unwrap();
        extra.iter_mut().for_each(|e| *e = Vec3::ZERO);
        let a = allocs();
        car.pre_step(&pos, &vel, &sim.mass, &sim.planes, step, &mut extra);
        pre_allocs += allocs() - a;
        let a = allocs();
        car.post_step(&mut pos, &mut vel, &sim.mass, &sim.planes, step.h);
        post_allocs += allocs() - a;
    }
    println!(
        "allocations in 2000 substeps: pre_step {pre_allocs}, post_step {post_allocs}; \
         capacities {:?} -> {:?}",
        caps,
        car.scratch_capacities()
    );
    assert_eq!(pre_allocs, 0, "pre_step allocated");
    assert_eq!(post_allocs, 0, "post_step allocated");
    assert_eq!(car.scratch_capacities(), caps, "a buffer was regrown");
    assert_eq!(car.scratch_addresses(), addrs, "a buffer was reallocated");
}
