# Crash sim physics

This describes the model in `src/lib.rs`, which is the source of truth. Units are SI (metres, kilograms, seconds, newtons). Everything is deterministic: no random numbers, fixed update order.

## The cube: 8 nodes, 28 beams
Moved to [physics/beams.md](physics/beams.md#the-cube-8-nodes-28-beams).

## Beam force
Moved to [physics/beams.md](physics/beams.md#beam-force).

## Plastic beams (yield)
Moved to [physics/beams.md](physics/beams.md#plastic-beams-yield).

## Breaking beams
Moved to [physics/beams.md](physics/beams.md#breaking-beams).

## Unloading return (hysteresis)
Moved to [physics/beams.md](physics/beams.md#unloading-return-hysteresis).

## Bodies from a file
Moved to [physics/bodies.md](physics/bodies.md#bodies-from-a-file).

## Wheels, suspension and tyres
Moved to [physics/wheels.md](physics/wheels.md#wheels-suspension-and-tyres).

### Tyre tuning log (nb-44, the skidpad)
Moved to [physics/wheels.md](physics/wheels.md#tyre-tuning-log-nb-44-the-skidpad).

## Gravity
Moved to [physics/contact.md](physics/contact.md#gravity).

## Ground contact
Moved to [physics/contact.md](physics/contact.md#ground-contact).

## Plane contact (the ground and walls)
Moved to [physics/contact.md](physics/contact.md#plane-contact-the-ground-and-walls).

## The box_wall material
Moved to [physics/contact.md](physics/contact.md#the-box_wall-material).

### Sensitivity runs
Moved to [physics/contact.md](physics/contact.md#sensitivity-runs).

## Solids (the m5 crash-zone solver)
Moved to [physics/solids.md](physics/solids.md#solids-the-m5-crash-zone-solver).

## Integrator: symplectic Euler with substeps
Moved to [physics/integrator.md](physics/integrator.md#integrator-symplectic-euler-with-substeps).

### Why the step limit is what it is
Moved to [physics/integrator.md](physics/integrator.md#why-the-step-limit-is-what-it-is).

### The damping limit
Moved to [physics/integrator.md](physics/integrator.md#the-damping-limit).

## CSV output
Moved to [physics/integrator.md](physics/integrator.md#csv-output).

## Safety checks
Moved to [physics/integrator.md](physics/integrator.md#safety-checks).

## Inter-car chassis contact (n20-19a kernel)
Moved to [physics/cars.md](physics/cars.md#inter-car-chassis-contact-n20-19a-kernel).

### Geometry, candidates and supported motion
Moved to [physics/cars.md](physics/cars.md#geometry-candidates-and-supported-motion).

### Impulses, atomicity and numerical band
Moved to [physics/cars.md](physics/cars.md#impulses-atomicity-and-numerical-band).

### Conservation checks and retained integration work
Moved to [physics/cars.md](physics/cars.md#conservation-checks-and-retained-integration-work).

## Several cars in one sim
Moved to [physics/cars.md](physics/cars.md#several-cars-in-one-sim).

### State words and geometry
Moved to [physics/cars.md](physics/cars.md#state-words-and-geometry).

