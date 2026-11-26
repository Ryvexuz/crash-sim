## Inter-car chassis contact (n20-19a kernel)

`car_contact::{detect_contacts, apply_contacts, apply_contacts_moving}` takes cars
in caller-defined fixed order, with chassis node positions **before and after one
physics substep**, post-substep velocities, node masses, beams and a parallel broken-beam mask.
Indices in those slices are the car/node/beam identities; broken beams are not
compacted. Save positions before stepping, step each car using the existing
single-car integrator, then call contact after **every physics substep**, not
only at output frames. `MultiSim` calls `apply_contacts_moving`, supplying the
immediately preceding successful substep's impulse-key set (empty initially).
For that entry only, an outside/outside pair within the existing
`skin+2*max_motion` final-distance band whose key is in that set is treated as
a contact, with the incoming closest-point normal and fraction. The sweep
certificate requirements below still apply to every other pair. The returned
set contains only keys that actually received impulses this substep, replacing
rather than accumulating the old set. A refusal changes neither the caller's
positions/velocities nor its retained key set. The keys are deterministic derived
state, not added to the race hash.
This module is not a race loop or a wheel collider.

The caller supplies `ContactSettings::new(skin, restitution, timestep,
max_motion, max_penetration)`. Distances are metres, timestep is seconds, mass is
kg, velocity is m/s and impulse is kg m/s. There are no scene defaults or
hard-coded scene caps in the kernel. Skin must be positive; restitution must be
finite in `[0,1]`; timestep must be positive and have a finite reciprocal.
`max_motion` and `max_penetration` must be positive, finite, and respectively at
most `skin/8` and `skin/2`. All settings are constructor-validated and immutable.
The test fixtures' values are explicit test inputs, not production defaults.

### Geometry, candidates and supported motion

A **live beam is a capsule of radius skin** about its current endpoint segment;
a source node is a point. Contact includes tangency: the computed Euclidean
closest-point distance is `<= skin`, with no extra contact tolerance.
The beam fraction is `t = clamp(dot(p-a,b-a)/dot(b-a,b-a), 0, 1)`.
The normal points from that closest point toward the source node.
Endpoints use spherical caps. Sparse beam capsules are **not** a solid car
surface: holes between beams, beam-versus-beam crossings without a nearby node,
wheels and self-contact are not resolved by this model. Nearby endpoints alone
are never evidence of contact. There is no tangential friction in this kernel.

Nodes are flattened in car/node order and passed to the merged
`broadphase::candidate_pairs`, skipping every same-car pair. The query radius is
`(longest_live_beam + skin + 2*max_motion) * (1 + 64*EPSILON)`;
the longest length includes both snapshots. The full beam length is a
conservative endpoint expansion: a node near any point on a segment is within
that length plus its distance from either endpoint. The motion term also
includes possible swept hits. Padding protects query recall from roundoff; it
does not change the narrow-phase boundary. Each endpoint candidate maps to its
incident live beams in **both directions**. A sorted set deduplicates keys
`(source_car, node, target_car, beam)`; geometry and responses use that fixed
lexicographic order. The exhaustive oracle in `tests/multi_car.rs` visits every
cross-car node/beam and uses endpoint spheres or a cross-product line distance,
independently of candidate generation and the kernel's closest-point formula.

The discrete rule requires a straight-line interpolation of each node between
the saved snapshots. Each node's actual displacement and `|velocity|*timestep`
must be `<= max_motion`, including the trial response velocities. The distance
to every candidate beam in **both** snapshots must be at least
`skin-max_penetration`. Thus a node and any point of a moving segment have at
most `2*max_motion <= skin/4` relative travel; a supported sample at distance at
least `skin/2` cannot cross the beam axis in that substep. Deep penetration,
zero-distance/undefined normals and zero-length live beams are explicit errors,
not guessed normals or successful empty results.

Shallow grazing could still enter and exit the skin between two outside
samples. For an outside/outside candidate within `skin+2*max_motion` at the
end, except a remembered impulse key in the moving entry, the kernel requires
a separating-plane certificate. Using either snapshot's
closest-point normal, **both endpoints in both snapshots** must project strictly
farther than skin from the source. Convex interpolation then certifies that
every point of the linearly moving segment stays outside that plane throughout
the substep. The implemented certificate accounts for the computed normal's
length and adds a conservative `64*EPSILON` roundoff margin scaled by the larger
position norm or skin. Identical source/endpoint snapshots need no sweep check.
If neither normal certifies separation, the kernel also tries the blended normals
`n0 + λ(n1 - n0)` at the at most six `λ` in (0, 1) where two of the four endpoint
projections (linear in `λ`) cross; the same strict test decides, and it is valid
for any normal because it scales by the normal's length. If none certifies, the
kernel halves the substep's linear motion at s = k/2^j, at most six times and only
where a piece is not yet certified, and accepts only when every piece passes the
same strict test with its own end normals; otherwise `UnsupportedSweep` refuses
the pass, even when a more expensive continuous solver might prove it safe.
Reduce the substep or use a future continuous contact model; do not ignore this
error and continue ghosting. Curved within-substep trajectories are outside this
API's geometric contract.

### Impulses, atomicity and numerical band

Let `u = 1-t`, source mass be `m`, endpoint masses be `ma, mb`, and normal relative
velocity be `vn = dot(v - u*va - t*vb, n)`. For approaching contact (`vn < 0`):

```
w = 1/m + u*u/ma + t*t/mb
j = -(1+restitution)*vn/w
v  += n*j/m
va -= n*j*u/ma
vb -= n*j*t/mb
```

The endpoint impulses sum to the negative source impulse; interpolation also
places their resultant at the closest point. Separating or stationary normal
motion receives no impulse. In exact arithmetic the kinetic change is
`-(1-restitution^2)*vn^2/(2*w) <= 0`. This is one sequential pass, not an
iterated simultaneous-contact solve. In `apply_contacts_moving`, each applied
impulse also moves its three nodes by their velocity change times the timestep
(`x += dv*h`), in the same order. No other positions change: there is no projection
to remove penetration and no second pass. This equals the unconstrained Euler
position update of the post-impulse velocity; wheel-mount and plane corrections
intervene in the actual integrator. Normals and fractions come from the incoming
positions. `apply_contacts` changes velocities only. Wheel state is unchanged
during the pass; its subsequent carrier/suspension interaction remains the
existing single-car implementation.

All lengths, indices (including broken-beam indices), masses, node components,
and beam scalar fields are validated before response. Masses must be positive
and have a normal finite reciprocal. Beam rest lengths must be positive;
stiffness and damping must be nonnegative and finite. To bound geometric
products, skin is restricted to
`[64*sqrt(MIN_POSITIVE), sqrt(MAX)/64]`, every coordinate magnitude to
`min(skin/(64*EPSILON), sqrt(MAX)/64)`, and live beam squared lengths must be
positive normal numbers in both snapshots. These are IEEE-754 representability
and resolution bounds, not tunable world-size limits. Speed/displacement norms
use `hypot` to avoid squared-norm overflow/underflow. Unrepresentable impulse,
effective inverse mass, kinetic sum or output velocity arithmetic is refused.
Tests use exact rational impulse references, power-of-two rescaling, numerical
skin-band endpoints, extreme masses and nearly cancelling common velocities.

Response runs on private trial velocities and positions. Only after finite output,
energy and next-step speed checks does `apply_contacts_moving` copy both back
(`apply_contacts` copies velocities only); **any error leaves all
incoming caller state untouched**, even after a trial impulse. A refusal names
the car and node/beam/contact key (settings errors have no car), with a named
failure such as `MotionBound`, `PenetrationBound` or `EnergyIncrease`.
Before commit, the moving entry refuses `PenetrationBound` when a follow changes
a node's `Plane::distance` from >= 0 to < 0, as computed by `Plane::distance`
(the same predicate as the plane rule). Nodes whose position bits did not change
are not tested; already-behind nodes remain the plane rule's responsibility.
It also rechecks final staged distances in contact-key order for every live
cross-car pair that received an impulse or whose node or endpoints were followed,
refusing `PenetrationBound` with that pair's key below the unchanged
`skin-max_penetration` bound.
The result counts deduplicated candidates, actual contacts and applied impulses,
and reports their summed normal impulse magnitude. Zero impulses are not an
error for valid separating contacts.

### Conservation checks and retained integration work

`head_on_at_low_speed_conserves_momentum` uses two `car_basic.bsn` cars and real
single-car substeps, with no gravity, planes, tyre contact, drag, rolling
resistance or controls supplying external momentum. Momentum includes chassis
and wheel translation. At every substep its error must be at most
`1e-6 * max(1 kg m/s, sum(initial mass * speed))`. It also requires nonzero
collision impulses and changed velocities on both cars.

`car_contact_never_creates_energy` measures chassis kinetic energy immediately
before/after each contact pass through `tests/multi_car.rs::step_contact`. Only
`1e-10 * max(1 J, |E_before|)` roundoff is allowed. The kernel itself enforces
that same tolerance on chassis kinetic energy before copying back. Position
following can change live-beam elastic energy, summed in beam order as
`0.5 * stiffness * stretch * stretch`, where
`stretch = (pos[a] - pos[b]).length() - rest`. With a common gravity vector,
gravity potential is unchanged in exact arithmetic because the impulse conserves
momentum and `dx = dv*h`.
The kernel does not measure elastic or wheel terms:
`head_on_follow_keeps_full_state_energy_and_momentum` measures full-state energy
over whole substeps, including the integrator and position follow. An in-pass
elastic-energy check would refuse a steady beam-driven push because of the
symplectic integrator's own energy error.
The complete unpowered collision run measures every substep through 12 s:
gain must be at most `0.01 * E(0)`, and the largest later-minus-earlier energy
rise over the window from 8 s through 12 s must be at most `0.01 J`.
These are the existing settle **energy** rules, not a claim that the separate
settle speed rule is tested here. A violation fails; tolerances are not widened.
The tests use `h=0.0005 s`, one single-car substep per advance
(`24000*h=12 s`, `16000*h=8 s`).
Arithmetic re-read with PowerShell: `run_seconds=12`, `settle_start_seconds=8`.

This is the contact kernel only. n20-19b still owns the ordered multi-car
stepping/control API, one-car byte identity, driving-apart independence, full-run
byte determinism, per-car chassis/wheel-mount geometry, full ordered
node/wheel/beam state access and STAGE7 2.4(b) replay integration. None of those
parent completion gates is certified by these contact tests.

## Several cars in one sim

`multi_car::MultiSim` owns cars in the caller's slice order (car 0 first),
fixed for its lifetime. Construction requires at least one wheeled car,
one placement per car, bit-identical `cfg.dt` and `cfg.substeps`, matching
initial output-step counters, and `substep_h` bit-identical to `dt/substeps`.
Config numbers must pass the existing finite-number validation. Placement
components and all resulting node/wheel centres must be finite. Placement is
plain vector addition, once, to every chassis node and wheel centre. An offset
whose three components compare equal to zero is not added at all, preserving
stored negative zero. Construction is not a replacement for `Sim::check` or
the contact kernel's numerical/geometry validation.

One `advance` holds throttle, brake and steer for one output interval (0.02 s
at 50 Hz when configured that way). The road-car tests use 20 substeps per
interval, keeping h below the body's stability limit. Calculation with
PowerShell: `road_substep_seconds=0.001` (`0.02 / 20`).
It checks the controls count and calls
`Car::set_controls` in car order before moving any car. Each physics substep:

1. Snapshots every car's chassis positions, in car order.
2. Calls the existing `Sim::substep(k + 1, h)` for every car in that order,
   including its existing breaking and yielding work, with `h = dt/substeps`.
3. Calls `Sim::check` for every car in order.
4. For two or more cars, calls `apply_contacts_moving` once over ordered car views,
   using those snapshots as `previous_pos`, so step 5 sees the moved positions.
   It supplies the last substep's impulse keys and replaces them only on success.
   Counts are summed across substeps.
5. For every car in order, sets a previously absent `first_contact_s` if a
   chassis node is at/behind a collider plane, to
   `sim.step as f64 * dt + (k + 1) as f64 * h`, just like `Sim::advance`.

After all substeps, every car's output-step counter increments once. A single
car bypasses the contact kernel entirely and reports zero candidates, contacts
and impulses. Its integration path is the existing single-car path; the new
tests compare the full specified state at every output step against a plain
`Sim`, including the first-plane-contact timestamp. Errors name the car where
applicable; contact errors are carried untouched, never converted into zero
contacts. Failure does not roll back work already performed: stop using that
`MultiSim`. A controls refusal moves no car, though earlier controls may have
been assigned.

Contact settings belong to the caller; `MultiSim` stores them, never invents
them. The caller must give `ContactSettings::new` the same timestep as
`substep_h` (settings have no timestep getter). The low-speed contact tests use
skin 0.125 m, restitution 0.0, max_motion 0.015625 m, and max_penetration
0.0625 m. These numbers apply only while `max_motion <= skin/8` covers both
displacement and `speed*h`. The driving-apart road test instead selects skin
1 m, max_motion 0.125 m and max_penetration 0.5 m at h=0.001 s: the speed bound
is 125 m/s, and violations still return errors. Calculation with PowerShell:
`road_speed_bound_m_per_s=125` (`0.125 / (0.02 / 20)`).
Higher-speed racing needs a larger skin chosen and
justified by n20-20, not a swallowed motion refusal.

The short deterministic collision fixture retains the 19a free-flight speeds
and h, but closes the initial gap by an additional 0.01001 m to ensure contact
within 400 steps without targeting tangency at an exact substep boundary.
Calculation with PowerShell:
`0.14 - 0.04*400*0.0005 = 0.132` m (outside skin);
`0.14 - 0.01001 - 0.04*400*0.0005 = 0.12199` m (inside skin without response).
The two-substep variant compares counts and car state with an independent
`Sim::advance` plus kernel driver at every physics substep, not just at the end.

### State words and geometry

`chassis_positions(car)` borrows live node positions in node order.
`wheel_mount_positions(car)` returns one list per wheel, each containing its
live mount-node positions in `WheelParams::mount` order. These are not wheel
centres, a mount centroid or tyre-edge points. Consumers needing a tyre-edge
point must derive it from the exposed sim/wheel geometry; this API itself makes
no tyre-edge approximation.

`state_words()` is read-only and emits this fixed sequence of u64 words:

1. Car count, then output-step index.
2. For each car, starting with car 0: node count, wheel count, beam count.
3. For that car's every node in node order: position x,y,z then velocity x,y,z.
4. For every wheel in wheel order: position x,y,z, velocity x,y,z, spin;
   every deflection vector's x,y,z in index order; every load in index order;
   then suspension length.
5. For every beam in beam order: current rest length (including yielding),
   then broken as 0 or 1. Continue with the next car's counts.

Counts, step index and broken flags are integers. All other words are
`f64::to_bits()`, with no negative-zero normalisation. A car contributes
`3 + 6*nodes + sum_wheels(8 + 3*deflections.len() + loads.len()) + 2*beams`
words, following the two-word global prefix. There are no extra delimiters or
per-wheel deflection/load counts in this contract. It is the specified
node/wheel/beam state stream, not a serialised solver checkpoint or a hash.

Every traversal uses fixed index order, with no randomness, threads, clocks,
addresses or unordered iteration in the state stream. Repeated inputs and
initial state must give identical words; swapping cars changes their ordered
state identity. Independent cars' substep/check operations commute, so merely
reversing those two loops cannot be detected by comparing successful-run
bytes; ordering is also a source-review obligation. The driving-apart test
compares each car with its independently placed single-car run: this is the
in-process premise for n20-20's `ai_inputs_replay_one_car_alone`. CLI replay,
wall policy and state hashing remain outside this module.
