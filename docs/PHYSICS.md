# Crash sim physics

This describes the model in `src/lib.rs`, which is the source of truth. Units are SI (metres, kilograms, seconds, newtons). Everything is deterministic: no random numbers, fixed update order.

## The cube: 8 nodes, 28 beams

Each node is a point mass of `NODE_MASS` = 1.0 kg. Node `i` (0 to 7) sits at a corner of a cube of side `edge` (default 1.0 m), centred on the origin. Bit 0 of `i` picks the x side, bit 1 the y side, bit 2 the z side.

Every pair of nodes is joined by a beam: 8 * 7 / 2 = 28. They group by how many bits the two node indices differ in:

| Bits differing | Kind | Count | Rest length |
|---|---|---|---|
| 1 | cube edge | 12 | `edge` |
| 2 | face diagonal | 12 | `edge` * sqrt(2) |
| 3 | body diagonal | 4 | `edge` * sqrt(3) |

A beam's rest length is the distance between its nodes at the start (after tilt and drop placement; rotation does not change distances). Each node has 7 beams.

Start state: if `tilt` is on (default), the cube is rotated `TILT_DEG` = 5 degrees about x, then 5 degrees about z. It is then moved up so the lowest node is exactly `drop_height` (default 2.0 m) above the ground. All velocities start at zero.

## Beam force

For a beam from node a to node b, let `d = pos_b - pos_a`, `len = |d|`, `dir = d / len`, and `rel_v = (vel_b - vel_a) . dir` (speed at which the ends separate). The force magnitude is

    mag = BEAM_STIFFNESS * (len - rest) + BEAM_DAMPING * rel_v

with `BEAM_STIFFNESS` = 20000 N/m and `BEAM_DAMPING` = 20 N*s/m. The first term is a spring (Hooke's law); the second is a damper that resists the ends moving apart or together. The force `dir * mag` is added to node a and its negative to node b, so a stretched beam pulls its ends together. A beam shorter than 1e-12 m is skipped to avoid dividing by zero.

## Plastic beams (yield)

A material of a `.bsn` body may give a `yield_strain` y (unitless, > 0). Its beams are then plastic; every other beam, and every beam of the built-in cube, is elastic and never changes. After every physics substep (after the solver step, `Sim::apply_yield`), for each plastic beam with current length `len` and rest length `rest`, the strain is

    s = (len - rest) / rest

and when |s| > y + 1e-12 (`YIELD_EPS`, so rounding right after a yield cannot trigger another) the rest length moves so that the strain at the current length returns to the yield limit (perfectly plastic: no hardening, the force never exceeds the one at strain y):

    stretched, s > y:     rest' = len / (1 + y)      so that (len - rest') / rest' = +y
    squeezed,  s < -y:    rest' = len / (1 - y)      so that (len - rest') / rest' = -y

Positions and velocities are not touched, and |len - rest'| < |len - rest|, so the elastic energy (1/2) k (len - rest)^2 goes down at every yield and nothing else changes: plastic flow only removes energy. (It is applied once per substep, so a beam can end a substep slightly past the limit and is brought back at the end of that substep.) The rest length never becomes zero or negative: for len > 0 both formulas give a positive number (a squeeze needs y < 1, and a strain below -1 cannot happen). If a beam yields while its two nodes are at the same place (len below 1e-12 m, where the force step skips it too) the new rest length would be zero: the rest length is left alone, the beam's index goes into `Sim::yield_refused`, and `Sim::check` fails with a message naming the beam. `MAX_STRAIN_LIMIT` still applies to the elastic strain, which for a plastic beam never exceeds y by more than one substep's worth. A beam that would be refused here can still break (see below) if its material has a `break_strain`; it is `check` that reports the refusal.

## Breaking beams

A material of a `.bsn` body may give a `break_strain` b (unitless, > 0, and above its `yield_strain` when both are given: `format::parse` refuses a body that breaks this, and `Sim::from_body` checks it again for a `Body` built another way). Every beam of the built-in cube is unbreakable. After each physics substep (`Sim::apply_breaks`, before the yield pass, so a plastic beam can still reach its break strain), every unbroken beam with |len - rest| / rest > b breaks. Beams are checked in index order, so breaks within one substep are listed by beam index.

- A broken beam applies no force from the next substep on: `Sim::integrate` gives the solver only `Sim::live_beams()`, so the solver (and its force loop) never sees it. It never comes back, whatever the nodes do afterwards. It stays in `Sim::beams` so beam indices do not move; `Sim::broken[i]` marks it.
- The strain checks use only unbroken beams: `Sim::check` (the `MAX_STRAIN_LIMIT` abort) and the `max_beam_strain` column skip broken ones, otherwise the first break (the beam is by then past b, and the nodes may be far apart) would abort the run. The yield pass skips them too. The step limit is checked once, before the run starts, when nothing has broken yet; breaking only removes stiffness, which never lowers the stable step.
- A node left with no beams is still simulated: gravity, ground contact and friction act on it as on any node (`disconnected_node_falls_freely`).
- Break time is the end of the substep where the strain was found past b (the first substep end after the true crossing), the same time base as `first_contact_s`. The reported strain is the absolute value at that moment.
- Deterministic: no state besides positions, velocities, rest lengths and the broken flags; the same input gives the same bytes.

Output (body runs only; the cube's output is byte-for-byte what it was): the CSV gets a last column `broken_beams`, the number of beams broken by that sample's time (`CSV_HEADER_BROKEN`, `to_csv_with_broken`); `--events` prints, after the CSV (or the `--summary` line), one line per break, `break t=<4 decimals> beam=<index> strain=<6 decimals>`, in the order they happened (`events_text`). Tests: `tests/breaking.rs`; the golden for the cube is an FNV-1a hash of its CSV taken from the program before breaking existed.

Total energy for a plastic body uses each beam's current rest length (`settle::total_energy` already does), so the energy before and after a yield can be compared directly (`tests/deform.rs`).

## Bodies from a file

`crash-sim --body <file.bsn>` builds the sim from a `.bsn` body (docs/FORMAT.md) instead of the built-in cube: node positions, masses and start velocities, beam stiffness and damping from the beam's material, rest lengths from the file. `--drop-height`, `--no-tilt` (they set up the cube) and `--settle-report` (it builds its own cube sim; not wired yet) are refused with exit code 2, and so is a body with collider planes (the sim has only the y = 0 ground until nb-36). `--duration`, `--dt`, `--substeps`, `--gravity` and `--summary` work as before and the output columns are the same. The CSV of `--body scenarios/cube.bsn` is byte-identical to the CSV of the default cube.

The step limit of a loaded body cannot use the cube's stiffest-mode numbers, so `solver::body_step_error` bounds both the spring rate and the damping rate of the stiffest mode by Gershgorin: w2 = 2 * (largest sum of the beam stiffnesses at one node) / (lightest mass), g = 2 * (largest sum of the beam dampings at one node) / (lightest mass), and the step must satisfy h^2 w2 + 2 h g < 4 (the cube's coupled condition, see below). For a body of one material this is a bound; for mixed materials it is conservative but not proven. It is stricter than the cube's own limit, so `--body scenarios/cube.bsn` refuses a physics step that the built-in cube still accepts above about 2.9 ms.

A body with wheels (`.bsn` version 2) is refused by `Sim::from_body`, the one constructor every command-line run uses (the CSV run, `--settle-report`, `--crumple-report`, the trajectory writer): `the body has 4 wheels, which this run does not simulate (they would be dropped): ...`, exit code 1. Those commands measure nodes only (the settle report's energy and speed, for example, would not count the wheels), so they refuse a car instead of dropping its wheels silently. Wheels are simulated by `Sim::from_car` (next section); wiring it into the commands is later work (`settle.rs` and `main.rs`).

## Wheels, suspension and tyres

nb-39, `src/vehicle.rs` (the source of truth), built by `Sim::from_car(cfg, body)`. Every number below is a starting value; each constant's reason is in its doc comment in `vehicle.rs`. Units SI; forces in N, torques in N m.

**What a wheel is.** A point mass `m` (the file's `mass`) at its centre, plus a spin `omega` (rad/s) about its axle with inertia `I` (the file's `inertia`). Its centre lives on its *suspension axis*: the line through the centroid `c` of its mount nodes along the chassis' local up axis `u`, at distance `s` (the suspension length) below `c`: `centre = c - u s`. It starts at `Body::wheel_start_center` (s = `rest`), with the mean velocity of its mount nodes and the spin that rolls it without slip.

**The chassis frame.** Fitted every substep from all chassis nodes: the least-squares linear map from the start shape to the current one, applied to +x and +z, gives `ax` and `az`; `u = unit(az x ax)`, forward `f = unit(ax - u (ax . u))`, axle `a = u x f` (positive spin about `a` rolls the car toward +x). For a rigid motion this is the rotation exactly, so a level car has u = (0, 1, 0). A chassis whose nodes lie in one plane is refused at the start; a frame that collapses during a run makes `Sim::check` fail.

**Suspension (along the axis).** Force `F_s = k (rest - s) - c_d ds/dt + stop`, positive pushing the wheel away from the mount: `-u F_s` on the wheel, `+u F_s` shared equally over the mount nodes (equal shares are what the virtual work of a spring on a plain centroid gives). `k` = `spring`, `c_d` = `damper`. The travel limits are stiff springs: below `travel_min` the stop adds `k_stop (travel_min - s)`, above `travel_max` it adds `-k_stop (s - travel_max)`, with `k_stop = STOP_STIFFNESS_FACTOR x k` (20).

**Across the axis: a rigid carrier.** Across `u` the wheel moves with its mount. In `pre_step` the part of the wheel's applied force (gravity, suspension, tyre) across `u` goes straight to the mount nodes (equal shares), together with its moment about `c` (the carrier is rigid, so the force reaches the chassis at the wheel centre, not at the mount). In `post_step` a constraint puts the wheel back on the axis: its offset from the axis and its velocity across the axis relative to the axis point it sits at (`dc/dt - s du/dt`) are removed, each correction shared between wheel and mount nodes by mass, so momentum and the centre of mass are unchanged. What is left there is the wheel's own inertia, and the moment of that residual impulse reaches the chassis in the next substep. A moment on the chassis is applied as a rigid-body share over all its nodes: node i gets `m_i (J^-1 T) x r_i` (J the chassis inertia tensor about its centre of mass), which adds no net force and exactly the torque T.

**Tyre contact, with each collider plane.** The tyre is the sphere of its `radius` around the wheel centre (its `width` is not used). Penetration `pen = radius - distance(centre, plane)`; with `pen > 0` the normal load is `Fz = max(0, k_r pen - c_r v . n)` (`k_r` = `radial_stiffness`, `c_r` = `radial_damping`, `n` the plane normal), acting along `n` on the wheel centre.

**Tyre forces in the plane: a relaxation-length (brush) model.** The contact patch carries a deflection `d` (m, in the plane), which follows the patch's slip velocity `v_s` (the velocity of the tyre's lowest point, `v + omega a x (-r n)`, in the plane) and relaxes over the rolling distance `sigma` = `TYRE_RELAXATION_LENGTH` (0.3 m):

    dd/dt = v_s - (V / sigma) d          V = speed of the wheel centre along the plane

(integrated as `d' = (d + h v_s) / (1 + h V / sigma)`). The force on the tyre is

    F = -(F_ref / sigma) (C_x (d_x + tau dd_x/dt) e_x + C_y (d_y + tau dd_y/dt) e_y),   then capped:  |F| <= mu Fz

with `e_x = unit(a x n)` the rolling direction, `e_y = n x e_x` the lateral one, `C_x` = `long_stiffness`, `C_y` = `lat_stiffness`, `mu` = `mu`, and `tau` = `TYRE_DAMPING_TIME` (0.01 s). When the cap binds the tyre slides and `d` is cut back to the deflection whose spring force is `mu Fz`. In steady rolling `d = sigma v_s / V`, so `F_x = C_x F_ref kappa` (kappa the slip ratio `(omega r - V_x) / |V_x|`) and `F_y = -C_y F_ref tan(alpha)`: the file's slopes, before the cap. At rest `V = 0` and the patch is a spring of rate `C F_ref / sigma` (147 kN/m longitudinally for `car_basic.bsn`), so a braked car holds on a slope with no creep. `F` acts on the wheel centre, with the torque `(-r n x F) . a` on the spin.

`F_ref` is the tyre's *reference load*, the static load per wheel (total mass x g / wheel count, 2943 N for `car_basic.bsn`), not the instantaneous `Fz`. This keeps the patch a spring of fixed rate with a well-defined stored energy: with `Fz` in its place the patch stiffness changes with the load at fixed `d`, which pumped energy in and out of a held car (measured: a 0.08 J rise after settling, against the 0.01 J allowed). So the slip slope does not grow with load; only the cap `mu Fz` does.

**Brakes.** No input yet (nb-40): `Car::brake` is a flag. While it is on, every wheel with `braked yes` gets an equal share of `max_brake_torque` (`T_b`). The brake acts on the spin *relative to the chassis* (the caliper turns with the chassis; the chassis' spin rate about the axle is taken from how far its forward axis turned): in a substep it can take up to `T_b h / I` off the relative spin, stopping it at most, never reversing it. What it takes from the spin, the chassis gets back about the axle in the same substep. (A brake on the absolute spin did work against the chassis' pitch and raised a held car's energy by 0.025 J after settling.)

**Order in a substep** (`Sim::integrate` when the sim has a car or a gravity tilt): `Car::pre_step` (frame; suspension, tyre and brake forces from the state at the start of the substep; reactions and moments into the solver's extra forces) -> the solver's step with those extra forces (`Solver::step_with_forces`; `SymplecticEuler` adds them into its own force sum) -> `Car::post_step` (wheel velocity then position, spin, the axis constraint, the patch deflections) -> plane contact for the nodes (`collide::apply`, unchanged). The cube and any body without wheels or tilt take the old path, operation for operation (`default_cube_output_unchanged`).

**Gravity tilt (slopes).** `Sim::set_gravity_tilt(theta)` makes gravity `g (sin theta, -cos theta, 0)`: on the flat ground that is a slope of angle theta, downhill +x. `vehicle::slope_angle(body, f)` gives `theta = atan(f mu)` (docs/STAGE6.md check 3.4) and refuses a body without tyres or with tyres of different `mu`. The solver gets the y part as its `gravity` and the x part as an extra force on every node.

**Step limit.** `Sim::from_car` refuses a step `h` that is not below the Gershgorin bound of the chassis together with the wheels (`solver::step_error_with`, the `body_step_error` rule with more degrees of freedom): each wheel adds its centre (mass `m`) and its spin as rim travel (mass `I / r^2`) to the lightest-mass search, and these elements: the suspension at its stiffest (`k + k_stop`, damping `c_d`) between the wheel and its mount centroid; the radial spring and damper to the ground; and the patch spring at its stiffest (`max(C_x, C_y) F_ref / sigma`, damping that times `tau`) between the wheel centre and its rim. For `car_basic.bsn` at h = 1 ms the binding entries are a mount node (17 beams plus the suspension): w2 = 2 x (5.1e6 + 349755) / 14, g = 2 x (3400 + 1272) / 14, h^2 w2 + 2 h g = 0.78 + 1.33 = 2.11 < 4, limit 1.57 ms against the chassis alone's 1.77 ms (*reasoned*, computed from the file; `car_step_limit_counts_suspension_and_wheels` checks that a 50 MN/m tyre or a near-massless spin is refused).

**Energy** (`Sim::total_energy`): the node and beam terms of `settle::total_energy` with potential `-m g_vec . r`, plus per wheel `(1/2) m |v|^2 + (1/2) I omega^2 - m g_vec . centre`, the suspension spring `(1/2) k (rest - s)^2` and stop `(1/2) k_stop overrun^2`, the tyre's radial spring `(1/2) k_r pen^2`, and the patch spring `(1/2) (F_ref / sigma) (C_x d_x^2 + C_y d_y^2)`.

**What the model ignores** (known, not fixed): the tyre is a sphere (no width, no camber, no tyre shape on a wall); the slip slope does not depend on load (see above), and there is no combined-slip shape beyond the friction circle, no peak-then-fall of the slip curve, no rolling resistance, no load-dependent mu; the suspension axis is the chassis' fitted up axis through the mount centroid (no linkage geometry, camber or toe change, anti-dive or anti-squat); there is no steering and no drive torque (nb-40); moments on the chassis are spread as a rigid-body share over the whole chassis, not only near the mount; the residual moment of the axis constraint reaches the chassis one substep late; `Sim::first_contact_s` counts nodes only, not tyres. `Car::check` fails the run on a non-finite wheel state or a lost frame.

**Tests** (`tests/vehicle.rs`; docs/STAGE6.md checks 3.3 and 3.4, with the settle numbers of check 3.2 measured at every 1 ms substep: after T = 8 s max speed (nodes, wheel centres and rims) below V = 0.001 m/s, energy rise at most E = 0.01 J, gain at most G = 0.01 of E(0)):
- `car_settles_level`: `car_basic.bsn` dropped from 0.2 m, free-rolling, 12 s. Measured: max speed after 8 s 5.4e-7 m/s, energy rise 2.6e-6 J; (a) loads 11772.000 N = M g; (b) all four loads 2943.000 N and compressions 0.159033 m; (c) compression 0.159033 m against statics 0.159033 m, off both stops; (d) front axle 5886.000 N against the moment balance 5886.000 N; (e) every corner 270.00 kg, 1.250 Hz, damping ratio 0.300; up = (7.6e-17, 1.000000000, 2.1e-16).
- `braked_car_holds_below_grip_slope`: braked, tan theta = 0.8 mu, 12 s. Measured: max speed after 8 s 3.9e-4 m/s, energy rise 1.6e-6 J; loads sum 9625.873 N against M g cos theta 9625.875 N; front (downhill) axle 6370.3 N against the moment balance about the rear contacts 6333.6 N (0.6%, 2% allowed).
- `braked_car_slides_above_grip_slope`: braked, tan theta = 1.2 mu, 4 s; chassis centre-of-mass acceleration along x from 2 to 4 s 1.1667 m/s^2 against g (sin theta - mu cos theta) = 1.1872 m/s^2 (1.7%, 15% allowed); every rim speed 8.1e-4 m/s while sliding at 5.45 m/s (the wheels turn only with the chassis' pitch).
- Also: `unbraked_car_rolls_on_the_hold_slope`, `car_step_limit_counts_suspension_and_wheels`, `body_with_wheels_is_refused_where_wheels_are_not_simulated` (library and both CLI paths), `slope_angle_follows_grip_and_refuses_what_has_no_grip`, `from_car_refuses_bad_wheel_numbers`, `free_fall_keeps_momentum_and_wheels_on_their_axes` (also with gravity tilted 0.5 rad, where the carrier must take the wheels across their axes).

### Tyre tuning log (nb-44, the skidpad)

Target and method are fixed in `docs/STAGE7.md` ("The skidpad", "The numbers, and why") before any try: `limit_g` 0.830 to 0.930 g (*sourced* there), measured by `tests/skidpad.rs` from the chassis centre-of-mass motion only, over the last 5 s (50 blocks of 0.1 s) of each throttle level, steady iff v_b, r_b and rho_b each vary at most 2% and the body slip angle stays under 10 degrees; a_g = mean(v_b) x mean(r_b) / g. `scenarios/skidpad.input`: steer 0.1588 (kinematic radius 2.5 m / tan(0.1588 x 30 deg) = 30.00 m), 60 throttle levels 0.005 to 0.300 in steps of 0.005, 30 s each, brake 0. One row per try; the tyre data is the four `tyre` lines of `car_basic.bsn`.

| Try | Tyre parameters (all four tyres) | `limit_g` (x1.0) | limit level, speed, radius | x0.8 / x1.2 `limit_g` | Balance | Change |
|---|---|---|---|---|---|---|
| 1 | mu 0.88, radial_stiffness 200000, radial_damping 500, long_stiffness 15, lat_stiffness 10 (as merged by nb-39) | 0.842 | level 34 of 60 (throttle 0.170), 59.0 km/h, 32.52 m | 0.683 (ratio 0.811) / 0.992 (ratio 1.178) | understeer in all three | no change needed: 0.842 is inside 0.830 to 0.930 and under the ceiling 1.02 x 0.88 = 0.898 |

What the run shows (not tuned, recorded): the limit is about 0.96 mu (0.842 / 0.88). A probe with every tyre at mu 0.70 (a temporary edit of `skidpad.bsn` only, reverted, never committed) measured 0.679 (0.97 mu), so the limit moves with mu, as the x0.8 and x1.2 copies also show. Below the limit the circle stays within 0.5% of 30.15 m; past it the car runs wide (understeer) with a_g falling slowly, 0.842 to 0.836 over the next 7 steady levels. Because the model has no aero drag or rolling resistance, the levels well past the limit keep accelerating onto ever larger circles (the x0.8 copy reaches about 1400 km/h on a 24 km circle at throttle 0.300, and its window still reads steady); those levels are below the limit in a_g, so they do not change `limit_g`, but the speeds there are not physical.

## Gravity

Each node gets a downward force of `gravity * NODE_MASS`. The default `gravity` is `GRAVITY` = 9.81 m/s^2; it is a config value (the code comments mention Moon 1.62 and Mars 3.71).

## Ground contact

The ground is the plane y = 0. After a node's position is updated, if `y < 0`:

1. The node is put back at y = 0.
2. If it is moving down with speed `vy`: when `-vy > REST_SPEED` (0.2 m/s) its vertical velocity becomes `-vy * RESTITUTION` (`RESTITUTION` = 0.5, so it keeps half its downward speed, now upward). Otherwise the vertical velocity is set to 0 (resting contact, no bounce).
3. Horizontal velocity (x and z) is multiplied by `max(0, 1 - FRICTION_RATE * h)`, where `FRICTION_RATE` = 5.0 per second and `h` is the substep. This is a simple speed-loss rate, not Coulomb friction: it does not depend on how hard the node presses on the ground.

Contact only triggers when a node ends a substep below the ground. Beams are never in contact with the ground, only nodes.

## Plane contact (the ground and walls)

The ground is one of a list of collider planes (`src/collide.rs`). A plane is a point and a unit normal; the side the normal points to is free space. A node with signed distance `d = (pos - point) . n < 0` after the position update is behind the plane, and the rule above is applied along the normal:

1. `pos += n * (-d)` (put back on the plane);
2. the normal velocity `vn = vel . n`: if `vn < 0` and `-vn > REST_SPEED`, it becomes `-vn * RESTITUTION`; if `vn < 0` and slower, 0; if `vn >= 0` it is kept;
3. the velocity along the plane is multiplied by `max(0, 1 - FRICTION_RATE * h)`.

For the ground (point 0, normal (0, 1, 0)) this is the rule above operation for operation, and a test (`ground_plane_collider_keeps_cube_output`) compares it bit for bit with a copy of the old in-solver code over the whole default run. The rule is applied by `Sim` after every solver step, for every node and every plane in order (the ground first, then the body's `plane` lines in file order); it is not part of a solver, so every solver gets the same contact. It uses only that node's own state, so applying it after the solver's whole loop gives the same numbers as applying it inside the loop. A wall drags a node sliding along it with the same loss rate as the ground. Planes are infinite, and only nodes touch them.

Limits of the rule (known, not fixed): the planes are infinite, so no node can tunnel through one, but a node that ends a substep deep behind a plane is moved straight along the normal onto it, whatever path it took (no sweep test; 10 m/s at 1 ms is 1 cm per substep). Two planes that meet at a corner are applied one after the other in list order, so a node in a corner can be corrected by both in the same substep, the later plane last.

Impacts (settle report): `collide::impact_this_substep` is the plane version of "a node sits on the surface and moves away"; `settle.rs` calls it too, so `last_impact_s` in the settle report counts impacts with every collider plane, walls included.

## The box_wall material

`scenarios/box_wall.bsn` (milestone 2, `docs/STAGE6.md`) uses the nb-33 box (12 nodes of 1 kg, 50 beams) with one material, `box_beam`:

| Number | Value | Source or reason |
|---|---|---|
| `stiffness` | 20000 N/m | The cube's `BEAM_STIFFNESS`, so the step-limit reasoning (the cube: 1 ms against 4.1 ms) carries over. The box's most loaded node has 11 beams, so Gershgorin gives omega_max <= sqrt(2 * 11 * 20000 / 1) = 663 rad/s and a limit of 3.0 ms, again against 1 ms. Not a real material's stiffness: a lumped beam stands for a whole panel. |
| `damping` | 20 N s/m | The cube's `BEAM_DAMPING`, same reason; it takes out impact ringing within the 10 s run. |
| `yield_strain` | 0.02 | A choice made before any run, not measured and not tuned. Real structural steel yields at about 0.2% (250 MPa / 210 GPa), but a lumped beam here stands for a whole panel whose crush force is set by buckling, so the material value does not apply. 0.02 gives a crush force per 1 m beam of `k * strain * length` = 20000 * 0.02 * 1 = 400 N. The box has 600 J of kinetic energy at 10 m/s; the four front nodes lose about 150 J at the wall's restitution (their 50 J becomes 12.5 J each), so about 450 J is left for the plastic work. With roughly 12 beams' worth of force along x at 400 N (about 5 kN), that is a crush of about 450 / 5000 = 0.09 m, an estimate good to a factor of about 2, inside the 0.02 to 1.0 m the check allows by a factor of 4 on one side and 10 on the other. That margin is for the estimated deepest crush; the gated number is the crumple kept after settle-by (rule 1, floor 0.02 m), which measured 0.024905 m, only about 1.25 times above the floor. |
| `break_strain` | none | The box crumples without breaking in this check (breaking is gated by check 2.9 on its own scenario). |

The speed 10 m/s is 50 J/kg, half of the 96 J/kg of a car at 50 km/h (`docs/STAGE6.md`). If a measured value breaks a limit of `docs/STAGE6.md`, the limit is not loosened: report it.

### Sensitivity runs

Measured 2026-11-20 on the merged base (h = 1 ms, `--duration 10`, the STAGE6 2.3 flags). `yield_strain` stayed 0.02 in every file; only the one number named below changed.

| file | what changed | `crumple_delta_m` (kept) | `crumple_delta_max_m` (deepest) | `crumple:` |
|---|---|---|---|---|
| `box_wall` (nominal) | none | 0.024905 | 0.095943 | ok |
| `box_wall_soft` | stiffness x 0.5 | 0.067613 | 0.141399 | ok |
| `box_wall_stiff` | stiffness x 2 | 0.012583 | 0.065681 | FAIL (below crumple-min, not gated) |
| `box_wall_slow` | speed 10 -> 7.07 m/s (half the energy) | 0.011589 | 0.061272 | FAIL (below crumple-min, not gated) |
| `box_wall_elastic` | no `yield_strain` | -0.000124 | 0.084893 | FAIL (check 2.4: the elastic box does not stay crumpled) |
| `box_wall` at `--substeps 20` | h = 0.5 ms | 0.024095 | 0.094973 | ok (3.3% from nominal, check 2.6 allows 10%) |

The result moves the way the physics says: a softer beam yields at a lower force, so the same energy crushes the box further (soft 0.0676 > nominal 0.0249 > stiff 0.0126); half the energy crushes it less (slow 0.0116 < nominal). The elastic box springs back to within 0.000124 m of its start length, so the permanent crumple comes from the plasticity and not from the metric. The nominal numbers were not changed after any run: `yield_strain` 0.02 and the estimate of about 0.09 m deepest crush were written before the first run, and the measured 0.0959 m agrees.

## Integrator: symplectic Euler with substeps

One output step of `dt` (default 0.01 s) is split into `substeps` (default 10) physics substeps of `h = dt / substeps` (default 1 ms). In each substep:

1. Compute all forces (gravity and beams) from the current state.
2. `vel += force * (h / NODE_MASS)`.
3. `pos += vel * h`, using the new velocity (this ordering is what "symplectic" or semi-implicit means; it keeps oscillation energy from growing the way plain Euler does).
4. Apply plane contact (the ground and any walls) as above; `Sim` does this after the solver's step.

### Why the step limit is what it is

A spring with angular frequency `omega` = sqrt(k/m) is stable under symplectic Euler only if `omega * h < 2`. The stiffest vibration of the cube bounds this. With 7 beams per node, Gershgorin's circle theorem gives a largest eigenvalue of the stiffness matrix of at most `2 * 7 * k / m`, so

    omega_max <= sqrt(14 * BEAM_STIFFNESS / NODE_MASS) = sqrt(280000) = about 529 rad/s
    max_stable_step() = 2 / omega_max = about 0.00378 s

`Config::validate()` refuses to run when `dt / substeps` is larger than `max_stable_step()`, and tells you to raise `--substeps` or lower `--dt`. It also rejects non-finite values, `dt <= 0`, `duration < 0`, `substeps < 1`, `drop_height < 0`, and `gravity <= 0`. The default 1 ms substep is about a quarter of the limit.

### The damping limit

Damping lowers the stable step, and `validate()` enforces the combined condition itself through `damped_step_error(h, k, c, m)`, which uses `stable_step_limit(k, c, m)`. Derivation, for the code's own update (`vel` first, then `pos` with the new `vel`):

1. In one vibration mode with spring rate `w2` (omega squared) and damping rate `g`, a step is `v' = (1 - h*g) v - h*w2 x`, `x' = x + h v'`. Its determinant is `1 - h*g` and its trace is `2 - h*g - h^2*w2`.
2. The Jury stability test gives two conditions: (A) `0 < h*g < 2` and (B) `h^2*w2 + 2*h*g < 4`.
3. Every beam's damper acts along the same direction as its spring, so the damping matrix is the stiffness matrix times `c/k`. The stiffest mode has eigenvalue 8 (`omega^2 = 8k/m`, 400 rad/s; checked numerically by `stiffest_mode_eigenvalue_is_eight`), so there `w2 = 8k/m` and `g = 8c/m`.
4. The largest `h` meeting (B) is the positive root `h = (-g + sqrt(g^2 + 4*w2)) / w2`. At `h = 2/g`, (B) would read `4*w2/g^2 + 4 < 4`, which is false when `w2 > 0`, so that root is always below `2/g`: (B) binds whenever there is a spring, and (A) only if `w2 = 0`. `stable_step_limit` returns the smaller of the two and says which binds; `validate()` refuses `h >= limit` with a message naming h, the limit and the binding condition.
5. Numbers for `k = 20000`, `m = 1`: `160000*h^2 + 16*c*h = 4`. The limit is 0.00500 s at `c = 0` (the undamped `2/omega`), 0.00410 s at `c = 20` (today's `BEAM_DAMPING`), 0.00339 s at `c = 40`, and falls with c.
6. The old separate test `c*h/m < 0.25` is only condition (A) for the damper alone. It is not enough: it is much looser than (B). At `c = 40` the old checks accepted `h = 0.0036 s`, but the limit is 0.003385 s and the stiffest mode diverges (`old_separate_checks_accepted_an_unstable_step_at_c_40`). An earlier version of this document wrongly said damping would only matter past `c` of about 66; it matters from about 28.

`max_stable_step()` (0.00378 s) is kept, and `validate()` checks it too: it is a stiffness-only Gershgorin bound that holds for any deformed shape, while the stiffest-mode limit uses the rest-shape eigenvalue 8. With today's constants the coupled limit at `c = 20` is 0.0041 s, above 0.00378 s, so the Gershgorin bound trips first and the coupled error is not reachable through `validate()` today. The two cross at `c` of about 28.3: above that, the coupled limit is the tighter one. `damped_step_error` takes `k`, `c` and `m` as arguments so it is tested for other constants (`tests/stability.rs`), and the limit is checked on a single damped oscillator stepped with the same integrator: bounded just below it, unbounded just above, for `c` = 0, 20, 40, 66 and 200.

## CSV output

`run` returns one sample at t = 0 and one after each output step. The step count is `step_count` = `floor(duration / dt)` with a relative epsilon of 1e-9 (so 5.0 / 0.01 stays 500), so the last row never lies after `duration` and can lie up to `dt` before it. A `dt` below `MIN_DT` = 0.0001 s is refused, because the time column has 4 decimals and its rows would repeat. `to_csv` writes the header `t,min_y,center_y,max_beam_strain` (`CSV_HEADER`) and one row per sample:

| Column | Meaning | Decimals |
|---|---|---|
| `t` | time in seconds (step number * `dt`) | 4 |
| `min_y` | height of the lowest node, metres | 6 |
| `center_y` | mean y of all 8 nodes, metres | 6 |
| `max_beam_strain` | largest beam strain over all 28 beams (a body's broken beams are skipped) | 6 |

Strain of a beam is `|len - rest| / rest`, so 0.01 means 1% off its rest length.

## Safety checks

After every output step (not every substep), `Sim::check()` fails the run if:

- any node position or velocity component is NaN or infinite;
- the max strain (over unbroken beams) is NaN, or exceeds `MAX_STRAIN_LIMIT` = 0.5 (50%). The strain calculation propagates NaN, so a blown-up state can never read as 0.

Contact time is tracked separately, in the substep loop: `Sim::first_contact_s` is the time of the first substep at which any node is at or behind any collider plane, ground or wall (0 if the body starts there). `--summary` prints it, so it does not depend on the output sampling, which can miss a touchdown that begins and ends between two samples.

`run_checked` then stops and returns the good samples so far plus an error reading `unstable at t = ... s: ...`. A config that `validate()` accepts can still abort here, for example `--gravity 5000` crushes the cube on impact (`tests/risky.rs`). A config refused by `validate()` returns no samples and the validation error. `run` is the same but panics on any error.
