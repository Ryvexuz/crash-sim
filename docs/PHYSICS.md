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

and the rest length moves (perfectly plastic: no hardening) in tension when s > y + 1e-12 (`YIELD_EPS`, so rounding right after a yield cannot trigger another) and in compression by the rule below. Tensile yield is unchanged: `rest' = len / (1 + y)`, so that `(len - rest') / rest' = +y`. Compressive yield alone changed: a beam is crushed past yield when `(len - rest) / rest0 < -(y + YIELD_EPS)`, where `rest0` is its original rest length, and its new rest length is `rest' = len + y*rest0`, so the force stays constant at `k*y*rest0`.

Positions and velocities are not touched, and |len - rest'| < |len - rest|, so the elastic energy (1/2) k (len - rest)^2 goes down at every yield and nothing else changes: plastic flow only removes energy. (It is applied once per substep, so a beam can end a substep slightly past the limit and is brought back at the end of that substep.) The rest length never becomes zero or negative: for len > 0 both formulas give a positive number. If a beam yields while its two nodes are at the same place (len below 1e-12 m, where the force step skips it too) the new rest length would be zero: the rest length is left alone, the beam's index goes into `Sim::yield_refused`, and `Sim::check` fails with a message naming the beam. `MAX_STRAIN_LIMIT` still applies to the elastic strain, which for a tensile-yielded beam never exceeds y by more than one substep's worth. A beam that would be refused here can still break (see below) if its material has a `break_strain`; it is `check` that reports the refusal.

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

**What the model ignores** (known, not fixed): the tyre is a sphere (no width, no camber, no tyre shape on a wall); the slip slope does not depend on load (see above), and there is no combined-slip shape beyond the friction circle, no peak-then-fall of the slip curve, no load-dependent mu; the suspension axis is the chassis' fitted up axis through the mount centroid (no linkage geometry, camber or toe change, anti-dive or anti-squat); there is no steering and no drive torque (nb-40); moments on the chassis are spread as a rigid-body share over the whole chassis, not only near the mount; the residual moment of the axis constraint reaches the chassis one substep late; `Sim::first_contact_s` counts nodes only, not tyres. **Rolling resistance and aero drag are modelled** (the `rolling_resistance` and `drag_area` keys of the `controls` line, 0 when absent, so a car without them is unchanged): each wheel's spin gets a moment `c_rr * N * r * tanh(|spin| r / v0)` against the spin (`rolling_resistance_moment`; N the wheel's normal load, r its radius, v0 = `ROLLING_RESISTANCE_RAMP_SPEED` = 0.1 m/s), which is 0 at zero spin, can stop the spin within a substep but never reverses it, and acts against the ground, so it is not in the chassis torque; and the chassis nodes get the drag `0.5 * AIR_DENSITY * CdA * |v| v` against the chassis centre-of-mass velocity v (`add_drag`, `AIR_DENSITY` = 1.225 kg/m^3, CdA = `drag_area`), shared by node mass so every node gets the same deceleration, which gives no pitch moment (it acts at the centre of mass, not at a centre of pressure). `car_basic.bsn`, `pad_and_wall.bsn` and `skidpad.bsn` use `rolling_resistance 0.012 drag_area 0.65`. `Car::check` fails the run on a non-finite wheel state or a lost frame.

**Tests** (`tests/vehicle.rs`; docs/STAGE6.md checks 3.3 and 3.4, with the settle numbers of check 3.2 measured at every 1 ms substep: after T = 8 s max speed (nodes, wheel centres and rims) below V = 0.001 m/s, energy rise at most E = 0.01 J, gain at most G = 0.01 of E(0)):
- `car_settles_level`: `car_basic.bsn` dropped from 0.2 m, free-rolling, 12 s. Measured: max speed after 8 s 5.4e-7 m/s, energy rise 2.6e-6 J; (a) loads 11772.000 N = M g; (b) all four loads 2943.000 N and compressions 0.159033 m; (c) compression 0.159033 m against statics 0.159033 m, off both stops; (d) front axle 5886.000 N against the moment balance 5886.000 N; (e) every corner 270.00 kg, 1.250 Hz, damping ratio 0.300; up = (7.6e-17, 1.000000000, 2.1e-16).
- `braked_car_holds_below_grip_slope`: braked, tan theta = 0.8 mu, 12 s. Measured: max speed after 8 s 3.9e-4 m/s, energy rise 1.6e-6 J; loads sum 9625.873 N against M g cos theta 9625.875 N; front (downhill) axle 6370.3 N against the moment balance about the rear contacts 6333.6 N (0.6%, 2% allowed).
- `braked_car_slides_above_grip_slope`: braked, tan theta = 1.2 mu, 4 s; chassis centre-of-mass acceleration along x from 2 to 4 s 1.1667 m/s^2 against g (sin theta - mu cos theta) = 1.1872 m/s^2 (1.7%, 15% allowed); every rim speed 8.1e-4 m/s while sliding at 5.45 m/s (the wheels turn only with the chassis' pitch).
- Also: `unbraked_car_rolls_on_the_hold_slope`, `car_step_limit_counts_suspension_and_wheels`, `body_with_wheels_is_refused_where_wheels_are_not_simulated` (library and both CLI paths), `slope_angle_follows_grip_and_refuses_what_has_no_grip`, `from_car_refuses_bad_wheel_numbers`, `free_fall_keeps_momentum_and_wheels_on_their_axes` (also with gravity tilted 0.5 rad, where the carrier must take the wheels across their axes).

### Tyre tuning log (nb-44, the skidpad)

Target and method are fixed in `docs/STAGE7.md` ("The skidpad", "The numbers, and why") before any try: `limit_g` 0.830 to 0.930 g (*sourced* there), measured by `tests/skidpad.rs` from the chassis centre-of-mass motion only, over the last 5 s (50 blocks of 0.1 s) of each throttle level, steady iff v_b, r_b and rho_b each vary at most 2% and the body slip angle stays under 10 degrees; a_g = mean(v_b) x mean(r_b) / g. `scenarios/skidpad.input`: steer 0.1588 (kinematic radius 2.5 m / tan(0.1588 x 30 deg) = 30.00 m), 52 throttle levels 0.005 to 0.260 in steps of 0.005, 20 s each (the 20 s minimum of STAGE7.md), brake 0. The three grip runs are independent and run on three threads. One row per try; the tyre data is the four `tyre` lines of `car_basic.bsn`. Try 1 used the 52-level script above; from try 2 `scenarios/skidpad.input` has 60 levels (throttle 0.005 to 0.300, 20 s each, the last line `1180 0.3 0 0.1588`), because with drag the limit level moved from 34 to 41.

The script was first run with 60 levels (to 0.300) of 30 s each, then shortened to cut test time; that changed the script, not the car, and is not a tuning try. With 30 s holds the result was `limit_g` 0.842 at level 34 (59.0 km/h, 32.52 m), x0.8 0.683 at level 22, x1.2 0.992 at level 49: the same three limits and limit levels as with 20 s holds. Per level, the 20 s and 30 s windows agree within 0.003 g from level 14 up in all three runs (within 0.001 g from level 25 up), and print the same a_g at all three limit levels. Below that the 20 s windows read lower (level 8, x1.0: 0.430 against 0.455 g), and levels 6 and 7, steady at 30 s, are not steady at 20 s: the slow climb from rest has not finished by 15 s. Those low levels are far below every limit, so they cannot set `limit_g`; at 30 s levels 1 to 5 were not steady either. The 52nd level is three levels above the x1.2 limit level (49), one more than the K - 2 rule needs, so a one-level shift of that limit does not break the rule. Levels never depend on later ones, so the first 52 levels of the 60-level 20 s run printed exactly the same lines.

| Try | Tyre parameters (all four tyres) | `limit_g` (x1.0) | limit level, speed, radius | x0.8 / x1.2 `limit_g` | Balance | Change |
|---|---|---|---|---|---|---|
| 1 | mu 0.88, radial_stiffness 200000, radial_damping 500, long_stiffness 15, lat_stiffness 10 (as merged by nb-39) | 0.842 | level 34 of 52 (throttle 0.170), 58.9 km/h, 32.35 m | 0.683 (ratio 0.811) / 0.992 (ratio 1.178) | understeer in all three | no change needed: 0.842 is inside 0.830 to 0.930 and under the ceiling 1.02 x 0.88 = 0.898 |
| 2 | unchanged (the same four `tyre` lines as try 1) | 0.834 | level 41 of 60 (throttle 0.205), 59.0 km/h, 32.86 m | 0.677 (ratio 0.812) / 0.982 (ratio 1.177) | understeer in all three | rolling resistance 0.012 + drag 0.65 m^2 added, tyres unchanged, skidpad.input levels to 60: limit_g 0.834 (inside 0.830 to 0.930 and under 0.898) |

What the try 1 run shows (not tuned, recorded; the car then had no drag or rolling resistance): the limit is about 0.96 mu (0.842 / 0.88). A probe with every tyre at mu 0.70 (a temporary edit of `skidpad.bsn` only, reverted, never committed) measured 0.679 (0.97 mu), so the limit moves with mu, as the x0.8 and x1.2 copies also show. Up to about 0.67 g (x1.0, steady levels 8 to 17) the circle stays within 0.1% of 30.15 m, and from there it widens slowly to 32.35 m at the limit; past it the car runs wide (understeer) with a_g falling slowly, 0.842 to 0.839 over the next 3 steady levels (levels 35 to 37). Because that model had no aero drag or rolling resistance, the levels well past the limit kept accelerating onto ever larger circles (the x0.8 copy reached about 577 km/h on a 3.9 km circle at throttle 0.260; with the earlier 30 s holds to throttle 0.300 it reached about 1400 km/h on a 24 km circle, and that window still read steady); those speeds were not physical.

What the try 2 run shows (`scratch/s7-1-3-t230302.txt`, the STAGE7 1.3 command): the car now has rolling resistance and drag, so the speeds past the limit are bounded. The limit moved from 0.842 to 0.834 (0.948 mu) at level 41, with the circle at 32.86 m (30.31 m at level 24, 0.670 g); past it a_g falls slowly, 0.834 at level 41 to 0.817 at level 60, where the x1.0 run is at 120.8 km/h on a 140.45 m circle and the x0.8 run at 164.3 km/h on a 324.43 m circle (a_g 0.655, below its limit 0.677 at level 28). The x1.2 limit is 0.982 at level 56 (steady_levels 53 in all three runs, all `limit_balance=understeer`); the four tests passed (`test result: ok. 4 passed; 0 failed`).

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

## Solids (the m5 crash-zone solver)

The method of `docs/M5-SOLVER.md` (sections 4, 5, 8 and 9, requirements R1 to R12 except R8): explicit linear tetrahedra with compressible Neo-Hookean elasticity and a compressive crushable plasticity with lock-up, on the same nodes as the beams, with one semi-implicit Euler update for every node. Code: `src/solver_m5.rs`; tests: `tests/solver_m5.rs`. No command-line run uses it yet (`--solver m5`, the `_m5` files and the calibration are task n20-11c); the default solver and every existing output are unchanged.

**Who runs it.** `SolverM5::from_body(&body, h)` builds it for a body (a `.bsn` version 3 file, `docs/FORMAT.md`) and the physics step `h = dt / substeps`; `Sim::from_body_with_solver` or `Sim::from_car_with_solver` then runs it on the same body. The `Solver` trait has two defaulted methods for it (`src/solver.rs`): `handles_solids()` (false by default) and `check()` (None by default). `Sim::build_from_body`, which every body constructor passes through, refuses a body with tets when the solver's `handles_solids()` is false, with `the body has <n> tets, which this solver does not simulate (they would be dropped): ...`, so `Sim::from_body` (the default `SymplecticEuler`) refuses them. `Sim::check` returns the solver's `check()` error first, before its node and beam checks, so a run ends on a tet error the way it ends on a beam strain.

**Kinematics.** A tet on nodes 0 to 3 has the rest edge matrix `Dm = [X1 - X0, X2 - X0, X3 - X0]` (columns, from the file's positions) and rest volume `V = det(Dm) / 6 > 0`. Its deformation gradient is `F = Ds Dm^-1` with `Ds = [x1 - x0, x2 - x0, x3 - x0]` (constant-gradient tet). `F = Fe Fp` with the plastic part `Fp` stored per tet (`TetEl::fp`, identity at the start); the solver also stores `b = Dm^-1 Fp^-1`, so the elastic part is `Fe = Ds b` (`SolverM5::add_tet_forces`). Row k of `b` is the gradient `g_k` of node k (k = 1, 2, 3) in the plastic intermediate shape, and `g_0 = -(g_1 + g_2 + g_3)`.

**Elastic law** (R4a). The energy per rest volume is

    W(Fe) = mu/2 (tr(Fe^T Fe) - 3) - mu ln Je + lambda/2 (ln Je)^2,     Je = det Fe

(`energy_density`), with the Lame constants of `E = plateau_stress / yield_strain` and the frozen Poisson ratio 0.3 (`POISSON_RATIO`): `mu = E / (2 (1 + nu))`, `lambda = E nu / ((1 + nu)(1 - 2 nu))` (`Law::new`). Its gradient is `P = dW/dFe = mu Fe + (lambda ln Je - mu) Fe^-T` (`first_piola`, `piola_from`; `Fe^-T` is the cofactor matrix over `Je`). The nodal forces are `f_k = -V P g_k` for k = 1, 2, 3 and `f_0 = -(f_1 + f_2 + f_3)`; with `g_k` the rows of `b` this is `P Fp^-T` carried back to the rest shape. So the forces are the exact gradient of the energy `V W(Fe)` (`SolverM5::elastic_energy`), their sum is zero, and their torque is zero because `Fe P^T = mu Fe Fe^T + (lambda ln Je - mu) I` is symmetric. `W` grows without bound as `Je -> 0` and the compressive stress with it (`compressive_force_grows_without_bound_as_je_goes_to_zero`; Saint Venant-Kirchhoff's falls to zero there, which is why it is not used, `docs/M5-SOLVER.md` section 4).

**Plastic flow, compression only** (R4b, `flow_from`, `return_map`, `apply_flow`). Every substep, for the trial `Fe`: `C = Fe^T Fe` is solved by `JACOBI_SWEEPS` = 6 cyclic Jacobi sweeps (`sym_eigen`; a fixed count, a rotation skipped only when its off-diagonal is 0 or below 1e-3 epsilon of its diagonals); `s_1` is the square root of the smallest eigenvalue and `n` its eigenvector. The principal Kirchhoff stress of that direction is `tau_1 = mu (s_1^2 - 1) + lambda ln Je`; it increases with `s_1`, so the smallest stretch carries the most compressive stress. If `tau_1 + sigma_y(e_c) >= 0`, or `e_c` has reached `e_max`, nothing flows; tension never flows (a state with every `s_i >= 1` has `tau_1 >= 0`). Otherwise `s_1 < 1` and the stretch moves back toward 1 by `e^d`, `d >= 0`:

    Fe' = Fe (I + (e^d - 1) n n^T)        Fp' = (I + (e^-d - 1) n n^T) Fp        e_c' = e_c + d

so `F = Fe' Fp'` is unchanged, `Je' = Je e^d` and `det Fp' = det Fp e^-d` (never increasing). `d` is the root of `g(d) = mu (s_1^2 e^(2d) - 1) + lambda (ln Je + d) + sigma_y(e_c + d)`, which is strictly increasing (`g' = 2 mu s_1^2 e^(2d) + lambda + sigma_y' > 0`) with `g(0) < 0`. The bracket is `[0, d_hi]`, `d_hi = min(-ln s_1, e_max - e_c)`; if `g(d_hi) < 0` the answer is `d_hi` (lock-up, or a flow that would take `s_1` past 1) and the rest of the stress stays elastic; otherwise `BISECTION_STEPS` = 60 bisection steps, and the answer is the upper end of the last bracket, so the stress is never more compressive than the yield stress. Along the path `dW/dd = tau_1 < 0`, so every update lowers the elastic energy (dissipation >= 0, `return_mapping_signs_bracket_and_dissipation`). One direction flows per substep: if, after the flow, another principal stress is still past the yield stress (a corner of the yield surface), it flows in a later substep.

**Hardening and lock-up** (R4c). `sigma_y(e_c) = plateau_stress / (1 - e_c / eps_D)` (`Law::sigma_y`) for `0 <= e_c <= e_max = eps_D - sqrt(eps_D yield_strain)`; at `e_max` its slope is exactly `E`, so the hardening modulus never exceeds the elastic one; beyond `e_max` the tet is elastic and the energy barrier resists. `eps_D = min(0.7, 1 - 1.4 phi_zone)`, `phi_zone = zone_mass / (zone_volume * 7850)` (`lockup_strain`, `LOCKUP_CAP`, `LOCKUP_SLOPE`, `STEEL_DENSITY`, the `frozen: lockup` line read by `m5_constants_are_the_frozen_lines`), with `zone_mass` the masses of the nodes of at least one tet (each node once) and `zone_volume` the sum of the tets' rest volumes, one zone for the whole body. A body whose `eps_D` is not above a solid's `yield_strain` (it could never crush) is refused. Confined uniaxial compression of one tet gives the plateau within 1%, follows `sigma_y(e_c)`, unloads elastically and locks up at `e_max` (`confined_compression_plateau_unload_and_lockup`, which reads `e_c` from the forces, not from the solver).

**Damping** (Q3, R4d). Each tet has a viscous stress `sigma_v = eta D`, `D = sym(Fdot F^-1)` on the total `F` (so a rigid rotation has none), with `eta = 2 zeta rho c L`: the critical damping of a bar of the tet's smallest altitude `L = 3 V / (largest face area)`, the dilatational wave speed `c = sqrt((lambda + 2 mu) / rho)` and the zone's mean density `rho = zone_mass / zone_volume`. Its first Piola stress is `J sigma_v F^-T = sigma_v cof(F)`, and its forces act through the rest gradients (the rows of `Dm^-1`); `F P_v^T = J sigma_v` is symmetric, so they too have no net force or torque. `zeta = 0` skips it.

**One update, shared nodes** (R1, section 5 of the note). `SolverM5::step`: gravity, then the beam forces with exactly the arithmetic and order of `SymplecticEuler`, then the tet forces in tet index order, into one per-node working array, then `vel += force * h / m`, `pos += vel * h`, written only after the checks of "Fail closed". A node of a tet and a beam has one position, one velocity and one mass (the file's); the tets add no mass. `step_with_forces` is the trait's operator splitting (velocities first get `extra * h / m`, then the step runs on them), done inside the solver with the same arithmetic so that its values are checked too. No threads, no hash maps, fixed orders: the same input gives the same bits, and a body with no tets gives exactly the default solver's bits (`m5_same_input_same_bytes`, which compares the CSV and the final positions and velocities bit for bit on `scenarios/cube.bsn`).

**Fail closed** (R4e). One substep (`SolverM5::step_inner`, behind both `step` and `step_with_forces`) computes everything into working arrays and writes the state only when all of them are finite. No non-finite value is ever written into the working force array: each contribution (a node's gravity, a beam's force, a tet's four nodal forces, damping included) is computed in a local value and checked, together with the sum it would make in the array, before it is written; a `debug_assert!` checks the whole array after each source (tests build in debug, `an_overflowing_beam_never_reaches_the_force_array`), and the array check after each source stays as a backstop. The state never receives one either. In order:

- every node's position and velocity: the first that is not finite records `node <id> is not finite at substep <n>` (the file's node id) before any force is summed (`a_nan_node_shared_by_a_beam_and_a_tet_stops_the_step_before_any_force`);
- the extra forces of `step_with_forces`, then the velocities they kick;
- each force contribution and its sum, source by source, gravity, the beams, the tets (damping included): the first that is not finite records `force on node <id> is not finite at substep <n> (from <source>)` and is not written, the source being `gravity`, `beams`, `tets` or `extra forces` (`the_force_check_catches_every_source_before_anything_is_applied`, and `a_beam_force_that_overflows_during_the_run_is_stopped_before_it_is_applied` for a body that passes construction and overflows in its first substep);
- the new velocities and positions: `update of node <id> is not finite at substep <n>`;
- every tet's `Je` on the new positions, with the plastic state about to be kept, by the same rule as below: `tet <index> inverted or collapsed at the updated positions of substep <n>: Je = <value>`. An apex that crosses its opposite face within one substep is caught here, before `Sim::check` could see an inverted tet (`a_tet_inverted_within_one_substep_is_caught_on_the_updated_positions`).

Every error path records its error through one function (`SolverM5::fail`), and one gate right before anything is applied (`SolverM5::failed`, in `step_inner` and in `tet_forces`) returns if an error is recorded, in this substep or an earlier one. So on any failure, the Je rule and the R5b stiffness verdict included, nothing of the substep is applied: no node moves and no tet's plastic state changes (the tets' new `b`, `Fp` and `e_c` are kept aside and taken only behind the gate), and no later substep applies anything either (`every_error_path_of_a_substep_applies_nothing` checks each path for bit-identical state and unchanged plastic state). `<n>` counts the calls of `step`, from 1. Inside the tet loop, a tet whose trial `Je` is not finite or is at or below `J_MIN_FAIL` = 0.05 adds no force, and the solver records `tet <index> inverted or collapsed at substep <n>: Je = <value>`, which stops the substep at the gate; a NaN node gives `Je = NaN` where this rule is reached first (`SolverM5::tet_forces`, which has no node check in front of it; in `step` the node check stops it before). `tet_forces` also checks its forces (`from tets`) before it keeps the plastic update. A step with a different node count or a different `h` than the solver was built for records an error and moves nothing. Nothing is clamped and nothing subdivides; the run ends at the next `Sim::check`, after the output step.

**Stability** (R5). For a tangent `A = d2W/dFe2` and `H = Fe^-1 dFe`, `dFe : A : dFe = mu |dFe|^2 + lambda (tr H)^2 + (mu - lambda ln Je) tr(H^2)`, and `(tr H)^2 <= 3 |H|^2`, `|tr(H^2)| <= |H|^2`, `|H| <= |dFe| / s_min(Fe)`, so

    |dFe : A : dFe| <= A_bound |dFe|^2,     A_bound = mu + (3 lambda + |mu - lambda ln Je|) / s_min(Fe)^2

(`bound_from`, `tangent_bound`). `s_min^2` is taken from below, as a Gershgorin bound on the Jacobi-rotated `C` minus 1e-13 of its trace for the rounding of the rotations (`Eigen::lower`), so the bound holds whatever the sweeps left. At rest it is `2 mu + 3 lambda`, the exact largest eigenvalue (`tangent_bound_covers_the_finite_difference_tangent` checks it against the eigenvalues of a finite-difference tangent of the test file's own stress, for 200 random `F` with stretches 0.3 to 3, shear and rotation). For one tet, `u^T K u = V dFe : A : dFe` with `dFe = sum_a u_a (x) g_a`, and `|dFe| <= sum_a |g_a| |u_a|` gives `u^T K u <= sum_a V A_bound |g_a| S |u_a|^2` with `S = sum_b |g_b|`: node a gets `V A_bound |g_a| S`. The damping does the same with `V eta J |F^-1|^2 |g_a| S` and the rest gradients, using `J |F^-1|^2 <= Je e^(e_c) / s_min(Fe)^2` (each flow factor of `Fp^-1` has norm `e^d`). The beams add `2 k` and `2 c` at each end, as `solver::body_step_error` (axial stiffness only). With `w2` the largest node sum of stiffnesses and `g` of dampings, both over the lightest node mass, the step must meet `h^2 w2 + 2 h g < 4`:

- (a) at construction, `from_body` uses the tet tangent at rest times `STIFFNESS_HEADROOM` = 2 and refuses `h^2 w2 + 2 h g >= 4` with `physics step h = dt/substeps = ... s reaches the m5 stability limit ... s (h^2*w2 + 2*h*g < 4 with the Gershgorin bounds w2 = ..., g = ..., the tet tangent at rest times 2); raise --substeps or lower --dt` (`omega_max`, `step_limit`);
- (b) every substep, from each tet's current `Fe`, `Je`, `e_c` and `b`, it records `stiffness bound exceeded at substep <n>: h omega = <value>`, with `value = sqrt(h^2 w2 + 2 h g)` (with no damping this is `h omega`; it reaches 2 exactly when the condition fails). The bound is elastic, so it is above the softer elastic-plastic tangent. As a tet crushes, the rows of `b` grow by up to `e^(e_c)`, so its share grows by up to `e^(2 e_c)` (3.2 times at `e_c` = 0.58): a thinner element has a higher frequency.

**Mesh rule** (R12, `cut_cell`). A hexahedral cell with corner k at bit 0 = +x, bit 1 = +y, bit 2 = +z is cut into 6 tets along its main diagonal, corner 0 to corner 7, the same way in every cell (corner 0, one axis step, two axis steps, corner 7, for the axis orders xyz, xzy, yxz, yzx, zxy, zyx, with the two middle nodes swapped for the odd orders so every volume is positive). The mesh is conforming across faces (`cut_cell_is_six_positive_tets_and_conforming`). The solver takes at most `M5_MAX_TETS` = 300 tets (`frozen: m5_max_tets`; a 5 x 5 x 2 zone is exactly that) and refuses more before it builds anything per tet. Which cells form the zone of a validation car (the cells its `crush` beams span) is n20-11c's (R14).

**Refusals** (R1, R10). `from_body` refuses, by name and in this order: more than 300 tets (a 1e6-tet body included); a step that is not a positive finite number; a node whose position is not finite (a NaN coordinate), or has a coordinate beyond `MAX_COORD_M` = 1e6 m in size (1 000 km: a car or a whole track lies within a few km, and it keeps every length, squared length and force product of a built body far below f64 overflow; a beam between coordinates of 1e200 m is refused here), or whose mass is not a positive finite number; a node id given twice; a start velocity faster than `MAX_SPEED_MPS` = 1e4 m/s (about 640 times the 56 km/h of the crash tests, above orbital speed) or not a number; a beam on a missing node or naming a missing material; a solid given twice or outside the format's ranges; a tet naming a missing solid or node or listing a node twice; a tet whose size is not finite, whose rest volume is below `TET_MIN_VOLUME` (zero volume, collinear or coplanar nodes), whose shape is below `TET_MIN_SHAPE`, whose volume is negative, or on the same four nodes as an earlier tet (the format's own rules, checked again for a body built in code); a node shared by tets of two solids; a zone that could never crush; and the step of (a). It also refuses, instead of replacing by 0 or dropping, every number of the bound that is not finite: a solid whose `E = plateau_stress / yield_strain` is not finite (plateau 1e308 Pa at yield 0.02), a beam whose material (in a body built in code) has a stiffness that is not a positive finite number or a damping that is not a non-negative finite number, a tet whose damping coefficient `eta` is not finite, a node whose stiffness or damping sum is not finite (each node's sum is checked, since a NaN would vanish in the maximum), and a `w2` or `g` that is not finite (`construction_refuses_every_non_finite_bound_input`). The other numbers built at construction cannot overflow inside these bounds: the rest gradients (rest volume at least 1e-9 m^3 and coordinates at most 1e6 m keep them below about 1e21), the lock-up strain (a zone too dense to crush is refused first) and with it the zone density, which then stays below 7850 / 1.4 kg/m^3. The refusals before this one are tested by `hostile_and_bad_bodies_are_refused_by_name`.

**What the tests measured** (debug build, `broadsword test`). The coupling test of `docs/M5-SOLVER.md` section 5 (`m5_coupling_creates_no_energy`: 48 tets and 2 x 2 x 2 cells of beams sharing 9 nodes, 20 000 substeps at a tenth of the limit, q = 0.1) peaks at E / E0 = 1.0031 against its bound of 1.2344, and conserves both momenta to 1e-9 of their scale. Its two mutations from section 5 both fail the test: the flipped tet force on the shared nodes on the momentum check at substep 1, and on the energy bound alone at substep 37; the shared-node mass counted twice only on the momentum check (with the momentum checks off, the energy stays inside the bound: a wrong mass changes which energy is conserved, not whether one is). **Cost** (measured once for task n20-11 core with a timing program kept outside the repo: `--release`, one thread, `SolverM5::step` alone on a 300-tet zone of 0.2 m cells with 1 kg nodes and no beams, the frozen grid's stiffest and softest corners, elastic and crushing, median of 5; the machine was at 93% CPU from other jobs, so these may read high): about 0.27 to 0.34 microseconds per tet per substep, so about 4.0 to 5.1 rtf for the tets alone at h = 20 microseconds; the projection with the 23 microseconds of beams, wheels and contact of the note's section 3 is 5.2 to 6.2. The real figure is R15's, on the validation car (n20-11c).

**Not covered here.** `Sim::total_energy` and the settle report count nodes and beams only, not the tets' elastic energy (`SolverM5::elastic_energy` gives it); a settle or energy check of an m5 run would need it (n20-11c).

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

`Config::validate()` refuses to run when `dt / substeps` is larger than `max_stable_step()`, and tells you to raise `--substeps` or lower `--dt`. It also rejects non-finite values, `dt <= 0`, `duration < 0`, `substeps < 1`, `drop_height < 0`, and gravity that is negative, NaN or infinite. Gravity must be finite and `>= 0`, enforced by `self.gravity.is_finite() && self.gravity >= 0.0` (`src/lib.rs` line 191): 0 is allowed for the headless pulse runs (the m4 frozen run flags start with `--gravity 0`). The default 1 ms substep is about a quarter of the limit.

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

- the solver reports an error of its own (`Solver::check`, checked first; for the m5 solver an inverted or collapsed tet or an exceeded stiffness bound, "Solids" above; the default solver never reports one);
- any node position or velocity component is NaN or infinite;
- the max strain (over unbroken beams) is NaN, or exceeds `MAX_STRAIN_LIMIT` = 0.5 (50%). The strain calculation propagates NaN, so a blown-up state can never read as 0.

Contact time is tracked separately, in the substep loop: `Sim::first_contact_s` is the time of the first substep at which any node is at or behind any collider plane, ground or wall (0 if the body starts there). `--summary` prints it, so it does not depend on the output sampling, which can miss a touchdown that begins and ends between two samples.

`run_checked` then stops and returns the good samples so far plus an error reading `unstable at t = ... s: ...`. A config that `validate()` accepts can still abort here, for example `--gravity 5000` crushes the cube on impact (`tests/risky.rs`). A config refused by `validate()` returns no samples and the validation error. `run` is the same but panics on any error.

## Inter-car chassis contact (n20-19a kernel)

`car_contact::{detect_contacts, apply_contacts}` takes cars in caller-defined fixed
order, with chassis node positions **before and after one physics substep**,
post-substep velocities, node masses, beams and a parallel broken-beam mask.
Indices in those slices are the car/node/beam identities; broken beams are not
compacted. Save positions before stepping, step each car using the existing
single-car integrator, then call contact after **every physics substep**, not
only at output frames. This module is not a race loop or a wheel collider.

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
end, the kernel requires a separating-plane certificate. Using either snapshot's
closest-point normal, **both endpoints in both snapshots** must project strictly
farther than skin from the source. Convex interpolation then certifies that
every point of the linearly moving segment stays outside that plane throughout
the substep. The implemented certificate accounts for the computed normal's
length and adds a conservative `64*EPSILON` roundoff margin scaled by the larger
position norm or skin. Identical source/endpoint snapshots need no sweep check.
If neither normal certifies separation, `UnsupportedSweep` refuses
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
iterated simultaneous-contact solve. Positions are **never** projected to
remove penetration: spring lengths, rest lengths and elastic energy are
unchanged by contact. Wheel state is unchanged during the pass; its subsequent
carrier/suspension interaction remains the existing single-car implementation.

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

Response runs on private trial velocities. Only after finite output, energy and
next-step speed checks does it copy velocities back; **any error leaves all
incoming caller state untouched**, even after a trial impulse. A refusal names
the car and node/beam/contact key (settings errors have no car), with a named
failure such as `MotionBound`, `PenetrationBound` or `EnergyIncrease`.
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

`car_contact_never_creates_energy` measures `Sim::total_energy` immediately
before/after each contact pass, including chassis kinetic, live beam elastic,
wheel translation/spin and suspension/tyre energy. Only
`1e-10 * max(1 J, |E_before|)` roundoff is allowed. The kernel itself enforces
that same tolerance on chassis kinetic energy before copying back; all other
energy terms are unchanged during contact. In the nonnegative-energy test
setup this is at least as strict as the full-state tolerance.
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
4. For two or more cars, calls `apply_contacts` once over ordered car views,
   using those snapshots as `previous_pos`. Counts are summed across substeps.
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
