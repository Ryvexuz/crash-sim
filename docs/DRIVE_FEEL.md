# Why the car drives "like ice" (task t-20261120-220016, measure only)

Lazlo, 2026-11-20 21:53Z, after a lap in the viewer: "done was like driving on ice, didn't have a
reverse". This file measures why. No physics, test or scenario file was changed; the only code
is `examples/drive_feel.rs`, which replays an input file through the public API and prints a
time series. Every number below is copied from the runs listed in "Commands", except where a
line names another source.

## Commands

Run from the Broadsword root, in the task worktree (so the input path has one more `..` than it
would from `projects/crash-sim`):

```
build\bin\broadsword.exe run crash-sim --task t-20261120-220016 -- cargo run --offline --release --example drive_feel -- ../../../scratch/s6-3-7.input
build\bin\broadsword.exe run crash-sim --task t-20261120-220016 -- cargo run --offline --release --example drive_feel -- scenarios/pad_lap.input
```

Both end `run: ok`. The outputs were saved as `scratch/t-20261120-220016-lazlo.txt` (476 rows)
and `scratch/t-20261120-220016-padlap.txt` (600 rows, the control: the scripted lap of check
3.6). The body is `scenarios/pad_and_wall.bsn` (the default of the example, the same body as the
viewer's replay in `scratch/s6-3-8.txt`), at the crate's default dt 0.01 s and 10 substeps.

What the example prints (column definitions are in the header comment of
`examples/drive_feel.rs`): every 0.1 s the controls, speed, yaw rate, body sideslip beta, the
0.1 s mean longitudinal and lateral acceleration in g, a wall flag, and per wheel the contact,
slip angle `a`, slip ratio `k` and the share of the friction circle the patch force uses (`ux`
along, `uy` across, `u` = |(ux, uy)|; `u` = 1.00 is sliding). The summary uses only rows before
the first wall contact. Not measurable without instrumentation (listed in the output): the tyre
force actually applied (its damping term is private) and the drive and brake torque reaching each
wheel after the brake clamp.

Cross-check against the earlier replay: this run's `wall_first_contact_s=43.358` matches
`wall_first_contact_s=43.3580` in `scratch/s6-3-8.txt`.

## The car, from the `# car` line (both runs print the same)

```
# car mass_kg=1200.0 reference_load_n=2943.0 driven_wheels=2 full_throttle_drive_force_per_driven_wheel_n=3149.6 mu_x_reference_load_n=2589.8 drive_over_grip=1.216 max_steer_deg=30.0 lat_saturation_alpha_deg=5.03 long_saturation_kappa=0.0587
```

- Full throttle pushes each rear wheel with 3149.6 N, while the tyre can carry at most 2589.8 N
  at its static load: **drive_over_grip = 1.216**. The drive torque is constant at every speed
  (`Car::drive_torque` in `src/vehicle.rs`, no engine curve or gears), so this holds from
  standstill to top speed.
- The linear patch force reaches the friction limit at a slip angle of 5.03 deg and a slip ratio
  of 0.0587 (at the static load). Full lock is 30.0 deg road-wheel angle.

## The inputs (from the input file, not a sim run)

`awk` over `scratch/s6-3-7.input` (2383 lines): every line is 0.02 s after the previous one
(`line_dt_min=0.02 line_dt_max=0.02`), and the steer changes by 0.06 in 1006 lines, 0.04 in 32,
0.02 in 4 and 0 in 1340 (`max_abs_steer_step_per_line=0.06`). That is the viewer's
`STEER_RATE = 3.0` full-lock units per second (`projects/render/examples/drive.rs` line 80), so
lock to lock takes about 0.67 s. Throttle is 0 in 1757 lines and 1 in 626; brake is 0 in 2264
and 1 in 119. The viewer sends throttle as 0 or 1 only (`drive.rs` line 176).

## Key windows and numbers

### Control lap (pad_lap.input): no spin

```
spin_moments=0
peak_abs_beta_deg=6.922 at t=3.00 thr=0.25 ...
throttle_part: rows=71 mean_throttle=0.387 driven_both_saturated_rows=0 mean_driven_kappa=0.027 mean_driven_abs_ux=0.42 mean_driven_abs_uy=0.05 mean_driven_abs_alpha_deg=0.3 mean_steered_abs_alpha_deg=0.6 mean_abs_beta_deg=2.9 mean_abs_ay_g=0.052
throttle_off: rows=134 ... mean_abs_beta_deg=5.6 mean_abs_ay_g=0.180
peak_abs_ay_g=0.250 at t=18.60
```

The scripted lap never uses more than throttle 0.5 and never saturates the rear tyres.

### Lazlo's lap (s6-3-7.input): 433 rows before the wall, 3 spin moments

A spin moment is a row where |beta| first passes 30 deg after being under 15 deg.

```
spin_moments=3
throttle_full: rows=99 mean_throttle=1.000 driven_both_saturated_rows=90 mean_driven_kappa=2.692 mean_driven_abs_ux=0.95 mean_driven_abs_uy=0.20 mean_driven_abs_alpha_deg=18.7 mean_steered_abs_alpha_deg=12.1 mean_abs_beta_deg=19.4 mean_abs_ay_g=0.319
throttle_off: rows=256 mean_throttle=0.000 driven_both_saturated_rows=23 mean_driven_kappa=0.029 mean_driven_abs_ux=0.04 mean_driven_abs_uy=0.35 mean_driven_abs_alpha_deg=5.9 mean_steered_abs_alpha_deg=6.7 mean_abs_beta_deg=11.9 mean_abs_ay_g=0.296
full_lock_at_or_above_5mps: rows=54 mean_steered_abs_alpha_deg=18.7 mean_steered_util=0.88 mean_abs_ay_g=0.472
coasting_straight_above_2mps: rows=27 mean_ax_g=0.0094
peak_abs_ay_g=0.859 at t=8.40 thr=0.00 brk=0.00 steer=0.66 speed_mps=8.33
```

All three spin moments happen at throttle 1.00, each with both rear tyres on their friction
circle (u = 1.00) and the rear slip angles larger than the front ones:

| Spin | t (s) | speed (m/s) | yaw (deg/s) | beta (deg) | rear k (rl, rr) | rear a (rl, rr, deg) | front a (fl, fr, deg) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | 10.40 | 9.21 | 91.5 | -31.4 | 0.887, 5.310 | 37.0, 46.0 | 16.9, 22.6 |
| 2 | 40.30 | 12.27 | 54.8 | -30.4 | 0.828, 4.215 | 33.1, 36.4 | -5.3, -2.5 |
| 3 | 42.10 | 13.56 | -85.1 | 32.2 | 3.003, 0.784 | -41.5, -36.0 | -6.0, -1.6 |

Spin 1, row by row (columns t, throttle, steer, speed, yaw, beta, ay, then rl a, k, ux):

```
9.30  thr 0  steer 1.00  6.76 m/s  yaw 53.8  beta  6.8  ay 0.681   rl a 3.0  k -0.001 ux 0.01
9.40  thr 1  steer 1.00  6.76 m/s  yaw 56.0  beta  6.5  ay 0.642   rl a 4.0  k 0.240  ux 0.97
9.70  thr 1  steer 1.00  7.63 m/s  yaw 72.0  beta -3.1  ay 0.460   rl a 13.2 k 0.221  ux 0.83
10.00 thr 1  steer 0.88  8.49 m/s  yaw 76.4  beta -14.2 ay 0.475   rl a 22.3 k 0.396  ux 0.83
10.40 thr 1  steer 0.00  9.21 m/s  yaw 91.5  beta -31.4 ay 0.509   rl a 37.0 k 0.887  ux 0.87
10.60 thr 0  steer 0.24  9.20 m/s  yaw 101.1 beta -42.7 ay 0.548   rl a 46.5 k 0.164  ux 0.28
```

Until 9.30 s the car corners at full lock, throttle off, with the rear slip angle at 3.0 deg and
0.681 g. In the first 0.1 s of throttle the rear slip ratio jumps to 0.240 (four times the
saturation ratio 0.0587) and the rear tyre's longitudinal share to 0.97. From then on the rear
slip angle grows every row (4.0 to 46.5 deg in 1.2 s) and the yaw rate rises (56.0 to 101.1
deg/s) although the steering is wound off to 0 by 10.30 s: power oversteer.

## Causes, with evidence

1. **Step throttle spins the driven wheels, and a spinning rear tyre has almost no side grip
   left (main cause).** Full throttle asks for 1.216 times the rear tyre's grip at any speed.
   Measured: in 90 of the 99 full-throttle rows before the wall both rear tyres were sliding,
   with a mean slip ratio of 2.692 (the linear range ends at 0.0587), 0.95 of the friction
   circle spent along the wheel and only 0.20 left across it. The rear slip angle (18.7 deg mean)
   is then larger than the front (12.1 deg), so the car turns more than it is steered: mean
   |beta| 19.4 deg against 11.9 deg off throttle and 2.9 deg in the control lap's part-throttle
   rows (0 of 71 saturated at mean throttle 0.387). All 3 spin moments are at throttle 1.00 with
   rear slip ratios from 0.784 to 5.310. The viewer only sends throttle 0 or 1, so every press of
   W is a full-grip-exceeding step.
2. **Full lock at speed is far past the tyre's saturation angle (secondary: dead steering, then
   late countersteer).** In the 54 rows at full lock above 5 m/s the front slip angle averaged
   18.7 deg against the 5.03 deg at which the tyre saturates, with 0.88 of the front friction
   circle used and 0.472 g. This tyre model has no peak-then-fall (docs/PHYSICS.md line 100,
   "no peak-then-fall of the slip curve"), so the extra lock does not lose grip, but it adds none:
   past about 5 deg of front slip the lock only scrubs. With the steer ramped at 3.0 per second,
   countersteer from full lock to centre takes about 1 / 3.0 = 0.33 s, during which (spin 1,
   9.70 to 10.00 s) the rear slip angle grows by about 3 deg per row. Expected effect of the steering on the spin itself:
   not separated from cause 1 by these runs.
3. **No rolling resistance or drag: lifting off does not slow the car (contributing).** Control
   lap, 22.50 to 23.30 s, throttle 0, brake 0, steer 0: speed 13.85 m/s in every row, and ax
   0.000 g in every row from 22.60 s (0.012 at 22.50 s, the first row after the throttle went
   to 0). Lazlo's lap: 27 straight coasting rows above 2 m/s average ax = +0.0094 g, so no slowing
   either. docs/PHYSICS.md line 100 lists "no rolling resistance" among what the model ignores.
   Letting go of W gives no engine braking or drag to help the car settle, which reads as ice.
4. **Steady-state lateral grip is not the problem.** Before the wall the highest 0.1 s mean
   lateral acceleration was 0.859 g (t = 8.40 s, throttle 0, steer 0.66, 8.33 m/s), against the
   skidpad's 0.842 g steady limit (docs/PHYSICS.md line 118; not re-measured here). Off throttle
   the car corners at its grip limit; under full throttle its mean |ay| drops to 0.319 g.
5. **Not established by these runs:** the low-speed tyre model and yaw damping as separate
   causes. The one large sideslip at low speed (peak |beta| 115.028 deg at t = 11.80 s, 1.40 m/s)
   is at throttle 1.00 with rear slip ratios 3.202 and 2.756, so it is cause 1 again; the slip
   ratio there is divided by the 1 m/s floor of the example, so its size is not exact. Nothing in
   these runs isolates yaw damping.

Also from the report: "didn't have a reverse". There is none: the input rules allow throttle 0
to 1 only (`src/input.rs`, module comment), and the drive torque never pushes backward. That is
a missing feature, not part of the ice feel; it goes to the orchestrator for `IDEAS.md`.

## Proposed fixes (every expected effect is an estimate)

1. **Driven-wheel saturation.** Two options, cheapest first:
   - a. Viewer only, no physics change: `projects/render/examples/drive.rs`, ramp throttle the
     way steer is ramped (a new `THROTTLE_RATE`) and cap the keyboard's full throttle at a
     fraction that keeps drive_over_grip below 1 (about 0.8 by the `# car` line's 1.216).
     Estimate: on a lap like Lazlo's, rear saturation under throttle falls from 90 of 99 rows to
     a small share, and power oversteer still happens in a corner (a tyre with 0.8 of its circle
     along the wheel has only part of its side grip), but builds slowly enough to catch.
   - b. Car or model: lower `max_drive_torque` in `scenarios/car_basic.bsn` (the controls line;
     `pad_and_wall.bsn` and `skidpad.bsn` must change with it, their tests require them to start
     with car_basic byte for byte), or add a simple traction limit to `Car::drive_torque` in
     `src/vehicle.rs` that caps the drive force per wheel at its `mu x load`. Estimate: rear slip
     ratio under full throttle near the 0.0587 saturation value instead of 2.692. Check 3.5 (its
     launch and full-lock tests in `tests/drive.rs`), check 3.6 (the lap) and the skidpad must be
     re-run, because they use this car.
   - Before/after test for 1a (a viewer change): a raw input file replayed through `drive_feel`
     never passes through the viewer, so it cannot show a viewer fix. Record the inputs through
     the viewer instead: run the old viewer and the new viewer with the same key script,
     `drive --sim-exe <crash-sim.exe> --body scenarios/pad_and_wall.bsn --script <seconds>
     --no-window --save-input <file>` (the keys come from the viewer's built-in `scripted_keys`,
     the same keys per tick in both runs), then replay each saved file through `drive_feel` and
     compare the full-throttle bin, `spin_moments` and the full-lock line. Because a throttle cap
     changes the value sent, the new recording's throttle-on rows land in `throttle_part`, not
     `throttle_full`: compare the old `throttle_full` line with the new `throttle_part` line (all
     rows with the throttle key held), and the rows at the same times (same tick, same keys). If
     the built-in key script holds throttle in a corner too briefly to spin the old car (check
     that in the old recording first), the fix task adds a key script that does, to `drive.rs`.
   - Before/after test for 1b (a car or model change): here the inputs do not change, so a raw
     scripted input replayed through `drive_feel` is enough: a new open-loop file that holds a
     steady circle at throttle 0 and steer 0.47 as pad_lap does, then steps to throttle 1. Before
     the fix, measure that |beta| passes 30 deg and both rear tyres reach u = 1.00; after it,
     assert |beta| stays under a stated bound (for example 15 deg, an estimate) and the mean
     driven slip ratio stays under a stated bound. Replaying Lazlo's own file after a fix is a
     weaker check, because his inputs were a reaction to the old car.
2. **Full lock at speed.** `projects/render/examples/drive.rs`: scale the steer the viewer sends
   down with speed (speed-sensitive steering), so that the front slip angle at full key stays a
   few degrees above the 5.03 deg saturation angle, and return to centre faster than
   `STEER_RATE`. Estimate: `full_lock_at_or_above_5mps mean_steered_abs_alpha_deg` from 18.7 to
   under 10. Before/after test: as for 1a, record through the viewer, not a raw file: old and new
   viewer, the same key script, `--script <seconds> --no-window --save-input <file>`, then
   `drive_feel` on each saved file, comparing `full_lock_at_or_above_5mps`, the full-throttle bin
   and `spin_moments`. Speed-sensitive steering sends less than 1 at speed, so the new
   recording's full-lock line may have no rows; compare instead the rows at the same times as the
   old recording's full-lock rows (same tick, same keys held), where the front slip angle should
   drop. The key script must include a full-key turn above 5 m/s; if the built-in
   `scripted_keys` has none in the old recording, the fix task adds one to `drive.rs`.
3. **No rolling resistance or drag.** `src/vehicle.rs` plus the `.bsn` format (`src/format.rs`
   and docs/FORMAT.md): a rolling resistance force per tyre (a coefficient times its load,
   against the rolling direction) and an aero drag on the body (a coefficient times speed
   squared). Estimate: a coasting deceleration of about 0.01 to 0.02 g instead of the measured
   0.000 (control) and +0.0094 (Lazlo). Before/after test: a coast-down from 13.85 m/s with
   throttle 0, brake 0 and steer 0 (the control lap's 22.50 to 23.30 s window), asserting a
   negative `coasting_straight_above_2mps mean_ax_g` within the stated band; the energy rule of
   the settle report must then count the work these forces take out, and the skidpad's 0.842 g
   must be re-measured.

Order suggested: 1a first (no physics, quick to feel in the viewer), then 1b or 3 as a physics
task with its own brief and tests.
