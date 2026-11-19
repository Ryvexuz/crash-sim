# Stage 6 done checks: milestones 4 and 5

Written before any m4 code, from crash-sim `26d0bea`. DEVELOPMENT_PLAN Stage 6:

4. Validation against real data: compare against published IIHS or NHTSA crash tests. *Done:* one test matches within a stated margin.
5. Higher-fidelity solver: soft bodies or FEM in the crash zone. *Done:* it measurably beats the node-beam solver in the validation test.

This file turns both lines into commands with numbers, in the conventions of `docs/STAGE6.md` ("How to run and read a check" there applies here unchanged: commands from the Broadsword root, `broadsword` = `build/bin/broadsword.exe`, `--task <id>` after the project name inside a task, output redirected to `scratch/`, never piped; "the output" means the program's lines between broadsword's job lines and `--- summary ---`; anything that is not PASS is FAIL, including a check that cannot run because a flag, file or test is missing). The briefs are in `state/next-batch-2026-11-20/` (n20-02 to n20-11); the table at the end maps every check to its brief.

The lesson of these two milestones is **tuning toward a real measurement without fitting the measurement**. Every rule below that looks like bookkeeping (frozen numbers, one run on test B, Git order checks) exists for that.

## Figures from memory

This file was written without network access. Every published figure it relies on is marked *(V<n>, from memory, to verify)* and listed here. n20-02 (which uses the network) checks each one and records the result in `validation/README.md` (check 4.1). A figure that turns out wrong is corrected **in this file first**, by the orchestrator, before any calibration run (check 4.6 enforces the order); a margin may shrink to a verified figure or grow to one, never move for any other reason.

| Id | Figure | Used for |
|---|---|---|
| V1 | NHTSA NCAP frontal test: full-width rigid barrier at 35 mph (56 km/h), with vehicle accelerometer time histories in the NHTSA Vehicle Crash Test Database, free to download without an account, US government work (public domain) | the protocol |
| V2 | Euro NCAP does not publish raw accelerometer time histories; IIHS frontal tests are offset (40% moderate, 25% small overlap), not full width, and IIHS data is not public domain | why not those |
| V3 | SAE J211-1 channel classes: vehicle structural acceleration for total vehicle comparison CFC 60; for integration to velocity or displacement CFC 180 | `--pulse-cfc-shape 60`, `--pulse-cfc-int 180` |
| V4 | OLC (Occupant Load Criterion, Kübler et al., Airbag 2008 conference; used in the Euro NCAP MPDB protocol): a virtual occupant moves freely 65 mm relative to the vehicle, then decelerates uniformly so that it reaches the vehicle's velocity after a further 235 mm; OLC is that uniform deceleration; computed from the CFC 180 velocity | the OLC metric |
| V5 | NHTSA vehicle accelerometers are sampled at about 10 kHz (recorded per channel in the data) | h = 0.1 ms |
| V6 | Repeat full-frontal rigid-barrier tests of one model: delta-V scatter about 2 to 3%; peak compartment deceleration scatter about 10%; time of peak a few ms, up to about 10 ms when the pulse has two similar peaks; OLC about 10% | the margins |
| V7 | Published FE vehicle models (for example the NCAC / CCSA models made for NHTSA) match NCAP compartment pulses to about 10 to 20% in peak | the margins (sanity: a node-beam model should not be held tighter than FE) |
| V8 | Static crush in NHTSA reports: a profile of 6 points (C1 to C6) across the front, measured to about 1 cm | the static-crush metric |
| V9 | Automotive body sheet steel: thickness 0.7 to 2.5 mm; yield strength about 210 MPa (mild) to 1200 MPa (press-hardened); E = 210 GPa | the calibration ranges |
| V10 | Welded steel structures: measured damping ratio 0.5 to 5% of critical | the calibration ranges |
| V11 | Rigid-barrier pulses of compact cars at 56 km/h end within about 0.10 to 0.12 s of contact; rebound speed is about 5 to 15% of impact speed | the window |

## The protocol, and why

**NHTSA NCAP full frontal rigid barrier, 56 km/h** (V1). Reasons: it is a full-width rigid wall, which crash-sim already has (a `plane`, nb-36), so no barrier model is needed; its accelerometer time histories are public and public domain (V1), so the raw data can live in the repo and anyone can re-download it; and NHTSA has tested many compact cars of the mass of `car_basic.bsn`, so two comparable tests exist. Not Euro NCAP or IIHS (V2): no raw data, or an offset barrier crash-sim cannot model yet.

**The link to m3.** STAGE6 m3 aims the drive-into-wall run at 45 to 55 km/h because Euro NCAP full width and UN R137 test at 50 km/h. It is the same physics (a car into a rigid wall); 56 km/h carries (56/50)^2 = 1.25 times the energy per kg. m4 does not reuse m3's drive script: it starts the car at the test's own speed (below), so m3's open-loop aim does not limit m4's accuracy. m3's recorded `wall_impact_speed_kmh` and `crumple_delta_m` stay in its evidence; m4 does not gate on them.

**Test A and test B** (n20-02). Two NHTSA NCAP frontal tests of two different compact cars (not two tests of one car), each 1200 to 1400 kg test mass, the accelerometer at the same location in both (below). **A is used to calibrate; B is used once, with everything frozen, and m4 passes or fails on B.** A car that matches A after tuning proves nothing; a car tuned on A that matches B with no further change shows that the model, not the tuning, carries the result.

## The run

Both tests are crashed the same way:

- `scenarios/validation_wall.bsn` (test A) and `scenarios/validation_wall_b.bsn` (test B) (n20-08): the car matched to that test (mass, wheelbase, track, length, front overhang, from the test report or the maker's data, each cited in `validation/CAR.md`), with its front toward +x, a wall plane 0.05 m ahead of its front-most point with normal (-1, 0, 0), and every node and wheel starting at (v0, 0, 0) with v0 = that test's `speed_kmh` / 3.6. The car starts with its tyres 0.20 m above the ground and its suspension at rest length, as `car_basic.bsn` (STAGE6 m3).
- **No gravity** (`--gravity 0`). In a rigid-barrier test the car's weight (1 g) is small beside a 20 to 40 g pulse, and the tyres roll freely, so the ground carries almost no force along x. Leaving gravity on would put the tyres on the ground with no rolling spin (skidding at full grip for tens of ms) or need a long run-up to settle. With no gravity the car flies into the wall level; a ground contact inside the window is a FAIL (rule below), so the simplification is checked, not assumed. Today's `Config` refuses `gravity = 0` (`gravity > 0`, crash-sim `src/lib.rs`); n20-07 changes that rule to `gravity >= 0`.
- **Step:** `--dt 0.001 --substeps 10`, so h = 0.1 ms: 10 kHz, the sample rate of the test data (V5), 5.5 times J211's minimum of 10 x CFC for CFC 180 (V3). It must be below the step limit of the stiffest calibration point (n20-08 states the Gershgorin limit at the top of the stiffness range in `validation/CAR.md`; if 0.1 ms is not below it, n20-08 stops and this file changes h before any calibration).
- **Run length:** `--duration 0.2`: contact is at about 0.05 / 15.6 = 3 ms, the window (below) ends at 0.153 s, and the last 0.05 s gives the static crush.

### Groups the validation files must have

`all` (every node; carries the start velocity), `sensor` (the chassis nodes nearest the test's accelerometer, in the occupant compartment, outside the crush zone), `front` and `rear` (as in `car_basic.bsn`), and material `crush` for every beam of the crush zone (the only calibrated material).

## The pulse report (n20-09)

`crash-sim --pulse-report <ref>.pulse.csv` replaces the CSV with a comparison of the run against a reference pulse. `<ref>.meta` is read from the same path with `.pulse.csv` replaced by `.meta` (a path not ending in `.pulse.csv` is a usage error). Every other flag below is required, with no default in code; a missing one, an unreadable or refused reference (n20-04's refusals), a meta file missing a required key, a sample rate under 10 x CFC (n20-03's refusal), or a window longer than either pulse is a usage error: exit 2, empty stdout, the reason on stderr.

| Flag | Value used here | Meaning |
|---|---|---|
| `--pulse-sensor <group>` | `sensor` | the sim pulse is the x-acceleration of this group's mass-weighted mean x-velocity u(t): a_k = (u_k - u_(k-1)) / h, every substep |
| `--pulse-crush-groups <a>,<b>` | `rear,front` | L(t) = distance between the two groups' centroids (as the crumple report) |
| `--pulse-cfc-shape <cfc>` | `60` | filter for peak, time of peak, duration (V3) |
| `--pulse-cfc-int <cfc>` | `180` | filter for delta-V, OLC, dynamic crush (V3, V4) |
| `--pulse-window <s>` | `0.15` | metrics use t in [0, window] after contact (V11: covers the pulse and the start of rebound) |
| `--pulse-threshold <m/s^2>` | `20` | duration counts samples at or below -threshold (about 2 g: below it the pulse is rebound and ringing) |
| `--pulse-crush-tail <s>` | `0.05` | static crush uses the mean of L over the last tail seconds of the run |
| `--setup-mass-tol <frac>` | `0.01` | sim total mass vs meta `mass_kg` |
| `--setup-speed-tol <frac>` | `0.005` | sim impact speed vs meta `speed_kmh` |
| `--margin-dv <frac>` | `0.05` | relative |
| `--margin-peak <frac>` | `0.15` | relative |
| `--margin-peak-time <s>` | `0.010` | absolute |
| `--margin-duration <frac>` | `0.10` | relative |
| `--margin-olc <frac>` | `0.15` | relative |
| `--margin-static-crush <frac>` | `0.20` | relative; gated only when the meta has `static_crush_m` |

Optional: `--pulse-csv <path>` (columns `t_s,accel_mps2,accel_cfc60_mps2,accel_cfc180_mps2`, sim pulse from contact, for plotting); `--timing` (one line on **stderr**, `timing: sim_s=<4 decimals> wall_s=<4 decimals> rtf=<4 decimals>`, wall_s = wall-clock time of the solver loop only, not build or parsing, rtf = wall_s / sim_s; on stderr so stdout stays byte-identical between runs); `--solver <name>` (n20-11: `node-beam`, the default, same output as no flag; `m5`).

**How the two pulses are made comparable.** Contact (t = 0) is, for the sim, the first substep in which any node or tyre touches the wall plane (the definition of `wall_first_contact_s`, STAGE6 m3); for the reference, the time zero n20-02 set (first barrier contact, from the test data). Each pulse is filtered at its own sample rate over its whole record (the reference keeps its samples before zero, so the filter's start-up is outside the window), the sim pulse is shifted so contact is t = 0, and the reference is resampled to the sim's h on [0, window] (n20-04 `resample`, linear, no extrapolation). Sim and reference then go through the same metric functions (n20-03).

**Metrics** (all printed as positive magnitudes; a frontal pulse is negative along +x):

- `dv_mps`: delta-V = -integral of a (CFC 180) over [0, window], trapezoid (n20-03 `delta_v`); includes rebound.
- `peak_mps2`: magnitude of the most negative CFC 60 value in the window (n20-03 `peak`); `peak_time_s` its time.
- `duration_s`: CFC 60, time from the first to the last sample in the window at or below -threshold (n20-03 `duration`).
- `olc_mps2`: OLC (V4) of the CFC 180 pulse with v0 = that pulse's impact speed (sim: the sensor group's u at contact; reference: meta `speed_kmh` / 3.6) (n20-03 `olc`).
- `static_crush_m`: sim L(0) - mean of L over the last tail seconds; reference meta `static_crush_m` (the mean of the report's crush profile points, V8).
- `dyn_crush_m` (reported, not gated): the largest displacement toward the wall, x(t) = integral of (v0 + integral of a) over [0, t], CFC 180, max over the window; the same formula for both, so the method's bias cancels.

Errors: for a relative metric err = (sim - ref) / ref, for `peak_time_s` err = sim - ref in s; norm = |err| / margin; a metric is `ok` iff norm <= 1. `pulse_score` = the mean norm over the gated metrics (five, or six with static crush).

Rules, in this order; the first that fails is named in the FAIL line:

1. Setup: |sim mass - `mass_kg`| / `mass_kg` <= mass tol, with sim mass the total of nodes and wheels; |sim impact speed - `speed_kmh`| / `speed_kmh` <= speed tol, with sim impact speed the `sensor` group's u at contact, in km/h (the wrong car or the wrong file).
2. The car touches the wall (otherwise `pulse: FAIL no contact`).
3. No node or tyre touches the ground between contact and the end of the window (otherwise `pulse: FAIL ground contact at t=<4 decimals>`).
4. Each gated metric, in the printed order: norm <= 1.

An abort (STAGE6 "Abort") prints `pulse: FAIL unstable at t = ... s: <reason>`. Exit code 0 iff `pulse: ok`; 1 on any FAIL or abort; 2 on a usage error.

Output, in this order:

```
pulse_ref=<meta test>
pulse_channel=<meta channel>
setup mass_kg sim=<1 decimal> ref=<1 decimal> err=<signed 4 decimals> tol=<4 decimals> ok|FAIL
setup speed_kmh sim=<2 decimals> ref=<2 decimals> err=<signed 4 decimals> tol=<4 decimals> ok|FAIL
contact_s=<4 decimals, or none>
ground_contacts_in_window=<count>
metric dv_mps sim=<4 decimals> ref=<4 decimals> err=<signed 4 decimals> norm=<4 decimals> ok|FAIL
metric peak_mps2 sim=<1 decimal> ref=<1 decimal> err=<signed 4 decimals> norm=<4 decimals> ok|FAIL
metric peak_time_s sim=<4 decimals> ref=<4 decimals> err=<signed 4 decimals> norm=<4 decimals> ok|FAIL
metric duration_s sim=<4 decimals> ref=<4 decimals> err=<signed 4 decimals> norm=<4 decimals> ok|FAIL
metric olc_mps2 sim=<1 decimal> ref=<1 decimal> err=<signed 4 decimals> norm=<4 decimals> ok|FAIL
metric static_crush_m sim=<4 decimals> ref=<4 decimals> err=<signed 4 decimals> norm=<4 decimals> ok|FAIL
info dyn_crush_m sim=<4 decimals> ref=<4 decimals>
pulse_score=<4 decimals>
pulse: ok
```

When the meta has no `static_crush_m`, its line is `metric static_crush_m sim=<4 decimals> ref=none` and it is not gated. The last line is `pulse: ok` or `pulse: FAIL <rule and numbers>` (for example `pulse: FAIL peak_mps2 norm=1.3120`).

## The frozen numbers

Every number that decides a verdict or bounds the calibration is on a line starting `frozen:` below. The commands repeat the margins as flags; a reviewer checks that they agree. `validation/sweep.py` (n20-10) and `tests/validation_files.rs` (n20-08) **read these lines from this file** (never their own copy), so a range cannot drift between the doc, the sweep and the test. Check 4.6 fails if any `frozen:` line or any `--margin-`, `--setup-` or `--pulse-` flag in this file changed after calibration began.

```
frozen: protocol = NHTSA NCAP full frontal rigid barrier 56 km/h
frozen: h_s = 0.0001
frozen: duration_s = 0.2
frozen: window_s = 0.15
frozen: cfc_shape = 60
frozen: cfc_int = 180
frozen: threshold_mps2 = 20
frozen: crush_tail_s = 0.05
frozen: setup_mass_tol = 0.01
frozen: setup_speed_tol = 0.005
frozen: margin dv = 0.05
frozen: margin peak = 0.15
frozen: margin peak_time_s = 0.010
frozen: margin duration = 0.10
frozen: margin olc = 0.15
frozen: margin static_crush = 0.20
frozen: steel_E_pa = 210e9
frozen: range crush.area_m2 = 0.00007 0.002
frozen: range crush.yield_strain = 0.0010 0.0057
frozen: range crush.damping_ratio = 0 0.05
frozen: grid crush.area_m2 = 7 log
frozen: grid crush.yield_strain = 5 lin
frozen: grid crush.damping_ratio = 0 0.01 0.025 0.05
frozen: step_tol_fraction_of_margin = 0.2
frozen: m5_rtf_max = 10
```

### The margins, and why

Each margin is the scatter between repeat tests of one car (V6): a sim inside it cannot be told apart from another run of the real test, and asking for more would be asking the sim to reproduce noise. None was chosen by looking at a sim result (none exists yet).

- **Delta-V 5%.** The start speed is set to the test's within 0.5% (setup rule), so the delta-V error is the rebound error. Repeat tests scatter by 2 to 3% (V6); 5% is about 3 km/h of rebound at 56 km/h, a third to a half of the rebound itself (V11), so a car with no rebound or double the rebound fails.
- **Peak 15%.** Repeat tests scatter by about 10% (V6), and FE models built for NHTSA match to 10 to 20% (V7). 15% sits between: tighter than the worst FE match, looser than one standard deviation of the tests.
- **Time of peak 10 ms.** Repeat tests move the peak by a few ms, and by up to about 10 ms when two peaks (rails, then engine against the barrier) are nearly equal (V6). An absolute margin, because a relative one would be tight for an early peak and loose for a late one.
- **Duration 10%.** Duration follows from the crush length over the mean speed; repeat tests of one structure crush to within a few cm of each other (V8 measurement plus V6), about 10% of a 0.5 to 0.7 m crush.
- **OLC 15%.** OLC is an integral severity measure of the same pulse (V4); its scatter follows the peak's (V6).
- **Static crush 20%.** The reference is one mean of 6 profile points measured to about 1 cm (V8), across a front that does not crush evenly, while the sim measures two group centroids; 20% allows for the different definitions, which is why it is the loosest.
- **Setup 1% mass, 0.5% speed.** NHTSA weighs the test vehicle and reports the speed to 0.1 km/h (V1); these only catch the wrong file.
- **Step tolerance one fifth of each margin** (check 4.5): the discretisation error then moves a norm by at most 0.2, so it cannot decide a verdict on its own.

### What is calibrated, and its physical range

Only the material `crush`, by three physical quantities. The file's numbers are computed from them, per file, so test B's car (other lengths, other node masses) gets the same physics, not the same raw numbers:

| Quantity | Range | Becomes in the file | Why this range |
|---|---|---|---|
| P1 `area_m2`, the steel cross-section one crush beam stands for | 0.00007 to 0.002 m^2 | `stiffness` k = E A / L_c, E = 210 GPa, L_c = the mean rest length of the `crush` beams of that file | thinnest body sheet (0.7 mm) over a 100 mm strip to thickest rail sheet (2.5 mm) over a full 800 mm rail perimeter (V9) |
| P2 `yield_strain` | 0.0010 to 0.0057 | `yield_strain` as is | yield strength 210 to 1200 MPa over E = 210 GPa (V9); with P1 it makes the beam's plateau force A x sigma_y, a real steel force |
| P3 `damping_ratio` | 0 to 0.05 | `damping` c = 2 zeta sqrt(k m_c), m_c = the mean mass of the nodes at the ends of the `crush` beams | welded steel structures, 0.5 to 5% of critical (V10) |

Not calibrated, and why: geometry and masses (measured, n20-08); every other material (as `car_basic.bsn`; they sit outside the crush zone); `break_strain` (none in `crush`: a rigid-barrier crush folds the front, it does not tear it); and the global contact constants `RESTITUTION`, `REST_SPEED`, `FRICTION_RATE` (they act on every body; a knob that also moves the cube and the box would be tuning by the back door). If the model misses because of one of these, that is a finding for m5 or `IDEAS.md`, never a new knob.

n20-08 makes the `crush` beams of one file the same rest length within 5% (so L_c means something), and records L_c, m_c and the step limit at the top of P1's range in `validation/CAR.md`.

### The calibration procedure (n20-10)

A fixed grid, not hand-tuning: P1 at 7 values from 0.00007 to 0.002 m^2 spaced evenly in log, P2 at 5 values from 0.0010 to 0.0057 spaced evenly, P3 at 0, 0.01, 0.025, 0.05: 140 points, each run on **test A only** with the 4.4 command (its `--body` replaced by a variant file under `scratch/m4-sweep/` that differs from `validation_wall.bsn` only in the `crush` material line). The chosen point is the lowest `pulse_score`, ties to the lowest grid index. A point refused for its step limit is logged as `refused`, never "fixed" by changing h. A chosen point on the edge of a range is logged as `edge` and kept: the model wanted a value outside physics, which is a result, not a reason to widen the range.

`validation/CALIBRATION.md` holds, in this order: `calibration 1`, one line per grid point `run A node-beam <index 1..140> area_m2=<> yield_strain=<> damping_ratio=<> score=<4 decimals> verdict=ok|FAIL|refused`, the chosen point `calibrated node-beam <index> area_m2=<> yield_strain=<> damping_ratio=<> score=<4 decimals> [edge]`, then `freeze = <40 hex digits>` (the crash-sim commit that writes the chosen values into the four validation files), then the test B line `run B node-beam substeps=10 score=<4 decimals> verdict=ok|FAIL`.

## Milestone 4: one test matches within a stated margin

### Checks

**4.1 Reference data and sources** (n20-02).
`python -I -B projects/crash-sim/validation/test_convert.py -v > scratch/s6-4-1.txt 2>&1`
PASS iff the run ends `OK`; `projects/crash-sim/validation/` has `test-a.pulse.csv`, `test-a.meta`, `test-b.pulse.csv`, `test-b.meta`; each pulse file's first line is exactly `t_s,accel_mps2`, its first time is <= 0 and its last >= 0.2; each meta has the keys `test`, `vehicle`, `mass_kg`, `speed_kmh`, `channel`, `channel_location`, `sample_rate_hz`, `prefilter`, `url` (and `static_crush_m` only if the report gives it), the two `test` values differ, the two `vehicle` values differ, both `mass_kg` are in [1200, 1400] and both `channel_location` are the same; and `validation/README.md` has, for each of V1 to V11, a line `verify V<n> = verified <source>` (or `corrected <new figure> <source>`, in which case this file was changed first and the check is run again).
FAIL: any of these.

**4.2 Tests.** `broadsword test crash-sim > scratch/s6-4-2.txt 2>&1`
PASS iff the last line is `test: ok — N passed, 0 failed` and the file contains each of:
`Running tests\pulse.rs` and `Running tests\reference.rs` (n20-03, n20-04: the cases named in their done checks);
`Running tests\pulse_report.rs`, `test pulse_report_lines_and_format ... ok`, `test pulse_report_same_run_same_bytes ... ok`, `test pulse_report_fails_on_scaled_reference ... ok` (the reference's acceleration x 1.3 gives `pulse: FAIL peak_mps2`), `test pulse_report_missing_flag_is_usage_error ... ok`, `test gravity_zero_run_has_no_ground_contact ... ok` (n20-09; n20-07's gravity rule);
`Running tests\validation_files.rs`, `test validation_wall_embeds_validation_car ... ok`, `test validation_wall_b_embeds_validation_car_b ... ok`, `test validation_files_share_crush_physics ... ok` (P1 to P3 computed back from each file agree within 1e-9 relative between A and B), `test crush_physics_inside_frozen_ranges ... ok` (both files, ranges read from this file's `frozen:` lines), `test crush_beams_one_length ... ok` (n20-08);
and every STAGE6 m1 to m3 test (STAGE6 1.1, 2.1, 3.1).
FAIL: any failed or missing test.

**4.3 Calibration is the procedure, not a choice** (n20-10).
`python -I -B projects/crash-sim/validation/sweep.py --verify > scratch/s6-4-3.txt 2>&1`
`--verify` reads `validation/CALIBRATION.md` and this file's `frozen:` lines and checks: 140 `run A node-beam` lines whose points are exactly the grid in order; each point inside its range; the `calibrated` line is the lowest score, ties to the lowest index, and carries `edge` iff a value is at a range end; a `freeze =` line follows it.
PASS iff the last line is `calibration: ok`. FAIL: `calibration: FAIL ...`, or the script missing.

**4.4 The frozen car on test A** (the calibration's own result, reproduced).
`broadsword run crash-sim -- cargo run --offline --release -- --body scenarios/validation_wall.bsn --gravity 0 --duration 0.2 --dt 0.001 --substeps 10 --pulse-report validation/test-a.pulse.csv --pulse-sensor sensor --pulse-crush-groups rear,front --pulse-cfc-shape 60 --pulse-cfc-int 180 --pulse-window 0.15 --pulse-threshold 20 --pulse-crush-tail 0.05 --setup-mass-tol 0.01 --setup-speed-tol 0.005 --margin-dv 0.05 --margin-peak 0.15 --margin-peak-time 0.010 --margin-duration 0.10 --margin-olc 0.15 --margin-static-crush 0.20 > scratch/s6-4-4.txt 2>&1`
PASS iff the output has every line of the pulse report format, both `setup` lines end `ok`, `ground_contacts_in_window=0`, and `pulse_score` equals the `calibrated` line's score to 4 decimals (the sweep's runs are the real command, and the result is deterministic). `run: ok` vs `run: FAILED (exit 1)` is **not** gated here: A's verdict is recorded, not required.
FAIL: a missing line, a `setup ... FAIL`, a ground contact, an abort, exit 2, or a different score.

**4.5 Step size** (on A, so B is still unseen). The 4.4 command with `--substeps 20` (h = 0.05 ms), `> scratch/s6-4-5.txt 2>&1`.
PASS iff for each gated metric |sim(20) - sim(10)| <= 0.2 x that metric's margin, as relative change for relative metrics (dv 1%, peak 3%, duration 2%, OLC 3%, static crush 4%) and absolute for `peak_time_s` (2 ms). FAIL: any one larger (the result belongs to the step size, not the model).

**4.6 Frozen before B.** Checked after 4.7 has run (it looks for 4.7's `run B` line). In Git Bash from the Broadsword root:
```
{ R=projects/crash-sim; F=$(sed -n 's/^freeze = \([0-9a-f]\{40\}\)$/\1/p' $R/validation/CALIBRATION.md); echo "freeze=$F"; git -C $R diff --stat "$F" HEAD -- scenarios/validation_car.bsn scenarios/validation_wall.bsn scenarios/validation_car_b.bsn scenarios/validation_wall_b.bsn; C=$(git -C $R log --reverse -S "run A node-beam 1 " --format=%H -- validation/CALIBRATION.md | head -1); echo "first_calibration=$C"; git -C $R diff "$C^" HEAD -- docs/STAGE6-M4-M5.md | grep -E '^[-+].*(frozen:|--margin-|--setup-|--pulse-)'; grep -n -E '^(freeze = |run B node-beam )' $R/validation/CALIBRATION.md; } > scratch/s6-4-6.txt 2>&1
```
PASS iff `freeze=` is followed by 40 hex digits, `first_calibration=` by 40 hex digits, the two diffs print nothing (the validation files did not change after the freeze; no added or removed line of this file containing `frozen:`, `--margin-`, `--setup-` or `--pulse-` in or after the commit that logged the first calibration run), and the last two lines are the `freeze =` line and then exactly one `run B node-beam ` line, in that order.
FAIL: any other output (a missing freeze, a change after it, a margin moved after calibration began, B run before the freeze or more than once).

**4.7 Test B, once, frozen: the m4 verdict.**
`broadsword run crash-sim -- cargo run --offline --release -- --body scenarios/validation_wall_b.bsn --gravity 0 --duration 0.2 --dt 0.001 --substeps 10 --pulse-report validation/test-b.pulse.csv --pulse-sensor sensor --pulse-crush-groups rear,front --pulse-cfc-shape 60 --pulse-cfc-int 180 --pulse-window 0.15 --pulse-threshold 20 --pulse-crush-tail 0.05 --setup-mass-tol 0.01 --setup-speed-tol 0.005 --margin-dv 0.05 --margin-peak 0.15 --margin-peak-time 0.010 --margin-duration 0.10 --margin-olc 0.15 --margin-static-crush 0.20 > scratch/s6-4-7.txt 2>&1`
Run once, after 4.6's freeze; its score and verdict are the `run B node-beam` line.
PASS iff `run: ok` and the last output line is `pulse: ok`.
FAIL: `run: FAILED`, any `FAIL` line. **A FAIL is a valid result:** it is recorded in `evidence/stage-6-m4.md` with the metric table, and nothing is re-calibrated on B (n20-10). The orchestrator files the follow-up (a model change, then a new test pair C and D, because B has been seen).

**4.8 Every number has a source.** `validation/CAR.md` (n20-08) lists, for each of A's and B's cars, each matched dimension and mass with its source, L_c, m_c and the step limit at the top of P1; and `validation/README.md` (n20-02) gives each test's number, vehicle, mass, speed, channel and URL.
PASS iff a reviewer finds every listed item with a source. FAIL: any item without one.

Milestone 4 is done when 4.1 to 4.8 pass on crash-sim main with the output saved in `evidence/stage-6-m4.md` (n20-10), and the m4 retro entry is in `RETRO.md`.

What guards against tuning to the test in milestone 4: the margins come from repeat-test scatter and are frozen, with Git proof, before the first calibration run (4.6); only three physical quantities move, inside ranges taken from steel data, read by the sweep and the test from one place (4.2, 4.3); the calibration is a fixed grid whose winner a script re-derives (4.3); the step-size run shows the result is the model's (4.5); B is a different car, run once after a commit-level freeze (4.6, 4.7); and a miss on B is recorded rather than fixed.

## Milestone 5: a higher-fidelity crash zone that measurably beats node-beam

### Does m5 need the numerics core (ADR 0001)?

**m4: no.** The J211 filter is a 1D recursive filter and the metrics are sums (n20-03); no matrix.

**m5: only if n20-06 recommends an implicit method.** The checks below do not depend on the method. Production crash codes integrate explicitly with lumped masses (central differences, as crash-sim's symplectic Euler), so an explicit FEM or shell crash zone needs no linear solve, only a smaller step. An implicit method (backward Euler node-beam, or implicit FEM) needs, each step, a Newton solve of (M / h^2 + K_t) dx = r with the tangent stiffness K_t: assembly of a sparse symmetric matrix from per-element triplets (duplicates summed), sparse matrix-vector product, its diagonal, and a Jacobi-preconditioned conjugate-gradient solve that reports non-convergence as an error. CG needs a positive definite matrix, and a beam under compression has a negative geometric stiffness, so the method must also say how it keeps K_t positive semi-definite (for example dropping the negative geometric term, which only slows Newton). That is exactly n20-05's brief; **n20-05 is boarded only if n20-06 picks an implicit method**, and dropped otherwise.

### What "measurably beats" means

On test B, with the same frozen-calibration process on test A, within a stated run time:

- **Same process.** At most 3 calibrated physical quantities, each with a physical range and a source, written as `frozen:` lines in `docs/M5-SOLVER.md` (n20-06) before the first m5 calibration run; a fixed grid of at most 140 points on A only, the lowest `pulse_score` chosen; then one run on B. The node-beam result it is compared with is m4's 4.7, never re-run.
- **Same metrics.** The `pulse_score` of the same pulse report with the same flags (only `--body`, `--solver m5` and `--substeps` differ).
- **Noise.** Each solver's step-size sensitivity on A: d = |`pulse_score` at substeps s - at 2 s| (node-beam: 4.4 vs 4.5; m5: 5.4). m5 beats node-beam iff **S_B(m5) + d(m5) < S_B(node-beam) - d(node-beam)**, so the gap is bigger than what the step size alone can move, and **no gated metric's norm on B is higher for m5 than for node-beam by more than 0.5** (m5 may not buy the score by giving up one metric).
- **Run time.** Median `rtf` of three timed runs on A is at most 10 (`frozen: m5_rtf_max`). Reason: at 10 times slower than real time a 0.15 s crash computes in 1.5 s, which the Stage 7 game can still show as a replay; slower than that it is an offline tool, and a solver that wins only by being 100 times slower has won by brute force. node-beam's `rtf` is recorded for comparison, not gated.

The m5 step is `--dt 0.001 --substeps N5`, with N5 a `frozen: m5_substeps` line in `docs/M5-SOLVER.md`; N5 >= 2, so the sample rate is at least J211's 1.8 kHz for CFC 180 (V3).

### Checks

**5.1 Tests.** `broadsword test crash-sim > scratch/s6-5-1.txt 2>&1`
PASS iff `test: ok — N passed, 0 failed` and the file contains each of: `Running tests\solver_m5.rs`, `test m5_solver_plugs_in ... ok` (through the `Solver` trait, nb-31), `test m5_same_input_same_bytes ... ok`, `test m5_coupling_creates_no_energy ... ok` (the boundary test n20-06 specifies: total energy of a free, undamped, elastic body across the crash-zone boundary never rises above its start by more than the integrator's stated bound), `test m5_files_differ_only_in_crush_lines ... ok` (n20-11); `test default_csv_matches_main_fixture ... ok` (the default solver and CSV are byte-identical); and every 4.2 test.
FAIL: any failed or missing test.

**5.2 The method and its ranges are fixed first.** `docs/M5-SOLVER.md` (n20-06) names the method, its `frozen:` lines (at most 3 ranges with sources, the grid, N5), and the requirements n20-11 was built to.
PASS iff a reviewer finds each, `src/solver_m5.rs` implements the named method, and the same Git test as 4.6 holds for it: in Git Bash, `{ R=projects/crash-sim; C=$(git -C $R log --reverse -S "run A m5 1 " --format=%H -- validation/CALIBRATION.md | head -1); echo "first_m5_calibration=$C"; git -C $R diff "$C^" HEAD -- docs/M5-SOLVER.md docs/STAGE6-M4-M5.md | grep -E '^[-+].*(frozen:|--margin-|--setup-|--pulse-)'; } > scratch/s6-5-2.txt 2>&1` prints only the `first_m5_calibration=` line with 40 hex digits.
FAIL: any of these.

**5.3 m5 calibration on A.** `python -I -B projects/crash-sim/validation/sweep.py --verify --solver m5 > scratch/s6-5-3.txt 2>&1`
The m5 block of `validation/CALIBRATION.md` follows the m4 block, in the same format with `m5` for `node-beam` and the m5 quantity names, its grid read from `docs/M5-SOLVER.md`, then `freeze_m5 = <40 hex digits>` (the commit writing the values into `scenarios/validation_wall_m5.bsn` and `scenarios/validation_wall_b_m5.bsn`), then one `run B m5 substeps=<N5> ...` line.
PASS iff the last line is `calibration: ok`. FAIL otherwise.

**5.4 m5 on A: score, step size, run time.** Three times:
`broadsword run crash-sim -- cargo run --offline --release -- --body scenarios/validation_wall_m5.bsn --solver m5 --timing --gravity 0 --duration 0.2 --dt 0.001 --substeps <N5> --pulse-report validation/test-a.pulse.csv --pulse-sensor sensor --pulse-crush-groups rear,front --pulse-cfc-shape 60 --pulse-cfc-int 180 --pulse-window 0.15 --pulse-threshold 20 --pulse-crush-tail 0.05 --setup-mass-tol 0.01 --setup-speed-tol 0.005 --margin-dv 0.05 --margin-peak 0.15 --margin-peak-time 0.010 --margin-duration 0.10 --margin-olc 0.15 --margin-static-crush 0.20 > scratch/s6-5-4-<1,2,3>.txt 2>&1`
and once with `--substeps <2 x N5>` to `scratch/s6-5-4-half.txt`.
Also once, for the record (not gated), the 4.4 command with `--timing` added, to `scratch/s6-5-4-nb.txt` (node-beam's `rtf`).
PASS iff the program's lines of the three runs (between broadsword's job lines and `--- summary ---`) are identical apart from the `timing:` line, `pulse_score` equals the `calibrated m5` line's score, the step-size rule of 4.5 holds between N5 and 2 N5, and the median of the three `rtf=` values is <= 10.
FAIL: any of these.

**5.5 m5 on B, once, frozen.** The 5.4 command (with `--substeps <N5>`) with `--body scenarios/validation_wall_b_m5.bsn` and `--pulse-report validation/test-b.pulse.csv`, `> scratch/s6-5-5.txt 2>&1`, run once after `freeze_m5`.
PASS iff the output has every pulse report line, both `setup` lines `ok`, `ground_contacts_in_window=0` and no abort. (Its own `pulse:` verdict is recorded, not gated: m5 is judged against node-beam in 5.6, whether or not either passes m4's margins.)

**5.6 The verdict.** From the saved files: S_B(node-beam) and its norms from 4.7, d(node-beam) from 4.4 and 4.5, S_B(m5) and its norms from 5.5, d(m5) from 5.4.
PASS iff S_B(m5) + d(m5) < S_B(node-beam) - d(node-beam), and for every gated metric norm_m5 - norm_node-beam <= 0.5. The four numbers and the per-metric norms are written into `evidence/stage-6-m5.md`.
FAIL: either rule broken. As in m4, a FAIL is recorded, not fixed on B.

Milestone 5 is done when 5.1 to 5.6 pass on crash-sim main, with the evidence saved and the m5 retro entry in `RETRO.md`.

What guards against tuning to the test in milestone 5: the same process, metrics and test B as m4, frozen before m5's calibration starts (5.2, 5.3); node-beam's number is m4's single B run, so it cannot be made worse after the fact; a gain must exceed both solvers' step-size noise; a gain bought on one metric at the cost of another fails; and a run-time cap stops brute force from counting as fidelity.

## Who builds what

| Check | Brief(s) |
|---|---|
| 4.1 reference pulses, meta keys, README, V1 to V11 verified | n20-02 |
| 4.2 `tests/pulse.rs`; the filter and metrics used by the report | n20-03 |
| 4.2 `tests/reference.rs`; `load_pulse`, `load_meta`, `resample`, `compare` | n20-04 |
| m5 numerics (sparse assembly, SpMV, Jacobi-PCG), only if n20-06 picks an implicit method | n20-05 |
| 5.2 `docs/M5-SOLVER.md` with its `frozen:` lines and N5 | n20-06 |
| `gravity >= 0` in `Config`; `pulse`, `reference` (and `numerics`) registered in `lib.rs` | n20-07 |
| 4.2 `tests/validation_files.rs`; 4.8 `validation/CAR.md`; the four validation `.bsn` files, their groups and `crush` material | n20-08 |
| 4.2 `tests/pulse_report.rs`; `--pulse-report` and all its flags, lines, rules and exit codes; `--pulse-csv`; `--timing` | n20-09 |
| 4.3 `validation/sweep.py` and its test; 4.4 to 4.7 runs; `validation/CALIBRATION.md` (m4 block); calibrated values in the four files; `evidence/stage-6-m4.md`; retro | n20-10 |
| 5.1 `tests/solver_m5.rs`; `--solver m5`; the two `_m5` files; 5.3 to 5.6 runs, the m5 block of `CALIBRATION.md`, `--verify --solver m5`; `evidence/stage-6-m5.md`; retro | n20-11 |

## What the briefs must take from this file

Where a brief and this file differ, this file sets the check; the brief's coder raises the difference with the orchestrator. Claims that must be added before a brief is boarded are listed with it.

- **n20-02:** A and B are two different vehicles, 1200 to 1400 kg, the same accelerometer location in both (an occupant-compartment x channel, rear-seat crossmember or rear sill, behind the crush zone); keep the samples before time zero (the filter needs them), and a record to at least 0.2 s; meta keys exactly as 4.1 (`static_crush_m` absent, never 0, when not reported; the mean of the profile points when it is); the `verify V<n> = ...` lines for V1 to V11 in `validation/README.md`; `test_convert.py` finds its sample through its own file path, so 4.1 runs from the Broadsword root.
- **n20-03:** `peak` returns the most negative value and its time; `duration` is the time from the first to the last sample at or below -threshold; `olc` follows V4 with v0 in m/s and returns m/s^2; `delta_v` is the trapezoid integral; the CFC filter refuses a sample rate under 10 x CFC. If it has already merged with a different meaning, n20-09 adapts at the call site and says so.
- **n20-04:** `load_meta` allows the extra keys of 4.1; `resample` is used on [0, window] only, after filtering at the native rate; relative error is (sim - ref) / ref.
- **n20-05:** boarded only if `docs/M5-SOLVER.md` picks an implicit method; then also: the caller keeps the matrix positive semi-definite (above), and CG errors, never returns, on an indefinite or non-converged system.
- **n20-06:** at most 3 calibrated quantities, each a physical quantity with a range and a source, as `frozen:` lines, plus `frozen: m5_substeps = N5` (N5 >= 2) and its grid (at most 140 points); the energy test at the coupling boundary named in 5.1; an estimate of the rtf against the cap of 10; and where m5's parameters live in the `.bsn` (if the format must change, that is a new brief before n20-11, not part of it).
- **n20-07:** besides registering modules: `Config` accepts `gravity = 0` (`gravity >= 0`, still finite), and nothing else in `lib.rs` changes; the default CSV stays byte-identical. Its claims already cover `src/lib.rs`.
- **n20-08:** the groups `all`, `sensor`, `front`, `rear` and the material `crush`; front toward +x, wall 0.05 m ahead with normal (-1, 0, 0); every node and wheel starts at v0 = `speed_kmh` / 3.6 (if the car format cannot give the wheels a start velocity, stop and report); `crush` beams one length within 5%; the four tests named in 4.2, reading the ranges from this file's `frozen:` lines; the B car built the same way from test B's data. **Claims to add:** `scenarios/validation_car_b.bsn (new)`, `scenarios/validation_wall_b.bsn (new)`, `tests/validation_files.rs (new)`.
- **n20-09:** every flag, rule, output line, format and exit code of "The pulse report"; the sim pulse from the `sensor` group's mass-weighted mean velocity, every substep; the time base and filter order of "How the two pulses are made comparable"; the ground-contact rule; `dyn_crush_m`; `--timing` on stderr only; the five tests named in 4.2.
- **n20-10:** the grid, the logging format and the order of `validation/CALIBRATION.md`; B run exactly once, after the freeze commit; the sweep runs the 4.4 command with only `--body` changed, writes its variant files under `scratch/m4-sweep/`, refuses to start inside a broadsword job, finds the Broadsword root from its own path, and reads ranges and grid from this file's `frozen:` lines. **Claims to add:** `projects/crash-sim/validation/sweep.py (new)`, `projects/crash-sim/validation/test_sweep.py (new)`, and the calibrated `crush` line in `scenarios/validation_wall.bsn`, `scenarios/validation_car_b.bsn`, `scenarios/validation_wall_b.bsn` (it lists only `validation_car.bsn`).
- **n20-11:** `--solver m5` (default unchanged), the five tests in 5.1, the `_m5` files differing from the node-beam ones only in the calibrated lines, the m5 block of `CALIBRATION.md` and `sweep.py --solver m5`, and the comparison rule of 5.6 against m4's recorded B run (never re-run). **Claims to add:** `scenarios/validation_wall_m5.bsn (new)`, `scenarios/validation_wall_b_m5.bsn (new)`, `validation/CALIBRATION.md (append the m5 block)`, `validation/sweep.py (the --solver m5 option)`. Its runs and evidence (5.3 to 5.6) can be split off as a records task like n20-10.
