# Stage 6 done checks: milestones 1 to 3

Written before any Stage 6 code, from crash-sim `abe471c`. DEVELOPMENT_PLAN Stage 6:

1. Stable node-beam solver: fixed timestep, substeps, damping, behind a swappable interface. *Done:* the cube settles without jitter or explosions.
2. Deformation and breaking. *Done:* a box hits a wall and crumples permanently.
3. Drivable car: your own car file format, wheels and suspension. *Done:* drive a car around and crash it.

This file turns each line into commands with numbers. A milestone is done when every one of its checks passes on crash-sim main (and render main where named), and the output is saved in `evidence/`. The briefs that build each part are in `state/next-batch-2026-11-19/` (nb-31 to nb-41); the table at the end maps every check to its brief.

## How to run and read a check

- Run every command from the Broadsword root. `broadsword` means `build/bin/broadsword.exe`. Inside a task, add `--task <id>` after the project name.
- Never call cargo directly. Builds and tests go through `broadsword test crash-sim`; a program run goes through `broadsword run crash-sim -- cargo run --offline --release -- <flags>`. The working folder of the program is the project folder, so `scenarios/...` paths are relative to `projects/crash-sim`.
- Redirect each command to a file in `scratch/` (for example `> scratch/s6-1-3.txt 2>&1`) and read the file. Never pipe a broadsword command into another program.
- `broadsword run` prints its own job lines first (`job:`, `source:`, `checkpoint:`, `build source:`), then the program's lines, then `--- summary ---` and `run: ok (...)` or `run: FAILED (exit N)`. "The output" below means the program's lines.
- `broadsword test` ends with `test: ok — N passed, 0 failed`. Above it, each test prints `test <name> ... ok`.
- Each check says what PASS is. Anything else is FAIL, including a check that cannot be run because a flag, file or test is missing.
- Numbers marked *measured* explain the thresholds; they are not themselves pass conditions. Except where a line says *`--summary`*, they come from a scratch probe that is **not committed** (it lived in `scratch/t28-probe/` and may be wiped): a small Rust program with crash-sim at `abe471c` as a path dependency (this file was committed on top of `abe471c` and changes no code). To redo it: for each case build `Config { duration, dt: h, substeps: 1, drop_height, tilt, gravity, ..Config::default() }`, so one `Sim::advance()` is exactly one substep of length h; after every advance read `sim.pos` and `sim.vel` and record total energy E (as defined below, using `Beam::rest` and `BEAM_STIFFNESS`), max node speed (`sim.max_speed()`), the mean node y, and whether any node has `pos.y == 0.0` and `vel.y > 0.0` (an impact: the contact rule only leaves a node at y = 0 moving up when it reversed a contact faster than `REST_SPEED`). From these: the last impact; the last substep at which max node speed >= V (the settle time); max of E(t) - E(0); the largest E(t2) - E(t1), t1 <= t2, after the last impact; the largest max node speed in the final second. The cases are the drop heights, tilts, g and h named next to each figure (default: drop and tilt as `Config::default()`, g = 9.81, h = 1 ms). The static sag is a separate linear solve, not a run (method in "The numbers, and why" below). Figures marked *`--summary`* are reproducible with committed code: `broadsword run crash-sim -- cargo run --offline --release -- --summary` prints `first_contact_s=0.6390,peak_strain=0.045146,final_center_y=0.4999`.

## Shared definitions

- **Substep:** one physics step of h = dt / substeps (default 0.01 / 10 = 1 ms). Its time is `step * dt + (k + 1) * h`, as `first_contact_s` uses. Every report below measures every substep, never only the output steps (audit S14: output sampling missed contacts).
- **Impact:** a substep in which the contact rule bounces a node: the node ended the substep behind a collider plane (today only the ground, y < 0) while moving into it faster than `REST_SPEED` (0.2 m/s), so its normal velocity is reversed and scaled by `RESTITUTION`. Resting contact (slower than `REST_SPEED`) is not an impact, so a body lying on the ground has a last impact. `last_impact_s` is the time of the last impact in the run, or `none`.
- **Total energy E:** kinetic plus gravitational plus elastic, in joules, with potential measured from y = 0:
  E = sum over nodes of (1/2) m_i |v_i|^2 + m_i g y_i, plus sum over unbroken beams of (1/2) k_b (len_b - rest_b)^2, using each beam's current rest length (it changes when a beam yields, nb-34). With wheels (nb-39) add each wheel's (1/2) m_w |v_w|^2 + (1/2) I_w w^2 + m_w g y_w and the stored energy of every suspension spring, bump stop and tyre spring. When gravity is tilted (`--slope-grip-frac`, check 3.4), every m g y term becomes -m (g_vec . r), which is the same thing when gravity is not tilted.
- **Max node speed:** the largest |v_i| over all nodes (and wheel centres, once wheels exist) at one substep.
- **Abort:** the run stops because `Sim::check()` fails (a non-finite position or velocity, or strain over `MAX_STRAIN_LIMIT` = 0.5). An abort is always a FAIL and always exit code 1.

## Milestone 1: stable solver, the cube settles without jitter or explosions

### The settle report (nb-32)

`crash-sim --settle-report` replaces the CSV with a report. It needs all five threshold flags; there are no defaults in code, and a missing one is a usage error (exit 2, empty stdout):

| Flag | Symbol | Meaning |
|---|---|---|
| `--settle-by <s>` | T | from this time on, the body must be at rest |
| `--settle-hold <s>` | S | the run must go on at least this long after T |
| `--settle-speed <m/s>` | V | "at rest" means max node speed below this |
| `--energy-rise <J>` | E | after T, total energy may rise by at most this |
| `--energy-gain <fraction>` | G | over the whole run, E(t) may exceed E(0) by at most G * E(0) |

Rules, checked in this order; the first that fails is named in the FAIL line:

1. The run ends (at its last substep) no earlier than T + S.
2. At every substep with t >= T, max node speed < V.
3. Over the substeps with t >= T, the largest rise E(t2) - E(t1) with t1 <= t2 is at most E.
4. At every substep, (E(t) - E(0)) / E(0) <= G. With `--input` (milestone 3) the engine adds energy, so the rule becomes (E(t) - E(0) - W(t)) / (E(0) + W(t)) <= G, where W(t) is the work the drive torque has done on the wheels up to t.

Output, one `key=value` per line, in this order:

```
last_impact_s=<4 decimals, or none>
settled_from_s=<4 decimals, or never>        earliest substep from which max node speed stays below V to the end
max_speed_after_settle_by=<m/s, {:.3e}>      largest max node speed at t >= T
energy_rise_after_settle_by_j=<J, {:.3e}>    the rule 3 value
max_energy_gain_frac=<{:.3e}>                the rule 4 value
peak_strain=<6 decimals>                     largest beam strain in the run (unbroken beams, elastic strain)
final_center_y=<m, 10 decimals>              mean y of all nodes at the last substep
settle: ok
```

The last line is `settle: ok`, or `settle: FAIL <rule and numbers>` (for example `settle: FAIL speed 0.0123 m/s >= 0.001 at t=2.1000`). On an abort it is `settle: FAIL unstable at t = ... s: <reason>`. Exit code 0 only when every requested verdict is `ok`; 1 on any FAIL or abort; 2 on a usage error.

### The numbers, and why

All four come from the physics of today's cube, then checked against the measurement.

- **V = 0.001 m/s.** At h = 1 ms gravity adds g * h = 0.0098 m/s to every node every substep. A contact rule that leaves even one substep of that kick visible (the classic resting jitter) shows up as about 0.01 m/s, ten times V. V is also 200 times below `REST_SPEED`, so it cannot be met by a node that is still bouncing. *Measured:* in the last second of a 10 s default run the max node speed is 3.8e-9 m/s.
- **T = 6 s.** The bounces end when an impact is slower than `REST_SPEED`; *measured* last impact 2.525 s. The cube then slides and rocks with only the ground drag to stop it: `FRICTION_RATE` = 5 /s on the 4 touching nodes out of 8 gives a decay rate of about 2.5 /s for the whole cube. From about 0.2 m/s at the last impact to V takes ln(0.2 / 0.001) / 2.5 = 2.1 s, so about 4.6 s. *Measured:* max node speed is below V for good from 4.02 s (h = 1 ms), 3.97 s (2 ms), 4.78 s (0.5 ms), 3.76 s (0.25 ms). The rocking is chaotic (the last impact moves between 2.31 s and 2.93 s with h), so T has 1.2 s of margin over the slowest.
- **S = 4 s (run to 10 s).** Long enough for a slow pump or creep to show: at the 2.5 /s decay, real motion falls by e^-10 over S, so anything still moving at the end is not decaying.
- **E = 0.001 J.** That is the energy to lift the 8 kg cube by 13 micrometres. A contact or friction rule that adds energy while the cube is at rest builds past it quickly. *Measured:* the rise after the last impact is 2.1e-14 J (rounding).
- **G = 1e-5.** The cube only ever loses energy. *Measured:* max E(t) - E(0) is 0 to 6 decimals in J (E(0) = 202.73 J), at every substep, for h from 0.25 to 2 ms. 1e-5 of E(0) is 0.002 J.
- **Peak strain at most 0.10.** *Measured* 0.045146 on the default run (0.0437 to 0.0451 for h = 2 to 0.25 ms); the existing `stable_no_nan_no_penetration_low_strain` test uses the same 0.10. Above it means an explosion, not a bounce.
- **Static sag, independent of the sim.** A cube at rest on a face is a statics problem with a known answer. Solving K u = f for the rest-shape cube (all 28 beams, k = 20000 N/m, 1 kg nodes, g = 9.81; bottom nodes held vertically only, because the ground drag holds no static sideways force, and the three rigid sideways motions removed by also holding node 0's x and z and node 1's z; the 24 x 24 system minus the held rows, by Gaussian elimination) gives each top node a sag of 2.58875e-4 m, which is (19/36) m g / k, so the mean sag is (19/72) m g / k = 1.294375e-4 m. The resting `center_y` must be edge/2 - (19/72) m g / k = 0.4998705625 (Earth) and 0.499978625 (Moon, g = 1.62), within 1% of the sag: +-1.294375e-6 m (Earth) and +-2.1375e-7 m (Moon). *Measured:* 0.4998705 and 0.4999786 (7 decimals) for every drop height, tilt and step size. This number comes from linear statics, not from a run, so it cannot be met by tuning damping or contact constants; if `BEAM_STIFFNESS` or `NODE_MASS` change, recompute it from the formula, never from the sim.

### Checks

**1.1 Tests.** `broadsword test crash-sim > scratch/s6-1-1.txt 2>&1`
PASS iff the last line is `test: ok — N passed, 0 failed` (N at least 54 plus the tests named here), and the file contains each of:
`Running tests\solver.rs`, `test default_csv_matches_main_fixture ... ok`, `test a_second_solver_plugs_in ... ok` (nb-31: the default CSV is byte-identical to a fixture captured on main before the change; a test-only second solver goes through the same `Solver` trait and gives different output, which proves the interface is real),
`Running tests\settle.rs`, `test two_node_energy_matches_hand_value ... ok`, `test planted_energy_injection_fails_settle ... ok`, `test settle_holds_across_the_invariants_grid ... ok` (nb-32; the grid is drop 0.5, 1, 2, 4 m x tilt on and off x g = 9.81 and 1.62, each with duration 20, T = 16, S = 4, V = 0.001, E = 0.001, G = 1e-5, `settle: ok`, and the resting `center_y` within 1% of the static sag for its g; *measured* slowest settle 12.3 s, 4 m tilted Moon drop at h = 0.5 ms).
FAIL: any failed test, a missing line, or a smaller N.

**1.2 The existing tests are unchanged.** `git -C projects/crash-sim diff --stat abe471c -- tests/csv.rs tests/determinism.rs tests/energy.rs tests/invariants.rs tests/physics.rs tests/risky.rs tests/stability.rs tests/summary.rs tests/sweep.rs`
PASS iff it prints nothing. FAIL: any output (an existing test was edited to fit new code).

**1.3 The default cube settles.**
`broadsword run crash-sim -- cargo run --offline --release -- --duration 10 --settle-report --settle-by 6 --settle-hold 4 --settle-speed 0.001 --energy-rise 0.001 --energy-gain 0.00001 > scratch/s6-1-3.txt 2>&1`
PASS iff `run: ok`, the output's last line is `settle: ok`, `peak_strain` <= 0.100000, and 0.4998692682 <= `final_center_y` <= 0.4998718568. The band is 0.4998705625 +- 1.294375e-6 (1% of the sag), rounded inward to the 10 printed decimals, so a value that passes is always within 1%.
FAIL: `run: FAILED`, any `settle: FAIL`, `peak_strain` above 0.1, or `final_center_y` outside the band. Expected values (*measured*, not gated): `last_impact_s` about 2.5, `settled_from_s` about 4.0, `peak_strain=0.045146` (*`--summary`* prints the same peak for its 5 s run).

**1.4 The report can fail.** Same command with `--settle-by 2 --settle-hold 8` (the cube is still bouncing at 2 s).
PASS iff `run: FAILED (exit 1)` and the last output line starts with `settle: FAIL speed`. FAIL: `settle: ok` (the report cannot see motion).

**1.5 The result is physics, not a step-size artefact.** The 1.3 command three more times, adding `--substeps 5`, `--substeps 20` and `--substeps 40` (h = 2, 0.5 and 0.25 ms; all below the 3.78 ms limit).
PASS iff each prints `settle: ok`, each `final_center_y` is in the 1.3 band, and each `peak_strain` is within 10% of the 1.3 value (*`--summary`* with `--substeps 5`, `20`, `40`: 0.043699, 0.044286, 0.044246, within 3.2% of 0.045146). FAIL: any of these.

What guards against tuning to the test in milestone 1: the static sag in 1.3 and 1.5 is predicted by an independent statics solve, so it fails if the stiffness or mass is changed to settle faster; 1.5 fails if settling only works at one step size; the grid in 1.1 fails if T, V or E were picked for one drop; 1.4 proves the report can say FAIL; 1.2 stops the old tests from being bent to the new code.

## Milestone 2: a box hits a wall and crumples permanently

### The scenario (nb-33 box, nb-36 wall and file)

`scenarios/box_wall.bsn` holds:

- the nb-33 box: 2.0 x 1.0 x 1.0 m, 3 x 2 x 2 nodes, long axis along x, 1 kg per node (the cube's node mass, so today's step-limit reasoning carries over), with groups `back` (the 4 nodes at the smallest x) and `front` (the 4 at the largest x);
- a plastic material (stiffness, damping, `yield_strain`, and `break_strain` if used), each number with a source or a reason in PHYSICS.md (nb-36), never picked to make this check pass;
- the box resting on the ground (lowest node at y = 0), not tilted, every node moving at (10, 0, 0) m/s;
- a wall: a plane through a point 0.10 m beyond the front face, normal (-1, 0, 0).

Four variants, each differing from `box_wall.bsn` in exactly one number or one line (a test checks it):
`box_wall_elastic.bsn` (no `yield_strain` and no `break_strain`), `box_wall_soft.bsn` (stiffness x 0.5), `box_wall_stiff.bsn` (stiffness x 2), `box_wall_slow.bsn` (7.07 m/s, half the kinetic energy).

Why 10 m/s: 50 J of kinetic energy per kg, half of the 96 J/kg of a car at the 50 km/h of milestone 3. The 0.10 m gap keeps the ground drag from taking more than about 3% of the speed before impact.

### The crumple report (nb-36)

`--crumple-report` adds to the settle report (it needs the settle flags too) and has its own required flags: `--crumple-groups <from>,<to>`, `--crumple-min <m>`, `--crumple-max <m>`, `--crumple-drift <m>`. The length L(t) is the distance between the centroids of the two groups; the crumple is delta(t) = L(0) - L(t) (positive means shorter).

Rules: (1) delta(t) >= crumple-min at every substep with t >= T; (2) delta(t) <= crumple-max at every substep of the run; (3) max minus min of delta(t) over t >= T is at most crumple-drift (it stays crumpled; it does not creep back).

Output, after the settle lines:

```
crumple_groups=back,front
crumple_start_m=<6 decimals>
crumple_end_m=<6 decimals>
crumple_delta_m=<6 decimals>              delta at the last substep
crumple_delta_min_after_settle_by_m=<6 decimals>
crumple_delta_range_after_settle_by_m=<6 decimals>
crumple_delta_max_m=<6 decimals>          largest delta in the run
crumple: ok
```

or `crumple: FAIL <rule and numbers>` as the last line. Exit code as for the settle report.

### The numbers, and why

- **crumple-min = 0.02 m** (1% of the length). A purely elastic box at rest differs from its start length only by its own weight: about 1e-4 m with 1 kg nodes and k = 20000 N/m (the cube's sag is 1.3e-4 m). 0.02 m is 100 times that, and check 2.4 shows the elastic box below 0.002 m, so there is a factor of 10 on each side.
- **crumple-max = 1.0 m** (half the length). More means the box is crushed flat, which a 50 J/kg impact must not do.
- **crumple-drift = 0.001 m** over S = 4 s. At V = 1 mm/s nothing at rest moves further.
- **T = 6 s, S = 4 s, V = 0.001 m/s, E = 0.001 J**, as milestone 1. The box rebounds at no more than RESTITUTION x 10 = 5 m/s and then slides with the same ground drag (6 of 12 nodes touching, 2.5 /s): ln(5 / 0.001) / 2.5 = 3.4 s, so T has about 2.5 s of margin.
- **G = 0.01.** E(0) = 600 J kinetic + 58.9 J potential = 658.9 J; plasticity, the wall and the drag only remove energy, so any gain is integrator error. Symplectic Euler's measured energy swings by about h * omega / 2 of the energy held in a fast vibration, and an impact puts only a small part of the energy into the stiffest modes. 1% allows that, and nothing more. This number is reasoned, not measured: nb-36 reports the measured value, and if it is above 0.01 it stops and reports instead of raising G.

### Checks

**2.1 Tests.** `broadsword test crash-sim > scratch/s6-2-1.txt 2>&1`
PASS iff `test: ok — N passed, 0 failed` and the file contains each of:
`test cube_bsn_matches_default_cube ... ok`, `test canonical_file_round_trips_byte_identical ... ok`, and one passing test per refusal listed in nb-33 (nb-33, `tests/format.rs`);
`test beam_past_yield_keeps_longer_rest ... ok`, `test energy_never_rises_through_yield ... ok`, `test cube_bsn_output_matches_default ... ok` (nb-34, `tests/deform.rs`; the last one runs `--body scenarios/cube.bsn` and compares its CSV byte for byte with the no-flag run);
`test beam_breaks_at_expected_substep ... ok`, `test disconnected_node_falls_freely ... ok`, `test broken_beam_does_not_trip_strain_limit ... ok`, `test default_cube_output_unchanged ... ok` (nb-35, `tests/breaking.rs`);
`test ground_plane_collider_keeps_cube_output ... ok`, `test box_wall_variants_differ_in_one_number ... ok`, `test break_pull_files_differ_only_in_speed ... ok` (nb-36, `tests/box_wall.rs`);
and every milestone 1 test from 1.1.
FAIL: any failed or missing test.

**2.2** Check 1.2 again. PASS iff it prints nothing. FAIL: any output.

**2.3 The box crumples and stays crumpled.**
`broadsword run crash-sim -- cargo run --offline --release -- --body scenarios/box_wall.bsn --duration 10 --settle-report --settle-by 6 --settle-hold 4 --settle-speed 0.001 --energy-rise 0.001 --energy-gain 0.01 --crumple-report --crumple-groups back,front --crumple-min 0.02 --crumple-max 1.0 --crumple-drift 0.001 > scratch/s6-2-3.txt 2>&1`
PASS iff `run: ok`, the output contains `settle: ok`, its last line is `crumple: ok`, and `crumple_start_m=2.000000`.
FAIL: `run: FAILED`, any `FAIL` line, or a different start length (the wrong groups or the wrong box).

**2.4 Elastic control: the permanence comes from plasticity.** The 2.3 command with `--body scenarios/box_wall_elastic.bsn`.
PASS iff `run: FAILED (exit 1)`, the output contains `settle: ok` (not an abort), the last line starts with `crumple: FAIL`, and |`crumple_delta_m`| <= 0.002000.
FAIL: `crumple: ok` (the box "crumples" without plasticity, so the metric measures something else), |delta| above 0.002, or an abort.

**2.5 The crumple moves the way physics says.** The 2.3 command with `--body` set to `scenarios/box_wall_soft.bsn`, `scenarios/box_wall_stiff.bsn` and `scenarios/box_wall_slow.bsn`.
PASS iff each output contains `settle: ok` (the crumple verdict of these three is not gated), and the `crumple_delta_m` values order strictly as soft > nominal (2.3) > stiff, and slow < nominal. Reason: with a fixed yield strain, a softer beam yields at a lower force, so the same energy crushes it further; less energy crushes less.
FAIL: any `settle: FAIL`, an abort, or either ordering broken (the result does not follow the material, so it was tuned or is numerical noise).

**2.6 Step size.** The 2.3 command with `--substeps 20` added (h = 0.5 ms).
PASS iff `settle: ok`, `crumple: ok`, and its `crumple_delta_m` is within 10% of the 2.3 value. FAIL: any of these.

**2.7 Breaks are counted and reported the same way twice.**
`broadsword run crash-sim -- cargo run --offline --release -- --body scenarios/box_wall.bsn --duration 10 --events > scratch/s6-2-7.txt 2>&1`
The output is the CSV with its extra `broken_beams` column (nb-35), plus one line per break, `break t=<4 decimals> beam=<index> strain=<6 decimals>`. Read it with `grep -c "^break " scratch/s6-2-7.txt` and the CSV row starting `10.0000,`.
PASS iff `run: ok` and the break-line count equals the `broken_beams` value of the `10.0000` row (zero is allowed here: the nominal crash need not break anything; that a beam breaks in a run is gated by 2.9).
FAIL: the two numbers differ, or the column or row is missing.

**2.8 It can be seen.** `broadsword test render > scratch/s6-2-8.txt 2>&1`, then, in Git Bash from the Broadsword root, the golden-file diff of exactly the commits that nb-37's render half (board task `t-20261119-171333-35`) merged:
```
{ Q=$(sed -n 's/^note = merged by queue entry \(q-[0-9-]*\) as .*/\1/p' state/tasks/t-20261119-171333-35.task); git -C projects/render diff --name-status "$(sed -n 's/^base_head = //p' state/queue/$Q.entry)" "$(sed -n 's/^main_sha = //p' state/queue/$Q.entry)" -- tests/golden; } > scratch/s6-2-8-golden.txt 2>&1
```
How it finds the commits: when the queue merges a task it writes a line `note = merged by queue entry q-... as ...` (the entry id, then the merged commit) into the task file (`broadsword task show t-20261119-171333-35` prints the same as `note: ...`). That entry's `base_head` is main's HEAD the branch was rebased onto, and its `main_sha` is the commit main was fast-forwarded to, so the range is exactly nb-37's commits. The task file's own `base` is not used: the queue's rebase does not update it, so it can include other tasks' commits.
PASS iff `test: ok — N passed, 0 failed`, `scratch/s6-2-8.txt` contains `test box_wall_final_frame_matches_golden ... ok`, and `scratch/s6-2-8-golden.txt` has at least one line and every line starts with `A` and a tab (only new golden images). FAIL: a failed test; an empty golden file (nb-37 added no golden image); a `sed:` or `fatal:` line (the task is not merged, or the entry is missing); or an `M`, `D` or `R` line (an existing golden image changed).
Also, the writer and the renderer's fixtures must not drift (nb-46 F2; the writer is `src/trajectory.rs` in crash-sim, its fixtures are `tests/fixtures/trajectory.csv` (v1 cube) and `tests/fixtures/trajectory_v2.csv` (v2 run with one break), both checked against the writer's output by `tests/trajectory.rs`): `cmp projects/crash-sim/tests/fixtures/trajectory.csv projects/render/tests/fixtures/trajectory.csv > scratch/s6-2-8-cmp.txt 2>&1; echo "cmp1=$?" >> scratch/s6-2-8-cmp.txt` and the same for `trajectory_v2.csv` with `>>` instead of `>` and `cmp2` instead of `cmp1`. PASS iff both print `=0` and `scratch/s6-2-8-cmp.txt` has no other text. FAIL: a `cmp:` line, a non-zero value, or a missing render copy of the v2 fixture.

**2.9 A beam breaks in a run, where the closed form says it must, and not where it says it must not** (nb-35 breaking and `--events`; nb-36 the two files and a per-group `velocity`).
`scenarios/break_pull.bsn`: two 1 kg nodes at (0, 2, 0) and (1, 2, 0) m, in groups `left` (node 0) and `right` (node 1); one beam between them (beam 0, rest 1.000 m) of a material with the cube's stiffness k = 20000 N/m and damping c = 20 N s/m, `break_strain 0.010` and no `yield_strain`; `left` starts at velocity (-2.5, 0, 0) m/s and `right` at (2.5, 0, 0) m/s. `scenarios/break_pull_slow.bsn` is the same file with the two speeds 0.75 m/s instead of 2.5 m/s (`break_pull_files_differ_only_in_speed` in 2.1 checks that nothing else differs).
Why these numbers: the stretch x of the beam is a damped oscillator with reduced mass mu = m/2 = 0.5 kg, so omega0 = sqrt(k / mu) = 200 rad/s and zeta = c / (2 sqrt(k mu)) = 0.1. From x = 0 with x' = v_rel, its first and largest peak is x_max = (v_rel / omega0) exp(-(zeta / sqrt(1 - zeta^2)) atan(sqrt(1 - zeta^2) / zeta)) = 0.8626 v_rel / omega0, at t = atan(sqrt(1 - zeta^2) / zeta) / (omega0 sqrt(1 - zeta^2)) = 0.0074 s. Gravity pulls both nodes the same way and does not stretch the beam, and the nodes are in the air until about 0.64 s. v_rel = 5 m/s gives a peak strain of 0.02157, 2.16 x `break_strain`, so the beam must break before 0.0074 s; v_rel = 1.5 m/s gives 0.00647, 0.65 x, so it must never break (later peaks are smaller). At h = 1 ms, omega0 h = 0.2, well inside the step limit, and symplectic Euler stays within a few percent of the closed form, far less than either margin. The material is a test material for the breaking rule, not a real one; it needs no source because the check compares the run with the closed form, not with a measurement.
`broadsword run crash-sim -- cargo run --offline --release -- --body scenarios/break_pull.bsn --duration 1 --events > scratch/s6-2-9.txt 2>&1`
`broadsword run crash-sim -- cargo run --offline --release -- --body scenarios/break_pull_slow.bsn --duration 1 --events > scratch/s6-2-9-slow.txt 2>&1`
PASS iff both print `run: ok`; `scratch/s6-2-9.txt` has exactly one line starting `break `, of the form `break t=T beam=0 strain=S` with T <= 0.0074 and S >= 0.010000, and the `broken_beams` value of its CSV row starting `1.0000,` is 1; and `scratch/s6-2-9-slow.txt` has no line starting `break ` and the `broken_beams` value of its `1.0000,` row is 0.
FAIL: `run: FAILED` (an abort included); no break line in `s6-2-9.txt` (beams do not break in a run); a break later than 0.0074 s or at a strain below 0.010000; any break line in `s6-2-9-slow.txt` (it breaks below its threshold); or a `broken_beams` value that disagrees with the break lines.

What guards against tuning to the test in milestone 2: the material numbers need a stated source before the run (nb-36); the elastic control (2.4) must fail the same metric; the sensitivity runs (2.5) must move in the direction physics predicts; the step-size run (2.6) must agree; the variants may differ in one number only, so a variant cannot be tuned separately; and the break (2.9) is predicted in closed form on both sides of the threshold, so a breaking rule that is too eager or too reluctant fails.

## Milestone 3: drive a car around and crash it

### The files

- `scenarios/car_basic.bsn` (nb-38): about 4 x 1.8 x 1.4 m, about 1200 kg in total, four wheels, rear drive, front steer, every number with a source or a reason. It is mirror-symmetric left to right (the "level" check depends on it), has groups `front` (the front-most chassis nodes) and `rear` (the rear-most), and starts with every tyre's lowest point 0.20 m above the ground with the suspension at its rest length (this is the 0.2 m drop of nb-39).
- `scenarios/pad_and_wall.bsn` (nb-41): the car sections of `car_basic.bsn`, byte for byte, plus one wall plane far enough ahead for the run-up. The car that crashes is the car that was tested.
- `scenarios/pad_lap.input` (nb-41): lines of `t throttle brake steer` (nb-40), first line `0 0 0 0`. It drives one loop of the pad, then straight into the wall, and its last line is exactly `60 0 1 0`: full brake from t_last = 60 s on, after the car has hit the wall. With `--input` and no `--duration`, a run ends at t_last; with a longer `--duration`, the last line's values hold to the end.
- `scenarios/full_brake.input` (nb-40): the single line `0 0 1 0` (full brake, no throttle, no steer, from t = 0), for check 3.4.

### The drive report (nb-40)

`--drive-report` (with `--input`) replaces the per-step drive output with a report. Required flags: `--lap-min-heading <deg>`, `--lap-min-path <m>`, `--lap-close <m>`, `--impact-min-kmh <km/h>`, `--impact-max-kmh <km/h>`. Heading is the direction in the ground plane of the vector from the `rear` centroid to the `front` centroid; its change is unwrapped and summed every substep.

Output:

```
lap_heading_deg=<1 decimal>        largest |heading change| from the start before the first wall contact
lap_path_m=<2 decimals, or never>  path length of the chassis centroid up to the substep where |heading change| first reaches lap-min-heading
lap_close_m=<3 decimals, or never> horizontal distance of the chassis centroid from its start at that substep
chassis_ground_contacts_before_wall=<count>   substeps before the first wall contact in which any chassis node is at or below the ground
wall_first_contact_s=<4 decimals, or none>    first substep in which any chassis node or tyre touches the wall
wall_impact_speed_kmh=<1 decimal, or none>    the chassis centre-of-mass speed along the wall normal, toward the wall, at that substep
final t=<4 decimals> x=<4 decimals> z=<4 decimals> heading_deg=<2 decimals>
drive: ok
```

`drive: ok` iff `lap_heading_deg` >= lap-min-heading, `lap_path_m` >= lap-min-path, `lap_close_m` <= lap-close, `chassis_ground_contacts_before_wall=0`, and the impact speed is within [impact-min, impact-max]; otherwise `drive: FAIL <rule and numbers>`. Without `--drive-report`, a run with `--input` still ends with the same `final ...` line.

### The live protocol (`--live`, task t-20261120-134815)

`crash-sim --body <car.bsn> [--dt <s>] [--substeps <n>] --live` runs the drive as a child process of a viewer (ADR 0001 (a)): input lines in on stdin, rows out on stdout. STAGE7's `--sim-exe` and `race --live` rely on it. `--live` takes no other flag but `--trajectory-v2` (not `--input`, `--duration`, a report or a slope flag; a `--body` with wheels is required): the input is stdin and the run ends at end of input.

**Lines in.** The grammar and the rules are those of `--input` (see "The files" and `src/input.rs`), and the parser is the same code (`LineParser`), so a line is accepted or refused the same way in both: `t throttle brake steer`, only `\n` line ends (a `CR` is an error), blank lines skipped, the first line at t = 0, times strictly increasing, throttle and brake in 0..=1, steer in -1..=1. The line must end with `\n` to be read at once; the viewer writes one line per input change and flushes it. `t` is simulation time, never wall-clock time. A last line with no `\n` at end of input counts as a line, as in a file.

**Rows out.** Before anything is written the program waits for the first line (it must be at t = 0). It then writes, as one flushed write, the header `t,speed_mps,x,y,z,heading_deg` and the row at t = 0; after that one row per output step (`dt`), flushed after each row, in the format of the per-step rows of `--input` (`{:.4},{:.4},{:.4},{:.4},{:.4},{:.2}`, each row ending in `\n`; the code that formats a row is shared, so the bytes cannot differ). The viewer can read a row as soon as it is written: there is no other buffering. The rows are the car's chassis-centre speed, position and heading, not per-node frames; a viewer that needs nodes passes `--trajectory-v2` (next paragraph).

**Trajectory v2 (`--trajectory-v2`, task t-20261120-152915).** With `--live --trajectory-v2` (and with `--input <file> --trajectory-v2`) the output is a trajectory v2 file (`src/trajectory.rs`, the format `projects/render` replays) instead of the drive rows: the header (`trajectory v2`, `nodes,N`, one `beam,a,b` line per beam, one `wall,AXIS,OFFSET` line per collider plane of the body, then `t,node,x,y,z`), then one frame of every node (`t,node,x,y,z` rows, 4 decimals, nodes in order) at t = 0 and after every output step (`dt`), each frame preceded by the `break,t,beam` events since the previous frame. In `--live` the header and the t = 0 frame are one flushed write after the first line, and every later frame is one flushed write, under the same rule of never running ahead of the input. The wheels are not nodes of the body and v2 has no wheel lines, so they are absent from the file: the viewer draws the body only. A body whose collider plane is not along the x, y or z axis is refused (exit 2) before anything is written, as `trajectory::walls_from_planes` refuses it. There is no `final` line (it would not be valid v2): the run ends at end of input, after any break events that came after the last frame, and exits 0; every error, exit code and the closed-stdout rule below are unchanged. The same lines piped live and given as `--input` (no `--duration`) with the flag give identical bytes (test `live_trajectory_v2_matches_file_bytes`); without the flag the output is the drive rows and the `final` line, byte for byte as before (test `trajectory_v2_flag_replaces_rows_and_default_is_unchanged`). With `--input` the flag replaces the per-step rows and the `final` line of a plain drive and cannot be combined with `--drive-report`, `--settle-report`, `--crumple-report`, `--slope-grip-frac` or `--slope-report` (exit 2). The 3.6 run (`scenarios/pad_and_wall.bsn`, `scenarios/pad_lap.input`, `--duration 72`) writes a 7201-frame, 40-node file that `projects/render`'s replay loads.

**Never ahead of the input.** The sim runs a substep only when the last time received is at or after the end of that substep, so after a line at time `t` it runs up to `t` (every whole substep not after `t`, with the relative epsilon of 1e-9 that a file uses) and then blocks reading the next line. A line takes effect at the first substep that starts at or after its time, as in a file, so a line sent at `t` changes the run from `t` on, and the rows already written (all at times up to the previous line's `t`) are never taken back.

**End.** End of input (the viewer closes the child's stdin) ends the run at the last line, as a file script with no `--duration` does: the run is exactly the substeps up to the last line's time, the last row is the last output step not after it, and the program writes the line `final t=<4 decimals> x=<4 decimals> z=<4 decimals> heading_deg=<2 decimals>` (the format of the drive's `final` line, at the end of the last substep) and exits 0. If the input ends before one whole substep (only a line at t = 0), it is an error naming the cause, exit 2, after the header and the t = 0 row.

**Same bytes.** The same lines piped live and given as `--input` (no `--duration`) give identical stdout bytes: the header, the rows, the `final` line (test `live_matches_file_input_bytes`).

**Errors.** Exit codes are those of the car runs.
- A bad line (any rule above, or a value the car cannot obey) writes `crash-sim: --live: input line N: <reason>` to stderr and exits 2. `N` counts every line received, blank ones too, from 1. What was written before stays written (rows are never retracted); no `final` line follows. The same holds for end of input before any line, a body without `front` and `rear` groups, and a line that is not valid UTF-8.
- If the sim becomes unstable (a non-finite state, a strain limit), the program writes `crash-sim: unstable at t = <t> s: <reason>` to stderr, no `final` line, and exits 1; the rows before the failure stay written.
- A closed stdout (the viewer went away) ends the run quietly: exit 0, nothing on stderr. Any other write error prints `crash-sim: cannot write output: <error>` and exits 1.
- stdout carries protocol only: header, rows and the `final` line. Every message goes to stderr.

Tests (`tests/live.rs`, run through the real binary): `live_matches_file_input_bytes`, `live_rows_arrive_before_end_of_input` (rows up to the last time received arrive with stdin open, none after it until the next line), `live_bad_line_exits_2_naming_the_line`, `live_closed_stdout_ends_quietly`, `live_trajectory_v2_matches_file_bytes`, `live_trajectory_v2_frames_arrive_before_end_of_input`.

### The numbers, and why

- **Impact 45 to 55 km/h.** 50 km/h is the speed of the full-width rigid-barrier frontal tests (Euro NCAP full width, UN Regulation 137), and the wall is a rigid plane, so milestone 4 can compare this run against such a test. The +-5 km/h lets an open-loop script aim; if physics changes and the speed drifts out, the script is re-aimed and the change is recorded, never the window.
- **Lap: heading >= 360 degrees, path >= 50 m, back within 5 m.** 360 degrees and back near the start is a closed loop; 50 m of path (a circle of radius 8 m) rules out spinning on the spot; 5 m is about one car length.
- **No chassis node touches the ground before the wall.** A car on a flat pad at these speeds stays on its tyres; a touch means it rolled or bottomed out.
- **Crumple >= 0.05 m, <= 1.0 m, drift <= 0.001 m** between `rear` and `front`. 0.05 m is far above any elastic residue (milestone 2 put that at 1e-4 m scale) and well below what a real car crushes at 50 km/h; 1.0 m would reach the cabin. The measured crush is recorded for milestone 4, which validates it against published data; milestone 3 does not gate on realism.
- **t_last = 60 s, fixed.** The script must fit a lap and a run-up before it. A lap of at least 50 m at an average 5 m/s (18 km/h) takes about 10 to 15 s; the run-up from the lap's end to 50 km/h (13.9 m/s) at a modest 2 to 3 m/s^2 takes 5 to 7 s and 35 to 50 m, so the impact comes at about 20 to 30 s, and 60 s leaves at least twice that. The number is fixed so that 3.6 has no value to fill in later; if nb-41 cannot reach the wall before 60 s, it stops and reports, and the number is changed here first, never only in the script.
- **Settle after the crash: T = t_last + 8 = 68 s, S = 4 s (the run lasts t_last + 12 = 72 s), V = 0.001 m/s, E = 0.01 J, G = 0.01.** The settle clock starts at t_last, when the brake goes full on with the car already against the wall. A car body on its springs decays at zeta * omega; with the slowest values check 3.3 allows (1.0 Hz, zeta = 0.2) that is 1.26 /s, and from the 1.98 m/s of a 0.2 m drop to V takes ln(1.98 / 0.001) / 1.26 = 6.0 s, so 8 s has margin. E = 0.01 J is the energy to lift 1200 kg by about 1 micrometre. G is as milestone 2, with the engine's work subtracted (settle rule 4).
- **Ride frequency 1.0 to 2.0 Hz and damping ratio 0.2 to 0.5 per corner.** These are the usual passenger-car design ranges (for example Gillespie, *Fundamentals of Vehicle Dynamics*, SAE, on ride: about 1 to 1.5 Hz, damping ratio 0.2 to 0.4), widened a little. They are computed from the file's spring and damper rates and the measured corner load, so a car cannot be made to settle by making the suspension unrealistically stiff or overdamped.

### Checks

**3.1 Tests.** `broadsword test crash-sim > scratch/s6-3-1.txt 2>&1`
PASS iff `test: ok — N passed, 0 failed` and the file contains each of:
`test car_basic_round_trips ... ok` and one passing test per refusal listed in nb-38 (nb-38, `tests/format.rs`);
`test car_settles_level ... ok`, `test braked_car_holds_below_grip_slope ... ok`, `test braked_car_slides_above_grip_slope ... ok` (nb-39, `tests/vehicle.rs`; their rules are in 3.3 and 3.4);
`test same_input_same_bytes ... ok`, `test launch_matches_torque_or_grip_limit ... ok`, `test full_lock_circle_matches_kinematic_radius ... ok` (nb-40, `tests/drive.rs`; rules in 3.5);
`test pad_and_wall_car_is_car_basic ... ok`, `test pad_lap_closes_and_hits_wall ... ok` (nb-41, `tests/m3_done.rs`; the second runs the 3.6 run in-process and asserts the same rules);
and every milestone 1 and 2 test.
FAIL: any failed or missing test.

**3.2 The car settles after a 0.2 m drop.**
`broadsword run crash-sim -- cargo run --offline --release -- --body scenarios/car_basic.bsn --duration 12 --settle-report --settle-by 8 --settle-hold 4 --settle-speed 0.001 --energy-rise 0.01 --energy-gain 0.01 > scratch/s6-3-2.txt 2>&1`
PASS iff `run: ok` and the last line is `settle: ok`. FAIL: anything else.

**3.3 It rests level, as statics predicts** (`car_settles_level`, nb-39). After the 3.2 run, at its last substep:
(a) the four tyre normal loads sum to the total weight M g within 0.1%;
(b) on each axle, left and right loads agree within 1% and left and right suspension compressions within 0.001 m;
(c) each suspension compression is within 5% of (tyre load - wheel weight) / spring rate, and no suspension is on a travel stop;
(d) the front and rear axle loads are within 2% of the moment balance about the centre of mass, W_front = M g (x_rear - x_cm) / (x_rear - x_front), using the tyre contact points;
(e) each corner's ride frequency (1 / 2 pi) sqrt(k / m_c) is in [1.0, 2.0] Hz and its damping ratio c / (2 sqrt(k m_c)) in [0.2, 0.5], with m_c = (tyre load - wheel weight) / g.
PASS iff the test passes; FAIL if any of (a) to (e) fails.

**3.4 Grip holds and then lets go** (nb-39 the slope flag and report, nb-40 `full_brake.input`). Scenario: `scenarios/car_basic.bsn` itself, unchanged, braked by `scenarios/full_brake.input`.
Tilt setup: `--slope-grip-frac f` (nb-39) turns the flat ground into a slope by tilting gravity instead of the plane: gravity becomes g (sin theta, -cos theta, 0) with tan theta = f x mu, where mu is the body's tyre grip coefficient (refused, exit 2, if the body has no tyres or its tyres' mu differ). On a flat infinite plane this is the same physics as tilting the plane by theta. Downhill is +x, along the car's long axis (nb-38), so the braked wheels resist it. The car starts as in 3.2, 0.2 m above the ground.
`--slope-report` (nb-39; required flags `--slope-from <s>` and `--slope-to <s>`, no defaults) prints, after any other report lines:
```
slope_mu=<4 decimals>                the tyre grip coefficient mu
slope_tan=<6 decimals>               tan theta = f x mu
slope_accel_mps2=<4 decimals>        (u(to) - u(from)) / (to - from), u = the chassis centre-of-mass velocity along +x
slope_predicted_mps2=<4 decimals>    g (sin theta - mu cos theta)
```
(a) Holds: `broadsword run crash-sim -- cargo run --offline --release -- --body scenarios/car_basic.bsn --input scenarios/full_brake.input --duration 12 --slope-grip-frac 0.8 --settle-report --settle-by 8 --settle-hold 4 --settle-speed 0.001 --energy-rise 0.01 --energy-gain 0.01 > scratch/s6-3-4a.txt 2>&1`
PASS iff `run: ok` and the output contains `settle: ok` (with `--input` it also prints the `final ...` line, so `settle: ok` need not be the last line): max node speed < 0.001 m/s at every substep from 8 s to 12 s, with T, S, V, E and G as in 3.2 (the car drops and settles as there, on a slope it must hold). This is also the rule of `braked_car_holds_below_grip_slope`.
(b) Slides: `broadsword run crash-sim -- cargo run --offline --release -- --body scenarios/car_basic.bsn --input scenarios/full_brake.input --duration 4 --slope-grip-frac 1.2 --slope-report --slope-from 2 --slope-to 4 > scratch/s6-3-4b.txt 2>&1`
PASS iff `run: ok`, `slope_predicted_mps2` > 0, and |`slope_accel_mps2` - `slope_predicted_mps2`| <= 0.15 x `slope_predicted_mps2`. This is also the rule of `braked_car_slides_above_grip_slope`.
FAIL: in (a), `run: FAILED` or any `settle: FAIL` (creep at or above 0.001 m/s on the gentle slope); in (b), `run: FAILED`, a missing slope line, or an acceleration more than 15% from the friction formula (a slide that does not follow friction physics).

**3.5 Driving follows the formulas, not a script of positions** (nb-40).
`same_input_same_bytes`: the same input file run twice gives byte-identical stdout.
`launch_matches_torque_or_grip_limit`: from rest on flat ground at full throttle, the mean acceleration over the first 0.5 s is within 10% of F / M_eff, with F = min(total drive torque / tyre radius, F_grip) and M_eff = M + (sum of wheel inertias) / radius^2. F_grip is mu x the load on the driven wheels while the car accelerates at a = F_grip / M_eff, which is their static load plus the load transfer from the other axle, M a h / L (h the height of the centre of mass above the ground at the suspension's rest length, L the wheelbase): F_grip = mu N_static / (1 - mu M h / (L M_eff)). (Written first with the static load alone, which is wrong for a car that is already near its grip limit at rest: for `car_basic.bsn` it gives 4.11 m/s^2, but the rear tyres carry 1.1 kN more while it accelerates, so the torque limit binds and the answer is 5.00 m/s^2, which the sim reproduces to 0.2%. Found by nb-40 when the test, written from the static rule, failed by 21%.) The test also runs the car with every tyre at mu 0.5, where the grip limit binds (expected 2.68 m/s^2), so both sides of the min are exercised. The load transfer term is an approximation: it uses M a h and leaves out the wheels' spin-up inertia from the pitch moment (about I a (h - r) / r^2 per wheel), a small effect inside the 10% band. There is no top-speed check in milestone 3, by design: the car has no drag term (aero or rolling), so at full throttle it keeps accelerating until the tyres run out of grip; a top speed needs a drag model, which belongs to a later milestone (Stage 7 racing).
`full_lock_circle_matches_kinematic_radius`: at a steady 2 m/s with full steer, the path radius is within 10% of wheelbase / tan(max steer angle) (low-speed kinematic steering; lateral acceleration is under 0.1 g, so tyre slip is small), and over two laps it varies by at most 5%.
PASS iff `scratch/s6-3-1.txt` (the 3.1 run) contains `test same_input_same_bytes ... ok`, `test launch_matches_torque_or_grip_limit ... ok` and `test full_lock_circle_matches_kinematic_radius ... ok`, each asserting the rule above.
FAIL: any of the three missing or failed (a rule above is broken).

**3.6 Headless: a lap of the pad and into the wall.** Every number is fixed in "The numbers, and why" above (t_last = 60 s, so the run lasts 72 s and settles by 68 s; impact 45 to 55 km/h); nothing is filled in later.
`broadsword run crash-sim -- cargo run --offline --release -- --body scenarios/pad_and_wall.bsn --input scenarios/pad_lap.input --duration 72 --drive-report --lap-min-heading 360 --lap-min-path 50 --lap-close 5 --impact-min-kmh 45 --impact-max-kmh 55 --settle-report --settle-by 68 --settle-hold 4 --settle-speed 0.001 --energy-rise 0.01 --energy-gain 0.01 --crumple-report --crumple-groups rear,front --crumple-min 0.05 --crumple-max 1.0 --crumple-drift 0.001 > scratch/s6-3-6.txt 2>&1`
PASS iff `run: ok`, the last line of `scenarios/pad_lap.input` is `60 0 1 0`, the output contains `drive: ok` and `settle: ok`, `wall_first_contact_s` is a number below 60.0000, and the last line is `crumple: ok`. Record `wall_impact_speed_kmh` and `crumple_delta_m` in the evidence for milestone 4.
FAIL: `run: FAILED`; any `FAIL` line (an abort, which includes any NaN, is `settle: FAIL unstable ...`); a different last input line; or `wall_first_contact_s=none` or at or after 60.0000 (the settle window would not start after the crash).

**3.7 A human drives it in the viewer** (nb-40 render half). Tell Lazlo first (nb-37: the PC shares RAM with the GPU; the viewer is software-rendered, but ask before long sessions).
`broadsword run render -- cargo run --offline --release --example drive -- --body ../crash-sim/scenarios/pad_and_wall.bsn --save-input ../../scratch/s6-3-7.input > scratch/s6-3-7.txt 2>&1`
Lazlo drives with the arrow keys or WASD, around the pad and into the wall, then closes the window. The viewer saves the inputs it used (ending with a line at the moment the window closed) and prints `final t=... x=... z=... heading_deg=...` and `saved input: ../../scratch/s6-3-7.input`. The orchestrator copies the input to `evidence/stage-6-m3-human.input` before scratch can be wiped.
PASS iff Lazlo says the car responded to the keys and he hit the wall, and both lines are printed. FAIL: no window, no key response, or no saved input.

**3.8 The human drive replays headless.**
`broadsword run crash-sim -- cargo run --offline --release -- --body scenarios/pad_and_wall.bsn --input ../../evidence/stage-6-m3-human.input --drive-report --lap-min-heading 360 --lap-min-path 50 --lap-close 1000 --impact-min-kmh 1 --impact-max-kmh 200 > scratch/s6-3-8.txt 2>&1`
(A human's lap need not close within 5 m or hit at 50 km/h; the loop and the crash are what is checked.)
PASS iff `drive: ok` and the `final ...` line is identical, character for character, to the one the viewer printed in 3.7. FAIL: any difference (the crash is not replayable), or `drive: FAIL`.

**3.9 It can be seen.** `broadsword test render > scratch/s6-3-9.txt 2>&1`
PASS iff `test: ok — N passed, 0 failed` and the file contains `test crashed_car_final_frame_matches_golden ... ok` (the last frame of the 3.6 run). FAIL otherwise.

What guards against tuning to the test in milestone 3: the car that crashes is byte-identical to the car whose statics are checked (3.1); the resting loads must match statics (3.3) and the suspension must sit in real-car ranges (3.3 e); grip, launch and turning are checked against independent formulas (3.4, 3.5); the impact speed comes from a real test protocol and the script, not the physics, is what gets re-aimed; and the crash must replay from inputs alone (3.5, 3.8).

After 3.6 to 3.9 pass, the Stage 6 m1-m3 retro entry in `RETRO.md` is due (AGENTS.md); nb-41 lists its inputs in `evidence/stage-6-m3.md`.

## Existing tests and constants

**Must keep passing unchanged through milestones 1 to 3** (check 1.2): all 54 tests in `tests/csv.rs` (4), `determinism.rs` (8), `energy.rs` (3), `invariants.rs` (1), `physics.rs` (13), `risky.rs` (6), `stability.rs` (10), `summary.rs` (7) and `sweep.rs` (2), and the default CSV byte for byte (nb-31's fixture). They use `Sim::pos`, `vel`, `beams`, `cfg`, `first_contact_s`, `check()`, `max_speed()`, `advance()`; `Beam::a`, `b`, `rest`; `max_strain(&[Beam], &[Vec3])`; `run`, `run_checked`, `run_checked_with_contact`, `to_csv`, `CSV_HEADER`; `Config` with struct-update syntax; `max_stable_step`, `stable_step_limit`, `stable_step_limit_with_cause`, `damped_step_error`, `Binding`; and the constants below. All of these stay public with today's meaning for the cube (adding fields is fine).

**Constants:**

| Today | In Stage 6 | Brief |
|---|---|---|
| `NODE_MASS` | per-node mass on the state and in `.bsn` nodes; the constant stays as the cube's value | nb-31, nb-33 |
| `BEAM_STIFFNESS`, `BEAM_DAMPING` | per-beam, from the beam's material; the constants stay as the cube's material | nb-31, nb-33 |
| `Beam::rest` | already per beam; a plastic beam's rest length changes | nb-34 |
| (new) `yield_strain`, `break_strain` | per material | nb-34, nb-35 |
| `STIFFEST_MODE_EIGENVALUE` = 8 | true only for the 28-beam cube. Any other body's step limit must come from a bound valid for that body (for example the Gershgorin bound, 2 x the largest per-node sum of k over the lightest mass), including suspension springs and wheel masses | nb-31, nb-39 |
| `RESTITUTION`, `REST_SPEED`, `FRICTION_RATE` | stay global; applied along each collider plane's normal, the ground being one plane with today's exact behaviour | nb-36 |
| `MAX_STRAIN_LIMIT` | stays global; applies to the elastic strain of unbroken beams only | nb-34, nb-35 |
| `GRAVITY`, `TILT_DEG` | unchanged (config default; cube start pose only) | none |

## Who builds what

| Check | Brief(s) |
|---|---|
| 1.1 solver tests | nb-31 |
| 1.1 settle tests, 1.3, 1.4, 1.5 | nb-32 |
| 1.2, 2.2 old tests unchanged | every brief that touches crash-sim |
| 2.1 format tests, `cube.bsn`, `box.bsn` | nb-33 |
| 2.1 deform tests, `--body` | nb-34 |
| 2.1 breaking tests, 2.7, the breaking rule and `break` lines of 2.9 | nb-35 |
| 2.1 box_wall tests, 2.3 to 2.6, wall planes, crumple report, the five box_wall files, per-group `velocity`, the two break_pull files of 2.9 | nb-36 (with nb-33's box and nb-34's plasticity) |
| 2.9 run | nb-36 (after nb-35) |
| 2.8 | nb-37 |
| 3.1 car format tests, `car_basic.bsn` | nb-38 |
| 3.1 vehicle tests, 3.2, 3.3, 3.4, `--slope-grip-frac`, `--slope-report` | nb-39 (3.4's runs with nb-40's `full_brake.input`) |
| 3.1 drive tests, 3.5, drive report, `final` line, drive work W(t), `full_brake.input`, 3.7 viewer | nb-40 |
| 3.1 m3_done tests, 3.6, 3.8, 3.9, `pad_and_wall.bsn`, `pad_lap.input` (last line `60 0 1 0`), evidence | nb-41 (3.9 with nb-37's replay) |

## What the briefs must take from this file

Where a brief and this file differ, this file sets the check; the brief's coder raises the difference with the orchestrator.

- nb-31: `STIFFEST_MODE_EIGENVALUE` may only be used for the cube; the test names in 1.1.
- nb-32: an "impact" (not every resting contact) sets `last_impact_s`; the energy-rise window starts at T, not at the last impact (a car on its springs keeps the integrator's energy swing well after its last impact); the flags, output lines, formats and exit codes above; the grid test. The energy function and the impact detection must be reachable by nb-36 (walls) and nb-39 (wheels), which do not claim `settle.rs`.
- nb-33: box node mass 1 kg, long axis x, groups `back` and `front`.
- nb-35: the strain abort and `max_beam_strain` skip broken beams (otherwise the first break aborts the run); the `break t= beam= strain=` line format; the 2.9 thresholds (break when |strain| exceeds `break_strain`, checked every substep).
- nb-36: the four variant files and their one-number test; plane impacts in the settle report; the crumple flags and output; a `velocity` that can be set per group; `break_pull.bsn`, `break_pull_slow.bsn` and their speed-only test (2.9). Its claims need `scenarios/break_pull.bsn` and `scenarios/break_pull_slow.bsn` added.
- nb-38: groups `front` and `rear`, left-right symmetry, tyres 0.20 m above the ground at start, long axis (the rolling direction) along x.
- nb-39: the energy terms for wheels, springs and tyres, and the tilted-gravity potential; test names and rules in 3.3 and 3.4; `--slope-grip-frac`, `--slope-report` and its flags and lines.
- nb-40: `--drive-report`, the `final` line, W(t), the end of a run at t_last without `--duration` and the last line holding after it, `--save-input`; `scenarios/full_brake.input` (its claims need it added).
- nb-41: `pad_and_wall.bsn` embeds `car_basic.bsn`'s car byte for byte; the script's first line and its last line `60 0 1 0`, with the wall hit before 60 s; the render golden test of 3.9 and `evidence/stage-6-m3-human.input`.
