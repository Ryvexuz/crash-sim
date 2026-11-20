# Stage 7 done checks: milestone 1, with drafts for milestones 2 to 5

Written before any Stage 7 code, on crash-sim `5415a5b` (board task `t-20261119-171333-43`, brief nb-42). DEVELOPMENT_PLAN Stage 7:

1. Racing basics: tire grip, a track, lap timing. *Done:* a timed lap you can drive.
2. AI drivers. *Done:* three AI cars finish a race without crashing into walls.
3. Game shell: menus, settings, saving. *Done:* a standalone executable a friend can run.
4. Multiplayer. *Done:* two machines race each other.
5. Second game: a small game in another genre on the same toolkit. *Done:* it ships without changes to the engine core.

Lesson focus: whole-product integration, where many parts must work together and "tests pass" stops being enough.

This file turns milestone 1 into commands with numbers. Milestone 1 is done when every one of its checks passes on crash-sim main and render master, the output is saved in `evidence/`, and the human check (1.8) is recorded. Milestones 2 to 5 get one paragraph each, marked as drafts. The table at the end maps every check to the brief that makes it pass: nb-43 and nb-44 (on the board as `t-20261119-171333-44` and `t-20261119-171333-45`) and three new briefs proposed here, nb-47, nb-48 and nb-49.

## Where the game lives

Inside crash-sim: a library module and a second binary, not a separate `projects/racing` repo. This is ADR 0001 option (a), accepted by Lazlo on 2026-11-19 (`docs/adr/0001-shared-core.md` in the Broadsword repo, follow-up F3), so it is not a new decision for him.

Reason: DEVELOPMENT_PLAN says "the crash sim grows into a racing game", and the racing game needs the crash sim's body, car, solver and input code as a library. A separate repo would need a relative path dependency on crash-sim, and ADR 0001 measured that no relative `path =` string works both in a main checkout (`projects/racing`, two folders below the root) and in a task worktree (`worktrees/racing/<task id>`, three below). Every task is built and checked in a worktree, so that layout fails in exactly the place work happens. The game engine of roadmap 4.1 can still become its own crate later through ADR 0001's (d2) route, one workload repo with a Cargo workspace, if a trigger in that ADR fires.

The layout in milestone 1:

| Part | Where |
|---|---|
| Track format, gates, `on_track`, `LapTimer` | crash-sim `src/track.rs` (nb-43) |
| The race: car on the grid, sim plus lap timer, race lines | crash-sim `src/race.rs` (nb-47) |
| The race program | crash-sim `src/bin/race.rs`, run as `cargo run --bin race` (nb-47) |
| Track files | crash-sim `scenarios/tracks/` (nb-43, nb-47) |
| Skidpad measurement | crash-sim `tests/skidpad.rs` (nb-44) |
| Lap timer on screen, track drawn in the viewer | render `src/hud.rs` and `examples/drive.rs`, with the race program as a child process (nb-48, after nb-40's F1 viewer) |

A second binary changes how `cargo run` works: with two binaries and no `default-run`, Cargo refuses `cargo run` with "could not determine which binary to run", and every STAGE6.md command uses plain `cargo run`. So crash-sim's `Cargo.toml` must say `default-run = "crash-sim"` in `[package]` in the same commit that adds `src/bin/race.rs` (nb-47). Check 1.2 runs STAGE6.md commands unchanged to prove it.

## How to run and read a check

As in STAGE6.md:

- Run every command from the Broadsword root. `broadsword` means `build/bin/broadsword.exe`. Inside a task, add `--task <id>` after the project name.
- Never call cargo directly. The race program runs as `broadsword run crash-sim -- cargo run --offline --release --bin race -- <flags>`. The program's working folder is the project folder, so `scenarios/...` paths are relative to `projects/crash-sim`.
- Redirect each command to a file in `scratch/` and read the file. Never pipe a broadsword command into another program. Reading a saved file with `grep`, `diff` or `cmp` afterwards is fine.
- `broadsword run` prints its own job lines first, then the program's lines, then `--- summary ---` and `run: ok (...)` or `run: FAILED (exit N)`. "The output" below means the program's lines.
- Each check says what PASS is. Anything else is FAIL, including a check that cannot be run because a flag, file or test is missing.
- Nothing has been measured yet: no Stage 7 code exists. Every number below is *reasoned* (derived here, with the working shown), *sourced* (from a public source that was read, with its URL and the date it was read), or *unverified, from memory* (recalled, not read; such a number cannot let a check pass until it is confirmed and relabelled *sourced*). Where a brief must report a measured value against a reasoned bound, it says so. If the measured value is outside the bound, the brief stops and reports; it never widens the bound.

## Shared definitions

- **Ground plane coordinates:** the track lies on the ground, y = 0. A track point is (x, z). Heading and the `final` line are as in STAGE6.md (nb-40).
- **Car position:** the chassis centre of mass, which is the mass-weighted mean of the body's nodes (wheels excluded), projected onto the ground plane. Laps, `on_track`, path length and speed all use this one point.
- **Substep:** as STAGE6.md, h = dt / substeps, 1 ms by default. Gate crossings and `on_track` are checked at every substep, never only at output steps (audit S14: output sampling missed contacts).
- **Direction of travel:** increasing centreline index, wrapping from the last point to point 0.
- **Tangent at point i:** the unit vector along p[i+1] - p[i-1] (indices wrap). The start gate's tangent is the one at the start index; for `oval.track` that is p[1] - p[63] = (100, 0), so +x. That tangent is also the direction the car faces on the grid (`car_on_grid_faces_the_track`).
- **Gate at point i:** the segment through p[i], perpendicular to the tangent at i, of total length equal to the track width at i. The start/finish line is the gate at the start index, and each checkpoint is the gate at its index.
- **Crossing:** within one substep the car moves in a straight line from its old position P0 (time t0) to its new position P1 (time t1). It crosses a gate when the segment P0 to P1 meets the gate segment at a fraction s with 0 < s <= 1 along P0 to P1. The half-open interval means a car that stops exactly on the line crosses it once, not twice. The crossing time is t0 + s (t1 - t0), interpolated within the substep and never rounded to it. A crossing is forward when (P1 - P0) . tangent > 0, otherwise backward.
- **On track:** the distance from the car position to the centreline polyline is at most half the track width there. On a segment with different widths at its two ends, the width at the nearest point is interpolated linearly between them.
- **The lap rule** is enforced in one place, nb-43's `LapTimer`, and nowhere else. The race program and the viewer only report what it decides.
  1. Before the first forward crossing of the start/finish line, nothing is timed (the out-lap from the grid).
  2. That first forward crossing starts lap 1 at its crossing time.
  3. While a lap runs, the checkpoints must be crossed forward in their listed order. A forward crossing of the next expected checkpoint advances to the one after it. Any other crossing, whether of a checkpoint out of order or of any gate backward, changes nothing.
  4. A forward crossing of the start/finish line ends the running lap and starts the next one at the same crossing time. The lap that ended is **valid** iff every checkpoint was crossed in order and the car was on track at every substep of that lap. Otherwise it is **invalid**, with reason `missed_checkpoint` (checked first) or `off_track`.
  5. A backward crossing of the start/finish line never ends a lap, never starts one, and never counts.
- **Lap time:** the integer number of milliseconds `round((t_end - t_start) x 1000)`. It is printed as that integer divided by 1000 with exactly 3 decimals, computed from the integer (for example `format!("{}.{:03}", ms / 1000, ms % 1000)`), never through a float format. The HUD shows the same integer as `M:SS.mmm` (minutes, then seconds with two digits, then milliseconds). Because both come from one integer, the printed time and the on-screen time can never round differently.

## Milestone 1: a timed lap you can drive

### The track file (nb-43)

The grammar is nb-43's (in `docs/FORMAT.md`'s track section, or in `docs/TRACK.md` if nb-43 takes that route). It is strict like `.bsn`: line numbers in every refusal, nothing defaulted, LF line endings only. It must hold:

- a version line `track 1`;
- the centreline points (x, z) in driving order, a closed loop of at least 3 points that does not cross itself;
- one track width for every point, with an optional per-point width that overrides it for that point;
- the start/finish index;
- the checkpoint indices, at least one, in driving order counting cyclically from the start index, none equal to the start index;
- `grid <m>`: the car starts with its position this far before the start/finish line, measured backwards along the centreline from the start point.

`scenarios/tracks/oval.track`, exactly:

- 64 points. Point 0 is (0, 0) and point 1 is (50, 0) (the front straight, driven in +x). Points 2 to 30 are (50 + 30 sin p, 30 - 30 cos p) for p = 6, 12, ..., 174 degrees (the first bend, centre (50, 30), radius 30 m). Point 31 is (50, 60), point 32 is (0, 60) and point 33 is (-50, 60) (the back straight). Points 34 to 62 are (-50 - 30 sin p, 30 + 30 cos p) for the same p (the second bend, centre (-50, 30)). Point 63 is (-50, 0). Both bends turn right, toward +z.
- Width 12 m everywhere. Start/finish index 0. Checkpoints 16 (the apex of bend 1, (80, 30)), 32 (the middle of the back straight) and 48 (the apex of bend 2, (-80, 30)). `grid 20`, so the car starts at (-20, 0) facing +x.
- Its centreline polygon is 388.4094 m long (two 100 m straights plus 60 chords of 2 x 30 x sin 3 degrees = 3.14016 m each). The race program prints this as `centre_length_m=388.41`.

`scenarios/tracks/oval_pinch.track` (nb-47) is `oval.track` with one change: points 5 to 10 (on bend 1, p = 24 to 54 degrees, about 16 m of bend) have a per-point width of 0.2 m. The gates at 0, 16, 32 and 48 keep 12 m. A test checks that nothing else differs.

`scenarios/tracks/chicane.track` (nb-43): any closed track with a chicane, for the parser and the lap timer tests. No run in milestone 1 uses it.

Why these numbers:

- **Width 12 m.** FIA rules for new permanent circuits ask for a track width of at least 12 m (FIA International Sporting Code, Appendix O; from memory, not checked online in this task). It is also three car widths for a 1.8 m wide car, enough room for an open-loop script to stay on track.
- **Bend radius 30 m, straights 100 m.** At a lateral limit of 0.75 to 0.95 g (check 1.3), the bend speed sqrt(a R) is 14.9 to 16.7 m/s (54 to 60 km/h), close to the 50 km/h of STAGE6's crash. A lap is short (about 390 m), so an out-lap plus one timed lap is well under a minute of simulated time.
- **Arc points every 6 degrees.** Each chord is then 0.05% shorter than its arc (sin 3 degrees / 3 degrees in radians = 0.99954). The path floor below is computed for exactly these points.

### The race program (nb-47)

`race --body <file> --track <file> --input <file> [--substeps <n>] [--duration <s>]` runs headless. The car from `--body` is turned about the vertical axis so its long axis (+x in the file, nb-38) points along the tangent at the start point, and moved so its position is `grid` metres before the start line; its height is unchanged, so it drops 0.2 m as in STAGE6.md 3.2. `--input` is nb-40's input script, and the run ends at its last line unless `--duration` is given (nb-40's rule). The track has no physical effect in milestone 1: off the track the ground is the same plane with the same grip. Leaving the track only makes a lap invalid.

`race --live --body <file> --track <file>` is the same run as a child process of the viewer. It uses nb-40's child-process protocol unchanged (input lines in on stdin, stamped in simulation time; trajectory v2 rows out), and adds two things: before the first trajectory row it prints one `track_line <kind> <x0> <z0> <x1> <z1>` line per segment to draw (kind `left` and `right` for the two edges, `start`, and `checkpoint`), and it prints each race line below as soon as it happens. The viewer never parses a track file itself; the only track parser is nb-43's.

Output (both modes), one line per event, in time order:

```
track name=<file stem> points=<n> checkpoints=<n> centre_length_m=<2 decimals> width_min_m=<2 decimals> width_max_m=<2 decimals>
start t=<4 decimals>                                    the first forward crossing of the start line
checkpoint lap=<n> k=<1..N> t=<4 decimals>              only crossings that advance the lap (rule 3)
lap n=<n> start_t=<4 decimals> end_t=<4 decimals> time_s=<3 decimals> valid=<yes|no> reason=<none|missed_checkpoint|off_track> path_m=<2 decimals> max_speed_kmh=<1 decimal> off_track_s=<3 decimals>
final t=<4 decimals> x=<4 decimals> z=<4 decimals> heading_deg=<2 decimals>
race: laps_valid=<count> laps_invalid=<count> best_lap_s=<3 decimals, or none>
```

`n` counts every completed lap, valid or not. `start_t` and `end_t` are the lap's two start-line crossing times, and lap n + 1's `start_t` is lap n's `end_t`. `path_m` is the length of the car position's path from the lap's start crossing to its end crossing, with the first and last substeps counted by their interpolated fraction. `max_speed_kmh` is the largest horizontal speed of the car position at any substep of the lap. `off_track_s` is the total time off track during the lap, summed over substeps. `best_lap_s` is the shortest valid lap. The `final` line is nb-40's, unchanged. Exit code: 0 when the run reaches its end, even with no valid lap; 1 on an abort (`Sim::check()` fails), with the last line `race: FAIL unstable at t=<4 decimals>: <reason>`; 2 on a usage error or a refused file, with the file's line number in the message and empty stdout.

### The skidpad (nb-44)

`scenarios/skidpad.bsn` is the car section of `scenarios/car_basic.bsn`, byte for byte, on flat ground with no walls. `scenarios/skidpad.input` holds one constant steer value whose low-speed kinematic radius, wheelbase / tan(steer x max steer angle), is 30 m within 1%. Its throttle is a series of K rising levels, each held for at least 20 s, starting from rest, with no brake. At each level the car settles onto a steady circle, at a speed where the drive force balances the cornering drag, and the higher levels take it past its grip limit. This is the constant-steer-angle test at discrete speeds (SAE J266). The levels must also take the x1.2 grip copy below past its limit.

`tests/skidpad.rs` runs it in-process. It measures from the motion of the car position only, never from the tyre model's own forces, so the measurement does not depend on the code being tuned. `limit_g` is a steady-state value, measured only over intervals in which the car is really on a steady circle, so a transient (a yaw overshoot, a suspension bounce, the jump between two throttle levels) cannot produce it:

- Every substep: v, the car position's horizontal speed, and psi, the direction of its horizontal velocity (unwrapped). These are averaged over consecutive 0.1 s blocks, giving v_b and the yaw rate of the path, r_b = (psi at the block's end - psi at its start) / 0.1 s, and the path radius rho_b = v_b / r_b.
- **The steady window of a level** is the last 5 s of the time that level is held (50 blocks). The level is **steady** iff, over those 50 blocks, max - min is at most 2% of the mean for each of v_b, r_b and rho_b, and the body slip angle (between the chassis heading, from the `rear` to the `front` centroid, and the velocity direction) stays below 10 degrees.
- A steady level's lateral acceleration is a_g = mean(v_b) x mean(r_b) / g, with g the sim's gravity (steady circular motion: a = v r = v^2 / rho).
- `limit_g` is the largest a_g over the steady levels, and `limit_level` is that level's number (1 to K).
- `limit_balance`: `oversteer` if, at any level after `limit_level`, the body slip angle exceeds 10 degrees at any substep (the car spins); otherwise `understeer` (it runs wide onto a bigger circle).
- The same run is repeated on two in-memory copies of the body with every tyre's grip coefficient mu multiplied by 0.8 and by 1.2. There are no extra files, so the copies cannot be tuned separately.

The test prints, in this order (visible with `--nocapture`), one `level` line per throttle level of each run:

```
skidpad mu=<4 decimals> steer_frac=<4 decimals> radius_kinematic_m=<2 decimals> levels=<K>
skidpad grip_scale=1.0 level=<k> throttle=<4 decimals> steady=<yes|no> speed_kmh=<1 decimal> radius_m=<2 decimals> a_g=<3 decimals>
skidpad grip_scale=1.0 limit_g=<3 decimals> limit_level=<k> steady_levels=<count> limit_balance=<understeer|oversteer>
skidpad grip_scale=0.8 level=<k> ...                  (the same level lines for the x0.8 copy)
skidpad grip_scale=0.8 limit_g=<3 decimals> limit_level=<k> steady_levels=<count> limit_balance=<understeer|oversteer>
skidpad grip_scale=1.2 level=<k> ...                  (the same level lines for the x1.2 copy)
skidpad grip_scale=1.2 limit_g=<3 decimals> limit_level=<k> steady_levels=<count> limit_balance=<understeer|oversteer>
```

`speed_kmh`, `radius_m` and `a_g` are the means over the level's steady window, printed even when the level is not steady.

### The numbers, and why

- **Target `limit_g` 0.75 to 0.95 g** (*unverified, from memory*). `car_basic.bsn` is a 1200 kg road car on road tyres, not a sports car. Both figures behind the range are recalled, not read: road-test skidpad figures for ordinary passenger cars on all-season tyres of about 0.78 to 0.90 g (as recalled from Car and Driver and Motor Trend skidpad results on 300 ft and 200 ft circles), and peak tyre-road friction on dry asphalt of about 0.8 to 0.9 (as recalled from Gillespie, *Fundamentals of Vehicle Dynamics*, SAE, 1992, and Wong, *Theory of Ground Vehicles*). No page, table or URL was read for either in this task, so neither is a citation yet. The band is the recalled skidpad figures widened by 0.03 g at the bottom (economy cars) and 0.05 g at the top, stopping below the 1.0 g that summer performance tyres reach.
  **Confirmation step, before nb-44 commits any change to a tuning file of 1.3 (e).** The orchestrator, or a task it names, reads at least one public source that states a skidpad or lateral-limit figure for ordinary passenger cars on road tyres, and records it in this bullet as one line per figure: `confirmed: <figure> | <source title> | <URL> | read <YYYY-MM-DD> UTC`. The label of this bullet then changes to *sourced*. If the confirmed figures do not support 0.75 to 0.95 g, the range is changed in this file in the same commit, before any tuning commit (1.3 (e) fails otherwise). Any figure that is not confirmed this way stays in the bullet marked *unverified, from memory*, and is not used to set the range. Check 1.3 (g) gates this. The step is best done before nb-39 and nb-40 merge: if the range then has to change after their tyre commits are on main, those commits fall between R1 and the new RL, and 1.3 (e) fails for them. That failure is reported to Lazlo as a finding; the check is not loosened to pass it.
- **Ceiling: `limit_g` <= 1.02 x mu** (*reasoned*). On flat ground with no downforce, the sum of the tyres' horizontal forces is at most mu times the sum of their normal loads, which is mu M g, so a_lat <= mu g. The 2% matches the steadiness tolerance (a steady window may still vary by 2%). A result above this ceiling means the measurement, or the physics, is creating grip.
- **Grip variants: 0.72 <= limit(0.8) / limit(1.0) <= 0.88 and 1.08 <= limit(1.2) / limit(1.0) <= 1.32, strictly ordered** (*reasoned*). With a friction-circle tyre whose mu does not depend on load, the limit is proportional to mu, so the ratios should be 0.8 and 1.2. The 10% allows for load-sensitive tyre terms, if nb-39 has any. A result that does not move with grip was tuned or is noise (the Stage 6 lesson).
- **Radius 30 m** (nb-44's example). Without downforce the limit in g does not depend on the radius, and a small radius keeps the limit speed near 16 m/s, so few throttle levels are needed.
- **Steady means 5 s within 2% in speed, yaw rate and radius, measured on 0.1 s blocks** (*reasoned*). On a steady circle, v, r and rho are constant, so a window where all three hold still is cornering at that acceleration, not passing through it. 5 s is at least five periods of the slowest ride mode STAGE6 allows (1.0 Hz), and longer than a yaw transient. 2% of 0.85 g is 0.017 g, a tenth of the 0.20 g target band. The 0.1 s blocks average out the integrator's per-substep noise but not body motions slower than 10 Hz, so a car that still bounces or yaws in the window fails the 2% rule rather than being smoothed into a pass. Each level is held at least 20 s, so the 15 s before its window lets the speed settle after the throttle step (STAGE6 3.3's slowest suspension decays by e^-19 in that time). If a level does not settle in 15 s, it is reported as not steady; nb-44 may hold the levels longer, but never loosens the window or the 2%.
- **`limit_level` <= K - 2, and `steady_levels` >= 5, for each of the three runs** (*reasoned*). Five steady levels show the curve rising to the limit. Two more levels after the limit level show that more drive did not give more lateral acceleration on a steady circle, so `limit_g` is the limit and not just the last level the script reached.
- **`limit_balance` is recorded, not gated.** Road cars are normally set up to understeer at the limit (*unverified, from memory*: as recalled from Gillespie and from Milliken and Milliken, *Race Car Vehicle Dynamics*; not gated, so it is not confirmed by 1.3 (g)). An `oversteer` result is reported to the orchestrator as a finding; it is not tuned away inside nb-44.

Lap-check numbers (oval):

- **Path floor 338.67 m** (*reasoned*). The track band of `oval.track` (all points within 6 m of the centreline polygon) has an inner edge that is the inward offset of a convex polygon by d = 6 m. Its perimeter is P - 2 d x (sum of tan(theta_i / 2) over the exterior angles theta_i) = 388.4094 - 12 x 3.144395 = 388.4094 - 37.7327 = 350.6767 m (exterior angles: four of 3 degrees where straight meets bend, 58 of 6 degrees, so the sum is 4 tan(1.5 degrees) + 58 tan(3 degrees) = 0.104744 + 3.039651 = 3.144395). A valid lap stays in the band from one point on the start gate to another; closing it along the gate (at most 12 m) gives a loop that encloses the convex infield, so it is at least as long as the infield's perimeter. A valid lap's path is therefore at least 350.6767 - 12 = 338.6767 m (computed to 4 decimals from the 64 points above). The floor is that bound less 0.0067 m, and nothing more. The margin covers two things. First, `path_m` is the polyline through the substep positions, whose 1 ms chords can dip inside the band's inner edge by their sagitta. For a 17 mm chord (17 m/s) on the 24 m inner radius that is c^2 / (8 R) = 1.5e-6 m, which lowers the bound by at most 2 pi x 1.5e-6 = 1e-5 m. Second, the printed value is rounded to 2 decimals, and a true value of at least 338.6767 never prints below 338.68. So the floor allows no corner cut at all: every on-track lap is at least this long, and a counted lap that is shorter left the band or was counted without going round. The bound is for `oval.track` exactly as specified here; if the file changes, the bound is recomputed here first.
- **Timing consistency: `time_s` x `max_speed_kmh` / 3.6 >= `path_m` - 2, and |`time_s` - (`end_t` - `start_t`)| <= 0.0006** (*reasoned*). The mean speed over a lap cannot exceed its maximum, so time x maximum speed >= distance. The 2 m covers the rounding of the printed values: `max_speed_kmh` is rounded to 0.05 km/h (0.014 m/s), which over a lap of up to 100 s is at most 1.4 m; the rounding of `time_s` and `path_m` adds under 0.02 m. The second rule ties the rounded lap time to the two printed crossing times (each rounded to 0.00005 s, plus 0.0005 s for the whole millisecond). Neither rule uses `LapTimer`'s own arithmetic: a timer that stops early, or ends a lap at the wrong gate, fails them.
- **Pinch 0.2 m wide over about 16 m of bend** (*reasoned*). An open-loop script cannot keep the car position within 0.1 m of the centreline through 30 degrees of bend. If nb-47's lap does, nb-47 reports it, and the pinch is narrowed here first.
- **Step-size agreement within 1% of the lap time** (*reasoned*, not measured). nb-47 reports the measured difference; if it is above 1%, nb-47 stops and reports instead of raising the bound.
- **The lap time itself is recorded, not gated.** It depends on the script. Milestone 2's AI drivers are compared with it.

### Checks

**1.1 Tests.** `broadsword test crash-sim > scratch/s7-1-1.txt 2>&1`
PASS iff the last line is `test: ok — N passed, 0 failed`, N is at least the N recorded for STAGE6 check 3.1 in `evidence/stage-6-m3.md` plus the 22 tests named here, and the file contains each of:
`Running tests\track.rs`, `test oval_track_parses_as_specified ... ok` (64 points with the coordinates above to 1e-9 m, start 0, checkpoints 16, 32, 48, width 12, grid 20), `test refuses_fewer_than_three_points ... ok`, `test refuses_self_intersecting_centreline ... ok`, `test refuses_checkpoint_out_of_range ... ok`, `test refuses_checkpoints_out_of_order ... ok`, `test track_files_have_no_carriage_return ... ok`, `test crossing_time_is_exact_for_a_straight_pass ... ok` (from x = -0.3 at t = 0 to x = +0.7 at t = 0.001 across a gate at x = 0, the crossing time is 0.0003 within 1e-12 s), `test endpoint_on_the_line_counts_once ... ok`, `test lap_with_a_skipped_checkpoint_is_not_counted ... ok`, `test reversing_over_the_line_never_counts ... ok` (forward over the line, back over it, forward again: no valid lap), `test driving_the_track_backwards_never_counts ... ok` (a whole loop against the direction of travel: no `start`, no lap), `test lap_off_track_is_invalid ... ok`, `test on_track_uses_half_width ... ok` (nb-43, `tests/track.rs`; the refusal tests check the line number in the message, as `tests/format.rs` does);
`Running tests\skidpad.rs`, `test skidpad_car_is_car_basic ... ok`, `test skidpad_input_holds_a_30_m_kinematic_circle ... ok` (one steer value for the whole file, with a kinematic radius within 1% of 30 m; throttle levels rising, each held at least 20 s; brake 0 throughout), `test skidpad_limit_in_road_tyre_range ... ok`, `test skidpad_limit_follows_grip ... ok` (nb-44; the last two assert the rules of 1.3);
`Running tests\race.rs`, `test race_same_input_same_bytes ... ok`, `test car_on_grid_faces_the_track ... ok` (after placement the chassis heading is the start tangent within 0.01 degree and the car position is 20 m before the line within 1e-6 m), `test oval_pinch_differs_only_in_width ... ok`, `test oval_lap_is_valid_and_above_floors ... ok` (runs the 1.4 run in-process and asserts its rules), `test live_and_file_input_give_same_race_lines ... ok` (the input of 1.4 fed through `--live` gives the same `track`, `start`, `checkpoint`, `lap` and `final` lines as the file run) (nb-47);
and the 26 crash-sim Stage 6 tests that check 1.2 lists by name.
FAIL: any failed or missing test, or a smaller N.

**1.2 Stage 6 still works on the Stage 7 code.** Whole-product integration: the second binary, the tyre tuning and the new modules must not break what Stage 6 delivered. Run every part on the commit being checked, from the Broadsword root. The commands and numbers below are copied from STAGE6.md as it stands at crash-sim `26d0bea`. If STAGE6.md changes one of them, this file is changed in the same commit, so the two never disagree silently.

(a) The old tests are unchanged: `git -C projects/crash-sim diff --stat abe471c -- tests/csv.rs tests/determinism.rs tests/energy.rs tests/invariants.rs tests/physics.rs tests/risky.rs tests/stability.rs tests/summary.rs tests/sweep.rs > scratch/s7-1-2a.txt 2>&1`
PASS iff `scratch/s7-1-2a.txt` is empty.

(b) Every Stage 6 crash-sim test still exists and passes, by name, in the 1.1 run. In Git Bash:
```
for n in a_second_solver_plugs_in beam_breaks_at_expected_substep beam_past_yield_keeps_longer_rest box_wall_variants_differ_in_one_number braked_car_holds_below_grip_slope braked_car_slides_above_grip_slope break_pull_files_differ_only_in_speed broken_beam_does_not_trip_strain_limit canonical_file_round_trips_byte_identical car_basic_round_trips car_settles_level cube_bsn_matches_default_cube cube_bsn_output_matches_default default_csv_matches_main_fixture default_cube_output_unchanged disconnected_node_falls_freely energy_never_rises_through_yield full_lock_circle_matches_kinematic_radius ground_plane_collider_keeps_cube_output launch_matches_torque_or_grip_limit pad_and_wall_car_is_car_basic pad_lap_closes_and_hits_wall planted_energy_injection_fails_settle same_input_same_bytes settle_holds_across_the_invariants_grid two_node_energy_matches_hand_value; do grep -qF "test $n ... ok" scratch/s7-1-1.txt && echo "ok $n" || echo "MISSING $n"; done > scratch/s7-1-2b.txt 2>&1
```
These are the 26 crash-sim test names in STAGE6.md checks 1.1, 2.1 and 3.1. The two render tests named there are checked by 1.7's render run: `box_wall_final_frame_matches_golden` and `crashed_car_final_frame_matches_golden`.
PASS iff `scratch/s7-1-2b.txt` has exactly 26 lines, all starting `ok `. Also, `scratch/s7-1-7.txt` (1.7) contains `test box_wall_final_frame_matches_golden ... ok` and `test crashed_car_final_frame_matches_golden ... ok`.

(c) The default cube and `cargo run` without `--bin`: `broadsword run crash-sim -- cargo run --offline --release -- --summary > scratch/s7-1-2c.txt 2>&1`
PASS iff `run: ok` and the program's output is exactly the one line `first_contact_s=0.6390,peak_strain=0.045146,final_center_y=0.4999`. This also proves the `default-run` line.

(d) The cube settles (STAGE6 1.3): `broadsword run crash-sim -- cargo run --offline --release -- --duration 10 --settle-report --settle-by 6 --settle-hold 4 --settle-speed 0.001 --energy-rise 0.001 --energy-gain 0.00001 > scratch/s7-1-2d.txt 2>&1`
PASS iff `run: ok`, the output's last line is `settle: ok`, `peak_strain` <= 0.100000, and 0.4998692682 <= `final_center_y` <= 0.4998718568.

(e) The car settles after its drop (STAGE6 3.2): `broadsword run crash-sim -- cargo run --offline --release -- --body scenarios/car_basic.bsn --duration 12 --settle-report --settle-by 8 --settle-hold 4 --settle-speed 0.001 --energy-rise 0.01 --energy-gain 0.01 > scratch/s7-1-2e.txt 2>&1`
PASS iff `run: ok` and the last line is `settle: ok`.

(f) Grip holds on the gentle slope (STAGE6 3.4 a): `broadsword run crash-sim -- cargo run --offline --release -- --body scenarios/car_basic.bsn --input scenarios/full_brake.input --duration 12 --slope-grip-frac 0.8 --settle-report --settle-by 8 --settle-hold 4 --settle-speed 0.001 --energy-rise 0.01 --energy-gain 0.01 > scratch/s7-1-2f.txt 2>&1`
PASS iff `run: ok` and the output contains `settle: ok`.

(g) The car slides on the steep slope as friction predicts (STAGE6 3.4 b): `broadsword run crash-sim -- cargo run --offline --release -- --body scenarios/car_basic.bsn --input scenarios/full_brake.input --duration 4 --slope-grip-frac 1.2 --slope-report --slope-from 2 --slope-to 4 > scratch/s7-1-2g.txt 2>&1`
PASS iff `run: ok`, `slope_predicted_mps2` > 0, and |`slope_accel_mps2` - `slope_predicted_mps2`| <= 0.15 x `slope_predicted_mps2`. If nb-44 changed mu, `slope_mu` shows the new value and the prediction follows it.

(h) The pad lap and the wall crash (STAGE6 3.6): `broadsword run crash-sim -- cargo run --offline --release -- --body scenarios/pad_and_wall.bsn --input scenarios/pad_lap.input --duration 72 --drive-report --lap-min-heading 360 --lap-min-path 50 --lap-close 5 --impact-min-kmh 45 --impact-max-kmh 55 --settle-report --settle-by 68 --settle-hold 4 --settle-speed 0.001 --energy-rise 0.01 --energy-gain 0.01 --crumple-report --crumple-groups rear,front --crumple-min 0.05 --crumple-max 1.0 --crumple-drift 0.001 > scratch/s7-1-2h.txt 2>&1`, and `tail -n 1 projects/crash-sim/scenarios/pad_lap.input > scratch/s7-1-2h-last.txt`
PASS iff `run: ok`; the output contains `drive: ok` and `settle: ok`; `wall_first_contact_s` is a number below 60.0000; `wall_impact_speed_kmh` is in [45.0, 55.0]; the output's last line is `crumple: ok`; and `scratch/s7-1-2h-last.txt` is exactly `60 0 1 0`. Record `wall_impact_speed_kmh` and `crumple_delta_m` next to the Stage 6 values in `evidence/stage-6-m3.md`. A change is recorded, not gated, as long as it stays inside the windows.

STAGE6.md checks 3.7 and 3.8 are not re-run: a human drive was recorded on the tyres of its day, and its evidence stays as recorded. If nb-44 changed the tyres, (h)'s script may need re-aiming, which STAGE6.md allows ("the script is re-aimed and the change is recorded, never the window").
FAIL: any of (a) to (h) fails.

**1.3 Tyre grip on the skidpad** (nb-44).
`broadsword run crash-sim -- cargo test --offline --release --test skidpad -- --nocapture --test-threads 1 > scratch/s7-1-3.txt 2>&1`
PASS iff `run: ok`, the file contains `test result: ok. 4 passed; 0 failed`, and the `skidpad` lines show all of the following. Below, `limit_g` is the `grip_scale=1.0` value, and limit(0.8) and limit(1.2) are the `limit_g` of the `grip_scale=0.8` and `grip_scale=1.2` lines.
(a) `0.750 <= limit_g <= 0.950`;
(b) `limit_g` <= 1.02 x `mu`, limit(0.8) <= 1.02 x 0.8 x `mu`, and limit(1.2) <= 1.02 x 1.2 x `mu`;
(c) limit(0.8) < `limit_g` < limit(1.2), with limit(0.8) / `limit_g` in [0.72, 0.88] and limit(1.2) / `limit_g` in [1.08, 1.32];
(d) for each of the three runs: the `level` line of its `limit_level` says `steady=yes` and its `a_g` equals that run's `limit_g`; `limit_level` <= `levels` - 2; and `steady_levels` >= 5. Also, `radius_kinematic_m` is within 1% of 30.00 (29.70 to 30.30), and there are exactly `levels` `level` lines per run;
(e) the target range was fixed before any tyre tuning, not only before the skidpad test. The tuning files are exactly these six crash-sim paths: `scenarios/car_basic.bsn` (its tyre section is the tyre data), `scenarios/pad_and_wall.bsn` (it embeds the same car), `scenarios/skidpad.bsn`, `scenarios/skidpad.input`, `src/vehicle.rs` (nb-39's tyre model) and `tests/skidpad.rs`. In Git Bash from the Broadsword root:
```
{ P=projects/crash-sim; T="scenarios/car_basic.bsn scenarios/pad_and_wall.bsn scenarios/skidpad.bsn scenarios/skidpad.input src/vehicle.rs tests/skidpad.rs"
R=$(git -C $P log --format=%H -S"0.750 <= limit_g <= 0.950" -- docs/STAGE7.md)
A=$(git -C $P log --diff-filter=A --format=%H -- tests/skidpad.rs)
C=$(git -C $P log --format=%H -- $T)
if [ -z "$R" ] || [ -z "$A" ] || [ "$(echo $A | wc -w)" -ne 1 ]; then echo "range-order: FAIL a commit is missing (range: ${R:-none}; skidpad test: ${A:-none})"; else
  R1=$(echo $R | tr ' ' '\n' | tail -n 1); RL=$(echo $R | tr ' ' '\n' | head -n 1); echo "range_first=$R1 range_last=$RL skidpad_added=$A"
  ok=1; n=0; after=0
  for X in $C; do n=$((n+1))
    if [ "$X" != "$R1" ] && git -C $P merge-base --is-ancestor "$X" "$R1"; then echo "before-range $X"
    elif [ "$X" != "$RL" ] && git -C $P merge-base --is-ancestor "$RL" "$X"; then echo "after-range $X"; after=$((after+1))
    else echo "range-order: FAIL $X touches the tyre files and is neither older than $R1 nor newer than $RL"; ok=0; fi
  done
  if [ "$A" = "$RL" ] || ! git -C $P merge-base --is-ancestor "$RL" "$A"; then echo "range-order: FAIL tests/skidpad.rs ($A) is not newer than $RL"; ok=0; fi
  echo "tyre_commits=$n after_range=$after"
  if [ $ok = 1 ] && [ $after -ge 1 ]; then echo "range-order: ok"; else echo "range-order: FAIL"; fi
fi; } > scratch/s7-1-3-range.txt 2>&1
```
How it proves the order. R lists every commit that changed how often the range string occurs in this file: R1, the oldest, wrote the range; RL, the newest, is its last edit (R1 = RL if it was never edited). A is the commit that first added `tests/skidpad.rs`. C lists every commit that touched any of the six tuning files. Each commit in C must be either strictly older than R1 (an ancestor of R1 and not R1 itself: written before the range existed in the repository, so it cannot have been aimed at it; these are nb-38's, nb-39's and nb-40's commits, which nb-44 depends on) or strictly newer than RL (RL is its ancestor and it is not RL itself: written after the range was final). Anything else fails: a tuning commit between the range and a later edit of the range, a tuning commit in the same commit as the range, or a commit on a branch that forked before the range and was merged after it. A, the skidpad test, must itself be strictly newer than RL.
What it prints: one `range_first=... range_last=... skidpad_added=...` line, one `before-range <sha>` or `after-range <sha>` line per tuning commit (or a `range-order: FAIL <sha> ...` line for one that is neither), a `tyre_commits=<n> after_range=<m>` line, and a last line that is `range-order: ok` or starts `range-order: FAIL`.
PASS iff the **last line** of `scratch/s7-1-3-range.txt` is exactly `range-order: ok`. That line is printed only when R and A are both found, A was added exactly once, every tuning commit is classified before or after, A is after RL, and at least one tuning commit is after RL (A always is, so an empty commit list cannot pass). An empty file, or a file ending in a git error, fails, because the rule needs the `ok` line. The script was run on nine throwaway Git histories (built by a test script in the coder's scratchpad, not kept). It printed `range-order: ok` for exactly two: a vehicle commit, then the range, then a tyre change, then the skidpad test; and the range, then a tyre change, then the skidpad test. It printed a last line starting `range-order: FAIL` for the other seven: the range removed and restored after a tyre change; a tyre change in the range's own commit; the skidpad test before the range; no range; no skidpad test; a tyre change on a branch forked before the range and merged after it; and a folder that is not a Git repository;
(f) `docs/PHYSICS.md` has a tyre tuning log (nb-44) with one row per try (the tyre parameters changed and the `limit_g` measured), and its last row's `limit_g` equals the one in (a). A log of one row that says "no change needed" is fine;
(g) the range rests on a confirmed source. In Git Bash from the Broadsword root:
`{ B=$(grep -E '^- \*\*Target `limit_g` 0\.75 to 0\.95 g\*\*' projects/crash-sim/docs/STAGE7.md); echo "label: $(echo "$B" | grep -o '(\*[a-z, ]*\*)')"; grep -E '^  confirmed: .+ \| .+ \| https?://[^ ]+ \| read [0-9]{4}-[0-9]{2}-[0-9]{2} UTC$' projects/crash-sim/docs/STAGE7.md; } > scratch/s7-1-3-source.txt 2>&1`
PASS iff the first line of `scratch/s7-1-3-source.txt` is exactly `label: (*sourced*)` and the file has at least one `confirmed:` line after it, each with a title, an `http` or `https` URL and a read date. The orchestrator opens each URL once when the evidence is written and records in the evidence that it states the figure quoted; a URL that does not, fails this check. While the label is still `(*unverified, from memory*)`, as it is in this version of the file, (g) fails and so does 1.3. A missing bullet prints `label: ` and fails. If the confirmation changes the range, the bullet, (a), (g)'s pattern and the `-S` string of (e) change in the same commit.
Record `limit_g`, the limit level's `speed_kmh` and `radius_m`, and `limit_balance` in the evidence, and the `confirmed:` lines of (g).
FAIL: any of (a) to (g), or a missing line.

**1.4 Headless: a timed lap of the oval** (nb-47).
`broadsword run crash-sim -- cargo run --offline --release --bin race -- --body scenarios/car_basic.bsn --track scenarios/tracks/oval.track --input scenarios/tracks/oval_lap.input > scratch/s7-1-4.txt 2>&1`
`scenarios/tracks/oval_lap.input` (nb-47) drives from the grid over the line, one full lap, over the line again, and then brakes to a stop.
PASS iff `run: ok` and the output contains:
- the line `track name=oval points=64 checkpoints=3 centre_length_m=388.41 width_min_m=12.00 width_max_m=12.00`;
- exactly one `start ` line, and before the first `lap ` line the three lines `checkpoint lap=1 k=1`, `k=2` and `k=3` in that order, with increasing `t`, each greater than the `start` time;
- a line `lap n=1 ... valid=yes reason=none ...` whose `start_t` equals the `start` line's `t`, with `off_track_s=0.000`, `path_m` >= 338.67, `time_s` x `max_speed_kmh` / 3.6 >= `path_m` - 2, and |`time_s` - (`end_t` - `start_t`)| <= 0.0006 (the test `oval_lap_is_valid_and_above_floors` checks the last one to the exact integer millisecond);
- a last line `race: laps_valid=<at least 1> laps_invalid=0 best_lap_s=<lap 1's time_s, or a faster valid lap>`.
Record lap 1's `time_s` in the evidence for milestone 2.
FAIL: `run: FAILED`; a missing or different `track` line; checkpoint lines missing or out of order; lap 1 invalid; a path below 338.67 m (a lap counted without driving round it on track); a timing inconsistency (the timer disagrees with the motion); or any invalid lap.

**1.5 Leaving the track voids the lap, and the track does not change the physics** (nb-47).
The 1.4 command with `--track scenarios/tracks/oval_pinch.track`, saved as `scratch/s7-1-5.txt`.
PASS iff `run: ok`, the `track` line ends `width_min_m=0.20 width_max_m=12.00`, there is a `lap n=1 ` line with `valid=no reason=off_track` and `off_track_s` greater than 0.000 whose `start_t`, `end_t` and `time_s` are character for character those of lap 1 in 1.4, the `start` and `checkpoint` lines are character for character those of 1.4, the `final` line is character for character the `final` line from 1.4, and the last line starts `race: laps_valid=0`.
FAIL: lap 1 valid (`on_track` is not applied); a different lap time or `final` line (the track changed the physics or the timing); or `reason=missed_checkpoint` (the pinch moved a gate).

**1.6 The lap is physics, not a step-size artefact** (nb-47). The 1.4 command with `--substeps 20` added (h = 0.5 ms), saved as `scratch/s7-1-6.txt`.
PASS iff `run: ok`, lap 1 is `valid=yes`, `laps_invalid=0`, and lap 1's `time_s` is within 1% of 1.4's. FAIL: any of these.

**1.7 The timer can be drawn** (nb-48). `broadsword test render > scratch/s7-1-7.txt 2>&1`, then, in Git Bash from the Broadsword root:
`git -C projects/render log --diff-filter=MDR --name-status --format=%h 9e1060b..master -- tests/golden > scratch/s7-1-7-golden.txt 2>&1`
PASS iff `test: ok — N passed, 0 failed`; `scratch/s7-1-7.txt` contains `test hud_digits_match_golden ... ok` (seven-segment digits drawn with `Framebuffer::line` only: the strings `0123456789`, `1:23.456` and `-:--.---` against the new `tests/golden/hud_digits.ppm`), `test hud_refuses_unknown_character ... ok` (any character other than `0` to `9`, `:`, `.`, `-` and space is an error naming it, and the framebuffer is left unchanged), and `test lap_time_text_comes_from_whole_milliseconds ... ok` (83456 ms gives `1:23.456`, 59999 gives `0:59.999`, 60000 gives `1:00.000`, 600000 gives `10:00.000`; parsing the race line text `83.456` gives 83456 with no float in between); and `scratch/s7-1-7-golden.txt` is empty (since render `9e1060b`, no golden image has been modified, deleted or renamed; new ones are allowed).
FAIL: a failed or missing test; any line in the golden file.

**1.8 A human drives a timed lap** (nb-48 viewer; nb-49 records it). This is the check that tests cannot replace. Tell Lazlo first (nb-37: the PC shares RAM with the GPU; the viewer is software-rendered, but ask before long sessions).
`broadsword run crash-sim -- cargo build --offline --release --bin race > scratch/s7-1-8-build.txt 2>&1` (from the main checkout, so `build/release/race.exe` is built from crash-sim main), then
`broadsword run render -- cargo run --offline --release --example drive -- --sim-exe ../../build/release/race.exe --body ../crash-sim/scenarios/car_basic.bsn --track ../crash-sim/scenarios/tracks/oval.track --save-input ../../scratch/s7-1-8.input > scratch/s7-1-8.txt 2>&1`
The viewer starts `race.exe --live` with the same `--body` and `--track` as a child process, draws the track edges, the start line and the checkpoints from the `track_line` lines, and shows the current lap time and the last valid lap time in seven-segment digits (`-:--.---` until there is one). It forwards every race line from the child (the `track`, `start`, `checkpoint`, `lap`, `final` and `race:` lines; not the `track_line` lines or trajectory rows) to its own stdout unchanged. When the window closes it closes the child's stdin. The child ends the run at the end of the substep it is in when it reads end of input; call that time T_end (a whole substep, so a whole millisecond at the default step). It prints its `final` line with `t=` T_end and its `race:` line, and exits. The viewer then saves the inputs (nb-40's `--save-input`) with one more line at the end, the **end line**: `<T_end> <throttle> <brake> <steer>`, where `<T_end>` is the `t=` text of the child's `final` line copied character for character (4 decimals, never re-formatted from a float), and the three values are the ones in force at T_end (the previous saved line's values, repeated, so the end line changes no input). If the last saved line's time text is already T_end, no end line is added, because that line already ends a replay at T_end. No saved line has a time after T_end: an input the child never applied is not saved. nb-40's file rule then ends a replay at that line's time, which is T_end. If the child's `final` line never arrives (the child crashed), the viewer writes no end line and says so in a `saved input: ... no end line (child gave no final line)` message, and the automatic part of 1.8 fails. After saving, the viewer prints `saved input: ../../scratch/s7-1-8.input` and `hud current=<text> last=<text>`, with exactly the two strings on screen in the last frame. The `hud` line prints the same two string values that were passed to the drawing call; 1.7's golden test covers the drawing, and Q3 below is the human check of what was on the screen. Lazlo drives with the arrow keys or WASD, from the grid, over the line, at least one full lap and over the line again, then closes the window. He may try as often as he likes; each attempt is a new run of the command, and the evidence keeps the last one and counts them.
Automatic part. PASS iff `scratch/s7-1-8-build.txt` shows `run: ok`, and `scratch/s7-1-8.txt` contains a `lap ... valid=yes` line, the `final` line, the `saved input:` line (without `no end line`), and a `hud` line whose `last=` text is the `M:SS.mmm` form of the last valid lap's `time_s`; and the end line is in place: in Git Bash from the Broadsword root, `{ echo "final_t=$(grep -E '^final ' scratch/s7-1-8.txt | sed -E 's/^final t=([^ ]+) .*/\1/')"; echo "input_end_t=$(tail -n 1 scratch/s7-1-8.input | cut -d ' ' -f 1)"; } > scratch/s7-1-8-end.txt 2>&1` writes two lines whose values after `=` are the same non-empty text (for example `final_t=95.4120` and `input_end_t=95.4120`).
Human part, recorded by the orchestrator in `evidence/stage-7-m1-human.md` (nb-49) before scratch can be wiped. It holds: the UTC date and time; the number of attempts; the `start`, `checkpoint`, `lap`, `final`, `saved input` and `hud` lines of the last attempt, pasted; Lazlo's answers to three questions, quoted verbatim from his own message: Q1 "Did the car respond to the keys?", Q2 "Did the timer on screen run during the lap and show the lap time when you crossed the line?", Q3 "Was the time on screen the same as the printed `lap` line?"; and his answer to "What felt wrong, or not like driving a car?", quoted verbatim, with one board task id or `IDEAS.md` line per point he raises. The orchestrator copies `scratch/s7-1-8.input` to `evidence/stage-7-m1-human.input`.
PASS iff the automatic part passes and Q1, Q2 and Q3 are each answered yes in Lazlo's own words. An agent never answers for him, and a paraphrase does not count. The "felt wrong" points are not gated, but each one must have its task or IDEAS line before the milestone is tagged.
FAIL: no window, no key response, no saved input, no valid lap, a `hud` text that differs from the printed lap time, any answer other than yes, or a missing record.

**1.9 The human lap replays headless** (nb-47, nb-49).
`broadsword run crash-sim -- cargo run --offline --release --bin race -- --body scenarios/car_basic.bsn --track scenarios/tracks/oval.track --input ../../evidence/stage-7-m1-human.input > scratch/s7-1-9.txt 2>&1`, then, in Git Bash from the Broadsword root:
`diff <(grep -E '^(track name=|start |checkpoint |lap |final )' scratch/s7-1-8.txt) <(grep -E '^(track name=|start |checkpoint |lap |final )' scratch/s7-1-9.txt) > scratch/s7-1-9-diff.txt 2>&1`
How the close time reaches the replay: the live run ended at T_end when the viewer closed the child's stdin, and 1.8 saved T_end as the time of the input's last line (the end line), copied from the live `final` line's `t=` text. The file run has no `--duration`, so by nb-40's rule it ends at that last line's time, which is the same T_end, and its `final` line is printed at the same substep. Before the diff, in Git Bash from the Broadsword root: `{ echo "live_final_t=$(grep -E '^final ' scratch/s7-1-8.txt | sed -E 's/^final t=([^ ]+) .*/\1/')"; echo "input_end_t=$(tail -n 1 evidence/stage-7-m1-human.input | cut -d ' ' -f 1)"; echo "replay_final_t=$(grep -E '^final ' scratch/s7-1-9.txt | sed -E 's/^final t=([^ ]+) .*/\1/')"; } > scratch/s7-1-9-end.txt 2>&1`
PASS iff `run: ok`; `scratch/s7-1-9-end.txt` has three lines whose values after `=` are the same non-empty text (the live close time, the saved end line's time and the replay's end time agree); `scratch/s7-1-9.txt` contains a `lap ... valid=yes` line (so the comparison is not of two empty sets); and `scratch/s7-1-9-diff.txt` is empty: the same lines, character for character, live and headless. A replay whose `final` line differs only because the saved input has no end line fails the first rule, and points at the viewer, not the physics. FAIL: any difference (the live run and the file run are not the same simulation, so a human lap cannot be checked or replayed).

What guards against tuning to the test in milestone 1: the grip target must be confirmed from a public source with its URL and read date (1.3 g) and is fixed in Git before any tyre tuning commit or the skidpad test (1.3 e); a physics ceiling and the grip variants must agree with friction (1.3 b, c); the skidpad measures motion, not tyre forces, and only over 5 s windows of steady cornering (1.3 d); the lap is checked against a geometric floor and a timing identity that do not use `LapTimer` (1.4); the pinch shows that `on_track` is applied and that the track does not change the physics (1.5); the lap must survive a different step size (1.6); the HUD and the printed time come from one integer (1.7); and a human has to drive it, and the drive must replay exactly (1.8, 1.9).

After 1.1 to 1.9 pass, the Stage 7 m1 retro entry in `RETRO.md` is due (AGENTS.md); nb-49 lists its inputs in `evidence/stage-7-m1.md`. The Stage 7 lesson goes in it: what the human check found that the tests did not.

## Milestones 2 to 5 (drafts)

**Milestone 2, AI drivers (draft, rewrite when the milestone starts).** Done: three AI cars finish a race without crashing into walls. Needs several cars in one sim (car-to-car contact is new physics), walls along the track edges (finite wall segments, not today's infinite planes), and an AI driver that is a pure function from what a driver could see (car state, track) to the same throttle, brake and steer that a human gives. The AI writes its inputs to a script, so every AI lap replays headless through `race --input` exactly as in 1.9, and the AI can never move the car by any other route. Checks to write: three cars, three laps each, every lap valid by nb-43's rule; zero wall contacts and zero car-to-car contacts in the race (counted every substep, as STAGE6 counts impacts); each AI's best lap within a stated margin of the scripted lap of 1.4, so the AI cannot win by crawling; and the same race twice gives the same bytes.

**Milestone 3, game shell (draft, rewrite when the milestone starts).** Done: a standalone executable a friend can run. Needs menus (track and car choice), a settings file (keys, window size), and saving (best laps per track), with strict, versioned files like `.bsn`, so a corrupt save is refused with a message and never crashes the game. Checks to write: the release build and its assets are copied to one folder and run on a fresh clone of the VM base image with no Rust installed (AGENTS.md: VMs are disposable); a scripted menu path starts a race and saves a lap; a corrupted save and settings file are each refused by name. Plus a human check like 1.8 with someone who is not Lazlo, if he chooses one, recorded the same way.

**Milestone 4, multiplayer (draft, rewrite when the milestone starts).** Done: two machines race each other. The sim is deterministic (STAGE6 3.5, check 1.9 here), so only inputs need to cross the network: lockstep, with each machine sending its sim-time-stamped input lines. Network services need Lazlo's OK first (AGENTS.md, ask first). Checks to write: two processes on one machine first, then the laptop and the PC; both machines print identical `lap` and `final` lines for both cars; a delayed or dropped packet is handled by waiting, never by guessing an input; a mismatch is detected, because each machine sends a hash of its state every second, and reported, never hidden.

**Milestone 5, second game (draft, rewrite when the milestone starts).** Done: it ships without changes to the engine core. Before the work starts, the engine core is written down as a list of files (likely the solver, body format, input, track and render modules). Checks to write: the second game is its own binary in the same repo; `git diff --stat <tag at the start of m5>..main -- <the core files>` prints nothing after the second game lands; both games' tests pass in one run. If the core must change, that is a finding recorded in `RETRO.md`, not a quiet edit.

## Who builds what

| Check | Brief(s) |
|---|---|
| 1.1 track tests, `oval.track`, `chicane.track`, the lap rule, `on_track`, the track grammar | nb-43 |
| 1.1 skidpad tests, 1.3, `skidpad.bsn`, `skidpad.input`, the tyre tuning log | nb-44 |
| 1.1 race tests, 1.4, 1.5, 1.6, the race program and `--live`, `default-run`, `oval_lap.input`, `oval_pinch.track` | nb-47 (proposed) |
| 1.2 | every Stage 7 brief that touches crash-sim; nb-44 and nb-47 in particular |
| 1.7, the HUD, the viewer's `--sim-exe` and `--track`, the `hud` line | nb-48 (proposed) |
| 1.8 record, 1.9 run, `evidence/stage-7-m1.md`, `evidence/stage-7-m1-human.md`, `evidence/stage-7-m1-human.input` | nb-49 (proposed) |

Proposed briefs (for the orchestrator to put on the board):

- **nb-47** `racing: race program (src/bin/race.rs, src/race.rs): car on the grid, LapTimer wired to the sim, race lines, --live child mode with track_line output`. Claims: `projects/crash-sim/src/race.rs` (new), `src/bin/race.rs` (new), `src/lib.rs` (the `pub mod race;` line), `Cargo.toml` (the `default-run` line only), `tests/race.rs` (new), `scenarios/tracks/oval_lap.input` (new), `scenarios/tracks/oval_pinch.track` (new). Depends on nb-43, nb-40 (crash-sim half) and nb-44 merged (the lap script is aimed with the final tyres). Done check: 1.1 (race tests), 1.4, 1.5, 1.6 and 1.2.
- **nb-48** `render: seven-segment HUD digits (src/hud.rs) with a golden test; drive viewer --sim-exe and --track draws the track and the lap timer`. Claims: `projects/render/src/hud.rs` (new), `src/lib.rs` (the `mod` line), `tests/hud.rs` (new), `tests/golden/hud_digits.ppm` (new), `examples/drive.rs`. Depends on nb-40 (render half) and nb-47 merged. Done check: 1.7, and the automatic part of 1.8 run with the window off screen, as `tests/window_smoke.rs` does.
- **nb-49** `Stage 7 m1 done check: the human lap, its headless replay and the evidence [broadsword half]`. Claims: `evidence/stage-7-m1.md` (new), `evidence/stage-7-m1-human.md` (new), `evidence/stage-7-m1-human.input` (new). Orchestrator. Depends on everything above. Done check: 1.1 to 1.9 on main, saved.

## What the briefs must take from this file

Where a brief and this file differ, this file sets the check, and the brief's coder raises the difference with the orchestrator. The items marked **(not in the brief)** are additions or conflicts that the orchestrator must apply to the board brief before it starts.

- **nb-43** (`t-20261119-171333-44`):
  - the gate, crossing (half-open, interpolated) and on-track definitions, and the lap rule 1 to 5 in "Shared definitions", all inside `LapTimer`;
  - **(not in the brief)** rule 4's off-track part: a lap with any substep off track is invalid. The brief has `on_track` separately; enforcing it inside `LapTimer` keeps the rule in one place. `update(prev_pos, pos, t)` also needs the previous substep's time to interpolate: either the timer keeps it, or the signature takes `t_prev`;
  - **(not in the brief)** the `grid <m>` line and the per-point width override in the track grammar;
  - the exact `oval.track` above and the 13 `tests/track.rs` names in 1.1;
  - **(not in the brief)** its claims need `projects/crash-sim/src/lib.rs` (the `pub mod track;` line; `tests/track.rs` cannot reach the module without it; lib.rs is also claimed by nb-39, so sequence after it) and `projects/crash-sim/.gitattributes` (a `*.track text eol=lf` line, as `*.bsn` and `*.input` have, or a Windows checkout gives the strict parser CRLF files).
- **nb-44** (`t-20261119-171333-45`):
  - the target range 0.75 to 0.95 g with its sources, the ceiling, the variants, the measurement from motion only and over steady windows only, `skidpad.input` as constant steer with throttle levels held at least 20 s each (the brief says "slowly rising speed"; the levels are how it rises, so the limit is read from steady circles, not from a ramp), the test names and printed lines of 1.3, and the tuning log row format;
  - the range must not change after any commit to the six tuning files of 1.3 (e), and nb-44 makes no tuning commit until the range is confirmed and labelled *sourced* (1.3 g); the range's figures are unverified recollections until then;
  - **(not in the brief; conflict)** changing the tyre section of `car_basic.bsn` breaks STAGE6's `pad_and_wall_car_is_car_basic` (nb-41: `pad_and_wall.bsn` embeds the car byte for byte) and may push STAGE6 3.6's impact speed or lap out of its window. If nb-44 changes the tyres, its claims must add `projects/crash-sim/scenarios/pad_and_wall.bsn` (the same tyre change) and `projects/crash-sim/scenarios/pad_lap.input` (re-aimed if 3.6 fails, as STAGE6 allows), and it must pass check 1.2 here. If the measured limit is already in range with the nb-38/nb-39 tyres, nothing changes and the conflict does not arise.
- **nb-40** (`t-20261119-171333-39` and `-40`), for its F1 child-process viewer: this file names the viewer's flag for the program to start `--sim-exe <path>`. If nb-40 picks another name, nb-48 adds `--sim-exe` as well, or the orchestrator changes 1.8 here before nb-48 starts.
- **nb-47**: the race program's flags, placement on the grid, output lines and exit codes; `default-run = "crash-sim"` in the commit that adds the binary; `oval_pinch.track` differs from `oval.track` only in the widths of points 5 to 10; the measured 1.6 difference in its report.
- **nb-48**: the HUD's character set and refusal; `M:SS.mmm` from whole milliseconds, parsed from the race line's text; the viewer never parses a track file; it forwards the child's race lines unchanged and prints the `hud` line at close; **(not in the brief)** the end line of 1.8: the saved input ends with a line at the child's `final` `t=` text, so a replay ends at the live close time (1.9).
- **nb-47** also: **(not in the brief)** in `--live`, end of input ends the run at the end of the current substep and the `final` line is printed at that time (1.8's T_end).
