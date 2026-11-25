# Stage 7 done checks: milestones 1 to 3 (milestones 4 and 5 are pointers to their own tasks)

Written before any Stage 7 code, on crash-sim `5415a5b` (board task `t-20261119-171333-43`, brief nb-42). DEVELOPMENT_PLAN Stage 7:

1. Racing basics: tire grip, a track, lap timing. *Done:* a timed lap you can drive.
2. AI drivers. *Done:* three AI cars finish a race without crashing into walls.
3. Game shell: menus, settings, saving. *Done:* a standalone executable a friend can run.
4. Multiplayer. *Done:* two machines race each other.
5. Second game: a small game in another genre on the same toolkit. *Done:* it ships without changes to the engine core.

Lesson focus: whole-product integration, where many parts must work together and "tests pass" stops being enough.

This file turns milestone 1 into commands with numbers. Milestone 1 is done when every one of its checks passes on crash-sim main and render master, the output is saved in `evidence/`, and the human check (1.8) is recorded. The table "Who builds what" maps every milestone 1 check to the brief that makes it pass: nb-43 and nb-44 (on the board as `t-20261119-171333-44` and `t-20261119-171333-45`) and three new briefs proposed here, nb-47, nb-48 and nb-49. Milestones 2 and 3 have their own numbered checks (2.1 to 3.6) in "Milestones 2 and 3: exact checks", written on crash-sim `0bb2b5e` by board task `t-20261121-001109-6` (brief n20-16), and their own table, "Who builds what, milestones 2 and 3", at the end. The checks of milestones 4 and 5 are written by the tasks named in "Milestones 4 and 5".

## Where the game lives

Inside crash-sim: a library module and a second binary, not a separate `projects/racing` repo. This is ADR 0001 option (a), accepted by Lazlo on 2026-11-19 (UTC) (`docs/adr/0001-shared-core.md` in the Broadsword repo, follow-up F3), so it is not a new decision for him.

Reason: DEVELOPMENT_PLAN says "the crash sim grows into a racing game", and the racing game needs the crash sim's body, car, solver and input code as a library. A separate repo would need a relative path dependency on crash-sim, and ADR 0001 measured that no relative `path =` string works both in a main checkout (`projects/racing`, two folders below the root) and in a task worktree (`worktrees/racing/<task id>`, three below). Every task is built and checked in a worktree, so that layout fails in exactly the place work happens. The game engine of roadmap 4.1 can still become its own crate later through ADR 0001's (d2) route, one workload repo with a Cargo workspace, if a trigger in that ADR fires.

The layout in milestone 1:

| Part | Where |
|---|---|
| Track format, gates, `on_track`, `LapTimer` | crash-sim `src/track.rs` (nb-43) |
| The race: car on the grid, sim plus lap timer, race lines | crash-sim `src/main.rs` and the module it uses (n20-13, `t-20261121-001109-5`; nb-47's `src/race.rs` is not used) |
| The race program | the default `crash-sim` program (`src/main.rs`), run as plain `cargo run` (n20-13; decided by the orchestrator, not a second binary) |
| Track files | crash-sim `scenarios/tracks/` (nb-43, nb-47) |
| Skidpad measurement | crash-sim `tests/skidpad.rs` (nb-44) |
| Lap timer on screen, track drawn in the viewer | render `src/hud.rs` and `examples/drive.rs`, with the race program as a child process (nb-48, after nb-40's F1 viewer) |

Milestone 1 adds no second binary: the race is the default `crash-sim` program with `--track` (and `--live`), so plain `cargo run` keeps meaning crash-sim, every STAGE6.md command runs unchanged (check 1.2 proves it), and no `default-run` line is needed. (The second game of milestone 5 is the first second binary, and the one `Cargo.toml` line it may need is that milestone's business.)

## How to run and read a check

As in STAGE6.md:

- Run every command from the Broadsword root. `broadsword` means `build/bin/broadsword.exe`. Inside a task, add `--task <id>` after the project name.
- Never call cargo directly. The race program runs as `broadsword run crash-sim -- cargo run --offline --release -- <flags>`. The program's working folder is the project folder, so `scenarios/...` paths are relative to `projects/crash-sim`.
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

`scenarios/tracks/oval_pinch.track` (board task `t-20261121-015242`, which replaces nb-47 for this file) is `oval.track` with two changes, in two stretches: points 5 to 10 (on bend 1, p = 24 to 54 degrees, about 16 m of bend) and points 33 to 36 (point 33 is the last point of the back straight, where bend 2 begins, and points 34 to 36 are the first 18 degrees of bend 2) have a per-point width of 0.2 m. There are two stretches because the oval script of 1.4 drives two laps and check 1.5 needs both laps invalid: one narrowed stretch cannot make both laps invalid, because lap 2 stays inside the band (the task `t-20261121-015242` found this when building it). The gates at 0, 16, 32 and 48 keep 12 m. A test checks that nothing else differs.

`scenarios/tracks/chicane.track` (nb-43): any closed track with a chicane, for the parser and the lap timer tests. No run in milestone 1 uses it.

Why these numbers:

- **Width 12 m.** FIA rules for new permanent circuits ask for a track width of at least 12 m (FIA International Sporting Code, Appendix O; from memory, not checked online in this task). It is also three car widths for a 1.8 m wide car, enough room for an open-loop script to stay on track.
- **Bend radius 30 m, straights 100 m.** At a lateral limit of 0.83 to 0.93 g (check 1.3), the bend speed sqrt(a R) is 15.6 to 16.5 m/s (56 to 60 km/h), close to the 50 km/h of STAGE6's crash. A lap is short (about 390 m), so an out-lap plus one timed lap is well under a minute of simulated time.
- **Arc points every 6 degrees.** Each chord is then 0.05% shorter than its arc (sin 3 degrees / 3 degrees in radians = 0.99954). The path floor below is computed for exactly these points.

### The race program (nb-47)

`crash-sim --body <file> --track <file> --input <file> [--substeps <n>] [--duration <s>]` runs headless. The car from `--body` is turned about the vertical axis so its long axis (+x in the file, nb-38) points along the tangent at the start point, and moved so its position is `grid` metres before the start line; its height is unchanged, so it drops 0.2 m as in STAGE6.md 3.2. `--input` is nb-40's input script, and the run ends at its last line unless `--duration` is given (nb-40's rule). The track has no physical effect in milestone 1: off the track the ground is the same plane with the same grip. Leaving the track only makes a lap invalid.

`crash-sim --live --body <file> --track <file>` is the same run as a child process of the viewer. It uses nb-40's child-process protocol unchanged (input lines in on stdin, stamped in simulation time; trajectory v2 rows out), and adds two things: before the first trajectory row it prints one `track_line <kind> <x0> <z0> <x1> <z1>` line per segment to draw (kind `left` and `right` for the two edges, `start`, and `checkpoint`), and it prints each race line below as soon as it happens. The viewer never parses a track file itself; the only track parser is nb-43's.

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

- **Target `limit_g` 0.83 to 0.93 g** (*sourced*). `car_basic.bsn` is a 1200 kg road car on road tyres, not a sports car. The range is exactly the lowest and highest of the confirmed figures below: Car and Driver 300 ft skidpad results for production sedans on their standard road tyres, read on 2026-11-20 (UTC) from two public pages that quote them (the Altima figure is on all-season tyres). Check 1.3 (g) enforces that the bounds equal the lowest and highest `confirmed:` figure, so the range cannot be moved without a confirmed public figure at the new bound. Limits of the evidence: the cars read are mid-size to large sedans, heavier than a 1200 kg small car (their masses were not read); no small-car skidpad figure could be read (Car and Driver's, Motor Trend's and Edmunds' own pages refused automated reads), so the low end for economy cars is not covered. Search snippets showed small-car figures of about 0.81 to 0.82 g, but those pages were not read, so they are not used. This range replaces the earlier 0.75 to 0.95 g, which was recalled, not read: no confirmed figure is below 0.83 g or above 0.93 g, so both old bounds went further than the sources. Still *unverified, from memory*, and not used to set the range: peak tyre-road friction on dry asphalt of about 0.8 to 0.9 (as recalled from Gillespie, *Fundamentals of Vehicle Dynamics*, SAE, 1992, and Wong, *Theory of Ground Vehicles*).
  confirmed: 0.83 g (2025 Mercedes-Benz S-Class, Car and Driver skidpad, the lowest sedan listed) | Understanding Vehicle Capability and Its Role in Protective Driving (ISDA, Tony Scotti, 2025-07-10; figures credited to Car and Driver, 300 ft pad) | https://isdacenter.org/understanding-vehicle-capability-and-its-role-in-protective-driving/ | read 2026-11-20 UTC
  confirmed: 0.89 g (2025 BMW 7-Series, Car and Driver skidpad) | Understanding Vehicle Capability and Its Role in Protective Driving (ISDA, Tony Scotti, 2025-07-10; figures credited to Car and Driver, 300 ft pad) | https://isdacenter.org/understanding-vehicle-capability-and-its-role-in-protective-driving/ | read 2026-11-20 UTC
  confirmed: 0.92 g (2025 Mercedes-Maybach S-Class, Car and Driver skidpad) | Understanding Vehicle Capability and Its Role in Protective Driving (ISDA, Tony Scotti, 2025-07-10; figures credited to Car and Driver, 300 ft pad) | https://isdacenter.org/understanding-vehicle-capability-and-its-role-in-protective-driving/ | read 2026-11-20 UTC
  confirmed: 0.93 g (2025 Audi A8, Car and Driver skidpad, the highest sedan listed) | Understanding Vehicle Capability and Its Role in Protective Driving (ISDA, Tony Scotti, 2025-07-10; figures credited to Car and Driver, 300 ft pad) | https://isdacenter.org/understanding-vehicle-capability-and-its-role-in-protective-driving/ | read 2026-11-20 UTC
  confirmed: 0.93 g (2019 Nissan Altima on Hankook Kinergy GT all-season tyres, Car and Driver skidpad) | Car and Driver claim the 2019 Altima has a higher cornering limit than a 3 Series (Hooniverse, Jeff Glucker, 2019-04-04) | https://www.hooniverse.com/car-and-driver-claim-the-2019-altima-has-a-higher-cornering-limit-than-a-3-series/ | read 2026-11-20 UTC
  confirmed: 0.89 g (BMW 330xi on Bridgestone Turanza T005 tyres, Car and Driver skidpad) | Car and Driver claim the 2019 Altima has a higher cornering limit than a 3 Series (Hooniverse, Jeff Glucker, 2019-04-04) | https://www.hooniverse.com/car-and-driver-claim-the-2019-altima-has-a-higher-cornering-limit-than-a-3-series/ | read 2026-11-20 UTC
  The same ISDA page lists SUVs and vans from 0.60 to 0.83 g; they are not passenger cars of `car_basic.bsn`'s kind, so they are left out of the range on purpose, not because they disagree.
  **Sources are verified by review, not by a script.** A script cannot open a web source offline, so 1.3 (g) checks form and consistency only: the label, that each `confirmed:` line has a figure, a title, a URL and a read date, and that the range equals the lowest and highest figure. It never opens a URL and cannot show that a page states the figure quoted. That is a review step: whoever adds or changes a `confirmed:` line must give its URL, and a reviewer re-opens that URL and checks the figure before the range can change. Any range change after tyre tuning has started needs Lazlo's yes. All dates in this file are UTC.

  **Changing the range later.** A new figure is added as one more `confirmed:` line in the format above, read from a public page, and the range, (a) and the `-S` string of (e) are changed in the same commit to its new lowest and highest figures, before any tuning commit. 1.3 (g) fails if they disagree. A changed range string is a new range for 1.3 (e): its R1 is the commit that wrote the new string, so a tuning commit made between the old and the new range is older than R1 and passes (e); (g)'s rule that the bounds are the confirmed extremes is what stops such a change being aimed at the tyres. A range change is reported to Lazlo as a finding, and the check is not loosened to pass it.
- **Ceiling: `limit_g` <= 1.02 x mu** (*reasoned*). On flat ground with no downforce, the sum of the tyres' horizontal forces is at most mu times the sum of their normal loads, which is mu M g, so a_lat <= mu g. The 2% matches the steadiness tolerance (a steady window may still vary by 2%). A result above this ceiling means the measurement, or the physics, is creating grip.
- **Grip variants: 0.72 <= limit(0.8) / limit(1.0) <= 0.88 and 1.08 <= limit(1.2) / limit(1.0) <= 1.32, strictly ordered** (*reasoned*). With a friction-circle tyre whose mu does not depend on load, the limit is proportional to mu, so the ratios should be 0.8 and 1.2. The 10% allows for load-sensitive tyre terms, if nb-39 has any. A result that does not move with grip was tuned or is noise (the Stage 6 lesson).
- **Radius 30 m** (nb-44's example). Without downforce the limit in g does not depend on the radius, and a small radius keeps the limit speed near 16 m/s, so few throttle levels are needed.
- **Steady means 5 s within 2% in speed, yaw rate and radius, measured on 0.1 s blocks** (*reasoned*). On a steady circle, v, r and rho are constant, so a window where all three hold still is cornering at that acceleration, not passing through it. 5 s is at least five periods of the slowest ride mode STAGE6 allows (1.0 Hz), and longer than a yaw transient. 2% of 0.88 g (the middle of the band) is 0.0176 g, under a fifth of the 0.10 g target band. The 0.1 s blocks average out the integrator's per-substep noise but not body motions slower than 10 Hz, so a car that still bounces or yaws in the window fails the 2% rule rather than being smoothed into a pass. Each level is held at least 20 s, so the 15 s before its window lets the speed settle after the throttle step (STAGE6 3.3's slowest suspension decays by e^-19 in that time). If a level does not settle in 15 s, it is reported as not steady; nb-44 may hold the levels longer, but never loosens the window or the 2%.
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
PASS iff the last line is `test: ok — N passed, 0 failed`, N is at least the N recorded for STAGE6 check 3.1 in `evidence/stage-6-m3.md` plus the 25 tests named here, and the file contains each of:
`Running tests\track.rs`, `test oval_track_parses_as_specified ... ok` (64 points with the coordinates above to 1e-9 m, start 0, checkpoints 16, 32, 48, width 12, grid 20), `test refuses_fewer_than_three_points ... ok`, `test refuses_self_intersecting_centreline ... ok`, `test refuses_checkpoint_out_of_range ... ok`, `test refuses_checkpoints_out_of_order ... ok`, `test track_files_have_no_carriage_return ... ok`, `test crossing_time_is_exact_for_a_straight_pass ... ok` (from x = -0.3 at t = 0 to x = +0.7 at t = 0.001 across a gate at x = 0, the crossing time is 0.0003 within 1e-12 s), `test endpoint_on_the_line_counts_once ... ok`, `test lap_with_a_skipped_checkpoint_is_not_counted ... ok`, `test reversing_over_the_line_never_counts ... ok` (forward over the line, back over it, forward again: no valid lap), `test driving_the_track_backwards_never_counts ... ok` (a whole loop against the direction of travel: no `start`, no lap), `test lap_off_track_is_invalid ... ok`, `test on_track_uses_half_width ... ok` (nb-43, `tests/track.rs`; the refusal tests check the line number in the message, as `tests/format.rs` does);
`Running tests\skidpad.rs`, `test skidpad_car_is_car_basic ... ok`, `test skidpad_input_holds_a_30_m_kinematic_circle ... ok` (one steer value for the whole file, with a kinematic radius within 1% of 30 m; throttle levels rising, each held at least 20 s; brake 0 throughout), `test skidpad_limit_in_road_tyre_range ... ok`, `test skidpad_limit_follows_grip ... ok` (nb-44; the last two assert the rules of 1.3);
`Running tests\timed_lap.rs` (n20-13's file name), `test oval_script_gives_counted_laps ... ok` (the oval script gives two counted laps), `test timed_lap_same_input_same_bytes ... ok` (the same run twice prints identical bytes), `test script_that_cuts_a_checkpoint_gives_no_counted_lap ... ok`, `test car_on_grid_faces_the_track ... ok` (after placement the chassis heading is the start tangent within 0.01 degree and the car position is 20 m before the line within 1e-6 m), `test oval_lap_is_valid_and_above_floors ... ok` (runs the 1.4 run in-process and asserts its rules), `test live_and_file_input_give_same_race_lines ... ok` (the input of 1.4 fed through `--live` gives the same `track`, `start`, `checkpoint`, `lap` and `final` lines as the file run) (n20-13, which includes `--track` with `--live`, and also has `live_prints_lap_lines_as_they_happen ... ok` and `track_refusals_exit_2_with_empty_stdout ... ok`);
`Running tests\oval_pinch.rs`, `test oval_pinch_differs_only_in_width ... ok` (`oval_pinch.track` has the widths of points 5 to 10 changed and nothing else differs from `oval.track`) (board task `t-20261121-015242`, which creates `scenarios/tracks/oval_pinch.track` and `tests/oval_pinch.rs`);
and the 26 crash-sim Stage 6 tests that check 1.2 lists by name.
FAIL: any failed or missing test, or a smaller N.

**1.2 Stage 6 still works on the Stage 7 code.** Whole-product integration: the second binary, the tyre tuning and the new modules must not break what Stage 6 delivered. Run every part on the commit being checked, from the Broadsword root. The commands and numbers below are copied from STAGE6.md as it stands at crash-sim `26d0bea`. If STAGE6.md changes one of them, this file is changed in the same commit, so the two never disagree silently.

(a) The old tests are unchanged: `git -C projects/crash-sim diff --stat abe471c -- tests/csv.rs tests/determinism.rs tests/energy.rs tests/invariants.rs tests/physics.rs tests/risky.rs tests/stability.rs tests/summary.rs tests/sweep.rs > scratch/s7-1-2a.txt 2>&1`
PASS iff `scratch/s7-1-2a.txt` is empty.
Those nine files are the tests that existed before Stage 6 (STAGE6.md 1.2). The test files Stage 6 added (for example `tests/format.rs`, `tests/settle.rs`, `tests/deform.rs` and `tests/solver.rs`) are pinned against the crash-sim commit at which Stage 6 milestone 3 passed, named S6. S6 is written in this file, on the line below, by the orchestrator when it records Stage 6 m3 in `evidence/stage-6-m3.md`, as the same commit it records there. Until then the line says `none yet`, and this part fails.

Stage 6 commit: none yet

In Git Bash from the Broadsword root:
```
{ P=projects/crash-sim; L=$(tr -d '\r' < $P/docs/STAGE7.md | grep -cE '^Stage 6 commit: '); S6=$(tr -d '\r' < $P/docs/STAGE7.md | grep -E '^Stage 6 commit: ' | head -n 1 | sed -E 's/^Stage 6 commit: //')
if [ "$L" != 1 ] || ! echo "$S6" | grep -qxE '[0-9a-f]{7,40}' || ! git -C $P merge-base --is-ancestor "$S6" HEAD; then echo "stage6-tests: FAIL S6 is not one commit that is an ancestor of HEAD (lines: $L; value: ${S6:-missing})"; else
  F=$(git -C $P ls-tree --name-only "$S6" tests/ | grep -E '^tests/[^/]+\.rs$'); N=$(echo $F | wc -w); echo "stage6_commit=$S6 test_files=$N"
  if ! D=$(git -C $P diff --name-only "$S6" HEAD -- $F); then echo "stage6-tests: FAIL git diff failed"; else
    for X in $D; do echo "changed $X"; done
    if [ "$N" -ge 1 ] && [ -z "$D" ]; then echo "stage6-tests: ok"; else echo "stage6-tests: FAIL"; fi
  fi
fi; } > scratch/s7-1-2a2.txt 2>&1
```
It lists every `tests/*.rs` file at S6 (the nine above included), and prints a `changed <file>` line for each one that differs at HEAD, deleted files included. Files added after S6, such as `tests/track.rs`, are not in the list. This file is read through `tr -d '\r'` here and in 1.3 (g), because crash-sim checkouts use `core.autocrlf`, so `docs/*.md` has CRLF lines on Windows. Git Bash's grep 3.0 drops the CR itself (tried on this machine), but a grep that keeps it would never match a pattern ending in `$` and would read the commit with a CR after it, so the check would fail on a correct file.
PASS iff the **last line** of `scratch/s7-1-2a2.txt` is exactly `stage6-tests: ok`, which is printed only when S6 is set on exactly one line to a commit that is an ancestor of HEAD, S6 has at least one test file, and none of them changed. An empty file, or one ending in a git error, fails. If a Stage 6 test must change during Stage 7, that is a finding for the orchestrator; S6 is never moved forward to hide it.

(b) Every Stage 6 crash-sim test still exists and passes, by name, in the 1.1 run. In Git Bash:
```
for n in a_second_solver_plugs_in beam_breaks_at_expected_substep beam_past_yield_keeps_longer_rest box_wall_variants_differ_in_one_number braked_car_holds_below_grip_slope braked_car_slides_above_grip_slope break_pull_files_differ_only_in_speed broken_beam_does_not_trip_strain_limit canonical_file_round_trips_byte_identical car_basic_round_trips car_settles_level cube_bsn_matches_default_cube cube_bsn_output_matches_default default_csv_matches_main_fixture default_cube_output_unchanged disconnected_node_falls_freely energy_never_rises_through_yield full_lock_circle_matches_kinematic_radius ground_plane_collider_keeps_cube_output launch_matches_torque_or_grip_limit pad_and_wall_car_is_car_basic pad_lap_closes_and_hits_wall planted_energy_injection_fails_settle same_input_same_bytes settle_holds_across_the_invariants_grid two_node_energy_matches_hand_value; do grep -qF "test $n ... ok" scratch/s7-1-1.txt && echo "ok $n" || echo "MISSING $n"; done > scratch/s7-1-2b.txt 2>&1
```
These are the 26 crash-sim test names in STAGE6.md checks 1.1, 2.1 and 3.1. The two render tests named there are checked by 1.7's render run: `box_wall_final_frame_matches_golden` and `crashed_car_final_frame_matches_golden`.
PASS iff `scratch/s7-1-2b.txt` has exactly 26 lines, all starting `ok `. Also, `scratch/s7-1-7.txt` (1.7) contains `test box_wall_final_frame_matches_golden ... ok` and `test crashed_car_final_frame_matches_golden ... ok`.

(c) The default cube and `cargo run` without `--bin`: `broadsword run crash-sim -- cargo run --offline --release -- --summary > scratch/s7-1-2c.txt 2>&1`
PASS iff `run: ok` and the program's output is exactly the one line `first_contact_s=0.6390,peak_strain=0.045146,final_center_y=0.4999`.

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
(a) `0.830 <= limit_g <= 0.930`;
(b) `limit_g` <= 1.02 x `mu`, limit(0.8) <= 1.02 x 0.8 x `mu`, and limit(1.2) <= 1.02 x 1.2 x `mu`;
(c) limit(0.8) < `limit_g` < limit(1.2), with limit(0.8) / `limit_g` in [0.72, 0.88] and limit(1.2) / `limit_g` in [1.08, 1.32];
(d) for each of the three runs: the `level` line of its `limit_level` says `steady=yes` and its `a_g` equals that run's `limit_g`; `limit_level` <= `levels` - 2; and `steady_levels` >= 5. Also, `radius_kinematic_m` is within 1% of 30.00 (29.70 to 30.30), and there are exactly `levels` `level` lines per run;
(e) no tuning file changed while the range could still move. Changing the range after tyre tuning has started needs Lazlo's yes, and a new `confirmed:` line is verified by a reviewer re-opening its URL (see (g)); (e) itself checks commit order only. Exactly what it proves: every commit that touches a tuning file is either strictly older than the commit that first wrote the range or strictly newer than the range's last edit, and the skidpad test is strictly newer than that last edit. Tuning-file commits older than the range pass on purpose: they were written before the range existed in the repository (nb-38's, nb-39's and nb-40's commits), so they cannot have been aimed at it. What (e) cannot rule out on its own is a range chosen afterwards to fit tyres that already existed; (g) rules that out, because the range's bounds must be the lowest and highest figures read from public sources, not a measured value. The tuning files are exactly these six crash-sim paths: `scenarios/car_basic.bsn` (its tyre section is the tyre data), `scenarios/pad_and_wall.bsn` (it embeds the same car), `scenarios/skidpad.bsn`, `scenarios/skidpad.input`, `src/vehicle.rs` (nb-39's tyre model) and `tests/skidpad.rs`. In Git Bash from the Broadsword root:
```
{ P=projects/crash-sim; T="scenarios/car_basic.bsn scenarios/pad_and_wall.bsn scenarios/skidpad.bsn scenarios/skidpad.input src/vehicle.rs tests/skidpad.rs"
R=$(git -C $P log --format=%H -S"0.830 <= limit_g <= 0.930" -- docs/STAGE7.md)
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
(g) the range rests on confirmed sources, and its bounds are their extremes. In Git Bash from the Broadsword root:
```
{ D=$(tr -d '\r' < projects/crash-sim/docs/STAGE7.md)
B=$(echo "$D" | grep -E '^- \*\*Target `limit_g` [0-9.]+ to [0-9.]+ g\*\*')
LAB=$(echo "$B" | grep -o '(\*[a-z, ]*\*)'); echo "label: $LAB"
L=$(echo "$D" | grep -E '^  confirmed: [0-9]+\.[0-9]+ g .+ \| .+ \| https?://[^ ]+ \| read [0-9]{4}-[0-9]{2}-[0-9]{2} UTC$')
[ -n "$L" ] && echo "$L"
N=$(echo "$L" | grep -c '^  confirmed: ')
CR=$(echo "$L" | grep -oE '^  confirmed: [0-9]+\.[0-9]+' | grep -oE '[0-9]+\.[0-9]+$' | sort -n | sed -n '1p;$p' | tr '\n' ' ')
BR=$(echo "$B" | grep -oE '^- \*\*Target `limit_g` [0-9.]+ to [0-9.]+ g' | grep -oE '[0-9.]+ to [0-9.]+')
AR=$(echo "$D" | grep -E '^\(a\) `[0-9.]+ <= limit_g <= [0-9.]+`;$' | grep -oE '[0-9.]+ <= limit_g <= [0-9.]+')
ER=$(echo "$D" | grep -oE '^R=\$\(git -C \$P log --format=%H -S"[0-9.]+ <= limit_g <= [0-9.]+"' | grep -oE '[0-9.]+ <= limit_g <= [0-9.]+')
echo "confirmed=$N confirmed-range: $CR"; echo "bullet-range: $BR"; echo "check-a: $AR"; echo "check-e: $ER"
if [ "$LAB" = "(*sourced*)" ] && [ "$N" -ge 1 ] && echo "$CR $BR $AR $ER" | awk 'NF==15 && $1+0==$3+0 && $2+0==$5+0 && $6+0==$3+0 && $10+0==$5+0 && $11==$6 && $15==$10 {f=1} END {exit !f}'; then echo "source: ok"; else echo "source: FAIL"; fi; } > scratch/s7-1-3-source.txt 2>&1
```
What it prints: `label: <label>`, every well-formed `confirmed:` line, `confirmed=<n> confirmed-range: <lowest> <highest>`, `bullet-range: <lo> to <hi>` (the Target bullet's range), `check-a: ...` and `check-e: ...` (the range written in (a) and in (e)'s `-S` string), and a last line `source: ok` or `source: FAIL`.
PASS iff the **last line** of `scratch/s7-1-3-source.txt` is exactly `source: ok`. That line is printed only when the Target bullet occurs exactly once and is labelled exactly `(*sourced*)`, there is at least one `confirmed:` line with a figure in g, a title, an `http` or `https` URL and a read date, the bullet's lower and upper bounds equal the lowest and highest confirmed figures, and (a) and (e) carry the same bounds as the bullet (the awk rule fails if any of the four ranges is missing or occurs twice, because the field count is then not 15). This gate checks form and consistency only; it never opens a URL. Verifying a source is a review step (see 'Sources are verified by review' under the Target bullet in the numbers section): the person who adds or changes a `confirmed:` line gives the URL, a reviewer re-opens it before the range can change, and any range change after tyre tuning has started needs Lazlo's yes. The orchestrator opens each URL once when the evidence is written and records in the evidence that it states the figure quoted; a URL that does not, fails this check. A label `(*unverified, from memory*)`, a missing bullet (`label: `), a missing `confirmed:` line, or a range that is not the confirmed extremes, fails. If a new confirmed figure changes the extremes, the bullet, (a) and the `-S` string of (e) change in the same commit.
Record `limit_g`, the limit level's `speed_kmh` and `radius_m`, and `limit_balance` in the evidence, and the `confirmed:` lines of (g).
FAIL: any of (a) to (g), or a missing line.

**1.4 Headless: a timed lap of the oval** (nb-47).
`broadsword run crash-sim -- cargo run --offline --release -- --body scenarios/car_basic.bsn --track scenarios/tracks/oval.track --input scenarios/tracks/oval_lap.input > scratch/s7-1-4.txt 2>&1`
`scenarios/tracks/oval_lap.input` (nb-47) drives from the grid over the line, two full laps (the line is crossed at the start and at the end of each lap), and then brakes to a stop.
PASS iff `run: ok` and the output contains:
- the line `track name=oval points=64 checkpoints=3 centre_length_m=388.41 width_min_m=12.00 width_max_m=12.00`;
- exactly one `start ` line, and before the first `lap ` line the three lines `checkpoint lap=1 k=1`, `k=2` and `k=3` in that order, with increasing `t`, each greater than the `start` time;
- a line `lap n=1 ...` and a line `lap n=2 ...`, each `valid=yes reason=none`, lap 1's `start_t` equal to the `start` line's `t` and lap 2's `start_t` equal to lap 1's `end_t`, and for each lap, with `off_track_s=0.000`, `path_m` >= 338.67, `time_s` x `max_speed_kmh` / 3.6 >= `path_m` - 2, and |`time_s` - (`end_t` - `start_t`)| <= 0.0006 (the test `oval_lap_is_valid_and_above_floors` checks the last one to the exact integer millisecond);
- a last line `race: laps_valid=2 laps_invalid=0 best_lap_s=<the smaller of the two time_s>`.
Record both laps' `time_s` in the evidence for milestone 2.
FAIL: `run: FAILED`; a missing or different `track` line; checkpoint lines missing or out of order; lap 1 or lap 2 invalid; a path below 338.67 m (a lap counted without driving round it on track); a timing inconsistency (the timer disagrees with the motion); or any invalid lap.

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
`broadsword run crash-sim -- cargo build --offline --release > scratch/s7-1-8-build.txt 2>&1` (from the main checkout, so `build/release/crash-sim.exe` is built from crash-sim main), then
`broadsword run render -- cargo run --offline --release --example drive -- --sim-exe ../../build/release/crash-sim.exe --body ../crash-sim/scenarios/car_basic.bsn --track ../crash-sim/scenarios/tracks/oval.track --save-input ../../scratch/s7-1-8.input > scratch/s7-1-8.txt 2>&1`
The viewer starts `crash-sim.exe --live` with the same `--body` and `--track` as a child process, draws the track edges, the start line and the checkpoints from the `track_line` lines, and shows the current lap time and the last valid lap time in seven-segment digits (`-:--.---` until there is one). It forwards every race line from the child (the `track`, `start`, `checkpoint`, `lap`, `final` and `race:` lines; not the `track_line` lines or trajectory rows) to its own stdout unchanged. When the window closes it closes the child's stdin. The child ends the run at the end of the substep it is in when it reads end of input; call that time T_end (a whole substep, so a whole millisecond at the default step). It prints its `final` line with `t=` T_end and its `race:` line, and exits. The viewer then saves the inputs (nb-40's `--save-input`) with one more line at the end, the **end line**: `<T_end> <throttle> <brake> <steer>`, where `<T_end>` is the `t=` text of the child's `final` line copied character for character (4 decimals, never re-formatted from a float), and the three values are the ones in force at T_end (the previous saved line's values, repeated, so the end line changes no input). If the last saved line's time text is already T_end, no end line is added, because that line already ends a replay at T_end. No saved line has a time after T_end: an input the child never applied is not saved. nb-40's file rule then ends a replay at that line's time, which is T_end. If the child's `final` line never arrives (the child crashed), the viewer writes no end line and says so in a `saved input: ... no end line (child gave no final line)` message, and the automatic part of 1.8 fails. After saving, the viewer prints `saved input: ../../scratch/s7-1-8.input` and `hud current=<text> last=<text>`, with exactly the two strings on screen in the last frame. The `hud` line prints the same two string values that were passed to the drawing call; 1.7's golden test covers the drawing, and Q3 below is the human check of what was on the screen. Lazlo drives with the arrow keys or WASD, from the grid, over the line, at least one full lap and over the line again, then closes the window. He may try as often as he likes; each attempt is a new run of the command, and the evidence keeps the last one and counts them.
Automatic part. PASS iff `scratch/s7-1-8-build.txt` shows `run: ok`, and `scratch/s7-1-8.txt` contains a `lap ... valid=yes` line, the `final` line, the `saved input:` line (without `no end line`), and a `hud` line whose `last=` text is the `M:SS.mmm` form of the last valid lap's `time_s`; and the end line is in place: in Git Bash from the Broadsword root, `{ echo "final_t=$(grep -E '^final ' scratch/s7-1-8.txt | sed -E 's/^final t=([^ ]+) .*/\1/')"; echo "input_end_t=$(tail -n 1 scratch/s7-1-8.input | cut -d ' ' -f 1)"; } > scratch/s7-1-8-end.txt 2>&1` writes two lines whose values after `=` are the same non-empty text (for example `final_t=95.4120` and `input_end_t=95.4120`).
Human part, recorded by the orchestrator in `evidence/stage-7-m1-human.md` (nb-49) before scratch can be wiped. It holds: the UTC date and time; the number of attempts; the `start`, `checkpoint`, `lap`, `final`, `saved input` and `hud` lines of the last attempt, pasted; Lazlo's answers to three questions, quoted verbatim from his own message: Q1 "Did the car respond to the keys?", Q2 "Did the timer on screen run during the lap and show the lap time when you crossed the line?", Q3 "Was the time on screen the same as the printed `lap` line?"; and his answer to "What felt wrong, or not like driving a car?", quoted verbatim, with one board task id or `IDEAS.md` line per point he raises. The orchestrator copies `scratch/s7-1-8.input` to `evidence/stage-7-m1-human.input`.
PASS iff the automatic part passes and Q1, Q2 and Q3 are each answered yes in Lazlo's own words. An agent never answers for him, and a paraphrase does not count. The "felt wrong" points are not gated, but each one must have its task or IDEAS line before the milestone is tagged.
FAIL: no window, no key response, no saved input, no valid lap, a `hud` text that differs from the printed lap time, any answer other than yes, or a missing record.

**1.9 The human lap replays headless** (nb-47, nb-49).
`broadsword run crash-sim -- cargo run --offline --release -- --body scenarios/car_basic.bsn --track scenarios/tracks/oval.track --input ../../evidence/stage-7-m1-human.input > scratch/s7-1-9.txt 2>&1`, then, in Git Bash from the Broadsword root:
`diff <(grep -E '^(track name=|start |checkpoint |lap |final )' scratch/s7-1-8.txt) <(grep -E '^(track name=|start |checkpoint |lap |final )' scratch/s7-1-9.txt) > scratch/s7-1-9-diff.txt 2>&1`
How the close time reaches the replay: the live run ended at T_end when the viewer closed the child's stdin, and 1.8 saved T_end as the time of the input's last line (the end line), copied from the live `final` line's `t=` text. The file run has no `--duration`, so by nb-40's rule it ends at that last line's time, which is the same T_end, and its `final` line is printed at the same substep. Before the diff, in Git Bash from the Broadsword root: `{ echo "live_final_t=$(grep -E '^final ' scratch/s7-1-8.txt | sed -E 's/^final t=([^ ]+) .*/\1/')"; echo "input_end_t=$(tail -n 1 evidence/stage-7-m1-human.input | cut -d ' ' -f 1)"; echo "replay_final_t=$(grep -E '^final ' scratch/s7-1-9.txt | sed -E 's/^final t=([^ ]+) .*/\1/')"; } > scratch/s7-1-9-end.txt 2>&1`
PASS iff `run: ok`; `scratch/s7-1-9-end.txt` has three lines whose values after `=` are the same non-empty text (the live close time, the saved end line's time and the replay's end time agree); `scratch/s7-1-9.txt` contains a `lap ... valid=yes` line (so the comparison is not of two empty sets); and `scratch/s7-1-9-diff.txt` is empty: the same lines, character for character, live and headless. A replay whose `final` line differs only because the saved input has no end line fails the first rule, and points at the viewer, not the physics. FAIL: any difference (the live run and the file run are not the same simulation, so a human lap cannot be checked or replayed).

What guards against tuning to the test in milestone 1: the grip target's bounds must be the lowest and highest figures confirmed from public sources with their URLs and read dates (1.3 g) and every tyre-file commit in Git is either older than the range or newer than its last edit, with the skidpad test newer than that edit (1.3 e); a physics ceiling and the grip variants must agree with friction (1.3 b, c); the skidpad measures motion, not tyre forces, and only over 5 s windows of steady cornering (1.3 d); the lap is checked against a geometric floor and a timing identity that do not use `LapTimer` (1.4); the pinch shows that `on_track` is applied and that the track does not change the physics (1.5); the lap must survive a different step size (1.6); the HUD and the printed time come from one integer (1.7); and a human has to drive it, and the drive must replay exactly (1.8, 1.9).

After 1.1 to 1.9 pass, the Stage 7 m1 retro entry in `RETRO.md` is due (AGENTS.md); nb-49 lists its inputs in `evidence/stage-7-m1.md`. The Stage 7 lesson goes in it: what the human check found that the tests did not.

## Milestones 2 and 3: exact checks

Written on crash-sim `0bb2b5e` (board task `t-20261121-001109-6`, brief n20-16). What exists on that commit, read from the files: the AI driver `src/ai.rs` (n20-17), the speed profile `src/speed_profile.rs` (n20-18), the track and lap timer `src/track.rs` (nb-43), the settings file `src/settings.rs` (n20-21), the save file `src/save.rs` (n20-22), the menu state machine `src/menu.rs` (n20-23), all registered in `src/lib.rs` (the state hash, the message codec and the lockstep session are there too, for milestone 4). What does not exist yet, and so is only *specified* here: several cars in one sim (n20-19), the race program with AI cars (n20-20), the lap lines of a headless `--track` run (n20-13), and the game front end and release folder (n20-26 and the proposed n20-26a). Every flag and file in a command below that is not in `src/main.rs` or the repository today is named in "Who builds what, milestones 2 and 3" with the brief that creates it; if a flag is in no row, the check is wrong, not the brief.

**Which program.** Decided by the orchestrator (2026-11-21), under its standing authority for reversible choices, not by Lazlo: the race is the default `crash-sim` program, not a second `--bin race`. The board brief n20-13 (`t-20261121-001109-5`) is built that way (it claims `src/main.rs` and adds `--track`), and milestone 1 above now says so; n20-20 also claims `src/main.rs`, after n20-13. Every command below uses `cargo run --offline --release -- ...` and the copied program `crash-sim.exe`, and assumes n20-13 prints the lap lines that milestone 1 specifies (`track`, `start`, `checkpoint`, `lap`, `final`, `race:`) for a `--track` run with `--input`. n20-13 also includes milestone 1's `--live` race (`crash-sim --live --track`, with `track_line` lines), by the orchestrator's note in its brief; it is built and in review, and n20-14 and n20-26 depend on it.

**When a check runs a built program directly.** Checks that need a pipe into stdin or a clean folder (m3; milestone 4 also needs two processes at once) cannot run through `broadsword run`: a `run` holds the build lock to its end (`build_lock_wait_seconds` in `broadsword.toml`), so a second concurrent `run` waits or is refused. Those checks first build with `broadsword run crash-sim -- cargo build --offline --release` and copy `build/release/crash-sim.exe` to `scratch/` in the same command block, and then run the copy. They never call cargo. The copy narrows a race and does not close it: another job's clean of the shared build folder can still delete the release program between the build and the `cp`, which is what happened on 2026-11-20 (a person-run `crash-sim.exe` was deleted one second after it was built). That race is closed only by the main.bins change (board task `t-20261120-191700`, running when this was written) and its follow-up for render's lookup of the program; until both are merged, a check whose `cp` finds no file fails (the `copied` line is missing) and is run again, and nothing in this file claims that the problem is fixed.

All numbers marked *reasoned* are derived in the text that gives them. Any number that a brief must measure against a reasoned bound follows milestone 1's rule: if the measured value is outside the bound, the brief stops and reports; it never moves the bound.

**Reading test names.** A check that says "the existing tests of file X still pass" reads the names from the test file, so the list cannot drift from the code. Save this as `scratch/s7-named.sh` and `source` it in Git Bash from the Broadsword root. `named <cargo test output> <test file stem> <least number of tests>` finds every `#[test]` function of `projects/crash-sim/tests/<stem>.rs` (other attribute lines between `#[test]` and `fn`, such as `#[should_panic]`, are skipped), looks for `test <name> ... ok` only in that file's own section of the output (the lines after its `Running tests\<stem>.rs` line, up to the next `Running` line, so a test of the same name in another file cannot stand in for it), prints `MISSING <stem> <name>` for each one not there, and a last line `named <stem>: ok (...)` or `named <stem>: FAIL (...)`. The number is what makes an empty or renamed file fail instead of passing with nothing to look for. The counts used in this file are the counts on crash-sim `0bb2b5e`: `ai` 14, `speed_profile` 10, `settings` 9, `save` 13, `menu` 8. It was run on a synthetic output with `Running` headers (all names present: `ok`; one name removed: `MISSING` and `FAIL`; an empty file: `FAIL`; the same name present only in another file's section: `MISSING`; a test with `#[should_panic]` between `#[test]` and `fn`: found).

```
named() {
  out=$1; f=$2; want=$3; ok=0
  sec=$(tr -d '\r' < "$out" | awk -v f="$f" '/Running /{ on = (index($0, "tests\\" f ".rs") > 0 || index($0, "tests/" f ".rs") > 0) } on')
  for n in $(awk '/^#\[test\]/{t=1; next} t && /^#\[/{next} t && /fn [a-z0-9_]+/{match($0, /fn [a-z0-9_]+/); print substr($0, RSTART+3, RLENGTH-3); t=0}' projects/crash-sim/tests/$f.rs); do
    if echo "$sec" | grep -qF "test $n ... ok"; then ok=$((ok+1)); else echo "MISSING $f $n"; fi
  done
  if [ "$ok" -ge "$want" ]; then echo "named $f: ok ($ok found, at least $want needed)"; else echo "named $f: FAIL ($ok found, at least $want needed)"; fi
}
```

**Scratch file names.** Each check writes only files that carry its own number, so one check never overwrites another's evidence. The exceptions are stated where they occur: 2.4 reads 2.2's input scripts, and 3.5 reads 3.2's `s7-3-2-settings.want`. The prefixes under `scratch/`:

| Check | Files it writes |
|---|---|
| 2.1 | `s7-2-1.txt`, `s7-2-1-named.txt` |
| 2.2 | `s7-2-2.txt`, `s7-2-2-verdict.txt`, `s7-2-in<k>.input`, `s7-2-verify.sh`, `s7-2-inputs.sh` |
| 2.3 | `s7-2-3.txt`, `s7-2-in-t<k>.input` |
| 2.4 | `s7-2-4a*.txt`, `s7-2-4-race<k>.lines`, `s7-2-4-replay<k>.txt` and `.lines`, `s7-2-4b.txt`, `s7-2-4c<k>.txt` |
| 2.5 | `s7-2-5.txt`, `s7-2-in-b<k>.input`, `s7-2-2.txt.prog`, `s7-2-5.txt.prog`, `s7-2-5-cmp.txt` |
| 2.6 | `s7-2-6.txt`, `s7-2-in-s20<k>.input`, `s7-2-6-verdict.txt`, `s7-2-6-best*.txt`, `s7-2-6-cmp.txt` |
| 3.1 | `s7-3-1.txt`, `s7-3-1-render.txt`, `s7-3-1-py.txt`, `s7-3-1-named.txt` |
| 3.2 | `s7-3/`, `s7-3-2*`, `s7-3-a.events`, `s7-3-b.events`, `s7-3-data/` |
| 3.3 | `s7-3-3*`, `s7-3-bad/`, `s7-3-bad-*.orig`, `s7-3-c.events` |
| 3.4 | `s7-3-4-build.txt`, `s7-3-4-check.txt`, `s7-3-release/` |
| 3.5 | `s7-3-5*`, `s7-3-clean/`, `s7-3-a.script`, `s7-3-b.script` |

Input prefixes of `--save-inputs` (the one place where two checks used one name before this table): `s7-2-in` (2.2), `s7-2-in-t` (2.3), `s7-2-in-b` (2.5), `s7-2-in-s20` (2.6). Each is followed by the car number and `.input`, so none is a prefix-collision of another (`s7-2-in-b1.input` is not `s7-2-in1.input`).

### Milestone 2: three AI cars finish a race without crashing into walls

**The race.** `crash-sim` runs `--ai 3` cars on `scenarios/tracks/oval.track` for 3 timed laps each. Each car starts from rest on a single-file grid on the centreline: slot k (1, 2, 3) is `grid + (k - 1) x grid_gap` metres before the start line, `grid_gap` is 10 m, so slot 3 is 40 m before the line, on the 100 m front straight (point 63, 50 m before the line, is the start of the straight). Every car is `scenarios/car_basic.bsn`. A car's laps are timed by its own `LapTimer` (nb-43), fed the chassis centre of mass at every substep as in "Shared definitions". The race ends when every car has finished its lap 3, or at the time limit. The cars keep driving after they finish, until the race ends, so a finished car is still an obstacle and a wall risk.

**What an AI sees and can do.** The driver is `ai::Driver::update` (n20-17), called once per 0.02 s of simulation time (50 Hz, the rate of the human viewer's input: `INPUT_HZ` in `projects/render/examples/drive.rs`), with the car's own state (its centre-of-mass position, heading and speed, wheelbase and maximum steer angle), the track centreline and a target speed per centreline point from `speed_profile::profile` (n20-18). The answer is throttle, brake and steer, held for the 0.02 s. It cannot see the other cars or the walls, so it does not overtake: with one speed profile and one driver for all three, the cars keep their grid gaps (that is why the grid is staggered, and why contact stays at zero without any sensing). It cannot reach the car by any other route: the only call that moves a car is the sim step with those three numbers, and 2.4 proves it with a replay.

**Car-to-car contact: allowed in the physics, required to be zero in the race.** n20-19 gives the sim car-to-car contact, because a racing game whose cars pass through each other is not the "crash sim grows into a racing game" of DEVELOPMENT_PLAN. The m2 race still requires none: "without crashing" is read to include each other, and with zero contacts a car's trajectory in the race must equal its trajectory driving alone (2.4). Ghost cars would hide a collision that the checks are meant to find. A contact test in n20-19 shows the physics exists; the race shows the AI does not use it.

**What counts as a wall contact.** The track has no physical walls in this milestone, as in milestone 1 (the ground is the same plane everywhere). An edge of the track is where a wall would stand, and the first part of a car to touch a wall is the first part to cross the edge. So, at every substep, a car has a wall contact if any chassis node, or the outer edge of any tyre, is off the track by `Track::on_track` (the distance from the point to the centreline is more than half the track width there). A tyre's outer edge is its mount node moved 0.0975 m (half the wheel width 0.195 m in `car_basic.bsn`) away from the car's centreline, sideways in the ground plane. On `car_basic.bsn` the outermost chassis nodes (z = 0.9 m) are outside the tyre edges (0.75 + 0.0975 = 0.8475 m), so the nodes decide there; the tyre clause is in force for other bodies and is tested on a synthetic one (`wall_rule_counts_tyre_outer_edges`). Finite wall colliders along the edges are a later idea, not part of this milestone.

**The numbers, and why.**

- **3 cars, 3 laps.** DEVELOPMENT_PLAN says three AI cars. Three laps show that the AI repeats a lap, not that one lap went well. Lap 1 starts at the car's first crossing of the line, so the grid run-up is not a lap.
- **A lap of at most 48.6 s: a mean speed of at least 8 m/s over the centreline.** 388.41 m / 8 m/s = 48.55 s. 8 m/s is half of the 15.6 m/s bend speed that milestone 1 reasons for the oval (sqrt(0.83 x 9.81 x 30) = 15.6 m/s at the lowest sourced limit). A car slower than that on average is crawling, which would pass "no wall contact" trivially. The bound does not depend on the AI's own numbers.
- **Time limit 160 s.** 3 laps at 48.6 s is 145.8 s, plus the run-up of the farthest slot, 40 m at 8 m/s is 5 s: 150.8 s, rounded up to 160 s. A car that has not finished by then fails.
- **Path at least 338.67 m, timing identity, `off_track_s` 0.000, every lap valid.** These are milestone 1's lap rules (1.4), applied to every lap of every car, so an AI cannot count a lap by cutting a corner.
- **Centres at least 4.40 m apart at every substep.** The body's nodes are within 2.193 m of its centre of mass in the ground plane: sqrt(2.0^2 + 0.9^2) = 2.193 m for the corner nodes of `car_basic.bsn` (x = -2 to 2, z = -0.9 to 0.9), and the mass-weighted centre is at x = 0.0000, z = 0.0000 (computed from the file's node masses). Two bodies whose centres are more than 2 x 2.193 = 4.386 m apart cannot touch; 4.40 m rounds up. The race reports the smallest centre distance and the number of substeps under 4.40 m. This is an independent measure, not n20-19's contact rule.
- **Grid gap 10 m.** 2.5 car lengths (the car is 4.0 m long), and the room for slot 3 on the straight. This is a starting value, not a derived one. All three cars launch at t = 0, so the gap changes over the first seconds, and the AI cannot see the other cars to control it; a rough argument is that with one profile the follower tends to keep a time headway behind the leader (it reaches each point later), so the distance between them grows with speed and is smallest where the speed is lowest. That argument is *reasoned, not measured*, and nothing in the checks relies on it: the 4.40 m floor is a measured rule (2.2 reads `gap_min_m` from the race), and a race that goes under it fails. The first measured `gap_min_m` and `gap_min_t` show where it is smallest; if the gap is the problem, n20-20 reports it and the grid gap is changed here, never the 4.40 m.
- **AI cadence 50 Hz and the file `scenarios/ai_driver.params`.** The file holds the driver's six `DriverParams` values and the profile's `mu`, `g`, `a_accel`, `a_brake` and `v_max`, one `key value` per line after a first line `ai 1`, strict like `.bsn` (a missing, repeated or unknown key is refused with the line number; nothing is defaulted). n20-20 chooses the values and writes a reason beside each. `v_max` and `a_accel` must not be above what the car does with the rolling resistance and aero drag that crash-sim `4fd84b0` added (`docs/PHYSICS.md` lists both): a profile above the car's real top speed asks for speeds it cannot reach, and the speed controller then saturates; n20-20 takes both from the speed and acceleration of the open-loop lap of 1.4 and names that evidence in the file. `mu` must stay at or below 0.9 x the `limit_g` that check 1.3 records (a profile that asks for the whole measured limit leaves no margin, and the AI is then the thing that gets tuned to the wall); this is a review step, not machine-checked, and the file's comment names the evidence line it took `limit_g` from.

**Flags and output (n20-20).** `crash-sim --body <car.bsn> --track <file> --ai <n> --laps <N> --time-limit <s> --grid-gap <m> --ai-params <file> [--save-inputs <prefix>] [--substeps <n>]`. The required flags, exactly: `--body`, `--track`, `--ai`, `--laps`, `--time-limit`, `--grid-gap` and `--ai-params`; a missing one is a usage error (exit 2). The optional flags are `--save-inputs` and `--substeps`. `--ai` is 1 to 3, `--laps` 1 to 10, and the other numbers are above zero; `--ai` cannot be combined with `--input`, `--live` or any report flag. `--save-inputs <prefix>` writes `<prefix>1.input`, `<prefix>2.input`, ... one input script per car, in the script format of `src/input.rs`: one line `t throttle brake steer` every 0.02 s from `0.000`, `t` written from whole milliseconds as `<ms / 1000>.<ms % 1000, 3 digits>`, and the three values written with Rust's `{}` for `f64` (shortest text that parses back to the same number, so a replay gets the very same inputs); a last line at the race's `end_t`, written from whole milliseconds with 3 decimals like every other line (`end_t` is a whole number of milliseconds, so its 4-decimal text in the race line ends in 0 and is the same number), repeating the values in force, unless the last driver line is already at that time as a number, when none is added (the rule of 1.8, so a replay ends at the same substep). The single-car mode of n20-13 gets one more flag, `--grid-slot <k>` (1 to 3, needs `--grid-gap`): the car is placed in slot k; without it, slot 1, which is milestone 1's placement. Exit codes: 0 when the last line is `race: ok`, 1 when it is `race: FAIL ...` (a rule broken, a time limit, or `Sim::check()` failing), 2 for a usage error or a refused file (line number in the message, empty stdout).

```
race track=<stem> cars=<n> laps=<N> time_limit_s=<3 decimals> grid_gap_m=<2 decimals> ai_params=<file stem>
profile lap_s=<3 decimals> v_min_kmh=<1 decimal> v_max_kmh=<1 decimal>
car <k> start t=<4 decimals>
car <k> checkpoint lap=<n> k=<1..> t=<4 decimals>
car <k> lap n=<n> start_t=<..> end_t=<..> time_s=<..> valid=<yes|no> reason=<..> path_m=<..> max_speed_kmh=<..> off_track_s=<..>     (the fields of milestone 1's lap line)
car <k> finish t=<4 decimals> place=<1..n>              the end of that car's lap N; place by finish time
car <k> walls substeps=<count> first_t=<4 decimals, or none>
car <k> final t=<4 decimals> x=<4 decimals> z=<4 decimals> heading_deg=<2 decimals>     milestone 1's final line at the end of the race
cars gap_min_m=<2 decimals> gap_min_t=<4 decimals> near_substeps=<count>
race: ok cars=<n> finished=<n> wall_substeps=<sum> near_substeps=<count> end_t=<4 decimals>
```

The fields after `car <k> ` in `start`, `checkpoint`, `lap` and `final` lines are exactly milestone 1's lines (same fields, same formats), so a car's lines in the race and the lines of a single-car run of the same inputs can be compared character for character. `profile lap_s` is the ideal lap time of the profile, the sum of ds / v over the centreline, recorded and not gated. `race: ok` is printed iff every car has finished lap N, `wall_substeps` is 0 and `near_substeps` is 0; otherwise the last line is `race: FAIL <the broken rules with their numbers>`. A verdict printed by the program is not trusted: 2.2 re-checks every rule from the per-car lines.

#### Checks

**2.1 Tests.** `broadsword test crash-sim > scratch/s7-2-1.txt 2>&1`
PASS iff the last line is `test: ok — N passed, 0 failed`, N is at least the N recorded for check 1.1 in `evidence/stage-7-m1.md` plus the 13 tests named here, and the file contains each of:
`Running tests\multi_car.rs`, then `test one_car_matches_the_single_car_run_byte_for_byte ... ok`, `test two_cars_driving_apart_never_interact ... ok`, `test head_on_at_low_speed_conserves_momentum ... ok`, `test car_contact_never_creates_energy ... ok`, `test multi_car_same_inputs_same_bytes ... ok` (n20-19; the momentum and energy tolerances are the brief's, stated in `docs/PHYSICS.md`);
`Running tests\ai_race.rs`, then `test three_ai_cars_finish_three_laps_on_oval ... ok` (runs the 2.2 race in-process and asserts its rules), `test ai_race_same_input_same_bytes ... ok`, `test ai_inputs_replay_one_car_alone ... ok` (the 2.4 replay of car 2), `test tight_track_reports_wall_contacts ... ok` (the 2.3 run), `test oval_tight_differs_only_in_width ... ok`, `test wall_rule_counts_tyre_outer_edges ... ok` (a synthetic body whose tyres are outboard of its nodes: nodes inside the edge, a tyre edge outside, one wall contact), `test near_rule_counts_centres_under_4_4_m ... ok`, `test ai_race_refuses_bad_flags ... ok` (each of `--ai 0`, `--ai 4`, `--laps 0`, `--time-limit 0`, `--grid-gap 0`, a missing `--ai-params` file and a bad line in it: exit 2, empty stdout, the line number named) (n20-20);
and every test of `tests\ai.rs` (14 on this commit) and `tests\speed_profile.rs` (10), found by the helper under "Reading test names": `{ source scratch/s7-named.sh; named scratch/s7-2-1.txt ai 14; named scratch/s7-2-1.txt speed_profile 10; } > scratch/s7-2-1-named.txt 2>&1`; its output has `named ai: ok` and `named speed_profile: ok` and no `MISSING` line.
FAIL: any failed or missing test, or a smaller N.

**2.2 Three AI cars finish three laps without a wall or a car.** Build and run, from the Broadsword root:
`broadsword run crash-sim -- cargo run --offline --release -- --body scenarios/car_basic.bsn --track scenarios/tracks/oval.track --ai 3 --laps 3 --time-limit 160 --grid-gap 10 --ai-params scenarios/ai_driver.params --save-inputs ../../scratch/s7-2-in > scratch/s7-2-2.txt 2>&1`
The orchestrator saves the block under "The m2 verifier" below as `scratch/s7-2-verify.sh`, then runs `bash scratch/s7-2-verify.sh scratch/s7-2-2.txt > scratch/s7-2-2-verdict.txt 2>&1`.
PASS iff `scratch/s7-2-2.txt` contains `run: ok` and the last line of `scratch/s7-2-2-verdict.txt` is exactly `m2-race: ok`. The verifier prints `cars_checked=3 laps_checked=9` and a `FAIL <rule>: <line>` line for every rule broken; it re-derives every rule from the per-car lines and does not read the program's own verdict except to require that it says `race: ok cars=3 finished=3 wall_substeps=0 near_substeps=0 end_t=...`. Its rules, in order: the `race` header line is exactly `race track=oval cars=3 laps=3 time_limit_s=160.000 grid_gap_m=10.00 ai_params=ai_driver`; each car has exactly one `start`, `finish`, `walls` and `final` line and, for each of laps 1 to 3, exactly one `lap` line and exactly three `checkpoint` lines (checkpoint times increasing); every `lap` line is `valid=yes reason=none`; for laps 1 to 3 also `off_track_s=0.000`, `path_m` >= 338.67, `time_s` <= 48.6, `time_s` x `max_speed_kmh` / 3.6 >= `path_m` - 2 and |`time_s` - (`end_t` - `start_t`)| <= 0.0006; each car's `finish t` is the text of the `end_t` of its lap 3 and is at most 160; the places are 1, 2, 3 once each, in finish-time order; every `walls` line is `substeps=0 first_t=none`; the `cars` line has `gap_min_m` >= 4.40 and `near_substeps=0`.
Record `profile lap_s`, each car's best lap and `gap_min_m` in the evidence, next to lap 1's `time_s` of check 1.4 (the open-loop script's lap). They are recorded, not gated.
FAIL: `run: FAILED`; any `FAIL` line; a missing car; `race: FAIL`.

**2.3 The wall counter can fail.** Same command with `--track scenarios/tracks/oval_tight.track --ai 1 --laps 1` and its own input prefix, `--save-inputs ../../scratch/s7-2-in-t` (the other flags unchanged; the prefix of 2.2 would overwrite the files that 2.4 replays), saved as `scratch/s7-2-3.txt`; `scenarios/tracks/oval_tight.track` (n20-20) is `oval.track` with the width changed to 1.6 m, which is less than the car's 1.8 m, so no path of the car can stay inside.
PASS iff `scratch/s7-2-3.txt` contains `run: FAILED (exit 1)`, a line `car 1 walls substeps=N first_t=T` with N at least 1 and T a number, and a last program line that starts `race: FAIL` and names `wall_substeps`. The run must fail for walls, not for something else: a last line that says `unstable` fails this check.
FAIL: `run: ok`, `walls substeps=0` or `first_t=none` (the wall rule does not see a car outside the edge, so 2.2's zero means nothing).

**2.4 The AI uses only throttle, brake and steer.** Three parts.
(a) The driver's source cannot reach the sim: in Git Bash from the Broadsword root, `grep -nE '^use |crate::|crash_sim::|\bSim\b|\bState\b' projects/crash-sim/src/ai.rs > scratch/s7-2-4a.txt 2>&1; grep -c 'pub struct Controls' projects/crash-sim/src/ai.rs > scratch/s7-2-4a-control.txt`. PASS iff `scratch/s7-2-4a.txt` is empty (the module has no top-level `use`, names no other module and no sim type; the `use super::*;` of its own test module is indented and does not match) and `scratch/s7-2-4a-control.txt` is `1` (the pattern is aimed at the right file: `Controls`, the type the driver answers with, is declared there once). It is empty on this commit: the same `grep` printed nothing. This is a cheap screen only: a path through `super::` or `self::` is not caught by it (the test module's own `use super::*;` is why those two are not in the pattern), so 2.4 (b) is the real proof that nothing but the three numbers moves a car.
(b) What the AI did is replayable as an input script, and a car driving alone with that script is the same car. Run, from the Broadsword root, for k in 1 2 3:
`broadsword run crash-sim -- cargo run --offline --release -- --body scenarios/car_basic.bsn --track scenarios/tracks/oval.track --input ../../scratch/s7-2-in<k>.input --grid-slot <k> --grid-gap 10 > scratch/s7-2-4-replay<k>.txt 2>&1`
then, in Git Bash:
```
{ for k in 1 2 3; do
  tr -d '\r' < scratch/s7-2-2.txt | grep -E "^car $k (start|checkpoint|lap|final) " | sed -E "s/^car $k //" > scratch/s7-2-4-race$k.lines
  tr -d '\r' < scratch/s7-2-4-replay$k.txt | grep -E '^(start|checkpoint|lap|final) ' > scratch/s7-2-4-replay$k.lines
  n=$(grep -c . scratch/s7-2-4-race$k.lines); echo "car $k race_lines=$n"
  if [ "$n" -ge 14 ] && cmp -s scratch/s7-2-4-race$k.lines scratch/s7-2-4-replay$k.lines; then echo "car $k replay: identical"; else echo "car $k replay: FAIL"; fi
done; } > scratch/s7-2-4b.txt 2>&1
```
PASS iff each replay's output contains `run: ok`, and `scratch/s7-2-4b.txt` has `car 1 replay: identical`, `car 2 replay: identical` and `car 3 replay: identical` and no `FAIL`. A race car has at least 14 lines (a start, 9 checkpoints, 3 laps, a final), so an empty comparison cannot pass. If the replay equals the race, nothing moved the car but those inputs, and no car touched another.
(c) The saved inputs are what a human could have typed. In Git Bash, for k in 1 2 3, with `E` the `end_t=` text of the `race: ok` line of `scratch/s7-2-2.txt`: `bash scratch/s7-2-inputs.sh scratch/s7-2-in<k>.input "$E" > scratch/s7-2-4c<k>.txt 2>&1`, where `scratch/s7-2-inputs.sh` is the second block under "The m2 verifier". PASS iff each output ends with `inputs: ok`: four fields on every line; the first time `0`, times increasing; throttle and brake in 0 to 1; steer in -1 to 1; never throttle and brake together (the driver cannot give both: it makes them from one signed number `u`, `(throttle, brake) = (u.min(1.0), 0.0)` if `u >= 0.0` and `(0.0, (-u).min(1.0))` otherwise, `src/ai.rs:185` on crash-sim `8fc0bab`, and its `STOP` is throttle 0, brake 1, `src/ai.rs:34`); at least 1000 lines; the last line's time equals `E` as a number (the file has 3 decimals and the race line 4, so they are compared as numbers, not as text; the 1.8 rule that adds no end line when the last driver line is already at that time makes the two meet). A human may also give a negative throttle (reverse, crash-sim `15499f4`: throttle -1 to 1); the driver of n20-17 never asks for it (its throttle is 0 to 1, said in `ai.rs`), so a negative throttle in a saved script fails here.
FAIL: a match in (a); a difference or an empty side in (b); any `FAIL` in (c).

**2.5 The same race twice gives the same bytes.** The 2.2 command again with `--save-inputs ../../scratch/s7-2-in-b`, saved as `scratch/s7-2-5.txt`, then in Git Bash: `{ for f in scratch/s7-2-2.txt scratch/s7-2-5.txt; do tr -d '\r' < $f | grep -E '^(race |race:|profile |car [0-9]+ |cars )' > $f.prog; done; cmp scratch/s7-2-2.txt.prog scratch/s7-2-5.txt.prog && echo "program lines: identical"; for k in 1 2 3; do cmp scratch/s7-2-in$k.input scratch/s7-2-in-b$k.input && echo "input $k: identical"; done; echo "lines=$(grep -c . scratch/s7-2-2.txt.prog)"; } > scratch/s7-2-5-cmp.txt 2>&1`
PASS iff the second run has `run: ok` and `scratch/s7-2-5-cmp.txt` has `program lines: identical`, `input 1: identical`, `input 2: identical`, `input 3: identical` and a `lines=` value of at least 50 (a race has at least 52 program lines: the header, the profile line, 16 per car, the `cars` line and the verdict). (The `broadsword run` job lines differ between runs, so only the program's lines are compared.)

**2.6 The race is physics, not a step-size artefact.** The 2.2 command with `--substeps 20` added and `--save-inputs ../../scratch/s7-2-in-s20`, saved as `scratch/s7-2-6.txt`, then `bash scratch/s7-2-verify.sh scratch/s7-2-6.txt > scratch/s7-2-6-verdict.txt 2>&1` and, in Git Bash:
`for k in 1 2 3; do for f in 2 6; do tr -d '\r' < scratch/s7-2-$f.txt | grep -E "^car $k lap n=[123] " | sed -E 's/.* time_s=([^ ]+) .*/\1/' | sort -n | head -n 1 > scratch/s7-2-6-best$k-$f.txt; done; awk -v k=$k -v a="$(cat scratch/s7-2-6-best$k-2.txt)" -v b="$(cat scratch/s7-2-6-best$k-6.txt)" 'BEGIN { d = a - b; if (d < 0) d = -d; if (a > 0 && b > 0 && d <= 0.01 * a) print "car " k " best lap within 1%: ok (" a " " b ")"; else print "car " k " best lap within 1%: FAIL (" a " " b ")" }'; done > scratch/s7-2-6-cmp.txt 2>&1`
PASS iff the 2.6 run has `run: ok`, the verdict's last line is `m2-race: ok`, and `scratch/s7-2-6-cmp.txt` has three `ok` lines and no `FAIL`. The 1% is milestone 1's step-size rule (1.6), *reasoned, not measured*: n20-20 reports the measured differences; if one is above 1% it stops and reports instead of raising the bound.

**What guards against tuning to the test in milestone 2:** the crawl bound (48.6 s) and the path floor do not come from the AI's own numbers (2.2); the wall counter is shown to fire on a track the car cannot fit (2.3); the AI's moves are shown to be only throttle, brake and steer by a replay that must equal the race, and a contact would break that equality (2.4); the verdict is re-derived from per-car lines, not read (2.2); the race must give the same bytes twice and survive a different step (2.5, 2.6). There is no human check in milestone 2: nothing a person can look at exists, because the viewer draws one body (trajectory v2 holds one body) and cannot show three cars. A multi-car viewer is the orchestrator's IDEAS line, not a task here.

#### The m2 verifier

Save the first block as `scratch/s7-2-verify.sh` and the second as `scratch/s7-2-inputs.sh`, unchanged. Both were run on synthetic outputs while this file was written (throwaway scripts under `scratch/`, not committed): the first printed `m2-race: ok` on a well-formed race and `m2-race: FAIL` on each of ten changes of it (a wall contact, a gap of 4.30 m, an invalid lap, a path of 330 m, a lap of 49 s, a wrong place, `off_track_s=0.020`, a missing `final` line, a wrong header and `finished=2`) and on an empty file; the second printed `inputs: ok` on a well-formed script and `inputs: FAIL` on a brake and throttle together and on a last time that differs from `E` in value, and `inputs: ok` when the last time is `120.000` and `E` is `120.0000`. Those runs show the scripts' logic only; the first real output is what shows the format matches.

```
# scratch/s7-2-verify.sh <race output file>: prints labelled lines and a last line m2-race: ok | FAIL
tr -d '\r' < "$1" | grep -E '^(race |race:|profile |car [0-9]+ |cars )' | awk '
function fail(m) { print "FAIL " m; bad = 1 }
function num(s, key,   a) { if (match(s, " " key "=[^ ]+")) { a = substr(s, RSTART + length(key) + 2, RLENGTH - length(key) - 2); return a } return "" }
BEGIN { bad = 0 }
$1 == "race" && $2 ~ /^track=/ { hdr++; if ($0 != "race track=oval cars=3 laps=3 time_limit_s=160.000 grid_gap_m=10.00 ai_params=ai_driver") fail("header line: " $0) }
$1 == "car" && $3 == "start" { st[$2]++ }
$1 == "car" && $3 == "checkpoint" { cp[$2, num($0, "lap")]++ ; ct = num($0, "t"); if (ct + 0 <= lastcp[$2] + 0 && lastcp[$2] != "") fail("checkpoint time not increasing: " $0); lastcp[$2] = ct }
$1 == "car" && $3 == "lap" {
  c = $2; n = num($0, "n"); lap[c, n]++
  v = num($0, "valid"); r = num($0, "reason"); off = num($0, "off_track_s")
  ts = num($0, "time_s"); pm = num($0, "path_m"); ms = num($0, "max_speed_kmh"); s = num($0, "start_t"); e = num($0, "end_t")
  if (v != "yes" || r != "none") fail("lap not valid: " $0)
  if (n + 0 <= 3) {
    laps_seen++
    if (off != "0.000") fail("off_track_s not 0.000: " $0)
    if (pm + 0 < 338.67) fail("path below the floor 338.67: " $0)
    if (ts + 0 > 48.6) fail("lap slower than 48.6 s (crawling): " $0)
    if (ts * ms / 3.6 < pm - 2) fail("timing identity: " $0)
    d = ts - (e - s); if (d < 0) d = -d; if (d > 0.0006) fail("time_s vs end_t-start_t: " $0)
    end3[c, n] = e
  }
}
$1 == "car" && $3 == "finish" { c = $2; fin[c]++; ft[c] = num($0, "t"); pl[c] = num($0, "place"); if (ft[c] + 0 > 160) fail("finish after the limit: " $0) }
$1 == "car" && $3 == "walls" { w[$2]++; if ($0 != "car " $2 " walls substeps=0 first_t=none") fail("wall contact: " $0) }
$1 == "car" && $3 == "final" { fl[$2]++ }
$1 == "cars" { g++; gm = num($0, "gap_min_m"); nr = num($0, "near_substeps"); if (gm + 0 < 4.40 || nr != "0") fail("car gap: " $0) }
$1 == "race:" { last = $0; rl++ }
END {
  for (c = 1; c <= 3; c++) {
    if (st[c] != 1) fail("car " c ": start lines " st[c] + 0)
    if (fin[c] != 1) fail("car " c ": finish lines " fin[c] + 0)
    if (w[c] != 1) fail("car " c ": walls lines " w[c] + 0)
    if (fl[c] != 1) fail("car " c ": final lines " fl[c] + 0)
    for (n = 1; n <= 3; n++) {
      if (lap[c, n] != 1) fail("car " c " lap " n ": lap lines " lap[c, n] + 0)
      if (cp[c, n] != 3) fail("car " c " lap " n ": checkpoint lines " cp[c, n] + 0)
    }
    if (ft[c] != end3[c, 3]) fail("car " c ": finish t " ft[c] " is not the end_t of lap 3 " end3[c, 3])
  }
  for (a = 1; a <= 3; a++) for (b = 1; b <= 3; b++) if (a != b) {
    if (pl[a] == pl[b]) fail("two cars share place " pl[a])
    if (ft[a] + 0 < ft[b] + 0 && pl[a] + 0 > pl[b] + 0) fail("place order disagrees with finish times")
  }
  for (c = 1; c <= 3; c++) if (pl[c] !~ /^[123]$/) fail("place not 1..3: car " c)
  if (hdr != 1) fail("header lines " hdr + 0)
  if (g != 1) fail("cars gap lines " g + 0)
  if (laps_seen != 9) fail("lap lines n<=3: " laps_seen + 0 " (need 9)")
  if (rl != 1 || last !~ /^race: ok cars=3 finished=3 wall_substeps=0 near_substeps=0 end_t=[0-9]+\.[0-9]{4}$/) fail("last race line: " last)
  print "cars_checked=3 laps_checked=" laps_seen + 0
  print (bad ? "m2-race: FAIL" : "m2-race: ok")
}'
```

```
# scratch/s7-2-inputs.sh <input file> <end_t text>: prints labelled lines and a last line inputs: ok | FAIL
awk -v E="$2" '
function bad(m) { print "FAIL " m; b = 1 }
NF != 4 { bad("line " NR ": " NF " fields"); next }
{ t = $1 + 0
  if (NR == 1 && t != 0) bad("first line time is " $1 ", not 0")
  if (NR > 1 && t <= prev) bad("line " NR ": time not increasing")
  if ($2 + 0 < 0 || $2 + 0 > 1) bad("line " NR ": throttle " $2)
  if ($3 + 0 < 0 || $3 + 0 > 1) bad("line " NR ": brake " $3)
  if ($4 + 0 < -1 || $4 + 0 > 1) bad("line " NR ": steer " $4)
  if ($2 + 0 > 0 && $3 + 0 > 0) bad("line " NR ": throttle and brake both on")
  prev = t; lt = $1 }
END { if (NR < 1000) bad("only " NR " lines"); if (lt + 0 != E + 0) bad("last line time " lt " is not end_t " E)
      print "lines=" NR " last_t=" lt; print (b ? "inputs: FAIL" : "inputs: ok") }' "$1"
```

### Milestone 3: a standalone executable a friend can run

**What "a friend can run" means, as checks.** A friend has a Windows 10 or 11 PC with no Rust, no Broadsword and no copy of this repository. They get one folder (the release folder), put it anywhere they can write, double-click `race.exe`, and can pick a track and a car, drive a timed lap, close the game, open it again, and find their best lap and their settings still there. A corrupt settings or save file does not crash the game: it is named in a message, left untouched, and the game runs on defaults. That is five claims, and each has a check: the folder holds everything and needs nothing outside Windows (3.4, 3.5); the menu path works (3.2, 3.5); settings and saves survive a restart (3.2, 3.5); corrupt files are refused by name and never overwritten (3.3); and a person on another machine or account did it (3.6).

**Where the data lives: a `data` folder next to `race.exe`, not in the user's app-data folder.** Reason: Broadsword must be portable (AGENTS.md: every path relative, the project moves between machines), the folder can be copied or deleted as a unit, nothing is written to the registry or to the user's profile, and a check can run the game from a clean folder and look at everything it wrote. The cost: a folder that is not writable (a protected one such as Program Files) cannot hold saves. The shell then prints `warn data folder not writable: <path>` and the game still runs; the README tells the friend to keep the folder somewhere they own.

**How the front end gets the menus: a text protocol with a crash-sim child (the proposed n20-26a).** The menu state machine, the settings file and the save file are in crash-sim (`menu.rs`, `settings.rs`, `save.rs`), and ADR 0001 (a) forbids render from linking crash-sim, so render's front end starts `crash-sim --shell` as a child process and talks to it in lines: events in on stdin, state out on stdout. This keeps one implementation of every rule (the same menu, the same strict parsers) and puts the whole menu path under a test that needs no window. The front end only draws what the lines say and starts races.

**The shell protocol (n20-26a, documented in `docs/SHELL.md`).** `crash-sim --shell --data <dir> --shell-track <file> [--shell-track <file> ...] --shell-car <file> [--shell-car <file> ...] --today <YYYY-MM-DD>`. The track and car lists are exactly the repeated flags, in the order given (never scanned, never hard-coded: n20-23's rule), at least one of each, and a track's or car's id is its file stem. `--today` is required (the shell has no clock, so a test is deterministic; the front end passes the UTC date). The data folder holds `settings.txt` (`settings 1`, n20-21) and `best.save` (`save 2`, n20-22), both written canonically and atomically. One line is read from stdin per event: `up`, `down`, `select`, `back`, `pause`, `finished <ms>` or `finished none` (the race ended; the best valid lap in whole milliseconds), and `set <key> <value>` (`width`, `height`, `fullscreen`, `volume`, `units`; only on the settings screen). The shell prints one line per fact, each flushed. At start, after the `settings` and `best` lines and before it reads any event, it prints the initial `state main cursor=0`. After every event except `set` it prints the `state` line (also when the state did not change), then an `action` line if the step returned one, then what carrying the action out printed. A `set` event prints only its `settings` line, or a `warn`, and no `state` line. The pinned text of 3.2 follows exactly this.

```
shell tracks=<n> cars=<n>                                           first line
settings width=<w> height=<h> fullscreen=<true|false> volume=<v> units=<kmh|mph> source=<defaults|file|unsaved>
best track=<id> seconds=<value> car=<id> date=<YYYY-MM-DD>          one per saved track, sorted by id, at start only
state main cursor=<i>  |  state track_select cursor=<i>  |  state car_select track=<id> cursor=<i>  |  state settings
state race track=<id> car=<id>  |  state paused track=<id> car=<id> cursor=<i>  |  state results track=<id> car=<id>  |  state quit
action start_race track=<id> track_file=<path as given> car=<id> car_file=<path as given>
action abort_race  |  action save_settings  |  action quit
saved settings                       after action save_settings
saved best track=<id> seconds=<3 decimals>      after `finished <ms>` when the lap beat the saved one (or there was none)
warn settings refused: <settings error text>     the file failed `settings::load`; defaults are in use; the file is not touched
warn save refused: <save error text>             the file failed `Save::parse`; an empty save is in use; the file is not touched
warn settings not saved: ... / warn best not saved: ...    a write was asked for while the file is in the refused state (never overwrites a refused file)
warn set refused: <message>                       a bad key, a bad value (settings::validate's text) or not on the settings screen; nothing changed
warn data folder not writable: <path>             printed once at start when a file cannot be created there; the shell still runs
```

The shell exits 0 after `action quit` or at end of stdin, and 2 on a usage error (empty stdout). A `finished <ms>` outside the race state is ignored with `warn finished ignored: not in a race`. `seconds` in `saved best` and in `best` is the integer milliseconds divided by 1000 with three decimals, made from the integer (milestone 1's rule), so the printed time, the saved time and the on-screen time are one number.

**The front end and the release folder (n20-26).** `race` is a render example (`projects/render/examples/race.rs`): a window with the menu drawn from the shell's `state` lines, and a race that starts when the shell prints `action start_race`: it runs `crash-sim --body <car_file> --track <track_file> --live` as a child, as the drive viewer does (the `--live` race with lap lines is the item that milestone 1 and n20-14 need; see "What the briefs must take"), shows the lap timer, and when the first timed lap ends or the player presses back, sends `finished <ms>` (or `finished none`) to the shell. `race [--sim-exe <path>] [--cars <dir>] [--tracks <dir>] [--data <dir>] [--today <YYYY-MM-DD>] [--script <file> --no-window]`: with no flag, the executable looks next to itself for `crash-sim.exe`, `cars/` (every `*.bsn`, sorted by file name), `tracks/` (every `*.track`, sorted) and `data/`, and uses today's UTC date; this is the double-click case. `--script <file>` replaces the keyboard and the window for a test: one line per event as the shell reads them, plus `drive <input file>` (path relative to the script's folder; only in the race state), which runs `crash-sim --body <car_file> --track <track_file> --input <file>` in place of the live race, prints its lines unchanged, takes the best lap from its `race: ... best_lap_s=<3 decimals, or none>` line as whole milliseconds by text (no float) and sends `finished` to the shell; blank lines and `#` comments are skipped; `--script` requires `--no-window` and `--today`. The front end prints every line the shell prints, unchanged, and the child's lines unchanged, and ends with exactly one of `game: ok` (the shell printed `action quit` and every child exited 0) or `game: FAIL <reason naming the script line or child>` (exit 1). `scripts/package_race.py build --out <folder>` (inside `scratch/` only) builds a release folder; `scripts/package_race.py check --folder <folder>` checks one. The folder holds `race.exe`, `crash-sim.exe`, `cars/car_basic.bsn`, `tracks/oval.track`, `tracks/chicane.track`, `demo/oval_lap.input` (a copy of `scenarios/tracks/oval_lap.input`, used by the scripted drive), `README.txt` (how to run, where the data is, that the folder must be writable), and `vcruntime140.dll` only if the executables import it (below). It has no `data/` folder: the game makes it on first run.

**Windows system DLLs, and the one that is not.** `crash-sim.exe` on `main` imports `vcruntime140.dll` (read with a small PE import reader, run on `build/release/crash-sim.exe` on this machine while this file was written: `api-ms-win-core-synch-l1-2-0.dll`, six `api-ms-win-crt-*.dll`, `bcryptprimitives.dll`, `kernel32.dll`, `ntdll.dll` and `vcruntime140.dll`). The `api-ms-win-crt-*` files are the Universal C Runtime, part of Windows 10 and 11. `vcruntime140.dll` is the Visual C++ runtime: a clean Windows install need not have it. So a folder that imports it and does not carry it would fail on a friend's machine and pass on every machine of ours. n20-26 must pick one of two fixes, and the folder check accepts exactly these two outcomes: the executables are built with the C runtime linked in statically (`-C target-feature=+crt-static`), so they import no `vcruntime140.dll`; that build goes into a target folder of its own under `scratch/` (the script sets `CARGO_TARGET_DIR` to it), because different compiler flags would rebuild and replace the release files of the shared build folder that other jobs use. This is a stated exception to the one-build-folder rule of AGENTS.md, for a wipeable scratch folder used by this script only; the orchestrator confirms it when n20-26 is boarded; or `vcruntime140.dll` sits beside them (an app-local copy of the redistributable DLL, which Microsoft's redistribution terms allow). Which one is a recorded decision in the evidence, not a loosened check.

**The numbers, and why.** Window 1280 x 720, volume 80, units km/h, throttle on Up and so on are the defaults of `settings.rs` (n20-21), not chosen here. The date `2026-11-20` in the scripted runs is a fixed test date. 83456 ms is the 83.456 s used by the milestone-1 time-text test (1.7). The second track of the menu, `scenarios/tracks/chicane.track`, already exists: nb-43 (board task `t-20261119-171333-44`) created it with the track parser, and it is in crash-sim on `0bb2b5e`; no task here creates it.

#### Checks

**3.1 Tests.** `broadsword test crash-sim > scratch/s7-3-1.txt 2>&1`, `broadsword test render > scratch/s7-3-1-render.txt 2>&1` and `python -I -B scripts/test_package_race.py -v > scratch/s7-3-1-py.txt 2>&1`.
PASS iff crash-sim's last line is `test: ok — N passed, 0 failed` with N at least the N of 2.1 plus the 7 tests named here; render's last line is `test: ok — N passed, 0 failed` with N at least the N recorded for 1.7's render run in `evidence/stage-7-m1.md`; the Python run ends `OK`; and the crash-sim file contains `Running tests\shell.rs` and each of `test shell_scripted_menu_path_starts_a_race_and_saves_the_lap ... ok`, `test shell_settings_and_best_survive_a_restart ... ok` (two processes, one data folder), `test shell_refuses_corrupt_settings_and_save_and_leaves_them_unchanged ... ok`, `test shell_set_refuses_bad_values_by_name ... ok`, `test shell_needs_at_least_one_track_and_car ... ok`, `test shell_never_overwrites_a_refused_file ... ok`, `test shell_unwritable_data_folder_is_a_warning ... ok` (n20-26a), plus the existing tests of the three files the shell is built on: `{ source scratch/s7-named.sh; named scratch/s7-3-1.txt settings 9; named scratch/s7-3-1.txt save 13; named scratch/s7-3-1.txt menu 8; } > scratch/s7-3-1-named.txt 2>&1` prints three `named ...: ok` lines and no `MISSING` line. `scripts/test_package_race.py` must hold a case where the folder check is given a fake executable that imports a DLL outside the allow-list and has no copy of it beside it (it must fail), one with the DLL beside it (it must pass), one whose file holds a drive-letter path that is not a `.pdb` path, once with `/` and once with `\` (each must fail), and a folder with a file missing (it must fail).
FAIL: any failed or missing test, or a smaller N.

**3.2 The scripted menu path, and a restart (the protocol, without a window).** In Git Bash from the Broadsword root:
```
mkdir -p scratch/s7-3
{ build/bin/broadsword.exe run crash-sim -- cargo build --offline --release; cp build/release/crash-sim.exe scratch/s7-3/crash-sim.exe && echo "copied"; } > scratch/s7-3-2-build.txt 2>&1
printf 'select\nselect\nselect\nfinished 83456\nback\ndown\nselect\nset volume 55\nset units mph\nselect\nback\ndown\nselect\n' > scratch/s7-3-a.events
printf 'down\nselect\nback\ndown\nselect\n' > scratch/s7-3-b.events
rm -rf scratch/s7-3-data; mkdir scratch/s7-3-data
( cd projects/crash-sim || exit 2; X=../../scratch/s7-3/crash-sim.exe; O="--shell --data ../../scratch/s7-3-data --shell-track scenarios/tracks/oval.track --shell-track scenarios/tracks/chicane.track --shell-car scenarios/car_basic.bsn --today 2026-11-20"
timeout 60 $X $O < ../../scratch/s7-3-a.events > ../../scratch/s7-3-2a.txt 2> ../../scratch/s7-3-2a.err; echo "exit_a=$?"
cp ../../scratch/s7-3-data/settings.txt ../../scratch/s7-3-2-settings-after-a.txt; cp ../../scratch/s7-3-data/best.save ../../scratch/s7-3-2-best-after-a.txt
timeout 60 $X $O < ../../scratch/s7-3-b.events > ../../scratch/s7-3-2b.txt 2> ../../scratch/s7-3-2b.err; echo "exit_b=$?" ) > scratch/s7-3-2-exit.txt 2>&1
```
The expected texts: the lines of the code block below that follow a `--- <name>` line, up to the next `---` line or the end of the block, are written to `scratch/<name>` (four files: `s7-3-2a.want`, `s7-3-2b.want`, `s7-3-2-settings.want`, `s7-3-2-best.want`; the `---` lines are not part of them). Every `( ... )` group in these blocks is a subshell on purpose: a `cd` inside it must not leak into the next command of the block (a `{ ... }` group runs in the current shell, and its `cd` stays). Then, in Git Bash, `for f in 2a 2b; do tr -d '\r' < scratch/s7-3-$f.txt | diff - scratch/s7-3-$f.want > scratch/s7-3-$f.diff 2>&1; echo "$f diff exit $?"; done > scratch/s7-3-2-diff.txt 2>&1; diff scratch/s7-3-2-settings-after-a.txt scratch/s7-3-2-settings.want > scratch/s7-3-2-settings.diff 2>&1; diff scratch/s7-3-2-best-after-a.txt scratch/s7-3-2-best.want > scratch/s7-3-2-best.diff 2>&1; cmp scratch/s7-3-2-settings-after-a.txt scratch/s7-3-data/settings.txt; cmp scratch/s7-3-2-best-after-a.txt scratch/s7-3-data/best.save`.

```
--- s7-3-2a.want
shell tracks=2 cars=1
settings width=1280 height=720 fullscreen=false volume=80 units=kmh source=defaults
state main cursor=0
state track_select cursor=0
state car_select track=oval cursor=0
state race track=oval car=car_basic
action start_race track=oval track_file=scenarios/tracks/oval.track car=car_basic car_file=scenarios/car_basic.bsn
state results track=oval car=car_basic
saved best track=oval seconds=83.456
state main cursor=0
state main cursor=1
state settings
settings width=1280 height=720 fullscreen=false volume=55 units=kmh source=unsaved
settings width=1280 height=720 fullscreen=false volume=55 units=mph source=unsaved
state settings
action save_settings
saved settings
state main cursor=1
state main cursor=2
state quit
action quit
--- s7-3-2b.want
shell tracks=2 cars=1
settings width=1280 height=720 fullscreen=false volume=55 units=mph source=file
best track=oval seconds=83.456 car=car_basic date=2026-11-20
state main cursor=0
state main cursor=1
state settings
state main cursor=1
state main cursor=2
state quit
action quit
--- s7-3-2-settings.want
settings 1
width = 1280
height = 720
fullscreen = false
volume = 55
units = mph
key.throttle = up
key.brake = down
key.left = left
key.right = right
key.pause = escape
key.reset = r
--- s7-3-2-best.want
save 2
best oval 83.456 car_basic 2026-11-20
```
PASS iff `scratch/s7-3-2-build.txt` contains `run: ok` and ends `copied`; `scratch/s7-3-2-exit.txt` is exactly `exit_a=0` and `exit_b=0` (an exit of 124 means a run hit its 60 s `timeout` and hung); `scratch/s7-3-2-diff.txt` is `2a diff exit 0` and `2b diff exit 0` (the output of both runs is the expected text, line for line: the menu path, the saved best lap, the saved settings, and, in the second process, the restored settings and best lap); `scratch/s7-3-2-settings.diff` and `scratch/s7-3-2-best.diff` are empty (the two files are the canonical text); the two `cmp` lines print nothing (run B changed no file); and both `.err` files are empty. The test of the expected text is exact, so a changed word in the protocol is a failed check until this file and `docs/SHELL.md` agree.
FAIL: any difference.

**3.3 Corrupt files are refused by name, and never overwritten.** In Git Bash from the Broadsword root, with `scratch/s7-3/crash-sim.exe` from 3.2:
```
rm -rf scratch/s7-3-bad; mkdir scratch/s7-3-bad
printf 'settings 1\nvolume = 101\n' > scratch/s7-3-bad/settings.txt
printf 'save 2\nbest oval abc car_basic 2026-11-20\n' > scratch/s7-3-bad/best.save
cp scratch/s7-3-bad/settings.txt scratch/s7-3-bad-settings.orig; cp scratch/s7-3-bad/best.save scratch/s7-3-bad-best.orig
printf 'select\nselect\nselect\nfinished 70000\nback\ndown\nselect\nselect\nback\ndown\nselect\n' > scratch/s7-3-c.events
( cd projects/crash-sim || exit 2; timeout 60 ../../scratch/s7-3/crash-sim.exe --shell --data ../../scratch/s7-3-bad --shell-track scenarios/tracks/oval.track --shell-track scenarios/tracks/chicane.track --shell-car scenarios/car_basic.bsn --today 2026-11-20 < ../../scratch/s7-3-c.events > ../../scratch/s7-3-3.txt 2> ../../scratch/s7-3-3.err; echo "exit=$?" ) > scratch/s7-3-3-exit.txt 2>&1
{ tr -d '\r' < scratch/s7-3-3.txt | grep -c '^warn settings refused: settings line 2: '; tr -d '\r' < scratch/s7-3-3.txt | grep -c '^warn save refused: .*line 2'; tr -d '\r' < scratch/s7-3-3.txt | grep -c '^warn settings not saved: '; tr -d '\r' < scratch/s7-3-3.txt | grep -c '^warn best not saved: '; tr -d '\r' < scratch/s7-3-3.txt | grep -c '^settings width=1280 height=720 fullscreen=false volume=80 units=kmh source=defaults$'; tr -d '\r' < scratch/s7-3-3.txt | tail -n 2 | tr '\n' '|'; echo; cmp scratch/s7-3-bad/settings.txt scratch/s7-3-bad-settings.orig && echo settings-unchanged; cmp scratch/s7-3-bad/best.save scratch/s7-3-bad-best.orig && echo save-unchanged; grep -c panicked scratch/s7-3-3.err; } > scratch/s7-3-3-counts.txt 2>&1
```
PASS iff `scratch/s7-3-3-exit.txt` is `exit=0` and `scratch/s7-3-3-counts.txt` is exactly these lines: `1`, `1`, `1`, `1`, `1`, `state quit|action quit|`, `settings-unchanged`, `save-unchanged`, `0` (the settings error names line 2; the save error names line 2; each refused file produced one "not saved" warning when a write was asked for; the game ran on defaults; both files are byte for byte what the test wrote; no panic).
FAIL: any other line. A count of 0 where a 1 is expected is a missing refusal, not a pass.

**3.4 The release folder holds what it must, imports only Windows and leaks no path.** In Git Bash from the Broadsword root, `python -I -B scripts/package_race.py build --out scratch/s7-3-release > scratch/s7-3-4-build.txt 2>&1`, then `python -I -B scripts/package_race.py check --folder scratch/s7-3-release > scratch/s7-3-4-check.txt 2>&1`.
PASS iff both commands exit 0, the last line of the build output is `package: ok folder=scratch/s7-3-release files=<n>`, and the check output has a `file <name> <bytes>` line, with a size above 0, for each of `race.exe`, `crash-sim.exe`, `cars/car_basic.bsn`, `tracks/oval.track`, `tracks/chicane.track`, `demo/oval_lap.input` and `README.txt`; an `import <exe> <dll> <allowed|beside>` line for every DLL each executable imports (none `MISSING`); no `leak` line; and the last line `release-check: ok files=<n> dlls=<m>`. The allow-list in `package_race.py` is **not final**: it is a starting list of Windows system DLLs that are in every Windows 10 and 11 install (`kernel32.dll`, `ntdll.dll`, `user32.dll`, `gdi32.dll`, `ws2_32.dll`, `advapi32.dll`, `bcryptprimitives.dll`, `ucrtbase.dll`, `shell32.dll`, `ole32.dll`, `msvcrt.dll`, and any name that starts `api-ms-win-`, all case-insensitive), written from recall of what a Win32 window and sockets need; only the imports of `crash-sim.exe` were read when this file was written (listed above). The imports of the windowed `race.exe` are **unknown until it is built**: n20-26 reads them with the same PE reader, lists them in its report, and for every name not on the starting list says why it is a system DLL on a clean Windows 10 and 11 (or puts the file beside the executable); the orchestrator accepts the final list in the evidence, and the list changes only with that record. any other import must be a file beside the executable (`beside`). A `leak` line is a string in an executable that starts with a drive letter, a colon and either slash, `C:/` or `C:\`, in both forms, other than one ending `.pdb` (the debug-information path that every MSVC build carries and that nothing reads). Registry crates and the standard library embed absolute paths in their panic messages (`C:\Users\...\registry\src\...`); there are no registry crates today, but the rule must hold when one is added. The release build therefore removes the paths: n20-26 builds with the rustflag `--remap-path-prefix`, once for the project root, once for the Cargo home and once for the Rust toolchain folder, each mapped to a fixed text such as `src`, together with the `+crt-static` flag if n20-26 chooses the static runtime (the DLL paragraph above allows either that or an app-local `vcruntime140.dll`), in the build's own target folder. Nothing is written outside `scratch/`: the script refuses an `--out` that is not inside it.
FAIL: a nonzero exit, a `MISSING` import, a `leak`, a missing file, or a missing last line.

**3.5 The game runs from a clean copy, with nothing of ours on the machine path, and remembers.** The front end lists the tracks of its `tracks/` folder sorted by file name, `chicane.track` then `oval.track`, so the first menu entry is the chicane; the script therefore presses `down` once before the second `select`, which picks `oval` (the second entry), and the pinned line `action start_race track=oval ` fails the check if the order or the pick is ever different. (3.2 does not have this issue: it names its tracks in the order of its flags, oval first.) In Git Bash from the Broadsword root:
```
rm -rf scratch/s7-3-clean; cp -r scratch/s7-3-release scratch/s7-3-clean
printf 'select\ndown\nselect\nselect\ndrive s7-3-clean/demo/oval_lap.input\nback\ndown\nselect\nset volume 55\nset units mph\nselect\nback\ndown\nselect\n' > scratch/s7-3-a.script
printf 'down\nselect\nback\ndown\nselect\n' > scratch/s7-3-b.script
( cd scratch || exit 2; timeout 120 env -i PATH="/c/Windows/System32:/c/Windows" SYSTEMROOT="C:\\Windows" s7-3-clean/race.exe --script s7-3-a.script --no-window --today 2026-11-20 > s7-3-5a.txt 2>&1; echo "exit_a=$?"
timeout 120 env -i PATH="/c/Windows/System32:/c/Windows" SYSTEMROOT="C:\\Windows" s7-3-clean/race.exe --script s7-3-b.script --no-window --today 2026-11-20 > s7-3-5b.txt 2>&1; echo "exit_b=$?" ) > scratch/s7-3-5-exit.txt 2>&1
{ A=scratch/s7-3-5a.txt; B=scratch/s7-3-5b.txt
T=$(tr -d '\r' < $A | grep -E '^race: ' | head -n 1 | sed -E 's/.* best_lap_s=([0-9]+\.[0-9]{3}).*/\1/'); echo "best_lap_s=$T"
tr -d '\r' < $A | grep -c -E '^lap n=1 .* valid=yes '
tr -d '\r' < $A | grep -c "^saved best track=oval seconds=$T\$"
tr -d '\r' < $A | grep -c '^saved settings$'
tr -d '\r' < $A | grep -c '^action start_race track=oval '
tr -d '\r' < $A | tail -n 1
tr -d '\r' < $B | grep -c "^best track=oval seconds=$T car=car_basic date=2026-11-20\$"
tr -d '\r' < $B | grep -c '^settings width=1280 height=720 fullscreen=false volume=55 units=mph source=file$'
tr -d '\r' < $B | tail -n 1
Tn=$(echo "$T" | sed -E 's/0+$//; s/\.$//'); printf 'save 2\nbest oval %s car_basic 2026-11-20\n' "$Tn" | cmp - scratch/s7-3-clean/data/best.save && echo best-save-bytes-ok
cmp scratch/s7-3-2-settings.want scratch/s7-3-clean/data/settings.txt && echo settings-bytes-ok
ls scratch/s7-3-clean; } > scratch/s7-3-5-lines.txt 2>&1
```
PASS iff `scratch/s7-3-5-exit.txt` is `exit_a=0` and `exit_b=0`, and `scratch/s7-3-5-lines.txt` is: a line `best_lap_s=` with a value like `83.456` (not empty, not `none`), then `1`, `1`, `1`, `1`, `game: ok`, `1`, `1`, `game: ok`, `best-save-bytes-ok`, `settings-bytes-ok`, then the listing of the clean folder, which holds `README.txt`, `cars`, `crash-sim.exe`, `data`, `demo`, `race.exe` and `tracks` (and `vcruntime140.dll` only if 3.4 shows it as `beside`). (`Tn` is `T` without trailing zeros: the save file writes the number with Rust's shortest text, so 83.450 is saved as `83.45`.) This shows, with `PATH` holding only the Windows folders and the run started from another folder, that the executable finds its own files next to itself, drives a scripted lap through the real child process, saves that lap, and finds it again in a second process; the file `scratch/s7-3-2-settings.want` is 3.2's expected settings text (the same settings were set).
FAIL: any difference; `exit_a=124` or `exit_b=124` (the game hung; each run is under `timeout 120`: the scripted lap is about 50 s of simulated time, which the bench car case puts near 5 ms per simulated second, so 120 s is a wide margin, *reasoned*); `game: FAIL ...`; a value `none` for `best_lap_s` (no valid lap, so nothing was saved and nothing proven).

**3.6 A person on another machine or account runs it** (the human check; Lazlo, or a friend if he chooses one). Tell Lazlo first. The release folder of 3.4, as a zip or a copied folder, goes to a Windows 10 or 11 machine that has no Rust and no Broadsword, or, if there is only one PC, to a second Windows user account (a standard user, signed in, with no Rust toolchain in its profile). The person double-clicks `race.exe` (no command line), and: picks the oval and the car from the menu, drives at least one full timed lap, closes the game with the menu's quit, opens it again, and reads the best-lap list and the settings screen. The orchestrator records `evidence/stage-7-m3-human.md` before scratch can be wiped, with: the UTC date and time; who ran it; the machine (computer name, Windows version from `winver`, and whether it is another PC or a second account on the main PC: a second account does not test a missing runtime, which is why 3.4 reads the imports); the contents of the release folder as handed over; the lines the game printed to its log if it has one; and the answers, quoted verbatim from the person's own message: Q1 "Did it start by double-clicking race.exe, with nothing to install?", Q2 "Could you pick a track and a car and start a race?", Q3 "Did your lap time show in the best laps after you closed and opened the game again?", Q4 "Did a setting you changed (volume or units) still hold after reopening?", and "What felt wrong, or not like a game you could hand to a friend?", with one board task id or `IDEAS.md` line per point raised.
PASS iff 3.1 to 3.5 pass, the record has every item above, and Q1 to Q4 are each answered yes in the person's own words. An agent never answers for them. The "felt wrong" points are not gated, but each needs its task or IDEAS line before the milestone is tagged.
FAIL: no record, a paraphrase, any answer other than yes, or a missing item.

**What guards against tuning to the test in milestone 3:** the expected text of the protocol is exact (3.2), so a menu that "works" in a different way fails; the state lines come from `menu::step` (n20-23's tests: no dead ends), so the shell is a thin layer; the restart in 3.2 and 3.5 is a second process reading what the first wrote, and the files are compared byte for byte; a corrupt file is shown to be refused, named and left alone (3.3); "no Rust, no Broadsword" is the environment of 3.5 and the import check of 3.4, not a claim; and the last word is a person on a machine that is not ours (3.6).

### Milestone 4: two machines race each other

**Lockstep, as built.** The sim is deterministic (STAGE6 3.5), so only inputs and state hashes cross the network (`src/net.rs`, `docs/NET.md`, `src/net/session.rs`). Both machines run the same two-car sim from the same files; the host's input drives car 1, the guest's drives car 2; a peer steps tick t only when it holds both inputs for t (a late input is waited for, never guessed); every N ticks both exchange a hash of the whole sim state, and a difference stops both with `desync at tick t` and both hashes. That is n20-25b's, merged. The input message carries throttle from -1 to 1 (reverse, crash-sim `6029157` on the wire, `15499f4` in the sim), brake 0 to 1 and steer -1 to 1, as integers in thousandths; the scripts of this milestone use no reverse. This milestone wires the session to the race and shows it on a real race.

**The numbers, and why.**

- **Tick rate 100 per second, one tick is one output step (`dt` 0.01 s, 10 substeps).** The input of a tick is held for its 10 substeps. 1000 ticks a second (one per substep) would need a round trip under 1 ms per tick, which no Wi-Fi LAN gives. `hello` carries the tick rate, so a peer with another `--dt` is refused (n20-25b: `HelloMismatch`).
- **Input delay D = 6 ticks (60 ms).** An input given at tick t is applied at t + 6, so a message has 60 ms to arrive before the peer needs it. A home Wi-Fi round trip is usually 2 to 10 ms and spikes to 30 to 50 ms; 60 ms covers a spike with a margin, and is short enough to drive with. *Reasoned; the LAN check (4.7) records the round trips.*
- **Hash every N = 100 ticks (1 s).** A divergence is found within one simulated second of its first visible effect. One hash is a 16-digit line.
- **Timeout T = 5 s per message; connect timeout 30 s.** Longer than any stall of a healthy LAN peer, short enough to be a failure rather than a hang (the n20-25b test shows a silent peer ends within 2 x T). The connect timeout is longer because a person may have to click "Allow" on the Windows firewall prompt.
- **`--duration 120` (12000 ticks, 120 hashes).** Milestone 1 (1.4) says `oval_lap.input` drives two laps and then brakes to a stop, as built in n20-13. Two laps at the 8 m/s floor of m2 are 2 x 48.6 = 97.2 s, plus the run-up of at most 5 s (40 m at 8 m/s is the slot-3 figure of m2; slot 2 is 30 m, so less): 102.2 s, so 120 s holds. The script of car 2 must end its two laps in the same time (4.1 tests that its last line is before 120 s). After its script ends, a car keeps the last input (the script ends with a brake to a stop).
- **Two scripts, one per slot.** Car 1 uses `oval_lap.input` from slot 1. Car 2 uses its own script, `oval_lap_b.input`, made for slot 2 by n20-27 (see "Who builds what, milestone 4"). It is not `oval_lap.input` run from slot 2, and not that script delayed: the track is flat with no physical walls, so the same inputs from a slot 10 m farther back give the same path shifted 10 m along the front straight, which puts car 2 about 10 m off the centreline at both bend apexes (off track, invalid laps), and a time delay does not help, since on the back straight the two cars would be only |v x delay - 10 m| apart (for a 1 s delay, a collision at speeds between about 5.6 and 14.4 m/s). So `oval_lap_b.input` is authored the way `oval_lap.input` is (open loop, aimed by running it, with the final tyres) but from slot 2, with throttle and steer times chosen so that car 2 stays on the track and behind car 1; it drives two laps and ends before 120 s. That it is well aimed is measured, not assumed: 4.1's `oval_lap_b_drives_a_valid_lap_from_slot_2` runs it alone from slot 2 (`--grid-slot 2`, n20-20) and 4.2 requires both cars to complete a valid lap and the cars to stay at least 4.40 m apart (the rule and reason of milestone 2). With one script on both sides a bug that gave the local input to car 1 on both machines would still print equal lines; with two slot-specific scripts it makes car 2's lap invalid or the cars touch, which 4.2 reports.
- **A state hash that covers every field the next step reads** (n20-27): the tick, then for car 1 and car 2 in that order: every node's position and velocity, every wheel's spin and steer, every beam's rest length and broken flag, all as `f64` bit patterns (n20-24's `write_f64`; NaN refused). A field left out can diverge without a hash ever showing it, and plastic beam state is state. The test `state_hash_covers_every_state_field` changes each kind of field alone and requires a different hash each time.

**Flags and output (n20-27).** `crash-sim --body <car.bsn> --track <file> --grid-gap <m> --duration <s> --host <addr:port> | --join <addr:port> --input <file> --net-delay <ticks> --net-hash-every <ticks> --net-timeout <s> --net-connect-timeout <s> --net-seed <u64>`; the same program without a network, as the reference, is `--input <file> --input2 <file> --net-delay <ticks> --net-hash-every <ticks>` (car 1 from `--input`, car 2 from `--input2`, each applied `--net-delay` ticks late as in a network run, the same sim, the same hash lines, no `net` lines). `--host 127.0.0.1:0` binds an OS-chosen port and prints it; `--host 0.0.0.0:47000` is the LAN case; the host binds exactly the address it is given. The hello's tick rate is 1 / `--dt`. Both peers must be given the same `--body`, `--track`, `--grid-gap`, `--duration`, `--net-delay`, `--net-hash-every` and `--net-seed`. A difference in the first six shows as a sequence error or a desync by design (4.3); a different `--net-seed` is refused at `hello` (`HelloMismatch`). **`--net-seed` does not affect the sim:** crash-sim has no random numbers (the header of `src/lib.rs`: "deterministic (no RNG, fixed step order)"), so the seed is not in the state hash, the reference run takes no seed, and it only has to be one agreed number that `hello` checks and the `net role=` line prints; it is there for a later game with random events. The host drives car 1, the guest car 2. Lines, in this order (the program's, after `broadsword`'s own when run through it):

```
net listening addr=<addr:port>                 host only, first, flushed before it waits
net role=<host|guest> addr=<addr:port> tick_rate=<n> input_delay=<n> hash_every=<n> timeout_s=<3 decimals> seed=<n>
car <k> start t=<4 decimals>                    k is 1 or 2; the lines of milestone 1, as in milestone 2
car <k> checkpoint lap=<n> k=<n> t=<4 decimals>
car <k> lap n=<n> ...
hash tick=<t> value=<16 lower-case hex>        after every hash_every-th tick, t = 99, 199, ...
car <k> final t=<4 decimals> x=<4 decimals> z=<4 decimals> heading_deg=<2 decimals>
cars gap_min_m=<2 decimals> gap_min_t=<4 decimals> near_substeps=<count>
net: ok ticks=<n> hashes=<n> last_hash=<16 hex>
```
The `cars` line is milestone 2's, with the same definition (the smallest distance between the two chassis centres in the ground plane over all substeps, and the number of substeps with the centres closer than 4.40 m; the reason for 4.40 m is in "The numbers, and why" of milestone 2); the reference run prints it too. Or, instead of the last line, one of `net: FAIL desync at tick <t> own=<hex> peer=<hex>`, `net: FAIL timeout at tick <t> after <seconds, 3 decimals> s`, `net: FAIL disconnected at tick <t>`, `net: FAIL peer left at tick <t>`, `net: FAIL no peer within <seconds, 3 decimals> s`, `net: FAIL cannot connect to <addr:port>: <reason>`, with exit code 1; exit 2 for a usage error. A failure line is always the last line.

**When a peer prints race lines.** A peer prints the `car` lines (`start`, `checkpoint`, `lap`) and the `hash` line of an interval only after that interval's hash has been confirmed equal on both peers. The intervals are the hash intervals: interval i is ticks 100i to 100i + 99, and a line belongs to the interval of the tick in which its event happened. At the end of interval i a peer prints, in this order, the `car` lines of the interval (in simulation-time order, ties by car number) and then `hash tick=<100i + 99> value=<hex>`; the `car ... final` lines and the `cars` line follow the last hash, and `net: ok` is last. So the output does not depend on timing: a peer that is ahead prints nothing of an interval the other has not confirmed, and the reference run prints in the same order (the interval's lines, then its hash). **The stall under D = 6:** after stepping tick 100i + 99 a peer sends its hash and does not step tick 100i + 100 until the other peer's hash for that tick has arrived (n20-25b). Every input is sent when it is given and applied D ticks later, so the inputs for the D = 6 ticks after the stall are already on their way before it, and nothing is lost: the stall costs one round trip per 100 ticks (once per simulated second), and nothing more. On a mismatch a peer prints no line of the interval at all (no `car` line, no `hash` line), only `net: FAIL desync ...`, and neither peer prints another `car`, `cars` or `hash` line after it.

#### Checks

**4.1 Tests.** `broadsword test crash-sim > scratch/s7-4-1.txt 2>&1`
PASS iff the last line is `test: ok — N passed, 0 failed`, N is at least the N of 3.1 plus the 9 tests named here, and the file contains `Running tests\two_player.rs` and each of `test two_peers_on_loopback_match_the_local_run ... ok`, `test forced_desync_is_reported_by_both_peers ... ok` (the peers are started with different `--grid-gap`), `test silent_peer_ends_with_timeout ... ok`, `test killed_peer_ends_with_a_named_error ... ok`, `test no_peer_ends_with_a_named_error ... ok`, `test state_hash_covers_every_state_field ... ok`, `test oval_lap_b_drives_a_valid_lap_from_slot_2 ... ok` (runs `oval_lap_b.input` alone from slot 2 and asserts a valid lap, and that its last line is before 120 s), `test host_and_guest_inputs_drive_their_own_cars ... ok` (swapping the two scripts changes which car moves), `test no_race_lines_of_an_unconfirmed_interval_are_printed ... ok` (a stand-in two-car run with events in interval 5, ticks 500 to 599, and a desync forced in that interval: neither peer's output has any `car` or `hash` line of interval 5, only the lines of intervals 0 to 4 and `net: FAIL desync at tick 599`) (n20-27), and the existing tests of `net_codec` (14), `net_session` (18) and `statehash` (7): `{ source scratch/s7-named.sh; named scratch/s7-4-1.txt net_codec 14; named scratch/s7-4-1.txt net_session 18; named scratch/s7-4-1.txt statehash 7; } > scratch/s7-4-1-named.txt 2>&1` prints three `named ...: ok` lines and no `MISSING` line.
FAIL: any failed or missing test, or a smaller N.

**4.2 Two processes on one PC play the same race as one process.** The orchestrator saves the launcher and the verifier under "The m4 launcher and verifier" below as `scratch/s7-4-peers.sh` and `scratch/s7-4-verify.sh`. Then, in Git Bash from the Broadsword root:
```
G="--body scenarios/car_basic.bsn --track scenarios/tracks/oval.track --grid-gap 10 --duration 120"
mkdir -p scratch/s7-4
build/bin/broadsword.exe run crash-sim -- cargo run --offline --release -- $G --input scenarios/tracks/oval_lap.input --input2 scenarios/tracks/oval_lap_b.input --net-delay 6 --net-hash-every 100 > scratch/s7-4-ref.txt 2>&1
{ build/bin/broadsword.exe run crash-sim -- cargo build --offline --release; cp build/release/crash-sim.exe scratch/s7-4/crash-sim.exe && echo copied; } > scratch/s7-4-2-build.txt 2>&1
bash scratch/s7-4-peers.sh p 10 10 120 no > scratch/s7-4-2-run.txt 2>&1
bash scratch/s7-4-verify.sh > scratch/s7-4-2-verdict.txt 2>&1
```
The first command is the reference (run through `broadsword run` without a `timeout`: killing a `run` leaves an interrupted job that needs `recover`, and the run is 120 s of simulated time, about a second of wall time by the bench note of crash-sim `3a11077`, *reasoned*): the same program with no network, car 1 from `--input` and car 2 from `--input2`. **Scripted inputs under the input delay D:** the session applies the input given at tick t at tick t + D, and for a script the input given at tick t is the script's value at simulation time t x dt; so in a network run each car sees its script D ticks (60 ms) late, and the first D ticks are neutral. The reference run is given the same `--net-delay` and applies the same delay to both scripts, so the three runs must print identical lines. The other choice, sending the script values D ticks ahead so that they apply at their script times, is not used: a live player cannot send an input ahead of when he gives it, so it would test a different input history. The launcher starts the host on `127.0.0.1` with an OS-chosen port, reads the port from the host's `net listening` line, and starts the guest against it; the two are the copy of the built program, run directly (see "When a check runs a built program directly").
PASS iff the reference output has `run: ok`, `scratch/s7-4-2-build.txt` contains `run: ok` and ends `copied`, `scratch/s7-4-2-run.txt` has `host_exit=0` and `guest_exit=0`, and the last line of `scratch/s7-4-2-verdict.txt` is `m4-local: ok`. The verifier, in order: the host's, the guest's and the reference's `car`/`hash` lines (the `start`, `checkpoint`, `lap` and `final` lines of both cars, and the `hash` lines) are identical, character for character, in all three files; there are exactly 120 `hash` lines in each; each peer's last line is `net: ok ticks=12000 hashes=120 last_hash=<H>` with `H` equal to the value of its `hash tick=11999` line and equal on both peers; car 1 and car 2 each have at least one `lap` line with `valid=yes`; and the `cars` line has `gap_min_m` at least 4.40 and `near_substeps=0` (the two scripts are slot-specific, so a swapped or ill-aimed input shows as an invalid lap or as cars touching). A run where a file has no lines, or where the reference has no hash lines, fails on the counts.
FAIL: a difference between the three files (the network changed the simulation); a missing or different hash; a non-zero exit.

**4.3 A forced desync is detected and reported by both peers.** In Git Bash from the Broadsword root, after 4.2 (the copy of the program is in place): `bash scratch/s7-4-peers.sh d 10 11 120 no > scratch/s7-4-3-run.txt 2>&1`. The host gets `--grid-gap 10`, the guest `--grid-gap 11`: the one parameter both sides must agree on differs, so the cars start from different places and the first hash, at tick 99, differs. Then `for p in host guest; do tr -d '\r' < scratch/s7-4-d-$p.txt | tail -n 1; done > scratch/s7-4-3-last.txt 2>&1`, and `grep -c '^net: ok' scratch/s7-4-d-host.txt scratch/s7-4-d-guest.txt > scratch/s7-4-3-ok.txt 2>&1; grep -cE '^(car [12] |cars |hash tick=)' scratch/s7-4-d-host.txt scratch/s7-4-d-guest.txt > scratch/s7-4-3-hash.txt 2>&1`.
PASS iff `scratch/s7-4-3-run.txt` has `host_exit=1` and `guest_exit=1`; `scratch/s7-4-3-last.txt` has two lines that each match `^net: FAIL desync at tick 99 own=[0-9a-f]{16} peer=[0-9a-f]{16}$`, with the host's `own` equal to the guest's `peer`, the host's `peer` equal to the guest's `own`, and `own` different from `peer` on each; and `scratch/s7-4-3-ok.txt` and `scratch/s7-4-3-hash.txt` each have two lines, one per file, and both end `:0` (no `net: ok` line and no `car`, `cars` or `hash` line: an unconfirmed interval prints nothing; interval 0 has no race events anyway, since a car cannot reach the start line in the first second, 20 m from rest needs at least sqrt(2 x 20 / 8.1) = 2.2 s at the 8.1 m/s^2 traction limit of m2, so for this check the zero is the absence of a `hash` line, and the rule itself is tested by `no_race_lines_of_an_unconfirmed_interval_are_printed` in 4.1). The tick is 99 because the first hash is exchanged after tick 99 and the cars differed from tick 0: it is the first hash after the divergence, and a peer that noticed later, or never, fails.
FAIL: `net: ok` on either side; a desync reported at another tick; only one peer reporting it; a hang (the launcher runs each peer under `timeout 300`; a peer killed by it has exit 124 and fails).

**4.4 A peer that dies ends the other with a named error, not a hang.** In Git Bash from the Broadsword root, after 4.2: `bash scratch/s7-4-peers.sh k 10 10 3000 yes > scratch/s7-4-4-run.txt 2>&1`, then `tr -d '\r' < scratch/s7-4-k-host.txt | tail -n 1 > scratch/s7-4-4-last.txt; grep -c '^hash tick=' scratch/s7-4-k-host.txt > scratch/s7-4-4-hash.txt; grep -c '^net: ok' scratch/s7-4-k-host.txt > scratch/s7-4-4-ok.txt`. The run is 3000 s of simulated time (300000 ticks), so the race cannot finish before the guest is stopped: the bench car case (`examples/bench.rs --case car`, crash-sim `3a11077`) says 10 s of the scripted lap took about 50 ms, about 5 ms per simulated second per car, so 3000 s of two cars is expected to take 30 s or more of wall time, against the half-second or less the launcher needs to see a hash line and stop the guest (*reasoned from that note, not measured*); the launcher stops the guest and its child with `taskkill //F //T` once the host has printed its first hash line.
PASS iff `scratch/s7-4-4-run.txt` has `host_exit=1` and a `seconds_after_kill=` value of at most 15 (T is 5 s; 10 s is the margin); `scratch/s7-4-4-last.txt` matches `^net: FAIL (disconnected|timeout) at tick [0-9]+`; `scratch/s7-4-4-hash.txt` is at least 1 (the race was running when the guest died) and `scratch/s7-4-4-ok.txt` is 0.
FAIL: `net: ok`; exit 124 (the host hung); a host that never printed a hash (the race did not start, so nothing was tested).

**4.5 Waiting for a peer that never comes ends with a named error.** In Git Bash from the Broadsword root, after 4.2:
```
( cd projects/crash-sim || exit 2; X=../../scratch/s7-4/crash-sim.exe; C="--body scenarios/car_basic.bsn --track scenarios/tracks/oval.track --grid-gap 10 --duration 120 --net-delay 6 --net-hash-every 100 --net-timeout 5 --net-connect-timeout 3 --net-seed 1"
T0=$SECONDS; timeout 60 $X $C --host 127.0.0.1:0 --input scenarios/tracks/oval_lap.input > ../../scratch/s7-4-5a.txt 2>&1; echo "host_alone_exit=$? seconds=$((SECONDS - T0))"
T0=$SECONDS; timeout 60 $X $C --join 127.0.0.1:1 --input scenarios/tracks/oval_lap_b.input > ../../scratch/s7-4-5b.txt 2>&1; echo "guest_alone_exit=$? seconds=$((SECONDS - T0))" ) > scratch/s7-4-5-run.txt 2>&1
tr -d '\r' < scratch/s7-4-5a.txt | tail -n 1 > scratch/s7-4-5a-last.txt; tr -d '\r' < scratch/s7-4-5b.txt | tail -n 1 > scratch/s7-4-5b-last.txt
```
PASS iff `scratch/s7-4-5-run.txt` has `host_alone_exit=1` with `seconds=` 3 to 8 (a whole number from 3 to 8) and `guest_alone_exit=1` with `seconds=` 0 to 8; `scratch/s7-4-5a-last.txt` is exactly `net: FAIL no peer within 3.000 s`; and `scratch/s7-4-5b-last.txt` starts `net: FAIL cannot connect to 127.0.0.1:1: `.
FAIL: a hang (exit 124), an earlier exit of the host, or any other last line.

**4.6 The sim step uses no maths-library function that a second machine might compute differently.** Equal state hashes on one machine cannot show this: the same binary on two CPUs can give different last digits for `sin`, `cos`, `tanh` and the like (the Windows C runtime picks code by CPU features; a result can differ by one unit in the last place between CPUs), and one different bit becomes a desync at the next hash. The step lives in `src/lib.rs`, `src/solver.rs`, `src/collide.rs` and `src/vehicle.rs`, but a call anywhere in the library can reach it, so the scan covers every `.rs` file under `src/`, except `src/bin/` (the games' own programs, which the second game adds there). In Git Bash from the Broadsword root:
```
( cd projects/crash-sim || exit 2; grep -rHE --include='*.rs' --exclude-dir=bin '\.(sin|cos|tan|asin|acos|atan|atan2|sinh|cosh|tanh|asinh|acosh|atanh|exp|exp2|exp_m1|ln|ln_1p|log|log2|log10|powf|cbrt|hypot|sin_cos)\(' src | tr -d '\r' | sed -E 's/^([^:]+):[[:space:]]*/\1: /' | LC_ALL=C sort ) > scratch/s7-4-6.txt 2>&1
```
The known list, on crash-sim main `c067bdc` (the pinned file `docs/stage7-maths-pin.txt` is the output of the command above on that commit, 30 lines; read there, with the reason each is or is not a risk. The first pin, on `71c9ca4`, had 13 lines; three of its lines have moved to another file since and fifteen are new, all named below). In the step of a race: `let (s, c) = (self.steer * self.max_steer).sin_cos();` in `src/vehicle.rs` (the steered wheels' direction, **every step**) and `c_rr * load * radius * (spin.abs() * radius / ROLLING_RESISTANCE_RAMP_SPEED).tanh()` in `src/vehicle.rs` (`grep -n tanh src/vehicle.rs` prints `957:` that line; it is `rolling_resistance_moment`, called at `src/vehicle.rs:738` in the wheel update of every step as `let stop = rolling_resistance_moment(c_rr, n, w.p.radius, w.spin) * h / w.p.inertia;`, and `stop` changes the wheel's spin, which is state and is in the hash: this is the one library call whose last bit can reach the state hash by a plain path, and the steer `sin_cos` is the same kind; **every step**, added by crash-sim `4fd84b0`). In the step of a race of two or more cars, every substep (added by `36f5085`, n20-19a): `v.x.hypot(v.y).hypot(v.z)` in `src/car_contact.rs` (`norm`, line 184), the length of a node's motion and speed for the motion bound, of a normal and of a coordinate scale in the separating-plane test, and of a speed after the impulses; every result is only compared with a bound and decides whether the kernel refuses (`MotionBound`, `UnsupportedSweep`), and none is written into a position, a velocity or the hash, so a one-ulp difference between machines matters only when a compared value is within about an ulp of its bound, and then one machine refuses the race and the other does not (a desync or a one-sided `race: FAIL` in 4.7); it is the third per-step suspect, with the steer `sin_cos` and the `tanh`. Once per run: two `sin_cos` of a tilt angle in `src/sim.rs` (moved from `src/lib.rs` by `53d6736`) (the gravity tilt of the built-in cube, from a constant) and `Ok((grip_frac * first.mu).atan())` in `src/vehicle.rs` (the slope angle of `--slope-grip-frac`). Not in the lockstep step: `src/format.rs` (`grep -n 'longest.log2' src/format.rs` prints `979:    let k = longest.log2().ceil() as i32;`. It is in `tet_size`, called from the body file's validation (`src/format.rs:858`) and nowhere in the step. Its doc comment in the file says the scaling by 2^-k is exact except where a component becomes subnormal, and that the volume is bit for bit the plain triple product otherwise; so `k` only picks which exact power of two is used, and the verdict does not depend on it. A one-ulp difference of `log2` between two machines could change `k` only if the longest edge is within an ulp of a power of two, and then chooses between two exact scalings. Nothing from `tet_size` enters the sim state or the hash. Recorded, not a step call), `src/cli/race.rs` (`atan2` for the printed heading of the single-car `--track` run, a report; moved from `src/main.rs` by `4f2fa56`), `src/ai.rs` (two lines, the AI driver: in milestone 4 the inputs come from scripts, but the driver is exactly what would be put in the loop later), `src/input.rs` (`atan2` of the reported heading), `src/pulse.rs` and `src/settle.rs` (analysis and reports). Not in the lockstep step either: `src/race.rs` (three lines, added by `d9afb02`, n20-20a, the library AI race `race::run`: `heading.sin()` and `heading.cos()` in `wall_contact`, the read-only wall rule, and `d.z.atan2(d.x)` with `heading.cos()` and `heading.sin()` in the AI driver's car state; milestone 4's `race::lockstep` calls neither, and like `src/ai.rs` they are what an AI car would bring into a networked race later) and `src/solver_m5.rs` (eleven lines of `ln` and `exp`, the m5 solid solver, run only for a body with tets, `bsn 3`; the race body `car_basic.bsn` is `bsn 2` with none). `sqrt` is exact in IEEE arithmetic and `powi` is plain multiplication; neither is listed. The two src/tune.rs lines (tune-1) are not in the step: they compute a log sweep's points in `crash-sim tune` before any run, and the run receives each value only as text in the body file, so they are not a desync risk.
PASS iff `scratch/s7-4-6.txt` is exactly the 30 lines of `docs/stage7-maths-pin.txt` (in Git Bash from the Broadsword root, `diff --strip-trailing-cr projects/crash-sim/docs/stage7-maths-pin.txt scratch/s7-4-6.txt` prints nothing and exits 0; `--strip-trailing-cr` because a Windows checkout may give the file CRLF line ends), so that no new maths-library call enters the library without a decision here.
This check pins the list; it does not say the list is safe. 4.7 is the test of that. If 4.7 desyncs, the first suspects are the three per-step calls (the steer `sin_cos` and the `tanh` in `src/vehicle.rs`, and the contact kernel's `hypot` in `src/car_contact.rs`, which decides refusals only), and the remedy is a brief, not a looser check: replace each by a function built from `+ - * /` and `sqrt` only (so every IEEE machine gives the same bits), with a test against the library function and a re-run of the Stage 6 checks. That brief is the proposed **n20-27a**; the orchestrator may board it before 4.7 to save a failed LAN session.
FAIL: any other output (a new call, a removed call, a changed line).

**4.7 Two machines on the LAN** (the human check; Lazlo). Needs Lazlo's yes first (AGENTS.md: ask first for network services), and he is told that the host program will make Windows show a firewall prompt ("Allow access") on the first run, and that the host listens on the LAN while it runs. The same `crash-sim.exe` copy (the same file, copied, not rebuilt: a rebuild could differ) goes to both machines with `scenarios/` and the scripts. Host on one machine: `... --host 0.0.0.0:47000 --input scenarios/tracks/oval_lap.input`; guest on the other: `... --join <host LAN address>:47000 --input scenarios/tracks/oval_lap_b.input`, with the options of 4.2. The orchestrator saves both outputs, the exit codes, the two machines (computer name, Windows version, CPU model from `systeminfo`), the round-trip time of `ping -n 20 <other machine>` from each side, and the address used, in `evidence/stage-7-m4-human.md` before scratch can be wiped, and quotes Lazlo's answers verbatim: Q1 "Did both machines run the race to the end?", Q2 "Did you see anything freeze, stutter or jump?", and "What felt wrong, or not like two machines racing each other?", with one task id or `IDEAS.md` line per point.
PASS iff both runs print `net: ok ticks=12000 hashes=120 ...`, both `last_hash` values are equal, the filtered lines (`car` and `hash` lines, as in 4.2) of the two machines are identical to each other and to 4.2's reference run on the main PC (`diff`), and Q1 is yes in Lazlo's words. The record must say whether the two machines have the same CPU model: if they differ and all hashes still match, that is the evidence 4.6 asks for. A `desync` here is a finding, not a failure to hide: record the tick and both hashes, and 4.6's remedy is the next task. The "felt wrong" points are not gated, but each needs its task or IDEAS line before the milestone is tagged.
FAIL: any difference; `net: FAIL`; a missing record.

#### The m4 launcher and verifier

Save the first block as `scratch/s7-4-peers.sh` and the second as `scratch/s7-4-verify.sh`, unchanged. The launcher was run with a stand-in program (a shell script that printed the `net listening` line and hash lines) to see that it starts a host and a guest, reads the address, waits for both, and, with `yes`, stops the guest and its child with `taskkill //F //T` and reports `seconds_after_kill=`; it stopped the stand-in guest (throwaway scripts under `scratch/`, not committed). The verifier was run on synthetic files while this file was written: `m4-local: ok` on three matching files, and `m4-local: FAIL` after each of: a changed `lap` line on the guest, a missing hash line on the host, a wrong `last_hash`, `guest_exit=1`, a `gap_min_m` of 4.30, `near_substeps=3`, and an empty reference. That shows the scripts' logic only; the first real output is what shows the format matches.

```
# scratch/s7-4-peers.sh <prefix> <host grid gap> <guest grid gap> <duration s> <kill: yes|no>
# Starts a host and a guest on loopback from projects/crash-sim with the copy scratch/s7-4/crash-sim.exe.
# Writes scratch/s7-4-<prefix>-host.txt and scratch/s7-4-<prefix>-guest.txt; prints addr=, host_exit=, and
# guest_exit= (kill no) or seconds_after_kill= (kill yes: the guest is stopped after the host's first hash line).
P=$1; HG=$2; GG=$3; DUR=$4; KILL=$5
R=$PWD; X=$R/scratch/s7-4/crash-sim.exe; H=$R/scratch/s7-4-$P-host.txt; G=$R/scratch/s7-4-$P-guest.txt
cd projects/crash-sim || exit 2
C="--body scenarios/car_basic.bsn --track scenarios/tracks/oval.track --duration $DUR --net-delay 6 --net-hash-every 100 --net-timeout 5 --net-connect-timeout 30 --net-seed 1"
timeout 300 $X $C --grid-gap $HG --host 127.0.0.1:0 --input scenarios/tracks/oval_lap.input > $H 2>&1 &
HP=$!
for i in $(seq 1 60); do grep -q '^net listening addr=' $H && break; sleep 0.5; done
ADDR=$(grep -m1 '^net listening addr=' $H | sed -E 's/^net listening addr=//' | tr -d '\r'); echo "addr=$ADDR"
timeout 300 $X $C --grid-gap $GG --join "$ADDR" --input scenarios/tracks/oval_lap_b.input > $G 2>&1 &
GP=$!
if [ "$KILL" = yes ]; then
  for i in $(seq 1 600); do [ "$(grep -c '^hash tick=' $H)" -ge 1 ] && break; sleep 0.1; done
  taskkill //F //T //PID "$(cat /proc/$GP/winpid)" > /dev/null; T0=$SECONDS
  wait $HP; echo "host_exit=$?"; echo "seconds_after_kill=$((SECONDS - T0))"; wait $GP 2>/dev/null
else
  wait $HP; echo "host_exit=$?"; wait $GP; echo "guest_exit=$?"
fi
```

```
# scratch/s7-4-verify.sh: prints labelled lines and a last line m4-local: ok | FAIL
# reads scratch/s7-4-p-host.txt, scratch/s7-4-p-guest.txt, scratch/s7-4-ref.txt and scratch/s7-4-2-run.txt
bad=0; f() { echo "FAIL $1"; bad=1; }
F='^(car [12] (start|checkpoint|lap|final) |cars |hash tick=)'
src() { case $1 in host) echo scratch/s7-4-p-host.txt;; guest) echo scratch/s7-4-p-guest.txt;; ref) echo scratch/s7-4-ref.txt;; esac; }
for p in host guest ref; do tr -d '\r' < "$(src $p)" | grep -E "$F" > scratch/s7-4-$p.lines; done
echo "lines host=$(grep -c . scratch/s7-4-host.lines) guest=$(grep -c . scratch/s7-4-guest.lines) ref=$(grep -c . scratch/s7-4-ref.lines)"
cmp -s scratch/s7-4-host.lines scratch/s7-4-ref.lines || f "host lines differ from the reference"
cmp -s scratch/s7-4-guest.lines scratch/s7-4-ref.lines || f "guest lines differ from the reference"
for p in host guest ref; do h=$(grep -c '^hash tick=' scratch/s7-4-$p.lines); echo "hashes $p=$h"; [ "$h" = 120 ] || f "$p has $h hash lines, need 120"; done
for p in host guest; do
  last=$(tr -d '\r' < "$(src $p)" | tail -n 1); echo "last $p: $last"
  lh=$(grep '^hash tick=11999 ' scratch/s7-4-$p.lines | sed -E 's/.* value=//')
  [ -n "$lh" ] && [ "$last" = "net: ok ticks=12000 hashes=120 last_hash=$lh" ] || f "$p last line is not net: ok with the hash of tick 11999"
done
[ "$(tail -n 1 "$(src host)" | tr -d '\r' | sed -E 's/.*last_hash=//')" = "$(tail -n 1 "$(src guest)" | tr -d '\r' | sed -E 's/.*last_hash=//')" ] || f "last_hash differs between peers"
for k in 1 2; do n=$(grep -cE "^car $k lap n=[0-9]+ start_t=[0-9.]+ end_t=[0-9.]+ time_s=[0-9.]+ valid=yes reason=none path_m=" scratch/s7-4-ref.lines); echo "valid laps car $k=$n"; [ "$n" -ge 1 ] || f "car $k has no valid lap"; done
C=$(grep -m1 '^cars ' scratch/s7-4-ref.lines); echo "$C"
GM=$(echo "$C" | sed -E 's/.* gap_min_m=([^ ]+).*/\1/'); NR4=$(echo "$C" | sed -E 's/.* near_substeps=([^ ]+).*/\1/')
awk -v g="$GM" 'BEGIN { exit !(g != "" && g + 0 >= 4.40) }' || f "cars came closer than 4.40 m (or no cars line)"
[ "$NR4" = 0 ] || f "near_substeps is $NR4"
grep -qx 'host_exit=0' scratch/s7-4-2-run.txt && grep -qx 'guest_exit=0' scratch/s7-4-2-run.txt || f "an exit code is not 0"
if [ $bad = 0 ]; then echo "m4-local: ok"; else echo "m4-local: FAIL"; fi
```

**What guards against tuning to the test in milestone 4:** the network run must equal a run with no network (4.2), so the lockstep cannot change the physics; two different scripts make a mix-up of the cars visible; the forced desync uses a real mismatch and must be reported by both peers at the first hash (4.3); a dead or absent peer ends with a named error under a time limit (4.4, 4.5); a list of every maths-library call in the step is pinned (4.6), because equal hashes on one machine cannot show a machine difference; and the last word is two real machines (4.7).

### Milestone 5: a second game on the same toolkit

**The genres, and the pick.**

| Genre | What it asks of the engine | Verdict |
|---|---|---|
| **Demolition derby (recommended)**: 4 cars in a walled arena, last car still running wins | car-to-car contact (n20-19), beam yield and breaking (STAGE6), plane walls (STAGE6), the AI driver (n20-17) with another target, the state hash and the shell's files; all exist after milestone 2 | Reuses the whole toolkit; nothing new is needed from the engine, so "no engine change" is a real test and not a hope. The score comes from engine outputs (broken beams, contacts). |
| Top-down physics puzzle: push a node-beam crate onto a target with the car | body-to-body contact between a car and a non-car body | n20-19 gives car-to-car contact only; a crate needs a second contact rule in the core. The genre would force the engine change that the milestone forbids. |
| Stunt jumps: score for air time over a ramp | ramps (sloped planes) and a rollover rule | Walls are planes along the axes only; a ramp is a new collider kind in the core. |

The pick is the demolition derby. It is a different genre (an arena with no track, no laps, no start line and damage as the goal) on the same solver, bodies, cars, input script, walls and hash, which is what "a small game in another genre on the same toolkit" means. The rules are small: an arena `scenarios/derby/arena.bsn` (a flat pad with four wall planes, as `pad_and_wall.bsn` has one: a `plane` line each, at x = +30, x = -30, z = +30 and z = -30 m), four cars from `car_basic.bsn` placed 10 m from the centre on the four sides, facing it, a round of 60 s, one car driven by a script (or a person, see 5.5) and three by the m2 driver aimed at the nearest opponent. Damage is the crumple of STAGE6 (`docs/STAGE6.md`, "`--crumple-report`": the distance between the centroids of the car's `rear` and `front` groups at the start minus that distance now); `crumple_max_m` is the largest value over the round. A car is **out** when its `crumple_max_m` is at least 0.5 m, or when its speed has stayed under 0.5 m/s for 5 s after its first contact. The winner is the last car not out at the end; if several are not out, the one with the least `crumple_max_m` (ties go to the lowest car number); if all are out, the least `crumple_max_m` of all.

Why damage is the crumple and not broken beams: `car_basic.bsn` has no `break_strain` (its material is `chassis stiffness 300000 damping 200 yield_strain 0.02`), so its beams yield and never break, and a rule about broken beams could not trigger. The crumple is what STAGE6 already measures on this car.

**The numbers, and why.** Every number here is a first value, *reasoned, not measured*; n20-derby (below) measures the first run, reports it, and the check does not depend on the values being right, only on the report being consistent (5.3).

- **4 cars: the player's and three AI.** Three AI are what m2 runs, so the driver and its params file are reused unchanged; the player's car is the fourth.
- **Arena 60 m square, walls 30 m from the centre.** A car launched over its 10 m placement reaches at most sqrt(2 x 8.1 x 10) = 12.7 m/s (8.1 m/s^2 is the traction limit mu g of m2, 0.83 x 9.81), stopping from it takes 12.7^2 / (2 x 8.1) = 10 m, the full-lock turning circle of `car_basic.bsn` is 2.5 m / tan 30 degrees = 4.33 m (wheelbase 2.5 m between the mounts at x = -1.25 and 1.25, `max_steer_deg 30`) and the car is 4 m long: 10 + 4.3 + 4 = 18.3 m. Each car has 30 - 10 = 20 m behind its start, more than 18.3 m, so a car that overshoots the centre can brake and turn before the far wall. A smaller arena would make the walls the main opponent; a larger one lets the AI avoid contact for the whole round.
- **Placement 10 m from the centre.** Neighbouring cars start 10 x sqrt(2) = 14.1 m apart and opposite cars 20 m, all above the 4.40 m centre floor of m2 (so no car starts in contact), and the closing distance is short enough that the first contacts come within a few seconds: 10 m at 8.1 m/s^2 takes sqrt(2 x 10 / 8.1) = 1.6 s.
- **Round 60 s.** Pursuit from 14 to 20 m at the 8 m/s lap floor of m2 closes in about 3 s, so 60 s allows many separate contacts (5.3 needs at least one); it is also the length of STAGE6's crash drive up to the wall (check 3.6 runs 72 s), and by the bench car case (about 5 ms per simulated second per car, crash-sim `3a11077`) a 4-car round is about 1.2 s of wall time, *reasoned*.
- **Out at crumple 0.5 m.** STAGE6's 50 km/h wall crash of this car crumpled it 0.524 m (`crumple_delta_m=0.524154`, `evidence/stage-6-m3.md`); 0.5 m is that rounded down, so a car is out after taking about one 50 km/h barrier crash.
- **Damage floor 0.05 m (5.3).** STAGE6's check 3.6 uses `--crumple-min 0.05`; below it the crumple is not told from noise (an elastic box differs by under 0.002 m, STAGE6 check 2.4, a factor of 25).
- **Stuck: under 0.5 m/s for 5 s.** 0.5 m/s is the speed below which the drive viewer switches a held brake to reverse (`REVERSE_ENGAGE_SPEED_MPS` in `projects/render/examples/drive.rs`), so a car under it is stopped for practical purposes; 5 s is the steady window of milestone 1's skidpad (at least five periods of the slowest ride mode), so a car that has been under 0.5 m/s that long has settled and is not just between reverse and forward.

**What "without changes to the engine core" means, as a check.** The engine core is every file of `src/`, `tests/` and the two build files of each of the two repositories as they are on the commit before milestone 5 starts, except the files named "Not core" below. The check compares each repository's end ref (its HEAD while work is under way, its milestone tag at the end) with that commit and fails on any added, modified, deleted or renamed core file, except the new files of the second game (listed), plus one allowed line in `Cargo.toml`, `default-run = "crash-sim"`, if a second binary needs it. Two things keep the definition honest: the lists are read from this file *as it was at the start commit*, so an edit of this file later cannot move a file out of the core; and the check fails if no second-game file was added at all, so an empty comparison cannot pass. Racing's own files (`ai.rs`, `race.rs`, `shell.rs`, `speed_profile.rs`, `track.rs`, and their tests, including `timed_lap.rs` and `oval_pinch.rs`) are "Not core": they belong to game 1 and may be fixed. The files that all games share (`lib.rs`, `solver.rs`, `collide.rs`, `format.rs`, `vehicle.rs`, `input.rs`, `trajectory.rs`, `settle.rs`, `statehash.rs`, `net.rs`, `net/session.rs`, `menu.rs`, `settings.rs`, `save.rs`, `pulse.rs`, `reference.rs`, `vec3.rs`, and `main.rs`, the program of game 1, kept in the core because the second game must not need it) are core. In `render`, all of `src/` is core; the examples (the front ends of the games) are not.

The list below is the one to freeze. It was written on crash-sim `24e415d` and render `550a40c`; when milestone 5 starts, the orchestrator re-reads `src/` of both repositories, puts any file added since in the right list, and commits this file as the **freeze commit**; then, in the next commit (the **pin commit**), writes the freeze commits' hashes on the two `Engine core start commit` lines and tags the pin commit `stage-7-m5-start` in crash-sim. Nothing else in this file may start a line with the text those lines start with, or the check cannot read them.

```
Not core (crash-sim): src/ai.rs src/race.rs src/shell.rs src/speed_profile.rs src/track.rs tests/ai.rs tests/ai_race.rs tests/oval_pinch.rs tests/shell.rs tests/speed_profile.rs tests/timed_lap.rs tests/track.rs
New files allowed (crash-sim): src/bin/derby.rs src/bin/derby/ tests/derby.rs tests/data/derby/
Not core (render): examples/drive.rs examples/race.rs
New files allowed (render): examples/derby_window.rs
Engine core start commit (crash-sim): none yet
Engine core start commit (render): none yet
```

**Flags and output (n20-derby).** `derby --arena <file> --body <car.bsn> --ai <n> --input <file> --duration <s> --ai-params <file>` runs a round headless: `--ai` is 1 to 3 AI cars (the m2 driver and its params file, aimed at the nearest opponent), car 1 is the player's, driven by the script in `--input` (the format of `src/input.rs`); with `--live` and `--trajectory-v2` in place of `--input` it takes the player's lines on stdin and writes trajectory v2 frames of the whole arena, as `crash-sim --live` does, for the window of 5.5. The lines are those of 5.3. Exit 0 when the last line is `derby: ok`, 1 when it is `derby: FAIL ...`, 2 for a usage error or a refused file. These flags and the arena file are the m5 briefs' to create. The derby is crash-sim's second binary (`src/bin/derby.rs`), so `Cargo.toml` needs `default-run = "crash-sim"` in `[package]`: milestone 1 added no second binary and no such line, and plain `cargo run` must keep meaning crash-sim for every STAGE6 command (5.2 re-runs check 1.2 (c) to show it). That one line is the only `Cargo.toml` change 5.1 allows.

#### Checks

**5.1 The engine core is unchanged.** In Git Bash from the Broadsword root, the block below saved as `scratch/s7-5-core.sh`, then `bash scratch/s7-5-core.sh projects/crash-sim projects/render > scratch/s7-5-1.txt 2>&1` while the milestone is under way (it compares each repository's HEAD with its start commit), and, when it is done, again with the tag of the milestone as a third word, `bash scratch/s7-5-core.sh projects/crash-sim projects/render stage-7-m5 > scratch/s7-5-1-tag.txt 2>&1`: the orchestrator tags both repositories `stage-7-m5` after 5.1 to 5.5 pass, and the second run, on the tags, is the "no diff between the commit before m5 and the m5 tag" of the milestone.
PASS iff the **last line** of `scratch/s7-5-1.txt` is exactly `core-unchanged: ok`, which is printed only when the tag `stage-7-m5-start` exists in crash-sim, both start commits (read from this file as committed at that tag, never from the working tree or from a later edit) are single hex values that are ancestors of the end ref (HEAD, or the tag), each repository has at least one core file at its start commit, every difference in `src`, `tests`, `Cargo.toml` and `Cargo.lock` is a not-core file, a new allowed file or the one allowed `Cargo.toml` line, and at least one new allowed file was added in crash-sim (the second game exists). The output lists every file with its class (`not-core`, `new-allowed`, `changed`), the number of core files and the lists used, so the evidence shows what was compared. The script was run on 19 throwaway Git histories while this file was written (built by a throwaway test script under `scratch/`, not committed): it printed `core-unchanged: ok` for exactly these three: a derby binary and its test added (and a derby example in render); a derby binary added with a not-core file modified; a derby binary added with the `default-run` line added to `Cargo.toml`; and, for the two cases where the start lines of the file were edited after the pin tag (one to a bad render hash, one to `none yet`) with nothing else wrong, `ok` as well, because the lines are read at the tag; and `core-unchanged: FAIL` for the others: a core source modified, a new source file outside the allowed list, a second new binary not on the list, another line added to `Cargo.toml`, an existing core test modified, a core file deleted, a core file renamed, a render core file modified, no second-game file added, a bad render start commit in the pinned file, a missing `stage-7-m5-start` tag, a start line moved after the tag together with a core edit, a not-core list loosened after the start (which the script ignores, reading the lists at the start commit) with a core file edited, and a core file edited after a tag (it fails at HEAD and prints `core-unchanged: ok` with the tag as the third word). If milestone 5 needs a core change, that is a finding recorded in `RETRO.md`, and the change goes through the orchestrator as a new decision (and this check is re-run against a new start commit, never loosened).
FAIL: any `changed` line; a last line other than `core-unchanged: ok`.

```
# scratch/s7-5-core.sh <crash-sim repo> <render repo> [end ref, default HEAD]
cs=$1; rd=$2; E=${3:-HEAD}
PIN=stage-7-m5-start
PD=$(git -C "$cs" show "$PIN:docs/STAGE7.md" 2>/dev/null | tr -d '\r')
rd_line() { echo "$PD" | grep -E "^$1: " ; }
one() { [ "$(printf '%s\n' "$1" | grep -c .)" = 1 ] && printf '%s\n' "$1" | sed -E 's/^[^:]*: //'; }
ok=1
CS=$(one "$(rd_line 'Engine core start commit \(crash-sim\)')")
RS=$(one "$(rd_line 'Engine core start commit \(render\)')")
for pair in "crash-sim:$CS" "render:$RS"; do
  n=${pair%%:*}; c=${pair#*:}; [ $n = crash-sim ] && R=$cs || R=$rd
  if ! echo "$c" | grep -qxE '[0-9a-f]{7,40}' || ! git -C "$R" merge-base --is-ancestor "$c" "$E" 2>/dev/null; then
    echo "core-unchanged: FAIL $n start commit is not one commit that is an ancestor of $E (value: ${c:-missing})"; ok=0; fi
done
if [ $ok = 1 ]; then
  FD=$(git -C "$cs" show "$CS:docs/STAGE7.md" | tr -d '\r')
  list() { echo "$FD" | grep -E "^$1: " | sed -E 's/^[^:]*: //'; }
  for lab in 'Not core \(crash-sim\)' 'New files allowed \(crash-sim\)' 'Not core \(render\)' 'New files allowed \(render\)'; do
    [ "$(echo "$FD" | grep -cE "^$lab: ")" = 1 ] || { echo "core-unchanged: FAIL the list '$lab' is not on exactly one line of the file at the start commit"; ok=0; }
  done
  NEWF=0
  for n in crash-sim render; do
    if [ $n = crash-sim ]; then R=$cs; C=$CS; else R=$rd; C=$RS; fi
    NC=$(list "Not core \\($n\\)"); AN=$(list "New files allowed \\($n\\)")
    CORE=$(git -C "$R" ls-tree -r --name-only "$C" -- src tests Cargo.toml Cargo.lock | while read -r f; do case " $NC " in *" $f "*) ;; *) echo "$f";; esac; done)
    echo "$n start=$C core_files=$(echo "$CORE" | grep -c .) not_core=[$NC] new_allowed=[$AN]"
    [ "$(echo "$CORE" | grep -c .)" -ge 1 ] || { echo "core-unchanged: FAIL $n has no core file at the start commit"; ok=0; }
    DIFF=$(git -C "$R" diff --name-status --no-renames "$C" "$E" -- src tests Cargo.toml Cargo.lock) || { echo "core-unchanged: FAIL git diff failed in $n"; ok=0; DIFF=; }
    while IFS=$'\t' read -r st f; do
      [ -z "$st" ] && continue
      case " $NC " in *" $f "*) echo "not-core $st $f"; continue;; esac
      if [ "$f" = Cargo.toml ]; then
        X=$(git -C "$R" diff -U0 "$C" "$E" -- Cargo.toml | grep -E '^[+-]' | grep -vE '^(\+\+\+|---) ' | grep -vxF '+default-run = "crash-sim"')
        if [ -z "$X" ]; then echo "allowed-line $n Cargo.toml default-run"; else echo "changed $st $n:$f"; ok=0; fi; continue; fi
      if [ "$st" = A ]; then a=0; for p in $AN; do case "$f" in "$p"*) a=1;; esac; done
        [ $a = 1 ] && { echo "new-allowed $f"; [ $n = crash-sim ] && NEWF=$((NEWF+1)); continue; }; fi
      echo "changed $st $n:$f"; ok=0
    done <<< "$DIFF"
  done
fi
if [ $ok = 1 ]; then echo "second_game_files_added=$NEWF"; [ "$NEWF" -ge 1 ] || { echo "core-unchanged: FAIL no second-game file was added in crash-sim (nothing was checked)"; ok=0; }; fi
if [ $ok = 1 ]; then echo "core-unchanged: ok"; else echo "core-unchanged: FAIL"; fi
```

**5.2 Both games' tests pass in one run.** `broadsword test crash-sim > scratch/s7-5-2.txt 2>&1` and `broadsword test render > scratch/s7-5-2-render.txt 2>&1`.
PASS iff both end `test: ok — N passed, 0 failed`, crash-sim's N is at least the N of 4.1 plus the 6 tests named here, the file contains `Running tests\derby.rs` and each of `test derby_round_same_input_same_bytes ... ok`, `test derby_report_is_consistent_with_the_sim ... ok` (the 5.3 rules, in-process), `test derby_cars_make_contact_and_crumple ... ok`, `test derby_uses_no_track_or_lap_code ... ok` (the derby sources import none of `crash_sim::track`, `crash_sim::race` and `crash_sim::shell`, grouped imports included, and name no `LapTimer`; `ai` and `speed_profile` are allowed, since the derby reuses the m2 driver), `test derby_refuses_bad_flags ... ok`, `test derby_winner_rule ... ok`, and every test the earlier checks name still passes (the first game still works): `{ source scratch/s7-named.sh; for t in "ai 14" "speed_profile 10" "settings 9" "save 13" "menu 8" "net_codec 14" "net_session 18" "statehash 7" "timed_lap 8" "oval_pinch 2" "multi_car 5" "ai_race 8" "shell 7" "two_player 9"; do named scratch/s7-5-2.txt $t; done; } > scratch/s7-5-2-named.txt 2>&1` prints fourteen `named ...: ok` lines and no `MISSING` line (the unquoted `$t` gives `named` the stem and the count as two words). 5.1 checks that no core test file that existed at the start commit is changed; the not-core test files (`ai`, `ai_race`, `oval_pinch`, `shell`, `speed_profile`, `timed_lap`, `track`) may change. Check 1.2 (c) is run again on the m5 commit and must still print exactly `first_contact_s=0.6390,peak_strain=0.045146,final_center_y=0.4999` with `run: ok`: with two binaries that only holds if the `default-run` line is there.
FAIL: a failed or missing test, a smaller N, or one of the first game's tests missing.

**5.3 The derby round, headless.** Build and run, from the Broadsword root: `broadsword run crash-sim -- cargo run --offline --release --bin derby -- --arena scenarios/derby/arena.bsn --body scenarios/car_basic.bsn --ai 3 --input scenarios/derby/player.input --duration 60 --ai-params scenarios/ai_driver.params > scratch/s7-5-3.txt 2>&1`. The player's script `scenarios/derby/player.input` (n20-derby) drives car 1 forward at the nearest opponent. `--bin derby` is needed because crash-sim has two binaries (the `default-run` line, see "Where the second game lives"). The run goes through `broadsword run` without a `timeout`, as the reference of 4.2 does (killing a `run` leaves an interrupted job that needs `recover`); a 4-car round is about 1.2 s of wall time (the bench note above, *reasoned*).
PASS iff `run: ok` and the output has, in this order: a line `derby arena=arena cars=4 duration_s=60.000`; for each car k = 1 to 4 a line `car <k> hits=<n> crumple_max_m=<3 decimals> out=<yes|no> out_t=<4 decimals, or none>`; a line `winner car=<k> reason=<last_standing|least_crumple>`; and a last line `derby: ok`. And, for the report to be consistent, in Git Bash: the winner is a car whose `out` is `no`, unless all four are out (then the winner has the least `crumple_max_m` of all); a winner with `reason=last_standing` is the only car with `out=no`; `crumple_max_m` is at least 0.000 for every car; `out=yes` iff `out_t` is a number; every car with `crumple_max_m` of 0.500 or more has `out=yes`; and the sum of `hits` over the cars is at least 1 and the sum of `crumple_max_m` is at least 0.05 (cars hit each other and are damaged: a derby in which nothing is damaged is not a derby; 0.05 m is STAGE6's own crumple floor, above). `hits` is the number of separate contacts between two cars (a contact is the interval of substeps in which the centres are closer than 4.40 m, the m2 rule, counted once per interval and per pair, and credited to both cars).
FAIL: `run: FAILED`; a missing line; an inconsistent report; no contact or no damage. How the report is checked: in Git Bash, `grep -E '^(derby |car |winner |derby:)' scratch/s7-5-3.txt` gives the lines and the rules are read from them one by one; the derby test `derby_report_is_consistent_with_the_sim` of 5.2 asserts the same rules in code.

**5.4 The derby is deterministic and uses the toolkit, not a copy of it.** The 5.3 command twice, saved as `scratch/s7-5-4a.txt` and `scratch/s7-5-4b.txt`, and then, in Git Bash from the Broadsword root, `for f in s7-5-4a s7-5-4b; do tr -d '\r' < scratch/$f.txt | grep -E '^(derby |car |winner |derby:)' > scratch/$f.prog; done; cmp scratch/s7-5-4a.prog scratch/s7-5-4b.prog > scratch/s7-5-4-cmp.txt 2>&1; echo "cmp_exit=$? lines=$(grep -c . scratch/s7-5-4a.prog)" >> scratch/s7-5-4-cmp.txt`, and:
```
{ find projects/crash-sim/src/bin -name '*.rs' \( -name derby.rs -o -path '*/derby/*' \) | LC_ALL=C sort | xargs cat | tr '\n' ' ' | grep -oE 'crash_sim::(\{[^}]*\}|[A-Za-z_]+)' | sort -u > scratch/s7-5-4-uses.txt
echo "use_lines=$(grep -c . scratch/s7-5-4-uses.txt)"
n=0; for m in Sim Body vehicle input format collide solver; do if grep -qw "$m" scratch/s7-5-4-uses.txt; then echo "uses $m"; n=$((n+1)); fi; done; echo "toolkit_names=$n"
for m in track race shell; do grep -qw "$m" scratch/s7-5-4-uses.txt && echo "FORBIDDEN $m" || echo "free of $m"; done
echo "own_step_fns=$(find projects/crash-sim/src/bin -name '*.rs' \( -name derby.rs -o -path '*/derby/*' \) | LC_ALL=C sort | xargs cat | grep -cE 'fn (step|integrate|advance)|impl[^{]*Solver')"; } > scratch/s7-5-4-toolkit.txt 2>&1
```
The `tr` joins the lines, so a `use crash_sim::{collide, format,` group that runs over several lines is read as one group; `grep -w` then finds the module names inside a group or in a plain path.
PASS iff both runs have `run: ok`, `scratch/s7-5-4-cmp.txt` is exactly `cmp_exit=0 lines=<n>` with n at least 6 (the program lines of the two runs are identical, and not empty), and `scratch/s7-5-4-toolkit.txt` is exactly `use_lines=<n>` with n at least 1, then one `uses <name>` line for each toolkit name that the derby imports, then `toolkit_names=<n>` with n at least 2, then `free of track`, `free of race`, `free of shell`, `own_step_fns=0` (the derby builds on the engine, with at least two of the names `Sim`, `Body`, `vehicle`, `input`, `format`, `collide` and `solver`; names none of the racing modules `track`, `race` and `shell`; and has no integrator of its own). The rule names no one module as required: a game on the same solver must at least build a `Sim` from a `Body` (both are `pub` at the crate root, `src/lib.rs:25` and `:247`), but whether it also needs `collide` (`pub mod collide;`, `src/lib.rs:7`) or any other module is its own business, and a check that required one the derby does not use would force a change to the core, which 5.1 forbids. The `ai` and `speed_profile` modules are allowed: the derby reuses the m2 driver and its params file, so the rule must not forbid what the driver needs. The `derby_uses_no_track_or_lap_code` test of 5.2 is the other half: the second game is not the first game with a different name.
FAIL: a difference between the two runs; fewer than two toolkit names; a racing module; a private step function.

**5.5 It ships like the first game, and a person plays it** (the m3 checks again, on a folder that has the derby in it; the human check). `python -I -B scripts/package_race.py build --out scratch/s7-5-release --with derby` (the `--with derby` flag is added to the packaging script by the m5 brief n20-derby-pack) and `python -I -B scripts/package_race.py check --folder scratch/s7-5-release`, as in 3.4, and then the clean-folder run of 3.5 for the headless `derby.exe` instead of `race.exe`: first `rm -rf scratch/s7-5-clean; cp -r scratch/s7-5-release scratch/s7-5-clean`, then `( cd scratch/s7-5-clean || exit 2; timeout 120 env -i PATH="/c/Windows/System32:/c/Windows" SYSTEMROOT="C:\\Windows" ./derby.exe --arena arenas/arena.bsn --body cars/car_basic.bsn --ai 3 --input demo/player.input --duration 60 --ai-params demo/ai_driver.params ) > scratch/s7-5-5.txt 2>&1` (it runs from inside the clean folder, because the relative paths of the flags are relative to the folder the program starts in, and under `timeout 120`: the round is about 1.2 s of wall time by the bench note, so 120 s is a wide margin, *reasoned*; `timeout` is outside `env -i` because the stripped `PATH` would not find it. An exit of 124 is a hang and fails.) The release is built by the packaging script as in 3.4, with the static C runtime or the app-local `vcruntime140.dll` that m3 chose, and the copy of the built program narrows the shared-build race and does not close it (see "When a check runs a built program directly").
PASS iff 3.4's rules hold for the new folder (it now also lists `derby.exe`, `derby_window.exe`, `arenas/arena.bsn`, `demo/player.input`, `demo/ai_driver.params`), the clean run has the same lines as 5.3's (a `derby: ok` last line, the same four `car` lines, the same `winner` line: the packaged game plays the same round), and the human part of 3.6 is done for the derby: the same record format in `evidence/stage-7-m5-human.md`, with Q1 "Could you start the derby by double-clicking derby_window.exe?", Q2 "Could you drive your car and hit the others?", Q3 "Did the winner on screen match the printed winner line?" and "What felt wrong, or not like a different game from the racing one?". Two programs share a name stem and must not share an executable name: crash-sim's `src/bin/derby.rs` builds `derby.exe` (the headless game and the child), and the render example is `examples/derby_window.rs`, which builds `derby_window.exe` (n20-derby-view). The person double-clicks `derby_window.exe`; it starts `derby.exe --live` from its own folder as a child, as the drive viewer starts crash-sim, and draws all four cars (trajectory v2 of the whole arena). The release folder holds both, and `derby.exe` alone is what the headless run above uses.
FAIL: any rule of 3.4 or 3.5 failing on this folder, exit 124, a different winner line in the clean run, a missing record, or any answer other than yes.

**5.6 The retro.** The Stage 7 retro entry is due after 5.1 to 5.5 pass, and again carries the Stage 7 lesson: what the human checks found that the tests did not. This file keeps no further check for it.

**What guards against tuning to the test in milestone 5:** the engine core is frozen by commit and the lists are read from the start commit, not from the current file (5.1); an empty comparison cannot pass (a second-game file must exist); the report is checked for consistency, not for a value (5.3); the game must show contact and damage (hits and crumple), so it is a derby (5.3), and a check that the game reads damage the way STAGE6 measures it, not by a rule that could never trigger on this car; it has no integrator of its own (5.4); and it must ship and be played the way the first game was (5.5).

#### Where the second game lives

Inside crash-sim, as `src/bin/derby.rs` (and `src/bin/derby/` for its modules), with `tests/derby.rs` and `scenarios/derby/`, and a render example `examples/derby_window.rs` for its window. Nothing is added to `src/lib.rs`: a `pub mod derby;` would be a change to the core, and the game needs none, because everything it uses is already public. Cargo finds `src/bin/derby.rs` by itself; `default-run = "crash-sim"` in `[package]` (the one allowed line of 5.1, which milestone 1 did not add) keeps plain `cargo run` meaning the crash-sim program. The derby's own maths-library calls, if any, are under `src/bin/`, which the 4.6 pin does not scan; they are not in the lockstep step of milestone 4 and the derby is not networked.

## Who builds what

| Check | Brief(s) |
|---|---|
| 1.1 track tests, `oval.track`, `chicane.track`, the lap rule, `on_track`, the track grammar | nb-43 |
| 1.1 skidpad tests, 1.3, `skidpad.bsn`, `skidpad.input`, the tyre tuning log | nb-44 |
| 1.1 race tests (`timed_lap.rs`), 1.4, 1.6, the race program and `--live`, `oval_lap.input` | n20-13 (`t-20261121-001109-5`; nb-47 as proposed here is dropped) |
| 1.1 `oval_pinch` test, 1.5, `oval_pinch.track` | `t-20261121-015242` |
| 1.2 | every Stage 7 brief that touches crash-sim; nb-44 and nb-47 in particular |
| 1.7, the HUD, the viewer's `--sim-exe` and `--track`, the `hud` line | nb-48 (proposed) |
| 1.8 record, 1.9 run, `evidence/stage-7-m1.md`, `evidence/stage-7-m1-human.md`, `evidence/stage-7-m1-human.input` | nb-49 (proposed) |

Proposed briefs (for the orchestrator to put on the board):

- **nb-47** `racing: race program on the default crash-sim program (src/main.rs): car on the grid, LapTimer wired to the sim, race lines, --live child mode with track_line output`. On the board this is n20-13 (`t-20261121-001109-5`), which claims `src/main.rs`, `tests/timed_lap.rs` and `scenarios/tracks/oval_lap.input`; nb-47's separate binary, `src/race.rs`, `src/lib.rs` line and `default-run` line are dropped (the orchestrator's decision). Claims: `projects/crash-sim/src/main.rs`, `tests/timed_lap.rs` (new; check 1.1 uses this name), `scenarios/tracks/oval_lap.input` (new). (`oval_pinch.track` and its test moved to board task `t-20261121-015242`.) Depends on nb-43, nb-40 (crash-sim half) and nb-44 merged (the lap script is aimed with the final tyres). Done check: 1.1 (race tests), 1.4, 1.5, 1.6 and 1.2.
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
  - the target range 0.83 to 0.93 g with its sources, the ceiling, the variants, the measurement from motion only and over steady windows only, `skidpad.input` as constant steer with throttle levels held at least 20 s each (the brief says "slowly rising speed"; the levels are how it rises, so the limit is read from steady circles, not from a ramp), the test names and printed lines of 1.3, and the tuning log row format;
  - the range must not change after any commit to the six tuning files of 1.3 (e), and nb-44 makes no tuning commit until the range is confirmed and labelled *sourced* (1.3 g); the range is sourced (six `confirmed:` figures, read 2026-11-20 UTC) and 1.3 (g) ties its bounds to them;
  - **(not in the brief; conflict)** changing the tyre section of `car_basic.bsn` breaks STAGE6's `pad_and_wall_car_is_car_basic` (nb-41: `pad_and_wall.bsn` embeds the car byte for byte) and may push STAGE6 3.6's impact speed or lap out of its window. If nb-44 changes the tyres, its claims must add `projects/crash-sim/scenarios/pad_and_wall.bsn` (the same tyre change) and `projects/crash-sim/scenarios/pad_lap.input` (re-aimed if 3.6 fails, as STAGE6 allows), and it must pass check 1.2 here. If the measured limit is already in range with the nb-38/nb-39 tyres, nothing changes and the conflict does not arise.
- **nb-40** (`t-20261119-171333-39` and `-40`), for its F1 child-process viewer: this file names the viewer's flag for the program to start `--sim-exe <path>`. If nb-40 picks another name, nb-48 adds `--sim-exe` as well, or the orchestrator changes 1.8 here before nb-48 starts.
- **nb-47**: the race program's flags, placement on the grid, output lines and exit codes; no second binary and no `default-run` line (the program is the default `crash-sim`); the measured 1.6 difference in its report. (`oval_pinch.track`, which must differ from `oval.track` only in the widths of points 5 to 10, is board task `t-20261121-015242`'s.)
- **nb-48**: the HUD's character set and refusal; `M:SS.mmm` from whole milliseconds, parsed from the race line's text; the viewer never parses a track file; it forwards the child's race lines unchanged and prints the `hud` line at close; **(not in the brief)** the end line of 1.8: the saved input ends with a line at the child's `final` `t=` text, so a replay ends at the live close time (1.9).
- **nb-47** also: **(not in the brief)** in `--live`, end of input ends the run at the end of the current substep and the `final` line is printed at that time (1.8's T_end).

## Who builds what, milestones 2 and 3

Status is from the board as it was when this was written; brief files are in `state/next-batch-2026-11-20/`. "Proposed" briefs are not written yet: the orchestrator boards them, and each takes the text named in the last column from this file.

| Brief | Board task | State | What it makes | Checks it must make pass |
|---|---|---|---|---|
| n20-17 AI driver | `t-20261119-223123-5` | merged | `src/ai.rs`, `tests/ai.rs` (14 tests) | 2.1, 2.4 (a) |
| n20-18 speed profile | `t-20261119-223123-6` | merged | `src/speed_profile.rs`, `tests/speed_profile.rs` (10 tests) | 2.1 |
| n20-13 headless timed lap | `t-20261121-001109-5` | open (after n20-09) | `--track` with `--input`: milestone 1's lap lines; `oval_lap.input` | 1.4 to 1.6, and the lap lines that 2.4 (b) and 3.5 compare |
| n20-19 several cars in one sim | not on the board yet | brief written | car-to-car contact in `src/vehicle.rs` and `src/collide.rs`; `tests/multi_car.rs` | 2.1 (the five `multi_car` tests), 2.4 (b) |
| n20-20 AI race | not on the board yet | brief written | `--ai`, `--laps`, `--time-limit`, `--grid-gap`, `--ai-params`, `--save-inputs`, `--grid-slot`; `src/race.rs`; `scenarios/ai_driver.params`, `scenarios/tracks/oval_tight.track`; `tests/ai_race.rs` | 2.1 to 2.6 |
| n20-21 settings file | `t-20261119-223123-7` | merged | `src/settings.rs`, `tests/settings.rs` (9 tests) | 3.1, 3.2 (the canonical settings text) |
| n20-22 save file | `t-20261119-223123-8` | merged | `src/save.rs`, `tests/save.rs` (13 tests) | 3.1, 3.2 (the canonical save text) |
| n20-23 menu state machine | `t-20261119-223123-9` | merged | `src/menu.rs`, `tests/menu.rs` (8 tests) | 3.1, 3.2 (the state lines) |
| n20-26a game shell protocol | proposed | not written | `--shell` and its flags, `src/shell.rs`, `tests/shell.rs`, `docs/SHELL.md` | 3.1 to 3.3, 3.5 |
| n20-26 game front end and release folder | not on the board yet | brief written (a draft) | `projects/render/examples/race.rs`, `scripts/package_race.py`, `scripts/test_package_race.py` | 3.1, 3.4, 3.5; 3.6 with the orchestrator |
| Orchestrator, with Lazlo | | | `evidence/stage-7-m2.md`, `evidence/stage-7-m3.md`; `evidence/stage-7-m3-human.md` | 3.6, the evidence of every check |

## What the briefs must take from this file, milestones 2 and 3

Where a brief and this file differ, this file sets the check, and the brief's coder raises the difference with the orchestrator. Items marked **(not in the brief)** are additions or conflicts that the orchestrator must apply to the board brief before it starts.

- **n20-13** (`t-20261121-001109-5`):
  - `--track` also works with `--live` (the race of milestone 1, with `track_line` lines before the first trajectory row): n20-14 (the viewer) and n20-26 (the game's race) both start crash-sim as a live child and need the lap lines in that mode. The brief includes it by the orchestrator's note, with the tests `live_and_file_input_give_same_race_lines` and `live_prints_lap_lines_as_they_happen`; n20-13 is built and in review.
  - the `race:` line of a single-car run prints `best_lap_s=<3 decimals, or none>` as milestone 1 sets it; n20-26 and 3.5 read the best lap from it as text.
  - the program is the default `crash-sim` binary (decided by the orchestrator; see "Which program"), which the brief already says.
- **n20-17 and n20-18 (merged):** nothing to change. 2.4 (a) pins `ai.rs`: no top-level `use`, no `crate::`, `crash_sim::`, `Sim` or `State`. The race calls `Driver::update` (the version with the integral state) every 0.02 s and takes the profile from `speed_profile::profile` over the track's centreline points (x, 0, z); `wheelbase` and `max_steer` of the driver's `CarState` come from the body file (the front-to-rear axle distance and the largest steer angle of the steerable wheels), and n20-20 states in `docs/` where it read them.
- **n20-19:** car-to-car contact is **on** in the physics, and the m2 race requires **zero** of it (see "Car-to-car contact"): the brief's "contact or ghosting" is settled as contact. The five tests named in 2.1 and their rules: one car byte-identical to the single-car run; two cars driving apart never interact; a head-on test at low speed conserves momentum within a tolerance written in `docs/PHYSICS.md`; contact never creates energy (the settle energy rule); same inputs twice give the same bytes, with a fixed car order in every loop. **(not in the brief)** the multi-car sim exposes, per car, its chassis node positions and its wheel mount positions, which n20-20's wall rule reads.
- **n20-20:** every flag, file, line and exit code of "Milestone 2", the eight `ai_race` tests of 2.1, and the numbers (3 laps, 160 s, 48.6 s, 4.40 m, grid gap 10 m, 50 Hz). **(not in the brief)** claims add `projects/crash-sim/src/lib.rs` (the `pub mod race;` line; `tests/ai_race.rs` cannot reach the module without it; `lib.rs` is also claimed by n20-19, so sequence after it), `projects/crash-sim/scenarios/ai_driver.params`, `projects/crash-sim/scenarios/tracks/oval_tight.track` and `projects/crash-sim/.gitattributes` if `*.params` files need LF (as `*.track` does). **(not in the brief)** `--grid-slot <k>` in the single-car mode that n20-13 adds. **(not in the brief)** `--save-inputs` writes the saved script as 2.4 (c) reads it, including the last line at the text of `end_t`. The brief's "grid start positions from the track file (or a stated rule)" is the stated rule of "The race": slot k is `grid + (k - 1) x grid_gap` before the line.
- **n20-21, n20-22, n20-23 (merged):** nothing to change. The shell (n20-26a) prints `settings::write`'s canonical text and `Save::to_text`'s, which 3.2 pins byte for byte; the menu has no way to edit a setting, so the settings screen's `set` lines are the shell's, validated by `settings::validate`.
- **n20-26a (proposed):** the shell protocol of "Milestone 3" exactly (flags, event lines, output lines, exit codes), the seven `shell` tests of 3.1, `docs/SHELL.md` with the same text, and claims `src/shell.rs` (new), `src/lib.rs` (the `pub mod shell;` line), `src/main.rs` (the `--shell` branch), `tests/shell.rs` (new), `docs/SHELL.md` (new). It depends on n20-20 and n20-13 merged (both claim `main.rs` and `lib.rs`).
- **n20-26:** the front end's flags, the `--script` grammar, the release folder's layout, the `data` folder next to the executable, `game: ok` and `game: FAIL`, and the two commands of `package_race.py` (`build --out` and `check --folder`), with the allow-list of 3.4. The choice between a static C runtime and an app-local `vcruntime140.dll` is made and recorded in the evidence. **(not in the brief)** it depends on n20-26a, and the brief's claim is enough for the files it names. **(not in the brief)** the packaging script never writes outside `scratch/`; its unit tests hold the four cases of 3.1. **(not in the brief; conflict)** the brief's "child process or a small text protocol: say which" is settled here: the text protocol of n20-26a, with a crash-sim child.
- m4: see "Who builds what, milestone 4" and "What the briefs must take from this file, milestone 4" at the end of this file.
- m5: see "Who builds what, milestone 5" and "What the briefs must take from this file, milestone 5" at the end of this file.

## Who builds what, milestone 4

Scratch file names for milestone 4 (the rule of "Scratch file names" above: each check writes only files with its own number; 4.3 to 4.5 reuse 4.2's copy of the program and its launcher, which is stated here):

| Check | Files it writes under `scratch/` |
|---|---|
| 4.1 | `s7-4-1.txt`, `s7-4-1-named.txt` |
| 4.2 | `s7-4-ref.txt`, `s7-4/` (the program copy), `s7-4-2-*`, `s7-4-p-host.txt`, `s7-4-p-guest.txt`, `s7-4-host.lines`, `s7-4-guest.lines`, `s7-4-ref.lines`, `s7-4-peers.sh`, `s7-4-verify.sh` |
| 4.3 | `s7-4-3-*`, `s7-4-d-host.txt`, `s7-4-d-guest.txt` |
| 4.4 | `s7-4-4-*`, `s7-4-k-host.txt`, `s7-4-k-guest.txt` |
| 4.5 | `s7-4-5*` |
| 4.6 | `s7-4-6.txt` |

Status is from the board when this was written; the brief files are in `state/next-batch-2026-11-20/`.

| Brief | Board task | State | What it makes | Checks it must make pass |
|---|---|---|---|---|
| n20-13 headless timed lap | `t-20261121-001109-5` | built (with `--live`) | `--track` with `--input` and `--live`; `oval_lap.input` (two laps); `tests/timed_lap.rs` | 4.2 (the lap lines it prints) |
| n20-19 several cars in one sim | not on the board yet | brief written | the two-car sim, car-to-car contact | 4.2 to 4.4 |
| n20-20 AI race | not on the board yet | brief written | `--grid-slot`, `--grid-gap` | 4.1 (`oval_lap_b_drives_a_valid_lap_from_slot_2`), 4.2 |
| n20-24 state hash | `t-20261119-223123-10` | merged | `src/statehash.rs`, `tests/statehash.rs` (7 tests) | 4.1, 4.2 (the hash lines) |
| n20-25a message codec | `t-20261119-223123-11` | merged | `src/net.rs`, `tests/net_codec.rs` (14 tests), `docs/NET.md` | 4.1 |
| n20-25b lockstep session | `t-20261120-175151` | merged | `src/net/session.rs`, `tests/net_session.rs` (18 tests) | 4.1 to 4.5 |
| n20-27 two-player lockstep race | not on the board yet | brief written (a draft) | `--host`, `--join`, `--net-*`, `--input2`; `scenarios/tracks/oval_lap_b.input`; `tests/two_player.rs` | 4.1 to 4.5; 4.7 with the orchestrator |
| n20-27a deterministic maths for the step | proposed, optional | not written | `src/vehicle.rs` (the two per-step library calls of 4.6), a test against the library function, `docs/PHYSICS.md` | 4.6 (the pinned list changes in the same commit), 4.7 |
| Orchestrator, with Lazlo | | | `evidence/stage-7-m4.md`, `evidence/stage-7-m4-human.md` | 4.7, the evidence of every check |

## What the briefs must take from this file, milestone 4

Where a brief and this file differ, this file sets the check, and the brief's coder raises the difference with the orchestrator. Items marked **(not in the brief)** are additions or conflicts that the orchestrator must apply to the board brief before it starts.

- **n20-19 (for m4):** **(not in the brief)** the multi-car sim exposes its full state in a fixed order (every node, wheel and beam of each car, car 1 first), which the hash of n20-27 reads.
- **n20-20 (for m4):** `--grid-slot <k>` and `--grid-gap` are what the `oval_lap_b` test uses to run the script alone from slot 2; the m2 `cars` line definition is reused by m4.
- **n20-24 (merged):** nothing to change. n20-27 feeds the fields in the order of "Milestone 4" (tick, then car 1, then car 2: nodes, wheels, beams) and a NaN is a sim failure, not a hash.
- **n20-25a and n20-25b (merged):** nothing to change. `hello` carries the seed and the tick rate only, so D and N are flags that both peers are given (`--net-delay`, `--net-hash-every`); a difference between the two is a sequence error or a desync by design. The session's `Timeout`, `Disconnected` and `PeerLeft` errors are what 4.4 and 4.5 print as `net: FAIL ...`.
- **n20-27:** every flag, line and exit code of "Milestone 4" (including the `cars` line), the nine `two_player` tests of 4.1, the hash coverage rule, and the numbers (tick rate 100, D = 6, N = 100, T = 5 s, connect timeout 30 s, `--duration 120`). **(not in the brief)** claims add `scenarios/tracks/oval_lap_b.input` (new): the open-loop script for car 2 in slot 2, aimed from slot 2 like `oval_lap.input` is aimed from slot 1 (not a copy of it, not a time shift: see "Two scripts, one per slot"), two laps, last line before 120 s, tested by `oval_lap_b_drives_a_valid_lap_from_slot_2`; n20-27 depends on n20-19, n20-20 and n20-13 merged for that reason. It also claims the file that holds the two-car sim and its hash if it is not `net.rs`. **(not in the brief)** the host binds exactly the address it is given (`--host 127.0.0.1:0` for tests, `0.0.0.0:<port>` for the LAN) and prints `net listening addr=...` first. **(not in the brief)** the reference mode `--input2` with no network and with `--net-delay` applied. **(not in the brief)** 4.6 pins the maths-library calls under `src/` in `docs/stage7-maths-pin.txt` (28 lines on `bc209f9`; three of them in the per-step path of a two-car race); a new call anywhere in `src/` outside `src/bin/` is a decision for the orchestrator, not an edit of that check.
- **n20-27a (proposed, optional):** replace the two per-step library calls (`sin_cos` of the steer angle and `tanh` of the rolling resistance, in `src/vehicle.rs`) by functions built from `+ - * /` and `sqrt` only, tested against the library function over the range the sim uses, and re-run the Stage 6 checks (the default CSV unchanged, the car checks, the 3.6 crash). The two `sin_cos` of the tilt angle in `src/lib.rs` are computed once per run from a constant and are left for the same task to decide. 4.6's list changes in the same commit. The orchestrator boards it before 4.7 if it wants no failed LAN session; otherwise 4.7 is the test.

## Who builds what, milestone 5

Scratch file names for milestone 5 (each check writes only files with its own number; 5.4 reuses 5.3's command text and 5.5 reuses 3.4's script):

| Check | Files it writes under `scratch/` |
|---|---|
| 5.1 | `s7-5-1.txt`, `s7-5-1-tag.txt`, `s7-5-core.sh` |
| 5.2 | `s7-5-2.txt`, `s7-5-2-render.txt`, `s7-5-2-named.txt` |
| 5.3 | `s7-5-3.txt` |
| 5.4 | `s7-5-4a.txt`, `s7-5-4b.txt`, `s7-5-4a.prog`, `s7-5-4b.prog`, `s7-5-4-cmp.txt`, `s7-5-4-uses.txt`, `s7-5-4-toolkit.txt` |
| 5.5 | `s7-5-release/`, `s7-5-clean/`, `s7-5-5.txt` |

| Brief | Board task | State | What it makes | Checks it must make pass |
|---|---|---|---|---|
| n20-derby the derby game | proposed (at the start of m5) | not written | `src/bin/derby.rs`, `src/bin/derby/`, `tests/derby.rs`, `scenarios/derby/`, `docs/DERBY.md`, the `default-run` line of `Cargo.toml` | 5.1 to 5.4 |
| n20-derby-view the derby window | proposed (at the start of m5) | not written | `projects/render/examples/derby_window.rs` | 5.5 |
| n20-derby-pack the derby in the release folder | proposed (at the start of m5) | not written | `scripts/package_race.py` (`--with derby`), `scripts/test_package_race.py` | 5.5 |
| Orchestrator, with Lazlo | | | the freeze commit, the pin commit and the `stage-7-m5-start` tag; `evidence/stage-7-m5.md`, `evidence/stage-7-m5-human.md`; the tags `stage-7-m5` | 5.1, 5.5, the evidence of every check |

## What the briefs must take from this file, milestone 5

Where a brief and this file differ, this file sets the check, and the brief's coder raises the difference with the orchestrator.

- **The m5 briefs (proposed, written at the start of m5):** the genre, the rules, the numbers with their reasons and the flags of "Milestone 5"; damage is the crumple, not broken beams; the lists of the freeze commit; no change to any core file; no `pub mod` in `lib.rs`; the `default-run` line of `Cargo.toml` and nothing else in that file; the six `derby` tests of 5.2; `src/bin/derby.rs` builds `derby.exe` and the render example is `derby_window.rs`, so the two names differ. **(not in the brief)** the orchestrator writes the freeze commit, then the pin commit with the two start-commit lines and the `stage-7-m5-start` tag, before the first m5 task starts.
