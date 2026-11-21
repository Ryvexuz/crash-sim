# The `.bsn` format, versions 1, 2 and 3

A `.bsn` file is a node-beam body as plain text: materials, nodes, beams, named groups, optional start velocities and collider planes. Version 2 adds a car on that body: wheels, their suspension and tyres, and the controls. Version 3 adds solids and linear tetrahedra (tets) on the body's nodes, for the m5 crush zone (`docs/M5-SOLVER.md` sections 5, 8 and 9). Code: `src/format.rs` (`parse`, `write`, `Body`). Tests: `tests/format.rs`. Scenario files: `scenarios/`.

## Grammar

One record per line. Tokens are separated by exactly one space (no tabs, no leading or trailing space). Line endings are `\n` and the file ends with one. Line 1 is `bsn 1`, `bsn 2` or `bsn 3`. After it, blank lines and lines starting with `#` are ignored (comments are not kept by `write`). Every other line starts with a keyword, and the keywords come in this order (a keyword may not follow a later one):

```
bsn 3
material <name> stiffness <k> damping <c> [yield_strain <y>] [break_strain <b>]
solid <name> plateau_stress <Pa> yield_strain <e> damping_ratio <z>
node <id> <x> <y> <z> mass <m>
beam <a> <b> <material> [rest <r>]
tet <a> <b> <c> <d> <solid>
group <name> <id> [<id> ...]
velocity <group> <vx> <vy> <vz>
wheel <name> mount <id> [<id> ...] radius <r> width <w> mass <m> inertia <i> steerable <yes|no> driven <yes|no> braked <yes|no>
suspension <wheel> spring <k> damper <c> rest <length> travel <min> <max>
tyre <wheel> mu <mu> radial_stiffness <k> radial_damping <c> long_stiffness <cx> lat_stiffness <cy>
controls max_steer_deg <a> max_drive_torque <t> max_brake_torque <t> [rolling_resistance <c_rr>] [drag_area <CdA>]
plane <px> <py> <pz> <nx> <ny> <nz>
```

The four lines `wheel`, `suspension`, `tyre` and `controls` are the car sections. They need version 2: a `bsn 1` file that uses one is refused (``` `wheel` needs version 2: this file is `bsn 1` ```). The two lines `solid` and `tet` are the solid sections and need version 3 (``` `tet` needs version 3: this file is `bsn 2` ```). Every other line means the same in every version, and a version 3 file may hold everything a version 2 file holds.

- `<name>`: letters, digits and `_`, starting with a letter or `_`. Material names are unique, solid names are unique, group names are unique, wheel names are unique. A solid and a material are different kinds: a beam names a material, a tet names a solid, and one cannot stand for the other.
- `<id>`: decimal digits, no sign, no leading zero, at most 4294967295. Node ids are unique. They need not be consecutive; a node's index in the sim is its position in the file.
- Numbers: anything Rust's `f64` parser reads, but it must be finite (`nan`, `inf` and `1e999` are refused).
- `material`: `stiffness` (N/m) > 0; `damping` (N s/m) >= 0; `yield_strain` and `break_strain`, if given, > 0 and in that order, and `break_strain` > `yield_strain` when both are given. Yield and break strain are stored now and used by later stages.
- `node`: any finite position (m); `mass` (kg) > 0. At least one node is required (an empty body is refused). A body with nodes but no beams is allowed.
- `beam`: both nodes must be declared above the beam, differ, and be at different positions (a zero-length beam is refused even when `rest` is given); the material must exist; a pair of nodes has at most one beam (either order). The rest length is the distance between the two nodes unless `rest <r>` gives it (`r` > 0).
- `group`: at least one node, each declared, none twice.
- `velocity`: sets the start velocity (m/s) of every node of an existing group (the sim starts the nodes with it); one line per group; two groups that both have a velocity must not share a node. Nodes in no such group start at rest.
- `plane`: a collider plane through a point with a normal (not zero by components; it is stored as written, not normalised, so a consumer must scale by the largest component before taking a length); the side the normal points to is free space. The sim loads it with `--body`: the ground plane (y = 0) is always a collider, and each `plane` line adds one (normalised as `collide::Plane::new` does, scaling by the largest component first); contact is the rule in `docs/PHYSICS.md`, "Plane contact". Planes come last, after the car, so a scenario that adds walls to a car file is that file with lines added at its end.

### The car sections (version 2)

Frame: y is up (the ground is y = 0). A car's long axis is x with its front toward +x, and z is across it; facing +x with y up, +z is the right side and -z the left. Wheels hang straight down at the start, so the suspension axis at the start is y; once the chassis moves, which axis the suspension follows is the vehicle code's rule (nb-39, `docs/PHYSICS.md`).

- `wheel`: `mount` lists one or more chassis nodes, each declared above, none twice (a wheel on a missing node is refused). The wheel's mount point is the centroid of its mount nodes; the suspension acts between that point and the wheel centre, and its forces go to the mount nodes. At the start the wheel centre is the mount point minus the suspension's `rest` along y (`Body::wheel_start_center`). `radius` (m, tyre outer radius) > 0; `width` (m, tyre width) > 0; `mass` (kg, the unsprung mass of the corner: wheel, tyre, hub, brake) > 0; `inertia` (kg m^2, about the axle) > 0. `steerable`, `driven` and `braked` are `yes` or `no`, nothing else.
- `suspension <wheel>`: exactly one per wheel, naming a wheel declared above. Lengths are from the mount point to the wheel centre. `spring` (N/m) > 0; `damper` (N s/m) >= 0; `rest` (m) > 0 is the length at which the spring carries no force, and the length at the start; `travel <min> <max>` are the bump stop and the droop stop (m), with 0 <= min < max, and `rest` strictly between them (a car never starts on a stop).
- `tyre <wheel>`: exactly one per wheel, naming a wheel declared above. `mu` (1) > 0 is the friction coefficient: the tyre's horizontal force is at most `mu` times its normal load (the friction circle of nb-39). `radial_stiffness` (N/m) > 0 and `radial_damping` (N s/m) >= 0 give the normal load from the tyre's penetration of a collider. `long_stiffness` (1: newtons of longitudinal force per newton of normal load per unit slip ratio) > 0 and `lat_stiffness` (1/rad: newtons of lateral force per newton of normal load per radian of slip angle) > 0 are the slopes of the tyre's force at small slip, before the `mu` cap. The model that uses these numbers, and what it leaves out, is nb-39's (`docs/PHYSICS.md`).
- `controls`: at most one line, and only when the body has a wheel. `max_steer_deg` (degrees, road-wheel angle of a steerable wheel at full steer) > 0 and < 90. `max_drive_torque` (N m, the total over all driven wheels at full throttle) > 0. `max_brake_torque` (N m, the total over all braked wheels at full brake) > 0. How the totals are split between wheels is the vehicle code's rule (nb-39, nb-40). At least one wheel must be steerable, one driven and one braked, so no control acts on nothing.
  Two optional keys may follow, in this order (task t-20261120-221319-2): `rolling_resistance` (1: the rolling resistance coefficient c_rr, newtons per newton of normal load) >= 0, and `drag_area` (m^2: CdA, the drag coefficient times the frontal area) >= 0. A missing key means 0, and the vehicle code then skips that force entirely, so a car file without the keys (and every body without wheels) runs byte for byte as before they existed; `parse` keeps a missing key as missing (`None`), so `write` does not add it. A negative value is refused (`rolling_resistance -0.01 must be >= 0`), and NaN or an infinity as for every number (``rolling_resistance `NaN` is not finite``). What the two numbers do, in `src/vehicle.rs` (`Car::pre_step`, `ROLLING_RESISTANCE_RAMP_SPEED`, `AIR_DENSITY`): every wheel gets the moment c_rr N r against its spin (N its normal load, r its radius), ramped smoothly to zero at standstill, which the tyre passes to the ground as a force c_rr N against the rolling direction; the chassis gets the drag 0.5 rho CdA v^2 against its centre-of-mass velocity v, shared over its nodes by mass, with rho = 1.225 kg/m^3.
- A body with a wheel must have, for every wheel, a `suspension` and a `tyre` line, and a `controls` line; a missing one is refused on the wheel's line (or, for `controls`, the first wheel's line). `suspension` and `tyre` lines need not follow the order of the wheels.
- A wheel starts with no start velocity of its own: the vehicle code (nb-39) gives it one from its mount nodes, if it needs one. `Body::build` builds the chassis only (nodes and beams); the wheels are built by the vehicle code.

### The solid sections (version 3)

A solid is the material of the m5 crush zone and a tet is one linear tetrahedron of it (`docs/M5-SOLVER.md` section 4). This format only stores them: `Body::build` and `Sim` ignore both in this version of the code (task n20-06a), and the solver that uses them (n20-11) is not on main. Tets add no mass: the node masses stay the lumped masses of the `node` lines (`docs/M5-SOLVER.md` section 5).

- `solid <name> plateau_stress <Pa> yield_strain <e> damping_ratio <z>`: after the `material` lines, all three keys, in this order. `plateau_stress` (Pa, the crush stress of the zone at zero compaction) > 0; `yield_strain` (1, the elastic strain at which the zone starts crushing) > 0 and < 1; `damping_ratio` (1) >= 0 and <= 1, both ends allowed. NaN and infinities are refused as for every number. What the numbers mean in the law is `docs/M5-SOLVER.md` section 8 (Q1 to Q3).
- `tet <a> <b> <c> <d> <solid>`: after the `beam` lines. Four distinct node ids, each declared above (a tet can only name earlier nodes, since nodes come first), and a solid declared above. Its rest shape is the four nodes' positions in the file; its rest volume is `V = (b - a) . ((c - a) x (d - a)) / 6`. The checks run in this order, and the first that fails is the refusal (each at the column of the first node id, except where named):
  - a node that is not declared (at its own column), or listed twice (at the second one);
  - an unknown solid (at the solid's column);
  - a tet on the same set of four nodes as an earlier tet, in any order and in any solid (`duplicate tet`);
  - a size that is not finite: `V` or an edge length overflows `f64` (very distant nodes; `refuse_tet_whose_volume_is_not_finite` puts two nodes 2e308 m apart);
  - `|V|` below `TET_MIN_VOLUME` = 1e-9 m^3 (1 mm^3; the minimum itself is allowed). Four coplanar or collinear nodes have `V` = 0 and are refused here;
  - `|V|` divided by the cube of the longest of its six edges below `TET_MIN_SHAPE` = 1e-6 (`is too flat`). A regular tet has 0.118, each of the 6 tets of a cube cut along a main diagonal 0.032. This catches a sliver whose volume is above the minimum only because it is large, and four nodes that are coplanar in decimal but whose rounded coordinates leave a small nonzero `V`;
  - `V` negative: the nodes are listed inside out (`is inverted`); swapping any two of them fixes it. The orientation rule is that `V` must be positive.
- A tet and a beam may share nodes (the coupling of `docs/M5-SOLVER.md` section 5). Nothing here checks that a mesh of tets is conforming: the format cannot know (section 5 says so).

## Canonical form

`write(&Body)` produces the canonical text: the header of the lowest version that holds the body (`bsn 3` with a `solid` or a `tet` line, else `bsn 2` with a car section, else `bsn 1`), the sections in the order above, one space between tokens, numbers printed by Rust's shortest round-trip form (`{}`: always the same digits that read back to the same value, never an exponent), and `rest` printed only when it differs from the exact distance between the nodes. So `parse(write(b)) == b` for every body `parse` can produce, and `write(parse(x)) == x` for every canonical file. A file with comments, blank lines, differently printed numbers (`1.0`, `1e3`), an explicit `rest` that equals the distance, `bsn 2` with no car section, or `bsn 3` with no solid section parses fine but is not canonical. Tets are written in file order with their nodes in the order given (the order is the orientation). `cube.bsn`, `box.bsn` and `car_basic.bsn` are canonical; `canonical_file_round_trips_byte_identical` and `car_basic_round_trips` check it. `every_scenario_file_writes_back_byte_for_byte` checks all 12 committed scenario files: each one's `write(parse(x))` is the file without its comment and blank lines (10 have none; `pad_and_wall.bsn` and `skidpad.bsn` have comments at the end), with the file's own header. `version_3_file_round_trips_byte_identical` checks a version 3 file with every line type.

## Refusals

`parse` returns `Err(ParseError { line, column, message })`; `Display` is `line 7, column 12: <message>` (column omitted when no single token is at fault, such as an empty body, which is reported on the last line). Nothing is ever defaulted: a missing or extra token, an unknown keyword, a wrong number, a wrong order, a bad version, a missing final newline, a carriage return or a tab is an error. Every refusal has a test in `tests/format.rs` that checks the line, the column and the words of the message. The car refusals: a car section in a version 1 file; a duplicate wheel; a wheel with no mount node, on a missing node, or listing a node twice; a radius, width, mass or inertia that is not > 0; a flag that is not `yes` or `no`; a suspension or tyre naming an unknown wheel, or a second one for a wheel; a wheel with no suspension or no tyre; a spring rate not > 0 or a damper rate below 0; a suspension rest length not > 0; travel min below 0, or not below travel max; a rest length not strictly inside the travel; `mu`, `radial_stiffness`, `long_stiffness` or `lat_stiffness` not > 0, or `radial_damping` below 0; a second `controls` line, `controls` without wheels, or wheels without `controls`; a steer angle not in (0, 90) degrees; a drive or brake torque not > 0; controls with no steerable, driven or braked wheel; and a `rolling_resistance` or `drag_area` that is below 0, not a number, not finite, missing its value, out of order or given twice (`refuse_bad_rolling_resistance_or_drag_area`). The solid refusals: a `solid` or `tet` line in a version 1 or 2 file; a duplicate solid or a bad solid name; a `plateau_stress` not > 0, a `yield_strain` not in (0, 1), a `damping_ratio` not in [0, 1], or any of the three not a number or not finite; a wrong, missing, extra or reordered key; a solid before a material or after a node; and each tet rule of "The solid sections" above, plus a tet before a beam or after a group, a node id that is not one, and a missing or extra token (the `refuse_*solid*` and `refuse_*tet*` tests, and `hostile_solid_and_tet_input_is_refused_or_bounded` for a 50 MB token and a flood of 20 000 solids and tets).

## Versions

The first line names the version. This parser reads versions 1, 2 and 3 and refuses any other number by name (`unknown version 4: this parser reads versions 1, 2 and 3`). Version 2 (nb-38) adds the car sections and changes nothing else, so one parser reads both layouts: a version 1 file loads exactly as before (`version_1_files_still_load_and_are_written_as_version_1` reads the committed v1 scenarios and writes them back byte for byte), and a `bsn 2` file without car sections loads to the same body as its `bsn 1` twin (`version_2_file_without_car_sections_loads_like_version_1`). Version 3 (n20-06a) adds the solid sections and changes nothing else, in the same way: a version 1 or 2 file loads and writes back exactly as before (`every_scenario_file_writes_back_byte_for_byte`), and a `bsn 3` file without solid sections loads to the same body as its lower-version twin and is written with that lower header (`version_3_file_without_solids_or_tets_loads_like_version_1_and_2`). When the schema changes again, the new version is a new branch of `parse` that keeps reading the old layouts (roadmap node 0.5).

## Scenario files

- `scenarios/cube.bsn`: today's cube for `Config::default()` (tilted 5 degrees, lowest corner 2 m up): 8 nodes of 1 kg, all 28 pairs as beams of one material (`cube_beam`, 20000 N/m, 20 N s/m), positions printed to the last digit. `cube_bsn_matches_default_cube` compares it bit for bit with `Sim::new(Config::default())`. It is generated from the sim's own arrays, so a change to the cube constants means regenerating it (the test then fails until you do).
- `scenarios/box.bsn`: the box for milestone 2: 2 x 1 x 1 m, 3 x 2 x 2 nodes at x in {0, 1, 2}, y and z in {0, 1} (long axis x, resting on y = 0), 1 kg each, node id = 4 ix + 2 iy + iz. Beams are all 28 pairs inside each of the two unit cells (edges, face and body diagonals; the 6 shared-face pairs once), 50 in all, of material `box_beam` (the cube's 20000 N/m and 20 N s/m, elastic). Groups `back` (nodes 0 to 3, x = 0) and `front` (nodes 8 to 11, x = 2).
- `scenarios/box_wall.bsn`: the box of `box.bsn` for the m2 done check: material `box_beam` (20000 N/m, 20 N s/m, `yield_strain 0.02`; the numbers and why are in `docs/PHYSICS.md`, "The box_wall material"), an extra group `all` (the 12 nodes), `velocity all 10 0 0` and `plane 2.1 0.5 0.5 -1 0 0`: a wall 0.10 m beyond the front face (x = 2), free space at smaller x. Four variants, each differing in exactly one number or one line (`box_wall_variants_differ_in_one_number` checks it): `box_wall_elastic.bsn` (no `yield_strain`), `box_wall_soft.bsn` (stiffness 10000), `box_wall_stiff.bsn` (stiffness 40000), `box_wall_slow.bsn` (`velocity all 7.07 0 0`).
- `scenarios/break_pull.bsn` and `break_pull_slow.bsn`: two 1 kg nodes 1 m apart at y = 2 m, one beam (material `pull_beam`, 20000 N/m, 20 N s/m, `break_strain 0.01`), groups `left` and `right` with opposite x velocities, 2.5 m/s (fast) or 0.75 m/s (slow). The test material for the breaking rule of `docs/STAGE6.md` check 2.9.
- `scenarios/car_basic.bsn`: the car of `docs/STAGE6.md` milestone 3 (nb-38), described below.

### car_basic.bsn

A version 2 file, canonical (so it holds no comments; its reasons are here). `car_basic_round_trips` checks the byte-identical round trip, and `car_basic_is_the_stage6_car` checks the size, the 1200 kg, the groups, front steer and rear drive, the mirror symmetry, the 0.20 m start height and the 1 ms step limit.

**Every number below is a starting value: chosen, reasoned or recalled, never measured on this car.** Labels: *reasoned* (derived here, working shown), *sourced* (from a public source that was read, named), *from memory, unverified* (recalled, not read). Nothing here was tuned to a test: no run of the car existed when they were chosen. Later tasks may change them only as their own briefs say (nb-44 changes the tyre section only, and records each try in `docs/PHYSICS.md`).

Chassis: 40 nodes on a grid, node id = 8 ix + 2 iz + iy, with x in {-2, -1.25, 0, 1.25, 2} (ix 0 to 4), z in {-0.9, -0.75, 0.75, 0.9} (iz 0 to 3) and y in {0.55, 1.95} (iy 0 floor, 1 roof). Beams are all 28 pairs inside each of the 12 cells between neighbouring stations, shared-face pairs once: 240 beams of one material, `chassis`. The most loaded node has 17 beams. Groups `front` (the 8 nodes at x = 2) and `rear` (the 8 at x = -2). It is mirror-symmetric about z = 0. Wheels `fl` (mount node 27), `fr` (29), `rl` (11) and `rr` (13): each hangs from one roof node at x = +-1.25, z = +-0.75.

| Number | Value | Source or reason |
|---|---|---|
| chassis size | 4.0 x 1.4 x 1.8 m (x, y, z) | The brief's "about 4 x 1.8 x 1.4 m", taken as the extent of the chassis nodes. *Reasoned.* |
| x stations | -2, -1.25, 0, 1.25, 2 | The wheels sit on the stations at +-1.25: wheelbase 2.5 m, overhangs 0.75 m. A small car's wheelbase is about 2.5 to 2.6 m (*from memory, unverified*). |
| z stations | +-0.75, +-0.9 | The wheels sit at +-0.75: track 1.5 m, about a small car's (*from memory, unverified*); the body is 1.8 m wide, so the tyres (0.195 m wide) are inside it, under wheel arches. |
| y stations | 0.55, 1.95 | Set by the start height below: the wheel centre starts at 0.20 + 0.3175 = 0.5175 m, the roof node above it at 0.5175 + 1.4325 = 1.95 m, and the floor 1.4 m lower. At rest (statics, *reasoned*: tyre deflection (270 + 30) x 9.81 / 200000 = 0.0147 m, spring sag 270 x 9.81 / 16655 = 0.1590 m) the floor is at about 0.176 m, a small car's ground clearance (*from memory, unverified*: about 0.14 to 0.17 m). |
| node mass | 40 kg floor, 14 kg roof | 20 x 40 + 20 x 14 = 1080 kg sprung, plus 4 x 30 kg wheels = 1200 kg, the brief's total. The split puts the mass low, as in a car (floor pan, engine, gearbox, tank and seats low; roof and pillars light): the whole car's centre of mass at rest is about 0.52 m up (*reasoned* from the rest heights above), near a small car's 0.5 to 0.55 m (*from memory, unverified*). With track 1.5 m that is a static stability factor 1.5 / (2 x 0.52) = 1.45, above the 0.83 to 0.93 g grip range of `docs/STAGE7.md`, so the car slides before it tips on a flat pad (quasi-static, *reasoned*). Uniform masses per layer put the centre of mass at x = 0, so the static load is 50/50 front to rear. |
| `chassis` stiffness | 300000 N/m | Stiff against the suspension and stable at the default 1 ms step. Step limit (*reasoned*, `solver::body_step_error`): w2 = 2 x 17 x 300000 / 14 = 728571 /s^2, g = 2 x 17 x 200 / 14 = 485.7 /s, and h^2 w2 + 2 h g = 0.729 + 0.971 = 1.70 < 4 at h = 1 ms (limit 1.77 ms). A lumped beam stands for a whole panel, so this is not a material's modulus. Under the full weight the chassis' static deformation is millimetres (a 2.6 kN corner load over a node with 17 beams of 300 kN/m). |
| `chassis` damping | 200 N s/m | Takes out chassis ringing: 0.035 of critical for one node of the mean 27 kg on one beam (200 / (2 sqrt(300000 x 27)), *reasoned*), the few-percent structural damping usual for a body (*from memory, unverified*); inside the step limit above. |
| `chassis` yield_strain | 0.02 | The crash material of nb-41, chosen before any run. The front cell (x from 1.25 to 2) has 40 beams with an x component; at yield each pushes back k x strain x |dx| = 300000 x 0.02 x 0.75 = 4.5 kN along x, 180 kN in all. A 1200 kg car at 50 km/h has 115.7 kJ, so a crude crush estimate is 115.7 / 180 = 0.64 m, inside the 0.05 to 1.0 m that `docs/STAGE6.md` check 3.6 allows, and near a real car's frontal crush at that speed (*from memory, unverified*: about 0.5 to 0.7 m). The estimate is good to about a factor of 2 (the box_wall estimate in `docs/PHYSICS.md` was 3.6 times the measured kept crumple). No `break_strain`: milestone 3 gates crumple, not breaking. |
| wheel `radius`, `width` | 0.3175 m, 0.195 m | A 195/65 R15 tyre, a common small-car size (*from memory, unverified*). The numbers follow from the size code (*reasoned*): width 195 mm; sidewall 0.65 x 195 = 126.75 mm; diameter 15 in x 25.4 + 2 x 126.75 = 635 mm, radius 0.3175 m. |
| wheel `mass` | 30 kg | Unsprung mass per corner (wheel, tyre, hub, brake, half the links): about 25 to 40 kg on a small car (*from memory, unverified*). |
| wheel `inertia` | 1.512 kg m^2 | A uniform disc of the wheel mass, 0.5 x 30 x 0.3175^2 = 1.512 (*reasoned*; a real wheel's tyre mass sits further out, but part of the unsprung mass does not spin, and the two roughly cancel). |
| steerable, driven, braked | front steer, rear drive, all braked | The brief. |
| `spring` | 16655 N/m | Ride frequency 1.25 Hz at the corner's sprung mass of 1080 / 4 = 270 kg: k = 270 x (2 pi x 1.25)^2 = 16655 (*reasoned*). 1.25 Hz is the middle of Gillespie's 1 to 1.5 Hz quoted in `docs/STAGE6.md` (check 3.3 e allows 1.0 to 2.0 Hz). |
| `damper` | 1272 N s/m | Damping ratio 0.3 at that corner: c = 2 x 0.3 x sqrt(16655 x 270) = 1272 (*reasoned*); 0.3 is the middle of the 0.2 to 0.4 quoted in `docs/STAGE6.md` (check 3.3 e allows 0.2 to 0.5). |
| `rest` | 1.4325 m | Mount (roof node, 1.95 m) to wheel centre (0.5175 m) at the start, so every tyre's lowest point starts 0.20 m above the ground with the suspension at its rest length (`docs/STAGE6.md` milestone 3). The spring has no preload, so the length is long: what the physics sees is the rate and the travel, not the length. |
| `travel` | 1.1825 to 1.5325 m | Bump stop 0.25 m below the rest length, droop stop 0.10 m above it. At rest the spring is 0.159 m shorter than `rest`, so 0.091 m of bump travel is left, about a small car's (*from memory, unverified*: about 0.08 to 0.10 m). On the bump stop the floor is still about 0.085 m above the ground (*reasoned*: 0.303 + 1.1825 - 1.4), so a bottomed suspension does not put the chassis on the ground. |
| `mu` | 0.88 | The middle of the 0.83 to 0.93 g skidpad range of `docs/STAGE7.md` (*sourced* there: Car and Driver figures quoted by two public pages, read 2026-11-20). With a friction-circle tyre whose mu does not depend on load, the steady cornering limit on a flat pad is at most mu g (`docs/STAGE7.md`), so 0.88 is a starting point for nb-44, not a result. |
| `radial_stiffness` | 200000 N/m | A passenger tyre's vertical stiffness, about 150 to 250 kN/m (*from memory, unverified*). With the 30 kg wheel it puts wheel hop at sqrt(200000 / 30) / (2 pi) = 13 Hz (*reasoned*), in the usual 10 to 15 Hz (*from memory, unverified*). |
| `radial_damping` | 500 N s/m | About 0.1 of critical for wheel hop: 500 / (2 sqrt(200000 x 30)) = 0.10 (*reasoned*); a tyre damps little. |
| `long_stiffness` | 15 | Longitudinal force per unit load per unit slip ratio: a road tyre reaches its peak force at about 0.1 slip ratio (*from memory, unverified*), and 15 x 0.06 = 0.9, so the linear part reaches mu = 0.88 at a slip ratio of 0.059, below that peak. |
| `lat_stiffness` | 10 /rad | Cornering stiffness per unit load: about 0.15 to 0.2 per degree for a road tyre (*from memory, unverified*), that is about 9 to 11 per radian. The linear part reaches mu = 0.88 at 0.088 rad (5.0 degrees) of slip angle. |
| `max_steer_deg` | 30 | A small car's road-wheel lock, about 30 to 35 degrees (*from memory, unverified*). Kinematic full-lock radius 2.5 / tan 30 = 4.33 m (*reasoned*), and at the 2 m/s of `full_lock_circle_matches_kinematic_radius` the lateral acceleration is 2^2 / 4.33 = 0.92 m/s^2, 0.094 g, under the 0.1 g that test assumes. |
| `max_drive_torque` | 2000 N m | Total at the driven wheels in first gear: an engine of about 150 N m through an overall first-gear ratio of about 13 (*from memory, unverified*). 2000 / 0.3175 = 6299 N, above the rear tyres' grip of 0.88 x 600 x 9.81 = 5180 N, so a full-throttle launch is grip-limited (`launch_matches_torque_or_grip_limit` takes the smaller of the two). |
| `max_brake_torque` | 5000 N m | Enough to lock all four wheels (*reasoned*): mu x M g x r = 0.88 x 1200 x 9.81 x 0.3175 = 3289 N m, and 5000 locks them up to mu = 1.34, so the braked car of `docs/STAGE6.md` check 3.4 slides on its tyres, not on rolling wheels, above the grip slope. |
| `rolling_resistance` | 0.012 | Added by task t-20261120-221319-2 (after nb-38, so not a starting value of the table above): *typical value, from memory, unverified*, a passenger-car tyre on dry asphalt has c_rr of about 0.010 to 0.015 (the range quoted in vehicle dynamics texts such as Gillespie, *Fundamentals of Vehicle Dynamics*, and Wong, *Theory of Ground Vehicles*, as recalled, not read). 0.012 x 1200 kg x 9.81 = 141 N, 0.0120 g of deceleration. |
| `drag_area` | 0.65 m^2 | Added by the same task: *typical value, from memory, unverified*, a small car has a drag coefficient Cd of about 0.30 to 0.35 and a frontal area of about 2.0 to 2.2 m^2, so CdA of about 0.60 to 0.77 m^2; 0.65 m^2 is Cd 0.31 on 2.1 m^2. At the 13.85 m/s of the coast-down test the drag is 0.5 x 1.225 x 0.65 x 13.85^2 = 76 N; at the 50 km/h of check 3.6 it is 77 N. Together with the rolling resistance the coast-down from 13.85 m/s decelerates at (141 + 76) / 1260 = 0.173 m/s^2 (0.0176 g; 1260 kg is the car's mass plus its wheels' I / r^2), in the 0.01 to 0.02 g that `docs/DRIVE_FEEL.md` fix 3 estimated; `tests/coast_down.rs` measures it. |

## Line endings in a Windows checkout

The format is strict about the line ending `\n` (LF). Git on Windows with `core.autocrlf=true` would check text files out with CRLF, which `parse` refuses by name. The repo now enforces LF: `.gitattributes` has `*.bsn text eol=lf` and `*.input text eol=lf`, and the tests read scenario files exactly as checked out (`scenario_files_have_no_carriage_returns` fails on a CRLF copy).

## The `.track` format, version 1

A `.track` file is a race track on the ground plane (y = 0) as plain text: a closed centreline, a width, a start/finish point, checkpoints and the grid distance. Code: `src/track.rs` (`Track::parse`, `LapTimer`). Tests: `tests/track.rs`. Files: `scenarios/tracks/`. The lap rule itself is in `docs/STAGE7.md` ("Shared definitions"); this section is the file grammar and what the code does with it.

```
track 1
width <w>
start <i>
grid <m>
checkpoint <i>
[checkpoint <i> ...]
point <x> <z> [width <w>]
point <x> <z> [width <w>]
point <x> <z> [width <w>]
[point ...]
```

Same layout rules as `.bsn`: one record per line, tokens separated by exactly one space (no tabs, no leading, trailing or doubled spaces), line endings `\n` and the file ends with one, line 1 is `track 1`, blank lines and lines starting with `#` after it are ignored. The keywords come in the order above (`width`, `start`, `grid`, `checkpoint`, `point`); `width`, `start` and `grid` appear once, `checkpoint` and `point` repeat.

- `width <w>`: the track width in metres, finite and > 0. Every point has it unless the point gives its own.
- `point <x> <z>`: a centreline point, finite, in driving order. The loop closes from the last point to point 0. The optional `width <w>` is that point's own width (finite, > 0); it replaces the one width at that point only. Along a segment the width is interpolated linearly between its two ends.
- `start <i>`: the index of the start/finish point (0-based, decimal digits, no sign, no leading zero).
- `checkpoint <i>`: a checkpoint index, at least one line. Counting cyclically from the start index, the checkpoints must be strictly increasing, and none may equal the start index.
- `grid <m>`: the car starts this far before the start line, measured backwards along the centreline from the start point; finite, 0 or more, and not longer than the loop.

The gate at point i is the segment through that point, perpendicular to the tangent there (the direction p[i+1] - p[i-1]), as long as the width at i. The start/finish line is the gate at the start index and each checkpoint is the gate at its index. `Track::on_track` is true when a position's distance to the centreline polyline is at most half the width at the nearest point.

### Refusals

`Track::parse` returns `Err(ParseError { kind, line, column, message })`; `Display` is `line 7, column 12: <message>` (no column when no single token is at fault). `kind` is one of these named errors, and nothing is ever defaulted:

| `kind` | refused when |
| --- | --- |
| `Header` / `UnknownVersion` | the file is empty or line 1 is not `track 1` / a version other than 1 |
| `CarriageReturn` | a `\r` anywhere (CR or CRLF line endings), at the line and column of the first one |
| `Tab`, `Spacing`, `MissingFinalNewline` | a tab; a leading, trailing or doubled space; no final `\n` |
| `UnknownKeyword`, `OutOfOrder`, `Syntax` | a keyword the grammar lacks; a keyword after a later one or a second `width`, `start` or `grid`; a missing or extra token |
| `BadNumber`, `NonFinite` | a token that is not a number or not a plain index; `nan`, `inf` or an overflowing number |
| `NonPositiveWidth` | a `width` (the track's or a point's) that is zero or negative |
| `BadGrid` | `grid` below 0, or longer than the centreline loop |
| `MissingLine` | no `width`, `start`, `grid` or `checkpoint` line |
| `TooFewPoints` | fewer than 3 points (reported at the last point line) |
| `ZeroLengthSegment` | two neighbouring points (the last and the first count) at the same place |
| `SelfIntersecting` | any two segments of the closed loop that are not neighbours share a point (touching counts), or two neighbours fold back over each other (a spike, three collinear points) |
| `StartOutOfRange`, `CheckpointOutOfRange` | an index that is not a point |
| `CheckpointIsStart`, `CheckpointsOutOfOrder` | a checkpoint equal to the start index; checkpoints (a repeat included) not strictly increasing from the start index |

### Lap timing

`LapTimer::update(prev_pos, pos, prev_t, t)` takes one substep: the car moved in a straight line from `prev_pos` at `prev_t` to `pos` at `t` (positions are `Vec3`; x and z are used). It returns the events the step caused, in time order: `Start`, `Checkpoint { lap, k, t }`, `Lap(LapRecord)`. A crossing is a meeting of that straight move with a gate at a fraction s of the step with 0 < s <= 1, so a car that stops exactly on the line crosses once; its time is `prev_t + s * (t - prev_t)`, never rounded to the step. A lap is valid only if every checkpoint was crossed forward in order and no substep of the lap ended off track; `MissedCheckpoint` is reported before `OffTrack`. Backward crossings of any gate never count. Off track is judged at the end position of every substep, and a substep that contains the start-line crossing belongs to the lap that starts there. A position or time that is not finite, or a `t` earlier than `prev_t`, is a `TimerError` and leaves the timer unchanged. `LapRecord::time_ms` is `round((end_t - start_t) x 1000)`; `format_seconds` and `format_clock` print that one integer as `83.456` and `1:23.456`.

### Files

- `scenarios/tracks/oval.track`: the oval of `docs/STAGE7.md` (64 points, width 12 m, start 0, checkpoints 16, 32 and 48, `grid 20`, centreline 388.4094 m). Numbers are rounded to 12 decimals.
- `scenarios/tracks/chicane.track`: a 12-point loop with an S-bend on the front straight, a start gate whose tangent is not along an axis, width 10 m with point 3 narrowed to 8 m, checkpoints 4, 7 and 9, `grid 15`.

`.gitattributes` has `*.track text eol=lf`, as for `.bsn`, so a Windows checkout does not give the strict parser CRLF files (`track_files_have_no_carriage_return` checks the committed files).
