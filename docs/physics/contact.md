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

