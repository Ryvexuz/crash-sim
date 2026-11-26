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

## Unloading return (hysteresis)

A material may specify `unload_return u`, a finite fraction with `0 < u <= 1`.
`Sim::unload_return` has one entry per beam in beam order, mapped by the shared
`build_from_body` for both body and car constructors; the cube has only `None`.
The parser and the shared constructor refuse invalid values by name, including a
`Body` built in code. No scenario file opts in as part of this change.

After every physics substep of length `h`, the order is `integrate` (solver and
contacts), `apply_breaks`, `apply_yield`, `apply_unload`. The last pass is implemented
in `src/sim/unload.rs`, allocates nothing, and returns how many beams changed:

1. For each unbroken beam with `Some(u)`, set `d = pos[b] - pos[a]`, `len = |d|`.
   Skip a non-finite length or `len < 1e-12` m (the force-step cutoff); `check`
   reports a non-finite state.
2. Set deflection `x = len - rest` (negative in compression), lengthening rate
   `r = d . (vel[b] - vel[a]) / len` (m/s, after solver and contacts), and
   length-change estimate `delta = r * h`.
3. Unloading means `(x < 0 && delta > 0) || (x > 0 && delta < 0)`. Otherwise
   (loading, zero deflection, zero delta, or NaN) nothing changes.
4. Set `beta = 1/u - 1`. If `beta == 0` (`u == 1`), nothing changes: this case is
   bit-identical to no key.
5. Set `shift = beta * delta`. If `|shift| >= |x|`, set `rest' = len`, stopping
   deflection at zero without changing sign. Otherwise `rest' = rest - shift`,
   so `x' = x + shift` moves toward zero.
6. Apply only a positive finite `rest'`; otherwise keep the old rest length
   (the fail-closed guard). Positions, velocities, stiffness, damping, `rest0`
   and every other field are untouched.

While unloading, `x` shrinks by `(1 + beta)` per unit of lengthening, so the spring
returns `k*x0^2 / (2*(1 + beta)) = u * (1/2)*k*x0^2`. Each pass has
`|x'| <= |x|`: stored elastic energy `(1/2)*k*x^2` never rises; the pass only removes
energy. Loading never triggers the pass, so the loading stroke, peak force path
and step limit are unchanged. Rest may move below `rest0` (compression set) or
above it (tension set); `apply_yield` already uses its exact path outside
`[0.5*rest0, rest0]`.

`tests/unload_return.rs` covers parsing, canonical writing and refusals,
bit-identical no-key/u=1 runs, the unchanged loading stroke, the returned-energy
fraction on release, tiny-u zero capping and energy, both constructors and invalid
in-code values, and broken beams. Direct pass checks include tension, compression,
exact rational references, subnormal/overflow and near-cancelling inputs, and
non-finite or collapsed geometry.

Follow-up: `examples/crash_energy_a.rs`, `examples/wall_force_a.rs`,
`examples/profile_step.rs` and `tests/validation_files.rs` copy the substep sequence
by hand and do not yet call this pass. Their `validation_wall.bsn` has no key.
The substep-copy sync and crash step 2b's test-A measurement
(`ready-crash2-02-unload-test-a.md`) are separate tasks.

