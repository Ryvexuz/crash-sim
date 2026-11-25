# crash-sim

The crash sim and racing core for Broadsword: node-and-beam bodies, cars on
tracks, AI drivers, a game shell and two-player lockstep races. Headless, std
only, deterministic: the same flags give byte-identical output. With no mode
flag it runs the starter scene: a cube of 8 nodes (1 kg each) joined by 28
spring-damper beams (12 edges, 12 face diagonals, 4 body diagonals) that drops
under gravity onto a ground plane at y = 0, bounces and settles.

## Run

```
cargo run --offline                       # 5 s, prints CSV to stdout
cargo run --offline -- --duration 2 --drop-height 3 --no-tilt
cargo test --offline
```

## Modes

`src/main.rs` picks the mode from the flags, in this order (the first match wins):

1. **Tune**: chosen by a leading `tune` command; usage `crash-sim tune --body <file.bsn> --out <fresh-dir> [--tune <file>]... [--set <keyword>.<name>.<field>=<value>]... [--jobs <n>] [--show <prefix>]... -- <run flags...>`; doc [runtime tuning](docs/TUNING.md).
2. **Game shell**: chosen by `--shell`; usage `crash-sim --shell`; doc [shell protocol](docs/SHELL.md), checks 3.1 to 3.3 of [Stage 7 milestone 3](docs/STAGE7.md).
3. **Network peer (one of two machines)**: chosen by `--host <addr:port>` or `--join <addr:port>`, without `--input2`; usage `usage: crash-sim --body <car.bsn> --track <file> --grid-gap <m> --duration <s> (--host <addr:port> | --join <addr:port>) --input <file> --net-delay <ticks> --net-hash-every <ticks> --net-timeout <s> --net-connect-timeout <s> --net-seed <u64>`; docs [net messages](docs/NET.md), [Stage 7 milestone 4](docs/STAGE7.md).
4. **Two-car reference run, no network**: chosen by any of `--input2`, `--net-delay`, `--net-hash-every`; usage `usage: crash-sim --body <car.bsn> --track <file> --grid-gap <m> --duration <s> --input <file> --input2 <file> --net-delay <ticks> --net-hash-every <ticks>`; doc [Stage 7 milestone 4](docs/STAGE7.md).
5. **AI race**: chosen by `--ai <n>`; usage `usage: crash-sim --body <car.bsn> --track <file> --ai <n> --laps <N> --time-limit <s> --grid-gap <m> --ai-params <file> [--save-inputs <prefix>] [--substeps <n>]`; doc [Stage 7 milestone 2](docs/STAGE7.md).
6. **Single run**: chosen by everything else; usage: the cube by default, `--body <file.bsn>` ([format](docs/FORMAT.md)), `--input <file>` (a driver script), `--live`, `--track <file>` (a timed lap, [Stage 7 milestone 1](docs/STAGE7.md)), and report flags `--summary`, `--events`, `--settle-report`, `--crumple-report`, `--slope-report`, `--drive-report`, `--pulse-report` ([Stage 6](docs/STAGE6.md), [Stage 6 milestones 4 and 5](docs/STAGE6-M4-M5.md)).

A tune command is added by task t-20261125-163520 (`docs/TUNING.md`).

## Docs and examples

- [docs/BENCH.md](docs/BENCH.md): Timing harness (`examples/bench.rs`).
- [docs/DRIVE_FEEL.md](docs/DRIVE_FEEL.md): Why the car drives "like ice" (measure only).
- [docs/FORMAT.md](docs/FORMAT.md): The `.bsn` format, versions 1, 2 and 3.
- [docs/M5-SOLVER.md](docs/M5-SOLVER.md): m5 design note: which higher-fidelity crash-zone solver.
- [docs/NET.md](docs/NET.md): Lockstep net messages (n20-25a).
- [docs/PERF_HOTSPOTS.md](docs/PERF_HOTSPOTS.md): Performance hot spots: the 3.6 run.
- [docs/PHYSICS.md](docs/PHYSICS.md): Crash sim physics.
- [docs/SHELL.md](docs/SHELL.md): The shell protocol (n20-26a).
- [docs/STAGE6-M4-M5.md](docs/STAGE6-M4-M5.md): Stage 6 done checks: milestones 4 and 5.
- [docs/STAGE6.md](docs/STAGE6.md): Stage 6 done checks: milestones 1 to 3.
- [docs/STAGE7.md](docs/STAGE7.md): Stage 7 done checks: milestones 1 to 3; milestones 4 and 5 point to their own tasks.
- [docs/TUNING.md](docs/TUNING.md): Runtime tuning (no rebuild).
- [docs/deform-reference.md](docs/deform-reference.md): Node 3.2: original synthetic deform references (S3).
- [docs/stage7-maths-pin.txt](docs/stage7-maths-pin.txt): Source expression pins, beginning with the AI heading-angle calculation.
- [examples/bench.rs](examples/bench.rs): Timing harness for the sim's hot paths; std only.
- [examples/crash_energy_a.rs](examples/crash_energy_a.rs): Test A only; no reference trace, body override, or model edits.
- [examples/drive_feel.rs](examples/drive_feel.rs): Drive-feel diagnosis (measure only).
- [examples/energy.rs](examples/energy.rs): Default simulation energy CSV: kinetic, potential, elastic and total.
- [examples/profile_step.rs](examples/profile_step.rs): Per-phase timing of the 3.6 run.
- [examples/sweep.rs](examples/sweep.rs): Drop-height sweep with default Config; CSV table on stdout.
- [examples/trajectory.rs](examples/trajectory.rs): Writes a trajectory for the renderer to stdout.
- [examples/wall_force_a.rs](examples/wall_force_a.rs): Report-only normal barrier force for the fixed wall scenario.

Flags: `--duration <s>` (5), `--dt <s>` output step (0.01), `--substeps <n>`
physics substeps per output step (10), `--drop-height <m>` height of the lowest
corner (2), `--gravity <m/s^2>` gravitational acceleration, for example 1.62 for the Moon (9.81), `--no-tilt` (default start has a 5 degree tilt about x and z),
`--summary` print one line instead of the CSV:
`first_contact_s=<s>,peak_strain=<x>,final_center_y=<m>` (time of the
first physics substep at which any node is at or below the ground, 4 decimals,
or `none`; largest `max_beam_strain`; last `center_y`). It combines
with the other flags; errors still print nothing on stdout and exit non-zero.

## Output columns

`t,min_y,center_y,max_beam_strain`, one row per output step.

- `t`: time, seconds
- `min_y`: height of the lowest node, metres
- `center_y`: centre-of-mass height, metres
- `max_beam_strain`: largest |length - rest| / rest over all beams (unitless)

The physics constants (stiffness, damping, restitution, friction) are public items of the crate root, for example `crash_sim::BEAM_STIFFNESS`, with the reasoning in each doc comment.

## Behaviour notes

- The run aborts with a message on stderr and exit code 1 if a node goes
  non-finite or max strain passes 50%. Rows already computed are still printed.
- A physics step `h = dt / substeps` is refused up front in two cases: above the
  stiffness-only bound (about 3.8 ms, `max_stable_step()`; exactly equal is accepted,
  the test is `>`), or at or above the damped stability limit of the stiffest mode
  (`stable_step_limit()`, about 4.1 ms with the current constants; exactly equal is
  refused, the test is `>=`). With the current constants the first trips first. See
  `docs/PHYSICS.md`.
- The step count is `round(duration / dt)`, so with a `--dt` that does not divide
  the duration the last row can end up to dt/2 away from it (`--dt 0.006` ends at 4.998).
- Ground friction is a linear drag on horizontal speed (starter simplification,
  not Coulomb friction).
