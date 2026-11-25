# Timing harness (examples/bench.rs)

Std-only example that times the sim's hot paths. Run it in release mode, from the crate root:

```
broadsword run crash-sim -- cargo run --release --offline --example bench -- --runs 5
```

Options: `--runs <n>` (integer >= 1, default 10) and `--case box|cube|settle|car|break|m5` (run one case only). The four argument refusals print `bench: <message>` and the usage line to stderr, then exit 2:

- Missing, invalid or zero `--runs`: `--runs needs an integer >= 1`.
- Unknown case: `unknown case '<c>' (cases: box, cube, settle, car, break, m5)`.
- Missing case name: `--case needs a name (box, cube, settle, car, break, m5)`.
- Unknown argument: `unknown argument '<a>'`.

## Output

One JSON line per selected case, in the order `box, cube, settle, car, break, m5`:

```
{"name":"box","runs":5,"median_ms":..,"min_ms":..,"max_ms":..,"mad_ms":..}
```

`mad_ms` is the median absolute deviation from the median. Each timed run builds a fresh sim or solver; scenario loading and the `m5` block construction are outside the timed closure.

After its JSON line, `break` prints `break: beams broken = <n>` and `m5` prints `m5: plastic tets = <n>`, each counting the final timed run. A count of 0 exits 2 with `bench: break: no beam broke (the case must break beams)` or `bench: m5: no tet yielded (the case must yield tets)`.

The cases use fixed inputs and step counts, not randomness. A run that blows up or is refused makes the bench exit 2 instead of reporting a time.

## Cases

- `box`: loads `scenarios/box.bsn` (12 nodes) with `Sim::from_body` and calls `Sim::advance` `STEPS = 500` times (5 s of sim time, 5000 solver substeps, each followed by the yield pass), using `Config::default()`.
- `cube`: the same for `scenarios/cube.bsn` (the 8-node cube; 28 beams), including `STEPS = 500`.
- `settle`: `settle::measure` on the default built-in cube with `duration` 10 and `SettleParams` by 6, hold 4, speed 0.001, energy_rise 0.001, energy_gain 0.00001. These are the values of the documented `--settle-report` run in docs/STAGE6.md; main.rs itself has no defaults for them. It measures the cost of the per-substep settle bookkeeping (energy, impact and speed checks) on top of the solver.
- `car`: drives `scenarios/pad_and_wall.bsn` with `scenarios/pad_lap.input` for the first `CAR_SECONDS = 5.0` s, using `input::drive` on a `CarSetup`, as `crash-sim --input` does.
- `break`: loads `scenarios/pad_and_wall.bsn`, sets every material's `break_strain` to `BREAK_STRAIN = 0.025`, and moves its one plane to x = `BREAK_WALL_X = 5.0` m. It runs for `BREAK_SECONDS = 4.0` s with `BREAK_SCRIPT` (`0 1 0 0`: full throttle from t = 0, no brake, no steer). `settle::car_sim` builds the sim with one substep per step; the script is applied before each substep.
- `m5`: builds a block in code with `M5_CELLS = [5, 3, 3]` cells of `M5_CELL_M = 0.1` m, each cut into 6 tets by `solver_m5::cut_cell` (270 tets, under `M5_MAX_TETS = 300`). Nodes have `M5_NODE_KG = 0.5` kg and start 1 cm above the floor, thrown down at `M5_IMPACT_MPS = 20.0` m/s. It steps `SolverM5` directly for `M5_STEPS = 500` substeps of `M5_H = 1e-5` s (`Sim` does not expose its solver), applying `collide::apply` against the floor after each step. The solid `foam` has plateau stress `M5_PLATEAU_PA = 2e6` Pa, yield strain `M5_YIELD_STRAIN = 0.02`, and damping ratio 0.05.

`box` and `cube` read relative paths, so run from the crate root. `car` and `break` resolve their scenario paths using `CARGO_MANIFEST_DIR`; `m5` needs no scenario file.

Broadsword's `perf_gate` setting runs this example. Baseline values are not recorded here.
