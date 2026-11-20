# Timing harness (examples/bench.rs)

Std-only example that times the sim's hot paths. Run it in release mode, from the crate root (it reads `scenarios/*.bsn` by relative path):

```
broadsword run crash-sim -- cargo run --release --offline --example bench -- --runs 5
```

Options: `--runs <n>` (integer >= 1, default 10) and `--case box|cube|settle` (run one case only). `--runs 0`, a non-numeric `--runs`, an unknown case or an unknown argument prints a message to stderr and exits 2.

Output: one JSON line per case, in the order box, cube, settle:

```
{"name":"box","runs":5,"median_ms":..,"min_ms":..,"max_ms":..,"mad_ms":..}
```

`mad_ms` is the median absolute deviation from the median. Each run builds the sim fresh, so the timings include building it.

There is no randomness: the config is `Config::default()` (dt 0.01 s, 10 substeps, gravity 9.81) and the step count is fixed, so only the time differs between runs. A run that blows up or is refused makes the bench exit 2 instead of reporting a time.

## Cases

- `box`: loads `scenarios/box.bsn` (12 nodes) with `Sim::from_body` and calls `Sim::advance` 500 times (5 s of sim time, 5000 solver substeps, each followed by the yield pass).
- `cube`: the same for `scenarios/cube.bsn` (the 8-node cube; 28 beams).
- `settle`: `settle::measure` on the default built-in cube with `duration` 10 and `SettleParams` by 6, hold 4, speed 0.001, energy_rise 0.001, energy_gain 0.00001. These are the values of the documented `--settle-report` run in docs/STAGE6.md; main.rs itself has no defaults for them. It measures the cost of the per-substep settle bookkeeping (energy, impact and speed checks) on top of the solver.

## Later

`box_wall` and car cases get added once plane colliders are wired into `Sim` (a body with planes is refused today) and the car scenario exists.
