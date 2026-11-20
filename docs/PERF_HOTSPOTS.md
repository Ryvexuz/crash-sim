# Performance hot spots: the 3.6 run (task t-20261120-192402)

Measure only. Nothing in `src/` or `tests/` changed and no simulated number moved. Every number below is from the run
named next to it, or quoted from a file named next to it.

## What was measured

The 3.6 run: `scenarios/pad_and_wall.bsn` with `scenarios/pad_lap.input`, 72 s, default `Config` (dt 0.01 s, 10 substeps),
so 72000 physics steps of h = 0.001 s. "Step" in this document is one such substep: the unit `input::drive` loops over.
The car is 40 nodes plus wheels; no beam has a break strain and gravity is not tilted (the example refuses to run
otherwise).

`examples/profile_step.rs` times each phase of the step with `std::time::Instant`. It repeats `Sim::integrate`'s car path
from public pieces (`Car::pre_step`, `SymplecticEuler::step_with_forces`, `Car::post_step`, `collide::apply`) so that each
can be timed. Every run also does the same 72000 steps with the real `Sim::advance`, and the program fails unless the
final node positions and velocities of the two runs are identical bit for bit; all 5 runs printed "final state
identical", so the split is the real step.

## Command lines

From `C:\Users\MrComputer\Desktop\Broadsword`, in the foreground:

```
build\bin\broadsword.exe run crash-sim --task t-20261120-192402 -- cargo run --offline --release --example profile_step
build\bin\broadsword.exe run crash-sim --task t-20261120-192402 -- cargo run --offline --release --example bench -- --runs 10 --case settle
```

## Load at the time

`build\bin\broadsword.exe status` before the profile run printed `cpu:  97% over 1s (warn at 90%) [WARN]` and after it
`cpu:  96% over 1s (warn at 90%) [WARN]` (`scratch/pf-status-before.txt`, `scratch/pf-status-after.txt`); after the bench run
it printed `cpu:  98% over 1s (warn at 90%) [WARN]` (`scratch/pf-status-b.txt`). Other Broadsword jobs were running (`job ... still running` lines in the run output). The machine was heavily loaded, so compare the shares,
not the absolute times.

## Raw table (median of 5 runs per phase)

Output of the profile run (`scratch/pf-run.txt`):

```
profile_step: scenarios/pad_and_wall.bsn + scenarios/pad_lap.input, 72 s = 72000 steps of h = 0.001 s, 5 runs
run 1: real advance loop 333.7 ms, timed loop 356.6 ms (final state identical)
run 2: real advance loop 340.4 ms, timed loop 367.9 ms (final state identical)
run 3: real advance loop 348.0 ms, timed loop 371.3 ms (final state identical)
run 4: real advance loop 333.0 ms, timed loop 357.8 ms (final state identical)
run 5: real advance loop 339.8 ms, timed loop 359.7 ms (final state identical)

median of 5 runs per phase
phase                                              total ms  ms per step   share
script.apply (controls for this substep)               3.08     0.000043    0.9%
extra-force setup (alloc + gravity part)               6.18     0.000086    1.7%
Car::pre_step (wheels, suspension, tyres)             64.95     0.000902   18.0%
solver step (beam forces + node integration)         116.06     0.001612   32.2%
Car::post_step (wheel update, axis constraint)        21.42     0.000298    5.9%
collide::apply (ground and wall contacts)             11.20     0.000156    3.1%
Sim::apply_yield (plastic flow)                       63.20     0.000878   17.5%
first-contact test + Sim::check                       74.86     0.001040   20.7%
sum of phases (step time)                            360.94     0.005013  100.0%

timed loop wall time (median) 359.71 ms = 0.004996 ms per step (includes the timer calls)
real Sim::advance loop (median) 339.81 ms = 0.004720 ms per step
```

The timer calls cost about 6% (timed loop 359.71 ms against 339.81 ms for the real loop, about 20 ms over 72000 steps
and 8 phases), so the smallest phases (script.apply, extra-force setup, collide::apply) carry the largest relative error. The sum of the phase medians (360.94 ms) is not the median of the timed loop (359.71 ms) because a median of
sums is not a sum of medians.

## Check against the bench

Done check 1 asks for the table's total per step to be within 25% of the `bench --runs 10` settle median. The bench
printed (`scratch/pf-bench.txt`):

```
{"name":"settle","runs":10,"median_ms":5.922,"min_ms":5.808,"max_ms":6.039,"mad_ms":0.083}
```

The two numbers are 0.005013 ms per step (this table, whole run 360.94 ms) and 5.922 ms (the whole settle case). They are
not within 25% of each other by any reading: the settle case is a different workload (the 8-node cube with no wheels,
`duration` 10 s at dt 0.01 and 10 substeps = 10000 substeps, 0.000592 ms per substep when divided out), and the 3.6 step is 8.5 times that per substep
(0.005013 / 0.000592). What this does show is that the 3.6 run's 72000 substeps cost 361 ms in this profile and 340 ms
in the real loop, which is consistent with the table; the bench cannot validate the 3.6 numbers, only the real-loop run in
the same program can (identical final state, 340 against 361 ms, about 6% apart).

## Top 5 hot spots by measured share

The fixes and gains below are plans and estimates, not measurements. Only the shares are measured.

1. **Solver step, 32.2%** (beam forces and node integration together; they cannot be split from outside, see below).
   Plan: keep the per-node force `Vec` in the solver between calls instead of allocating it each substep, and skip the
   `Vec3` temporaries in the beam loop. Estimate: 10% to 20% of this phase, so 3% to 6% of the step.
2. **First-contact test + `Sim::check`, 20.7%.** `check` runs every substep in `input::drive` and walks every beam for the
   strain (`max_strain_unbroken`, one more `sqrt` per beam) and every node for finiteness; the solver already computed
   each beam's length. Plan: judge the strain from the squared length without the `sqrt` unless it is near the limit, and
   run the finiteness test on the output step only if its refusal time can stay the same. Estimate: 30% to 50% of this
   phase if the `sqrt` goes, so 6% to 10% of the step.
3. **`Car::pre_step`, 18.0%** (wheels, suspension, tyres). It allocates the `brake`, `drive` and `axles` `Vec`s every
   substep. Plan: hold them in the `Car` and refill. Estimate: 5% to 15% of this phase, so 1% to 3% of the step; the tyre
   maths itself is not measured and may be the larger part.
4. **`Sim::apply_yield`, 17.5%.** A `sqrt` per beam with a yield strain each substep to find the strain, also for a beam that
   does not exceed it (how many beams yield was not measured). Plan: compare the squared length with the squared yield band and take the `sqrt` only for a beam that may
   yield. Estimate: 50% to 70% of this phase, so 9% to 12% of the step.
5. **`Car::post_step`, 5.9%** (wheel update, axis constraint, patch deflections). Plan: nothing until the instrumentation
   below shows which part of it is slow. Estimate: unknown.

Contacts (`collide::apply`, 3.1%), `script.apply` (0.9%) and the extra-force setup (1.7%) are below the top 5.

## What was not measured

- Inside the solver step: beam forces against node integration. They are one private function (`symplectic_euler`); a
  split needs instrumentation inside `src/solver.rs` (follow-up).
- Inside `Car::pre_step`: the wheel loop against `tyre_force` against `add_torque`; inside `Car::post_step`: the axis
  constraint against the patch update (needs instrumentation in `src/vehicle.rs`, follow-up).
- `Sim::apply_breaks` (private). No beam in this scenario has a break strain, and the example fails if one had.
- `input::drive`'s per-step bookkeeping (centroid, heading, `wall_touched`, rows, the trajectory frames of `--trajectory-v2`)
  and the program's startup, parsing and output. The profile covers `script.apply`, the step and `check` only.
- Memory use, allocation counts, cache misses and anything other than wall time on one thread.
- The viewer side was not re-measured (below).
- A quiet machine: all numbers are from a machine at 96% to 98% CPU.

## The viewer (not re-measured; quoted)

Quoted from the render task t-20261120-183043 (render commit `edd31ff`, "render: drive viewer sends a line every tick, paces
sim time to the wall clock and draws only new frames"), from the files in `C:\Users\MrComputer\Desktop\Broadsword\scratch`:

Before the fix (`scratch/t183043-A-window.txt`, scripted 20 s run, CPU 100% per `scratch/briefs/t-20261120-183043.md`):

```
timing: per drawn frame draw 0.27 ms, present 0.14 ms, wait 1.36 ms; parse 0.022 ms per child frame (1841 frames, lock wait 38.1 ms total)
timing: child round trip over 140 lines: median 0.37 ms, p90 1.31 ms, max 28.09 ms
timing: sim starved for input at 9698 of 11336 draws; drawn frame behind the wall clock by 842.1 ms on average; 11336 loops in 20.00 s
summary: frames drawn 11336 (214 showing a new sim frame, 10.7 per s), sim seconds 20.00, wall seconds 20.00, average fps 566.7
```

After the fix (`scratch/t183043-C-window.txt`, same run, CPU 99% per the brief file):

```
timing: per drawn frame draw 0.24 ms, present 0.15 ms, wait 10.92 ms; parse 0.031 ms per child frame (1999 frames, 235 not drawn, lock wait 3.3 ms total)
timing: child round trip over 1000 lines: median 0.29 ms, p90 0.35 ms, max 12.53 ms
timing: sim starved for input at 0 of 1765 draws; drawn frame behind the wall clock by 4.3 ms on average; 4012 loops in 20.01 s
summary: frames drawn 1765, sim seconds 20.00, wall seconds 20.01, average fps 88.2
```

The "average fps 566.7" before the fix counts redraws of the same frame; the line that matters is "214 showing a new
sim frame, 10.7 per s". The per-frame draw (0.24 ms) and present (0.15 ms) are small beside the 10.92 ms "wait"; what the wait is
was not investigated here.
