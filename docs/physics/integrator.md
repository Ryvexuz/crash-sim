## Integrator: symplectic Euler with substeps

One output step of `dt` (default 0.01 s) is split into `substeps` (default 10) physics substeps of `h = dt / substeps` (default 1 ms). In each substep:

1. Compute all forces (gravity and beams) from the current state.
2. `vel += force * (h / NODE_MASS)`.
3. `pos += vel * h`, using the new velocity (this ordering is what "symplectic" or semi-implicit means; it keeps oscillation energy from growing the way plain Euler does).
4. Apply plane contact (the ground and any walls) as above; `Sim` does this after the solver's step.

### Why the step limit is what it is

A spring with angular frequency `omega` = sqrt(k/m) is stable under symplectic Euler only if `omega * h < 2`. The stiffest vibration of the cube bounds this. With 7 beams per node, Gershgorin's circle theorem gives a largest eigenvalue of the stiffness matrix of at most `2 * 7 * k / m`, so

    omega_max <= sqrt(14 * BEAM_STIFFNESS / NODE_MASS) = sqrt(280000) = about 529 rad/s
    max_stable_step() = 2 / omega_max = about 0.00378 s

`Config::validate()` refuses to run when `dt / substeps` is larger than `max_stable_step()`, and tells you to raise `--substeps` or lower `--dt`. It also rejects non-finite values, `dt <= 0`, `duration < 0`, `substeps < 1`, `drop_height < 0`, and gravity that is negative, NaN or infinite. Gravity must be finite and `>= 0`, enforced by `self.gravity.is_finite() && self.gravity >= 0.0` (`src/lib.rs` line 191): 0 is allowed for the headless pulse runs (the m4 frozen run flags start with `--gravity 0`). The default 1 ms substep is about a quarter of the limit.

### The damping limit

Damping lowers the stable step, and `validate()` enforces the combined condition itself through `damped_step_error(h, k, c, m)`, which uses `stable_step_limit(k, c, m)`. Derivation, for the code's own update (`vel` first, then `pos` with the new `vel`):

1. In one vibration mode with spring rate `w2` (omega squared) and damping rate `g`, a step is `v' = (1 - h*g) v - h*w2 x`, `x' = x + h v'`. Its determinant is `1 - h*g` and its trace is `2 - h*g - h^2*w2`.
2. The Jury stability test gives two conditions: (A) `0 < h*g < 2` and (B) `h^2*w2 + 2*h*g < 4`.
3. Every beam's damper acts along the same direction as its spring, so the damping matrix is the stiffness matrix times `c/k`. The stiffest mode has eigenvalue 8 (`omega^2 = 8k/m`, 400 rad/s; checked numerically by `stiffest_mode_eigenvalue_is_eight`), so there `w2 = 8k/m` and `g = 8c/m`.
4. The largest `h` meeting (B) is the positive root `h = (-g + sqrt(g^2 + 4*w2)) / w2`. At `h = 2/g`, (B) would read `4*w2/g^2 + 4 < 4`, which is false when `w2 > 0`, so that root is always below `2/g`: (B) binds whenever there is a spring, and (A) only if `w2 = 0`. `stable_step_limit` returns the smaller of the two and says which binds; `validate()` refuses `h >= limit` with a message naming h, the limit and the binding condition.
5. Numbers for `k = 20000`, `m = 1`: `160000*h^2 + 16*c*h = 4`. The limit is 0.00500 s at `c = 0` (the undamped `2/omega`), 0.00410 s at `c = 20` (today's `BEAM_DAMPING`), 0.00339 s at `c = 40`, and falls with c.
6. The old separate test `c*h/m < 0.25` is only condition (A) for the damper alone. It is not enough: it is much looser than (B). At `c = 40` the old checks accepted `h = 0.0036 s`, but the limit is 0.003385 s and the stiffest mode diverges (`old_separate_checks_accepted_an_unstable_step_at_c_40`). An earlier version of this document wrongly said damping would only matter past `c` of about 66; it matters from about 28.

`max_stable_step()` (0.00378 s) is kept, and `validate()` checks it too: it is a stiffness-only Gershgorin bound that holds for any deformed shape, while the stiffest-mode limit uses the rest-shape eigenvalue 8. With today's constants the coupled limit at `c = 20` is 0.0041 s, above 0.00378 s, so the Gershgorin bound trips first and the coupled error is not reachable through `validate()` today. The two cross at `c` of about 28.3: above that, the coupled limit is the tighter one. `damped_step_error` takes `k`, `c` and `m` as arguments so it is tested for other constants (`tests/stability.rs`), and the limit is checked on a single damped oscillator stepped with the same integrator: bounded just below it, unbounded just above, for `c` = 0, 20, 40, 66 and 200.

## CSV output

`run` returns one sample at t = 0 and one after each output step. The step count is `step_count` = `floor(duration / dt)` with a relative epsilon of 1e-9 (so 5.0 / 0.01 stays 500), so the last row never lies after `duration` and can lie up to `dt` before it. A `dt` below `MIN_DT` = 0.0001 s is refused, because the time column has 4 decimals and its rows would repeat. `to_csv` writes the header `t,min_y,center_y,max_beam_strain` (`CSV_HEADER`) and one row per sample:

| Column | Meaning | Decimals |
|---|---|---|
| `t` | time in seconds (step number * `dt`) | 4 |
| `min_y` | height of the lowest node, metres | 6 |
| `center_y` | mean y of all 8 nodes, metres | 6 |
| `max_beam_strain` | largest beam strain over all 28 beams (a body's broken beams are skipped) | 6 |

Strain of a beam is `|len - rest| / rest`, so 0.01 means 1% off its rest length.

## Safety checks

After every output step (not every substep), `Sim::check()` fails the run if:

- the solver reports an error of its own (`Solver::check`, checked first; for the m5 solver an inverted or collapsed tet or an exceeded stiffness bound, "Solids" above; the default solver never reports one);
- any node position or velocity component is NaN or infinite;
- the max strain (over unbroken beams) is NaN, or exceeds `MAX_STRAIN_LIMIT` = 0.5 (50%). The strain calculation propagates NaN, so a blown-up state can never read as 0.

Contact time is tracked separately, in the substep loop: `Sim::first_contact_s` is the time of the first substep at which any node is at or behind any collider plane, ground or wall (0 if the body starts there). `--summary` prints it, so it does not depend on the output sampling, which can miss a touchdown that begins and ends between two samples.

`run_checked` then stops and returns the good samples so far plus an error reading `unstable at t = ... s: ...`. A config that `validate()` accepts can still abort here, for example `--gravity 5000` crushes the cube on impact (`tests/risky.rs`). A config refused by `validate()` returns no samples and the validation error. `run` is the same but panics on any error.

