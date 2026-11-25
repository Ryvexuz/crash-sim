# Runtime tuning (no rebuild)

`crash-sim tune` rewrites numeric tokens in a `.bsn` body into variant files,
then runs the **same executable** (`current_exe`) for every valid point.
It does not change physics code, compile-time constants, or the source body.

Usage:

```text
crash-sim tune --body <file.bsn> --out <fresh-dir>
  [--tune <file>]... [--set <keyword>.<name>.<field>=<value>]...
  [--jobs <n>] [--show <prefix>]... -- <run flags...>
```

For example, put this in `box.tune`:

```text
tune 1
set material box_beam damping 20
sweep material box_beam stiffness 10000 40000 3 lin
```

Then use these arguments with your existing binary:

```text
tune --body scenarios/box_wall.bsn --out tune-box --tune box.tune -- --summary
```

For a four-wheel body such as `validation_wall.bsn`, plain `--summary` is
refused. Use driving input and explicit integration settings instead:

```text
tune --body scenarios/validation_wall.bsn --out tune-wall --tune scenarios/presets/realistic.tune -- --input scenarios/pad_lap.input --dt 0.001 --substeps 10 --duration 0.2
```

Change the tune file and rerun into a fresh output directory; no rebuild is
needed. Jobs default to 4 and must be integers from 1 through 16.
Children receive `--body point-file` followed by the flags after `--`;
those flags may not supply another `--body`. Child stdin is closed.

## Tune format

Each file has a `tune 1` header before its instructions. Blank lines and lines
whose first non-whitespace character is `#` are ignored, including before the
header. Tokens are whitespace-separated; inline comments are not tune syntax.

```text
set <keyword> <name> <field> <value>
scale <keyword> <name> <field> <positive-factor>
sweep <keyword> <name> <field> <from> <to> <steps> [lin|log]
include <relative-file>
```

Sets replace a numeric field token, scales multiply it. Matching uses the first
two body tokens and an exact field token, never a substring. Name `*` targets
every line with that keyword that has the field. Missing names, missing fields,
and nonnumeric existing values are errors, not silent no-ops.
Untouched bytes and line endings are preserved, including CRLF in the library
rewriter. The existing body parser still rejects CRLF at CLI validation.
Changed numbers use Rust's shortest round-trip f64 display.

All numbers must be finite; scale factors must be positive. Nonzero number
literals that underflow to zero, and scale overflow/underflow to zero, are errors.
There are no additional realism limits.
Sweeps accept 2 through 200 steps, default to linear, and require positive
endpoints for logarithmic spacing. Descending sweeps are supported.
The endpoints are assigned exactly; interior values use the specified linear
or logarithmic interpolation, with weighted forms when a difference or ratio
would overflow. Results stay within the finite endpoint interval.
There may be at most two sweeps and at most 400 total points.
Two sweeps form a grid with the first sweep outermost.
Without sweeps there is one run.

Includes apply at their instruction's location and each included file has its
own header. Paths resolve relative to the including file, accepting `/` or `\`;
absolute paths and drive-prefixed includes are rejected.
Four include levels below the root are allowed; cycles are errors.
Whitespace in an include filename cannot be represented by this token format.
Top-level `--tune` paths may contain spaces.

All `--tune` files apply in their command-line order, then all `--set` overrides
in their command-line order. Sets and scales execute in that sequence;
sweep values are applied last. Errors carry the tune filename and 1-based line.

## Results and failures

Each point writes `point-001.bsn`, `point-001.txt`, etc. into `--out`.
The text log combines child stdout and stderr; selection uses **stdout only**.
The tab-separated table goes to stdout and `results.tsv`, in point order:
`point`, swept-key columns, `exit`, `wall_s` (two decimals), `stdout`.
Each `--show` selects stdout lines starting with that prefix; matching lines
appear once each in output order, joined by ` | `. Without `--show`, the last
stdout line is selected. Tabs, line breaks, and backslashes are escaped in cells.

Tune parsing and rewriting finish for all points before any child starts.
A body-parser failure makes that point `invalid`, records its values and error
in its log/table row, and skips its child. Other valid points still run.
Exit 0 means every point ran, regardless of child exit codes; exit 1 means at
least one point was invalid. Usage/tune errors return 2 without running points.
I/O or runner failures also return 2 and report the first encountered error;
partial artifacts are retained for diagnosis.
Existing output files are never overwritten. Use a fresh directory for reruns.

## Presets

`scenarios/presets/realistic.tune` records the validated test A material values
from `scenarios/validation_wall.bsn`; applying it to that body is an identity.
`exaggerated.tune` and `racing-light.tune` include it, then scale yield strain
and stiffness. They are **examples, NOT validated**: softer/more crumple versus
less damage, respectively. They target the validation body's material names,
not arbitrary bodies such as `box_wall.bsn`.

## Verification

The orchestrator runs the full suite with
`broadsword test crash-sim --task t-20261125-163520`.
Verify the denominator, missing-name, and substring-match mutations individually
against the named regression tests in `tests/tune.rs`, restoring after each one.
