"""The m4 calibration sweep (docs/STAGE6-M4-M5.md, "The calibration procedure (n20-10)").

Runs the fixed grid of the two-stage crush zone on test A only, with the 4.4 command, only `--body`
changed, and logs each point to validation/CALIBRATION.md. A grid point is the front stage's
(`crush_front`) P1 to P3; the rear stage (`crush_rear`) is never swept: its P1 is
`frozen: crush.rear_to_front_strength` times the front's, with the front's P2 and P3. A point whose
rear would leave the frozen ranges is logged `refused` without a run.

    python -I -B validation/sweep.py --run --task <task id> --from <i> --to <j>
        Runs grid points i..j (1-based, inclusive) through `broadsword run crash-sim --task <id>`.
        Each point's body is a variant of scenarios/validation_wall.bsn that differs only in the
        two crush material lines, written to <Broadsword root>/<scratch_dir>/m4-sweep/<task id>/, with the run's
        whole output next to it. Lines are appended to validation/CALIBRATION.md (created with its
        `calibration 1` line by the first chunk); i must be the next index not yet logged, so the
        grid can be run in chunks but never twice or out of order.
    python -I -B validation/sweep.py --choose
        Appends the `calibrated node-beam` line once all grid points are logged.
    python -I -B validation/sweep.py --apply
        Writes the calibrated point into the two crush lines of test A's car and wall files, each
        stage computed from the file's own L_c and m_c, the rear by the frozen ratio. The other
        car's copies are not this script's: they are edited by their own task.
    python -I -B validation/sweep.py --record-deviation --score-from <4.4 output> --note <reason>
        Instead of --choose, a recorded deviation: appends the grid's own choice (`grid_best`),
        `deviation: <reason>` and `calibrated node-beam deviation ...` with the values computed
        back from test A's car file and the score of a saved 4.4 run on its wall file.
    python -I -B validation/sweep.py --verify
        Check 4.3: the log is the grid in order, inside the ranges, the calibrated line follows the
        rule, and a `freeze =` line follows it. Last line `calibration: ok` or `calibration: FAIL ...`.

    --solver m5 (n20-11c, option (1)) with --run, --choose, --apply and --verify does the same for the
        implicit solver: variants of scenarios/validation_wall_m5.bsn under <scratch_dir>/m5-sweep/<task id>/,
        the command `--body <variant> --solver m5 --timing --pulse-report ...` with the run flags'
        ` --substeps 10 ` replaced by N5; its block (`calibration m5 1`, `run A m5 <i> ...`,
        `calibrated m5 <i> ...`, then `freeze_m5 = <40 hex>`) follows m4's `run B node-beam` line.
        --apply writes the calibrated point into validation_wall_m5.bsn only. There is no m5 deviation.
    python -I -B validation/sweep.py --solver m5 --probe <i> --task <task id>
        Runs grid point i's m5 variant once with --timing (output in <scratch_dir>/m5-sweep/<task id>/
        probe-<i>.txt, rtf printed); logs nothing.

Ranges and grid (m5: the `frozen:` lines of docs/M5-SOLVER.md, N5 = `frozen: m5_substeps` too), E,
the strength ratio and the run flags are read from the docs' `frozen:` lines, never
copied here. The script refuses to start inside a Broadsword job (nested runs deadlock the build
lock), and finds the Broadsword root by walking up from its own path to the folder holding
broadsword.toml. It runs test A only; it opens no other test's files.
"""

import math
import os
import re
import subprocess
import sys
import tomllib
import types
from decimal import Decimal

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
DOC = os.path.join(REPO, "docs", "STAGE6-M4-M5.md")
LOG = os.path.join(REPO, "validation", "CALIBRATION.md")
WALL_A = "scenarios/validation_wall.bsn"
# The files that get the calibrated values, as (car, wall) pairs: test A's only.
FILE_PAIRS = (
    ("scenarios/validation_car.bsn", "scenarios/validation_wall.bsn"),
)
PULSE_A = "validation/test-a.pulse.csv"
PROJECT = "crash-sim"
SWEEP_DIR = "m4-sweep"
PARAMS = ("area_m2", "yield_strain", "damping_ratio")

# The m5 option (1) calibration (docs/STAGE6-M4-M5.md n20-11c): the same grid on test A with the
# implicit solver. Its ranges, grid and N5 are the `frozen:` lines of docs/M5-SOLVER.md; its wall
# file is the `_m5` copy; its block follows m4's `run B node-beam` line in the log.
M5 = "m5"
M5_DOC = os.path.join(REPO, "docs", "M5-SOLVER.md")
WALL_A_M5 = "scenarios/validation_wall_m5.bsn"
SWEEP_DIR_M5 = "m5-sweep"
M5_HEADER = "calibration m5 1"
M4_SUBSTEPS_TEXT = " --substeps 10 "  # the frozen run_flags' step, the one thing m5 replaces (by N5)
LABEL = {None: "node-beam", M5: "m5"}


def _run_re(label):
    return re.compile(
        rf"^run A {re.escape(label)} (\d+) area_m2=(\S+) yield_strain=(\S+) damping_ratio=(\S+) "
        r"score=(\d+\.\d{4}|none) verdict=(ok|FAIL|refused)$"
    )


RUN_RE = _run_re("node-beam")
CAL_RE = re.compile(
    r"^calibrated node-beam (\d+|deviation) area_m2=(\S+) yield_strain=(\S+) damping_ratio=(\S+) "
    r"score=(\d+\.\d{4})( edge)?$"
)
CAL_RE_M5 = re.compile(
    r"^calibrated m5 (\d+) area_m2=(\S+) yield_strain=(\S+) damping_ratio=(\S+) "
    r"score=(\d+\.\d{4})( edge)?$"
)
# A recorded deviation (orchestrator decision of 2026-11-22, docs/STAGE6-M4-M5.md, "Calibration
# n20-10"): the calibrated point is not the grid's choice but the values the test A car file
# already carries. The log then holds, after the run lines and in this order, the grid's own
# choice as `grid_best node-beam ... verdict=<v>`, one `deviation: <reason>` line, and
# `calibrated node-beam deviation ...` whose values are computed back from the car file.
DEVIATION_PREFIX = "deviation: "
CAR_A = "scenarios/validation_car.bsn"


class SweepError(Exception):
    pass


# ---------------------------------------------------------------- numbers and the frozen lines

def fmt(x):
    """A float as Rust's `{}` prints it (shortest round-trip digits, never an exponent), so a
    number written here reads back as the same f64 and the file stays in canonical form."""
    if not math.isfinite(x):
        raise SweepError(f"not a finite number: {x}")
    s = format(Decimal(repr(float(x))), "f")
    if "." in s:
        s = s.rstrip("0").rstrip(".")
    if s in ("-0", ""):
        s = "0"
    return s


def frozen(doc_text, name):
    """The text after `frozen: <name> = ` (exactly one such line), stripped."""
    prefix = f"frozen: {name} = "
    found = [l[len(prefix):].strip() for l in doc_text.splitlines() if l.startswith(prefix)]
    if len(found) != 1:
        raise SweepError(f"docs/STAGE6-M4-M5.md has {len(found)} `{prefix}` lines, not 1")
    return found[0]


def frozen_numbers(doc_text, name):
    text = frozen(doc_text, name)
    try:
        return [float(t) for t in text.split()]
    except ValueError:
        raise SweepError(f"`frozen: {name} = {text}` is not all numbers") from None


def axis(doc_text, param):
    """The grid values of one parameter: `<n> log`, `<n> lin` over its range, or an explicit list."""
    lo, hi = ranges(doc_text)[param]
    spec = frozen(doc_text, f"grid crush.{param}").split()
    if len(spec) == 2 and spec[1] in ("log", "lin"):
        n = int(spec[0])
        if n < 2:
            raise SweepError(f"grid crush.{param}: {n} points")
        vals = []
        for i in range(n):
            if i == 0:
                v = lo
            elif i == n - 1:
                v = hi  # the end is the range end exactly, not a rounded power
            elif spec[1] == "log":
                v = math.exp(math.log(lo) + (math.log(hi) - math.log(lo)) * i / (n - 1))
            else:
                v = float(f"{lo + (hi - lo) * i / (n - 1):.12g}")  # no binary noise in the logged value
            vals.append(v)
        return vals
    try:
        vals = [float(t) for t in spec]
    except ValueError:
        raise SweepError(f"grid crush.{param}: {' '.join(spec)} is not a grid") from None
    for v in vals:
        if not lo <= v <= hi:
            raise SweepError(f"grid crush.{param}: {v} is outside [{lo}, {hi}]")
    return vals


def ranges(doc_text):
    out = {}
    for p in PARAMS:
        r = frozen_numbers(doc_text, f"range crush.{p}")
        if len(r) != 2 or not r[0] < r[1]:
            raise SweepError(f"range crush.{p}: {r}")
        out[p] = (r[0], r[1])
    return out


def grid(doc_text):
    """The grid points in index order (index 1 first): area_m2 outermost, damping_ratio innermost."""
    a1, a2, a3 = (axis(doc_text, p) for p in PARAMS)
    return [(x, y, z) for x in a1 for y in a2 for z in a3]


def run_flags(doc_text):
    return frozen(doc_text, "run_flags").split()


def is_edge(point, rng):
    return any(v == rng[p][0] or v == rng[p][1] for p, v in zip(PARAMS, point))


# ---------------------------------------------------------------- the body files

# The two crush stages (docs/STAGE6-M4-M5.md, "The two-stage crumple zone"): the grid point is the
# front stage's P1 to P3; the rear has P1 = ratio x the front's, the same P2 and P3.
STAGES = ("crush_front", "crush_rear")


def stage_points(point, ratio):
    """{stage: (P1, P2, P3)} of a grid point (the front stage's values)."""
    area, ys, zeta = point
    return {"crush_front": (area, ys, zeta), "crush_rear": (ratio * area, ys, zeta)}


def stages_inside(point, ratio, rng):
    """True iff both stages' P1 to P3 lie inside the frozen ranges."""
    return all(rng[p][0] <= v <= rng[p][1]
               for sp in stage_points(point, ratio).values() for p, v in zip(PARAMS, sp))


def strength_ratio(doc_text):
    r = frozen_numbers(doc_text, "crush.rear_to_front_strength")
    if len(r) != 1 or not r[0] > 0:
        raise SweepError(f"frozen: crush.rear_to_front_strength is not one positive number: {r}")
    return r[0]


def parse_crush(text, stage):
    """(index of the `material <stage>` line, L_c, m_c) of a .bsn text, for one crush stage.

    L_c is the mean rest length of the stage's beams, m_c the mean mass of the distinct nodes at
    their ends (docs/STAGE6-M4-M5.md, "What is calibrated"; tests/validation_files.rs)."""
    lines = text.split("\n")
    crush_idx = [i for i, l in enumerate(lines) if l.split()[:2] == ["material", stage]]
    if len(crush_idx) != 1:
        raise SweepError(f"{len(crush_idx)} `material {stage}` lines, not 1")
    nodes = {}
    rests = []
    ends = set()
    for l in lines:
        t = l.split()
        if not t:
            continue
        if t[0] == "node":
            if len(t) != 7 or t[5] != "mass":
                raise SweepError(f"unexpected node line: {l}")
            nodes[int(t[1])] = ((float(t[2]), float(t[3]), float(t[4])), float(t[6]))
        elif t[0] == "beam" and len(t) >= 4 and t[3] == stage:
            a, b = int(t[1]), int(t[2])
            if len(t) == 6 and t[4] == "rest":
                rest = float(t[5])
            elif len(t) == 4:
                pa, pb = nodes[a][0], nodes[b][0]
                d = [pb[k] - pa[k] for k in range(3)]
                rest = math.sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2])
            else:
                raise SweepError(f"unexpected beam line: {l}")
            rests.append(rest)
            ends.update((a, b))
    if not rests:
        raise SweepError(f"no `{stage}` beams")
    lc = sum(rests) / len(rests)
    mc = sum(nodes[i][1] for i in sorted(ends)) / len(ends)
    return crush_idx[0], lc, mc


NUDGE_ULPS = 16


def _into(value, back, lo, hi, what):
    """`value` moved by the fewest ulps (at most NUDGE_ULPS) so that `back(value)`, the quantity
    computed back from it as tests/validation_files.rs does, is inside [lo, hi]. A grid value on a
    range end can compute back one ulp outside it (k = E A / L_c, then A = k L_c / E); the change
    is far below every printed digit of the sim's report."""
    for _ in range(NUDGE_ULPS + 1):
        b = back(value)
        if lo <= b <= hi:
            return value
        value = math.nextafter(value, math.inf if b < lo else -math.inf)
    raise SweepError(f"{what} cannot be brought inside [{lo}, {hi}] within {NUDGE_ULPS} ulps")


def crush_line(stage, point, lc, mc, steel_e, rng):
    area, ys, zeta = point
    k = _into(steel_e * area / lc, lambda k: k * lc / steel_e, *rng["area_m2"], f"{stage} area_m2")
    c = 2.0 * zeta * math.sqrt(k * mc)
    if c != 0.0:
        c = _into(c, lambda c: c / (2.0 * math.sqrt(k * mc)), *rng["damping_ratio"],
                  f"{stage} damping_ratio")
    return f"material {stage} stiffness {fmt(k)} damping {fmt(c)} yield_strain {fmt(ys)}"


def with_crush(text, point, steel_e, ratio, rng):
    """The text with only its two crush stage lines replaced: the front stage by the grid point,
    the rear by the frozen ratio, each computed from that stage's own L_c and m_c."""
    lines = text.split("\n")
    for stage, sp in stage_points(point, ratio).items():
        idx, lc, mc = parse_crush(text, stage)
        old = lines[idx].split()
        if len(old) != 8 or old[2::2] != ["stiffness", "damping", "yield_strain"]:
            raise SweepError(f"the `{stage}` line has other fields than stiffness, damping, yield_strain: "
                             f"{' '.join(old)}")
        lines[idx] = crush_line(stage, sp, lc, mc, steel_e, rng)
    return "\n".join(lines)


# ---------------------------------------------------------------- Broadsword

def in_job(env):
    """A Broadsword job sets CARGO_TARGET_DIR (the shared build folder) for every child it starts."""
    return bool(env.get("BROADSWORD_JOB") or env.get("CARGO_TARGET_DIR"))


def broadsword_root(start):
    d = os.path.abspath(start)
    while True:
        if os.path.isfile(os.path.join(d, "broadsword.toml")):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            raise SweepError(f"no broadsword.toml in any folder above {start}")
        d = parent


def broadsword_paths(root, sweep_dir_name=SWEEP_DIR):
    """(broadsword binary, sweep folder) from broadsword.toml's install_dir and scratch_dir."""
    with open(os.path.join(root, "broadsword.toml"), "rb") as f:
        cfg = tomllib.load(f)
    for key in ("install_dir", "scratch_dir"):
        if key not in cfg:
            raise SweepError(f"broadsword.toml has no `{key}`")
    exe = "broadsword.exe" if os.name == "nt" else "broadsword"
    return (os.path.join(root, cfg["install_dir"], exe),
            os.path.join(root, cfg["scratch_dir"], sweep_dir_name))


def command(exe, task, body_rel, flags, solver=None):
    """The 4.4 command, run in the task's worktree, with only `--body` changed. With the m5 solver
    the flags are the frozen string with N5 for the step (`m5_flags`) and `--solver m5 --timing`
    come right after `--body <file>`, as 5.4 writes them."""
    extra = ["--solver", M5, "--timing"] if solver == M5 else []
    return [exe, "run", PROJECT, "--task", task, "--", "cargo", "run", "--offline", "--release",
            "--", "--body", body_rel] + extra + ["--pulse-report", PULSE_A] + flags


def m5_flags(flags, n5):
    """The frozen run flags with ` --substeps 10 ` replaced by ` --substeps <N5> `, nothing else
    changed; refused if that text is not there exactly once."""
    text = " " + " ".join(flags) + " "
    if text.count(M4_SUBSTEPS_TEXT) != 1:
        raise SweepError(f"frozen: run_flags does not hold `{M4_SUBSTEPS_TEXT.strip()}` exactly once")
    return text.replace(M4_SUBSTEPS_TEXT, f" --substeps {n5} ").split()


def m5_substeps(m5_doc):
    """N5, the `frozen: m5_substeps` line of docs/M5-SOLVER.md: one integer of at least 2."""
    v = frozen_numbers(m5_doc, "m5_substeps")
    if len(v) != 1 or v[0] != int(v[0]) or v[0] < 2:
        raise SweepError(f"frozen: m5_substeps is not one integer >= 2: {v}")
    return int(v[0])


def outcome(code, out):
    """(score text or 'none', verdict) of one run's output, or SweepError if the run is not a
    sim result (a Broadsword refusal, a usage error other than the step limit).

    The score is the `pulse_score=` line the 4.4 command prints, exactly as printed. Broadsword's
    verdict is the last `run:` line after the last `--- summary ---` line (`run: ok (N s)` or
    `run: FAILED (exit N)`); the sim's own lines are the ones before that summary."""
    all_lines = out.splitlines()
    marks = [i for i, l in enumerate(all_lines) if l == "--- summary ---"]
    if not marks:
        raise SweepError(f"not a sim result (exit {code}, no `--- summary ---` line); see the output file")
    sim, summary = all_lines[:marks[-1]], all_lines[marks[-1] + 1:]
    runs = [l for l in summary if l.startswith("run: ")]
    verdict = runs[-1] if runs else ""
    pulse = [l for l in sim if l.startswith("pulse: ")]
    scores = [l[len("pulse_score="):] for l in sim if l.startswith("pulse_score=")]
    if len(scores) > 1 or len(pulse) > 1:
        raise SweepError("more than one pulse_score or pulse line")
    score = scores[0] if scores else "none"
    if score != "none" and not re.fullmatch(r"\d+\.\d{4}", score):
        raise SweepError(f"pulse_score={score} is not 4 decimals")
    if re.fullmatch(r"run: ok( \([0-9.]+ s\))?", verdict) and code == 0 and pulse == ["pulse: ok"] and scores:
        return score, "ok"
    if (verdict.startswith("run: FAILED (exit 1)") and code == 1 and pulse
            and pulse[0].startswith("pulse: FAIL")):
        return score, "FAIL"
    if (verdict.startswith("run: FAILED (exit 2)") and code == 2 and not pulse and not scores
            and any("stability limit" in l and l.startswith("crash-sim: physics step h = ") for l in sim)):
        return "none", "refused"
    raise SweepError(f"not a sim result (exit {code}, {verdict or 'no run: line'}); see the output file")


def run_line(index, point, score, verdict, solver=None):
    a, y, z = (fmt(v) for v in point)
    return (f"run A {LABEL[solver]} {index} area_m2={a} yield_strain={y} damping_ratio={z} "
            f"score={score} verdict={verdict}")


# ---------------------------------------------------------------- the log

def read_text(path):
    with open(path, encoding="utf-8", newline="") as f:
        return f.read()


def read_log(path):
    if not os.path.exists(path):
        return None
    with open(path, encoding="utf-8", newline="") as f:
        return f.read()


def log_runs(text, solver=None):
    """The parsed `run A node-beam` (or, for m5, `run A m5`) lines of the log, in order."""
    out = []
    label = LABEL[solver]
    run_re = _run_re(label)
    for l in text.splitlines():
        if l.startswith(f"run A {label} "):
            m = run_re.match(l)
            if not m:
                raise SweepError(f"a malformed run line: {l}")
            out.append(m.groups())
    return out


def choose(runs):
    """(index, run) of the lowest score, ties to the lowest index; points without a score are
    never chosen."""
    best = None
    for r in runs:
        if r[4] == "none":
            continue
        key = (Decimal(r[4]), int(r[0]))
        if best is None or key < best[0]:
            best = (key, r)
    if best is None:
        raise SweepError("no grid point has a score")
    return best[1]


def check_runs_are_grid(runs, pts):
    if len(runs) != len(pts):
        raise SweepError(f"{len(runs)} run lines, not {len(pts)}")
    for i, (r, p) in enumerate(zip(runs, pts), start=1):
        if int(r[0]) != i:
            raise SweepError(f"run line {i} has index {r[0]}")
        if tuple(r[1:4]) != tuple(fmt(v) for v in p):
            raise SweepError(f"run line {i} is not grid point {i}")


def calibrated_line(run, rng, solver=None):
    point = tuple(float(v) for v in run[1:4])
    edge = " edge" if is_edge(point, rng) else ""
    return (f"calibrated {LABEL[solver]} {run[0]} area_m2={run[1]} yield_strain={run[2]} "
            f"damping_ratio={run[3]} score={run[4]}{edge}")


def append(path, text):
    with open(path, "a", encoding="utf-8", newline="\n") as f:
        f.write(text)


# ---------------------------------------------------------------- the four commands

def setup_for(repo, solver):
    """Everything a run needs, read from the docs' `frozen:` lines: the grid's ranges and points
    (m4: docs/STAGE6-M4-M5.md; m5: docs/M5-SOLVER.md), the run flags (m5: with N5 for the step),
    E, the strength ratio, and the test A wall file the variants are made from."""
    st6 = read_text(os.path.join(repo, "docs", "STAGE6-M4-M5.md"))
    steel_e = frozen_numbers(st6, "steel_E_pa")
    if len(steel_e) != 1:
        raise SweepError("frozen: steel_E_pa is not one number")
    flags = run_flags(st6)
    if solver == M5:
        gdoc = read_text(os.path.join(repo, "docs", "M5-SOLVER.md"))
        flags = m5_flags(flags, m5_substeps(gdoc))
        wall_rel, dir_name = WALL_A_M5, SWEEP_DIR_M5
    else:
        gdoc, wall_rel, dir_name = st6, WALL_A, SWEEP_DIR
    return types.SimpleNamespace(pts=grid(gdoc), rng=ranges(gdoc), flags=flags, steel_e=steel_e[0],
                                 ratio=strength_ratio(st6), wall_rel=wall_rel, dir_name=dir_name)


def write_variant(s, repo, sweep_dir, name, point):
    """Writes the wall file's variant for a grid point under the sweep folder; returns its path
    relative to the repo, as `--body` takes it."""
    with open(os.path.join(repo, s.wall_rel), encoding="utf-8", newline="") as f:
        wall = f.read()
    os.makedirs(sweep_dir, exist_ok=True)
    body = os.path.join(sweep_dir, name)
    with open(body, "w", encoding="utf-8", newline="") as f:
        f.write(with_crush(wall, point, s.steel_e, s.ratio, s.rng))
    return os.path.relpath(body, repo).replace(os.sep, "/")


def start_m5_block(text):
    """Appends nothing: says whether the m5 block may start, i.e. m4's `run B node-beam` line is
    in the log and the m5 header is not yet."""
    lines = text.splitlines() if text is not None else []
    if M5_HEADER in lines:
        return False
    if sum(1 for l in lines if l.startswith("run B node-beam ")) != 1:
        raise SweepError("the m5 block follows m4's `run B node-beam` line, and the log has none (or two)")
    return True


def do_run(task, first, last, env, runner=subprocess.run, repo=REPO, log=LOG, root_from=HERE, solver=None):
    if in_job(env):
        raise SweepError("refusing to start inside a Broadsword job (CARGO_TARGET_DIR or BROADSWORD_JOB "
                         "is set): run the sweep from a plain shell; it starts its own jobs")
    root = broadsword_root(root_from)
    s = setup_for(repo, solver)
    exe, sweep_base = broadsword_paths(root, s.dir_name)
    sweep_dir = os.path.join(sweep_base, task)  # one folder per task: an earlier try's files are never mixed in
    pts, rng, ratio, label = s.pts, s.rng, s.ratio, LABEL[solver]
    if not 1 <= first <= last <= len(pts):
        raise SweepError(f"--from {first} --to {last} is not inside 1..{len(pts)}")
    text = read_log(log)
    done = 0
    new_block = text is None
    if solver == M5:
        new_block = start_m5_block(text)
    if text is not None:
        if any(l.startswith(f"calibrated {label} ") for l in text.splitlines()):
            raise SweepError("the log already has its calibrated line")
        runs = log_runs(text, solver)
        check_runs_are_grid(runs, pts[:len(runs)])
        done = len(runs)
    if first != done + 1:
        raise SweepError(f"the log has {done} points, so the next chunk starts at {done + 1}, not {first}")
    if new_block:
        append(log, (M5_HEADER if solver == M5 else "calibration 1") + "\n")
    for i in range(first, last + 1):
        p = pts[i - 1]
        if not stages_inside(p, ratio, rng):
            # The rear stage (ratio x the front's P1) would leave its range: refused, not run.
            line = run_line(i, p, "none", "refused", solver)
            append(log, line + "\n")
            print(line, flush=True)
            continue
        rel = write_variant(s, repo, sweep_dir, f"point-{i:03d}.bsn", p)
        cmd = command(exe, task, rel, s.flags, solver)
        res = runner(cmd, cwd=root, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                     stdin=subprocess.DEVNULL)
        out = res.stdout.decode("utf-8", "replace")
        with open(os.path.join(sweep_dir, f"point-{i:03d}.txt"), "w", encoding="utf-8") as f:
            f.write(out)
        score, verdict = outcome(res.returncode, out)
        line = run_line(i, p, score, verdict, solver)
        append(log, line + "\n")
        print(line, flush=True)


def rtf_of(out):
    """The `rtf=` of the `timing:` line of a run's output, or None."""
    found = [l for l in out.splitlines() if l.startswith("timing: ")]
    if len(found) != 1:
        return None
    m = re.search(r" rtf=([0-9.]+)$", found[0])
    return float(m.group(1)) if m else None


def do_probe(task, index, env, runner=subprocess.run, repo=REPO, root_from=HERE):
    """R15: one grid point's m5 variant, run once with `--timing`, its whole output kept at
    <scratch>/m5-sweep/<task>/probe-<index>.txt. Nothing is logged and the score is not used."""
    if in_job(env):
        raise SweepError("refusing to start inside a Broadsword job (CARGO_TARGET_DIR or BROADSWORD_JOB "
                         "is set): run the sweep from a plain shell; it starts its own jobs")
    root = broadsword_root(root_from)
    s = setup_for(repo, M5)
    exe, sweep_base = broadsword_paths(root, s.dir_name)
    sweep_dir = os.path.join(sweep_base, task)
    if not 1 <= index <= len(s.pts):
        raise SweepError(f"--probe {index} is not inside 1..{len(s.pts)}")
    p = s.pts[index - 1]
    if not stages_inside(p, s.ratio, s.rng):
        raise SweepError(f"grid point {index}: its rear stage is outside the ranges (a refused point)")
    rel = write_variant(s, repo, sweep_dir, f"probe-{index}.bsn", p)
    res = runner(command(exe, task, rel, s.flags, M5), cwd=root, stdout=subprocess.PIPE,
                 stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
    out = res.stdout.decode("utf-8", "replace")
    with open(os.path.join(sweep_dir, f"probe-{index}.txt"), "w", encoding="utf-8") as f:
        f.write(out)
    rtf = rtf_of(out)
    line = f"probe {index} exit={res.returncode} rtf={'none' if rtf is None else rtf}"
    print(line, flush=True)
    return rtf


def do_choose(repo=REPO, log=LOG, solver=None):
    gdoc = read_text(os.path.join(repo, "docs", "M5-SOLVER.md" if solver == M5 else "STAGE6-M4-M5.md"))
    text = read_log(log)
    if text is None:
        raise SweepError("no validation/CALIBRATION.md")
    label = LABEL[solver]
    if any(l.startswith(f"calibrated {label} ") for l in text.splitlines()):
        raise SweepError("the log already has its calibrated line")
    if solver == M5 and M5_HEADER not in text.splitlines():
        raise SweepError("the log has no m5 block")
    runs = log_runs(text, solver)
    check_runs_are_grid(runs, grid(gdoc))
    line = calibrated_line(choose(runs), ranges(gdoc), solver)
    append(log, line + "\n")
    print(line)


def calibrated_point(text):
    cal = [CAL_RE.match(l) for l in text.splitlines() if l.startswith("calibrated node-beam ")]
    if len(cal) != 1 or cal[0] is None:
        raise SweepError("the log needs exactly one well-formed calibrated line")
    if cal[0].group(1) == "deviation":
        raise SweepError("the calibrated line is a recorded deviation: its values are the car file's, "
                         "so there is nothing to apply")
    return tuple(float(v) for v in cal[0].groups()[1:4])


def car_point(repo, doc):
    """(area_m2, yield_strain, damping_ratio) of test A's car file, the front stage's P1 to P3
    computed back as tests/validation_files.rs does, as logged text (fmt). The rear stage must be
    the frozen ratio times the front's P1 with the same P2 and P3 (within 1e-9), and the wall file
    the car file plus one plane line, or SweepError."""
    steel_e = frozen_numbers(doc, "steel_E_pa")[0]
    ratio = strength_ratio(doc)
    car = read_text(os.path.join(repo, CAR_A))
    wall = read_text(os.path.join(repo, WALL_A))
    rest = wall[len(car):] if wall.startswith(car) else None
    if rest is None or len(rest.splitlines()) != 1 or not rest.startswith("plane "):
        raise SweepError(f"{WALL_A} is not {CAR_A} plus one plane line")
    vals = {}
    for stage in STAGES:
        idx, lc, mc = parse_crush(car, stage)
        t = car.split("\n")[idx].split()
        if len(t) != 8 or t[2::2] != ["stiffness", "damping", "yield_strain"]:
            raise SweepError(f"the `{stage}` line has other fields: {' '.join(t)}")
        k, c, y = float(t[3]), float(t[5]), float(t[7])
        vals[stage] = (k * lc / steel_e, y, c / (2.0 * math.sqrt(k * mc)))
    f, r = vals["crush_front"], vals["crush_rear"]

    def close(a, b, tol):
        return a == b or abs(a - b) <= tol * max(abs(a), abs(b))

    # The strength ratio within 1e-9, as tests/validation_files.rs checks it. P3 within 1e-6: step
    # 8's files print the damping to 3 decimals, so the two stages' P3 differ by about 4e-7.
    if not (close(r[0] * r[1], ratio * f[0] * f[1], 1e-9) and r[1] == f[1] and close(r[2], f[2], 1e-6)):
        raise SweepError(f"the car file's rear stage {r} is not {ratio} x the front's P1 with its P2, P3 {f}")
    return tuple(fmt(v) for v in f)


def grid_best_line(run, rng):
    return "grid_best " + calibrated_line(run, rng)[len("calibrated "):] + f" verdict={run[5]}"


def deviation_lines(runs, rng, note, values, score):
    """The three lines a recorded deviation appends after the run lines."""
    if not note.strip() or "\n" in note:
        raise SweepError("a deviation needs a one-line reason")
    point = tuple(float(v) for v in values)
    edge = " edge" if is_edge(point, rng) else ""
    return [grid_best_line(choose(runs), rng), DEVIATION_PREFIX + note.strip(),
            f"calibrated node-beam deviation area_m2={values[0]} yield_strain={values[1]} "
            f"damping_ratio={values[2]} score={score}{edge}"]


def score_of_4_4(out, doc):
    """The `pulse_score=` of a saved 4.4 run on test A's wall file with the frozen run flags."""
    want = " ".join(["--body", WALL_A, "--pulse-report", PULSE_A] + run_flags(doc))
    lines = out.splitlines()
    marks = [i for i, l in enumerate(lines) if l == "--- summary ---"]
    if not marks:
        raise SweepError("the 4.4 output has no `--- summary ---` line")
    sim = lines[:marks[-1]]
    if not any(l.strip().startswith("Running `") and l.rstrip().endswith(" " + want + "`") for l in sim):
        raise SweepError("the 4.4 output is not the 4.4 command on test A's wall file with the frozen flags")
    scores = [l[len("pulse_score="):] for l in sim if l.startswith("pulse_score=")]
    if len(scores) != 1 or not re.fullmatch(r"\d+\.\d{4}", scores[0]):
        raise SweepError(f"the 4.4 output has {len(scores)} pulse_score lines, not one with 4 decimals")
    return scores[0]


def do_record_deviation(score_file, note, repo=REPO, log=LOG):
    doc = read_text(os.path.join(repo, "docs", "STAGE6-M4-M5.md"))
    text = read_log(log)
    if text is None:
        raise SweepError("no validation/CALIBRATION.md")
    if any(l.startswith(("calibrated node-beam ", "grid_best ", DEVIATION_PREFIX)) for l in text.splitlines()):
        raise SweepError("the log already has a calibrated, grid_best or deviation line")
    runs = log_runs(text)
    check_runs_are_grid(runs, grid(doc))
    score = score_of_4_4(read_text(score_file), doc)
    out = deviation_lines(runs, ranges(doc), note, car_point(repo, doc), score)
    append(log, "\n".join(out) + "\n")
    print("\n".join(out))


def calibrated_point_m5(text):
    cal = [CAL_RE_M5.match(l) for l in text.splitlines() if l.startswith("calibrated m5 ")]
    if len(cal) != 1 or cal[0] is None:
        raise SweepError("the log needs exactly one well-formed calibrated m5 line")
    return tuple(float(v) for v in cal[0].groups()[1:4])


def do_apply_m5(repo=REPO, log=LOG):
    """Writes the calibrated m5 point into the two crush lines of `validation_wall_m5.bsn` only,
    each stage computed from that file's own L_c and m_c, the rear by the frozen ratio."""
    s = setup_for(repo, M5)
    text = read_log(log)
    if text is None:
        raise SweepError("no validation/CALIBRATION.md")
    point = calibrated_point_m5(text)
    if not stages_inside(point, s.ratio, s.rng):
        raise SweepError("the calibrated point's stages are not inside the frozen ranges")
    path = os.path.join(repo, WALL_A_M5)
    new = with_crush(read_text(path), point, s.steel_e, s.ratio, s.rng)
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write(new)
    for stage in STAGES:
        print(f"{WALL_A_M5}: {[l for l in new.splitlines() if l.startswith(f'material {stage} ')][0]}")


def do_apply(repo=REPO, log=LOG, solver=None):
    if solver == M5:
        return do_apply_m5(repo, log)
    doc = read_text(os.path.join(repo, "docs", "STAGE6-M4-M5.md"))
    steel_e = frozen_numbers(doc, "steel_E_pa")[0]
    ratio = strength_ratio(doc)
    text = read_log(log)
    if text is None:
        raise SweepError("no validation/CALIBRATION.md")
    point = calibrated_point(text)
    rng = ranges(doc)
    if not stages_inside(point, ratio, rng):
        raise SweepError("the calibrated point's stages are not inside the frozen ranges")
    for car, wall in FILE_PAIRS:
        for rel in (car, wall):
            path = os.path.join(repo, rel)
            with open(path, encoding="utf-8", newline="") as f:
                old = f.read()
            new = with_crush(old, point, steel_e, ratio, rng)
            with open(path, "w", encoding="utf-8", newline="") as f:
                f.write(new)
        with open(os.path.join(repo, car), encoding="utf-8", newline="") as f:
            c = f.read()
        with open(os.path.join(repo, wall), encoding="utf-8", newline="") as f:
            w = f.read()
        rest = w[len(c):] if w.startswith(c) else None
        if rest is None or len(rest.splitlines()) != 1 or not rest.startswith("plane "):
            raise SweepError(f"{wall} is no longer {car} plus one plane line")
        for stage in STAGES:
            print(f"{car}: {[l for l in c.splitlines() if l.startswith(f'material {stage} ')][0]}")


def verify_m5(repo=REPO, log=LOG):
    """Check 5.3: the m5 block follows m4's `run B node-beam` line and is the grid in order, inside
    the ranges, with the calibrated m5 line the lowest score, then a `freeze_m5 = <40 hex>` line.
    Returns the last line to print."""
    try:
        gdoc = read_text(os.path.join(repo, "docs", "M5-SOLVER.md"))
        st6 = read_text(os.path.join(repo, "docs", "STAGE6-M4-M5.md"))
        text = read_log(log)
        if text is None:
            raise SweepError("no validation/CALIBRATION.md")
        lines = text.splitlines()
        hdr = [i for i, l in enumerate(lines) if l == M5_HEADER]
        if len(hdr) != 1:
            raise SweepError(f"{len(hdr)} `{M5_HEADER}` lines, not 1")
        if not any(l.startswith("run B node-beam ") for l in lines[:hdr[0]]):
            raise SweepError("the m5 block does not follow m4's `run B node-beam` line")
        if any(l.startswith(("run A m5 ", "calibrated m5 ", "freeze_m5")) for l in lines[:hdr[0]]):
            raise SweepError("an m5 line comes before the m5 header")
        pts, rng = grid(gdoc), ranges(gdoc)
        runs = log_runs(text, M5)
        check_runs_are_grid(runs, pts)
        ratio = strength_ratio(st6)
        for r in runs:
            for p, v in zip(PARAMS, r[1:4]):
                if not rng[p][0] <= float(v) <= rng[p][1]:
                    raise SweepError(f"run {r[0]}: {p}={v} is outside its range")
            if (not stages_inside(tuple(float(v) for v in r[1:4]), ratio, rng)
                    and (r[4], r[5]) != ("none", "refused")):
                raise SweepError(f"run {r[0]}: its rear stage is outside the ranges but it is not refused")
        cal_at = [i for i, l in enumerate(lines) if l.startswith("calibrated m5 ")]
        if len(cal_at) != 1:
            raise SweepError(f"{len(cal_at)} calibrated m5 lines, not 1")
        last_run = max(i for i, l in enumerate(lines) if l.startswith("run A m5 "))
        if cal_at[0] < last_run:
            raise SweepError("the calibrated m5 line comes before a run line")
        want = calibrated_line(choose(runs), rng, M5)
        if lines[cal_at[0]] != want:
            raise SweepError(f"the calibrated m5 line is not `{want}`")
        after = lines[cal_at[0] + 1:]
        if not after or not re.fullmatch(r"freeze_m5 = [0-9a-f]{40}", after[0]):
            raise SweepError("no `freeze_m5 = <40 hex>` line follows the calibrated m5 line")
        return "calibration: ok"
    except (SweepError, OSError, ValueError) as e:
        return f"calibration: FAIL {e}"


def verify(repo=REPO, log=LOG, solver=None):
    """Check 4.3. Returns the last line to print."""
    if solver == M5:
        return verify_m5(repo, log)
    try:
        doc = read_text(os.path.join(repo, "docs", "STAGE6-M4-M5.md"))
        text = read_log(log)
        if text is None:
            raise SweepError("no validation/CALIBRATION.md")
        lines = text.splitlines()
        if not lines or lines[0] != "calibration 1":
            raise SweepError("the first line is not `calibration 1`")
        pts, rng = grid(doc), ranges(doc)
        runs = log_runs(text)
        check_runs_are_grid(runs, pts)
        ratio = strength_ratio(doc)
        for r in runs:
            for p, v in zip(PARAMS, r[1:4]):
                if not rng[p][0] <= float(v) <= rng[p][1]:
                    raise SweepError(f"run {r[0]}: {p}={v} is outside its range")
            if (not stages_inside(tuple(float(v) for v in r[1:4]), ratio, rng)
                    and (r[4], r[5]) != ("none", "refused")):
                raise SweepError(f"run {r[0]}: its rear stage is outside the ranges but it is not refused")
        cal_at = [i for i, l in enumerate(lines) if l.startswith("calibrated node-beam ")]
        if len(cal_at) != 1:
            raise SweepError(f"{len(cal_at)} calibrated lines, not 1")
        last_run = max(i for i, l in enumerate(lines) if l.startswith("run A node-beam "))
        if cal_at[0] < last_run:
            raise SweepError("the calibrated line comes before a run line")
        extra = [i for i, l in enumerate(lines) if l.startswith(("grid_best ", DEVIATION_PREFIX))]
        m = CAL_RE.match(lines[cal_at[0]])
        if m is not None and m.group(1) == "deviation":
            # A recorded deviation: the grid's own choice is still logged, a reason is given, and
            # the calibrated values are exactly the ones the test A car file carries.
            if extra != [cal_at[0] - 2, cal_at[0] - 1] or cal_at[0] - 2 <= last_run:
                raise SweepError("a deviation needs `grid_best` then `deviation:` right before the calibrated line")
            want_best = grid_best_line(choose(runs), rng)
            if lines[cal_at[0] - 2] != want_best:
                raise SweepError(f"the grid_best line is not `{want_best}`")
            if not lines[cal_at[0] - 1][len(DEVIATION_PREFIX):].strip():
                raise SweepError("the deviation line gives no reason")
            car = car_point(repo, doc)
            want = deviation_lines(runs, rng, "x", car, m.group(5))[2]
            if lines[cal_at[0]] != want:
                raise SweepError(f"the deviation's calibrated line is not the car file's values: `{want}`")
        else:
            if extra:
                raise SweepError("grid_best or deviation lines without a deviation calibrated line")
            want = calibrated_line(choose(runs), rng)
            if lines[cal_at[0]] != want:
                raise SweepError(f"the calibrated line is not `{want}`")
        after = lines[cal_at[0] + 1:]
        if not after or not re.fullmatch(r"freeze = [0-9a-f]{40}", after[0]):
            raise SweepError("no `freeze = <40 hex>` line follows the calibrated line")
        return "calibration: ok"
    except (SweepError, OSError, ValueError) as e:
        return f"calibration: FAIL {e}"


def main(argv, env):
    args = argv[1:]
    solver = None
    if "--solver" in args:
        i = args.index("--solver")
        if args[i + 1:i + 2] != [M5] or "--solver" in args[i + 2:]:
            print("sweep: --solver takes `m5` (once); without it the m4 sweep runs", file=sys.stderr)
            return 2
        solver, args = M5, args[:i] + args[i + 2:]
    try:
        if args == ["--verify"]:
            line = verify(solver=solver)
            print(line)
            return 0 if line == "calibration: ok" else 1
        if args == ["--choose"]:
            do_choose(solver=solver)
            return 0
        if args == ["--apply"]:
            do_apply(solver=solver)
            return 0
        if solver == M5 and len(args) == 4 and args[0] == "--probe" and args[2] == "--task":
            do_probe(args[3], int(args[1]), env)
            return 0
        if solver is None and len(args) == 5 and args[0] == "--record-deviation" and args[1] == "--score-from" and args[3] == "--note":
            do_record_deviation(args[2], args[4])
            return 0
        if len(args) == 7 and args[0] == "--run" and args[1] == "--task" and args[3] == "--from" and args[5] == "--to":
            do_run(args[2], int(args[4]), int(args[6]), env, solver=solver)
            return 0
    except SweepError as e:
        print(f"sweep: {e}", file=sys.stderr)
        return 1
    except ValueError as e:
        print(f"sweep: {e}", file=sys.stderr)
        return 2
    print("usage: sweep.py [--solver m5] --run --task <id> --from <i> --to <j> | --choose | --apply | --verify"
          " | --solver m5 --probe <i> --task <id>"
          " | --record-deviation --score-from <4.4 output> --note <reason> (m4 only)", file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv, os.environ))
