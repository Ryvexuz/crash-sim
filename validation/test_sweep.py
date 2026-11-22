"""Tests for validation/sweep.py.

Run directly: python -I -B validation/test_sweep.py -v
(-m unittest does not work under -I; this file adds its own folder to sys.path itself.)

The name of the other test's files is never written in this file: where a test needs it, it is
built from parts, so this file itself passes the "no file names it" rule it checks. No test here
reads, runs or edits the other car's files.
"""

import os
import shutil
import subprocess
import sys
import tempfile
import types
import unittest
from decimal import Decimal

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import sweep  # noqa: E402

REPO = os.path.dirname(HERE)
DOC_TEXT = sweep.read_text(os.path.join(REPO, "docs", "STAGE6-M4-M5.md"))
OTHER_TEST = "test-" + chr(ord("a") + 1)  # the name of the test that is never run here

# Lazlo's pre-approved ranges on test A (state/standing-orders.md, 2026-11-22 04:44Z), recorded in
# docs/STAGE6-M4-M5.md, "Calibration step 8". They are an outer bound the frozen grid must sit in;
# the grid itself is read from the doc's `frozen:` lines, never from these numbers.
APPROVED_FRONT_AREA = (7e-5, 0.004)
APPROVED_FRONT_YIELD = (0.0010, 0.0114)
APPROVED_DAMPING_RATIO = (0.0, 0.15)
APPROVED_RATIO = (2.0, 4.0)
APPROVED_CABIN_DAMPING = (5 * 666.667, 20 * 666.667)


def result(code, text):
    return types.SimpleNamespace(returncode=code, stdout=text.encode("utf-8"))


OK_OUT = ("job: run crash-sim\npulse_ref=7504\npulse_score=0.8123\npulse: ok\n--- summary ---\n"
          "run: ok (4.6 s)\n")
FAIL_OUT = ("pulse_ref=7504\npulse_score=2.3070\npulse: FAIL ground contact at t=0.1022\n"
            "--- summary ---\nrun: FAILED (exit 1)\n")
UNSTABLE_OUT = "pulse: FAIL unstable at t = 0.0100 s: NaN\n--- summary ---\nrun: FAILED (exit 1)\n"
REFUSED_OUT = ("crash-sim: physics step h = dt/substeps = 0.000100 s reaches the body's stability limit "
               "0.000095 s (...)\n--- summary ---\nrun: FAILED (exit 2)\n")
OTHER_USAGE_OUT = "crash-sim: the body has no group `sensor`\n--- summary ---\nrun: FAILED (exit 2)\n"
BUSY_OUT = "broadsword: build folder busy\n"
# The sim's own output may hold text that looks like Broadsword's verdict; only the summary counts.
FAKE_OK_OUT = ("pulse_score=0.1000\npulse: FAIL peak\nrun: ok\n--- summary ---\nrun: FAILED (exit 1)\n")


class Grid(unittest.TestCase):
    def test_140_points_in_order(self):
        g = sweep.grid(DOC_TEXT)
        self.assertEqual(len(g), 140)
        self.assertEqual(g[0], (0.00007, 0.001, 0.0))
        self.assertEqual(g[1], (0.00007, 0.001, 0.01))
        self.assertEqual(g[3], (0.00007, 0.001, 0.05))
        self.assertEqual(g[4], (0.00007, sweep.axis(DOC_TEXT, "yield_strain")[1], 0.0))
        self.assertEqual(g[19], (0.00007, 0.0057, 0.05))
        self.assertEqual(g[20], (sweep.axis(DOC_TEXT, "area_m2")[1], 0.001, 0.0))
        self.assertEqual(g[139], (0.002, 0.0057, 0.05))
        self.assertEqual(len(set(g)), 140)

    def test_at_most_140_points(self):
        """STAGE6: a fixed grid of at most 140 points."""
        self.assertLessEqual(len(sweep.grid(DOC_TEXT)), 140)

    def test_axis_values(self):
        a1 = sweep.axis(DOC_TEXT, "area_m2")
        self.assertEqual(len(a1), 7)
        self.assertEqual((a1[0], a1[-1]), (0.00007, 0.002))  # the ends are the range ends exactly
        for i in range(6):
            self.assertAlmostEqual(a1[i + 1] / a1[i], (0.002 / 0.00007) ** (1 / 6), places=12)
        a2 = sweep.axis(DOC_TEXT, "yield_strain")
        self.assertEqual([sweep.fmt(v) for v in a2], ["0.001", "0.002175", "0.00335", "0.004525", "0.0057"])
        self.assertEqual(sweep.axis(DOC_TEXT, "damping_ratio"), [0.0, 0.01, 0.025, 0.05])

    def test_grid_inside_the_frozen_ranges(self):
        rng = sweep.ranges(DOC_TEXT)
        for p in sweep.grid(DOC_TEXT):
            for name, v in zip(sweep.PARAMS, p):
                self.assertTrue(rng[name][0] <= v <= rng[name][1], (name, v))

    def test_grid_inside_the_approved_ranges(self):
        """Every point (the front stage's P1 to P3, the rear's P3) and the frozen ratio lie inside
        Lazlo's pre-approved ranges of 2026-11-22 04:44Z; the cabin damping the variants keep too."""
        ratio = sweep.strength_ratio(DOC_TEXT)
        self.assertTrue(APPROVED_RATIO[0] <= ratio <= APPROVED_RATIO[1], ratio)
        for p in sweep.grid(DOC_TEXT):
            area, ys, zeta = p
            self.assertTrue(APPROVED_FRONT_AREA[0] <= area <= APPROVED_FRONT_AREA[1], p)
            self.assertTrue(APPROVED_FRONT_YIELD[0] <= ys <= APPROVED_FRONT_YIELD[1], p)
            for stage, sp in sweep.stage_points(p, ratio).items():
                self.assertTrue(APPROVED_DAMPING_RATIO[0] <= sp[2] <= APPROVED_DAMPING_RATIO[1], (stage, p))
        wall = sweep.read_text(os.path.join(REPO, sweep.WALL_A))
        cabin = [l.split() for l in wall.split("\n") if l.split()[:2] == ["material", "cabin"]]
        self.assertEqual(len(cabin), 1)
        self.assertTrue(APPROVED_CABIN_DAMPING[0] <= float(cabin[0][5]) <= APPROVED_CABIN_DAMPING[1])

    def test_deterministic(self):
        """The grid, its order and every variant file are the same bytes on every call."""
        self.assertEqual(sweep.grid(DOC_TEXT), sweep.grid(DOC_TEXT))
        wall = sweep.read_text(os.path.join(REPO, sweep.WALL_A))
        rng, ratio = sweep.ranges(DOC_TEXT), sweep.strength_ratio(DOC_TEXT)
        pts = [p for p in sweep.grid(DOC_TEXT) if sweep.stages_inside(p, ratio, rng)]
        self.assertEqual(len(pts), 80)
        for p in pts[::7]:
            self.assertEqual(sweep.with_crush(wall, p, 210e9, ratio, rng),
                             sweep.with_crush(wall, p, 210e9, ratio, rng))

    def test_grid_comes_from_the_doc(self):
        doc = DOC_TEXT.replace("frozen: grid crush.area_m2 = 7 log", "frozen: grid crush.area_m2 = 3 log")
        self.assertNotEqual(doc, DOC_TEXT)
        self.assertEqual(len(sweep.grid(doc)), 60)
        doc = DOC_TEXT.replace("frozen: range crush.yield_strain = 0.0010 0.0057",
                               "frozen: range crush.yield_strain = 0.0010 0.0050")
        self.assertNotEqual(doc, DOC_TEXT)
        self.assertEqual(sweep.axis(doc, "yield_strain")[-1], 0.005)
        with self.assertRaises(sweep.SweepError):
            sweep.grid(DOC_TEXT.replace("frozen: grid crush.damping_ratio", "nothing"))
        bad = DOC_TEXT.replace("frozen: grid crush.damping_ratio = 0 0.01 0.025 0.05",
                               "frozen: grid crush.damping_ratio = 0 0.01 0.025 0.06")
        self.assertNotEqual(bad, DOC_TEXT)
        with self.assertRaises(sweep.SweepError):  # an explicit value outside the range
            sweep.grid(bad)


class Flags(unittest.TestCase):
    def test_flags_are_the_frozen_line(self):
        line = [l for l in DOC_TEXT.splitlines() if l.startswith("frozen: run_flags = ")]
        self.assertEqual(len(line), 1)
        self.assertEqual(sweep.run_flags(DOC_TEXT), line[0][len("frozen: run_flags = "):].split())
        doc = DOC_TEXT.replace("--substeps 10 --pulse-sensor", "--substeps 11 --pulse-sensor", 1)
        self.assertIn("11", sweep.run_flags(doc))

    def test_command_is_4_4_with_only_body_changed(self):
        # The 4.4 command as the doc writes it, from `cargo run` to the redirection.
        four4 = [l for l in DOC_TEXT.splitlines()
                 if l.startswith("`broadsword run crash-sim -- cargo run") and "validation_wall.bsn" in l]
        self.assertEqual(len(four4), 1)
        want = four4[0].split(" -- ", 1)[1].split(" > ")[0].split()
        got = sweep.command("bs.exe", "t-1", "../x/point-001.bsn", sweep.run_flags(DOC_TEXT))
        self.assertEqual(got[:6], ["bs.exe", "run", "crash-sim", "--task", "t-1", "--"])
        body_at = got.index("--body") + 1
        self.assertEqual(got[body_at], "../x/point-001.bsn")
        got_rest = got[6:]
        got_rest[got_rest.index("--body") + 1] = "scenarios/validation_wall.bsn"
        self.assertEqual(got_rest, want)


class Numbers(unittest.TestCase):
    def test_fmt_is_rust_display(self):
        self.assertEqual(sweep.fmt(0.00007), "0.00007")
        self.assertEqual(sweep.fmt(300000.0), "300000")
        self.assertEqual(sweep.fmt(0.0), "0")
        self.assertEqual(sweep.fmt(1e-7), "0.0000001")
        for x in sweep.axis(DOC_TEXT, "area_m2") + [62820512.82051281, 1883.2794410260665]:
            self.assertEqual(float(sweep.fmt(x)), x)

    def stage_values(self, text, stage):
        """(P1, P2, P3) of one stage computed back from a file's text, as tests/validation_files.rs."""
        idx, lc, mc = sweep.parse_crush(text, stage)
        t = text.split("\n")[idx].split()
        k, c, y = float(t[3]), float(t[5]), float(t[7])
        return k * lc / 210e9, y, c / (2.0 * (k * mc) ** 0.5)

    def test_crush_lines(self):
        ratio = sweep.strength_ratio(DOC_TEXT)
        self.assertEqual(ratio, 4.0)
        rng = sweep.ranges(DOC_TEXT)
        for rel in ("scenarios/validation_car.bsn", "scenarios/validation_wall.bsn"):
            text = sweep.read_text(os.path.join(REPO, rel))
            point = (0.0003741657386773942, 0.00335, 0.025)
            new = sweep.with_crush(text, point, 210e9, ratio, rng)
            old_lines, new_lines = text.split("\n"), new.split("\n")
            diff = [i for i, (a, b) in enumerate(zip(old_lines, new_lines)) if a != b]
            self.assertEqual(len(old_lines), len(new_lines))
            self.assertEqual([new_lines[i].split()[:2] for i in diff],
                             [["material", "crush_front"], ["material", "crush_rear"]])
            front = self.stage_values(new, "crush_front")
            rear = self.stage_values(new, "crush_rear")
            self.assertAlmostEqual(front[0] / point[0], 1.0, places=12)
            self.assertEqual(front[1], point[1])
            self.assertAlmostEqual(front[2] / point[2], 1.0, places=12)
            self.assertEqual(rear[1:2], front[1:2])
            self.assertAlmostEqual(rear[2] / front[2], 1.0, places=12)

    def test_range_ends_compute_back_inside(self):
        """A grid value on a range end computes back inside the frozen range (the step 8 record:
        a front stiffness printed one digit low computed back to P1 below 0.00007)."""
        rng, ratio = sweep.ranges(DOC_TEXT), sweep.strength_ratio(DOC_TEXT)
        text = sweep.read_text(os.path.join(REPO, "scenarios", "validation_car.bsn"))
        for p in sweep.grid(DOC_TEXT):
            if not sweep.stages_inside(p, ratio, rng):
                continue
            new = sweep.with_crush(text, p, 210e9, ratio, rng)
            for stage in sweep.STAGES:
                for name, v in zip(sweep.PARAMS, self.stage_values(new, stage)):
                    self.assertTrue(rng[name][0] <= v <= rng[name][1], (p, stage, name, v))
        with self.assertRaises(sweep.SweepError):  # never more than a few ulps
            sweep._into(1.0, lambda x: x, 2.0, 3.0, "x")
        self.assertEqual(sweep._into(1.0, lambda x: x, 1.0, 3.0, "x"), 1.0)

    def test_rear_follows_the_frozen_ratio(self):
        """The rear stage's plateau strength (P1 x P2) is the frozen ratio times the front's, for
        every grid point; the ratio is read from the doc, not a copy."""
        text = sweep.read_text(os.path.join(REPO, "scenarios", "validation_car.bsn"))
        rng = sweep.ranges(DOC_TEXT)
        wide = dict(rng, area_m2=(rng["area_m2"][0], 1.0))  # so every grid point builds
        for ratio in (sweep.strength_ratio(DOC_TEXT), 3.0):
            for point in sweep.grid(DOC_TEXT)[::17]:
                new = sweep.with_crush(text, point, 210e9, ratio, wide)
                f, r = self.stage_values(new, "crush_front"), self.stage_values(new, "crush_rear")
                self.assertAlmostEqual((r[0] * r[1]) / (f[0] * f[1]) / ratio, 1.0, places=12)
        doc = DOC_TEXT.replace("frozen: crush.rear_to_front_strength = 4",
                               "frozen: crush.rear_to_front_strength = 3")
        self.assertNotEqual(doc, DOC_TEXT)
        self.assertEqual(sweep.strength_ratio(doc), 3.0)
        with self.assertRaises(sweep.SweepError):
            sweep.strength_ratio(DOC_TEXT.replace("frozen: crush.rear_to_front_strength", "nothing"))

    def test_rear_outside_the_range(self):
        rng, ratio = sweep.ranges(DOC_TEXT), sweep.strength_ratio(DOC_TEXT)
        areas = sweep.axis(DOC_TEXT, "area_m2")
        inside = [a for a in areas if sweep.stages_inside((a, 0.001, 0.0), ratio, rng)]
        self.assertEqual(inside, [a for a in areas if ratio * a <= rng["area_m2"][1]])
        self.assertEqual(len(inside), 4)
        self.assertFalse(sweep.stages_inside((areas[4], 0.001, 0.0), ratio, rng))


class Outcome(unittest.TestCase):
    def test_verdicts(self):
        self.assertEqual(sweep.outcome(0, OK_OUT), ("0.8123", "ok"))
        self.assertEqual(sweep.outcome(0, OK_OUT.replace("run: ok (4.6 s)", "run: ok")), ("0.8123", "ok"))
        self.assertEqual(sweep.outcome(1, FAIL_OUT), ("2.3070", "FAIL"))
        self.assertEqual(sweep.outcome(1, UNSTABLE_OUT), ("none", "FAIL"))
        self.assertEqual(sweep.outcome(2, REFUSED_OUT), ("none", "refused"))

    def test_score_is_the_pulse_score_line(self):
        """The score is 4.4's `pulse_score=` line as printed, never recomputed."""
        out = OK_OUT.replace("pulse_score=0.8123", "pulse_score=12.3456")
        self.assertEqual(sweep.outcome(0, out), ("12.3456", "ok"))
        with self.assertRaises(sweep.SweepError):
            sweep.outcome(0, OK_OUT.replace("pulse_score=0.8123", "pulse_score=0.812"))
        with self.assertRaises(sweep.SweepError):
            sweep.outcome(0, OK_OUT.replace("pulse_score=0.8123\n", "pulse_score=0.8123\npulse_score=0.1\n"))

    def test_not_a_result_stops(self):
        for code, out in ((2, OTHER_USAGE_OUT), (3, BUSY_OUT), (0, FAIL_OUT), (1, OK_OUT),
                          (1, REFUSED_OUT), (1, FAKE_OK_OUT.replace("exit 1", "exit 3"))):
            with self.assertRaises(sweep.SweepError, msg=out):
                sweep.outcome(code, out)

    def test_only_the_summary_verdict_counts(self):
        self.assertEqual(sweep.outcome(1, FAKE_OK_OUT), ("0.1000", "FAIL"))
        with self.assertRaises(sweep.SweepError):
            sweep.outcome(0, FAKE_OK_OUT)

    def test_tie_rule(self):
        runs = [("1", "a", "b", "c", "1.0000", "FAIL"), ("2", "a", "b", "c", "0.5000", "FAIL"),
                ("3", "a", "b", "c", "none", "refused"), ("4", "a", "b", "c", "0.5000", "ok"),
                ("5", "a", "b", "c", "0.5001", "ok")]
        self.assertEqual(sweep.choose(runs)[0], "2")
        self.assertEqual(sweep.choose(runs[2:])[0], "4")
        with self.assertRaises(sweep.SweepError):
            sweep.choose([runs[2]])

    def test_edge(self):
        rng = sweep.ranges(DOC_TEXT)
        g = sweep.grid(DOC_TEXT)
        inside = (sweep.axis(DOC_TEXT, "area_m2")[3], 0.00335, 0.025)
        self.assertIn(inside, g)
        self.assertFalse(sweep.is_edge(inside, rng))
        for p in ((0.00007, 0.00335, 0.025), (inside[0], 0.0057, 0.025), (inside[0], 0.00335, 0.0),
                  (inside[0], 0.00335, 0.05), (0.002, 0.00335, 0.025), (inside[0], 0.001, 0.025)):
            self.assertTrue(sweep.is_edge(p, rng), p)
        run = ("6", "0.00007", "0.00335", "0.025", "0.4000", "FAIL")
        self.assertTrue(sweep.calibrated_line(run, rng).endswith(" score=0.4000 edge"))
        run = ("7",) + tuple(sweep.fmt(v) for v in inside) + ("0.4000", "FAIL")
        self.assertTrue(sweep.calibrated_line(run, rng).endswith(" score=0.4000"))


class Sandbox(unittest.TestCase):
    """A fake Broadsword root with a fake crash-sim repo inside, and a fake runner."""

    def setUp(self):
        self.root = tempfile.mkdtemp()
        with open(os.path.join(self.root, "broadsword.toml"), "w") as f:
            f.write('install_dir = "build/bin"\nscratch_dir = "scratch"\n')
        self.repo = os.path.join(self.root, "worktrees", "crash-sim", "t-1")
        for d in ("docs", "scenarios", "validation"):
            os.makedirs(os.path.join(self.repo, d))
        shutil.copy(os.path.join(REPO, "docs", "STAGE6-M4-M5.md"), os.path.join(self.repo, "docs"))
        for car, wall in sweep.FILE_PAIRS:
            for rel in (car, wall):
                shutil.copy(os.path.join(REPO, rel), os.path.join(self.repo, "scenarios"))
        self.log = os.path.join(self.repo, "validation", "CALIBRATION.md")
        self.calls = []
        self.outputs = {}

    def tearDown(self):
        shutil.rmtree(self.root)

    def runner(self, cmd, **kw):
        self.calls.append((cmd, kw))
        body = cmd[cmd.index("--body") + 1]
        i = int(body.rsplit("point-", 1)[1][:3])
        return self.outputs.get(i, result(1, FAIL_OUT.replace("2.3070", f"{2 + i / 1000:.4f}")))

    def run_chunk(self, first, last, env=None):
        sweep.do_run("t-1", first, last, env or {}, runner=self.runner, repo=self.repo, log=self.log,
                     root_from=os.path.join(self.repo, "validation"))

    def test_apply_touches_test_a_files_only(self):
        self.assertEqual(sweep.FILE_PAIRS, (("scenarios/validation_car.bsn", "scenarios/validation_wall.bsn"),))

    def test_chunks_log_and_variants(self):
        self.outputs[2] = result(2, REFUSED_OUT)
        self.outputs[3] = result(0, OK_OUT)
        self.run_chunk(1, 3)
        with self.assertRaises(sweep.SweepError):  # not the next index
            self.run_chunk(5, 6)
        with self.assertRaises(sweep.SweepError):  # already logged
            self.run_chunk(3, 4)
        self.run_chunk(4, 4)
        lines = sweep.read_text(self.log).splitlines()
        self.assertEqual(lines[0], "calibration 1")
        self.assertEqual(lines[1], "run A node-beam 1 area_m2=0.00007 yield_strain=0.001 damping_ratio=0 "
                                   "score=2.0010 verdict=FAIL")
        self.assertEqual(lines[2], "run A node-beam 2 area_m2=0.00007 yield_strain=0.001 damping_ratio=0.01 "
                                   "score=none verdict=refused")
        self.assertTrue(lines[3].endswith("score=0.8123 verdict=ok"))
        self.assertTrue(lines[4].startswith("run A node-beam 4 ")
                        and lines[4].endswith("damping_ratio=0.05 score=2.0040 verdict=FAIL"))
        self.assertEqual(len(lines), 5)
        # Each variant is the wall file with only the two crush lines changed, under scratch/m4-sweep/<task id>.
        wall = sweep.read_text(os.path.join(self.repo, "scenarios", "validation_wall.bsn"))
        sweep_dir = os.path.join(self.root, "scratch", "m4-sweep", "t-1")
        for i in range(1, 5):
            v = sweep.read_text(os.path.join(sweep_dir, f"point-{i:03d}.bsn"))
            d = [(a, b) for a, b in zip(wall.split("\n"), v.split("\n")) if a != b]
            self.assertEqual(len(d), 2)
            self.assertTrue(d[0][1].startswith("material crush_front "))
            self.assertTrue(d[1][1].startswith("material crush_rear "))
            self.assertTrue(os.path.exists(os.path.join(sweep_dir, f"point-{i:03d}.txt")))
        cmd, kw = self.calls[0]
        self.assertEqual(cmd[0], os.path.join(self.root, "build/bin", "broadsword.exe" if os.name == "nt" else "broadsword"))
        self.assertEqual(kw["cwd"], self.root)
        self.assertEqual(cmd[cmd.index("--body") + 1], "../../../scratch/m4-sweep/t-1/point-001.bsn")
        self.assertEqual(cmd[cmd.index("--pulse-report") + 1], "validation/test-a.pulse.csv")

    def test_not_a_result_stops_the_chunk(self):
        self.outputs[2] = result(3, BUSY_OUT)
        with self.assertRaises(sweep.SweepError):
            self.run_chunk(1, 3)
        lines = sweep.read_text(self.log).splitlines()
        self.assertEqual(len(lines), 2)  # point 2 is not logged, so the next chunk starts at 2
        self.assertEqual(len(self.calls), 2)
        del self.outputs[2]
        self.run_chunk(2, 2)

    def test_refused_inside_a_job(self):
        for env in ({"CARGO_TARGET_DIR": "x"}, {"BROADSWORD_JOB": "1"}):
            with self.assertRaises(sweep.SweepError) as e:
                self.run_chunk(1, 1, env)
            self.assertIn("inside a Broadsword job", str(e.exception))
        self.assertEqual(self.calls, [])
        self.assertFalse(os.path.exists(self.log))

    def test_refused_inside_a_job_as_a_process(self):
        env = dict(os.environ, CARGO_TARGET_DIR=os.path.join(self.root, "build"))
        p = subprocess.run([sys.executable, "-I", "-B", os.path.join(HERE, "sweep.py"), "--run", "--task",
                            "t-none", "--from", "1", "--to", "1"], env=env, capture_output=True, text=True)
        self.assertEqual(p.returncode, 1)
        self.assertIn("refusing to start inside a Broadsword job", p.stderr)
        self.assertEqual(p.stdout, "")

    def test_rear_outside_is_refused_without_a_run(self):
        self.full_log({}, upto=80)
        self.run_chunk(81, 82)
        self.assertEqual(self.calls, [])
        lines = sweep.read_text(self.log).splitlines()
        self.assertTrue(lines[-1].startswith("run A node-beam 82 ") and lines[-1].endswith(" score=none verdict=refused"))

    def full_log(self, scores, upto=140):
        rng, ratio = sweep.ranges(DOC_TEXT), sweep.strength_ratio(DOC_TEXT)
        with open(self.log, "w", encoding="utf-8", newline="\n") as f:
            f.write("calibration 1\n")
            for i, p in enumerate(sweep.grid(DOC_TEXT)[:upto], start=1):
                if sweep.stages_inside(p, ratio, rng):
                    f.write(sweep.run_line(i, p, scores.get(i, "9.0000"), "FAIL") + "\n")
                else:
                    f.write(sweep.run_line(i, p, "none", "refused") + "\n")

    def test_choose_apply_verify(self):
        self.full_log({70: "0.5000", 72: "0.5000"})
        self.assertTrue(sweep.verify(self.repo, self.log).startswith("calibration: FAIL 0 calibrated lines"))
        sweep.do_choose(self.repo, self.log)
        lines = sweep.read_text(self.log).splitlines()
        rng = sweep.ranges(DOC_TEXT)
        p70 = sweep.grid(DOC_TEXT)[69]
        self.assertEqual(lines[-1], "calibrated node-beam 70 area_m2={} yield_strain={} damping_ratio={} score=0.5000"
                         .format(*(sweep.fmt(v) for v in p70)) + (" edge" if sweep.is_edge(p70, rng) else ""))
        with self.assertRaises(sweep.SweepError):
            sweep.do_choose(self.repo, self.log)
        with self.assertRaises(sweep.SweepError):  # no more runs after the calibrated line
            self.run_chunk(141, 141)
        self.assertEqual(sweep.verify(self.repo, self.log),
                         "calibration: FAIL no `freeze = <40 hex>` line follows the calibrated line")
        wall_before = sweep.read_text(os.path.join(self.repo, "scenarios", "validation_wall.bsn"))
        sweep.do_apply(self.repo, self.log)
        for car, wall in sweep.FILE_PAIRS:
            c = sweep.read_text(os.path.join(self.repo, car))
            w = sweep.read_text(os.path.join(self.repo, wall))
            self.assertTrue(w.startswith(c))
            self.assertTrue(w[len(c):].startswith("plane "))
            for stage, sp in sweep.stage_points(p70, 4.0).items():
                _, lc, mc = sweep.parse_crush(c, stage)
                self.assertIn(sweep.crush_line(stage, sp, lc, mc, 210e9, rng) + "\n", c)
        # The applied wall file is byte for byte the variant the sweep runs for that point, so 4.4
        # reproduces the logged score.
        variant = sweep.with_crush(wall_before, p70, 210e9, 4.0, rng)
        self.assertEqual(sweep.read_text(os.path.join(self.repo, "scenarios", "validation_wall.bsn")), variant)
        with open(self.log, "a", encoding="utf-8") as f:
            f.write("freeze = " + "a" * 40 + "\n")
        self.assertEqual(sweep.verify(self.repo, self.log), "calibration: ok")

    def test_verify_catches_a_rear_outside_run(self):
        self.full_log({})
        text = sweep.read_text(self.log)
        p = sweep.grid(DOC_TEXT)[139]
        text = text.replace(sweep.run_line(140, p, "none", "refused"), sweep.run_line(140, p, "0.1000", "ok"))
        with open(self.log, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
        sweep.do_choose(self.repo, self.log)
        with open(self.log, "a", encoding="utf-8") as f:
            f.write("freeze = " + "a" * 40 + "\n")
        self.assertTrue(sweep.verify(self.repo, self.log).startswith("calibration: FAIL run 140: its rear stage"))
        with self.assertRaises(sweep.SweepError):
            sweep.do_apply(self.repo, self.log)

    def test_verify_catches_a_wrong_choice(self):
        self.full_log({70: "0.5000"})
        with open(self.log, "a", encoding="utf-8") as f:
            line = sweep.calibrated_line(("71",) + tuple(sweep.fmt(v) for v in sweep.grid(DOC_TEXT)[70])
                                         + ("9.0000", "FAIL"), sweep.ranges(DOC_TEXT))
            f.write(line + "\nfreeze = " + "a" * 40 + "\n")
        self.assertTrue(sweep.verify(self.repo, self.log).startswith("calibration: FAIL the calibrated line is not"))

    def four4_out(self, body="scenarios/validation_wall.bsn", score="0.6832", flags=None):
        flags = sweep.run_flags(DOC_TEXT) if flags is None else flags
        cmd = " ".join(["--body", body, "--pulse-report", sweep.PULSE_A] + flags)
        path = os.path.join(self.root, f"four4-{len(os.listdir(self.root))}.txt")  # a new file each call
        with open(path, "w", encoding="utf-8") as f:
            f.write(f"     Running `C:\\x\\crash-sim.exe {cmd}`\npulse_score={score}\npulse: ok\n"
                    "--- summary ---\nrun: ok (6.4 s)\n")
        return path

    def record_deviation(self, note="chosen elsewhere; no threshold change"):
        self.full_log({41: "0.6456"})
        sweep.do_record_deviation(self.four4_out(), note, self.repo, self.log)
        with open(self.log, "a", encoding="utf-8") as f:
            f.write("freeze = " + "a" * 40 + "\n")

    def write_log(self, text):
        with open(self.log, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)

    def test_deviation_accepted_with_the_car_files_values(self):
        self.record_deviation()
        lines = sweep.read_text(self.log).splitlines()
        rng = sweep.ranges(DOC_TEXT)
        self.assertEqual(lines[-4], sweep.grid_best_line(sweep.log_runs(sweep.read_text(self.log))[40], rng))
        self.assertTrue(lines[-4].startswith("grid_best node-beam 41 ") and lines[-4].endswith(" verdict=FAIL"))
        self.assertEqual(lines[-3], "deviation: chosen elsewhere; no threshold change")
        self.assertTrue(lines[-2].startswith("calibrated node-beam deviation area_m2="))
        self.assertTrue(lines[-2].endswith(" score=0.6832"), lines[-2])
        car = sweep.car_point(self.repo, DOC_TEXT)
        self.assertEqual(lines[-2], f"calibrated node-beam deviation area_m2={car[0]} yield_strain={car[1]} "
                                    f"damping_ratio={car[2]} score=0.6832")
        self.assertAlmostEqual(float(car[0]) / 7e-5, 1.0, places=9)  # step 8's point in main's files
        self.assertEqual(car[1], "0.0024")
        self.assertAlmostEqual(float(car[2]) / 0.015, 1.0, places=5)
        self.assertEqual(sweep.verify(self.repo, self.log), "calibration: ok")
        with self.assertRaises(sweep.SweepError):  # nothing to apply
            sweep.do_apply(self.repo, self.log)
        with self.assertRaises(sweep.SweepError):  # recorded once only
            sweep.do_record_deviation(self.four4_out(), "again", self.repo, self.log)

    def test_deviation_fails_when_the_car_file_differs(self):
        self.record_deviation()
        for rel in ("scenarios/validation_car.bsn", "scenarios/validation_wall.bsn"):
            path = os.path.join(self.repo, rel)
            text = sweep.read_text(path)
            for stage in sweep.STAGES:  # both stages, so the car file stays a valid two-stage car
                line = [l for l in text.split("\n") if l.startswith(f"material {stage} ")][0]
                t = line.split()
                t[7] = "0.0025"
                text = text.replace(line, " ".join(t))
            with open(path, "w", encoding="utf-8", newline="") as f:
                f.write(text)
        self.assertTrue(sweep.verify(self.repo, self.log).startswith(
            "calibration: FAIL the deviation's calibrated line is not the car file's values"))

    def test_deviation_fails_on_edited_values(self):
        self.record_deviation()
        text = sweep.read_text(self.log)
        self.write_log(text.replace(" yield_strain=0.0024 ", " yield_strain=0.0025 "))
        self.assertTrue(sweep.verify(self.repo, self.log).startswith(
            "calibration: FAIL the deviation's calibrated line is not the car file's values"))

    def test_deviation_needs_grid_best_and_a_reason(self):
        self.record_deviation()
        text = sweep.read_text(self.log)
        best = [l for l in text.splitlines() if l.startswith("grid_best ")][0]
        self.write_log(text.replace(best + "\n", ""))
        self.assertTrue(sweep.verify(self.repo, self.log).startswith("calibration: FAIL a deviation needs"))
        self.write_log(text.replace(best, best.replace("score=0.6456", "score=0.6455")))
        self.assertTrue(sweep.verify(self.repo, self.log).startswith("calibration: FAIL the grid_best line is not"))
        self.write_log(text.replace("deviation: chosen elsewhere; no threshold change", "deviation: "))
        self.assertEqual(sweep.verify(self.repo, self.log), "calibration: FAIL the deviation line gives no reason")
        # Without the deviation marker the grid rule applies again: the grid's choice is #41.
        cal = [l for l in text.splitlines() if l.startswith("calibrated ")][0]
        self.write_log(text.replace(cal, cal.replace("node-beam deviation", "node-beam 41")))
        self.assertTrue(sweep.verify(self.repo, self.log).startswith("calibration: FAIL grid_best or deviation lines"))

    def test_deviation_score_is_a_4_4_run_on_the_wall_file(self):
        self.full_log({41: "0.6456"})
        for path, why in ((self.four4_out(body="../x/point-041.bsn"), "not the 4.4 command"),
                          (self.four4_out(flags=sweep.run_flags(DOC_TEXT)[:-2]), "not the 4.4 command"),
                          (self.four4_out(score="0.683"), "pulse_score lines")):
            with self.assertRaises(sweep.SweepError) as e:
                sweep.do_record_deviation(path, "x", self.repo, self.log)
            self.assertIn(why, str(e.exception))
            self.assertFalse(any(l.startswith("calibrated ") for l in sweep.read_text(self.log).splitlines()))
        with self.assertRaises(sweep.SweepError):
            sweep.do_record_deviation(self.four4_out(), " ", self.repo, self.log)

    def test_verify_catches_a_reordered_grid(self):
        self.full_log({})
        text = sweep.read_text(self.log).split("\n")
        text[1], text[2] = text[2], text[1]
        with open(self.log, "w", encoding="utf-8", newline="\n") as f:
            f.write("\n".join(text))
        self.assertTrue(sweep.verify(self.repo, self.log).startswith("calibration: FAIL run line 1 has index 2"))


class NoOtherTest(unittest.TestCase):
    def test_the_other_test_is_never_named(self):
        for name in ("sweep.py", "test_sweep.py"):
            text = sweep.read_text(os.path.join(HERE, name)).lower()
            self.assertNotIn(OTHER_TEST, text, name)
            self.assertNotIn(OTHER_TEST.replace("-", "_"), text, name)
            self.assertNotIn("validation_car_" + chr(ord("a") + 1), text, name)
            self.assertNotIn("validation_wall_" + chr(ord("a") + 1), text, name)
        self.assertEqual(sweep.PULSE_A, "validation/test-a.pulse.csv")


class Decimals(unittest.TestCase):
    def test_scores_compare_as_decimals(self):
        self.assertLess(Decimal("0.9999"), Decimal("10.0000"))
        runs = [("1", "a", "b", "c", "10.0000", "FAIL"), ("2", "a", "b", "c", "9.9999", "FAIL")]
        self.assertEqual(sweep.choose(runs)[0], "2")


if __name__ == "__main__":
    unittest.main()
