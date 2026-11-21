"""Tests for validation/v6_scatter.py.

Run directly: python -I -B validation/test_v6_scatter.py -v
(-m unittest does not work under -I; this file adds its own folder itself.)

The pulse tests use synthetic pulses whose answers are worked out by hand in
the comments, not taken from the code's own output. The last test reads the
real NHTSA files, and is skipped when the reference folder is not there (the
raw files are deleted later).
"""

import io
import math
import os
import sys
import tempfile
import unittest
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import v6_scatter as v6  # noqa: E402  (after the path set-up above)


def find_reference_root():
    """reference/nhtsa-pairs above this file (the repo sits at different depths
    in the main checkout and in a task worktree), or None."""
    d = HERE
    for _ in range(8):
        cand = os.path.join(d, "reference", "nhtsa-pairs")
        if os.path.isdir(cand):
            return cand
        d = os.path.dirname(d)
    return None


class CfcFilter(unittest.TestCase):
    def test_constant_stays_constant(self):
        out = v6.cfc_filter([-7.0] * 400, 1e-4, 60)
        self.assertTrue(all(abs(x + 7.0) < 1e-9 for x in out))

    def test_low_frequency_passes_high_frequency_is_cut(self):
        dt = 1e-4
        n = 20000
        low = [math.sin(2 * math.pi * 10 * k * dt) for k in range(n)]
        high = [math.sin(2 * math.pi * 1000 * k * dt) for k in range(n)]
        # CFC 60: the 10 Hz sine is far below its -3 dB point (100 Hz), the
        # 1000 Hz sine is 10x above it, where a 4-pole filter is down >60 dB.
        mid = slice(5000, 15000)
        self.assertGreater(max(v6.cfc_filter(low, dt, 60)[mid]), 0.99)
        self.assertLess(max(abs(x) for x in v6.cfc_filter(high, dt, 60)[mid]), 0.01)

    def test_refuses_low_sample_rate_and_non_finite(self):
        with self.assertRaises(v6.ScatterError):
            v6.cfc_filter([0.0] * 10, 1e-3, 180)  # 1 kHz < 10 x 180
        with self.assertRaises(v6.ScatterError):
            v6.cfc_filter([0.0, float("nan")], 1e-4, 60)


class Metrics(unittest.TestCase):
    def test_delta_v_trapezoid(self):
        # constant -10 m/s^2 over 1 s (1001 samples) integrates to -10 m/s
        self.assertAlmostEqual(v6.delta_v([-10.0] * 1001, 1e-3), -10.0, places=9)

    def test_peak_is_most_negative_and_first_tie_wins(self):
        self.assertEqual(v6.peak([1.0, -3.0, -5.0, -5.0, 2.0], 0.5), (-5.0, 1.0))

    def test_olc_constant_deceleration(self):
        # Constant vehicle deceleration A, start speed v0. Free flight ends when
        # A t1^2 / 2 = 0.065, t1 = sqrt(0.13 / A). With the occupant braked at
        # a_o to meet the vehicle at t2 (tau = t2 - t1) the relative
        # displacement is A tau t1 / 2 = 0.235, so tau = 0.47 / (A t1) and
        # a_o = A t2 / tau = A (1 + t1 / tau) = A (1 + 0.13 / 0.47) = 1.2766 A,
        # whatever A is. A = 100, v0 = 30: t2 = 0.166 s, vehicle still moving.
        dt = 1e-4
        samples = [-100.0] * 2001
        got = v6.olc(samples, dt, 30.0)
        self.assertAlmostEqual(got / 100.0, 1.0 + 0.13 / 0.47, delta=0.005)

    def test_olc_none_for_a_pulse_too_mild(self):
        # -1 m/s^2 for 0.2 s: the occupant moves 0.02 m, never 65 mm.
        self.assertIsNone(v6.olc([-1.0] * 2001, 1e-4, 15.0))

    def test_metrics_half_sine(self):
        # a(t) = -A sin(pi t / T), T = 0.1 s, A = 300 m/s^2, zero elsewhere; the
        # record starts 0.03 s before zero (nfp = -300). Peak 300 m/s^2 at
        # T/2 = 50 ms; delta-V = 2 A T / pi = 19.099 m/s. At 5 Hz the CFC 60
        # and CFC 180 filters change these by well under 1%.
        dt, A, T = 1e-4, 300.0, 0.1
        nfp = -300
        accel = [-A * math.sin(math.pi * (nfp + k) * dt / T) if 0 <= (nfp + k) * dt <= T else 0.0
                 for k in range(-nfp + 1801)]
        m = v6.metrics(accel, dt, nfp, 15.6)
        self.assertAlmostEqual(m["peak_mps2"], A, delta=0.01 * A)
        self.assertAlmostEqual(m["peak_time_s"], 0.05, delta=0.001)
        self.assertAlmostEqual(m["dv_mps"], 2 * A * T / math.pi, delta=0.01 * 2 * A * T / math.pi)

    def test_metrics_refuses_a_record_that_does_not_cover_the_window(self):
        with self.assertRaises(v6.ScatterError):
            v6.metrics([0.0] * 500, 1e-4, -100, 15.0)


class Statistics(unittest.TestCase):
    def test_rel_diff_is_over_the_mean(self):
        self.assertAlmostEqual(v6.rel_diff(9.0, 11.0), 0.2)
        self.assertIsNone(v6.rel_diff(None, 1.0))

    def test_median(self):
        self.assertEqual(v6.median([3.0, 1.0, 2.0]), 2.0)
        self.assertEqual(v6.median([4.0, 1.0, 2.0, 3.0]), 2.5)
        self.assertIsNone(v6.median([]))


class CountRule(unittest.TestCase):
    def veh(self, **kw):
        d = {"mass_kg": 2000, "dim13": 3000, "dim14": 5000, "year": "2010", "make": "01",
             "model": "02", "body": "4S"}
        d.update(kw)
        return d

    def test_same_vehicle_counts(self):
        self.assertTrue(v6.pair_count_decision(self.veh(), self.veh(mass_kg=2050))[0])

    def test_mass_over_5_percent_does_not_count(self):
        self.assertFalse(v6.pair_count_decision(self.veh(), self.veh(mass_kg=2200))[0])

    def test_long_dimension_over_5_percent_does_not_count(self):
        self.assertFalse(v6.pair_count_decision(self.veh(), self.veh(dim14=5400))[0])

    def test_other_year_does_not_count(self):
        self.assertFalse(v6.pair_count_decision(self.veh(), self.veh(year="2011"))[0])


class Inputs(unittest.TestCase):
    def test_zip_that_is_not_the_sources_hash_is_refused(self):
        with tempfile.TemporaryDirectory() as root:
            os.mkdir(os.path.join(root, "1234"))
            zpath = os.path.join(root, "1234", "v01234.zip")
            with zipfile.ZipFile(zpath, "w") as z:
                z.writestr("v01234.EV5", "x")
            with self.assertRaises(v6.ScatterError):
                v6.load_test(root, 1234, v6.RIGHT_REAR, {1234: "0" * 64})

    def test_sources_file_is_parsed(self):
        with tempfile.TemporaryDirectory() as root:
            with open(os.path.join(root, "SOURCES.md"), "w", encoding="utf-8") as f:
                f.write("tstno | bytes | sha256 | files\n3915 | 1 | " + "ab" * 32 + " | 143\n")
            self.assertEqual(v6.read_sources(root), {3915: "AB" * 32})


@unittest.skipIf(find_reference_root() is None, "reference/nhtsa-pairs is not present")
class ReferenceData(unittest.TestCase):
    def test_door_sill_is_not_a_seat_channel_and_pairs_without_a_common_kind_are_excluded(self):
        self.assertNotIn("DSRR", v6.RIGHT_REAR)
        self.assertNotIn("DSLR", v6.LEFT_REAR)
        buf = io.StringIO()
        v6.run(find_reference_root(), buf)
        lines = buf.getvalue().splitlines()
        excl = [ln for ln in lines if ln.startswith("pair ") and "| EXCLUDED |" in ln]
        self.assertEqual([ln.split(" ")[1] for ln in excl], ["6", "7"])
        self.assertIn("excluded pairs: 6,7", lines)

    def test_sixteen_tests_read_and_the_summary_is_printed(self):
        buf = io.StringIO()
        self.assertEqual(v6.run(find_reference_root(), buf), 0)
        lines = buf.getvalue().splitlines()
        test_rows = [ln for ln in lines if ln.startswith("test ")]
        self.assertEqual(len(test_rows), 16)
        marked = sorted(int(ln.split(" ")[1]) for ln in test_rows
                        if "channel excluded: no common channel kind in pair" in ln)
        self.assertEqual(marked, [6642, 6643, 6736, 6766])
        self.assertTrue(all("peak" not in ln for ln in test_rows
                            if "channel excluded" in ln))
        self.assertEqual(sum(1 for ln in lines if ln.startswith("pair ")), 8)
        self.assertIn("tests read: 16", lines)
        self.assertEqual(sum(1 for ln in lines if ln.startswith("counted | ")), 4)


if __name__ == "__main__":
    unittest.main()
