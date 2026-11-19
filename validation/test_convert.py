"""Tests for validation/convert.py.

Run directly: python -I -B validation/test_convert.py -v
(-m unittest does not work under -I, because -I leaves the script folder
off sys.path only for -m; this file adds its own folder itself.)

The sample below is hand-written in the NHTSA EV5 format (the .EV5
specification file and one ASCII channel file, both inside a zip, as the
database serves them). It is small on purpose: every expected number can be
checked by hand.
"""

import io
import os
import sys
import unittest
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import convert  # noqa: E402  (after the path set-up above)

G = 9.80665

# One TEST record, one VEHICLE record (81 fields), one BARRIER record and two
# INSTRUMENTATION records, in the field order of NHTSA Test Reference Guide
# Volume I, Version 5. Vehicle: VEHTWT 1300 kg (field 11), DPD1..DPD6 (fields
# 48..53) 100, 200, 300, 300, 200, 100 mm, mean 200 mm.
VEHICLE_FIELDS = (
    ["1", "34", "07", "2012", "MC0000", "4S", "VIN0", "4CTF", "1.6", "AF",
     "1300", "1150", "2568", "4364", "1688", "1083", "UN", "UNK", "P", ""]
    + ["0"] * 21              # 21..41 pre-test measurements
    + ["56.0", "0", "0", "DE", "NA", "NA"]   # 42..47
    + ["100", "200", "300", "300", "200", "100"]  # 48..53 DPD1..DPD6
    + ["12FDEW6", "1366", "0", "300"]        # 54..57
    + ["0"] * 21              # 58..78
    + ["0", "0", ""]          # 79..81
)
assert len(VEHICLE_FIELDS) == 81

INSTR_94 = ("1|94|AC|NA|FLRR|XG|SEC|G'S|2800|Mfg: X SN: 1|15/JUN/2011|-800|14|56.0"
            "|-3|4|100|AM|P|RIGHT REAR SEAT CROSSMEMBER X")
INSTR_95 = ("1|95|AC|NA|FLRR|YG|SEC|G'S|2800|Mfg: X SN: 2|15/JUN/2011|-800|14|56.0"
            "|-3|4|100|AM|P|RIGHT REAR SEAT CROSSMEMBER Y")


def ev5_text(vehicle=None, instr=(INSTR_94, INSTR_95), title="NCAP - 2012 SAMPLE CAR 4-DR"):
    vehicle = VEHICLE_FIELDS if vehicle is None else vehicle
    return "\n".join([
        "# Source: NHTSA Vehicle Database - Test Number: 01234",
        "# Date: <9/19/2023>",
        "----- EV5 -----",
        "----- TEST -----",
        "V5|" + title + "|OBJ|03/NOV/2011|MGA|C1|BT1|NCA|VTB|CON|DRY|22|OTH|OTH"
        "|56.0|0|0|0|2|DAS",
        "----- VEHICLE -----",
        "|".join(vehicle),
        "----- BARRIER -----",
        "R|134|0|0|RIGID BARRIER",
        "----- INSTRUMENTATION -----",
    ] + list(instr)) + "\n"


# 8 samples, NFP -3 .. NLP 4, DELT 100 us: t = -0.0003 .. 0.0004 s.
# Values in G's; the pulse after time zero is negative (frontal crash).
CHANNEL_94 = (
    "-0.000300\t0.100000\n"
    "-0.000200\t-0.100000\n"
    "-0.000100\t0.000000\n"
    "0.000000\t0.000000\n"
    "0.000100\t-1.000000\n"
    "0.000200\t-2.000000\n"
    "0.000300\t-1.000000\n"
    "0.000400\t0.500000\n"
)


def sample_zip(ev5=None, channels=None):
    """An in-memory zip laid out like the database's EV zip (v01234.zip)."""
    channels = {"v01234.094": CHANNEL_94} if channels is None else channels
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as z:
        z.writestr("v01234.EV5", ev5_text() if ev5 is None else ev5)
        for name, text in channels.items():
            z.writestr(name, text)
    buf.seek(0)
    return buf


URL = "https://nrd-static.nhtsa.dot.gov/compress/ev/vehdb/v00000/v01200/v01234.zip"


class ConvertSample(unittest.TestCase):
    def run_sample(self, **kw):
        return convert.convert_zip(sample_zip(**kw), 94, URL)

    def test_units_g_to_mps2(self):
        rows, _ = self.run_sample()
        accel = [a for _, a in rows]
        self.assertAlmostEqual(accel[4], -1.0 * G, places=9)
        self.assertAlmostEqual(accel[5], -2.0 * G, places=9)
        self.assertAlmostEqual(accel[0], 0.1 * G, places=9)

    def test_sign_kept_forward_positive(self):
        # NHTSA global X is positive forward, the same as crash-sim's +x, so a
        # frontal crash stays negative and a positive sample stays positive.
        rows, _ = self.run_sample()
        self.assertLess(rows[5][1], 0.0)
        self.assertGreater(rows[7][1], 0.0)

    def test_other_axis_refused(self):
        with self.assertRaises(convert.ConvertError):
            convert.convert_zip(sample_zip(channels={"v01234.095": CHANNEL_94}), 95, URL)

    def test_local_axis_refused(self):
        local = INSTR_94.replace("|XG|", "|XL|")
        with self.assertRaises(convert.ConvertError):
            self.run_sample(ev5=ev5_text(instr=(local,)))

    def test_other_units_refused(self):
        other = INSTR_94.replace("|G'S|", "|NWT|")
        with self.assertRaises(convert.ConvertError):
            self.run_sample(ev5=ev5_text(instr=(other,)))

    def test_questionable_channel_refused(self):
        qd = INSTR_94.replace("|AM|P|", "|QD|P|")
        with self.assertRaises(convert.ConvertError):
            self.run_sample(ev5=ev5_text(instr=(qd,)))

    def test_time_zero_and_pre_zero_samples_kept(self):
        rows, _ = self.run_sample()
        times = [t for t, _ in rows]
        self.assertEqual(len(rows), 8)
        self.assertAlmostEqual(times[0], -0.0003, places=12)
        self.assertAlmostEqual(times[3], 0.0, places=12)
        self.assertAlmostEqual(times[-1], 0.0004, places=12)

    def test_time_column_must_match_nfp_and_delt(self):
        shifted = CHANNEL_94.replace("0.000000\t0.000000", "0.000050\t0.000000")
        with self.assertRaises(convert.ConvertError):
            self.run_sample(channels={"v01234.094": shifted})

    def test_time_tolerance_edge(self):
        # The column is printed to 1 us; 0.4 us off passes, 0.6 us off is refused.
        near = CHANNEL_94.replace("0.000100\t-1.000000", "0.0001004\t-1.000000")
        rows, _ = self.run_sample(channels={"v01234.094": near})
        self.assertAlmostEqual(rows[4][0], 0.0001, places=12)  # time from NFP/DELT
        far = CHANNEL_94.replace("0.000100\t-1.000000", "0.0001006\t-1.000000")
        with self.assertRaises(convert.ConvertError):
            self.run_sample(channels={"v01234.094": far})

    def test_non_finite_value_refused(self):
        for bad in ("nan", "inf", "-inf"):
            with self.subTest(bad=bad):
                text = CHANNEL_94.replace("0.000200\t-2.000000", "0.000200\t" + bad)
                with self.assertRaises(convert.ConvertError):
                    self.run_sample(channels={"v01234.094": text})

    def test_point_count_must_match_nfp_nlp(self):
        short = "".join(CHANNEL_94.splitlines(True)[:-1])
        with self.assertRaises(convert.ConvertError):
            self.run_sample(channels={"v01234.094": short})

    def test_missing_curve_refused(self):
        with self.assertRaises(convert.ConvertError):
            convert.convert_zip(sample_zip(), 7, URL)

    def test_meta_keys_and_values(self):
        _, meta = self.run_sample()
        self.assertEqual(list(meta), [
            "test", "vehicle", "mass_kg", "speed_kmh", "channel", "channel_location",
            "sample_rate_hz", "prefilter", "url", "static_crush_m"])
        self.assertEqual(meta["test"], "1234")
        self.assertEqual(meta["vehicle"], "2012 SAMPLE CAR 4-DR")
        self.assertEqual(meta["mass_kg"], "1300")
        self.assertEqual(meta["speed_kmh"], "56.0")
        self.assertEqual(meta["sample_rate_hz"], "10000")
        self.assertEqual(meta["channel_location"], "right rear seat crossmember")
        self.assertEqual(meta["static_crush_m"], "0.2000")
        self.assertEqual(meta["url"], URL)
        self.assertIn("2800", meta["prefilter"])
        self.assertIn("94", meta["channel"])

    def test_static_crush_absent_when_not_reported(self):
        veh = list(VEHICLE_FIELDS)
        veh[47:53] = ["0", "0", "0", "0", "0", "0"]
        _, meta = self.run_sample(ev5=ev5_text(vehicle=veh))
        self.assertNotIn("static_crush_m", meta)
        veh[47:53] = ["", "", "", "", "", ""]
        _, meta = self.run_sample(ev5=ev5_text(vehicle=veh))
        self.assertNotIn("static_crush_m", meta)

    def test_title_without_ncap_prefix_refused(self):
        with self.assertRaises(convert.ConvertError):
            self.run_sample(ev5=ev5_text(title="SOME OTHER STUDY"))

    def test_rendered_csv(self):
        rows, _ = self.run_sample()
        text = convert.render_pulse_csv(rows)
        lines = text.splitlines()
        self.assertEqual(lines[0], "t_s,accel_mps2")
        self.assertEqual(lines[1], "-0.000300,0.980665")
        self.assertEqual(lines[5], "0.000100,-9.806650")
        self.assertEqual(len(lines), 9)

    def test_rendered_meta(self):
        _, meta = self.run_sample()
        text = convert.render_meta(meta)
        self.assertTrue(text.startswith("test = 1234\n"))
        for line in text.splitlines():
            key, sep, value = line.partition(" = ")
            self.assertEqual(sep, " = ")
            self.assertEqual(value.strip(), value)
            self.assertTrue(value)

    def test_delta_v_of_sample(self):
        rows, _ = self.run_sample()
        # trapezoid by hand over t >= 0: 0.0001 * G * (-0.5 - 1.5 - 1.5 - 0.25)
        self.assertAlmostEqual(convert.delta_v(rows), 0.0001 * G * -3.75, places=12)


class CommittedPulses(unittest.TestCase):
    """Done check 2 on the committed files: units, sign and time zero."""

    def test_reference_pulses(self):
        for name in ("test-a", "test-b"):
            with self.subTest(name=name):
                with open(os.path.join(HERE, name + ".pulse.csv"), encoding="ascii") as f:
                    lines = f.read().splitlines()
                self.assertEqual(lines[0], "t_s,accel_mps2")
                rows = [tuple(float(x) for x in ln.split(",")) for ln in lines[1:]]
                self.assertLessEqual(rows[0][0], 0.0)
                self.assertGreaterEqual(rows[-1][0], 0.2)
                meta = convert.read_meta(os.path.join(HERE, name + ".meta"))
                speed = float(meta["speed_kmh"]) / 3.6
                dv = convert.delta_v(rows)
                self.assertLess(dv, 0.0)
                self.assertLessEqual(abs(-dv - speed) / speed, 0.10)


def load_pulse(name):
    with open(os.path.join(HERE, name + ".pulse.csv"), encoding="ascii") as f:
        lines = f.read().splitlines()
    return [tuple(float(x) for x in ln.split(",")) for ln in lines[1:]]


def readme_row(text, label):
    """The cells after the label of the one README table row starting with it."""
    rows = [ln for ln in text.splitlines() if ln.startswith("| " + label + " |")]
    if len(rows) != 1:
        raise AssertionError("README needs exactly one row %r, found %d" % (label, len(rows)))
    return [c.strip() for c in rows[0].strip().strip("|").split("|")][1:]


class ReadmeFigures(unittest.TestCase):
    """Every figure the README records for tests A and B is recomputed from the
    committed pulse and meta files; a README figure that drifts fails here."""

    @classmethod
    def setUpClass(cls):
        with open(os.path.join(HERE, "README.md"), encoding="utf-8") as f:
            cls.readme = f.read()
        cls.names = ("test-a", "test-b")
        cls.meta = {n: convert.read_meta(os.path.join(HERE, n + ".meta")) for n in cls.names}
        cls.rows = {n: load_pulse(n) for n in cls.names}

    def test_number_mass_speed(self):
        numbers = readme_row(self.readme, "NHTSA test number")
        mass = readme_row(self.readme, "Test mass (VEHTWT)")
        speed = readme_row(self.readme, "Impact speed (CLSSPD)")
        for i, n in enumerate(self.names):
            with self.subTest(name=n):
                m = self.meta[n]
                self.assertTrue(numbers[i].startswith(m["test"] + " "), numbers[i])
                self.assertTrue(mass[i].startswith(m["mass_kg"] + " kg"), mass[i])
                self.assertTrue(speed[i].startswith(m["speed_kmh"] + " km/h"), speed[i])

    def test_delta_v_row(self):
        cells = readme_row(self.readme, "Delta-V, trapezoid over t >= 0")
        for i, n in enumerate(self.names):
            with self.subTest(name=n):
                kmh = -convert.delta_v(self.rows[n]) * 3.6
                pct = (kmh / float(self.meta[n]["speed_kmh"]) - 1.0) * 100.0
                self.assertEqual(cells[i], "%.2f km/h, %+.1f%% of impact speed" % (kmh, pct))

    def test_candidate_table_full_record(self):
        # "Why these two": delta-V of the full record (t < 0 included).
        for n in self.names:
            with self.subTest(name=n):
                r = self.rows[n]
                full = sum(0.5 * (a0 + a1) * (t1 - t0) for (t0, a0), (t1, a1) in zip(r, r[1:]))
                pct = (-full * 3.6 / float(self.meta[n]["speed_kmh"]) - 1.0) * 100.0
                cells = readme_row(self.readme, self.meta[n]["test"])
                self.assertEqual(cells[1], self.meta[n]["mass_kg"])
                self.assertEqual(cells[2], "%+.1f%%" % pct)

    def test_static_crush_row(self):
        cells = readme_row(self.readme, "Static crush C1..C6 (DPD1..6)")
        for i, n in enumerate(self.names):
            with self.subTest(name=n):
                profile, sep, mean = cells[i].partition(" mm; mean ")
                self.assertEqual(sep, " mm; mean ", cells[i])
                c = [int(x) for x in profile.split(",")]
                self.assertEqual(len(c), 6)
                self.assertEqual(mean, "%.4f m" % (sum(c) / 6 / 1000.0))
                self.assertEqual(mean, self.meta[n]["static_crush_m"] + " m")


if __name__ == "__main__":
    unittest.main()
