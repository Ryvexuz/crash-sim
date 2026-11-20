"""Convert one NHTSA Vehicle Crash Test Database channel into a reference pulse.

Standard library only. Input: the test's EV zip as the database serves it
(v<test>.zip: the v<test>.EV5 specification file plus one ASCII file per
curve, v<test>.<curve>). Field positions follow NHTSA Test Reference Guide
Volume I (Vehicle Tests), Version 5, Revision November 2014.

    python -I -B validation/convert.py <ev zip> --curve 94 \
        --url <the zip's URL> --out validation/test-a

writes validation/test-a.pulse.csv and validation/test-a.meta.

What it does to the data, and nothing more:
  * time: t = (NFP + k) * DELT, the database's own time zero (index 0); the
    samples before time zero are kept;
  * units: G'S times 9.80665 gives m/s^2;
  * sign: none changed. The database's global X axis (XG) is positive
    forward (Guide Vol. I, AXIS), which is crash-sim's +x, so a frontal
    crash is negative in both;
  * no filtering and no trimming.

Anything it cannot convert exactly (another axis, other units, a channel
that is not "as measured", a time column that disagrees with NFP/DELT, a
missing field) is refused with exit code 2 and the reason on stderr.
"""

import argparse
import math
import sys
import zipfile

G = 9.80665  # m/s^2 per g (standard gravity; NHTSA's ISO export uses it too)

# AXIS codes accepted, with the factor that maps them onto crash-sim's +x
# (vehicle forward). Only the vehicle-fixed global X axis is accepted.
AXIS_SIGN = {"XG": 1.0}

# YUNITS codes accepted, with the factor to m/s^2.
UNIT_FACTOR = {"G'S": G}

# SENATT codes accepted, with the location text written to the meta file.
# Both are rear-seat-crossmember accelerometers in the tests used here
# (instrumentation commentary "LEFT/RIGHT REAR SEAT CROSSMEMBER X").
LOCATION = {
    "FLRR": "right rear seat crossmember",
    "FLLR": "left rear seat crossmember",
}

META_KEYS = ("test", "vehicle", "mass_kg", "speed_kmh", "channel", "channel_location",
             "sample_rate_hz", "prefilter", "url", "static_crush_m")

TIME_TOL_S = 0.5e-6  # the time column is printed to 1 us

# The fields read, by name and 1-based field number, exactly as the table of
# contents of NHTSA Test Reference Guide Vol. I, Version 5 (Revision Nov 2014)
# numbers them: 2.3 Test Information ("2. TITLE", "15. CLSSPD -- Closing
# Speed"), 2.4 Vehicle Information ("11. VEHTWT -- Vehicle Test Weight",
# "42. VEHSPD -- Vehicle Speed", "48 - 53. DPD1 - DPD6 -- Damage Profile
# Distances", "57. CRHDST -- Maximum Crush Distance") and Instrumentation
# Information ("2. CURNO" ... "20. INSCOM"). The EV5 records are '|'-separated
# fields in that order with no field names in the file, so every read goes
# through these tables and the VEHICLE record is cross-checked (VEHSPD equals
# CLSSPD, no DPD deeper than CRHDST) before anything is taken from it.
TEST_FIELD = {"TITLE": 2, "CLSSPD": 15}
VEHICLE_FIELD = {"VEHNO": 1, "VEHTWT": 11, "VEHSPD": 42,
                 "DPD1": 48, "DPD2": 49, "DPD3": 50, "DPD4": 51, "DPD5": 52, "DPD6": 53,
                 "CRHDST": 57}
INSTR_FIELD = {"VEHNO": 1, "CURNO": 2, "SENTYP": 3, "SENATT": 5, "AXIS": 6, "XUNITS": 7,
               "YUNITS": 8, "PREFIL": 9, "NFP": 15, "NLP": 16, "DELT": 17, "DASTAT": 18,
               "INSCOM": 20}
DPD_NAMES = ("DPD1", "DPD2", "DPD3", "DPD4", "DPD5", "DPD6")


def _field(record, table, name):
    """Field `name` of a split EV5 record, by its Guide field number."""
    return record[table[name] - 1]


class ConvertError(Exception):
    pass


def _section(lines, name):
    marker = "----- %s -----" % name
    if marker not in lines:
        raise ConvertError("EV5 file has no %s section" % name)
    out = []
    for line in lines[lines.index(marker) + 1:]:
        if line.startswith("----- "):
            break
        if line.strip():
            out.append(line.split("|"))
    if not out:
        raise ConvertError("EV5 %s section is empty" % name)
    return out


def _num(text, what):
    try:
        value = float(text)
    except ValueError:
        raise ConvertError("%s is not a number: %r" % (what, text)) from None
    if not math.isfinite(value):
        raise ConvertError("%s is not a finite number: %r" % (what, text))
    return value


def _int(text, what):
    try:
        return int(text)
    except ValueError:
        raise ConvertError("%s is not an integer: %r" % (what, text)) from None


def parse_ev5(text):
    lines = text.splitlines()
    test_no = None
    for line in lines:
        if line.startswith("#") and "Test Number:" in line:
            test_no = _int(line.split("Test Number:")[1].strip(), "test number")
    if test_no is None:
        raise ConvertError("EV5 header has no 'Test Number:' line")
    test = _section(lines, "TEST")[0]
    if len(test) < 20:
        raise ConvertError("EV5 TEST record has %d fields, expected 20" % len(test))
    vehicles = _section(lines, "VEHICLE")
    veh = [v for v in vehicles if _field(v, VEHICLE_FIELD, "VEHNO") == "1"]
    if len(veh) != 1 or len(veh[0]) < max(VEHICLE_FIELD.values()):
        raise ConvertError("EV5 needs exactly one VEHICLE record for vehicle 1 with >= %d fields"
                           % max(VEHICLE_FIELD.values()))
    instr = {}
    for row in _section(lines, "INSTRUMENTATION"):
        if len(row) < 20:
            raise ConvertError("EV5 INSTRUMENTATION record has %d fields, expected 20" % len(row))
        instr[_int(_field(row, INSTR_FIELD, "CURNO"), "CURNO")] = row
    return {"test_no": test_no, "test": test, "vehicle": veh[0], "instr": instr}


def parse_channel(text):
    vals = text.split()
    if len(vals) % 2:
        raise ConvertError("channel file is not time/value pairs")
    return [(_num(vals[i], "time"), _num(vals[i + 1], "value")) for i in range(0, len(vals), 2)]


def build(spec, points, curve, url):
    """Return (rows, meta) from a parsed EV5 spec and one channel's points."""
    if curve not in spec["instr"]:
        raise ConvertError("curve %d is not in the EV5 instrumentation list" % curve)
    f = spec["instr"][curve]
    vehno, sentyp, senatt, axis, xunits, yunits, prefil, nfp, nlp, delt, dastat, inscom = (
        _field(f, INSTR_FIELD, n) for n in ("VEHNO", "SENTYP", "SENATT", "AXIS", "XUNITS", "YUNITS",
                                            "PREFIL", "NFP", "NLP", "DELT", "DASTAT", "INSCOM"))
    if vehno != "1":
        raise ConvertError("curve %d is on vehicle %r, not vehicle 1" % (curve, vehno))
    if sentyp != "AC":
        raise ConvertError("curve %d is sensor type %r, not an accelerometer (AC)" % (curve, sentyp))
    if axis not in AXIS_SIGN:
        raise ConvertError("curve %d axis %r is not one of %s" % (curve, axis, sorted(AXIS_SIGN)))
    if xunits != "SEC":
        raise ConvertError("curve %d time units %r are not SEC" % (curve, xunits))
    if yunits not in UNIT_FACTOR:
        raise ConvertError("curve %d units %r are not one of %s" % (curve, yunits, sorted(UNIT_FACTOR)))
    if senatt not in LOCATION:
        raise ConvertError("curve %d attachment %r is not one of %s" % (curve, senatt, sorted(LOCATION)))
    if dastat != "AM":
        raise ConvertError("curve %d data status %r is not AM (as measured)" % (curve, dastat))
    nfp, nlp, delt = _int(nfp, "NFP"), _int(nlp, "NLP"), _int(delt, "DELT")
    prefil = _int(prefil, "PREFIL")
    if delt <= 0 or nfp > 0 or nlp <= 0:
        raise ConvertError("curve %d has NFP %d, NLP %d, DELT %d" % (curve, nfp, nlp, delt))
    if len(points) != nlp - nfp + 1:
        raise ConvertError("curve %d has %d points, NFP/NLP say %d" % (curve, len(points), nlp - nfp + 1))
    factor = AXIS_SIGN[axis] * UNIT_FACTOR[yunits]
    rows = []
    for k, (t_file, value) in enumerate(points):
        t = (nfp + k) * delt * 1e-6
        if abs(t - t_file) > TIME_TOL_S:
            raise ConvertError("curve %d point %d: time %r, NFP/DELT give %.7f" % (curve, k, t_file, t))
        rows.append((t, value * factor))

    test, veh = spec["test"], spec["vehicle"]
    title = _field(test, TEST_FIELD, "TITLE").strip()
    if not title.startswith("NCAP - "):
        raise ConvertError("test title %r does not start with 'NCAP - '" % title)
    mass = _int(_field(veh, VEHICLE_FIELD, "VEHTWT"), "VEHTWT")
    speed = _field(test, TEST_FIELD, "CLSSPD").strip()
    vehspd = _field(veh, VEHICLE_FIELD, "VEHSPD").strip()
    if _num(speed, "CLSSPD") != _num(vehspd, "VEHSPD"):
        raise ConvertError("VEHICLE field 42 (VEHSPD) %r is not TEST field 15 (CLSSPD) %r: "
                           "the VEHICLE record is not laid out as Guide Vol. I numbers it"
                           % (vehspd, speed))
    meta = {
        "test": str(spec["test_no"]),
        "vehicle": title[len("NCAP - "):],
        "mass_kg": str(mass),
        "speed_kmh": speed,
        "channel": "curve %d %s %s %s (%s)" % (curve, senatt, axis, yunits, inscom.strip()),
        "channel_location": LOCATION[senatt],
        "sample_rate_hz": "%g" % (1e6 / delt),
        "prefilter": "%d Hz low-pass by the lab (PREFIL); data as measured (DASTAT AM); "
                     "no further filtering" % prefil,
        "url": url,
    }
    dpd = [_field(veh, VEHICLE_FIELD, n).strip() for n in DPD_NAMES]
    if all(dpd) and any(_int(d, "DPD") != 0 for d in dpd):
        mm = [_int(d, n) for d, n in zip(dpd, DPD_NAMES)]
        crhdst = _int(_field(veh, VEHICLE_FIELD, "CRHDST"), "CRHDST")
        if max(mm) > crhdst:
            raise ConvertError("DPD1..DPD6 %s go deeper than CRHDST (maximum crush) %d mm: "
                               "the VEHICLE record is not laid out as Guide Vol. I numbers it"
                               % (mm, crhdst))
        meta["static_crush_m"] = "%.4f" % (sum(mm) / 6 / 1000.0)
    return rows, meta


def convert_zip(zip_file, curve, url):
    """zip_file: a path or a binary file object holding the database's EV zip."""
    with zipfile.ZipFile(zip_file) as z:
        names = z.namelist()
        specs = [n for n in names if n.upper().endswith(".EV5")]
        if len(specs) != 1:
            raise ConvertError("zip holds %d .EV5 files, expected 1" % len(specs))
        spec = parse_ev5(z.read(specs[0]).decode("latin-1"))
        chan = "%s.%03d" % (specs[0][:-4], curve)
        if chan not in names:
            raise ConvertError("zip has no channel file %s" % chan)
        points = parse_channel(z.read(chan).decode("latin-1"))
    return build(spec, points, curve, url)


def delta_v(rows, t_from=0.0):
    """Trapezoid integral of the acceleration in m/s, over the intervals that
    start at t >= t_from (default: t >= 0, the figure the README records and
    done check 2 judges); t_from=None integrates the full record."""
    dv = 0.0
    for (t0, a0), (t1, a1) in zip(rows, rows[1:]):
        if t_from is None or t0 >= t_from:
            dv += 0.5 * (a0 + a1) * (t1 - t0)
    return dv


def render_pulse_csv(rows):
    return "t_s,accel_mps2\n" + "".join("%.6f,%.6f\n" % (t, a) for t, a in rows)


def render_meta(meta):
    for k, v in meta.items():
        if k not in META_KEYS or not v or "\n" in v or v != v.strip():
            raise ConvertError("bad meta entry %r = %r" % (k, v))
    return "".join("%s = %s\n" % (k, v) for k, v in meta.items())


def read_meta(path):
    meta = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            if line.strip():
                key, sep, value = line.rstrip("\n").partition(" = ")
                if not sep:
                    raise ConvertError("%s: line without ' = ': %r" % (path, line))
                meta[key] = value
    return meta


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("zip", help="the test's EV zip from the NHTSA database")
    p.add_argument("--curve", type=int, required=True, help="curve number (CURNO)")
    p.add_argument("--url", required=True, help="where the zip was downloaded from")
    p.add_argument("--out", required=True, help="output prefix, e.g. validation/test-a")
    a = p.parse_args(argv)
    try:
        rows, meta = convert_zip(a.zip, a.curve, a.url)
        csv_text, meta_text = render_pulse_csv(rows), render_meta(meta)
    except (ConvertError, OSError, zipfile.BadZipFile) as e:
        print("convert: %s" % e, file=sys.stderr)
        return 2
    with open(a.out + ".pulse.csv", "w", encoding="ascii", newline="\n") as f:
        f.write(csv_text)
    with open(a.out + ".meta", "w", encoding="utf-8", newline="\n") as f:
        f.write(meta_text)
    speed = float(meta["speed_kmh"])
    dv = -delta_v(rows) * 3.6
    print("%s: test %s, %d samples, t %.4f..%.4f s, delta-V %.2f km/h vs impact %.2f km/h (%+.1f%%)"
          % (a.out, meta["test"], len(rows), rows[0][0], rows[-1][0], dv, speed, 100 * (dv - speed) / speed))
    return 0


if __name__ == "__main__":
    sys.exit(main())
