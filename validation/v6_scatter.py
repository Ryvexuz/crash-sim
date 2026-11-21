"""Measure repeat-test scatter (Stage 6 figure V6) from the 8 NHTSA full-frontal pairs.

Standard library only. Reads the raw NHTSA EV5 zips under a folder laid out as
<root>/<tstno>/v0<tstno>.zip (reference/nhtsa-pairs, read-only; this script
never writes anywhere) and prints one row per test, one per pair, and the
median and max of the pair differences.

    python -I validation/v6_scatter.py reference/nhtsa-pairs

Definitions follow docs/STAGE6-M4-M5.md ("Metrics", line 81 to 84), which are
the Rust functions in src/pulse.rs ported line for line:

  * "dv_mps: delta-V = -integral of a (CFC 180) over [0, window], trapezoid
    (n20-03 `delta_v`); includes rebound."
  * "peak_mps2: magnitude of the most negative CFC 60 value in the window
    (n20-03 `peak`); `peak_time_s` its time."
  * "olc_mps2: OLC (V4) of the CFC 180 pulse with v0 = ... meta `speed_kmh`
    / 3.6 (n20-03 `olc`)."
  * "Each pulse is filtered at its own sample rate over its whole record (the
    reference keeps its samples before zero, so the filter's start-up is
    outside the window)".

The window is 0.15 s from time zero (`--pulse-window 0.15`, line 151 of the
same file). Time zero is the database's own index 0. Channels are read with
convert.py's parsers and unit/axis tables (convert.py is not edited); its
build() is not used because it only accepts titles starting "NCAP - " and two
seat-crossmember codes, while the older tests here use other codes.
"""

import hashlib
import math
import os
import sys
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import convert  # noqa: E402  (after the path set-up above)

WINDOW_S = 0.15          # docs/STAGE6-M4-M5.md line 151: --pulse-window 0.15
CFC_SHAPE = 60.0         # same line: --pulse-cfc-shape 60 (peak, time of peak)
CFC_INT = 180.0          # same line: --pulse-cfc-int 180 (delta-V, OLC)
OLC_FREE_FLIGHT_M = 0.065
OLC_RESTRAINT_M = 0.235
G = convert.G

# Pairs as printed in SAE 2018-01-1055 Appendix A Table 22 (state/nhtsa-download-list.md).
PAIRS = [(6258, 6271), (4244, 5036), (5061, 4240), (6724, 6729),
         (6759, 6764), (6736, 6766), (6642, 6643), (3915, 5073)]

# A pair counts only if mass differs by <= 5% (brief), the model year is the
# same and VEHICLE fields 13 and 14 (the two long dimensions, mm) differ by
# <= 5%: the body style code alone (EX, PU, 4P) does not say whether the
# front structure is the same, the dimensions do. The result is also printed
# for the counted pairs whose body codes are identical.
MASS_TOL = 0.05
DIM_TOL = 0.05

# Rear-seat-crossmember SENATT codes, right side first. SELR/SERR/DSLR/DSRR
# are the old-format and 2900 Hz-class codes for the same two seat
# accelerometers (INSCOM of 3915: "LEFT REAR X", "RIGHT REAR X"; 6729: "V1
# RIGHT REAR #2X"); FLLR/FLRR are convert.py's own LOCATION codes ("left/right
# rear seat crossmember"). The Guide's SENATT table was not read here, so the
# code meaning is taken from those comments only.
RIGHT_REAR = ("SERR", "DSRR", "FLRR")
LEFT_REAR = ("SELR", "DSLR", "FLLR")


class ScatterError(Exception):
    pass


# ---------------------------------------------------------------- pulse maths

def cfc_filter(samples, dt, cfc):
    """SAE J211-1 Appendix C phaseless 4-pole Butterworth; port of pulse.rs cfc_filter."""
    if not (math.isfinite(dt) and dt > 0 and math.isfinite(cfc) and cfc > 0):
        raise ScatterError("bad dt %r or cfc %r" % (dt, cfc))
    if 1.0 / dt < 10.0 * cfc:
        raise ScatterError("sample rate %.0f Hz is under 10 x CFC %g" % (1.0 / dt, cfc))
    if any(not math.isfinite(x) for x in samples):
        raise ScatterError("non-finite sample")
    n = len(samples)
    if n < 2:
        return list(samples)
    wd = 2.0 * math.pi * cfc * 2.0775
    wa = math.sin(wd * dt / 2.0) / math.cos(wd * dt / 2.0)
    s2 = math.sqrt(2.0)
    den = 1.0 + s2 * wa + wa * wa
    a0 = wa * wa / den
    a1 = 2.0 * a0
    a2 = a0
    b1 = -2.0 * (wa * wa - 1.0) / den
    b2 = (-1.0 + s2 * wa - wa * wa) / den
    pad = min(int(math.ceil(3.0 / (cfc * dt))), n - 1)
    ext = [2.0 * samples[0] - samples[k] for k in range(pad, 0, -1)]
    ext.extend(samples)
    ext.extend(2.0 * samples[n - 1] - samples[n - 1 - k] for k in range(1, pad + 1))

    def one_pass(x):
        y = []
        x1 = x2 = y1 = y2 = x[0]
        for xt in x:
            yt = a0 * xt + a1 * x1 + a2 * x2 + b1 * y1 + b2 * y2
            y.append(yt)
            x2, x1, y2, y1 = x1, xt, y1, yt
        return y

    fwd = one_pass(ext)
    fwd.reverse()
    both = one_pass(fwd)
    both.reverse()
    return both[pad:pad + n]


def delta_v(samples, dt):
    """Trapezoid integral of the samples (pulse.rs delta_v)."""
    return sum(0.5 * (samples[i - 1] + samples[i]) * dt for i in range(1, len(samples)))


def peak(samples, dt):
    """(most negative sample, its time); ties keep the first (pulse.rs peak)."""
    if not samples:
        raise ScatterError("peak of an empty pulse")
    best = 0
    for i, x in enumerate(samples):
        if x < samples[best]:
            best = i
    return samples[best], best * dt


def olc(samples, dt, v0):
    """Occupant Load Criterion in m/s^2, or None; port of pulse.rs olc."""
    if not (math.isfinite(v0) and v0 > 0):
        raise ScatterError("v0 must be finite and > 0")
    n = len(samples)
    if n < 2:
        return None
    vv = [v0] * n
    xv = [0.0] * n
    s = [0.0] * n
    for i in range(1, n):
        vv[i] = vv[i - 1] + 0.5 * (samples[i - 1] + samples[i]) * dt
        xv[i] = xv[i - 1] + 0.5 * (vv[i - 1] + vv[i]) * dt
        s[i] = s[i - 1] + 0.5 * ((v0 - vv[i - 1]) + (v0 - vv[i])) * dt

    def lerp(a, b, f):
        return a + (b - a) * f

    i1 = next((i for i in range(1, n) if s[i] >= OLC_FREE_FLIGHT_M), None)
    if i1 is None:
        return None
    f1 = (OLC_FREE_FLIGHT_M - s[i1 - 1]) / (s[i1] - s[i1 - 1])
    t1 = (i1 - 1) * dt + f1 * dt
    vv1 = lerp(vv[i1 - 1], vv[i1], f1)
    xv1 = lerp(xv[i1 - 1], xv[i1], f1)
    prev_t, prev_vv, prev_d = t1, vv1, 0.0
    for j in range(i1, n):
        t2 = j * dt
        if t2 <= t1:
            continue
        d = (t2 - t1) * (v0 + vv[j]) / 2.0 - (xv[j] - xv1)
        if d >= OLC_RESTRAINT_M:
            f = (OLC_RESTRAINT_M - prev_d) / (d - prev_d)
            t2i = lerp(prev_t, t2, f)
            vv2 = lerp(prev_vv, vv[j], f)
            return (v0 - vv2) / (t2i - t1)
        prev_t, prev_vv, prev_d = t2, vv[j], d
    return None


def metrics(accel, dt, nfp, v0, window_s=WINDOW_S):
    """The four V6 measures of one record. `accel` is the whole record in m/s^2
    (sample k at t = (nfp + k) dt); filters run over the whole record, the
    measures over [0, window_s]. Returns a dict."""
    a60 = cfc_filter(accel, dt, CFC_SHAPE)
    a180 = cfc_filter(accel, dt, CFC_INT)
    k0 = -nfp
    n_win = int(round(window_s / dt)) + 1
    if k0 < 0 or k0 + n_win > len(accel):
        raise ScatterError("record does not cover [0, %g] s" % window_s)
    w60 = a60[k0:k0 + n_win]
    w180 = a180[k0:k0 + n_win]
    pk, pk_t = peak(w60, dt)
    return {"dv_mps": -delta_v(w180, dt), "peak_mps2": -pk, "peak_time_s": pk_t,
            "olc_mps2": olc(w180, dt, v0)}


def rel_diff(a, b):
    """|a - b| / mean(a, b); None if either is None."""
    if a is None or b is None:
        return None
    mean = 0.5 * (a + b)
    if mean == 0:
        raise ScatterError("mean of the two values is zero")
    return abs(a - b) / mean


def median(values):
    v = sorted(values)
    if not v:
        return None
    m = len(v) // 2
    return v[m] if len(v) % 2 else 0.5 * (v[m - 1] + v[m])


# ------------------------------------------------------------------ test I/O

def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest().upper()


def read_sources(root):
    """tstno -> zip sha256 from SOURCES.md ('tstno | bytes | sha256 | files')."""
    path = os.path.join(root, "SOURCES.md")
    out = {}
    with open(path, encoding="utf-8-sig") as f:
        for line in f:
            parts = [p.strip() for p in line.split("|")]
            if len(parts) == 4 and parts[0].isdigit() and len(parts[2]) == 64:
                out[int(parts[0])] = parts[2].upper()
    return out


def pick_channel(spec, tstno, senatt_codes):
    """The one accelerometer curve on vehicle 1, global X, G'S, as measured
    (DASTAT AM), whose SENATT is in `senatt_codes`; None if there is none."""
    found = []
    for curno, row in sorted(spec["instr"].items()):
        def f(name):
            return convert._field(row, convert.INSTR_FIELD, name)
        if (f("VEHNO") == "1" and f("SENTYP") == "AC" and f("AXIS") == "XG"
                and f("YUNITS") == "G'S" and f("DASTAT") == "AM" and f("SENATT") in senatt_codes):
            found.append(curno)
    if len(found) > 1:
        raise ScatterError("test %d: %d candidate channels %s" % (tstno, len(found), found))
    return found[0] if found else None


def load_test(root, tstno, senatt_codes, sources):
    """Read one test's zip: hashes, vehicle facts, the chosen channel in m/s^2."""
    zpath = os.path.join(root, str(tstno), "v%05d.zip" % tstno)
    with open(zpath, "rb") as f:
        zbytes = f.read()
    zsha = sha256_bytes(zbytes)
    if sources.get(tstno) != zsha:
        raise ScatterError("test %d: zip sha256 %s is not the SOURCES.md value %s"
                           % (tstno, zsha, sources.get(tstno)))
    with zipfile.ZipFile(zpath) as z:
        names = z.namelist()
        specs = [n for n in names if n.upper().endswith(".EV5")]
        if len(specs) != 1:
            raise ScatterError("test %d: %d EV5 files" % (tstno, len(specs)))
        ev5_bytes = z.read(specs[0])
        spec = convert.parse_ev5(ev5_bytes.decode("latin-1"))
        if spec["test_no"] != tstno:
            raise ScatterError("test %d: EV5 says test %d" % (tstno, spec["test_no"]))
        curve = pick_channel(spec, tstno, senatt_codes)
        if curve is None:
            return {"tstno": tstno, "zip_sha": zsha, "ev5_name": specs[0],
                    "ev5_sha": sha256_bytes(ev5_bytes), "spec": spec, "curve": None}
        chan_name = "%s.%03d" % (specs[0][:-4], curve)
        chan_bytes = z.read(chan_name)
    points = convert.parse_channel(chan_bytes.decode("latin-1"))
    row = spec["instr"][curve]
    f = lambda n: convert._field(row, convert.INSTR_FIELD, n)  # noqa: E731
    nfp, nlp, delt = (convert._int(f(n), n) for n in ("NFP", "NLP", "DELT"))
    if delt <= 0 or nfp > 0 or nlp <= 0 or len(points) != nlp - nfp + 1:
        raise ScatterError("test %d curve %d: NFP %d NLP %d DELT %d with %d points"
                           % (tstno, curve, nfp, nlp, delt, len(points)))
    if f("XUNITS") != "SEC":
        raise ScatterError("test %d curve %d: time units %r" % (tstno, curve, f("XUNITS")))
    factor = convert.AXIS_SIGN[f("AXIS")] * convert.UNIT_FACTOR[f("YUNITS")]
    accel = []
    for k, (t_file, value) in enumerate(points):
        t = (nfp + k) * delt * 1e-6
        if abs(t - t_file) > convert.TIME_TOL_S:
            raise ScatterError("test %d curve %d point %d: time %r, NFP/DELT give %.7f"
                               % (tstno, curve, k, t_file, t))
        accel.append(value * factor)
    veh = spec["vehicle"]
    vf = lambda n: convert._field(veh, convert.VEHICLE_FIELD, n)  # noqa: E731
    return {
        "tstno": tstno, "zip_sha": zsha, "ev5_name": specs[0], "ev5_sha": sha256_bytes(ev5_bytes),
        "chan_name": chan_name, "chan_sha": sha256_bytes(chan_bytes), "spec": spec,
        "curve": curve, "senatt": f("SENATT"), "inscom": f("INSCOM").strip(),
        "dt": delt * 1e-6, "delt_us": delt, "nfp": nfp, "accel": accel,
        "mass_kg": convert._int(vf("VEHTWT"), "VEHTWT"),
        "speed_kmh": convert._num(convert._field(spec["test"], convert.TEST_FIELD, "CLSSPD"), "CLSSPD"),
        "title": convert._field(spec["test"], convert.TEST_FIELD, "TITLE").strip(),
        "year": veh[3], "make": veh[1], "model": veh[2], "body": veh[5],
        "dim13": convert._int(veh[12], "VEHICLE field 13"),
        "dim14": convert._int(veh[13], "VEHICLE field 14"),
    }


def load_pair_tests(root, pair, sources):
    """Both tests of a pair on the same channel kind: right-rear seat
    crossmember if both have a usable one, else left-rear, else None."""
    for kind, codes in (("right rear seat crossmember X", RIGHT_REAR),
                        ("left rear seat crossmember X", LEFT_REAR)):
        tests = [load_test(root, t, codes, sources) for t in pair]
        if all(t["curve"] is not None for t in tests):
            for t in tests:
                t["kind"] = kind
            return tests
    return None


def pair_count_decision(a, b):
    """(counted, reason) for a pair, from VEHICLE-record facts only."""
    mass = rel_diff(a["mass_kg"], b["mass_kg"])
    d13 = rel_diff(a["dim13"], b["dim13"])
    d14 = rel_diff(a["dim14"], b["dim14"])
    facts = "mass %.1f%%, dim13 %.1f%%, dim14 %.1f%%, year %s/%s, make %s/%s, model %s/%s, body %s/%s" % (
        100 * mass, 100 * d13, 100 * d14, a["year"], b["year"], a["make"], b["make"],
        a["model"], b["model"], a["body"], b["body"])
    if a["year"] != b["year"]:
        return False, "model year differs; " + facts
    if mass > MASS_TOL:
        return False, "mass differs by more than %.0f%%; " % (100 * MASS_TOL) + facts
    if d13 > DIM_TOL or d14 > DIM_TOL:
        return False, "dimensions differ by more than %.0f%%; " % (100 * DIM_TOL) + facts
    return True, facts


# ----------------------------------------------------------------- reporting

def fmt(x, spec):
    return "n/a" if x is None else format(x, spec)


def run(root, out):
    """Print the report to `out`; return 0, or raise ScatterError."""
    sources = read_sources(root)
    p = lambda s: print(s, file=out)  # noqa: E731
    p("V6 scatter: window %g s, CFC %g (peak, time of peak), CFC %g (delta-V, OLC)"
      % (WINDOW_S, CFC_SHAPE, CFC_INT))
    p("count rule: same year, mass within %.0f%%, VEHICLE fields 13 and 14 within %.0f%%"
      % (100 * MASS_TOL, 100 * DIM_TOL))
    pair_rows = []
    n_tests = 0
    for number, pair in enumerate(PAIRS, 1):
        tests = load_pair_tests(root, pair, sources)
        if tests is None:
            raise ScatterError("pair %d %s: no rear-seat crossmember X channel in both tests"
                               % (number, pair))
        res = []
        for t in tests:
            v0 = t["speed_kmh"] / 3.6
            m = metrics(t["accel"], t["dt"], t["nfp"], v0)
            res.append(m)
            n_tests += 1
            p("test %d | %s | year %s make %s model %s body %s | mass %d kg | v0 %.2f km/h | "
              "channel curve %d %s (%s) | %s | dt %d us | dv %.3f km/h | peak %.3f g | "
              "peak_time %.1f ms | olc %s g"
              % (t["tstno"], t["title"], t["year"], t["make"], t["model"], t["body"], t["mass_kg"],
                 t["speed_kmh"], t["curve"], t["senatt"], t["inscom"], t["kind"], t["delt_us"],
                 m["dv_mps"] * 3.6, m["peak_mps2"] / G, m["peak_time_s"] * 1e3,
                 fmt(None if m["olc_mps2"] is None else m["olc_mps2"] / G, ".3f")))
        for t in tests:
            p("files %d | zip sha256 %s (= SOURCES.md) | %s sha256 %s | %s sha256 %s"
              % (t["tstno"], t["zip_sha"], t["ev5_name"], t["ev5_sha"], t["chan_name"], t["chan_sha"]))
        counted, why = pair_count_decision(*tests)
        dv = rel_diff(res[0]["dv_mps"], res[1]["dv_mps"])
        pk = rel_diff(res[0]["peak_mps2"], res[1]["peak_mps2"])
        tp = abs(res[0]["peak_time_s"] - res[1]["peak_time_s"])
        ol = rel_diff(res[0]["olc_mps2"], res[1]["olc_mps2"])
        same_body = tests[0]["body"] == tests[1]["body"]
        pair_rows.append({"n": number, "counted": counted, "same_body": same_body,
                          "dv": dv, "peak": pk, "tpeak_ms": tp * 1e3, "olc": ol})
        p("pair %d %d/%d | counted %s | %s | dv_rel %.4f | peak_rel %.4f | peak_time_diff %.1f ms | "
          "olc_rel %s" % (number, pair[0], pair[1], "yes" if counted else "no", why, dv, pk,
                          tp * 1e3, fmt(ol, ".4f")))
    p("tests read: %d" % n_tests)

    def summary(label, rows):
        for name, key in (("dv_rel", "dv"), ("peak_rel", "peak"), ("peak_time_diff_ms", "tpeak_ms"),
                          ("olc_rel", "olc")):
            vals = [r[key] for r in rows if r[key] is not None]
            if not vals:
                raise ScatterError("%s: no values for %s" % (label, name))
            p("%s | %s | n %d | median %.4f | max %.4f" % (label, name, len(vals), median(vals), max(vals)))

    counted = [r for r in pair_rows if r["counted"]]
    p("counted pairs: %s" % ",".join(str(r["n"]) for r in counted))
    summary("counted", counted)
    same = [r for r in counted if r["same_body"]]
    p("counted pairs with identical body code: %s" % ",".join(str(r["n"]) for r in same))
    summary("counted_same_body_code", same)
    return 0


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print("usage: v6_scatter.py <folder holding <tstno>/v0<tstno>.zip and SOURCES.md>", file=sys.stderr)
        return 2
    try:
        return run(argv[0], sys.stdout)
    except (ScatterError, convert.ConvertError, OSError, zipfile.BadZipFile) as e:
        print("v6_scatter: %s" % e, file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
