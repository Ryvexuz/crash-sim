use super::*;

// ---------------------------------------------------------------------------------------
// The pulse report (docs/STAGE6-M4-M5.md, "The pulse report")

/// Most substeps a pulse report run may take: the run keeps several f64 per substep, so a longer
/// run is refused (exit 2) instead of exhausting memory. 5 million is 500 s at h = 0.1 ms.
const PULSE_MAX_SUBSTEPS: u64 = 5_000_000;

/// The solver of a pulse report (`--solver`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PulseSolver {
    /// `node-beam` or no `--solver`: `SymplecticEuler` through `Sim::from_car`, as before.
    NodeBeam,
    /// `m5`: `solver_implicit::ImplicitEuler` with `M5_TOLERANCES` (docs/M5-SOLVER.md option (1)).
    M5,
}

/// The checked flags of a pulse report. Every number is finite; see `pulse_params`.
pub(super) struct PulseParams {
    ref_path: String,
    csv: Option<String>,
    timing: bool,
    solver: PulseSolver,
    sensor: String,
    crush: (String, String),
    cfc_shape: f64,
    cfc_int: f64,
    window: f64,
    threshold: f64,
    tail: f64,
    mass_tol: f64,
    speed_tol: f64,
    margin_dv: f64,
    margin_peak: f64,
    margin_peak_time: f64,
    margin_duration: f64,
    margin_olc: f64,
    margin_static: f64,
}

/// Check the pulse flags: every required one is present, every number finite, and in its range.
/// The error text is for a usage error (exit 2).
pub(super) fn pulse_params(f: &PulseFlags, ref_path: String) -> Result<PulseParams, String> {
    if !ref_path.ends_with(".pulse.csv") {
        return Err(format!(
            "--pulse-report {ref_path}: the reference must be a path ending in .pulse.csv (its .meta is read beside it)"
        ));
    }
    // The complete rule, exact and case-sensitive: absent or `node-beam`, or `m5`.
    let solver = match f.solver.as_deref() {
        None | Some("node-beam") => PulseSolver::NodeBeam,
        Some("m5") => PulseSolver::M5,
        Some(s) => {
            return Err(format!(
                "--solver {s}: accepted values are node-beam (default) and m5"
            ))
        }
    };
    fn need<T: Clone>(v: &Option<T>, flag: &str) -> Result<T, String> {
        v.clone()
            .ok_or_else(|| format!("--pulse-report needs {flag}"))
    }
    // (value, flag, lowest allowed, whether the lowest itself is allowed)
    fn number(v: &Option<f64>, flag: &str, min: f64, min_ok: bool) -> Result<f64, String> {
        let x = need(v, flag)?;
        if x.is_finite() && (x > min || (min_ok && x == min)) {
            Ok(x)
        } else {
            let rel = if min_ok { ">=" } else { ">" };
            Err(format!("{flag} must be finite and {rel} {min}, got {x}"))
        }
    }
    Ok(PulseParams {
        ref_path,
        csv: f.csv.clone(),
        timing: f.timing,
        solver,
        sensor: need(&f.sensor, "--pulse-sensor")?,
        crush: need(&f.crush, "--pulse-crush-groups")?,
        cfc_shape: number(&f.cfc_shape, "--pulse-cfc-shape", 0.0, false)?,
        cfc_int: number(&f.cfc_int, "--pulse-cfc-int", 0.0, false)?,
        window: number(&f.window, "--pulse-window", 0.0, false)?,
        threshold: number(&f.threshold, "--pulse-threshold", 0.0, true)?,
        tail: number(&f.tail, "--pulse-crush-tail", 0.0, false)?,
        mass_tol: number(&f.mass_tol, "--setup-mass-tol", 0.0, true)?,
        speed_tol: number(&f.speed_tol, "--setup-speed-tol", 0.0, true)?,
        margin_dv: number(&f.margin_dv, "--margin-dv", 0.0, false)?,
        margin_peak: number(&f.margin_peak, "--margin-peak", 0.0, false)?,
        margin_peak_time: number(&f.margin_peak_time, "--margin-peak-time", 0.0, false)?,
        margin_duration: number(&f.margin_duration, "--margin-duration", 0.0, false)?,
        margin_olc: number(&f.margin_olc, "--margin-olc", 0.0, false)?,
        margin_static: number(&f.margin_static, "--margin-static-crush", 0.0, false)?,
    })
}

/// What a pulse report produced.
struct PulseOutcome {
    /// Everything for stdout, ending with the `pulse:` line.
    text: String,
    /// The `--pulse-csv` file's text, when the car touched the wall.
    csv: Option<String>,
    /// `pulse: ok`.
    ok: bool,
    /// `(sim_s, wall_s)` of the solver loop, when it ran to the end.
    timing: Option<(f64, f64)>,
}

/// Run the pulse report; print it; return the exit code: 0 ok, 1 a FAIL or an abort, 2 a refusal
/// (usage, reference, setup), in which case nothing is printed on stdout.
pub(super) fn run_pulse(cfg: &Config, body: &Body, p: &PulseParams) -> i32 {
    let out = match pulse_report(cfg, body, p) {
        Ok(o) => o,
        Err(e) => {
            eprintln!("crash-sim: {e}");
            return 2;
        }
    };
    if let (Some(path), Some(csv)) = (&p.csv, &out.csv) {
        if let Err(e) = std::fs::write(path, csv) {
            eprintln!("crash-sim: --pulse-csv {path}: {e}");
            return 1;
        }
    }
    if let (true, Some((sim_s, wall_s))) = (p.timing, out.timing) {
        eprintln!(
            "timing: sim_s={sim_s:.4} wall_s={wall_s:.4} rtf={:.4}",
            wall_s / sim_s
        );
    }
    emit(&out.text);
    i32::from(!out.ok)
}

/// The sensor group's mass-weighted mean x-velocity u (m/s).
fn group_u(sim: &crash_sim::Sim, ix: &[usize]) -> f64 {
    let (mut p, mut m) = (0.0, 0.0);
    for &i in ix {
        p += sim.mass[i] * sim.vel[i].x;
        m += sim.mass[i];
    }
    p / m
}

/// Distance between the centroids of two node groups (m), as the crumple report measures it.
fn group_len(sim: &crash_sim::Sim, a: &[usize], b: &[usize]) -> f64 {
    let centroid = |ix: &[usize]| {
        let mut s = crash_sim::vec3::Vec3::ZERO;
        for &i in ix {
            s += sim.pos[i];
        }
        s * (1.0 / ix.len() as f64)
    };
    (centroid(b) - centroid(a)).length()
}

/// Number of points of the crush profile: NHTSA's C1 to C6 (docs/STAGE6-M4-M5.md V8).
const PROFILE_POINTS: usize = 6;
/// Two front nodes whose start z differs by at most this (m) are one column of the profile.
const PROFILE_COLUMN_TOL_M: f64 = 1e-9;

/// Weights of the six-point crush profile (docs/STAGE6-M4-M5.md, "Metrics", `static_crush_m`),
/// one per node of the front group, from the nodes' start z (the lateral axis; x is the crash
/// axis, y is up). The six points are equally spaced across the front, from its leftmost to its
/// rightmost node column, both ends included, as C1 (left side) to C6 (right side) are across the
/// damage width. Nodes whose z agree within `PROFILE_COLUMN_TOL_M` are one column, its value the
/// plain mean of its nodes; a point between two columns takes the linear interpolation of their
/// values. The profile is the mean of the six points, so it is a fixed weighted mean of the nodes:
/// each column gets the sum of its interpolation shares over the six points, divided by six,
/// shared equally by its nodes. The weights sum to 1. One column takes the plain mean.
fn profile_weights(z: &[f64]) -> Vec<f64> {
    let mut cols: Vec<f64> = Vec::new();
    for &v in z {
        if !cols.iter().any(|c| (c - v).abs() <= PROFILE_COLUMN_TOL_M) {
            cols.push(v);
        }
    }
    cols.sort_by(f64::total_cmp);
    let col_of = |v: f64| {
        cols.iter()
            .position(|c| (c - v).abs() <= PROFILE_COLUMN_TOL_M)
            .expect("every node is in a column")
    };
    let mut col_w = vec![0.0; cols.len()];
    if cols.len() == 1 {
        col_w[0] = 1.0;
    } else {
        let (lo, hi) = (cols[0], cols[cols.len() - 1]);
        let share = 1.0 / PROFILE_POINTS as f64;
        for i in 0..PROFILE_POINTS {
            let zi = lo + (hi - lo) * i as f64 / (PROFILE_POINTS - 1) as f64;
            // The bracketing columns j, j + 1 (the last pair for the right end).
            let j = (0..cols.len() - 1)
                .find(|&j| zi <= cols[j + 1])
                .unwrap_or(cols.len() - 2);
            let t = ((zi - cols[j]) / (cols[j + 1] - cols[j])).clamp(0.0, 1.0);
            col_w[j] += share * (1.0 - t);
            col_w[j + 1] += share * t;
        }
    }
    let mut count = vec![0usize; cols.len()];
    for &v in z {
        count[col_of(v)] += 1;
    }
    z.iter()
        .map(|&v| {
            let c = col_of(v);
            col_w[c] / count[c] as f64
        })
        .collect()
}

/// The six-point profile depth (m): the weighted mean (`profile_weights`) over the front group
/// `b` of each node's x ahead of the plain mean x of the reference group `a`. Static crush is its
/// start value minus its mean over the tail. Known limit: it measures along the world x axis, so a
/// body that turns about y reads its rotation as crush; the validation runs hit a wall square to x
/// with a car symmetric in z.
fn profile_depth(sim: &crash_sim::Sim, a: &[usize], b: &[usize], w: &[f64]) -> f64 {
    let xa = a.iter().map(|&i| sim.pos[i].x).sum::<f64>() / a.len() as f64;
    b.iter()
        .zip(w)
        .map(|(&i, wi)| wi * (sim.pos[i].x - xa))
        .sum()
}

/// True if any node is on or behind one of `planes[range]`, or any wheel (a tyre) is within its
/// radius of one: the sim's contact test (`any_at_or_behind`) extended to the tyres.
fn touches(sim: &crash_sim::Sim, range: std::ops::Range<usize>) -> bool {
    let planes = &sim.planes[range];
    if collide::any_at_or_behind(&sim.pos, planes) {
        return true;
    }
    sim.car.as_ref().is_some_and(|c| {
        c.wheels
            .iter()
            .any(|w| planes.iter().any(|pl| pl.distance(w.pos) <= w.p.radius))
    })
}

/// The metrics of one filtered pulse window (docs: "Metrics"), as positive magnitudes.
struct PulseMetrics {
    dv: f64,
    peak: f64,
    peak_time: f64,
    /// None when the pulse never reaches the threshold.
    duration: Option<f64>,
    /// None when the OLC occupant never completes its travel.
    olc: Option<f64>,
    dyn_crush: f64,
}

/// Metrics of `f60` (CFC shape) and `f180` (CFC int) windows sampled every `h` s from contact,
/// with impact speed `v0` (m/s). The duration counts samples at or below -threshold, so the
/// samples are clamped at 0 from above first (`pulse::duration` itself counts |sample|).
fn pulse_metrics(
    f60: &[f64],
    f180: &[f64],
    h: f64,
    v0: f64,
    threshold: f64,
) -> Result<PulseMetrics, String> {
    let e = |e: pulse::PulseError| e.to_string();
    let dv = -pulse::delta_v(f180, h).map_err(e)?;
    let (pk, pk_t) = pulse::peak(f60, h).map_err(e)?;
    let below: Vec<f64> = f60.iter().map(|v| v.min(0.0)).collect();
    let duration = pulse::duration(&below, h, threshold).map_err(e)?;
    let olc = pulse::olc(f180, h, v0).map_err(e)?;
    // Displacement toward the wall: x(t) = integral of (v0 + integral of a), trapezoid.
    let (mut v, mut x, mut dyn_crush) = (v0, 0.0_f64, 0.0_f64);
    for w in f180.windows(2) {
        let vn = v + 0.5 * (w[0] + w[1]) * h;
        x += 0.5 * (v + vn) * h;
        v = vn;
        dyn_crush = dyn_crush.max(x);
    }
    Ok(PulseMetrics {
        dv,
        peak: pk.abs(),
        peak_time: pk_t,
        duration,
        olc,
        dyn_crush,
    })
}

/// The report. `Err` is a refusal (exit 2): a file the sim or reference cannot use, a rate under
/// 10 x CFC, a window longer than either pulse. A FAIL or an abort is an `Ok` outcome with
/// `ok == false` (exit 1).
///
/// Choices the docs leave open, fixed here:
/// - Substep k (1-based) gives the sample a_k = (u_k - u_(k-1)) / h at t = k h; contact is the
///   first substep k whose end has a node or tyre on or behind a body plane (the ground, the
///   first plane, is not a wall), and sample j of the window, t = j h from contact, is a_(k + j).
/// - The impact speed (setup line and OLC / dynamic crush start) is u just before the contact
///   substep; with no contact it is the start speed, and only `no contact` follows.
/// - `ground_contacts_in_window` counts substeps in the window with a node or tyre on or behind
///   the ground; the FAIL line's `t` is the first of them, in seconds from contact.
/// - Every line that can be computed is printed even when an earlier rule fails; the last line
///   names the first rule that failed. A sim OLC or duration that does not exist counts as 0.
/// - Static crush (the six-point profile depth, `profile_weights`, and the old centroid distance
///   L) uses the series' value at the start of the run, and the substeps with
///   t >= run end - tail (to 1e-6 of h).
fn pulse_report(cfg: &Config, body: &Body, p: &PulseParams) -> Result<PulseOutcome, String> {
    // The reference: the pulse and its meta, both strict (reference.rs).
    let meta_path = format!(
        "{}.meta",
        &p.ref_path[..p.ref_path.len() - ".pulse.csv".len()]
    );
    let refp = reference::load_pulse(&p.ref_path).map_err(|e| format!("reference: {e}"))?;
    let meta = reference::load_meta(&meta_path).map_err(|e| format!("reference meta: {e}"))?;

    // The sim's step, and the J211 rate rule for both pulses (n20-03's refusal).
    let h = cfg.dt / cfg.substeps as f64;
    for cfc in [p.cfc_shape, p.cfc_int] {
        pulse::cfc_filter(&[0.0, 0.0], h, cfc).map_err(|e| format!("sim pulse: {e}"))?;
        pulse::cfc_filter(&[0.0, 0.0], refp.dt, cfc).map_err(|e| format!("reference: {e}"))?;
    }
    // A huge --duration must be a refusal, not an overflow: the product saturates.
    let n_sub = crash_sim::step_count(cfg).saturating_mul(u64::from(cfg.substeps));
    if n_sub == 0 {
        return Err("the run has no substep: raise --duration".into());
    }
    if n_sub > PULSE_MAX_SUBSTEPS {
        return Err(format!(
            "the run has {n_sub} substeps, over the limit of {PULSE_MAX_SUBSTEPS}: lower --duration or raise --dt"
        ));
    }
    if p.tail > cfg.duration {
        return Err(format!(
            "--pulse-crush-tail {} s is longer than the run ({} s)",
            p.tail, cfg.duration
        ));
    }

    // Groups.
    let group = |name: &str| -> Result<Vec<usize>, String> {
        let g = body
            .groups
            .iter()
            .find(|g| g.name == name)
            .ok_or_else(|| format!("the body has no group `{name}`"))?;
        Ok(g.nodes
            .iter()
            .map(|id| body.node_index(*id).expect("group names declared nodes"))
            .collect())
    };
    let sensor = group(&p.sensor)?;
    let crush_a = group(&p.crush.0)?;
    let crush_b = group(&p.crush.1)?;

    // The sim, built with the real config (so every refusal is the usual one), then stepped one
    // substep at a time: with dt = h and one substep, `advance` is exactly one substep of the
    // same length h (`cfg` is public).
    let mut sim = match p.solver {
        PulseSolver::NodeBeam => crash_sim::Sim::from_car(cfg.clone(), body)?,
        PulseSolver::M5 => {
            let solver = ImplicitEuler::new(M5_TOLERANCES)?;
            crash_sim::Sim::from_car_with_solver(cfg.clone(), body, Box::new(solver))?
        }
    };
    sim.cfg.dt = h;
    sim.cfg.substeps = 1;
    let wheel_mass: f64 = sim
        .car
        .as_ref()
        .map_or(0.0, |c| c.wheels.iter().map(|w| w.p.mass).sum());
    let sim_mass = sim.mass.iter().sum::<f64>() + wheel_mass;

    let n = n_sub as usize;
    let mut u = Vec::with_capacity(n + 1);
    let mut len = Vec::with_capacity(n + 1);
    let mut depth = Vec::with_capacity(n + 1);
    let mut ground = Vec::with_capacity(n + 1);
    let z0: Vec<f64> = crush_b.iter().map(|&i| sim.pos[i].z).collect();
    let weights = profile_weights(&z0);
    u.push(group_u(&sim, &sensor));
    len.push(group_len(&sim, &crush_a, &crush_b));
    depth.push(profile_depth(&sim, &crush_a, &crush_b, &weights));
    ground.push(false);
    let mut contact: Option<usize> = None;
    let started = std::time::Instant::now();
    for k in 1..=n {
        sim.advance();
        if let Err(e) = sim.check() {
            return Ok(PulseOutcome {
                text: format!("pulse: FAIL unstable at t = {:.4} s: {e}\n", k as f64 * h),
                csv: None,
                ok: false,
                timing: None,
            });
        }
        u.push(group_u(&sim, &sensor));
        len.push(group_len(&sim, &crush_a, &crush_b));
        depth.push(profile_depth(&sim, &crush_a, &crush_b, &weights));
        ground.push(touches(&sim, 0..1));
        if contact.is_none() && touches(&sim, 1..sim.planes.len()) {
            contact = Some(k);
        }
    }
    let wall_s = started.elapsed().as_secs_f64();
    let timing = Some((n as f64 * h, wall_s));

    // The sim pulse: a_k = (u_k - u_(k-1)) / h at t = k h, k = 1..=n.
    let accel: Vec<f64> = u.windows(2).map(|w| (w[1] - w[0]) / h).collect();

    // Header and setup lines (rule 1).
    let mut text = String::new();
    text.push_str(&format!(
        "pulse_ref={}\npulse_channel={}\n",
        meta.test, meta.channel
    ));
    // The impact speed is the sensor group's u just before the contact substep (u_0 if there is
    // no contact, where only the no-contact FAIL follows).
    let v0 = u[contact.map_or(0, |k| k - 1)];
    let rel = |sim: f64, r: f64| (sim - r) / r;
    let mass_err = rel(sim_mass, meta.mass_kg);
    let speed_kmh = v0 * 3.6;
    let speed_err = rel(speed_kmh, meta.speed_kmh);
    let mass_ok = mass_err.abs() <= p.mass_tol;
    let speed_ok = speed_err.abs() <= p.speed_tol;
    let tag = |ok: bool| if ok { "ok" } else { "FAIL" };
    text.push_str(&format!(
        "setup mass_kg sim={:.1} ref={:.1} err={:+.4} tol={:.4} {}\n",
        sim_mass,
        meta.mass_kg,
        mass_err,
        p.mass_tol,
        tag(mass_ok)
    ));
    text.push_str(&format!(
        "setup speed_kmh sim={:.2} ref={:.2} err={:+.4} tol={:.4} {}\n",
        speed_kmh,
        meta.speed_kmh,
        speed_err,
        p.speed_tol,
        tag(speed_ok)
    ));
    let Some(kc) = contact else {
        text.push_str("contact_s=none\npulse: FAIL no contact\n");
        return Ok(PulseOutcome {
            text,
            csv: None,
            ok: false,
            timing,
        });
    };
    text.push_str(&format!("contact_s={:.4}\n", kc as f64 * h));

    // The window: samples j = 0..n_win at t = j h from contact, sim sample j being a_(kc + j).
    // The step count is compared as a float first, so a huge window cannot overflow the count.
    let window_steps = (p.window / h * (1.0 + 1e-9)).floor();
    let n_win = if window_steps <= (n - kc) as f64 {
        window_steps as usize + 1
    } else {
        usize::MAX
    };
    if n_win == usize::MAX || kc + n_win - 1 > n {
        return Err(format!(
            "--pulse-window {} s is longer than the sim pulse after contact (contact at {:.4} s, run {} s)",
            p.window,
            kc as f64 * h,
            cfg.duration
        ));
    }
    let ground_window = &ground[kc..kc + n_win];
    let ground_count = ground_window.iter().filter(|&&g| g).count();
    text.push_str(&format!("ground_contacts_in_window={ground_count}\n"));
    let ground_first = ground_window.iter().position(|&g| g);

    // Filters: each pulse over its whole record at its own rate, then the window.
    let sim_fail = |mut text: String, what: &str, e: String| {
        text.push_str(&format!("pulse: FAIL {what}: {e}\n"));
        PulseOutcome {
            text,
            csv: None,
            ok: false,
            timing,
        }
    };
    let f60 = match pulse::cfc_filter(&accel, h, p.cfc_shape) {
        Ok(v) => v,
        Err(e) => return Ok(sim_fail(text, "sim pulse", e.to_string())),
    };
    let f180 = match pulse::cfc_filter(&accel, h, p.cfc_int) {
        Ok(v) => v,
        Err(e) => return Ok(sim_fail(text, "sim pulse", e.to_string())),
    };
    let win = |v: &[f64]| v[kc - 1..kc - 1 + n_win].to_vec();
    let (s60, s180) = (win(&f60), win(&f180));
    let ref_window = |cfc: f64| -> Result<Vec<f64>, String> {
        let filtered =
            pulse::cfc_filter(&refp.accel, refp.dt, cfc).map_err(|e| format!("reference: {e}"))?;
        let rec = reference::Pulse {
            t0: refp.t0,
            dt: refp.dt,
            accel: filtered,
        };
        let r = reference::resample_at(&rec, 0.0, h, n_win).map_err(|e| {
            format!(
                "--pulse-window {} s does not fit the reference pulse: {e}",
                p.window
            )
        })?;
        Ok(r.accel)
    };
    let (r60, r180) = (ref_window(p.cfc_shape)?, ref_window(p.cfc_int)?);

    let ref_v0 = meta.speed_kmh / 3.6;
    let sm = match pulse_metrics(&s60, &s180, h, v0, p.threshold) {
        Ok(m) => m,
        Err(e) => return Ok(sim_fail(text, "sim metrics", e)),
    };
    let rm = pulse_metrics(&r60, &r180, h, ref_v0, p.threshold)
        .map_err(|e| format!("reference metrics: {e}"))?;
    let ref_dur = rm
        .duration
        .ok_or("the reference pulse never reaches --pulse-threshold")?;
    let ref_olc = rm
        .olc
        .ok_or("the reference pulse has no OLC (the occupant never completes its travel)")?;
    // A missing sim duration or OLC counts as 0 (a norm of 1 / margin, a FAIL).
    let (sim_dur, sim_olc) = (sm.duration.unwrap_or(0.0), sm.olc.unwrap_or(0.0));

    // Static crush: a series' start value minus its mean over the last `tail` seconds of the
    // run. The gated value is the six-point profile depth (the reference's mean of C1 to C6,
    // rule change of 2026-11-22); the old centroid distance L is printed as
    // `info static_crush_centroid_m` for comparison and not gated.
    let t_end = n as f64 * h;
    let tol = h * 1e-6;
    let settled = |series: &[f64]| {
        let tail: Vec<f64> = series
            .iter()
            .enumerate()
            .filter(|(k, _)| *k as f64 * h >= t_end - p.tail - tol)
            .map(|(_, l)| *l)
            .collect();
        series[0] - tail.iter().sum::<f64>() / tail.len() as f64
    };
    let sim_static = settled(&depth);
    let sim_static_centroid = settled(&len);

    // Gated metrics, in the printed order: (name, decimals, sim, ref, margin, absolute error?).
    struct Row {
        name: &'static str,
        dec: usize,
        sim: f64,
        refv: f64,
        margin: f64,
        absolute: bool,
    }
    let mut rows = vec![
        Row {
            name: "dv_mps",
            dec: 4,
            sim: sm.dv,
            refv: rm.dv,
            margin: p.margin_dv,
            absolute: false,
        },
        Row {
            name: "peak_mps2",
            dec: 1,
            sim: sm.peak,
            refv: rm.peak,
            margin: p.margin_peak,
            absolute: false,
        },
        Row {
            name: "peak_time_s",
            dec: 4,
            sim: sm.peak_time,
            refv: rm.peak_time,
            margin: p.margin_peak_time,
            absolute: true,
        },
        Row {
            name: "duration_s",
            dec: 4,
            sim: sim_dur,
            refv: ref_dur,
            margin: p.margin_duration,
            absolute: false,
        },
        Row {
            name: "olc_mps2",
            dec: 1,
            sim: sim_olc,
            refv: ref_olc,
            margin: p.margin_olc,
            absolute: false,
        },
    ];
    if let Some(s) = meta.static_crush_m {
        rows.push(Row {
            name: "static_crush_m",
            dec: 4,
            sim: sim_static,
            refv: s,
            margin: p.margin_static,
            absolute: false,
        });
    }
    let mut norms = Vec::new();
    let mut first_bad: Option<String> = None;
    for r in &rows {
        if !(r.refv.is_finite() && (r.absolute || r.refv > 0.0)) {
            return Err(format!(
                "the reference's {} is {}: a relative error needs a finite value above zero",
                r.name, r.refv
            ));
        }
        let err = if r.absolute {
            r.sim - r.refv
        } else {
            (r.sim - r.refv) / r.refv
        };
        let norm = err.abs() / r.margin;
        // A non-finite sim value is a FAIL, never an ok.
        let ok = r.sim.is_finite() && norm <= 1.0;
        if !ok && first_bad.is_none() {
            first_bad = Some(if r.sim.is_finite() {
                format!("{} norm={norm:.4}", r.name)
            } else {
                format!("{} is not finite", r.name)
            });
        }
        norms.push(norm);
        text.push_str(&format!(
            "metric {} sim={:.d$} ref={:.d$} err={:+.4} norm={:.4} {}\n",
            r.name,
            r.sim,
            r.refv,
            err,
            norm,
            tag(ok),
            d = r.dec
        ));
    }
    if meta.static_crush_m.is_none() {
        text.push_str(&format!(
            "metric static_crush_m sim={sim_static:.4} ref=none\n"
        ));
    }
    text.push_str(&format!(
        "info static_crush_centroid_m sim={sim_static_centroid:.4}\n"
    ));
    text.push_str(&format!(
        "info dyn_crush_m sim={:.4} ref={:.4}\n",
        sm.dyn_crush, rm.dyn_crush
    ));
    let score = norms.iter().sum::<f64>() / norms.len() as f64;
    text.push_str(&format!("pulse_score={score:.4}\n"));

    // The verdict: the first rule that fails, in the order of the docs.
    let verdict = if !mass_ok {
        Some(format!(
            "setup mass_kg err={mass_err:+.4} tol={:.4}",
            p.mass_tol
        ))
    } else if !speed_ok {
        Some(format!(
            "setup speed_kmh err={speed_err:+.4} tol={:.4}",
            p.speed_tol
        ))
    } else if let Some(j) = ground_first {
        Some(format!("ground contact at t={:.4}", j as f64 * h))
    } else {
        first_bad
    };
    let ok = verdict.is_none();
    match verdict {
        None => text.push_str("pulse: ok\n"),
        Some(v) => text.push_str(&format!("pulse: FAIL {v}\n")),
    }

    // The CSV: the sim pulse from contact to the end of the run.
    let mut csv = String::from("t_s,accel_mps2,accel_cfc60_mps2,accel_cfc180_mps2\n");
    for j in kc - 1..accel.len() {
        csv.push_str(&format!(
            "{:.6},{:.4},{:.4},{:.4}\n",
            (j + 1 - kc) as f64 * h,
            accel[j],
            f60[j],
            f180[j]
        ));
    }
    Ok(PulseOutcome {
        text,
        csv: Some(csv),
        ok,
        timing,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(got: &[f64], want: &[f64]) {
        assert_eq!(got.len(), want.len(), "{got:?} against {want:?}");
        for (g, w) in got.iter().zip(want) {
            assert!((g - w).abs() < 1e-12, "{got:?} against {want:?}");
        }
    }

    /// The validation car's front: columns at z = -0.844, -0.75425, 0.75425, 0.844, two nodes
    /// each. Points 1 and 6 sit on the outer columns; points 2 to 5 fall between the inner two at
    /// t = 0.1643, 0.3881, 0.6119, 0.8357, whose shares sum to 2 for each inner column. So the
    /// column weights are 1/6, 2/6, 2/6, 1/6, halved for the two nodes of each column.
    #[test]
    fn profile_weights_of_the_validation_front() {
        let z = [
            -0.844, -0.844, -0.75425, -0.75425, 0.75425, 0.75425, 0.844, 0.844,
        ];
        let (a, b) = (1.0 / 12.0, 2.0 / 12.0);
        close(&profile_weights(&z), &[a, a, b, b, b, b, a, a]);
    }

    /// Six evenly spaced columns: each point is one column, 1/6 each, in any node order.
    #[test]
    fn profile_weights_six_even_columns() {
        let z = [0.5, 0.0, 0.3, 0.1, 0.4, 0.2];
        close(&profile_weights(&z), &[1.0 / 6.0; 6]);
    }

    /// Two columns, one with two nodes: 1/2 per column (not 1/3 per node, as a centroid has).
    #[test]
    fn profile_weights_two_columns_uneven_counts() {
        let z = [0.0, 1.0, 0.0];
        close(&profile_weights(&z), &[0.25, 0.5, 0.25]);
    }

    /// One column: the plain mean.
    #[test]
    fn profile_weights_one_column() {
        close(&profile_weights(&[2.0, 2.0, 2.0, 2.0]), &[0.25; 4]);
    }
}
