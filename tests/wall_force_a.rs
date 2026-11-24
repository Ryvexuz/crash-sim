#[path = "../examples/wall_force_a.rs"]
mod wall_force_a;

use crash_sim::format::{self, Body};
use crash_sim::statehash::Hasher;
use crash_sim::vec3::Vec3;
use crash_sim::{pulse, step_count, Config, Sim};
use std::f64::consts::PI;
use std::io::{self, Write};
use std::sync::OnceLock;
use wall_force_a::{
    cfc_coefficients, force_vs_crush, measure_test_a, parse_run_flags, run, summarize,
    validate_body, WallError, WallForce,
};

fn measured() -> &'static WallForce {
    static MEASURED: OnceLock<WallForce> = OnceLock::new();
    MEASURED.get_or_init(|| measure_test_a().expect("wall measurement refused"))
}

struct Plain {
    cfg: Config,
    body: Body,
    hashes: Vec<u64>,
    tyre_ns: f64,
    p0: f64,
    p_end: f64,
    pos: Vec<Vec3>,
    vel: Vec<Vec3>,
}

// Independent field traversal: do not call the example's hash or momentum code.
fn hash(sim: &Sim) -> u64 {
    fn vector(h: &mut Hasher, v: Vec3) {
        h.write_f64(v.x).unwrap();
        h.write_f64(v.y).unwrap();
        h.write_f64(v.z).unwrap();
    }
    let mut h = Hasher::new();
    for (&pos, &vel) in sim.pos.iter().zip(&sim.vel) {
        vector(&mut h, pos);
        vector(&mut h, vel);
    }
    for beam in &sim.beams {
        h.write_f64(beam.rest).unwrap();
    }
    for wheel in &sim.car.as_ref().unwrap().wheels {
        vector(&mut h, wheel.pos);
        vector(&mut h, wheel.vel);
        h.write_f64(wheel.spin).unwrap();
        for &deflection in &wheel.deflection {
            vector(&mut h, deflection);
        }
        for &load in &wheel.load {
            h.write_f64(load).unwrap();
        }
    }
    h.finish()
}

fn px(sim: &Sim) -> f64 {
    let mut p = 0.0;
    for (&mass, vel) in sim.mass.iter().zip(&sim.vel) {
        p += mass * vel.x;
    }
    for w in &sim.car.as_ref().unwrap().wheels {
        p += w.p.mass * w.vel.x;
    }
    p
}

fn plain() -> &'static Plain {
    static PLAIN: OnceLock<Plain> = OnceLock::new();
    PLAIN.get_or_init(|| {
        let text = std::fs::read_to_string("docs/STAGE6-M4-M5.md").unwrap();
        let lines: Vec<_> = text
            .lines()
            .filter_map(|l| l.strip_prefix("frozen: run_flags = "))
            .collect();
        assert_eq!(lines.len(), 1);
        let words: Vec<_> = lines[0].split_whitespace().collect();
        let flag = |name: &str| words[words.iter().position(|w| *w == name).unwrap() + 1];
        let cfg = Config {
            gravity: flag("--gravity").parse().unwrap(),
            duration: flag("--duration").parse().unwrap(),
            dt: flag("--dt").parse().unwrap(),
            substeps: flag("--substeps").parse().unwrap(),
            ..Config::default()
        };
        let body_text = std::fs::read_to_string("scenarios/validation_wall.bsn").unwrap();
        let body = format::parse(&body_text).unwrap();
        let mut sim = Sim::from_car(cfg.clone(), &body).unwrap();
        let h = cfg.dt / cfg.substeps as f64;
        let n = (step_count(&cfg) * u64::from(cfg.substeps)) as usize;
        sim.cfg.dt = h;
        sim.cfg.substeps = 1;
        let p0 = px(&sim);
        let mut hashes = Vec::new();
        let mut tyre_ns = 0.0;
        for k in 1..=n {
            sim.advance();
            sim.check()
                .unwrap_or_else(|e| panic!("plain substep {k}: {e}"));
            hashes.push(hash(&sim));
            // Sum wheel loads in wheel order, then multiply by h, then add in
            // substep order: exactly the impulse definition for a sampled force.
            let mut force = 0.0;
            for w in &sim.car.as_ref().unwrap().wheels {
                force += w.load[1].max(0.0);
            }
            tyre_ns += force * h;
        }
        let p_end = px(&sim);
        Plain {
            cfg,
            body,
            hashes,
            tyre_ns,
            p0,
            p_end,
            pos: sim.pos,
            vel: sim.vel,
        }
    })
}

fn close(label: &str, actual: f64, expected: f64, tolerance: f64) {
    assert!(actual.is_finite() && expected.is_finite(), "{label}: finite");
    assert!(
        (actual - expected).abs() <= tolerance,
        "{label}: actual={actual:.17e} expected={expected:.17e} tolerance={tolerance:.17e}"
    );
}

fn bits(v: Vec3) -> [u64; 3] {
    [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]
}

#[test]
fn wall_force_a_matches_unobserved_bits() {
    let (w, p) = (measured(), plain());
    let n = (step_count(&p.cfg) * u64::from(p.cfg.substeps)) as usize;
    assert!(n > 0, "plain run must not be empty");
    assert_eq!(w.state_hashes.len(), n);
    assert_eq!(p.hashes.len(), n);
    for (i, (&observed, &unobserved)) in w.state_hashes.iter().zip(&p.hashes).enumerate() {
        assert_eq!(observed, unobserved, "state hash at substep {}", i + 1);
    }
    let mut frozen = Sim::from_car(p.cfg.clone(), &p.body).unwrap();
    for _ in 0..step_count(&p.cfg) {
        frozen.advance();
        frozen.check().unwrap();
    }
    assert_eq!(frozen.pos.len(), p.pos.len());
    for (i, &pos) in p.pos.iter().enumerate() {
        assert_eq!(bits(frozen.pos[i]), bits(pos), "final pos {i}");
        assert_eq!(bits(frozen.vel[i]), bits(p.vel[i]), "final vel {i}");
    }
}

#[test]
fn wall_force_a_includes_tyre_wall_load() {
    let (w, p) = (measured(), plain());
    let s = summarize(&w.raw_n, &w.cfc60_n, &w.node_n, &w.tyre_n, w.h);
    assert_eq!(s.tyre_ns.to_bits(), p.tyre_ns.to_bits(), "tyre impulse");
    assert!(s.tyre_ns > 0.0, "no tyre wall impulse: {}", s.tyre_ns);
    assert_eq!(w.raw_n.len(), w.node_n.len());
    assert_eq!(w.raw_n.len(), w.tyre_n.len());
    for (k, &raw) in w.raw_n.iter().enumerate() {
        assert_eq!(raw, w.node_n[k] + w.tyre_n[k], "force {k}");
        assert!(w.node_n[k] >= 0.0, "negative node force {k}");
        if k + 1 < w.contact_k {
            assert_eq!(w.node_n[k], 0.0);
            assert_eq!(w.tyre_n[k], 0.0);
            assert_eq!(raw, 0.0);
        }
    }
}

#[test]
fn wall_force_a_momentum_identity_closes() {
    let (w, p) = (measured(), plain());
    let m = &w.momentum;
    assert!(m.residual.abs() <= m.tolerance, "{m:?}");
    assert_eq!(m.tolerance, 1e-6 * m.p0_x.abs());
    assert_eq!(m.p0_x.to_bits(), p.p0.to_bits());
    assert_eq!(m.p_end_x.to_bits(), p.p_end.to_bits());
    assert!(m.aero_x < 0.0);
    assert_eq!(w.ground_tyre_substeps, 0);
    assert_eq!(
        m.residual,
        (p.p_end - p.p0) - (m.wall_x + m.ground_node_x + m.aero_x)
    );
    // A missing external term must not be hidden inside the allowed residual.
    let without_aero = (p.p_end - p.p0) - (m.wall_x + m.ground_node_x);
    assert!(without_aero.abs() > m.tolerance, "aero omission not detected");
}

#[test]
fn wall_force_a_cfc60_matches_j211() {
    let expected = [
        1.452367473553412e-3,
        2.904734947106825e-3,
        1.452367473553412e-3,
        1.889343224221574,
        -8.951526941157872e-1,
    ];
    let coeff = cfc_coefficients(60.0, 1e-4);
    for (&actual, reference) in coeff.iter().zip(expected) {
        close("J211 coefficient", actual, reference, 1e-15 * reference.abs());
    }
    close("unit DC gain", coeff.iter().sum(), 1.0, 1e-15);
    let w = measured();
    let filtered = pulse::cfc_filter(&w.raw_n, w.h, 60.0).unwrap();
    assert_eq!(filtered.len(), w.cfc60_n.len());
    for (a, b) in filtered.iter().zip(&w.cfc60_n) {
        assert_eq!(a.to_bits(), b.to_bits());
    }
    let constant = pulse::cfc_filter(&[1000.0; 2000], 1e-4, 60.0).unwrap();
    for v in constant {
        close("constant", v, 1000.0, 1e-9);
    }
    let sine: Vec<f64> = (0..2000)
        .map(|k| 1000.0 * (2.0 * PI * 1000.0 * k as f64 * 1e-4).sin())
        .collect();
    let filtered = pulse::cfc_filter(&sine, 1e-4, 60.0).unwrap();
    let amplitude = filtered[500..1500]
        .iter()
        .map(|v| v.abs())
        .fold(0.0, f64::max);
    assert!(amplitude < 10.0, "1 kHz amplitude={amplitude}");
    assert!(sine[500..1500].iter().any(|v| v.abs() >= 10.0));
    for (cfc, dt) in [
        (0.0, 1e-4),
        (60.0, 0.0),
        (f64::NAN, 1e-4),
        (60.0, f64::INFINITY),
        (f64::from_bits(1), 1e-4),
        (f64::MAX, 1e-4),
    ] {
        assert!(cfc_coefficients(cfc, dt).iter().all(|v| v.is_nan()));
    }
}

#[test]
fn wall_force_a_peak_time_and_impulse_on_a_fixture() {
    let h = 1e-4;
    let mut raw = vec![0.0; 100];
    raw.extend((0..=400).map(|j| 1e5 * (PI * j as f64 / 400.0).sin()));
    raw.extend([0.0; 100]);
    let node: Vec<_> = raw.iter().map(|f| 0.75 * f).collect();
    let tyre: Vec<_> = raw.iter().map(|f| 0.25 * f).collect();
    let filtered = pulse::cfc_filter(&raw, h, 60.0).unwrap();
    let s = summarize(&raw, &filtered, &node, &tyre, h);
    assert_eq!(s.peak_raw, (1e5, 301));
    let own_sum: f64 = raw.iter().fold(0.0, |sum, f| sum + f * h);
    close(
        "sum vs closed form",
        own_sum,
        1e5 * h / (PI / 800.0).tan(),
        1e-9,
    );
    assert_eq!(s.total_ns.to_bits(), own_sum.to_bits());
    close(
        "partition impulse",
        s.node_ns + s.tyre_ns,
        s.total_ns,
        1e-12 * s.total_ns,
    );
    let mut ties = vec![0.0; 40];
    ties[10] = 3.0;
    ties[30] = 3.0;
    let tie = summarize(&ties, &ties, &ties, &[0.0; 40], h);
    assert_eq!(tie.peak_raw, (3.0, 11));
    assert_eq!(tie.peak_cfc60, (3.0, 11));
    let crush: Vec<_> = (1..=50).map(|k| 0.001 * k as f64).collect();
    let rows = force_vs_crush(&crush, &[0.0; 50], &[0.0; 50], 0.01);
    assert_eq!(rows.len(), 5);
    for (i, &(target, k)) in rows.iter().enumerate() {
        assert_eq!(target, 0.01 * (i + 1) as f64);
        assert_eq!(k, 10 * (i + 1));
    }
    let nonmonotonic = [0.0, 0.02, 0.0, 0.03];
    assert_eq!(
        force_vs_crush(&nonmonotonic, &[0.0; 4], &[0.0; 4], 0.01),
        vec![(0.01, 2), (0.02, 2), (0.03, 4)]
    );
    for rows in [
        force_vs_crush(&crush, &[0.0], &[0.0], 0.01),
        force_vs_crush(&crush, &crush, &crush, 0.0),
        force_vs_crush(&[f64::MAX], &[0.0], &[0.0], f64::from_bits(1)),
    ] {
        assert_eq!(rows.len(), 1);
        assert!(rows[0].0.is_nan());
        assert_eq!(rows[0].1, 0);
    }
    assert!(summarize(&[], &[], &[], &[], h).total_ns.is_nan());
    // Dyadic fixtures have exact integer references even at binary64's edges.
    for (input, expected) in [
        (1, 2),
        (0x0010_0000_0000_0000, 0x0020_0000_0000_0000),
        (0x7fdf_ffff_ffff_ffff, 0x7fef_ffff_ffff_ffff),
    ] {
        let unit = f64::from_bits(input);
        let f = [unit, unit];
        let s = summarize(&f, &f, &f, &[0.0; 2], 1.0);
        assert_eq!(s.total_ns.to_bits(), expected);
    }
    let f = [2.0_f64.powi(52), -2.0_f64.powi(52) + 1.0];
    let s = summarize(&f, &f, &f, &[0.0; 2], 1.0);
    assert_eq!(s.total_ns, 1.0, "exact integer cancellation reference");
    let overflow = [f64::MAX, f64::MAX];
    assert!(summarize(&overflow, &overflow, &overflow, &[0.0; 2], 1.0)
        .total_ns
        .is_nan());
    let tiny = [f64::from_bits(1)];
    assert!(summarize(&tiny, &tiny, &tiny, &[0.0], 0.5).total_ns.is_nan());
    let boundary = [0.01 * 29.0];
    let rows = force_vs_crush(&boundary, &[0.0], &[0.0], 0.01);
    assert_eq!(rows.len(), 29);
    assert_eq!(rows.last(), Some(&(0.01 * 29.0, 1)));
}

struct FailingWriter {
    bytes: Vec<u8>,
    remaining: usize,
    fail_flush: bool,
}

impl Write for FailingWriter {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        if self.remaining == 0 {
            return Err(io::Error::other("intentional write failure"));
        }
        let n = self.remaining.min(buf.len());
        self.bytes.extend_from_slice(&buf[..n]);
        self.remaining -= n;
        Ok(n)
    }

    fn flush(&mut self) -> io::Result<()> {
        if self.fail_flush {
            Err(io::Error::other("intentional flush failure"))
        } else {
            Ok(())
        }
    }
}

fn output_rows(text: &str, w: &WallForce) {
    let lines: Vec<_> = text.lines().collect();
    assert!(text.ends_with('\n'));
    let n = w.raw_n.len();
    assert_eq!(lines.iter().filter(|l| l.starts_with("force ")).count(), n);
    let schema = [
        ("cfc", vec!["cfc", "dt_s", "a0", "a1", "a2", "b1", "b2"]),
        ("contact", vec!["contact_s", "substep"]),
        ("peak", vec!["series", "force_n", "t_rel_s"]),
        ("peak", vec!["series", "force_n", "t_rel_s"]),
        ("impulse", vec!["node_ns", "tyre_ns", "total_ns", "cfc60_ns"]),
        ("crush", vec!["max_m", "t_rel_s"]),
        ("ground", vec!["node_substeps", "tyre_substeps"]),
        (
            "momentum",
            vec![
                "p0_x_ns",
                "p_end_x_ns",
                "dp_x_ns",
                "wall_x_ns",
                "ground_node_x_ns",
                "aero_x_ns",
                "residual_ns",
                "tolerance_ns",
            ],
        ),
    ];
    let crossings = force_vs_crush(&w.crush_m, &w.raw_n, &w.cfc60_n, 0.01);
    assert!(!crossings.is_empty(), "force/crush output must not be vacuous");
    let start = n + crossings.len();
    assert_eq!(lines.len(), start + schema.len() + 1);
    for (i, line) in lines[..lines.len() - 1].iter().enumerate() {
        let words: Vec<_> = line.split_whitespace().collect();
        let keys = if i < n {
            assert_eq!(words[0], "force");
            vec!["t_s", "raw_n", "node_n", "tyre_n", "cfc60_n", "crush_m"]
        } else if i < start {
            assert_eq!(words[0], "fvc");
            vec!["crush_target_m", "t_rel_s", "crush_m", "raw_n", "cfc60_n"]
        } else {
            let (label, keys) = &schema[i - start];
            assert_eq!(words[0], *label);
            keys.clone()
        };
        assert_eq!(words.len(), keys.len() + 1, "row {i}");
        for (&word, key) in words[1..].iter().zip(keys) {
            let (name, value) = word.split_once('=').unwrap();
            assert_eq!(name, key, "row {i}");
            if key == "series" {
                assert_eq!(value, if i == start + 2 { "raw" } else { "cfc60" });
            } else {
                let parsed: f64 = value.parse().unwrap();
                assert!(parsed.is_finite(), "row {i}, {key}");
                if ["a0", "a1", "a2", "b1", "b2", "residual_ns", "tolerance_ns"].contains(&key) {
                    let (mantissa, _) = value.split_once('e').unwrap();
                    assert_eq!(mantissa.split_once('.').unwrap().1.len(), 15);
                } else if !["cfc", "substep", "node_substeps", "tyre_substeps"].contains(&key) {
                    assert_eq!(value.split_once('.').unwrap().1.len(), 6);
                } else {
                    value.parse::<usize>().unwrap();
                }
            }
        }
    }
    assert_eq!(lines.last(), Some(&"wall_force_a: ok"));
    let summary = summarize(&w.raw_n, &w.cfc60_n, &w.node_n, &w.tyre_n, w.h);
    for (offset, p) in [(2, summary.peak_raw), (3, summary.peak_cfc60)] {
        let rel = (p.1 as f64 - w.contact_k as f64) * w.h;
        assert!(lines[start + offset].ends_with(&format!("t_rel_s={rel:.6}")));
    }
    for (i, &(target, k)) in crossings.iter().enumerate() {
        let rel = (k as f64 - w.contact_k as f64) * w.h;
        assert_eq!(
            lines[n + i],
            format!(
                "fvc crush_target_m={target:.6} t_rel_s={rel:.6} crush_m={:.6} raw_n={:.6} cfc60_n={:.6}",
                w.crush_m[k - 1], w.raw_n[k - 1], w.cfc60_n[k - 1]
            )
        );
    }
}

#[test]
fn wall_force_a_output_contract_and_exit_codes() {
    let (mut out, mut err) = (Vec::new(), Vec::new());
    assert_eq!(run(&["--test-a".into()], &mut out, &mut err), 0);
    assert!(err.is_empty());
    output_rows(std::str::from_utf8(&out).unwrap(), measured());
    let before_ok = out
        .strip_suffix(b"wall_force_a: ok\n")
        .expect("successful output must end with the ok line")
        .to_vec();
    assert!(!before_ok.is_empty());
    for args in [
        vec![],
        vec!["--test-x".into()],
        vec!["--test-a".into(), "--x".into()],
    ] {
        out.clear();
        err.clear();
        assert_eq!(run(&args, &mut out, &mut err), 2);
        assert!(out.is_empty());
        let text = std::str::from_utf8(&err).unwrap();
        assert!(text.contains("usage:"));
        assert_eq!(text.lines().count(), 1);
    }
    for (remaining, fail_flush) in [
        (0, false),
        (100, false),
        (usize::MAX, true),
        (before_ok.len(), false),
    ] {
        let mut writer = FailingWriter {
            bytes: Vec::new(),
            remaining,
            fail_flush,
        };
        err.clear();
        assert_eq!(run(&["--test-a".into()], &mut writer, &mut err), 1);
        if remaining == before_ok.len() {
            assert_eq!(writer.bytes, before_ok, "all output before the ok line");
        }
        assert!(!String::from_utf8(writer.bytes)
            .unwrap()
            .contains("wall_force_a: ok"));
        let text = String::from_utf8(err.clone()).unwrap();
        assert!(text.starts_with("wall_force_a: FAIL output:"));
        assert_eq!(text.lines().count(), 1);
    }
    for path in ["examples/wall_force_a.rs", "tests/wall_force_a.rs"] {
        let text = std::fs::read_to_string(path).unwrap();
        for forbidden in [
            ["test", "-b"].concat(),
            ["_", "b.bsn"].concat(),
            ["_", "b_m5"].concat(),
        ] {
            assert!(!text.contains(&forbidden), "{path}: forbidden scenario name");
        }
    }
    let text = std::fs::read_to_string("docs/STAGE6-M4-M5.md").unwrap();
    let line = text
        .lines()
        .find(|l| l.starts_with("frozen: run_flags = "))
        .unwrap();
    let valid = parse_run_flags(line).unwrap();
    assert_eq!(valid.h, measured().h);
    assert_eq!(valid.n, measured().raw_n.len());
    let mut invalid = vec![
        String::new(),
        format!("{line}\n{line}"),
        format!("{line} --dt 0.001"),
        line.replace("--pulse-crush-groups rear,front", "--pulse-crush-groups ,front"),
        line.replace(
            "--pulse-crush-groups rear,front",
            "--pulse-crush-groups rear,front,extra",
        ),
        line.replace("--dt ", "--other "),
        line.replace("--dt ", "--dt \r"),
        line.replace("--dt ", "--dt \u{2028}"),
    ];
    for bad in ["NaN", "inf", "-1", "0", "1e309", "1e-320", "C:\\x", ""] {
        invalid.push(line.replace("--dt 0.001", &format!("--dt {bad}")));
    }
    for bad in ["-1", "0", "1.5", "4294967296", "NaN", ""] {
        invalid.push(line.replace("--substeps 10", &format!("--substeps {bad}")));
    }
    invalid.push(line.replace("--duration 0.2", "--duration 1e308"));
    invalid.push(line.replace("--gravity 0", "--gravity 1"));
    invalid.push(line.replace("--dt 0.001", &format!("--dt {}", "9".repeat(1_000_000))));
    for flag in ["--gravity", "--duration"] {
        let old = if flag == "--gravity" { "0" } else { "0.2" };
        for bad in ["NaN", "-1", "inf", "1e-999", ""] {
            invalid.push(line.replace(&format!("{flag} {old}"), &format!("{flag} {bad}")));
        }
    }
    for (i, text) in invalid.into_iter().enumerate() {
        let result = parse_run_flags(&text);
        assert!(matches!(result, Err(WallError::Input(_))), "invalid case {i}");
    }
    let mut body = plain().body.clone();
    body.materials[0].break_strain = Some(1.0);
    assert!(matches!(validate_body(&body), Err(WallError::Input(_))));
    body = plain().body.clone();
    body.wheels.clear();
    assert!(matches!(validate_body(&body), Err(WallError::Input(_))));
    body = plain().body.clone();
    body.planes.clear();
    assert!(matches!(validate_body(&body), Err(WallError::Input(_))));
    body = plain().body.clone();
    body.tets.push(format::Tet {
        nodes: [0, 1, 2, 3],
        solid: "unused".into(),
    });
    assert!(matches!(validate_body(&body), Err(WallError::Input(_))));
}
