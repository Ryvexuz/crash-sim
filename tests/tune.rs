use crash_sim::tune::{apply, parse_tune, points, Tune};
use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::{Command, Output};
use std::sync::atomic::{AtomicUsize, Ordering};

mod run_tmp;

/// Use broadsword's runtime binary path, falling back to the cargo layout.
fn crash_sim_bin() -> PathBuf {
    if let Some(p) = std::env::var_os("CARGO_BIN_EXE_crash-sim") {
        return PathBuf::from(p);
    }
    let exe = std::env::current_exe().expect("current_exe of the test executable");
    let name = format!("crash-sim{}", std::env::consts::EXE_SUFFIX);
    let dir = exe.parent().and_then(|p| p.parent()).unwrap_or_else(|| {
        panic!(
            "CARGO_BIN_EXE_crash-sim is not set and {} has no grandparent folder",
            exe.display()
        )
    });
    let bin = dir.join(name);
    assert!(
        bin.is_file(),
        "CARGO_BIN_EXE_crash-sim is not set and the cargo layout path {} does not exist",
        bin.display()
    );
    bin
}

fn no_read(path: &str) -> Result<String, String> {
    Err(format!("unexpected include {path}"))
}

fn tune(lines: &str) -> Tune {
    parse_tune(&format!("tune 1\n{lines}"), "test.tune", &no_read).unwrap()
}

fn rewrite(body: &str, lines: &str) -> String {
    let t = tune(lines);
    apply(body, &t, &points(&t)[0]).unwrap()
}

#[test]
fn sweep_values_hit_both_ends_lin_and_log() {
    for (mode, from, to, expected) in [
        ("lin", 2.0, 6.0, vec![2.0, 4.0, 6.0]),
        ("log", 1.0, 16.0, vec![1.0, 4.0, 16.0]),
        ("lin", 6.0, 2.0, vec![6.0, 4.0, 2.0]),
        ("log", 16.0, 1.0, vec![16.0, 4.0, 1.0]),
    ] {
        let t = tune(&format!("sweep material box stiffness {from} {to} 3 {mode}"));
        let values: Vec<_> = points(&t).iter().map(|p| p[0].1).collect();
        // Interior values are independently specified too: changing the denominator
        // must fail even if endpoint assignment remains explicit.
        assert_eq!(values, expected, "{mode} {from}..{to}");
    }
    let t = tune("sweep material box stiffness 0.1 0.7 200 log");
    let p = points(&t);
    assert_eq!(p.len(), 200);
    assert_eq!(p[0][0].1, 0.1);
    assert_eq!(p[199][0].1, 0.7);
}

#[test]
fn set_unknown_name_is_an_error_with_line() {
    let t = tune("# ignored\nset material missing stiffness 10");
    let error = apply("material box stiffness 1\n", &t, &[]).unwrap_err();
    assert_eq!(error.file, "test.tune");
    assert_eq!(error.line, 3);
    assert!(error.message.contains("no body line matches material missing"));
    assert!(error.to_string().starts_with("test.tune:3:"));
}

#[test]
fn field_matches_whole_token_only() {
    let body = "tyre fl radial_damping 12 damping 3\n";
    assert_eq!(
        rewrite(body, "set tyre fl damping 7"),
        "tyre fl radial_damping 12 damping 7\n"
    );
    let t = tune("set tyre fl damping 7");
    let error = apply("tyre fl radial_damping 12\n", &t, &[]).unwrap_err();
    assert!(error.message.contains("no \"damping\" field token"));
}

#[test]
fn realistic_preset_is_identity_on_validation_wall() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let path = root.join("scenarios/presets/realistic.tune");
    let text = fs::read_to_string(&path).unwrap();
    let t = parse_tune(&text, path.to_str().unwrap(), &no_read).unwrap();
    let body = fs::read_to_string(root.join("scenarios/validation_wall.bsn")).unwrap();
    assert_eq!(points(&t), vec![Vec::<(String, f64)>::new()]);
    assert_eq!(apply(&body, &t, &[]).unwrap().as_bytes(), body.as_bytes());
    assert!(crash_sim::format::parse(&body).is_ok());
}

#[test]
fn untouched_lines_and_crlf_are_kept() {
    let body = "# untouched \t comment\r\nbsn 1\r\n  material box\tstiffness 1  damping 2\r\n\
                node 0 1 2 3 mass 4\r\n# material box stiffness 888\r\nlast without newline";
    let expected = body.replacen("stiffness 1 ", "stiffness 10 ", 1);
    assert_eq!(rewrite(body, "set material box stiffness 10"), expected);
    assert_eq!(apply(body, &Tune::default(), &[]).unwrap(), body);
    assert_eq!(
        rewrite("material box stiffness 1", "set material box stiffness 2"),
        "material box stiffness 2"
    );
}

fn virtual_parse(
    root: &str,
    text: &str,
    files: &[(&str, &str)],
) -> Result<Tune, crash_sim::tune::TuneError> {
    let files: HashMap<_, _> = files.iter().copied().collect();
    parse_tune(text, root, &|path| {
        files
            .get(path)
            .map(|text| (*text).to_owned())
            .ok_or_else(|| format!("missing virtual file {path}"))
    })
}

#[test]
fn include_cycle_and_depth_are_errors() {
    let cycle = virtual_parse(
        "dir/a.tune",
        "tune 1\ninclude b.tune\n",
        &[("dir/b.tune", "tune 1\ninclude .\\a.tune\n")],
    )
    .unwrap_err();
    assert_eq!(cycle.file, "dir/b.tune");
    assert_eq!(cycle.line, 2);
    assert!(cycle.message.contains("cycle"));
    let chain = [
        ("b", "tune 1\ninclude c\n"),
        ("c", "tune 1\ninclude d\n"),
        ("d", "tune 1\ninclude e\n"),
        ("e", "tune 1\ninclude f\n"),
        ("f", "tune 1\n"),
    ];
    let error = virtual_parse("a", "tune 1\ninclude b\n", &chain).unwrap_err();
    assert_eq!(error.file, "e");
    assert_eq!(error.line, 2);
    assert!(error.message.contains("depth"));
    let mut allowed = chain;
    allowed[3] = ("e", "tune 1\n");
    assert!(virtual_parse("a", "tune 1\ninclude b\n", &allowed).is_ok());
}

#[test]
fn two_sweeps_make_a_grid_in_order() {
    let t = tune("sweep material a stiffness 10 20 2\nsweep material b damping 1 3 3");
    let p = points(&t);
    let expected = [
        (10.0, 1.0),
        (10.0, 2.0),
        (10.0, 3.0),
        (20.0, 1.0),
        (20.0, 2.0),
        (20.0, 3.0),
    ];
    assert_eq!(p.len(), expected.len());
    for (point, (a, b)) in p.iter().zip(expected) {
        assert_eq!(
            point,
            &vec![
                ("material.a.stiffness".to_owned(), a),
                ("material.b.damping".to_owned(), b)
            ]
        );
    }
}

#[test]
fn scale_star_touches_every_material_line() {
    let body = "material a stiffness 10 damping 1\nmaterial b stiffness 20 damping 2\n\
                material c damping 3\nnode 0 0 0 0 mass 1\n";
    let expected = "material a stiffness 5 damping 1\nmaterial b stiffness 10 damping 2\n\
                    material c damping 3\nnode 0 0 0 0 mass 1\n";
    assert_eq!(rewrite(body, "scale material * stiffness 0.5"), expected);
    let t = tune("scale material * stiffness 2");
    assert!(apply("material c damping 3\n", &t, &[]).is_err());
    assert!(apply("material c stiffness nope\n", &t, &[]).is_err());
}

struct Scratch {
    root: PathBuf,
    path: PathBuf,
}

impl Scratch {
    fn new() -> Self {
        static NEXT: AtomicUsize = AtomicUsize::new(0);
        let root = run_tmp::run_tmp().canonicalize().unwrap();
        let path = root.join(format!("tune-{}", NEXT.fetch_add(1, Ordering::Relaxed)));
        fs::create_dir(&path).unwrap();
        Self { root, path }
    }
}

impl Drop for Scratch {
    fn drop(&mut self) {
        let resolved = self
            .path
            .canonicalize()
            .expect("resolve tune scratch directory");
        assert_eq!(resolved.parent(), Some(self.root.as_path()));
        assert_ne!(resolved, self.root);
        fs::remove_dir_all(&resolved).expect("remove only this test's tune directory");
    }
}

fn cli(scratch: &Scratch, extra: &[&str], run_flags: &[&str]) -> Output {
    cli_body(
        scratch,
        Path::new("scenarios/box_wall.bsn"),
        extra,
        run_flags,
    )
}

fn cli_body(scratch: &Scratch, body: &Path, extra: &[&str], run_flags: &[&str]) -> Output {
    Command::new(crash_sim_bin())
        .current_dir(env!("CARGO_MANIFEST_DIR"))
        .args(["tune", "--body"])
        .arg(body)
        .arg("--out")
        .arg(scratch.path.join("out"))
        .args(extra)
        .arg("--")
        .args(run_flags)
        .output()
        .unwrap()
}

fn assert_code(output: &Output, code: i32) {
    assert_eq!(
        output.status.code(),
        Some(code),
        "stdout: {}\nstderr: {}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
}

fn read_table(scratch: &Scratch, output: &Output) -> Vec<Vec<String>> {
    let text = fs::read_to_string(scratch.path.join("out/results.tsv")).unwrap();
    assert_eq!(text.as_bytes(), output.stdout);
    assert!(text.ends_with('\n'));
    text.lines()
        .map(|line| line.split('\t').map(str::to_owned).collect())
        .collect()
}

#[test]
fn cli_sweep_runs_each_point_with_no_rebuild() {
    let scratch = Scratch::new();
    let file = scratch.path.join("sweep.tune");
    fs::write(
        &file,
        "tune 1\nsweep material box_beam stiffness 20000 40000 3\n",
    )
    .unwrap();
    let output = cli(
        &scratch,
        &[
            "--set",
            "material.box_beam.stiffness=12345",
            "--tune",
            file.to_str().unwrap(),
            "--jobs",
            "2",
        ],
        &["--summary"],
    );
    assert_code(&output, 0);
    let table = read_table(&scratch, &output);
    assert_eq!(table.len(), 4);
    assert_eq!(
        table[0],
        [
            "point",
            "material.box_beam.stiffness",
            "exit",
            "wall_s",
            "stdout"
        ]
    );
    for (i, expected) in [20000.0, 30000.0, 40000.0].into_iter().enumerate() {
        let body =
            fs::read_to_string(scratch.path.join(format!("out/point-{:03}.bsn", i + 1))).unwrap();
        let parsed = crash_sim::format::parse(&body).unwrap();
        assert_eq!(parsed.materials[0].stiffness, expected);
        assert_eq!(table[i + 1][0], (i + 1).to_string());
        assert_eq!(table[i + 1][1], expected.to_string());
        assert!(table[i + 1][2].parse::<i32>().is_ok());
        let wall = &table[i + 1][3];
        assert!(wall.parse::<f64>().unwrap().is_finite());
        assert_eq!(wall.split_once('.').unwrap().1.len(), 2);
        let log =
            fs::read_to_string(scratch.path.join(format!("out/point-{:03}.txt", i + 1))).unwrap();
        assert!(!log.is_empty(), "child {i} must actually produce output");
    }
    // Printed for the verifier's requested results.tsv evidence.
    println!(
        "test 9 results.tsv:\n{}",
        String::from_utf8(output.stdout).unwrap()
    );
}

#[test]
fn cli_invalid_point_is_reported_and_others_run() {
    let scratch = Scratch::new();
    let file = scratch.path.join("sweep.tune");
    fs::write(&file, "tune 1\nsweep material box_beam stiffness -1 1e5 3\n").unwrap();
    let output = cli(&scratch, &["--tune", file.to_str().unwrap()], &["--summary"]);
    assert_code(&output, 1);
    let table = read_table(&scratch, &output);
    assert_eq!(table.len(), 4);
    assert_eq!(table[1][1], "-1");
    assert_eq!(table[1][2], "invalid");
    assert!(table[1][4].contains("stiffness"));
    assert!(table[1][4].contains("-1"));
    let invalid_log = fs::read_to_string(scratch.path.join("out/point-001.txt")).unwrap();
    assert!(invalid_log.starts_with("invalid: values "));
    for (index, row) in table.iter().enumerate().skip(2) {
        assert!(row[2].parse::<i32>().is_ok());
        let log =
            fs::read_to_string(scratch.path.join(format!("out/point-{index:03}.txt"))).unwrap();
        assert!(!log.is_empty(), "valid point {index} must run");
    }
}

#[test]
fn parser_errors_are_located_and_limits_cannot_be_bypassed_by_merge() {
    for (line, reason) in [
        ("nonsense a b c", "unknown line kind"),
        ("set material a stiffness", "wrong token count"),
        ("scale material a stiffness 0", "positive"),
        ("scale material a stiffness -1", "positive"),
        ("set material a stiffness NaN", "bad number"),
        ("set material a stiffness inf", "bad number"),
        ("set material a stiffness 1e999", "bad number"),
        ("set material a stiffness 1e-999", "underflows"),
        ("sweep material a stiffness 1 2 1", "steps"),
        ("sweep material a stiffness 1 2 201", "steps"),
        ("sweep material a stiffness 1 2 -2", "steps"),
        (
            "sweep material a stiffness 1 2 184467440737095516160",
            "steps",
        ),
        ("sweep material a stiffness 1 2 3.0", "steps"),
        ("sweep material a stiffness 1 2 3 cubic", "mode"),
        ("sweep material a stiffness 0 2 3 log", "positive"),
        ("sweep material a stiffness 1 NaN 3", "bad number"),
        ("include C:\\bad.tune", "relative"),
        ("include \\\\server\\bad.tune", "relative"),
        ("include /bad.tune", "relative"),
        ("include bad\0.tune", "control character"),
        ("include", "wrong token count"),
    ] {
        let error = parse_tune(&format!("tune 1\n{line}\n"), "bad.tune", &no_read).unwrap_err();
        assert_eq!(error.file, "bad.tune");
        assert_eq!(error.line, 2);
        assert!(error.message.contains(reason), "{line}: {error}");
    }
    for text in ["", "# only comments\n", "tune 2\n", "set material a stiffness 2\n"] {
        assert!(parse_tune(text, "empty", &no_read).is_err());
    }
    let mut a = tune("sweep material a stiffness 1 2 20");
    a.append(tune("sweep material b stiffness 1 2 20")).unwrap();
    assert_eq!(points(&a).len(), 400);
    assert!(a.append(tune("sweep material c stiffness 1 2 2")).is_err());
    assert_eq!(points(&a).len(), 400, "failed append must be atomic");
    let mut b = tune("sweep material a stiffness 1 2 200");
    let error = b
        .append(tune("sweep material b stiffness 1 2 3"))
        .unwrap_err();
    assert!(error.message.contains("400"));
    assert_eq!(points(&b).len(), 200);
    let error = parse_tune(
        "tune 1\nsweep material a stiffness 1 2 200\nsweep material b stiffness 1 2 3\n",
        "large.tune",
        &no_read,
    )
    .unwrap_err();
    assert_eq!(error.line, 3);
    assert!(error.message.contains("400"));
}

#[test]
fn includes_and_edits_apply_in_place_and_sweeps_apply_last() {
    let t = virtual_parse(
        "dir/root.tune",
        "# comment before header\n\ntune 1\nset material a stiffness 4\ninclude sub\\child\n\
         scale material a stiffness 2\nsweep material a damping 5 7 2\nset material a damping 99",
        &[("dir/sub/child", "tune 1\nscale material a stiffness 3\n")],
    )
    .unwrap();
    let p = points(&t);
    assert_eq!(
        apply("material a stiffness 1 damping 1\n", &t, &p[0]).unwrap(),
        "material a stiffness 24 damping 5\n"
    );
    let error = virtual_parse("dir/root", "tune 1\ninclude missing\n", &[]).unwrap_err();
    assert_eq!(error.line, 2);
    assert!(error.message.contains("cannot include dir/missing"));
}

#[test]
fn portable_paths_preserve_roots_and_reject_ambiguous_names() {
    for (root, expected) in [
        ("/root.tune", "/child"),
        ("C:\\dir\\root.tune", "C:/child"),
        ("//server/share/root.tune", "//server/share/child"),
        ("\\\\?\\C:\\root.tune", "//?/C:/child"),
        (
            "\\\\?\\UNC\\server\\share\\root.tune",
            "//?/UNC/server/share/child",
        ),
    ] {
        assert!(virtual_parse(
            root,
            "tune 1\ninclude ../child\n",
            &[(expected, "tune 1\n")]
        )
        .is_ok());
    }
    for file in ["", "bad\rfile", "bad\nfile", "C:relative.tune"] {
        let error = parse_tune("tune 1\n", file, &no_read).unwrap_err();
        assert_eq!(error.line, 1);
    }
    let cycle = virtual_parse("dir/root", "tune 1\ninclude sub/../root\n", &[]).unwrap_err();
    assert!(cycle.message.contains("cycle"));
}

#[test]
fn huge_comments_and_excess_tokens_do_not_hide_errors() {
    let comment = format!("# {}\n", "ignored ".repeat(131_072));
    let text = format!("{comment}tune 1\nset material a stiffness 2\n");
    let t = parse_tune(&text, "huge.tune", &no_read).unwrap();
    let body = format!("{comment}material a stiffness 1\n");
    assert_eq!(
        apply(&body, &t, &[]).unwrap(),
        format!("{comment}material a stiffness 2\n")
    );
    let excessive = format!(
        "tune 1\nset material a stiffness 2 {}",
        "extra ".repeat(131_072)
    );
    let error = parse_tune(&excessive, "huge.tune", &no_read).unwrap_err();
    assert_eq!(error.line, 2);
    assert!(error.message.contains("wrong token count"));
    assert!(parse_tune("tune 1\ntune 1\n", "duplicate.tune", &no_read).is_err());
    assert!(parse_tune("tune 1\ninclude \"a b\"\n", "quoted.tune", &no_read).is_err());
}

#[test]
fn bad_fields_and_nonfinite_points_fail_at_the_library_entry_point() {
    for body in [
        "material a stiffness nope\n",
        "material a stiffness NaN\n",
        "material a stiffness\n",
        "material a damping 1\n",
    ] {
        let t = tune("set material a stiffness 2");
        let error = apply(body, &t, &[]).unwrap_err();
        assert_eq!(error.file, "test.tune");
        assert_eq!(error.line, 2);
    }
    let t = tune("sweep material a stiffness 1 2 2");
    for point in [
        vec![],
        vec![("wrong".into(), 1.0)],
        vec![("material.a.stiffness".into(), f64::INFINITY)],
        vec![("material.a.stiffness".into(), f64::NAN)],
    ] {
        assert!(apply("material a stiffness 1\n", &t, &point).is_err());
    }
    let t = tune("scale material a stiffness 2");
    assert!(apply(&format!("material a stiffness {}\n", f64::MAX), &t, &[]).is_err());
    let t = tune("scale material a stiffness 0.25");
    assert!(apply(&format!("material a stiffness {}\n", f64::from_bits(1)), &t, &[]).is_err());
    assert_eq!(
        rewrite("material a stiffness 0\n", "scale material a stiffness 2"),
        "material a stiffness 0\n"
    );
    for factor in [f64::from_bits(1), f64::MAX] {
        let text = rewrite(
            "material a stiffness 1\n",
            &format!("scale material a stiffness {factor}"),
        );
        assert_eq!(text, format!("material a stiffness {factor}\n"));
    }
}

#[test]
fn interpolation_covers_extreme_and_near_cancelling_finite_values() {
    // References are exact binary rationals, specified independently of interpolation.
    // The subnormal bit patterns are integer multiples of 2^-1074.
    let low = 2.0_f64.powi(-1022);
    let high = 2.0_f64.powi(1022);
    for (from, to, mode, middle) in [
        (-f64::MAX, f64::MAX, "lin", 0.0),
        (f64::MAX, -f64::MAX, "lin", 0.0),
        (
            f64::from_bits(1),
            f64::from_bits(5),
            "lin",
            f64::from_bits(3),
        ),
        (0.0, f64::from_bits(1), "lin", 0.0),
        (
            1.0,
            1.0 + 4.0 * f64::EPSILON,
            "lin",
            1.0 + 2.0 * f64::EPSILON,
        ),
        (low, high, "log", 1.0),
        (high, low, "log", 1.0),
        (
            f64::from_bits(1),
            f64::from_bits(16),
            "log",
            f64::from_bits(4),
        ),
        (f64::MAX, f64::MAX, "log", f64::MAX),
    ] {
        let t = tune(&format!("sweep material a stiffness {from} {to} 3 {mode}"));
        let p = points(&t);
        assert_eq!(p[0][0].1, from);
        assert_eq!(p[1][0].1, middle, "{from}..{to} {mode}");
        assert_eq!(p[2][0].1, to);
        assert!(p.iter().all(|point| point[0].1.is_finite()));
    }
}

#[test]
fn cli_tune_errors_run_nothing_and_usage_is_validated() {
    for extra in [
        vec!["--set", "material.missing.stiffness=1"],
        vec!["--set", "material.box_beam.stiffness=NaN"],
        vec!["--set", "material..stiffness=1"],
        vec![
            "--set",
            "material.box_beam.stiffness=1\nset material x stiffness 2",
        ],
        vec!["--jobs", "0"],
        vec!["--jobs", "17"],
        vec!["--jobs", "999999999999999999999999999"],
        vec!["--jobs", "-1"],
        vec!["--unknown", "x"],
        vec!["--set"],
    ] {
        let scratch = Scratch::new();
        let output = cli(&scratch, &extra, &["--summary"]);
        assert_code(&output, 2);
        assert!(!output.stderr.is_empty());
        assert!(!scratch.path.join("out").exists(), "{extra:?}");
    }
    let scratch = Scratch::new();
    let output = cli(&scratch, &[], &["--body", "scenarios/box_wall.bsn"]);
    assert_code(&output, 2);
    assert!(!scratch.path.join("out").exists());
}

#[test]
fn cli_show_selects_stdout_and_nonzero_child_exit_is_still_a_run() {
    let scratch = Scratch::new();
    let output = cli(
        &scratch,
        &["--show", "first_contact_s="],
        &["--summary", "--duration", "0.001"],
    );
    assert_code(&output, 0);
    let table = read_table(&scratch, &output);
    assert_eq!(table.len(), 2);
    assert!(table[1][3].starts_with("first_contact_s="));
    let scratch = Scratch::new();
    let output = cli(&scratch, &[], &["--not-a-run-flag"]);
    assert_code(&output, 0);
    let table = read_table(&scratch, &output);
    assert_eq!(table[1][1], "2");
    assert!(
        table[1][3].is_empty(),
        "stderr must not become a stdout summary"
    );
    let log = fs::read_to_string(scratch.path.join("out/point-001.txt")).unwrap();
    assert!(log.contains("usage:"));
}

#[test]
fn cli_show_joins_all_matching_stdout_lines_once_in_output_order() {
    let scratch = Scratch::new();
    let output = cli(
        &scratch,
        &["--show", "t,", "--show", "0.0000,", "--show", "0.0000,"],
        &["--duration", "0.001"],
    );
    assert_code(&output, 0);
    let table = read_table(&scratch, &output);
    assert_eq!(
        table[1][3],
        "t,min_y,center_y,max_beam_strain,broken_beams | 0.0000,0.000000,0.500000,0.000000,0"
    );
}

#[test]
fn cli_drains_large_stdout_and_keeps_the_last_line() {
    let scratch = Scratch::new();
    let body = scratch.path.join("stationary.bsn");
    fs::write(
        &body,
        "bsn 1\nmaterial still stiffness 20000 damping 20\n\
         node 0 0 1 0 mass 1\nnode 1 1 1 0 mass 1\nbeam 0 1 still\n",
    )
    .unwrap();
    let steps = 10_000;
    let duration = crash_sim::MIN_DT * f64::from(steps);
    let duration_arg = duration.to_string();
    let dt = crash_sim::MIN_DT.to_string();
    let output = cli_body(
        &scratch,
        &body,
        &[],
        &["--duration", &duration_arg, "--dt", &dt, "--gravity", "0"],
    );
    assert_code(&output, 0);
    let table = read_table(&scratch, &output);
    assert_eq!(table.len(), 2);
    assert_eq!(table[1][1], "0");
    assert_eq!(
        table[1][3],
        format!("{duration:.4},1.000000,1.000000,0.000000,0")
    );
    let log = fs::read_to_string(scratch.path.join("out/point-001.txt")).unwrap();
    assert_eq!(log.lines().count(), usize::try_from(steps).unwrap() + 2);
}

#[test]
fn cli_top_level_paths_with_spaces_and_relative_includes_work() {
    let scratch = Scratch::new();
    let file = scratch.path.join("with spaces.tune");
    fs::write(&file, "tune 1\ninclude .\\child.tune\n").unwrap();
    fs::write(
        scratch.path.join("child.tune"),
        "tune 1\nset material box_beam stiffness 18000\n",
    )
    .unwrap();
    let output = cli(
        &scratch,
        &["--tune", file.to_str().unwrap()],
        &["--summary", "--duration", "0.001"],
    );
    assert_code(&output, 0);
    let body = fs::read_to_string(scratch.path.join("out/point-001.bsn")).unwrap();
    assert_eq!(
        crash_sim::format::parse(&body).unwrap().materials[0].stiffness,
        18000.0
    );
}

#[test]
fn cli_output_reservation_failure_starts_no_children_and_preserves_files() {
    let scratch = Scratch::new();
    let out = scratch.path.join("out");
    fs::create_dir(&out).unwrap();
    fs::write(out.join("point-001.txt"), "keep this log\n").unwrap();
    let output = cli(&scratch, &[], &["--summary"]);
    assert_code(&output, 2);
    assert_eq!(
        fs::read_to_string(out.join("point-001.txt")).unwrap(),
        "keep this log\n"
    );
    assert!(fs::read(out.join("results.tsv")).unwrap().is_empty());
    let body = fs::read_to_string(out.join("point-001.bsn")).unwrap();
    assert!(crash_sim::format::parse(&body).is_ok());
}

#[test]
fn cli_files_then_sets_and_existing_outputs_are_not_overwritten() {
    let scratch = Scratch::new();
    let file = scratch.path.join("base.tune");
    fs::write(&file, "tune 1\nset material box_beam stiffness 15000\n").unwrap();
    let extra = [
        "--set",
        "material.box_beam.stiffness=18000",
        "--tune",
        file.to_str().unwrap(),
        "--set",
        "material.box_beam.stiffness=20000",
    ];
    let output = cli(&scratch, &extra, &["--summary"]);
    assert_code(&output, 0);
    let path = scratch.path.join("out/point-001.bsn");
    let body = fs::read_to_string(&path).unwrap();
    assert_eq!(
        crash_sim::format::parse(&body).unwrap().materials[0].stiffness,
        20000.0
    );
    let table = fs::read(scratch.path.join("out/results.tsv")).unwrap();
    let output = cli(&scratch, &extra, &["--summary"]);
    assert_code(&output, 2);
    assert_eq!(fs::read_to_string(&path).unwrap(), body);
    assert_eq!(fs::read(scratch.path.join("out/results.tsv")).unwrap(), table);
}
