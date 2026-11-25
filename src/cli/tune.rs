//! File/child-process adapter for the pure tune module.

use crash_sim::tune::{apply, parse_tune, points, Tune};
use std::fs::{self, File, OpenOptions};
use std::io::{BufRead, BufReader, Write};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::Instant;

const USAGE: &str = "crash-sim tune --body <file.bsn> --out <dir> [--tune <file>]... \
    [--set <keyword>.<name>.<field>=<value>]... [--jobs <1..=16>] [--show <prefix>]... \
    -- <run flags...>";

struct Args {
    body: PathBuf,
    out: PathBuf,
    files: Vec<String>,
    sets: Vec<String>,
    jobs: usize,
    show: Vec<String>,
    flags: Vec<String>,
}

fn args(args: &[String]) -> Result<Args, String> {
    let mut body = None;
    let mut out = None;
    let mut files = Vec::new();
    let mut sets = Vec::new();
    let mut jobs = 4;
    let mut show = Vec::new();
    let mut flags = Vec::new();
    let mut index = 0;
    while let Some(flag) = args.get(index) {
        if flag == "--" {
            flags.extend_from_slice(&args[index + 1..]);
            break;
        }
        if !matches!(
            flag.as_str(),
            "--body" | "--out" | "--tune" | "--set" | "--jobs" | "--show"
        ) {
            return Err(format!("unknown tune flag {flag:?}\n{USAGE}"));
        }
        let value = args
            .get(index + 1)
            .filter(|v| !v.is_empty() && !v.chars().any(char::is_control))
            .ok_or_else(|| format!("missing or invalid value for {flag}\n{USAGE}"))?;
        match flag.as_str() {
            "--body" => {
                if body.is_some() {
                    return Err("duplicate --body".into());
                }
                body = Some(PathBuf::from(value));
            }
            "--out" => {
                if out.is_some() {
                    return Err("duplicate --out".into());
                }
                out = Some(PathBuf::from(value));
            }
            "--tune" => files.push(value.clone()),
            "--set" => sets.push(value.clone()),
            "--jobs" => {
                jobs = value
                    .parse::<usize>()
                    .ok()
                    .filter(|n| (1..=16).contains(n))
                    .ok_or_else(|| "bad --jobs: expected integer 1..=16".to_owned())?;
            }
            "--show" => show.push(value.clone()),
            _ => unreachable!(),
        }
        index += 2;
    }
    if flags.iter().any(|v| v == "--body") {
        return Err("run flags may not replace the point's --body".into());
    }
    if flags.iter().any(|v| v.chars().any(char::is_control)) {
        return Err("run flags may not contain control characters".into());
    }
    Ok(Args {
        body: body.ok_or_else(|| format!("missing --body\n{USAGE}"))?,
        out: out.ok_or_else(|| format!("missing --out\n{USAGE}"))?,
        files,
        sets,
        jobs,
        show,
        flags,
    })
}

fn read(path: &str) -> Result<String, String> {
    // Windows verbatim paths require backslashes, including paths resolved by the
    // portable pure parser from a canonical \\?\... top-level filename.
    let native = if cfg!(windows) {
        path.replace('/', "\\")
    } else {
        path.to_owned()
    };
    fs::read_to_string(native).map_err(|e| format!("{path}: {e}"))
}

fn load(options: &Args) -> Result<Tune, String> {
    let mut tune = Tune::default();
    for file in &options.files {
        let text = read(file)?;
        let parsed = parse_tune(&text, file, &read).map_err(|e| e.to_string())?;
        tune.append(parsed).map_err(|e| e.to_string())?;
    }
    // All file edits precede all --set edits, retaining order within each category.
    for (i, value) in options.sets.iter().enumerate() {
        let (key, number) = value
            .split_once('=')
            .ok_or_else(|| format!("bad --set {value:?}: expected keyword.name.field=value"))?;
        let parts: Vec<_> = key.split('.').collect();
        if parts.len() != 3
            || parts.iter().any(|p| p.is_empty())
            || value.chars().any(char::is_whitespace)
            || number.is_empty()
        {
            return Err(format!(
                "bad --set {value:?}: expected keyword.name.field=value"
            ));
        }
        let text = format!("tune 1\nset {} {} {} {number}\n", parts[0], parts[1], parts[2]);
        let parsed =
            parse_tune(&text, &format!("<--set {}>", i + 1), &read).map_err(|e| e.to_string())?;
        tune.append(parsed).map_err(|e| e.to_string())?;
    }
    Ok(tune)
}

struct Point {
    values: Vec<(String, f64)>,
    invalid: Option<String>,
}

// All tune/rewrite errors are found before creating files or starting any children.
fn prepare(body: &str, tune: &Tune) -> Result<Vec<Point>, String> {
    points(tune)
        .into_iter()
        .map(|values| {
            let text = apply(body, tune, &values).map_err(|e| e.to_string())?;
            let invalid = crash_sim::format::parse(&text)
                .err()
                .map(|e| format!("values {values:?}: {e}"));
            Ok(Point { values, invalid })
        })
        .collect()
}

fn create(path: &Path) -> Result<File, String> {
    OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)
        .map_err(|e| format!("{}: {e}", path.display()))
}

fn cell(text: &str) -> String {
    text.replace('\\', "\\\\")
        .replace('\t', "\\t")
        .replace('\r', "\\r")
        .replace('\n', "\\n")
}

struct Row {
    exit: String,
    wall: f64,
    summary: String,
}

fn run_point(
    index: usize,
    point: &Point,
    options: &Args,
    exe: &Path,
    mut log: File,
) -> Result<Row, String> {
    let stem = format!("point-{:03}", index + 1);
    let body_path = options.out.join(format!("{stem}.bsn"));
    if let Some(reason) = &point.invalid {
        writeln!(log, "invalid: {reason}").map_err(|e| format!("{stem}: {e}"))?;
        return Ok(Row {
            exit: "invalid".into(),
            wall: 0.0,
            summary: reason.clone(),
        });
    }
    let stderr = log.try_clone().map_err(|e| format!("{stem}: {e}"))?;
    let start = Instant::now();
    let mut child = Command::new(exe)
        .arg("--body")
        .arg(&body_path)
        .args(&options.flags)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::from(stderr))
        .spawn()
        .map_err(|e| format!("{stem}: cannot start child: {e}"))?;
    // stderr goes straight to the shared log handle. Drain stdout while the child runs,
    // teeing it to that same file; only stdout participates in --show selection.
    let captured = (|| -> Result<String, String> {
        let stdout = child
            .stdout
            .take()
            .ok_or_else(|| format!("{stem}: missing stdout pipe"))?;
        let mut stdout = BufReader::new(stdout);
        let mut bytes = Vec::new();
        let mut selected = Vec::new();
        let mut last = String::new();
        loop {
            bytes.clear();
            let count = stdout
                .read_until(b'\n', &mut bytes)
                .map_err(|e| format!("{stem}: stdout read failed: {e}"))?;
            if count == 0 {
                break;
            }
            log.write_all(&bytes)
                .map_err(|e| format!("{stem}: log write failed: {e}"))?;
            let line = std::str::from_utf8(&bytes)
                .map_err(|e| format!("{stem}: stdout is not UTF-8: {e}"))?
                .trim_end_matches(['\r', '\n']);
            if options.show.is_empty() {
                last = line.to_owned();
            } else if options.show.iter().any(|prefix| line.starts_with(prefix)) {
                selected.push(line.to_owned());
            }
        }
        log.flush()
            .map_err(|e| format!("{stem}: log flush failed: {e}"))?;
        Ok(if options.show.is_empty() {
            last
        } else {
            selected.join(" | ")
        })
    })();
    // On capture failure reap the child too, without hiding the original failure.
    let summary = match captured {
        Ok(text) => text,
        Err(error) => {
            let _ = child.kill();
            let _ = child.wait();
            return Err(error);
        }
    };
    let status = match child.wait() {
        Ok(status) => status,
        Err(error) => {
            let _ = child.kill();
            let _ = child.wait();
            return Err(format!("{stem}: child wait failed: {error}"));
        }
    };
    Ok(Row {
        exit: status
            .code()
            .map_or_else(|| "signal".into(), |code| code.to_string()),
        wall: start.elapsed().as_secs_f64(),
        summary,
    })
}

fn execute(options: &Args) -> Result<i32, String> {
    let tune = load(options)?;
    let body =
        fs::read_to_string(&options.body).map_err(|e| format!("{}: {e}", options.body.display()))?;
    let prepared = prepare(&body, &tune)?;
    let exe = std::env::current_exe().map_err(|e| format!("current_exe: {e}"))?;
    fs::create_dir_all(&options.out).map_err(|e| format!("{}: {e}", options.out.display()))?;
    // Reserve all output names before running anything; never overwrite inputs, old
    // results, or existing file links. Use a fresh output directory for each invocation.
    let mut results = create(&options.out.join("results.tsv"))?;
    let mut logs = Vec::with_capacity(prepared.len());
    for (index, point) in prepared.iter().enumerate() {
        let path = options.out.join(format!("point-{:03}.bsn", index + 1));
        // Recompute from the same immutable inputs after preflight rather than retaining
        // a complete body string for every grid point simultaneously.
        let text = apply(&body, &tune, &point.values).map_err(|e| e.to_string())?;
        create(&path)?
            .write_all(text.as_bytes())
            .map_err(|e| format!("{}: {e}", path.display()))?;
        logs.push(create(
            &options.out.join(format!("point-{:03}.txt", index + 1)),
        )?);
    }
    let mut logs = logs.into_iter();
    let mut rows = Vec::with_capacity(prepared.len());
    // Bounded batches: every handle is joined even when another worker fails.
    for (batch_index, batch) in prepared.chunks(options.jobs).enumerate() {
        let batch_rows = std::thread::scope(|scope| {
            let handles: Vec<_> = batch
                .iter()
                .zip(logs.by_ref())
                .enumerate()
                .map(|(offset, (point, log))| {
                    let exe = &exe;
                    std::thread::Builder::new()
                        .spawn_scoped(scope, move || {
                            run_point(batch_index * options.jobs + offset, point, options, exe, log)
                        })
                        .map_err(|e| format!("cannot start tune worker: {e}"))
                })
                .collect();
            handles
                .into_iter()
                .map(|handle| match handle {
                    Ok(handle) => handle
                        .join()
                        .unwrap_or_else(|_| Err("tune worker panicked".into())),
                    Err(error) => Err(error),
                })
                .collect::<Vec<_>>()
        });
        for row in batch_rows {
            rows.push(row?);
        }
    }
    let mut table = String::from("point");
    if let Some(point) = prepared.first() {
        for (key, _) in &point.values {
            table.push('\t');
            table.push_str(&cell(key));
        }
    }
    table.push_str("\texit\twall_s\tstdout\n");
    for (index, (point, row)) in prepared.iter().zip(&rows).enumerate() {
        table.push_str(&(index + 1).to_string());
        for (_, value) in &point.values {
            table.push_str(&format!("\t{value}"));
        }
        table.push_str(&format!(
            "\t{}\t{:.2}\t{}\n",
            row.exit,
            row.wall,
            cell(&row.summary)
        ));
    }
    results
        .write_all(table.as_bytes())
        .and_then(|()| results.flush())
        .map_err(|e| format!("results.tsv: {e}"))?;
    let stdout = std::io::stdout();
    let mut stdout = stdout.lock();
    stdout
        .write_all(table.as_bytes())
        .and_then(|()| stdout.flush())
        .map_err(|e| format!("table stdout: {e}"))?;
    Ok(i32::from(prepared.iter().any(|p| p.invalid.is_some())))
}

pub fn run_tune_cli(arguments: &[String]) -> i32 {
    match args(arguments).and_then(|options| execute(&options)) {
        Ok(code) => code,
        Err(error) => {
            eprintln!("crash-sim tune: {error}");
            2
        }
    }
}
