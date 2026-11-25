use crash_sim::shell::{self, Shell};
use std::fs;
use std::io::{self, Cursor, Write};
use std::path::{Path, PathBuf};
mod run_tmp;

fn dir(name: &str) -> PathBuf {
    let root = run_tmp::run_tmp();
    fs::create_dir_all(&root).unwrap();
    let root = root.canonicalize().unwrap();
    let d = root.join(format!("shell-core-{name}"));
    assert_eq!(d.parent(), Some(root.as_path()));
    match fs::symlink_metadata(&d) {
        Ok(meta) => {
            assert!(
                !meta.file_type().is_symlink(),
                "test folder must not be a link"
            );
            assert_eq!(d.canonicalize().unwrap(), d);
            fs::remove_dir_all(&d).unwrap();
        }
        Err(e) if e.kind() == io::ErrorKind::NotFound => {}
        Err(e) => panic!("cannot inspect test folder {}: {e}", d.display()),
    }
    fs::create_dir_all(&d).unwrap();
    d
}

fn args(d: &Path) -> Vec<String> {
    [
        "--shell",
        "--data",
        d.to_str().unwrap(),
        "--shell-track",
        "scenarios/tracks/oval.track",
        "--shell-track",
        "scenarios/tracks/chicane.track",
        "--shell-car",
        "scenarios/car_basic.bsn",
        "--today",
        "2026-11-20",
    ]
    .into_iter()
    .map(str::to_owned)
    .collect()
}

fn run(args: &[String], events: &str) -> (i32, String, String) {
    let (mut out, mut err) = (Vec::new(), Vec::new());
    let code = shell::run(args, &mut Cursor::new(events), &mut out, &mut err);
    (
        code,
        String::from_utf8(out).unwrap(),
        String::from_utf8(err).unwrap(),
    )
}

fn good(args: &[String], events: &str) -> String {
    let (code, out, err) = run(args, events);
    assert_eq!(code, 0, "{out}\n{err}");
    assert!(err.is_empty(), "{err}");
    out
}

fn start(args: &[String]) -> Shell {
    Shell::start(shell::parse_args(args).unwrap()).0
}

fn event(shell: &mut Shell, input: &str, expected: &[&str]) {
    assert_eq!(shell.event(input), expected, "event {input:?}");
}

const SETTINGS: &str = "\
settings 1
width = 1280
height = 720
fullscreen = false
volume = 55
units = mph
key.throttle = up
key.brake = down
key.left = left
key.right = right
key.pause = escape
key.reset = r
";
const BEST: &str = "save 2\nbest oval 83.456 car_basic 2026-11-20\n";
const START_RACE: &str = "action start_race track=oval track_file=scenarios/tracks/oval.track car=car_basic car_file=scenarios/car_basic.bsn";

#[test]
fn shell_core_menu_path_matches_check_3_2() {
    let d = dir("menu");
    let args = args(&d);
    let a = good(
        &args,
        "\
select
select
select
finished 83456
back
down
select
set volume 55
set units mph
select
back
down
select
",
    );
    assert_eq!(
        a,
        "\
shell tracks=2 cars=1
settings width=1280 height=720 fullscreen=false volume=80 units=kmh source=defaults
state main cursor=0
state track_select cursor=0
state car_select track=oval cursor=0
state race track=oval car=car_basic
action start_race track=oval track_file=scenarios/tracks/oval.track car=car_basic car_file=scenarios/car_basic.bsn
state results track=oval car=car_basic
saved best track=oval seconds=83.456
state main cursor=0
state main cursor=1
state settings
settings width=1280 height=720 fullscreen=false volume=55 units=kmh source=unsaved
settings width=1280 height=720 fullscreen=false volume=55 units=mph source=unsaved
state settings
action save_settings
saved settings
state main cursor=1
state main cursor=2
state quit
action quit
"
    );
    assert_eq!(fs::read(d.join("settings.txt")).unwrap(), SETTINGS.as_bytes());
    assert_eq!(fs::read(d.join("best.save")).unwrap(), BEST.as_bytes());
    let b = good(&args, "down\nselect\nback\ndown\nselect\n");
    assert_eq!(
        b,
        "\
shell tracks=2 cars=1
settings width=1280 height=720 fullscreen=false volume=55 units=mph source=file
best track=oval seconds=83.456 car=car_basic date=2026-11-20
state main cursor=0
state main cursor=1
state settings
state main cursor=1
state main cursor=2
state quit
action quit
"
    );
    assert_eq!(fs::read(d.join("settings.txt")).unwrap(), SETTINGS.as_bytes());
    assert_eq!(fs::read(d.join("best.save")).unwrap(), BEST.as_bytes());
    assert!(!d.join(".shell-probe").exists());
}

#[test]
fn shell_core_refuses_corrupt_files_and_leaves_them() {
    let d = dir("corrupt");
    let args = args(&d);
    let settings = "settings 1\nvolume = 101\n";
    let best = "save 2\nbest oval abc car_basic 2026-11-20\n";
    fs::write(d.join("settings.txt"), settings).unwrap();
    fs::write(d.join("best.save"), best).unwrap();
    let out = good(&args, "select\nselect\nselect\nfinished 70000\nback\ndown\nselect\nselect\nback\ndown\nselect\n");
    for prefix in [
        "warn settings refused: settings line 2: ",
        "warn save refused: line 2",
        "warn settings not saved: ",
        "warn best not saved: ",
    ] {
        assert_eq!(
            out.lines().filter(|l| l.starts_with(prefix)).count(),
            1,
            "{out}"
        );
    }
    let defaults = "settings width=1280 height=720 fullscreen=false volume=80 units=kmh source=defaults";
    assert_eq!(out.lines().filter(|l| *l == defaults).count(), 1);
    assert!(out.ends_with("state quit\naction quit\n"), "{out}");
    assert_eq!(fs::read(d.join("settings.txt")).unwrap(), settings.as_bytes());
    assert_eq!(fs::read(d.join("best.save")).unwrap(), best.as_bytes());

    // Exercise the public lower-level entry point, including changes made after refusal.
    let mut shell = start(&args);
    shell.event("down");
    shell.event("select");
    event(
        &mut shell,
        "set volume 55",
        &["settings width=1280 height=720 fullscreen=false volume=55 units=kmh source=unsaved"],
    );
    event(
        &mut shell,
        "select",
        &[
            "state settings",
            "action save_settings",
            "warn settings not saved: settings.txt was refused at start and is left unchanged",
        ],
    );
    assert_eq!(fs::read_to_string(d.join("settings.txt")).unwrap(), settings);
    shell.event("back");
    shell.event("up");
    for event in ["select", "select", "select"] {
        shell.event(event);
    }
    event(
        &mut shell,
        "finished 60000",
        &[
            "state results track=oval car=car_basic",
            "warn best not saved: best.save was refused at start and is left unchanged",
        ],
    );
    assert_eq!(fs::read_to_string(d.join("best.save")).unwrap(), best);
    shell.event("select");
    event(
        &mut shell,
        "finished 61000",
        &["state results track=oval car=car_basic"],
    );

    // Hostile file formats remain refused rather than being partially accepted.
    for (settings, best) in [
        (
            "settings 1\nvolume = 55\nvolume = 56\n".into(),
            "save 2\nbest oval 1 c 2026-11-20\nbest oval 2 c 2026-11-20\n",
        ),
        (
            format!("settings 1\nvolume = {}\n", "9".repeat(100_000)),
            "save 2\n# not a save comment\n",
        ),
    ] {
        fs::write(d.join("settings.txt"), &settings).unwrap();
        fs::write(d.join("best.save"), best).unwrap();
        let out = good(&args, "");
        assert!(out.contains("warn settings refused: "), "{out}");
        assert!(out.contains("warn save refused: "), "{out}");
        assert_eq!(fs::read_to_string(d.join("settings.txt")).unwrap(), settings);
        assert_eq!(fs::read_to_string(d.join("best.save")).unwrap(), best);
    }
}

#[test]
fn shell_core_set_rules() {
    let d = dir("set");
    let args = args(&d);
    let cases = [
        ("set volume 55", "not on the settings screen"),
        ("set speed 3", "unknown key speed"),
        ("set width abc", "width needs an integer, got abc"),
        ("set fullscreen yes", "fullscreen needs true or false, got yes"),
        ("set units kph", "units needs kmh or mph, got kph"),
        (
            "set volume 101",
            "invalid settings: volume must be 0..=100, got 101",
        ),
        ("set volume", "expected set <key> <value>"),
        ("set width 99999999999", "width needs an integer, got 99999999999"),
        ("set width 4294967296", "width needs an integer, got 4294967296"),
        ("set width +320", "width needs an integer, got +320"),
        ("set volume -1", "volume needs an integer, got -1"),
        ("set volume NaN", "volume needs an integer, got NaN"),
        ("set volume 1\r", "volume needs an integer, got 1\r"),
        ("set  volume 1", "expected set <key> <value>"),
        ("set volume ", "expected set <key> <value>"),
        ("set", "expected set <key> <value>"),
    ];
    let mut shell = start(&args);
    for (i, (event, message)) in cases.iter().enumerate() {
        if i == 1 {
            shell.event("down");
            shell.event("select");
        }
        assert_eq!(shell.event(event), [format!("warn set refused: {message}")]);
    }
    event(
        &mut shell,
        "set width 1920",
        &["settings width=1920 height=720 fullscreen=false volume=80 units=kmh source=unsaved"],
    );
    let input = "set volume 55\ndown\nselect\nset speed 3\nset width abc\nset fullscreen yes\nset units kph\nset volume 101\nset volume\nset width 99999999999\nset width 1920\n";
    let out = good(&args, input);
    let mut expected = vec![
        "warn set refused: not on the settings screen".to_owned(),
        "state main cursor=1".into(),
        "state settings".into(),
    ];
    expected.extend(cases[1..8].iter().map(|(_, m)| format!("warn set refused: {m}")));
    expected.push("settings width=1920 height=720 fullscreen=false volume=80 units=kmh source=unsaved".into());
    assert_eq!(out.lines().skip(3).collect::<Vec<_>>(), expected);

    // A failed replacement leaves the old settings and the blocking directory intact.
    fs::write(d.join("settings.txt"), SETTINGS).unwrap();
    fs::create_dir(d.join("settings.txt.tmp")).unwrap();
    let out = good(&args, "down\nselect\nset volume 56\nselect\n");
    assert!(out.contains("warn settings not saved: "), "{out}");
    assert_eq!(fs::read_to_string(d.join("settings.txt")).unwrap(), SETTINGS);
    assert!(d.join("settings.txt.tmp").is_dir());
}

#[test]
fn shell_core_usage_errors() {
    let d = dir("usage");
    let common = args(&d);
    let mut cases = Vec::new();
    for flag in ["--shell-track", "--shell-car", "--data", "--today", "--shell"] {
        let mut a = Vec::new();
        let mut i = 0;
        while i < common.len() {
            if common[i] == flag {
                i += if flag == "--shell" { 1 } else { 2 };
            } else {
                a.push(common[i].clone());
                i += 1;
            }
        }
        cases.push((a, flag));
    }
    for value in ["2026-02-30", "unknown", "2026-11-20\nbest injected 1 c unknown"] {
        let mut a = common.clone();
        *a.last_mut().unwrap() = value.into();
        cases.push((a, "--today"));
    }
    let mut duplicate = common.clone();
    duplicate[4] = "a/oval.track".into();
    duplicate[6] = "b/oval.track".into();
    cases.push((duplicate, "--shell-track"));
    for (extra, flag) in [
        (vec!["--body", "x"], "--body"),
        (vec!["--data"], "--data"),
        (vec!["--data", "--today"], "--data"),
        (vec!["--shell-track", "a/oval.track"], "--shell-track"),
        (vec!["--shell-car", "b/car_basic.bsn"], "--shell-car"),
        (vec!["--today", "2026-11-20"], "--today"),
        (vec!["--data", "other"], "--data"),
        (vec!["--shell"], "--shell"),
        (vec!["--shell-track", "/"], "--shell-track"),
        (vec!["--shell-car", ""], "--shell-car"),
        (vec!["--shell-track", "a\nb.track"], "--shell-track"),
    ] {
        let mut a = common.clone();
        a.extend(extra.into_iter().map(str::to_owned));
        cases.push((a, flag));
    }
    for (a, flag) in cases {
        let (code, out, err) = run(&a, "");
        assert_eq!(code, 2, "args={a:?}, err={err}");
        assert!(out.is_empty(), "args={a:?}, out={out}");
        assert!(err.starts_with("shell: usage: ") && err.contains(flag), "{err}");
    }
    let mut reordered = vec![
        "--today",
        "2024-02-29",
        "--shell-car",
        "z.bsn",
        "--shell-track",
        "b.track",
        "--shell-track",
        "a.track",
        "--data",
    ]
    .into_iter()
    .map(str::to_owned)
    .collect::<Vec<_>>();
    reordered.push(d.to_str().unwrap().into());
    reordered.push("--shell".into());
    let parsed = shell::parse_args(&reordered).unwrap();
    assert_eq!(parsed.tracks, ["b.track", "a.track"]);
    // Path::file_stem handles native Windows separators and drive letters.
    if cfg!(windows) {
        let mut a = common.clone();
        a.extend(["--shell-track".into(), r"C:\tracks\oval.track".into()]);
        assert!(shell::parse_args(&a).unwrap_err().contains("--shell-track"));
    }
}

#[test]
fn shell_core_unwritable_data_folder_warns_and_runs() {
    let d = dir("unwritable");
    fs::write(d.join("file.txt"), b"keep").unwrap();
    let data = d.join("file.txt").join("data");
    let out = good(&args(&data), "down\nselect\nselect\nback\ndown\nselect\n");
    let warning = format!("warn data folder not writable: {}", data.display());
    assert_eq!(out.lines().nth(1), Some(warning.as_str()));
    assert_eq!(out.lines().filter(|l| *l == warning).count(), 1);
    assert!(out.contains("warn settings not saved: "), "{out}");
    assert!(out.ends_with("state quit\naction quit\n"), "{out}");
    assert_eq!(fs::read(d.join("file.txt")).unwrap(), b"keep");
}

#[test]
fn shell_core_finished_rules() {
    let d = dir("finished");
    let args = args(&d);
    let out = good(&args, "finished 5000\nselect\nselect\nselect\nfinished none\n");
    assert_eq!(
        out.lines().skip(3).take(2).collect::<Vec<_>>(),
        ["state main cursor=0", "warn finished ignored: not in a race"],
    );
    assert!(
        out.ends_with("state results track=oval car=car_basic\n"),
        "{out}"
    );
    assert!(!out.contains("saved "), "{out}");
    assert!(!d.join("best.save").exists());

    let mut shell = start(&args);
    for event in ["select", "select", "select"] {
        shell.event(event);
    }
    event(
        &mut shell,
        "finished 70000",
        &[
            "state results track=oval car=car_basic",
            "saved best track=oval seconds=70.000",
        ],
    );
    let best = fs::read(d.join("best.save")).unwrap();
    assert_eq!(best, b"save 2\nbest oval 70 car_basic 2026-11-20\n");
    shell.event("select");
    event(
        &mut shell,
        "finished 71000",
        &["state results track=oval car=car_basic"],
    );
    assert_eq!(fs::read(d.join("best.save")).unwrap(), best);
    for event in [
        "finished 0",
        "finished 1.5",
        "finished -3",
        "finished 1234567890",
        "finished +5",
        "finished 000000000",
        "finished 1 ",
        "finished \u{661}",
        "finished NaN",
    ] {
        assert_eq!(
            shell.event(event),
            [format!("warn event refused: unknown event {event:?}")]
        );
    }
    shell.event("select");
    event(
        &mut shell,
        "pause",
        &["state paused track=oval car=car_basic cursor=0"],
    );
    event(
        &mut shell,
        "finished 65000",
        &[
            "state results track=oval car=car_basic",
            "saved best track=oval seconds=65.000",
        ],
    );
    event(
        &mut shell,
        "select",
        &["state race track=oval car=car_basic", START_RACE],
    );
    event(&mut shell, "back", &["state main cursor=0", "action abort_race"]);

    // Integer boundary references, not derived from the implementation's floating output.
    fs::remove_file(d.join("best.save")).unwrap();
    let out = good(&args, "select\nselect\nselect\nfinished 999999999\n");
    assert!(
        out.contains("saved best track=oval seconds=999999.999\n"),
        "{out}"
    );
    assert_eq!(
        fs::read_to_string(d.join("best.save")).unwrap(),
        "save 2\nbest oval 999999.999 car_basic 2026-11-20\n"
    );
    let out = good(&args, "select\nselect\nselect\nfinished 1\n");
    assert!(out.ends_with("saved best track=oval seconds=0.001\n"), "{out}");
    assert_eq!(
        fs::read_to_string(d.join("best.save")).unwrap(),
        "save 2\nbest oval 0.001 car_basic 2026-11-20\n"
    );
    // Failure after recording a faster lap preserves the disk file and reports the error.
    fs::create_dir(d.join("best.save.tmp")).unwrap();
    fs::write(
        d.join("best.save"),
        b"save 2\nbest oval 70 car_basic 2026-11-20\n",
    )
    .unwrap();
    let out = good(&args, "select\nselect\nselect\nfinished 60000\n");
    assert!(out.contains("warn best not saved: "), "{out}");
    assert_eq!(
        fs::read_to_string(d.join("best.save")).unwrap(),
        "save 2\nbest oval 70 car_basic 2026-11-20\n"
    );
    assert!(d.join("best.save.tmp").is_dir());
}

struct Output {
    bytes: Vec<u8>,
    flushes: usize,
    fail_flush: bool,
    fail_write: bool,
}

impl Write for Output {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        if self.fail_write {
            return Err(io::Error::other("injected write failure"));
        }
        self.bytes.extend_from_slice(bytes);
        Ok(bytes.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        self.flushes += 1;
        if self.fail_flush {
            Err(io::Error::other("injected flush failure"))
        } else {
            Ok(())
        }
    }
}

#[test]
fn shell_core_end_of_input_and_unknown_lines() {
    let d = dir("eof");
    let args = args(&d);
    let out = good(&args, "select\r\njump\n\nback\n");
    assert_eq!(
        out.lines().skip(3).collect::<Vec<_>>(),
        [
            "state track_select cursor=0",
            "warn event refused: unknown event \"jump\"",
            "warn event refused: unknown event \"\"",
            "state main cursor=0",
        ]
    );
    let mut shell = start(&args);
    assert!(!shell.finished());
    shell.event("down");
    shell.event("down");
    assert_eq!(shell.event("select"), ["state quit", "action quit"]);
    assert!(shell.finished());
    let huge = "x".repeat(100_000);
    assert_eq!(
        shell.event(&huge),
        [format!("warn event refused: unknown event {huge:?}")]
    );

    let mut input = Cursor::new("down\ndown\nselect\nunread\n");
    let mut output = Output {
        bytes: Vec::new(),
        flushes: 0,
        fail_flush: false,
        fail_write: false,
    };
    let mut err = Vec::new();
    assert_eq!(shell::run(&args, &mut input, &mut output, &mut err), 0);
    assert_eq!(input.position(), b"down\ndown\nselect\n".len() as u64);
    assert_eq!(
        output.flushes,
        output.bytes.iter().filter(|b| **b == b'\n').count()
    );
    assert!(err.is_empty());
    output.fail_flush = true;
    assert_eq!(
        shell::run(&args, &mut Cursor::new(""), &mut output, &mut err),
        1
    );
    output.fail_flush = false;
    output.fail_write = true;
    assert_eq!(
        shell::run(&args, &mut Cursor::new(""), &mut output, &mut err),
        1
    );
    let mut failing = io::BufReader::new(io::Read::take(io::repeat(0xff), 1));
    assert_eq!(shell::run(&args, &mut failing, &mut Vec::new(), &mut err), 0);

    fs::write(
        d.join("best.save"),
        "save 2\nbest z 1 c unknown\nbest a 2 c 2026-11-20\n",
    )
    .unwrap();
    let out = good(&args, "");
    assert_eq!(
        out.lines().skip(2).collect::<Vec<_>>(),
        [
            "best track=a seconds=2.000 car=c date=2026-11-20",
            "best track=z seconds=1.000 car=c date=unknown",
            "state main cursor=0",
        ]
    );
}
