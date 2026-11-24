use std::fs;
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::process::{Command, Output, Stdio};

fn exe() -> std::path::PathBuf {
    if let Some(p) = std::env::var_os("CARGO_BIN_EXE_crash-sim") {
        return std::path::PathBuf::from(p);
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
    assert!(bin.is_file(), "missing binary {}", bin.display());
    bin
}

fn dir(name: &str) -> PathBuf {
    let root = PathBuf::from(env!("CARGO_TARGET_TMPDIR"));
    fs::create_dir_all(&root).unwrap();
    let root = root.canonicalize().unwrap();
    let d = root.join(format!("shell-bin-{name}"));
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

fn run(args: &[String], events: &str) -> Output {
    let mut child = Command::new(exe())
        .args(args)
        .current_dir(env!("CARGO_MANIFEST_DIR"))
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    {
        let mut input = child.stdin.take().unwrap();
        input.write_all(events.as_bytes()).unwrap();
    }
    let out = child.wait_with_output().unwrap();
    assert!(!out.stdout.contains(&b'\r'), "{out:?}");
    out
}

fn good(args: &[String], events: &str) -> String {
    let out = run(args, events);
    assert_eq!(out.status.code(), Some(0), "{out:?}");
    assert!(out.stderr.is_empty(), "{out:?}");
    String::from_utf8(out.stdout).unwrap()
}

fn files(d: &Path) {
    let mut names = fs::read_dir(d)
        .unwrap()
        .map(|entry| entry.unwrap().file_name())
        .collect::<Vec<_>>();
    names.sort();
    assert_eq!(names, ["best.save", "settings.txt"]);
    assert!(d.join("best.save").is_file());
    assert!(d.join("settings.txt").is_file());
}

fn once(out: &str, line: &str) {
    assert_eq!(out.lines().filter(|l| *l == line).count(), 1, "{out}");
}

const A: &str = "select\nselect\nselect\nfinished 83456\nback\ndown\nselect\nset volume 55\nset units mph\nselect\nback\ndown\nselect\n";
const B: &str = "down\nselect\nback\ndown\nselect\n";
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
const OUT_A: &str = "\
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
";
const OUT_B: &str = "\
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
";

#[test]
fn shell_scripted_menu_path_starts_a_race_and_saves_the_lap() {
    let d = dir("menu");
    assert_eq!(good(&args(&d), A).as_bytes(), OUT_A.as_bytes());
    assert_eq!(
        fs::read(d.join("settings.txt")).unwrap(),
        SETTINGS.as_bytes()
    );
    assert_eq!(fs::read(d.join("best.save")).unwrap(), BEST.as_bytes());
    files(&d);
}

#[test]
fn shell_settings_and_best_survive_a_restart() {
    let d = dir("restart");
    assert_eq!(good(&args(&d), A).as_bytes(), OUT_A.as_bytes());
    let settings = fs::read(d.join("settings.txt")).unwrap();
    let best = fs::read(d.join("best.save")).unwrap();
    assert_eq!(settings, SETTINGS.as_bytes());
    assert_eq!(best, BEST.as_bytes());
    assert_eq!(good(&args(&d), B).as_bytes(), OUT_B.as_bytes());
    assert_eq!(fs::read(d.join("settings.txt")).unwrap(), settings);
    assert_eq!(fs::read(d.join("best.save")).unwrap(), best);
}

#[test]
fn shell_refuses_corrupt_settings_and_save_and_leaves_them_unchanged() {
    let d = dir("corrupt");
    let settings = "settings 1\nvolume = 101\n";
    let best = "save 2\nbest oval abc car_basic 2026-11-20\n";
    fs::write(d.join("settings.txt"), settings).unwrap();
    fs::write(d.join("best.save"), best).unwrap();
    let result = run(
        &args(&d),
        "select\nselect\nselect\nfinished 70000\nback\ndown\nselect\nselect\nback\ndown\nselect\n",
    );
    assert_eq!(result.status.code(), Some(0), "{result:?}");
    assert!(!String::from_utf8_lossy(&result.stderr).contains("panicked"));
    let out = String::from_utf8(result.stdout).unwrap();
    for prefix in [
        "warn settings refused: settings line 2: ",
        "warn save refused: ",
        "warn settings not saved: ",
        "warn best not saved: ",
    ] {
        let lines = out
            .lines()
            .filter(|line| line.starts_with(prefix))
            .collect::<Vec<_>>();
        assert_eq!(lines.len(), 1, "{out}");
        if prefix == "warn save refused: " {
            assert!(lines[0].contains("line 2"), "{out}");
        }
    }
    once(
        &out,
        "settings width=1280 height=720 fullscreen=false volume=80 units=kmh source=defaults",
    );
    assert!(out.ends_with("state quit\naction quit\n"), "{out}");
    assert_eq!(
        fs::read(d.join("settings.txt")).unwrap(),
        settings.as_bytes()
    );
    assert_eq!(fs::read(d.join("best.save")).unwrap(), best.as_bytes());
    files(&d);
}

#[test]
fn shell_set_refuses_bad_values_by_name() {
    let d = dir("set");
    let out = good(&args(&d), "set volume 55\ndown\nselect\nset volume 101\nset speed 3\nset units kph\nset width abc\nset width 1920\n");
    assert_eq!(
        out.as_bytes(),
        b"shell tracks=2 cars=1\n\
settings width=1280 height=720 fullscreen=false volume=80 units=kmh source=defaults\n\
state main cursor=0\n\
warn set refused: not on the settings screen\n\
state main cursor=1\n\
state settings\n\
warn set refused: invalid settings: volume must be 0..=100, got 101\n\
warn set refused: unknown key speed\n\
warn set refused: units needs kmh or mph, got kph\n\
warn set refused: width needs an integer, got abc\n\
settings width=1920 height=720 fullscreen=false volume=80 units=kmh source=unsaved\n"
    );
    assert!(!d.join("settings.txt").exists());
}

#[test]
fn shell_needs_at_least_one_track_and_car() {
    let d = dir("usage");
    let never = d.join("never");
    for removed in [Some("--shell-track"), Some("--shell-car"), None] {
        let common = args(&never);
        let mut a = Vec::new();
        let mut i = 0;
        while i < common.len() {
            if Some(common[i].as_str()) == removed {
                i += 2;
            } else {
                a.push(common[i].clone());
                i += 1;
            }
        }
        if removed.is_none() {
            a.extend(["--ai".to_owned(), "2".to_owned()]);
        }
        let out = run(&a, "select\n");
        assert_eq!(out.status.code(), Some(2), "{out:?}");
        assert!(out.stdout.is_empty(), "{out:?}");
        assert!(out.stderr.starts_with(b"shell: usage: "), "{out:?}");
        assert!(!never.exists());
    }
}

#[test]
fn shell_never_overwrites_a_refused_file() {
    let events = "down\nselect\nset volume 55\nselect\nback\nup\nselect\nselect\nselect\nfinished 70000\nback\ndown\ndown\nselect\n";
    let bad_settings = "settings 1\nvolume = 101\n";
    let bad_best = "save 2\nbest oval abc car_basic 2026-11-20\n";
    let d = dir("refused-settings");
    fs::write(d.join("settings.txt"), bad_settings).unwrap();
    fs::write(d.join("best.save"), BEST).unwrap();
    let out = good(&args(&d), events);
    once(
        &out,
        "warn settings not saved: settings.txt was refused at start and is left unchanged",
    );
    once(&out, "saved best track=oval seconds=70.000");
    assert_eq!(
        fs::read(d.join("settings.txt")).unwrap(),
        bad_settings.as_bytes()
    );
    assert_eq!(
        fs::read(d.join("best.save")).unwrap(),
        b"save 2\nbest oval 70 car_basic 2026-11-20\n"
    );
    files(&d);

    let d = dir("refused-best");
    fs::write(d.join("best.save"), bad_best).unwrap();
    let out = good(&args(&d), events);
    once(
        &out,
        "warn best not saved: best.save was refused at start and is left unchanged",
    );
    assert_eq!(fs::read(d.join("best.save")).unwrap(), bad_best.as_bytes());
    assert_eq!(
        fs::read(d.join("settings.txt")).unwrap(),
        SETTINGS.replace("units = mph", "units = kmh").as_bytes()
    );
    files(&d);
}

#[test]
fn shell_unwritable_data_folder_is_a_warning() {
    let d = dir("unwritable");
    let file = d.join("file.txt");
    fs::write(&file, b"keep").unwrap();
    let data = file.join("data");
    let out = good(&args(&data), "down\nselect\nselect\nback\ndown\nselect\n");
    assert_eq!(
        out.lines().nth(1).unwrap().as_bytes(),
        format!("warn data folder not writable: {}", data.to_str().unwrap()).as_bytes()
    );
    assert!(out.contains("warn settings not saved: "), "{out}");
    assert!(out.ends_with("state quit\naction quit\n"), "{out}");
    assert_eq!(fs::read(file).unwrap(), b"keep");
}
