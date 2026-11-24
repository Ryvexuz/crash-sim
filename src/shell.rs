//! Line-oriented game shell. Menus, settings validation and save rules stay in their modules.

use crate::menu::{self, Action, Event, State};
use crate::save::{self, Save};
use crate::settings::{self, Settings, Units};
use std::collections::BTreeSet;
use std::fs;
use std::io::{BufRead, ErrorKind, Write};
use std::path::{Path, PathBuf};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ShellArgs {
    pub data: PathBuf,
    pub tracks: Vec<String>,
    pub cars: Vec<String>,
    pub today: String,
}

fn stem(file: &str) -> Option<&str> {
    Path::new(file).file_stem()?.to_str().filter(|s| !s.is_empty())
}

fn add_file(files: &mut Vec<String>, flag: &str, value: &str) -> Result<(), String> {
    let id = stem(value).ok_or_else(|| format!("{flag}: missing or invalid file stem"))?;
    if files.iter().any(|file| stem(file) == Some(id)) {
        return Err(format!("{flag}: duplicate file stem {id:?}"));
    }
    files.push(value.to_owned());
    Ok(())
}

/// `args` excludes the program name and must contain `--shell` once.
/// Err is the usage message.
pub fn parse_args(args: &[String]) -> Result<ShellArgs, String> {
    let (mut shell, mut data, mut today) = (false, None, None);
    let (mut tracks, mut cars) = (Vec::new(), Vec::new());
    let mut args = args.iter();
    while let Some(flag) = args.next() {
        if flag == "--shell" {
            if shell {
                return Err("--shell: repeated flag".into());
            }
            shell = true;
            continue;
        }
        if !matches!(
            flag.as_str(),
            "--data" | "--shell-track" | "--shell-car" | "--today"
        ) {
            return Err(format!("{flag}: unknown flag"));
        }
        let value = args
            .next()
            .filter(|v| !v.starts_with("--") && !v.is_empty())
            .ok_or_else(|| format!("{flag}: missing value"))?;
        // Text facts must remain single lines, including paths printed verbatim.
        if value
            .chars()
            .any(|c| c.is_control() || matches!(c, '\u{2028}' | '\u{2029}'))
        {
            return Err(format!(
                "{flag}: value contains a line break or control character"
            ));
        }
        match flag.as_str() {
            "--data" => {
                if data.replace(PathBuf::from(value)).is_some() {
                    return Err("--data: repeated flag".into());
                }
            }
            "--today" => {
                if today.replace(value.clone()).is_some() {
                    return Err("--today: repeated flag".into());
                }
                if value == "unknown" {
                    return Err("--today: unknown is not a date".into());
                }
                Save::parse(&format!("save 2\nbest t 1 c {value}\n"))
                    .map_err(|e| format!("--today: {e}"))?;
            }
            "--shell-track" => add_file(&mut tracks, flag, value)?,
            "--shell-car" => add_file(&mut cars, flag, value)?,
            _ => {}
        }
    }
    if !shell {
        return Err("--shell: required".into());
    }
    let data = data.ok_or("--data: required")?;
    let today = today.ok_or("--today: required")?;
    if tracks.is_empty() {
        return Err("--shell-track: at least one required".into());
    }
    if cars.is_empty() {
        return Err("--shell-car: at least one required".into());
    }
    Ok(ShellArgs {
        data,
        tracks,
        cars,
        today,
    })
}

fn load_file<T, E: std::fmt::Display>(
    path: &Path,
    parse: impl FnOnce(&str) -> Result<T, E>,
) -> Result<Option<T>, String> {
    match fs::read_to_string(path) {
        Ok(text) => parse(&text).map(Some).map_err(|e| e.to_string()),
        Err(e) if e.kind() == ErrorKind::NotFound => Ok(None),
        Err(e) => Err(e.to_string()),
    }
}

pub struct Shell {
    args: ShellArgs,
    state: State,
    settings: Settings,
    source: &'static str,
    save: Save,
    refused: BTreeSet<&'static str>,
    finished: bool,
}

impl Shell {
    /// Never fails: file problems are `warn` lines. Returns the start lines through main.
    pub fn start(args: ShellArgs) -> (Shell, Vec<String>) {
        let mut lines = vec![format!(
            "shell tracks={} cars={}",
            args.tracks.len(),
            args.cars.len()
        )];
        let probe = args.data.join(".shell-probe");
        let writable = fs::create_dir_all(&args.data).is_ok()
            && save::write_atomic(&probe, b"").is_ok()
            && fs::remove_file(&probe).is_ok();
        if !writable {
            lines.push(format!(
                "warn data folder not writable: {}",
                args.data.display()
            ));
        }
        let mut shell = Shell {
            args,
            state: menu::start(),
            settings: Settings::default(),
            source: "defaults",
            save: Save::new(),
            refused: BTreeSet::new(),
            finished: false,
        };
        match load_file(&shell.args.data.join("settings.txt"), settings::load) {
            Ok(Some(settings)) => {
                shell.settings = settings;
                shell.source = "file";
            }
            Ok(None) => {}
            Err(e) => {
                shell.refused.insert("settings.txt");
                lines.push(format!("warn settings refused: {e}"));
            }
        }
        lines.push(shell.settings_line());
        match load_file(&shell.args.data.join("best.save"), Save::parse) {
            Ok(Some(save)) => shell.save = save,
            Ok(None) => {}
            Err(e) => {
                shell.refused.insert("best.save");
                lines.push(format!("warn save refused: {e}"));
            }
        }
        for (track, best) in &shell.save.best {
            lines.push(format!(
                "best track={track} seconds={:.3} car={} date={}",
                best.seconds, best.car, best.date
            ));
        }
        lines.push(shell.state_line());
        (shell, lines)
    }

    fn settings_line(&self) -> String {
        let s = &self.settings;
        let units = if s.units == Units::Kmh { "kmh" } else { "mph" };
        format!(
            "settings width={} height={} fullscreen={} volume={} units={units} source={}",
            s.width, s.height, s.fullscreen, s.volume, self.source
        )
    }

    // Safe even for a directly constructed ShellArgs with empty or invalid lists.
    fn file(files: &[String], index: usize) -> &str {
        files.get(index).map_or("", String::as_str)
    }

    fn track(&self, index: usize) -> &str {
        stem(Self::file(&self.args.tracks, index)).map_or("", |s| s)
    }

    fn car(&self, index: usize) -> &str {
        stem(Self::file(&self.args.cars, index)).map_or("", |s| s)
    }

    fn state_line(&self) -> String {
        match self.state {
            State::Main { cursor } => format!("state main cursor={cursor}"),
            State::TrackSelect { cursor } => format!("state track_select cursor={cursor}"),
            State::CarSelect { track, cursor } => {
                format!("state car_select track={} cursor={cursor}", self.track(track))
            }
            State::Settings => "state settings".into(),
            State::Race { track, car } | State::Results { track, car } => {
                let name = if matches!(self.state, State::Race { .. }) {
                    "race"
                } else {
                    "results"
                };
                format!("state {name} track={} car={}", self.track(track), self.car(car))
            }
            State::Paused { track, car, cursor } => format!(
                "state paused track={} car={} cursor={cursor}",
                self.track(track),
                self.car(car)
            ),
            State::Quit => "state quit".into(),
        }
    }

    // All persistent writes pass the refusal guard, keyed by the actual data filename.
    fn write_file(&self, name: &'static str, text: &str) -> Result<(), String> {
        if self.refused.contains(name) {
            return Err(format!("{name} was refused at start and is left unchanged"));
        }
        save::write_atomic(&self.args.data.join(name), text.as_bytes()).map_err(|e| e.to_string())
    }

    fn action(&mut self, action: Action, lines: &mut Vec<String>) {
        match action {
            Action::StartRace { track, car } => lines.push(format!(
                "action start_race track={} track_file={} car={} car_file={}",
                self.track(track),
                Self::file(&self.args.tracks, track),
                self.car(car),
                Self::file(&self.args.cars, car)
            )),
            Action::AbortRace => lines.push("action abort_race".into()),
            Action::Quit => {
                lines.push("action quit".into());
                self.finished = true;
            }
            Action::SaveSettings => {
                lines.push("action save_settings".into());
                let result = settings::write(&self.settings)
                    .map_err(|e| e.to_string())
                    .and_then(|text| self.write_file("settings.txt", &text));
                match result {
                    Ok(()) => {
                        self.source = "file";
                        lines.push("saved settings".into());
                    }
                    Err(e) => lines.push(format!("warn settings not saved: {e}")),
                }
            }
        }
    }

    fn set(&mut self, line: &str) -> Result<String, String> {
        if self.state != State::Settings {
            return Err("not on the settings screen".into());
        }
        let mut tokens = line.split(' ');
        let (_, Some(key), Some(value), None) =
            (tokens.next(), tokens.next(), tokens.next(), tokens.next())
        else {
            return Err("expected set <key> <value>".into());
        };
        if key.is_empty() || value.is_empty() {
            return Err("expected set <key> <value>".into());
        }
        let mut settings = self.settings.clone();
        let needs = |kind| format!("{key} needs {kind}, got {value}");
        match key {
            "width" | "height" | "volume" => {
                let n = digits(value, 10).ok_or_else(|| needs("an integer"))?;
                match key {
                    "width" => settings.width = n,
                    "height" => settings.height = n,
                    _ => settings.volume = n,
                }
            }
            "fullscreen" => {
                settings.fullscreen = match value {
                    "true" => true,
                    "false" => false,
                    _ => return Err(needs("true or false")),
                };
            }
            "units" => {
                settings.units = match value {
                    "kmh" => Units::Kmh,
                    "mph" => Units::Mph,
                    _ => return Err(needs("kmh or mph")),
                };
            }
            _ => return Err(format!("unknown key {key}")),
        }
        settings::validate(&settings).map_err(|e| e.to_string())?;
        self.settings = settings;
        self.source = "unsaved";
        Ok(self.settings_line())
    }

    fn record(&mut self, track: usize, car: usize, ms: u32, lines: &mut Vec<String>) {
        // The protocol bounds ms to 1..=999999999: division cannot overflow or underflow.
        // record_lap validates finiteness and all entry fields before changing the save.
        let track = self.track(track).to_owned();
        let car = self.car(car).to_owned();
        let result = self
            .save
            .record_lap(&track, ms as f64 / 1000.0, &car, &self.args.today);
        let result = result.map_err(|e| e.to_string()).and_then(|changed| {
            if changed {
                self.write_file("best.save", &self.save.to_text())?;
            }
            Ok(changed)
        });
        match result {
            Ok(true) => lines.push(format!(
                "saved best track={track} seconds={}.{:03}",
                ms / 1000,
                ms % 1000
            )),
            Ok(false) => {}
            Err(e) => lines.push(format!("warn best not saved: {e}")),
        }
    }

    /// One event line (its line end already removed). Returns output lines in order.
    pub fn event(&mut self, line: &str) -> Vec<String> {
        if line == "set" || line.starts_with("set ") {
            return vec![match self.set(line) {
                Ok(line) => line,
                Err(e) => format!("warn set refused: {e}"),
            }];
        }
        let mut ms = None;
        let event = match line {
            "up" => Event::Up,
            "down" => Event::Down,
            "select" => Event::Select,
            "back" => Event::Back,
            "pause" => Event::Pause,
            "finished none" => Event::RaceFinished,
            _ => {
                ms = line
                    .strip_prefix("finished ")
                    .and_then(|s| digits(s, 9))
                    .filter(|n| *n > 0);
                if ms.is_none() {
                    return vec![format!("warn event refused: unknown event {line:?}")];
                }
                Event::RaceFinished
            }
        };
        let racing = matches!(self.state, State::Race { .. } | State::Paused { .. });
        let (state, action) =
            menu::step(self.state, event, self.args.tracks.len(), self.args.cars.len());
        self.state = state;
        let mut lines = vec![self.state_line()];
        if let Some(action) = action {
            self.action(action, &mut lines);
        }
        if event == Event::RaceFinished {
            if !racing {
                lines.push("warn finished ignored: not in a race".into());
            } else if let (State::Results { track, car }, Some(ms)) = (state, ms) {
                self.record(track, car, ms, &mut lines);
            }
        }
        lines
    }

    /// True once `action quit` was printed.
    pub fn finished(&self) -> bool {
        self.finished
    }
}

fn digits(text: &str, max_len: usize) -> Option<u32> {
    if text.is_empty() || text.len() > max_len || !text.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    text.parse().ok()
}

fn emit(out: &mut dyn Write, lines: Vec<String>) -> std::io::Result<()> {
    for line in lines {
        writeln!(out, "{line}")?;
        out.flush()?;
    }
    Ok(())
}

/// Usage errors return 2 with empty stdout; output errors return 1.
/// EOF (including a read error) or `action quit` returns 0. Each fact is flushed.
pub fn run(
    args: &[String],
    input: &mut dyn BufRead,
    out: &mut dyn Write,
    err: &mut dyn Write,
) -> i32 {
    let args = match parse_args(args) {
        Ok(args) => args,
        Err(e) => {
            let _ = writeln!(err, "shell: usage: {e}");
            return 2;
        }
    };
    let (mut shell, lines) = Shell::start(args);
    if emit(out, lines).is_err() {
        return 1;
    }
    let mut line = String::new();
    loop {
        line.clear();
        match input.read_line(&mut line) {
            Ok(0) | Err(_) => return 0,
            Ok(_) => {}
        }
        if line.ends_with('\n') {
            line.pop();
        }
        if line.ends_with('\r') {
            line.pop();
        }
        if emit(out, shell.event(&line)).is_err() {
            return 1;
        }
        if shell.finished() {
            return 0;
        }
    }
}
