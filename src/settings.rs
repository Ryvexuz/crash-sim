//! Game settings file: strict, versioned, canonical.
//!
//! Format: first line `settings 1`, then `key = value` lines; blank lines and lines starting with
//! `#` are ignored. Keys (each may appear once; a missing key takes its default):
//!
//! | key            | value                                   | default |
//! |----------------|-----------------------------------------|---------|
//! | `width`        | integer 320..=7680                      | 1280    |
//! | `height`       | integer 320..=7680                      | 720     |
//! | `fullscreen`   | `true` or `false`                       | false   |
//! | `volume`       | integer 0..=100                         | 80      |
//! | `units`        | `kmh` or `mph`                          | kmh     |
//! | `key.throttle` | a name from [`KEY_NAMES`]               | up      |
//! | `key.brake`    | same                                    | down    |
//! | `key.left`     | same                                    | left    |
//! | `key.right`    | same                                    | right   |
//! | `key.pause`    | same                                    | escape  |
//! | `key.reset`    | same                                    | r       |
//!
//! No two actions may share a key. Everything unknown, repeated, out of range or malformed is an
//! error that names its line (1-based). [`write`] output is canonical (fixed key order, no
//! comments, `\n` line ends), so `load(write(s)) == s` and `write(load(write(s))) == write(s)`.
//! Self-contained, std only.

pub const VERSION_LINE: &str = "settings 1";
pub const MIN_SIZE: u32 = 320;
pub const MAX_SIZE: u32 = 7680;
pub const MAX_VOLUME: u32 = 100;

/// The fixed list of key names a binding may use.
pub const KEY_NAMES: [&str; 36] = [
    "a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l", "m", "n", "o", "p", "q", "r", "s",
    "t", "u", "v", "w", "x", "y", "z", "up", "down", "left", "right", "space", "escape", "enter",
    "tab", "shift", "ctrl",
];

/// Actions in canonical order.
pub const ACTIONS: [&str; 6] = ["throttle", "brake", "left", "right", "pause", "reset"];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Units {
    Kmh,
    Mph,
}

/// A key, held as an index into [`KEY_NAMES`] so only listed keys exist.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Key(usize);

impl Key {
    pub fn from_name(name: &str) -> Option<Key> {
        KEY_NAMES.iter().position(|k| *k == name).map(Key)
    }
    pub fn name(self) -> &'static str {
        KEY_NAMES[self.0]
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Settings {
    pub width: u32,
    pub height: u32,
    pub fullscreen: bool,
    pub volume: u32,
    pub units: Units,
    /// Bindings in [`ACTIONS`] order.
    pub keys: [Key; 6],
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SettingsError {
    /// 1-based line the problem is on.
    pub line: usize,
    pub message: String,
}

impl std::fmt::Display for SettingsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "settings line {}: {}", self.line, self.message)
    }
}

impl std::error::Error for SettingsError {}

fn err<T>(line: usize, message: impl Into<String>) -> Result<T, SettingsError> {
    Err(SettingsError {
        line,
        message: message.into(),
    })
}

impl Default for Settings {
    fn default() -> Self {
        let k = |n: &str| Key::from_name(n).expect("default key is in KEY_NAMES");
        Settings {
            width: 1280,
            height: 720,
            fullscreen: false,
            volume: 80,
            units: Units::Kmh,
            keys: [
                k("up"),
                k("down"),
                k("left"),
                k("right"),
                k("escape"),
                k("r"),
            ],
        }
    }
}

/// A `Settings` value that breaks a rule of the format (the same rules [`load`] enforces).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Invalid(pub String);

impl std::fmt::Display for Invalid {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "invalid settings: {}", self.0)
    }
}

impl std::error::Error for Invalid {}

fn check_range(name: &str, v: u32, lo: u32, hi: u32) -> Result<(), Invalid> {
    if (lo..=hi).contains(&v) {
        Ok(())
    } else {
        Err(Invalid(format!("{name} must be {lo}..={hi}, got {v}")))
    }
}

/// The value ranges. [`load`] calls this after every line (so the error names that line) and
/// [`validate`] calls it too: one copy of the rule.
fn validate_ranges(s: &Settings) -> Result<(), Invalid> {
    check_range("width", s.width, MIN_SIZE, MAX_SIZE)?;
    check_range("height", s.height, MIN_SIZE, MAX_SIZE)?;
    check_range("volume", s.volume, 0, MAX_VOLUME)
}

/// The first pair of actions (in [`ACTIONS`] order) bound to one key.
fn key_clash(s: &Settings) -> Option<(usize, usize)> {
    (0..6).find_map(|a| {
        ((a + 1)..6)
            .find(|&b| s.keys[a] == s.keys[b])
            .map(|b| (a, b))
    })
}

fn clash_error(s: &Settings, a: usize, b: usize) -> Invalid {
    Invalid(format!(
        "key.{} and key.{} are both {}",
        ACTIONS[a],
        ACTIONS[b],
        s.keys[a].name()
    ))
}

/// Every value rule: ranges and "no two actions on one key". [`write`] goes through it and [`load`]
/// applies the same two pieces (ranges per line, the key clash once all lines are read, because
/// swapping two default keys is fine), so a file `write` produces is always one `load` accepts.
/// Key names cannot be invalid: [`Key`] only holds an index into [`KEY_NAMES`].
pub fn validate(s: &Settings) -> Result<(), Invalid> {
    validate_ranges(s)?;
    match key_clash(s) {
        Some((a, b)) => Err(clash_error(s, a, b)),
        None => Ok(()),
    }
}

/// Syntax only: digits that fit a `u32`. The range is [`validate`]'s job.
fn parse_u32(line: usize, key: &str, v: &str) -> Result<u32, SettingsError> {
    if v.is_empty() || !v.bytes().all(|b| b.is_ascii_digit()) {
        return err(line, format!("{key} must be a whole number, got {v:?}"));
    }
    match v.parse::<u32>() {
        Ok(n) => Ok(n),
        Err(_) => err(line, format!("{key} is too large, got {v}")),
    }
}

/// Parse a settings file. Strict: see the module doc.
pub fn load(text: &str) -> Result<Settings, SettingsError> {
    let mut lines = text
        .split('\n')
        .enumerate()
        .map(|(i, l)| (i + 1, l.trim_end_matches('\r')));
    // `split` always yields at least one item, even for "".
    let (n0, first) = lines.next().unwrap_or((1, ""));
    if first != VERSION_LINE {
        if text.is_empty() {
            return err(1, "empty file");
        }
        return err(
            n0,
            format!("first line must be {VERSION_LINE:?}, got {first:?}"),
        );
    }
    let mut s = Settings::default();
    let mut seen: Vec<&str> = Vec::new();
    let mut key_line = [0usize; 6]; // line each binding was set on (0 = default)
    for (n, raw) in lines {
        let l = raw.trim();
        if l.is_empty() || l.starts_with('#') {
            continue;
        }
        let Some((k, v)) = l.split_once('=') else {
            return err(n, format!("expected `key = value`, got {l:?}"));
        };
        let (k, v) = (k.trim(), v.trim());
        if seen.contains(&k) {
            return err(n, format!("duplicate key {k}"));
        }
        match k {
            "width" => s.width = parse_u32(n, k, v)?,
            "height" => s.height = parse_u32(n, k, v)?,
            "volume" => s.volume = parse_u32(n, k, v)?,
            "fullscreen" => {
                s.fullscreen = match v {
                    "true" => true,
                    "false" => false,
                    _ => return err(n, format!("fullscreen must be true or false, got {v:?}")),
                }
            }
            "units" => {
                s.units = match v {
                    "kmh" => Units::Kmh,
                    "mph" => Units::Mph,
                    _ => return err(n, format!("units must be kmh or mph, got {v:?}")),
                }
            }
            _ => {
                let Some(action) = k.strip_prefix("key.") else {
                    return err(n, format!("unknown key {k:?}"));
                };
                let Some(ai) = ACTIONS.iter().position(|a| *a == action) else {
                    return err(n, format!("unknown action {action:?}"));
                };
                let Some(key) = Key::from_name(v) else {
                    return err(n, format!("unknown key name {v:?}"));
                };
                s.keys[ai] = key;
                key_line[ai] = n;
            }
        }
        // The defaults are in range, so only the line just applied can be out of range.
        if let Err(Invalid(m)) = validate_ranges(&s) {
            return err(n, m);
        }
        seen.push(k);
    }
    if let Some((a, b)) = key_clash(&s) {
        // The defaults never clash, so one of the two was set explicitly: name the later line.
        let at = key_line[a].max(key_line[b]);
        return err(at, clash_error(&s, a, b).0);
    }
    Ok(s)
}

/// Canonical text for `s`: fixed key order, no comments, one `\n` after every line.
/// Refuses a `Settings` that [`validate`] rejects, so it never produces a file `load` would refuse.
pub fn write(s: &Settings) -> Result<String, Invalid> {
    validate(s)?;
    let mut o = format!("{VERSION_LINE}\n");
    o += &format!("width = {}\nheight = {}\n", s.width, s.height);
    o += &format!("fullscreen = {}\nvolume = {}\n", s.fullscreen, s.volume);
    o += &format!(
        "units = {}\n",
        if s.units == Units::Kmh { "kmh" } else { "mph" }
    );
    for (a, k) in ACTIONS.iter().zip(s.keys.iter()) {
        o += &format!("key.{a} = {}\n", k.name());
    }
    Ok(o)
}
