//! Runtime numeric overrides. Parsing and rewriting are pure; include reads are supplied.

use std::fmt;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TuneError {
    pub file: String,
    pub line: usize,
    pub message: String,
}

impl fmt::Display for TuneError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}:{}: {}", self.file, self.line, self.message)
    }
}

impl std::error::Error for TuneError {}

#[derive(Clone, Debug)]
struct Site {
    file: String,
    line: usize,
}

impl Site {
    fn error(&self, message: impl Into<String>) -> TuneError {
        TuneError {
            file: self.file.clone(),
            line: self.line,
            message: message.into(),
        }
    }
}

#[derive(Clone, Debug)]
struct Target {
    keyword: String,
    name: String,
    field: String,
    site: Site,
}

impl Target {
    fn key(&self) -> String {
        format!("{}.{}.{}", self.keyword, self.name, self.field)
    }
}

#[derive(Clone, Debug)]
struct Edit {
    target: Target,
    value: f64,
    scale: bool,
}

#[derive(Clone, Debug)]
struct Sweep {
    target: Target,
    from: f64,
    to: f64,
    steps: usize,
    log: bool,
}

/// Constructed only by the parser; merging retains the same sweep limits.
#[derive(Clone, Debug, Default)]
pub struct Tune {
    edits: Vec<Edit>,
    sweeps: Vec<Sweep>,
}

impl Tune {
    /// Append another file's edits and sweeps, preserving their source locations.
    pub fn append(&mut self, other: Tune) -> Result<(), TuneError> {
        let mut candidate = self.clone();
        candidate.edits.extend(other.edits);
        candidate.sweeps.extend(other.sweeps);
        candidate.check()?;
        *self = candidate;
        Ok(())
    }

    fn check(&self) -> Result<(), TuneError> {
        if let Some(sweep) = self.sweeps.get(2) {
            return Err(sweep.target.site.error("at most 2 sweeps are allowed"));
        }
        if let Some(sweep) = self
            .sweeps
            .last()
            .filter(|_| self.sweeps.iter().map(|s| s.steps).product::<usize>() > 400)
        {
            return Err(sweep.target.site.error("total points exceeds 400"));
        }
        Ok(())
    }
}

fn number(token: &str, site: &Site) -> Result<f64, TuneError> {
    let value = token
        .parse::<f64>()
        .ok()
        .filter(|v| v.is_finite())
        .ok_or_else(|| site.error(format!("bad number {token:?}: expected a finite f64")))?;
    let mantissa = token.split(['e', 'E']).next().unwrap_or("");
    if value == 0.0 && mantissa.bytes().any(|b| matches!(b, b'1'..=b'9')) {
        return Err(site.error(format!("bad number {token:?}: underflows f64 to zero")));
    }
    Ok(value)
}

// Use portable lexical paths: includes accept both slash styles, including on Unix.
// No filesystem canonicalization (or I/O) is hidden in the library.
fn normalized(path: &str) -> String {
    let path = path.replace('\\', "/");
    let (prefix, rest) = if let Some(rest) = path.strip_prefix("//") {
        ("//", rest)
    } else if let Some(rest) = path.strip_prefix('/') {
        ("/", rest)
    } else if path.as_bytes().get(1) == Some(&b':') && path.as_bytes().get(2) == Some(&b'/') {
        (&path[..3], &path[3..])
    } else {
        ("", path.as_str())
    };
    let mut leading = rest.split('/');
    let verbatim_unc = leading.next() == Some("?")
        && leading.next().is_some_and(|p| p.eq_ignore_ascii_case("UNC"));
    let protected = match (prefix, verbatim_unc) {
        ("//", true) => 4,
        ("//", false) => 2,
        _ => 0,
    };
    let mut parts: Vec<&str> = Vec::new();
    for part in rest.split('/') {
        match part {
            "" | "." => {}
            ".." if parts.len() > protected && parts.last() != Some(&"..") => {
                parts.pop();
            }
            ".." if !prefix.is_empty() => {}
            _ => parts.push(part),
        }
    }
    format!("{prefix}{}", parts.join("/"))
}

fn path_key(path: &str) -> String {
    if cfg!(windows) {
        path.to_lowercase()
    } else {
        path.to_owned()
    }
}

/// Parse a tune file. `read` receives a lexically resolved path relative to its includer.
/// The root is depth zero; four nested includes are allowed.
pub fn parse_tune(
    text: &str,
    file: &str,
    read: &dyn Fn(&str) -> Result<String, String>,
) -> Result<Tune, TuneError> {
    if file.is_empty() || file.chars().any(char::is_control) {
        return Err(Site {
            file: file.into(),
            line: 1,
        }
        .error("invalid tune filename: empty or contains a control character"));
    }
    let portable = file.replace('\\', "/");
    if portable.as_bytes().get(1) == Some(&b':') && portable.as_bytes().get(2) != Some(&b'/') {
        return Err(Site {
            file: file.into(),
            line: 1,
        }
        .error("drive-relative tune filename is ambiguous; use an absolute path"));
    }
    let mut tune = Tune::default();
    let file = normalized(file);
    parse_file(text, &file, read, &mut vec![path_key(&file)], &mut tune)?;
    Ok(tune)
}

fn parse_file(
    text: &str,
    file: &str,
    read: &dyn Fn(&str) -> Result<String, String>,
    stack: &mut Vec<String>,
    tune: &mut Tune,
) -> Result<(), TuneError> {
    let mut header = false;
    for (index, line) in text.lines().enumerate() {
        let site = Site {
            file: file.into(),
            line: index + 1,
        };
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        // The longest production is sweep with its optional mode (eight tokens).
        // A ninth token proves a count error without collecting a hostile whole line.
        let tokens: Vec<_> = line.split_whitespace().take(9).collect();
        if !header {
            if tokens != ["tune", "1"] {
                return Err(site.error("expected tune 1 header"));
            }
            header = true;
            continue;
        }
        if tokens[0] == "include" {
            if tokens.len() != 2 {
                return Err(site.error("wrong token count for include"));
            }
            let relative = tokens[1].replace('\\', "/");
            if relative.chars().any(char::is_control) {
                return Err(site.error("invalid include path: contains a control character"));
            }
            if relative.starts_with('/') || relative.contains(':') {
                return Err(site.error("include path must be relative (no drive or absolute path)"));
            }
            let parent = file.rsplit_once('/').map_or("", |(p, _)| {
                if p.is_empty() && file.starts_with('/') {
                    "/"
                } else {
                    p
                }
            });
            let path = normalized(&if parent.is_empty() {
                relative
            } else if parent == "/" {
                format!("/{relative}")
            } else {
                format!("{parent}/{relative}")
            });
            let key = path_key(&path);
            if stack.contains(&key) {
                return Err(site.error(format!("include cycle: {path}")));
            }
            if stack.len() > 4 {
                return Err(site.error("include depth exceeds 4"));
            }
            let included = read(&path)
                .map_err(|reason| site.error(format!("cannot include {path}: {reason}")))?;
            stack.push(key);
            parse_file(&included, &path, read, stack, tune)?;
            stack.pop();
            continue;
        }
        let kind = tokens[0];
        let count_ok = match kind {
            "set" | "scale" => tokens.len() == 5,
            "sweep" => matches!(tokens.len(), 7 | 8),
            _ => return Err(site.error(format!("unknown line kind {kind:?}"))),
        };
        if !count_ok {
            return Err(site.error(format!("wrong token count for {kind}")));
        }
        let target = Target {
            keyword: tokens[1].into(),
            name: tokens[2].into(),
            field: tokens[3].into(),
            site: site.clone(),
        };
        let value = number(tokens[4], &site)?;
        if kind == "sweep" {
            let to = number(tokens[5], &site)?;
            let steps = tokens[6]
                .parse::<usize>()
                .ok()
                .filter(|n| (2..=200).contains(n))
                .ok_or_else(|| site.error("bad steps: expected integer 2..=200"))?;
            let log = match tokens.get(7).copied().unwrap_or("lin") {
                "lin" => false,
                "log" => true,
                _ => return Err(site.error("bad sweep mode: expected lin or log")),
            };
            if log && (value <= 0.0 || to <= 0.0) {
                return Err(site.error("log sweep endpoints must be positive"));
            }
            tune.sweeps.push(Sweep {
                target,
                from: value,
                to,
                steps,
                log,
            });
            tune.check()?;
        } else {
            if kind == "scale" && value <= 0.0 {
                return Err(site.error("scale factor must be positive"));
            }
            tune.edits.push(Edit {
                target,
                value,
                scale: kind == "scale",
            });
        }
    }
    if !header {
        return Err(Site {
            file: file.into(),
            line: 1,
        }
        .error("missing tune 1 header"));
    }
    Ok(())
}

impl Sweep {
    fn value(&self, i: usize) -> f64 {
        let fraction = i as f64 / (self.steps - 1) as f64;
        // Use the specified formula in its ordinary range. Weighted interpolation avoids
        // overflow of a difference/ratio across the full finite f64 input band.
        let value = if self.log {
            let ratio = self.to / self.from;
            if ratio.is_finite() && ratio > 0.0 {
                self.from * ratio.powf(fraction)
            } else {
                (self.from.ln() * (1.0 - fraction) + self.to.ln() * fraction).exp()
            }
        } else {
            let delta = self.to - self.from;
            if delta.is_finite() {
                self.from + delta * fraction
            } else {
                self.from * (1.0 - fraction) + self.to * fraction
            }
        };
        // Clamp rounding overflow/underflow to the finite endpoint interval.
        let value = value.clamp(self.from.min(self.to), self.from.max(self.to));
        if i == 0 {
            self.from
        } else if i == self.steps - 1 {
            self.to
        } else {
            value
        }
    }
}

/// Cartesian points, first sweep outermost. No sweeps produces one empty point.
pub fn points(t: &Tune) -> Vec<Vec<(String, f64)>> {
    let mut result = vec![Vec::new()];
    for sweep in &t.sweeps {
        result = result
            .into_iter()
            .flat_map(|prefix| {
                (0..sweep.steps).map(move |i| {
                    let mut point = prefix.clone();
                    point.push((sweep.target.key(), sweep.value(i)));
                    point
                })
            })
            .collect();
    }
    result
}

// Byte spans preserve indentation, all whitespace, comments, and line endings.
fn spans(line: &str) -> Vec<(usize, usize)> {
    let mut result = Vec::new();
    let mut start = None;
    for (index, ch) in line.char_indices().chain(std::iter::once((line.len(), ' '))) {
        if ch.is_whitespace() {
            if let Some(begin) = start.take() {
                result.push((begin, index));
            }
        } else if start.is_none() {
            start = Some(index);
        }
    }
    result
}

fn rewrite(body: &str, target: &Target, value: f64, scale: bool) -> Result<String, TuneError> {
    let mut output = String::with_capacity(body.len());
    let mut matched = false;
    let mut touched = false;
    for line in body.split_inclusive('\n') {
        let mut head = line.split_whitespace();
        let keyword = head.next();
        let name = head.next();
        if keyword.is_some_and(|token| token.starts_with('#'))
            || keyword != Some(target.keyword.as_str())
            || name.is_none()
            || (target.name != "*" && name != Some(target.name.as_str()))
        {
            output.push_str(line);
            continue;
        }
        let positions = spans(line);
        let tokens: Vec<_> = positions.iter().map(|&(a, b)| &line[a..b]).collect();
        matched = true;
        let field = tokens
            .iter()
            .enumerate()
            .skip(2)
            .take_while(|(_, token)| !token.starts_with('#'))
            .find(|(_, token)| **token == target.field)
            .map(|(i, _)| i);
        let Some(field) = field else {
            if target.name == "*" {
                output.push_str(line);
                continue;
            }
            return Err(target.site.error(format!(
                "matched line has no {:?} field token",
                target.field
            )));
        };
        let old = tokens.get(field + 1).ok_or_else(|| {
            target
                .site
                .error(format!("field {:?} has no value", target.field))
        })?;
        let old = number(old, &target.site)?;
        let replacement = if scale { old * value } else { value };
        if !replacement.is_finite() || (scale && old != 0.0 && replacement == 0.0) {
            return Err(target.site.error("scaled field overflow or underflow"));
        }
        let (a, b) = positions[field + 1];
        output.push_str(&line[..a]);
        output.push_str(&replacement.to_string());
        output.push_str(&line[b..]);
        touched = true;
    }
    if !matched {
        return Err(target.site.error(format!(
            "no body line matches {} {}",
            target.keyword, target.name
        )));
    }
    if !touched {
        return Err(target
            .site
            .error(format!("matched lines have no {:?} field token", target.field)));
    }
    Ok(output)
}

/// Apply ordinary edits in order, then sweep values. Only value-token bytes change.
/// A point must have exactly the keys/order from `points`, with finite values.
pub fn apply(body: &str, t: &Tune, point: &[(String, f64)]) -> Result<String, TuneError> {
    let bad_point = || TuneError {
        file: "<point>".into(),
        line: 1,
        message: "point must contain each sweep key in order with a finite value".into(),
    };
    if point.len() != t.sweeps.len()
        || point
            .iter()
            .zip(&t.sweeps)
            .any(|((key, value), sweep)| *key != sweep.target.key() || !value.is_finite())
    {
        return Err(bad_point());
    }
    let mut text = body.to_owned();
    for edit in &t.edits {
        text = rewrite(&text, &edit.target, edit.value, edit.scale)?;
    }
    for (sweep, (_, value)) in t.sweeps.iter().zip(point) {
        text = rewrite(&text, &sweep.target, *value, false)?;
    }
    Ok(text)
}
