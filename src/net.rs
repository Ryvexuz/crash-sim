//! Lockstep net message codec (n20-25a): one message is one line of text, so values round-trip
//! exactly. Std only. Inputs are integers in 1/1000 units (no floats cross the wire).
//!
//! The wire format is documented in `docs/NET.md`. Parsing is strict: exactly one space between
//! fields, no leading or trailing whitespace, no line terminator, canonical decimal integers
//! (no `+`, no leading zeros, no `-0`), lower-case hex only.

use std::fmt;

/// Wire protocol version carried by `hello`.
pub const VERSION: u32 = 1;
/// Throttle and brake range 0..=INPUT_SCALE; steer ranges -INPUT_SCALE..=INPUT_SCALE.
pub const INPUT_SCALE: i32 = 1000;
/// Highest accepted tick rate (ticks per second) in `hello`; the lowest is 1.
pub const MAX_TICK_RATE: u32 = 1000;
/// Longest line `decode` looks at. Longer input is refused before any parsing.
pub const MAX_LINE_BYTES: usize = 128;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Msg {
    Hello {
        seed: u64,
        tick_rate: u32,
    },
    Input {
        tick: u64,
        throttle: i32,
        brake: i32,
        steer: i32,
    },
    Hash {
        tick: u64,
        hash: u64,
    },
    Bye,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum NetError {
    Empty,
    TooLong,
    UnknownWord(String),
    /// A field count other than the message kind needs.
    WrongFieldCount {
        word: &'static str,
        expected: usize,
        got: usize,
    },
    /// A field that is not a canonical decimal integer (or hex string) of the right shape.
    BadNumber {
        field: &'static str,
    },
    OutOfRange {
        field: &'static str,
    },
    WrongVersion(u32),
    /// Whitespace other than single spaces between fields, a control character, or any non-ASCII byte.
    BadSpacing,
}

impl fmt::Display for NetError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            NetError::Empty => write!(f, "empty message"),
            NetError::TooLong => write!(f, "message longer than {MAX_LINE_BYTES} bytes"),
            NetError::UnknownWord(w) => write!(f, "unknown message word {w:?}"),
            NetError::WrongFieldCount {
                word,
                expected,
                got,
            } => {
                write!(f, "{word}: expected {expected} fields, got {got}")
            }
            NetError::BadNumber { field } => write!(f, "{field}: not a canonical number"),
            NetError::OutOfRange { field } => write!(f, "{field}: out of range"),
            NetError::WrongVersion(v) => write!(f, "unsupported protocol version {v}"),
            NetError::BadSpacing => write!(f, "bad spacing or control character"),
        }
    }
}

impl std::error::Error for NetError {}

/// One message as one line, without a line terminator (the transport adds `\n`).
pub fn encode(msg: &Msg) -> String {
    match *msg {
        Msg::Hello { seed, tick_rate } => format!("hello {VERSION} {seed} {tick_rate}"),
        Msg::Input {
            tick,
            throttle,
            brake,
            steer,
        } => {
            format!("input {tick} {throttle} {brake} {steer}")
        }
        Msg::Hash { tick, hash } => format!("hash {tick} {hash:016x}"),
        Msg::Bye => "bye".to_string(),
    }
}

pub fn decode(line: &str) -> Result<Msg, NetError> {
    if line.is_empty() {
        return Err(NetError::Empty);
    }
    if line.len() > MAX_LINE_BYTES {
        return Err(NetError::TooLong);
    }
    // Printable ASCII only: this also refuses tabs, \r, \n and any non-ASCII space.
    if !line.bytes().all(|b| (0x20..0x7f).contains(&b)) {
        return Err(NetError::BadSpacing);
    }
    // Splitting on a single space makes leading, trailing and doubled spaces show up as empty fields.
    let parts: Vec<&str> = line.split(' ').collect();
    if parts.iter().any(|p| p.is_empty()) {
        return Err(NetError::BadSpacing);
    }
    let word = parts[0];
    let args = &parts[1..];
    match word {
        "hello" => {
            expect_count("hello", args, 3)?;
            let version = parse_u64("version", args[0])?;
            if version != VERSION as u64 {
                return Err(NetError::WrongVersion(
                    u32::try_from(version).unwrap_or(u32::MAX),
                ));
            }
            let seed = parse_u64("seed", args[1])?;
            let tick_rate = parse_u64("tick_rate", args[2])?;
            if !(1..=MAX_TICK_RATE as u64).contains(&tick_rate) {
                return Err(NetError::OutOfRange { field: "tick_rate" });
            }
            Ok(Msg::Hello {
                seed,
                tick_rate: tick_rate as u32,
            })
        }
        "input" => {
            expect_count("input", args, 4)?;
            let tick = parse_u64("tick", args[0])?;
            let throttle = parse_i32("throttle", args[1], 0, INPUT_SCALE)?;
            let brake = parse_i32("brake", args[2], 0, INPUT_SCALE)?;
            let steer = parse_i32("steer", args[3], -INPUT_SCALE, INPUT_SCALE)?;
            Ok(Msg::Input {
                tick,
                throttle,
                brake,
                steer,
            })
        }
        "hash" => {
            expect_count("hash", args, 2)?;
            let tick = parse_u64("tick", args[0])?;
            let h = args[1];
            if h.len() != 16
                || !h
                    .bytes()
                    .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
            {
                return Err(NetError::BadNumber { field: "hash" });
            }
            let hash =
                u64::from_str_radix(h, 16).map_err(|_| NetError::BadNumber { field: "hash" })?;
            Ok(Msg::Hash { tick, hash })
        }
        "bye" => {
            expect_count("bye", args, 0)?;
            Ok(Msg::Bye)
        }
        other => Err(NetError::UnknownWord(other.to_string())),
    }
}

fn expect_count(word: &'static str, args: &[&str], expected: usize) -> Result<(), NetError> {
    if args.len() == expected {
        Ok(())
    } else {
        Err(NetError::WrongFieldCount {
            word,
            expected,
            got: args.len(),
        })
    }
}

/// Canonical unsigned decimal: ASCII digits only, no leading zero (except "0" itself).
fn parse_u64(field: &'static str, s: &str) -> Result<u64, NetError> {
    let bytes = s.as_bytes();
    if bytes.is_empty()
        || !bytes.iter().all(|b| b.is_ascii_digit())
        || (bytes.len() > 1 && bytes[0] == b'0')
    {
        return Err(NetError::BadNumber { field });
    }
    s.parse::<u64>().map_err(|_| NetError::OutOfRange { field })
}

/// Canonical signed decimal within lo..=hi: optional `-` (never on zero), no `+`, no leading zeros.
fn parse_i32(field: &'static str, s: &str, lo: i32, hi: i32) -> Result<i32, NetError> {
    let (neg, digits) = match s.strip_prefix('-') {
        Some(rest) => (true, rest),
        None => (false, s),
    };
    let bytes = digits.as_bytes();
    if bytes.is_empty()
        || !bytes.iter().all(|b| b.is_ascii_digit())
        || (bytes.len() > 1 && bytes[0] == b'0')
        || (neg && digits == "0")
    {
        return Err(NetError::BadNumber { field });
    }
    let mag = digits
        .parse::<i64>()
        .map_err(|_| NetError::OutOfRange { field })?;
    let v = if neg { -mag } else { mag };
    if v < lo as i64 || v > hi as i64 {
        return Err(NetError::OutOfRange { field });
    }
    Ok(v as i32)
}
