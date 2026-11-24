//! Race lines are held until their interval's hash is confirmed, then released in simulation-time
//! order (ties by car and then push order), followed by the hash line; discard drops all held lines
//! and permanently stops acceptance after a desync or other failure.
//! The reference run without networking uses the same buffer so both print in the same order;
//! times use `f64::total_cmp`, including negative zero before positive zero.

use std::fmt;

#[derive(Clone, Debug, PartialEq)]
pub enum ConfirmError {
    /// The hash interval was zero.
    ZeroInterval,
    /// The tick belongs to an already confirmed interval.
    LateLine { tick: u64, confirmed_through: u64 },
    /// The simulation time is not finite.
    BadTime { tick: u64 },
    /// The line is empty or contains a line terminator.
    BadLine { tick: u64 },
    /// The hash tick is not the next interval's last tick.
    WrongHashTick { expected: u64, got: u64 },
    /// The next interval's last tick does not fit in u64.
    Overflow,
    /// The buffer was discarded.
    Stopped,
}

impl fmt::Display for ConfirmError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ZeroInterval => write!(f, "The hash interval must be nonzero."),
            Self::LateLine {
                tick,
                confirmed_through,
            } => write!(
                f,
                "Tick {tick} is already confirmed through {confirmed_through}."
            ),
            Self::BadTime { tick } => write!(f, "The time at tick {tick} is not finite."),
            Self::BadLine { tick } => write!(
                f,
                "The line at tick {tick} is empty or contains a line terminator."
            ),
            Self::WrongHashTick { expected, got } => {
                write!(f, "Expected hash tick {expected}, but got {got}.")
            }
            Self::Overflow => write!(f, "The next hash tick exceeds u64."),
            Self::Stopped => write!(f, "The line buffer has stopped."),
        }
    }
}

impl std::error::Error for ConfirmError {}

struct Line {
    tick: u64,
    t: f64,
    car: u32,
    text: String,
}

/// A buffer that releases race output only for confirmed hash intervals.
pub struct ConfirmedLines {
    hash_every: u64,
    next: Option<u64>,
    confirmed_through: Option<u64>,
    lines: Vec<Line>,
    stopped: bool,
}

impl ConfirmedLines {
    /// Interval i is ticks i*N through i*N + N - 1, where N is `hash_every`.
    pub fn new(hash_every: u64) -> Result<Self, ConfirmError> {
        if hash_every == 0 {
            return Err(ConfirmError::ZeroInterval);
        }
        Ok(Self {
            hash_every,
            next: Some(hash_every - 1),
            confirmed_through: None,
            lines: Vec::new(),
            stopped: false,
        })
    }

    /// Holds a whole event line without a terminator; finite times use total ordering.
    pub fn push(&mut self, tick: u64, t: f64, car: u32, line: String) -> Result<(), ConfirmError> {
        if self.stopped {
            return Err(ConfirmError::Stopped);
        }
        if !t.is_finite() {
            return Err(ConfirmError::BadTime { tick });
        }
        if line.is_empty() || line.contains(['\n', '\r']) {
            return Err(ConfirmError::BadLine { tick });
        }
        if let Some(confirmed_through) = self.confirmed_through {
            if tick <= confirmed_through {
                return Err(ConfirmError::LateLine {
                    tick,
                    confirmed_through,
                });
            }
        }
        self.lines.push(Line {
            tick,
            t,
            car,
            text: line,
        });
        Ok(())
    }

    /// Releases only this confirmed interval's lines, followed by its formatted hash.
    pub fn confirm(&mut self, hash_tick: u64, value: u64) -> Result<Vec<String>, ConfirmError> {
        let expected = self.next_hash_tick()?;
        if hash_tick != expected {
            return Err(ConfirmError::WrongHashTick {
                expected,
                got: hash_tick,
            });
        }
        let (mut ready, later): (Vec<_>, Vec<_>) =
            self.lines.drain(..).partition(|line| line.tick <= hash_tick);
        ready.sort_by(|a, b| a.t.total_cmp(&b.t).then_with(|| a.car.cmp(&b.car)));
        let mut output: Vec<_> = ready.into_iter().map(|line| line.text).collect();
        output.push(format!("hash tick={hash_tick} value={value:016x}"));
        self.lines = later;
        self.confirmed_through = Some(hash_tick);
        self.next = hash_tick.checked_add(self.hash_every);
        Ok(output)
    }

    /// Drops all pending lines and permanently stops the buffer.
    pub fn discard(&mut self) -> usize {
        let count = self.lines.len();
        self.lines.clear();
        self.stopped = true;
        count
    }

    /// Returns the number of held, unreleased lines.
    pub fn pending(&self) -> usize {
        self.lines.len()
    }

    /// Returns the next interval's last tick, or a named stopped/overflow error.
    pub fn next_hash_tick(&self) -> Result<u64, ConfirmError> {
        if self.stopped {
            return Err(ConfirmError::Stopped);
        }
        self.next.ok_or(ConfirmError::Overflow)
    }
}
