//! Lockstep session (n20-25b, Stage 7 m4): two peers step one deterministic sim in lockstep and
//! exchange only inputs and state hashes, as the one-line messages of `net.rs` (`docs/NET.md`).
//! Std only.
//!
//! Protocol, as this module runs it:
//!
//! - **Hello.** Each peer sends `hello` and waits for the other's. The seed and tick rate must
//!   be equal on both sides, or the session is refused (`HelloMismatch`); a protocol version
//!   other than 1 is refused by the codec (`Codec(WrongVersion)`). The tick rate is only agreed
//!   here: the caller paces the ticks.
//! - **Input delay D.** The input a peer gives at tick t is sent at once and applied at tick
//!   t + D. The first D ticks use the neutral input, which each peer sends explicitly right
//!   after the hello, so every tick from 0 has one input message from each peer.
//! - **Lockstep.** A peer steps tick t only when it holds both peers' inputs for t. A late input
//!   is waited for, never guessed. Inputs must arrive in tick order with no gap and no repeat
//!   (`InputGap`, `DuplicateInput`), and never further ahead than the peer could honestly be
//!   (`InputTooFarAhead`), so the buffer stays bounded.
//! - **Hash every N ticks.** After stepping tick t with (t + 1) % N == 0, a peer sends
//!   `hash t <state hash>` and does not return from that tick until the other peer's hash for t
//!   has arrived. If they differ, the session stops with `desync at tick t` and both hashes
//!   (`Desync`); because both peers hold both hashes, both peers report it at the same tick.
//!   A hash for any other tick, or for a tick the peer cannot have stepped yet, is refused.
//! - **Timeout T.** Each wait for a message lasts at most T. A peer silent for longer ends the
//!   session with `Timeout`, never a hang.
//! - **Ending.** `bye` from the peer ends it with `PeerLeft`; a closed connection with
//!   `Disconnected`; any line the codec refuses with `Codec`. A failed send to a peer that is
//!   gone is not an error by itself (a peer that finished first needs none of our later inputs):
//!   what it sent before leaving is still read, and the session ends only when it needs a
//!   message that is not there. Every error stops the session for good: it sends `bye` (best
//!   effort) and every later `advance` returns the same error.
//!
//! D and N are not carried in `hello` (its wire format is fixed in `docs/NET.md`), so both peers
//! must be configured with the same values. A mismatch is not hidden: it shows up as a
//! sequence error, an unexpected hash or a desync.
//!
//! Transports: [`TcpTransport`] (a `std::net::TcpStream`, one `\n`-terminated line per message)
//! and [`ChannelTransport`] (an in-process `std::sync::mpsc` pair, one `String` per message).

use std::collections::VecDeque;
use std::fmt;
use std::io::{ErrorKind, Read, Write};
use std::net::{Shutdown, TcpStream};
use std::sync::mpsc;
use std::time::{Duration, Instant};

use super::{decode, encode, Msg, NetError, INPUT_SCALE, MAX_LINE_BYTES, MAX_TICK_RATE};

/// Highest accepted input delay in ticks (the neutral inputs for the first D ticks are sent at once).
pub const MAX_INPUT_DELAY: u64 = 1000;

/// One peer's controls for one tick, in 1/1000 units: throttle and brake 0..=1000,
/// steer -1000..=1000 (the ranges of the `input` message).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Input {
    pub throttle: i32,
    pub brake: i32,
    pub steer: i32,
}

impl Input {
    /// No throttle, no brake, straight ahead: the input of the first D ticks.
    pub const NEUTRAL: Input = Input {
        throttle: 0,
        brake: 0,
        steer: 0,
    };

    pub fn in_range(&self) -> bool {
        (0..=INPUT_SCALE).contains(&self.throttle)
            && (0..=INPUT_SCALE).contains(&self.brake)
            && (-INPUT_SCALE..=INPUT_SCALE).contains(&self.steer)
    }
}

/// Which slot a peer's inputs fill in `LockstepSim::step`: the host's input is `inputs[0]` and
/// the guest's `inputs[1]` on both machines. The two peers must take different roles.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Role {
    Host,
    Guest,
}

impl Role {
    fn local_index(self) -> usize {
        match self {
            Role::Host => 0,
            Role::Guest => 1,
        }
    }
}

/// The deterministic simulation both peers run. Same inputs in the same order must give the
/// same `state_hash` on both machines.
pub trait LockstepSim {
    /// Step tick `tick` with `inputs[0]` from the host and `inputs[1]` from the guest.
    fn step(&mut self, tick: u64, inputs: [Input; 2]);
    fn state_hash(&self) -> u64;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SessionConfig {
    /// Must equal the peer's (checked in `hello`).
    pub seed: u64,
    /// Ticks per second, 1..=MAX_TICK_RATE. Must equal the peer's (checked in `hello`).
    pub tick_rate: u32,
    /// D: ticks between giving an input and applying it, 0..=MAX_INPUT_DELAY. Must equal the peer's.
    pub input_delay: u64,
    /// N: a hash is exchanged after every tick t with (t + 1) % N == 0. At least 1. Must equal the peer's.
    pub hash_every: u64,
    /// T: the longest wait for any one message from the peer. Must be more than zero.
    pub timeout: Duration,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SessionError {
    /// A config value out of range; nothing was sent.
    Config(&'static str),
    /// The caller gave an input outside the message ranges.
    LocalInputOutOfRange(Input),
    /// The transport reported an I/O error other than a closed connection (a send timeout, for example).
    Transport(String),
    /// The peer closed the connection without `bye`, and a message the session needs is not there.
    Disconnected { tick: u64 },
    /// No message from the peer for `waited` while the session needed one.
    Timeout { tick: u64, waited: Duration },
    /// The peer sent `bye`.
    PeerLeft { tick: u64 },
    /// The peer sent a line the codec refuses (including a protocol version other than 1).
    Codec(NetError),
    /// The peer's first message was not `hello`.
    ExpectedHello { got: &'static str },
    HelloMismatch {
        local_seed: u64,
        local_tick_rate: u32,
        remote_seed: u64,
        remote_tick_rate: u32,
    },
    /// A second `hello` after the session started.
    UnexpectedHello,
    /// An input for a tick the peer already sent.
    DuplicateInput { tick: u64 },
    /// An input that skips ticks.
    InputGap { expected: u64, got: u64 },
    /// An input further ahead than an honest peer can be, given the inputs sent to it.
    InputTooFarAhead { tick: u64, limit: u64 },
    /// A hash for a tick that is not the next hash tick.
    UnexpectedHash { expected: u64, got: u64 },
    /// A hash for a tick the peer cannot have stepped yet.
    HashTooEarly { tick: u64 },
    /// The two peers' state hashes differ after tick `tick`.
    Desync { tick: u64, local: u64, remote: u64 },
}

impl fmt::Display for SessionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SessionError::Config(what) => write!(f, "bad session config: {what}"),
            SessionError::LocalInputOutOfRange(i) => write!(
                f,
                "local input out of range: throttle {} brake {} steer {}",
                i.throttle, i.brake, i.steer
            ),
            SessionError::Transport(e) => write!(f, "transport error: {e}"),
            SessionError::Disconnected { tick } => {
                write!(f, "peer disconnected at tick {tick}")
            }
            SessionError::Timeout { tick, waited } => write!(
                f,
                "peer silent for more than {} ms at tick {tick}",
                waited.as_millis()
            ),
            SessionError::PeerLeft { tick } => write!(f, "peer left (bye) at tick {tick}"),
            SessionError::Codec(e) => write!(f, "peer sent a bad message: {e}"),
            SessionError::ExpectedHello { got } => write!(f, "expected hello, got {got}"),
            SessionError::HelloMismatch {
                local_seed,
                local_tick_rate,
                remote_seed,
                remote_tick_rate,
            } => write!(
                f,
                "hello mismatch: local seed {local_seed} tick rate {local_tick_rate}, \
                 remote seed {remote_seed} tick rate {remote_tick_rate}"
            ),
            SessionError::UnexpectedHello => {
                write!(f, "unexpected hello after the session started")
            }
            SessionError::DuplicateInput { tick } => {
                write!(f, "duplicate input for tick {tick}")
            }
            SessionError::InputGap { expected, got } => {
                write!(f, "input for tick {got}, expected tick {expected}")
            }
            SessionError::InputTooFarAhead { tick, limit } => {
                write!(f, "input for tick {tick} is too far ahead (limit {limit})")
            }
            SessionError::UnexpectedHash { expected, got } => {
                write!(f, "hash for tick {got}, expected tick {expected}")
            }
            SessionError::HashTooEarly { tick } => {
                write!(f, "hash for tick {tick} before the peer could step it")
            }
            SessionError::Desync {
                tick,
                local,
                remote,
            } => write!(
                f,
                "desync at tick {tick}: local hash {local:016x}, remote hash {remote:016x}"
            ),
        }
    }
}

impl std::error::Error for SessionError {}

/// Why `Transport::recv_line` returned no line.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RecvError {
    Timeout,
    Closed,
    /// More than MAX_LINE_BYTES arrived without a line end.
    TooLong,
    Io(String),
}

/// Why `Transport::send_line` could not send.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SendError {
    /// The peer is gone (closed channel, reset or broken connection). The session reads on.
    Closed,
    Io(String),
}

/// Carries one message line each way. Lines never contain the terminator.
pub trait Transport {
    fn send_line(&mut self, line: &str) -> Result<(), SendError>;
    /// The next line, waiting at most `timeout`.
    fn recv_line(&mut self, timeout: Duration) -> Result<String, RecvError>;
}

/// In-process transport: one `String` per message over a pair of mpsc channels.
pub struct ChannelTransport {
    tx: mpsc::Sender<String>,
    rx: mpsc::Receiver<String>,
}

/// Two connected ends: what one sends, the other receives.
pub fn channel_pair() -> (ChannelTransport, ChannelTransport) {
    let (tx_a, rx_b) = mpsc::channel();
    let (tx_b, rx_a) = mpsc::channel();
    (
        ChannelTransport { tx: tx_a, rx: rx_a },
        ChannelTransport { tx: tx_b, rx: rx_b },
    )
}

impl Transport for ChannelTransport {
    fn send_line(&mut self, line: &str) -> Result<(), SendError> {
        self.tx
            .send(line.to_string())
            .map_err(|_| SendError::Closed)
    }

    fn recv_line(&mut self, timeout: Duration) -> Result<String, RecvError> {
        match self.rx.recv_timeout(timeout) {
            Ok(line) => Ok(line),
            Err(mpsc::RecvTimeoutError::Timeout) => Err(RecvError::Timeout),
            Err(mpsc::RecvTimeoutError::Disconnected) => Err(RecvError::Closed),
        }
    }
}

/// TCP transport: each message is its line plus `\n`. Nagle is turned off, since lockstep waits
/// on every small message. Bytes that are not UTF-8 are passed on replaced (U+FFFD), so the
/// codec refuses them as non-ASCII.
///
/// Dropping it closes gracefully: it shuts down its sending side and reads (and discards) until
/// the peer's end of stream, for at most the `timeout` given to `new`. Closing a socket with
/// unread data makes the OS reset the connection, and a reset can discard data the peer has
/// received but not yet read, such as the last input or hash it needs.
pub struct TcpTransport {
    stream: TcpStream,
    pending: Vec<u8>,
    close_timeout: Duration,
}

impl TcpTransport {
    /// `timeout` bounds each send, so a peer that stops reading cannot hang the session, and
    /// the graceful close on drop.
    pub fn new(stream: TcpStream, timeout: Duration) -> std::io::Result<TcpTransport> {
        stream.set_nodelay(true)?;
        stream.set_write_timeout(Some(timeout))?;
        Ok(TcpTransport {
            stream,
            pending: Vec::new(),
            close_timeout: timeout,
        })
    }
}

impl Drop for TcpTransport {
    fn drop(&mut self) {
        let _ = self.stream.shutdown(Shutdown::Write);
        let deadline = Instant::now() + self.close_timeout;
        let mut chunk = [0u8; 512];
        loop {
            let now = Instant::now();
            if now >= deadline || self.stream.set_read_timeout(Some(deadline - now)).is_err() {
                return;
            }
            match self.stream.read(&mut chunk) {
                Ok(0) => return,
                Ok(_) => {}
                Err(e) => match e.kind() {
                    ErrorKind::WouldBlock | ErrorKind::TimedOut | ErrorKind::Interrupted => {}
                    _ => return,
                },
            }
        }
    }
}

impl Transport for TcpTransport {
    fn send_line(&mut self, line: &str) -> Result<(), SendError> {
        let mut bytes = Vec::with_capacity(line.len() + 1);
        bytes.extend_from_slice(line.as_bytes());
        bytes.push(b'\n');
        self.stream.write_all(&bytes).map_err(|e| match e.kind() {
            ErrorKind::ConnectionReset | ErrorKind::ConnectionAborted | ErrorKind::BrokenPipe => {
                SendError::Closed
            }
            _ => SendError::Io(e.to_string()),
        })
    }

    fn recv_line(&mut self, timeout: Duration) -> Result<String, RecvError> {
        let deadline = Instant::now() + timeout;
        let mut chunk = [0u8; 512];
        loop {
            if let Some(pos) = self.pending.iter().position(|&b| b == b'\n') {
                let mut line: Vec<u8> = self.pending.drain(..=pos).collect();
                line.pop();
                return Ok(String::from_utf8_lossy(&line).into_owned());
            }
            // No line end in the buffer: the line is already longer than any the codec accepts.
            if self.pending.len() > MAX_LINE_BYTES {
                return Err(RecvError::TooLong);
            }
            let now = Instant::now();
            if now >= deadline {
                return Err(RecvError::Timeout);
            }
            self.stream
                .set_read_timeout(Some(deadline - now))
                .map_err(|e| RecvError::Io(e.to_string()))?;
            match self.stream.read(&mut chunk) {
                Ok(0) => return Err(RecvError::Closed),
                Ok(n) => self.pending.extend_from_slice(&chunk[..n]),
                Err(e) => match e.kind() {
                    ErrorKind::WouldBlock | ErrorKind::TimedOut | ErrorKind::Interrupted => {}
                    ErrorKind::ConnectionReset
                    | ErrorKind::ConnectionAborted
                    | ErrorKind::BrokenPipe
                    | ErrorKind::UnexpectedEof => return Err(RecvError::Closed),
                    _ => return Err(RecvError::Io(e.to_string())),
                },
            }
        }
    }
}

/// What one `advance` did.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Step {
    /// The tick just stepped.
    pub tick: u64,
    /// The state hash after it, when this was a hash tick and both peers' hashes matched.
    pub checked_hash: Option<u64>,
}

pub struct Session<T: Transport, S: LockstepSim> {
    transport: T,
    sim: S,
    cfg: SessionConfig,
    role: Role,
    /// The next tick to step.
    tick: u64,
    /// Local inputs sent so far: ticks 0..sent.
    sent: u64,
    /// Local inputs for ticks tick..sent, not yet applied.
    local_inputs: VecDeque<Input>,
    /// Remote inputs for ticks tick..tick + len, received but not yet applied.
    remote_inputs: VecDeque<Input>,
    /// Remote hashes received, by tick, not yet compared.
    remote_hashes: VecDeque<(u64, u64)>,
    /// The tick of the next remote hash.
    next_remote_hash: u64,
    stopped: Option<SessionError>,
}

impl<T: Transport, S: LockstepSim> Session<T, S> {
    /// Exchanges `hello`, checks the peer agrees, and sends the neutral inputs of the first D ticks.
    pub fn connect(
        transport: T,
        sim: S,
        cfg: SessionConfig,
        role: Role,
    ) -> Result<Session<T, S>, SessionError> {
        if !(1..=MAX_TICK_RATE).contains(&cfg.tick_rate) {
            return Err(SessionError::Config("tick_rate must be 1..=MAX_TICK_RATE"));
        }
        if cfg.input_delay > MAX_INPUT_DELAY {
            return Err(SessionError::Config(
                "input_delay must be 0..=MAX_INPUT_DELAY",
            ));
        }
        if cfg.hash_every == 0 {
            return Err(SessionError::Config("hash_every must be at least 1"));
        }
        if cfg.timeout.is_zero() {
            return Err(SessionError::Config("timeout must be more than zero"));
        }
        let mut s = Session {
            transport,
            sim,
            cfg,
            role,
            tick: 0,
            sent: 0,
            local_inputs: VecDeque::new(),
            remote_inputs: VecDeque::new(),
            remote_hashes: VecDeque::new(),
            next_remote_hash: cfg.hash_every - 1,
            stopped: None,
        };
        match s.handshake() {
            Ok(()) => Ok(s),
            Err(e) => Err(s.fail(e)),
        }
    }

    /// The next tick to step (also the number of ticks stepped).
    pub fn tick(&self) -> u64 {
        self.tick
    }

    pub fn sim(&self) -> &S {
        &self.sim
    }

    /// Gives `local` for tick `tick() + D`, waits for the peer's input for `tick()`, steps it,
    /// and on a hash tick exchanges and compares hashes before returning.
    pub fn advance(&mut self, local: Input) -> Result<Step, SessionError> {
        if let Some(e) = &self.stopped {
            return Err(e.clone());
        }
        match self.advance_inner(local) {
            Ok(step) => Ok(step),
            Err(e) => Err(self.fail(e)),
        }
    }

    /// Sends `bye` (best effort) and returns the sim.
    pub fn close(mut self) -> S {
        let _ = self.transport.send_line(&encode(&Msg::Bye));
        self.sim
    }

    fn handshake(&mut self) -> Result<(), SessionError> {
        self.send(&Msg::Hello {
            seed: self.cfg.seed,
            tick_rate: self.cfg.tick_rate,
        })?;
        match self.recv()? {
            Msg::Hello { seed, tick_rate } => {
                if seed != self.cfg.seed || tick_rate != self.cfg.tick_rate {
                    return Err(SessionError::HelloMismatch {
                        local_seed: self.cfg.seed,
                        local_tick_rate: self.cfg.tick_rate,
                        remote_seed: seed,
                        remote_tick_rate: tick_rate,
                    });
                }
            }
            Msg::Bye => return Err(SessionError::PeerLeft { tick: 0 }),
            other => return Err(SessionError::ExpectedHello { got: word(&other) }),
        }
        for _ in 0..self.cfg.input_delay {
            self.send_local(Input::NEUTRAL)?;
        }
        Ok(())
    }

    fn advance_inner(&mut self, local: Input) -> Result<Step, SessionError> {
        if !local.in_range() {
            return Err(SessionError::LocalInputOutOfRange(local));
        }
        let t = self.tick;
        self.send_local(local)?;
        // Lockstep: wait for the peer's input for t; never step without it.
        while self.remote_inputs.is_empty() {
            let msg = self.recv()?;
            self.handle(msg)?;
        }
        let remote = self.remote_inputs.pop_front().expect("checked non-empty");
        let mine = self
            .local_inputs
            .pop_front()
            .expect("an input for every tick below sent");
        let mut inputs = [Input::NEUTRAL; 2];
        inputs[self.role.local_index()] = mine;
        inputs[1 - self.role.local_index()] = remote;
        self.sim.step(t, inputs);
        self.tick = t + 1;

        if !(t + 1).is_multiple_of(self.cfg.hash_every) {
            return Ok(Step {
                tick: t,
                checked_hash: None,
            });
        }
        let local_hash = self.sim.state_hash();
        self.send(&Msg::Hash {
            tick: t,
            hash: local_hash,
        })?;
        // Remote hashes arrive in hash-tick order, each checked against next_remote_hash, and are
        // compared one per hash tick, so the front is the hash for t once one has arrived.
        while self.remote_hashes.is_empty() {
            let msg = self.recv()?;
            self.handle(msg)?;
        }
        let (rt, remote_hash) = self.remote_hashes.pop_front().expect("checked non-empty");
        if rt != t {
            // Unreachable while `handle` keeps hashes in sequence; checked so it can never pass silently.
            return Err(SessionError::UnexpectedHash {
                expected: t,
                got: rt,
            });
        }
        if remote_hash != local_hash {
            return Err(SessionError::Desync {
                tick: t,
                local: local_hash,
                remote: remote_hash,
            });
        }
        Ok(Step {
            tick: t,
            checked_hash: Some(local_hash),
        })
    }

    /// Checks one message from the peer and buffers it.
    fn handle(&mut self, msg: Msg) -> Result<(), SessionError> {
        match msg {
            Msg::Hello { .. } => Err(SessionError::UnexpectedHello),
            Msg::Bye => Err(SessionError::PeerLeft { tick: self.tick }),
            Msg::Input {
                tick,
                throttle,
                brake,
                steer,
            } => {
                let expected = self.remote_received();
                if tick < expected {
                    return Err(SessionError::DuplicateInput { tick });
                }
                if tick > expected {
                    return Err(SessionError::InputGap {
                        expected,
                        got: tick,
                    });
                }
                // The peer can step every tick below `sent` and then send its input for the
                // tick after that plus D, so its highest honest input tick is sent + D.
                let limit = self.sent + self.cfg.input_delay;
                if tick > limit {
                    return Err(SessionError::InputTooFarAhead { tick, limit });
                }
                self.remote_inputs.push_back(Input {
                    throttle,
                    brake,
                    steer,
                });
                Ok(())
            }
            Msg::Hash { tick, hash } => {
                if tick != self.next_remote_hash {
                    return Err(SessionError::UnexpectedHash {
                        expected: self.next_remote_hash,
                        got: tick,
                    });
                }
                // To hash tick h the peer must have stepped it: it needs our input for h, and it
                // sends its own input for h before stepping.
                if tick >= self.sent || tick >= self.remote_received() {
                    return Err(SessionError::HashTooEarly { tick });
                }
                self.remote_hashes.push_back((tick, hash));
                self.next_remote_hash += self.cfg.hash_every;
                Ok(())
            }
        }
    }

    /// Remote inputs received so far: ticks 0..remote_received().
    fn remote_received(&self) -> u64 {
        self.tick + self.remote_inputs.len() as u64
    }

    fn send_local(&mut self, input: Input) -> Result<(), SessionError> {
        self.send(&Msg::Input {
            tick: self.sent,
            throttle: input.throttle,
            brake: input.brake,
            steer: input.steer,
        })?;
        self.local_inputs.push_back(input);
        self.sent += 1;
        Ok(())
    }

    fn send(&mut self, msg: &Msg) -> Result<(), SessionError> {
        match self.transport.send_line(&encode(msg)) {
            Ok(()) => Ok(()),
            // Not an error by itself: a peer that finished its last tick and left needs none of
            // our later inputs. What it sent before leaving is still read; the session ends
            // (PeerLeft or Disconnected) only when it needs a message that is not there.
            Err(SendError::Closed) => Ok(()),
            Err(SendError::Io(e)) => Err(SessionError::Transport(e)),
        }
    }

    fn recv(&mut self) -> Result<Msg, SessionError> {
        match self.transport.recv_line(self.cfg.timeout) {
            Ok(line) => decode(&line).map_err(SessionError::Codec),
            Err(RecvError::Timeout) => Err(SessionError::Timeout {
                tick: self.tick,
                waited: self.cfg.timeout,
            }),
            Err(RecvError::Closed) => Err(SessionError::Disconnected { tick: self.tick }),
            Err(RecvError::TooLong) => Err(SessionError::Codec(NetError::TooLong)),
            Err(RecvError::Io(e)) => Err(SessionError::Transport(e)),
        }
    }

    /// Stops the session for good: records the error, tells the peer `bye` (best effort).
    fn fail(&mut self, e: SessionError) -> SessionError {
        self.stopped = Some(e.clone());
        let _ = self.transport.send_line(&encode(&Msg::Bye));
        e
    }
}

fn word(msg: &Msg) -> &'static str {
    match msg {
        Msg::Hello { .. } => "hello",
        Msg::Input { .. } => "input",
        Msg::Hash { .. } => "hash",
        Msg::Bye => "bye",
    }
}
