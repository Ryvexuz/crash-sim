//! One TCP peer: validate and build before connecting, then release only confirmed intervals.

use super::net_race;
use super::race::{race_fail_line, HeadingGroups};
use crash_sim::net::confirm::ConfirmedLines;
use crash_sim::net::session::{
    Input, LockstepSim, Role, Session, SessionConfig, SessionError, TcpTransport, MAX_INPUT_DELAY,
};
use crash_sim::net::MAX_TICK_RATE;
use crash_sim::race::lockstep::{LockstepRace, LockstepSetup, TickEvent};
use crash_sim::track::Track;
use crash_sim::Config;
use std::cell::RefCell;
use std::collections::BTreeMap;
use std::io::ErrorKind;
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::rc::Rc;
use std::time::{Duration, Instant};

const USAGE: &str = "usage: crash-sim --body <car.bsn> --track <file> --grid-gap <m> --duration <s> (--host <addr:port> | --join <addr:port>) --input <file> --net-delay <ticks> --net-hash-every <ticks> --net-timeout <s> --net-connect-timeout <s> --net-seed <u64>";
const REQUIRED: [&str; 10] = [
    "--body",
    "--track",
    "--grid-gap",
    "--duration",
    "--input",
    "--net-delay",
    "--net-hash-every",
    "--net-timeout",
    "--net-connect-timeout",
    "--net-seed",
];

struct Flags<'a> {
    values: BTreeMap<&'a str, &'a str>,
    addr: SocketAddr,
    address_text: &'a str,
    role: Role,
    gap: f64,
    ticks: u64,
    connect_timeout: Duration,
    session: SessionConfig,
    cfg: Config,
}

fn positive(flag: &str, text: &str) -> Result<f64, String> {
    text.parse::<f64>()
        .ok()
        .filter(|v| v.is_finite() && *v > 0.0)
        .ok_or_else(|| format!("{flag}: must be finite and above zero"))
}

fn integer(flag: &str, text: &str) -> Result<u64, String> {
    if text.is_empty()
        || !text.bytes().all(|b| b.is_ascii_digit())
        || (text.len() > 1 && text.starts_with('0'))
    {
        return Err(format!("{flag}: expected a canonical decimal integer"));
    }
    text.parse()
        .map_err(|_| format!("{flag}: integer is out of range"))
}

fn timeout(flag: &str, text: &str) -> Result<Duration, String> {
    let seconds = positive(flag, text)?;
    // The specified cap also keeps the conversion and socket deadlines representable.
    if seconds > 3600.0 {
        return Err(format!("{flag}: must be at most 3600 seconds"));
    }
    // Positive subnanosecond values use the smallest nonzero Duration, never a zero timeout.
    Duration::try_from_secs_f64(seconds)
        .map(|v| v.max(Duration::from_nanos(1)))
        .map_err(|_| format!("{flag}: timeout is out of range"))
}

impl<'a> Flags<'a> {
    fn parse(args: &'a [String]) -> Result<Self, String> {
        let mut values = BTreeMap::new();
        let mut iter = args.iter();
        while let Some(flag) = iter.next() {
            let flag = flag.as_str();
            if !REQUIRED.contains(&flag) && !matches!(flag, "--host" | "--join") {
                return Err(format!("{flag}: unknown or incompatible flag"));
            }
            if values.contains_key(flag) {
                return Err(format!("{flag}: repeated flag"));
            }
            let value = iter
                .next()
                .filter(|v| !v.starts_with("--"))
                .ok_or_else(|| format!("{flag}: needs a value"))?;
            if value.is_empty()
                || value
                    .chars()
                    .any(|c| c.is_control() || matches!(c, '\u{2028}' | '\u{2029}'))
            {
                return Err(format!(
                    "{flag}: empty value or forbidden line/control character"
                ));
            }
            values.insert(flag, value.as_str());
        }
        for flag in REQUIRED {
            if !values.contains_key(flag) {
                return Err(format!("{flag}: required flag is missing"));
            }
        }
        let (role, address_text, flag) = match (values.get("--host"), values.get("--join")) {
            (Some(addr), None) => (Role::Host, *addr, "--host"),
            (None, Some(addr)) => (Role::Guest, *addr, "--join"),
            _ => return Err("--host/--join: expected exactly one".into()),
        };
        let addr = address_text
            .parse::<SocketAddr>()
            .map_err(|_| format!("{flag}: expected a socket address"))?;
        let cfg = Config::default();
        let gap = positive("--grid-gap", values["--grid-gap"])?;
        let duration = positive("--duration", values["--duration"])?;
        let delay = integer("--net-delay", values["--net-delay"])?;
        if delay > MAX_INPUT_DELAY {
            return Err(format!("--net-delay: expected 0 to {MAX_INPUT_DELAY}"));
        }
        let hash_every = integer("--net-hash-every", values["--net-hash-every"])?;
        if hash_every == 0 {
            return Err("--net-hash-every: expected at least 1".into());
        }
        let ticks = net_race::duration_ticks(duration, &cfg, hash_every)?;
        let tick_rate = (1.0 / cfg.dt).round();
        if !tick_rate.is_finite() || !(1.0..=f64::from(MAX_TICK_RATE)).contains(&tick_rate) {
            return Err("default config: invalid network tick rate".into());
        }
        let session = SessionConfig {
            seed: integer("--net-seed", values["--net-seed"])?,
            tick_rate: tick_rate as u32,
            input_delay: delay,
            hash_every,
            timeout: timeout("--net-timeout", values["--net-timeout"])?,
        };
        let connect_timeout = timeout("--net-connect-timeout", values["--net-connect-timeout"])?;
        Ok(Self {
            values,
            addr,
            address_text,
            role,
            gap,
            ticks,
            connect_timeout,
            session,
            cfg,
        })
    }
}

struct PeerSim {
    race: LockstepRace,
    events: Rc<RefCell<Vec<TickEvent>>>,
}

impl LockstepSim for PeerSim {
    fn step(&mut self, tick: u64, inputs: [Input; 2]) {
        self.race.step(tick, inputs);
        self.events.borrow_mut().extend(self.race.take_events());
    }

    fn state_hash(&self) -> u64 {
        self.race.state_hash()
    }
}

struct Prepared {
    sim: PeerSim,
    heading: HeadingGroups,
    script: Vec<(f64, Input)>,
}

fn prepare(flags: &Flags<'_>) -> Result<Prepared, String> {
    let read = |flag: &str| {
        let path = flags.values[flag];
        std::fs::read_to_string(path).map_err(|e| format!("{flag} {path}: {e}"))
    };
    let body = crash_sim::format::parse(&read("--body")?)
        .map_err(|e| format!("{}: {e}", flags.values["--body"]))?;
    let track = Track::parse(&read("--track")?)
        .map_err(|e| format!("{}: {e}", flags.values["--track"]))?;
    let script = net_race::script("--input", flags.values["--input"])?;
    let heading =
        HeadingGroups::of(&body).ok_or_else(|| "the body has no heading groups".to_string())?;
    let race = LockstepRace::new(&LockstepSetup {
        cfg: &flags.cfg,
        body: &body,
        track: &track,
        grid_gap_m: flags.gap,
    })
    .map_err(|e| e.to_string())?;
    Ok(Prepared {
        sim: PeerSim {
            race,
            events: Rc::new(RefCell::new(Vec::new())),
        },
        heading,
        script,
    })
}

fn print_line(text: &str) {
    super::emit(&format!("{text}\n"));
}

fn connect(flags: &Flags<'_>) -> Result<(TcpStream, SocketAddr), String> {
    if flags.role == Role::Guest {
        let stream = TcpStream::connect_timeout(&flags.addr, flags.connect_timeout)
            .map_err(|e| format!("net: FAIL cannot connect to {}: {e}", flags.address_text))?;
        let addr = stream.peer_addr().map_err(|e| format!("net: FAIL {e}"))?;
        return Ok((stream, addr));
    }
    let listener = TcpListener::bind(flags.addr)
        .map_err(|e| format!("net: FAIL cannot listen on {}: {e}", flags.address_text))?;
    let addr = listener.local_addr().map_err(|e| format!("net: FAIL {e}"))?;
    print_line(&format!("net listening addr={addr}"));
    listener
        .set_nonblocking(true)
        .map_err(|e| format!("net: FAIL {e}"))?;
    let started = Instant::now();
    loop {
        if started.elapsed() >= flags.connect_timeout {
            return Err(format!(
                "net: FAIL no peer within {:.3} s",
                flags.connect_timeout.as_secs_f64()
            ));
        }
        match listener.accept() {
            Ok((stream, _)) => {
                stream
                    .set_nonblocking(false)
                    .map_err(|e| format!("net: FAIL {e}"))?;
                return Ok((stream, addr));
            }
            Err(e) if accept_error_is_retried(e.kind()) => {
                std::thread::sleep(
                    Duration::from_millis(10)
                        .min(flags.connect_timeout.saturating_sub(started.elapsed())),
                );
            }
            Err(e) => return Err(format!("net: FAIL {e}")),
        }
    }
}

/// An accept error that leaves the listener usable, so the host keeps waiting (within the
/// connect timeout): no connection yet, a signal, or one incoming connection that was reset or
/// aborted before it was accepted (a port scan, a peer that gave up). Anything else ends the host.
fn accept_error_is_retried(kind: ErrorKind) -> bool {
    matches!(
        kind,
        ErrorKind::WouldBlock
            | ErrorKind::Interrupted
            | ErrorKind::ConnectionReset
            | ErrorKind::ConnectionAborted
    )
}

fn session_failure(error: SessionError) -> String {
    match error {
        SessionError::Desync { tick, local, remote } => {
            format!("net: FAIL desync at tick {tick} own={local:016x} peer={remote:016x}")
        }
        SessionError::Timeout { tick, waited } => format!(
            "net: FAIL timeout at tick {tick} after {:.3} s",
            waited.as_secs_f64()
        ),
        SessionError::Disconnected { tick } => format!("net: FAIL disconnected at tick {tick}"),
        SessionError::PeerLeft { tick } => format!("net: FAIL peer left at tick {tick}"),
        other => format!("net: FAIL {other}"),
    }
}

fn drive(flags: &Flags<'_>, prepared: Prepared, lines: &mut ConfirmedLines) -> Result<(), String> {
    let (stream, addr) = connect(flags)?;
    let transport =
        TcpTransport::new(stream, flags.session.timeout).map_err(|e| format!("net: FAIL {e}"))?;
    let events = Rc::clone(&prepared.sim.events);
    let mut session = Session::connect(transport, prepared.sim, flags.session, flags.role)
        .map_err(session_failure)?;
    let role = if flags.role == Role::Host {
        "host"
    } else {
        "guest"
    };
    print_line(&format!(
        "net role={role} addr={addr} tick_rate={} input_delay={} hash_every={} timeout_s={:.3} seed={}",
        flags.session.tick_rate,
        flags.session.input_delay,
        flags.session.hash_every,
        flags.session.timeout.as_secs_f64(),
        flags.session.seed,
    ));
    let dt = flags.cfg.dt;
    let tol = dt / f64::from(flags.cfg.substeps) * 1e-6;
    let mut hashes = 0_u64;
    let mut last_hash = 0_u64;
    for tick in 0..flags.ticks {
        let local = net_race::input_at(&prepared.script, tick, 0, dt, tol);
        let step = session.advance(local).map_err(session_failure)?;
        if let Some(failure) = session.sim().race.failure() {
            let line = race_fail_line(tick as f64 * dt, &format!("{failure:?}"));
            return Err(line.trim_end().to_string());
        }
        for event in events.borrow_mut().drain(..) {
            let (t, line) = net_race::event_line(&event).map_err(|e| match e {
                net_race::Error::Usage(e)
                | net_race::Error::Setup(e)
                | net_race::Error::Internal(e) => format!("net: FAIL {e}"),
            })?;
            lines
                .push(event.tick, t, event.car, line)
                .map_err(|e| format!("net: FAIL {e}"))?;
        }
        if let Some(hash) = step.checked_hash {
            for line in lines
                .confirm(step.tick, hash)
                .map_err(|e| format!("net: FAIL {e}"))?
            {
                print_line(&line);
            }
            hashes += 1;
            last_hash = hash;
        }
    }
    let sim = session.close();
    for car in 0..2 {
        super::emit(&net_race::final_line(
            &prepared.heading,
            &sim.race,
            car,
            flags.ticks as f64 * dt,
        ));
    }
    super::emit(&net_race::cars_line(&sim.race));
    print_line(&format!(
        "net: ok ticks={} hashes={hashes} last_hash={last_hash:016x}",
        flags.ticks
    ));
    Ok(())
}

pub fn run_peer_cli(args: &[String]) -> i32 {
    let setup = Flags::parse(args).and_then(|flags| prepare(&flags).map(|p| (flags, p)));
    let (flags, prepared) = match setup {
        Ok(value) => value,
        Err(e) => {
            eprintln!("crash-sim: {e}\n{USAGE}");
            return 2;
        }
    };
    let mut lines = match ConfirmedLines::new(flags.session.hash_every) {
        Ok(lines) => lines,
        Err(e) => {
            print_line(&format!("net: FAIL {e}"));
            return 1;
        }
    };
    match drive(&flags, prepared, &mut lines) {
        Ok(()) => 0,
        Err(e) => {
            lines.discard();
            print_line(&e);
            1
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn accept_errors_that_keep_the_listener_are_retried() {
        let retried: [ErrorKind; 4] = [
            ErrorKind::WouldBlock,
            ErrorKind::Interrupted,
            ErrorKind::ConnectionReset,
            ErrorKind::ConnectionAborted,
        ];
        for k in retried {
            assert!(accept_error_is_retried(k), "{k:?}");
        }
        let fatal: [ErrorKind; 7] = [
            ErrorKind::AddrInUse,
            ErrorKind::PermissionDenied,
            ErrorKind::InvalidInput,
            ErrorKind::OutOfMemory,
            ErrorKind::Other,
            ErrorKind::NotFound,
            ErrorKind::TimedOut,
        ];
        for k in fatal {
            assert!(!accept_error_is_retried(k), "{k:?}");
        }
    }
}
