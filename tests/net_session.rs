//! Lockstep session tests (n20-25b). Two peers run as two threads over in-process channels, and
//! once over TCP on 127.0.0.1 with an OS-chosen port (no fixed ports). A scripted fake peer
//! (the other channel end, driven by the test) covers the misbehaving cases.

use std::net::{TcpListener, TcpStream};
use std::thread;
use std::time::{Duration, Instant};

use crash_sim::net::session::{
    channel_pair, ChannelTransport, Input, LockstepSim, RecvError, Role, SendError, Session,
    SessionConfig, SessionError, TcpTransport, Transport,
};
use crash_sim::net::{decode, encode, Msg, NetError};

fn mix(mut z: u64) -> u64 {
    z = z.wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// Small deterministic stand-in sim: every input of every tick feeds the state.
#[derive(Clone, Debug)]
struct StandIn {
    state: u64,
    /// Every applied input pair, by tick, so tests can see exactly what was stepped.
    applied: Vec<[Input; 2]>,
}

impl StandIn {
    fn new(seed: u64) -> StandIn {
        StandIn {
            state: mix(seed),
            applied: Vec::new(),
        }
    }
}

fn pack(i: Input) -> u64 {
    ((i.throttle as u64) << 42) ^ ((i.brake as u64) << 21) ^ (i.steer as i64 as u64 & 0x1F_FFFF)
}

impl LockstepSim for StandIn {
    fn step(&mut self, tick: u64, inputs: [Input; 2]) {
        self.state = mix(self.state ^ mix(tick) ^ mix(pack(inputs[0])) ^ mix(pack(inputs[1]) ^ 1));
        self.applied.push(inputs);
    }
    fn state_hash(&self) -> u64 {
        self.state
    }
}

/// The input a peer seeded `peer_seed` gives at its local tick `t` (a pure function of both).
fn input_at(peer_seed: u64, t: u64) -> Input {
    let r = mix(peer_seed ^ mix(t));
    Input {
        throttle: (r % 1001) as i32,
        brake: ((r >> 16) % 1001) as i32,
        steer: ((r >> 32) % 2001) as i32 - 1000,
    }
}

const SEED: u64 = 42;
const HOST_INPUTS: u64 = 0xA11CE;
const GUEST_INPUTS: u64 = 0xB0B;

fn cfg(d: u64, n: u64, timeout_ms: u64) -> SessionConfig {
    SessionConfig {
        seed: SEED,
        tick_rate: 60,
        input_delay: d,
        hash_every: n,
        timeout: Duration::from_millis(timeout_ms),
    }
}

/// The hash a single machine gets stepping the same inputs with delay D: tick t applies what
/// each peer gave at tick t - D, and neutral input before D.
fn reference_hash(d: u64, ticks: u64) -> u64 {
    let mut sim = StandIn::new(SEED);
    for t in 0..ticks {
        let pick = |seed| {
            if t < d {
                Input::NEUTRAL
            } else {
                input_at(seed, t - d)
            }
        };
        sim.step(t, [pick(HOST_INPUTS), pick(GUEST_INPUTS)]);
    }
    sim.state_hash()
}

/// One peer: connect, give `input_at(input_seed, t)` for `ticks` ticks, close.
/// Returns the sim, the ticks whose hash was checked, or the error with the tick it hit.
fn run_peer<T: Transport>(
    transport: T,
    role: Role,
    c: SessionConfig,
    input_seed: u64,
    ticks: u64,
) -> Result<(StandIn, Vec<u64>), (SessionError, u64)> {
    run_peer_with(transport, role, c, input_seed, ticks, input_at)
}

/// `run_peer` with the per-tick input chosen by `input`.
fn run_peer_with<T: Transport>(
    transport: T,
    role: Role,
    c: SessionConfig,
    input_seed: u64,
    ticks: u64,
    input: fn(u64, u64) -> Input,
) -> Result<(StandIn, Vec<u64>), (SessionError, u64)> {
    let mut s = Session::connect(transport, StandIn::new(SEED), c, role).map_err(|e| (e, 0))?;
    let mut checked = Vec::new();
    for t in 0..ticks {
        match s.advance(input(input_seed, t)) {
            Ok(step) => {
                assert_eq!(step.tick, t);
                if step.checked_hash.is_some() {
                    checked.push(t);
                }
            }
            Err(e) => {
                // Errors are final: the next call returns the same error and steps nothing.
                let tick = s.tick();
                assert_eq!(s.advance(Input::NEUTRAL), Err(e.clone()));
                assert_eq!(s.tick(), tick);
                return Err((e, t));
            }
        }
    }
    Ok((s.close(), checked))
}

type PeerResult = Result<(StandIn, Vec<u64>), (SessionError, u64)>;

fn run_pair<A, B>(a: A, b: B, c: SessionConfig, ticks: u64) -> (PeerResult, PeerResult)
where
    A: Transport + Send + 'static,
    B: Transport + Send + 'static,
{
    let host = thread::spawn(move || run_peer(a, Role::Host, c, HOST_INPUTS, ticks));
    let guest = thread::spawn(move || run_peer(b, Role::Guest, c, GUEST_INPUTS, ticks));
    (host.join().unwrap(), guest.join().unwrap())
}

#[test]
fn thousand_ticks_of_seeded_inputs_end_with_equal_hashes() {
    let c = cfg(3, 60, 5000);
    let (a, b) = channel_pair();
    let (h, g) = run_pair(a, b, c, 1000);
    let (hs, hc) = h.expect("host ok");
    let (gs, gc) = g.expect("guest ok");
    assert_eq!(hs.state_hash(), gs.state_hash());
    // Both stepped exactly the same input pairs, and those are the delayed inputs.
    assert_eq!(hs.applied, gs.applied);
    assert_eq!(hs.state_hash(), reference_hash(3, 1000));
    for (t, pair) in hs.applied.iter().enumerate().take(3) {
        assert_eq!(*pair, [Input::NEUTRAL; 2], "tick {t} before D is neutral");
    }
    assert_eq!(
        hs.applied[3],
        [input_at(HOST_INPUTS, 0), input_at(GUEST_INPUTS, 0)]
    );
    // Hashes were exchanged and matched at ticks 59, 119, ..., 959.
    let expect: Vec<u64> = (0..1000).filter(|t| (t + 1) % 60 == 0).collect();
    assert_eq!(expect.len(), 16);
    assert_eq!(hc, expect);
    assert_eq!(gc, expect);

    // Determinism: a second run with the same inputs reaches the identical hash.
    let (a, b) = channel_pair();
    let (h2, g2) = run_pair(a, b, c, 1000);
    assert_eq!(h2.unwrap().0.state_hash(), hs.state_hash());
    assert_eq!(g2.unwrap().0.state_hash(), hs.state_hash());
    // And the inputs matter: a different delay gives a different hash.
    assert_ne!(reference_hash(2, 1000), hs.state_hash());
}

#[test]
fn thousand_ticks_over_tcp_loopback() {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind loopback");
    let addr = listener.local_addr().unwrap();
    let c = cfg(2, 60, 5000);
    let accept = thread::spawn(move || listener.accept().unwrap().0);
    let client = TcpStream::connect(addr).unwrap();
    let server = accept.join().unwrap();
    let t = Duration::from_secs(5);
    let a = TcpTransport::new(server, t).unwrap();
    let b = TcpTransport::new(client, t).unwrap();
    let start = Instant::now();
    let (h, g) = run_pair(a, b, c, 1000);
    let (hs, _) = h.expect("host ok");
    let (gs, gc) = g.expect("guest ok");
    assert_eq!(hs.state_hash(), gs.state_hash());
    assert_eq!(hs.state_hash(), reference_hash(2, 1000));
    assert_eq!(gc.len(), 16);
    assert!(
        start.elapsed() < Duration::from_secs(5),
        "{:?}",
        start.elapsed()
    );
}

/// Changes the throttle of the outgoing input line for one tick: the receiver applies a
/// different input from the one the sender applies.
struct Tamper {
    inner: ChannelTransport,
    tick: u64,
}

impl Transport for Tamper {
    fn send_line(&mut self, line: &str) -> Result<(), SendError> {
        let out = match decode(line) {
            Ok(Msg::Input {
                tick,
                throttle,
                brake,
                steer,
            }) if tick == self.tick => encode(&Msg::Input {
                tick,
                throttle: (throttle + 1) % 1001,
                brake,
                steer,
            }),
            _ => line.to_string(),
        };
        self.inner.send_line(&out)
    }
    fn recv_line(&mut self, timeout: Duration) -> Result<String, RecvError> {
        self.inner.recv_line(timeout)
    }
}

fn altered_input_case(altered: u64, expect_tick: u64) {
    let c = cfg(3, 60, 5000);
    let (a, b) = channel_pair();
    let a = Tamper {
        inner: a,
        tick: altered,
    };
    let (h, g) = run_pair(a, b, c, 1000);
    let (he, ht) = h.expect_err("host must stop");
    let (ge, gt) = g.expect_err("guest must stop");
    let (hl, hr) = match he {
        SessionError::Desync {
            tick,
            local,
            remote,
        } => {
            assert_eq!(tick, expect_tick);
            (local, remote)
        }
        other => panic!("host: expected desync, got {other}"),
    };
    match &ge {
        SessionError::Desync {
            tick,
            local,
            remote,
        } => {
            assert_eq!(*tick, expect_tick);
            // Each side reports both hashes: its own and the other's.
            assert_eq!((*local, *remote), (hr, hl));
        }
        other => panic!("guest: expected desync, got {other}"),
    }
    assert_ne!(hl, hr);
    assert_eq!((ht, gt), (expect_tick, expect_tick));
    let text = ge.to_string();
    assert!(
        text.starts_with(&format!("desync at tick {expect_tick}: local hash ")),
        "{text}"
    );
    assert!(text.contains(&format!("{hl:016x}")) && text.contains(&format!("{hr:016x}")));
}

#[test]
fn one_altered_input_is_a_desync_at_the_first_hash_after_it() {
    // Input for tick 500 altered on the wire: hashes 59..479 match, 539 is the first after 500.
    altered_input_case(500, 539);
}

#[test]
fn an_altered_input_on_a_hash_tick_is_caught_at_that_tick() {
    altered_input_case(59, 59);
}

// ---- scripted fake peer ----

fn say(fake: &mut ChannelTransport, line: &str) {
    fake.send_line(line).unwrap();
}

/// Everything the session sent to the fake so far.
fn heard(fake: &mut ChannelTransport) -> Vec<String> {
    let mut out = Vec::new();
    while let Ok(l) = fake.recv_line(Duration::from_millis(1)) {
        out.push(l);
    }
    out
}

fn connect(
    t: ChannelTransport,
    c: SessionConfig,
) -> Result<Session<ChannelTransport, StandIn>, SessionError> {
    Session::connect(t, StandIn::new(SEED), c, Role::Host)
}

/// Runs advance with neutral input until an error; panics after `max` ticks.
fn until_error(s: &mut Session<ChannelTransport, StandIn>, max: u64) -> SessionError {
    for _ in 0..max {
        if let Err(e) = s.advance(Input::NEUTRAL) {
            return e;
        }
    }
    panic!("no error within {max} ticks");
}

#[test]
fn version_mismatch_is_refused() {
    let (a, mut fake) = channel_pair();
    say(&mut fake, "hello 2 42 60");
    let e = connect(a, cfg(1, 60, 2000)).err().expect("refused");
    assert_eq!(e, SessionError::Codec(NetError::WrongVersion(2)));
    // The session said hello, sent nothing else, and told the peer bye.
    assert_eq!(heard(&mut fake), vec!["hello 1 42 60", "bye"]);
}

#[test]
fn seed_or_tick_rate_mismatch_is_refused() {
    let (a, mut fake) = channel_pair();
    say(&mut fake, "hello 1 43 60");
    match connect(a, cfg(1, 60, 2000)) {
        Err(SessionError::HelloMismatch {
            remote_seed: 43, ..
        }) => {}
        other => panic!("{:?}", other.err()),
    }
    let (a, mut fake) = channel_pair();
    say(&mut fake, "hello 1 42 30");
    match connect(a, cfg(1, 60, 2000)) {
        Err(SessionError::HelloMismatch {
            remote_tick_rate: 30,
            ..
        }) => {}
        other => panic!("{:?}", other.err()),
    }
    let (a, mut fake) = channel_pair();
    say(&mut fake, "input 0 0 0 0");
    assert_eq!(
        connect(a, cfg(1, 60, 2000)).err(),
        Some(SessionError::ExpectedHello { got: "input" })
    );
}

#[test]
fn a_silent_peer_times_out_within_twice_t() {
    let t = Duration::from_millis(300);
    // Silent during the hello.
    let (a, fake) = channel_pair();
    let start = Instant::now();
    let e = connect(a, cfg(1, 60, 300)).err().expect("timeout");
    let took = start.elapsed();
    assert!(matches!(e, SessionError::Timeout { tick: 0, .. }), "{e}");
    assert!(took >= t && took < 2 * t, "{took:?}");
    drop(fake);

    // Silent after the hello, while the session waits for the input for tick 0.
    let (a, mut fake) = channel_pair();
    say(&mut fake, "hello 1 42 60");
    let mut s = connect(a, cfg(1, 60, 300)).unwrap();
    let start = Instant::now();
    let e = s.advance(Input::NEUTRAL).unwrap_err();
    let took = start.elapsed();
    assert!(matches!(e, SessionError::Timeout { tick: 0, .. }), "{e}");
    assert!(took >= t && took < 2 * t, "{took:?}");
    assert_eq!(s.tick(), 0, "nothing stepped without the peer's input");
    drop(fake);
}

#[test]
fn a_late_input_is_waited_for_never_guessed() {
    let (a, mut fake) = channel_pair();
    say(&mut fake, "hello 1 42 60");
    say(&mut fake, "input 0 0 0 0");
    let mut s = connect(a, cfg(1, 60, 2000)).unwrap();
    s.advance(Input::NEUTRAL).unwrap();
    let late = thread::spawn(move || {
        thread::sleep(Duration::from_millis(150));
        say(&mut fake, "input 1 700 20 -300");
        fake
    });
    let start = Instant::now();
    let step = s.advance(Input::NEUTRAL).unwrap();
    assert!(
        start.elapsed() >= Duration::from_millis(140),
        "{:?}",
        start.elapsed()
    );
    assert_eq!(step.tick, 1);
    // The host slot is ours, the guest slot holds exactly what the peer sent.
    assert_eq!(
        s.sim().applied[1][1],
        Input {
            throttle: 700,
            brake: 20,
            steer: -300
        }
    );
    let _fake = late.join().unwrap();
}

#[test]
fn inputs_out_of_order_are_refused() {
    let (a, mut fake) = channel_pair();
    for l in [
        "hello 1 42 60",
        "input 0 0 0 0",
        "input 2 0 0 0",
        "input 1 0 0 0",
    ] {
        say(&mut fake, l);
    }
    let mut s = connect(a, cfg(1, 60, 2000)).unwrap();
    let e = until_error(&mut s, 5);
    assert_eq!(
        e,
        SessionError::InputGap {
            expected: 1,
            got: 2
        }
    );
    assert_eq!(s.tick(), 1, "tick 1 never stepped");
    assert_eq!(heard(&mut fake).last().map(String::as_str), Some("bye"));
}

#[test]
fn duplicated_inputs_are_refused() {
    let (a, mut fake) = channel_pair();
    for l in ["hello 1 42 60", "input 0 0 0 0", "input 0 0 0 0"] {
        say(&mut fake, l);
    }
    let mut s = connect(a, cfg(1, 60, 2000)).unwrap();
    assert_eq!(
        until_error(&mut s, 5),
        SessionError::DuplicateInput { tick: 0 }
    );
    assert_eq!(s.tick(), 1);
}

#[test]
fn inputs_too_far_ahead_are_refused() {
    // D = 1, hash every tick. After tick 0 the session has sent inputs 0 and 1, so the peer can
    // have stepped ticks 0 and 1 and sent its input for 2 + D = 3, never 4.
    let (a, mut fake) = channel_pair();
    for l in [
        "hello 1 42 60",
        "input 0 0 0 0",
        "input 1 0 0 0",
        "input 2 0 0 0",
        "input 3 0 0 0",
        "input 4 0 0 0",
    ] {
        say(&mut fake, l);
    }
    let mut s = connect(a, cfg(1, 1, 2000)).unwrap();
    assert_eq!(
        until_error(&mut s, 5),
        SessionError::InputTooFarAhead { tick: 4, limit: 3 }
    );
}

#[test]
fn hashes_for_the_wrong_tick_are_refused() {
    // Not a hash tick (N = 60, so the first is 59).
    let (a, mut fake) = channel_pair();
    for l in ["hello 1 42 60", "input 0 0 0 0", "hash 5 0000000000000001"] {
        say(&mut fake, l);
    }
    let mut s = connect(a, cfg(1, 60, 2000)).unwrap();
    assert_eq!(
        until_error(&mut s, 5),
        SessionError::UnexpectedHash {
            expected: 59,
            got: 5
        }
    );

    // The right hash tick, but before the peer sent its own input for it.
    let (a, mut fake) = channel_pair();
    for l in ["hello 1 42 60", "hash 0 0000000000000001"] {
        say(&mut fake, l);
    }
    let mut s = connect(a, cfg(0, 1, 2000)).unwrap();
    assert_eq!(
        until_error(&mut s, 5),
        SessionError::HashTooEarly { tick: 0 }
    );
    assert_eq!(s.tick(), 0);
}

#[test]
fn bye_or_a_closed_connection_ends_the_session() {
    let (a, mut fake) = channel_pair();
    for l in ["hello 1 42 60", "input 0 0 0 0", "input 1 0 0 0", "bye"] {
        say(&mut fake, l);
    }
    let mut s = connect(a, cfg(1, 60, 2000)).unwrap();
    assert_eq!(until_error(&mut s, 5), SessionError::PeerLeft { tick: 2 });

    // Channel peer dropped with nothing more sent: the next needed input is not there.
    let (a, mut fake) = channel_pair();
    say(&mut fake, "hello 1 42 60");
    let mut s = connect(a, cfg(1, 60, 2000)).unwrap();
    drop(fake);
    let start = Instant::now();
    assert_eq!(
        until_error(&mut s, 5),
        SessionError::Disconnected { tick: 0 }
    );
    assert!(
        start.elapsed() < Duration::from_secs(1),
        "no wait for the timeout"
    );

    // A peer that finished its last tick first and left: our sends to it fail, but what it
    // sent before leaving is still stepped, and only then does its bye end the session.
    let (a, mut fake) = channel_pair();
    for l in ["hello 1 42 60", "input 0 0 0 0", "input 1 5 0 0", "bye"] {
        say(&mut fake, l);
    }
    let mut s = connect(a, cfg(1, 60, 2000)).unwrap();
    drop(fake);
    assert_eq!(until_error(&mut s, 5), SessionError::PeerLeft { tick: 2 });
    assert_eq!(s.sim().applied.len(), 2);
    assert_eq!(s.sim().applied[1][1].throttle, 5);

    // TCP peer closed its sending side: the next receive sees end of stream, without waiting
    // for the timeout.
    use std::io::Write;
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap();
    let accept = thread::spawn(move || listener.accept().unwrap().0);
    let mut raw = TcpStream::connect(addr).unwrap();
    let server = accept.join().unwrap();
    raw.write_all(b"hello 1 42 60\ninput 0 0 0 0\n").unwrap();
    raw.shutdown(std::net::Shutdown::Write).unwrap();
    let t = TcpTransport::new(server, Duration::from_secs(2)).unwrap();
    let mut s = Session::connect(t, StandIn::new(SEED), cfg(1, 60, 2000), Role::Host).unwrap();
    let start = Instant::now();
    s.advance(Input::NEUTRAL).unwrap();
    assert_eq!(
        s.advance(Input::NEUTRAL),
        Err(SessionError::Disconnected { tick: 1 })
    );
    assert!(
        start.elapsed() < Duration::from_secs(1),
        "no wait for the timeout"
    );
    drop(raw);
}

#[test]
fn garbage_from_the_peer_is_a_codec_error() {
    let cases: [(&str, NetError); 4] = [
        ("input 1 abc 0 0", NetError::BadNumber { field: "throttle" }),
        ("input 1 0 0 0 ", NetError::BadSpacing),
        ("INPUT 1 0 0 0", NetError::UnknownWord("INPUT".into())),
        (
            "input 1 1001 0 0",
            NetError::OutOfRange { field: "throttle" },
        ),
    ];
    for (line, err) in cases {
        let (a, mut fake) = channel_pair();
        for l in ["hello 1 42 60", "input 0 0 0 0", line] {
            say(&mut fake, l);
        }
        let mut s = connect(a, cfg(1, 60, 2000)).unwrap();
        assert_eq!(until_error(&mut s, 5), SessionError::Codec(err), "{line:?}");
        assert_eq!(s.tick(), 1);
        assert_eq!(heard(&mut fake).last().map(String::as_str), Some("bye"));
    }
    // A second hello mid-session.
    let (a, mut fake) = channel_pair();
    for l in ["hello 1 42 60", "input 0 0 0 0", "hello 1 42 60"] {
        say(&mut fake, l);
    }
    let mut s = connect(a, cfg(1, 60, 2000)).unwrap();
    assert_eq!(until_error(&mut s, 5), SessionError::UnexpectedHello);
}

#[test]
fn tcp_garbage_bytes_are_codec_errors() {
    use std::io::Write;
    // A line with no end that is longer than the codec accepts, and bytes that are not UTF-8.
    let payloads: [(&[u8], NetError); 2] = [
        (&[b'x'; 300], NetError::TooLong),
        (b"hello \xff\xfe\n", NetError::BadSpacing),
    ];
    for (bytes, err) in payloads {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let accept = thread::spawn(move || listener.accept().unwrap().0);
        let mut raw = TcpStream::connect(addr).unwrap();
        let server = accept.join().unwrap();
        raw.write_all(bytes).unwrap();
        raw.shutdown(std::net::Shutdown::Write).unwrap();
        let t = TcpTransport::new(server, Duration::from_secs(2)).unwrap();
        let e = Session::connect(t, StandIn::new(SEED), cfg(1, 60, 2000), Role::Host)
            .err()
            .expect("refused");
        assert_eq!(e, SessionError::Codec(err));
    }
}

#[test]
fn bad_config_and_bad_local_input_are_refused() {
    let (a, mut fake) = channel_pair();
    let mut c = cfg(1, 0, 2000);
    assert_eq!(
        connect(a, c).err(),
        Some(SessionError::Config("hash_every must be at least 1"))
    );
    assert!(
        heard(&mut fake).is_empty(),
        "nothing sent on a config error"
    );
    c.hash_every = 60;
    c.timeout = Duration::ZERO;
    let (a, _fake) = channel_pair();
    assert!(matches!(connect(a, c).err(), Some(SessionError::Config(_))));

    let (a, mut fake) = channel_pair();
    say(&mut fake, "hello 1 42 60");
    let mut s = connect(a, cfg(1, 60, 2000)).unwrap();
    let bad = Input {
        throttle: 0,
        brake: 0,
        steer: 1001,
    };
    assert_eq!(s.advance(bad), Err(SessionError::LocalInputOutOfRange(bad)));
    // Hello and the one neutral input, then bye: the bad input never went out.
    assert_eq!(
        heard(&mut fake),
        vec!["hello 1 42 60", "input 0 0 0 0", "bye"]
    );
}

/// Like `input_at` but ticks 200..400 are a reverse segment: throttle -1..-1000, no brake.
fn reverse_input_at(peer_seed: u64, t: u64) -> Input {
    let mut i = input_at(peer_seed, t);
    if (200..400).contains(&t) {
        i.throttle = -1 - (mix(peer_seed ^ t) % 1000) as i32;
        i.brake = 0;
    }
    i
}

#[test]
fn two_peers_stay_in_sync_through_a_reverse_segment() {
    let c = cfg(3, 60, 5000);
    let (a, b) = channel_pair();
    let host =
        thread::spawn(move || run_peer_with(a, Role::Host, c, HOST_INPUTS, 1000, reverse_input_at));
    let guest = thread::spawn(move || {
        run_peer_with(b, Role::Guest, c, GUEST_INPUTS, 1000, reverse_input_at)
    });
    let (hs, hc) = host.join().unwrap().expect("host ok");
    let (gs, gc) = guest.join().unwrap().expect("guest ok");
    assert_eq!(hs.state_hash(), gs.state_hash());
    assert_eq!(hs.applied, gs.applied);
    // The reverse segment really crossed the wire: both sides stepped negative throttle from both peers.
    let reversed = hs
        .applied
        .iter()
        .filter(|p| p[0].throttle < 0 && p[1].throttle < 0)
        .count();
    assert_eq!(
        reversed, 200,
        "ticks 203..403 apply the delayed reverse inputs"
    );
    // Every hash check passed (no Desync) and they match a single machine stepping the same inputs.
    assert_eq!(hc.len(), 16);
    assert_eq!(hc, gc);
    let mut sim = StandIn::new(SEED);
    for t in 0..1000u64 {
        let pick = |seed| {
            if t < 3 {
                Input::NEUTRAL
            } else {
                reverse_input_at(seed, t - 3)
            }
        };
        sim.step(t, [pick(HOST_INPUTS), pick(GUEST_INPUTS)]);
    }
    assert_eq!(hs.state_hash(), sim.state_hash());
}

#[test]
fn local_throttle_past_full_reverse_is_refused_and_not_sent() {
    let (a, mut fake) = channel_pair();
    say(&mut fake, "hello 1 42 60");
    let mut s = connect(a, cfg(1, 60, 2000)).unwrap();
    let bad = Input {
        throttle: -1001,
        brake: 0,
        steer: 0,
    };
    assert_eq!(s.advance(bad), Err(SessionError::LocalInputOutOfRange(bad)));
    assert_eq!(
        heard(&mut fake),
        vec!["hello 1 42 60", "input 0 0 0 0", "bye"]
    );
}
