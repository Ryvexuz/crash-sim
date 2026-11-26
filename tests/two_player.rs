use crash_sim::car_contact::ContactSettings;
use crash_sim::multi_car::{CarControls, CarPlacement, MultiSim};
use crash_sim::race;
use crash_sim::race_hash::{multi_state_hash, state_hash, RaceHashError};
use crash_sim::settle::{self, CarSetup};
use crash_sim::statehash::Hasher;
use crash_sim::track::Track;
use crash_sim::vec3::Vec3;
use crash_sim::{Body, Config, Sim};
use std::collections::HashSet;

fn body() -> Body {
    crash_sim::format::parse(include_str!("../scenarios/car_basic.bsn")).unwrap()
}

fn track() -> Track {
    Track::parse(include_str!("../scenarios/tracks/oval.track")).unwrap()
}

fn placed_cars() -> (Sim, Sim) {
    let track = track();
    let make = |slot| {
        let placed =
            race::place_on_grid(&body(), &track, race::slot_distance(&track, slot, 10.0)).unwrap();
        settle::car_sim(Config::default(), &placed, &CarSetup::default()).unwrap()
    };
    (make(1), make(2))
}

fn two_cars() -> (Sim, Sim) {
    let (mut a, mut b) = placed_cars();
    a.car.as_mut().unwrap().set_controls(0.5, 0.0, 0.3).unwrap();
    b.car
        .as_mut()
        .unwrap()
        .set_controls(0.4, 0.2, -0.2)
        .unwrap();
    for _ in 0..20 {
        a.advance();
        b.advance();
    }
    (a, b)
}

fn flip(x: &mut f64) {
    *x = f64::from_bits(x.to_bits() ^ 1);
}

#[test]
fn state_hash_covers_every_state_field() {
    let (a, b) = two_cars();
    let base = state_hash(599, &[&a, &b]).unwrap();
    let (a, b) = two_cars();
    assert_eq!(state_hash(599, &[&a, &b]).unwrap(), base);
    let mut seen = HashSet::new();
    seen.insert(base);
    for car_index in 0..2 {
        for case in 0..22 {
            let (mut a, mut b) = two_cars();
            let sim = if car_index == 0 { &mut a } else { &mut b };
            let last_node = sim.pos.len() - 1;
            let last_beam = sim.beams.len() - 1;
            let car = sim.car.as_mut().unwrap();
            let (field, name) = match case {
                0 => (&mut sim.pos[0].x, "node 0 pos.x"),
                1 => (&mut sim.pos[last_node].z, "last node pos.z"),
                2 => (&mut sim.vel[0].y, "node 0 vel.y"),
                3 => (&mut sim.vel[last_node].x, "last node vel.x"),
                4 => (&mut car.wheels[0].pos.y, "wheel 0 pos.y"),
                5 => (&mut car.wheels[3].vel.z, "wheel 3 vel.z"),
                6 => (&mut car.wheels[0].spin, "wheel 0 spin"),
                7 => (&mut car.wheels[3].spin, "wheel 3 spin"),
                8 => (&mut car.wheels[0].deflection[0].x, "wheel 0 deflection"),
                9 => (&mut car.wheels[0].load[0], "wheel 0 load"),
                10 => (&mut car.wheels[2].length, "wheel 2 length"),
                11 => (&mut car.throttle, "throttle"),
                12 => (&mut car.brake, "brake"),
                13 => (&mut car.steer, "steer"),
                14 => (&mut car.frame.up.y, "frame up"),
                15 => (&mut car.frame.forward.x, "frame forward"),
                16 => (&mut car.frame.axle.z, "frame axle"),
                17 => (&mut car.carrier_torque.x, "carrier torque"),
                18 => (&mut car.drive_work, "drive work"),
                19 => (&mut sim.beams[0].rest, "beam 0 rest"),
                20 => (&mut sim.beams[last_beam].rest, "last beam rest"),
                21 => {
                    sim.broken[0] = !sim.broken[0];
                    // No float changes in this case.
                    (&mut sim.beams[0].rest, "broken")
                }
                _ => unreachable!(),
            };
            if case != 21 {
                flip(field);
            }
            let hash = state_hash(599, &[&a, &b]).unwrap();
            assert_ne!(hash, base, "car {} {name}", car_index + 1);
            assert!(seen.insert(hash), "duplicate: car {} {name}", car_index + 1);
        }
    }
    let (mut a, b) = two_cars();
    for (name, hash) in [
        ("tick", state_hash(600, &[&a, &b]).unwrap()),
        ("swapped", state_hash(599, &[&b, &a]).unwrap()),
    ] {
        assert_ne!(hash, base, "{name}");
        assert!(seen.insert(hash), "{name}");
    }
    a.beams.last_mut().unwrap().rest *= 0.99;
    let hash = state_hash(599, &[&a, &b]).unwrap();
    assert_ne!(hash, base, "yield");
    assert!(seen.insert(hash), "yield");
}

#[test]
fn state_hash_order_is_documented() {
    let (a, b) = two_cars();
    let mut h = Hasher::new();
    h.write_u64(599);
    h.write_u64(2);
    for sim in [&a, &b] {
        let car = sim.car.as_ref().unwrap();
        h.write_u64(sim.pos.len() as u64);
        h.write_u64(car.wheels.len() as u64);
        h.write_u64(sim.beams.len() as u64);
        for i in 0..sim.pos.len() {
            let p = sim.pos[i];
            let v = sim.vel[i];
            for x in [p.x, p.y, p.z, v.x, v.y, v.z] {
                h.write_f64(x).unwrap();
            }
        }
        for w in &car.wheels {
            for x in [w.pos.x, w.pos.y, w.pos.z, w.vel.x, w.vel.y, w.vel.z, w.spin] {
                h.write_f64(x).unwrap();
            }
            for d in &w.deflection {
                h.write_f64(d.x).unwrap();
                h.write_f64(d.y).unwrap();
                h.write_f64(d.z).unwrap();
            }
            for &load in &w.load {
                h.write_f64(load).unwrap();
            }
            h.write_f64(w.length).unwrap();
        }
        h.write_f64(car.throttle).unwrap();
        h.write_f64(car.brake).unwrap();
        h.write_f64(car.steer).unwrap();
        for v in [
            car.frame.up,
            car.frame.forward,
            car.frame.axle,
            car.carrier_torque,
        ] {
            h.write_f64(v.x).unwrap();
            h.write_f64(v.y).unwrap();
            h.write_f64(v.z).unwrap();
        }
        h.write_f64(car.drive_work).unwrap();
        h.write_f64(sim.gravity_tilt()).unwrap();
        h.write_u64(u64::from(car.frame_error.is_some()));
        for i in 0..sim.beams.len() {
            h.write_f64(sim.beams[i].rest).unwrap();
            h.write_u64(u64::from(sim.broken[i]));
        }
    }
    assert_eq!(h.finish(), state_hash(599, &[&a, &b]).unwrap());
    let (a, b) = placed_cars();
    let settings = ContactSettings::new(0.125, 0.0, 0.001, 0.015625, 0.0625).unwrap();
    let mut multi = MultiSim::new(
        vec![a, b],
        &[CarPlacement { offset: Vec3::ZERO }; 2],
        settings,
        0.001,
    )
    .unwrap();
    multi
        .advance(
            &[CarControls {
                throttle: 0.0,
                brake: 0.0,
                steer: 0.0,
            }; 2],
        )
        .unwrap();
    let tick = multi.step_index();
    assert_eq!(
        multi_state_hash(tick, &multi).unwrap(),
        state_hash(tick, &[multi.sim(0), multi.sim(1)]).unwrap()
    );
}

#[test]
fn state_hash_refuses_nan_and_bad_shapes() {
    let (a, mut b) = two_cars();
    b.vel[0].x = f64::NAN;
    assert!(matches!(
        state_hash(0, &[&a, &b]),
        Err(RaceHashError::Failed { car: 1, .. })
    ));
    let (mut a, b) = two_cars();
    a.car.as_mut().unwrap().wheels[0].load[0] = f64::NAN;
    assert_eq!(state_hash(0, &[&a, &b]), Err(RaceHashError::Nan { car: 0 }));
    assert_eq!(state_hash(0, &[]), Err(RaceHashError::NoCars));
    let (mut a, b) = two_cars();
    a.broken.pop();
    assert_eq!(
        state_hash(0, &[&a, &b]),
        Err(RaceHashError::Shape { car: 0 })
    );
    let (mut a, _) = two_cars();
    let cube = Sim::new(Config::default());
    assert_eq!(
        state_hash(0, &[&a, &cube]),
        Err(RaceHashError::NoCar { car: 1 })
    );
    a.vel[0].x = 0.0;
    let positive = state_hash(0, &[&a]).unwrap();
    a.vel[0].x = -0.0;
    assert_ne!(positive, state_hash(0, &[&a]).unwrap());
}

#[test]
fn state_hash_same_input_same_hash() {
    let (mut a, mut b) = two_cars();
    let (mut c, mut d) = two_cars();
    let before = state_hash(4999, &[&a, &b]).unwrap();
    for _ in 0..50 {
        a.advance();
        b.advance();
        c.advance();
        d.advance();
    }
    let after = state_hash(4999, &[&a, &b]).unwrap();
    assert_eq!(after, state_hash(4999, &[&c, &d]).unwrap());
    assert_ne!(before, after);
}

#[test]
fn state_hash_refuses_failed_car() {
    let (mut a, b) = two_cars();
    a.pos[0].x = f64::INFINITY;
    assert!(matches!(
        state_hash(0, &[&a, &b]),
        Err(RaceHashError::Failed { car: 0, reason }) if !reason.is_empty()
    ));
}

mod run_tmp;

use std::io::{BufRead, BufReader, Read, Write};
use std::process::{Child, Command, Stdio};
use std::sync::{mpsc, Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

const PEER_USAGE: &str = "usage: crash-sim --body <car.bsn> --track <file> --grid-gap <m> --duration <s> (--host <addr:port> | --join <addr:port>) --input <file> --net-delay <ticks> --net-hash-every <ticks> --net-timeout <s> --net-connect-timeout <s> --net-seed <u64>";

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
    assert!(
        bin.is_file(),
        "CARGO_BIN_EXE_crash-sim is not set and {} does not exist",
        bin.display()
    );
    bin
}

fn tmp_file(name: &str, text: &str) -> String {
    let path = run_tmp::run_tmp().join(format!("two_player_{}_{name}.input", std::process::id()));
    std::fs::write(&path, text).unwrap();
    path.to_string_lossy().into_owned()
}

struct PeerChild {
    child: Arc<Mutex<Child>>,
    started: Instant,
    first: mpsc::Receiver<String>,
    lines: mpsc::Receiver<String>,
    stdout: Option<JoinHandle<String>>,
    stderr: Option<JoinHandle<String>>,
    done: mpsc::Sender<()>,
    watchdog: Option<JoinHandle<bool>>,
}

struct PeerOutput {
    code: i32,
    stdout: String,
    stderr: String,
    elapsed: Duration,
}

impl PeerChild {
    fn spawn(args: &[String]) -> Self {
        let started = Instant::now();
        let mut child = Command::new(exe())
            .args(args)
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .expect("spawn crash-sim");
        let stdout = child.stdout.take().unwrap();
        let mut stderr = child.stderr.take().unwrap();
        let (first_tx, first) = mpsc::channel();
        let (lines_tx, lines) = mpsc::channel();
        let stdout = thread::spawn(move || {
            let mut reader = BufReader::new(stdout);
            let mut text = String::new();
            let mut line = String::new();
            while reader.read_line(&mut line).expect("read stdout") != 0 {
                if text.is_empty() {
                    let _ = first_tx.send(line.trim_end_matches(['\r', '\n']).to_string());
                }
                let _ = lines_tx.send(line.trim_end_matches(['\r', '\n']).to_string());
                text.push_str(&line);
                line.clear();
            }
            text
        });
        let stderr = thread::spawn(move || {
            let mut text = String::new();
            stderr.read_to_string(&mut text).expect("read stderr");
            text
        });
        let child = Arc::new(Mutex::new(child));
        let watched = Arc::clone(&child);
        let (done, receiver) = mpsc::channel();
        let watchdog = thread::spawn(move || {
            if receiver
                .recv_timeout(Duration::from_secs(60).saturating_sub(started.elapsed()))
                .is_ok()
            {
                return false;
            }
            let mut child = watched.lock().unwrap();
            if child.try_wait().unwrap().is_none() {
                child.kill().expect("kill timed-out crash-sim");
                child.wait().expect("reap timed-out crash-sim");
                return true;
            }
            false
        });
        Self {
            child,
            started,
            first,
            lines,
            stdout: Some(stdout),
            stderr: Some(stderr),
            done,
            watchdog: Some(watchdog),
        }
    }

    fn finish(mut self) -> PeerOutput {
        let status = loop {
            if let Some(status) = self.child.lock().unwrap().try_wait().unwrap() {
                break status;
            }
            thread::sleep(Duration::from_millis(10));
        };
        let _ = self.done.send(());
        let killed = self.watchdog.take().unwrap().join().unwrap();
        let stdout = self.stdout.take().unwrap().join().unwrap();
        let stderr = self.stderr.take().unwrap().join().unwrap();
        assert!(!killed, "crash-sim exceeded 60 s\n{stdout}\n{stderr}");
        PeerOutput {
            code: status.code().expect("exit code"),
            stdout,
            stderr,
            elapsed: self.started.elapsed(),
        }
    }
}

impl Drop for PeerChild {
    fn drop(&mut self) {
        let mut child = self.child.lock().unwrap();
        if child.try_wait().unwrap().is_none() {
            child.kill().expect("kill unfinished crash-sim");
            child.wait().expect("reap unfinished crash-sim");
        }
        drop(child);
        let _ = self.done.send(());
        if let Some(watchdog) = self.watchdog.take() {
            watchdog.join().unwrap();
        }
    }
}

fn manifest(path: &str) -> String {
    format!("{}/{path}", env!("CARGO_MANIFEST_DIR"))
}

fn common_args(input: &str) -> Vec<String> {
    vec![
        "--body".into(),
        manifest("scenarios/car_basic.bsn"),
        "--track".into(),
        manifest("scenarios/tracks/oval.track"),
        "--grid-gap".into(),
        "10".into(),
        "--duration".into(),
        "20".into(),
        "--net-delay".into(),
        "6".into(),
        "--net-hash-every".into(),
        "100".into(),
        "--input".into(),
        input.into(),
    ]
}

fn peer_args(flag: &str, addr: &str, input: &str) -> Vec<String> {
    let mut args = common_args(input);
    args.extend(
        [
            "--net-timeout",
            "5",
            "--net-connect-timeout",
            "10",
            "--net-seed",
            "1",
            flag,
            addr,
        ]
        .map(String::from),
    );
    args
}

fn replace(args: &mut [String], flag: &str, value: &str) {
    let i = args.iter().position(|a| a == flag).expect("known flag");
    args[i + 1] = value.into();
}

fn pair(guest_input: &str, guest_gap: &str) -> (String, PeerChild, PeerChild) {
    let host = PeerChild::spawn(&peer_args(
        "--host",
        "127.0.0.1:0",
        &manifest("scenarios/tracks/oval_lap.input"),
    ));
    let first = host
        .first
        .recv_timeout(Duration::from_secs(60).saturating_sub(host.started.elapsed()))
        .expect("host listening line before deadline");
    let addr = first
        .strip_prefix("net listening addr=")
        .expect("listening line")
        .to_string();
    let socket: std::net::SocketAddr = addr.parse().expect("numeric listening address");
    assert_eq!(socket.ip(), std::net::Ipv4Addr::LOCALHOST);
    assert_ne!(socket.port(), 0);
    let mut args = peer_args("--join", &addr, guest_input);
    replace(&mut args, "--grid-gap", guest_gap);
    (addr, host, PeerChild::spawn(&args))
}

fn race_lines(text: &str) -> Vec<&str> {
    text.lines()
        .filter(|line| {
            line.starts_with("car ") || line.starts_with("cars ") || line.starts_with("hash ")
        })
        .collect()
}

fn last(text: &str) -> &str {
    text.lines().last().expect("nonempty output")
}

fn check_exit(run: &PeerOutput, code: i32) {
    assert_eq!(run.code, code, "{}\n{}", run.stdout, run.stderr);
    assert!(run.stderr.is_empty(), "{}", run.stderr);
}

fn check_hex(hash: &str) {
    assert_eq!(hash.len(), 16, "{hash}");
    assert!(hash
        .bytes()
        .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b)));
}

#[test]
fn two_peers_on_loopback_match_the_local_run() {
    let still = tmp_file("still", "0 0 1 0\n");
    let (addr, host, guest) = pair(&still, "10");
    let (host, guest) = (host.finish(), guest.finish());
    let mut args = common_args(&manifest("scenarios/tracks/oval_lap.input"));
    args.extend(["--input2".into(), still]);
    let reference = PeerChild::spawn(&args).finish();
    check_exit(&reference, 0);
    assert!(!reference.stdout.is_empty());
    assert_eq!(
        host.stdout.lines().next().unwrap(),
        format!("net listening addr={addr}")
    );
    let mut last_hash = None;
    for (role, run, role_index) in [("host", &host, 1), ("guest", &guest, 0)] {
        check_exit(run, 0);
        assert_eq!(
            run.stdout.lines().nth(role_index).unwrap(),
            format!("net role={role} addr={addr} tick_rate=100 input_delay=6 hash_every=100 timeout_s=5.000 seed=1")
        );
        assert_eq!(race_lines(&run.stdout).join("\n") + "\n", reference.stdout);
        assert_eq!(
            run.stdout.lines().filter(|l| l.starts_with("hash ")).count(),
            20
        );
        assert!(run.stdout.lines().any(|l| l.starts_with("car 1 start ")));
        let hash = run
            .stdout
            .lines()
            .find_map(|l| l.strip_prefix("hash tick=1999 value="))
            .unwrap();
        check_hex(hash);
        assert_eq!(
            last(&run.stdout),
            format!("net: ok ticks=2000 hashes=20 last_hash={hash}")
        );
        if let Some(previous) = last_hash {
            assert_eq!(hash, previous);
        }
        last_hash = Some(hash);
        eprintln!("{role} last: {}", last(&run.stdout));
    }
}

#[test]
fn forced_desync_is_reported_by_both_peers() {
    let (_, host, guest) = pair(&manifest("scenarios/tracks/oval_lap.input"), "11");
    let (host, guest) = (host.finish(), guest.finish());
    let hashes = [&host, &guest].map(|run| {
        check_exit(run, 1);
        assert!(race_lines(&run.stdout).is_empty(), "{}", run.stdout);
        let line = last(&run.stdout);
        let (own, peer) = line
            .strip_prefix("net: FAIL desync at tick 99 own=")
            .expect("desync at first hash tick")
            .split_once(" peer=")
            .expect("both hashes");
        for hash in [own, peer] {
            check_hex(hash);
        }
        assert_ne!(own, peer);
        eprintln!("desync last: {line}");
        (own, peer)
    });
    assert_eq!(hashes[0], (hashes[1].1, hashes[1].0));
}

#[test]
fn no_peer_ends_with_a_named_error() {
    let input = manifest("scenarios/tracks/oval_lap.input");
    let mut args = peer_args("--host", "127.0.0.1:0", &input);
    replace(&mut args, "--net-connect-timeout", "1");
    let host = PeerChild::spawn(&args).finish();
    check_exit(&host, 1);
    assert!(host.elapsed < Duration::from_secs(10), "{:?}", host.elapsed);
    assert!(host
        .stdout
        .lines()
        .next()
        .unwrap()
        .starts_with("net listening addr=127.0.0.1:"));
    assert_eq!(last(&host.stdout), "net: FAIL no peer within 1.000 s");
    let guest = PeerChild::spawn(&peer_args("--join", "127.0.0.1:1", &input)).finish();
    check_exit(&guest, 1);
    assert!(guest.elapsed < Duration::from_secs(15), "{:?}", guest.elapsed);
    assert!(last(&guest.stdout).starts_with("net: FAIL cannot connect to 127.0.0.1:1: "));
}

#[test]
fn peer_usage_errors() {
    let base = peer_args(
        "--host",
        "127.0.0.1:0",
        &manifest("scenarios/tracks/oval_lap.input"),
    );
    let mut cases = Vec::new();
    let mut missing_value = base.clone();
    missing_value.pop();
    cases.push(missing_value);
    for flag in ["--net-timeout", "--net-connect-timeout", "--net-seed"] {
        let mut args = base.clone();
        let i = args.iter().position(|a| a == flag).unwrap();
        args.drain(i..i + 2);
        cases.push(args);
    }
    let bad_script = tmp_file("bad", "0 0.1234 0 0\n");
    let huge_seed = "9".repeat(2048);
    for (flag, value) in [
        ("--net-timeout", "0"),
        ("--net-timeout", "-1"),
        ("--net-connect-timeout", "nan"),
        ("--net-timeout", "inf"),
        ("--net-connect-timeout", "3600.1"),
        ("--net-seed", "01"),
        ("--net-seed", "18446744073709551616"),
        ("--net-seed", huge_seed.as_str()),
        ("--net-seed", "+1"),
        ("--net-seed", "-1"),
        ("--net-seed", ""),
        ("--net-seed", "1\r2"),
        ("--host", "localhost:47000"),
        ("--duration", "20.005"),
        ("--input", bad_script.as_str()),
        ("--grid-gap", "nan"),
        ("--net-delay", "1001"),
        ("--net-hash-every", "0"),
    ] {
        let mut args = base.clone();
        replace(&mut args, flag, value);
        cases.push(args);
    }
    for extra in [
        vec!["--join", "127.0.0.1:1"],
        vec!["--ai", "3"],
        vec!["--live"],
        vec!["--dt", "0.01"],
        vec!["--substeps", "10"],
        vec!["--net-seed", "1"],
    ] {
        let mut args = base.clone();
        args.extend(extra.into_iter().map(String::from));
        cases.push(args);
    }
    for args in cases {
        let run = PeerChild::spawn(&args).finish();
        assert_eq!(run.code, 2, "{args:?}\n{}\n{}", run.stdout, run.stderr);
        assert!(run.stdout.is_empty(), "{args:?}\n{}", run.stdout);
        assert!(run.stderr.starts_with("crash-sim: "), "{}", run.stderr);
        assert!(run.stderr.lines().any(|l| l == PEER_USAGE), "{}", run.stderr);
        assert!(
            run.elapsed < Duration::from_secs(5),
            "{args:?}: {:?}",
            run.elapsed
        );
    }
}

struct FakePlan {
    corrupt_hash_at: Option<u64>,
    silent_after_hash: Option<u64>,
}

fn fake_guest(addr: &str, plan: FakePlan) -> JoinHandle<()> {
    let addr = loopback_addr(addr);
    thread::spawn(move || {
        use crash_sim::net::{decode, encode, Msg};
        let mut stream = std::net::TcpStream::connect(addr).expect("fake guest connect");
        stream
            .set_read_timeout(Some(Duration::from_secs(60)))
            .unwrap();
        stream
            .set_write_timeout(Some(Duration::from_secs(60)))
            .unwrap();
        let mut reader = BufReader::new(stream.try_clone().unwrap());
        let mut line = String::new();
        assert_ne!(reader.read_line(&mut line).unwrap(), 0, "host hello");
        let hello = Msg::Hello {
            seed: 1,
            tick_rate: 100,
        };
        assert_eq!(decode(line.trim_end_matches(['\r', '\n'])), Ok(hello));
        writeln!(stream, "{}", encode(&hello)).unwrap();
        stream.flush().unwrap();
        let mut silent = false;
        loop {
            line.clear();
            if reader.read_line(&mut line).expect("fake guest read") == 0 {
                break;
            }
            let msg = decode(line.trim_end_matches(['\r', '\n'])).expect("host message");
            if msg == Msg::Bye {
                break;
            }
            if silent {
                continue;
            }
            let reply = match msg {
                Msg::Input { tick, .. } => Msg::Input {
                    tick,
                    throttle: 0,
                    brake: if tick < 6 { 0 } else { 1000 },
                    steer: 0,
                },
                Msg::Hash { tick, hash } => {
                    silent = plan.silent_after_hash == Some(tick);
                    Msg::Hash {
                        tick,
                        hash: if plan.corrupt_hash_at == Some(tick) {
                            hash ^ 1
                        } else {
                            hash
                        },
                    }
                }
                other => panic!("unexpected host message: {other:?}"),
            };
            writeln!(stream, "{}", encode(&reply)).expect("fake guest reply");
            stream.flush().unwrap();
        }
    })
}

fn loopback_addr(addr: &str) -> std::net::SocketAddr {
    let socket: std::net::SocketAddr = addr.parse().expect("numeric listening address");
    assert_eq!(socket.ip(), std::net::Ipv4Addr::LOCALHOST);
    assert_ne!(socket.port(), 0);
    socket
}

fn listening_addr(host: &PeerChild) -> String {
    let first = host
        .first
        .recv_timeout(Duration::from_secs(60).saturating_sub(host.started.elapsed()))
        .expect("host listening line before deadline");
    let addr = first
        .strip_prefix("net listening addr=")
        .expect("listening line");
    loopback_addr(addr);
    addr.to_string()
}

fn reference_run(input: &str, still: &str) -> PeerOutput {
    let mut args = common_args(input);
    args.extend(["--input2".into(), still.into()]);
    let reference = PeerChild::spawn(&args).finish();
    check_exit(&reference, 0);
    assert!(!reference.stdout.is_empty());
    reference
}

#[test]
fn silent_peer_ends_with_timeout() {
    let input = manifest("scenarios/tracks/oval_lap.input");
    let still = tmp_file("silent_still", "0 0 1 0\n");
    let reference = reference_run(&input, &still);
    let mut args = peer_args("--host", "127.0.0.1:0", &input);
    replace(&mut args, "--net-timeout", "1");
    let host = PeerChild::spawn(&args);
    let guest = fake_guest(
        &listening_addr(&host),
        FakePlan {
            corrupt_hash_at: None,
            silent_after_hash: Some(99),
        },
    );
    let host = host.finish();
    guest.join().expect("fake guest completed");
    check_exit(&host, 1);
    assert!(host.elapsed < Duration::from_secs(10), "{:?}", host.elapsed);
    let hashes: Vec<_> = host
        .stdout
        .lines()
        .filter(|l| l.starts_with("hash "))
        .collect();
    let first_hash = reference
        .stdout
        .lines()
        .find(|l| l.starts_with("hash tick=99 value="))
        .expect("reference first hash");
    assert_eq!(hashes, vec![first_hash], "{}", host.stdout);
    assert!(!host.stdout.lines().any(|l| l.starts_with("net: ok")));
    assert_eq!(
        last(&host.stdout),
        "net: FAIL timeout at tick 106 after 1.000 s",
        "{}",
        host.stdout
    );
}

#[test]
fn killed_peer_ends_with_a_named_error() {
    let input = manifest("scenarios/tracks/oval_lap.input");
    let still = tmp_file("killed_still", "0 0 1 0\n");
    let mut args = peer_args("--host", "127.0.0.1:0", &input);
    replace(&mut args, "--duration", "3000");
    let host = PeerChild::spawn(&args);
    let addr = listening_addr(&host);
    let mut args = peer_args("--join", &addr, &still);
    replace(&mut args, "--duration", "3000");
    let mut guest = PeerChild::spawn(&args);
    loop {
        let line = host
            .lines
            .recv_timeout(Duration::from_secs(60).saturating_sub(host.started.elapsed()))
            .expect("host first hash before deadline");
        if line.starts_with("hash ") {
            break;
        }
    }
    guest.child.lock().unwrap().kill().expect("kill guest");
    let killed_at = Instant::now();
    let host = host.finish();
    let after_kill = killed_at.elapsed();
    // A process killed by a signal need not have a numeric exit code.
    guest.child.lock().unwrap().wait().expect("reap killed guest");
    guest.stdout.take().unwrap().join().expect("guest stdout");
    guest.stderr.take().unwrap().join().expect("guest stderr");
    drop(guest);
    check_exit(&host, 1);
    assert!(after_kill < Duration::from_secs(15), "{after_kill:?}");
    assert!(host.stdout.lines().any(|l| l.starts_with("hash ")));
    assert!(!host.stdout.lines().any(|l| l.starts_with("net: ok")));
    let line = last(&host.stdout);
    eprintln!("killed peer host last: {line}");
    let tick = line
        .strip_prefix("net: FAIL disconnected at tick ")
        .or_else(|| {
            line.strip_prefix("net: FAIL timeout at tick ")?
                .strip_suffix(" after 5.000 s")
        })
        .expect("named disconnected or timeout failure");
    let parsed: u64 = tick.parse().expect("failure tick");
    assert_eq!(tick, parsed.to_string(), "{line}");
}

#[test]
fn host_and_guest_inputs_drive_their_own_cars() {
    let input = manifest("scenarios/tracks/oval_lap.input");
    let still = tmp_file("routing_still", "0 0 1 0\n");
    let mut args = peer_args("--host", "127.0.0.1:0", &input);
    replace(&mut args, "--duration", "2");
    let host = PeerChild::spawn(&args);
    let mut args = peer_args("--join", &listening_addr(&host), &still);
    replace(&mut args, "--duration", "2");
    let guest = PeerChild::spawn(&args);
    let a = [host.finish(), guest.finish()];
    let mut args = peer_args("--host", "127.0.0.1:0", &still);
    replace(&mut args, "--duration", "2");
    let host = PeerChild::spawn(&args);
    let mut args = peer_args("--join", &listening_addr(&host), &input);
    replace(&mut args, "--duration", "2");
    let guest = PeerChild::spawn(&args);
    let b = [host.finish(), guest.finish()];
    for (label, runs) in [("A", &a), ("B", &b)] {
        for run in runs {
            check_exit(run, 0);
            assert!(
                last(&run.stdout).starts_with("net: ok "),
                "run {label}\n{}",
                run.stdout
            );
        }
    }
    for (peer, (a, b)) in ["host", "guest"].into_iter().zip(a.iter().zip(&b)) {
        let positions = [1, 2].map(|car| {
            let prefix = format!("car {car} final ");
            let final_a = a.stdout.lines().find(|l| l.starts_with(&prefix)).unwrap();
            let final_b = b.stdout.lines().find(|l| l.starts_with(&prefix)).unwrap();
            assert_ne!(final_a, final_b);
            [final_a, final_b].map(|line| {
                let fields: Vec<_> = line
                    .split_whitespace()
                    .filter_map(|field| field.strip_prefix("x="))
                    .collect();
                assert_eq!(fields.len(), 1, "one final x field: {line}");
                let x: f64 = fields[0].parse().expect("numeric final x");
                assert!(x.is_finite(), "finite final x: {line}");
                x
            })
        });
        let [[a1, b1], [a2, b2]] = positions;
        eprintln!("{peer} final x: A car 1={a1}, A car 2={a2}, B car 1={b1}, B car 2={b2}");
        assert!(a1 - b1 > 3.0, "{peer}: car 1 must drive only in A");
        assert!(b2 - a2 > 3.0, "{peer}: car 2 must drive only in B");
    }
}

#[test]
fn no_race_lines_of_an_unconfirmed_interval_are_printed() {
    let input = manifest("scenarios/tracks/oval_lap.input");
    let still = tmp_file("unconfirmed_still", "0 0 1 0\n");
    let reference = reference_run(&input, &still);
    let lines: Vec<_> = reference.stdout.lines().collect();
    let first_car = lines
        .iter()
        .position(|l| l.starts_with("car "))
        .expect("reference race event");
    let i = lines[..first_car]
        .iter()
        .filter(|l| l.starts_with("hash "))
        .count();
    assert!((1..=18).contains(&i), "first event interval i={i}");
    let t = 100 * i + 99;
    eprintln!("unconfirmed interval i={i} t={t}");
    let prefix = format!("hash tick={t} value=");
    let hash = lines
        .iter()
        .find_map(|l| l.strip_prefix(&prefix))
        .expect("reference hash of event interval");
    check_hex(hash);
    let own = u64::from_str_radix(hash, 16).unwrap();
    let host = PeerChild::spawn(&peer_args("--host", "127.0.0.1:0", &input));
    let addr = listening_addr(&host);
    let guest = fake_guest(
        &addr,
        FakePlan {
            corrupt_hash_at: Some(u64::try_from(t).unwrap()),
            silent_after_hash: None,
        },
    );
    let host = host.finish();
    guest.join().expect("fake guest completed");
    check_exit(&host, 1);
    assert!(host.elapsed < Duration::from_secs(30), "{:?}", host.elapsed);
    // The first event follows exactly i confirmed hashes; none of its interval is released.
    let mut expected = format!(
        "net listening addr={addr}\nnet role=host addr={addr} tick_rate=100 input_delay=6 hash_every=100 timeout_s=5.000 seed=1\n"
    );
    for line in &lines[..first_car] {
        expected.push_str(line);
        expected.push('\n');
    }
    expected.push_str(&format!(
        "net: FAIL desync at tick {t} own={own:016x} peer={:016x}\n",
        own ^ 1
    ));
    assert_eq!(host.stdout, expected);
}
