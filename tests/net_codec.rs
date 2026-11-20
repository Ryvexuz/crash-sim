//! Net message codec tests (n20-25a). The module is included by path; lib.rs is not touched.
#![allow(dead_code)]

#[path = "../src/net.rs"]
mod net;

use net::{decode, encode, Msg, NetError, INPUT_SCALE, MAX_LINE_BYTES, MAX_TICK_RATE};

/// Small seeded generator (splitmix64): no RNG crate, same values on every run.
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    fn range(&mut self, lo: i32, hi: i32) -> i32 {
        lo + (self.next() % ((hi - lo) as u64 + 1)) as i32
    }
}

fn roundtrip(m: Msg) {
    let s = encode(&m);
    assert_eq!(decode(&s), Ok(m), "round trip of {s:?}");
    assert_eq!(encode(&decode(&s).unwrap()), s, "re-encode of {s:?}");
}

#[test]
fn exact_wire_text() {
    assert_eq!(
        encode(&Msg::Hello {
            seed: 42,
            tick_rate: 60
        }),
        "hello 1 42 60"
    );
    assert_eq!(
        encode(&Msg::Input {
            tick: 7,
            throttle: 1000,
            brake: 0,
            steer: -250
        }),
        "input 7 1000 0 -250"
    );
    assert_eq!(
        encode(&Msg::Hash {
            tick: 9,
            hash: 0xAB
        }),
        "hash 9 00000000000000ab"
    );
    assert_eq!(encode(&Msg::Bye), "bye");
}

#[test]
fn round_trip_seeded_values_every_kind() {
    let mut r = Rng(0x5EED);
    for _ in 0..2000 {
        roundtrip(Msg::Hello {
            seed: r.next(),
            tick_rate: r.range(1, MAX_TICK_RATE as i32) as u32,
        });
        roundtrip(Msg::Input {
            tick: r.next(),
            throttle: r.range(0, INPUT_SCALE),
            brake: r.range(0, INPUT_SCALE),
            steer: r.range(-INPUT_SCALE, INPUT_SCALE),
        });
        roundtrip(Msg::Hash {
            tick: r.next(),
            hash: r.next(),
        });
    }
    roundtrip(Msg::Bye);
}

#[test]
fn round_trip_edges() {
    roundtrip(Msg::Hello {
        seed: 0,
        tick_rate: 1,
    });
    roundtrip(Msg::Hello {
        seed: u64::MAX,
        tick_rate: MAX_TICK_RATE,
    });
    for (t, b, s) in [
        (0, 0, -1000),
        (1000, 1000, 1000),
        (0, 0, 0),
        (1, 1, -1),
        (1000, 0, 1000),
    ] {
        roundtrip(Msg::Input {
            tick: 0,
            throttle: t,
            brake: b,
            steer: s,
        });
        roundtrip(Msg::Input {
            tick: u64::MAX,
            throttle: t,
            brake: b,
            steer: s,
        });
    }
    roundtrip(Msg::Hash { tick: 0, hash: 0 });
    roundtrip(Msg::Hash {
        tick: u64::MAX,
        hash: u64::MAX,
    });
}

fn refused(s: &str) -> NetError {
    match decode(s) {
        Err(e) => e,
        Ok(m) => panic!("{s:?} should be refused but decoded to {m:?}"),
    }
}

#[test]
fn refuses_empty_unknown_and_wrong_case() {
    assert_eq!(refused(""), NetError::Empty);
    assert!(matches!(refused("nope"), NetError::UnknownWord(_)));
    assert!(matches!(refused("Hello 1 1 60"), NetError::UnknownWord(_)));
    assert!(matches!(refused("BYE"), NetError::UnknownWord(_)));
    assert!(matches!(refused("hello1 1 60"), NetError::UnknownWord(_)));
}

#[test]
fn refuses_wrong_field_counts() {
    for s in [
        "hello",
        "hello 1 42",
        "hello 1 42 60 9",
        "input 1 2 3",
        "input 1 2 3 4 5",
        "input",
        "hash 1",
        "hash 1 0000000000000000 x",
        "bye now",
        "bye 1",
    ] {
        assert!(
            matches!(refused(s), NetError::WrongFieldCount { .. }),
            "{s:?}"
        );
    }
}

#[test]
fn refuses_wrong_version() {
    assert_eq!(refused("hello 2 42 60"), NetError::WrongVersion(2));
    assert_eq!(refused("hello 0 42 60"), NetError::WrongVersion(0));
    assert!(matches!(
        refused("hello 99999999999999999999 42 60"),
        NetError::OutOfRange { .. }
    ));
}

#[test]
fn refuses_out_of_range() {
    for s in [
        "input 1 1001 0 0",
        "input 1 -1 0 0",
        "input 1 0 1001 0",
        "input 1 0 -1 0",
        "input 1 0 0 1001",
        "input 1 0 0 -1001",
        "input 1 99999999999 0 0",
        "input 18446744073709551616 0 0 0",
        "hello 1 18446744073709551616 60",
        "hello 1 1 0",
        "hello 1 1 1001",
        "hash 18446744073709551616 0000000000000000",
    ] {
        assert!(matches!(refused(s), NetError::OutOfRange { .. }), "{s:?}");
    }
}

#[test]
fn refuses_non_canonical_numbers() {
    for s in [
        "input +1 0 0 0",
        "input 1 +5 0 0",
        "input 01 0 0 0",
        "input 1 007 0 0",
        "input 1 0 0 -0",
        "input 1 0 0 --5",
        "input 1 0 0 -",
        "input 1 0 0 5.0",
        "input 1 0 0 1e2",
        "input 1 0 0 0x10",
        "input 1 0 0 1_0",
        "input -1 0 0 0",
        "hello 1 007 60",
        "hello 1 42 060",
        "hello 01 42 60",
        "hello 1 -3 60",
    ] {
        assert!(
            matches!(refused(s), NetError::BadNumber { .. }),
            "{s:?} -> {:?}",
            refused(s)
        );
    }
}

#[test]
fn refuses_bad_hash_text() {
    for s in [
        "hash 1 ABCDEF0123456789",
        "hash 1 abcdef012345678",
        "hash 1 abcdef01234567890",
        "hash 1 0xabcdef0123456789",
        "hash 1 abcdef012345678g",
        "hash 1 +bcdef0123456789",
        "hash 1 -bcdef0123456789",
    ] {
        assert!(matches!(refused(s), NetError::BadNumber { .. }), "{s:?}");
    }
}

#[test]
fn refuses_bad_spacing_and_control_characters() {
    for s in [
        " bye",
        "bye ",
        "bye\n",
        "bye\r\n",
        "bye\0",
        "hello  1 42 60",
        "hello 1 42 60 ",
        "hello\t1 42 60",
        "input 1 2 3 4\n",
        "hash 1 0000000000000000\n",
        "hello 1 42\u{a0}60",
        " ",
        "input 1 0 0 \u{0661}",
    ] {
        assert_eq!(refused(s), NetError::BadSpacing, "{s:?}");
    }
}

#[test]
fn refuses_overlong_lines_before_parsing() {
    let long = format!("bye{}", " ".repeat(MAX_LINE_BYTES));
    assert_eq!(refused(&long), NetError::TooLong);
    let long = format!("input 1 2 3 {}", "9".repeat(MAX_LINE_BYTES));
    assert_eq!(refused(&long), NetError::TooLong);
}

#[test]
fn every_encoding_fits_the_line_limit() {
    let worst = [
        Msg::Hello {
            seed: u64::MAX,
            tick_rate: MAX_TICK_RATE,
        },
        Msg::Input {
            tick: u64::MAX,
            throttle: 1000,
            brake: 1000,
            steer: -1000,
        },
        Msg::Hash {
            tick: u64::MAX,
            hash: u64::MAX,
        },
    ];
    for m in worst {
        assert!(encode(&m).len() <= MAX_LINE_BYTES);
    }
}

#[test]
fn garbage_never_panics_and_accepted_text_is_canonical() {
    let mut r = Rng(0xBAD5EED);
    let alphabet: &[u8] = b"hellointpusbyae 0123456789-+abcdefx\t\n";
    for _ in 0..20000 {
        let len = (r.next() % 40) as usize;
        let s: String = (0..len)
            .map(|_| alphabet[(r.next() % alphabet.len() as u64) as usize] as char)
            .collect();
        if let Ok(m) = decode(&s) {
            assert_eq!(
                encode(&m),
                s,
                "decoded text must be exactly the canonical encoding"
            );
        }
    }
}
