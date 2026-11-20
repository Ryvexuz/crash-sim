//! Tests for the state hash module. Included by path because `src/lib.rs` does not register it yet.
#![allow(dead_code)]

#[path = "../src/statehash.rs"]
mod statehash;

use statehash::*;

#[test]
fn published_vectors() {
    let h = Hasher::new();
    assert_eq!(h.finish(), 0xcbf29ce484222325);
    assert_eq!(h.hex(), "cbf29ce484222325");
    let mut h = Hasher::new();
    h.write_bytes(b"a");
    assert_eq!(h.finish(), 0xaf63dc4c8601ec8c);
    let mut h = Hasher::new();
    h.write_bytes(b"foobar");
    assert_eq!(h.finish(), 0x85944171f73967e8);
}

#[test]
fn write_u64_is_little_endian_bytes() {
    let mut a = Hasher::new();
    a.write_u64(0x0102030405060708);
    let mut b = Hasher::new();
    b.write_bytes(&[8, 7, 6, 5, 4, 3, 2, 1]);
    assert_eq!(a.finish(), b.finish());
}

#[test]
fn zero_and_negative_zero_differ() {
    let mut a = Hasher::new();
    a.write_f64(0.0).unwrap();
    let mut b = Hasher::new();
    b.write_f64(-0.0).unwrap();
    assert_ne!(a.finish(), b.finish());
}

#[test]
fn one_changed_bit_changes_hash() {
    let x = 1.2345_f64;
    let y = f64::from_bits(x.to_bits() ^ 1);
    let mut a = Hasher::new();
    a.write_f64(x).unwrap();
    let mut b = Hasher::new();
    b.write_f64(y).unwrap();
    assert_ne!(a.finish(), b.finish());
}

#[test]
fn nan_is_refused_and_leaves_state_unchanged() {
    let mut h = Hasher::new();
    h.write_u64(7);
    let before = h.finish();
    assert_eq!(h.write_f64(f64::NAN), Err(HashError::Nan));
    assert_eq!(h.write_slice_f64(&[1.0, f64::NAN]), Err(HashError::Nan));
    assert_eq!(h.finish(), before);
}

#[test]
fn slice_equals_individual_writes_and_order_matters() {
    let mut a = Hasher::new();
    a.write_slice_f64(&[1.0, 2.0, 3.0]).unwrap();
    let mut b = Hasher::new();
    for v in [1.0, 2.0, 3.0] {
        b.write_f64(v).unwrap();
    }
    assert_eq!(a.finish(), b.finish());
    let mut c = Hasher::new();
    c.write_slice_f64(&[3.0, 2.0, 1.0]).unwrap();
    assert_ne!(a.finish(), c.finish());
}

#[test]
fn hex_is_16_lowercase_digits_and_deterministic() {
    let mut h = Hasher::new();
    h.write_u64(0);
    let s = h.hex();
    assert_eq!(s.len(), 16);
    assert!(s.chars().all(|c| c.is_ascii_digit() || ('a'..='f').contains(&c)));
    let mut h2 = Hasher::new();
    h2.write_u64(0);
    assert_eq!(s, h2.hex());
    assert_eq!(Hasher::new().hex().len(), 16);
}
