use crash_sim::net::confirm::{ConfirmError, ConfirmedLines};

#[test]
fn lines_wait_for_their_interval_hash() {
    let mut lines = ConfirmedLines::new(100).unwrap();
    for (tick, t) in [(5, 0.05), (150, 1.50), (99, 0.99)] {
        lines.push(tick, t, 1, format!("car 1 tick={tick}")).unwrap();
    }
    assert_eq!(
        lines.confirm(99, 0xabc).unwrap(),
        [
            "car 1 tick=5",
            "car 1 tick=99",
            "hash tick=99 value=0000000000000abc"
        ]
    );
    assert_eq!(lines.pending(), 1);
    assert_eq!(
        lines.confirm(199, 1).unwrap(),
        ["car 1 tick=150", "hash tick=199 value=0000000000000001"]
    );
    assert_eq!(
        lines.confirm(299, 2).unwrap(),
        ["hash tick=299 value=0000000000000002"]
    );
    assert_eq!(lines.pending(), 0);
}

#[test]
fn interval_lines_sort_by_time_then_car_then_push_order() {
    let mut lines = ConfirmedLines::new(100).unwrap();
    for (t, car, text) in [
        (0.5, 2, "a"),
        (0.3, 1, "b"),
        (0.5, 1, "c"),
        (0.3, 1, "d"),
        (0.3, 2, "e"),
    ] {
        lines.push(0, t, car, text.into()).unwrap();
    }
    assert_eq!(
        lines.confirm(99, 0).unwrap(),
        ["b", "d", "e", "c", "a", "hash tick=99 value=0000000000000000"]
    );
    for (t, text) in [(0.0, "positive"), (-0.0, "negative")] {
        lines.push(100, t, 1, text.into()).unwrap();
    }
    assert_eq!(
        lines.confirm(199, 0).unwrap(),
        ["negative", "positive", "hash tick=199 value=0000000000000000"]
    );
}

#[test]
fn desync_drops_the_unconfirmed_interval() {
    let mut lines = ConfirmedLines::new(100).unwrap();
    for interval in 0..=5 {
        for car in 1..=2 {
            let tick = interval * 100;
            lines
                .push(tick, 0.0, car, format!("car {car} tick={tick}"))
                .unwrap();
        }
    }
    let mut output = Vec::new();
    for tick in [99, 199, 299, 399, 499] {
        output.extend(lines.confirm(tick, 0).unwrap());
    }
    assert_eq!(lines.discard(), 2);
    assert_eq!(lines.pending(), 0);
    let expected: Vec<_> = (0..5)
        .flat_map(|i| {
            [
                format!("car 1 tick={}", i * 100),
                format!("car 2 tick={}", i * 100),
                format!("hash tick={} value=0000000000000000", i * 100 + 99),
            ]
        })
        .collect();
    assert_eq!(output, expected);
    assert_eq!(output.iter().filter(|s| s.starts_with("hash ")).count(), 5);
    assert_eq!(
        lines.push(500, 0.0, 1, "car".into()),
        Err(ConfirmError::Stopped)
    );
    assert_eq!(
        lines.push(0, f64::NAN, 0, String::new()),
        Err(ConfirmError::Stopped)
    );
    assert_eq!(lines.confirm(599, 0), Err(ConfirmError::Stopped));
    assert_eq!(lines.next_hash_tick(), Err(ConfirmError::Stopped));
    assert_eq!(lines.discard(), 0);
}

#[test]
fn confirm_and_push_refusals_are_named() {
    assert!(matches!(
        ConfirmedLines::new(0),
        Err(ConfirmError::ZeroInterval)
    ));
    let mut lines = ConfirmedLines::new(100).unwrap();
    lines.push(0, 0.0, 1, "held".into()).unwrap();
    for got in [98, 199] {
        assert_eq!(
            lines.confirm(got, 0),
            Err(ConfirmError::WrongHashTick { expected: 99, got })
        );
        assert_eq!(lines.pending(), 1);
        assert_eq!(lines.next_hash_tick(), Ok(99));
    }
    assert_eq!(
        lines.confirm(99, 0).unwrap(),
        ["held", "hash tick=99 value=0000000000000000"]
    );
    assert_eq!(
        lines.push(50, 0.0, 1, "late".into()),
        Err(ConfirmError::LateLine {
            tick: 50,
            confirmed_through: 99
        })
    );
    assert_eq!(lines.pending(), 0);
    for t in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert_eq!(
            lines.push(50, t, 1, String::new()),
            Err(ConfirmError::BadTime { tick: 50 })
        );
        assert_eq!(lines.pending(), 0);
    }
    for text in ["", "a\nb", "a\rb"] {
        assert_eq!(
            lines.push(50, 0.0, 1, text.into()),
            Err(ConfirmError::BadLine { tick: 50 })
        );
        assert_eq!(lines.pending(), 0);
    }
    let mut lines = ConfirmedLines::new(u64::MAX).unwrap();
    assert_eq!(lines.next_hash_tick(), Ok(u64::MAX - 1));
    lines.push(u64::MAX, f64::MAX, 1, "future".into()).unwrap();
    assert_eq!(lines.confirm(u64::MAX - 1, 0).unwrap().len(), 1);
    assert_eq!(lines.next_hash_tick(), Err(ConfirmError::Overflow));
    assert_eq!(lines.confirm(u64::MAX, 0), Err(ConfirmError::Overflow));
    assert_eq!(lines.pending(), 1);
    let mut lines = ConfirmedLines::new(1).unwrap();
    for (t, text) in [
        (f64::MAX, "max"),
        (f64::from_bits(1), "tiny"),
        (-f64::MAX, "min"),
    ] {
        lines.push(0, t, 1, text.into()).unwrap();
    }
    assert_eq!(
        lines.confirm(0, 0).unwrap(),
        ["min", "tiny", "max", "hash tick=0 value=0000000000000000"]
    );
}

#[test]
fn hash_line_format_is_sixteen_lower_hex() {
    let mut lines = ConfirmedLines::new(10).unwrap();
    assert_eq!(
        lines.confirm(9, 0x1F).unwrap(),
        ["hash tick=9 value=000000000000001f"]
    );
    assert_eq!(
        lines.confirm(19, u64::MAX).unwrap(),
        ["hash tick=19 value=ffffffffffffffff"]
    );
}
