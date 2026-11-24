//! Broad-phase recall is compared exactly with the specified f64 predicate, not
//! with the implementation's output or a tolerance-expanded distance.

use std::cell::RefCell;

use crash_sim::broadphase::{candidate_pairs, BroadphaseError};
use crash_sim::vec3::Vec3;

fn naive(pos: &[Vec3], radius: f64, skip: &dyn Fn(usize, usize) -> bool) -> Vec<(usize, usize)> {
    let mut result = Vec::new();
    for (i, a) in pos.iter().enumerate() {
        for (j, b) in pos.iter().enumerate().skip(i + 1) {
            let differences = [a.x - b.x, a.y - b.y, a.z - b.z];
            let squares = differences.map(|d| d * d);
            if (squares[0] + squares[1]) + squares[2] <= radius * radius && !skip(i, j) {
                result.push((i, j));
            }
        }
    }
    result
}

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }

    fn coordinate(&mut self) -> f64 {
        // Exactly representable integer divided by a power of two.
        (self.next() >> 11) as f64 / ((1_u64 << 53) as f64) * 4.0 - 2.0
    }
}

fn random_nodes(count: usize) -> Vec<Vec3> {
    let mut rng = Rng(0x9e37_79b9_7f4a_7c15);
    (0..count)
        .map(|_| Vec3::new(rng.coordinate(), rng.coordinate(), rng.coordinate()))
        .collect()
}

#[test]
fn pairs_equal_naive_on_2000_random_nodes() {
    let pos = random_nodes(2000);
    let radius = 0.19;
    let expected = naive(&pos, radius, &|_, _| false);
    assert!(!expected.is_empty(), "the seeded fixture must contain pairs");
    assert_eq!(
        candidate_pairs(&pos, radius, &|_, _| false).unwrap(),
        expected
    );
}

#[test]
fn pair_at_exactly_radius_is_included() {
    let outside = f64::from_bits(5.0_f64.to_bits() + 1);
    let pos = [
        Vec3::ZERO,
        Vec3::new(3.0, 4.0, 0.0),
        Vec3::new(0.0, 0.0, -5.0),
        Vec3::new(outside, 0.0, 0.0),
    ];
    let actual = candidate_pairs(&pos, 5.0, &|_, _| false).unwrap();
    assert!(actual.contains(&(0, 1)), "3*3 + 4*4 == 5*5");
    assert!(actual.contains(&(0, 2)), "axis-aligned boundary");
    assert!(!actual.contains(&(0, 3)), "one ulp beyond the radius");
    assert_eq!(actual, naive(&pos, 5.0, &|_, _| false));
}

#[test]
fn decimal_lattice_where_cell_rounding_bites() {
    // 0.6 / 0.1 falls below cell 6, whereas (0.7 - 0.1) * (1 / 0.1)
    // lands on 6. The separation still satisfies the exact distance predicate.
    // Enough occupied cells keep this fixture on the bounded-search path.
    let mut pos = vec![Vec3::new(0.7, 0.0, 0.0), Vec3::new(0.6, 0.0, 0.0)];
    pos.extend(
        (-200..=200)
            .rev()
            .map(|i| Vec3::new(f64::from(i) / 10.0, 0.0, 0.0)),
    );
    let expected = naive(&pos, 0.1, &|_, _| false);
    assert!(expected.contains(&(0, 1)), "rounding witness must be inside");
    let actual = candidate_pairs(&pos, 0.1, &|_, _| false).unwrap();
    assert!(actual.contains(&(0, 1)), "padded search lost the witness");
    assert_eq!(actual, expected);
}

#[test]
fn pairs_across_cell_borders_and_negative_coordinates() {
    let mut pos = vec![
        Vec3::new(0.9, 0.0, 0.0),
        Vec3::new(1.1, 0.0, 0.0),
        Vec3::new(-0.1, 0.0, 0.0),
        Vec3::new(0.1, 0.0, 0.0),
        Vec3::new(-1.02, -1.02, -1.02),
        Vec3::new(-0.98, -0.98, -0.98),
        Vec3::new(4.0, -0.1, 0.0),
        Vec3::new(4.0, 0.1, 0.0),
        Vec3::new(8.0, 0.0, -0.1),
        Vec3::new(8.0, 0.0, 0.1),
    ];
    // Isolated cells prevent the occupied-cell fallback from hiding a broken
    // neighbor search. They add no pairs to the five deliberately close pairs.
    pos.extend((0..128).map(|i| Vec3::new(100.0 + f64::from(i) * 4.0, 100.0, 100.0)));
    let expected = vec![(0, 1), (2, 3), (4, 5), (6, 7), (8, 9)];
    assert_eq!(naive(&pos, 0.25, &|_, _| false), expected);
    assert_eq!(
        candidate_pairs(&pos, 0.25, &|_, _| false).unwrap(),
        expected
    );
}

#[test]
fn skip_removes_only_the_named_pairs() {
    let pos = [
        Vec3::ZERO,
        Vec3::new(0.25, 0.0, 0.0),
        Vec3::new(0.5, 0.0, 0.0),
        Vec3::new(0.75, 0.0, 0.0),
    ];
    let calls = RefCell::new(Vec::new());
    let actual = candidate_pairs(&pos, 1.0, &|i, j| {
        assert!(i < j);
        calls.borrow_mut().push((i, j));
        (i, j) == (0, 2) || (i, j) == (1, 3)
    })
    .unwrap();
    assert_eq!(actual, vec![(0, 1), (0, 3), (1, 2), (2, 3)]);
    assert_eq!(
        calls.into_inner(),
        vec![(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)]
    );
    assert!(candidate_pairs(&pos, 1.0, &|_, _| true)
        .unwrap()
        .is_empty());
    let far_apart = [Vec3::ZERO, Vec3::new(100.0, 0.0, 0.0)];
    assert!(candidate_pairs(&far_apart, 1.0, &|_, _| panic!("not a candidate"))
        .unwrap()
        .is_empty());
}

#[test]
fn same_input_same_order() {
    let pos = random_nodes(300);
    let expected = naive(&pos, 0.4, &|_, _| false);
    assert!(!expected.is_empty());
    for _ in 0..16 {
        let actual = candidate_pairs(&pos, 0.4, &|_, _| false).unwrap();
        assert!(actual.windows(2).all(|w| w[0] < w[1]));
        assert_eq!(actual, expected);
    }
    // The unrestricted occupied-cell path must also sort before calling skip.
    let all_pairs = naive(&pos, f64::MAX, &|_, _| false);
    for _ in 0..4 {
        assert_eq!(
            candidate_pairs(&pos, f64::MAX, &|_, _| false).unwrap(),
            all_pairs
        );
    }
}

#[test]
fn refuses_nan_inf_and_bad_radius() {
    let no_skip = |_, _| panic!("invalid input must not reach skip");
    for radius in [
        -1.0,
        -f64::from_bits(1),
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
    ] {
        for pos in [&[][..], &[Vec3::ZERO][..], &[Vec3::ZERO; 2][..]] {
            assert_eq!(
                candidate_pairs(pos, radius, &no_skip),
                Err(BroadphaseError::BadRadius)
            );
        }
    }
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        for bad in [
            Vec3::new(value, 0.0, 0.0),
            Vec3::new(0.0, value, 0.0),
            Vec3::new(0.0, 0.0, value),
        ] {
            assert_eq!(
                candidate_pairs(&[bad], 1.0, &no_skip),
                Err(BroadphaseError::NonFinite { index: 0 })
            );
            assert_eq!(
                candidate_pairs(&[Vec3::ZERO, Vec3::ZERO, bad, bad], 1.0, &no_skip),
                Err(BroadphaseError::NonFinite { index: 2 })
            );
            assert_eq!(
                candidate_pairs(&[bad], f64::MAX, &no_skip),
                Err(BroadphaseError::NonFinite { index: 0 })
            );
        }
    }
}

#[test]
fn empty_and_single_node_give_no_pairs() {
    let no_skip = |_, _| panic!("no pairs to skip");
    for radius in [0.0, 0.1, f64::MAX] {
        assert_eq!(candidate_pairs(&[], radius, &no_skip).unwrap(), vec![]);
        assert_eq!(
            candidate_pairs(&[Vec3::new(-1.0, 2.0, 3.0)], radius, &no_skip).unwrap(),
            vec![]
        );
    }
}

#[test]
fn exact_dyadic_extremes_and_near_cancellation() {
    // Exact integer/dyadic references, independently specified rather than
    // inferred from a norm computed by the implementation.
    let adjacent = vec![(0, 1), (1, 2)];
    for radius in [f64::MIN_POSITIVE.sqrt(), 1.0, f64::MAX.sqrt()] {
        let pos = [
            Vec3::new(-radius, 0.0, 0.0),
            Vec3::ZERO,
            Vec3::new(radius, 0.0, 0.0),
        ];
        assert_eq!(
            candidate_pairs(&pos, radius, &|_, _| false).unwrap(),
            adjacent
        );
    }
    // An independent higher-precision oracle: exact i128 squared distances on
    // an integer lattice, then exact power-of-two scaling into three f64 bands.
    let integers = [
        [0_i32, 0, 0],
        [3, 4, 0],
        [0, 0, -5],
        [-3, -4, 0],
        [1, 1, 1],
        [6, -6, 6],
        [-6, 6, -6],
    ];
    let integer_radius = 5_i128;
    let mut exact = Vec::new();
    for (i, a) in integers.iter().enumerate() {
        for (j, b) in integers.iter().enumerate().skip(i + 1) {
            let squared: i128 = a
                .iter()
                .zip(b)
                .map(|(&x, &y)| {
                    let d = i128::from(x) - i128::from(y);
                    d * d
                })
                .sum();
            if squared <= integer_radius * integer_radius {
                exact.push((i, j));
            }
        }
    }
    assert!(exact.contains(&(0, 1)), "exact boundary in the oracle");
    assert!(!exact.contains(&(0, 5)), "oracle must also exclude pairs");
    for (scale, origin) in [
        (f64::MIN_POSITIVE.sqrt(), 0.0),
        (0.25, (1_u64 << 50) as f64),
        (2.0_f64.powi(507), 0.0),
    ] {
        let pos: Vec<_> = integers
            .iter()
            .map(|p| {
                Vec3::new(
                    (origin + f64::from(p[0])) * scale,
                    (origin + f64::from(p[1])) * scale,
                    (origin + f64::from(p[2])) * scale,
                )
            })
            .collect();
        assert_eq!(
            candidate_pairs(&pos, 5.0 * scale, &|_, _| false).unwrap(),
            exact,
            "scale={scale:e}, origin={origin:e}"
        );
    }

    let previous = f64::from_bits(f64::MAX.to_bits() - 1);
    let pos = [
        Vec3::new(f64::MAX, 0.0, 0.0),
        Vec3::new(f64::MAX, 0.0, 0.0),
        Vec3::new(-f64::MAX, 0.0, 0.0),
        Vec3::new(previous, 0.0, 0.0),
    ];
    assert_eq!(
        candidate_pairs(&pos, 1.0, &|_, _| false).unwrap(),
        vec![(0, 1)]
    );
    // MAX^2 is infinity: even an overflowing separation is inside by contract.
    assert_eq!(
        candidate_pairs(&pos, f64::MAX, &|_, _| false).unwrap(),
        vec![(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)]
    );
}

#[test]
fn zero_and_underflow_follow_the_literal_squared_predicate() {
    let tiny = f64::from_bits(1);
    let two_to_minus_538 = f64::from_bits(485_u64 << 52);
    let pos = [
        Vec3::ZERO,
        Vec3::new(tiny, 0.0, 0.0),
        Vec3::new(two_to_minus_538, 0.0, 0.0),
    ];
    // Each square rounds to zero (2^-1076 is below half of 2^-1074).
    for radius in [0.0, -0.0, tiny, f64::MIN_POSITIVE] {
        assert_eq!(
            candidate_pairs(&pos, radius, &|_, _| false).unwrap(),
            vec![(0, 1), (0, 2), (1, 2)]
        );
    }

    let radius = f64::from_bits(486_u64 << 52); // 2^-537
    let pos = [
        Vec3::ZERO,
        Vec3::new(radius * 1.125, 0.0, 0.0),
        Vec3::new(radius * 2.0, 0.0, 0.0),
    ];
    // In units of the smallest subnormal, squared distances round to 1, 4, 1,
    // while radius^2 is exactly 1. No geometric-radius bound is valid here.
    assert_eq!(
        candidate_pairs(&pos, radius, &|_, _| false).unwrap(),
        vec![(0, 1), (1, 2)]
    );
}

#[test]
fn finite_bit_patterns_match_naive_across_exponents() {
    let mut rng = Rng(0xd1b5_4a32_d192_ed03);
    let mut coordinate = || {
        // Preserve sign/fraction, replace exponent with a finite exponent.
        let bits = rng.next();
        let exponent = (rng.next() % 2047) << 52;
        f64::from_bits((bits & 0x800f_ffff_ffff_ffff) | exponent)
    };
    let mut pos: Vec<_> = (0..128)
        .map(|_| Vec3::new(coordinate(), coordinate(), coordinate()))
        .collect();
    // Include genuine contacts at the sampled exponents, not just distant pairs.
    pos.extend_from_within(..16);
    for radius in [
        0.0,
        f64::from_bits(1),
        f64::MIN_POSITIVE.sqrt(),
        0.1,
        1.0,
        1e100,
        f64::MAX.sqrt(),
        f64::MAX,
    ] {
        assert_eq!(
            candidate_pairs(&pos, radius, &|_, _| false).unwrap(),
            naive(&pos, radius, &|_, _| false),
            "radius={radius:e}"
        );
    }
}
