#[path = "../src/vec3.rs"]
mod vec3;
#[path = "../src/speed_profile.rs"]
mod speed_profile;

use speed_profile::{curvature, profile, ProfileError};
use vec3::Vec3;

const MU: f64 = 1.0;
const G: f64 = 9.81;

fn circle(r: f64, n: usize) -> Vec<Vec3> {
    (0..n)
        .map(|i| {
            let a = std::f64::consts::TAU * i as f64 / n as f64;
            Vec3::new(r * a.cos(), 0.0, r * a.sin())
        })
        .collect()
}

/// Stadium: two straights of length `len` joined by semicircles of radius `r`.
fn stadium(len: f64, r: f64, per_arc: usize, per_straight: usize) -> Vec<Vec3> {
    let mut p = Vec::new();
    for i in 0..per_straight {
        p.push(Vec3::new(len * i as f64 / per_straight as f64, 0.0, -r));
    }
    for i in 0..per_arc {
        let a = -std::f64::consts::FRAC_PI_2 + std::f64::consts::PI * i as f64 / per_arc as f64;
        p.push(Vec3::new(len + r * a.cos(), 0.0, r * a.sin()));
    }
    for i in 0..per_straight {
        p.push(Vec3::new(len - len * i as f64 / per_straight as f64, 0.0, r));
    }
    for i in 0..per_arc {
        let a = std::f64::consts::FRAC_PI_2 + std::f64::consts::PI * i as f64 / per_arc as f64;
        p.push(Vec3::new(r * a.cos(), 0.0, r * a.sin()));
    }
    p
}

fn ds(p: &[Vec3], i: usize) -> f64 {
    let d = p[(i + 1) % p.len()] - p[i];
    (d.x * d.x + d.z * d.z).sqrt()
}

#[test]
fn huge_radius_loop_gives_v_max() {
    let p = circle(1.0e7, 64);
    let v = profile(&p, MU, G, 5.0, 8.0, 60.0).unwrap();
    for x in v {
        assert_eq!(x, 60.0);
    }
}

#[test]
fn circle_gives_grip_limit_everywhere() {
    let r = 40.0;
    let p = circle(r, 90);
    let want = (MU * G * r).sqrt();
    let v = profile(&p, MU, G, 5.0, 8.0, 100.0).unwrap();
    for x in v {
        assert!(((x - want) / want).abs() < 1e-9, "{x} vs {want}");
    }
}

#[test]
fn curvature_sign_and_size() {
    let r = 25.0;
    let k = curvature(&circle(r, 40)).unwrap();
    for x in &k {
        assert!(((x - 1.0 / r) / (1.0 / r)).abs() < 1e-9);
    }
    let mut rev = circle(r, 40);
    rev.reverse();
    for x in curvature(&rev).unwrap() {
        assert!(x < 0.0);
    }
}

#[test]
fn hairpin_respects_accel_and_brake_bounds() {
    let (a_acc, a_brk) = (4.0, 9.0);
    let p = stadium(300.0, 8.0, 24, 30);
    let v = profile(&p, MU, G, a_acc, a_brk, 70.0).unwrap();
    let n = p.len();
    let corner = (MU * G * 8.0).sqrt();
    assert!(v.iter().cloned().fold(f64::MAX, f64::min) <= corner * (1.0 + 1e-6));
    assert!(v.iter().cloned().fold(0.0, f64::max) > corner * 2.0, "straights should be fast");
    for i in 0..n {
        let j = (i + 1) % n;
        let d = ds(&p, i);
        assert!(v[j] * v[j] <= v[i] * v[i] + 2.0 * a_acc * d + 1e-6, "accel bound at {i}");
        assert!(v[i] * v[i] <= v[j] * v[j] + 2.0 * a_brk * d + 1e-6, "brake bound at {i}");
    }
}

#[test]
fn profile_is_continuous_across_the_seam() {
    let (a_acc, a_brk) = (4.0, 9.0);
    // Start the list in the middle of a corner so the seam sits in a slow part.
    let mut p = stadium(200.0, 10.0, 20, 25);
    p.rotate_left(33);
    let v = profile(&p, MU, G, a_acc, a_brk, 70.0).unwrap();
    let n = p.len();
    let d = ds(&p, n - 1);
    assert!(v[0] * v[0] <= v[n - 1] * v[n - 1] + 2.0 * a_acc * d + 1e-6);
    assert!(v[n - 1] * v[n - 1] <= v[0] * v[0] + 2.0 * a_brk * d + 1e-6);
    // Same profile whichever point the list starts at.
    let mut q = stadium(200.0, 10.0, 20, 25);
    let w = profile(&q, MU, G, a_acc, a_brk, 70.0).unwrap();
    q.rotate_left(33);
    for i in 0..n {
        assert!((v[i] - w[(i + 33) % n]).abs() < 1e-9);
    }
}

#[test]
fn refusals() {
    let ok = circle(30.0, 16);
    let run = |p: &[Vec3], mu, g, aa, ab, vm| profile(p, mu, g, aa, ab, vm);
    assert_eq!(run(&ok[..2], MU, G, 5.0, 8.0, 50.0), Err(ProfileError::TooFewPoints(2)));
    assert!(matches!(curvature(&ok[..2]), Err(ProfileError::TooFewPoints(2))));
    let mut dup = ok.clone();
    dup[4] = dup[5];
    assert_eq!(run(&dup, MU, G, 5.0, 8.0, 50.0), Err(ProfileError::DuplicatePoint(4)));
    let mut wrap = ok.clone();
    *wrap.last_mut().unwrap() = wrap[0];
    assert_eq!(run(&wrap, MU, G, 5.0, 8.0, 50.0), Err(ProfileError::DuplicatePoint(15)));
    assert_eq!(run(&ok, 0.0, G, 5.0, 8.0, 50.0), Err(ProfileError::BadParameter("mu")));
    assert_eq!(run(&ok, MU, -1.0, 5.0, 8.0, 50.0), Err(ProfileError::BadParameter("g")));
    assert_eq!(run(&ok, MU, G, 0.0, 8.0, 50.0), Err(ProfileError::BadParameter("a_accel")));
    assert_eq!(run(&ok, MU, G, 5.0, -2.0, 50.0), Err(ProfileError::BadParameter("a_brake")));
    assert_eq!(run(&ok, MU, G, 5.0, 8.0, 0.0), Err(ProfileError::BadParameter("v_max")));
    assert_eq!(run(&ok, f64::NAN, G, 5.0, 8.0, 50.0), Err(ProfileError::BadParameter("mu")));
    assert_eq!(run(&ok, MU, G, 5.0, 8.0, f64::INFINITY), Err(ProfileError::BadParameter("v_max")));
}

#[test]
fn non_finite_coordinates_are_refused_not_run_at_full_speed() {
    for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        for axis in 0..3 {
            let mut p = circle(30.0, 16);
            match axis {
                0 => p[7].x = bad,
                1 => p[7].y = bad,
                _ => p[7].z = bad,
            }
            assert_eq!(curvature(&p), Err(ProfileError::NonFinitePoint(7)), "curvature {bad} axis {axis}");
            assert_eq!(
                profile(&p, MU, G, 5.0, 8.0, 50.0),
                Err(ProfileError::NonFinitePoint(7)),
                "profile {bad} axis {axis}"
            );
        }
    }
}

#[test]
fn non_finite_parameters_are_refused() {
    let ok = circle(30.0, 16);
    let n = f64::NAN;
    let i = f64::INFINITY;
    assert_eq!(profile(&ok, MU, n, 5.0, 8.0, 50.0), Err(ProfileError::BadParameter("g")));
    assert_eq!(profile(&ok, MU, i, 5.0, 8.0, 50.0), Err(ProfileError::BadParameter("g")));
    assert_eq!(profile(&ok, MU, G, n, 8.0, 50.0), Err(ProfileError::BadParameter("a_accel")));
    assert_eq!(profile(&ok, MU, G, i, 8.0, 50.0), Err(ProfileError::BadParameter("a_accel")));
    assert_eq!(profile(&ok, MU, G, 5.0, n, 50.0), Err(ProfileError::BadParameter("a_brake")));
    assert_eq!(profile(&ok, MU, G, 5.0, i, 50.0), Err(ProfileError::BadParameter("a_brake")));
    assert_eq!(profile(&ok, MU, G, 5.0, 8.0, n), Err(ProfileError::BadParameter("v_max")));
    assert_eq!(profile(&ok, i, G, 5.0, 8.0, 50.0), Err(ProfileError::BadParameter("mu")));
}

#[test]
fn a_spike_is_refused_not_given_full_speed() {
    // Out along +x, then straight back: point 2's neighbours (1 and 3) coincide.
    let p = vec![
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(10.0, 0.0, 0.0),
        Vec3::new(20.0, 0.0, 0.0),
        Vec3::new(10.0, 0.0, 0.0),
        Vec3::new(10.0, 0.0, 5.0),
    ];
    // p[1] == p[3]: spike at 2.
    assert_eq!(curvature(&p), Err(ProfileError::Spike(2)));
    assert_eq!(profile(&p, MU, G, 5.0, 8.0, 50.0), Err(ProfileError::Spike(2)));
}

#[test]
fn overflowing_curvature_is_refused() {
    let big = 1.0e200;
    let p = vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(big, 0.0, 0.0), Vec3::new(big, 0.0, big)];
    assert!(matches!(curvature(&p), Err(ProfileError::NonFiniteCurvature(_))));
}
