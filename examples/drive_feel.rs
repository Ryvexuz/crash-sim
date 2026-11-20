//! Drive-feel diagnosis (measure only, task t-20261120-220016; results in docs/DRIVE_FEEL.md).
//!
//! `drive_feel <input file> [--body <car.bsn>] [--every <s>]` replays an input script through the
//! crate's public API the way `crash-sim --body <car.bsn> --input <file>` does (`settle::car_sim`,
//! `settle::run_substeps` with the run ending at the script's last line, `Script::apply` at the
//! start of every substep, then `Sim::advance` of one substep), and prints one row every `--every`
//! seconds (default 0.1) of what the drive rows do not show. The body defaults to
//! `scenarios/pad_and_wall.bsn`, the car of check 3.6 and of the viewer. Columns:
//!
//! - `speed_mps`: ground-plane speed of the centre of mass (chassis nodes and wheels);
//! - `yaw_dps`: heading change over the last interval / interval (deg/s), heading from the
//!   car's fitted frame (`Car::frame.forward`, atan2(z, x) as in `input.rs`);
//! - `beta_deg`: body sideslip, the angle from the heading to the centre-of-mass velocity
//!   (positive toward the car's right); NaN below 1 m/s;
//! - `ax_g`, `ay_g`: mean acceleration of the centre of mass over the last interval, along the
//!   heading and to the car's right, in g (`cfg.gravity`);
//! - per wheel `<w>_c` contact (1 when its ground load is above 0), `<w>_a` slip angle (deg,
//!   atan2(lateral, |longitudinal|) of the wheel centre's ground velocity in the wheel's rolling
//!   axes; NaN below 1 m/s), `<w>_k` slip ratio (spin x radius - v_long) / max(|v_long|, 1 m/s),
//!   and `<w>_ux`, `<w>_uy` the longitudinal and lateral parts of the patch's elastic force
//!   divided by mu x load (|(ux, uy)| = 1 means the tyre is sliding on its friction circle).
//!
//! The wheel's rolling axes are rebuilt from public state with the same formula as the private
//! `Car::wheel_axle` (the chassis axle turned by steer x max_steer for a steerable wheel) and
//! `patch_axes` (rolling = unit(axle x n), lateral = n x rolling). The patch force here is the
//! elastic part only (load-scaled stiffness x deflection, after the friction-circle clamp of
//! `post_step`); the damping term of `tyre_force` and the actual tyre force vector are private,
//! so they are listed as not measurable without instrumentation.
//!
//! - `wall`: 1 from the first substep at which the car touches a plane after the ground (a
//!   chassis node at or behind it, or a tyre pressed into it: the rule of the private
//!   `input::wall_touched`, rebuilt here). The summary uses only the rows before it, so the crash
//!   does not count as driving.
//!
//! Before the rows a `# car` line prints figures computed from the car file: the drive force per
//! driven wheel at full throttle (max_drive_torque / driven wheels / radius) against the friction
//! limit mu x reference load, and the slip angle and slip ratio at which the linear patch force
//! reaches that limit at the reference load (atan(mu / lat_stiffness), mu / long_stiffness: the
//! steady-state patch deflection over sigma is tan(alpha) laterally and the slip ratio along).
//! After the rows a summary block prints the peaks and the windows the diagnosis uses.

use crash_sim::format::Body;
use crash_sim::input::Script;
use crash_sim::settle::{car_sim, run_substeps, CarSetup};
use crash_sim::vec3::Vec3;
use crash_sim::vehicle::{Car, TYRE_RELAXATION_LENGTH};
use crash_sim::{Config, Sim};
use std::io::Write;

fn fail(msg: &str) -> ! {
    eprintln!("drive_feel: {msg}");
    eprintln!("usage: drive_feel <input file> [--body <car.bsn>] [--every <s>]");
    std::process::exit(2);
}

/// Below this ground speed (m/s) slip angle and sideslip are not defined (printed NaN), and
/// it is the floor of the slip ratio's denominator.
const SLOW: f64 = 1.0;

struct Args {
    input: String,
    body: String,
    every: f64,
}

fn args() -> Args {
    let mut input = None;
    let mut body = "scenarios/pad_and_wall.bsn".to_string();
    let mut every = 0.1;
    let mut a = std::env::args().skip(1);
    while let Some(x) = a.next() {
        match x.as_str() {
            "--body" => body = a.next().unwrap_or_else(|| fail("--body needs a file path")),
            "--every" => {
                let v = a.next().unwrap_or_else(|| fail("--every needs a value"));
                every = match v.parse::<f64>() {
                    Ok(e) if e.is_finite() && e > 0.0 => e,
                    _ => fail(&format!("--every must be a positive number, got '{v}'")),
                };
            }
            _ if input.is_none() && !x.starts_with("--") => input = Some(x),
            _ => fail(&format!("unknown argument '{x}'")),
        }
    }
    Args {
        input: input.unwrap_or_else(|| fail("an input file is required")),
        body,
        every,
    }
}

fn read(path: &str, what: &str) -> String {
    std::fs::read_to_string(path).unwrap_or_else(|e| fail(&format!("{what} {path}: {e}")))
}

/// Ground-plane part of `v` for a plane with unit normal `n`.
fn in_plane(v: Vec3, n: Vec3) -> Vec3 {
    v - n * v.dot(n)
}

/// Centre-of-mass velocity of the whole car (chassis nodes and wheels) and its total mass.
fn com_velocity(sim: &Sim, car: &Car) -> (Vec3, f64) {
    let mut m = 0.0;
    let mut p = Vec3::ZERO;
    for (v, mi) in sim.vel.iter().zip(&sim.mass) {
        p += *v * *mi;
        m += mi;
    }
    for w in &car.wheels {
        p += w.vel * w.p.mass;
        m += w.p.mass;
    }
    (p * (1.0 / m), m)
}

/// Same as the private `Car::wheel_axle`: the chassis axle, turned about up by steer x max_steer
/// for a steerable wheel.
fn wheel_axle(car: &Car, steerable: bool) -> Vec3 {
    let fr = car.frame;
    if !steerable || car.steer == 0.0 {
        return fr.axle;
    }
    let (s, c) = (car.steer * car.max_steer).sin_cos();
    let right = fr.forward.cross(fr.up);
    let rolling = fr.forward * c + right * s;
    fr.up.cross(rolling)
}

#[derive(Clone, Copy, Default)]
struct WheelRow {
    contact: bool,
    alpha_deg: f64,
    kappa: f64,
    ux: f64,
    uy: f64,
}

#[derive(Clone, Default)]
struct Row {
    t: f64,
    throttle: f64,
    brake: f64,
    steer: f64,
    speed: f64,
    yaw_dps: f64,
    beta_deg: f64,
    ax_g: f64,
    ay_g: f64,
    wall: bool,
    wheels: Vec<WheelRow>,
}

/// The rule of the private `input::wall_touched`: a chassis node at or behind a plane after the
/// ground, or a tyre pressed into one.
fn wall_touched(sim: &Sim, car: &Car) -> bool {
    sim.planes.iter().skip(1).any(|pl| {
        sim.pos.iter().any(|p| pl.distance(*p) <= 0.0)
            || car
                .wheels
                .iter()
                .any(|w| w.p.radius - pl.distance(w.pos) > 0.0)
    })
}

fn wheel_rows(sim: &Sim, car: &Car) -> Vec<WheelRow> {
    let n = sim.planes[0].normal;
    car.wheels
        .iter()
        .map(|w| {
            let axle = wheel_axle(car, w.p.steerable);
            let roll = axle.cross(n).normalized().unwrap_or(car.frame.forward);
            let lat = n.cross(roll);
            let vt = in_plane(w.vel, n);
            let (vx, vy) = (vt.dot(roll), vt.dot(lat));
            let speed = vt.length();
            let alpha_deg = if speed >= SLOW {
                vy.atan2(vx.abs()).to_degrees()
            } else {
                f64::NAN
            };
            let kappa = (w.spin * w.p.radius - vx) / vx.abs().max(SLOW);
            let load = w.load[0];
            let (ux, uy) = if load > 0.0 {
                let kz = car.reference_load / TYRE_RELAXATION_LENGTH;
                let d = w.deflection[0];
                let cap = w.p.mu * load;
                // The patch force opposes the deflection; report it as the force on the car.
                (
                    -kz * w.p.long_stiffness * d.dot(roll) / cap,
                    -kz * w.p.lat_stiffness * d.dot(lat) / cap,
                )
            } else {
                (0.0, 0.0)
            };
            WheelRow {
                contact: load > 0.0,
                alpha_deg,
                kappa,
                ux,
                uy,
            }
        })
        .collect()
}

fn heading(car: &Car, n: Vec3) -> (f64, Vec3, Vec3) {
    let f = in_plane(car.frame.forward, n)
        .normalized()
        .unwrap_or(Vec3::new(1.0, 0.0, 0.0));
    let right = f.cross(n);
    (f.z.atan2(f.x), f, right)
}

fn wrap(a: f64) -> f64 {
    let tau = std::f64::consts::TAU;
    let mut x = a % tau;
    if x > std::f64::consts::PI {
        x -= tau;
    } else if x <= -std::f64::consts::PI {
        x += tau;
    }
    x
}

fn f(x: f64, d: usize) -> String {
    if x.is_finite() {
        format!("{x:.d$}")
    } else {
        "nan".into()
    }
}

fn main() {
    let a = args();
    let body: Body = crash_sim::format::parse(&read(&a.body, "--body"))
        .unwrap_or_else(|e| fail(&format!("{}: {e}", a.body)));
    let script = Script::parse(&read(&a.input, "input"))
        .unwrap_or_else(|e| fail(&format!("{}: {e}", a.input)));
    let cfg = Config::default();
    let setup = CarSetup {
        script: Some(script),
        until_script_end: true,
        grip_frac: None,
    };
    let h = cfg.dt / cfg.substeps as f64;
    let total = run_substeps(&cfg, &setup).unwrap_or_else(|e| fail(&e));
    let mut one = cfg.clone();
    one.dt = h;
    one.substeps = 1;
    let mut sim = car_sim(one, &body, &setup).unwrap_or_else(|e| fail(&e));
    let script = setup.script.as_ref().expect("set above");
    let tol = h * 1e-6;
    let every = ((a.every / h).round() as u64).max(1);
    let g = cfg.gravity;
    let n = sim.planes[0].normal;

    let (names, driven, steered): (Vec<String>, Vec<usize>, Vec<usize>) = {
        let car = sim
            .car
            .as_ref()
            .unwrap_or_else(|| fail("the body has no car"));
        let pick = |want: fn(&crash_sim::vehicle::Wheel) -> bool| {
            (0..car.wheels.len())
                .filter(|&i| want(&car.wheels[i]))
                .collect::<Vec<usize>>()
        };
        (
            car.wheels.iter().map(|w| w.p.name.clone()).collect(),
            pick(|w| w.p.driven),
            pick(|w| w.p.steerable),
        )
    };
    if driven.is_empty() || steered.is_empty() {
        fail("the car needs a driven and a steerable wheel for this diagnosis");
    }
    let (mut v_prev, _) = com_velocity(&sim, sim.car.as_ref().expect("car"));
    let (h0, _, _) = heading(sim.car.as_ref().expect("car"), n);
    let mut hd_prev = h0;
    let mut yaw_acc = 0.0_f64; // radians since the last row
    let mut heading_total = 0.0_f64;
    let mut rows: Vec<Row> = Vec::new();
    let mut abort = None;
    let mut wall_s: Option<f64> = None;
    let car_line = {
        let car = sim.car.as_ref().expect("car");
        let w0 = &car.wheels[driven[0]];
        let drive_force = car.max_drive_torque / driven.len() as f64 / w0.p.radius;
        let grip = w0.p.mu * car.reference_load;
        let s0 = &car.wheels[steered[0]];
        format!(
            "# car mass_kg={} reference_load_n={} driven_wheels={} full_throttle_drive_force_per_driven_wheel_n={} mu_x_reference_load_n={} drive_over_grip={} max_steer_deg={} lat_saturation_alpha_deg={} long_saturation_kappa={}\n",
            f(com_velocity(&sim, car).1, 1),
            f(car.reference_load, 1),
            driven.len(),
            f(drive_force, 1),
            f(grip, 1),
            f(drive_force / grip, 3),
            f(car.max_steer.to_degrees(), 1),
            f((s0.p.mu / s0.p.lat_stiffness).atan().to_degrees(), 2),
            f(w0.p.mu / w0.p.long_stiffness, 4)
        )
    };
    for i in 1..=total {
        let t_start = (i - 1) as f64 * h;
        if let Err(e) = script.apply(&mut sim, t_start, tol) {
            fail(&e);
        }
        sim.advance();
        if let Err(e) = sim.check() {
            abort = Some(format!("unstable at t = {:.4} s: {e}", i as f64 * h));
            break;
        }
        let car = sim.car.as_ref().expect("car");
        let (hd, fwd, right) = heading(car, n);
        let dh = wrap(hd - hd_prev);
        yaw_acc += dh;
        heading_total += dh;
        hd_prev = hd;
        if wall_s.is_none() && wall_touched(&sim, car) {
            wall_s = Some(i as f64 * h);
        }
        if i % every == 0 {
            let dt = every as f64 * h;
            let (v, _) = com_velocity(&sim, car);
            let acc = in_plane(v - v_prev, n) * (1.0 / dt);
            v_prev = v;
            let vg = in_plane(v, n);
            let speed = vg.length();
            let beta_deg = if speed >= SLOW {
                vg.dot(right).atan2(vg.dot(fwd)).to_degrees()
            } else {
                f64::NAN
            };
            rows.push(Row {
                t: i as f64 * h,
                throttle: car.throttle,
                brake: car.brake,
                steer: car.steer,
                speed,
                yaw_dps: (yaw_acc / dt).to_degrees(),
                beta_deg,
                ax_g: acc.dot(fwd) / g,
                ay_g: acc.dot(right) / g,
                wall: wall_s.is_some(),
                wheels: wheel_rows(&sim, car),
            });
            yaw_acc = 0.0;
        }
    }

    let mut out = String::new();
    out.push_str(&format!(
        "# drive_feel input={} body={} dt={} substeps={} row_every_s={}\n",
        a.input,
        a.body,
        cfg.dt,
        cfg.substeps,
        every as f64 * h
    ));
    out.push_str(&car_line);
    out.push_str("# not measurable without instrumentation: the tyre force vector actually applied (damping term of tyre_force), the per-wheel drive and brake torque reaching the spin after the brake clamp\n");
    let mut head = "t,thr,brk,steer,speed_mps,yaw_dps,beta_deg,ax_g,ay_g,wall".to_string();
    for w in &names {
        head.push_str(&format!(",{w}_c,{w}_a,{w}_k,{w}_ux,{w}_uy"));
    }
    out.push_str(&head);
    out.push('\n');
    for r in &rows {
        out.push_str(&format!(
            "{},{},{},{},{},{},{},{},{},{}",
            f(r.t, 2),
            f(r.throttle, 2),
            f(r.brake, 2),
            f(r.steer, 2),
            f(r.speed, 2),
            f(r.yaw_dps, 1),
            f(r.beta_deg, 1),
            f(r.ax_g, 3),
            f(r.ay_g, 3),
            u8::from(r.wall)
        ));
        for w in &r.wheels {
            out.push_str(&format!(
                ",{},{},{},{},{}",
                u8::from(w.contact),
                f(w.alpha_deg, 1),
                f(w.kappa, 3),
                f(w.ux, 2),
                f(w.uy, 2)
            ));
        }
        out.push('\n');
    }
    out.push_str(&summary(
        &rows,
        &names,
        &driven,
        &steered,
        heading_total,
        wall_s,
    ));
    if let Some(e) = &abort {
        out.push_str(&format!("abort: {e}\n"));
    }
    // A closed stdout must not kill the run (SPEC_CHECKLIST, output streams).
    let _ = std::io::stdout().write_all(out.as_bytes());
    if abort.is_some() {
        std::process::exit(1);
    }
}

/// A number read from a row (for the peaks).
type RowKey = fn(&Row) -> f64;
/// Which rows a bin of the summary takes.
type RowPick = fn(&Row) -> bool;

fn max_by(rows: &[Row], key: impl Fn(&Row) -> f64) -> Option<&Row> {
    rows.iter()
        .filter(|r| key(r).is_finite())
        .max_by(|a, b| key(a).total_cmp(&key(b)))
}

fn row_brief(r: &Row, names: &[String]) -> String {
    let mut s = format!(
        "t={} thr={} brk={} steer={} speed_mps={} yaw_dps={} beta_deg={} ax_g={} ay_g={}",
        f(r.t, 2),
        f(r.throttle, 2),
        f(r.brake, 2),
        f(r.steer, 2),
        f(r.speed, 2),
        f(r.yaw_dps, 1),
        f(r.beta_deg, 1),
        f(r.ax_g, 3),
        f(r.ay_g, 3)
    );
    for (w, name) in r.wheels.iter().zip(names) {
        s.push_str(&format!(
            " {name}[a={} k={} u={}]",
            f(w.alpha_deg, 1),
            f(w.kappa, 3),
            f((w.ux * w.ux + w.uy * w.uy).sqrt(), 2)
        ));
    }
    s
}

/// Mean of `key` over `rows` (finite values only), and how many.
fn mean_of(rows: &[&Row], key: impl Fn(&Row) -> f64) -> (f64, usize) {
    let v: Vec<f64> = rows
        .iter()
        .map(|r| key(r))
        .filter(|x| x.is_finite())
        .collect();
    if v.is_empty() {
        (f64::NAN, 0)
    } else {
        (v.iter().sum::<f64>() / v.len() as f64, v.len())
    }
}

fn summary(
    all: &[Row],
    names: &[String],
    driven: &[usize],
    front: &[usize],
    heading_total: f64,
    wall_s: Option<f64>,
) -> String {
    let mut s = String::from(
        "# summary (rows before the first wall contact only)
",
    );
    let rows: Vec<Row> = all.iter().filter(|r| !r.wall).cloned().collect();
    s.push_str(&format!(
        "rows_total={} rows_before_wall={} wall_first_contact_s={} heading_total_deg_whole_run={}
",
        all.len(),
        rows.len(),
        wall_s.map_or("never".to_string(), |t| f(t, 3)),
        f(heading_total.to_degrees(), 1)
    ));
    if rows.is_empty() {
        s.push_str(
            "summary: no rows before the wall
",
        );
        return s;
    }
    let peaks: [(&str, RowKey); 5] = [
        ("peak_abs_ay_g", |r| r.ay_g.abs()),
        ("peak_abs_yaw_dps", |r| r.yaw_dps.abs()),
        ("peak_abs_beta_deg", |r| r.beta_deg.abs()),
        ("peak_speed_mps", |r| r.speed),
        ("peak_abs_ax_g", |r| r.ax_g.abs()),
    ];
    for (label, key) in peaks {
        if let Some(r) = max_by(&rows, key) {
            s.push_str(&format!(
                "{label}={} at {}
",
                f(key(r), 3),
                row_brief(r, names)
            ));
        }
    }
    // Spin moments: each time |beta| first rises past 30 deg (after being under 15 deg).
    let mut armed = true;
    let mut spins = 0;
    for (k, r) in rows.iter().enumerate() {
        let b = r.beta_deg.abs();
        if !b.is_finite() {
            continue;
        }
        if armed && b > 30.0 {
            spins += 1;
            armed = false;
            s.push_str(&format!(
                "spin_moment {spins}: {}
",
                row_brief(r, names)
            ));
            // The rows just before it: what the car was doing as it let go.
            for back in [10usize, 5, 2] {
                if k >= back {
                    s.push_str(&format!(
                        "  before -{back}: {}
",
                        row_brief(&rows[k - back], names)
                    ));
                }
            }
        } else if b < 15.0 {
            armed = true;
        }
    }
    s.push_str(&format!(
        "spin_moments={spins}
"
    ));
    let util = |r: &Row, i: usize| (r.wheels[i].ux.powi(2) + r.wheels[i].uy.powi(2)).sqrt();
    let avg = |r: &Row, ix: &[usize], v: &dyn Fn(&WheelRow) -> f64| {
        ix.iter().map(|&i| v(&r.wheels[i])).sum::<f64>() / ix.len() as f64
    };
    // By throttle: how much of the driven tyres' friction circle the drive takes, and what is
    // left for cornering.
    let bins: [(&str, RowPick); 3] = [
        ("throttle_full", |r| r.throttle >= 0.999),
        ("throttle_part", |r| {
            r.throttle > 0.001 && r.throttle < 0.999
        }),
        ("throttle_off", |r| r.throttle <= 0.001),
    ];
    for (label, pick) in bins {
        let sel: Vec<&Row> = rows.iter().filter(|r| pick(r) && r.speed >= SLOW).collect();
        let sat = sel
            .iter()
            .filter(|r| {
                driven
                    .iter()
                    .all(|&i| r.wheels[i].contact && util(r, i) > 0.98)
            })
            .count();
        let (thr, _) = mean_of(&sel, |r| r.throttle);
        let (kap, _) = mean_of(&sel, |r| avg(r, driven, &|w| w.kappa));
        let (ux, _) = mean_of(&sel, |r| avg(r, driven, &|w| w.ux.abs()));
        let (uy, _) = mean_of(&sel, |r| avg(r, driven, &|w| w.uy.abs()));
        let (ad, _) = mean_of(&sel, |r| avg(r, driven, &|w| w.alpha_deg.abs()));
        let (af, _) = mean_of(&sel, |r| avg(r, front, &|w| w.alpha_deg.abs()));
        let (beta, _) = mean_of(&sel, |r| r.beta_deg.abs());
        let (ay, _) = mean_of(&sel, |r| r.ay_g.abs());
        s.push_str(&format!(
            "{label}: rows={} mean_throttle={} driven_both_saturated_rows={} mean_driven_kappa={} mean_driven_abs_ux={} mean_driven_abs_uy={} mean_driven_abs_alpha_deg={} mean_steered_abs_alpha_deg={} mean_abs_beta_deg={} mean_abs_ay_g={}
",
            sel.len(),
            f(thr, 3),
            sat,
            f(kap, 3),
            f(ux, 2),
            f(uy, 2),
            f(ad, 1),
            f(af, 1),
            f(beta, 1),
            f(ay, 3)
        ));
    }
    // Full lock at speed: the steered tyres' slip angles.
    let lock: Vec<&Row> = rows
        .iter()
        .filter(|r| r.steer.abs() >= 0.999 && r.speed >= 5.0)
        .collect();
    let (fa, _) = mean_of(&lock, |r| avg(r, front, &|w| w.alpha_deg.abs()));
    let (fu, _) = mean_of(&lock, |r| {
        front.iter().map(|&i| util(r, i)).sum::<f64>() / front.len() as f64
    });
    let (ay, _) = mean_of(&lock, |r| r.ay_g.abs());
    s.push_str(&format!(
        "full_lock_at_or_above_5mps: rows={} mean_steered_abs_alpha_deg={} mean_steered_util={} mean_abs_ay_g={}
",
        lock.len(),
        f(fa, 1),
        f(fu, 2),
        f(ay, 3)
    ));
    // Coasting: no throttle, no brake, small steer, above 2 m/s.
    let coast: Vec<&Row> = rows
        .iter()
        .filter(|r| {
            r.throttle <= 0.001 && r.brake <= 0.001 && r.steer.abs() <= 0.05 && r.speed >= 2.0
        })
        .collect();
    let (ax, _) = mean_of(&coast, |r| r.ax_g);
    s.push_str(&format!(
        "coasting_straight_above_2mps: rows={} mean_ax_g={}
",
        coast.len(),
        f(ax, 4)
    ));
    s
}
