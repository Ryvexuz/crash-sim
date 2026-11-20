//! Trajectory files (nb-37, crash-sim half): the v2 writer is checked against an independent minimal
//! parser written here from the format's description, never against the render crate; the v1 cube text
//! and a v2 run are checked against committed fixtures that render keeps byte copies of.

use crash_sim::format::{parse, Plane};
use crash_sim::trajectory::{body_text, cube_text, header_v2, walls_from_planes, Wall};
use crash_sim::vec3::Vec3;
use crash_sim::{Beam, Config, Sim};

const K: f64 = 20_000.0;
const C: f64 = 20.0;

/// A triangle in the air. Beam 0 (nodes 0-1) breaks at strain 0.01 and is pulled apart at 5 m/s relative
/// speed, so it must break; beams 1 and 2 never break.
const TRIANGLE: &str = "bsn 1\n\
material weak stiffness 20000 damping 20 break_strain 0.01\n\
material strong stiffness 20000 damping 20\n\
node 0 0 2 0 mass 1\nnode 1 1 2 0 mass 1\nnode 2 0.5 2.8 0 mass 1\n\
beam 0 1 weak\nbeam 1 2 strong\nbeam 2 0 strong\n\
group left 0\ngroup right 1\nvelocity left -2.5 0 0\nvelocity right 2.5 0 0\n";

fn cfg() -> Config {
    Config {
        duration: 0.3,
        ..Config::default()
    }
}

/// What the independent parser reads from a v2 file.
#[derive(Debug)]
struct Parsed {
    nodes: usize,
    beams: Vec<(usize, usize)>,
    walls: Vec<(String, f64)>,
    frames: Vec<(f64, Vec<[f64; 3]>)>,
    /// `(time, beam, number of frames complete before the row)`
    breaks: Vec<(f64, usize, usize)>,
}

/// A minimal v2 parser from the format description: first line `trajectory v2`, then `nodes,N`, then
/// `beam,a,b` lines, then optional `wall,AXIS,OFFSET` lines, then `t,node,x,y,z`, then frames of exactly N
/// rows in node order and `break,t,beam` rows only between whole frames. It refuses anything else.
fn parse_v2(text: &str) -> Result<Parsed, String> {
    let mut lines = text.lines();
    if lines.next() != Some("trajectory v2") {
        return Err("first line is not `trajectory v2`".into());
    }
    let nodes: usize = lines
        .next()
        .and_then(|l| l.strip_prefix("nodes,"))
        .and_then(|n| n.parse().ok())
        .filter(|&n| n >= 1)
        .ok_or("second line is not `nodes,N`")?;
    let (mut beams, mut walls) = (Vec::new(), Vec::new());
    loop {
        let line = lines.next().ok_or("header never reaches the column line")?;
        let f: Vec<&str> = line.split(',').collect();
        match f[0] {
            "t" => {
                if line != "t,node,x,y,z" {
                    return Err(format!("bad column line `{line}`"));
                }
                break;
            }
            "beam" if walls.is_empty() => {
                if f.len() != 3 {
                    return Err(format!("bad beam line `{line}`"));
                }
                let (a, b): (usize, usize) = (
                    f[1].parse().map_err(|_| "beam a")?,
                    f[2].parse().map_err(|_| "beam b")?,
                );
                if a >= nodes || b >= nodes || a == b {
                    return Err(format!("beam {a},{b} does not join two nodes of {nodes}"));
                }
                if beams
                    .iter()
                    .any(|&(x, y)| (x, y) == (a, b) || (x, y) == (b, a))
                {
                    return Err(format!("beam {a},{b} twice"));
                }
                beams.push((a, b));
            }
            "wall" => {
                if f.len() != 3 || !["x", "y", "z"].contains(&f[1]) {
                    return Err(format!("bad wall line `{line}`"));
                }
                walls.push((
                    f[1].to_string(),
                    f[2].parse::<f64>().map_err(|_| "wall offset")?,
                ));
            }
            _ => return Err(format!("unexpected header line `{line}`")),
        }
    }
    let mut frames: Vec<(f64, Vec<[f64; 3]>)> = Vec::new();
    let mut breaks: Vec<(f64, usize, usize)> = Vec::new();
    let mut open: Vec<[f64; 3]> = Vec::new();
    let mut open_t = 0.0;
    for line in lines {
        let f: Vec<&str> = line.split(',').collect();
        if f[0] == "break" {
            if !open.is_empty() {
                return Err("break row inside a frame".into());
            }
            if f.len() != 3 {
                return Err(format!("bad break row `{line}`"));
            }
            let (t, b): (f64, usize) = (
                f[1].parse().map_err(|_| "break t")?,
                f[2].parse().map_err(|_| "break beam")?,
            );
            if b >= beams.len() || breaks.iter().any(|x| x.1 == b) {
                return Err(format!("bad break row `{line}`"));
            }
            if breaks.last().is_some_and(|x| t < x.0) {
                return Err("break times go backwards".into());
            }
            breaks.push((t, b, frames.len()));
            continue;
        }
        if f.len() != 5 {
            return Err(format!("bad row `{line}`"));
        }
        let t: f64 = f[0].parse().map_err(|_| "t")?;
        let n: usize = f[1].parse().map_err(|_| "node")?;
        if n != open.len() {
            return Err(format!("node {n} out of order, expected {}", open.len()));
        }
        if n == 0 {
            open_t = t;
        } else if t != open_t {
            return Err("time changes inside a frame".into());
        }
        let p = [f[2], f[3], f[4]].map(|v| v.parse::<f64>().unwrap());
        open.push(p);
        if open.len() == nodes {
            if frames.last().is_some_and(|x| t < x.0) {
                return Err("frame time goes backwards".into());
            }
            frames.push((open_t, std::mem::take(&mut open)));
        }
    }
    if !open.is_empty() || frames.is_empty() {
        return Err("no frames, or the last frame is cut short".into());
    }
    Ok(Parsed {
        nodes,
        beams,
        walls,
        frames,
        breaks,
    })
}

fn run_text() -> String {
    body_text(&cfg(), &parse(TRIANGLE).unwrap(), 1).unwrap()
}

#[test]
fn v2_round_trips_through_an_independent_parser() {
    let text = run_text();
    let p = parse_v2(&text).expect("the writer's output must parse");
    let sim = Sim::from_body(cfg(), &parse(TRIANGLE).unwrap()).unwrap();
    assert_eq!(p.nodes, 3);
    let want: Vec<(usize, usize)> = sim.beams.iter().map(|b| (b.a, b.b)).collect();
    assert_eq!(p.beams, want, "beam lines are in Sim::beams index order");
    assert_eq!(p.beams.len(), 3);
    assert!(p.walls.is_empty());
    // t = 0, 0.01 ... 0.30
    assert_eq!(p.frames.len(), 31);
    assert!((p.frames[30].0 - 0.30).abs() < 1e-9);
    // Every frame's positions are the sim's, to the 4 decimals the file keeps.
    let mut sim = sim;
    for (i, (t, pos)) in p.frames.iter().enumerate() {
        if i > 0 {
            sim.advance();
        }
        assert!((t - i as f64 * 0.01).abs() < 1e-9, "frame {i} time {t}");
        for (n, q) in pos.iter().enumerate() {
            let s = sim.pos[n];
            assert!(
                (q[0] - s.x).abs() < 6e-5 && (q[1] - s.y).abs() < 6e-5 && (q[2] - s.z).abs() < 6e-5,
                "frame {i} node {n}"
            );
        }
    }
    // Exactly one break, of beam 0, which the sim also records, written before the first frame that is at
    // or after its time and after every frame before it.
    assert_eq!(p.breaks.len(), 1, "{:?}", p.breaks);
    let (bt, beam, frames_before) = p.breaks[0];
    assert_eq!(beam, 0);
    assert_eq!(sim.breaks.len(), 1);
    assert_eq!(sim.breaks[0].beam, 0);
    assert!(
        (bt - sim.breaks[0].t).abs() < 5.1e-5,
        "{bt} vs {}",
        sim.breaks[0].t
    );
    assert!(
        bt < 0.0074 + 1e-3,
        "the closed-form peak is at 0.0074 s, broke at {bt}"
    );
    assert!(
        frames_before >= 1 && p.frames[frames_before - 1].0 < bt,
        "frame before the event is earlier than it"
    );
    assert!(
        p.frames[frames_before].0 >= bt - 5.1e-5,
        "the next frame is not earlier than the event"
    );
}

#[test]
fn v2_stride_keeps_breaks_before_the_frame_that_shows_them() {
    let body = parse(TRIANGLE).unwrap();
    // Stride 7: frames at 0, 0.07, 0.14 ... ; the break (about 0.007 s) lands before the 0.07 frame.
    let p = parse_v2(&body_text(&cfg(), &body, 7).unwrap()).unwrap();
    assert_eq!(p.frames.len(), 5);
    assert_eq!(p.breaks.len(), 1);
    assert_eq!(
        p.breaks[0].2, 1,
        "one whole frame (t=0) comes before the break row"
    );
}

#[test]
fn v2_writes_nothing_for_a_bad_stride_or_a_body_the_sim_refuses() {
    let body = parse(TRIANGLE).unwrap();
    assert!(body_text(&cfg(), &body, 0).unwrap_err().contains("stride"));
    assert!(cube_text(&cfg(), 0).unwrap_err().contains("stride"));
    // A plane makes Sim::from_body refuse until planes are wired; the writer passes the refusal on.
    let with_plane = format!("{TRIANGLE}plane 6 0 0 -1 0 0\n");
    let body = parse(&with_plane).unwrap();
    let err = body_text(&cfg(), &body, 1).unwrap_err();
    assert!(err.contains("plane"), "{err}");
}

#[test]
fn header_refuses_beams_the_renderer_would_refuse() {
    let beam = |a, b| Beam {
        a,
        b,
        rest: 1.0,
        stiffness: K,
        damping: C,
    };
    assert!(header_v2(3, &[beam(0, 3)], &[])
        .unwrap_err()
        .contains("names node 3"));
    assert!(header_v2(3, &[beam(1, 1)], &[])
        .unwrap_err()
        .contains("itself"));
    assert!(header_v2(3, &[beam(0, 1), beam(1, 0)], &[])
        .unwrap_err()
        .contains("twice"));
    assert!(header_v2(0, &[], &[]).unwrap_err().contains("no nodes"));
    assert!(header_v2(
        2,
        &[],
        &[Wall {
            axis: 0,
            offset: f64::NAN
        }]
    )
    .is_err());
    assert!(header_v2(
        2,
        &[],
        &[Wall {
            axis: 3,
            offset: 1.0
        }]
    )
    .is_err());
}

#[test]
fn walls_come_from_axis_aligned_planes_only() {
    let plane = |p: (f64, f64, f64), n: (f64, f64, f64)| Plane {
        point: Vec3::new(p.0, p.1, p.2),
        normal: Vec3::new(n.0, n.1, n.2),
    };
    let walls = walls_from_planes(&[
        plane((6.0, 0.0, 0.0), (-1.0, 0.0, 0.0)),
        plane((0.0, 0.0, -2.5), (0.0, 0.0, 1.0)),
    ])
    .unwrap();
    assert_eq!(
        walls,
        vec![
            Wall {
                axis: 0,
                offset: 6.0
            },
            Wall {
                axis: 2,
                offset: -2.5
            }
        ]
    );
    assert!(
        walls_from_planes(&[plane((0.0, 0.0, 0.0), (1.0, 1.0, 0.0))])
            .unwrap_err()
            .contains("axis")
    );
    assert!(walls_from_planes(&[plane((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))]).is_err());
    assert!(walls_from_planes(&[plane((f64::INFINITY, 0.0, 0.0), (1.0, 0.0, 0.0))]).is_err());
    // The header carries them, and the independent parser reads them back.
    let h = header_v2(1, &[], &walls).unwrap();
    assert_eq!(
        h,
        "trajectory v2\nnodes,1\nwall,x,6\nwall,z,-2.5\nt,node,x,y,z\n"
    );
    let p = parse_v2(&format!("{h}0.0000,0,0,0,0\n")).unwrap();
    assert_eq!(
        p.walls,
        vec![("x".to_string(), 6.0), ("z".to_string(), -2.5)]
    );
}

#[test]
fn the_independent_parser_can_fail() {
    let good = run_text();
    assert!(parse_v2(&good).is_ok());
    // A break row inside a frame, a renumbered beam in a break row, and a cut-off last frame are refused.
    let lines: Vec<&str> = good.lines().collect();
    let mut inside = lines.clone();
    let at = lines
        .iter()
        .position(|l| l.starts_with("0.0000,1,"))
        .unwrap();
    inside.insert(at, "break,0.0000,0");
    assert!(parse_v2(&inside.join("\n"))
        .unwrap_err()
        .contains("inside a frame"));
    assert!(parse_v2(&good.replace("break,", "break,0.5,9,")).is_err());
    let cut: Vec<&str> = lines[..lines.len() - 1].to_vec();
    assert!(parse_v2(&cut.join("\n")).unwrap_err().contains("cut short"));
}

fn lf(s: &str) -> String {
    s.replace("\r\n", "\n")
}

/// The committed v1 fixture is what the cube run writes today (stride 10, the default 5 s run). Render's
/// copy of this file is checked against it byte for byte.
#[test]
fn v1_cube_text_matches_the_committed_fixture() {
    let want = lf(&std::fs::read_to_string("tests/fixtures/trajectory.csv").unwrap());
    let got = cube_text(&Config::default(), 10).unwrap();
    assert_eq!(got.lines().count(), 409);
    assert_eq!(got, want);
}

/// The committed v2 fixture is what the triangle run writes (stride 3). To regenerate it after a change
/// on purpose: run `body_text(&cfg(), &parse(TRIANGLE)?, 3)` and write it to the fixture file.
#[test]
fn v2_triangle_text_matches_the_committed_fixture() {
    let want = lf(&std::fs::read_to_string("tests/fixtures/trajectory_v2.csv").unwrap());
    let got = body_text(&cfg(), &parse(TRIANGLE).unwrap(), 3).unwrap();
    assert!(!want.is_empty());
    assert_eq!(got, want);
    assert!(parse_v2(&want).is_ok());
}
