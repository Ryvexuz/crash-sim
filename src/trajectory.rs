//! Trajectory files for the renderer: the v1 cube CSV and the v2 body format.
//!
//! The renderer (`projects/render/src/replay.rs`) is a separate project that meets crash-sim only through
//! this text. v2 is:
//!
//! ```text
//! trajectory v2
//! nodes,N                  node count of every frame
//! beam,a,b                 one line per beam, in `Sim::beams` index order (`break` rows use that index)
//! wall,x,6                 zero or more planes `AXIS = OFFSET`, AXIS one of x, y, z
//! t,node,x,y,z
//! <t>,<node>,<x>,<y>,<z>   N rows per frame, nodes in order
//! break,<t>,<beam>         a beam broken from time t on; only between whole frames
//! ```
//!
//! v1 (no `--body`) is the bare `t,node,x,y,z` header and the cube's 8 nodes per frame, unchanged.

use crate::format::{Body, Plane};
use crate::vec3::Vec3;
use crate::{Beam, Config, Sim};

/// One wall of a v2 file: the plane whose `axis` coordinate (0 = x, 1 = y, 2 = z) equals `offset`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Wall {
    pub axis: usize,
    pub offset: f64,
}

/// The first line of a v2 file.
pub const HEADER_V2: &str = "trajectory v2";
/// The column line that ends the header (and is the whole header of a v1 file).
pub const COLUMNS: &str = "t,node,x,y,z";

/// The walls a body's collider planes make. A plane is a wall only when its normal points along one
/// axis (the other two components exactly 0, the one component finite and non-zero) and its point is
/// finite; the offset is the point's coordinate on that axis. Any other plane is refused, so a file
/// never shows a wall in the wrong place.
pub fn walls_from_planes(planes: &[Plane]) -> Result<Vec<Wall>, String> {
    let mut walls = Vec::new();
    for (i, p) in planes.iter().enumerate() {
        let n = [p.normal.x, p.normal.y, p.normal.z];
        let q = [p.point.x, p.point.y, p.point.z];
        if !n.iter().chain(q.iter()).all(|v| v.is_finite()) {
            return Err(format!("plane {i}: a non-finite number"));
        }
        let axes: Vec<usize> = (0..3).filter(|&a| n[a] != 0.0).collect();
        if axes.len() != 1 {
            return Err(format!("plane {i}: normal ({}, {}, {}) is not along the x, y or z axis, so it cannot be written as a wall", n[0], n[1], n[2]));
        }
        walls.push(Wall {
            axis: axes[0],
            offset: q[axes[0]],
        });
    }
    Ok(walls)
}

/// The v2 header: the first line through the column line. Refuses a body with no node, a beam that names
/// a missing node or joins a node to itself, a beam listed twice, or a wall with a non-finite offset, so
/// that the renderer never has to.
pub fn header_v2(node_count: usize, beams: &[Beam], walls: &[Wall]) -> Result<String, String> {
    if node_count == 0 {
        return Err("the body has no nodes".to_string());
    }
    let mut seen = std::collections::HashSet::new();
    let mut s = format!("{HEADER_V2}\nnodes,{node_count}\n");
    for (i, b) in beams.iter().enumerate() {
        if b.a >= node_count || b.b >= node_count {
            return Err(format!(
                "beam {i} names node {}, but the body has {node_count} nodes",
                b.a.max(b.b)
            ));
        }
        if b.a == b.b {
            return Err(format!("beam {i} joins node {} to itself", b.a));
        }
        if !seen.insert((b.a.min(b.b), b.a.max(b.b))) {
            return Err(format!("beam {i} ({},{}) is listed twice", b.a, b.b));
        }
        s.push_str(&format!("beam,{},{}\n", b.a, b.b));
    }
    for (i, w) in walls.iter().enumerate() {
        if w.axis > 2 || !w.offset.is_finite() {
            return Err(format!(
                "wall {i}: axis {} offset {} is not valid",
                w.axis, w.offset
            ));
        }
        s.push_str(&format!("wall,{},{}\n", ["x", "y", "z"][w.axis], w.offset));
    }
    s.push_str(COLUMNS);
    s.push('\n');
    Ok(s)
}

/// The rows of one frame: `t,node,x,y,z` per node, 4 decimals.
pub fn frame_rows(t: f64, pos: &[Vec3]) -> String {
    let mut s = String::new();
    for (n, p) in pos.iter().enumerate() {
        s.push_str(&format!("{t:.4},{n},{:.4},{:.4},{:.4}\n", p.x, p.y, p.z));
    }
    s
}

/// One break event row: beam `beam` is broken from time `t` on.
pub fn break_row(t: f64, beam: usize) -> String {
    format!("break,{t:.4},{beam}\n")
}

/// Checks the stride (a whole number of at least 1 is the only valid one).
fn check_stride(stride: u64) -> Result<(), String> {
    if stride < 1 {
        return Err("stride must be a whole number >= 1".to_string());
    }
    Ok(())
}

/// The v1 text of the built-in cube run: every `stride`-th output step, 8 nodes per frame.
pub fn cube_text(cfg: &Config, stride: u64) -> Result<String, String> {
    check_stride(stride)?;
    cfg.validate()?;
    let steps = crate::step_count(cfg);
    let mut sim = Sim::new(cfg.clone());
    let mut s = format!("{COLUMNS}\n");
    for i in 0..=steps {
        if i > 0 {
            sim.advance();
        }
        if i % stride == 0 {
            s.push_str(&frame_rows(i as f64 * cfg.dt, &sim.pos));
        }
    }
    Ok(s)
}

/// The v2 text of a run of `body`: the header, then every `stride`-th output step as a frame, with the
/// break events that happened up to a frame written just before it (and any left over after the last
/// frame). Refuses (Err) what `Sim::from_body` refuses, a bad stride, a plane that is not a wall, and a
/// run that aborts (`Sim::check` fails), so a half-run is never written as if it were whole.
pub fn body_text(cfg: &Config, body: &Body, stride: u64) -> Result<String, String> {
    check_stride(stride)?;
    let walls = walls_from_planes(&body.planes)?;
    let mut sim = Sim::from_body(cfg.clone(), body)?;
    let steps = crate::step_count(cfg);
    let mut s = header_v2(sim.pos.len(), &sim.beams, &walls)?;
    let mut written = 0;
    for i in 0..=steps {
        if i > 0 {
            sim.advance();
            sim.check()
                .map_err(|e| format!("the run aborted at t={:.4}: {e}", i as f64 * cfg.dt))?;
        }
        if i % stride == 0 {
            for b in &sim.breaks[written..] {
                s.push_str(&break_row(b.t, b.beam));
            }
            written = sim.breaks.len();
            s.push_str(&frame_rows(i as f64 * cfg.dt, &sim.pos));
        }
    }
    for b in &sim.breaks[written..] {
        s.push_str(&break_row(b.t, b.beam));
    }
    Ok(s)
}
