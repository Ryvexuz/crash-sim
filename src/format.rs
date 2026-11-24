//! The `.bsn` text format (versions 1, 2 and 3): a node-beam body, from version 2 a car on
//! it (wheels, suspension, tyres, controls), and from version 3 solids and the tetrahedra made
//! of them (docs/M5-SOLVER.md section 9), as data. The grammar is in docs/FORMAT.md. The
//! parser is strict: every malformed input is a `ParseError` naming the line (and the column
//! when a token is at fault), never a silent default.
//!
//! `write(&parse(x)?)` is byte-identical to `x` for a canonical file (one `write` produced);
//! `parse(&write(b)) == b` for every body `parse` can produce.

use crate::vec3::Vec3;
use crate::Beam;
use std::collections::{HashMap, HashSet};
use std::fmt;

/// The newest version this parser reads. Every older version still loads: version 2 only adds
/// the car sections and version 3 only the `solid` and `tet` lines, so one parser reads all
/// three, and a file may not use a line newer than its version (docs/FORMAT.md, "Versions").
pub const VERSION: u32 = 3;

/// The smallest rest volume (m^3) a `tet` may have: 1 mm^3 (inclusive). A tet below it is
/// refused by name, which also refuses four coplanar or collinear nodes (volume 0).
pub const TET_MIN_VOLUME: f64 = 1e-9;

/// The smallest shape ratio a `tet` may have: its rest volume over the cube of its longest
/// edge. A regular tet has 0.118 and each of the 6 tets of a cube cut along its diagonal 0.032;
/// below this a tet is a sliver, which also catches four nodes that are coplanar in decimal
/// but whose rounded coordinates leave a volume above `TET_MIN_VOLUME` at a large size.
pub const TET_MIN_SHAPE: f64 = 1e-6;

/// The most `solid` lines a body may have. A solid is the material of one zone; m5 uses one
/// (`crush_solid`, docs/M5-SOLVER.md section 8), and 1 000 leaves room for a body with a solid
/// per part while bounding a hostile file. The 1 001st `solid` line is refused by name.
pub const MAX_SOLIDS: usize = 1000;

/// The most `tet` lines a body may have. The m5 solver takes at most `m5_max_tets` = 300
/// (docs/M5-SOLVER.md section 8), but that is the solver's cap, not the format's: the format
/// should also hold a finer crush zone for a mesh study, or a whole car in tets (about 60 000
/// at 10 cm cells, 480 000 at 5 cm), and a 1e6-tet body that the solver itself must refuse
/// (n20-11 R10). At the cap a parse holds of the order of 100 MB of tet records (an estimate
/// from the record sizes, not measured), which bounds a hostile file.
/// The 1 000 001st `tet` line is refused by name.
pub const MAX_TETS: usize = 1_000_000;

/// Why a file was refused. `line` is 1-based; `column` is the 1-based character column of the
/// offending token when there is one.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ParseError {
    pub line: usize,
    pub column: Option<usize>,
    pub message: String,
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.column {
            Some(c) => write!(f, "line {}, column {}: {}", self.line, c, self.message),
            None => write!(f, "line {}: {}", self.line, self.message),
        }
    }
}

impl std::error::Error for ParseError {}

#[derive(Clone, Debug, PartialEq)]
pub struct Material {
    pub name: String,
    /// Spring stiffness (N/m), > 0.
    pub stiffness: f64,
    /// Axial damping (N*s/m), >= 0.
    pub damping: f64,
    /// Strain beyond which a beam deforms permanently (used from nb-34 on).
    pub yield_strain: Option<f64>,
    /// Strain beyond which a beam breaks (used from nb-35 on).
    pub break_strain: Option<f64>,
    /// Fraction of stored elastic energy returned on unloading, finite and in (0, 1].
    pub unload_return: Option<f64>,
}

/// A solid (version 3): the material of a set of tets. Units in docs/FORMAT.md; the law that
/// uses them is m5's (docs/M5-SOLVER.md section 8). `Body::build` and `Sim` do not read it.
#[derive(Clone, Debug, PartialEq)]
pub struct Solid {
    pub name: String,
    /// Crush stress of the solid at zero compaction (Pa), > 0.
    pub plateau_stress: f64,
    /// Elastic strain at which the solid starts crushing (1), > 0 and < 1.
    pub yield_strain: f64,
    /// Damping ratio (1), >= 0 and <= 1.
    pub damping_ratio: f64,
}

/// A linear tetrahedron (version 3) on four nodes of the body. Its rest shape is the nodes'
/// positions in the file, oriented so that its rest volume
/// `(b - a) . ((c - a) x (d - a)) / 6` is positive. `Body::build` and `Sim` do not read it.
#[derive(Clone, Debug, PartialEq)]
pub struct Tet {
    /// Node ids, in file order: four distinct declared nodes.
    pub nodes: [u32; 4],
    /// Name of a solid of the body.
    pub solid: String,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Node {
    pub id: u32,
    pub pos: Vec3,
    /// kg, > 0.
    pub mass: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct BodyBeam {
    /// Node ids.
    pub a: u32,
    pub b: u32,
    /// Name of a material of the body.
    pub material: String,
    /// Rest length (m), > 0. The distance between the two nodes unless the file gave it.
    pub rest: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Group {
    pub name: String,
    /// Node ids, in file order, at least one, no repeats.
    pub nodes: Vec<u32>,
}

/// Start velocity (m/s) of every node of a group.
#[derive(Clone, Debug, PartialEq)]
pub struct Velocity {
    pub group: String,
    pub velocity: Vec3,
}

/// A collider plane: the side the normal points to is free space.
#[derive(Clone, Debug, PartialEq)]
pub struct Plane {
    pub point: Vec3,
    pub normal: Vec3,
}

/// A wheel (version 2). It hangs from the centroid of its mount nodes: at the start its centre
/// is that centroid minus its suspension's rest length along y (`Body::wheel_start_center`).
#[derive(Clone, Debug, PartialEq)]
pub struct Wheel {
    pub name: String,
    /// Chassis node ids the suspension acts on, in file order, at least one, no repeats.
    pub mount: Vec<u32>,
    /// Tyre outer radius (m), > 0.
    pub radius: f64,
    /// Tyre width (m), > 0.
    pub width: f64,
    /// Unsprung mass (kg), > 0.
    pub mass: f64,
    /// Moment of inertia about the axle (kg m^2), > 0.
    pub inertia: f64,
    pub steerable: bool,
    pub driven: bool,
    pub braked: bool,
}

/// The spring-damper between a wheel centre and its mount point, along the chassis' up axis.
/// Lengths are mount point to wheel centre (m).
#[derive(Clone, Debug, PartialEq)]
pub struct Suspension {
    pub wheel: String,
    /// Spring rate (N/m), > 0.
    pub spring: f64,
    /// Damper rate (N s/m), >= 0.
    pub damper: f64,
    /// Length at which the spring carries no force, and the length at the start (m).
    pub rest: f64,
    /// Shortest length (bump stop), >= 0 (m).
    pub travel_min: f64,
    /// Longest length (droop stop) (m). travel_min < rest < travel_max.
    pub travel_max: f64,
}

/// A wheel's tyre. Units in docs/FORMAT.md; the model that uses them is nb-39's
/// (docs/PHYSICS.md).
#[derive(Clone, Debug, PartialEq)]
pub struct Tyre {
    pub wheel: String,
    /// Friction coefficient: the largest horizontal force is mu times the normal load. > 0.
    pub mu: f64,
    /// Radial (vertical) spring rate (N/m), > 0.
    pub radial_stiffness: f64,
    /// Radial damping (N s/m), >= 0.
    pub radial_damping: f64,
    /// Longitudinal force per unit normal load per unit slip ratio (1), > 0.
    pub long_stiffness: f64,
    /// Lateral force per unit normal load per radian of slip angle (1/rad), > 0.
    pub lat_stiffness: f64,
}

/// What full inputs mean. Torques are totals over the driven (braked) wheels.
#[derive(Clone, Debug, PartialEq)]
pub struct Controls {
    /// Road-wheel angle of a steerable wheel at full steer (degrees), 0 < a < 90.
    pub max_steer_deg: f64,
    /// Total drive torque at full throttle (N m), > 0.
    pub max_drive_torque: f64,
    /// Total brake torque at full brake (N m), > 0.
    pub max_brake_torque: f64,
    /// Rolling resistance coefficient c_rr (1: newtons of rolling resistance per newton of
    /// normal load), >= 0, from the optional `rolling_resistance` key. None when the line has
    /// no such key, which the vehicle code reads as 0 (no rolling resistance, as before the key
    /// existed).
    pub rolling_resistance: Option<f64>,
    /// Aero drag area CdA (m^2: drag coefficient times frontal area), >= 0, from the optional
    /// `drag_area` key. None when the line has no such key, which the vehicle code reads as 0.
    pub drag_area: Option<f64>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Body {
    pub materials: Vec<Material>,
    /// Version 3: in file order, names unique.
    pub solids: Vec<Solid>,
    pub nodes: Vec<Node>,
    pub beams: Vec<BodyBeam>,
    /// Version 3: in file order, no two on the same set of nodes.
    pub tets: Vec<Tet>,
    pub groups: Vec<Group>,
    pub velocities: Vec<Velocity>,
    /// Version 2: in file order. Every wheel has exactly one suspension and one tyre, and a
    /// body with wheels has `controls`; a body without wheels has none of the four.
    pub wheels: Vec<Wheel>,
    pub suspensions: Vec<Suspension>,
    pub tyres: Vec<Tyre>,
    pub controls: Option<Controls>,
    pub planes: Vec<Plane>,
}

/// A body turned into the arrays a `Sim` is made of, one entry per node in file order.
#[derive(Clone, Debug)]
pub struct Built {
    pub pos: Vec<Vec3>,
    pub vel: Vec<Vec3>,
    pub beams: Vec<Beam>,
    pub mass: Vec<f64>,
}

impl Body {
    /// Index (file order) of the node with this id.
    pub fn node_index(&self, id: u32) -> Option<usize> {
        self.nodes.iter().position(|n| n.id == id)
    }

    /// The suspension of the wheel with this name.
    pub fn suspension(&self, wheel: &str) -> Option<&Suspension> {
        self.suspensions.iter().find(|s| s.wheel == wheel)
    }

    /// The tyre of the wheel with this name.
    pub fn tyre(&self, wheel: &str) -> Option<&Tyre> {
        self.tyres.iter().find(|t| t.wheel == wheel)
    }

    /// Where the named wheel's centre is at the start: the centroid of its mount nodes minus
    /// its suspension's rest length along y. None if there is no such wheel, or it has no
    /// suspension or a mount node is missing (a body `parse` produced always has them).
    pub fn wheel_start_center(&self, wheel: &str) -> Option<Vec3> {
        let w = self.wheels.iter().find(|w| w.name == wheel)?;
        let s = self.suspension(wheel)?;
        let mut sum = Vec3::ZERO;
        for id in &w.mount {
            sum += self.nodes[self.node_index(*id)?].pos;
        }
        let c = sum * (1.0 / w.mount.len() as f64);
        Some(Vec3::new(c.x, c.y - s.rest, c.z))
    }

    /// Positions, start velocities, beams (stiffness and damping from the material) and
    /// masses, node indices in file order. The chassis only: wheels are built by the vehicle
    /// code (nb-39). Solids and tets are not built (they add no mass and no beam).
    pub fn build(&self) -> Built {
        let idx: HashMap<u32, usize> = self.nodes.iter().enumerate().map(|(i, n)| (n.id, i)).collect();
        let mut vel = vec![Vec3::ZERO; self.nodes.len()];
        for v in &self.velocities {
            let g = self.groups.iter().find(|g| g.name == v.group).expect("velocity names an existing group");
            for id in &g.nodes {
                vel[idx[id]] = v.velocity;
            }
        }
        let beams = self
            .beams
            .iter()
            .map(|b| {
                let m = self.materials.iter().find(|m| m.name == b.material).expect("beam names an existing material");
                Beam { a: idx[&b.a], b: idx[&b.b], rest: b.rest, stiffness: m.stiffness, damping: m.damping }
            })
            .collect();
        Built {
            pos: self.nodes.iter().map(|n| n.pos).collect(),
            vel,
            beams,
            mass: self.nodes.iter().map(|n| n.mass).collect(),
        }
    }
}

// ---------------------------------------------------------------------------------------
// Parsing

/// The tokens of one line: (1-based column, text).
struct Toks<'a> {
    line: usize,
    items: Vec<(usize, &'a str)>,
    pos: usize,
    end_col: usize,
}

impl<'a> Toks<'a> {
    fn err(&self, col: Option<usize>, message: String) -> ParseError {
        ParseError { line: self.line, column: col, message }
    }

    fn next(&mut self, what: &str) -> Result<(usize, &'a str), ParseError> {
        match self.items.get(self.pos) {
            Some(&t) => {
                self.pos += 1;
                Ok(t)
            }
            None => Err(self.err(Some(self.end_col), format!("missing {what}"))),
        }
    }

    fn peek(&self) -> Option<(usize, &'a str)> {
        self.items.get(self.pos).copied()
    }

    fn keyword(&mut self, kw: &str) -> Result<(), ParseError> {
        let (c, t) = self.next(&format!("`{kw}`"))?;
        if t != kw {
            return Err(self.err(Some(c), format!("expected `{kw}`, found `{t}`")));
        }
        Ok(())
    }

    fn number(&mut self, what: &str) -> Result<(usize, f64), ParseError> {
        let (c, t) = self.next(what)?;
        let v: f64 = t.parse().map_err(|_| self.err(Some(c), format!("{what} `{t}` is not a number")))?;
        if !v.is_finite() {
            return Err(self.err(Some(c), format!("{what} `{t}` is not finite")));
        }
        Ok((c, v))
    }

    /// `<kw> <number>` where the number must be > 0 (`what` names it in errors).
    fn positive_field(&mut self, kw: &str, what: &str) -> Result<(usize, f64), ParseError> {
        self.keyword(kw)?;
        let (c, v) = self.number(what)?;
        positive(self, c, what, v)?;
        Ok((c, v))
    }

    /// `<kw> <number>` where the number must be >= 0.
    fn nonnegative_field(&mut self, kw: &str, what: &str) -> Result<(usize, f64), ParseError> {
        self.keyword(kw)?;
        let (c, v) = self.number(what)?;
        if v < 0.0 {
            return Err(self.err(Some(c), format!("{what} {v} must be >= 0")));
        }
        Ok((c, v))
    }

    /// `<kw> yes|no`.
    fn flag(&mut self, kw: &str) -> Result<bool, ParseError> {
        self.keyword(kw)?;
        let (c, t) = self.next(kw)?;
        match t {
            "yes" => Ok(true),
            "no" => Ok(false),
            _ => Err(self.err(Some(c), format!("{kw} `{t}` must be `yes` or `no`"))),
        }
    }

    fn id(&mut self, what: &str) -> Result<(usize, u32), ParseError> {
        let (c, t) = self.next(what)?;
        parse_id(t).map(|v| (c, v)).ok_or_else(|| {
            self.err(Some(c), format!("{what} `{t}` is not a node id (decimal digits, no sign, no leading zero, at most {})", u32::MAX))
        })
    }

    fn name(&mut self, what: &str) -> Result<(usize, &'a str), ParseError> {
        let (c, t) = self.next(what)?;
        if !is_name(t) {
            return Err(self.err(Some(c), format!("{what} `{t}` is not a name (letters, digits and _, starting with a letter or _)")));
        }
        Ok((c, t))
    }

    fn vec3(&mut self, what: &str) -> Result<Vec3, ParseError> {
        let (_, x) = self.number(&format!("{what} x"))?;
        let (_, y) = self.number(&format!("{what} y"))?;
        let (_, z) = self.number(&format!("{what} z"))?;
        Ok(Vec3::new(x, y, z))
    }

    fn done(&self) -> Result<(), ParseError> {
        match self.peek() {
            None => Ok(()),
            Some((c, t)) => Err(self.err(Some(c), format!("unexpected `{t}` at the end of the line"))),
        }
    }
}

fn parse_id(t: &str) -> Option<u32> {
    let digits = !t.is_empty() && t.bytes().all(|b| b.is_ascii_digit());
    if !digits || (t.len() > 1 && t.starts_with('0')) {
        return None;
    }
    t.parse().ok()
}

fn is_name(t: &str) -> bool {
    let mut cs = t.chars();
    match cs.next() {
        Some(c) if c.is_ascii_alphabetic() || c == '_' => {}
        _ => return false,
    }
    cs.all(|c| c.is_ascii_alphanumeric() || c == '_')
}

fn positive(t: &Toks, col: usize, what: &str, v: f64) -> Result<(), ParseError> {
    if v > 0.0 {
        Ok(())
    } else {
        Err(t.err(Some(col), format!("{what} {v} must be > 0")))
    }
}

/// The line keywords in the order they must come.
const SECTIONS: [&str; 12] = [
    "material",
    "solid",
    "node",
    "beam",
    "tet",
    "group",
    "velocity",
    "wheel",
    "suspension",
    "tyre",
    "controls",
    "plane",
];

/// Keywords a version 1 file may not use.
const CAR_SECTIONS: [&str; 4] = ["wheel", "suspension", "tyre", "controls"];

/// Keywords a version 1 or 2 file may not use.
const SOLID_SECTIONS: [&str; 2] = ["solid", "tet"];

fn section_rank(keyword: &str) -> Option<usize> {
    SECTIONS.iter().position(|k| *k == keyword)
}

/// The lowest version whose files may use this keyword.
fn needs_version(keyword: &str) -> u32 {
    if SOLID_SECTIONS.contains(&keyword) {
        3
    } else if CAR_SECTIONS.contains(&keyword) {
        2
    } else {
        1
    }
}

/// Line and name column of each wheel, and the wheels that have a suspension or a tyre, for
/// the checks made at the end of the file.
#[derive(Default)]
struct CarLines {
    wheel_at: Vec<(usize, usize)>,
    suspended: HashSet<String>,
    tyred: HashSet<String>,
}

/// Parse a `.bsn` file. See docs/FORMAT.md for the grammar.
pub fn parse(text: &str) -> Result<Body, ParseError> {
    let err = |line: usize, column: Option<usize>, message: String| ParseError {
        line,
        column,
        message,
    };

    if text.is_empty() {
        return Err(err(
            1,
            None,
            format!("empty file: the first line must be {HEADERS}"),
        ));
    }
    if !text.ends_with('\n') {
        let last = text.split('\n').count();
        return Err(err(last, None, "the file does not end with a newline".into()));
    }
    let body_text = &text[..text.len() - 1];

    let mut body = Body {
        materials: Vec::new(),
        solids: Vec::new(),
        nodes: Vec::new(),
        beams: Vec::new(),
        tets: Vec::new(),
        groups: Vec::new(),
        velocities: Vec::new(),
        wheels: Vec::new(),
        suspensions: Vec::new(),
        tyres: Vec::new(),
        controls: None,
        planes: Vec::new(),
    };
    let mut node_ids: HashMap<u32, usize> = HashMap::new();
    let mut beam_pairs: HashSet<(u32, u32)> = HashSet::new();
    let mut solid_names: HashSet<String> = HashSet::new();
    let mut tet_sets: HashSet<[u32; 4]> = HashSet::new();
    let mut velocity_nodes: HashSet<u32> = HashSet::new();
    let mut car = CarLines::default();
    let mut version = 0u32;
    let mut rank = 0usize;
    let mut last_line = 1usize;

    for (i, line) in body_text.split('\n').enumerate() {
        let n = i + 1;
        last_line = n;
        if n == 1 {
            version = parse_header(line)?;
            continue;
        }
        if let Some(p) = line.find('\r') {
            return Err(err(n, Some(line[..p].chars().count() + 1), "carriage return: the file must use \\n line endings".into()));
        }
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if let Some(p) = line.find('\t') {
            return Err(err(n, Some(line[..p].chars().count() + 1), "tab: tokens are separated by single spaces".into()));
        }
        let mut items = Vec::new();
        let mut col = 1usize;
        for piece in line.split(' ') {
            if piece.is_empty() {
                return Err(err(n, Some(col), "extra space: tokens are separated by exactly one space, with none at the ends of a line".into()));
            }
            items.push((col, piece));
            col += piece.chars().count() + 1;
        }
        let mut t = Toks { line: n, items, pos: 0, end_col: col - 1 };
        let (kc, kw) = t.next("keyword")?;
        let this_rank = match section_rank(kw) {
            Some(r) => r,
            None => {
                return Err(t.err(
                    Some(kc),
                    format!("unknown keyword `{kw}` (expected material, solid, node, beam, tet, group, velocity, wheel, suspension, tyre, controls or plane)"),
                ));
            }
        };
        let needed = needs_version(kw);
        if version < needed {
            return Err(t.err(
                Some(kc),
                format!("`{kw}` needs version {needed}: this file is `bsn {version}`"),
            ));
        }
        if this_rank < rank {
            return Err(t.err(
                Some(kc),
                format!("`{kw}` after a later section: lines come in the order material, solid, node, beam, tet, group, velocity, wheel, suspension, tyre, controls, plane"),
            ));
        }
        rank = this_rank;
        match kw {
            "material" => {
                let (nc, name) = t.name("material name")?;
                if body.materials.iter().any(|m| m.name == name) {
                    return Err(t.err(Some(nc), format!("duplicate material `{name}`")));
                }
                t.keyword("stiffness")?;
                let (c, stiffness) = t.number("stiffness")?;
                positive(&t, c, "stiffness", stiffness)?;
                t.keyword("damping")?;
                let (c, damping) = t.number("damping")?;
                if damping < 0.0 {
                    return Err(t.err(Some(c), format!("damping {damping} must be >= 0")));
                }
                let mut yield_strain = None;
                let mut break_strain = None;
                if t.peek().map(|p| p.1) == Some("yield_strain") {
                    t.keyword("yield_strain")?;
                    let (c, v) = t.number("yield_strain")?;
                    positive(&t, c, "yield_strain", v)?;
                    yield_strain = Some(v);
                }
                if t.peek().map(|p| p.1) == Some("break_strain") {
                    t.keyword("break_strain")?;
                    let (c, v) = t.number("break_strain")?;
                    positive(&t, c, "break_strain", v)?;
                    if let Some(y) = yield_strain {
                        if y >= v {
                            return Err(t.err(Some(c), format!("break_strain {v} must be above yield_strain {y}")));
                        }
                    }
                    break_strain = Some(v);
                }
                let mut unload_return = None;
                if t.peek().map(|p| p.1) == Some("unload_return") {
                    t.keyword("unload_return")?;
                    let (c, v) = t.number("unload_return")?;
                    if !(v > 0.0 && v <= 1.0) {
                        return Err(t.err(Some(c), format!("unload_return {v} must be > 0 and <= 1")));
                    }
                    unload_return = Some(v);
                }
                t.done()?;
                body.materials.push(Material { name: name.to_string(), stiffness, damping, yield_strain, break_strain, unload_return });
            }
            "solid" => {
                if body.solids.len() >= MAX_SOLIDS {
                    return Err(t.err(
                        Some(kc),
                        format!("too many solids: a body has at most {MAX_SOLIDS} (MAX_SOLIDS)"),
                    ));
                }
                let solid = parse_solid(&mut t, &solid_names)?;
                solid_names.insert(solid.name.clone());
                body.solids.push(solid);
            }
            "tet" => {
                if body.tets.len() >= MAX_TETS {
                    return Err(t.err(
                        Some(kc),
                        format!("too many tets: a body has at most {MAX_TETS} (MAX_TETS)"),
                    ));
                }
                let tet = parse_tet(&mut t, &body, &node_ids, &solid_names, &mut tet_sets)?;
                body.tets.push(tet);
            }
            "node" => {
                let (ic, id) = t.id("node id")?;
                if node_ids.contains_key(&id) {
                    return Err(t.err(Some(ic), format!("duplicate node id {id}")));
                }
                let pos = t.vec3("position")?;
                t.keyword("mass")?;
                let (c, mass) = t.number("mass")?;
                positive(&t, c, "mass", mass)?;
                t.done()?;
                node_ids.insert(id, body.nodes.len());
                body.nodes.push(Node { id, pos, mass });
            }
            "beam" => {
                let (ac, a) = t.id("beam node a")?;
                let (bc, b) = t.id("beam node b")?;
                let (mc, mat) = t.name("beam material")?;
                for (c, id) in [(ac, a), (bc, b)] {
                    if !node_ids.contains_key(&id) {
                        return Err(t.err(Some(c), format!("beam to missing node {id}")));
                    }
                }
                if a == b {
                    return Err(t.err(Some(bc), format!("beam from node {a} to itself")));
                }
                if !body.materials.iter().any(|m| m.name == mat) {
                    return Err(t.err(Some(mc), format!("beam uses unknown material `{mat}`")));
                }
                let (pa, pb) = (body.nodes[node_ids[&a]].pos, body.nodes[node_ids[&b]].pos);
                let dist = (pb - pa).length();
                if dist == 0.0 {
                    return Err(t.err(Some(ac), format!("zero-length beam: nodes {a} and {b} are at the same position")));
                }
                let mut rest = dist;
                if t.peek().is_none() && !dist.is_finite() {
                    return Err(t.err(Some(ac), format!("beam between nodes {a} and {b} is too long: its length is not finite, so give an explicit `rest`")));
                }
                if t.peek().is_some() {
                    t.keyword("rest")?;
                    let (c, r) = t.number("rest length")?;
                    positive(&t, c, "rest length", r)?;
                    rest = r;
                }
                t.done()?;
                if !beam_pairs.insert((a.min(b), a.max(b))) {
                    return Err(t.err(Some(ac), format!("duplicate beam between nodes {a} and {b}")));
                }
                body.beams.push(BodyBeam { a, b, material: mat.to_string(), rest });
            }
            "group" => {
                let (nc, name) = t.name("group name")?;
                if body.groups.iter().any(|g| g.name == name) {
                    return Err(t.err(Some(nc), format!("duplicate group `{name}`")));
                }
                let mut nodes: Vec<u32> = Vec::new();
                loop {
                    if nodes.is_empty() || t.peek().is_some() {
                        let (c, id) = t.id("group member")?;
                        if !node_ids.contains_key(&id) {
                            return Err(t.err(Some(c), format!("group `{name}` names missing node {id}")));
                        }
                        if nodes.contains(&id) {
                            return Err(t.err(Some(c), format!("group `{name}` lists node {id} twice")));
                        }
                        nodes.push(id);
                    } else {
                        break;
                    }
                }
                body.groups.push(Group { name: name.to_string(), nodes });
            }
            "velocity" => {
                let (gc, gname) = t.name("group name")?;
                let group = match body.groups.iter().find(|g| g.name == gname) {
                    Some(g) => g,
                    None => return Err(t.err(Some(gc), format!("velocity names unknown group `{gname}`"))),
                };
                if body.velocities.iter().any(|v| v.group == gname) {
                    return Err(t.err(Some(gc), format!("group `{gname}` already has a velocity")));
                }
                let velocity = t.vec3("velocity")?;
                t.done()?;
                for id in &group.nodes {
                    if !velocity_nodes.insert(*id) {
                        return Err(t.err(Some(gc), format!("node {id} would get a second velocity (groups with a velocity must not share nodes)")));
                    }
                }
                body.velocities.push(Velocity { group: gname.to_string(), velocity });
            }
            "wheel" => {
                let wheel = parse_wheel(&mut t, &body, &node_ids)?;
                car.wheel_at.push((n, t.items[1].0));
                body.wheels.push(wheel);
            }
            "suspension" => {
                let s = parse_suspension(&mut t, &body, &mut car)?;
                body.suspensions.push(s);
            }
            "tyre" => {
                let ty = parse_tyre(&mut t, &body, &mut car)?;
                body.tyres.push(ty);
            }
            "controls" => {
                let c = parse_controls(&mut t, &body, kc)?;
                body.controls = Some(c);
            }
            _ => {
                let point = t.vec3("plane point")?;
                let normal = t.vec3("plane normal")?;
                t.done()?;
                if normal.x == 0.0 && normal.y == 0.0 && normal.z == 0.0 {
                    return Err(t.err(None, "plane normal is the zero vector".into()));
                }
                body.planes.push(Plane { point, normal });
            }
        }
    }
    if body.nodes.is_empty() {
        return Err(err(last_line, None, "empty body: the file has no nodes".into()));
    }
    for (w, &(line, col)) in body.wheels.iter().zip(&car.wheel_at) {
        if !car.suspended.contains(&w.name) {
            return Err(err(line, Some(col), format!("wheel `{}` has no `suspension` line", w.name)));
        }
        if !car.tyred.contains(&w.name) {
            return Err(err(line, Some(col), format!("wheel `{}` has no `tyre` line", w.name)));
        }
    }
    if let (Some(&(line, _)), None) = (car.wheel_at.first(), &body.controls) {
        return Err(err(line, Some(1), "the body has wheels but no `controls` line".into()));
    }
    Ok(body)
}

/// `solid <name> plateau_stress <Pa> yield_strain <e> damping_ratio <z>` (version 3), with
/// Pa > 0, 0 < e < 1 and 0 <= z <= 1; `number` refuses NaN and infinities by name.
fn parse_solid(t: &mut Toks, names: &HashSet<String>) -> Result<Solid, ParseError> {
    let (nc, name) = t.name("solid name")?;
    if names.contains(name) {
        return Err(t.err(Some(nc), format!("duplicate solid `{name}`")));
    }
    let (_, plateau_stress) = t.positive_field("plateau_stress", "plateau_stress")?;
    t.keyword("yield_strain")?;
    let (yc, yield_strain) = t.number("yield_strain")?;
    if !(yield_strain > 0.0 && yield_strain < 1.0) {
        return Err(t.err(
            Some(yc),
            format!("yield_strain {yield_strain} must be > 0 and < 1"),
        ));
    }
    t.keyword("damping_ratio")?;
    let (zc, damping_ratio) = t.number("damping_ratio")?;
    if !(0.0..=1.0).contains(&damping_ratio) {
        return Err(t.err(
            Some(zc),
            format!("damping_ratio {damping_ratio} must be >= 0 and <= 1"),
        ));
    }
    t.done()?;
    Ok(Solid {
        name: name.to_string(),
        plateau_stress,
        yield_strain,
        damping_ratio,
    })
}

/// `tet <a> <b> <c> <d> <solid>` (version 3): four distinct nodes declared above, a solid
/// declared above, no other tet on the same set of nodes (in any order), a finite size, a rest
/// volume `(b - a) . ((c - a) x (d - a)) / 6` of at least `TET_MIN_VOLUME` in size, a shape
/// ratio (that volume over the cube of the longest edge) of at least `TET_MIN_SHAPE`, and a
/// positive volume (the orientation). The checks run in that order; the size, volume and
/// shape come from `tet_size`, which cannot overflow when the true values are finite. The
/// `MAX_TETS` cap is checked by `parse` before this is called.
fn parse_tet(
    t: &mut Toks,
    body: &Body,
    node_ids: &HashMap<u32, usize>,
    solids: &HashSet<String>,
    sets: &mut HashSet<[u32; 4]>,
) -> Result<Tet, ParseError> {
    let mut ids = [0u32; 4];
    let mut cols = [0usize; 4];
    for (k, what) in ["tet node a", "tet node b", "tet node c", "tet node d"]
        .into_iter()
        .enumerate()
    {
        (cols[k], ids[k]) = t.id(what)?;
    }
    let (sc, solid) = t.name("tet solid")?;
    for k in 0..4 {
        if !node_ids.contains_key(&ids[k]) {
            return Err(t.err(Some(cols[k]), format!("tet to missing node {}", ids[k])));
        }
        if ids[..k].contains(&ids[k]) {
            return Err(t.err(Some(cols[k]), format!("tet lists node {} twice", ids[k])));
        }
    }
    if !solids.contains(solid) {
        return Err(t.err(Some(sc), format!("tet uses unknown solid `{solid}`")));
    }
    t.done()?;
    let label = format!("tet {} {} {} {}", ids[0], ids[1], ids[2], ids[3]);
    let at = Some(cols[0]);
    let mut set = ids;
    set.sort_unstable();
    if sets.contains(&set) {
        return Err(t.err(
            at,
            format!(
                "duplicate tet: nodes {} {} {} {} already form a tet",
                ids[0], ids[1], ids[2], ids[3]
            ),
        ));
    }
    let p = ids.map(|id| body.nodes[node_ids[&id]].pos);
    let size = tet_size(p);
    let (volume, longest, shape) = match size {
        TetSize::EdgeNotFinite => {
            return Err(t.err(
                at,
                format!("{label}: its size is not finite: an edge is longer than the largest f64 (the nodes are too far apart)"),
            ));
        }
        TetSize::VolumeNotFinite { longest } => {
            return Err(t.err(
                at,
                format!("{label}: its size is not finite: its rest volume overflows f64 (longest edge {longest})"),
            ));
        }
        TetSize::Finite {
            volume,
            longest,
            shape,
        } => (volume, longest, shape),
    };
    if volume.abs() < TET_MIN_VOLUME {
        return Err(t.err(
            at,
            format!("{label}: its rest volume {volume} is below the minimum {TET_MIN_VOLUME} m^3"),
        ));
    }
    if shape.abs() < TET_MIN_SHAPE {
        return Err(t.err(
            at,
            format!("{label} is too flat: its rest volume {volume} is below {TET_MIN_SHAPE} times the cube of its longest edge {longest}"),
        ));
    }
    if volume < 0.0 {
        return Err(t.err(
            at,
            format!(
                "{label} is inverted: its rest volume {volume} is negative (swap two of its nodes)"
            ),
        ));
    }
    sets.insert(set);
    Ok(Tet {
        nodes: ids,
        solid: solid.to_string(),
    })
}

/// What `tet_size` found for four node positions.
enum TetSize {
    /// An edge's length overflows f64.
    EdgeNotFinite,
    /// Every edge is finite but the rest volume overflows f64.
    VolumeNotFinite { longest: f64 },
    /// `volume` = (b - a) . ((c - a) x (d - a)) / 6 (m^3), `longest` the longest of the six edges
    /// (m), `shape` = volume / longest^3 (signed, like the volume).
    Finite {
        volume: f64,
        longest: f64,
        shape: f64,
    },
}

/// 2^n, exactly, for -1022 <= n <= 1023 (a normal f64 built from its exponent bits).
fn pow2(n: i32) -> f64 {
    debug_assert!((-1022..=1023).contains(&n));
    f64::from_bits(((n + 1023) as u64) << 52)
}

/// x * 2^n, in steps of at most 2^1000 so that no factor overflows. Each step moves |x| toward
/// the result, so no intermediate overflows or underflows past it, and the result is exact
/// whenever it is a normal number.
fn scale2(mut x: f64, mut n: i32) -> f64 {
    while n != 0 {
        let step = n.clamp(-1000, 1000);
        x *= pow2(step);
        n -= step;
    }
    x
}

/// The length of `v` without overflow or underflow in the squares: the components are divided
/// by the largest one first. Infinite when a component is not finite or the length overflows.
fn length_scaled(v: Vec3) -> f64 {
    if !(v.x.is_finite() && v.y.is_finite() && v.z.is_finite()) {
        return f64::INFINITY;
    }
    let m = v.x.abs().max(v.y.abs()).max(v.z.abs());
    if m == 0.0 {
        return 0.0;
    }
    let (x, y, z) = (v.x / m, v.y / m, v.z / m);
    m * (x * x + y * y + z * z).sqrt()
}

/// The rest volume, longest edge and shape ratio of a tet, with no intermediate value that
/// overflows when the true values are finite (round 3 of t-20261121-005119). The three edges
/// from `a` are scaled by 2^-k, the power of two nearest above the longest edge, so every
/// component is at most about 1 and the triple product at most about 8; the shape ratio is
/// taken in that scaled form; the volume is the scaled triple product / 6 scaled back by 2^3k.
/// Scaling by a power of two is exact unless it makes a component subnormal (a component
/// below 2^-1022 times the longest edge, where only that component's lowest bits are lost),
/// so for every other tet whose plain triple product neither overflows nor underflows the
/// volume is bit for bit `(b - a) . ((c - a) x (d - a)) / 6`. The
/// volume can only underflow when the true volume is below about 2e-308 m^3, far below
/// `TET_MIN_VOLUME`, so the minimum test is never passed or failed by an underflow.
fn tet_size(p: [Vec3; 4]) -> TetSize {
    let mut longest = 0.0f64;
    for (i, j) in [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)] {
        let l = length_scaled(p[j] - p[i]);
        if !l.is_finite() {
            return TetSize::EdgeNotFinite;
        }
        longest = longest.max(l);
    }
    if longest == 0.0 {
        return TetSize::Finite {
            volume: 0.0,
            longest,
            shape: 0.0,
        };
    }
    let k = longest.log2().ceil() as i32;
    let scaled = |v: Vec3| Vec3::new(scale2(v.x, -k), scale2(v.y, -k), scale2(v.z, -k));
    let (u, v, w) = (
        scaled(p[1] - p[0]),
        scaled(p[2] - p[0]),
        scaled(p[3] - p[0]),
    );
    let sixth = u.dot(v.cross(w)) / 6.0;
    let ls = scale2(longest, -k);
    let shape = sixth / ls / ls / ls;
    let volume = scale2(sixth, 3 * k);
    if !volume.is_finite() {
        return TetSize::VolumeNotFinite { longest };
    }
    TetSize::Finite {
        volume,
        longest,
        shape,
    }
}

/// `wheel <name> mount <id> [<id> ...] radius <r> width <w> mass <m> inertia <i>
/// steerable <yes|no> driven <yes|no> braked <yes|no>`
fn parse_wheel(t: &mut Toks, body: &Body, node_ids: &HashMap<u32, usize>) -> Result<Wheel, ParseError> {
    let (nc, name) = t.name("wheel name")?;
    if body.wheels.iter().any(|w| w.name == name) {
        return Err(t.err(Some(nc), format!("duplicate wheel `{name}`")));
    }
    t.keyword("mount")?;
    let mut mount: Vec<u32> = Vec::new();
    loop {
        match t.peek() {
            Some((c, "radius")) if mount.is_empty() => {
                return Err(t.err(Some(c), format!("wheel `{name}` needs at least one mount node")));
            }
            Some((_, "radius")) | None if !mount.is_empty() => break,
            _ => {}
        }
        let (c, id) = t.id("mount node")?;
        if !node_ids.contains_key(&id) {
            return Err(t.err(Some(c), format!("wheel `{name}` is mounted on missing node {id}")));
        }
        if mount.contains(&id) {
            return Err(t.err(Some(c), format!("wheel `{name}`: mount lists node {id} twice")));
        }
        mount.push(id);
    }
    let (_, radius) = t.positive_field("radius", "radius")?;
    let (_, width) = t.positive_field("width", "width")?;
    let (_, mass) = t.positive_field("mass", "wheel mass")?;
    let (_, inertia) = t.positive_field("inertia", "inertia")?;
    let steerable = t.flag("steerable")?;
    let driven = t.flag("driven")?;
    let braked = t.flag("braked")?;
    t.done()?;
    Ok(Wheel { name: name.to_string(), mount, radius, width, mass, inertia, steerable, driven, braked })
}

/// The wheel a suspension or tyre line names: it must exist and not have one already.
fn claim_wheel<'a>(t: &mut Toks<'a>, body: &Body, taken: &mut HashSet<String>, what: &str) -> Result<&'a str, ParseError> {
    let (wc, wname) = t.name("wheel name")?;
    if !body.wheels.iter().any(|w| w.name == wname) {
        return Err(t.err(Some(wc), format!("{what} names unknown wheel `{wname}`")));
    }
    if !taken.insert(wname.to_string()) {
        return Err(t.err(Some(wc), format!("wheel `{wname}` already has a {what}")));
    }
    Ok(wname)
}

/// `suspension <wheel> spring <k> damper <c> rest <length> travel <min> <max>`
fn parse_suspension(t: &mut Toks, body: &Body, car: &mut CarLines) -> Result<Suspension, ParseError> {
    let wheel = claim_wheel(t, body, &mut car.suspended, "suspension")?;
    let (_, spring) = t.positive_field("spring", "spring rate")?;
    let (_, damper) = t.nonnegative_field("damper", "damper rate")?;
    let (rc, rest) = t.positive_field("rest", "suspension rest length")?;
    t.keyword("travel")?;
    let (minc, travel_min) = t.number("travel min")?;
    let (_, travel_max) = t.number("travel max")?;
    t.done()?;
    if travel_min < 0.0 {
        return Err(t.err(Some(minc), format!("travel min {travel_min} must be >= 0")));
    }
    if travel_min >= travel_max {
        return Err(t.err(Some(minc), format!("travel min {travel_min} must be below travel max {travel_max}")));
    }
    if !(travel_min < rest && rest < travel_max) {
        return Err(t.err(Some(rc), format!("rest length {rest} must be inside the travel ({travel_min}, {travel_max}), off both stops")));
    }
    Ok(Suspension { wheel: wheel.to_string(), spring, damper, rest, travel_min, travel_max })
}

/// `tyre <wheel> mu <mu> radial_stiffness <k> radial_damping <c> long_stiffness <cx>
/// lat_stiffness <cy>`
fn parse_tyre(t: &mut Toks, body: &Body, car: &mut CarLines) -> Result<Tyre, ParseError> {
    let wheel = claim_wheel(t, body, &mut car.tyred, "tyre")?;
    let (_, mu) = t.positive_field("mu", "mu")?;
    let (_, radial_stiffness) = t.positive_field("radial_stiffness", "radial_stiffness")?;
    let (_, radial_damping) = t.nonnegative_field("radial_damping", "radial_damping")?;
    let (_, long_stiffness) = t.positive_field("long_stiffness", "long_stiffness")?;
    let (_, lat_stiffness) = t.positive_field("lat_stiffness", "lat_stiffness")?;
    t.done()?;
    Ok(Tyre { wheel: wheel.to_string(), mu, radial_stiffness, radial_damping, long_stiffness, lat_stiffness })
}

/// `controls max_steer_deg <a> max_drive_torque <t> max_brake_torque <t>
/// [rolling_resistance <c_rr>] [drag_area <CdA>]`
fn parse_controls(t: &mut Toks, body: &Body, kc: usize) -> Result<Controls, ParseError> {
    if body.controls.is_some() {
        return Err(t.err(Some(kc), "second `controls` line: a body has at most one".into()));
    }
    if body.wheels.is_empty() {
        return Err(t.err(Some(kc), "`controls` without any wheel".into()));
    }
    t.keyword("max_steer_deg")?;
    let (sc, max_steer_deg) = t.number("max_steer_deg")?;
    if !(max_steer_deg > 0.0 && max_steer_deg < 90.0) {
        return Err(t.err(Some(sc), format!("max_steer_deg {max_steer_deg} must be > 0 and < 90")));
    }
    let (dc, max_drive_torque) = t.positive_field("max_drive_torque", "max_drive_torque")?;
    let (bc, max_brake_torque) = t.positive_field("max_brake_torque", "max_brake_torque")?;
    // Optional, in this order. `number` refuses NaN and infinities by name, and
    // `nonnegative_field` a value below 0.
    let mut rolling_resistance = None;
    let mut drag_area = None;
    if t.peek().map(|p| p.1) == Some("rolling_resistance") {
        let (_, v) = t.nonnegative_field("rolling_resistance", "rolling_resistance")?;
        rolling_resistance = Some(v);
    }
    if t.peek().map(|p| p.1) == Some("drag_area") {
        drag_area = Some(t.nonnegative_field("drag_area", "drag_area")?.1);
    }
    t.done()?;
    for (c, what, any) in [
        (sc, "steerable", body.wheels.iter().any(|w| w.steerable)),
        (dc, "driven", body.wheels.iter().any(|w| w.driven)),
        (bc, "braked", body.wheels.iter().any(|w| w.braked)),
    ] {
        if !any {
            return Err(t.err(Some(c), format!("no wheel is {what}, so this control would act on nothing")));
        }
    }
    Ok(Controls {
        max_steer_deg,
        max_drive_torque,
        max_brake_torque,
        rolling_resistance,
        drag_area,
    })
}

/// The headers this parser reads, for messages.
const HEADERS: &str = "`bsn 1`, `bsn 2` or `bsn 3`";

/// The version on line 1, 1 to `VERSION`.
fn parse_header(line: &str) -> Result<u32, ParseError> {
    let err = |column: Option<usize>, message: String| ParseError {
        line: 1,
        column,
        message,
    };
    let rest = match line.strip_prefix("bsn ") {
        Some(r) => r,
        None => {
            return Err(err(
                None,
                format!("the first line must be {HEADERS}, found `{line}`"),
            ))
        }
    };
    match parse_id(rest) {
        Some(v) if (1..=VERSION).contains(&v) => Ok(v),
        Some(v) => Err(err(
            Some(5),
            format!("unknown version {v}: this parser reads versions 1, 2 and 3"),
        )),
        None if !rest.is_empty() && rest.bytes().all(|b| b.is_ascii_digit()) => Err(err(
            Some(5),
            format!("unknown version {rest}: this parser reads versions 1, 2 and 3"),
        )),
        None => Err(err(
            Some(5),
            format!("version `{rest}` is not a version number (the first line must be {HEADERS})"),
        )),
    }
}

// ---------------------------------------------------------------------------------------
// Writing

fn yes_no(b: bool) -> &'static str {
    if b {
        "yes"
    } else {
        "no"
    }
}

/// The canonical text of a body (the one `parse` reads back to an equal body). The header is
/// the lowest version that holds the body: `bsn 3` with a solid or a tet, else `bsn 2` with car
/// sections, else `bsn 1`. A beam's rest length is written only when it is not exactly the
/// distance between its nodes. Panics if the body is not one `parse` could produce (a beam
/// naming a missing node, for example).
pub fn write(body: &Body) -> String {
    let pos: HashMap<u32, Vec3> = body.nodes.iter().map(|n| (n.id, n.pos)).collect();
    let has_car = !body.wheels.is_empty() || !body.suspensions.is_empty() || !body.tyres.is_empty() || body.controls.is_some();
    let has_solids = !body.solids.is_empty() || !body.tets.is_empty();
    let version = if has_solids {
        3
    } else if has_car {
        2
    } else {
        1
    };
    let mut s = format!("bsn {version}\n");
    for m in &body.materials {
        s.push_str(&format!("material {} stiffness {} damping {}", m.name, m.stiffness, m.damping));
        if let Some(y) = m.yield_strain {
            s.push_str(&format!(" yield_strain {y}"));
        }
        if let Some(b) = m.break_strain {
            s.push_str(&format!(" break_strain {b}"));
        }
        if let Some(u) = m.unload_return {
            s.push_str(&format!(" unload_return {u}"));
        }
        s.push('\n');
    }
    for m in &body.solids {
        s.push_str(&format!(
            "solid {} plateau_stress {} yield_strain {} damping_ratio {}\n",
            m.name, m.plateau_stress, m.yield_strain, m.damping_ratio
        ));
    }
    for n in &body.nodes {
        s.push_str(&format!("node {} {} {} {} mass {}\n", n.id, n.pos.x, n.pos.y, n.pos.z, n.mass));
    }
    for b in &body.beams {
        s.push_str(&format!("beam {} {} {}", b.a, b.b, b.material));
        let dist = (pos[&b.b] - pos[&b.a]).length();
        if b.rest != dist {
            s.push_str(&format!(" rest {}", b.rest));
        }
        s.push('\n');
    }
    for t in &body.tets {
        let [a, b, c, d] = t.nodes;
        s.push_str(&format!("tet {a} {b} {c} {d} {}\n", t.solid));
    }
    for g in &body.groups {
        s.push_str(&format!("group {}", g.name));
        for id in &g.nodes {
            s.push_str(&format!(" {id}"));
        }
        s.push('\n');
    }
    for v in &body.velocities {
        s.push_str(&format!("velocity {} {} {} {}\n", v.group, v.velocity.x, v.velocity.y, v.velocity.z));
    }
    for w in &body.wheels {
        s.push_str(&format!("wheel {} mount", w.name));
        for id in &w.mount {
            s.push_str(&format!(" {id}"));
        }
        s.push_str(&format!(
            " radius {} width {} mass {} inertia {} steerable {} driven {} braked {}\n",
            w.radius,
            w.width,
            w.mass,
            w.inertia,
            yes_no(w.steerable),
            yes_no(w.driven),
            yes_no(w.braked)
        ));
    }
    for p in &body.suspensions {
        s.push_str(&format!(
            "suspension {} spring {} damper {} rest {} travel {} {}\n",
            p.wheel, p.spring, p.damper, p.rest, p.travel_min, p.travel_max
        ));
    }
    for ty in &body.tyres {
        s.push_str(&format!(
            "tyre {} mu {} radial_stiffness {} radial_damping {} long_stiffness {} lat_stiffness {}\n",
            ty.wheel, ty.mu, ty.radial_stiffness, ty.radial_damping, ty.long_stiffness, ty.lat_stiffness
        ));
    }
    if let Some(c) = &body.controls {
        s.push_str(&format!(
            "controls max_steer_deg {} max_drive_torque {} max_brake_torque {}",
            c.max_steer_deg, c.max_drive_torque, c.max_brake_torque
        ));
        if let Some(v) = c.rolling_resistance {
            s.push_str(&format!(" rolling_resistance {v}"));
        }
        if let Some(v) = c.drag_area {
            s.push_str(&format!(" drag_area {v}"));
        }
        s.push('\n');
    }
    for p in &body.planes {
        s.push_str(&format!(
            "plane {} {} {} {} {} {}\n",
            p.point.x, p.point.y, p.point.z, p.normal.x, p.normal.y, p.normal.z
        ));
    }
    s
}
