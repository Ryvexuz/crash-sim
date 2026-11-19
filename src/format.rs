//! The `.bsn` text format (version 1): a node-beam body as data. The grammar is in
//! docs/FORMAT.md. The parser is strict: every malformed input is a `ParseError` naming the
//! line (and the column when a token is at fault), never a silent default.
//!
//! `write(&parse(x)?)` is byte-identical to `x` for a canonical file (one `write` produced);
//! `parse(&write(b)) == b` for every body `parse` can produce.

use crate::vec3::Vec3;
use crate::Beam;
use std::collections::{HashMap, HashSet};
use std::fmt;

/// The only version this parser reads. A new version adds a branch in `parse`, so older
/// files keep loading (docs/FORMAT.md, "Versions").
pub const VERSION: u32 = 1;

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

#[derive(Clone, Debug, PartialEq)]
pub struct Body {
    pub materials: Vec<Material>,
    pub nodes: Vec<Node>,
    pub beams: Vec<BodyBeam>,
    pub groups: Vec<Group>,
    pub velocities: Vec<Velocity>,
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

    /// Positions, start velocities, beams (stiffness and damping from the material) and
    /// masses, node indices in file order.
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

fn section_rank(keyword: &str) -> Option<usize> {
    ["material", "node", "beam", "group", "velocity", "plane"].iter().position(|k| *k == keyword)
}

/// Parse a `.bsn` file. See docs/FORMAT.md for the grammar.
pub fn parse(text: &str) -> Result<Body, ParseError> {
    let err = |line: usize, column: Option<usize>, message: String| ParseError { line, column, message };

    if text.is_empty() {
        return Err(err(1, None, "empty file: the first line must be `bsn 1`".into()));
    }
    if !text.ends_with('\n') {
        let last = text.split('\n').count();
        return Err(err(last, None, "the file does not end with a newline".into()));
    }
    let body_text = &text[..text.len() - 1];

    let mut body = Body {
        materials: Vec::new(),
        nodes: Vec::new(),
        beams: Vec::new(),
        groups: Vec::new(),
        velocities: Vec::new(),
        planes: Vec::new(),
    };
    let mut node_ids: HashMap<u32, usize> = HashMap::new();
    let mut beam_pairs: HashSet<(u32, u32)> = HashSet::new();
    let mut velocity_nodes: HashSet<u32> = HashSet::new();
    let mut rank = 0usize;
    let mut last_line = 1usize;

    for (i, line) in body_text.split('\n').enumerate() {
        let n = i + 1;
        last_line = n;
        if n == 1 {
            parse_header(line)?;
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
                return Err(t.err(Some(kc), format!("unknown keyword `{kw}` (expected material, node, beam, group, velocity or plane)")));
            }
        };
        if this_rank < rank {
            return Err(t.err(Some(kc), format!("`{kw}` after a later section: lines come in the order material, node, beam, group, velocity, plane")));
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
                t.done()?;
                body.materials.push(Material { name: name.to_string(), stiffness, damping, yield_strain, break_strain });
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
    Ok(body)
}

fn parse_header(line: &str) -> Result<(), ParseError> {
    let err = |column: Option<usize>, message: String| ParseError { line: 1, column, message };
    let rest = match line.strip_prefix("bsn ") {
        Some(r) => r,
        None => return Err(err(None, format!("the first line must be `bsn {VERSION}`, found `{line}`"))),
    };
    match parse_id(rest) {
        Some(v) if v == VERSION => Ok(()),
        Some(v) => Err(err(Some(5), format!("unknown version {v}: this parser reads version {VERSION} only"))),
        None if !rest.is_empty() && rest.bytes().all(|b| b.is_ascii_digit()) => {
            Err(err(Some(5), format!("unknown version {rest}: this parser reads version {VERSION} only")))
        }
        None => Err(err(Some(5), format!("version `{rest}` is not a version number (the first line must be `bsn {VERSION}`)"))),
    }
}

// ---------------------------------------------------------------------------------------
// Writing

/// The canonical text of a body (the one `parse` reads back to an equal body). A beam's rest
/// length is written only when it is not exactly the distance between its nodes. Panics if
/// the body is not one `parse` could produce (a beam naming a missing node, for example).
pub fn write(body: &Body) -> String {
    let pos: HashMap<u32, Vec3> = body.nodes.iter().map(|n| (n.id, n.pos)).collect();
    let mut s = format!("bsn {VERSION}\n");
    for m in &body.materials {
        s.push_str(&format!("material {} stiffness {} damping {}", m.name, m.stiffness, m.damping));
        if let Some(y) = m.yield_strain {
            s.push_str(&format!(" yield_strain {y}"));
        }
        if let Some(b) = m.break_strain {
            s.push_str(&format!(" break_strain {b}"));
        }
        s.push('\n');
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
    for p in &body.planes {
        s.push_str(&format!(
            "plane {} {} {} {} {} {}\n",
            p.point.x, p.point.y, p.point.z, p.normal.x, p.normal.y, p.normal.z
        ));
    }
    s
}
