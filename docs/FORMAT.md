# The `.bsn` format, version 1

A `.bsn` file is a node-beam body as plain text: materials, nodes, beams, named groups, optional start velocities and collider planes. Code: `src/format.rs` (`parse`, `write`, `Body`). Tests: `tests/format.rs`. Scenario files: `scenarios/`.

## Grammar

One record per line. Tokens are separated by exactly one space (no tabs, no leading or trailing space). Line endings are `\n` and the file ends with one. Line 1 is `bsn 1`. After it, blank lines and lines starting with `#` are ignored (comments are not kept by `write`). Every other line starts with a keyword, and the keywords come in this order (a keyword may not follow a later one):

```
bsn 1
material <name> stiffness <k> damping <c> [yield_strain <y>] [break_strain <b>]
node <id> <x> <y> <z> mass <m>
beam <a> <b> <material> [rest <r>]
group <name> <id> [<id> ...]
velocity <group> <vx> <vy> <vz>
plane <px> <py> <pz> <nx> <ny> <nz>
```

- `<name>`: letters, digits and `_`, starting with a letter or `_`. Material names are unique, group names are unique.
- `<id>`: decimal digits, no sign, no leading zero, at most 4294967295. Node ids are unique. They need not be consecutive; a node's index in the sim is its position in the file.
- Numbers: anything Rust's `f64` parser reads, but it must be finite (`nan`, `inf` and `1e999` are refused).
- `material`: `stiffness` (N/m) > 0; `damping` (N s/m) >= 0; `yield_strain` and `break_strain`, if given, > 0 and in that order, and `break_strain` > `yield_strain` when both are given. Yield and break strain are stored now and used by later stages.
- `node`: any finite position (m); `mass` (kg) > 0. At least one node is required (an empty body is refused). A body with nodes but no beams is allowed.
- `beam`: both nodes must be declared above the beam, differ, and be at different positions (a zero-length beam is refused even when `rest` is given); the material must exist; a pair of nodes has at most one beam (either order). The rest length is the distance between the two nodes unless `rest <r>` gives it (`r` > 0).
- `group`: at least one node, each declared, none twice.
- `velocity`: sets the start velocity (m/s) of every node of an existing group; one line per group; two groups that both have a velocity must not share a node. Nodes in no such group start at rest.
- `plane`: a collider plane through a point with a normal (not zero by components; it is stored as written, not normalised, so a consumer must scale by the largest component before taking a length); the side the normal points to is free space. Wiring planes and velocities into the sim is nb-34 and nb-36's job; the format only holds them.

## Canonical form

`write(&Body)` produces the canonical text: the sections in the order above, one space between tokens, numbers printed by Rust's shortest round-trip form (`{}`: always the same digits that read back to the same value, never an exponent), and `rest` printed only when it differs from the exact distance between the nodes. So `parse(write(b)) == b` for every body `parse` can produce, and `write(parse(x)) == x` for every canonical file. A file with comments, blank lines, differently printed numbers (`1.0`, `1e3`) or an explicit `rest` that equals the distance parses fine but is not canonical. `cube.bsn` and `box.bsn` are canonical; `canonical_file_round_trips_byte_identical` checks it.

## Refusals

`parse` returns `Err(ParseError { line, column, message })`; `Display` is `line 7, column 12: <message>` (column omitted when no single token is at fault, such as an empty body, which is reported on the last line). Nothing is ever defaulted: a missing or extra token, an unknown keyword, a wrong number, a wrong order, a bad version, a missing final newline, a carriage return or a tab is an error. Every refusal has a test in `tests/format.rs` that checks the line, the column and the words of the message.

## Versions

The first line names the version. This parser reads version 1 only and refuses any other number by name (`unknown version 2: this parser reads version 1 only`). When the schema changes, a new version is added as a new branch of `parse` that keeps reading the old layout, so an old file still loads (roadmap node 0.5); a test for that is added with the first new version.

## Scenario files

- `scenarios/cube.bsn`: today's cube for `Config::default()` (tilted 5 degrees, lowest corner 2 m up): 8 nodes of 1 kg, all 28 pairs as beams of one material (`cube_beam`, 20000 N/m, 20 N s/m), positions printed to the last digit. `cube_bsn_matches_default_cube` compares it bit for bit with `Sim::new(Config::default())`. It is generated from the sim's own arrays, so a change to the cube constants means regenerating it (the test then fails until you do).
- `scenarios/box.bsn`: the box for milestone 2: 2 x 1 x 1 m, 3 x 2 x 2 nodes at x in {0, 1, 2}, y and z in {0, 1} (long axis x, resting on y = 0), 1 kg each, node id = 4 ix + 2 iy + iz. Beams are all 28 pairs inside each of the two unit cells (edges, face and body diagonals; the 6 shared-face pairs once), 50 in all, of material `box_beam` (the cube's 20000 N/m and 20 N s/m, elastic). Groups `back` (nodes 0 to 3, x = 0) and `front` (nodes 8 to 11, x = 2).

## Line endings in a Windows checkout

The format is strict about the line ending `\n` (LF). Git on Windows with `core.autocrlf=true` would check text files out with CRLF, which `parse` refuses by name. The repo now enforces LF: `.gitattributes` has `*.bsn text eol=lf` and `*.input text eol=lf`, and the tests read scenario files exactly as checked out (`scenario_files_have_no_carriage_returns` fails on a CRLF copy).
