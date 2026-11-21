# validation/CAR.md: the validation car (test A)

`scenarios/validation_car.bsn` is the car that m4 crashes against test A (docs/STAGE6-M4-M5.md, "The run"); `scenarios/validation_wall.bsn` is that file plus one `plane` line. Test A is NHTSA test 7504, a 2012 Hyundai Accent GLS 4-door at 56.2 km/h into a rigid full-width barrier (`validation/test-a.meta`, `validation/README.md`). Test B's car (`validation_car_b.bsn`) is not part of this file.

Labels used below: **sourced** (read from the named file in this task), **derived** (computed from sourced numbers, working shown), **assumed** (not read from any source in this task; chosen here, and the reason given). Nothing here was tuned to a pulse: no run against test A's pulse exists yet.

## Sources

| Id | File | Where it came from | SHA-256 |
|---|---|---|---|
| R | `v07504R001.pdf`, NHTSA test report "Frontal Impact Testing of 2012 Hyundai Accent GLS 4-Dr Sedan" (MGA Research) | `https://nrd-static.nhtsa.dot.gov/reports/vehdb/v00000/v07500/v07504R001.pdf` (`validation/README.md`, "Redo the download", step 5) | `744c272eb19c1be29973dff1c520cefdbcb368ee9c4f61d8f9df01ef8030ac87` |
| E | `v07504.zip`, member `v07504.EV5`, the VEHICLE record (line 7), fields 11 to 16 | `https://nrd-static.nhtsa.dot.gov/compress/ev/vehdb/v00000/v07500/v07504.zip` (the `url` of `validation/test-a.meta`) | `e59fc567f56fb428f571fe80fb2a4ac814218e6608d30337c9edd35470196551` |
| G | NHTSA Test Reference Guide Vol. I, Version 5, section 2.4 (field names of the EV5 VEHICLE record) | `validation/README.md`, "Redo the download", step 6 | `afdd3e66abd72a2a399fc9723e9158cd5ce22b882b53962008f510b6d9a3376b` (`vol1.pdf`, the copy read for n20-02) |
| D | `docs/STAGE6-M4-M5.md` of this repo, the `frozen:` lines 143 to 148 | the repo | (Git) |

R, E and G were read from the copies downloaded for n20-02 (`scratch/n20-02/ref/`, `scratch/n20-02/raw/`; the checksums above are of those copies). Their text was extracted to text with line numbers (`v07504R001.txt`; the line numbers below are lines of that extraction, and the section names are the report's own, so a reader of the PDF can find them). Raw downloads are not in this repo (validation/README.md).

The 2013 Camry reference set under `reference/camry-2013/` was **not** used (test A is NHTSA 7504, the Accent, per `validation/test-a.meta`, not a Camry): test A is the Accent, and nothing in that set describes the Accent.

## Matched dimensions

Axes as in docs/FORMAT.md: x forward, y up, z across (+z right). The file puts the middle of the wheelbase at x = 0. Every file value below was read back from `scenarios/validation_car.bsn` by the check in "Checks" (output pasted there).

| Quantity | Test A value | In the file | Source | Label |
|---|---|---|---|---|
| Test mass | 1330 kg (1329.5 kg as tested, with the two dummies; VEHTWT 1330) | 1210 kg on 48 nodes + 4 x 30 kg wheels = 1330 kg | R "Test Vehicle Weights", "As Tested (ATW)" total 1329.5 (line 391), and "Calculated Vehicle Target Weight" 1336.8 (lines 399 to 400); E field 11 = 1330; `validation/test-a.meta` `mass_kg = 1330` | sourced |
| Wheelbase | 2568 mm | 2.568 m (front axle x = 1.284, rear axle x = -1.284) | R "General Test Vehicle Data", "Total Vehicle Wheel Base" 2568 (line 416); also data sheet 13 "Wheelbase (mm): 2568" (line 1066); E field 13 = 2568 | sourced |
| Overall length | 4364 mm at the centreline (4175 mm at the left and right sides) | 4.364 m (x from -2.261 to 2.103) | R "Total Vehicle Length at Centerline" 4364 (line 418), "Target Vehicle Structural Measurement" item 1 "Total Length" 4364 (line 438); E field 14 = 4364 | sourced |
| Front overhang (the crush zone) | 819 mm, front bumper to front axle | 0.819 m (front-most node x = 2.103, front axle x = 1.284) | R "Target Vehicle Structural Measurement" item 15 "Front Bumper - Front Axle Distance" 819 (line 466) | sourced |
| Rear overhang | 977 mm | 0.977 m (rear axle -1.284, rear-most node -2.261) | 4364 - 2568 - 819 = 977 | derived |
| Overall width | 1688 mm | 1.688 m, the z extent of the nodes (-0.844 to +0.844) | R item 2 "Total Width" 1688 (line 440); E field 15 = 1688 | sourced |
| Centre of gravity | 1083 mm behind the front axle (as tested) | 1.083 m behind the front axle (mass-weighted, wheels included) | E field 16 (VEHCG, Guide field 16 "Vehicle Center of Gravity Distance Behind Front Axle") = 1083; R "Test Vehicle Attitudes and CG" gives the as-tested weights front 768.9 kg, rear 560.6 kg, and 560.6 x 2568 / 1329.5 = 1082.9 mm is the Guide's own formula for field 16 | sourced |
| Track width | not in R, E or G (searched; see "Track width sources") | 1.5085 m, the z distance between the left and right wheel mounts, the same at the front and the rear axle (mounts at z = -0.75425 and +0.75425) | public web pages, three agreeing (below): front 1506 mm, rear 1511 mm; the mean 1508.5 mm is used. Upper bound from R: body width less one tyre width, 1688 - 175 (R "Tire Width" 175, line 354) = 1513 mm; 1508.5 mm is under it by 4.5 mm (the rear figure, 1511 mm, by 2 mm) | sourced (web, other sources than R) |
| Speed | 56.2 km/h (`speed_kmh` of `validation/test-a.meta`; E field 42 VEHSPD) | `velocity all 15.61111111111111 0 0`: 56.2 / 3.6 = 15.611111... m/s on all 48 nodes; wheels start with the mean velocity of their mount nodes (docs/FORMAT.md, car sections) | `validation/test-a.meta`; E field 42 | sourced |
| Wall | rigid, full width | `plane 2.153 0 0 -1 0 0`: 0.05 m ahead of the front-most node (2.103 + 0.05), normal (-1, 0, 0) | docs/STAGE6-M4-M5.md, "The run" (0.05 m ahead, normal -x) | brief |

### Track width sources

Local files were searched first and had no track figure: R's text (only "Treadwear" and seat track), the EV5 record, the ISO export `v07504ISO.zip` (`.tst`, `.occ`, `.mme`) and `reference/` (the Camry set only). Then web pages, public, no account or fee, read 2026-11-21 (Lazlo's approval for internet lookups, 2026-11-21 03:48Z). The 2012 Accent sedan is the second-generation "RB" car (same car as the 2013 to 2016 sedan the maker's brochure shows).

| Id | Page | URL | Quoted figure | Read |
|---|---|---|---|---|
| W1 | Hyundai Motor Company, "Accent" e-brochure (2014, `GEN. LHD 1410 ENG. ID-KM`), dimensions page; the maker's own publication. A sedan (length 4,370, wheelbase 2,570, front and rear overhang 820 and 980 mm on that page) | `https://www.hyundai.com/content/dam/hyundai/ww/en/images/footer/downloads/cars/e-brochure/accent-ebrochure-2014.pdf` (SHA-256 of the file read `a79194949fe43c81b8359e277f390bfe0e3a036db72485deb591a00a1a6ad559`) | "Wheel tread 14" (Front /rear) : 1,506 /1,511   16" (Front /rear) : 1,488 /1,493" (units mm: the page is headed "Dimensions unit : mm") | 2026-11-21 |
| W2 | "2012 Hyundai Accent 4dr Sdn Man L Price & Specifications - The Car Guide" (Canadian sedan) | `https://www.guideautoweb.com/en/makes/hyundai/accent/2012/specifications/4dr-sdn-man-l` | "Front wheel spacing: 1,506 mm (59″)"; "Rear wheel spacing: 1,511 mm (59″)"; also "Axle-to-axle distance: 2,570 mm (101″)", "Overall vehicle length: 4,369 mm (172″)" | 2026-11-21 |
| W3 | "2012 Hyundai Accent Dimensions - iSeeCars.com", GLS 4dr Sedan | `https://www.iseecars.com/car/2012-hyundai-accent-dimensions` | "Front Track" 59.29999923706055 in; "Rear Track" 59.5 in (= 1506.2 mm and 1511.3 mm); also "Wheel Base" 101.2 in, "Length" 172.0 in | 2026-11-21 |

Not usable: the JD Power page for the 2012 Accent GLS answered HTTP 403, so it was not read; the search result's snippet of it was not used. Australian Carsales pages quote 1489 / 1493 mm (Elite) and 1507 / 1511 mm (Active) in search snippets only; not read, not used.

**Agreement:** W1, W2 and W3 all give 1506 mm front and 1511 mm rear (W3 within 0.3 mm after rounding); the sources do not disagree by more than 10 mm. W1 is the maker's. **Which value:** W1's 14" line applies, because the test car wore 14-inch wheels (P175/70R14, R line 346); the 16" line (1488 / 1493) does not. The grid has one z for all four mounts, so the mean of front and rear, 1508.5 mm, is used; this is 2.5 mm from either axle's figure. The web figures are for the production car, not measured on the tested vehicle; the pages do not give a model year of 2012 for W1 (a 2014 brochure of the same generation), a limit of this source.

Where the file differs from the reference car, and why it was left that way (the brief: "like `car_basic.bsn` everywhere else"):

- **Tyres:** `car_basic.bsn`'s 195/65 R15 (radius 0.3175 m, width 0.195 m) were kept (so the tyre's outer face, 0.75425 + 0.0975 = 0.852 m from the centreline, is 8 mm outside the node grid's body width of 0.844 m; the grid has no wheel-arch geometry and the tyre does not touch the wall in the pulse window). The test car wore P175/70R14 (R, "Tire Size on Vehicle" P175/70R14, line 346; "Tire Width" 175, line 354; "Aspect Ratio" 70, line 356; "Wheel Diameter" 14, line 360), derived radius 14 x 25.4 / 2 + 0.70 x 175 = 300.3 mm. Not matched, because the start height rule (tyres 0.20 m above the ground, suspension at its rest length, `car_basic.bsn`) and the wheel numbers were chosen together; the tyres are not in the pulse's first 150 ms (gravity 0, wheels roll freely, docs/STAGE6-M4-M5.md).
- **Wheel mass 30 kg each, suspension, tyre and controls lines:** as `car_basic.bsn`, **assumed**.
- **Node masses:** the total (1330 kg) and the centre of gravity (1083 mm behind the front axle) are matched; the split between stations and between floor and roof is **assumed**. Station totals (8 nodes each, rear to front): 132, 220, 316, 288, 136, 118 kg, chosen so that the sum and the centre of gravity come out right. Within a station every roof node is 14.75 kg and the four floor nodes take the rest (floor per node, rear to front: 18.25, 40.25, 64.25, 57.25, 19.25, 14.75 kg), floor-heavy as `car_basic.bsn` (40 / 14 kg). The roof cannot be lighter than 14.75 kg because the step limit below uses the lightest mass in the body, so the split is less extreme than `car_basic.bsn`'s. Read back from the file: the car's centre of gravity is 0.920 m above the ground at the start (wheels at their start height; sprung nodes alone 0.960 m), against `car_basic.bsn`'s derived 0.52 m. It is still higher than a real car's (the lumped nodes cannot be lighter), and nothing has measured the effect of that on pitch.
- **Layout:** 6 stations of 8 nodes (x = -2.261, -1.284, 0, 1.284, 1.6935, 2.103; z = -0.844, -0.75425, 0.75425, 0.844; y = 0.55, 1.95), node id = 8 ix + 2 iz + iy, as `car_basic.bsn` with one more station in the front overhang, so the crush zone is two equal segments of 0.4095 m. The two outer z stations are at the body width (1.688 m); the inner two are the wheel mounts' z (1.5085 m track). 296 beams: all 28 pairs inside each of the 15 cells, shared-face pairs once, as `car_basic.bsn`.
- **Groups:** `all` (48 nodes, carries the velocity), `front` (the 8 nodes at x = 2.103), `rear` (the 8 at x = -2.261), `sensor` (nodes 12 and 14: the two floor nodes at the rear axle on the right, z = +0.75425 and +0.844). `sensor` is the right rear floor, placed at the rear axle because the channel is "FLOORPAN - RIGHT REAR", ISO name "Floor over Rear Axle", comment "RIGHT REAR SEAT CROSSMEMBER X" (`validation/README.md`, "Channel details"). It is 3.387 m behind the front-most node (2.103 - (-1.284)), outside the crush zone. *Assumed:* that the crossmember is within a station of the rear axle (R gives B-pillar to rear axle 1095 mm, item 18, so the rear seat is between the two but its position is not stated).
- **Where the velocity line sits:** docs/FORMAT.md puts `velocity` before the car sections and `plane` last, so the start velocity is in `validation_car.bsn` (the car at test speed) and `validation_wall.bsn` is that file followed by the one `plane` line. `cmp` shows the wall file starts with the car file byte for byte (Checks).

## Calibratable parameters and their ranges

Only the material `crush` is calibratable, through three physical quantities (docs/STAGE6-M4-M5.md, "What is calibrated, and its physical range"; the `frozen: range` lines 143 to 145 are the limits, read here from that file at this commit). The 16 `crush` beams are the longitudinal beams of the two front segments (node `a` to node `a + 8` for `a` in stations 3 and 4); every other beam is `chassis`, which is not calibratable and stays at `car_basic.bsn`'s numbers (stiffness 300000 N/m, damping 200 N s/m, yield strain 0.02).

| Quantity | Physical range (frozen:) | Becomes in the file | Starting value in the file | Result in the file |
|---|---|---|---|---|
| P1 `area_m2`, steel cross-section one crush beam stands for | 0.00007 to 0.002 m^2 | `stiffness` k = E A / L_c, E = 210e9 Pa (`frozen: steel_E_pa`) | A = 0.0001225 m^2 | k = 62820512.821 N/m |
| P2 `yield_strain` | 0.0010 to 0.0057 | `yield_strain` as is | 0.00167 | `yield_strain 0.00167` |
| P3 `damping_ratio` | 0 to 0.05 | `damping` c = 2 zeta sqrt(k m_c), m_c = mean node mass at the ends of the `crush` beams | zeta = 0.025 | c = 1883.279 N s/m |

- **L_c = 0.4095 m** (the 16 `crush` beams, all equal: spread 5.4e-16 of the mean, far inside the 5% rule). **m_c = 22.5833 kg** (the 24 nodes at the ends of the `crush` beams: stations 3, 4 and 5 with 36, 17 and 14.75 kg per node).
- **Starting values are reasoned, not calibrated, and were set before any run.** Energy balance: the car's kinetic energy is 0.5 x 1330 x 15.6111^2 = 162065 J (computed, Checks). R's maximum static crush at the centreline is 470 mm (`validation/README.md`, table "The two tests"), so the mean force over that crush is about 162065 / 0.470 = 345 kN. The crush front is 8 beams in parallel (4 z stations x 2 y stations); each yields at a force of about E A yield_strain, so E A yield_strain = 345000 / 8 = 43 kN per beam. With yield_strain 0.00167 (a yield stress of 0.00167 x 210 GPa = 351 MPa, **assumed**: a mild to high-strength body steel) that gives A = 43100 / (210e9 x 0.00167) = 1.23e-4 m^2, inside P1's range. zeta 0.025 is the middle of P3's range and of the 0.5 to 5% of docs/STAGE6-M4-M5.md V10 (which `validation/README.md` marks `unverified`). The estimate is good to a factor of 2 at best (`car_basic.bsn`'s own crush estimate was). n20-10 replaces these three numbers with the swept values; it may only move them inside the ranges above.
- **Nothing outside the ranges.** The table's starting values are all inside (checked below with the numbers read back from the file). The ranges themselves come from V9 and V10 of docs/STAGE6-M4-M5.md; `validation/README.md` records both as `unverified` (steel properties and welded-structure damping were outside the sources allowed for n20-02), so the **ranges are not yet backed by a read source**.
- **Not calibratable, on purpose:** geometry and masses (measured above or assumed above, fixed), every `chassis` number, `break_strain` (none), the contact constants (docs/STAGE6-M4-M5.md).

### Step limit at the top of the stiffness range

The run steps at h = 0.0001 s (`--dt 0.001 --substeps 10`, `frozen: h_s`). The sim refuses a step at or above the body's Gershgorin limit (`solver::step_error_with`). At the top of P1 (A = 0.002 m^2, k = 1025641025.641 N/m) with zeta at its top (0.05, c = 15219.196 N s/m, the worst case for the damping term) the limit is **0.000105 s**: the real binary printed it (Checks, "step limit"), and a hand calculation of the same formula (lightest mass 14.75 kg, largest per-node stiffness sum 2.0558e9 N/m at a node with two `crush` beams plus 17-odd `chassis` beams, largest damping sum 33438 N s/m) gives 0.00010462 s. So h = 0.0001 s is below it, by about 5%, and the freeze of h in docs/STAGE6-M4-M5.md does not need to change. The margin is thin: any change that lowers the lightest node mass or raises the number of beams at a node needs this check again.

## Checks

Run from the Broadsword root; `broadsword` = `build/bin/broadsword.exe`; the task is t-20261121-001109-2. These were run in this task; their outputs are below.

**Read-back from the files** (a Python script, scratch, independent of the generator that wrote the files; it parses the file text):

```
nodes 48 beams 296 sprung kg 1210.0 wheels kg 120.0 total kg 1330.0
length m 4.364 front x 2.103 rear x -2.261
front axle x 1.284 rear axle x -1.284 wheelbase 2.568
track m (fl z to fr z) 1.5085 (rl to rr) 1.5085
front overhang m (front-most x - front axle x) 0.819 rear overhang 0.977
body width m (z extent) 1.688
CG x 0.201 CG behind front axle m 1.083
crush beams 16 rest lengths min/max/mean 0.4095 0.4095000000000002 0.4095000000000001 spread frac 5.422334674603938e-16
crush end nodes 24 m_c 22.583333333333332
velocity ['all', '15.61111111111111', '0', '0'] 56.2
```

**The wall file is the car file plus one line:** `cmp -n <size of validation_car.bsn> validation_car.bsn validation_wall.bsn` printed nothing (equal), and the rest of the wall file is the single line `plane 2.153 0 0 -1 0 0`.

**Step limit** (`broadsword run crash-sim --task t-20261121-001109-2 -- cargo run --offline --release -- --body <variant> --input scenarios/pad_lap.input --dt 0.001 --substeps <n> --duration 0.01`, variants in the scratch folder with only the `crush` material line changed, as the sweep will make them):

```
A=0.002 zeta=0.05  substeps 10 (h=0.0001):  run: ok
A=0.002 zeta=0.05  substeps 9  (h=0.000111): crash-sim: physics step h = dt/substeps = 0.000111 s reaches the body's stability limit 0.000105 s (h^2*w2 + 2*h*g < 4 ... w2 = 278750108.6484068, g = 4534.019254237288) ... run: FAILED (exit 2)
A=0.002 zeta=0     substeps 10:                run: ok
A=0.00007 zeta=0.05 substeps 10:               run: ok
```

The substeps 9 line is the proof the limit check can fail on this body: the same file is refused one substep coarser.

**The wall file loads and the car hits the wall** (re-run on the final files): `broadsword run crash-sim --task t-20261121-001109-2 -- cargo run --offline --release -- --body scenarios/validation_wall.bsn --input scenarios/pad_lap.input --dt 0.001 --substeps 10 --duration 0.2` ended `run: ok (0.1 s)`. The rows (time s, the car's mean speed m/s) show the start at 15.6111 m/s, which matches 56.2 km/h, then the wall contact at about 3 ms (0.05 m / 15.61 m/s = 3.2 ms): 15.6109 at t = 0.003 and 13.3470 at t = 0.004, then 4.1417 at t = 0.047, and the car ends the run at 1.7932 m/s with its mean x falling (0.6769 at t = 0.10, 0.5420 at t = 0.20), that is moving backward from the wall (rebound). This is the existing CLI with gravity at its default 9.81 m/s^2 (`Config` still refuses gravity 0 until n20-07, and `--pulse-report` does not exist until n20-09), so it shows that the file parses, the car starts at test speed, and the wall stops it. It is not a pulse comparison and says nothing about how well the car matches test A.

**Not checked:** that the two files are canonical (`parse(write(x)) == x`). The crash-sim CLI has no flag that parses and re-writes a `.bsn` (its flags run or report on a body; none prints it), and no test reads `validation_*.bsn` except the carriage-return scan of `tests/format.rs`; the files were written in canonical form by construction (shortest-round-trip numbers, no comments, keywords in order), and a test task is to follow. Also not checked: that the web figures are for the exact test car (below); the position of the real accelerometer relative to the `sensor` group; anything about the pulse.

## End-state structure

*Notes only: this milestone keeps the lighter model above. Lazlo, 2026-11-19: the finished sim models the car's internal structures so they drive the crash.*

**What is known about the reference car (test A, the 2012 Accent), from R "Target Vehicle Structural Measurement" (lines 436 to 482, pre-test, mm):**

| Item | mm |
|---|---|
| 3, 4 Bumper top, bottom height | 532, 420 |
| 5 Longitudinal member top height | 498 |
| 6 Distance between longitudinal members | 725 |
| 7 Longitudinal member width | 60 |
| 8, 9 Engine top, bottom height | 766, 170 |
| 10 Engine and gearbox width | 770 |
| 11 Front bumper to engine distance | 215 |
| 12, 14 Front shock absorber fixing height, width | 832, 1120 |
| 13 Bonnet leading edge height | 746 |
| 15 Front bumper to front axle | 819 |
| 16 Front axle to A-pillar | 394 |
| 17 A-pillar to B-pillar | 1115 |
| 18 B-pillar to rear axle | 1095 |
| 19 B-pillar to C-pillar | 670 |
| 20, 21 Roof sill bottom, top height | 1320, 1429 |
| 22, 23 Floor sill bottom, top height | 175, 325 |

R's other structural table ("RSOV", distances from the rear surface to the engine, firewall, doors and pillars) came out of the text extraction with its columns scrambled, so it was not used; it is in the PDF. R does not describe the members' sections, sheet thicknesses, materials or joints. Everything below about those is general automotive knowledge, **not read from a source in this task**.

**The reference car's main internal members and joints, and what the lighter model does with each:**

| Member or joint | In the lighter model today | Data a model of it would need |
|---|---|---|
| Front rails (longitudinal members, two, 725 mm apart, 60 mm wide, top at 498 mm: R items 5 to 7) | Lumped with the crush cans and the bumper beam into the 16 `crush` longitudinal beams, 8 per segment, one material | Section outline and wall thickness along the length, material curve, the fold pattern (initiators, convolutions), the connection to the rear subframe and the firewall; the engine's position relative to them |
| Crush cans (the energy absorbers between the rails and the bumper beam) | Lumped into the same `crush` beams | Can section, thickness, material, trigger geometry, the flange and weld layout |
| Bumper beam and bumper cover (top 532 mm, bottom 420 mm: R items 3 and 4) | The 8 `front` nodes, with the wall contact on the node positions; no beam between them except the `chassis` lattice | Beam section, material, stiffness across the width (it spreads the load into the two rails), mass |
| Engine and gearbox (770 mm wide, 170 to 766 mm high, 215 mm behind the bumper: R items 8 to 11) | A share of the station masses (136 and 118 kg per station at the two front stations are assumed, not the engine's real mass), no stiffness of its own | Mass and inertia, an outline that can contact the firewall and the cross members, mounts' stiffness and failure |
| Firewall (bulkhead) and toe pan | Part of the `chassis` lattice (where it sits along x is not read from R, so no station is assigned to it) | Panel thickness, the section, the stiffness against intrusion, joints to the sills and the floor |
| A-pillar, B-pillar, C-pillar, roof rails (R items 16 to 21) | The roof-level `chassis` beams between stations | Section and material of each, the hinge pillar's load path into the sill, the roof's contribution to the stiffness of the cabin |
| Sills (floor sill 175 to 325 mm high: R items 22 and 23) | The floor-level `chassis` beams | Section, material, how they take the load from the front rails |
| Floor, tunnel, cross members (the rear seat crossmember carries the reference accelerometer) | The floor `chassis` beams; `sensor` at the right rear floor node pair | Floor panel thickness, cross member sections and positions; the exact accelerometer mount point and its local stiffness (the measured pulse has local ringing the lumped model cannot have) |
| Spot welds | Not modelled: every beam joins at a node that is rigid in position | Weld positions and counts, strength in shear and in peel (the failure that opens a flange), a failure model |
| Bolted joints (subframe, bumper, suspension mounts, engine mounts) | Not modelled; suspension mounts are the wheel's mount nodes | Bolt positions, preload, slip and shear failure, the stiffness of each mount |
| Suspension and wheels | The existing wheel, suspension and tyre sections, `car_basic.bsn` numbers | Wheel and control arm geometry for the part that is pushed back into the cabin in a full-frontal crash, the tyre's size (P175/70R14 against the file's 195/65R15) |

**The data an end-state model needs, in order:** the geometry and wall thicknesses of the members above (a teardown, a CAD file or an FE model; for a vehicle with a published FE model that gives all of it at once, but the one on this PC is the Camry's, not the Accent's, and was not opened in this task); the material curves of the steels used; weld and bolt positions and strengths; the masses of the parts. Without them the calibrated `crush` material of m4 stands in for all of the front structure together.
