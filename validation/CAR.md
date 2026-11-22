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

- **Tyres:** `car_basic.bsn`'s 195/65 R15 (radius 0.3175 m, width 0.195 m) were kept (so the tyre's outer face, 0.75425 + 0.0975 = 0.852 m from the centreline, is 8 mm outside the node grid's body width of 0.844 m; the grid has no wheel-arch geometry). The front tyres do reach the wall in the pulse window at some crush values, as the front crushes past them: at the starting values they take 5807 of the 23824 N s of wall impulse in the window, at grid point 081 334 N s, at points 070 and 140 none (`validation_wall_load_acts_below_the_centre_of_gravity` and the exploration run of t-20261121-204929; "Pitch at the wall"). The test car wore P175/70R14 (R, "Tire Size on Vehicle" P175/70R14, line 346; "Tire Width" 175, line 354; "Aspect Ratio" 70, line 356; "Wheel Diameter" 14, line 360), derived radius 14 x 25.4 / 2 + 0.70 x 175 = 300.3 mm. Not matched, because the start height rule (tyres 0.20 m above the ground, suspension at its rest length, `car_basic.bsn`) and the wheel numbers were chosen together; the tyres carry no ground load in the window (gravity 0, wheels roll freely, docs/STAGE6-M4-M5.md), though they can reach the wall (above).
- **Wheel mass 30 kg each, suspension, tyre and controls lines:** as `car_basic.bsn`, **assumed**.
- **Node masses:** the total (1330 kg) and the centre of gravity (1083 mm behind the front axle) are matched; the split between stations and between floor and roof is **assumed**. Station totals (8 nodes each, rear to front): 132, 220, 316, 288, 136, 118 kg, chosen so that the sum and the centre of gravity come out right. Within a station every roof node is 14.75 kg and the four floor nodes take the rest (floor per node, rear to front: 18.25, 40.25, 64.25, 57.25, 19.25, 14.75 kg), floor-heavy as `car_basic.bsn` (40 / 14 kg). The roof cannot be lighter than 14.75 kg because the step limit below uses the lightest mass in the body, so the split is less extreme than `car_basic.bsn`'s. Read back from the file: the car's centre of gravity is 0.920 m above the ground at the start (wheels at their start height; sprung nodes alone 0.960 m), against `car_basic.bsn`'s derived 0.52 m. It is still higher than a real car's (the lumped nodes cannot be lighter). Its effect on pitch, together with the height of the wall load, was measured in t-20261121-204929 ("Pitch at the wall").
- **Layout:** 6 stations of 8 nodes (x = -2.261, -1.284, 0, 1.284, 1.6935, 2.103; z = -0.844, -0.75425, 0.75425, 0.844; y = 0.55, 1.95), node id = 8 ix + 2 iz + iy, as `car_basic.bsn` with one more station in the front overhang, so the crush zone is two equal segments of 0.4095 m. The two outer z stations are at the body width (1.688 m); the inner two are the wheel mounts' z (1.5085 m track). 296 beams: all 28 pairs inside each of the 15 cells, shared-face pairs once, as `car_basic.bsn`.
- **Groups:** `all` (48 nodes, carries the velocity), `front` (the 8 nodes at x = 2.103), `rear` (the 8 at x = -2.261), `sensor` (nodes 12 and 14: the two floor nodes at the rear axle on the right, z = +0.75425 and +0.844). `sensor` is the right rear floor, placed at the rear axle because the channel is "FLOORPAN - RIGHT REAR", ISO name "Floor over Rear Axle", comment "RIGHT REAR SEAT CROSSMEMBER X" (`validation/README.md`, "Channel details"). It is 3.387 m behind the front-most node (2.103 - (-1.284)), outside the crush zone. *Assumed:* that the crossmember is within a station of the rear axle (R gives B-pillar to rear axle 1095 mm, item 18, so the rear seat is between the two but its position is not stated).
- **Where the velocity line sits:** docs/FORMAT.md puts `velocity` before the car sections and `plane` last, so the start velocity is in `validation_car.bsn` (the car at test speed) and `validation_wall.bsn` is that file followed by the one `plane` line. `cmp` shows the wall file starts with the car file byte for byte (Checks).

## Calibratable parameters and their ranges

**Since t-20261121-231938 the `crush` material below is two stages, `crush_front` (the 6 beams of stations 4 to 5) and `crush_rear` (the 6 of stations 3 to 4), the rear twice as strong, three times since t-20261122-042859 ("The two-stage crumple zone" and "Calibration step 7" below, which has their numbers; the table and the L_c / m_c bullet here are of the one-material zone before it).** Only the material `crush` is calibratable, through three physical quantities (docs/STAGE6-M4-M5.md, "What is calibrated, and its physical range"; the `frozen: range` lines 143 to 145 are the limits, read here from that file at this commit). The 12 `crush` beams are longitudinal beams of the two front segments (node `a` to node `a + 8` for `a` in stations 3 and 4): the four floor-level ones (y = 0.55) of each segment and the two roof-level ones (y = 1.95) at the body sides (z = -0.844 and +0.844). The four roof-level longitudinal beams over the wheel mounts (z = -0.75425 and +0.75425: 27-35, 29-37, 35-43, 37-45) are `chassis`, so that the wall load acts low enough for the car not to pitch nose up ("Pitch at the wall" below; until t-20261121-204929 all 16 longitudinal beams were `crush`). The 184 beams of the passenger cell (both ends at or behind x = 1.284, stations 0 to 3) are `cabin`, a fixed design value ("The passenger cell" below), since t-20261121-225636. Every other beam is `chassis`, which is not calibratable and stays at `car_basic.bsn`'s numbers (stiffness 300000 N/m, damping 200 N s/m, yield strain 0.02).

| Quantity | Physical range (frozen:) | Becomes in the file | Starting value in the file | Result in the file |
|---|---|---|---|---|
| P1 `area_m2`, steel cross-section one crush beam stands for | 0.00007 to 0.002 m^2 | `stiffness` k = E A / L_c, E = 210e9 Pa (`frozen: steel_E_pa`) | A = 0.0001225 m^2 | k = 62820512.821 N/m |
| P2 `yield_strain` | 0.0010 to 0.0057 | `yield_strain` as is | 0.00167 | `yield_strain 0.00167` |
| P3 `damping_ratio` | 0 to 0.05 | `damping` c = 2 zeta sqrt(k m_c), m_c = mean node mass at the ends of the `crush` beams | zeta = 0.025 | c = 1989.176 N s/m |

- **L_c = 0.4095 m** (the 12 `crush` beams, all equal: `crush_beams_one_length` printed `validation_car.bsn: 12 crush beams, 0.4095 to 0.4095000000000002 m`, far inside the 5% rule). **m_c = 25.1944 kg** (the 18 nodes at the ends of the `crush` beams: the floor nodes of stations 3, 4 and 5, 57.25, 19.25 and 14.75 kg, and the six outer roof nodes of those stations, 14.75 kg; the script that applied the layout printed `A: m_c 25.194444444444443 over 18 end nodes, damping 1989.176, P3 0.025000004291247345`). With the 16-beam layout m_c was 22.5833 kg over 24 nodes and c was 1883.279 N s/m; k is unchanged because L_c is.
- **Starting values are reasoned, not calibrated, and were set before any run.** Energy balance: the car's kinetic energy is 0.5 x 1330 x 15.6111^2 = 162065 J (computed, Checks). R's maximum static crush at the centreline is 470 mm (`validation/README.md`, table "The two tests"), so the mean force over that crush is about 162065 / 0.470 = 345 kN. The crush front was 8 beams in parallel (4 z stations x 2 y stations) when this estimate was made; since t-20261121-204929 it is 6 (4 floor, 2 roof), and A was left at this estimate, not re-derived, because the calibration replaces it. Each yields at a force of about E A yield_strain, so E A yield_strain = 345000 / 8 = 43 kN per beam. With yield_strain 0.00167 (a yield stress of 0.00167 x 210 GPa = 351 MPa, **assumed**: a mild to high-strength body steel) that gives A = 43100 / (210e9 x 0.00167) = 1.23e-4 m^2, inside P1's range. zeta 0.025 is the middle of P3's range and of the 0.5 to 5% of docs/STAGE6-M4-M5.md V10 (which `validation/README.md` marks `unverified`). The estimate is good to a factor of 2 at best (`car_basic.bsn`'s own crush estimate was). n20-10 replaces these three numbers with the swept values; it may only move them inside the ranges above.
- **Nothing outside the ranges.** The table's starting values are all inside (checked below with the numbers read back from the file). The ranges themselves come from V9 and V10 of docs/STAGE6-M4-M5.md; `validation/README.md` records both as `unverified` (steel properties and welded-structure damping were outside the sources allowed for n20-02), so the **ranges are not yet backed by a read source**.
- **Not calibratable, on purpose:** geometry and masses (measured above or assumed above, fixed), every `chassis` number, `break_strain` (none), the contact constants (docs/STAGE6-M4-M5.md).

### Step limit at the top of the stiffness range

The run steps at h = 0.0001 s (`--dt 0.001 --substeps 10`, `frozen: h_s`). The sim refuses a step at or above the body's Gershgorin limit (`solver::step_error_with`). At the top of P1 (A = 0.002 m^2, k = 1025641025.641 N/m) with zeta at its top (0.05, c = 15219.196 N s/m, the worst case for the damping term) the limit is **0.000105 s**: the real binary printed it (Checks, "step limit"), and a hand calculation of the same formula (lightest mass 14.75 kg, largest per-node stiffness sum 2.0558e9 N/m at a node with two `crush` beams plus 17-odd `chassis` beams, largest damping sum 33438 N s/m) gives 0.00010462 s. So h = 0.0001 s is below it, by about 5%, and the freeze of h in docs/STAGE6-M4-M5.md does not need to change. The margin is thin: any change that lowers the lightest node mass or raises the number of beams at a node needs this check again. **Re-checked for the 12-beam layout** (t-20261121-204929): m_c rose to 25.1944 kg, so c at the top of the range rose to 16074.967 N s/m; the same command on that variant (A = 0.002 m^2, zeta = 0.05) printed `run: ok (0.1 s)` at `--substeps 10`, and at `--substeps 9` `crash-sim: physics step h = dt/substeps = 0.000111 s reaches the body's stability limit 0.000104 s (h^2*w2 + 2*h*g < 4 with the Gershgorin bounds w2 = 278750108.6484068, g = 4766.092745762712)` and `run: FAILED (exit 2)`. The limit is now **0.000104 s**, still above h = 0.0001 s. **Re-checked for the `cabin` material** (t-20261121-225636): at k = 1e6, c = 666.667 (the files) the largest per-node damping sum is again at a crush node, so the limit stays **0.000104 s**. The same command on that variant (`scratch/t-20261121-225636/top1e6.bsn`, the wall file with only the `crush` line at A = 0.002 m^2, zeta = 0.05, yield strain 0.0057) printed `run: ok (0.2 s)` at `--substeps 10`, and at `--substeps 9` `crash-sim: physics step h = dt/substeps = 0.000111 s reaches the body's stability limit 0.000104 s (h^2*w2 + 2*h*g < 4 with the Gershgorin bounds w2 = 278750108.6484137, g = 4766.09269813392)` and `run: FAILED (exit 2)`. A beams-only reimplementation of the formula (`scratch/t-20261121-225636/gersh.py`) gives the same w2 and g, limit 0.000103907 s. (Round 1's k = 3e6, c = 2000 had lowered it to 0.000102 s, g = 5298.300586355095.)

## Checks

Run from the Broadsword root; `broadsword` = `build/bin/broadsword.exe`; the task is t-20261121-001109-2. These were run in this task; their outputs are below. They are of the 16-beam layout: the read-back's `crush beams 16` and `m_c 22.583333333333332` and the step limit 0.000105 s were superseded by t-20261121-204929 (12 beams, m_c 25.1944 kg, limit 0.000104 s; "Pitch at the wall" and the step limit section above). Nodes, masses, dimensions and the velocity did not change.

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

**Canonical form:** `every_scenario_file_writes_back_byte_for_byte` (`tests/format.rs`) parses every `scenarios/*.bsn`, writes it back and compares byte for byte; it expects 14 files, 12 identical and 2 pinned, so `validation_car.bsn` and `validation_wall.bsn` are counted among the 12 identical (its output is in the task report). **Not checked:** that the web track figures are for the exact test car; the position of the real accelerometer relative to the `sensor` group; anything about the pulse.

## Pitch at the wall

*Task t-20261121-204929, after the m4 sweep (task t-20261121-202636) failed on ground contact.* With `crush` beams on every floor and roof line, the front station's floor (y = 0.55) and roof (y = 1.95) nodes take the wall load alike, and the car pitched nose up: its rear tyres, 0.20 m above the ground at the start, came down to it inside the window. The layout was changed (no node, mass, group, velocity, wheel or wall line changed; only four beams' material and the `crush` damping) so that the roof-level `crush` beams over the wheel mounts are `chassis`; the floor-level ones and the roof-level ones at the body sides stay `crush`. `validation_car_b.bsn` got the same change (its beams 27-35, 29-37, 35-43, 37-45), with its damping set from A's P3 as before (`validation/CAR_B.md`); test B was not run.

**How it was measured.** Each layout was run with the 4.4 command (test A, `frozen: run_flags` unchanged) for `ground_contacts_in_window`, and once more with the same `--gravity 0 --duration 0.2 --dt 0.001 --substeps 10` and `--input` of a file holding the one line `0 0 0 0` (no throttle, brake or steer) and `--trajectory-v2`, which prints every node's position each 1 ms. A scratch script (`scratch/t-20261121-204929/pitch.py`) fits a line to y against x over the 32 nodes of stations 0 to 3 (every node behind the crush zone) and gives its angle minus the angle at t = 0: positive is nose up. The crush values are the starting ones (above) or a grid point of the sweep (`scratch/m4-sweep/point-<n>.bsn`, copied, with P1 to P3 computed back from that file and the damping recomputed for this layout's m_c). Outputs: `scratch/t-20261121-204929/<run>-4.4.txt`, `-traj.txt`, `-pitch.txt`.

**The wall load's height, measured (round 2 of the task).** `wall_load_of` in `tests/validation_files.rs` runs a wall file as the 4.4 command does (gravity 0, h = `frozen: h_s`, `frozen: duration_s`, one substep at a time), checks its run bit for bit against `Sim::advance` at every substep (so the measured run is the real one), and sums over the pulse window (contact to contact + `frozen: window_s`, 0.0033 to 0.1533 s here) the wall's normal impulse J on each node (its momentum change in the wall's contact pass) and on each tyre (its wall load times h). The **resultant height** is sum(J y) / sum(J), y the node's or wheel centre's height at that substep; the **CG height** is the sprung centre of gravity's height (mass-weighted node y) at each substep, weighted by the same J. The same run gives the ground-contact count by the 4.4 report's rule (its numbers below equal the 4.4 lines) and the passenger-cell pitch (the fit above) at **every substep** from contact to the window end; "pitch" below is its largest and smallest value there. The test `validation_wall_load_acts_below_the_centre_of_gravity` requires resultant < CG and 0 ground contacts for `validation_wall.bsn`; `validation_checks_reject_wrong_copies` requires the 16-beam layout to fail it. The other rows were measured with the same function on the variant files in `scratch/t-20261121-204929/` (output `r2/explore-summary.txt`).

| Layout | Crush values | `ground_contacts_in_window` | Last line | Resultant / CG height (m) | Pitch in the window (deg, every substep) |
|---|---|---|---|---|---|
| 16 beams (before) | starting | 479 | `pulse: FAIL ground contact at t=0.1022` | 1.1622 / 0.9587 (above) | +9.654 at 0.1533 s to -0.024 |
| **12 beams, roof crush at the sides (the files now)** | starting | 0 | `pulse: FAIL dv_mps norm=2.8663` | **0.8261 / 0.9591** (0.133 below) | +0.862 at 0.0534 s to -0.976 at 0.1533 s |
| same | point 001 (P1 7e-05, P2 0.001, P3 0) | 0 | `pulse: FAIL dv_mps norm=3.6230` | 0.8946 / 0.9586 | +0.838 to -0.017 |
| same | point 070 (P1 0.000374, P2 0.00335, P3 0.01) | 0 | `pulse: FAIL dv_mps norm=1.2688` | 0.8259 / 0.9594 | +1.616 to -1.470 |
| same | point 081 (P1 0.000654, P2 0.001, P3 0; the sweep's best score) | 0 | `pulse: FAIL dv_mps norm=1.2594` | 0.8287 / 0.9594 | +1.520 to -1.354 |
| same | point 140 (P1 0.002, P2 0.0057, P3 0.05) | 0 | `pulse: FAIL dv_mps norm=1.3905` | 0.8418 / 0.9593 | +1.941 to -0.915 |
| 8 beams, floor only | starting | 0 | `pulse: FAIL dv_mps norm=2.9656` | 0.7995 / 0.9600 | +0.441 to -4.341 |
| same | point 081 | 284 | `pulse: FAIL ground contact at t=0.1217` | 0.7179 / 0.9603 | +1.049 to -6.533 (nose down) |
| same | point 140 | 289 | `pulse: FAIL ground contact at t=0.1212` | 0.7323 / 0.9596 | +1.267 to -6.402 |
| 12 beams, roof crush over the wheel mounts | point 081 | 218 | `pulse: FAIL ground contact at t=0.1283` | 1.0831 / 0.9595 (above) | +8.080 at 0.1533 s (nose up) |
| same | point 140 | 223 | `pulse: FAIL ground contact at t=0.1278` | not measured | +7.993 at 0.15 s (10 ms rows of round 1 only) |

**Result.** With the chosen layout the measured wall load acts below the sprung centre of gravity at every crush value tried: 0.133 m below at the starting values, 0.064 to 0.134 m below at the four grid points; the two layouts with the roof crush over the wheel mounts loaded the wall above it (by 0.20 and 0.12 m) and pitched the car nose up onto its rear tyres. At the starting values the front tyres take 5807 of the 23824 N s of wall impulse, at the wheel centre height, which lowers the resultant; at points 070 and 140 they take none and the nodes' load alone is 0.13 and 0.12 m below the CG. The car stays level, within +1.941 and -1.470 degrees in the window at every substep and every value tried, turning slightly nose down by the window's end, with no ground contact. Floor-only crush beams put the load lower still and pitched the car nose down hard enough that, at the stiffer points, the front came down to the ground instead. The load is not in proportion to the beam count: a count of the chosen layout's crush beams, (4 x 0.55 + 2 x 1.95) / 6 = 1.017 m, is above the CG, the measured resultant is not. Every run above still fails a pulse metric (`dv_mps`); the fit is the calibration's to redo (re-run from task t-20261121-202636), not this section's.

## The passenger cell

*Task t-20261121-225636 (calibration redesign, step 1), the rule in docs/STAGE6-M4-M5.md, "The passenger cell (`cabin`), a fixed design value", approved by Lazlo 2026-11-21 22:56Z.* The diagnosis of t-20261121-221015 found that the passenger cell, at the `chassis` numbers, folded before the crush zone did and capped the wall force below what test A's reference peak needs. The cell's beams now carry a fixed material, not calibrated:

`material cabin stiffness 1000000 damping 666.667 yield_strain 0.02` (line 3 of `scenarios/validation_car.bsn`; damping 6666.67 since t-20261122-042859, "Calibration step 7" below)

- **Which beams:** every beam with both ends at or behind the rearmost end of a `crush` beam (x <= 1.284, stations 0 to 3, the front axle station included): 184 beams, the same 184 the diagnosis stiffened. The crush-zone `chassis` beams (cross beams of stations 4 and 5 and every beam between stations 3 and 5 that is not `crush`) stay `chassis`: stiffening those too pitched the car onto the ground (207 ground contacts, diagnosis variant `p044_ch10`).
- **Numbers:** the rule is a cell at about 2 x the needed force (Lazlo's approval). k is the smallest round value at or above the floor the rule sets (below: 936 kN/m), k = 1000000 N/m; damping 666.667 N s/m, `chassis`'s ratio c / k = 200 / 300000 kept; yield strain 0.02, as `chassis`. **Derived from the rule, not tuned to a pulse.** (Round 1 of this task used k = 3000000 N/m, the diagnosis's "cabin 10x" value, ratio 6.408; it was brought down to about 2x in round 2, and both were measured below.)
- **Gap capacity, derived from the file's own values** (reproduce: for each pair of neighbouring stations, the beams with one end on each, times k x yield_strain x |dx|; `cabin_gap_capacity_at_least_factor_times_needed` in `tests/validation_files.rs` prints these lines). Each gap has 40 beams across it (3 cells of 16 cross pairs, less the 8 pairs on the two shared faces), and every one of them has |dx| equal to the gap's length, so a gap's capacity is 40 x k x 0.02 x dx. The needed force is the mass behind the gap (its nodes, and the rear wheels for the gaps ahead of station 1, where they mount) times `frozen: cabin.design_accel_mps2` = 502.6 m/s^2 (test A's reference peak, CFC 60, `ref=502.6` in the 4.4 report).

| Gap (x, m) | dx (m) | Capacity 40 x 1e6 x 0.02 x dx | Mass behind | Needed (x 502.6) | Before (`chassis`) |
|---|---|---|---|---|---|
| -2.261 to -1.284 | 0.977 | 781600 N | 132 kg (station 0) | 66343 N | 234480 N |
| -1.284 to 0 | 1.284 | 1027200 N | 412 kg (stations 0, 1 + 2 rear wheels x 30) | 207071 N | 308160 N |
| 0 to 1.284 | 1.284 | 1027200 N | 728 kg (stations 0 to 2 + rear wheels) | 365893 N | 308160 N |

  The rule asks the weakest gap to carry `frozen: cabin.capacity_factor` = 2 times the largest needed force: 2 x 365893 = 731786 N, so k >= 731786 / (40 x 0.02 x 0.977) = 936 kN/m. With k = 1e6 the weakest gap carries 781600 N, **2.136 times** the largest needed force (the test prints `weakest capacity 781600 N, largest needed 365893 N, ratio 2.136, required 2`).
- **The test fails on the files before this change (fail-first, reproducible):** with this task's `tests/validation_files.rs` and `docs/STAGE6-M4-M5.md` and the four scenario files as on the branch's base (`git stash push -- scenarios/`, then `broadsword run crash-sim --task t-20261121-225636 -- cargo test --offline --test validation_files cabin_gap`, then `git stash pop`; output `scratch/t-20261121-225636/failfirst.txt`) it printed:

```
test cabin_gap_capacity_at_least_factor_times_needed ... FAILED
validation_car.bsn: the cell's weakest gap carries 234480 N, under 2 x the needed 365893 N
gap x -2.2610..-1.2840: 40 beams, capacity 234480 N, mass behind 132.00 kg, needed 66343 N
gap x -1.2840..0.0000: 40 beams, capacity 308160 N, mass behind 412.00 kg, needed 207071 N
gap x 0.0000..1.2840: 40 beams, capacity 308160 N, mass behind 728.00 kg, needed 365893 N
weakest capacity 234480 N, largest needed 365893 N, ratio 0.641, required 2
test result: FAILED. 0 passed; 1 failed; 0 ignored; 0 measured; 8 filtered out
```

  `validation_checks_reject_wrong_copies` also sets the cabin back to the `chassis` numbers in a copy of the current file and requires the same failure.
- **The cell holds at about 2x, measured on test A** (the starting crush values; `--gravity 0 --duration 0.2 --dt 0.001 --substeps 10 --input <file with the line 0 0 0 0> --trajectory-v2` on the wall file, every node each 1 ms, `scratch/t-20261121-225636/cell.py`; window 0.003 to 0.153 s). Cell shortening = mass-weighted x centroid of station 3 minus station 0 (3.5450 m at the start), at the start minus at t:

| k (N/m) | Shortening, max in the window | At the window end | Before (`chassis`, diagnosis p044) |
|---|---|---|---|
| 1000000 (the files) | 0.0315 m at 0.018 s | -0.0092 m (recovered: elastic) | 0.3873 m at 0.097 s, 0.3719 m at the end |
| 3000000 (round 1) | 0.0101 m at 0.012 s | -0.0107 m | |

  At both values the cell takes no lasting shortening along x (the diagnosis's cell lost 0.387 m). Some short cross beams (0.09 m, along z, between the outer and the wheel-mount z) pass the yield strain as the cell bulges sideways: 4 at k = 3e6 (station 3's, 24-26, 25-27, 28-30, 29-31), 12 at k = 1e6 (those 4, up to 0.736 strain, and the same pairs at stations 1 and 2, up to 0.127). They are cross beams, not beams across a gap, so they are outside the section capacity above; noted for step 2.
- **Step limit:** re-checked at k = 1e6: 0.000104 s, as before the cabin (the largest damping sum is again at a crush node), against h = 0.0001 s ("Step limit at the top of the stiffness range").
- **Test A at the starting crush values, after the change** (the 4.4 command, `frozen: run_flags` unchanged, output `scratch/t-20261121-225636/run-a-1e6.txt`): `ground_contacts_in_window=0`; `metric dv_mps sim=17.9456 ref=16.1958 err=+0.1080 norm=1.8006 FAIL`, `metric peak_mps2 sim=494.6 ref=502.6 err=-0.0158 norm=0.0879 ok`, `metric peak_time_s sim=0.0144 ref=0.0314 err=-0.0170 norm=0.7727 ok`, `metric duration_s sim=0.1393 ref=0.0748 err=+0.8623 norm=8.6230 FAIL`, `metric olc_mps2 sim=198.7 ref=347.8 err=-0.4286 norm=2.8575 FAIL`, `metric static_crush_m sim=0.5816 ref=0.3508 err=+0.6580 norm=3.2899 FAIL`, `pulse_score=2.9053`, `pulse: FAIL dv_mps norm=1.8006`. (At k = 3e6, round 1, `run-a.txt`: peak 463.4 ok, peak time 0.0087 FAIL, score 3.2007.) These are the uncalibrated starting values; the crush beams are re-swept in step 3 and the timing is step 2's (the two-stage crumple zone). Not tuned here.
- **Wall load, measured** (`validation_wall_load_acts_below_the_centre_of_gravity`): `resultant_y_m=0.9056 cg_y_m=0.9594 arm_m=-0.0538 node_impulse_ns=18566 tyre_impulse_ns=5129 ground_contacts=0 pitch_deg max +1.334 at 0.1313 s, min -0.004 at 0.0068 s`. The resultant is still below the centre of gravity, by 0.054 m against 0.133 m before the cabin, and the cell turns slightly nose up (+1.334 degrees) instead of nose down; the 16-beam layout still fails the check (resultant 1.1443 against 0.9593, 483 ground contacts). The calibration should watch this margin.

## The two-stage crumple zone

*Task t-20261121-231938 (calibration redesign, step 2), the rule in docs/STAGE6-M4-M5.md, "The two-stage crumple zone (`crush_front`, `crush_rear`), a fixed strength ratio", approved by Lazlo 2026-11-21 22:56Z.* The 12 crush beams were one material; they are now two stages in series (lines 4 and 5 of `scenarios/validation_car.bsn`):

```
material crush_front stiffness 62820512.821 damping 1597.524 yield_strain 0.00167
material crush_rear stiffness 125641025.642 damping 3090.947 yield_strain 0.00167
```

- **Which beams:** `crush_rear` is the 6 beams of the rear segment, stations 3 to 4 (24-32, 25-33, 26-34, 28-36, 30-38, 31-39); `crush_front` the 6 of the front segment, stations 4 to 5 (32-40, 33-41, 34-42, 36-44, 38-46, 39-47), which the wall reaches first. Every other beam is unchanged: the soft `chassis` cross beams of stations 4 and 5 are kept, and the `cabin` line and its 184 beams are as step 1 left them.
- **The ratio, 2 (`frozen: crush.rear_to_front_strength`), and why:** the rear stage must stay below its yield while the front folds. The force the front passes into station 4 is its yield force plus its damping force: 43.0 kN per beam (210e9 x 0.0001225 x 0.00167) plus 1597.5 x 15.6 = 24.9 kN at the impact speed, 1.58 times the yield force; 2 is the smallest round number above it. Derived at the starting values, set before any run; the reasoning is in docs/STAGE6-M4-M5.md.
- **The starting values, from the one-material line scaled by the ratio** (`scratch/t-20261121-231938/stages.py`, output `stages.txt`): the front stage keeps the old line's P1 = 0.0001225 m^2, P2 = 0.00167 and P3 = 0.025; the rear has P1 = 2 x 0.0001225 = 0.000245 m^2 with the same P2 and P3. (Front at the old value and rear doubled, not the other way round, because half the old area, 0.00006125 m^2, is below P1's range, 0.00007 m^2.) Each stage's damping is c = 2 zeta sqrt(k m_c) with its own m_c, the mean mass of the 12 distinct nodes at its beams' ends: front 16.25 kg (stations 4 and 5: 4 x 19.25, 4 x 14.75 and 4 roof x 14.75), rear 30.4167 kg (stations 3 and 4: 4 x 57.25, 4 x 19.25, 4 roof x 14.75). L_c = 0.4095 m for both. Computed back by the test: front P1 0.00012250000000095008, P3 0.024999998027214; rear P1 0.0002450000000019, P3 0.025000000477919865; both inside the `frozen: range` lines; P1 x P2 rear / front = 1.9999999999999987 against the frozen 2.
- **Calibration:** each stage's P1 to P3 remain the calibration quantities (step 3); the ratio is fixed, so at most half of P1's range top, 0.001 m^2, is open to the front stage when the rear sits at 0.002 m^2.
- **Step limit** (a calculation, `scratch/t-20261121-231938/gersh2.py`, the beams-only formula of `step_error_with` as step 1's `gersh.py`): at the top of the ranges the ratio allows (rear A = 0.002 m^2, front 0.001 m^2, zeta 0.05 for both) w2 = 209215123.859, g = 4039.493, **limit 0.000120 s**, against 0.000104 s for the one-material zone at its top. Only the front stage reaches the lightest nodes of station 5, and it is now the weaker one, so the margin over h = 0.0001 s grew.
- **The new test fails on main's files (fail-first):** with this task's `tests/validation_files.rs` and `docs/STAGE6-M4-M5.md` committed first (commit bf621c8) and the four scenario files still as on main, `broadsword run crash-sim --task t-20261121-231938 -- cargo test --offline --test validation_files crush_rear_stronger` (output `scratch/t-20261121-231938/failfirst.txt`) printed:

```
test crush_rear_stronger_than_front_by_frozen_ratio ... FAILED
validation_car.bsn: no material `crush_front`
test result: FAILED. 0 passed; 1 failed; 0 ignored; 0 measured; 9 filtered out; finished in 0.00s
```

  After the scenario change it passes, printing `validation_car.bsn: crush_front x 1.6935..2.1030, P1 x P2 2.0457500000158664e-7; crush_rear x 1.2840..1.6935, P1 x P2 4.09150000003173e-7; rear / front 1.9999999999999987, frozen 2` and the same for B. `validation_checks_reject_wrong_copies` also requires the check to fail on a rear stage halved back to the front's strength (`strength ratio`), on the two stage names swapped (`are not all at or ahead of`), and on the one-material file (``no beam uses the material `crush_front` ``).
- **Test A at the starting values, after the change** (the 4.4 command: `broadsword run crash-sim --task t-20261121-231938 -- cargo run --offline --release -- --body scenarios/validation_wall.bsn --pulse-report validation/test-a.pulse.csv <frozen: run_flags>`, output `scratch/t-20261121-231938/run-a.txt`): `contact_s=0.0033`, `ground_contacts_in_window=0`, `metric dv_mps sim=16.9221 ref=16.1958 err=+0.0448 norm=0.7473 ok`, `metric peak_mps2 sim=610.5 ref=502.6 err=+0.2148 norm=1.1931 FAIL`, `metric peak_time_s sim=0.0509 ref=0.0314 err=+0.0195 norm=0.8864 ok`, `metric duration_s sim=0.1197 ref=0.0748 err=+0.6003 norm=6.0027 FAIL`, `metric olc_mps2 sim=260.6 ref=347.8 err=-0.2508 norm=1.6720 FAIL`, `metric static_crush_m sim=0.4717 ref=0.3508 err=+0.3446 norm=1.7230 FAIL`, `info dyn_crush_m sim=0.5320 ref=0.5386`, `pulse_score=2.0374`, `pulse: FAIL peak_mps2 norm=1.1931`. Against step 1 (one material, same front values): peak 494.6 at 14.4 ms to 610.5 at 50.9 ms, duration 0.1393 to 0.1197 s, dv +10.8% (FAIL) to +4.5% (ok), OLC 198.7 to 260.6, static crush 0.5816 to 0.4717 m, score 2.9053 to 2.0374. Uncalibrated starting values: not tuned here; step 3 re-sweeps.
- **Wall load, measured** (`validation_wall_load_acts_below_the_centre_of_gravity`, output `scratch/t-20261121-231938/vf.txt`): `resultant_y_m=0.8663 cg_y_m=0.9596 arm_m=-0.0933 node_impulse_ns=22130 tyre_impulse_ns=313 ground_contacts=0 pitch_deg max +0.742 at 0.0634 s, min -0.777 at 0.1533 s`. The resultant is 0.093 m below the centre of gravity (0.054 m after step 1). The 16-beam layout (the four roof beams over the wheel mounts made crush beams of their own segment's stage) still fails the check: resultant 1.0363 against 0.9595, 207 ground contacts.
- **When each stage crushes, measured** (`--gravity 0 --duration 0.2 --dt 0.001 --substeps 10 --input <file with the line 0 0 0 0> --trajectory-v2` on the wall file, `scratch/t-20261121-231938/stage.py`, output `stage.txt`; a stage's shortening is the change in the distance between the mass-weighted x centroids of its two stations):

| t (s) | Front stage (m) | Rear stage (m) | Cell (m) |
|---|---|---|---|
| 0.010 | 0.0921 | 0.0043 | 0.0167 |
| 0.020 | 0.1940 | 0.0112 | 0.0326 |
| 0.030 | 0.2895 | 0.0231 | 0.0019 |
| 0.040 | 0.3700 | 0.0316 | -0.0051 |
| 0.045 | 0.3895 | 0.0429 | 0.0080 |
| 0.050 | 0.3999 | 0.0462 | 0.0331 |
| 0.060 | 0.4104 | 0.0699 | 0.0365 |
| 0.070 | 0.4027 | 0.1106 | 0.0027 |
| 0.100 | 0.3817 | 0.1215 | 0.0013 |

  The front stage crushes first and the rear takes up most of its crush (0.043 to 0.111 m) from 0.045 to 0.070 s, once the front has crushed about 0.39 m: the second stage's onset is at the depth asked for (0.3 to 0.4 m), and the sensor's peak follows it at 50.9 ms. In time the onset is later than the 25 to 35 ms the brief aimed at (the front's force sets how fast the front is used up; that is step 3's calibration). **Finding for step 3:** the front stage folds completely: at 0.060 s its shortening, 0.4104 m, exceeds its length, 0.4095 m (station 5's centroid passes station 4's by 0.9 mm), so the rear stage engages when the front bottoms out, not by the strength ratio alone. The rear creeps by 0.01 m by 0.019 s, while the front has crushed 0.18 m.

## Compressive yield at constant force

*Task t-20261122-013607 (calibration redesign, step 4). The model change and Lazlo's approval of 2026-11-22 01:34Z are in docs/STAGE6-M4-M5.md, "Compressive yield at constant force".* A crushed beam now flows at k y rest0 (rest' = len + y rest0) instead of rest' = len / (1 - y). The car files are unchanged.

- **Test A at the current crush values, after the change.** This is the 4.4 command: `broadsword run crash-sim --task t-20261122-013607 -- cargo run --offline --release -- --body scenarios/validation_wall.bsn --pulse-report validation/test-a.pulse.csv <frozen: run_flags>`, with the flags read from that line. Output is in `scratch/t-20261122-013607/run-a.txt`: `contact_s=0.0033`, `ground_contacts_in_window=0`, `metric dv_mps sim=17.8202 ref=16.1958 err=+0.1003 norm=1.6716 FAIL`, `metric peak_mps2 sim=627.9 ref=502.6 err=+0.2494 norm=1.3857 FAIL`, `metric peak_time_s sim=0.0145 ref=0.0314 err=-0.0169 norm=0.7682 ok`, `metric duration_s sim=0.1274 ref=0.0748 err=+0.7032 norm=7.0321 FAIL`, `metric olc_mps2 sim=276.8 ref=347.8 err=-0.2041 norm=1.3605 FAIL`, `metric static_crush_m sim=0.3838 ref=0.3508 err=+0.0940 norm=0.4701 ok`, `info dyn_crush_m sim=0.4916 ref=0.5386`, `pulse_score=2.1147`, `pulse: FAIL dv_mps norm=1.6716`. These are uncalibrated starting values: the crush materials were not tuned for the new law, and step 5 re-sweeps.
- **Wall load, measured** (`validation_wall_load_acts_below_the_centre_of_gravity`, output `scratch/t-20261122-013607/vf.txt`): `resultant_y_m=0.8572 cg_y_m=0.9597 arm_m=-0.1025 node_impulse_ns=22970 tyre_impulse_ns=0 ground_contacts=0 pitch_deg max +0.592 at 0.0764 s, min -1.174 at 0.1533 s`. The resultant is 0.103 m below the centre of gravity (it was 0.093 m after step 2).

## Calibration step 7: ratio 3, cabin damping 10x, six-point static crush

*Task t-20261122-042859. The three rule changes and Lazlo's approvals of 2026-11-22 (04:28Z, and 03:12Z for the test A runs and the B car files) are in docs/STAGE6-M4-M5.md: "The passenger cell" (cabin damping), "The two-stage crumple zone" (the ratio) and "Static crush as the six-point mean".* Lines 3 to 5 of `scenarios/validation_car.bsn` (and of `validation_wall.bsn`, which still differs only by its `plane` line):

```
material cabin stiffness 1000000 damping 6666.67 yield_strain 0.02
material crush_front stiffness 62820512.821 damping 1597.524 yield_strain 0.00167
material crush_rear stiffness 188461538.463 damping 3785.621 yield_strain 0.00167
```

- **What changed:** `cabin` damping 666.667 -> 6666.67 N s/m (stiffness unchanged); `crush_rear` stiffness 3 x the front's (was 2 x; one rest length, 0.4095 m, so P1 x P2 is 3 x: the test prints `rear / front 2.999999999999998, frozen 3`), its damping recomputed for the same damping ratio (3090.947 x sqrt(1.5) = 3785.621); `crush_front` unchanged. The crush values were not tuned here.
- **Test A at today's crush values** (the 4.4 command: `broadsword run crash-sim --task t-20261122-042859 -- cargo run --offline --release -- --body scenarios/validation_wall.bsn --pulse-report validation/test-a.pulse.csv ` `<frozen: run_flags>`, with the flags read from that line; output `scratch/t-20261122-042859/run-a.txt`): `contact_s=0.0033`, `ground_contacts_in_window=0`, `metric dv_mps sim=17.2562 ref=16.1958 err=+0.0655 norm=1.0912 FAIL`, `metric peak_mps2 sim=425.2 ref=502.6 err=-0.1540 norm=0.8556 ok`, `metric peak_time_s sim=0.0115 ref=0.0314 err=-0.0199 norm=0.9045 ok`, `metric duration_s sim=0.0845 ref=0.0748 err=+0.1297 norm=1.2968 FAIL`, `metric olc_mps2 sim=284.2 ref=347.8 err=-0.1828 norm=1.2187 FAIL`, `metric static_crush_m sim=0.3720 ref=0.3508 err=+0.0605 norm=0.3027 ok`, `info static_crush_centroid_m sim=0.3725`, `info dyn_crush_m sim=0.4678 ref=0.5386`, `pulse_score=0.9449`, `pulse: FAIL dv_mps norm=1.0912`.
- **Test A at the review's probe q5** (`scratch/t-20261122-042859/q5.bsn`: the wall file with `crush_front stiffness 35897436 damping 0 yield_strain 0.0022`, P1 = 7e-5 m^2, and `crush_rear stiffness 107692308 damping 0 yield_strain 0.0022`, 3 x the front; nothing else changed; same command, output `run-q5.txt`): `contact_s=0.0033`, `ground_contacts_in_window=0`, `metric dv_mps sim=17.0636 ref=16.1958 err=+0.0536 norm=0.8930 ok`, `metric peak_mps2 sim=511.9 ref=502.6 err=+0.0186 norm=0.1032 ok`, `metric peak_time_s sim=0.0422 ref=0.0314 err=+0.0108 norm=0.4909 ok`, `metric duration_s sim=0.0795 ref=0.0748 err=+0.0628 norm=0.6283 ok`, `metric olc_mps2 sim=299.0 ref=347.8 err=-0.1404 norm=0.9357 ok`, `metric static_crush_m sim=0.4792 ref=0.3508 err=+0.3660 norm=1.8301 FAIL`, `info static_crush_centroid_m sim=0.4829`, `info dyn_crush_m sim=0.5477 ref=0.5386`, `pulse_score=0.8135`, `pulse: FAIL static_crush_m norm=1.8301`. The five pulse metrics and the centroid value 0.4829 equal the review's q5 run.
- **Six-point against centroid:** 0.3720 against 0.3725 and 0.4792 against 0.4829. On this flat-fronted car the two rules differ by under 1%, so the new rule does not close q5's static crush FAIL.
- **Wall load, measured** (`validation_wall_load_acts_below_the_centre_of_gravity`, output `scratch/t-20261122-042859/vf.txt`): `resultant_y_m=0.8592 cg_y_m=0.9597 arm_m=-0.1004 node_impulse_ns=22646 tyre_impulse_ns=0 ground_contacts=0 pitch_deg max +0.561 at 0.0631 s, min -1.150 at 0.1533 s`. At q5 (the same test with the wall file temporarily replaced by `q5.bsn` and restored, output `vf-q5.txt`): `resultant_y_m=0.8703 cg_y_m=0.9596 arm_m=-0.0894 node_impulse_ns=21931 tyre_impulse_ns=535 ground_contacts=0 pitch_deg max +0.617 at 0.0578 s, min -0.931 at 0.1533 s`, the review's numbers.
- **Step limit** (the binary, `--input scenarios/pad_lap.input --dt 0.001 --substeps <n> --duration 0.01`): at today's values substeps 10 `run: ok`, substeps 1 refused with limit **0.000114 s**; at q5 substeps 10 `run: ok`, substeps 1 limit **0.000119 s**. **At the top of the ranges the ratio allows** (`scratch/t-20261122-042859/top.bsn`: rear A = 0.002 m^2, k 1025641025.641, c 17662.564; front A = 0.002 / 3, k 341880341.880, c 7453.560; zeta 0.05 for both, m_c 30.4167 and 16.25 kg) **the run is refused at h = 0.0001 s**: `physics step h = dt/substeps = 0.000100 s reaches the body's stability limit 0.000085 s`. The same file with the cabin at 666.667 (`top666.bsn`) runs (`run: ok`). The beams-only bound (`scratch/t-20261122-042859/gersh3.py`) gives 0.000086 s there, its damping term set by the cabin (g = 15367 = 2 x 113333 / 14.75, the largest per-node damping sum being 17 x 6666.67 N s/m), and 0.000115 s at today's values. So with the cabin at 10x damping, grid points near the top of P1 at zeta 0.05 will be refused at the frozen step: a finding for the next calibration task, not changed here.
- **Fail-first** (the new test run before the files were changed; output `scratch/t-20261122-042859/ff-vf1.txt` and `ff-vf2.txt`, test `step7_rear_ratio_three_and_cabin_damping_tenfold`). Before the ratio change: `panicked at testsalidation_files.rs:528:5: assertion `left == right` failed: `frozen: crush.rear_to_front_strength`  left: [2.0] right: [3.0]`, `test result: FAILED. 0 passed; 1 failed`. After the frozen line was changed but before the car files were: `panicked at testsalidation_files.rs:545:9: assertion `left == right` failed: validation_car.bsn: cabin damping  left: 666.667 right: 6666.67`, `test result: FAILED. 0 passed; 1 failed`. For the six-point pulse test (`static_crush_is_the_six_point_profile_mean`): run with `src/main.rs` temporarily set to the base `git show 8caf647:src/main.rs` (restored afterwards with `git checkout HEAD -- src/main.rs`), the new test kept; output `scratch/t-20261122-042859/ff-pulse3.txt`: `thread 'static_crush_is_the_six_point_profile_mean' panicked at tests\pulse_report.rs:364:28: no info static_crush_centroid_m line in:`, `test result: FAILED. 12 passed; 3 failed` (the other two failures: `pulse_report_lines_and_format`, `pulse_report_without_static_crush_prints_none`).
- **Test B:** not run, and its results, reference and meta not opened; `validation_car_b.bsn` and `validation_wall_b.bsn` were edited to match (`validation/CAR_B.md`).

## Calibration step 8: test A values inside the pre-approved ranges

*Task t-20261122-084300. Lazlo's pre-approved ranges (2026-11-22 04:44Z), the sweep counts, the three passing points and why point #10 was chosen are in docs/STAGE6-M4-M5.md, "Calibration step 8".* Lines 3 to 5 of `scenarios/validation_car.bsn` (and of `validation_wall.bsn`, which still differs only by its `plane` line):

```
material cabin stiffness 1000000 damping 6666.67 yield_strain 0.02
material crush_front stiffness 35897435.898 damping 724.569 yield_strain 0.0024
material crush_rear stiffness 143589743.59 damping 1982.618 yield_strain 0.0024
```

- **What changed:** both crush stages, from sweep point fine1 #10 (front P1 7e-5 m^2, P2 0.0024, P3 0.015; rear 4 x the front's area, the same P2 and P3; L_c 0.4095 m, m_c 16.25 and 30.4167 kg); `frozen: crush.rear_to_front_strength` 3 -> 4. `cabin` is unchanged (damping 6666.67, 10x). Computed back from the file: front P1 7.000000000110004e-05, P3 0.01500000336789605; rear P1 0.0002800000000005, P3 0.015000009355318132; rear / front 3.9999999999442837 (`scratch/t-20261122-084300/blines.py`, output `blines.txt`). The front stiffness is the variant's 35897435.897 rounded up in the last digit so P1 is not below the range floor, and the rear's `143589743.590` is printed `143589743.59` (the same f64) so the file writes back byte for byte; docs/STAGE6-M4-M5.md, "Calibration step 8", has both.
- **Test A at these values** (the 4.4 command: `broadsword run crash-sim --task t-20261122-084300 -- cargo run --offline --release -- --body scenarios/validation_wall.bsn --pulse-report validation/test-a.pulse.csv ` `<frozen: run_flags>`, with the flags read from that line; output `scratch/t-20261122-084300/run-a-final.txt`, job 20261122-163721-crash-sim-run): `contact_s=0.0033`, `ground_contacts_in_window=0`, `metric dv_mps sim=17.1624 ref=16.1958 err=+0.0597 norm=0.9947 ok`, `metric peak_mps2 sim=510.5 ref=502.6 err=+0.0157 norm=0.0873 ok`, `metric peak_time_s sim=0.0464 ref=0.0314 err=+0.0150 norm=0.6818 ok`, `metric duration_s sim=0.0784 ref=0.0748 err=+0.0481 norm=0.4813 ok`, `metric olc_mps2 sim=299.7 ref=347.8 err=-0.1383 norm=0.9218 ok`, `metric static_crush_m sim=0.4162 ref=0.3508 err=+0.1865 norm=0.9325 ok`, `info static_crush_centroid_m sim=0.4137`, `info dyn_crush_m sim=0.5045 ref=0.5386`, `pulse_score=0.6832`, `pulse: ok`, `run: ok`. These equal the sweep's row #10 (`fine1.csv`), so the rounding of the front stiffness did not move any printed metric. dv passes narrowly (norm 0.9947).
- **Fail-first** (`scratch/t-20261122-084300/test-wip2.txt`: the A files and the frozen line at step 8, the B files still at step 7): `validation_car_b.bsn: the strength ratio rear / front is 3.0000000000000018, not the frozen 4` (`crush_rear_stronger_than_front_by_frozen_ratio` and `step8_rear_ratio_four_and_cabin_damping_tenfold`), `crush_front: P1 area_m2: 0.00006999999999915003 against 0.00012250000000095006` (`validation_files_share_crush_physics`), and with the variant's unrounded front stiffness `validation_car.bsn crush_front: crush.area_m2 = 0.00006999999999915003 is outside [0.00007, 0.002]` (`crush_physics_inside_frozen_ranges`); `test: FAILED (exit 101) — 541 passed, 5 failed`. After the rounding and the B lines: `test: ok — 567 passed, 0 failed` (`test-3.txt`).
- **Test B:** not run, and its results, reference, meta and thresholds not opened or changed; `validation_car_b.bsn` and `validation_wall_b.bsn` were edited to match (`validation/CAR_B.md`, "Calibration step 8").

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
| Front rails (longitudinal members, two, 725 mm apart, 60 mm wide, top at 498 mm: R items 5 to 7) | Lumped with the crush cans and the bumper beam into the 12 `crush` longitudinal beams, 6 per segment (4 floor, 2 roof at the body sides), one material | Section outline and wall thickness along the length, material curve, the fold pattern (initiators, convolutions), the connection to the rear subframe and the firewall; the engine's position relative to them |
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
