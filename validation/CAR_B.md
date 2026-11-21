# validation/CAR_B.md: the validation car for test B

`scenarios/validation_car_b.bsn` is the car that m4 crashes against the second reference test; `scenarios/validation_wall_b.bsn` is that file plus one `plane` line. The method, the labels (**sourced**, **derived**, **assumed**), the layout and the reasons for what is kept from `car_basic.bsn` are those of `validation/CAR.md` (test A's car); this file records what is specific to B. The second test is NHTSA test 7751, a 2012 Kia Rio LX 4-door at 56.1 km/h into a rigid full-width barrier (`validation/test-b.meta`, `validation/README.md`). Nothing in this task opened or ran B's pulse data: the files below were built from the test report, the EV5 vehicle record's header fields and public web pages, and the checks run the car against the existing plain CLI, not `--pulse-report`.

## Sources

| Id | File | Where it came from | SHA-256 |
|---|---|---|---|
| R | `v07751R001.pdf`, NHTSA test report for the 2012 Kia Rio LX 4-Dr Sedan (MGA Research, test date 03/13/2012) | `https://nrd-static.nhtsa.dot.gov/reports/vehdb/v00000/v07700/v07751R001.pdf` (`validation/README.md`, "Redo the download", step 5) | `55514c4d266ff3fa1253c7a1f4f224f93a15d725131f91ba83a095ecff6c9198` |
| E | `v07751.zip`, member `v07751.EV5`, the VEHICLE record (line 7), fields 11 to 16 and 42 | `https://nrd-static.nhtsa.dot.gov/compress/ev/vehdb/v00000/v07700/v07751.zip` | `c39366f44b55d3b2f0d66c76be89379aa577ffcf614457428c6f1fb897c1feea` |
| G | NHTSA Test Reference Guide Vol. I, Version 5 (field names of the EV5 VEHICLE record) | `validation/README.md`, step 6 | `afdd3e66abd72a2a399fc9723e9158cd5ce22b882b53962008f510b6d9a3376b` (`vol1.pdf`, the copy read for n20-02) |
| D | `docs/STAGE6-M4-M5.md`, the `frozen:` lines 143 to 148 | the repo | (Git) |

R and E are the copies downloaded for n20-02 (`scratch/n20-02/ref/`, `scratch/n20-02/raw/`; the checksums are of those copies); line numbers below are lines of the text extraction of R (`v07751R001.txt`), and the section names are the report's own.

## Matched dimensions

Axes as in docs/FORMAT.md: x forward, y up, z across (+z right); the middle of the wheelbase is x = 0. Every file value below was read back from `scenarios/validation_car_b.bsn` (Checks).

| Quantity | Test B value | In the file | Source | Label |
|---|---|---|---|---|
| Test mass | 1377 kg (1377.1 kg as tested, with the two dummies; VEHTWT 1377) | 1257 kg on 48 nodes + 4 x 30 kg wheels = 1377 kg | R "Test Vehicle Weights", "As Tested (ATW)" total 1377.1 (line 385); data sheet 13 "Test Weight (kg): 1377.1" (line 1081); E field 11 = 1377; `validation/test-b.meta` `mass_kg = 1377` | sourced |
| Wheelbase | 2570 mm | 2.57 m (front axle x = 1.285, rear axle x = -1.285) | R "General Test Vehicle Data", "Total Vehicle Wheel Base" 2570 (line 413); data sheet 13 "Wheelbase (mm): 2570" (line 1080); E field 13 = 2570 | sourced |
| Overall length | 4364 mm at the centreline (4244 mm left side, 4424 mm right side) | 4.364 m (x from -2.28 to 2.084) | R "Total Vehicle Length at Centerline" 4364 (line 415); structural measurement item 1 "Total Length" 4364 (line 435); E field 14 = 4364 | sourced |
| Front overhang (the crush zone) | 799 mm, front bumper to front axle | 0.799 m (front-most node x = 2.084, front axle 1.285) | R structural measurement item 15 "Front Bumper - Front Axle Distance" 799 (line 463) | sourced |
| Rear overhang | 995 mm | 0.995 m (rear axle -1.285, rear-most node -2.28) | 4364 - 2570 - 799 = 995 | derived |
| Overall width | 1709 mm | 1.709 m, the z extent of the nodes (-0.8545 to +0.8545) | R item 2 "Total Width" 1709 (line 437); E field 15 = 1709 | sourced |
| Centre of gravity | 1129 mm behind the front axle (as tested) | 1.129 m behind the front axle (mass-weighted, wheels included) | E field 16 (VEHCG) = 1129; R "Test Vehicle Attitudes and CG" as-tested weights front 772.0 kg, rear 605.1 kg (line 385): 605.1 x 2570 / 1377.1 = 1129.3 mm, the Guide's own formula for field 16 | sourced |
| Track width | not in R, E or G | 1.5227 m, the z distance between the left and right wheel mounts, the same at both axles (z = -0.76135 and +0.76135) | public web pages (below): front 59.9 in = 1521.5 mm, rear 60.0 in = 1524.0 mm; the mean 1522.7 mm is used. Upper bound from R: width less one tyre width, 1709 - 185 (R "Tire Width" 185, line 348) = 1524 mm; 1522.7 mm is under it by 1.3 mm (the rear figure equals it) | sourced (web) |
| Speed | 56.1 km/h (`speed_kmh` of `validation/test-b.meta`; E field 42) | `velocity all 15.583333333333334 0 0`: 56.1 / 3.6 m/s on all 48 nodes; wheels start with the mean velocity of their mount nodes | `validation/test-b.meta`; E field 42 | sourced |
| Wall | rigid, full width | `plane 2.134 0 0 -1 0 0`: 0.05 m ahead of the front-most node (2.084 + 0.05), normal (-1, 0, 0) | docs/STAGE6-M4-M5.md, "The run" | brief |

Kept from `validation/CAR.md` (the rules are the same; the reasons are there):
- Tyre, wheel mass, suspension, tyre and controls lines are `car_basic.bsn`'s, **assumed**. B's real tyre is P185/65R15 (R "Tire Size on Vehicle", line 340; width 185, aspect 65, wheel diameter 15, lines 348 to 354), derived radius 15 x 25.4 / 2 + 0.65 x 185 = 310.75 mm, near `car_basic.bsn`'s 317.5 mm (195/65 R15). Not matched. The kept tyre is 0.195 m wide, so its outer face (0.76135 + 0.0975 = 0.859 m) is 4.5 mm outside the node grid's half width of 0.8545 m.
- Layout: 6 stations of 8 nodes (x = -2.28, -1.285, 0, 1.285, 1.6845, 2.084; z = -0.8545, -0.76135, 0.76135, 0.8545; y = 0.55, 1.95), node id = 8 ix + 2 iz + iy; the crush zone is two equal segments of 0.3995 m; 296 beams, the 12 longitudinal beams (counted in `scenarios/validation_car_b.bsn:223-331`) at floor level and at the two outer roof positions of the two front segments are material `crush`, the rest `chassis` (stiffness 300000, damping 200, yield strain 0.02 as `car_basic.bsn`). The roof beams over the wheel mounts (27-35, 29-37, 35-43, 37-45) are `chassis`, as in the layout fix described in `validation/CAR.md`.
- Node masses (**assumed** split, matched total and centre of gravity): station totals (8 nodes each, rear to front) 136, 248, 330, 296, 129, 118 kg; every roof node 14.75 kg and the four floor nodes the rest (floor per node, rear to front: 19.25, 47.25, 67.75, 59.25, 17.5, 14.75 kg). The lightest node is 14.75 kg for the step limit (below). The centre of gravity read back is 0.907 m up (wheels at their start height).
- Groups `all`, `front`, `rear`, `sensor` as A: `sensor` is nodes 12 and 14, the two floor nodes at the rear axle on the right (z = +0.76135 and +0.8545), the same channel location as A (`validation/README.md`, "Channel details"; both tests use the right rear seat crossmember X channel). Its position is **assumed** as in `validation/CAR.md`.
- The start velocity sits in the car file (docs/FORMAT.md orders `velocity` before the car sections and `plane` last); `cmp` shows the wall file starts with the car file byte for byte.

### Track width sources

Local files had no track figure (R's text, the EV5 record). Web pages, public, no account or fee, read 2026-11-21 (Lazlo's standing approval for internet lookups, 2026-11-21 03:48Z). Kia's own press page (`kiamedia.com`, release 3817) and Carfolio, The Car Connection and Edmunds answered HTTP 403, so the maker's figure could not be read.

| Id | Page | URL | Quoted figure |
|---|---|---|---|
| W1 | "2012 Kia Rio Dimensions - iSeeCars.com" (EX, LX 6M, LX 6A and SX, all listed as 4dr Sedan) | `https://www.iseecars.com/car/2012-kia-rio-dimensions` | "Front Track" "59.900001525878906" in; "Rear Track" "60.0" in (LX 4dr Sedan; the same for all four trims); "Wheel Base" "101.19999694824219" in; "Width" "67.69999694824219" in |
| W2 | "2012 Kia Rio Specs, Dimensions & Colors \| Cars.com" (trim "4dr Sdn Man LX") | `https://www.cars.com/research/kia-rio-2012/specs` | "Track Width, Front: 60 in"; "Track Width, Rear: 60 in" (whole inches); "Wheelbase: 101 in" |
| W3 | "2012 Kia Rio SLi Exterior & Dimensions \| CarExpert" (a hatchback, same platform, AU market) | `https://www.carexpert.com.au/kia/rio/2012/sli/exterior-and-dimensions` | "Front track: 1,521 mm"; "Rear track: 1,525 mm"; "Wheelbase: 2,570 mm" |
| W4 | "2012 Kia Rio Si Price and Specifications \| CarExpert" (a sedan, AU market, 195/55 R16 tyres) | `https://www.carexpert.com.au/kia/rio/2012-si-1-6l-sedan-fwd-petrol-automatic-jjo5af5s20120215` | "Front track: 1,470 mm"; "Rear track: 1,460 mm"; "Wheelbase: 2,570 mm" |

**Disagreement, reported:** W4 gives 1470 / 1460 mm, 51 and 64 mm below W1 (59.9 / 60.0 in = 1521.5 / 1524.0 mm) and W3 (1521 / 1525 mm). That is more than 10 mm, and there is no maker's figure to settle it. W1 is the US LX sedan, the test's own market, trim and body, and W3 (hatchback, same platform) and W2 (rounded) agree with it; W4's page lists the narrowest track with 195/55 R16 tyres, wider than the test car's 185 mm, which is odd for a narrower track, and three pages disagree with it; it is treated as an outlier and **not used**, but this is a judgement, not a measurement. **Value:** 1.5227 m, the mean of W1's front and rear, 2.7 mm from either axle's figure. If a maker's figure is later read and differs, the wheel mount z of both files changes together and CAR_B.md with them.

## Calibratable parameters and their ranges

The same three quantities as `validation/CAR.md`, with the same ranges (the `frozen: range` lines 143 to 145 of docs/STAGE6-M4-M5.md). **The rule for B is that its physics equal A's** (gate 4.2, `validation_files_share_crush_physics`: P1 to P3 computed back from each file agree within 1e-9 relative), so B's raw numbers are computed from A's P1 to P3 for B's own L_c and m_c:

| Quantity | Range | A's value, computed back from `validation_car.bsn` | B's value, computed back from `validation_car_b.bsn` |
|---|---|---|---|
| P1 area, k L_c / E, E = 210e9 | 0.00007 to 0.002 m^2 | 0.00012250000000095006 | 0.00012250000000094997 |
| P2 yield strain | 0.0010 to 0.0057 | 0.00167 | 0.00167 |
| P3 damping ratio, c / (2 sqrt(k m_c)) | 0 to 0.05 | 0.0250000042912473 | 0.0250000042912473 |

(P3 is computed from the current material lines and crush end-node masses, not from a scenario run.) A's P3 is slightly above 0.025 because A's file prints its damping to three decimals (`damping 1989.176`, `validation/CAR.md`); B's current material line agrees with A's P3 as computed back from the file, not from 0.025, and prints its stiffness and damping with every digit (`stiffness 64392991.239548184`, `damping 2016.1371400388848`). B's L_c = 0.3995 m (the 12 `crush` beams, one length) and m_c = 25.25 kg, using A's definition in `validation/CAR.md`: the arithmetic mean of the distinct nodes at the ends of the `crush` beams, not all nodes in the car and not an endpoint-occurrence-weighted mean. The node ids and masses from `scenarios/validation_car_b.bsn:28-51` are: 24, 26, 28, 30 at 59.25 kg each; 32, 34, 36, 38 at 17.5 kg each; 40, 42, 44, 46 at 14.75 kg each; and the outer roof nodes 25, 31, 33, 39, 41, 47 at 14.75 kg each. Thus m_c = (4 x 59.25 + 4 x 17.5 + 4 x 14.75 + 6 x 14.75) / 18 = 454.5 / 18 = 25.25 kg. Stations 3 to 5 here mean node ids 24 to 47; station 2's 67.75 kg floor nodes (ids 16, 18, 20, 22) are not crush endpoints. Recomputing from line 3 gives P3 = 2016.1371400388848 / (2 sqrt(64392991.239548184 x 25.25)) = 0.0250000042912473, matching A's P3; conversely c = 2 x A_P3 x sqrt(64392991.239548184 x 25.25) = 2016.13714003888 N s/m (rounded). No damping mismatch is found by this definition. These arithmetic values were recomputed with PowerShell from the node and beam lines, not from a scenario run. The starting values are A's, which are reasoned in `validation/CAR.md` (energy balance for A's 1330 kg and 0.470 m); the original energy estimate used 8 beams in parallel, while the current layout has 6 (four floor and two roof) per segment. As for A, the starting area was retained rather than re-derived after the layout change. n20-10 replaces the three numbers for A; the freeze copies them to B by the same rule (n20-10 owns that edit).

### Step limit at the top of the stiffness range

h = 0.0001 s (`frozen: h_s`). At the top of P1 (A = 0.002 m^2, k = 1051314142.678 N/m with L_c = 0.3995 m) and zeta at its top (0.05), the current m_c = 25.25 kg gives c = 16292.8457006836 N s/m. A hand calculation (Gershgorin formula) from the current beam list (`scenarios/validation_car_b.bsn:52-347`, masses at lines 4-51, chassis damping at line 2) using `src/solver.rs`'s Gershgorin formula gives w2 = 285712309.878874 and g = 4825.17849510063, hence a limit of 4 / (g + sqrt(g^2 + 4 w2)) = 0.000102632926135736 s (0.000103 s to six decimals). This is a calculation, not a new run or a check of B's pulse. The historical CLI checks below predate the layout fix. The margin above h is thin, as for A (now 0.000104 s, `validation/CAR.md`), and a change that lowers the lightest node mass (14.75 kg) or adds beams at a node needs this calculation again.

## Checks

Run from the Broadsword root, task t-20261121-042915-2, `broadsword` = `build/bin/broadsword.exe`.

**Read-back from `validation_car_b.bsn`** (an independent parse of the file text, not the generator):

```
mass 1377.0  min node 14.75  len 4.364  wb 2.57  fo 0.799  ro 0.995  width 1.709  track 1.5227 1.5227  cg 1.129  cgy 0.9071
```

**The wall file is the car file plus one line:** `cmp -n <size of validation_car_b.bsn> validation_car_b.bsn validation_wall_b.bsn` printed nothing (equal), and the rest of the file is `plane 2.134 0 0 -1 0 0`; `validation_wall_b_embeds_validation_car_b` checks it with the setup rules.

**Step limit** (`broadsword run crash-sim --task t-20261121-042915-2 -- cargo run --offline --release -- --body <variant> --input scenarios/pad_lap.input --dt 0.001 --substeps <n> --duration 0.01`, variants with only the `crush` material line changed, in the scratch folder):

```
A=0.002 zeta=0.05  substeps 10 (h=0.0001):   run: ok
A=0.002 zeta=0.05  substeps 9  (h=0.000111): crash-sim: physics step h = dt/substeps = 0.000111 s reaches the body's stability limit 0.000103 s ... run: FAILED (exit 2)
A=0.002 zeta=0     substeps 10:              run: ok
A=0.00007 zeta=0.05 substeps 10:             run: ok
```

**The wall file loads and the car hits the wall:** `... -- cargo run --offline --release -- --body scenarios/validation_wall_b.bsn --input scenarios/pad_lap.input --dt 0.001 --substeps 10 --duration 0.2` ended `run: ok`. Mean speed (m/s): 15.5831 at t = 0.003, 13.4081 at t = 0.004 (the wall is hit at about 3 ms: 0.05 m / 15.58 m/s = 3.2 ms), 4.6822 at t = 0.047, 1.7446 at t = 0.2 with the mean x falling from 0.6895 (t = 0.1) to 0.5578 (t = 0.2): rebound. This is the plain CLI with gravity 9.81, a check that the file parses, starts at test speed and is stopped by the wall; it says nothing about the pulse and is not a run against the reference data.

**The five tests of gate 4.2** are in `tests/validation_files.rs` (and a sixth, `validation_checks_reject_wrong_copies`, which runs the same checks on deliberately wrong copies). Each was shown failing first by mutating the real B files and restoring them (task report): a wall 0.1 m too far ahead fails `validation_wall_b_embeds_validation_car_b`; a stiffness 1% off fails `validation_files_share_crush_physics`; a yield strain of 0.02 fails `crush_physics_inside_frozen_ranges`; one crush beam given `rest 0.5` fails `crush_beams_one_length`.

**Not checked:** that the web track figures are for the exact test car (and W4 disagrees, above); the position of the real accelerometer relative to the `sensor` group; anything about the pulse, which this task never opened.
