# validation/: real crash reference pulses

Two NHTSA NCAP full-frontal rigid-barrier tests at 56 km/h, converted into
reference pulses for Stage 6 m4 (docs/STAGE6-M4-M5.md). Test A is for
calibration, test B for the check.

| File | What it is |
|---|---|
| `test-a.pulse.csv` | NHTSA test 7504, 2012 Hyundai Accent GLS 4-door, right rear seat crossmember X, `t_s,accel_mps2` |
| `test-a.meta` | its provenance, `key = value` lines (keys of check 4.1) |
| `test-b.pulse.csv` | NHTSA test 7751, 2012 Kia Rio LX 4-door, same channel location, same format |
| `test-b.meta` | its provenance |
| `convert.py` | std-only converter: database EV zip + curve number -> the two files above |
| `test_convert.py` | tests of the converter on a hand-written EV5 sample, plus the delta-V check on the committed pulses |

Raw downloads are not committed. Every meta file has the `url` of the zip it
came from, so it can be fetched again (see "Redo the download").

## The two tests

| | Test A | Test B |
|---|---|---|
| NHTSA test number | 7504 (lab ref. BT11110301) | 7751 (lab ref. BT12031301) |
| Vehicle | 2012 Hyundai Accent GLS 4-dr sedan | 2012 Kia Rio LX 4-dr sedan |
| Lab, date (contract DTNH22-06-D-00028 for both) | MGA Research, 03 Nov 2011 | MGA Research, 13 Mar 2012 |
| Test mass (VEHTWT) | 1330 kg (report: 1329.5 kg) | 1377 kg (report: 1377.1 kg) |
| Impact speed (CLSSPD) | 56.2 km/h (report: 56.2) | 56.1 km/h (report: 56.1) |
| Barrier | rigid, 0 deg, full width, load-cell face | same |
| Channel | curve 94, SENATT FLRR, AXIS XG, "RIGHT REAR SEAT CROSSMEMBER X" | curve 94, same |
| Static crush C1..C6 (DPD1..6) | 253, 351, 402, 403, 391, 305 mm; mean 0.3508 m | 409, 448, 452, 458, 447, 407 mm; mean 0.4368 m |
| Max static crush (report) | 470 mm at centreline | 507 mm at centreline |
| Delta-V, trapezoid over t >= 0 | 58.08 km/h, +3.4% of impact speed | 57.78 km/h, +3.0% of impact speed |

The Accent and the Rio are two different vehicles on a shared Hyundai/Kia
platform (a sibling pair, which the brief allows). Delta-V is above the
impact speed because it includes the rebound.

### Channel details (both tests)

- **Location:** accelerometer on the rear seat crossmember, right side
  (database: SENATT `FLRR`, "FLOORPAN - RIGHT REAR"; instrumentation comment
  "RIGHT REAR SEAT CROSSMEMBER X"; ISO export "Floor over Rear Axle",
  channel code `10FORARIRE00ACXP`). Occupant compartment, behind the crush zone.
- **Axis and sign:** `XG`, vehicle-fixed global X. NHTSA Test Reference Guide
  Vol. I, field AXIS: "X is positive forward, Y is positive right ... Z is
  positive down". That is crash-sim's +x (vehicle forward), so no sign is
  changed and the frontal pulse is negative.
- **Units:** the EV channel file is in G's; `convert.py` multiplies by
  9.80665. Cross-check: NHTSA's own ISO-MME export of the same curve (unit
  `m/(s*s)`) matches the converted files to 5e-6 m/s^2 on all 3200 samples
  for both tests.
- **Sample rate:** DELT 100 us, 10 kHz.
- **Pre-filter:** PREFIL 2800 Hz (the lab's -3 dB low-pass before
  submission); data status AM (as measured); ISO export filter class `P`
  (pre-filter only). No further filtering here: filtering is n20-03's job and
  must be the same for the sim and the test.
- **Record:** NFP -200 to NLP 2999: t = -0.0200 s to 0.2999 s, 3200 samples.
  The 200 samples before time zero are kept (the filter needs them).
- **Time zero:** the database's time zero (index 0, Guide Vol. I, NFP). The
  pre-zero signal stays within 0.51 g (A) and 0.43 g (B); the first sample
  above 3 g comes at t = 2.0 ms in both, consistent with t = 0 at first
  barrier contact (the front crushes before the rear crossmember feels it).
  The sources read do not define time zero in words beyond "index 0"; that
  NCAP time zero is first contact is the usual practice, not quoted here.

### Why these two

Candidates were NCAP frontal tests (VEHICLE INTO BARRIER, impact angle 0,
offset 0) with test mass 1200 to 1400 kg, from the database search API. The
choice needed the same channel location in both, a good primary channel and
a delta-V within 10% of the impact speed. Delta-V of the primary right rear
seat crossmember X channel (full record, raw):

| Test | Vehicle | Mass kg | Delta-V vs speed | Used? |
|---|---|---|---|---|
| 7504 | 2012 Hyundai Accent | 1330 | +3.4% | **A** |
| 7751 | 2012 Kia Rio | 1377 | +3.0% | **B** |
| 7579 | 2012 Honda Fit | 1310 | +10.9% | no: over 10% (rebound 14% of impact speed); its left crossmember primary is flagged questionable |
| 10966 | 2020 Hyundai Accent | 1333 | +8.9% | no: A and B from one era and one lab were preferred |
| 6996 | 2011 Ford Fiesta | 1359 | right channel failed | no |
| 7147 | 2011 Honda Civic | 1408 | +9.2% | no: mass over 1400 kg |

The Fit was the first choice for B (a non-sibling) and was dropped only on
the 10% sanity check; its excess is rebound, not a units or sign error.

## Licence

The data are US federal government records from NHTSA's Vehicle Crash Test
Database. Works of the US government are not subject to copyright in the US
(17 U.S.C. 105), and that is the basis for treating the files as public
domain. **How this was checked, and what was not:** the database API page
(`https://nrd.api.nhtsa.dot.gov/`) and the API descriptions were read; they
state the databases "provide public access to vehicle and motor vehicle
equipment tests funded by NHTSA and performed by NHTSA and its contractors"
and carry no licence or terms. The downloads need no account, key or fee.
The nhtsa.gov pages that would state a site-wide policy
(`www.nhtsa.gov`, `api.nhtsa.gov`) answered 403 to automated clients, so no
explicit "public domain" statement from NHTSA was read. Two points to note:
the Test Reference Guide carries "Copyright 1998-2014 NHTSA / US DOT", and
the tests were run by a contractor (MGA Research) under an NHTSA contract.
**Status: not confirmed from an NHTSA statement.** Before relying on it,
open an nhtsa.gov policy page in a browser and record what it says here.

## Redo the download

No account or key is needed. For test 7504 (7751 is the same with its number):

1. Search (optional): `https://nrd.api.nhtsa.dot.gov/nhtsa/vehicle/api/v1/vehicle-database-test-results/by-search?vehicleMake=HYUNDAI&vehicleModel=ACCENT&count=200&pageNumber=0`
2. Channel list: `https://nrd.api.nhtsa.dot.gov/nhtsa/vehicle/api/v1/vehicle-database-test-results/get-instrumentation-info/7504?count=200&pageNumber=0`
3. File links: `https://nrd.api.nhtsa.dot.gov/nhtsa/vehicle/api/v1/vehicle-documents/test-no/7504` (its `evFiles` is the `url` in the meta file).
4. Download the EV zip (the meta `url`) into a scratch folder and run, from
   the crash-sim root:

   ```
   python -I -B validation/convert.py <scratch>/v07504.zip --curve 94 --url https://nrd-static.nhtsa.dot.gov/compress/ev/vehdb/v00000/v07500/v07504.zip --out validation/test-a
   python -I -B validation/convert.py <scratch>/v07751.zip --curve 94 --url https://nrd-static.nhtsa.dot.gov/compress/ev/vehdb/v00000/v07700/v07751.zip --out validation/test-b
   ```

   Each prints its sample count, time range and delta-V.
5. Reports (mass, speed, crush profile): `https://nrd-static.nhtsa.dot.gov/reports/vehdb/v00000/v07500/v07504R001.pdf`, `https://nrd-static.nhtsa.dot.gov/reports/vehdb/v00000/v07700/v07751R001.pdf`.
6. Field definitions: NHTSA Test Reference Guide Volume I, Version 5 (Revision Nov 2014), `https://static.nhtsa.gov/nhtsa/downloads/NRD/Research/pre-2026/Volume%20I%20-%20Vehicle%20-%20Version%205%20(Revision).pdf`.

Tests: `python -I -B validation/test_convert.py -v`.

## Figures V1 to V11 of docs/STAGE6-M4-M5.md

Only NHTSA sources were allowed for this task (the crash test database, its
documentation, and nhtsa.gov licence pages). A figure that needs any other
source is marked `unverified`, not guessed. `unverified` is not one of the
two forms check 4.1 accepts, so 4.1 stays failing until the orchestrator
settles those lines.

verify V1 = unverified (partly): the protocol is verified (tests 7504 and 7751: TSTCFN VTB, rigid barrier, IMPANG 0, OFFSET 0, CLSSPD 56.2 and 56.1 km/h; reports: "56.3 km/h NCAP Frontal Impact Test") and the time histories download free with no account (nrd-static.nhtsa.dot.gov EV zips); "US government work (public domain)" is not confirmed by an NHTSA statement, see Licence
verify V2 = unverified: Euro NCAP and IIHS are outside the sources allowed for this task
verify V3 = unverified: SAE J211-1 is outside the allowed sources; note that the database's filter-class lookup (`/nhtsa/nhtsadb/api/v1/filterclass/FLRR/AC/G'S`) returns class 600 for this channel, a component-analysis default, not the 60/180 that m4 uses
verify V4 = unverified: the OLC definition (Kübler et al.) is outside the allowed sources
verify V5 = verified DELT = 100 us (10 kHz) on curve 94 of 7504 and 7751 (EV5 instrumentation records), recorded per channel; Guide Vol. I 2.8: "The minimum sample rate shall be 10000 Hz"; some older tests use 20 kHz (5586: DELT 50 us)
verify V6 = unverified: no repeat-test scatter study was in scope; one datum against it: the four rear seat crossmember X channels inside one test differ in delta-V by up to 3.5 km/h (7504: 58.1 to 61.6 km/h) and 7.1 km/h (7751: 55.4 to 62.5, one channel flagged "DELTA V QUESTIONABLE")
verify V7 = unverified: NCAC/CCSA FE model validation reports are outside the allowed sources
verify V8 = unverified (partly): the 6-point profile is verified (Guide Vol. I fields 48-53, DPD1-DPD6, integer mm, left to right for frontal damage, 4 points if the damage is 400 mm wide or less; reports 7504 and 7751 "CRUSH PROFILE" C1..C6 equal the DPDs); the "about 1 cm" measurement accuracy is not stated in those sources
verify V9 = unverified: steel sheet properties are outside the allowed sources
verify V10 = unverified: damping of welded structures is outside the allowed sources
verify V11 = corrected pulse end (time of maximum rebound, raw integrated velocity) 0.08 to 0.12 s after contact; rebound speed 7 to 17% of impact speed: rear seat crossmember (5586: rear seat) X channels of NCAP tests 7504 (0.077 s, 7.1%), 7751 (0.082 s, 6.6%), 7579 (0.111 s, 14.4%), 10966 (0.082 s, 11.6%), 5586 (0.102 s, 12.9%), 7147 (0.120 s, 16.9%); the 0.15 s window still covers all six
