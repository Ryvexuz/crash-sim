# V6 evidence, second attempt (task t-20261121-045101), 2026-11-21 UTC

Status: **V6 is still not confirmed.** No `confirmed:` line was added and `docs/STAGE6-M4-M5.md` is unchanged. 4.1b output
(run on this worktree's copy of the file, redirected to `scratch/t-045101/s6-4-1b.txt`):

```
label: | V6 (*unverified, from memory*)
```

4.1b therefore fails (needs `label: | V6 (*sourced*)` plus a `confirmed:` line).

## Part A: sources tried

"Readable" means the text could be read by a script or fetch tool without an account. A copy of a readable paper was read at
`web.archive.org` where the original host answered HTTP 403 to every automated request (nhtsa.gov, www-nrd.nhtsa.dot.gov, rosap
for most documents, gao.gov, researchgate, core.ac.uk, academia.edu). Pages that showed a bot-check (swov.nl) or cookie wall
(pubmed) were not bypassed.

| # | Source | URL | Readable? | What it says that matters (exact quote) | Supports V6 (full-frontal rigid barrier, repeat scatter)? |
|---|---|---|---|---|---|
| 1 | Meyerson, Zuby, Lund, "Repeatability of Frontal Offset Crash Tests", ESV 1998, 98-S11-O-02 | https://www-nrd.nhtsa.dot.gov/pdf/esv/esv16/98s11o02.pdf | original 403; read via https://web.archive.org/web/2020/https://www-nrd.nhtsa.dot.gov/pdf/esv/esv16/98s11o02.pdf (1,493,121 bytes) | "Acceleration differences in the repeated tests of the Saab 900, Pontiac Trans Sport, and Dodge Neon were less than 5 g"; "Despite somewhat larger peak acceleration differences (8-14 g) in the repeated tests of the Ford Taurus, Infiniti Q45, and Hyundai Elantra"; the tests are "40 percent overlap with a deformable barrier" at 64.4 km/h. On NHTSA's full-overlap rigid-barrier study it only cites: "12 full-overlap frontal crashes at 35 mi/h against a rigid barrier at three test sites" with HIC and chest acceleration, no pulse figures. | No. Offset deformable barrier, not full-frontal rigid. Peak differences in g are given, but no delta-V, time of peak or OLC, and no rigid-barrier numbers. |
| 2 | Saunders and Parent (NHTSA), "Repeatability of a Small Overlap and an Oblique Moving Deformable Barrier Test Procedure", SAE 2013-01-0762 | https://www.nhtsa.gov/sites/nhtsa.gov/files/2013-01-0762.pdf | original 403; read via https://web.archive.org/web/2020/https://www.nhtsa.gov/sites/nhtsa.gov/files/2013-01-0762.pdf (15,855,313 bytes) | "The difference between the Peak Gs and the CV for SOI was 6.6 Gs and 7.8 percent and 4.8 Gs and 7.3 percent for Oblique." "The range in peak change in velocity (delta-V or DV) was approximately 2 kph at 150 ms for both procedures and the CV was approximately 2 percent for both procedures." Full frontal: "The Full Frontal group, representing crash tests run at 56 kph into a full-width flat rigid barrier, consists of 12 pairs of tests and one set of three tests", but "Since many of the full frontal tests include only two data points per group, CVs were not calculated". | No for the figures: the quoted peak and delta-V scatter is for three repeats of one 2011 Cruze in the small-overlap and oblique moving-barrier tests. The full-frontal peak, time-of-peak and delta-V differences exist only as bar charts (Figures 26 to 28, images, no numbers in the text); the 56 kph pair list is in Appendix E, also an image. |
| 3 | Saunders and Parent, "Repeatability and Reproducibility of Oblique Moving Deformable Barrier Test Procedure", SAE 2018-01-1055 | https://downloads.regulations.gov/NHTSA-2019-0106-0005/attachment_1.pdf (same bytes at https://lindseyresearch.com/wp-content/uploads/2020/03/NHTSA-2019-0106-0005-Repeatability_and_Reproducibility_of_Oblique_Moving_Deformable_Barrier_Test_Procedure_Saunders_2018.pdf) | yes, direct (1,936,533 bytes) | "Saunders and Parent [11] used a CV of less than or equal to 20 percent to determine if the oblique test procedure repeatability is acceptable"; Appendix A lists 16 NHTSA 56 kph full-frontal rigid-barrier tests in pairs (TSTNO 6258 and 6271 Cadillac CTS, 4244 and 5036 Chevrolet Trailblazer, and so on) with no scatter numbers; the vehicle channels are rated by CORA, ISO and CVmod scores, not by delta-V or peak. | No. Oblique moving barrier; the full-frontal part compares dummy and vehicle time histories by CORA score only. Useful as a lead: it names NHTSA full-frontal pairs (see Limits). |
| 4 | Machey and Gauthier, "Results, Analysis and Conclusions of NHTSA's 35 MPH Frontal Crash Test Repeatability Program", SAE 840201 | https://saemobilus.sae.org/content/840201 | abstract only (paper paywalled) | "The program consisted of a minimum of four frontal barrier impacts of 1982 Chevrolet Citations at three different test sites, and resulted in a total of fourteen tests." Abstract gives no numbers. | No: no figures readable. This is the full-frontal rigid-barrier repeat study; the paper itself is behind SAE. |
| 5 | McLaughlin and Saul (NHTSA), "Crashworthiness, Aggressiveness and Crash Test Procedures", DOT HS 806 749, 1985 | https://rosap.ntl.bts.gov/view/dot/29989/dot_29989_DS1.pdf (WebFetch worked; scanned report with a text layer, sha256 starts 94c384ef) | yes | Section 3.2.2.1 "Repeatability": the FRB and fixed load cell barrier tests of the same car are compared as "an indication of the vehicle response repeatability", but "vehicles tested were of different model years", "test weights were different", and the Concord crush differed by "15% to 18%"; "Responses of the two Marquis' were very similar". | No. Two cars, mixed model years and masses, no numbers for delta-V, peak or time of peak. |
| 6 | Reichert and Kan (NHTSA), IRCOBI 2019, IRC-19-82, oblique repeatability | https://www.ircobi.org/wordpress/downloads/irc19/pdf-files/82.pdf | yes, direct | "Higher impact speed tended to show marginally higher delta-v in longitudinal vehicle direction. Values ranged from 14.5 m/s to 15.1 m/s." | No. A finite-element simulation (design of experiments), not repeat tests, and an oblique barrier. |
| 7 | Saunders et al., "Repeatability study of replicate crash tests: A signal analysis approach", Traffic Injury Prevention 18(7) | https://www.tandfonline.com/doi/abs/10.1080/15389588.2016.1204445 ; https://pubmed.ncbi.nlm.nih.gov/27824504/ | no (403; cookie wall) | none read | Unknown. The only listed paper that may hold numbers for replicate tests; not reachable without a bypass. |
| 8 | Other hosts tried | gao.gov/assets/230/221114.pdf; researchgate 274740683; core.ac.uk/download/57707365.pdf; academia.edu 21512524; rosap dot_29983, 30397, 30202, 62654, 29906 (curl only); nhtsa.gov 2012-01-0577; swov.nl; trid.trb.org (connection refused) | no | none | Unknown |

Result of Part A: **no readable source states repeat-test scatter of full-frontal rigid-barrier tests (delta-V, peak, time of
peak or OLC).** The nearest numbers, in source 2, are for another configuration, so they were not entered as `confirmed:`
lines (a `confirmed:` line has to be a figure of the full-frontal rigid-barrier tests V6 names). Whether the
Saunders and Parent numbers (delta-V CV about 2%, peak CV 7 to 8% at one vehicle and a small-overlap or oblique barrier) are
close enough to stand in for V6 is Lazlo's decision, as is the use of Part B below.

Copies read are in `scratch/t-045101/` (wiped within 24 h): meyerson98.pdf, saunders2013.pdf, saunders2018.pdf, r29989.pdf,
and the text extracts.

## Part B: measured scatter in the Camry reference set

### Which tests repeat

From `reference/camry-2013/nhtsa/<test>/v0<test>.EV5` (TEST, VEHICLE and BARRIER records) and `MANIFEST.md`:

| Test | Configuration | Vehicle | Speed (km/h, CLSSPD) | Test mass (VEHTWT) | Lab / barrier record | Rate / prefilter of curves 93, 94 |
|---|---|---|---|---|---|---|
| 7520 | NCAP frontal, 15 Nov 2011 | 2012 Toyota Camry LE | 56.3 | 1662 kg | MGA / "ADVANCED RESEARCH LOAD CELL BARRIER" | 10 kHz, 2800 |
| 8545 | NCAP frontal, 24 Feb 2014 | 2014 Toyota Camry | 56.71 | 1664 kg | CAL / "FRONTAL FLAT BARRIER WITH 36 LOADCELLS" | 20 kHz, 4000 |
| 7517, 7523 | side MDB, side pole | 2012 Camry | not frontal | | | not used |
| 9121, 9124 | R&D oblique, 15 degrees, 35% overlap with a 2491 kg MDB, 90.46 and 89.68 km/h | 2012 Camry | not frontal rigid barrier; 9121 is the right oblique (impact angle 15) and 9124 the "LEFT OBLIQUE" (345), mirror images, different days | | | not used |

So the set holds **one pair of the same frontal rigid-barrier configuration: 7520 and 8545**. They are not true repeats: model
year 2012 against 2014, two laboratories, two load-cell barriers, 20 kHz against 10 kHz data (the prefilter differs too). Whether
the 2014 body structure was changed from the 2012 one was not checked. Mass (0.1%) and speed (0.7%) match. Everything below is
the scatter of that one pair, so each "scatter" is one difference, not a standard deviation from many tests.

### Method

1. `scratch/t-045101/mkpulse.py` reads each curve's ASCII file from the zip, checks the EV5 fields (AC, axis XG, unit G'S, data
   status AM, the time column against NFP and DELT), converts to m/s^2 (9.80665) and writes a `t_s,accel_mps2` file with the
   database's own time zero, as `validation/convert.py` does (that converter itself refuses curve 94 of test 8545 because its
   SENATT is `SERR`, not `FLRR`, so a scratch script was used; it was not changed).
2. A scratch example run through `build/bin/broadsword.exe run crash-sim --task t-20261121-045101 -- cargo run --release
   --example v6_scatter_tmp -- ../../../scratch/t-045101/pulses` (job `20261121-045750-crash-sim-run`, exit 0). It used the
   repo's `crash_sim::reference::load_pulse`, `resample_at` and the `crash_sim::pulse` functions `cfc_filter`, `delta_v`, `peak`,
   `duration`, `olc` exactly as the `--pulse-report` code (`src/main.rs`, `pulse_metrics`) does: each record filtered over its whole
   length at its own rate with CFC 60 (peak, time of peak, duration) and CFC 180 (delta-V, OLC); resampled to h = 1e-4 s on
   [0, 0.15] s (1501 samples, contact t = 0 at the database time zero); duration threshold 20 m/s^2; OLC v0 = CLSSPD / 3.6. The
   example file was deleted afterwards and is not committed (it is outside the claim).
3. The two tests are compared curve for curve: right rear seat crossmember X (curve 94) and left rear seat crossmember X
   (curve 93). Both are structural sensors behind the crush zone, like the sensor the m4 plan uses.

### Raw numbers (run output, `scratch/t-045101/run1.txt`)

```
label | dv_mps | dv_kmh | peak_mps2 | peak_g | peak_time_s | duration_s | olc_mps2 | olc_g
7520 c94 R-crossmember | 17.1042 | 61.575 | 453.8 | 46.27 | 0.0437 | 0.0864 | 291.8 | 29.75
8545 c94 R-crossmember | 17.2467 | 62.088 | 418.3 | 42.65 | 0.0440 | 0.0864 | 297.2 | 30.31
7520 c93 L-crossmember | 15.3425 | 55.233 | 429.4 | 43.78 | 0.0441 | 0.0837 | 264.1 | 26.93
8545 c93 L-crossmember | 17.0384 | 61.338 | 367.4 | 37.46 | 0.0447 | 0.0866 | 297.9 | 30.38
```

(Delta-V over the 0.15 s window exceeds the impact speed because the window includes the rebound.)

### Scatter of the pair (difference / mean of the two; sample SD = difference / sqrt 2; CV = SD / mean)

| Metric | Curve 94 (right): 7520, 8545 | difference / mean | CV | Curve 93 (left): 7520, 8545 | difference / mean | CV |
|---|---|---|---|---|---|---|
| delta-V (m/s) | 17.1042, 17.2467 | 0.83% | 0.59% | 15.3425, 17.0384 | 10.47% | 7.41% |
| peak (m/s^2) | 453.8, 418.3 | 8.14% | 5.76% | 429.4, 367.4 | 15.56% | 11.00% |
| time of peak (s) | 0.0437, 0.0440 | 0.3 ms | | 0.0441, 0.0447 | 0.6 ms | |
| duration at 20 m/s^2 (s) | 0.0864, 0.0864 | 0.00% | 0.00% | 0.0837, 0.0866 | 3.41% | 2.41% |
| OLC (m/s^2) | 291.8, 297.2 | 1.83% | 1.30% | 264.1, 297.9 | 12.03% | 8.51% |

Against the margins in `docs/STAGE6-M4-M5.md` (delta-V 5%, peak 15%, time of peak 10 ms, duration 10%, OLC 15%): the right
sensor's pair sits inside every margin; the left sensor's pair is inside time of peak (0.6 ms), duration and OLC, and at the margin edge
for peak (15.56% difference against 15%) and outside for delta-V (10.47% against 5%).

A cross-check on the left sensor: integrated over the whole 0.3 s record, 7520 curve 93 gives a speed change of 14.98 m/s against
16.76 m/s for its own curve 94, while 8545's two curves give 16.96 and 17.26 m/s. So the left sensor of 7520 is the outlier, and the
pair difference for curve 93 is mostly one sensor in one test (a within-test left-right difference of 10% in 7520), not a model-year
or lab effect that the right sensor would also show.

### Limits

- One pair, so no standard deviation can be estimated; the CV column is the two-point formula and has one degree of freedom.
- The pair differs in model year (2012, 2014), laboratory, barrier (two different load-cell barriers), data rate and prefilter, and
  the 2014 vehicle's structure was not checked against the 2012. The scatter here is therefore an upper-leaning mix of
  vehicle-to-vehicle, lab and barrier effects and is not repeat-test scatter of one build in one lab. It could also be smaller than a
  true repeat's scatter by chance: two points.
- Two sensors (left and right rear seat crossmember) in two tests: the right one agrees closely and the left one does not,
  and the evidence does not say which better represents what a sim sensor would see.
- Contact time zero is the database's own, as the m4 plan uses. Sensor positions on the two vehicles are named alike but not
  surveyed against each other.
- Delta-V is the 0.15 s window figure with rebound, not the impact-speed change.
- The 9121 and 9124 pair (oblique, mirror-image sides) was not used: not the configuration V6 names, and left versus right is not a repeat.
- Not read: validation tests A and B (`validation/test-a*`, `test-b*`); the Camry set is separate.
- Lead for Lazlo: source 3's Appendix A names 8 NHTSA full-frontal 56 kph pairs of other models (for example TSTNO 6258 and 6271,
  4244 and 5036, 6724 and 6729, 6642 and 6643) in the NHTSA database. Their raw channels could give measured full-frontal pair scatter
  for more than one vehicle; not done here (outside the Camry set this task was given). Saunders and Parent 2013, Figures 26
  to 28, show the average differences for exactly these pairs as bar charts that this task could not read as numbers.
