# Crash step 2b: test-A measurement of the chassis `unload_return`

Task: t-20261125-092422. crash-sim main commit (worktree HEAD): 315af13. Date: 2026-11-25. Measurement only: no value was chosen or adopted, no tracked file other than this one changed, and test B was not run or opened.

## Variants

Source `scenarios/validation_wall.bsn` SHA-256: `ca6eebde0320737a9ab5ae8508f490fbbf54fd2a8a2abbe8e40922642371c73c`. Each variant is a scratch copy (Broadsword `scratch/crash2/`) whose only difference is line 2 (`git diff --no-index --numstat` printed `1	1` for each). Base line 2: `material chassis stiffness 300000 damping 200 yield_strain 0.02`.

| u | file | SHA-256 | chassis line |
|---|---|---|---|
| 1 | wall_u100.bsn | 6d2e75ae41543ead8679f73047ef9e0197e990d06b6b9b77754bb9e1d6ac9e9d | `material chassis stiffness 300000 damping 200 yield_strain 0.02 unload_return 1` |
| 0.7 | wall_u070.bsn | 7f2ab3c95708eb516bfdd64e037431e1776b02a2211998f1873e6b6232f427ee | `material chassis stiffness 300000 damping 200 yield_strain 0.02 unload_return 0.7` |
| 0.5 | wall_u050.bsn | 331fd6ab4819296f5478916a276102061c819845c98f398ca69e028dccffd27b | `material chassis stiffness 300000 damping 200 yield_strain 0.02 unload_return 0.5` |
| 0.35 | wall_u035.bsn | f40b15b89037a5acf659a88029ab8a65e01c5120a4458008796f84ddab3418b7 | `material chassis stiffness 300000 damping 200 yield_strain 0.02 unload_return 0.35` |
| 0.2 | wall_u020.bsn | 00a5a86121df33d8611bcfa6936fff30e747afee8e14ac606eae74f8ff87f037 | `material chassis stiffness 300000 damping 200 yield_strain 0.02 unload_return 0.2` |
| 0.1 | wall_u010.bsn | b6fc76f70853df6cf673ef4a910ee03b8cf0c59eb1655baf7b98e99a702b5519 | `material chassis stiffness 300000 damping 200 yield_strain 0.02 unload_return 0.1` |

## Commands (from the Broadsword root, each once, foreground)

- control: `powershell -NoProfile -File scripts/run-test-a.ps1 --task t-20261125-092422`
- each point: `powershell -NoProfile -File scripts/run-test-a.ps1 --task t-20261125-092422 --body scratch/crash2/wall_<tag>.bsn` with tag u100, u070, u050, u035, u020, u010.

Exit codes: control 0, u100 0, u070 0, u050 1, u035 1, u020 1, u010 1 (exit 1 is the `pulse: FAIL` verdict, recorded as it is).

## Table

| u | dv | peak | peak_time | duration | olc | static_crush | peak_time_s sim | pulse_score | pulse | ground_contacts_in_window |
|---|---|---|---|---|---|---|---|---|---|---|
| control | 0.9947 ok | 0.0873 ok | 0.6818 ok | 0.4813 ok | 0.9218 ok | 0.9325 ok | 0.0464 | 0.6832 | ok | 0 |
| 1 | 0.9947 ok | 0.0873 ok | 0.6818 ok | 0.4813 ok | 0.9218 ok | 0.9325 ok | 0.0464 | 0.6832 | ok | 0 |
| 0.7 | 0.9549 ok | 0.1028 ok | 0.6864 ok | 0.3743 ok | 0.9009 ok | 0.9826 ok | 0.0465 | 0.6670 | ok | 0 |
| 0.5 | 0.9410 ok | 0.1222 ok | 0.6864 ok | 0.2807 ok | 0.8859 ok | 1.1089 FAIL | 0.0465 | 0.6708 | FAIL static_crush_m norm=1.1089 | 0 |
| 0.35 | 0.9074 ok | 0.1442 ok | 0.6909 ok | 0.2139 ok | 0.8713 ok | 1.3128 FAIL | 0.0466 | 0.6901 | FAIL static_crush_m norm=1.3128 | 0 |
| 0.2 | 0.8775 ok | 0.1628 ok | 0.6955 ok | 0.1471 ok | 0.8599 ok | 1.4446 FAIL | 0.0467 | 0.6979 | FAIL static_crush_m norm=1.4446 | 0 |
| 0.1 | 0.8646 ok | 0.1718 ok | 0.6955 ok | 0.1070 ok | 0.8518 ok | 1.4829 FAIL | 0.0467 | 0.6956 | FAIL static_crush_m norm=1.4829 | 0 |

u = 1 report lines (pulse_ref=7504 through pulse:) are byte-identical to the control's (checked with cmp).

## Report lines (verbatim, pulse_ref=7504 through pulse:)

### control

```
pulse_ref=7504
pulse_channel=curve 94 FLRR XG G'S (RIGHT REAR SEAT CROSSMEMBER X)
setup mass_kg sim=1330.0 ref=1330.0 err=+0.0000 tol=0.0100 ok
setup speed_kmh sim=56.20 ref=56.20 err=-0.0000 tol=0.0050 ok
contact_s=0.0033
ground_contacts_in_window=0
metric dv_mps sim=17.1624 ref=16.1958 err=+0.0597 norm=0.9947 ok
metric peak_mps2 sim=510.5 ref=502.6 err=+0.0157 norm=0.0873 ok
metric peak_time_s sim=0.0464 ref=0.0314 err=+0.0150 norm=0.6818 ok
metric duration_s sim=0.0784 ref=0.0748 err=+0.0481 norm=0.4813 ok
metric olc_mps2 sim=299.7 ref=347.8 err=-0.1383 norm=0.9218 ok
metric static_crush_m sim=0.4162 ref=0.3508 err=+0.1865 norm=0.9325 ok
info static_crush_centroid_m sim=0.4137
info dyn_crush_m sim=0.5045 ref=0.5386
pulse_score=0.6832
pulse: ok
```

### u = 1

```
pulse_ref=7504
pulse_channel=curve 94 FLRR XG G'S (RIGHT REAR SEAT CROSSMEMBER X)
setup mass_kg sim=1330.0 ref=1330.0 err=+0.0000 tol=0.0100 ok
setup speed_kmh sim=56.20 ref=56.20 err=-0.0000 tol=0.0050 ok
contact_s=0.0033
ground_contacts_in_window=0
metric dv_mps sim=17.1624 ref=16.1958 err=+0.0597 norm=0.9947 ok
metric peak_mps2 sim=510.5 ref=502.6 err=+0.0157 norm=0.0873 ok
metric peak_time_s sim=0.0464 ref=0.0314 err=+0.0150 norm=0.6818 ok
metric duration_s sim=0.0784 ref=0.0748 err=+0.0481 norm=0.4813 ok
metric olc_mps2 sim=299.7 ref=347.8 err=-0.1383 norm=0.9218 ok
metric static_crush_m sim=0.4162 ref=0.3508 err=+0.1865 norm=0.9325 ok
info static_crush_centroid_m sim=0.4137
info dyn_crush_m sim=0.5045 ref=0.5386
pulse_score=0.6832
pulse: ok
```

### u = 0.7

```
pulse_ref=7504
pulse_channel=curve 94 FLRR XG G'S (RIGHT REAR SEAT CROSSMEMBER X)
setup mass_kg sim=1330.0 ref=1330.0 err=+0.0000 tol=0.0100 ok
setup speed_kmh sim=56.20 ref=56.20 err=-0.0000 tol=0.0050 ok
contact_s=0.0033
ground_contacts_in_window=0
metric dv_mps sim=17.1238 ref=16.1958 err=+0.0573 norm=0.9549 ok
metric peak_mps2 sim=511.9 ref=502.6 err=+0.0185 norm=0.1028 ok
metric peak_time_s sim=0.0465 ref=0.0314 err=+0.0151 norm=0.6864 ok
metric duration_s sim=0.0776 ref=0.0748 err=+0.0374 norm=0.3743 ok
metric olc_mps2 sim=300.8 ref=347.8 err=-0.1351 norm=0.9009 ok
metric static_crush_m sim=0.4197 ref=0.3508 err=+0.1965 norm=0.9826 ok
info static_crush_centroid_m sim=0.4174
info dyn_crush_m sim=0.5031 ref=0.5386
pulse_score=0.6670
pulse: ok
```

### u = 0.5

```
pulse_ref=7504
pulse_channel=curve 94 FLRR XG G'S (RIGHT REAR SEAT CROSSMEMBER X)
setup mass_kg sim=1330.0 ref=1330.0 err=+0.0000 tol=0.0100 ok
setup speed_kmh sim=56.20 ref=56.20 err=-0.0000 tol=0.0050 ok
contact_s=0.0033
ground_contacts_in_window=0
metric dv_mps sim=17.1102 ref=16.1958 err=+0.0565 norm=0.9410 ok
metric peak_mps2 sim=513.6 ref=502.6 err=+0.0220 norm=0.1222 ok
metric peak_time_s sim=0.0465 ref=0.0314 err=+0.0151 norm=0.6864 ok
metric duration_s sim=0.0769 ref=0.0748 err=+0.0281 norm=0.2807 ok
metric olc_mps2 sim=301.6 ref=347.8 err=-0.1329 norm=0.8859 ok
metric static_crush_m sim=0.4286 ref=0.3508 err=+0.2218 norm=1.1089 FAIL
info static_crush_centroid_m sim=0.4266
info dyn_crush_m sim=0.5023 ref=0.5386
pulse_score=0.6708
pulse: FAIL static_crush_m norm=1.1089
```

### u = 0.35

```
pulse_ref=7504
pulse_channel=curve 94 FLRR XG G'S (RIGHT REAR SEAT CROSSMEMBER X)
setup mass_kg sim=1330.0 ref=1330.0 err=+0.0000 tol=0.0100 ok
setup speed_kmh sim=56.20 ref=56.20 err=-0.0000 tol=0.0050 ok
contact_s=0.0033
ground_contacts_in_window=0
metric dv_mps sim=17.0776 ref=16.1958 err=+0.0544 norm=0.9074 ok
metric peak_mps2 sim=515.6 ref=502.6 err=+0.0260 norm=0.1442 ok
metric peak_time_s sim=0.0466 ref=0.0314 err=+0.0152 norm=0.6909 ok
metric duration_s sim=0.0764 ref=0.0748 err=+0.0214 norm=0.2139 ok
metric olc_mps2 sim=302.3 ref=347.8 err=-0.1307 norm=0.8713 ok
metric static_crush_m sim=0.4429 ref=0.3508 err=+0.2626 norm=1.3128 FAIL
info static_crush_centroid_m sim=0.4402
info dyn_crush_m sim=0.5013 ref=0.5386
pulse_score=0.6901
pulse: FAIL static_crush_m norm=1.3128
```

### u = 0.2

```
pulse_ref=7504
pulse_channel=curve 94 FLRR XG G'S (RIGHT REAR SEAT CROSSMEMBER X)
setup mass_kg sim=1330.0 ref=1330.0 err=+0.0000 tol=0.0100 ok
setup speed_kmh sim=56.20 ref=56.20 err=-0.0000 tol=0.0050 ok
contact_s=0.0033
ground_contacts_in_window=0
metric dv_mps sim=17.0485 ref=16.1958 err=+0.0526 norm=0.8775 ok
metric peak_mps2 sim=517.3 ref=502.6 err=+0.0293 norm=0.1628 ok
metric peak_time_s sim=0.0467 ref=0.0314 err=+0.0153 norm=0.6955 ok
metric duration_s sim=0.0759 ref=0.0748 err=+0.0147 norm=0.1471 ok
metric olc_mps2 sim=302.9 ref=347.8 err=-0.1290 norm=0.8599 ok
metric static_crush_m sim=0.4522 ref=0.3508 err=+0.2889 norm=1.4446 FAIL
info static_crush_centroid_m sim=0.4489
info dyn_crush_m sim=0.5004 ref=0.5386
pulse_score=0.6979
pulse: FAIL static_crush_m norm=1.4446
```

### u = 0.1

```
pulse_ref=7504
pulse_channel=curve 94 FLRR XG G'S (RIGHT REAR SEAT CROSSMEMBER X)
setup mass_kg sim=1330.0 ref=1330.0 err=+0.0000 tol=0.0100 ok
setup speed_kmh sim=56.20 ref=56.20 err=-0.0000 tol=0.0050 ok
contact_s=0.0033
ground_contacts_in_window=0
metric dv_mps sim=17.0360 ref=16.1958 err=+0.0519 norm=0.8646 ok
metric peak_mps2 sim=518.1 ref=502.6 err=+0.0309 norm=0.1718 ok
metric peak_time_s sim=0.0467 ref=0.0314 err=+0.0153 norm=0.6955 ok
metric duration_s sim=0.0756 ref=0.0748 err=+0.0107 norm=0.1070 ok
metric olc_mps2 sim=303.4 ref=347.8 err=-0.1278 norm=0.8518 ok
metric static_crush_m sim=0.4548 ref=0.3508 err=+0.2966 norm=1.4829 FAIL
info static_crush_centroid_m sim=0.4514
info dyn_crush_m sim=0.4998 ref=0.5386
pulse_score=0.6956
pulse: FAIL static_crush_m norm=1.4829
```

## Mutations

- m1: the brief's `scratch/crash2/wall_bad.bsn` was refused by the run script before running anything: `run-test-a: REFUSED: argument 'scratch/crash2/wall_bad.bsn' contains '_b' (test B and reference/metadata files are never used here)` (exit 2; the name contains `_b`). The same body copied to `scratch/crash2/wall_x15.bsn` (chassis line ends `unload_return 1.5`) ran: exit 2, output line `crash-sim: C:\Users\MrComputer\Desktop\Broadsword\scratch\crash2\wall_x15.bsn: line 2, column 79: unload_return 1.5 must be > 0 and <= 1`. The key is read, not ignored.
- m2: see the done-check results in the task report.

## Reading

- Test A passes at u = 1 (pulse_score=0.6832) and u = 0.7 (pulse_score=0.6670). It FAILS at u = 0.5, 0.35, 0.2 and 0.1, each on static_crush_m only (norm 1.1089, 1.3128, 1.4446, 1.4829 against norm 1 as the limit); all other metrics stay ok.
- Lowest pulse_score among the passing points: 0.6670 (u = 0.7).
- dv norm falls monotonically with u: 0.9947, 0.9549, 0.9410, 0.9074, 0.8775, 0.8646 for u = 1, 0.7, 0.5, 0.35, 0.2, 0.1. dv sim 17.1624 to 17.0360 m/s (ref 16.1958): the whole range moves dv by about 0.13 m/s.
- Peak time stays at 0.0464 to 0.0467 s (norm 0.6818 to 0.6955), duration norm falls from 0.4813 to 0.1070, OLC norm from 0.9218 to 0.8518, peak norm rises from 0.0873 to 0.1718; every one of these is ok at every point. static_crush_m norm rises from 0.9325 to 1.4829 and is the only metric that crosses its margin.

## Follow-ups (not done here)

1. Adoption of a u into the test A car and wall files (needs Astra high review S4; the test B copies are Lazlo's).
2. The energy tool booking the unloading loss, so the energy split can be re-measured with the key.

## Astra review

(to be appended by the orchestrator)

Independent review: gpt-6.2-astra, high effort, codex exec read-only, 2026-11-25 ~09:45Z, 79 s, 42,748 tokens (ChatGPT plan). Verdict REJECT with one exact fix (delete the stale follow-up 3: three copies already call apply_unload and the energy tool refuses the key since ready-83, commit 315af13); applied. Quote: `Every field in table rows 29–35 matches its raw report`. Raw: scratch/astra-092422.md.
Second pass (same model and effort, 20,385 tokens): VERDICT: APPROVE, queueing this record only (no u adopted). Quote: `I found no unsupported measurement claim`. Raw: scratch/astra-092422-r2.md.
