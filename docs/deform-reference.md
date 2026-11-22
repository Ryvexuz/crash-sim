# Node 3.2: original synthetic deform references (S3)
All new fixtures, recipes and derivations here are original, authored here and repo-owned.
Formula references by name only: linear elasticity, constant-strain tetrahedron, membrane edge forces, linear beam theory.
No copied tables, prose, images, external models or vehicle data; nothing is Camry-calibrated.
Repo sources read: `docs/FORMAT.md`, `scenarios/box_wall.bsn`, `scenarios/car_basic.bsn`.
The manifest states exactly which structural conventions are reused and which synthetic inputs differ.
S1 has not been reconciled here: require `tests/fixtures/deform/manifest.txt` before S4; no implementation layout is assumed.

## Contract, units and derivations
CSV is UTF-8, LF, final newline, header `case_id,input,expected`; each ID occurs once.
Cells contain space-separated `key=value` entries; `:` separates vector components or successive samples; `;` separates event tuples.
Numbers are reduced exact rationals (integer or numerator/positive-denominator); parse exactly before conversion to binary64.
No solver supplies any reference. The generator below uses only stdlib `fractions`; all reference arithmetic is exact.
All lengths/displacements are m, masses kg, times s, moduli/stresses Pa, forces N, energies J, moments N m.
Strain, angles in radians, ratios and event counts are dimensionless; `swap_yz` is a discrete orientation selector, not an angle.
Invalid cells are literal hostile payload descriptions, not normal rational fields; error names are proposals owned finally by S4.
Reject before allocation or mutation, including unsupported subnormal inputs; never silently clamp, flush or substitute zero.
The nonempty exact manifest inventory includes hostile rows: they must be exercised, not filtered out.

**d2:** nodes `(0,0,0),(L,0,0),(0,L,0),(0,0,L)`; volume `L^3/6`, Poisson ratio zero; linear kinematics only.
Supply the prescribed u directly for this field oracle; recovering tiny u by subtracting rounded absolute positions is outside its supported contract.
`u(X)=H X+(t,t,t)`, with rows of H `(a,b,0),(c,0,0),(0,0,0)`.
Tensor strain order is xx:yy:zz:xy:xz:yz, `eps=(H+H^T)/2`; stress `E*eps` since nu=0.
Shape gradients are `(-1,-1,-1)/L,e_x/L,e_y/L,e_z/L`; internal nodal forces are `-V*sigma*grad(N)`.
These are internal forces; balancing applied forces have opposite sign. Their vector sum is exactly zero.
T-rotate is an infinitesimal rotation only: `|theta|<=1e-6`; not a finite-rotation/objectivity claim.
T-cancel has exact xy strain `1/2000000000000000000`, not zero; T-min/T-max exercise the supported affine component endpoints.
Geometry/modulus endpoints are also exercised there; the reference includes displacement, strain, stress and every nodal force.
Binary64 cancellation loses relative accuracy in T-cancel; use the mixed tolerance, but also require the shear sign positive (not zero).
At T-min require positive nonzero axial strain/stress and opposite nonzero x forces; otherwise absolute tolerance could hide flushing.
**d3:** a minimal pinned orthotropic membrane-edge coupon, not an isotropic continuum patch.
Pin node 0 at `(tx,ty)`; prescribed nodes 1 and 2 at `(tx+sx,ty)` and `(tx,ty+sy)`.
Rest edges 01 and 02 have length 1, axial rigidity EA=100 N (E=10000 Pa, A=1/100 m^2).
Each edge has `q=max(stretch-1,0)`, end force `-100*q` along its axis, energy `50*q^2`.
Pin reaction is the negative sum of end forces. Move the pin too in C-rigid; compression stores no energy and exerts no force.
**d4:** proposed tensile elastic/perfectly-plastic law, not calibrated steel; L=1, A=1/100, E=1000, yield strain=1/100, failure strain=1/20.
For these tensile paths, `p_new=max(p_old,e-1/100,0)`, force `EA*(e-p_new)`, residual extension `L*p_new`.
Plastic work increment is `EA*(1/100)*L*(p_new-p_old)`; unloading/reloading stays elastic until the tensile yield surface.
Plastic strain is irreversible on these paths; accumulated dissipation cannot decrease. This does not specify reversed compressive plasticity.
At `e>=1/20` localize first crossing to 1/20, update plasticity there, then disconnect once, transferring stored `EA*L*(e-p)^2/2` to dissipation.
Later steps produce no force, plastic update or event; `events` is cumulative disconnection count. Residual denotes the plastic set, even after separation.
Yield below/at/above appears in A-cycle; break below/at/above in A-break. Exact threshold equality fails the link.
**d9:** local x is beam axis; rectangular envelope dimensions along y=1/10, z=1/5 m; L=4 m.
A=1/50 m^2, Iy=1/15000 m^4, Iz=1/60000 m^4; declared effective J=1/20000 m^4 (not an exact rectangular Saint-Venant solution).
Synthetic E=1e9 Pa, nu=1/4, G=E/[2(1+nu)]=4e8 Pa; root fully clamped, tip free.
Unit transverse load: `delta=F L^3/(3 E I)`, slope `F L^2/(2 E I)`, root moment magnitude `F L`; y uses Iz, z uses Iy.
`swap_yz=1` rotates the section about x by pi/2 rad; zero means aligned axes. References are positive magnitudes, not signed vectors.
Axial tip load=100 N: `delta=F L/(EA)`; torsional tip moment=1 N m: `phi=M L/(GJ)`. Energy is half load times its conjugate displacement.
Only L/max(depth,width)>=20, |delta|/L<=1/100, |slope| and |phi|<=1/100 are supported; no shear/large-deflection claim.
**d10:** proposed instantaneous brittle connector; two endpoint masses=1 kg each; scalar mass is invariant at every sample, including failure.
|kind|normal/shear stiffness N/m|rotation stiffness N m/rad|normal/shear strengths N|moment strength N m|
|---|---|---|---|---|
|weld|1000 / 1000|100|10 / 10|1|
|bolt|2000 / 2000|200|20 / 10|2|
Trial `(N,S,M)=(k*u,k*v,kr*theta)`; fail when `|N|/Nmax+|S|/Smax+|M|/Mmax>=1`.
Stored energy `U=(k*(u^2+v^2)+kr*theta^2)/2`; failure transfers this entire trial energy to dissipation and zeroes force/storage.
Samples are displacement-controlled, external work equals change in stored plus dissipated energy; overshoot uses trial energy, not interpolated threshold energy.
Samples occur at times `(index+1)/1000` s. v/theta are constant across u samples; already-broken starts with zero storage/dissipation and stays inert.
Events serialize `link_id:generation:node_a:node_b:failure_time_s`; round-trip preserves IDs exactly and time under the time tolerance.
J-old then J-new reuse slot 3, generations 7 then 8: inject stale_event before J-new's first sample; it must not break generation 8.
J-order creates slots 9 then 2 simultaneously; endpoint pairs belong to sorted IDs 2:(10,11),9:(12,13); report order is (time,ID,generation).
J-order gives forces per sorted link, total energy/mass; disjoint endpoint masses total 4 kg, versus 2 kg otherwise; check events/order and energy too.
## Values for S1 to record in broadsword.toml [node_3_2]
These are required proposed settings, not embedded runtime limits. S1 owns validation and final agreement; no fallback.
Mixed comparison: `abs(actual-reference) <= abs_tol + relative_tolerance*abs(reference)`; finite actual required first.
|key|value|unit / reason|
|---|---|---|
|relative_tolerance|1e-6|dimensionless; plan candidate|
|force_abs_tolerance|1e-8|N; plan candidate|
|displacement_abs_tolerance|1e-9|m; plan candidate|
|strain_abs_tolerance|1e-15|1; cancellation roundoff at affine components <=0.2|
|stress_abs_tolerance|1e-3|Pa; E_max times strain roundoff allowance|
|energy_abs_tolerance|1e-10|J; tabulated U<=5/2 J, far above binary64 roundoff|
|moment_abs_tolerance|1e-10|N m; coupon moments <=4 N m|
|angle_abs_tolerance|1e-12|rad; coupon rotations <=3/6250|
|time_abs_tolerance|1e-12|s; event times rational multiples of 1/1000|
|mass_min / mass_max|1e-6 / 1e6|kg; no zero, negative or subnormal masses|
|length_min / length_max|1e-2 / 1e2|m; characteristic/rest lengths (tetra L, longest edge <=sqrt(2)*L), not coordinates|
|modulus_min / modulus_max|1 / 1e12|Pa; positive elastic material|
|strain_component_min / strain_max|1e-24 / 0.2|1; nonzero primitive affine components; zero valid, cancellation-derived values may be smaller|
|step_min / step_max|1e-8 / 1e-3|s; representability band, not a stability guarantee|
|area_min / area_max|1e-12 / 1e4|m^2; positive finite section/edge area|
|inertia_min / inertia_max|1e-16 / 1e8|m^4; positive finite I and J|
|poisson_min / poisson_max|0 / 0.49|1; avoid incompressible constitutive singularity|
|coordinate_abs_max / displacement_abs_max|1e3 / 1e2|m; signed components; zero valid|
|node_count_max / beam_count_max / fixture_case_count_max|4096 / 32768 / 1024|count; proposed bounded allocation settings|
|step_seconds / run_seconds|required S1 values, unspecified here|s; identities only, disagreements rejected|
|energy_residual_fraction / mass_relative_tolerance|0.05 / 1e-12|unchanged d5 candidates, not measurements|
|penetration_limit_m / permanent_set_min_m|0.01 / 0.01|m; unchanged d5 candidates|
|crush_ratio_min / crush_ratio_max|0.05 / 0.70|unchanged d5 candidates|
|frame_hz / warmup_frames / sample_frames / seed_cases|60 / 120 / 600 / 256|Hz / counts; unchanged plan candidates|
Shape conditioning must also be checked: use FORMAT.md's positive-volume and shape checks, not edge-length bounds alone.
Arithmetic spot-check maxima (PowerShell output): `E_strain_max_Pa=200000000000.0; EA_over_L_max_N_per_m=1000000000000000000; EI_over_L3_max_N_per_m=100000000000000000000000000; section_scale_force_squared_over_mass_max=1E+62`; finite, including step-scaled increments.
Do not infer stability from finiteness; check finite assembled values before any state mutation and reject ill-conditioned/unsupported geometry.
T-min's volume exceeds FORMAT.md's 1e-9 m^3 floor. Independent PowerShell decimal spot-check output:
`car nodes=12 beams=50 mass_kg=120; tet_min_volume_m3=0.0000001666666666666666666667; tet_max_force_N=333333333333333.33333333333333`.
`joint_combined_energy_J=0.012950; axial_break_dissipation_J=0.0045`.
Rationals carry zero reference rounding error; tolerances above budget binary64 operations, not solver error measured by a run.

## Regeneration and inventory (not executed in this sandbox)
Extract the following block to an orchestrator-owned scratch script, then use the task's Broadsword Python route with `python -I`.
It writes nothing: comparison of independently generated bytes is the diff. Empty, missing, duplicate, extra or reordered cases fail.
It prints labelled per-file counts and `diff: no difference` only after all comparisons pass; do not treat this document as a passing run.
Behavioural negative checks: change T-load's force or drop a CSV row in a disposable orchestrator copy; byte diff/inventory must fail, then restore.

```python
from fractions import Fraction as F
from pathlib import Path
import csv
import io

root = Path("tests/fixtures/deform")
tables = {}
def val(x):
    return ":".join(map(val, x)) if isinstance(x, (list, tuple)) else str(x)
def cell(**kw):
    return " ".join(k + "=" + val(v) for k, v in kw.items())
def add(name, cid, inp, **out):
    tables.setdefault(name, []).append((cid, inp, cell(**out)))
def q(x):
    return F(str(x))
for cid, L, E, a, b, c, t in [
    ("load", "1", "600", "1/100", "0", "0", "0"),
    ("translate", "1", "600", "0", "0", "0", "1/10"),
    ("rotate", "1", "600", "0", "-1/1000000", "1/1000000", "0"),
    ("cancel", "1", "600", "0", "1/1000000", "-999999999999/1000000000000000000", "0"),
    ("min", "1/100", "1", "1/1000000000000000000000000", "0", "0", "0"),
    ("max", "100", "1000000000000", "1/5", "0", "0", "0")]:
    L, E, a, b, c, t = map(q, (L, E, a, b, c, t))
    eps = [a, 0, 0, (b+c)/2, 0, 0]
    stress = [E*x for x in eps]
    n, s = stress[0]*L*L/6, stress[3]*L*L/6
    add("tetra", "T-"+cid, cell(L=L, E=E, a=a, b=b, c=c, t=t),
        u0=[t]*3, u1=[t+a*L, t+c*L, t], u2=[t+b*L, t, t], u3=[t]*3,
        eps=eps, stress=stress, f0=[n+s, s, 0], f1=[-n, -s, 0], f2=[-s, 0, 0], f3=[0]*3)
for cid, sx, sy, tx, ty in [("stretch","11/10","6/5","0","0"), ("rigid","1","1","1/10","-1/10"),
                            ("compress","9/10","4/5","0","0"), ("mixed","9/10","11/10","0","0")]:
    sx, sy, tx, ty = map(q, (sx, sy, tx, ty))
    x, y = max(sx-1, 0), max(sy-1, 0)
    add("cloth", "C-"+cid, cell(sx=sx, sy=sy, tx=tx, ty=ty),
        f1=[-100*x, 0], f2=[0, -100*y], reaction=[100*x, 100*y], energy=50*(x*x+y*y))
for cid, path in [("elastic","0:9/1000:0:1/200:0"),
                  ("cycle","0:9/1000:1/100:11/1000:3/100:1/50:1/40:1/25:3/100"),
                  ("break","0:49/1000:1/20:51/1000:0:3/50")]:
    p, D, broken = F(0), F(0), False
    forces, residual, dissipated, events = [], [], [], []
    for e in map(q, path.split(":")):
        force = F(0)
        if not broken:
            e = min(e, F(1, 20))
            pn = max(p, e-F(1, 100), 0)
            D += (pn-p)/10
            p, force = pn, 10*(e-pn)
            if e == F(1, 20):
                D += 5*(e-p)**2
                broken, force = True, F(0)
        forces.append(force); residual.append(p); dissipated.append(D); events.append(int(broken))
    add("axial", "A-"+cid, "strain="+path, force=forces, residual=residual, dissipated=dissipated, events=events)
for cid, mode, swap in [("y","y",0), ("z","z",0), ("y90","y",1), ("z90","z",1), ("axial","axial",0), ("torsion","torsion",0)]:
    if mode in ("y", "z"):
        I = F(1, 60000) if (mode == "y") == (swap == 0) else F(1, 15000)
        d, r, m, load = F(4**3, 3*10**9)/I, F(4**2, 2*10**9)/I, 4, 1
    elif mode == "axial":
        d, r, m, load = F(100*4*50, 10**9), 0, 0, 100
    else:
        d, r, m, load = 0, F(4*20000, 4*10**8), 1, 1
    add("section", "S-"+cid, cell(L=4, A=F(1,50), Iy=F(1,15000), Iz=F(1,60000), J=F(1,20000), E=10**9, nu=F(1,4), mode=mode, swap_yz=swap), displacement=d, rotation=r, moment=m,
        energy=load*(r if mode == "torsion" else d)/F(2))
for cid, kind, lid, gen, dead, path, v, theta in [
    ("weld","weld",3,0,0,"9/1000:1/100:11/1000:0","0","0"),
    ("bolt","bolt",4,0,0,"9/1000:1/100:11/1000","0","0"),
    ("combined","weld",5,0,0,"1/250","3/1000","3/1000"),
    ("above","weld",6,0,0,"11/1000","0","0"),
    ("old","weld",3,7,1,"1/10","0","0"), ("new","weld",3,8,0,"9/1000:1/100","0","0"),
    ("order","weld",9,0,0,"1/100","0","0")]:
    k, kr, Nmax, Smax, Mmax = (1000,100,10,10,1) if kind == "weld" else (2000,200,20,10,2)
    v, theta, D = q(v), q(theta), F(0)
    ns, ss, ms, us, ds, ev = [], [], [], [], [], []
    for i, u in enumerate(map(q, path.split(":"))):
        N, S, M, U = (F(0),)*4
        if not dead:
            N, S, M = k*u, k*v, kr*theta
            U = (k*(u*u+v*v)+kr*theta*theta)/2
            if abs(N)/Nmax+abs(S)/Smax+abs(M)/Mmax >= 1:
                D += U
                ev.append(val([lid, gen, 10, 11, F(i+1,1000)]))
                N, S, M, U, dead = F(0), F(0), F(0), F(0), True
        ns.append(N); ss.append(S); ms.append(M); us.append(U); ds.append(D)
    inp = cell(kind=kind, id=lid, generation=gen, broken=int(cid == "old"), u=path, v=v, theta=theta, k=k, kr=kr, Nmax=Nmax, Smax=Smax, Mmax=Mmax)
    if cid == "new": inp += " stale_event=3:7:10:11:1/1000"
    mass = 2
    if cid == "order":
        inp = cell(kind=kind, ids=[9,2], generation=0, broken=0, u=path, v=v, theta=theta, k=k, kr=kr, Nmax=Nmax, Smax=Smax, Mmax=Mmax)
        ns, ss, ms, mass = [0,0], [0,0], [0,0], 4
        ds = [2*D]
        ev = [val([2,0,10,11,F(1,1000)]), val([9,0,12,13,F(1,1000)])]
    add("joint", "J-"+cid, inp, normal=ns, shear=ss, moment=ms, stored=us, dissipated=ds, mass=mass, events=";".join(ev) or "none")
for name, cid, payload, error in [
    ("tetra","T-count","count:18446744073709551616","CountOverflow"),
    ("tetra","T-duplicate","node_ids:0:1:1:3","DuplicateId"), ("tetra","T-nan","E:NaN","NonFinite"),
    ("tetra","T-overflow","L:1e309","NonFinite"), ("tetra","T-subnormal","L:5e-324","UnsupportedSubnormal"),
    ("cloth","C-area-zero","area:0","NonPositiveArea"), ("cloth","C-area-negative","area:-1","NonPositiveArea"),
    ("axial","A-order","yield:1/20;break:1/100","StrainOrdering"),
    ("axial","A-mass-zero","mass:0","NonPositiveMass"), ("axial","A-mass-negative","mass:-1","NonPositiveMass"),
    ("section","S-band","L_over_depth:1","OutsideBeamBand"), ("joint","J-time","time:-1","InvalidTime")]:
    add(name, cid, "invalid="+payload, error=error)
inventory = [line.split() for line in (root/"manifest.txt").read_text(encoding="utf-8").splitlines() if line.startswith("inventory ")]
assert len(inventory) == len(tables) and len({x[1] for x in inventory}) == len(tables), "inventory files"
for _, filename, check, count, *ids in inventory:
    assert filename == (root/Path(filename).name).as_posix(), "manifest path"
    assert check == dict(tetra="d2", cloth="d3", axial="d4", section="d9", joint="d10")[Path(filename).stem], "check identity"
    rows = tables[Path(filename).stem]
    assert ids and len(ids) == int(count) == len(set(ids)) == len(rows), "inventory count"
    assert ids == [r[0] for r in rows], "inventory IDs"
    stream = io.StringIO(newline="")
    writer = csv.writer(stream, lineterminator="\n")
    writer.writerow(["case_id","input","expected"]); writer.writerows(rows)
    expected = stream.getvalue().encode("utf-8")
    actual = Path(filename).read_bytes()
    assert actual == expected, "byte diff: "+filename
    parsed = list(csv.reader(io.StringIO(actual.decode("utf-8"))))
    assert ids == [r[0] for r in parsed[1:]], "CSV IDs"
    print(f"{check} {filename}: cases={len(rows)} exact inventory")
print("diff: no difference")
```

Orchestrator done checks (not run here): `build/bin/broadsword.exe test crash-sim --task t-20261122-033600` -> `test: ok`;
`git diff --stat main` -> only seven added claimed paths; regenerate with `python -I` through Broadsword and paste no-difference/count output.
The same block is the counting command: print all five labelled inventories, never a success token on empty output.
