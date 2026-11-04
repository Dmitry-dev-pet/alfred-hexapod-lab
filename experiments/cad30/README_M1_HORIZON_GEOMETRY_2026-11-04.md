# Math-first M1: Pose B lookahead, geometry-only (2026-11-04)

Related: Issue #26 (MPC horizon), Issue #21 (actual 4-leg gait goal), draft PR #25 (verified 6→4 pose B).

## What was calculated

Source: **owner's archived MuJoCo CAD30 replay**, *not* generated gait or animation: Google Drive
[Hexapod_Middle_Leg_Step_Probe_2026-11-04_no_CAD.zip](https://drive.google.com/file/d/16nMoh9fS5wu0BnJj_sbbHld2TpytEbRg/view), \`viewer/middle-step-negative.json\`, frame index **92** at **t = 9.3 seconds**, 4 supporting toe markers \`ml/mr/bl/br\`, with COM from the pose-B \`semi_tucked.after_tuck\` receipt of PR #25. Coordinates are in world meters; forward is +Y. Original CAD/STL assets are **not** copied.

**Do not confuse toe marker points with the two MuJoCo collision contacts per tibia or their force-weighted centroid.** The polygon based on toe markers gives ~26.61 mm current margin, close to but not identical with the previously logged physically contact-derived ~26.23 mm.

In \`m1_geometry_oracle.py\`:
1. Form the **future** three-toe support triangle *before* choosing the next lifted leg; calculate exact planar signed margin to its sides.
2. For each of four possible first lifts, project COM to the triangle interior with an exploratory 5 mm margin, minimizing translation distance. This deliberately ignores IK and torques.
3. Add an optimistic but **necessary** kinematic reach inequality: hip-to-toe distance ≤ 233 mm (=33+90+110 mm), while freezing torso attitude and height and allowing pure translation of hip positions. Relax each 2D disk of possible COM positions by its 96-side *circumscribed tangent polygon* (a superset). A margin-maximization LP whose optimum is negative **excludes** nonnegative-margin solutions under these restrictive assumptions.
4. Enumerate all 24 permutations of 4 steps, where each lifted toe lands a fixed distance **straight forward (+Y)**. At each liftoff/touchdown, all four toe markers must be within this optimistic reach ball, at swing COM must have ≥5 mm margin inside the other three-toe triangle, and final COM must advance ≥20 mm beyond the original COM. This is a **linear relaxed necessary-condition** test, NOT a trajectory controller.

## Numerical findings

| First toe lifted | Current 3-toe signed margin | Smallest 2D COM shift for ≥5 mm margin (ignoring reach) | LP **upper bound** on maximum margin with all 4 original feet reachable, frozen torso |
| --- | ---: | ---: | ---: |
| \`bl\` rear left | +26.61 mm | 0 mm | +64.60 mm |
| \`br\` rear right | +26.61 mm | 0 mm | +64.55 mm |
| \`ml\` middle left | −86.76 mm | 91.76 mm | **−14.04 mm** |
| \`mr\` middle right | −86.86 mm | 91.86 mm | **−14.13 mm** |

As expected, **rear-first** is geometrically much easier from pose B. More importantly, the fixed-toe/fixed-attitude model cannot reach a positive-margin 3-support state before directly lifting a middle foot, even with the generous 233-mm reach upper bound. This does **not** mean the real robot cannot lift a middle foot after modifying placements and body attitude.

For a full sequence of four different legs, straight +Y landings of **20 mm** or **40 mm** per swing, final COM advance ≥20 mm, 5 mm reserve and the same rigid frozen-body reach: **0/24 orders admitted** at each stride. The reach constraints are **relaxed outward**, so LP infeasibility is meaningful *only within this narrowly defined parameterization*. No arbitrary foothold positioning, side steps, changing body pitch/height, reconfiguration during 6-support preparation, true joint IK, force balance, slip or dynamic feasibility were searched.

**This is not a proof of general non-walkability.** In particular, a smarter four-step planner must select touchdown (x,y), body attitude/height, and preceding stance placements to make future 3-support states viable; this motivates actual horizon planning instead of greedy balancing.

## How to reproduce

With NumPy/SciPy installed, run from \`experiments/cad30/\`:

\`\`\`bash
python m1_geometry_oracle.py
python -m unittest -v test_m1_geometry_oracle.py
\`\`\`

Local initial validation: **5/5 unit tests PASS**. No MuJoCo or GitHub Actions compute needed. Outputs are a compact JSON stdout receipt; no 3D viewer and no motor claim.

## Gates before a gait claim

Next M1 iteration must free **2D foothold positions** and **6D torso state** (at minimum height/pitch/roll) and explicitly verify full IK/joint limits and contact force/torque feasibility over *all* transitions. After a full mathematically viable cycle is found, check it by free-base MuJoCo dynamic integration and honest contact/margin scoring at every physics step. Keep original traces/media on Drive with SHA-256.

**Status:** negative finding for a restricted necessary-condition baseline. Issue #26 remains open; no merge to main without owner review.
