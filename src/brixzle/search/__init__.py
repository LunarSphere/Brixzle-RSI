"""Script-driven search over the ``prim`` family's parameter region (#166).

Candidates are sampled (a scrambled Sobol sequence over ``prim``'s bounds) and pass down a funnel of
stages, each costlier than the last. A candidate reaches the design agent, or a full evaluation,
only when it "makes sense" by every cheaper check:

0. **Rules** (``stage_rules``): the parameters' own validation (R6 guide angles, a hook inside the
   ramp's cone, a body thick enough for its slot), and analytic bounds from #164's objective: length
   2U, an estimated release lift and an estimated mass (``objective.Settings``, R9, R10).
   Predicted, quasi-static 2D. Microseconds per candidate.
1. **Geometry and predictor** (``stage_predict``): the exact section, release lift and rule margins
   (``families.rules``), then the analytic predictor (engine/predictor) at the aim and at X offsets
   of +-r (r = ``predict_r_mm``) on both courses. All must seat. Predicted, quasi-static 2D.
2. **Reduced drops** (``stage_drops``): MuJoCo drops at a small fixed set of key poses: the aim, X
   offsets across the mean box, Y offsets, a high release and a pitch, on both courses. The score is
   (S - J) / N, the form of Z. Simulated, uncalibrated contact.
3. **Build and grip** (``stage_build``): the half-supported step each way (structure_gate.step_check)
   and the strict stock-jaw grip rule (objective.grip_rule).
4. **Full evaluation**: survivors only, through ``tools.core.evaluate``, as a package written under
   the search's run directory, followed by the build-aware objective for feasible designs.

``run_search`` reports, for every stage, how many candidates went in and out and its evaluations per
second. Add ``--reference <objective-baselines/report.json>`` to judge qualified candidates against
a fixed Pareto front. Without a reference, full evaluation reports feasibility.

Run ``python -m brixzle.search --calibrate N --seed S --reference REPORT`` to evaluate the same
seeded sample at every stage independently. Invalid parameter vectors and unevaluable designs are
reported separately. Rates are conditional on full-evaluation keepers, so a sample with zero keepers
has undefined rates. Use a small explicit sample because a full evaluation can take minutes.
"""
