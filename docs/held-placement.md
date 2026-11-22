# Held placement with the Crazyflie

`held_trial(family, p, placement, sample, gripper_cfg)` follows one brick from its pickup fixture to its final pose. The Crazyflie, wrist, fork, brick and supporters share one MuJoCo scene. Only contact between the fork and brick carries the brick. Read `phases` for the individual checks and `passed` for the whole mission. All contact results are **simulated, uncalibrated contact**. [Implementation](../src/brixzle/engine/sim/held.py)

```python
from brixzle.engine.sim.held import HeldConfig, HeldRig, HeldSample, mission_samples

rig = HeldRig(family, family.defaults, "plus_bare", HeldConfig())
mission = rig.trial(seed=191)              # Independent empirical station and target errors
aligned = rig.trial(HeldSample())          # Explicit target error uses an aligned pickup
_, target = mission_samples(191)
placement = rig.trial(target)             # Isolate this target error with an aligned pickup
```

Reuse the rig for multiple samples of a seat. Each call restores the settled fixture and supporters. The full-seat courses, both bare edges and the corbel's reaching seats use the drop rig's layouts and supporter reference. An explicit `HeldSample` takes world offsets in metres and angles in radians. Pass `pickup_sample=` to override station error independently. For a secondary radius study, pass explicit X/Y offsets from `objective.radius_error` and use a fixed seed. [Seat layouts](../src/brixzle/engine/sim/trials.py) · [Disc sampler](../src/brixzle/engine/objective.py)

## Mission checks

| Phase | What the trial checks |
|---|---|
| Take-off | Compute maximum thrust divided by the weight of the drone, deck and empty gripper. A value below the drone's configured minimum stops the mission at the pad. |
| Approach | Start airborne outside the station and track a short trajectory to the error-displaced standoff. Record the drone's actual tracking error. A drone can reach its commanded standoff and still miss the windows. |
| Grip | Move the entire fork along drone-local +Y into the stationary brick. Check each shaft's depth and intersection with its own window or bore, then lift under payload compensation. Measure rise, contact load, carry offset and angle. |
| Transport | Fly the loaded drone to the target. Check combined payload mass and loaded thrust-to-weight. Measure carry offset and angle throughout acceleration and braking, and check engagement again at arrival. A lost brick skips release. A brick still on the fork can continue to provide placement evidence even if its alignment failed. |
| Release | Descend at the configured speed, stop at the force or depth limit, unload, fly along drone-local −Y and climb. Check that the shafts clear the brick, measure the brick's maximum drift through withdrawal and climb, then settle it while the departed drone hovers. |

R10 limits the payload to **35 g**. R12 caps the extra force the drone supplies at half its loaded thrust margin, `0.5 * (thrust_max_N - loaded_weight_N)`. The gripper setting `press_force_N` can lower that cap. The controller also limits commanded thrust and stops advancing on an instantaneous support reaction at the threshold, so it can respond to touchdown promptly. [R10 and R12](requirements.md) · [Drone budget](../src/brixzle/engine/sim/drone.py)

The release verdict uses `sustained_push_N`. At each step the trial sums world-frame contact forces **on the fork**, including friction and direct strikes. It averages that vector over the last 50 ms and takes the upward component above the gripper's own weight, with a floor of zero. The maximum from descent through withdrawal and climb is the downward push charged to the drone. Measuring on the fork side leaves the seated brick's weight on its support. The instantaneous peak reaction on the whole assembly (`peak_support_N`) and the peak sum of contact normals (`peak_contact_normals_N`) record impacts. [Release force calculation and tests](../src/brixzle/engine/sim/held.py) · [Zero-press and overpressure trials](../tests/test_held.py)

During grip, `sustained_engage_N` uses the same 50 ms mean force vector. The fork can carry part of the brick's weight or rest part of its own gripper weight on the fixture. The trial discounts that vertical weight transfer and counts the full lateral load and any excess vertical load. It compares the largest resulting magnitude during insertion and hold with the cap. `peak_engage_N` records the instantaneous sum of fork contact normals. Both phases expose their force verdict as `within_force_limit`. [Grip force calculation](../src/brixzle/engine/sim/held.py)

After departure, `post_departure_outcome`, `err_mm` and `angle_deg` use the drop classifier, including its `wrong_valley` class. A seated result requires settling. A retained fork or failed climb is a jammed release with a phase reason and a null post-departure outcome. A skipped release has a null overall placement outcome. `before_release`, `stayed_seated` and `cleared_along_y` show what happened during withdrawal. The diagnostic combination `WristConfig(rigid=True)` and `release_at_contact=True` removes the fork at first brick/support contact, preserves the brick's momentum and is excluded from a mission pass. [Acceptance tests](../tests/test_held.py)

## Inputs and physical limits

| Source | Use in the model |
|---|---|
| Measured flight approaches | Bootstrap an observed Exp 3 X/Y radius in a uniformly random direction. Station and target use independent draws. [Approach model](../src/brixzle/engine/sim/approach_error.py) · [Exp 3 radii](../src/brixzle/data/error-exp3-r4.json) |
| Estimated from measured unloaded hover | Draw one recorded Z, pitch and yaw RMS triplet per approach. Assign independent random signs, a modelling assumption. These estimates are probably too small during pickup and release. Z uses the drone's own Lighthouse estimate. [Hover observations](../src/brixzle/data/hover-exp3-r2.json) · [Measurement limitations](requirements.md#target-boxes) |
| Datasheet and crazyflow | Use the configured motor thrust source (datasheet by default), the Crazyflie 2.1 Brushless body and deck masses, body inertia and crazyflow Mellinger position gains. Acceleration feedforward and position feedback apply force at the free body's centre of mass. The separate attitude spring is an estimate. The fork's offset, brick contact and wrist reactions load the drone directly. The controller approximates the inner flight loop with a force and torque, without modelling individual rotors. [Drone parameters and provenance](../src/brixzle/engine/sim/drone.py) · [Shared pickup controller](../src/brixzle/engine/sim/pickup.py) |
| Printed parts and fixture assumptions | The rigid fork uses pure PLA geometry and mass properties. The compliant wrist represents TPU. Its linear and angular stiffness, damping, lumped mass and inertia are a **guess**. Compatible windows take a straight full-depth fork. On the unchanged campaign brick, the V across the depth obstructs those tines. The trial fits a single fixed fork at an oblique yaw relative to the brick, reaching beyond its mid-plane. The shafts are parallel to the drone's Y axis. The fork shape, speed profile, station spacing and campaign fixture saddles are simulation design choices pending a print. [Fork model](../src/brixzle/engine/sim/fork.py) · [Settings and fixture](../src/brixzle/engine/sim/held.py) · [Campaign geometry](../tests/fixtures/gentle_inboard_sill/family.py) |

Known sim-to-real gaps include rotor downwash, ground effect near the structure, estimator noise beyond the measured hover RMS, and uncalibrated contact and wrist properties. Bench test #193 should measure the printed wrist stiffness and damping and include a hand or arm placement check with the printed gripper and brick before flight. Any real flight test belongs to the maintainer in aerial-assembly. [Bench test #193](https://github.com/LunarSphere/Brixzle-RSI/issues/193)

## Recorded trials

**Simulated, uncalibrated contact.** The aligned full-seat trial on the unchanged `gentle_inboard_sill` took **3.756 s** on a compiled scene. First scene setup, including CAD and fixture settling, took **3.242 s**. The brick stayed seated through withdrawal and climb. The gripper failed the overall mission. Wall-clock for that trial and the measurements behind each verdict appear below. [Run inputs, outcomes and timings](../runs/stageb/held191-force-MaGElm/result.json) · [Reproduction script](../runs/stageb/held191-force-MaGElm/reproduce.py)

| Phase | Aligned full-seat result | Wall-clock |
|---|---|---:|
| Take-off | Passed the computed thrust-to-weight check | 0.000002 s |
| Approach | Reached the aligned station standoff | 0.139 s |
| Grip | Both tines entered. Sustained engagement was 0.0035 N, with a 0.0681 N peak sum of normals. Carry alignment exceeded the R12 tolerance | 0.461 s |
| Transport | The payload was 26.52 g and loaded thrust-to-weight was 1.65. One tine missed the required engagement depth at arrival | 0.228 s |
| Release | Passed. The brick stayed seated. Sustained downward push was 0.000 N against the 0.232 N cap. Peak support reaction was 1.039 N | 2.929 s |

The full empirical mission at seed **191** missed pickup and skipped transport and release. With pickup aligned and the **same empirical target error**, the full seat seated, the positive bare edge jammed, the negative bare edge retained the fork, and the brick fell from the first corbel reach. The secondary corbel checks at disc radii of 0, 2.5 and 5 mm each seated after departure. Each cell is a single observation. [Per-cell samples, phase verdicts and wall-clock](../runs/stageb/held191-force-MaGElm/result.json)

Moving the aim one voxel pitch in X produced `wrong_valley` after departure. A 90 mm Y miss retained the fork. At a 2 mm X offset the rigid first-contact diagnostic and its free-drop reference both seated. [Recorded comparison](../runs/stageb/held191-force-MaGElm/result.json) · [Regression tests](../tests/test_held.py)

## Scoring a candidate with heldmap

Run the following from the checkout. The command prints scores, phase counts and timings, and writes the individual trials and build histories to a fresh run directory. The result records the family parameters, resolved gripper settings, error sources and seed. Contact results are **simulated, uncalibrated contact**. The TPU wrist stiffness and damping defaults are a **guess** pending #193. [Tool](../src/brixzle/tools/__main__.py) · [Objective and sampling settings](../src/brixzle/engine/heldmap.py)

```sh
uv run --no-sync python -m brixzle.tools heldmap tests/fixtures:gentle_inboard_sill
```

| Measurement | Station approach | Target approach | Read in result.json |
|---|---|---|---|
| Release Z on the full seat (both courses), each bare edge and the first corbel reach | Aligned physical pickup | Empirical target draws inside the mean X/Y disc, Z and pitch bounds | `cases`, `scores` |
| End-to-end performance on the same seats | Empirical | Independent empirical draw over the full distribution | `missions`, `mission_passed`, each trial in `rows` |
| First-reach stress test | Aligned physical pickup | Seeded disc at each radius, retaining the measured hover draw | `first_reach_by_radius` |
| Sequential back-first corbel | Empirical for every brick | Independent empirical draw for the primary build. The secondary study replaces only target X/Y with the radius disc | `builds.measured`, `median_steps_by_radius`, `builds` |

All five phases run for each attempt until a failure prevents continuation. Each trial reports each phase's verdict and wall-clock. The phase summaries count passed, failed and skipped attempts separately. A seat earns +1, a jam earns −1, and all other outcomes, including a skipped release after a missed pickup, earn zero. Every attempt counts in the denominator. A brick seated after an excessive contact force can contribute to release Z while the mission fails. The radius study uses common seeds across radii so a change in target radius can be compared with the same pickup. [Held trial](../src/brixzle/engine/sim/held.py) · [Drop Z convention](../src/brixzle/engine/zscore.py) · [Error draws](../src/brixzle/engine/heldmap.py)

Each corbel starts with an empty base. Subsequent placements inherit the measured positions, orientations and velocities of all previously seated bricks. A missed placement or a brick leaving its supporter ends that build. At each length the success rate is the fraction of seeded builds that reached it with every brick seated. **Reliable reach** is the longest length reached by a strict majority. Course zero sits on the base and contributes zero horizontal reach. Each subsequent complete reaching course adds one voxel pitch. R4 requires three protruding reaching bricks. The report also gives the reliable reach restricted to builds that passed every mission phase through that length. The aggregate mission verdict requires every empirical key-seat mission to pass, the payload to fit, and a majority of builds to pass every phase up to the R4 length. [R4 and R10](requirements.md) · [Corbel layout](../src/brixzle/engine/structures.py) · [Score implementation](../src/brixzle/engine/heldmap.py)

Place an optional `gripper.json` alongside the candidate's `family.py`. For a standalone comparison, `--gripper PATH` selects a replacement configuration. Use SI values for the scalar fork and motion fields of `HeldConfig` and a nested `wrist` object for `WristConfig`. For example, `{"tine_d_m": 0.002}` selects the default fork diameter. The mounted fork must fit across courses with the same shaft and plate geometry. The simulated payload includes the brick, PLA fork and lumped TPU wrist. The drone, contact physics and diagnostic first-contact removal are fixed for scoring. [Gripper settings and material assumptions](../src/brixzle/engine/sim/held.py) · [Fork dimensions](../src/brixzle/engine/sim/fork.py)

### Running a director campaign

Use a settings file when initializing the director, then start the Codex driver on the initialized run. For example, the following settings give the archive a measured reach term, release Z on the key placements, and a whole-mission term.

```json
{
  "stage": "held",
  "scored": ["z_target", "held_reliable_reach_mm", "held_full_z", "held_plus_bare_z",
             "held_minus_bare_z", "held_first_reach_z", "held_mission_rate"]
}
```

```sh
uv run --no-sync python -m brixzle.director init --run runs/rsi/stage-b --goal 'Improve reliable reach' --baseline tests/fixtures:gentle_inboard_sill --settings settings.json
uv run --no-sync python -m brixzle.director.drive --run runs/rsi/stage-b
```

Baseline and candidate verification both run heldmap for a held goal. Claims can reference `held.scores.held_full_z` or `held.mission_passed`. The verifier saves the full report under `measured/held/result.json`. The archive uses the verified scores, the stage-A constraints, gate and ring, and the held payload and R4 checks. Stage B takes the baseline's verified mean drop Z as the candidate floor unless the maintainer sets `min_drop_z` in the goal. Additional scored terms include `held_reliable_steps`, `held_mission_reach_mm`, `held_overhang_mean_steps` and a median for each scoring radius such as `held_median_steps_5mm`. A gripper-only brief can name a change unit such as `gripper:tine_d_m` or `gripper:wrist.linear_k`. [Verifier](../src/brixzle/director/verify.py) · [Director and archive wiring](../src/brixzle/director/__main__.py) · [Change checks](../src/brixzle/director/change.py)

The step offset is still one voxel on the two-voxel brick. A smaller positive offset requires extending the lattice representation or supporting longer-span bricks, with corresponding changes to support connectivity and expected seat poses. The reach report uses the family's actual pitch. [Structure assumptions](../src/brixzle/engine/structures.py) · [Voxel connectivity](../src/brixzle/engine/sim/assemble.py)

### Recorded heldmap on gentle_inboard_sill

The default heldmap at seed **192** took **273.46 s** end to end, including process startup and writing the report. The evaluator recorded **271.18 s** internally. [Command and process timing](../runs/stageb/held192-20261122T214933984234Z/wall.json) · [Evaluator timing and inputs](../runs/stageb/held192-20261122T214933984234Z/heldmap-20261122T215407253674Z-gentle_inboard_sill/result.json)

**Simulated, uncalibrated contact.** All ten empirical key-seat missions passed the computed take-off check. Five passed approach tracking. All ten missed the grip and skipped transport and release. All three measured builds stopped at the first pickup, giving **0 mm reliable reach**. With pickup aligned, full-seat Z was **0** (two seats and two jams), positive bare-edge Z was **+0.5** (one seat and one fall), and negative bare-edge and first-reach Z were both **0** (one seat and one jam each). These cases reuse the same seeded approaches across layouts. [Trial rows, phase timings and builds](../runs/stageb/held192-20261122T214933984234Z/heldmap-20261122T215407253674Z-gentle_inboard_sill/result.json)
