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

R10 limits the payload to **35 g**. R12 supplies the carry and drift tolerances and caps contact at **half the thrust remaining after hovering with the payload**, calculated as `0.5 * (thrust_max_N - loaded_weight_N)`. A gripper setting can lower the cap. The controller bounds the thrust vector and the commanded downward load during pressing. The trial also measures contact impulses. For the release check it sums the external contact force vectors on the brick and fork in world coordinates, including friction and direct fork strikes. A peak resultant above the cap fails release even if the brick seats. The report also gives the sum of contact normal magnitudes. Opposing forces on guide faces can make that sum much larger than the load opposing the drone. [R10 and R12](requirements.md) · [Drone budget](../src/brixzle/engine/sim/drone.py) · [Pickup tolerances](../src/brixzle/engine/sim/pickup.py)

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

**Simulated, uncalibrated contact.** The aligned full-seat trial on the unchanged `gentle_inboard_sill` took **3.549 s** on a compiled scene. First scene setup, including CAD and fixture settling, took **3.298 s**. The brick stayed seated through withdrawal and climb. The gripper failed the overall mission. Wall-clock for that trial and the measurements behind each verdict appear below. [Run inputs, outcomes and timings](../runs/stageb/held191-replay-zvndi30p/result.json) · [Reproduction script](../runs/stageb/held191-replay-zvndi30p/reproduce.py)

| Phase | Aligned full-seat result | Wall-clock |
|---|---|---:|
| Take-off | Passed the computed thrust-to-weight check | 0.000004 s |
| Approach | Reached the aligned station standoff | 0.141 s |
| Grip | Both tines entered and lifted the brick. Carry alignment exceeded the R12 tolerance | 0.442 s |
| Transport | The payload was 26.52 g and loaded thrust-to-weight was 1.65. One tine missed the required engagement depth at arrival | 0.232 s |
| Release | Seated after departure. The peak support reaction was 1.039 N against a 0.232 N cap | 2.735 s |

The full empirical mission at seed **191** missed pickup and skipped transport and release. With pickup aligned and the **same empirical target error**, the full seat seated, the positive bare edge jammed, the negative bare edge retained the fork, and the brick fell from the first corbel reach. The secondary corbel checks at disc radii of 0, 2.5 and 5 mm each seated after departure. Each cell is a single observation. [Per-cell samples, phase verdicts and wall-clock](../runs/stageb/held191-replay-zvndi30p/result.json)

Moving the aim one voxel pitch in X produced `wrong_valley` after departure. A 90 mm Y miss retained the fork. At a 2 mm X offset the rigid first-contact diagnostic and its free-drop reference both seated. [Recorded comparison](../runs/stageb/held191-replay-zvndi30p/result.json) · [Regression tests](../tests/test_held.py)
