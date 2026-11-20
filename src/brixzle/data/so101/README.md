# SO-101 arm model (copied in, issue #152)

**Source:** [google-deepmind/mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie), `robotstudio_so101/`, at commit `370c43965323c0944b433b202aac5058b5f907a9`. Licensed under Apache-2.0; see `LICENSE`.

Menagerie derived this model from TheRobotStudio's `so101_new_calib.xml` (SO-ARM100 at commit `aec17bbc256d1a7342d53aaa4950595d4c30b40d`).

**What was copied:** `so101.xml` (unchanged apart from a source header) and the 18 meshes it references in `assets/`. The photo, the scene files and the 2 unreferenced meshes were left out.

## Why menagerie's version and not the original

I compared the two files on 2026-11-19. Both give the STS3215 servos the same position gains: `kp="998.22" kv="2.731"`, and the same joint damping, friction loss and armature. A third-party note claims a 56× difference in kp; that is not true of these two files.

| | TheRobotStudio original | menagerie (used) |
|---|---|---|
| Actuator force limit | ±3.35 N·m on each actuator | ±2.94 N·m (class default) |
| Gripper collision | jaw meshes only | primitive boxes, spheres and capsules on both jaws, plus convex jaw meshes, with contact settings tuned for manipulation |
| Integrator | Euler | implicitfast |

The primitive gripper collision shapes are the reason for the choice: grasping bricks (#153) depends on jaw contact.

## How brixzle uses it

`brixzle.engine.sim.arm` attaches this model to a brick scene through `mujoco.MjSpec`, under the prefix `arm/`. The brick scene keeps its own `<option>` settings, such as the 1 ms step, and the arm's are not used.
