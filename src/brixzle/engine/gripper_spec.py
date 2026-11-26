"""The gripper interface every brick designs against (#206).

The gripper is designed and bench-tested by hand, outside the loop: a passive magnet under the
drone, released by yawing the drone about 30 deg once the brick is seated, so the magnet shears off
a steel insert in the brick. Pickup error is absorbed by a landing pad, not by the brick.

**Every value here is a guess until the bench tests** (magnet pull and shear through a printed
spacer, the Crazyflie's yaw torque). Change them here, in a reviewed PR, when the tests are done.

A brick shows its magnet target as a pickup feature (``family.pickup``) of kind ``MAGNET_TARGET``:
``x_mm`` and ``z_mm`` its centre in the section frame (on the top face), ``size_mm`` its (width
along X, height of the insert pocket).
"""
from dataclasses import dataclass

MAGNET_TARGET = "magnet_target"


@dataclass(frozen=True)
class GripperSpec:
    gripper_mass_g: float = 10.0     # magnet, mount and probe under the drone (guess)
    insert_mass_g: float = 0.5       # the steel insert pressed into the brick (guess)
    min_target_mm: float = 8.0       # the target's width along X, so the magnet lands on it (guess)
    max_com_offset_mm: float = 5.0   # target centre to the brick's centre of mass, along X (guess)
    payload_max_g: float = 35.0      # brick, insert and gripper together (R10)
    brick_max_g: float = 25.0        # brick with its insert (R9)


SPEC = GripperSpec()


def target_check(family, p, spec: GripperSpec = SPEC):
    """Violations of the gripper interface for every part, and what was found."""
    from brixzle.families import builders as B

    out, found = [], {}
    for part in family.parts:
        targets = [f for f in family.pickup(p, part) if f.kind == MAGNET_TARGET]
        bodies = B.sim_bodies(family, p, part)
        com_x = float(bodies.com_mm[0]) if getattr(bodies, "com_mm", None) is not None else None
        found[part] = [{"x_mm": f.x_mm, "z_mm": f.z_mm, "size_mm": list(f.size_mm)} for f in targets]
        if not targets:
            out.append(f"{part}: no magnet target")
            continue
        best = max(targets, key=lambda f: f.size_mm[0])
        if best.size_mm[0] < spec.min_target_mm:
            out.append(f"{part}: magnet target {best.size_mm[0]:.1f} mm wide, needs {spec.min_target_mm}")
        if com_x is not None and abs(best.x_mm - com_x) > spec.max_com_offset_mm:
            out.append(f"{part}: magnet target {abs(best.x_mm - com_x):.1f} mm from the centre of mass, "
                       f"at most {spec.max_com_offset_mm}")
        total = float(bodies.mass_g) + spec.insert_mass_g
        if total > spec.brick_max_g:
            out.append(f"{part}: {total:.1f} g with its insert, at most {spec.brick_max_g}")
    return {"violations": out, "targets": found, "spec": spec.__dict__}
