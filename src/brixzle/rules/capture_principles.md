# Capture principles for explore rounds

Issue #106. In an explore round, each design call is given one of these principles to build a new
brick on. They are principles, not designs: each says what the brick must do and why, and cites the
design rules (`design_rules.md`) it rests on. Never copy a brick, joint or structure from a paper or
from another group's system (CLAUDE.md); the maintainer's own A/B may be reused.

Each entry is `## CPn. Title`, then the principle, the failure it targets and what it costs.

## CP1. A funnel that centres in X and Y together

The guide faces lead the brick down towards its seat from any start inside the error disc, in X and
in Y at once, and the X interlock runs across the Y guide (a shear of the section does this), so
neither guide has to finish before the other can act. Targets: jams and falls in the X-Y slice.
Costs: depth and mass for the Y guide (DR1, DR7, DR9).

## CP2. One seat only: a key with no offset rest

The interlock has no periodic pattern a brick can land in one voxel off, so a wrong valley does not
exist; anywhere else the brick slides on. Targets: wrong valleys. Costs: the key must still lay a
running bond, so it repeats per brick, not per voxel (DR5, DR8).

## CP3. A seat stable only at the aim

The brick rests on three or four contacts whose friction-free equilibrium exists at the seat alone;
away from it, the contacts tip or slide it towards the seat. Targets: jams (stuck on a face) and
wrong valleys. Costs: contacts must be stiff and printable (DR4, DR6).

## CP4. A deep single valley per brick

Instead of a sawtooth of many small teeth, one long, deep V per brick catches the whole error box
and leaves no small faces to stick on. Targets: two-point jams between teeth. Costs: height and
mass; the hook (R4) must come from the valley's shape or a separate keel (DR1, DR2, DR10).

## CP5. Capture past the course below

A Y guide on the course below can only centre a brick whose centre of mass is still over it, so
capture is at most about D / 2. To reach the error disc, make the lower course wider than the brick
above, or let the brick reach out over it, within R10's 35 g for brick plus gripper. Targets: falls
and jams at large Y. Costs: mass and ring corners: features past the core must leave the ring's
turn pieces room (DR7, DR11).

## CP6. Lead in first, lock last

A long smooth lead-in centres the brick before any hook or tooth engages, and the hook engages only
in the last few millimetres, along a single stroke. Targets: jams where the interlock catches early,
off centre. Costs: release height and a longer stroke (DR9, DR10).
