# Design rules

The critic agent (#19) reads this file. Each rule states a mechanism, says what to do about it in a brick design, and cites its source by key from [`citations.md`](citations.md). The rules explain the mechanics; none of them describes a design to copy. Use papers for mechanics and principles, never for brick, joint or structure designs (CLAUDE.md).

**How to cite a rule in a brief.** Name its id, for example `DR3`. A brief that proposes a change must say which rule it rests on.

**Labels.** A rule's evidence is one of:

- **paper:** stated in the cited source, and read in this repo (`citations.md` says how far);
- **principle:** standard mechanics that the cited source sets out; not checked against the source's text in this repo;
- **user observation:** found by the maintainer on hardware; not in any paper.

Results the loop produces are "simulated, uncalibrated contact" or "predicted, quasi-static 2D". A rule is not a result.

<!-- Each rule starts with "## DR<n>." and has "Cites:" and "Evidence:" lines; tests/test_rules_corpus.py checks them. -->

## DR1. A guide face must be steeper than its friction cone

- **Mechanism.** A brick sliding on a face under gravity alone keeps sliding only if the face's angle from horizontal is larger than atan(μ). At a shallower angle the friction cone holds the brick still: it sticks on the face (a one-contact jam).
- **In a design.** Make every guide face steeper than atan(μ_max) with margin. R6 asks for 8° above atan(0.4) ≈ 21.8°. A one-contact jam band in the jam analysis points at a face that is too shallow for its friction.
- **Cites:** balkcom-trinkle-2002, mason-1985
- **Evidence:** principle

## DR2. Two-point contact wedges when the friction cones see each other

- **Mechanism.** With two contacts on opposite sides of a part, the part can be held by contact forces that point straight at each other. That happens when the line between the two contacts lies inside both friction cones. For a peg in a hole, wedging needs two-point contact early in the insertion, at a depth below about μ times the diameter (Whitney 1982, eq. 49, l_w = μ·d).
- **In a design.** Keep two-point contact from forming while the brick is shallow: let one face guide it alone until it is deep, widen the clearance at the mouth, or set the two faces at angles whose friction cones cannot both contain the line between them. A band named "wedged" in the jam analysis is this rule.
- **Cites:** whitney-1982
- **Evidence:** paper

## DR3. Jamming is a matter of force proportions; wedging is a matter of geometry

- **Mechanism.** Whitney separates jamming, where the applied forces and moments are in the wrong proportion for the contact state, from wedging, which the geometry alone causes. A jam can clear if the push changes; a wedge does not.
- **In a design.** A dropped brick has one push, its weight, and no control of it. So treat every jam as geometric for the brick: change the contact geometry (face angles, where the centre of mass sits relative to the contacts) rather than hoping the load will vary.
- **Cites:** whitney-1982
- **Evidence:** paper

## DR4. A seat should hold without friction

- **Mechanism.** A rest that only friction holds is weakly stable: some solution of the dynamics stays still, but others move. If the normal forces alone can balance the load, the rest is strongly stable for every μ (Pang & Trinkle 2000, Theorem 1). Equilibrium alone can give false positives, and designing with weak stability is "not recommended".
- **In a design.** Shape the seat so its faces carry the brick's weight with normal forces alone: two faces that lean against each other under the centre of mass, not a ledge that holds by friction.
- **Cites:** pang-trinkle-2000
- **Evidence:** paper

## DR5. A rest that needs no friction, away from the seat, is a wrong valley

- **Mechanism.** The same test, applied to a rest in the wrong place: if the contacts there balance gravity without friction, the brick is stably at rest for every μ, so lowering friction or steepening one face will not free it. The jam analysis reports this as `needs_friction: false`.
- **In a design.** Remove the second valley: cut away or slope the ledge the brick sits on, so there is no stable rest between first contact and the seat. A friction jam (`needs_friction: true`) is DR1 or DR2 instead.
- **Cites:** pang-trinkle-2000, berretty-2001
- **Evidence:** principle

## DR6. Seat robustness is the room around gravity inside the stable wrench cone

- **Mechanism.** The set of external wrenches under which a part stays in a contact mode is a cone. A biasing wrench inside the strongly stable cone resists disturbances up to "the radius of the largest ball that would fit in the cone, centered on the biasing wrench" (Balkcom & Trinkle 2002, §8). For many shapes no constant wrench guarantees seating; the shape has to change (§10).
- **In a design.** Gravity is the biasing wrench. Contacts that are wide apart, and faces whose normals spread around the vertical, put gravity deeper inside the cone and make the seat tolerate more. If no seat holds across the target box, change the shape, not the parameters.
- **Cites:** balkcom-trinkle-2002, balkcom-gottlieb-trinkle-2002
- **Evidence:** paper

## DR7. Capture is the set of starts that end in the seat

- **Mechanism.** A compliant motion (here, sliding under gravity) succeeds from a region of starting poses, its preimage. A strategy is good when that region covers the whole uncertainty. Planning fine motion means growing the preimage until it contains the error.
- **In a design.** The target box is the uncertainty. Widen the region that slides into the seat until it covers the box in X, release height and pitch: lengthen guide faces, widen the mouth, and bias the aim toward the side with the longer guide (R5). A capture limit inside the target box in the jam analysis is this rule.
- **Cites:** lozano-perez-mason-taylor-1984, erdmann-mason-1988
- **Evidence:** principle

## DR8. Shape alone can sort a part into one resting orientation

- **Mechanism.** Without sensing, a sequence of actions can bring any polygonal part to one orientation up to symmetry, because the part's shape fixes a finite set of stable rests and how each action maps them (Goldberg 1993). Tilting a tray does the same with gravity (Erdmann & Mason 1988).
- **In a design.** Count the stable rests the brick has on what it lands on, inside the target box. Each one other than the seat is a wrong valley to remove (DR5). Fewer rests, with the seat's basin the widest, means more of the box seats.
- **Cites:** goldberg-1993, erdmann-mason-1988
- **Evidence:** principle

## DR9. Every contact transition from first touch should lead to the seat

- **Mechanism.** The contact states between two parts and the transitions between them form a graph. A task is reliable when every path the motion can take through that graph ends in the goal state.
- **In a design.** For the first contacts the map shows inside the target box (corner on face, face on face), follow what the brick does next. A transition into a two-contact state with no exit is a jam (DR2) or a wrong valley (DR5); change the face it starts on.
- **Cites:** xiao-ji-2001, brost-1991
- **Evidence:** principle

## DR10. A longer engagement jams less (user observation)

- **Mechanism.** The maintainer found on hardware that lengthening a voxel like A/B's, by inserting a rod horizontally, reduced jamming. It was not measured further. It is consistent with DR2: a longer engagement relative to the clearance narrows the range of depths at which two-point contact can wedge.
- **In a design.** When two-point jams recur along X, try a longer brick or a longer engaging face before steeper ones.
- **Cites:** whitney-1982
- **Evidence:** user observation

## DR11. Systems that build with flying or climbing robots shape the parts to correct the robot's error

- **Mechanism.** Robotic construction systems put passive alignment into the parts themselves (TERMES's blocks; magnetic nodes and members; foam modules placed by quadrocopters), so a robot's positioning error is corrected by geometry at contact.
- **In a design.** Context only: these systems motivate R5 and R11. Do not copy their parts, joints or structures.
- **Cites:** petersen-nagpal-werfel-2011, lindsey-mellinger-kumar-2011, augugliaro-2014, jenett-2019
- **Evidence:** principle
