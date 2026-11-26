# Twinhook-line notes

Rules found on one brick, `gentle` and its descendants, in the half-supported studies of #178 and #184 (simulated,
uncalibrated contact, mostly one family and one seed, and under free drop). They are written in that brick's anatomy
(horn, tooth, pocket, sill), so they were moved out of `design_rules.md` in #206: the designer and the critic no
longer read them by default, and a new family should not be pulled back toward that line. Their ids are kept, so the
runs that cite them still resolve. Promote one back only when a design of another family confirms it.

## DR12. A lower hook lean shrinks the hook's undercut

- **Mechanism.** A leaning hook blocks a straight lift by the undercut of its hook wall, about (hook length - tip chamfer - tip set-back) x sin(lean) on the twin-hook line (predicted, quasi-static 2D). Lowering the lean helps a half-supported brick seat (gentle's bare edges: +x -0.38 -> -0.08, -x +0.33 -> +0.59 edge-map Z at 23 -> 18 deg), but below about asin(1 mm / effective hook length), 18.8 deg for gentle's hook, the undercut falls under 1 mm and R4's hook depth breaks (-0.04 mm at 18 deg).
- **In a design.** When the lean comes down to free a half-supported seat, lengthen the hook's effective length in the same change: less tip chamfer or set-back, or a longer hook. On gentle a 0.4 mm chamfer at 18 deg restored the margin (+0.08 mm) and kept most of the edge gain (`gentle_lean18_ch04`: +x -0.20, -x +0.56), at a cost in friction margin (Z at mu 0.8 +0.83 against +0.93).
- **Cites:** whitney-1982
- **Evidence:** simulation
- **Runs:** runs/diag-gentle-edge-20261122/sweep1.json, runs/diag-gentle-edge-20261122/measured-lean18, runs/rsi/gentle-edge-20261122/iter-01/verified.json, runs/edgemap-20261122T031954257942Z-gentle_lean18_ch04

## DR15. On a half-supported seat, the free side's hold-down undercut is also a trap

- **Mechanism.** A hold-down undercut keeps a seated brick from lifting out. On a half-supported seat the brick can tip toward the free side before it seats, and the same undercut then catches it short: on gentle, a brick tipped about 16 deg toward the bare +x edge came to rest under the supporter's horn undercut, and a bevel that put an upward component on the end-face reaction just moved jams into that undercut (simulated, uncalibrated contact). R4.hook_depth and the free-side edge pull against each other.
- **In a design.** Put the retention that must hold a reaching brick on the supported side, or make the free-side undercut unreachable by a brick tipped by the expected error. Check a tipped approach (about 16 deg on gentle) against every undercut on the free side.
- **Cites:** balkcom-trinkle-2002, whitney-1982
- **Evidence:** simulation
- **Runs:** runs/diag-gentle-edge-20261122/gentle_plus_bare.json, runs/rsi/gentle-edge-20261122/iter-01/contacts-20261122T025342055042Z-gentle_lean18_ch04, runs/rsi/all-placement-20261122/iter-01/diagnosis.md

## DR16. A sharp free-side tooth hooks a bare vertical end face

- **Mechanism.** A brick tipped 20 to 32 deg toward a bare edge brings its free-side tooth against the supporter's vertical end face. The tip bears with a horizontal normal about 5 mm above the face's bottom corner, low on the face, not at its top corner, and holds the brick short. On gentle it was the main +x jam in every edge layout: 36 of 50 plus_bare jams, 4 of 5 plus_anchored, 4 of 5 plus_cantilever (simulated, uncalibrated contact). A top-corner bevel does not reach this contact.
- **In a design.** Keep the free-side tooth of a tipped brick from meeting a bare vertical end face: a tooth that cannot reach past the face, a face that slopes the tooth inboard along its whole height, or no tooth on the free side. Check the contact height, not just the corner.
- **Cites:** whitney-1982, balkcom-trinkle-2002
- **Evidence:** simulation
- **Runs:** runs/contacts-20261122T031344558193Z-gentle_lean18_ch04, runs/rsi/all-placement-20261122/edge/contacts-20261122T035544558275Z-gentle_lean18_ch04, runs/rsi/all-placement-20261122/edge/contacts-20261122T035554515201Z-gentle_lean18_ch04

## DR17. The face beside a socket pocket wedges against the opposite tooth

- **Mechanism.** A brick offset about 1 mm toward a bare edge lands its body on the flat face beside the pocket that should take its horn, while its opposite tooth bears on the slot wall. The two contacts form a wedge (DR2) and the brick jams above its seat. On gentle, a pocket lead-in opened to the underside rescued 3 of 8 diagnosed wedge cells but lost seats elsewhere on the edge and in the 4 mm corbel (simulated, uncalibrated contact).
- **In a design.** Do not leave a flat landing face beside a pocket within the expected placement error; slope it into the pocket at least as wide as the error, and check that the opposite tooth's contact cannot see the landing face's friction cone (DR2).
- **Cites:** whitney-1982, balkcom-trinkle-2002
- **Evidence:** simulation
- **Runs:** runs/contacts-20261122T031008301714Z-gentle_lean18_ch04, runs/contacts-20261122T031101278271Z-gentle_lean18_ch04, runs/rsi/all-placement-20261122/iter-06/design_note.md, runs/rsi/all-placement-20261122/iter-06/screen.json

