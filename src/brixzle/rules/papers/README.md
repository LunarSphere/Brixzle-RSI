# Papers

The papers behind the design rules and the evaluation engine. They are the critic agent's reference library, and the analytic predictor (`src/brixzle/engine/predictor/`) follows the first three.

**Use papers for mechanics and principles, not for designs.** Don't copy a brick, joint or structure design from any of them (see CLAUDE.md).

"Read" means the text was extracted and read when the paper was used (issue #10). Citations come from each PDF's own front matter unless marked otherwise.

## Contact mechanics and jamming

### Balkcom & Trinkle 2002: wrench cones

- **Citation:** D. J. Balkcom and J. C. Trinkle, "Computing Wrench Cones for Planar Rigid Body Contact Tasks."
  - The PDF gives no venue or year; "2002" comes from the filename.
  - It "collects and expands upon" their two ICRA 2002 papers (§1.1).
- **File:** `Balkcom2002b.pdf`.
- **Status:** read in full.
- **Model (§2).** Planar contact for one fixed and one moving body, at small velocity: a = A c + B g, with A = Jᵀ M⁻¹ J and B = Jᵀ M⁻¹. Each contact rolls (n), slides left or right (l, r), separates (s) or approaches (a). The constraints are in Table 2.
- **Indeterminacy (§7).** The same applied wrench can be consistent with several contact modes.
  - **Weak stability:** a solution with zero acceleration exists.
  - **Strong stability:** every solution has zero acceleration.
- **Seat robustness (§8).** A biasing wrench inside the strongly stable cone resists disturbances up to "the radius of the largest ball that would fit in the cone, centered on the biasing wrench".
- **Shape (§10).** For many shapes, no constant wrench guarantees seating. The shape has to change.
- **Used in:** `predictor/contact_modes.py`, for the mode tests and the §4 disc example as a test.

### Pang & Trinkle 2000: stability characterizations

- **Citation:** J. S. Pang and J. C. Trinkle, "Stability Characterizations of Fixtured Rigid Bodies with Coulomb Friction," Proc. IEEE ICRA, San Francisco, April 2000.
- **File:** `Stability_characterizations_of_fixtured_rigid_bodies_with_Coulomb_friction.pdf`.
- **Status:** read: the definitions, Propositions 1–2 and Theorem 1.
- **Definitions.** Weak and strong stability come from the dynamic model, not from equilibrium alone. Equilibrium alone can give false positives.
- **Design advice.** Designing with weak stability is "not recommended".
- **Theorem 1.** If normal forces alone balance the load (frictionless stability), the body is strongly stable for every μ.
- **Used in:** the predictor's seat test and seat margin.

### Whitney 1982: quasi-static assembly

- **Citation:** D. E. Whitney, "Quasi-Static Assembly of Compliantly Supported Rigid Parts," J. Dynamic Systems, Measurement, and Control 104:65–77, March 1982.
- **File:** `quasistatic.pdf`.
- **Status:** read: §4 on jamming and wedging, and the assembly conditions.
- **Jamming** comes from applied forces in the wrong proportions.
- **Wedging** is geometric. It happens when two-point contact occurs early enough that the two friction cones intersect, at l_w = μ·d (eq. 49). Contact forces can then point straight at each other.
- **Used in:** the predictor's wedging flag.

### Erdmann 1988

- **File:** `erdmann_michael_1988_1.pdf`.
- **Status:** a scan with no text layer; not read.
- The filename suggests M. Erdmann, 1988. Balkcom & Trinkle cite Erdmann & Mason, "An exploration of sensorless manipulation," IEEE J. Robotics and Automation 4(4), 1988, but **this file has not been confirmed to be that paper**.

### AIM-759

- **File:** `AIM-759.pdf`.
- **Status:** an MIT AI Memo scan with no text layer; not read. Title and authors not yet confirmed.

## Parts feeding and orienting (geometry does the work)

### Goldberg 1993

- **Citation:** K. Y. Goldberg, "Orienting Polygonal Parts Without Sensors," Algorithmica 10:201–225, 1993.
- **File:** `Orienting-Polygonal-Parts-without-Sensors-Algorithmica-1993.pdf`.
- **Status:** skimmed.
- **Takeaway:** for any polygonal part there is a sequence of parallel-jaw squeezes that orients it up to symmetry, without sensing.

### Berretty et al. 2000

- **Citation:** R.-P. Berretty, K. Goldberg, M. H. Overmars and A. F. van der Stappen, "Trap Design for Vibratory Bowl Feeders," Utrecht University tech. report UU-CS-2000-39, 2000.
- **File:** `berretty_00_trap.pdf`.
- **Status:** skimmed.
- **Takeaway:** traps whose geometry passes only the correctly oriented part.

### Xiao & Ji 2001

- **Citation:** J. Xiao and X. Ji, "Automatic Generation of High-Level Contact State Space," Int. J. Robotics Research 20(7):584, 2001.
- **File:** `IJRRXiaoJi.pdf`.
- **Status:** skimmed.
- **Takeaway:** enumerating the contact states between polyhedral objects and the transitions between them.

## Robots building structures

These are context for the critic. **They are not sources of brick designs.**

### Augugliaro et al.: Flight Assembled Architecture

- **Citation:** F. Augugliaro, S. Lupashin, M. Hamer, C. Male, M. Hehn, M. W. Mueller, J. Willmann, F. Gramazio, M. Kohler and R. D'Andrea, "The Flight Assembled Architecture Installation: Cooperative construction with flying machines," IEEE Control Systems 34(4), 2014.
- **File:** `2014_FlightAssembledArchitecture.pdf`.
- **Status:** not read for #10.
- **Takeaway:** a 6 m tower of 1500 foam modules, built by four quadrocopters.

### Lindsey, Mellinger & Kumar: cubic structures

- **Citation:** Q. Lindsey, D. Mellinger and V. Kumar, "Construction of Cubic Structures with Quadrotor Teams." The venue and year aren't on the first page; the newest reference it cites is from 2009.
- **File:** `p25.pdf`.
- **Status:** not read for #10.
- **Takeaway:** quadrotors assembling 2.5D truss structures from magnetic nodes and members.

### Petersen, Nagpal & Werfel: TERMES

- **Citation:** K. Petersen, R. Nagpal and J. Werfel, "TERMES: An Autonomous Robotic System for Three-Dimensional Collective Construction." The venue and year aren't on the first page.
- **File:** `p35.pdf`.
- **Status:** not read for #10.
- **Takeaway:** climbing robots building 3D structures under gravity, with the bricks shaped for passive alignment.
