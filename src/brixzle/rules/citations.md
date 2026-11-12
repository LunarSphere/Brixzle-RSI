# Citations

The sources behind the design rules ([`design_rules.md`](design_rules.md)) and the evaluation engine. The repo is public, so it holds citations and links, not the papers. Get a paper from its link.

**Use papers for mechanics and principles, not for designs.** Don't copy a brick, joint, gripper or structure design from any of them (CLAUDE.md).

- **Key:** what a rule's `Cites:` line names.
- **Link:** a DOI where one was confirmed, otherwise the publisher's, the authors' or a library's record. Links were checked on 2026-11-12.
- **Read:** how far the paper was read in this repo. "Read" means the text was extracted and read when it was used (issue #10). A rule that rests on a paper not read here is labelled "principle" in `design_rules.md`.

<!-- Each entry starts with "## <key>" and has "Citation:", "Link:" and "Read:" lines; tests/test_rules_corpus.py checks them. -->

## Contact mechanics and jamming

## balkcom-trinkle-2002

- **Citation:** D. J. Balkcom and J. C. Trinkle, "Computing Wrench Cones for Planar Rigid Body Contact Tasks," Int. J. Robotics Research 21(12):1053–1066, 2002.
- **Link:** https://doi.org/10.1177/0278364902021012003
- **Read:** in full (#10). The predictor's contact modes follow §2; weak and strong stability are §7; seat robustness §8; "the shape has to change" §10.
- **Used in:** `engine/predictor/contact_modes.py`; DR1, DR6.

## balkcom-gottlieb-trinkle-2002

- **Citation:** D. Balkcom, E. Gottlieb and J. Trinkle, "A Sensorless Insertion Strategy for Rigid Planar Parts," Proc. IEEE ICRA 2002, pp. 882–887.
- **Link:** https://doi.org/10.1109/ROBOT.2002.1013468
- **Read:** not read here; it applies the wrench-cone algorithm of balkcom-trinkle-2002 to sensorless insertion into a fixture.
- **Used in:** DR6.

## pang-trinkle-2000

- **Citation:** J. S. Pang and J. C. Trinkle, "Stability Characterizations of Fixtured Rigid Bodies with Coulomb Friction," Proc. IEEE ICRA, San Francisco, April 2000, pp. 361–368.
- **Link:** https://www.sandia.gov/research/publications/details/stability-characterizations-of-fixtured-rigid-bodies-with-coulomb-friction-2000-02-15/
- **Read:** the definitions, Propositions 1–2 and Theorem 1 (#10).
- **Used in:** the predictor's seat test and the jam analysis's holding set; DR4, DR5.

## whitney-1982

- **Citation:** D. E. Whitney, "Quasi-Static Assembly of Compliantly Supported Rigid Parts," J. Dynamic Systems, Measurement, and Control 104(1):65–77, March 1982.
- **Link:** https://portal.mardi4nfdi.de/wiki/Item:Q3946036 (no DOI confirmed)
- **Read:** §3 on chamfer crossing and one- and two-point contact, and §4 on jamming and wedging (#10, #12).
- **Used in:** the predictor's wedging flag and the jam analysis's wedging test; DR2, DR3, DR10.

## mason-1985

- **Citation:** M. T. Mason, "The Mechanics of Manipulation," Proc. IEEE ICRA 1985, pp. 544–548.
- **Link:** https://ri.cmu.edu/?p=15628
- **Read:** not read here. Cited for sliding under Coulomb friction, which DR1 states as standard mechanics.
- **Used in:** DR1.

## Fine motion, sensorless manipulation and parts feeding

## lozano-perez-mason-taylor-1984

- **Citation:** T. Lozano-Pérez, M. T. Mason and R. H. Taylor, "Automatic Synthesis of Fine-Motion Strategies for Robots," Int. J. Robotics Research 3(1):3–24, 1984.
- **Link:** https://doi.org/10.1177/027836498400300101
- **Read:** not read here. Cited for preimages: the starts from which a compliant motion reaches the goal.
- **Used in:** DR7.

## erdmann-mason-1988

- **Citation:** M. A. Erdmann and M. T. Mason, "An Exploration of Sensorless Manipulation," IEEE J. Robotics and Automation 4(4):369–379, 1988.
- **Link:** https://doi.org/10.1109/56.800
- **Read:** not read. The scan that used to sit in the papers folder (`erdmann_michael_1988_1.pdf`) had no text layer and was never confirmed to be this paper.
- **Used in:** DR7, DR8.

## goldberg-1993

- **Citation:** K. Y. Goldberg, "Orienting Polygonal Parts Without Sensors," Algorithmica 10:201–225, 1993.
- **Link:** https://doi.org/10.1007/BF01891840
- **Read:** skimmed (#10).
- **Used in:** DR8.

## berretty-2001

- **Citation:** R.-P. Berretty, K. Goldberg, M. H. Overmars and A. F. van der Stappen, "Trap Design for Vibratory Bowl Feeders," Int. J. Robotics Research 20(11):891–908, 2001. Utrecht technical report UU-CS-2000-39 is the version that was skimmed.
- **Link:** https://dspace.library.uu.nl/bitstream/1874/18998/1/berretty_00_trap.pdf
- **Read:** skimmed (#10).
- **Used in:** DR5.

## brost-1991

- **Citation:** R. C. Brost, "Analysis and Planning of Planar Manipulation Tasks," PhD thesis, CMU-CS-91-149, Carnegie Mellon University, 1991.
- **Link:** https://ri.cmu.edu/?p=15815
- **Read:** not read here. Cited for the sets of starting configurations from which a planar action reliably succeeds.
- **Used in:** DR9.

## xiao-ji-2001

- **Citation:** J. Xiao and X. Ji, "Automatic Generation of High-Level Contact State Space," Int. J. Robotics Research 20(7):584–606, 2001.
- **Link:** https://doi.org/10.1177/02783640122067552
- **Read:** skimmed (#10).
- **Used in:** DR9.

## Robots building structures

Context for the critic. **They are not sources of brick designs.**

## petersen-nagpal-werfel-2011

- **Citation:** K. Petersen, R. Nagpal and J. Werfel, "TERMES: An Autonomous Robotic System for Three-Dimensional Collective Construction," Robotics: Science and Systems VII, 2011.
- **Link:** https://doi.org/10.15607/RSS.2011.VII.035
- **Read:** not read here.
- **Used in:** DR11.

## lindsey-mellinger-kumar-2011

- **Citation:** Q. Lindsey, D. Mellinger and V. Kumar, "Construction of Cubic Structures with Quadrotor Teams," Robotics: Science and Systems VII, 2011.
- **Link:** https://doi.org/10.15607/RSS.2011.VII.025
- **Read:** not read here.
- **Used in:** DR11.

## augugliaro-2014

- **Citation:** F. Augugliaro, S. Lupashin, M. Hamer, C. Male, M. Hehn, M. W. Mueller, J. Willmann, F. Gramazio, M. Kohler and R. D'Andrea, "The Flight Assembled Architecture Installation: Cooperative Construction with Flying Machines," IEEE Control Systems 34(4):46–64, 2014.
- **Link:** https://doi.org/10.1109/MCS.2014.2320359
- **Read:** not read here.
- **Used in:** DR11.

## jenett-2019

- **Citation:** B. Jenett, A. Abdel-Rahman, K. Cheung and N. Gershenfeld, "Material–Robot System for Assembly of Discrete Cellular Structures," IEEE Robotics and Automation Letters 4(4):4019–4026, 2019.
- **Link:** https://ieeexplore.ieee.org/document/8769886
- **Read:** not read here.
- **Used in:** DR11.

## Removed from the folder, not cited

- **`AIM-759.pdf`:** an MIT AI Memo scan with no text layer. Its title and authors were never confirmed, so it is not cited.
- **Whitney, Gustavson & Hennessey 1983, "Designing chamfers":** in #16's starting list, but the maintainer decided not to use it (2026-11-06). Chamfer analysis comes from whitney-1982 §3.
