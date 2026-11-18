# How we got to the twinhook bricks (13–17 November 2026, concise)

This is the short account of the work. The people and agents involved:

- **Human:** the maintainer.
- **Opus:** Claude Opus 5.5, which ran the loop, evaluated designs and wrote prompts.
- **Fable:** Claude Fable 5.2, a second session the maintainer prompted to design bricks by hand.

Every result here is **simulated, uncalibrated contact** (MuJoCo), at seed 0, unless it says otherwise. None of it is a hardware rate. Opus reran every Fable design through the same evaluation stages the loop uses, so each number below cites the run it came from.

## The result

| Design | Z mean / p95 / max | Z at μ 0.5 / 0.6 / 0.7 / 0.8 | Tower / wall / bridge (of 5) | Overhang | Ring | Mass |
|---|---|---|---|---|---|---|
| A/B (baseline, the maintainer's) | +0.506 / +0.139 / +0.045 | +0.38 / +0.37 / +0.42 / +0.29 | 5 / 3 / 5 | 9 | closes | 14.3 g |
| twinhook4 (the main line's final version) | +0.967 / +0.417 / +0.266 | +0.84 / +0.60 / +0.46 / +0.46 | 5 / 5 / 5 | 11 | closes | 19.7 g |
| **gentle (seats with no push; best overall in sim)** | **+0.978 / +0.443 / +0.285** | **+1.00 / +0.93 / +0.94 / +0.93** | 5 / 5 / 5 | 10 | closes | 19.2 g |
| feather (lightest) | +0.844 / +0.361 / +0.235 | +0.75 / +0.56 / +0.44 / +0.37 | 5 / 5 / 5 | 11 | closes | **9.9 g** |
| reach (longest overhang) | +0.933 / +0.404 / +0.262 | +0.68 / +0.23 / +0.08 / +0.10 | 5 / 5 / 5 | 12 of 12; 17 at an ask of 30 | closes | 19.9 g |
| wide (worst-case miss; 60 g allowed) | +0.978 / +0.956 / +0.944 at twinhook4's poses (own coarse map: +1.00 / +1.00 / +0.98) | +0.97 / +0.90 / +0.57 / −0.17 | 5 / 5 / 5 | 9 | closes | 38.5 g |
| **allround (wide boxes at a legal weight)** | **+0.978 / +0.926 / +0.655** at twinhook4's poses (own coarse map: +0.955 / +0.984 / +0.687) | +0.93 / +0.98 / +0.86 / +0.82 | 5 / 5 / 5 | 10 | closes | 24.65 g |

**Gentle is the strongest design in sim on the mean box and under friction.** It matches or beats twinhook4 on all three boxes and at every friction level, and is 0.5 g lighter, for one overhang step less (10 against 11). It is twinhook4 with three defaults changed, so it belongs to the same line.

**Allround (17 November) is the strongest on the wider error boxes at a weight the drone can carry.** It doubles gentle's p95 Z and more than doubles its max-box Z, at 24.65 g. It pays with a smaller friction margin (+0.82 at μ 0.8, against gentle's +0.93), a much larger brick (about 116 × 56 × 68 mm) and a 62 mm release height.

Reading the table:

- **Z** is (seated − jammed) / drops, over the drone's mean, 95th-percentile and worst measured error boxes.
- **The μ columns** are the friction margin: Z on the mean box at rising friction, with no push (`clean_results/scripts/friction_margin.py`, seed 1 held-out poses).
- **A/B's friction margin** comes from the same script.
- **Every design builds the whole gate and closes the 6 × 6 ring**, and every ring part is one printable solid (CadQuery counts one solid for each of L3, I3 and K2 in all six Fable designs).

Sources:

- **Each design's evaluation:** `RSI_RESULTS/<name>-verify-20261117/eval.json` and `friction.json`, where `<name>` is `fable-twinhook4`, `fable_feather`, `fable_reach`, `fable_wide` or `fable_gentle`.
- **A/B's row:** `runs/loop/wrapup-20261116-mean/design_log.jsonl` (iteration 0), `runs/gate-20261116T062057018710Z-ab`, and A/B's friction run from this session.
- **Allround's evaluation:** `RSI_RESULTS/fable_allround-20261117-opus/` (`eval.json`, `friction.json`, `fixed_poses.json`, `fixed_slices.json` with Z per slice, and `parts.json` with ring-part solids, size and release height).
- **The extra checks:** `RSI_RESULTS/fable-checks-20261117.json` (reach at an ask of 30: `runs/gate-20261117T042947712097Z-fable_reach`, 17 steps on all 5 repeats; ring-part solid counts) and `RSI_RESULTS/fable_wide-verify-20261117/fixed_poses.json`.

Caveats:

- **Wide** needs a bigger drone. With the 5 g gripper allowance it comes to about 44 g against our 35 g payload, so `check_family` reports `R9.mass_max` and `R10.within_payload` as broken; the maintainer allowed exactly those for this experiment. Wide's own map is coarse, because the map's cells scale with the pitch: its boxes hold 60 / 128 / 196 drops, not 180 / 592 / 1100. Its row therefore gives Z at twinhook4's own 180 / 592 / 1100 poses (`clean_results/scripts/fixed_poses.py`), so it compares like for like. As a check, the same method reproduces twinhook4's own map to within two drops. Fable reported +0.978 / +0.966 / +0.955 from its own, unsaved script; the rerun is slightly lower. At the worst-case box it seats 1069 of 1100, with no falls and no wrong valleys.
- **The fork's clean-release check could not run on any Fable design**, because they are all Solid3D. Gentle's "no push" release is therefore unchecked.
- **`fable_reach_skew`** is a discarded reach variant. It fails the contract: its default hook lean of 23° is outside its own bounds of 14–22°.
- **Hardware:** the maintainer printed a twinhook on 16 November and judged it the best design yet. That judgement is the only hardware result here. The session named the print as twinhook3, but the printed brick's open tooth slots match the first twinhook (`clean_results/README.md`).

## How we got here

### 1. The loop (13–16 November): new bricks, but none that could build

- **The overhaul (issues #105–#113).** Merged on 13–14 November, these taught the loop to:
  - explore new concepts from the capture principles (`rules/capture_principles.md`);
  - screen them cheaply;
  - refuse near copies of A/B;
  - repair a provisional design;
  - run Opus in persistent sessions with per-message limits (#125, #131, #139, #141).
- **Thirteen loop runs** spent $63.19 by their own ledgers (`runs/loop/*/api-spend.jsonl`). A few killed calls were charged their full budget, so true spend was somewhat lower.
- **Every run's best design stayed A/B.**
- **What the runs did find:** wide-pitch V bricks (31–34 mm per voxel) with almost no jams, at mean Z +0.93 to +0.98. They could stack a tower but not hold a brick that was only half supported: the end of a wall, a bridge step or an overhang step. They tipped about 10° and propped on one keel.
  - Examples: zigkeel in `runs/gate-20261116T064108226196Z-zigkeel`, and the 33 mm V in `runs/gate-20261116T013206447311Z-rsi_hold_20261116_mean_i01_c2`.
- **The loop then picked badly.** It ranked each round's concepts on mean Z alone, so it chose a brick that could not be stacked at all and spent both repairs on it (`wrapup-20261116-mean`).
- **The maintainer's own ideas changed the gate during this phase:**
  - aim each drop at the brick below (#137);
  - build the overhang back-first, as the lab does, from a photo of a real corbel (#137);
  - judge collapse on bricks leaving their seats, not on sag (#135).
  - With these, A/B's overhang went from 5 steps to 9.

### 2. Fable by hand (16 November): one evidence-backed change per step

The maintainer asked Opus for a prompt that summarised every design so far and the trade-off each made. They then had Fable design one brick against it. Each later step was a new prompt that Opus rewrote from the last evaluation's evidence.

1. **twinhook** (`RSI_RESULTS/fable-twinhook-20261116-2`).
   - Idea: a wide-pitch chevron with two mirrored hooks (a tooth under each valley, a horn on each peak), so a half-supported brick is held down whichever way it reaches.
   - First non-A/B design to pass the whole gate and close the ring: Z +0.589 / +0.269 / +0.168.
   - But it jammed across Y and was friction-sensitive: +0.10 at μ 0.5.
2. **twinhook2** (`RSI_RESULTS/fable-twinhook2-20261116`).
   - Fable read the stage data: off-centre bricks had dropped their teeth into full-depth slots, and half-supported bricks met a hook wall early.
   - It closed the slots outside a centre band and made each hook the apex of the V across Y.
   - Z rose to +0.967 / +0.426 / +0.273, and the gate went to 5 / 5 / 5.
   - Its new ring kit had never been simulated, and it jammed on the first brick.
3. **twinhook3** (`RSI_RESULTS/fable-twinhook3-20261116`).
   - Ring kit only; the brick is unchanged, confirmed by an identical map and gate.
   - The turn piece's arm had hung on a horn, and a straight brick clipped the corner cell.
   - The ring now closes.
4. **twinhook4** (`RSI_RESULTS/fable-twinhook4-verify-20261117`).
   - A web makes the turn piece one printable solid.
   - The overhang went from 8 to 11 steps. Opus had guessed the back brick was being lifted. Fable's contact readout showed the reaching brick was being pushed out along its own slot by a wall 10° off the stroke, so it made the last 2 mm of each slot parallel to the stroke.
   - Lids instead of solid plugs saved 1.9 g.
5. **Trade-off experiments, each pushing one quantity:**
   - **feather:** half the mass for 0.12 of mean Z, mostly from cutting depth.
   - **reach:** a 36° hook lean for more overhang, paid for in friction margin.
   - **wide:** an 81 mm pitch and Y wings for the worst-case box, at 38.5 g.
   - **gentle:**
     - What it fixed: twinhook4's high-friction jams were two-contact wedges, about 3° above friction.
     - What it changed: a 50° V across Y and 1.5 mm mouths.
     - Result: Z +0.93 at μ 0.8, against twinhook4's +0.46 (6 of 180 jam), with capture at least as good as twinhook4's.

### 3. Allround (17 November): good on every slice

- **Why.** Splitting each design's Z by map slice showed where it lost:
  - gentle and twinhook4 were perfect in the mean box;
  - along the wall (the X–height and X–pitch slices), every loss outside the mean box was a wrong valley, because a 34 mm pitch catches only about ±17 mm;
  - on the X–Y slice, falls added to that, because the depth catches only about ±15 mm across the wall.
- **The goal.** Opus prompted Fable for wide's reach at the normal mass limits, keeping gentle's friction margin.
- **Fable's design** (`runs/fable/fable_allround/`): twinhook4's joint at a 58 mm pitch, with ribbed wings reaching 28 mm across the wall, a 50° V and feather's thinning. Its first full evaluation passed the gate and closed the ring at 24.7 g.
- **Fable then ran out of budget.** On the brick's own coarse map the mean box showed 6 jams (Z +0.864, below the +0.95 asked for). Fable traced them to a pocket's lip catching a horn tip, and was adding a lead-in chamfer when its session hit the monthly spend limit, leaving the module broken.
- **Opus finished it** (v0.1.1):
  - Fable's chamfer was a buffered union whose micro-edges broke the solid's mesh. Opus rebuilt it as one clean polygon, at 0.6 mm. On the brick's own map, mean-box jams fell from 6 to 2. At twinhook4's finer poses it changed almost nothing (3 jams to 2): the coarse grid had happened to land on one narrow jam band.
  - Opus also tried moving the valley so the crests sit evenly about the aim. It traded jams for wrong valleys and falls, with no gain inside the noise band, so it was not kept.
- **Where the limit is.** What remains at p95 is all on the X–Y slice: 21 jams in 592 drops, mostly a horn rubbing a lid edge with 0.25 mm of Y clearance (Fable found that more clearance costs friction margin). The max box loses to wrong valleys past about ±29 mm along the wall and falls past about ±26 mm across it. Going further needs a longer pitch or wider wings, and at 24.65 g of 25 there is no mass left.

**Time.** Each Fable evaluation took about 3 minutes, locally and with no API spend. Fable's design work for all its bricks fits in one session, from 08:58 on 16 November to 01:42 on 17 November, including the gaps between prompts. Compare one loop run: 20–45 minutes and $3–10 to try one round of concepts.

## Why the hand-run sequence beat the loop

The maintainer's observation: **"I came up with a couple initial ideas but they needed to be updated after each step."** The record bears that out.

- **The goal changed as evidence came in.**
  - The first prompt asked for capture plus building.
  - The next asked to fix Y jams and friction.
  - Then the ring.
  - Then printability and overhang.
  - Then four trade-offs, two of which ("minimal force", "worst-case error, up to 60 g") the maintainer only defined midway.
- **The loop has no such channel.** It works to fixed requirements, a fixed objective (Z on one box, then the gate) and a static list of principles.
- **Each prompt carried the last result's specific evidence:** the failing brick, its rest pose and its contacts, not just a score. Fable diagnosed before it changed anything, and twice its diagnosis overturned the guess in the prompt.
- **Each step changed one thing and froze the rest.** The ring-only fix in twinhook3 kept the map and gate valid, so verifying it was cheap and unambiguous.
- **Fable could run the whole evaluation itself, as often as it liked.** That meant map, gate, ring, friction margin and probe re-drops, at about 3 minutes each, free and local. The loop's design calls get `check_family` and a 30-drop probe; the gate and the ring come only after the call has ended.
- **Claims were verified independently.** Opus re-read each module and reran it, and caught one unsimulated claim: "the ring kit builds". Fable's own reports reproduced on every rerun: exactly for the twinhooks, feather, reach and gentle, and within about 0.01 for wide's fixed-pose Z.

## How to improve the loop

In rough order of payoff:

1. **Let the brief evolve, with a human checkpoint.**
   - Between rounds, write a short "state of the problem" file: what the best designs trade off, what failed and why (stage data, contacts), and the current goal. Both the design agent and the maintainer read it.
   - Let the maintainer amend the goal or add a constraint at the checkpoint, as happened by hand here.
   - The current `lessons.jsonl` and critic brief are the seed of this.
2. **Give the design agent the full evaluation as a tool.**
   - Map, gate, ring, friction margin and fixed-pose re-drops (`clean_results/scripts/eval_family.py`, `friction_margin.py`, `fixed_poses.py`), with a wall-clock budget instead of a two-tool limit.
   - Fable's progress came from many cheap local evaluations inside one design effort.
3. **Diagnose before designing.** Make the critic, or the design call's first step, read the failing stage's rest contacts (`trials.rest_contacts`) and name the mechanism, before any geometry changes. The loop already has the data; its briefs reason from Z and outcome counts.
4. **Judge designs on several objectives.**
   - Select and repair on capture across all three boxes, gate builds, the ring and the friction margin together, not mean Z alone.
   - Check that the structures can be planned (`assembly_order`) inside `check_family`, so an unstackable brick is caught in milliseconds.
5. **One focused change per iteration, with the rest frozen.**
   - Let the brief name what must stay identical (for example "ring kit only").
   - Verify it by diffing and by a matching map, so an iteration can't trade a regression for a gain.
6. **Require sim evidence for every claim.** A "builds" or "passes" in a design note must point to a run directory. A separate verifier reruns the final design, as Opus did here.
7. **Fix the measurement gaps this work exposed:**
   - **The map's resolution scales with pitch,** so long bricks are scored on few drops; score on fixed poses as well. On allround's coarse map, 6 jams in the mean box looked like a real flaw; at the finer poses they were 3 in 180.
   - **The fork's release check supports only Prismatic families,** so none of the Fable bricks could be checked.
   - **Z ignores the friction margin,** although twinhook4 drops from +0.84 to +0.46 between μ 0.5 and 0.8.
   - **The predicted margin `R11.y_before_hooks` disagreed with the sim** on gentle.

## Open items

- **Print and drop-test gentle, twinhook4 and allround.** In sim gentle is the strongest at the mean box and under friction, and allround on the wider boxes. STLs are in their `RSI_RESULTS/…-verify-20261117/` folders, ring parts included. Gentle's 1.5 mm mouths and twinhook4's 0.08 mm hook gap are the clearances most likely to differ on hardware.
- **Measure real PLA friction** with a tilt test, to know whether gentle's margin matters.
- **Register the chosen design as a repo family,** or make it the loop's baseline. Per CLAUDE.md that needs an issue and a reviewed PR. All the Fable modules live in `runs/fable/`, which is git-ignored.
- **Wide's wings** are 31 mm cantilevers 1.3 mm thick with no ribs. That is Fable's guess at the print risk, not tested.
- **Allround's wings** are 2 mm plates on 1.2 mm gussets, and the brick releases from 62 mm above its seat. Neither the print nor the harder landing has been tested.
