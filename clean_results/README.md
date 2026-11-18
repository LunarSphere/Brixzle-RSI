# Clean results: one figure per brick concept

Each folder is one brick concept at its final version, numbered in chronological order (`NN-family_name`). Files are named after the family:

- `<family>-figure.png`: the module render (front view) beside the jam map.
- `<family>-module.png`: the run's full render (front, side and top views of each part).
- `<family>-drop-1.mp4` to `-drop-5.mp4`: the drop videos the run picked from its map (the numbered markers on the jam map).
- The STL of each part, under its original name, where the run saved one.

The drop videos are git-ignored (`*.mp4`), so they exist only in the local copy of this folder.

To repeat the method, use the agent prompts in `prompts.md` and the evaluation scripts in `scripts/`.

`02-zigzag_chevron` has no videos or STL, because that run saved neither. `03-zigzag_w`'s videos come from `smoke3-20261112-mean-videos/iter-02`, a re-render of the same design. For the Fable twinhook line, each branch gets its own folder.

All numbers are **simulated, uncalibrated contact** (MuJoCo), not hardware rates. Z = (seated − jammed) / drops over the drone's mean / p95 / max error boxes. Every value is copied from the source run's `summary.md` or `eval.json`, listed in the tables. Rows 02–04 were scored before the map had an X–Y slice, so their Z is not comparable with later rows.

## How the work was run, and the models

| Phase | Dates | Who designed | Who critiqued / evaluated | Rows |
|---|---|---|---|---|
| SDK loop, early | 12–13 Nov | Design agent: Claude Sonnet 5.5 | Critic: Sonnet 5.5 | 02–04 (no per-call model log yet; Sonnet 5.5 was the config default) |
| SDK loop | 14 Nov | `ab4iter2`: Sonnet 5.5; `overhaul2`: Opus 5.5 | Sonnet 5.5 | 05–06 |
| SDK loop, persistent sessions (#125) | 15–16 Nov | Opus 5.5 | Opus 5.5 (`concept-track`: Sonnet 5.5) | 07–13 |
| **Swap:** maintainer as middle man between two Claude Code sessions | from 16 Nov, about 08:58 | Claude Fable 5.2 (Claude Code), designing by hand | Claude Opus 5.5 (Claude Code): reran each design through the loop's evaluation and wrote the next prompt; the maintainer relayed between them | 14–22 |

The swap came after `wrapup-20261116-mean`, the last SDK loop run. Across thirteen loop runs ($63.19 by their own ledgers), every run's best design stayed A/B. The loop found wide-pitch V bricks with few jams, but none of them could hold a half-supported brick, so none passed the structure gate. Fable's evaluations ran locally, with no API spend. Source: `docs/november-16-2026-concise.md`.

## The concepts

| # | Folder | Design | Source run | Z mean / p95 / max | What it tried, and how it ended |
|---|---|---|---|---|---|
| 01 | `01-ab` | A/B, the maintainer's design (baseline) | `RSI_RESULTS/ab-20261117` | +0.506 / +0.139 / +0.045 | The baseline every design had to beat. Builds the gate; overhang 9; ring closes. |
| 02 | `02-zigzag_chevron` | zigzag chevron | `smoke2-20261112-mean/iter-01` | +1.000 / +0.901 / +0.646 | First loop design: a constant-thickness 45° V band that slides to its centre. Smoke test only. |
| 03 | `03-zigzag_w` | zigzag W | `smoke3-20261112-mean/iter-02` | +1.000 / +0.643 / +0.460 | A W sawtooth that nests like an egg crate. Iteration 2 fixed wrong valleys at the box edges by raising the pitch from 30 to 36 mm. |
| 04 | `04-chevron` | chevron | `ab-concept2-20261113-mean/iter-01` | +1.000 / +0.839 / +0.601 | A single 40° V brick with a centre grip bore. |
| 05 | `05-ab_loop` | A/B reshaped by the loop | `ab4iter2-20261114-mean/iter-04` | +0.573 / +0.175 / +0.069 | Four tries to widen A/B's ±4 mm Y capture and its X window: deeper D (broke the ring), a wider mouth chamfer, a steeper Y gutter (kept, +0.537), then a longer tooth (rejected on held-out). Nudges moved Z by only about ±0.03. |
| 06 | `06-keel_valley` | keel valley | `overhaul2-20261114-mean/iter-01` | +0.471 / +0.044 / −0.028 | First explore concept: a symmetric V per voxel with a leaning peg, for two-sided capture. Jammed; rejected. |
| 07 | `07-hopper_keel` | hopper keel | `sessions-20261115-mean/iter-01` | +0.848 / +0.343 / +0.181 | V hopper ramps into a leaning slot, with a pointed keel. Better capture, but failed the gate (overhang 1) and the ring. |
| 08 | `08-keystone` | keystone | `clock-repair-20261115-mean/iter-01` | +0.675 / +0.271 / +0.161 | One seat per brick (keel and roof), to remove the one-voxel wrong valley. Rejected: held-out Z fell at μ 0.5. |
| 09 | `09-chevron_cup` | chevron cup | `concept-track-20261115-mean/iter-01` | +0.357 / +0.110 / +0.049 | A 35° V plate at a 32 mm pitch with a short tooth. Rejected: many jams. |
| 10 | `10-w_valley_hook` | W-valley hook | `gate2-20261116-mean/iter-01` | +0.761 / +0.345 / +0.217 | A 34 mm pitch with one leaning tooth hooked under a lip, to hold a half-supported step. Overhang 5, but failed the gate. |
| 11 | `11-keyed_v` | wide-basin keyed V | `hold-20261116-mean/iter-01` | +0.979 / +0.464 / +0.289 | A 33 mm V with a slanted key. Almost no jams, but tipped on half-supported steps, so it failed the gate. |
| 12 | `12-nested_v_keel` | nested-V keel | `long-20261116-mean/iter-01` | +0.939 / +0.466 / +0.287 | A 44° V plate with a centre peg. Failed the gate (overhang 1) and the ring. |
| 13 | `13-joint_keel` | joint keel | `wrapup-20261116-mean/iter-01` | +0.961 / +0.234 / +0.049 | One V per brick at the joint, 50 mm apart. It could not be stacked, and the loop still spent both repairs on it (it ranked on mean Z alone). Last loop design. |
| 14 | `14-twinhook` | twinhook (first version) | `fable-twinhook-20261116-2` | +0.589 / +0.269 / +0.168 | First hand-run design: a wide-pitch chevron with two mirrored hooks, with tooth slots running the full depth (visible through the front view). First non-A/B design to pass the gate and close the ring (overhang 8; 17.7 g), but it jammed across Y and was friction-sensitive (+0.10 at μ 0.5). |
| 15 | `15-twinhook2` | twinhook2 | `fable-twinhook2-20261116` | +0.967 / +0.426 / +0.273 | Fixed the Y jams: closed the slots outside a centre band and made each hook the apex of the V across Y (depth 26 → 30 mm). Gate passed (overhang 8; 21.6 g), but its new ring kit jammed on the first brick. |
| 16 | `16-twinhook3` | twinhook3 | `fable-twinhook3-20261116` | +0.967 / +0.426 / +0.273 | Same brick as twinhook2 (identical map); only the ring kit changed, so the ring now closes. Gate passed; overhang 8; 21.6 g. |
| 17 | `17-twinhook4` | twinhook4 (the main line's final version) | `fable-twinhook4-verify-20261117` | +0.967 / +0.417 / +0.266 | See the twinhook line below. Builds the gate 5 / 5 / 5; overhang 11; ring closes; 19.7 g. |
| 18 | `18-feather` | feather | `fable_feather-verify-20261117` | +0.844 / +0.361 / +0.235 | Branch off twinhook4: half the mass (9.9 g), mostly from cutting depth, for 0.12 of mean Z. |
| 19 | `19-reach` | reach | `fable_reach-verify-20261117` | +0.933 / +0.404 / +0.262 | Branch: a 36° hook lean for a longer overhang (12 of 12), paid for in friction margin. |
| 20 | `20-gentle` | gentle | `fable_gentle-verify-20261117` | +0.978 / +0.443 / +0.285 | Branch: fixed twinhook4's high-friction two-contact wedges with a 50° V across Y and 1.5 mm mouths. Z +0.93 at μ 0.8, against twinhook4's +0.46. |
| 21 | `21-wide` | wide | `fable_wide-verify-20261117` | +1.000 / +1.000 / +0.980 (own coarse map) | Branch: an 81 mm pitch and Y wings for the worst-case box, at 38.5 g, beyond the 35 g payload. At twinhook4's poses: +0.978 / +0.956 / +0.944. |
| 22 | `22-allround` | allround v0.1.1 | `fable_allround-20261117-opus` | +0.955 / +0.984 / +0.687 (own coarse map) | Branch: wide's reach at a legal mass (24.65 g): a 58 mm pitch, ribbed wings and a 50° V. Fable ran out of budget mid-fix; Opus finished it. At twinhook4's poses: +0.978 / +0.926 / +0.655. |

## The twinhook line (Fable 5.2, prompted via Opus 5.5)

Each step changed one thing, based on the last evaluation's evidence:

1. **twinhook** (`fable-twinhook-20261116-2`, in `14-twinhook`): a wide-pitch chevron with two mirrored hooks, so a half-supported brick is held down either way. It was the first non-A/B design to pass the gate and close the ring (Z +0.589 / +0.269 / +0.168), but it jammed across Y and was friction-sensitive.
2. **twinhook2** (`fable-twinhook2-20261116`, in `15-twinhook2`): closed the slots outside a centre band and made each hook the apex of the V across Y, to stop the Y jams. Z rose to +0.967 / +0.426 / +0.273, but its new ring kit jammed.
3. **twinhook3** (`fable-twinhook3-20261116`, in `16-twinhook3`): changed only the ring kit, and the ring closes.
4. **twinhook4** (`fable-twinhook4-verify-20261117`, in `17-twinhook4`): a web makes the turn piece one printable solid. Slot ends were made parallel to the stroke, which took the overhang from 8 to 11. Lids save 1.9 g.

From twinhook4 the line branches into feather, reach, gentle, wide and allround (rows 18–22), each pushing one quantity.

**Which one was printed.** The Nov 16 write-up and the prompts after it say twinhook3 was printed. The printed brick has its tooth slots open through the front view, which matches the first twinhook (`14-twinhook`), not twinhook2 or twinhook3, whose slots are closed outside a centre band.
