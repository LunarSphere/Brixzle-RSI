# Physical tests for bounce-out and mixed materials (issue #167)

The sim's contact values are uncalibrated, and the TPU values are guesses (`fable_gentle_tpuhook`'s `TPU_CONTACT`). The tests below give the numbers that replace them. None of them is run from this repo: a person prints and tests the parts.

Record every measurement with:

- the printer, the material and its brand, the layer height, the infill and the print orientation;
- the date and the room temperature.

## 1. Friction by tilt: PLA on PLA, and TPU on PLA

Gives the PLA-on-PLA and TPU-on-PLA friction coefficients. The bounce trial uses 0.35 for PLA and an unmeasured estimate of 0.6 for TPU.

1. **Print:**
   - two 40 × 40 × 3 mm PLA plates, printed flat, with the top surface as the test face;
   - one 20 × 20 × 3 mm PLA coupon and one TPU (Shore 95A) coupon, each loaded to about 20 g, as a brick is.
2. **Set up:**
   - tape a plate to a board hinged at one end;
   - put the coupon on the plate;
   - raise the board slowly, about 1° per second, with a phone inclinometer on the board.
3. **Record** the angle θ at which the coupon starts to slide. Then μ = tan θ.
4. **Repeat** 10 times per pair, turning the coupon 90° between runs, because the print lines matter.
5. **Report** the mean and the standard deviation of μ, for PLA on PLA and for TPU on PLA.

## 2. Restitution: how high a brick bounces

Gives the contact damping. The sim uses a damping ratio of 1.0, which barely bounces; the trial also runs 0.3 and 0.1.

1. **Print** one brick of each design under test, and its base plate (the course-0 seat).
2. **Film** at 240 fps or more, side on, with a mm scale behind the seat.
3. **Drop** the brick flat, from 50, 100 and 150 mm above its seat (a jig that opens, not a hand), onto the plate laid flat, without the seat geometry: this is a plain face-on-face impact.
4. **Record** the drop height h and the first rebound's peak height h′. Then e = √(h′/h).
5. **Repeat** 5 times per height.
6. **Do the same** for the TPU pad: a PLA coupon with a TPU face, landing on PLA.

## 3. Bounce-out at an overhang step

The test the sim trial stands in for (`engine/bounce.py`, the "half" seat).

1. **Build** one overhang step: a brick on the base and the next brick's seat one voxel out, as `structures.overhang(2)`.
2. **Release** the reaching brick above its seat with no X error, using a guide tube or a jig. Release heights are its own release lift and then +25, +50, +75 and +100 mm. Release flat, not thrown.
3. **Record** whether the supporter was in position at release. If it was, record whether the reaching brick seated, jammed short of the seat or bounced out. Film it at 240 fps.
4. **Repeat** 10 times per height per design.
5. **Designs:** at least A/B, gentle, and gentle with TPU hooks if a two-material print is available. Otherwise print TPU hooks separately and glue them in place, and say which.
6. **Report** the bounce-outs against height and against energy m g h, with m the weighed brick. Use actual reaching-brick releases as the denominator and report setup failures separately.

## 4. Grip on the SO-101 (for #164)

The sim's grip rule and 100 mm lift (#153) need a real check.

1. **Pick** each brick from its printed mould station with the stock SO-101 jaws. A person drives the arm.
2. **Lift** 100 mm and photograph the brick in the jaws, before and after the lift, against a mm grid.
3. **Measure** the shift and the turn in the jaws. The bar is 1 mm and 1°.
4. **Repeat** 5 times per design. Designs: A/B, twinhook4, gentle, allround.

Use these measurements with the print and arm-test work tracked in issue #23.

What to send back: the table of each test's results, the photos or video files, and the print settings. The sim is then recalibrated from them in a reviewed PR.
