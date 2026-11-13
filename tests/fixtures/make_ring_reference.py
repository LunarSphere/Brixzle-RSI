"""Print aerial-assembly's ring references (issue #89): plans, order, part masses.

Run inside aerial-assembly's brixzle/ (fc4f724), not this repo, and read-only:
    cd ../aerial-assembly/brixzle && uv run --frozen python <this file> > ref.json
tests/fixtures/ring_reference.json holds its output, with a source block added by hand."""
import json, sys
from brixzle import ring as RG
from brixzle.params import BrickParams
p = BrickParams.from_dict(json.load(open("configs/ab.json")))
out = {"params": p.__dict__ if hasattr(p, "__dict__") else None, "plans": {}, "masses_g": {}, "stroke_travel_mm": RG.stroke_travel(p)}
for courses in (2, 4):
    bricks = RG.ring(6, courses)
    RG.check_columns(bricks)
    order = RG.assembly_order(bricks)
    out["plans"][str(courses)] = {"bricks": [[b.kind, b.course, b.x, b.y, b.yaw] for b in bricks],
                                  "order": [b.name for b in order]}
parts = RG.build_parts(p)
for k, v in parts.items():
    out["masses_g"][k] = float(v["mass_g"])
    out.setdefault("n_collision", {})[k] = len(v["collision"])
json.dump(out, sys.stdout, default=str)
