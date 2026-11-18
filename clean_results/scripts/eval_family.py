"""Run an outside family package through the loop's evaluation stages, as SimEngine does:
check_family, quick check, full map + Z, jam analysis, held-out Z, structure gate, ring, renders.
Simulated, uncalibrated contact. Usage: eval_family.py <root> <module> <out_dir>"""
import json, sys, time
from pathlib import Path
from brixzle.agents import check_family as CF
from brixzle.loop.config import LoopConfig
from brixzle.loop import orchestrator as O, render as RD, videos as VD
from brixzle.engine import sim_map as SM

def main():
    global root, module, out, cfg, eng, rec, secs
    root, module, out = sys.argv[1], sys.argv[2], Path(sys.argv[3])
    out.mkdir(parents=True, exist_ok=False)
    cfg = LoopConfig()
    eng = O.SimEngine(cfg)
    rec, secs = {"root": root, "module": module, "label": "simulated, uncalibrated contact", "seed": cfg.seed}, {}

    def stage(name, fn):
        t = time.time()
        try:
            return fn()
        finally:
            secs[name] = round(time.time() - t, 1)
            print(f"{name}: {secs[name]} s", flush=True)

    def save():
        rec["seconds"] = secs
        (out / "eval.json").write_text(json.dumps(rec, indent=1, default=str, allow_nan=False) + "\n")

    chk = stage("check_family", lambda: CF.check_in_subprocess(root, module))
    rec["check_family"] = {k: chk.get(k) for k in ("ok", "violations", "mass_g", "margins", "size_mm") if k in chk}
    print("check_family ok:", chk.get("ok"), (chk.get("violations") or [])[:5], flush=True); save()
    fam = CF.load_family(root, module); p = fam.defaults
    rec["mass_g"] = eng.masses(fam, p)
    check, check_dir = stage("quick_check", lambda: eng.quick_check(fam, p))
    rec["quick_check"] = {"passed": check["passed"], "run": str(check_dir)}
    print("quick check passed:", check["passed"], flush=True); save()
    report, map_dir = stage("map", lambda: eng.score(fam, p, check, allow_failed_check=True))
    rec["z"], rec["map"] = O.objective_z(report, cfg), str(map_dir)
    print("Z:", {k: round(v["Z"], 3) for k, v in rec["z"].items()}, flush=True); save()
    jam = stage("jam", lambda: eng.jam(fam, p, map_dir))
    rec["held_out"] = stage("held_out", lambda: eng.held_out(fam, p)); save()
    gate, gate_dir = stage("gate", lambda: eng.gate(fam, p))
    rec["gate"] = O.gate_record(gate, gate_dir)
    print("gate:", gate["passed"], {k: (b.get("complete"), b.get("n"), b.get("steps")) for k, b in gate["builds"].items()}, flush=True); save()
    try:
        ring, ring_dir = stage("ring", lambda: eng.ring(fam, p))
        rec["ring"] = {k: ring.get(k) for k in ("status", "seated", "total", "run", "failed_brick")}
    except Exception as e:  # noqa: BLE001
        rec["ring"] = {"error": f"{type(e).__name__}: {e}"[:300]}
    print("ring:", rec["ring"], flush=True); save()
    def renders():
        sim_map = eng.read_map(map_dir)
        picks = eng.drop_videos(fam, p, sim_map, map_dir, out, 5, seed=1)
        RD.jam_map(sim_map, rec["z"], out / "jam_map.png", target="mean", jam=jam, title=f"{module} (Fable 5.2)", picks=picks)
        RD.module_png(fam, p, out / "module.png")
        rec["stl"] = [x.name for x in VD.export_stl(fam, p, out)]
    stage("renders", renders); save()
    print("done", out, flush=True)


if __name__ == "__main__":
    main()
