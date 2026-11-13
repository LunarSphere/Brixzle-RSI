"""Per-iteration renders (issue #15): jam map, module render, summary and progress.

Every outer iteration of the loop (#21) writes to ``RSI_RESULTS/<run>/iter-NN/``:

- ``jam_map.png``: the full sim map (#48), one panel per slice and course, coloured like
  aerial-assembly's ``figures/biasbrick-drop-grid``: seated green, jammed orange, fell red, with
  wrong valley in purple (it is the A/B baseline's main failure, so it is not hidden in another
  colour). Counts are in each legend, the target box is drawn on every panel, the jam bands from
  the jam analysis (#49) are outlined, and Z (#61) per target is in the title. Labelled
  "simulated, uncalibrated contact".
- ``module.png``: the iteration's brick as three orthographic views of its printed solid (front, side
  and top), one row per part, to scale (issue #74).
- ``summary.md``: Z per target with its counts, what changed and why (the critic's brief), the
  decision, and the run directories behind every number (written by the loop).

``RSI_RESULTS/<run>/progress.png`` shows Z per iteration, with kept and rejected designs marked.
RSI_RESULTS/ is git-ignored; figures for the repo go through #25.
"""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402
from matplotlib.patches import Circle, Patch, Rectangle  # noqa: E402
import numpy as np  # noqa: E402

LABEL = "simulated, uncalibrated contact"
BG = "#fcfbf8"
COLOURS = {"S": "#13a10e", "J": "#f6b21b", "F": "#d23c3c", "W": "#7b52ab", "?": "#9a9a9a"}
NAMES = {"S": "seated", "J": "jammed", "F": "fell", "W": "wrong valley", "?": "other"}
ORDER = ("S", "J", "F", "W", "?")
AXIS_LABELS = {"x": "X offset about the aim (mm)", "y": "Y offset about the aim (mm)",
               "z": "release height above the minimum (mm)", "t": "pitch (deg)"}
SLICE_ORDER = ["xz", "xt", "zt", "xy"]
DPI = 110


def _grid_array(g):
    """(rows, cols) of codes, row 0 = lowest value of the second axis."""
    return np.array([list(r) for r in g["rows"]])


def _box_rect(g, box):
    a, b = g["axes"]
    hw = box["half_widths"]
    y = hw.get("y_mm", 0.0)
    ranges = {"x": (-hw["x_mm"], hw["x_mm"]), "y": (-y, y), "z": (0.0, 2 * hw["z_mm"]),
              "t": (-hw["theta_deg"], hw["theta_deg"])}
    (x0, x1), (y0, y1) = ranges[a], ranges[b]
    return x0, y0, x1 - x0, y1 - y0


def legend_counts(g):
    """{outcome name: cells} for one grid, in legend order, outcomes with no cell left out."""
    arr = _grid_array(g)
    known = set(ORDER) - {"?"}
    counts = {c: int((arr == c).sum()) for c in ORDER if c != "?"}
    counts["?"] = int(sum(1 for v in arr.flat if v not in known))
    return {NAMES[c]: n for c, n in counts.items() if n}


def jam_map_text(sim_map, z, target=None, title=None):
    """The figure's title: what was mapped, Z per target, and the label."""
    head = title or f"{sim_map['family']} drop map, aimed at the {sim_map['box']['statistic']} box"
    zline = "   ".join(f"Z {s} {c['Z']:+.3f} (S {c['S']}, J {c['J']}, N {c['N']})"
                       + (" target" if s == target else "") for s, c in z.items())
    return f"{head}\n{zline}\n{LABEL.capitalize()} (MuJoCo); not a hardware rate"


def _fmt_fixed(fixed):
    units = {"x": "mm", "y": "mm", "z": "mm", "t": "deg"}
    return ", ".join(f"{k} {v:.1f} {units.get(k, '')}".strip() for k, v in fixed.items())


def pick_marks(sim_map, picks):
    """Where each picked drop (loop/videos.py) sits on the map: (slice, course, x, y, k), the
    coordinates in the slice's own two axes."""
    axes = {(g["slice"], g["course"]): g["axes"] for g in sim_map["grids"]}
    out = []
    for q in picks or ():
        a, b = axes.get((q["slice"], q["course"]), (None, None))
        if a is not None:
            out.append((q["slice"], q["course"], q["pose"][a], q["pose"][b], q["k"]))
    return out


def jam_map(sim_map, z, out, target=None, jam=None, title=None, picks=None):
    """Draw the map's grids to ``out`` (PNG). ``z`` is {statistic: counts}; ``jam`` the jam analysis."""
    grids = sorted(sim_map["grids"], key=lambda g: (SLICE_ORDER.index(g["slice"]), g["course"]))
    slices = sorted({g["slice"] for g in grids}, key=SLICE_ORDER.index)
    courses = sorted({g["course"] for g in grids})
    fig, axes = plt.subplots(len(slices), len(courses), figsize=(6.4 * len(courses), 3.3 * len(slices)),
                             squeeze=False, facecolor=BG)
    target = target or sim_map["box"]["statistic"]
    box = next((b for b in sim_map.get("cover_boxes", [sim_map["box"]]) if b["statistic"] == target), sim_map["box"])
    cmap = ListedColormap([COLOURS[c] for c in ORDER])
    for g in grids:
        ax = axes[slices.index(g["slice"])][courses.index(g["course"])]
        ax.set_facecolor(BG)
        a, b = g["axes"]
        arr = _grid_array(g)
        idx = np.vectorize(lambda c: ORDER.index(c) if c in ORDER else ORDER.index("?"))(arr)
        sa, sb = g["step"][a], g["step"][b]
        (ia0, _), (ib0, _) = g["index_extent"][a], g["index_extent"][b]
        extent = (ia0 * sa - sa / 2, (ia0 + arr.shape[1]) * sa - sa / 2,
                  ib0 * sb - sb / 2, (ib0 + arr.shape[0]) * sb - sb / 2)
        ax.imshow(idx, origin="lower", extent=extent, cmap=cmap, vmin=-0.5, vmax=len(ORDER) - 0.5,
                  aspect="auto", interpolation="nearest")
        if (a, b) == ("x", "y") and box.get("xy_shape") == "disc":
            # The X-Y target is a disc (issue #86): x^2 + y^2 <= r^2.
            ax.add_patch(Circle((0.0, 0.0), box["half_widths"]["x_mm"], fill=False, lw=1.6, ls="--", ec="black"))
            ax.set_aspect("equal", adjustable="datalim")
        else:
            x, y, w, h = _box_rect(g, box)
            if h < sb:  # a box thinner than one cell (pitch): draw it one cell tall so it shows
                y, h = y + h / 2 - sb / 2, sb
            ax.add_patch(Rectangle((x, y), w, h, fill=False, lw=1.6, ls="--", ec="black"))
        for band in (jam or {}).get("bands", []):
            if band["slice"] == g["slice"] and band["course"] == g["course"]:
                (b0, b1), (c0, c1) = band["extent"][a], band["extent"][b]
                ax.add_patch(Rectangle((b0 - sa / 2, c0 - sb / 2), b1 - b0 + sa, c1 - c0 + sb, fill=False,
                                       lw=1.4, ec="black"))
        for sl, co, x, y, k in pick_marks(sim_map, picks):
            if (sl, co) == (g["slice"], g["course"]):
                ax.plot(x, y, marker="o", ms=11, mfc="white", mec="black", mew=1.2, zorder=5)
                ax.text(x, y, str(k), ha="center", va="center", fontsize=7, fontweight="bold", zorder=6)
        names = legend_counts(g)
        ax.legend(handles=[Patch(color=COLOURS[c], label=f"{NAMES[c]} ({names[NAMES[c]]})") for c in ORDER
                           if NAMES[c] in names]
                  + [Patch(fill=False, ls="--", ec="black", label=f"{target} target box")],
                  loc="upper left", fontsize=7, framealpha=0.85)
        on = "onto the base" if g["course"] == 0 else "across two seated bricks"
        fixed = _fmt_fixed(g["fixed"])
        ax.set_title(f"{g['slice']} slice, course {g['course']} ({on}); {fixed}", fontsize=9, loc="left")
        ax.set_xlabel(AXIS_LABELS[a], fontsize=8)
        ax.set_ylabel(AXIS_LABELS[b], fontsize=8)
        ax.tick_params(labelsize=7)
    fig.suptitle(jam_map_text(sim_map, z, target, title), fontsize=10, x=0.01, y=0.995, ha="left", va="top")
    fig.tight_layout(rect=(0, 0, 1, 1 - 0.75 / fig.get_figheight()))  # room for the three-line title
    fig.savefig(out, dpi=DPI, facecolor=BG)
    plt.close(fig)
    return Path(out)


# The three orthographic views (issue #74): which mesh axes each shows, and where the viewer stands.
VIEWS = (("front", ("x", "z"), (0.0, -1.0, 0.0)),   # looking along +Y
         ("side", ("y", "z"), (1.0, 0.0, 0.0)),     # looking along -X, from the +X side
         ("top", ("x", "y"), (0.0, 0.0, 1.0)))      # looking down
_AXIS = {"x": 0, "y": 1, "z": 2}


def ortho_panels(family, p):
    """The three orthographic views of each part's printed solid, as data: for each part and
    view, the projected triangles that face the viewer (farthest first, so nearer ones cover
    them), their shading, and the view's extent in mm."""
    from brixzle.families import builders as B

    out = []
    for part in family.parts:
        mesh = B.sim_bodies(family, p, part).mesh
        tris, normals = np.asarray(mesh.triangles), np.asarray(mesh.face_normals)
        lo, hi = mesh.bounds
        for view, axes, eye in VIEWS:
            eye = np.asarray(eye)
            facing = normals @ eye
            keep = facing > 1e-9
            depth = tris[keep] @ eye                      # larger is nearer the viewer
            order = np.argsort(depth.mean(axis=1))        # farthest first
            a, b = _AXIS[axes[0]], _AXIS[axes[1]]
            light = eye + np.array([-0.45, -0.35, 0.55])  # off the view axis, so flat faces differ
            light /= np.linalg.norm(light)
            shade = np.clip(normals[keep] @ light, 0.0, 1.0)
            out.append({"part": part, "view": view, "axes": axes,
                        "polys": tris[keep][order][:, :, [a, b]], "shade": shade[order],
                        "extent": ((float(lo[a]), float(hi[a])), (float(lo[b]), float(hi[b])))})
    return out


def module_png(family, p, out, title=None):
    """The family's brick as three orthographic views of its printed solid (front X-Z, side Y-Z,
    top X-Y), one row per part, to scale in mm (issue #74)."""
    from matplotlib.collections import PolyCollection

    panels = ortho_panels(family, p)
    parts = list(family.parts)
    fig, axes = plt.subplots(len(parts), len(VIEWS), figsize=(4.0 * len(VIEWS), 3.6 * len(parts)), squeeze=False,
                             facecolor=BG)
    base = np.array([0.62, 0.71, 0.84])
    for q in panels:
        ax = axes[parts.index(q["part"])][[v[0] for v in VIEWS].index(q["view"])]
        ax.set_facecolor(BG)
        colours = np.clip(base[None, :] * (0.45 + 0.55 * q["shade"][:, None]), 0, 1)
        ax.add_collection(PolyCollection(q["polys"], facecolors=colours, edgecolors=colours, linewidths=0.2))
        (x0, x1), (y0, y1) = q["extent"]
        pad = 0.05 * max(x1 - x0, y1 - y0)
        ax.set_xlim(x0 - pad, x1 + pad)
        ax.set_ylim(y0 - pad, y1 + pad)
        ax.set_aspect("equal")
        a, b = q["axes"]
        ax.set_xlabel(f"{a.upper()} (mm)", fontsize=8)
        ax.set_ylabel(f"{b.upper()} (mm)", fontsize=8)
        ax.tick_params(labelsize=7)
        ax.set_title(f"part {q['part']}: {q['view']} ({a.upper()}-{b.upper()}), {x1 - x0:.1f} x {y1 - y0:.1f} mm",
                     fontsize=9, loc="left")
    fig.suptitle(title or f"{family.name} {family.version} ({type(family.geometry).__name__}): orthographic views",
                 fontsize=10, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(out, dpi=DPI, facecolor=BG)
    plt.close(fig)
    return Path(out)


def progress_png(history, out, target, title=None, baseline=None):
    """Z per iteration on the target box (thick) and the other boxes (thin), kept designs filled,
    rejected ones hollow, iterations with no scored design marked on the axis.

    ``history`` is [{iteration, decision, z: {statistic: counts} | None, target}]: ``target`` is
    the rung each design was judged on (#84; default ``target``); a dashed line marks where it
    changed, and the marker shows Z on that rung. ``baseline`` is the parent's {family, z}
    (iteration 0, #85), drawn as one horizontal line per box. An ``overhang_n`` in the history
    (#83) is drawn on a second axis, in bricks."""
    fig, ax = plt.subplots(figsize=(7.0, 3.8), facecolor=BG)
    ax.set_facecolor(BG)
    stats = []
    for h in history:
        for s in (h.get("z") or {}):
            if s not in stats:
                stats.append(s)
    styles = {"mean": "#1f3b63", "p95": "#5b7fb5", "max": "#9fb4d6"}
    for s, c in ((baseline or {}).get("z") or {}).items():
        ax.axhline(c["Z"], color=styles.get(s, "#555"), lw=1.0, ls=":",
                   label=f"{baseline['family']} baseline, Z {s} {c['Z']:+.3f}")
    for s in stats:
        pts = [(h["iteration"], h["z"][s]["Z"]) for h in history if h.get("z")]
        if pts:
            xs, ys = zip(*pts)
            ax.plot(xs, ys, color=styles.get(s, "#555"), lw=2.4 if s == target else 1.0,
                    label=f"Z {s}" + (" (target)" if s == target else ""))
    rung = None
    for h in history:
        t = h.get("target") or target
        if rung is not None and t != rung:
            ax.axvline(h["iteration"] - 0.5, color="#666", lw=0.9, ls="--")
            ax.text(h["iteration"] - 0.45, -0.95, f"rung -> {t}", fontsize=7, color="#444", rotation=90, va="bottom")
        rung = t
        if not h.get("z"):
            ax.plot(h["iteration"], -1.0, marker="x", color=COLOURS["F"], ms=8, ls="none")
            continue
        zt = h["z"][t]["Z"]
        kept = h.get("decision") == "kept"
        ax.plot(h["iteration"], zt, marker="o", ms=9, ls="none", mec=styles.get(t, "#1f3b63"),
                mfc=styles.get(t, "#1f3b63") if kept else "white")
    ax.plot([], [], marker="o", ls="none", mfc="#1f3b63", mec="#1f3b63", label="kept (Z on its rung)")
    ax.plot([], [], marker="o", ls="none", mfc="white", mec="#1f3b63", label="rejected")
    ax.plot([], [], marker="x", ls="none", color=COLOURS["F"], label="no design passed the quick check and the gate")
    over = [(h["iteration"], h["overhang_n"]) for h in history if h.get("overhang_n") is not None]
    if over:
        ax2 = ax.twinx()
        xs, ys = zip(*over)
        ax2.plot(xs, ys, color="#b5651d", lw=1.2, ls="-.", marker="s", ms=4)
        ax2.set_ylabel("overhang (bricks)", color="#b5651d")
        ax2.set_ylim(0, max(12, max(ys)) + 0.5)
        ax2.tick_params(axis="y", colors="#b5651d")
        ax.plot([], [], color="#b5651d", lw=1.2, ls="-.", marker="s", ms=4, label="overhang (right axis)")
    ax.axhline(1.0, color=COLOURS["S"], lw=1, ls="--")
    ax.set_ylim(-1.05, 1.05)
    ax.set_xlabel("iteration")
    ax.set_ylabel("Z = (seated - jammed) / drops in box")
    ax.set_xticks([h["iteration"] for h in history] or [1])
    ax.legend(fontsize=7, loc="lower right")
    ax.set_title(title or f"Z per iteration (target {target}); {LABEL}", fontsize=9, loc="left")
    fig.tight_layout()
    fig.savefig(out, dpi=DPI, facecolor=BG)
    plt.close(fig)
    return Path(out)


# --- summaries (markdown) ---


def _z_table(z, target):
    rows = ["| Target | Z | S | J | Fell | Wrong valley | Other | N |", "|---|---|---|---|---|---|---|---|"]
    for s, c in z.items():
        name = f"**{s}** (target)" if s == target else s
        rows.append(f"| {name} | {c['Z']:+.3f} | {c['S']} | {c['J']} | {c.get('fell', 0)} | "
                    f"{c.get('wrong_valley', 0)} | {c.get('other', 0)} | {c['N']} |")
    return "\n".join(rows)


def _brief_lines(brief):
    if not brief:
        return ["- none"]
    out = [f"- **Decision:** {brief['decision']}. {brief['reason']}"]
    for c in brief.get("clusters", []):
        out.append(f"- **{c['failure']}** ({', '.join(c['rules'])}; {c['scope']}): {c['diagnosis']} "
                   f"Change: {c['change']} Expected: {c['expected_effect']}")
    return out


GATE_SLOW_S = 60.0  # a gate slower than this is flagged in the summary (#83)


def gate_lines(gate, seconds=None):
    """The structure gate's table for summary.md (#83)."""
    if not gate:
        return []
    lines = [f"## Structure gate", "",
             f"{'Passed' if gate['passed'] else 'FAILED'}: overhang {gate['overhang_n']} bricks "
             f"(floor {gate['overhang_floor']}). Each structure built on {gate.get('repeats', '?')} seeded repeats with "
             f"ideal placement and a tiny release jitter; it passes when most complete. {LABEL}.", "",
             "| Structure | Verdict | Complete | Stable bricks per repeat | Stopped at |", "|---|---|---|---|---|"]
    for name, b in gate["builds"].items():
        if b.get("error"):
            lines.append(f"| {name} | failed | - | - | {b['error']} |")
            continue
        stops = ", ".join(sorted({x for x in b["failed_brick"] if x})) or "-"
        lines.append(f"| {name} | {'passed' if b['passed'] else 'failed'} | {b['complete']} of {b['n']} | "
                     f"{b['stable_bricks']} of {b['total']} | {stops} |")
    if seconds is not None and seconds > GATE_SLOW_S:
        lines += ["", f"The gate took {seconds:.1f} s, more than {GATE_SLOW_S:.0f} s."]
    return lines + [""]


def iteration_summary(rec, run, target, ledger_path, spent_usd, cap_usd):
    """summary.md for one iteration, from the loop's record of it (orchestrator.Loop.iterate)."""
    i = rec["iteration"]
    lines = [f"# {run}: iteration {i:02d}", ""]
    fam = rec.get("family", "no design")
    lines += [f"**Design:** {fam}" + (f" (`{rec['module']}`)" if rec.get("module") else "")
              + f". **Decision: {rec['decision']}.** {rec.get('reason', '')}", "",
              f"Sim results are {LABEL}; mechanisms in the jam analysis are predicted, quasi-static 2D. "
              "Neither is a hardware rate.", ""]
    if rec.get("z"):
        lines += [f"The design was judged on the **{target}** box (the target ladder's rung for this iteration, #84).",
                  ""]
    if rec.get("quick_check") and not rec.get("attempts"):
        qc = rec["quick_check"]
        lines += [f"Quick check: {qc['seated']} of {qc['dropped']} seated ("
                  + ("passed" if qc["passed"] else "failed; the baseline is scored either way") + ").", ""]
    if rec.get("overhang_n") is not None:
        lines += [f"Overhang: {rec['overhang_n']} bricks (the second objective, #83).", ""]
    if rec.get("mass_g"):
        lines += ["Mass per part: " + ", ".join(f"{k} {v:.2f} g" for k, v in rec["mass_g"].items()) + ".", ""]
    lines += ["## Z per target box", ""]
    if rec.get("z"):
        lines += [_z_table(rec["z"], target), "",
                  "Z = (S − J) / N over the full sim map's drops inside each box (#61).", ""]
        if rec.get("held_out"):
            lines += ["Held-out Z on the target box (poses shifted by another seed; the critic never sees it):", "",
                      "| Check | Z | S | J | N |", "|---|---|---|---|---|"]
            lines += [f"| {k} | {c['Z']:+.3f} | {c['S']} | {c['J']} | {c['N']} |" for k, c in rec["held_out"].items()]
            lines.append("")
    else:
        lines += ["No design reached the full map this iteration, so there is no Z.", ""]
    lines += gate_lines(rec.get("gate"), (rec.get("seconds") or {}).get("gate"))
    lines += ["## What changed and why", "", "The brief this iteration followed (the critic's, from the last iteration):", ""]
    lines += _brief_lines(rec.get("brief_in")) + [""]
    note = rec.get("note") or {}
    if note:
        lines += [f"The design agent's note: {note.get('summary', '')}", "",
                  f"Changes: {note.get('changes', '')}", "",
                  f"Geometry: {note.get('geometry', '?')}" + (f" ({note['solid3d_reason']})"
                                                              if note.get("solid3d_reason") else ""), ""]
    lines += ["Attempts:", "", "| Attempt | Package | Outcome | Quick check | Structure gate |", "|---|---|---|---|---|"]
    for a in rec.get("attempts", []):
        qc, g = a.get("quick_check"), a.get("gate")
        q = f"{qc['seated']} of {qc['dropped']} seated" if qc else "not run"
        gt = (f"{'passed' if g['passed'] else 'failed'}, overhang {g['overhang_n']}" if g else "not run")
        lines.append(f"| {a['attempt']} | `{a['module']}` | {a.get('decision', '?')} | {q} | {gt} |")
    lines += ["", "## The critic's brief for the next iteration", ""]
    lines += _brief_lines(rec.get("brief")) if rec.get("brief") else [
        f"- none ({rec.get('critic_error') or 'the critic was not run this iteration'})"]
    if rec.get("videos"):
        lines += ["", "## Drop videos", "",
                  "Picked at random from the full sim map (seeded) and marked with their numbers on `jam_map.png`.", "",
                  "| # | Video | Slice, course | Pose | Outcome in the map |", "|---|---|---|---|---|"]
        for q in rec["videos"]:
            v = q.get("video") or f"not rendered: {q.get('error', '?')}"
            pose = f"x {q['pose']['x']:+.1f} mm, z {q['pose']['z']:.1f} mm, pitch {q['pose']['t']:+.1f} deg"
            again = q.get("rerun_outcome")
            note = f" (re-run: {again})" if again and again != q["outcome"] else ""
            lines.append(f"| {q['k']} | `{v}` | {q['slice']}, course {q['course']} | {pose} | {q['outcome']}{note} |")
    if rec.get("stl"):
        lines += ["", "## STL", "", "The printed solid of each part, in mm:", ""] + [f"- `{n}`" for n in rec["stl"]]
    lines += ["", "## Runs behind these numbers", ""]
    if rec.get("runs", {}).get("quick_check") and not rec.get("attempts"):
        lines.append(f"- quick check (seed in `check.json`): `{rec['runs']['quick_check']}`")
    if rec.get("runs", {}).get("gate") and not rec.get("attempts"):
        lines.append(f"- structure gate (seed in `gate.json`): `{rec['runs']['gate']}`")
    for a in rec.get("attempts", []):
        lines.append(f"- design package, attempt {a['attempt']}: `{a['root']}/{a['module']}`")
        if a.get("quick_check"):
            lines.append(f"- quick check, attempt {a['attempt']} (seed in `check.json`): `{a['quick_check']['run']}`")
        if a.get("gate"):
            lines.append(f"- structure gate, attempt {a['attempt']} (seed in `gate.json`): `{a['gate']['run']}`")
    runs = rec.get("runs", {})
    if runs.get("map"):
        lines += [f"- full sim map and report (Z): `{runs['map']}/map.json`, `{runs['map']}/report.json`",
                  f"- jam analysis: `{runs.get('jam')}`"]
    lines += [f"- loop state and design log: `{Path(rec['dir']).parent}/state.json`, "
              f"`{Path(rec['dir']).parent}/design_log.jsonl`",
              f"- API spend: ${rec.get('spent_usd', 0.0):.4f} this iteration; ${spent_usd:.4f} of ${cap_usd:.2f} "
              f"in all (`{ledger_path}`)",
              "- wall-clock (s): " + ", ".join(f"{k} {v:.1f}" for k, v in rec.get("seconds", {}).items())]
    return "\n".join(lines) + "\n"


def _zc(c):
    return f"{c['Z']:+.3f} (S {c['S']}, J {c['J']}, N {c['N']})"


def baseline_table(state):
    """'Best against the baseline' (#85): Z on each box with its counts and the mass of each part,
    for the parent (iteration 0) and the best design. Empty without a baseline."""
    base = next((i for i in state["iterations"] if i["iteration"] == 0 and i.get("z")), None)
    best = state.get("best")
    if base is None or best is None:
        return []
    best_rec = next((i for i in state["iterations"] if i["iteration"] == best["iteration"]), best)
    cols = [(f"baseline: {base['family']} (iteration 0)", base)]
    if best["iteration"] != 0:
        cols.append((f"best: {best['family']} (iteration {best['iteration']})", best_rec))
    out = ["## Best against the baseline", "",
           "Simulated, uncalibrated contact. Z = (S − J) / N over each box's drops (#61).", "",
           "| | " + " | ".join(c[0] for c in cols) + " |", "|---|" + "---|" * len(cols)]
    for s in base["z"]:
        out.append(f"| Z {s} | " + " | ".join(_zc(r["z"][s]) if s in (r.get("z") or {}) else "–" for _, r in cols)
                   + " |")
    out.append("| mass per part (g) | " + " | ".join(
        ", ".join(f"{k} {v:.2f}" for k, v in (r.get("mass_g") or {}).items()) or "–" for _, r in cols) + " |")
    out.append("| map | " + " | ".join(f"`{r.get('runs', {}).get('map') or r.get('map')}`" for _, r in cols) + " |")
    if best["iteration"] == 0:
        out += ["", "No design beat the baseline, so the baseline is still the best."]
    return out + [""]


def run_summary(state, target, ledger_path, spent_usd, cap_usd):
    """summary.md for the whole run."""
    lines = [f"# {state['run']}", "", f"Starting box: **{target}**. Sim results are {LABEL}.", "",
             "| Iteration | Design | Decision | Z on its rung | Z mean | Z p95 | Z max | Rung (#84) |",
             "|---|---|---|---|---|---|---|---|"]
    for it in state["iterations"]:
        z = it.get("z") or {}

        def zz(s):
            return f"{z[s]['Z']:+.3f}" if s in z else "–"

        rung = it.get("target") or target
        lines.append(f"| {it['iteration']} | {it.get('family', '–')} | {it['decision']} | {zz(rung)} | "
                     f"{zz('mean')} | {zz('p95')} | {zz('max')} | {rung if z else '–'} |")
    best = state.get("best")
    lines += ["", f"**Stopped:** {state.get('stop')}", "",
              f"**Best:** {best['family']} from iteration {best['iteration']}, Z {best.get('target', target)} "
              f"{best['z_target']:+.3f} "
              f"(`{best.get('family_py') or best['root'] + '/' + best['module']}`; map `{best['map']}`)" if best
              else "**Best:** none", ""]
    lines += baseline_table(state)
    tune = state.get("tune")
    if tune:
        lines += ["**End tune:** " + (f"error: {tune['error']}" if "error" in tune else
                                      f"Z {tune.get('target', target)} {tune['baseline']:+.3f} -> "
                                      f"{tune['tuned']:+.3f}, "
                                      f"{'accepted' if tune['accepted'] else 'not accepted'} (`{tune['run']}`)"), ""]
    lines += [f"API spend: ${spent_usd:.4f} of ${cap_usd:.2f} (`{ledger_path}`).",
              f"Wall-clock: {state.get('elapsed_s', 0.0) / 60:.1f} min.", "", "![progress](progress.png)", ""]
    return "\n".join(lines)
