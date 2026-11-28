"""The module picture (issue #74): a brick as three orthographic views of its printed solid (front,
side and top), one row per part, to scale. The design library draws each card's ``module.png`` with
it. The SDK loop's other renders (jam map, progress, summaries) were removed with that loop (#204, #227).
"""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

BG = "#fcfbf8"
DPI = 110
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
