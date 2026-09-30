"""Figures for the README (media/figures/)."""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

GREEN, DEEP, YELLOW, SOIL, INK, WATER = "#367c2b", "#1f4a17", "#ffde00", "#c9a77c", "#16210f", "#2f6fb5"

plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "axes.edgecolor": "#555",
                     "axes.spines.top": False, "axes.spines.right": False})


def field(tr: dict, out: Path):
    fig, ax = plt.subplots(figsize=(10, 5.2), constrained_layout=True)
    L, W = tr["bed"]
    ax.add_patch(Rectangle((0, -W / 2), L, W, color=SOIL, alpha=0.55, lw=0))
    for w in tr["wet"]:
        ax.scatter(w[0], w[1], s=w[2] * 18, color=WATER, alpha=0.08, lw=0)
    ax.plot([p[1] for p in tr["trail"]], [p[2] for p in tr["trail"]], color=INK, lw=0.8, alpha=0.55, label="path")
    ax.scatter([s[0] for s in tr["seeds"]], [s[1] for s in tr["seeds"]], s=7, color="#3b2412", zorder=3, label="seeds")
    for h in tr["hills"]:
        ax.add_patch(plt.Circle((h[0], h[1]), max(0.03, h[3] / 2), fill=False, ec=GREEN, lw=1.3))
    ax.set_xlim(-0.2, L + 0.2)
    ax.set_ylim(-W / 2 - 0.08, W / 2 + 0.08)
    ax.set_aspect("equal")
    ax.set_title("One planting run from above: path, seeds (dots), hills (circles), water (blue)", loc="left", fontsize=11, color=INK)
    ax.set_xlabel("along the bed (m)")
    ax.set_ylabel("across (m)")
    fig.savefig(out, dpi=140)
    plt.close(fig)


def distributions(planting: dict, out: Path):
    fig, (a, b) = plt.subplots(1, 2, figsize=(10, 3.6), constrained_layout=True)
    sp = [x * 100 for x in planting["spacings"]]
    a.hist(sp, bins=24, color=GREEN, edgecolor="white")
    a.axvline(25, color=INK, ls="--", lw=1)
    a.text(25.3, a.get_ylim()[1] * 0.92, "set: 25 cm", fontsize=9, color=INK)
    a.set_xlabel("distance between neighbouring hills (cm)")
    a.set_ylabel("hills")
    n = planting["seeds_per_hill"]
    ks = list(range(0, max(n) + 2))
    b.bar(ks, [n.count(k) for k in ks], color=YELLOW, edgecolor="#8a7700")
    b.set_xlabel("seeds per hill")
    b.set_ylabel("hills")
    fig.suptitle(f"Five runs, {len(n)} hills", x=0.01, ha="left", fontsize=11, color=INK)
    fig.savefig(out, dpi=140)
    plt.close(fig)
