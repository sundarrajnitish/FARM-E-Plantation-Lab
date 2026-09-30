"""python -m farmelab [--quick] [--no-figures]

Runs every scenario and writes
  results/findings.json      numbers quoted in the README and on the site
  docs/data/findings.json    the same, for the site
  media/figures/*.png        README figures (also copied to docs/img/)
"""
from __future__ import annotations

import argparse
import json
import shutil
import time
from pathlib import Path

from . import scenarios as S

ROOT = Path(__file__).resolve().parent.parent


def main():
    ap = argparse.ArgumentParser(prog="farmelab")
    ap.add_argument("--quick", action="store_true", help="one seed per scenario")
    ap.add_argument("--no-figures", action="store_true")
    a = ap.parse_args()
    seeds = (1,) if a.quick else (1, 2, 3, 4, 5)
    t0 = time.time()
    planting = S.scenario_planting(seeds)
    findings = {
        "planting": planting["mean"],
        "spacings": planting["spacings"],
        "seeds_per_hill": planting["seeds_per_hill"],
        "edges": S.scenario_edges(seeds[:3]),
        "battery": S.scenario_battery(seeds[:3]),
        "control": S.scenario_control(),
        "http_latency": S.scenario_latency(),
        "link": S.scenario_link(seeds),
        "seeds": list(seeds),
    }
    print(f"scenarios     {time.time() - t0:5.1f} s")
    (ROOT / "results").mkdir(exist_ok=True)
    (ROOT / "docs" / "data").mkdir(parents=True, exist_ok=True)
    (ROOT / "results" / "findings.json").write_text(json.dumps(findings, indent=1) + "\n")
    (ROOT / "docs" / "data" / "findings.json").write_text(json.dumps(findings, separators=(",", ":")))
    if not a.no_figures:
        from . import figures
        fig = ROOT / "media" / "figures"
        fig.mkdir(parents=True, exist_ok=True)
        figures.field(planting["trace"], fig / "field.png")
        figures.distributions(planting, fig / "distributions.png")
        for name in ("field.png", "distributions.png"):
            shutil.copy(fig / name, ROOT / "docs" / "img" / name)
    p = findings["planting"]
    print(f"hills {p['hills']:.1f} in {p['rows']:.1f} rows, spacing {p['spacing_mean']*100:.1f} +/- {p['spacing_sd']*100:.1f} cm, "
          f"{p['seeds_per_hill']:.1f} seeds/hill, water on hills {p['water_frac']*100:.0f} %, edge {p['far_overrun']*100:.1f} cm")
    print("edges:", {k: (v.get("fell_count"), v.get("far_overrun"), v.get("state")) for k, v in findings["edges"].items()})
    print("control:", findings["control"])
    print("link:", findings["link"])


if __name__ == "__main__":
    main()
