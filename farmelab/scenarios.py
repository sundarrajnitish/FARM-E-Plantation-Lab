"""Scripted experiments on the simulated robot.

Every scenario returns plain dicts (JSON-ready). `python -m farmelab` runs them
all and writes results/findings.json, docs/data/findings.json and media/figures/*.png.
"""
from __future__ import annotations

import json
import statistics as st

from .native import EV_MOTOR, UNO, Rig


def _mean(xs):
    xs = [x for x in xs if x is not None]
    return round(st.fmean(xs), 4) if xs else None


def _metrics(r: Rig) -> dict:
    keys = ["t_s", "x", "y", "th", "fell", "hills", "seeds_per_hill", "seeds_per_hill_sd", "hill_extent",
            "spacing_mean", "spacing_sd", "water_used", "water_on_hills", "water_frac", "hills_watered",
            "tank_mL", "pump_on_s", "pump_dry_s", "seeds_dropped", "seeds_off_bed", "far_overrun",
            "near_overrun", "dist", "energy_Wh", "batt_V", "link_collide", "uno_loop_max_ms", "esp_loop_max_ms"]
    m = {k: round(r[k], 4) for k in keys}
    m["state"] = r.state
    m["rows"] = int(r.probe(UNO, "row")) + 1
    return m


def plant_run(seed: int = 1, max_s: float = 150.0, handle: int = 0, **params) -> tuple[Rig, dict]:
    """Power up, press "Plant" at t = 2 s, run until the robot reports DONE or FAULT."""
    r = Rig(seed=seed, handle=handle, **params)
    r.run(2.0)
    r.get("/api/cmd?c=auto")
    r.run_until(lambda r: r.state in ("DONE", "FAULT") or (r["fell"] and r.t > 3), max_s - r.t, 0.5)
    return r, _metrics(r)


def summarize(runs: list[dict]) -> dict:
    out = {}
    for k in runs[0]:
        vals = [r[k] for r in runs]
        out[k] = _mean(vals) if isinstance(vals[0], (int, float)) else vals
    out["fell_count"] = sum(1 for r in runs if r["fell"])
    out["n"] = len(runs)
    return out


def field_trace(r: Rig, every: int = 2) -> dict:
    """Arrays for top-down plots (decimated trail)."""
    tr = r.trail()[::every]
    return {
        "trail": [[round(p[0], 2), round(p[1], 3), round(p[2], 3), round(p[3], 3)] for p in tr],
        "seeds": [[round(s[0], 3), round(s[1], 3), round(s[2], 2)] for s in r.seeds()],
        "wet": [[round(w[0], 3), round(w[1], 3), round(w[2], 2), round(w[3], 2)] for w in r.wet()],
        "hills": [[round(h[0], 3), round(h[1], 3), int(h[2]), round(h[3], 3), round(h[4], 1)] for h in r.hills()],
        "bed": [r["bed_len"], r["bed_w"]],
    }


def _hill_spacings(hills):
    out = []
    for a, b in zip(hills, hills[1:]):
        if abs(b[1] - a[1]) <= 0.12:
            out.append(((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5)
    return out


# ---------------------------------------------------------------- scenarios
def scenario_planting(seeds=(1, 2, 3, 4, 5)) -> dict:
    """A full run on the default table-top bed (2.4 x 1.2 m), default settings."""
    runs, trace, spacings, per_hill = [], None, [], []
    for s in seeds:
        r, m = plant_run(s)
        runs.append(m)
        hills = r.hills()
        spacings += _hill_spacings(hills)
        per_hill += [int(h[2]) for h in hills]
        if trace is None:
            trace = field_trace(r)
    return {"mean": summarize(runs), "runs": runs, "spacings": [round(x, 4) for x in spacings],
            "seeds_per_hill": per_hill, "trace": trace}


def scenario_edges(seeds=(1, 2, 3)) -> dict:
    """The end of the bed in different forms, and a sonar with its echo wire open."""
    cases = {
        "table-top, 75 cm drop": dict(),
        "raised bed, 12 cm drop": dict(drop=0.12),
        "shallow step, 6 cm": dict(drop=0.06),
    }
    out = {}
    for name, params in cases.items():
        out[name] = summarize([plant_run(s, **params)[1] for s in seeds])
    r = Rig(seed=1)
    r.set("echo_mode", 1)
    r.run(2)
    r.get("/api/cmd?c=auto")
    r.run(5)
    status = json.loads(r.request("/api/status")[1])
    out["echo wire open"] = {"state": r.state, "fault": status["fault"], "moved_m": round(r["dist"], 3)}
    return out


def scenario_battery(seeds=(1, 2, 3)) -> dict:
    """Hill spacing on a full and on a tired battery (drive times stretch as the pack sags)."""
    out = {}
    for label, soc in (("full (12.5 V)", 0.97), ("tired (11.2 V)", 0.30)):
        runs = [plant_run(s, soc0=soc, max_s=120)[1] for s in seeds]
        out[label] = {"spacing_mean": _mean([m["spacing_mean"] for m in runs]),
                      "spacing_sd": _mean([m["spacing_sd"] for m in runs]),
                      "batt_V": _mean([m["batt_V"] for m in runs])}
    return out


def scenario_control(seed: int = 1) -> dict:
    """Driving from the phone: STOP while moving, the phone leaving mid-drive,
    driving toward an edge, one tap on Seed and on Water."""
    out = {}
    # STOP while driving forward (the page repeats "drive" while the arrow is held)
    r = Rig(seed=seed, start_x=0.4)
    r.run(2.0)
    for k in range(8):
        r.get("/api/cmd?c=drive&d=F&ms=450", at=2.0 + 0.2 * k)
    r.run(1.2)
    t_stop = r.t
    r.get("/api/cmd?c=stop")
    r.run(2.0)
    ev = sorted((e for e in r.events() if e[1] == EV_MOTOR and e[0] > t_stop), key=lambda e: e[0])
    stop_t = next((e[0] for e in ev if e[2] in (0, 2) and e[3] in (0, 2)), None)
    out["stop_ms"] = round((stop_t - t_stop) * 1000) if stop_t else None
    # the phone leaves mid-drive
    r = Rig(seed=seed, start_x=0.4)
    r.run(2.0)
    for k in range(5):
        r.get("/api/cmd?c=drive&d=F&ms=450", at=2.0 + 0.2 * k)
    r.run(1.0)
    x0 = r["x"]
    r.run(10.0)
    out["phone_lost_moved_m"] = round(abs(r["x"] - x0), 3)
    # holding "forward" toward the end of the bed
    r = Rig(seed=seed, start_x=2.0)
    r.run(2)
    for k in range(40):
        r.get("/api/cmd?c=drive&d=F&ms=450", at=2 + 0.2 * k)
    r.run(10)
    out["edge_guard_past_edge_m"] = round(r["sensor_x"] - r["bed_len"], 3)
    out["edge_guard_fell"] = bool(r["fell"])
    # one tap on Seed, one on Water
    r = Rig(seed=seed, start_x=0.9)
    r.run(2)
    r.get("/api/cmd?c=seed")
    r.get("/api/cmd?c=water&ms=1500", at=4.0)
    r.run(10)
    out["seed_tap_seeds"] = int(r["seeds_dropped"])
    out["water_tap_mL"] = round(r["water_used"], 1)
    return out


def scenario_latency(seed: int = 1) -> dict:
    """HTTP response times of the NodeMCU."""
    r = Rig(seed=seed)
    r.run(3.0)
    out = {}
    for u in ["/api/status", "/api/cmd?c=manual", "/"]:
        ms = []
        for _ in range(10):
            resp = r.request(u, 10)
            ms.append(resp[2] if resp else None)
            r.run(0.137)
        out[u] = {"mean_ms": _mean(ms), "max_ms": round(max(m for m in ms if m is not None), 1)}
    return out


def scenario_link(seeds=(1, 2, 3, 4, 5)) -> dict:
    """Link health over full planting runs: bytes lost to half-duplex collisions,
    frames resent, commands never delivered."""
    tot = {"collisions": 0, "retries": 0, "lost": 0, "uno_frames_ok": 0, "uno_loop_max_ms": 0.0, "runs": len(seeds)}
    for s in seeds:
        r, _ = plant_run(s)
        st_ = json.loads(r.request("/api/status")[1])
        tot["collisions"] += int(r["link_collide"])
        tot["retries"] += st_["retries"]
        tot["lost"] += st_["lost"]
        tot["uno_frames_ok"] += int(r.probe(UNO, "frames_ok"))
        tot["uno_loop_max_ms"] = max(tot["uno_loop_max_ms"], round(r["uno_loop_max_ms"], 1))
    return tot
