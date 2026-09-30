"""The firmware on the simulated robot: the UNO runs FarmeDrive, the NodeMCU runs FarmeLink."""
import json

from farmelab import UNO, Rig
from farmelab import scenarios as S


def test_idle_at_power_up_and_serves_its_page():
    r = Rig(seed=1).run(3)
    assert r["dist"] < 0.001 and r["pump"] == 0
    code, body, _ = r.request("/")
    assert code == 200 and "<title>FARM-E</title>" in body
    code, body, ms = r.request("/api/status")
    s = json.loads(body)
    assert s["link"] and s["state"] == "IDLE" and ms < 20
    assert r.request("/nope")[0] == 404


def test_plants_serpentine_rows_of_tidy_hills():
    r, m = S.plant_run(1)
    assert m["state"] == "DONE" and m["rows"] == 3 and not m["fell"]
    assert 3 <= m["seeds_per_hill"] <= 6 and m["hill_extent"] < 0.04
    assert abs(m["spacing_mean"] - 0.25) < 0.02
    assert m["water_frac"] > 0.85 and m["pump_dry_s"] == 0
    assert m["far_overrun"] < 0.04


def test_stops_at_every_kind_of_edge():
    for kw in ({}, {"drop": 0.12}, {"drop": 0.06}):
        _, m = S.plant_run(2, **kw)
        assert not m["fell"] and m["far_overrun"] < 0.04, kw


def test_open_echo_wire_is_a_fault_not_a_drive():
    r = Rig(seed=1)
    r.set("echo_mode", 1)
    r.run(2)
    r.request("/api/cmd?c=auto")
    r.run(5)
    assert r.state == "FAULT" and r["dist"] < 0.001
    assert json.loads(r.request("/api/status")[1])["fault"] == 2
    r.request("/api/cmd?c=clear")
    r.run(0.5)
    assert r.state == "IDLE"


def test_phone_control_is_safe():
    c = S.scenario_control()
    assert c["stop_ms"] < 100
    assert c["phone_lost_moved_m"] < 0.15
    assert not c["edge_guard_fell"] and c["edge_guard_past_edge_m"] < 0.03
    assert 1 <= c["seed_tap_seeds"] <= 8
    assert abs(c["water_tap_mL"] - 27) < 1.5


def test_battery_compensation_keeps_spacing():
    b = S.scenario_battery((1,))
    assert abs(b["full (12.5 V)"]["spacing_mean"] - b["tired (11.2 V)"]["spacing_mean"]) < 0.02


def test_settings_are_range_checked_and_used():
    r = Rig(seed=1).run(2)
    assert json.loads(r.request("/api/cmd?c=set&p=spacing&v=400")[1])["ok"]
    r.run(1)
    r.request("/api/cmd?c=set&p=spacing&v=5000")  # accepted by the link, refused by the UNO
    r.run(1)
    r.request("/api/cmd?c=auto")
    r.run(30)
    _ = r.hills()
    assert abs(r["spacing_mean"] - 0.40) < 0.03


def test_tank_runs_low_and_is_refilled():
    r, m = S.plant_run(1)
    s = json.loads(r.request("/api/status")[1])
    assert s["flags"] & 2 and s["tank_mL"] < 20       # TANK_LOW after 17 doses of 20 mL
    r.request("/api/cmd?c=refill")
    r.run(0.5)
    assert json.loads(r.request("/api/status")[1])["tank_mL"] == 350


def test_http_is_fast():
    lat = S.scenario_latency()
    assert all(v["max_ms"] < 40 for v in lat.values())


def test_link_is_clean_over_a_full_run():
    l = S.scenario_link((1,))
    assert l["lost"] == 0 and l["uno_frames_ok"] > 100
    assert l["uno_loop_max_ms"] < 30
