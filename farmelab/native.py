"""ctypes bindings for sim/build/libfarmesim.so (built on demand with make)."""
from __future__ import annotations

import ctypes
import os
import subprocess
import threading
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SIM_DIR = ROOT / "sim"
LIB_PATH = SIM_DIR / "build" / "libfarmesim.so"

UNO, ESP = 0, 1

# sim::Probe
PROBES = ["mode", "state", "cmd", "dist_mm", "hills", "row", "tank_mL", "flags", "link_ok",
          "frames_ok", "frames_bad", "temp_C", "hum", "seeds_est", "batt_mV"]
# sim::EventType
EV_MOTOR, EV_PUMP, EV_GATE, EV_FALL, EV_EDGE, EV_TANK_EMPTY, EV_HTTP, EV_WDT, EV_LINK_DROP = range(1, 10)
STATES = ["IDLE", "MANUAL", "CALIBRATE", "DRIVE", "SETTLE", "SEED", "WATER", "BACKOFF",
             "TURN1", "SHIFT", "TURN2", "DONE", "PAUSED", "FAULT"]

_lock = threading.Lock()
_lib = None


def _stale() -> bool:
    if not LIB_PATH.exists():
        return True
    built = LIB_PATH.stat().st_mtime
    watched = [p for p in SIM_DIR.rglob("*") if p.suffix in (".cpp", ".h")]
    watched += list((ROOT / "firmware").rglob("*.cpp")) + list((ROOT / "firmware").rglob("*.h"))
    watched += list((ROOT / "firmware").rglob("*.ino"))
    return any(p.stat().st_mtime > built for p in watched if p.is_file() and "build" not in p.parts)


def load() -> ctypes.CDLL:
    """Build (if needed) and load the native simulator."""
    global _lib
    with _lock:
        if _lib is not None:
            return _lib
        if os.environ.get("FARMELAB_NO_BUILD") != "1" and _stale():
            subprocess.run(["make", "-s", "-C", str(SIM_DIR), "native"], check=True)
        lib = ctypes.CDLL(str(LIB_PATH))
        d, i, s, fp = ctypes.c_double, ctypes.c_int, ctypes.c_char_p, ctypes.POINTER(ctypes.c_float)
        sigs = {
            "sim_rig_count": ([], i),
            "sim_param_count": ([], i), "sim_param_name": ([i], s),
            "sim_field_count": ([], i), "sim_field_name": ([i], s),
            "sim_action_count": ([], i), "sim_action_name": ([i], s),
            "sim_scratch": ([], ctypes.c_void_p), "sim_scratch_size": ([], i),
            "sim_param": ([i, i, d], None), "sim_param_get": ([i, i], d),
            "sim_init": ([i, d], i), "sim_run": ([i, d], None), "sim_time": ([i], d),
            "sim_set": ([i, i, d], None), "sim_get": ([i, i], d), "sim_probe": ([i, i, i], d),
            "sim_http": ([i], i), "sim_http_at": ([i, d], i), "sim_http_state": ([i, i], i),
            "sim_http_code": ([i, i], i), "sim_http_latency_ms": ([i, i], d), "sim_http_body": ([i, i], i),
            "sim_log": ([i, i, d], i), "sim_events": ([i, d], i), "sim_ui_html": ([i], i),
            "sim_seeds": ([i], fp), "sim_wet": ([i], fp), "sim_trail": ([i], fp), "sim_hills": ([i, i], fp),
        }
        for name, (args, res) in sigs.items():
            fn = getattr(lib, name)
            fn.argtypes = args
            fn.restype = res
        _lib = lib
        return lib


def _names(count, getter):
    lib = load()
    return {getattr(lib, getter)(k).decode(): k for k in range(getattr(lib, count)())}


class Rig:
    """One simulated FARM-E: the UNO running FarmeDrive, the NodeMCU running
    FarmeLink, the tracked robot and its bed.

    Rig handles 0..3 are independent; use different handles to run several at once.
    """

    def __init__(self, seed: int = 1, handle: int = 0, **params):
        self.lib = load()
        self.h = handle
        self.F = _names("sim_field_count", "sim_field_name")
        self.P = _names("sim_param_count", "sim_param_name")
        self.A = _names("sim_action_count", "sim_action_name")
        self.lib.sim_param(self.h, -1, 0.0)
        for k, v in params.items():
            self.lib.sim_param(self.h, self.P[k], float(v))
        self.lib.sim_init(self.h, float(seed))
        self._log_pos = [0, 0]
        self._ev_pos = 0

    # ---- time
    @property
    def t(self) -> float:
        return self.lib.sim_time(self.h)

    def run(self, seconds: float) -> "Rig":
        self.lib.sim_run(self.h, seconds * 1000.0)
        return self

    def run_until(self, pred, timeout: float, step: float = 0.05) -> bool:
        end = self.t + timeout
        while self.t < end:
            if pred(self):
                return True
            self.run(step)
        return bool(pred(self))

    # ---- state
    def __getitem__(self, name: str) -> float:
        return self.lib.sim_get(self.h, self.F[name])

    def probe(self, board: int, name: str) -> float:
        return self.lib.sim_probe(self.h, board, PROBES.index(name))

    @property
    def state(self) -> str:
        return STATES[int(self.probe(UNO, "state"))]

    def set(self, action: str, value: float = 0.0) -> "Rig":
        self.lib.sim_set(self.h, self.A[action], float(value))
        return self

    # ---- the phone
    def _url(self, url: str):
        b = url.encode()[:1023] + b"\0"
        ctypes.memmove(self.lib.sim_scratch(), b, len(b))

    def get(self, url: str, at: float | None = None) -> int:
        """Queue an HTTP GET (now, or at simulated time `at` seconds); returns the request id."""
        self._url(url)
        if at is None:
            return self.lib.sim_http(self.h)
        return self.lib.sim_http_at(self.h, at * 1000.0)

    def response(self, rid: int):
        """(status, body, latency_ms) or None while the request is still waiting."""
        if self.lib.sim_http_state(self.h, rid) != 1:
            return None
        n = self.lib.sim_http_body(self.h, rid)
        body = ctypes.string_at(self.lib.sim_scratch(), max(n, 0)).decode("latin-1") if n >= 0 else ""
        return self.lib.sim_http_code(self.h, rid), body, self.lib.sim_http_latency_ms(self.h, rid)

    def request(self, url: str, timeout: float = 5.0):
        rid = self.get(url)
        self.run_until(lambda r: r.response(rid) is not None, timeout, 0.01)
        return self.response(rid)

    # ---- logs and events
    def log(self, board: int, since_last: bool = True) -> str:
        pos = self._log_pos[board] if since_last else 0
        n = self.lib.sim_log(self.h, board, float(pos))
        s = ctypes.string_at(self.lib.sim_scratch(), n).decode("latin-1")
        self._log_pos[board] = int(self[("uno" if board == UNO else "esp") + "_log_total"])
        return s

    def events(self, since_last: bool = False):
        pos = self._ev_pos if since_last else 0
        n = self.lib.sim_events(self.h, float(pos))
        buf = (ctypes.c_float * (n * 5)).from_address(self.lib.sim_scratch())
        out = [(buf[k * 5], int(buf[k * 5 + 1]), int(buf[k * 5 + 2]), int(buf[k * 5 + 3]), int(buf[k * 5 + 4]))
               for k in range(n)]
        self._ev_pos = int(self["events_total"])
        return out

    # ---- arrays
    def _floats(self, ptr, n, stride):
        return [tuple(ptr[i * stride + k] for k in range(stride)) for i in range(n)]

    def seeds(self):
        return self._floats(self.lib.sim_seeds(self.h), int(self["nseeds"]), 3)

    def wet(self):
        return self._floats(self.lib.sim_wet(self.h), int(self["nwet"]), 4)

    def trail(self):
        return self._floats(self.lib.sim_trail(self.h), int(self["ntrail"]), 6)

    def hills(self):
        n = int(self["hills"])
        cols = [self.lib.sim_hills(self.h, k) for k in range(5)]
        return [tuple(c[i] for c in cols) for i in range(n)]

    def ui_html(self) -> str:
        n = self.lib.sim_ui_html(self.h)
        return ctypes.string_at(self.lib.sim_scratch(), n).decode("utf-8")
