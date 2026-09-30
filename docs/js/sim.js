// sim.js - thin wrapper around the WebAssembly build of sim/ (C++).
// The same C++ runs the Python analysis natively; tests/test_parity.py checks
// both builds agree bit for bit.
import { WASM_BASE64 } from "./farmesim-wasm.js";

export const UNO = 0, ESP = 1;
export const PROBES = ["mode", "state", "cmd", "dist_mm", "hills", "row", "tank_mL", "flags", "link_ok",
  "frames_ok", "frames_bad", "temp_C", "hum", "seeds_est", "batt_mV"];
export const STATES = ["IDLE", "MANUAL", "CALIBRATE", "DRIVE", "SETTLE", "SEED", "WATER", "BACKOFF",
  "TURN1", "SHIFT", "TURN2", "DONE", "PAUSED", "FAULT"];
export const EV = { MOTOR: 1, PUMP: 2, GATE: 3, FALL: 4, EDGE: 5, TANK_EMPTY: 6, HTTP: 7, WDT: 8, LINK_DROP: 9 };

let modulePromise = null;
function wasmBytes() {
  const bin = (typeof atob === "function" ? atob(WASM_BASE64) : Buffer.from(WASM_BASE64, "base64").toString("binary"));
  const out = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
  return out;
}
export function loadModule() {
  if (!modulePromise) modulePromise = WebAssembly.compile(wasmBytes());
  return modulePromise;
}

// One WebAssembly instance = up to four independent rigs.
export class SimWorld {
  static async create() {
    const mod = await loadModule();
    let ex = null;
    // Asyncify: a sketch's fiber yields by unwinding its stack; resuming rewinds it.
    const env = {
      fiber_suspend(p) {
        if (ex.asyncify_get_state() === 2) ex.asyncify_stop_rewind();
        else ex.asyncify_start_unwind(p);
      },
    };
    const inst = await WebAssembly.instantiate(mod, { env });
    ex = inst.exports;
    return new SimWorld(ex);
  }
  constructor(e) {
    this.e = e;
    e.__wasm_call_ctors();
    const names = (count, get) => {
      const m = {};
      for (let i = 0; i < e[count](); i++) m[this.str(e[get](i))] = i;
      return m;
    };
    this.F = names("sim_field_count", "sim_field_name");
    this.A = names("sim_action_count", "sim_action_name");
    this.P = names("sim_param_count", "sim_param_name");
    this.enc = new TextEncoder();
    this.dec = new TextDecoder("utf-8");
  }
  str(ptr) {
    const m = new Uint8Array(this.e.memory.buffer);
    let end = ptr;
    while (m[end]) end++;
    return new TextDecoder("latin1").decode(m.subarray(ptr, end));
  }
  scratchText(n) {
    return this.dec.decode(new Uint8Array(this.e.memory.buffer, this.e.sim_scratch(), n));
  }
  writeScratch(s) {
    const b = this.enc.encode(s + "\0");
    new Uint8Array(this.e.memory.buffer, this.e.sim_scratch(), b.length).set(b);
  }
  rig(h) { return new SimRig(this, h); }
}

export class SimRig {
  constructor(world, h) { this.w = world; this.e = world.e; this.h = h; this.logPos = [0, 0]; this.evPos = 0; }
  init(params = {}, seed = 1) {
    const { e, h, w } = this;
    e.sim_param(h, -1, 0);
    for (const [k, v] of Object.entries(params)) {
      if (!(k in w.P)) throw new Error(`unknown parameter ${k}`);
      e.sim_param(h, w.P[k], v);
    }
    e.sim_init(h, seed);
    this.logPos = [0, 0];
    this.evPos = 0;
    this.pending = new Map();
    return this;
  }
  get t() { return this.e.sim_time(this.h); }
  run(ms) {
    this.e.sim_run(this.h, ms);
    this.settle();
    return this;
  }
  get(name) { return this.e.sim_get(this.h, this.w.F[name]); }
  set(action, v = 0) { this.e.sim_set(this.h, this.w.A[action], v); return this; }
  probe(board, name) { return this.e.sim_probe(this.h, board, PROBES.indexOf(name)); }
  state() { return STATES[this.probe(UNO, "state")] || "?"; }

  // ---- the phone: returns a promise for {status, body, ms}
  http(url) {
    this.w.writeScratch(url);
    const id = this.e.sim_http(this.h);
    return new Promise((resolve) => this.pending.set(id, resolve));
  }
  // fire-and-forget; returns the request id
  send(url, atMs) {
    this.w.writeScratch(url);
    return atMs == null ? this.e.sim_http(this.h) : this.e.sim_http_at(this.h, atMs);
  }
  response(id) {
    if (this.e.sim_http_state(this.h, id) !== 1) return null;
    const n = this.e.sim_http_body(this.h, id);
    return {
      status: this.e.sim_http_code(this.h, id),
      body: n >= 0 ? this.w.scratchText(n) : "",
      ms: this.e.sim_http_latency_ms(this.h, id),
    };
  }
  settle() {
    if (!this.pending || !this.pending.size) return;
    for (const [id, resolve] of this.pending) {
      const r = this.response(id);
      if (r) { this.pending.delete(id); resolve(r); }
    }
  }

  // ---- serial monitors
  log(board) {
    const n = this.e.sim_log(this.h, board, this.logPos[board]);
    const s = new TextDecoder("latin1").decode(new Uint8Array(this.e.memory.buffer, this.e.sim_scratch(), n));
    this.logPos[board] = this.get(board === UNO ? "uno_log_total" : "esp_log_total");
    return s;
  }
  events() {
    const n = this.e.sim_events(this.h, this.evPos);
    const f = new Float32Array(this.e.memory.buffer, this.e.sim_scratch(), n * 5);
    const out = [];
    for (let i = 0; i < n; i++) out.push({ t: f[i * 5], type: f[i * 5 + 1], a: f[i * 5 + 2], b: f[i * 5 + 3], c: f[i * 5 + 4] });
    this.evPos = this.get("events_total");
    return out;
  }

  // ---- world arrays (views are only valid until the next run)
  seeds() { return new Float32Array(this.e.memory.buffer, this.e.sim_seeds(this.h), this.get("nseeds") * 3); }
  wet() { return new Float32Array(this.e.memory.buffer, this.e.sim_wet(this.h), this.get("nwet") * 4); }
  trail() { return new Float32Array(this.e.memory.buffer, this.e.sim_trail(this.h), this.get("ntrail") * 6); }
  hills() {
    const n = this.get("hills");
    const col = (k) => new Float32Array(this.e.memory.buffer, this.e.sim_hills(this.h, k), n).slice();
    return { x: col(0), y: col(1), n: col(2), ext: col(3), w: col(4), count: n };
  }
  uiHtml() { const n = this.e.sim_ui_html(this.h); return this.w.scratchText(n); }
}
