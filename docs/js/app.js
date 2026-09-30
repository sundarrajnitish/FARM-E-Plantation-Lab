// app.js - wires the page together: results, the 3D hero, the field trial,
// the phone, the sonar and UART labs, and the live state machine.
import { SimWorld, UNO, ESP } from "./sim.js";

const $ = (s, el = document) => el.querySelector(s);
const $$ = (s, el = document) => [...el.querySelectorAll(s)];
const REDUCED = matchMedia("(prefers-reduced-motion: reduce)").matches;
const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
const fmt = (v, d = 0) => (v == null || Number.isNaN(v) ? "–" : Number(v).toFixed(d));
const onScreen = (el, margin = 150) => { const r = el.getBoundingClientRect(); return !document.hidden && r.bottom > -margin && r.top < innerHeight + margin; };

// ---------------------------------------------------------------- theme + nav
const labRedrawers = [];
(function theme() {
  const btn = $("#themeBtn");
  let saved = null;
  try { saved = localStorage.getItem("farme-theme"); } catch (e) { /* storage blocked */ }
  if (saved) document.documentElement.dataset.theme = saved;
  const label = () => { const t = document.documentElement.dataset.theme; btn.textContent = t === "dark" ? "Dark" : t === "light" ? "Light" : "Auto"; };
  label();
  btn.addEventListener("click", () => {
    const cur = document.documentElement.dataset.theme;
    const next = !cur ? "dark" : cur === "dark" ? "light" : "";
    if (next) document.documentElement.dataset.theme = next; else delete document.documentElement.dataset.theme;
    try { next ? localStorage.setItem("farme-theme", next) : localStorage.removeItem("farme-theme"); } catch (e) { /* ignore */ }
    label();
    labRedrawers.forEach((f) => f());
  });
  const links = $$(".nav a");
  const sections = $$("main section[id]");
  const mark = () => {
    let cur = sections[0].id;
    for (const s of sections) if (s.getBoundingClientRect().top < innerHeight * 0.45) cur = s.id;
    links.forEach((a) => a.classList.toggle("on", a.getAttribute("href") === `#${cur}`));
  };
  addEventListener("scroll", mark, { passive: true });
  mark();
})();

// ---------------------------------------------------------------- results
async function loadResults() {
  let f;
  try { f = await (await fetch("data/findings.json")).json(); } catch (e) { return; }
  const P = f.planting, E = f.edges, B = f.battery, C = f.control, H = f.http_latency, L = f.link;
  const k = (key, html) => { const el = $(`[data-k="${key}"]`); if (el) el.innerHTML = html; };
  k("spacing", `${fmt(P.spacing_mean * 100, 1)} <small>cm</small>`);
  k("seeds", `${fmt(P.seeds_per_hill, 1)}`);
  k("water", `${Math.round(P.water_frac * 100)}%`);
  k("edge", `${fmt(P.far_overrun * 100, 1)} <small>cm</small>`);
  const cm = (m) => `${(m * 100).toFixed(1)} cm`;
  const edgeRow = (name) => E[name] ? `${cm(E[name].far_overrun)}` : "–";
  const cards = [
    { t: "Even spacing", d: "Distance between neighbouring hills in the same row. Drive time comes from the set spacing and the battery-compensated ground speed; counting starts once the tracks are up to speed.",
      a: [cm(P.spacing_mean), "mean, set to 25 cm"], b: [`± ${cm(P.spacing_sd)}`, "standard deviation"], test: "test_plants_serpentine_rows_of_tidy_hills" },
    { t: "Tight hills", d: "The flap opens only with the robot stopped, for 80 ms of travel plus the set seed time, so each hill is one small cluster.",
      a: [fmt(P.seeds_per_hill, 1), "seeds per hill"], b: [cm(P.hill_extent), "cluster width"], test: "test_plants_serpentine_rows_of_tidy_hills" },
    { t: "Water on target", d: "A measured dose per hill, pumped while standing still over the seed tube. The tank covers 17 doses; after that the robot keeps seeding and flags the tank.",
      a: [`${Math.round(P.water_frac * 100)} %`, "within 5 cm of a hill"], b: [`${Math.round(P.pump_dry_s)} s`, "pump running dry"], test: "test_tank_runs_low_and_is_refilled" },
    { t: "A full run", d: "Three serpentine rows on the 2.4 × 1.2 m bed, from pressing Plant to DONE.",
      a: [`${fmt(P.hills)}`, "hills per run"], b: [`${fmt(P.t_s - 2)} s`, `${fmt(P.dist, 1)} m driven`], test: "test_plants_serpentine_rows_of_tidy_hills" },
    { t: "Stops at any edge", d: "Two readings below the learned ground brake the tracks. How far past the edge the sonar got, three runs per bed:",
      a: [edgeRow("table-top, 75 cm drop"), "table-top"], b: [`${edgeRow("raised bed, 12 cm drop")} / ${edgeRow("shallow step, 6 cm")}`, "12 cm bed / 6 cm step"], test: "test_stops_at_every_kind_of_edge" },
    { t: "Sensor faults stop it", d: "With the echo wire open, calibration finds no echoes and the robot reports a sonar fault instead of moving.",
      a: [E["echo wire open"]?.state || "FAULT", "state"], b: [cm(E["echo wire open"]?.moved_m || 0), "distance moved"], test: "test_open_echo_wire_is_a_fault_not_a_drive" },
    { t: "Battery-proof spacing", d: "The UNO reads the pack through a divider on A3 and stretches drive times as it sags.",
      a: [cm(B["full (12.5 V)"].spacing_mean), "full pack, 12.5 V"], b: [cm(B["tired (11.2 V)"].spacing_mean), "tired pack, 11.2 V"], test: "test_battery_compensation_keeps_spacing" },
    { t: "STOP is immediate", d: "STOP jumps the NodeMCU's queue and goes out on the wire at once; the UNO brakes on the next loop.",
      a: [`${C.stop_ms} ms`, "tap to tracks stopped"], b: [`${fmt(H["/api/status"].mean_ms, 1)} ms`, "status request"], test: "test_phone_control_is_safe" },
    { t: "Phone goes quiet, robot stops", d: "Every drive command carries a hold time (450 ms from the page). If the phone leaves the Wi-Fi mid-drive, the tracks stop by themselves.",
      a: [cm(C.phone_lost_moved_m), "moved after the last request"], b: [cm(C.edge_guard_past_edge_m), "past the edge when driven at it"], test: "test_phone_control_is_safe" },
    { t: "One tap, one action", d: "Seed dispenses one hill; Water gives one timed squirt. Queued taps are carried out in order, none twice.",
      a: [`${C.seed_tap_seeds}`, "seeds from one Seed tap"], b: [`${fmt(C.water_tap_mL)} mL`, "from one 1.5 s Water tap"], test: "test_phone_control_is_safe" },
    { t: "A clean link", d: `Over ${L.runs} full runs: bytes lost when both boards talk at once are caught by the CRC and resent; no command went undelivered.`,
      a: [`${L.lost}`, "commands lost"], b: [`${L.uno_frames_ok}`, "frames received by the UNO"], test: "test_link_is_clean_over_a_full_run" },
    { t: "Never blocks", d: "The UNO's loop is bounded by one echo wait (6 ms) and one telemetry line (~23 ms at 19200 baud), well inside its 1 s watchdog.",
      a: [`${fmt(L.uno_loop_max_ms, 1)} ms`, "longest UNO loop"], b: [`${fmt(H["/"].mean_ms, 1)} ms`, "phone page served"], test: "test_link_is_clean_over_a_full_run" },
  ];
  const box = $("#defects");
  box.innerHTML = "";
  for (const c of cards) {
    const el = document.createElement("article");
    el.className = "defect";
    el.innerHTML = `<h3></h3><p class="note"></p><div class="nums"><div class="b"><b></b><span class="note"></span></div><div class="b"><b></b><span class="note"></span></div></div><code></code>`;
    el.querySelector("h3").textContent = c.t;
    el.querySelector("p").textContent = c.d;
    const [x, y] = el.querySelectorAll(".nums > div");
    x.querySelector("b").textContent = c.a[0]; x.querySelector(".note").textContent = c.a[1];
    y.querySelector("b").textContent = c.b[0]; y.querySelector(".note").textContent = c.b[1];
    el.querySelector("code").textContent = c.test;
    box.appendChild(el);
  }
}

// ---------------------------------------------------------------- hero
async function hero() {
  const host = $("#heroViewer");
  try {
    const { heroViewer } = await import("./robot3d.js");
    const hv = await heroViewer(host, { onRotate: (on) => $("#rotateBtn").classList.toggle("on", on) });
    $("#rotateBtn").classList.toggle("on", !REDUCED);
    $("#rotateBtn").onclick = () => { const on = !$("#rotateBtn").classList.contains("on"); $("#rotateBtn").classList.toggle("on", on); hv.setRotate(on); };
    $("#cycleBtn").onclick = () => { const on = hv.toggleCycle(); $("#cycleBtn").classList.toggle("on", on); $("#cycleBtn").textContent = on ? "Stop the cycle" : "Play a planting cycle"; };
    $("#explode").oninput = (e) => hv.setExplode(+e.target.value);
  } catch (e) {
    console.error(e);
    host.querySelector(".loading").textContent = "3D needs WebGL, which this browser has turned off.";
  }
}

// ---------------------------------------------------------------- signal path
function flow() {
  const svg = $("#flowSvg");
  const stages = [
    { k: "phone", t: "Phone", s: "the page", d: "The NodeMCU serves its own control page. Holding an arrow repeats a drive request every 200 ms with a 450 ms hold, so letting go, or losing the Wi-Fi, stops the robot within half a second. Status is polled every 400 ms." },
    { k: "wifi", t: "Wi-Fi", s: "HTTP, soft AP", d: "The NodeMCU is its own access point, FARM-E, at 192.168.4.1, with the password kept in an untracked secrets.h. A small JSON API: /api/status and /api/cmd." },
    { k: "esp", t: "NodeMCU", s: "ESP8266", d: "Never blocks: requests are answered from cached state in a few milliseconds. Commands are queued and sent one at a time with a sequence number, resent after 150 ms until the UNO acknowledges; a heartbeat carries the DHT11's temperature every 300 ms." },
    { k: "uart", t: "UART", s: "2 wires", d: "19200 baud, 8N1. One text line per frame, \"$C,seq,code,a,b*CRC\", checked with CRC-8. A telemetry line comes back within 20 ms of each command. A 1k/2k divider brings the UNO's 5 V TX down to the ESP8266's 3.3 V." },
    { k: "uno", t: "UNO", s: "ATmega328P", d: "A state machine ticks every loop. The sonar is read every 30 ms; drive commands carry hold times; settings live in EEPROM with a CRC; a 1 s watchdog restarts the board if anything ever hangs." },
    { k: "act", t: "Actuators", s: "tracks, flap, pump", d: "The tracks stop before the flap opens; the pump gives a measured dose; both H-bridge inputs HIGH brake a track dead at an edge. Tank level and battery voltage are tracked in firmware." },
  ];
  const W = 1100, NW = 132, x0 = 20, gap = (W - 40 - NW) / 5;
  let html = `<defs><marker id="arr" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto"><path d="M0,0 L10,5 L0,10 z" fill="var(--muted)"/></marker></defs>`;
  stages.forEach((s, i) => {
    const x = x0 + i * gap;
    if (i < stages.length - 1) html += `<line x1="${x + NW}" y1="85" x2="${x + gap - 6}" y2="85" stroke="var(--muted)" stroke-width="2" marker-end="url(#arr)"/><circle class="pkt" r="5" cy="85" fill="var(--yellow)" stroke="#8a7700" data-i="${i}" cx="${x + NW}"/>`;
    html += `<g class="fnode" data-k="${s.k}" tabindex="0" role="button" aria-label="${s.t}" style="cursor:pointer"><rect x="${x}" y="45" width="${NW}" height="80" rx="10" fill="var(--surface)" stroke="var(--line)" stroke-width="2"/><text x="${x + NW / 2}" y="80" text-anchor="middle" style="font:800 20px var(--display);fill:var(--ink);text-transform:uppercase">${s.t}</text><text x="${x + NW / 2}" y="103" text-anchor="middle" style="font:600 12px var(--body);fill:var(--muted)">${s.s}</text></g>`;
  });
  html += `<text x="20" y="25" style="font:700 12px var(--body);fill:var(--muted);letter-spacing:.12em">CLICK A STAGE</text>`;
  svg.innerHTML = html;
  const pick = (k) => {
    const s = stages.find((x) => x.k === k);
    $$(".fnode rect", svg).forEach((r) => { r.setAttribute("stroke", "var(--line)"); r.setAttribute("fill", "var(--surface)"); });
    const g = svg.querySelector(`.fnode[data-k="${k}"] rect`);
    g.setAttribute("stroke", "#c9ad00"); g.setAttribute("fill", "color-mix(in srgb, var(--yellow) 30%, var(--surface))");
    $("#flowTitle").textContent = s.t; $("#flowText").textContent = s.d;
  };
  $$(".fnode", svg).forEach((g) => { g.addEventListener("click", () => pick(g.dataset.k)); g.addEventListener("keydown", (e) => { if (e.key === "Enter" || e.key === " ") { e.preventDefault(); pick(g.dataset.k); } }); });
  pick("uart");
  if (!REDUCED) {
    const pk = $$(".pkt", svg);
    let t = 0;
    const loop = () => {
      t += 0.012;
      pk.forEach((p) => { const i = +p.dataset.i, xs = x0 + i * gap + NW, xe = x0 + (i + 1) * gap - 6; const f = ((t - i * 0.18) % 1 + 1) % 1; p.setAttribute("cx", xs + (xe - xs) * f); p.setAttribute("opacity", f < 0.95 ? 1 : 0); });
      requestAnimationFrame(loop);
    };
    loop();
  }
}

// ---------------------------------------------------------------- simulation world (shared)
let worldPromise = null;
const getWorld = () => (worldPromise ||= SimWorld.create());

// ---------------------------------------------------------------- field trial
const PRESETS = {
  table: { params: {}, note: "Table-top bed: the ground falls 75 cm past every edge." },
  raised: { params: { drop: 0.12 }, note: "A 12 cm raised bed." },
  step: { params: { drop: 0.06 }, note: "A 6 cm step down at the end of the bed." },
  unplugged: { params: {}, echo: 1, note: "The sonar's echo wire is open: calibration finds no echoes and reports a fault." },
  tired: { params: { soc0: 0.3 }, note: "Battery at about 11.2 V: drive times stretch to keep the spacing." },
};
const MAX_T = 180;

async function trial() {
  const stageEl = $("#stage");
  let fs;
  try {
    const { fieldScene } = await import("./robot3d.js");
    fs = await fieldScene(stageEl, { count: 1 });
  } catch (e) {
    console.error(e);
    stageEl.insertAdjacentHTML("beforeend", `<p class="note" style="position:absolute;inset:auto 12px 12px 12px;color:#fff">3D is unavailable here (WebGL is off); the numbers below still run.</p>`);
  }
  const world = await getWorld();
  const rig = world.rig(0);
  const panel = $("#rig0");
  const logs = ["", ""];
  let shown = 0, speed = 4, paused = false;

  function start() {
    const p = PRESETS[$("#preset").value];
    rig.init({ ...p.params }, 1);
    if (p.echo) rig.set("echo_mode", p.echo);
    const set = (name, v) => rig.send(`/api/cmd?c=set&p=${name}&v=${v}`, 600);
    set("spacing", $("#setSpacing").value);
    set("rows", $("#setRows").value);
    set("water", $("#setWater").value);
    rig.send("/api/cmd?c=auto", 2000);
    logs[0] = ""; logs[1] = "";
    fs?.setup(0, { L: rig.get("bed_len"), W: rig.get("bed_w"), drop: rig.get("drop") });
    $("#presetNote").textContent = `${p.note} "Plant" pressed at 2 s.`;
    paused = false; $("#pause").textContent = "Pause";
  }
  ["#preset", "#setSpacing", "#setRows", "#setWater"].forEach((id) => { $(id).onchange = start; });
  $("#restart").onclick = start;
  $("#pause").onclick = () => { paused = !paused; $("#pause").textContent = paused ? "Resume" : "Pause"; };
  $$("#speed button").forEach((b) => b.onclick = () => { speed = +b.dataset.s; $$("#speed button").forEach((x) => x.classList.toggle("on", x === b)); });
  $$("#cam button").forEach((b) => b.onclick = () => { fs?.setCamera(b.dataset.c); $$("#cam button").forEach((x) => x.classList.toggle("on", x === b)); });
  $$(".console-tabs button", panel).forEach((b) => b.onclick = () => { shown = +b.dataset.b; $$(".console-tabs button", panel).forEach((x) => x.classList.toggle("on", x === b)); });
  start();
  fs?.setCamera("overview");
  window.__farmeTrial = { rig };

  let last = performance.now(), uiT = 0;
  const loop = () => {
    const now = performance.now();
    const frameDt = Math.min(0.05, Math.max(0, (now - last) / 1000));
    last = now;
    try { frame(frameDt); } catch (e) { console.error("trial frame", e); }
    requestAnimationFrame(loop);
  };
  function frame(frameDt) {
    if (!onScreen(stageEl, 200)) return;
    const done = rig.state() === "DONE" || rig.state() === "FAULT";
    const run = !paused && rig.t < MAX_T && !(done && rig.t > 4);
    const ms = run ? Math.min(frameDt * speed * 1000, 400) : 0;
    if (ms > 0) {
      rig.run(ms);
      logs[0] = (logs[0] + rig.log(UNO)).slice(-4000);
      logs[1] = (logs[1] + rig.log(ESP)).slice(-4000);
    }
    if (fs) {
      const s = ms / 1000;
      fs.update(0, {
        x: rig.get("x"), y: rig.get("y"), th: rig.get("th"), fell: rig.get("fell") > 0, gate: rig.get("servo_deg"), pump: rig.get("pump") > 0, tank: rig.get("tank_mL"),
        dl: rig.get("vl") * s, dr: rig.get("vr") * s, frameDt: paused ? 0 : frameDt, trail: rig.trail(), wet: rig.wet(), seeds: rig.seeds(),
      });
      fs.frame();
    }
    uiT += frameDt;
    if (uiT > 0.15) { uiT = 0; panelUpdate(); }
  }
  requestAnimationFrame(loop);

  function panelUpdate() {
    const st = rig.state();
    $("#clock").textContent = `t = ${rig.t.toFixed(1)} s${st === "DONE" ? " · run complete" : ""}${paused ? " · paused" : ""} · ${speed}× speed`;
    const set = (f, v, alert) => { const el = panel.querySelector(`[data-f="${f}"]`); el.textContent = v; el.parentElement.classList.toggle("alert", !!alert); };
    panel.querySelector('[data-f="state"]').textContent = st;
    const hills = rig.get("hills");
    set("hills", fmt(hills));
    set("row", fmt(rig.probe(UNO, "row") + 1));
    set("sph", hills ? fmt(rig.get("seeds_per_hill"), 1) : "–");
    set("spacing", hills > 1 ? `${fmt(rig.get("spacing_mean") * 100, 1)} cm` : "–");
    set("wf", rig.get("water_used") > 0.5 ? `${fmt(rig.get("water_frac") * 100)}%` : "–");
    set("tank", `${fmt(rig.probe(UNO, "tank_mL"))} mL`, rig.probe(UNO, "tank_mL") < 20);
    const g = rig.probe(UNO, "dist_mm");
    set("ground", g < 0 ? "no echo" : `${fmt(g)} mm`, st === "FAULT");
    set("batt", `${fmt(rig.probe(UNO, "batt_mV") / 1000, 1)} V`);
    const lines = logs[shown].replace(/\r/g, "").split("\n");
    panel.querySelector(".console").textContent = lines.slice(-10).join("\n") || "(nothing yet)";
    smHighlight(st);
  }
}

// ---------------------------------------------------------------- phone
async function phone() {
  const world = await getWorld();
  const r2 = world.rig(1);
  const params = { start_x: 1.2, start_y: 0 };
  const frameEl = $("#phoneFrame");
  const SHIM = `<script>(function(){var P={},n=0;window.fetch=function(u){return new Promise(function(res){var id=++n;P[id]=res;parent.postMessage({farme:1,id:id,url:String(u)},"*");});};addEventListener("message",function(e){var d=e.data;if(!d||!d.farmeReply)return;var r=P[d.id];if(!r)return;delete P[d.id];r({ok:d.status>=200&&d.status<300,status:d.status,json:function(){return Promise.resolve(JSON.parse(d.body));},text:function(){return Promise.resolve(d.body);}});});})();<\/script>`;
  const trace = (html, cls) => {
    const el = $("#traceBot");
    const d = document.createElement("div");
    if (cls) d.className = cls;
    d.innerHTML = html;
    el.prepend(d);
    while (el.children.length > 40) el.lastChild.remove();
  };
  function reset() {
    r2.init(params, 3);
    $("#traceBot").innerHTML = "<div>Hold the arrows to drive. Try \"Plant\".</div>";
    frameEl.srcdoc = r2.uiHtml().replace("<head>", "<head>" + SHIM);
  }
  $("#resetBot").onclick = reset;
  window.addEventListener("message", (e) => {
    const d = e.data;
    if (!d || !d.farme || e.source !== frameEl.contentWindow) return;
    const url = d.url.replace(/^https?:\/\/[^/]+/, "");
    const quiet = url.startsWith("/api/status");
    r2.http(url).then((res) => {
      frameEl.contentWindow?.postMessage({ farmeReply: 1, id: d.id, status: res.status, body: res.body }, "*");
      if (!quiet) trace(`GET ${url.replace("/api/cmd?", "cmd ")} → ${res.status} ${res.body.slice(0, 40)} <span class="note">${Math.round(res.ms)} ms</span>`, res.status === 200 ? "ok" : "bad");
    });
  });
  reset();
  const phoneSec = $("#phone"), map = $("#mapBot");
  let last = performance.now();
  const loop = () => {
    const now = performance.now();
    const dt = Math.min(0.1, Math.max(0, (now - last) / 1000));
    last = now;
    if (onScreen(phoneSec, 100)) {
      r2.run(dt * 1000);
      drawMap(map, r2);
    }
    requestAnimationFrame(loop);
  };
  requestAnimationFrame(loop);
}

function drawMap(cv, r) {
  const g = cv.getContext("2d");
  const W = cv.width, H = cv.height;
  const L = r.get("bed_len"), BW = r.get("bed_w");
  const pad = 14, s = Math.min((W - 2 * pad) / L, (H - 2 * pad) / BW);
  const ox = (W - L * s) / 2, oy = (H - BW * s) / 2;
  const P = (x, y) => [ox + x * s, oy + (BW / 2 - y) * s];
  g.clearRect(0, 0, W, H);
  g.fillStyle = "#9b6b40"; g.fillRect(ox, oy, L * s, BW * s);
  const wet = r.wet();
  g.fillStyle = "rgba(47,111,181,0.22)";
  for (let k = 0; k < wet.length; k += 4) { const [x, y] = P(wet[k], wet[k + 1]); g.beginPath(); g.arc(x, y, 4 + wet[k + 2], 0, 7); g.fill(); }
  const tr = r.trail();
  g.strokeStyle = "rgba(20,20,10,0.55)"; g.lineWidth = 1.5; g.beginPath();
  for (let k = 0; k < tr.length; k += 6) { const [x, y] = P(tr[k + 1], tr[k + 2]); k ? g.lineTo(x, y) : g.moveTo(x, y); }
  g.stroke();
  const sd = r.seeds();
  g.fillStyle = "#1b0f06";
  for (let k = 0; k < sd.length; k += 3) { const [x, y] = P(sd[k], sd[k + 1]); g.fillRect(x - 1.5, y - 1.5, 3, 3); }
  const [rx, ry] = P(r.get("x"), r.get("y"));
  g.save(); g.translate(rx, ry); g.rotate(-r.get("th"));
  g.fillStyle = "#ffde00"; g.strokeStyle = "#1a1a0a"; g.lineWidth = 2;
  g.fillRect(-0.13 * s, -0.1 * s, 0.26 * s, 0.2 * s); g.strokeRect(-0.13 * s, -0.1 * s, 0.26 * s, 0.2 * s);
  g.beginPath(); g.moveTo(0.13 * s, 0); g.lineTo(0.2 * s, 0); g.stroke();
  g.restore();
  g.fillStyle = "rgba(255,255,255,0.95)"; g.font = "600 22px Barlow, sans-serif";
  g.fillText(`${r.state()} · hills ${r.get("hills")} · water ${r.get("water_used").toFixed(0)} mL`, 22, H - 22);
}

// ---------------------------------------------------------------- sonar lab
function sonar() {
  const cv = $("#sonarCanvas"), g = cv.getContext("2d");
  const st = { pos: -0.2, T: 28, drop: 0.75, wire: "ok", t0: performance.now() };
  const C = (T) => 331.3 + 0.606 * T;
  const SENSOR_H = 0.055, BASE = 55, EDGE = 40;
  function compute() {
    const past = st.pos > 0;
    const d = SENSOR_H + (past ? st.drop : 0);
    const us = 2 * d / C(st.T) * 1e6;
    if (st.wire === "open") return { echo: "none", mm: null, say: "No echo: sonar fault", why: "Three missing echoes in a row stop any motion. Standing still with no echo at all, Plant refuses to start and the status shows fault 2 until the wire is fixed and the fault cleared." };
    if (st.wire === "absorb") return { echo: "> 6 ms", mm: null, say: "No echo: stop", why: "Soft, sloping soil can swallow a ping now and then; one miss is ignored. pulseIn() gives up at 6 ms (about 1 m), so a missing echo is never mistaken for a distance, and three in a row count as no ground." };
    const mm = Math.round(us * C(st.T) / 2 / 1000);
    const edge = mm > BASE + EDGE;
    return {
      echo: `${us.toFixed(0)} µs`, mm,
      say: edge ? `${mm} mm: edge, brake` : `${mm} mm: ground, keep going`,
      why: edge ? `More than ${EDGE} mm below the learned ${BASE} mm. Two such pings, 60 ms apart, and both tracks brake; then the robot backs off 8 cm and turns for the next row.`
        : `Within ${EDGE} mm of the ${BASE} mm learned while standing still, so the robot keeps driving. The temperature feeds the speed of sound, so the reading stays true from 0 to 45 °C.`,
    };
  }
  function ui() {
    const c = compute();
    $("#sEcho").textContent = c.echo;
    $("#sDist").textContent = c.mm == null ? "no echo" : `${c.mm} mm`;
    $("#vDec").textContent = c.say; $("#vDecB").textContent = c.why;
    $("#sPosV").textContent = `${Math.round(st.pos * 100) > 0 ? "+" : ""}${Math.round(st.pos * 100)} cm`;
    $("#sTempV").textContent = `${st.T} °C`;
  }
  function draw() {
    const W = cv.width, H = cv.height;
    const ink = css("--ink"), muted = css("--muted"), yellow = "#ffde00";
    g.clearRect(0, 0, W, H);
    const s = W / 0.95, X = (x) => (x + 0.6) * s, groundY = 150, Y = (y) => groundY - y * s;
    const floorY = st.drop > 0.3 ? H - 10 : groundY + st.drop * s;
    g.fillStyle = "#9b6b40"; g.fillRect(0, groundY, X(0), 16);
    if (st.drop > 0.3) {
      g.fillStyle = css("--surface-2"); g.fillRect(0, groundY + 16, X(0), 16);
      g.fillStyle = muted; g.fillRect(X(0) - 26, groundY + 32, 12, H - groundY - 32);
      g.fillStyle = css("--line"); g.fillRect(X(0), H - 10, W - X(0), 10);
      g.fillStyle = muted; g.font = "600 12px Barlow, sans-serif"; g.fillText("floor, 75 cm below", X(0) + 10, H - 16);
    } else {
      g.fillStyle = "#6f4a2a"; g.fillRect(0, groundY + 16, X(0), Math.max(0, floorY - groundY - 16));
      g.fillStyle = "#77895a"; g.fillRect(0, floorY, W, H);
      g.fillStyle = muted; g.font = "600 12px Barlow, sans-serif"; g.fillText(`ground, ${Math.round(st.drop * 100)} cm below`, X(0) + 10, floorY - 6);
    }
    const cx = st.pos - 0.13;
    const tipped = cx > 0;
    g.save();
    if (tipped) { g.translate(X(0), groundY); g.rotate(0.5); g.translate(-X(0), -groundY); }
    g.fillStyle = "#1b1b1b";
    const tl = X(cx - 0.11), tr = X(cx + 0.11);
    g.beginPath(); g.roundRect(tl, Y(0.068), tr - tl, 0.068 * s, 0.034 * s); g.fill();
    g.fillStyle = "#333"; for (let k = 0; k < 18; k++) g.fillRect(tl + 6 + k * (tr - tl - 12) / 18, Y(0.066), 3, 0.064 * s);
    g.fillStyle = "#101010"; g.fillRect(X(cx - 0.125), Y(0.082), 0.25 * s, 8);
    g.fillStyle = "#fb6b0d"; g.beginPath(); g.moveTo(X(cx - 0.04), Y(0.13)); g.lineTo(X(cx + 0.02), Y(0.13)); g.lineTo(X(cx + 0.005), Y(0.088)); g.lineTo(X(cx - 0.025), Y(0.088)); g.fill();
    g.fillStyle = "#eeeeea"; g.fillRect(X(cx - 0.105), Y(0.15), 0.06 * s, 0.068 * s);
    g.fillStyle = "#0a0f30"; g.fillRect(X(cx - 0.11), Y(0.158), 0.07 * s, 6);
    g.fillStyle = "#c1121f"; g.fillRect(X(cx + 0.01), Y(0.108), 0.1 * s, 0.022 * s);
    g.fillStyle = "#9aa0a6"; g.fillRect(X(cx + 0.11), Y(0.08), X(st.pos) - X(cx + 0.11), 4);
    g.fillStyle = "#1f5aa6"; g.fillRect(X(st.pos) - 16, Y(SENSOR_H) - 4, 32, 6);
    g.fillStyle = "#c9cdd2"; g.fillRect(X(st.pos) - 13, Y(SENSOR_H) + 2, 10, 6); g.fillRect(X(st.pos) + 3, Y(SENSOR_H) + 2, 10, 6);
    g.restore();
    if (!tipped && st.wire !== "open") {
      const yEnd = st.pos > 0 ? floorY : groundY;
      const y0 = Y(SENSOR_H) + 8;
      const grd = g.createLinearGradient(0, y0, 0, yEnd);
      grd.addColorStop(0, "rgba(255,222,0,0.45)"); grd.addColorStop(1, "rgba(255,222,0,0.05)");
      const spread = Math.tan(7.5 * Math.PI / 180) * (yEnd - y0);
      g.fillStyle = grd; g.beginPath(); g.moveTo(X(st.pos) - 4, y0); g.lineTo(X(st.pos) + 4, y0); g.lineTo(X(st.pos) + spread, yEnd); g.lineTo(X(st.pos) - spread, yEnd); g.fill();
      if (!REDUCED) {
        const ph = ((performance.now() - st.t0) / 900) % 1;
        const f = ph < 0.5 ? ph * 2 : 2 - ph * 2;
        const y = y0 + (yEnd - y0) * f;
        g.strokeStyle = yellow; g.lineWidth = 3; g.beginPath(); g.moveTo(X(st.pos) - 10 - f * spread * 0.6, y); g.lineTo(X(st.pos) + 10 + f * spread * 0.6, y); g.stroke();
      }
    }
    g.strokeStyle = ink; g.setLineDash([4, 4]); g.beginPath(); g.moveTo(X(0), 20); g.lineTo(X(0), groundY); g.stroke(); g.setLineDash([]);
    g.fillStyle = ink; g.font = "700 12px Barlow, sans-serif"; g.fillText("END OF BED", X(0) + 6, 30);
    if (tipped) { g.fillStyle = css("--bad"); g.font = "800 20px Barlow, sans-serif"; g.fillText("In a real run the robot brakes long before this", 20, 40); }
  }
  const bindSeg = (id, key, conv) => $$(`#${id} button`).forEach((b) => b.onclick = () => { st[key] = conv(b.dataset); $$(`#${id} button`).forEach((x) => x.classList.toggle("on", x === b)); ui(); draw(); });
  bindSeg("sBed", "drop", (d) => +d.d);
  bindSeg("sWire", "wire", (d) => d.w);
  $("#sPos").oninput = (e) => { st.pos = e.target.value / 100; ui(); draw(); };
  $("#sTemp").oninput = (e) => { st.T = +e.target.value; ui(); draw(); };
  ui(); draw();
  labRedrawers.push(draw);
  const loop = () => { if (onScreen(cv, 0) && !REDUCED) draw(); requestAnimationFrame(loop); };
  requestAnimationFrame(loop);
}

// ---------------------------------------------------------------- UART lab
function crc8(bytes) {
  let c = 0;
  for (const b of bytes) { c ^= b; for (let i = 0; i < 8; i++) c = c & 0x80 ? ((c << 1) ^ 0x07) & 0xff : (c << 1) & 0xff; }
  return c;
}
function frameText(type, fields) {
  const body = type + fields.map((f) => "," + f).join("");
  const c = crc8([...body].map((ch) => ch.charCodeAt(0)));
  return "$" + body + "*" + c.toString(16).toUpperCase().padStart(2, "0") + "\n";
}
function uart() {
  const cv = $("#uartCanvas"), g = cv.getContext("2d");
  const msgs = {
    cmd: { text: frameText("C", [17, 4, 3, 450]), info: "Command 17: drive (4), left (3), for 450 ms. 0.52 ms per byte at 19200 baud, CRC-8 at the end." },
    tel: { text: frameText("T", [17, 1, 1, 8, 54, 0, 0, 350, 123, 0]), info: "The UNO's reply: ack 17, state MANUAL, mode MANUAL, flags, ground 54 mm, 0 hills, row 0, 350 mL, 12.3 V, no fault." },
    hb: { text: frameText("H", [280, 62]), info: "Heartbeat every 300 ms: 28.0 °C and 62 %RH from the DHT11, used by the UNO for the speed of sound." },
  };
  const BAUD = 19200;
  let cur = "cmd", hover = -1;
  const show = (ch) => (ch === "\n" ? "\\n" : ch === "\r" ? "\\r" : ch);
  function draw() {
    const m = msgs[cur];
    const W = cv.width, H = cv.height;
    const bytes = [...m.text].map((c) => c.charCodeAt(0));
    const bits = bytes.length * 10;
    const x0 = 70, x1 = W - 20, bw = (x1 - x0) / bits;
    const hi = 60, lo = 130;
    const ink = css("--ink"), muted = css("--muted"), line = css("--line"), accent = css("--green");
    g.clearRect(0, 0, W, H);
    g.font = "600 12px JetBrains Mono, monospace";
    g.fillStyle = muted; g.fillText("HIGH", 6, hi + 4); g.fillText("LOW", 12, lo + 4);
    let x = x0, lvl = 1;
    g.lineWidth = 2;
    const pts = [[x0 - 30, hi]];
    bytes.forEach((b, i) => {
      const seq = [0]; for (let k = 0; k < 8; k++) seq.push((b >> k) & 1); seq.push(1);
      if (i === hover) { g.fillStyle = "rgba(255,222,0,0.25)"; g.fillRect(x, 30, bw * 10, 120); }
      seq.forEach((bit) => {
        const y = bit ? hi : lo;
        pts.push([x, lvl ? hi : lo], [x, y]);
        lvl = bit;
        x += bw;
        pts.push([x, y]);
      });
      if (bw * 10 > 14) { g.fillStyle = accent; g.fillText(show(String.fromCharCode(b)), x - bw * 5 - 4, 22); }
      g.strokeStyle = line; g.beginPath(); g.moveTo(x, 30); g.lineTo(x, 150); g.stroke();
    });
    g.strokeStyle = ink; g.beginPath(); pts.forEach(([px, py], k) => (k ? g.lineTo(px, py) : g.moveTo(px, py))); g.stroke();
    const ms = bits / BAUD * 1000;
    g.fillStyle = muted; g.fillText(`${bytes.length} bytes · ${bits} bits · ${ms.toFixed(1)} ms at ${BAUD} baud · each byte: start bit, 8 data bits LSB first, stop bit`, x0, H - 10);
    $("#uInfo").textContent = m.info;
    const bx = $("#uBytes");
    bx.innerHTML = "";
    bytes.forEach((b, i) => {
      const d = document.createElement("div");
      d.className = "byte" + (i === hover ? " hl" : "");
      d.innerHTML = `<b></b>0x${b.toString(16).toUpperCase().padStart(2, "0")}`;
      d.querySelector("b").textContent = show(String.fromCharCode(b));
      d.onmouseenter = () => { hover = i; draw(); };
      bx.appendChild(d);
    });
  }
  $$("#uMsg button").forEach((b) => b.onclick = () => { cur = b.dataset.m; hover = -1; $$("#uMsg button").forEach((x) => x.classList.toggle("on", x === b)); draw(); });
  cv.addEventListener("mousemove", (e) => {
    const rect = cv.getBoundingClientRect();
    const px = (e.clientX - rect.left) * cv.width / rect.width;
    const n = msgs[cur].text.length;
    const i = Math.floor((px - 70) / ((cv.width - 90) / n));
    if (i !== hover && i >= 0 && i < n) { hover = i; draw(); }
  });
  draw();
  labRedrawers.push(draw);

  // stop-and-wait delivery over a noisy line (same rules as link_core)
  const pc = $("#pfCanvas"), pg = pc.getContext("2d");
  let seed = 7;
  const rnd = () => ((seed = (seed * 1103515245 + 12345) % 2147483648) / 2147483648);
  function deliver() {
    const p = +$("#noise").value / 100;
    $("#noiseV").textContent = `${$("#noise").value} %`;
    const byteMs = 10 / BAUD * 1000;
    const cmdLen = frameText("C", [17, 4, 3, 450]).length, telLen = frameText("T", [17, 1, 1, 8, 54, 0, 0, 350, 123, 0]).length;
    const ok = (n) => { for (let i = 0; i < n; i++) if (rnd() < p) return false; return true; };
    const ev = [];
    let t = 0, sent = 0, retries = 0, lost = 0;
    for (let c = 0; c < 12; c++) {
      let tries = 0, done = false;
      while (!done && tries < 8) {
        tries++; sent++;
        const txEnd = t + cmdLen * byteMs;
        const arrived = ok(cmdLen);
        ev.push({ c, t0: t, t1: txEnd, kind: arrived ? "cmd" : "cmdlost", retry: tries > 1 });
        if (arrived) {
          const a0 = txEnd + 20, a1 = a0 + telLen * byteMs;
          const ackOk = ok(telLen);
          ev.push({ c, t0: a0, t1: a1, kind: ackOk ? "ack" : "acklost" });
          if (ackOk) { done = true; t = a1 + 5; break; }
        }
        t += 150;  // no acknowledgement: resend after the retry interval
      }
      if (!done) lost++;
      retries += tries - 1;
    }
    const T = Math.max(t, 400);
    const W = pc.width, H = pc.height, x0 = 90, s = (W - 110) / T;
    const ink = css("--ink"), muted = css("--muted"), green = css("--green"), bad = css("--bad");
    pg.clearRect(0, 0, W, H);
    pg.font = "600 12px Barlow, sans-serif";
    pg.fillStyle = muted; pg.fillText("NodeMCU → UNO", 4, 64); pg.fillText("UNO → NodeMCU", 4, 134);
    for (const e of ev) {
      const x = x0 + e.t0 * s, w = Math.max(3, (e.t1 - e.t0) * s);
      const y = e.kind.startsWith("cmd") ? 50 : 120;
      pg.fillStyle = e.kind.endsWith("lost") ? bad : (e.kind === "ack" ? "#c9ad00" : green);
      pg.fillRect(x, y, w, 22);
      if (e.kind.endsWith("lost")) { pg.strokeStyle = bad; pg.beginPath(); pg.moveTo(x, y); pg.lineTo(x + w, y + 22); pg.stroke(); }
    }
    pg.fillStyle = ink;
    for (let t0 = 0; t0 <= T; t0 += 100) pg.fillText(`${t0} ms`, x0 + t0 * s - 12, H - 8);
    pg.fillStyle = muted; pg.fillText("green: command · yellow: telemetry acknowledging it · red: corrupted, dropped by the CRC", x0, 30);
    $("#pfNote").textContent = `12 commands, ${sent} frames sent, ${retries} resends, ${lost} undelivered, finished in ${Math.round(t)} ms. The UNO acted on each command exactly once: a resend it has already seen only refreshes the acknowledgement.`;
  }
  $("#noise").oninput = () => { seed = 7; deliver(); };
  $("#resend").onclick = () => { seed = (seed * 31 + 17) % 2147483648; deliver(); };
  deliver();
  labRedrawers.push(deliver);
}

// ---------------------------------------------------------------- state machine
const SM_X = (i) => 20 + i * 178;
const SM_NODES = {
  IDLE: [SM_X(0), 70], MANUAL: [SM_X(0), 180], CALIBRATE: [SM_X(1), 70], SETTLE: [SM_X(2), 70], SEED: [SM_X(3), 70], WATER: [SM_X(4), 70], DRIVE: [SM_X(5), 70],
  BACKOFF: [SM_X(5), 180], TURN1: [SM_X(4), 180], SHIFT: [SM_X(3), 180], TURN2: [SM_X(2), 180], DONE: [SM_X(5), 285], PAUSED: [SM_X(0), 285], FAULT: [SM_X(1), 285],
};
const SM_EDGES = [
  ["IDLE", "CALIBRATE", "Plant"], ["CALIBRATE", "SETTLE", "ground"], ["SETTLE", "SEED", "250 ms"], ["SEED", "WATER", "shut"],
  ["WATER", "DRIVE", "dosed"], ["BACKOFF", "TURN1", "rows left"], ["TURN1", "SHIFT", "90°"], ["SHIFT", "TURN2", "row gap"],
  ["BACKOFF", "DONE", "last row"], ["IDLE", "MANUAL", "arrows"], ["CALIBRATE", "FAULT", "no echo"],
];
function stateMachine() {
  const svg = $("#smSvg");
  const w = 120, h = 40;
  let html = `<defs><marker id="sma" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto"><path d="M0,0 L10,5 L0,10 z" fill="var(--muted)"/></marker></defs>`;
  const anchor = (a, b) => {
    const [ax, ay] = SM_NODES[a], [bx, by] = SM_NODES[b];
    const cx = ax + w / 2, cy = ay + h / 2, dx = bx - ax, dy = by - ay;
    if (Math.abs(dx) >= Math.abs(dy)) return [[cx + Math.sign(dx) * w / 2, cy], [bx + w / 2 - Math.sign(dx) * w / 2, by + h / 2]];
    return [[cx, cy + Math.sign(dy) * h / 2], [bx + w / 2, by + h / 2 - Math.sign(dy) * h / 2]];
  };
  for (const [a, b, lab] of SM_EDGES) {
    const [[x1, y1], [x2, y2]] = anchor(a, b);
    html += `<line class="edge" x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}" marker-end="url(#sma)"/>`;
    const vertical = Math.abs(x1 - x2) < 1;
    const lx = vertical ? x1 + 8 : (x1 + x2) / 2, ly = vertical ? (y1 + y2) / 2 + 4 : (y1 + y2) / 2 - 6;
    html += `<text class="elabel" x="${lx}" y="${ly}" text-anchor="${vertical ? "start" : "middle"}">${lab}</text>`;
  }
  const cxD = SM_X(5) + w / 2, cxS = SM_X(2) + w / 2;
  html += `<path class="edge" d="M ${cxD} 70 C ${cxD} 22, ${cxS} 22, ${cxS} 70" marker-end="url(#sma)"/><text class="elabel" x="${(cxD + cxS) / 2}" y="30" text-anchor="middle">spacing driven (25 cm): next hill</text>`;
  html += `<line class="edge" x1="${cxD}" y1="110" x2="${cxD}" y2="180" marker-end="url(#sma)"/><text class="elabel" x="${cxD + 8}" y="150">edge seen</text>`;
  html += `<line class="edge" x1="${cxS}" y1="180" x2="${cxS}" y2="110" marker-end="url(#sma)"/><text class="elabel" x="${cxS + 8}" y="150">next row</text>`;
  html += `<text class="elabel" x="${SM_X(0)}" y="342">PAUSED: from any AUTO state, or a low battery</text>`;
  for (const [k, [x, y]] of Object.entries(SM_NODES)) html += `<g class="node" data-s="${k}"><rect x="${x}" y="${y}" width="${w}" height="${h}" rx="8"/><text x="${x + w / 2}" y="${y + 25}" text-anchor="middle">${k}</text></g>`;
  svg.innerHTML = html;
}
let smLast = null;
function smHighlight(state) {
  if (state === smLast) return;
  smLast = state;
  $$("#smSvg .node").forEach((n) => n.classList.toggle("live", n.dataset.s === state));
}

// ---------------------------------------------------------------- boot
flow();
sonar();
uart();
stateMachine();
loadResults();
hero();
trial().catch((e) => console.error(e));
phone().catch((e) => console.error(e));
