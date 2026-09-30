// robot3d.js - the FARM-E model (tools/make_model.py) in three.js: animated
// tracks, wheels, seed flap and pump; the hero viewer; the field-trial scene.
import * as THREE from "three";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { RoomEnvironment } from "three/addons/environments/RoomEnvironment.js";

const REDUCED = matchMedia("(prefers-reduced-motion: reduce)").matches;
let templatePromise = null;

// The .glb ships as a base64 ES module so it loads the same from GitHub Pages or any static host.
export function loadTemplate() {
  if (!templatePromise) {
    templatePromise = import("./farme-glb.js").then(async (mod) => {
      const bin = Uint8Array.from(atob(mod.default), (c) => c.charCodeAt(0));
      const gltf = await new GLTFLoader().parseAsync(bin.buffer, "");
      const scene = gltf.scene;
      const track = gltf.scenes[0].userData.track || gltf.parser.json.scenes[0].extras.track;
      let pad = null;
      scene.traverse((o) => {
        if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; }
        if (o.name === "track_pad") pad = o;
      });
      const padMesh = pad.isMesh ? pad : pad.children.find((c) => c.isMesh);
      pad.visible = false;
      return { scene, track, padGeo: padMesh.geometry, padMat: padMesh.material };
    });
  }
  return templatePromise;
}

// Evenly spaced points around the stadium-shaped track path (same maths as make_model.py).
function trackPoints(T, n, offset) {
  const L1 = T.x_front - T.x_rear, r = T.r, per = 2 * L1 + 2 * Math.PI * r;
  const out = [];
  for (let k = 0; k < n; k++) {
    let s = ((per * k) / n + offset) % per;
    if (s < 0) s += per;
    let x, y, a;
    if (s < L1) { x = T.x_rear + s; y = T.y_c - r; a = 0; }
    else if (s < L1 + Math.PI * r) { const th = (s - L1) / r; x = T.x_front + r * Math.sin(th); y = T.y_c - r * Math.cos(th); a = th; }
    else if (s < 2 * L1 + Math.PI * r) { const u = s - L1 - Math.PI * r; x = T.x_front - u; y = T.y_c + r; a = Math.PI; }
    else { const th = (s - 2 * L1 - Math.PI * r) / r; x = T.x_rear - r * Math.sin(th); y = T.y_c + r * Math.cos(th); a = Math.PI + th; }
    out.push([x, y, a]);
  }
  return { out, per };
}

// One robot instance with its moving parts.
export function makeRobot(tpl) {
  const root = tpl.scene.clone(true);
  const T = tpl.track;
  const G = T.gauge / 2;
  const parts = new Map();
  const wheels = { L: [], R: [] };
  let gate = null, pump = null, nozzle = null;
  root.traverse((o) => {
    if (o.name.startsWith("part-") && o.userData && o.userData.label) {
      const ex = o.userData || {};
      parts.set(o.name.slice(5), { node: o, base: o.position.clone(), label: ex.label, desc: ex.desc, explode: new THREE.Vector3(...(ex.explode || [0, 0, 0])) });
    }
    if (/^wheel_[LR]\d+$/.test(o.name)) wheels[o.name[6]].push({ node: o, r: (o.userData && o.userData.radius) || 0.02 });
    if (o.name === "gate") gate = o;
    if (o.name === "pump") pump = o;
    if (o.name === "track_L_static" || o.name === "track_R_static" || o.name === "track_pad") o.visible = false;
  });
  // materials are shared between clones: give this robot its own so highlights stay local
  root.traverse((o) => { if (o.isMesh) o.material = o.material.clone(); });

  const n = Math.round((2 * (T.x_front - T.x_rear) + 2 * Math.PI * T.r) / T.pitch);
  const tracks = {};
  for (const [side, z] of [["L", -G], ["R", G]]) {
    const inst = new THREE.InstancedMesh(tpl.padGeo, tpl.padMat, n);
    inst.castShadow = true; inst.receiveShadow = true;
    const part = parts.get(`track_${side}`);
    (part ? part.node : root).add(inst);
    tracks[side] = { inst, z, off: 0 };
  }
  const m4 = new THREE.Matrix4(), q = new THREE.Quaternion(), zAxis = new THREE.Vector3(0, 0, 1), one = new THREE.Vector3(1, 1, 1), pos = new THREE.Vector3();
  function layTrack(side) {
    const t = tracks[side];
    const { out } = trackPoints(T, n, t.off);
    out.forEach(([x, y, a], k) => {
      q.setFromAxisAngle(zAxis, a);
      pos.set(x, y, t.z);
      m4.compose(pos, q, one);
      t.inst.setMatrixAt(k, m4);
    });
    t.inst.instanceMatrix.needsUpdate = true;
  }
  layTrack("L"); layTrack("R");

  // seed stream and water stream (simple particles, local to the robot)
  const seedGeo = new THREE.SphereGeometry(0.0022, 5, 4);
  const seedMat = new THREE.MeshStandardMaterial({ color: 0x2a1809, roughness: 0.8 });
  const falling = new THREE.InstancedMesh(seedGeo, seedMat, 24);
  falling.count = 0;
  root.add(falling);
  const fallState = [];
  const waterMat = new THREE.MeshStandardMaterial({ color: 0x5aa0e6, transparent: true, opacity: 0.75, roughness: 0.1 });
  const stream = new THREE.Mesh(new THREE.CylinderGeometry(0.0022, 0.0035, 0.062, 8), waterMat);
  stream.position.set(-0.04, 0.03, 0.03);
  stream.visible = false;
  root.add(stream);

  let gateDeg = 0, pumpOn = false, t = 0;
  return {
    group: root, parts,
    // advance the tracks by the distance each side moved (metres)
    drive(dl, dr) {
      tracks.L.off -= dl; tracks.R.off -= dr;
      layTrack("L"); layTrack("R");
      for (const w of wheels.L) w.node.rotation.z -= dl / w.r;
      for (const w of wheels.R) w.node.rotation.z -= dr / w.r;
    },
    setGate(deg) { gateDeg = deg; if (gate) gate.rotation.x = -THREE.MathUtils.degToRad(Math.min(70, deg * 1.4)); },
    setPump(on) { pumpOn = on; stream.visible = on; },
    tick(dt) {
      t += dt;
      if (pump) pump.position.x = pumpOn ? Math.sin(t * 90) * 0.0006 : 0;
      if (pumpOn) stream.scale.set(1 + 0.15 * Math.sin(t * 40), 1, 1);
      // seeds fall from the tube end while the flap is open
      if (gateDeg > 14 && Math.random() < dt * 30 * Math.min(1, (gateDeg - 12) / 30)) fallState.push({ y: 0.012, x: -0.012 + (Math.random() - 0.5) * 0.006, z: (Math.random() - 0.5) * 0.006, v: 0 });
      for (const s of fallState) { s.v += 9.8 * dt; s.y -= s.v * dt; }
      while (fallState.length && fallState[0].y < 0.001) fallState.shift();
      falling.count = Math.min(24, fallState.length);
      for (let k = 0; k < falling.count; k++) {
        const s = fallState[k];
        m4.makeTranslation(s.x, s.y, s.z);
        falling.setMatrixAt(k, m4);
      }
      falling.instanceMatrix.needsUpdate = true;
    },
    explode(f) {
      for (const p of parts.values()) p.node.position.copy(p.base).addScaledVector(p.explode, f);
    },
    highlight(key) {
      for (const [k, p] of parts) {
        p.node.traverse((o) => {
          if (!o.isMesh || !o.material.emissive) return;
          o.material.emissive.setHex(k === key ? 0x5a4a00 : 0x000000);
        });
      }
    },
  };
}

function renderer(host, { alpha = true } = {}) {
  const r = new THREE.WebGLRenderer({ antialias: true, alpha, powerPreference: "high-performance" });
  r.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.75));
  r.outputColorSpace = THREE.SRGBColorSpace;
  r.toneMapping = THREE.ACESFilmicToneMapping;
  r.toneMappingExposure = 1.05;
  r.shadowMap.enabled = true;
  r.shadowMap.type = THREE.PCFSoftShadowMap;
  host.prepend(r.domElement);
  return r;
}

function visibility(el) {
  return () => { const r = el.getBoundingClientRect(); return !document.hidden && r.bottom > -120 && r.top < innerHeight + 120; };
}

// ---------------------------------------------------------------- hero viewer
export async function heroViewer(host, ui) {
  const tpl = await loadTemplate();
  const r = renderer(host);
  const scene = new THREE.Scene();
  const pmrem = new THREE.PMREMGenerator(r);
  scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
  const cam = new THREE.PerspectiveCamera(30, 1.3, 0.01, 20);
  cam.position.set(0.52, 0.34, 0.6);
  const ctl = new OrbitControls(cam, r.domElement);
  ctl.target.set(0, 0.07, 0);
  ctl.enableDamping = true; ctl.enablePan = false;
  ctl.minDistance = 0.3; ctl.maxDistance = 1.6; ctl.maxPolarAngle = Math.PI * 0.49;
  ctl.autoRotate = !REDUCED; ctl.autoRotateSpeed = 0.8;
  ctl.addEventListener("start", () => { ctl.autoRotate = false; ui.onRotate?.(false); });

  const key = new THREE.DirectionalLight(0xfff6e0, 2.4);
  key.position.set(0.6, 1.2, 0.5); key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048);
  Object.assign(key.shadow.camera, { left: -0.4, right: 0.4, top: 0.4, bottom: -0.4, near: 0.1, far: 3 });
  key.shadow.bias = -0.0004;
  scene.add(key, new THREE.HemisphereLight(0xdfeccf, 0x2a2010, 0.7));
  // a disc of soil to stand on
  const soilTex = soilTexture(512, 512, 7);
  const ground = new THREE.Mesh(new THREE.CircleGeometry(0.62, 64), new THREE.MeshStandardMaterial({ map: soilTex, roughness: 1 }));
  ground.rotation.x = -Math.PI / 2; ground.receiveShadow = true;
  scene.add(ground);
  const ring = new THREE.Mesh(new THREE.RingGeometry(0.6, 0.62, 96), new THREE.MeshBasicMaterial({ color: 0xffde00 }));
  ring.rotation.x = -Math.PI / 2; ring.position.y = 0.001;
  scene.add(ring);

  const bot = makeRobot(tpl);
  scene.add(bot.group);

  // hotspots
  const tmp = new THREE.Vector3(), box = new THREE.Box3();
  const keys = [...bot.parts.keys()].filter((k) => k !== "track_R" && k !== "wires");
  const buttons = keys.map((k, i) => {
    const b = document.createElement("button");
    b.type = "button"; b.className = "hotspot"; b.textContent = String(i + 1);
    b.setAttribute("aria-label", bot.parts.get(k).label);
    b.title = bot.parts.get(k).label;
    b.addEventListener("click", () => select(k));
    host.appendChild(b);
    return b;
  });
  let card = null, selected = null;
  function select(k) {
    selected = k;
    bot.highlight(k);
    buttons.forEach((b, i) => b.classList.toggle("on", keys[i] === k));
    card?.remove();
    const p = bot.parts.get(k);
    card = document.createElement("div");
    card.className = "part-card";
    card.innerHTML = `<button type="button" aria-label="Close">&times;</button><h3></h3><p></p>`;
    card.querySelector("h3").textContent = p.label;
    card.querySelector("p").textContent = p.desc;
    card.querySelector("button").onclick = () => { card.remove(); card = null; selected = null; bot.highlight(null); buttons.forEach((b) => b.classList.remove("on")); };
    host.appendChild(card);
  }

  let explodeTarget = 0, explode = 0, cycle = null;
  const resize = () => {
    const w = host.clientWidth, h = host.clientHeight;
    r.setSize(w, h, false); cam.aspect = w / h; cam.updateProjectionMatrix();
  };
  new ResizeObserver(resize).observe(host); resize();
  const visible = visibility(host);
  const clock = new THREE.Clock();
  r.setAnimationLoop(() => {
    const dt = Math.min(clock.getDelta(), 0.05);
    if (!visible()) return;
    ctl.update();
    explode += (explodeTarget - explode) * Math.min(1, dt * 6);
    bot.explode(explode);
    // planting-cycle demo: drive 25 cm, stop, open the flap, water, repeat
    if (cycle) {
      cycle.t += dt;
      const t = cycle.t % 3.2;
      const v = t < 1.1 ? 0.24 : 0;
      bot.drive(v * dt, v * dt);
      bot.setGate(t > 1.35 && t < 1.6 ? 40 : 0);
      bot.setPump(t > 1.75 && t < 2.85);
      soilTex.offset.x += (v * dt) / 1.24;
    } else {
      bot.setGate(0); bot.setPump(false);
    }
    bot.tick(dt);
    r.render(scene, cam);
    const w = host.clientWidth, h = host.clientHeight;
    keys.forEach((k, i) => {
      box.setFromObject(bot.parts.get(k).node);
      box.getCenter(tmp);
      if (k === "track_L") tmp.y = 0.07;
      tmp.project(cam);
      buttons[i].style.left = `${(tmp.x * 0.5 + 0.5) * w}px`;
      buttons[i].style.top = `${(-tmp.y * 0.5 + 0.5) * h}px`;
    });
  });
  host.querySelector(".loading")?.remove();
  return {
    setExplode(f) { explodeTarget = f; },
    toggleCycle() { cycle = cycle ? null : { t: 0 }; return !!cycle; },
    setRotate(on) { ctl.autoRotate = on; },
    select,
  };
}

// procedural soil
function soilTexture(w, h, seed = 1) {
  const c = document.createElement("canvas");
  c.width = w; c.height = h;
  const g = c.getContext("2d");
  g.fillStyle = "#9a6a3f"; g.fillRect(0, 0, w, h);
  let s = seed * 9301 + 49297;
  const rnd = () => ((s = (s * 9301 + 49297) % 233280) / 233280);
  for (let i = 0; i < w * h / 6; i++) {
    const v = rnd();
    g.fillStyle = v < 0.5 ? `rgba(70,42,20,${0.12 + 0.2 * rnd()})` : `rgba(200,160,110,${0.08 + 0.18 * rnd()})`;
    const r = 0.6 + rnd() * 1.8;
    g.fillRect(rnd() * w, rnd() * h, r, r);
  }
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.anisotropy = 4;
  return t;
}

// ---------------------------------------------------------------- field-trial scene
// One bed (or several side by side). Each bed has a
// canvas texture the robot "prints" its tracks, water and seed onto.
export async function fieldScene(host, { count = 1 } = {}) {
  const tpl = await loadTemplate();
  const r = renderer(host, { alpha: false });
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0xb9cfdd);
  scene.fog = new THREE.Fog(0xb9cfdd, 9, 22);
  const pmrem = new THREE.PMREMGenerator(r);
  scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
  const cam = new THREE.PerspectiveCamera(38, 16 / 8, 0.02, 60);
  cam.position.set(0.2, 2.4, 3.6);
  const ctl = new OrbitControls(cam, r.domElement);
  ctl.enableDamping = true; ctl.maxPolarAngle = Math.PI * 0.47; ctl.minDistance = 0.4; ctl.maxDistance = 9;
  ctl.target.set(0, 0, 0);
  const sun = new THREE.DirectionalLight(0xfff3dd, 2.6);
  sun.position.set(2.5, 5, 2.2); sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  Object.assign(sun.shadow.camera, { left: -3.2, right: 3.2, top: 3.2, bottom: -3.2, near: 0.5, far: 14 });
  sun.shadow.bias = -0.0005;
  scene.add(sun, new THREE.HemisphereLight(0xe3efff, 0x3b4a25, 0.8));
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(40, 40), new THREE.MeshStandardMaterial({ color: 0x77895a, roughness: 1 }));
  floor.rotation.x = -Math.PI / 2; floor.receiveShadow = true;
  scene.add(floor);

  const beds = Array.from({ length: count }, (_, i) => makeBed(i));
  let camMode = "overview";

  function makeBed(i) {
    const grp = new THREE.Group();
    grp.position.z = count === 1 ? 0 : (i === 0 ? -0.95 : 0.95);
    scene.add(grp);
    const cv = document.createElement("canvas");
    cv.width = 1200; cv.height = 600;
    const ctx = cv.getContext("2d");
    const tex = new THREE.CanvasTexture(cv);
    tex.colorSpace = THREE.SRGBColorSpace; tex.anisotropy = 8;
    const bot = makeRobot(tpl);
    grp.add(bot.group);
    const seedMesh = new THREE.InstancedMesh(new THREE.SphereGeometry(0.0035, 6, 4), new THREE.MeshStandardMaterial({ color: 0x2b170a, roughness: 0.9 }), 4000);
    seedMesh.count = 0;
    grp.add(seedMesh);
    const label = document.createElement("div");
    label.className = "label3d";
    label.textContent = "FARM-E";
    label.style.background = "var(--green-deep)";
    host.appendChild(label);
    return { grp, cv, ctx, tex, bot, seedMesh, label, deco: null, L: 2.4, W: 1.2, drop: 0.75, nTrail: 0, nWet: 0, nSeed: 0, last: null, fallAnim: 0 };
  }

  function paintBase(b) {
    const { ctx, cv } = b;
    ctx.fillStyle = "#9b6b40";
    ctx.fillRect(0, 0, cv.width, cv.height);
    let s = 1234 + b.cv.width;
    const rnd = () => ((s = (s * 9301 + 49297) % 233280) / 233280);
    for (let k = 0; k < 90000; k++) {
      const v = rnd();
      ctx.fillStyle = v < 0.5 ? `rgba(70,42,20,${0.1 + 0.2 * rnd()})` : `rgba(205,165,115,${0.07 + 0.16 * rnd()})`;
      const rr = 0.8 + rnd() * 2.2;
      ctx.fillRect(rnd() * cv.width, rnd() * cv.height, rr, rr);
    }
    b.tex.needsUpdate = true;
  }

  // (re)build a bed for a scenario: bed_len L, width W, drop below the soil
  function setup(i, { L = 2.4, W = 1.2, drop = 0.75 }) {
    const b = beds[i];
    b.L = L; b.W = W; b.drop = drop;
    if (b.deco) { b.grp.remove(b.deco); b.deco.traverse((o) => o.geometry?.dispose()); }
    const deco = new THREE.Group();
    const soilMat = new THREE.MeshStandardMaterial({ map: b.tex, roughness: 1 });
    const side = new THREE.MeshStandardMaterial({ color: 0x6f4a2a, roughness: 1 });
    const top = new THREE.Mesh(new THREE.BoxGeometry(L, 0.03, W), [side, side, soilMat, side, side, side]);
    top.position.y = -0.015; top.receiveShadow = true;
    deco.add(top);
    if (drop > 0.3) {
      // a table under a thin tray of soil
      const wood = new THREE.MeshStandardMaterial({ color: 0xe8e4da, roughness: 0.7 });
      const slab = new THREE.Mesh(new THREE.BoxGeometry(L, 0.04, W), wood);
      slab.position.y = -0.05; slab.castShadow = true; slab.receiveShadow = true;
      deco.add(slab);
      const legG = new THREE.BoxGeometry(0.05, drop - 0.07, 0.05);
      for (const x of [-L / 2 + 0.08, L / 2 - 0.08]) for (const z of [-W / 2 + 0.08, W / 2 - 0.08]) {
        const leg = new THREE.Mesh(legG, wood);
        leg.position.set(x, -0.07 - (drop - 0.07) / 2, z); leg.castShadow = true;
        deco.add(leg);
      }
    } else {
      const plank = new THREE.MeshStandardMaterial({ color: 0x8a6a45, roughness: 0.9 });
      const body = new THREE.Mesh(new THREE.BoxGeometry(L, drop - 0.03, W), new THREE.MeshStandardMaterial({ color: 0x5e3f24, roughness: 1 }));
      body.position.y = -0.03 - (drop - 0.03) / 2; body.receiveShadow = true;
      deco.add(body);
      for (const s of [-1, 1]) {
        const long = new THREE.Mesh(new THREE.BoxGeometry(L + 0.04, drop + 0.01, 0.02), plank);
        long.position.set(0, -drop / 2 + 0.005, s * (W / 2 + 0.01)); long.castShadow = true;
        deco.add(long);
        const short = new THREE.Mesh(new THREE.BoxGeometry(0.02, drop + 0.01, W + 0.04), plank);
        short.position.set(s * (L / 2 + 0.01), -drop / 2 + 0.005, 0); short.castShadow = true;
        deco.add(short);
      }
    }
    deco.position.y = drop;
    b.grp.add(deco);
    b.deco = deco;
    b.bot.group.position.set(0, drop, 0);
    b.nTrail = b.nWet = b.nSeed = 0;
    b.seedMesh.count = 0;
    b.last = null;
    b.fallAnim = 0;
    b.bot.group.rotation.set(0, 0, 0);
    paintBase(b);
  }

  const m4 = new THREE.Matrix4();
  const toPx = (b, x, y) => [x / b.L * b.cv.width, (b.W / 2 - y) / b.W * b.cv.height];

  // push one rig's state into its bed; data: {x,y,th,vl,vr,gate,pump,fell, trail, wet, seeds, dt}
  function update(i, d) {
    const b = beds[i];
    const gx = d.x - b.L / 2, gz = -d.y;
    if (!d.fell) {
      b.bot.group.position.set(gx, b.drop, gz);
      b.bot.group.rotation.set(0, d.th, 0);
      b.fallAnim = 0;
    } else {
      // tip over the edge and land on the floor below
      b.fallAnim = Math.min(1, b.fallAnim + d.frameDt * 1.6);
      const e = b.fallAnim * b.fallAnim;
      b.bot.group.position.set(gx + Math.cos(d.th) * 0.12 * b.fallAnim, b.drop * (1 - e) + 0.04 * e, gz - Math.sin(d.th) * 0.12 * b.fallAnim);
      b.bot.group.rotation.set(0, d.th, -1.2 * b.fallAnim, "YXZ");
    }
    b.bot.drive(d.dl, d.dr);
    b.bot.setGate(d.gate);
    b.bot.setPump(d.pump && d.tank > 0);
    b.bot.tick(d.frameDt);
    const { ctx } = b;
    let dirty = false;
    // tread marks along the trail
    const tr = d.trail, nT = tr.length / 6;
    if (nT < b.nTrail) b.nTrail = 0;
    ctx.lineCap = "round";
    for (let k = Math.max(1, b.nTrail); k < nT; k++) {
      const x0 = tr[(k - 1) * 6 + 1], y0 = tr[(k - 1) * 6 + 2], x1 = tr[k * 6 + 1], y1 = tr[k * 6 + 2], th = tr[k * 6 + 3];
      if (Math.hypot(x1 - x0, y1 - y0) < 1e-4) continue;
      for (const s of [-1, 1]) {
        const ox = -Math.sin(th) * s * 0.085, oy = Math.cos(th) * s * 0.085;
        const [a, bb] = toPx(b, x0 + ox, y0 + oy), [c, e] = toPx(b, x1 + ox, y1 + oy);
        ctx.strokeStyle = "rgba(60,36,18,0.30)"; ctx.lineWidth = 0.042 / b.L * b.cv.width;
        ctx.beginPath(); ctx.moveTo(a, bb); ctx.lineTo(c, e); ctx.stroke();
        ctx.strokeStyle = "rgba(40,24,10,0.18)"; ctx.lineWidth = 2;
        ctx.setLineDash([2, 5]); ctx.beginPath(); ctx.moveTo(a, bb); ctx.lineTo(c, e); ctx.stroke(); ctx.setLineDash([]);
      }
      dirty = true;
    }
    b.nTrail = nT;
    // wet soil
    const wt = d.wet, nW = wt.length / 4;
    for (let k = b.nWet; k < nW; k++) {
      const [px, py] = toPx(b, wt[k * 4], wt[k * 4 + 1]);
      const rad = (0.022 + 0.006 * Math.sqrt(wt[k * 4 + 2])) / b.L * b.cv.width;
      const gr = ctx.createRadialGradient(px, py, 0, px, py, rad);
      gr.addColorStop(0, "rgba(46,30,22,0.55)"); gr.addColorStop(0.6, "rgba(52,36,30,0.25)"); gr.addColorStop(1, "rgba(58,34,16,0)");
      ctx.fillStyle = gr; ctx.beginPath(); ctx.arc(px, py, rad, 0, Math.PI * 2); ctx.fill();
      dirty = true;
    }
    b.nWet = nW;
    // seeds as tiny 3D grains
    const sd = d.seeds, nS = Math.min(4000, sd.length / 3);
    for (let k = b.nSeed; k < nS; k++) {
      const x = sd[k * 3], y = sd[k * 3 + 1];
      const on = x >= 0 && x <= b.L && Math.abs(y) <= b.W / 2;
      m4.makeTranslation(x - b.L / 2, on ? b.drop + 0.002 : 0.004, -y);
      b.seedMesh.setMatrixAt(k, m4);
      const [px, py] = toPx(b, x, y);
      if (on) { ctx.fillStyle = "rgba(35,20,8,0.9)"; ctx.fillRect(px - 1.5, py - 1.5, 3, 3); dirty = true; }
    }
    if (nS !== b.nSeed) { b.seedMesh.count = nS; b.seedMesh.instanceMatrix.needsUpdate = true; }
    b.nSeed = nS;
    if (dirty) b.dirty = true;
  }

  let api = null;
  const resize = () => {
    const w = host.clientWidth, h = host.clientHeight;
    r.setSize(w, h, false); cam.aspect = w / h; cam.updateProjectionMatrix();
    if (api && camMode === "overview") api.setCamera("overview");
  };
  new ResizeObserver(resize).observe(host); resize();
  const visible = visibility(host);
  const tmp = new THREE.Vector3();
  let lastUpload = 0;
  function frame() {
    const now = performance.now();
    if (now - lastUpload > 180) {  // re-upload the painted soil at most ~5 times a second
      lastUpload = now;
      for (const b of beds) if (b.dirty) { b.tex.needsUpdate = true; b.dirty = false; }
    }
    if (camMode === "follow") {
      const b = beds[0];
      b.bot.group.getWorldPosition(tmp);
      ctl.target.lerp(tmp, 0.08);
    }
    ctl.update();
    r.render(scene, cam);
    const w = host.clientWidth, h = host.clientHeight;
    for (const b of beds) {
      b.bot.group.getWorldPosition(tmp);
      tmp.y += 0.26;
      tmp.project(cam);
      b.label.style.left = `${(tmp.x * 0.5 + 0.5) * w}px`;
      b.label.style.top = `${(-tmp.y * 0.5 + 0.5) * h}px`;
      b.label.style.opacity = tmp.z < 1 ? 1 : 0;
    }
  }
  api = {
    setup, update, frame, visible,
    setCamera(mode) {
      camMode = mode;
      if (mode === "overview") {
        const k = (cam.aspect < 1.2 ? 1.6 : 1) * (count === 1 ? 0.78 : 1);
        ctl.target.set(0.1, 0.55, 0); cam.position.set(0.5 * k, 0.55 + 1.5 * k, 2.75 * k);
      } else { const b = beds[0]; b.bot.group.getWorldPosition(tmp); cam.position.set(tmp.x - 0.7, tmp.y + 0.6, tmp.z + 0.8); }
    },
  };
  return api;
}
