"""Generate the FARM-E 3D model (glTF 2.0 binary) procedurally, from a photo of the build.

    python tools/make_model.py   ->  docs/models/farme.glb and docs/js/farme-glb.js (base64 module)

Everything is built from primitives with numpy - no third-party model, no textures,
no logos. Units are metres; +X is forward, +Y up, -Z is the robot's left.
Each top-level part is a node named "part-<key>" whose extras carry the label,
a description and an explode direction for the site's exploded view. Nodes the
site animates: wheel_L*/wheel_R* (spin about Z), gate (servo horn + flap, about X),
pump, track_L_static / track_R_static (replaced by animated instanced pads) and
"track_pad" (the pad template).
"""
from __future__ import annotations

import base64
import json
import math
import struct
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent

# ---------------------------------------------------------------- materials
MATERIALS = {
    # name: (sRGB colour, metallic, roughness)
    "black": ((0.035, 0.035, 0.04), 0.1, 0.65),
    "rubber": ((0.03, 0.03, 0.03), 0.0, 0.92),
    "steel": ((0.62, 0.63, 0.65), 0.85, 0.35),
    "alu": ((0.78, 0.79, 0.8), 0.8, 0.3),
    "meccano": ((0.72, 0.06, 0.05), 0.35, 0.45),
    "pcb_blue": ((0.03, 0.26, 0.55), 0.1, 0.5),
    "pcb_dark": ((0.05, 0.08, 0.12), 0.1, 0.5),
    "pcb_red": ((0.55, 0.05, 0.04), 0.1, 0.5),
    "chip": ((0.02, 0.02, 0.025), 0.1, 0.4),
    "gold": ((0.85, 0.66, 0.25), 0.9, 0.3),
    "white_plastic": ((0.9, 0.9, 0.87), 0.0, 0.55),
    "breadboard": ((0.93, 0.92, 0.88), 0.0, 0.6),
    "orange_print": ((0.98, 0.42, 0.05), 0.0, 0.55),
    "seed": ((0.16, 0.09, 0.05), 0.0, 0.7),
    "lipo": ((0.72, 0.05, 0.06), 0.1, 0.4),
    "lipo_band": ((0.95, 0.95, 0.95), 0.0, 0.5),
    "solar": ((0.03, 0.05, 0.16), 0.3, 0.2),
    "solar_line": ((0.7, 0.72, 0.75), 0.8, 0.3),
    "servo_blue": ((0.12, 0.3, 0.75), 0.0, 0.45),
    "sonar_silver": ((0.8, 0.8, 0.82), 0.9, 0.25),
    "sonar_mesh": ((0.12, 0.12, 0.13), 0.3, 0.8),
    "water": ((0.25, 0.55, 0.85), 0.0, 0.1),
    "yellow_motor": ((0.95, 0.78, 0.1), 0.0, 0.5),
    "wire_red": ((0.8, 0.08, 0.06), 0.0, 0.5),
    "wire_black": ((0.05, 0.05, 0.05), 0.0, 0.5),
    "wire_orange": ((0.95, 0.45, 0.05), 0.0, 0.5),
    "wire_blue": ((0.1, 0.35, 0.85), 0.0, 0.5),
    "wire_yellow": ((0.95, 0.85, 0.1), 0.0, 0.5),
    "wire_green": ((0.1, 0.6, 0.25), 0.0, 0.5),
    "wire_white": ((0.92, 0.92, 0.92), 0.0, 0.5),
    "blue_sensor": ((0.2, 0.5, 0.85), 0.0, 0.5),
}


# ---------------------------------------------------------------- mesh helpers
class M:
    """Triangle mesh: positions, normals, indices."""

    def __init__(self, p=None, n=None, i=None):
        self.p = np.zeros((0, 3), np.float32) if p is None else np.asarray(p, np.float32)
        self.n = np.zeros((0, 3), np.float32) if n is None else np.asarray(n, np.float32)
        self.i = np.zeros((0,), np.uint32) if i is None else np.asarray(i, np.uint32)

    def __add__(self, o):
        return M(np.vstack([self.p, o.p]), np.vstack([self.n, o.n]), np.concatenate([self.i, o.i + len(self.p)]))

    def moved(self, t=(0, 0, 0), R=None):
        p, n = self.p, self.n
        if R is not None:
            R = np.asarray(R, np.float32)
            p = p @ R.T
            n = n @ R.T
        return M(p + np.asarray(t, np.float32), n, self.i)


def rot(axis, deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def quad_faces(corners, faces):
    """Flat-shaded polyhedron from corner points and quads (lists of 4 indices, CCW from outside)."""
    P, N, I = [], [], []
    for f in faces:
        v = [np.asarray(corners[k], float) for k in f]
        nrm = np.cross(v[1] - v[0], v[2] - v[0])
        if np.linalg.norm(nrm) < 1e-12:
            nrm = np.cross(v[2] - v[0], v[3] - v[0])
        nrm = nrm / (np.linalg.norm(nrm) + 1e-12)
        b = len(P)
        P += v
        N += [nrm] * 4
        I += [b, b + 1, b + 2, b, b + 2, b + 3]
    return M(P, N, I)


def hexa(c):
    """8 corners: bottom 0-3 (x-z-, x+z-, x+z+, x-z+), top 4-7 same order."""
    return quad_faces(c, [[0, 1, 2, 3][::-1], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]])


def box(sx, sy, sz, c=(0, 0, 0)):
    x, y, z = sx / 2, sy / 2, sz / 2
    cs = [(-x, -y, -z), (x, -y, -z), (x, -y, z), (-x, -y, z), (-x, y, -z), (x, y, -z), (x, y, z), (-x, y, z)]
    return hexa(cs).moved(c)


def rbox(sx, sy, sz, r, c=(0, 0, 0), seg=3):
    """Rounded box with smooth normals."""
    h = np.array([sx, sy, sz]) / 2
    r = min(r, *(h * 0.999))
    ih = h - r
    P, N, I = [], [], []

    def coords(k):
        return np.concatenate([np.linspace(-h[k], -ih[k], seg + 1), np.linspace(ih[k], h[k], seg + 1)])

    for axis in range(3):
        for sign in (-1, 1):
            u, v = [a for a in range(3) if a != axis]
            cu, cv = coords(u), coords(v)
            b = len(P)
            for a in cu:
                for bb in cv:
                    p = np.zeros(3)
                    p[axis] = sign * h[axis]
                    p[u], p[v] = a, bb
                    inner = np.clip(p, -ih, ih)
                    d = p - inner
                    nn = d / (np.linalg.norm(d) + 1e-12)
                    P.append(inner + nn * r)
                    N.append(nn)
            nv = len(cv)
            for ia in range(len(cu) - 1):
                for ib in range(nv - 1):
                    q0 = b + ia * nv + ib
                    q1, q2, q3 = q0 + nv, q0 + nv + 1, q0 + 1
                    tri = [q0, q1, q2, q0, q2, q3]
                    # orient outward
                    pa, pb, pc = np.array(P[q0]), np.array(P[q1]), np.array(P[q2])
                    if np.dot(np.cross(pb - pa, pc - pa), N[q0]) < 0:
                        tri = [q0, q2, q1, q0, q3, q2]
                    I += tri
    return M(P, N, I).moved(c)


def cyl(r, h, seg=24, c=(0, 0, 0), axis="y", caps=True, r2=None):
    r2 = r if r2 is None else r2
    P, N, I = [], [], []
    for k in range(seg + 1):
        a = 2 * math.pi * k / seg
        ca, sa = math.cos(a), math.sin(a)
        slope = (r - r2) / h
        nn = np.array([ca, slope, sa]); nn /= np.linalg.norm(nn)
        P += [(r * ca, -h / 2, r * sa), (r2 * ca, h / 2, r2 * sa)]
        N += [nn, nn]
    for k in range(seg):
        a, b = 2 * k, 2 * k + 2
        I += [a, a + 1, b + 1, a, b + 1, b]
    m = M(P, N, I)
    if caps:
        for y, rr, s in ((-h / 2, r, -1), (h / 2, r2, 1)):
            b = len(m.p)
            P = [(0, y, 0)] + [(rr * math.cos(2 * math.pi * k / seg), y, rr * math.sin(2 * math.pi * k / seg)) for k in range(seg + 1)]
            N = [(0, s, 0)] * len(P)
            I = []
            for k in range(seg):
                I += [0, k + 2, k + 1] if s > 0 else [0, k + 1, k + 2]
            m = m + M(P, N, I)
    R = {"y": None, "x": rot("z", -90), "z": rot("x", 90)}[axis]
    return m.moved(c, R)


def sphere(r, c=(0, 0, 0), seg=10):
    P, N, I = [], [], []
    for a in range(seg + 1):
        th = math.pi * a / seg
        for b in range(2 * seg + 1):
            ph = 2 * math.pi * b / (2 * seg)
            nn = (math.sin(th) * math.cos(ph), math.cos(th), math.sin(th) * math.sin(ph))
            P.append(tuple(r * x for x in nn)); N.append(nn)
    w = 2 * seg + 1
    for a in range(seg):
        for b in range(2 * seg):
            q = a * w + b
            I += [q, q + 1, q + w + 1, q, q + w + 1, q + w]
    return M(P, N, I).moved(c)


def tube(points, r, seg=8):
    pts = [np.asarray(p, float) for p in points]
    P, N, I = [], [], []
    prev = None
    for k, p in enumerate(pts):
        t = (pts[min(k + 1, len(pts) - 1)] - pts[max(k - 1, 0)])
        t /= np.linalg.norm(t) + 1e-12
        if prev is None:
            a = np.array([0, 1, 0]) if abs(t[1]) < 0.9 else np.array([1, 0, 0])
            prev = np.cross(t, a); prev /= np.linalg.norm(prev)
        u = prev - t * np.dot(prev, t); u /= np.linalg.norm(u) + 1e-12
        v = np.cross(t, u)
        prev = u
        for s in range(seg + 1):
            a = 2 * math.pi * s / seg
            nn = u * math.cos(a) + v * math.sin(a)
            P.append(p + nn * r); N.append(nn)
    w = seg + 1
    for k in range(len(pts) - 1):
        for s in range(seg):
            q = k * w + s
            I += [q, q + w, q + w + 1, q, q + w + 1, q + 1]
    return M(P, N, I)


def bezier(p0, p1, p2, p3, n=16):
    p0, p1, p2, p3 = (np.asarray(p, float) for p in (p0, p1, p2, p3))
    return [(1 - t) ** 3 * p0 + 3 * (1 - t) ** 2 * t * p1 + 3 * (1 - t) * t ** 2 * p2 + t ** 3 * p3 for t in np.linspace(0, 1, n)]


# ---------------------------------------------------------------- scene graph
class Node:
    def __init__(self, name, t=(0, 0, 0), extras=None, rq=None):
        self.name, self.t, self.extras, self.rq = name, t, extras, rq
        self.children = []
        self.prims = {}   # material -> M

    def add(self, mat, mesh):
        self.prims[mat] = self.prims.get(mat, M()) + mesh
        return self

    def child(self, name, t=(0, 0, 0), extras=None, rq=None):
        n = Node(name, t, extras, rq)
        self.children.append(n)
        return n


# track geometry shared with the site (docs/js/robot3d.js reads it from extras)
TRACK = dict(x_rear=-0.082, x_front=0.082, y_c=0.034, r=0.034, gauge=0.17, width=0.042, pitch=0.0118)


def track_path_points(n):
    """Evenly spaced points (and tangents) around the stadium track path, in the X-Y plane."""
    L1 = TRACK["x_front"] - TRACK["x_rear"]
    r = TRACK["r"]
    per = 2 * L1 + 2 * math.pi * r
    out = []
    for k in range(n):
        s = per * k / n
        if s < L1:  # bottom run, rear -> front
            p = (TRACK["x_rear"] + s, TRACK["y_c"] - r); a = 0.0
        elif s < L1 + math.pi * r:  # front wheel, bottom -> top
            th = (s - L1) / r
            p = (TRACK["x_front"] + r * math.sin(th), TRACK["y_c"] - r * math.cos(th)); a = th
        elif s < 2 * L1 + math.pi * r:  # top run, front -> rear
            u = s - L1 - math.pi * r
            p = (TRACK["x_front"] - u, TRACK["y_c"] + r); a = math.pi
        else:
            th = (s - 2 * L1 - math.pi * r) / r
            p = (TRACK["x_rear"] - r * math.sin(th), TRACK["y_c"] + r * math.cos(th)); a = math.pi + th
        out.append((p, a))
    return out, per


def track_pad():
    w = TRACK["width"]
    pad = box(0.0105, 0.0035, w) + box(0.004, 0.004, w * 0.96, c=(0, -0.0035, 0))  # plate + grouser
    pad = pad + box(0.006, 0.004, 0.006, c=(0, 0.0035, 0))  # guide tooth
    return pad


def build():
    root = Node("FARM-E")
    G = TRACK["gauge"] / 2

    # -------------------------------------------------- tracks and running gear
    for side, z in (("L", -G), ("R", G)):
        tr = root.child(f"part-track_{side}", extras={
            "label": "Rubber track" if side == "L" else "Rubber track (right)",
            "desc": "Moulded rubber track on a sprocket, an idler and road wheels. The motor drives the rear sprocket.",
            "explode": [0, 0, (-1 if side == "L" else 1) * 0.07]})
        pts, per = track_path_points(int(round((2 * (TRACK["x_front"] - TRACK["x_rear"]) + 2 * math.pi * TRACK["r"]) / TRACK["pitch"])))
        pads = M()
        for (x, y), a in pts:
            pads = pads + track_pad().moved((x, y, z), rot("z", math.degrees(a)))
        tr.child(f"track_{side}_static").add("rubber", pads)
        # sprocket (rear, toothed), idler (front), 3 road wheels, side frame
        for k, (x, r, nm) in enumerate(((TRACK["x_rear"], TRACK["r"] - 0.006, "sprocket"), (TRACK["x_front"], TRACK["r"] - 0.006, "idler"),
                                         (-0.03, 0.012, "road"), (0.0, 0.012, "road"), (0.03, 0.012, "road"))):
            y = TRACK["y_c"] if nm != "road" else 0.016
            wn = tr.child(f"wheel_{side}{k}", t=(x, y, z), extras={"radius": r})
            wn.add("black", cyl(r, TRACK["width"] * 0.7, 28, axis="z"))
            wn.add("steel", cyl(r * 0.35, TRACK["width"] * 0.74, 16, axis="z"))
            if nm == "sprocket":
                for t in range(12):
                    wn.add("black", box(0.006, 0.008, TRACK["width"] * 0.4).moved((0, r + 0.002, 0)).moved((0, 0, 0), rot("z", t * 30)))
            if nm in ("sprocket", "idler"):
                for t in range(5):  # spokes make rotation visible
                    wn.add("steel", box(0.004, r * 0.8, 0.002).moved((0, r * 0.45, TRACK["width"] * 0.36)).moved((0, 0, 0), rot("z", t * 72)))
        frame_z = z + (0.012 if side == "L" else -0.012)
        tr.add("meccano", box(0.19, 0.012, 0.004, c=(0, 0.034, frame_z)))
        for hx in np.linspace(-0.084, 0.084, 13):
            tr.add("black", cyl(0.0022, 0.0045, 10, c=(hx, 0.034, frame_z), axis="z"))

    # -------------------------------------------------- motors
    mo = root.child("part-motors", extras={"label": "Geared motors (x2)",
        "desc": "12 V, 200 rpm geared DC motors, one per track, switched forward / reverse / brake by an L298N-style H-bridge on D2, D4 (left) and D6, D7 (right). No speed control: the enable pins are jumpered on.",
        "explode": [-0.03, -0.03, 0]})
    for z in (-G + 0.03, G - 0.03):
        mo.add("steel", cyl(0.0125, 0.034, 24, c=(TRACK["x_rear"] + 0.02, TRACK["y_c"], z * 0.62), axis="z"))
        mo.add("yellow_motor", box(0.03, 0.022, 0.02, c=(TRACK["x_rear"] + 0.01, TRACK["y_c"], z)))
        mo.add("steel", cyl(0.003, 0.03, 10, c=(TRACK["x_rear"], TRACK["y_c"], z + (-0.015 if z < 0 else 0.015)), axis="z"))

    # -------------------------------------------------- chassis
    ch = root.child("part-chassis", extras={"label": "Chassis",
        "desc": "Black steel deck on a slotted metal frame, carrying the electronics, the hopper and the tank.",
        "explode": [0, -0.02, 0]})
    ch.add("black", rbox(0.25, 0.006, 0.13, 0.002, c=(0, 0.074, 0)))
    ch.add("black", box(0.2, 0.03, 0.004, c=(0, 0.056, -0.062)))
    ch.add("black", box(0.2, 0.03, 0.004, c=(0, 0.056, 0.062)))
    for x in (-0.09, 0.09):
        ch.add("steel", box(0.008, 0.008, 0.16, c=(x, 0.066, 0)))
    ch.add("meccano", box(0.012, 0.004, 0.11, c=(0.105, 0.079, 0)))  # red strip at the front
    for hz in np.linspace(-0.045, 0.045, 7):
        ch.add("black", cyl(0.0022, 0.0045, 10, c=(0.105, 0.079, hz), axis="y"))

    # -------------------------------------------------- H-bridge
    hb = root.child("part-hbridge", t=(-0.06, 0.08, 0.025), extras={"label": "Dual H-bridge",
        "desc": "Switches the two motors from the UNO's logic pins (IN1-IN4). Both inputs HIGH brakes a motor, which the firmware uses to stop dead at an edge.",
        "explode": [0, 0.05, 0.03]})
    hb.add("pcb_red", box(0.043, 0.0016, 0.043))
    hb.add("black", box(0.023, 0.022, 0.016, c=(-0.004, 0.012, 0)))
    for k in range(6):
        hb.add("black", box(0.002, 0.012, 0.014, c=(-0.014 + k * 0.004, 0.028, 0)))
    for z in (-0.014, 0.014):
        hb.add("servo_blue", box(0.008, 0.009, 0.011, c=(0.017, 0.005, z)))

    # -------------------------------------------------- Arduino UNO
    un = root.child("part-uno", t=(0.035, 0.0785, -0.032), extras={"label": "Arduino UNO (ATmega328P)",
        "desc": "Runs the tracks, the seed gate servo (D9), the pump (D12) and the downward-looking sonar (A4/A5). Talks to the NodeMCU over a software UART on A0/A1. Firmware: FarmeDrive.",
        "explode": [0.02, 0.07, -0.04]})
    un.add("pcb_blue", rbox(0.0686, 0.0016, 0.0534, 0.002, c=(0, 0.0008, 0)))
    un.add("chip", box(0.035, 0.004, 0.009, c=(0.008, 0.005, 0.012)))           # DIP-28
    for k in range(14):
        for s in (-1, 1):
            un.add("steel", box(0.0006, 0.003, 0.0014, c=(-0.009 + k * 0.00254, 0.004, 0.012 + s * 0.0055)))
    un.add("steel", box(0.016, 0.011, 0.012, c=(-0.028, 0.0065, -0.011)))       # USB-B
    un.add("black", box(0.014, 0.011, 0.009, c=(-0.029, 0.0065, 0.016)))        # barrel jack
    for z, n in ((-0.0245, 18), (0.0245, 14)):                                   # headers
        un.add("black", box(0.0026 * n, 0.0085, 0.0026, c=(0.004, 0.0058, z)))
    un.add("steel", cyl(0.0035, 0.006, 16, c=(-0.012, 0.004, 0.0), axis="y"))   # crystal can / reset
    un.add("chip", box(0.006, 0.0015, 0.006, c=(-0.014, 0.0023, -0.004)))

    # -------------------------------------------------- breadboard + NodeMCU + DHT11
    bb = root.child("part-nodemcu", t=(0.055, 0.0785, 0.032), extras={"label": "NodeMCU (ESP8266) on a breadboard",
        "desc": "Wi-Fi access point FARM-E at 192.168.4.1: serves the control page and JSON API, reads the DHT11, and forwards commands to the UNO. Firmware: FarmeLink.",
        "explode": [0.03, 0.07, 0.05]})
    bb.add("breadboard", rbox(0.083, 0.0085, 0.055, 0.0015, c=(0, 0.0043, 0)))
    bb.add("black", box(0.078, 0.0003, 0.002, c=(0, 0.0087, 0)))
    for s in (-1, 1):
        bb.add("wire_red", box(0.078, 0.0003, 0.001, c=(0, 0.0087, s * 0.024)))
        bb.add("wire_blue", box(0.078, 0.0003, 0.001, c=(0, 0.0087, s * 0.021)))
    bb.add("pcb_dark", box(0.049, 0.0016, 0.026, c=(0.0, 0.012, 0.0)))
    bb.add("sonar_silver", box(0.016, 0.003, 0.024, c=(0.012, 0.0145, 0.0)))    # ESP-12 can
    bb.add("black", box(0.049, 0.0026, 0.0026, c=(0, 0.0098, -0.0115)))
    bb.add("black", box(0.049, 0.0026, 0.0026, c=(0, 0.0098, 0.0115)))
    bb.add("steel", box(0.006, 0.003, 0.008, c=(-0.024, 0.0145, 0)))            # micro-USB
    dht = bb.child("dht11", t=(-0.03, 0.009, 0.019))
    dht.add("blue_sensor", box(0.012, 0.016, 0.0055, c=(0, 0.008, 0)))
    for k in range(4):
        dht.add("white_plastic", box(0.0012, 0.0012, 0.0005, c=(0, 0.005 + k * 0.003, 0.0029)))

    # -------------------------------------------------- battery
    bt = root.child("part-battery", t=(0.06, 0.092, -0.012), rq=rot("y", 18), extras={"label": "3S LiPo, 2200 mAh",
        "desc": "11.1 V nominal (12.6 V full). The UNO measures it through a 10k/3.3k divider on A3 and stretches drive times as it sags, so hill spacing stays put.",
        "explode": [0.06, 0.08, -0.02]})
    bt.add("lipo", rbox(0.105, 0.022, 0.034, 0.004))
    bt.add("lipo_band", box(0.03, 0.0225, 0.0345, c=(-0.02, 0, 0)))
    bt.add("wire_red", tube(bezier((0.052, 0.004, 0.006), (0.08, 0.004, 0.01), (0.1, -0.01, 0.03), (0.13, -0.03, 0.04)), 0.0018))
    bt.add("wire_black", tube(bezier((0.052, 0.004, -0.004), (0.08, 0.004, 0.0), (0.1, -0.01, 0.022), (0.13, -0.03, 0.032)), 0.0018))
    bt.add("wire_red", rbox(0.012, 0.008, 0.016, 0.001, c=(0.135, -0.032, 0.036)))

    # -------------------------------------------------- seed hopper + gate
    hp = root.child("part-hopper", t=(-0.012, 0.077, -0.018), extras={"label": "Seed hopper (3D-printed)",
        "desc": "Open-top funnel holding the seed. A servo-driven flap under it meters seed into a tube that drops just behind the centre of the robot.",
        "explode": [0, 0.13, 0]})
    top, bot, hgt, wall = 0.052, 0.022, 0.045, 0.0025
    y0, y1 = 0.012, 0.012 + hgt
    o_t, o_b = top / 2, bot / 2
    walls = [
        [(-o_b, y0, -o_b), (o_b, y0, -o_b), (o_b, y0, -o_b + wall), (-o_b, y0, -o_b + wall), (-o_t, y1, -o_t), (o_t, y1, -o_t), (o_t, y1, -o_t + wall), (-o_t, y1, -o_t + wall)],
    ]
    for k in range(4):
        R = rot("y", 90 * k)
        c = [tuple(R @ np.array(p)) for p in walls[0]]
        hp.add("orange_print", hexa(c))
    hp.add("orange_print", box(bot, 0.002, bot, c=(0, y0, 0)))
    hp.add("orange_print", cyl(0.006, 0.012, 16, c=(0, 0.006, 0)))
    rng = np.random.default_rng(3)
    seeds = M()
    for k in range(170):
        yy = y0 + 0.004 + rng.uniform(0, hgt * 0.78)
        half = o_b + (o_t - o_b) * (yy - y0) / hgt - wall - 0.002
        seeds = seeds + sphere(0.0018, c=(rng.uniform(-half, half), yy, rng.uniform(-half, half)), seg=4)
    hp.child("hopper_seeds").add("seed", seeds)
    gate = hp.child("gate", t=(0, 0.002, 0), extras={"axis": "x"})
    gate.add("white_plastic", box(0.016, 0.0015, 0.016, c=(0, -0.001, 0)))
    gate.add("white_plastic", box(0.004, 0.002, 0.018, c=(0, -0.001, -0.017)))
    tube_pts = bezier((0, 0.0, 0), (0, -0.02, 0), (0.0, -0.05, 0.01), (0.0, -0.068, 0.018))
    hp.add("orange_print", tube(tube_pts, 0.0045, 12))

    sv = root.child("part-servo", t=(-0.012, 0.074, -0.052), extras={"label": "SG90 gate servo (D9)",
        "desc": "Opens the hopper flap to 40 degrees for 80 ms plus the set seed time, always with the robot stopped, so each hill is one tight cluster.",
        "explode": [0, 0.06, -0.06]})
    sv.add("servo_blue", box(0.023, 0.0225, 0.0122))
    sv.add("servo_blue", box(0.032, 0.0025, 0.0122, c=(0, 0.004, 0)))
    sv.add("white_plastic", cyl(0.0025, 0.004, 12, c=(0.006, 0.0133, 0)))

    # -------------------------------------------------- water tank + pump + solar panel
    tk = root.child("part-tank", t=(-0.075, 0.077, 0.0), extras={"label": "Water tank and pump (D12)",
        "desc": "About 350 mL, with a small 5 V pump inside and a nozzle behind the hopper. Gives a measured dose per hill (20 mL by default), keeps track of what is left and asks for a refill.",
        "explode": [-0.1, 0.05, 0]})
    tk.add("white_plastic", rbox(0.06, 0.07, 0.085, 0.004, c=(0, 0.035, 0)))
    tk.add("water", box(0.052, 0.045, 0.077, c=(0, 0.025, 0)))
    pump = tk.child("pump", t=(0.0, 0.012, 0.02))
    pump.add("black", cyl(0.009, 0.022, 16, axis="y"))
    tk.add("wire_white", tube(bezier((0.02, 0.07, 0.02), (0.05, 0.08, 0.02), (0.045, 0.03, 0.03), (0.035, -0.01, 0.03)), 0.0025))
    tk.add("black", cyl(0.003, 0.012, 10, c=(0.035, -0.015, 0.03)))
    sp = root.child("part-solar", t=(-0.075, 0.151, 0.0), rq=rot("z", 8), extras={"label": "Small solar panel",
        "desc": "Sits on top of the tank. The electronics run from the LiPo; the panel is not wired into the firmware.",
        "explode": [-0.04, 0.12, 0]})
    sp.add("black", box(0.068, 0.004, 0.095))
    sp.add("solar", box(0.064, 0.0012, 0.091, c=(0, 0.0025, 0)))
    for k in range(1, 4):
        sp.add("solar_line", box(0.064, 0.0004, 0.0008, c=(0, 0.0033, -0.0455 + k * 0.02275)))
    sp.add("solar_line", box(0.0008, 0.0004, 0.091, c=(0, 0.0033, 0)))

    # -------------------------------------------------- sonar (looking down, ahead of the tracks)
    so = root.child("part-sonar", t=(0.128, 0.055, 0.0), extras={"label": "HC-SR04 ultrasonic (A4/A5)",
        "desc": "Looks down at the soil just ahead of the tracks. The UNO learns the ground distance while standing still and stops within two pings when the ground drops away.",
        "explode": [0.06, 0, 0]})
    so.add("pcb_blue", box(0.02, 0.0016, 0.045))
    for z in (-0.013, 0.013):
        so.add("sonar_silver", cyl(0.008, 0.012, 24, c=(0, -0.007, z)))
        so.add("sonar_mesh", cyl(0.0068, 0.0005, 24, c=(0, -0.013, z)))
    so.add("steel", box(0.01, 0.028, 0.004, c=(-0.012, 0.012, 0)))
    so.add("steel", box(0.02, 0.003, 0.012, c=(-0.02, 0.026, 0)))

    # -------------------------------------------------- furrow opener
    fo = root.child("part-tine", t=(0.11, 0.06, -0.03), extras={"label": "Furrow tine",
        "desc": "A fixed foot at the front that scratches a shallow furrow for the seed. Optional sweep-arm servos can be added on D10 and D11.",
        "explode": [0.05, -0.02, -0.03]})
    fo.add("black", hexa([(-0.008, -0.06, -0.012), (0.012, -0.055, -0.012), (0.012, -0.055, 0.012), (-0.008, -0.06, 0.012),
                         (-0.01, 0.0, -0.008), (0.006, 0.0, -0.008), (0.006, 0.0, 0.008), (-0.01, 0.0, 0.008)]))

    # -------------------------------------------------- wiring (loose jumpers, as in the photo)
    wr = root.child("part-wires", extras={"label": "Jumper wiring",
        "desc": "Link: NodeMCU D6 to UNO A0, UNO A1 to NodeMCU D5 through a 1k/2k divider (the ESP8266 is a 3.3 V part). Common ground between the boards.",
        "explode": [0, 0.1, 0]})
    colors = ["wire_orange", "wire_blue", "wire_red", "wire_black", "wire_yellow", "wire_green", "wire_white"]
    rng = np.random.default_rng(11)
    anchors_a = [(0.02, 0.09, -0.012), (0.05, 0.09, -0.012), (0.035, 0.09, -0.056), (0.0, 0.09, -0.056), (0.03, 0.09, -0.02)]
    anchors_b = [(0.04, 0.09, 0.02), (0.07, 0.09, 0.02), (-0.012, 0.1, -0.05), (-0.06, 0.095, 0.025), (0.12, 0.07, 0.0), (0.02, 0.09, 0.045)]
    k = 0
    for a in anchors_a:
        for b in anchors_b[:4] if k < 12 else anchors_b:
            if rng.random() < 0.45:
                continue
            h = rng.uniform(0.02, 0.07)
            a_ = np.array(a); b_ = np.array(b)
            pts = bezier(a_, a_ + (0, h, 0), b_ + (0, h, 0), b_, 18)
            wr.add(colors[k % len(colors)], tube(pts, 0.0011, 6))
            k += 1
    # the pad template for the site's animated tracks
    root.child("track_pad", t=(0, -10, 0)).add("rubber", track_pad())
    root.extras = {"track": TRACK, "units": "m", "forward": "+X", "up": "+Y"}
    return root


# ---------------------------------------------------------------- glTF writer
def write_glb(root: Node, out: Path):
    mat_names = list(MATERIALS)
    gl = {"asset": {"version": "2.0", "generator": "FARM-E Lab tools/make_model.py"}, "scene": 0,
          "scenes": [{"nodes": [0], "extras": getattr(root, "extras", {})}], "nodes": [], "meshes": [], "materials": [],
          "accessors": [], "bufferViews": [], "buffers": []}
    used = {}
    bin_ = bytearray()

    def view(data: bytes, target):
        while len(bin_) % 4:
            bin_.append(0)
        off = len(bin_)
        bin_.extend(data)
        gl["bufferViews"].append({"buffer": 0, "byteOffset": off, "byteLength": len(data), "target": target})
        return len(gl["bufferViews"]) - 1

    def accessor(arr, kind, target):
        if kind == "SCALAR":
            v = view(arr.astype(np.uint32).tobytes(), target)
            gl["accessors"].append({"bufferView": v, "componentType": 5125, "count": int(arr.size), "type": "SCALAR"})
        else:
            a = arr.astype(np.float32)
            v = view(a.tobytes(), target)
            acc = {"bufferView": v, "componentType": 5126, "count": int(len(a)), "type": "VEC3"}
            if kind == "POS":
                acc["min"] = a.min(0).tolist(); acc["max"] = a.max(0).tolist()
            gl["accessors"].append(acc)
        return len(gl["accessors"]) - 1

    def material(name):
        if name not in used:
            rgb, met, rough = MATERIALS[name]
            lin = [round(c ** 2.2, 5) for c in rgb]  # the table is in sRGB; glTF wants linear
            m = {"name": name, "pbrMetallicRoughness": {"baseColorFactor": [*lin, 1.0], "metallicFactor": met, "roughnessFactor": rough}}
            if name == "water":
                m["pbrMetallicRoughness"]["baseColorFactor"][3] = 0.55
                m["alphaMode"] = "BLEND"
            gl["materials"].append(m)
            used[name] = len(gl["materials"]) - 1
        return used[name]

    def add(node: Node):
        idx = len(gl["nodes"])
        n = {"name": node.name}
        gl["nodes"].append(n)
        if any(abs(v) > 0 for v in node.t):
            n["translation"] = [float(v) for v in node.t]
        if node.rq is not None:
            R = node.rq
            w = math.sqrt(max(0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
            x = (R[2, 1] - R[1, 2]) / (4 * w); y = (R[0, 2] - R[2, 0]) / (4 * w); z = (R[1, 0] - R[0, 1]) / (4 * w)
            n["rotation"] = [float(x), float(y), float(z), float(w)]
        if node.extras:
            n["extras"] = node.extras
        if node.prims:
            prims = []
            for mname in mat_names:
                if mname not in node.prims:
                    continue
                m = node.prims[mname]
                prims.append({"attributes": {"POSITION": accessor(m.p, "POS", 34962), "NORMAL": accessor(m.n, "NRM", 34962)},
                              "indices": accessor(m.i, "SCALAR", 34963), "material": material(mname)})
            gl["meshes"].append({"name": node.name, "primitives": prims})
            n["mesh"] = len(gl["meshes"]) - 1
        kids = [add(c) for c in node.children]
        if kids:
            n["children"] = kids
        return idx

    add(root)
    gl["buffers"].append({"byteLength": len(bin_)})
    js = json.dumps(gl, separators=(",", ":")).encode()
    js += b" " * ((4 - len(js) % 4) % 4)
    while len(bin_) % 4:
        bin_.append(0)
    total = 12 + 8 + len(js) + 8 + len(bin_)
    data = struct.pack("<III", 0x46546C67, 2, total) + struct.pack("<II", len(js), 0x4E4F534A) + js + struct.pack("<II", len(bin_), 0x004E4942) + bytes(bin_)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(data)
    return data


def stats(root):
    tris = 0
    stack = [root]
    while stack:
        n = stack.pop()
        tris += sum(len(m.i) // 3 for m in n.prims.values())
        stack += n.children
    return tris


if __name__ == "__main__":
    r = build()
    glb = write_glb(r, ROOT / "docs" / "models" / "farme.glb")
    (ROOT / "docs" / "js" / "farme-glb.js").write_text(
        "// generated by tools/make_model.py - do not edit\nexport default \"" + base64.b64encode(glb).decode() + "\";\n")
    print(f"farme.glb: {len(glb) / 1024:.0f} KiB, {stats(r)} triangles")
