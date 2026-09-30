"""The generated 3D model is a valid glTF binary with the parts the site animates."""
import json
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))


def test_model_builds_and_has_named_parts(tmp_path):
    import make_model
    data = make_model.write_glb(make_model.build(), tmp_path / "m.glb")
    magic, version, total = struct.unpack("<III", data[:12])
    assert magic == 0x46546C67 and version == 2 and total == len(data)
    jlen = struct.unpack("<I", data[12:16])[0]
    gl = json.loads(data[20:20 + jlen])
    names = {n["name"] for n in gl["nodes"]}
    for part in ("uno", "nodemcu", "battery", "hopper", "servo", "tank", "sonar", "track_L", "track_R"):
        assert f"part-{part}" in names
    assert {"gate", "pump", "track_pad", "wheel_L0", "wheel_R0"} <= names
    assert all("label" in n.get("extras", {}) for n in gl["nodes"] if n["name"].startswith("part-"))
    assert len(data) < 2_000_000


def test_shipped_model_is_current():
    import base64
    import make_model
    shipped = (ROOT / "docs" / "models" / "farme.glb").read_bytes()
    assert shipped == make_model.write_glb(make_model.build(), Path("/tmp/_farme_check.glb"))
    js = (ROOT / "docs" / "js" / "farme-glb.js").read_text()
    assert base64.b64encode(shipped).decode() in js
