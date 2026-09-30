"""The website runs the WebAssembly build; the analysis runs the native one.
Same C++, same inputs -> bit-identical results."""
import json
import shutil
import subprocess
from pathlib import Path

import pytest

from farmelab import Rig

ROOT = Path(__file__).resolve().parent.parent
SIMJS = ROOT / "docs" / "js" / "sim.js"
WASM = ROOT / "docs" / "js" / "farmesim-wasm.js"
KEYS = ["x", "y", "th", "hills", "seeds_dropped", "tank_mL", "energy_Wh", "uno_loops", "esp_loops", "pings"]

SCRIPT = r"""
import { SimWorld } from '%s';
const w = await SimWorld.create();
const out = {};
for (const h of [0, 1]) {
  const r = w.rig(h).init({}, 7 + h);
  r.run(2000);
  r.send('/api/cmd?c=auto');
  r.run(25000);
  out[h] = %s.map(k => r.get(k));
}
console.log(JSON.stringify(out));
"""


@pytest.mark.skipif(shutil.which("node") is None or not WASM.exists(), reason="needs node and the wasm build")
def test_native_and_wasm_agree_bit_for_bit(tmp_path):
    js = tmp_path / "parity.mjs"
    js.write_text(SCRIPT % (SIMJS.as_uri(), json.dumps(KEYS)))
    got = json.loads(subprocess.run(["node", str(js)], capture_output=True, text=True, check=True).stdout)
    for h in (0, 1):
        r = Rig(seed=7 + h, handle=h).run(2)
        r.get("/api/cmd?c=auto")
        r.run(25)
        assert got[str(h)] == [r[k] for k in KEYS]
