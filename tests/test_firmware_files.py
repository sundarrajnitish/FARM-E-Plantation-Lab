"""Generated and shared firmware files stay in sync; no secrets are committed."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))


def test_web_ui_header_is_up_to_date():
    import embed_ui
    assert (ROOT / "firmware/FarmeLink/src/web_ui.h").read_text(encoding="utf-8") == embed_ui.render()


def test_protocol_copies_are_identical():
    for name in ("farme_proto.h", "farme_proto.cpp"):
        a = (ROOT / "firmware/FarmeDrive/src" / name).read_bytes()
        b = (ROOT / "firmware/FarmeLink/src" / name).read_bytes()
        assert a == b, name


def test_wifi_secrets_stay_out_of_git():
    assert "firmware/FarmeLink/secrets.h" in (ROOT / ".gitignore").read_text()
    assert "change-me" in (ROOT / "firmware/FarmeLink/secrets.example.h").read_text()
