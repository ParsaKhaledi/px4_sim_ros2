from pathlib import Path


def _step(text: str, name: str) -> str:
    marker = f"name: {name}"
    start = text.index(marker)
    rest = text[start + len(marker):]
    nxt = rest.find("\n      - name:")
    if nxt < 0:
        nxt = len(rest)
    return rest[:nxt]


def test_camera_flight_is_non_blocking():
    text = Path(".github/workflows/_build.yml").read_text(encoding="utf-8")
    plain = _step(text, "Flight test (plain x500)")
    camera = _step(text, "Flight test (x500_depth, software rendering)")
    assert "continue-on-error" not in plain
    assert "continue-on-error: true" in camera
    assert "PR #20" in camera
    assert "camera_link" in camera
    assert 'GZ_CAMERA_UPDATE_RATE: "10"' in camera
    assert 'HEALTH_CAMERA_MIN_HZ: "1"' in camera
