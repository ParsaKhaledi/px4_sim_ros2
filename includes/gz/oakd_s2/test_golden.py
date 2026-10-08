#!/usr/bin/env python3
"""Golden snapshots of today's vision outputs.

Regenerate them on purpose with:

    UPDATE_GOLDEN=1 python3 includes/gz/oakd_s2/test_golden.py

That rewrites includes/gz/oakd_s2/fixtures/golden/ and then checks the new
files. A mismatch fails with a unified diff. Rendering stays in memory:
these tests never call render_oakd.main or write_models, so the checked-in
SDF and URDF are left untouched.

Snapshots:
- render_sdf text for stereo and rgbd, and render_urdf text, for cpu, full,
  and hw at the default mount, a non-default CAM_X/Y/Z mount (translated),
  and a non-default CAM_PITCH_DEG mount (pitched).
- stdout of ``rtabmap_params.py --shell`` and its stderr parameter log, for
  each profile and camera (stereo, rgbd, rgbd-wrapper). The checkout path
  to the ini directory is rewritten to ``rtabmap_profiles/``.
- thresholds_from_env() for each profile, with the VISION_MIN_* and
  VISION_MAX_* overrides unset.
- K and right-camera P from rectified_k and rectified_p, the two calls
  stereo_info_relay assigns onto the right camera_info. No ROS node starts.
"""

from __future__ import annotations

import difflib
import hashlib
import importlib.util
import json
import os
import subprocess
import sys
import unittest
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from pathlib import Path


HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import geometry as geo  # noqa: E402
import render_oakd as render  # noqa: E402
import rtabmap_params as rtab  # noqa: E402


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    # dataclasses resolve the class module through sys.modules during exec.
    sys.modules[name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


REPO = HERE.parents[2]
HEALTH = load_module(REPO / "HealthCheck" / "rtabmap_health_log.py", "rtabmap_health_log_golden")
FIXTURES = HERE / "fixtures" / "golden"

PROFILES = ("cpu", "full", "hw")
VARIANTS = ("stereo", "rgbd")
# translated changes CAM_X/Y/Z. pitched changes CAM_PITCH_DEG. default is Mount().
MOUNTS: dict[str, dict[str, str]] = {
    "default": {},
    "translated": {"CAM_X": "0.20", "CAM_Y": "-0.01", "CAM_Z": "0.30"},
    "pitched": {"CAM_PITCH_DEG": "25.5"},
}

# Cleared in the rtabmap subprocess so a developer shell cannot change the snapshot.
_PROFILE_ENV = (
    "VISION_PROFILE",
    "CAM_STEREO_RES",
    "CAM_STEREO_WIDTH",
    "CAM_STEREO_HEIGHT",
    "CAM_RATE_HZ",
    "CAM_COLOR_WIDTH",
    "CAM_COLOR_HEIGHT",
    "IMU_RATE_HZ",
)
# Cleared around thresholds_from_env so only the profile floors are recorded.
_HEALTH_ENV = (
    "VISION_PROFILE",
    "VISION_MAX_LOST_STREAK",
    "VISION_MAX_RECOVERY_FRAMES",
    "VISION_MIN_MEDIAN_FEATURES",
    "VISION_MIN_INLIERS",
    "VISION_MIN_ODOM_HZ",
)

CHECKED_IN = (render.STEREO_SDF, render.RGBD_SDF, render.URDF_PATH)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


_CHECKED_IN_BEFORE = {path: _sha256(path) for path in CHECKED_IN}


def _child_env(updates: dict[str, str]) -> dict[str, str]:
    env = dict(os.environ)
    for key in _PROFILE_ENV:
        env.pop(key, None)
    env.update(updates)
    return env


def _portable(text: str) -> str:
    """Drop the checkout prefix so the ini path is the same on every machine."""
    return text.replace(str(rtab.INI_DIR), "rtabmap_profiles")


def _rtabmap_cli(profile_name: str, camera: str) -> tuple[str, str]:
    completed = subprocess.run(
        [sys.executable, str(HERE / "rtabmap_params.py"), "--shell", "--camera", camera],
        check=True,
        text=True,
        capture_output=True,
        env=_child_env({"VISION_PROFILE": profile_name}),
    )
    return _portable(completed.stdout), _portable(completed.stderr)


@contextmanager
def _health_env(profile_name: str) -> Iterator[None]:
    saved = {key: os.environ.get(key) for key in _HEALTH_ENV}
    try:
        for key in _HEALTH_ENV:
            os.environ.pop(key, None)
        os.environ["VISION_PROFILE"] = profile_name
        yield
    finally:
        for key, value in saved.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value


def _json(payload: dict[str, object]) -> str:
    return json.dumps(payload, indent=2, sort_keys=True) + "\n"


def _health_json(profile_name: str) -> str:
    with _health_env(profile_name):
        got = HEALTH.thresholds_from_env()
    return _json(
        {
            "profile": profile_name,
            "max_lost_streak": got.max_lost_streak,
            "max_recovery_frames": got.max_recovery_frames,
            "min_median_features": got.min_median_features,
            "min_inliers": got.min_inliers,
            "min_odom_hz": got.min_odom_hz,
        }
    )


def _relay_json(profile_name: str) -> str:
    # Same two calls stereo_info_relay.on_info uses. The node is not started.
    profile = geo.profile_from_env({"VISION_PROFILE": profile_name})
    return _json(
        {
            "profile": profile.name,
            "k": geo.rectified_k(profile),
            "p": geo.rectified_p(right=True, profile=profile),
        }
    )


def _sdf_text(profile_name: str, overrides: dict[str, str], variant: str) -> str:
    profile = geo.profile_from_env({"VISION_PROFILE": profile_name})
    mount = geo.mount_from_env(dict(overrides))
    return render.render_sdf(variant, mount, profile)


def _urdf_text(overrides: dict[str, str]) -> str:
    return render.render_urdf(geo.mount_from_env(dict(overrides)))


def _producers() -> dict[str, Callable[[], str]]:
    producers: dict[str, Callable[[], str]] = {}
    for profile_name in PROFILES:
        for mount_name, overrides in MOUNTS.items():
            for variant in VARIANTS:
                relative = f"sdf/{profile_name}/{mount_name}/{variant}.sdf"
                producers[relative] = _bind(_sdf_text, profile_name, dict(overrides), variant)
            relative = f"urdf/{profile_name}/{mount_name}.urdf"
            producers[relative] = _bind(_urdf_text, dict(overrides))
        for camera in rtab.CAMERAS:
            stdout_name = f"rtabmap/{profile_name}/{camera}.shell"
            stderr_name = f"rtabmap/{profile_name}/{camera}.log"
            producers[stdout_name] = _bind(_rtabmap_stream, profile_name, camera, 0)
            producers[stderr_name] = _bind(_rtabmap_stream, profile_name, camera, 1)
        producers[f"health/{profile_name}.json"] = _bind(_health_json, profile_name)
        producers[f"relay/{profile_name}.json"] = _bind(_relay_json, profile_name)
    return producers


def _bind(func: Callable, *args: object) -> Callable[[], str]:
    def produce() -> str:
        return func(*args)

    return produce


_RTAB_CACHE: dict[tuple[str, str], tuple[str, str]] = {}


def _rtabmap_stream(profile_name: str, camera: str, index: int) -> str:
    key = (profile_name, camera)
    if key not in _RTAB_CACHE:
        _RTAB_CACHE[key] = _rtabmap_cli(profile_name, camera)
    return _RTAB_CACHE[key][index]


PRODUCERS = _producers()
_TEXT: dict[str, str] = {}


def snapshot_text(relative: str) -> str:
    if relative not in _TEXT:
        text = PRODUCERS[relative]()
        if not text.endswith("\n") or "\r" in text:
            raise RuntimeError(f"{relative} is not LF text ending in a newline")
        _TEXT[relative] = text
    return _TEXT[relative]


def setUpModule() -> None:
    if os.environ.get("UPDATE_GOLDEN") != "1":
        return
    root = FIXTURES.resolve()
    for relative in PRODUCERS:
        path = FIXTURES / relative
        if ".." in Path(relative).parts or not path.resolve().is_relative_to(root):
            raise RuntimeError(f"refusing to write outside the golden fixtures: {relative}")
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(snapshot_text(relative), encoding="utf-8")


class GoldenSnapshotTest(unittest.TestCase):
    def assert_fixture(self, relative: str, actual: str) -> None:
        path = FIXTURES / relative
        if not path.is_file():
            self.fail(
                f"missing golden fixture {relative}. "
                "Regenerate with UPDATE_GOLDEN=1 python3 includes/gz/oakd_s2/test_golden.py"
            )
        expected = path.read_text(encoding="utf-8")
        if expected == actual:
            return
        diff = "".join(
            difflib.unified_diff(
                expected.splitlines(keepends=True),
                actual.splitlines(keepends=True),
                fromfile=f"fixtures/golden/{relative}",
                tofile="current output",
            )
        )
        self.fail(f"snapshot {relative} changed:\n{diff}")

    def test_fixture_files_match_the_snapshot_set(self) -> None:
        expected = sorted(PRODUCERS)
        if not FIXTURES.is_dir():
            self.fail(
                "missing fixtures/golden. "
                "Regenerate with UPDATE_GOLDEN=1 python3 includes/gz/oakd_s2/test_golden.py"
            )
        found = sorted(
            path.relative_to(FIXTURES).as_posix() for path in FIXTURES.rglob("*") if path.is_file()
        )
        self.assertEqual(found, expected)

    def test_rtabmap_fixtures_use_a_portable_ini_path(self) -> None:
        for relative in PRODUCERS:
            if not relative.startswith("rtabmap/"):
                continue
            with self.subTest(relative=relative):
                text = snapshot_text(relative)
                self.assertNotIn(str(REPO), text)
                self.assertIn("rtabmap_profiles/", text)

    def test_zzz_checked_in_models_were_not_rewritten(self) -> None:
        after = {path: _sha256(path) for path in CHECKED_IN}
        self.assertEqual(after, _CHECKED_IN_BEFORE)


def _make_test(relative: str):
    def test(self: GoldenSnapshotTest) -> None:
        self.assert_fixture(relative, snapshot_text(relative))

    test.__name__ = "test_" + relative.replace("/", "_").replace(".", "_").replace("-", "_")
    test.__doc__ = f"golden {relative}"
    return test


for _relative in PRODUCERS:
    _method = _make_test(_relative)
    setattr(GoldenSnapshotTest, _method.__name__, _method)


if __name__ == "__main__":
    unittest.main()
