#!/usr/bin/env python3
"""Add PX4's single-ray downward lidar to the x500_depth model.

The block matches PX4 v1.17 ``x500_lidar_down``: an LW20 include, the two
fixed joints, and ``lidar_sensor_link`` with sensor ``lidar``. Those names
are what GZBridge subscribes to
(``/world/<world>/model/<model>/link/lidar_sensor_link/sensor/lidar/scan``).
The link pose ``0 0 -0.05 0 1.57 0`` and the sensor pose ``0 0 0 3.14 0 0``
are the pair PX4 treats as downward, world orientation quaternion (0, 1, 0, 0).

``LIDAR_DOWN=1`` (the default) inserts the block. ``LIDAR_DOWN=0`` removes it.
``LIDAR_DOWN_RATE_HZ`` (default 30) is the sensor ``update_rate``. The edit is
idempotent and only runs when ``x500_depth/model.sdf`` is present.
"""

from __future__ import annotations

import os
import re
import sys
from pathlib import Path

from patch_x500_ground_truth import candidate_models

DEFAULT_RATE_HZ = 30.0

_LINK = re.compile(
    r"<link\b[^>]*\bname=\"lidar_sensor_link\"[^>]*>.*?</link>",
    re.DOTALL,
)
_UPDATE_RATE = re.compile(r"<update_rate>\s*[^<]*\s*</update_rate>")
_REMOVALS = (
    re.compile(
        r"[ \t]*<include\b[^>]*>\s*<uri>\s*model://LW20\s*</uri>.*?</include>[ \t]*\n?",
        re.DOTALL,
    ),
    re.compile(
        r"[ \t]*<joint\b[^>]*\bname=\"lidar_model_joint\"[^>]*>.*?</joint>[ \t]*\n?",
        re.DOTALL,
    ),
    re.compile(
        r"[ \t]*<joint\b[^>]*\bname=\"lidar_sensor_joint\"[^>]*>.*?</joint>[ \t]*\n?",
        re.DOTALL,
    ),
    re.compile(
        r"[ \t]*<link\b[^>]*\bname=\"lidar_sensor_link\"[^>]*>.*?</link>[ \t]*\n?",
        re.DOTALL,
    ),
)


def lidar_enabled(environ: dict[str, str] | None = None) -> bool:
    """True unless ``LIDAR_DOWN`` is 0, false, or no. Unset means on."""
    env = os.environ if environ is None else environ
    raw = env.get("LIDAR_DOWN", "1").strip().lower()
    if raw == "":
        raw = "1"
    return raw in {"1", "true", "yes"}


def lidar_rate_hz(environ: dict[str, str] | None = None) -> float:
    """Sensor update rate. Unset or blank uses 30 Hz."""
    env = os.environ if environ is None else environ
    raw = env.get("LIDAR_DOWN_RATE_HZ", "").strip()
    if raw == "":
        return DEFAULT_RATE_HZ
    return float(raw)


def format_rate(rate_hz: float) -> str:
    """SDF ``update_rate`` text. Whole numbers drop the decimal."""
    if rate_hz == int(rate_hz):
        return str(int(rate_hz))
    return str(rate_hz)


def lidar_fragment(rate_hz: float) -> str:
    """SDF inserted into x500_depth. One horizontal sample and one vertical sample."""
    rate = format_rate(rate_hz)
    return f"""\
    <include merge='true'>
      <uri>model://LW20</uri>
      <pose relative_to="base_link">0 0 -0.079 0 1.57 0</pose>
    </include>
    <joint name="lidar_model_joint" type="fixed">
      <parent>base_link</parent>
      <child>lw20_link</child>
      <pose relative_to="base_link">-0 0 0 0 0 0</pose>
    </joint>
    <joint name="lidar_sensor_joint" type="fixed">
      <parent>base_link</parent>
      <child>lidar_sensor_link</child>
    </joint>
    <link name="lidar_sensor_link">
      <pose relative_to="base_link">0 0 -0.05 0 1.57 0</pose>
      <inertial>
        <mass>0.001</mass>
        <inertia>
          <ixx>0.00001</ixx>
          <iyy>0.00001</iyy>
          <izz>0.00001</izz>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyz>0.0</iyz>
        </inertia>
      </inertial>
      <sensor name='lidar' type='gpu_lidar'>
        <gz_frame_id>lidar_sensor_link</gz_frame_id>
        <pose>0 0 0 3.14 0 0</pose>
        <update_rate>{rate}</update_rate>
        <ray>
          <scan>
            <horizontal>
              <samples>1</samples>
              <resolution>1</resolution>
              <min_angle>0</min_angle>
              <max_angle>0</max_angle>
            </horizontal>
            <vertical>
              <samples>1</samples>
              <resolution>1</resolution>
              <min_angle>0</min_angle>
              <max_angle>0</max_angle>
            </vertical>
          </scan>
          <range>
            <min>0.1</min>
            <max>100.0</max>
            <resolution>0.01</resolution>
          </range>
        </ray>
        <always_on>1</always_on>
        <visualize>false</visualize>
      </sensor>
    </link>
"""


def _has_lidar(model_xml: str) -> bool:
    return "lidar_sensor_link" in model_xml or "model://LW20" in model_xml


def _set_update_rate(model_xml: str, rate_hz: float) -> str:
    link = _LINK.search(model_xml)
    if link is None:
        return model_xml
    block = _UPDATE_RATE.sub(
        f"<update_rate>{format_rate(rate_hz)}</update_rate>",
        link.group(0),
        count=1,
    )
    return model_xml[: link.start()] + block + model_xml[link.end() :]


def _remove_lidar_elements(model_xml: str) -> str:
    text = model_xml
    for pattern in _REMOVALS:
        text = pattern.sub("", text)
    return text


def patch_text(model_xml: str, environ: dict[str, str] | None = None) -> str:
    """Insert or remove the downward lidar. A second call with the same env is a no-op."""
    env = os.environ if environ is None else environ
    enabled = lidar_enabled(env)
    rate_hz = lidar_rate_hz(env)
    fragment = lidar_fragment(rate_hz)
    if enabled and fragment in model_xml:
        return model_xml
    if not enabled and fragment in model_xml:
        return model_xml.replace(fragment, "", 1)
    if _has_lidar(model_xml):
        if enabled:
            updated = _set_update_rate(model_xml, rate_hz)
            if fragment in updated:
                return updated
        cleaned = _remove_lidar_elements(model_xml)
        if not enabled:
            return cleaned
        model_xml = cleaned
    elif not enabled:
        return model_xml
    close = model_xml.rfind("</model>")
    if close < 0:
        raise ValueError("model.sdf has no </model> element")
    return model_xml[:close] + fragment + model_xml[close:]


def patch_file(path: Path, environ: dict[str, str] | None = None) -> bool:
    """Patch one model file. Return True when the file changed."""
    original = path.read_text(encoding="utf-8")
    updated = patch_text(original, environ)
    if updated == original:
        state = "absent" if not lidar_enabled(environ) else "present"
        print(f"downward lidar already {state}: {path}")
        return False
    path.write_text(updated, encoding="utf-8")
    action = "removed" if not lidar_enabled(environ) else "added"
    print(f"{action} downward lidar on {path}")
    return True


def main() -> int:
    """Insert or remove the downward lidar on each present x500_depth model."""
    found = False
    for path in candidate_models():
        if path.is_file():
            patch_file(path)
            found = True
    if not found:
        print(
            "x500_depth model.sdf not found; downward lidar was not applied. "
            "This is expected outside the PX4 SITL container.",
            file=sys.stderr,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
