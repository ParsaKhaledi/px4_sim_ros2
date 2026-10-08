"""Patch the OakD-Lite include pose in PX4's x500_depth model."""

from __future__ import annotations

import re
from pathlib import Path

from frames import Mount

POSE_RE = re.compile(r"(<pose\b[^>]*>)(.*?)(</pose>)", re.DOTALL)


def _replace_first_pose(block: str, pose: str) -> str:
    """Replace the first pose, or insert one before the closing tag."""
    if POSE_RE.search(block):
        return POSE_RE.sub(lambda match: match.group(1) + pose + match.group(3), block, count=1)
    close = block.rfind("</")
    if close < 0:
        raise ValueError("no closing tag to attach a pose to")
    return block[:close] + f"<pose>{pose}</pose>\n" + block[close:]


def _block_span(xml: str, start_tag: str, end_tag: str, needle: str) -> tuple[int, int]:
    """Span of the first start/end block whose text contains ``needle``."""
    cursor = 0
    while True:
        start = xml.find(start_tag, cursor)
        if start < 0:
            raise ValueError(f"no {start_tag} containing {needle}")
        end = xml.find(end_tag, start)
        if end < 0:
            raise ValueError(f"unclosed {start_tag}")
        end += len(end_tag)
        if needle in xml[start:end]:
            return start, end
        cursor = end


def patch_x500_depth_xml(xml: str, mount: Mount | None = None) -> str:
    """Set the OakD-Lite include pose and the camera_link joint to ``mount``.

    The rest of the PX4 model is left as text. Only those two pose values change,
    so a Gazebo camera and the URDF camera_joint describe the same bracket.
    """
    mount = mount or Mount()
    pose = mount.pose_text()
    start, end = _block_span(xml, "<include", "</include>", "OakD-Lite")
    xml = xml[:start] + _replace_first_pose(xml[start:end], pose) + xml[end:]
    start, end = _block_span(xml, "<joint", "</joint>", "camera_link")
    xml = xml[:start] + _replace_first_pose(xml[start:end], pose) + xml[end:]
    return xml


def patch_x500_depth_file(path: Path, mount: Mount | None = None) -> str:
    """Write the patched x500_depth XML and return the pose text."""
    file_path = Path(path)
    original = file_path.read_text(encoding="utf-8")
    updated = patch_x500_depth_xml(original, mount)
    file_path.write_text(updated, encoding="utf-8")
    return (mount or Mount()).pose_text()
