"""Flight grades that do not require another simulation run."""

from __future__ import annotations

from dataclasses import dataclass

# A vision hover this close to ground truth for every sample is not vision.
HOVER_SUSPECT_M = 0.001


@dataclass(frozen=True)
class HoverGrade:
    grade: str
    failed: bool


def grade_hover(estimation_mode: str, height_errors_m) -> HoverGrade:
    """Grade a hover by how tightly the EKF height follows ground truth.

    In vision mode, a hover whose every sample stays within 1 mm fails as
    ``suspect``: that match is ground truth being fused as vision. GPS mode
    is not graded this way. An empty window is unscored.
    """
    if estimation_mode.strip().lower() != 'vision':
        return HoverGrade('ok', False)
    errors = [abs(float(error)) for error in height_errors_m]
    if not errors:
        return HoverGrade('unscored', False)
    if max(errors) <= HOVER_SUSPECT_M:
        return HoverGrade('suspect', True)
    return HoverGrade('ok', False)
