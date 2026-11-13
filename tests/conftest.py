"""One pytest entry. Levels are unit, then flight. Stop at the first failure."""

from __future__ import annotations

import pytest


def pytest_addoption(parser):
    parser.addoption(
        "--level",
        choices=("unit", "flight", "all"),
        default="unit",
        help="unit (no simulator), flight (headless hover grade), or all",
    )


def pytest_collection_modifyitems(config, items):
    for item in items:
        if item.get_closest_marker("flight") is None:
            item.add_marker(pytest.mark.unit)

    level = config.getoption("--level")
    selected = []
    deselected = []
    for item in items:
        is_flight = item.get_closest_marker("flight") is not None
        keep = level == "all" or (level == "flight" and is_flight) or (level == "unit" and not is_flight)
        if keep:
            selected.append(item)
        else:
            deselected.append(item)
    selected.sort(key=lambda item: item.get_closest_marker("flight") is not None)
    items[:] = selected
    if deselected:
        config.hook.pytest_deselected(items=deselected)
