"""``python -m flight_analysis`` entry point."""

from __future__ import annotations

from flight_analysis.cli import main


def _run() -> None:
    """Exit with the command's status code."""

    raise SystemExit(main())


if __name__ == "__main__":
    _run()
