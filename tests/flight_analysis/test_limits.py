"""The shared limit loader has names only, and values live in env files."""

from __future__ import annotations

import re
import sys
import tempfile
import unittest
from dataclasses import MISSING
from dataclasses import fields
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from e2e_limits import E2ELimitError  # noqa: E402
from e2e_limits import E2ELimits  # noqa: E402
from e2e_limits import LIMIT_FIELDS  # noqa: E402
from e2e_limits import LIMIT_NAMES  # noqa: E402
from e2e_limits import limit_field_names  # noqa: E402
from e2e_limits import load_limits  # noqa: E402
from e2e_limits import parse_env_file  # noqa: E402


FIXTURE = Path(__file__).resolve().parent / "fixtures" / "e2e.env"
OVERRIDE = Path(__file__).resolve().parent / "fixtures" / "override.env"
TEXT_SUFFIXES = {
    ".py",
    ".md",
    ".sh",
    ".yml",
    ".yaml",
    ".txt",
    ".env",
    ".example",
    ".ini",
    ".cfg",
    ".json",
    ".toml",
    ".bash",
}


class LimitLoaderTests(unittest.TestCase):
    """Resolution order, validation, and the single home for values."""

    def test_dataclass_fields_match_the_names_and_have_no_defaults(self) -> None:
        """Each limit name has one field, and the class stores no numbers."""

        self.assertEqual(limit_field_names(), tuple(field for _name, field, _kind in LIMIT_FIELDS))
        for field in fields(E2ELimits):
            self.assertIs(field.default, MISSING)

    def test_fixture_file_loads_every_limit(self) -> None:
        """A complete env file is enough when the process environment is empty."""

        loaded = load_limits(environ={}, env_path=FIXTURE, example_path=Path("missing.env"))
        raw = parse_env_file(FIXTURE)
        for name, field, kind in LIMIT_FIELDS:
            expected = float(raw[name])
            if kind == "int":
                expected = int(expected)
            self.assertEqual(getattr(loaded, field), expected)

    def test_process_env_beats_dotenv_and_dotenv_beats_example(self) -> None:
        """The first source that has a value wins."""

        base = parse_env_file(FIXTURE)
        override = parse_env_file(OVERRIDE)
        name, raw = next(iter(override.items()))
        field = next(item[1] for item in LIMIT_FIELDS if item[0] == name)
        from_files = load_limits(environ={}, env_path=OVERRIDE, example_path=FIXTURE)
        self.assertEqual(getattr(from_files, field), float(raw))
        untouched = next(item for item in LIMIT_FIELDS if item[0] != name)
        self.assertEqual(getattr(from_files, untouched[1]), float(base[untouched[0]]))

        with tempfile.TemporaryDirectory() as folder:
            dotenv = Path(folder) / ".env"
            dotenv.write_text(FIXTURE.read_text(encoding="utf-8"), encoding="utf-8")
            from_process = load_limits(
                environ={name: raw},
                env_path=dotenv,
                example_path=FIXTURE,
            )
        self.assertEqual(getattr(from_process, field), float(raw))

    def test_missing_limit_names_the_key(self) -> None:
        """A gap in every source fails and includes that limit's name."""

        dropped = LIMIT_NAMES[0]
        kept = [
            line
            for line in FIXTURE.read_text(encoding="utf-8").splitlines()
            if not line.startswith(dropped + "=")
        ]
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "partial.env"
            path.write_text("\n".join(kept) + "\n", encoding="utf-8")
            with self.assertRaises(E2ELimitError) as caught:
                load_limits(environ={}, env_path=path, example_path=Path("missing.env"))
        self.assertIn(dropped, str(caught.exception))

    def test_limit_values_are_not_defined_elsewhere(self) -> None:
        """``.env.example`` is the only home for values outside the test fixture."""

        assignment = re.compile(
            r"(" + "|".join(re.escape(name) for name in LIMIT_NAMES) + r")\s*=\s*[-+]?\d"
        )
        default_arg = re.compile(
            r"""["'](""" + "|".join(re.escape(name) for name in LIMIT_NAMES) + r""")["']\s*,\s*["']?[-+]?\d"""
        )
        allowed = {ROOT / ".env.example", ROOT / "e2e_limits.py"}
        fixture_dir = FIXTURE.parent
        offenders: list[str] = []
        module_text = (ROOT / "e2e_limits.py").read_text(encoding="utf-8")
        self.assertIsNone(assignment.search(module_text))
        self.assertIsNone(default_arg.search(module_text))
        for path in ROOT.rglob("*"):
            if not path.is_file() or ".git" in path.parts:
                continue
            if path in allowed or fixture_dir in path.parents:
                continue
            if path.suffix not in TEXT_SUFFIXES and path.name not in {".env", ".env.example"}:
                continue
            if path.stat().st_size > 1_000_000:
                continue
            text = path.read_text(encoding="utf-8", errors="ignore")
            if assignment.search(text) or default_arg.search(text):
                offenders.append(str(path.relative_to(ROOT)))
        self.assertEqual(offenders, [])


if __name__ == "__main__":
    unittest.main()
