"""The start scripts must find includes/gz from the compose mount."""

import subprocess
from pathlib import Path

RESOLVER = Path(__file__).resolve().parents[1] / "startFiles" / "gz_resolve_dir.sh"


def _run(script: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["bash", "-c", script],
        check=False,
        capture_output=True,
        text=True,
    )


def test_script_dir_parent_wins_when_the_tree_is_there(tmp_path):
    gz = tmp_path / "includes" / "gz"
    scripts = gz / "scripts"
    scripts.mkdir(parents=True)
    (scripts / "sim_origin.py").write_text("# marker\n", encoding="utf-8")
    start = gz / "startFiles"
    start.mkdir()
    completed = _run(
        f'SCRIPT_DIR="{start}"\n'
        f'source "{RESOLVER}"\n'
        "gz_resolve_dir\n"
        'printf "%s" "$GZ_DIR"\n'
    )
    assert completed.returncode == 0, completed.stderr
    assert completed.stdout == str(gz.resolve())


def test_sim_gz_dir_override_and_missing_tree(tmp_path):
    gz = tmp_path / "gz"
    (gz / "scripts").mkdir(parents=True)
    (gz / "scripts" / "sim_origin.py").write_text("# marker\n", encoding="utf-8")
    start = tmp_path / "startFiles"
    start.mkdir()
    completed = _run(
        f'SCRIPT_DIR="{start}"\n'
        f'SIM_GZ_DIR="{gz}"\n'
        f'source "{RESOLVER}"\n'
        "gz_resolve_dir\n"
        'printf "%s" "$GZ_DIR"\n'
    )
    assert completed.returncode == 0, completed.stderr
    assert completed.stdout == str(gz.resolve())

    missing = _run(
        f'SCRIPT_DIR="{start}"\n'
        f'source "{RESOLVER}"\n'
        "gz_resolve_dir\n"
    )
    assert missing.returncode != 0
    assert "cannot find includes/gz" in missing.stderr