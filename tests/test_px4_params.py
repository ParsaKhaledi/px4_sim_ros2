from pathlib import Path

import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

import check_px4_params  # noqa: E402
import px4_params  # noqa: E402


SAMPLE = """
Symbols: x = used, + = saved, * = unsaved
x   NAV_DLL_ACT [0, 7] : 0
x   NAV_RCL_ACT [0, 6] : 1
x   COM_RC_IN_MODE [0, 4] : 1
x   COM_RC_LOSS_T [0.000, 35.000] : 35.000
"""


def test_parse_uses_the_value_after_the_colon():
    assert check_px4_params.parse_param_value(SAMPLE, "NAV_DLL_ACT") == 0
    assert check_px4_params.parse_param_value(SAMPLE, "COM_RC_LOSS_T") == 35


def test_matching_params_pass():
    expected = px4_params.merge_expected(environ={})
    assert check_px4_params.mismatches(SAMPLE, expected) == []


def test_real_param_show_line_uses_the_value_after_the_colon():
    text = "x   NAV_DLL_ACT [646,1115] : 2"
    assert check_px4_params.parse_param_value(text, "NAV_DLL_ACT") == 2


def test_datalink_failsafe_left_at_return_fails():
    text = SAMPLE.replace("NAV_DLL_ACT [0, 7] : 0", "NAV_DLL_ACT [0, 7] : 2")
    expected = px4_params.merge_expected(environ={})
    problems = check_px4_params.mismatches(text, expected)
    assert any("NAV_DLL_ACT is 2" in problem for problem in problems)


def test_parse_file_and_env_override(tmp_path):
    (tmp_path / "headless.params").write_text("# note\nNAV_DLL_ACT 0\nNAV_RCL_ACT 1\n", encoding="utf-8")
    merged = px4_params.merge_expected(
        directory=tmp_path,
        environ={"PX4_PARAM_FILES": "headless.params", "PX4_PARAM_NAV_DLL_ACT": "0"},
    )
    assert merged["NAV_DLL_ACT"] == 0
    assert merged["NAV_RCL_ACT"] == 1


def test_repo_headless_file_parses():
    text = (px4_params.default_params_dir() / "headless.params").read_text(encoding="utf-8")
    parsed = px4_params.parse_params_text(text, "headless.params")
    assert parsed["NAV_DLL_ACT"] == "0"
    assert parsed["NAV_RCL_ACT"] == "1"
    assert parsed["COM_RC_IN_MODE"] == "1"
    assert parsed["COM_RC_LOSS_T"] == "35"


def test_two_files_disagree(tmp_path):
    (tmp_path / "a.params").write_text("NAV_DLL_ACT 0\n", encoding="utf-8")
    (tmp_path / "b.params").write_text("NAV_DLL_ACT 2\n", encoding="utf-8")
    try:
        px4_params.load_param_files(tmp_path, ["a.params", "b.params"])
    except px4_params.ParamConflict as exc:
        message = str(exc)
    else:
        raise AssertionError("conflict was accepted")
    assert "a.params" in message
    assert "b.params" in message


def test_env_override_replaces_the_file(tmp_path):
    (tmp_path / "headless.params").write_text("COM_RC_LOSS_T 35\n", encoding="utf-8")
    merged = px4_params.merge_expected(
        directory=tmp_path,
        environ={"PX4_PARAM_FILES": "headless.params", "PX4_PARAM_COM_RC_LOSS_T": "10"},
    )
    assert merged["COM_RC_LOSS_T"] == 10


def test_files_merge_in_listed_order(tmp_path):
    (tmp_path / "headless.params").write_text("NAV_DLL_ACT 0\n", encoding="utf-8")
    (tmp_path / "ekf2_vision.params").write_text("EKF2_EV_DELAY 0\n", encoding="utf-8")
    merged = px4_params.merge_expected(
        directory=tmp_path,
        environ={"PX4_PARAM_FILES": "headless.params,ekf2_vision.params", "PX4_PARAM_EKF2_EV_DELAY": "5"},
    )
    assert merged["NAV_DLL_ACT"] == 0
    assert merged["EKF2_EV_DELAY"] == 5


def test_post_is_written_for_every_airframe(tmp_path):
    airframes = tmp_path / "airframes"
    airframes.mkdir()
    (airframes / "4001_gz_x500").write_text("param set-default NAV_DLL_ACT 2\n", encoding="utf-8")
    (airframes / "4019_gz_x500_depth").write_text("param set-default NAV_DLL_ACT 2\n", encoding="utf-8")
    (airframes / "4020_gz_oak").write_text("param set-default NAV_DLL_ACT 2\n", encoding="utf-8")
    (airframes / "notes.txt").write_text("skip\n", encoding="utf-8")
    script = px4_params.render_post([("NAV_DLL_ACT", "0", "headless.params")], [])
    written = px4_params.write_posts(airframes, script)
    assert sorted(path.name for path in written) == [
        "4001_gz_x500.post",
        "4019_gz_x500_depth.post",
        "4020_gz_oak.post",
    ]
    assert "param set NAV_DLL_ACT 0" in (airframes / "4020_gz_oak.post").read_text(encoding="utf-8")


def test_start_script_does_not_edit_rcs():
    start = Path("includes/gz/startFiles/gz_start_px4_gz_sim.sh").read_text(encoding="utf-8")
    assert "px4_params.py apply" in start
    assert "--rcs" not in start
    assert "apply_px4_params.sh" not in start
    for name in ("scripts/smoke_test.sh", "scripts/run_e2e.sh"):
        assert "apply_px4_params.sh" not in Path(name).read_text(encoding="utf-8")


def test_unknown_metadata_name(tmp_path):
    (tmp_path / "headless.params").write_text("EKF2_EV_DLAY 5\n", encoding="utf-8")
    metadata = '<parameters><parameter name="NAV_DLL_ACT" type="INT32"/></parameters>'
    problems = px4_params.unknown_names(metadata, tmp_path)
    assert any("EKF2_EV_DLAY" in problem for problem in problems)


def test_env_override_rejects_a_bad_value():
    try:
        px4_params.env_overrides({"PX4_PARAM_NAV_DLL_ACT": "on"})
    except ValueError as exc:
        message = str(exc)
    else:
        raise AssertionError("bad value was accepted")
    assert "PX4_PARAM_NAV_DLL_ACT" in message


def test_timeout_runs_docker_not_the_compose_function():
    cli = Path("scripts/compose_cli.sh").read_text(encoding="utf-8")
    assert "timeout" in cli
    assert '"${COMPOSE[@]}" exec -T' in cli
    assert cli.count("< /dev/null") >= 2
    text = Path("scripts/assert_px4_params.sh").read_text(encoding="utf-8")
    assert "timeout 30 compose_exec" not in text
    assert "compose_exec_timeout" in text


def test_render_logs_an_override():
    script = px4_params.render_post([("NAV_DLL_ACT", "0", "headless.params")], [("NAV_RCL_ACT", "1")])
    assert 'echo "PX4 params: headless.params NAV_DLL_ACT 0"' in script
    assert 'echo "PX4 params: override NAV_RCL_ACT 1"' in script
    assert "exit 1" in script
