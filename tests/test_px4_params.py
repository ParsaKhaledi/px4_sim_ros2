from pathlib import Path

import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

import check_px4_params  # noqa: E402


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
    assert check_px4_params.mismatches(SAMPLE) == []


def test_datalink_failsafe_left_at_return_fails():
    text = SAMPLE.replace("NAV_DLL_ACT [0, 7] : 0", "NAV_DLL_ACT [0, 7] : 2")
    problems = check_px4_params.mismatches(text)
    assert any("NAV_DLL_ACT is 2" in problem for problem in problems)
