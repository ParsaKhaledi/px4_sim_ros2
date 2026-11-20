"""The control node must subscribe with the same reliability RTAB-Map publishes."""

from pathlib import Path

_NODE = Path(__file__).resolve().parents[2] / "ros2_ws/src/px4_control/px4_control/node.py"


def test_rtabmap_subscription_is_best_effort():
    text = _NODE.read_text(encoding="utf-8")
    # qos:=2 on the RTAB-Map launch is sensor data: best effort, volatile.
    assert "RTAB-Map is launched with qos:=2" in text
    assert "reliability=ReliabilityPolicy.BEST_EFFORT" in text
    vision = text.split("str(self.get_parameter('vision_odom_topic').value)", 1)[1]
    vision = vision.split("self._try_odom_info", 1)[0]
    assert "vision_qos" in vision
    assert "cmd_qos" not in vision
    info = text.split("def _try_odom_info", 1)[1]
    assert "BEST_EFFORT" in info
