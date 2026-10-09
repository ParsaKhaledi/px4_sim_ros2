import os
import subprocess
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


def _config(*files, profiles=()):
    command = ["docker", "compose"]
    for name in files:
        command.extend(["-f", str(ROOT / name)])
    for profile in profiles:
        command.extend(["--profile", profile])
    command.append("config")
    completed = subprocess.run(command, check=True, capture_output=True, text=True, cwd=ROOT)
    return yaml.safe_load(completed.stdout)


def _command_text(service):
    command = service.get("command") or []
    return " ".join(str(part) for part in command)


def test_no_compose_file_sets_a_container_name():
    files = list(ROOT.glob("compose*.yml")) + list(ROOT.glob("docker-compose*.yml"))
    assert files
    for path in files:
        for line in path.read_text(encoding="utf-8").splitlines():
            stripped = line.split("#", 1)[0]
            assert "container_name:" not in stripped, path


def test_every_compose_file_parses():
    variants = [
        ("compose.yml",),
        ("compose.yml", "compose.gpu.yml"),
        ("compose.yml", "compose.gui.yml"),
        ("compose.yml", "compose.ci.yml"),
        ("compose.yml", "compose.ci.yml", "compose.xvfb.yml"),
        ("compose.yml", "compose.gpu.yml", "compose.gui.yml"),
        ("docker-compose-px4.yml",),
        ("docker-compose-px4-GPU.yml",),
    ]
    for files in variants:
        rendered = _config(*files)
        assert "PX4" in rendered["services"]
        for service in rendered["services"].values():
            assert "container_name" not in service


def test_gpu_and_cpu_stacks_match_apart_from_gpu():
    profiles = ("gcs", "slam", "nav")
    cpu = _config("docker-compose-px4.yml", profiles=profiles)
    gpu = _config("docker-compose-px4-GPU.yml", profiles=profiles)
    assert set(cpu["services"]) == set(gpu["services"])
    for name in cpu["services"]:
        assert "MicroXRCEAgent" in _command_text(cpu["services"]["PX4"])
        assert "healthcheck.py" in " ".join(cpu["services"][name]["healthcheck"]["test"])
        assert cpu["services"][name]["environment"]["PX4_GZ_MODEL_POSE"] == "-3,-1.6,0,0,0,3.14"
        assert gpu["services"][name]["environment"]["PX4_GZ_MODEL_POSE"] == "-3,-1.6,0,0,0,3.14"
        assert "nvidia" not in cpu["services"][name].get("deploy", {}).get("resources", {}).get("reservations", {}).get("devices", [{}])[0].get("driver", "")
    assert gpu["services"]["PX4"]["deploy"]["resources"]["reservations"]["devices"][0]["driver"] == "nvidia"
    assert gpu["services"]["PX4"]["image"].endswith("1.17.0_01_GPU_GPUEnabled")


def test_ci_override_is_headless():
    rendered = _config("compose.yml", "compose.ci.yml", profiles=("slam", "nav", "gcs"))
    assert set(rendered["services"]) == {"PX4", "StatePublisher", "Rtabmap", "NAV2"}
    for name, service in rendered["services"].items():
        env = service["environment"]
        assert str(env["HEADLESS"]) == "1"
        assert str(env["RTABMAPVIZ"]) == "false"
        assert env["DISPLAY"] == ""
        assert str(env["GZ_HEADLESS_RENDERING"]) == "0"
        assert str(env["GZ_USE_XVFB"]) == "0"
        targets = []
        for volume in service.get("volumes") or []:
            if isinstance(volume, str):
                targets.append(volume)
            else:
                targets.append(volume.get("source", ""))
                targets.append(volume.get("target", ""))
        assert not any(item in ("/dev", "/dev/", "/tmp/.X11-unix") or str(item).endswith("/dev") for item in targets)
    px4_env = rendered["services"]["PX4"]["environment"]
    assert str(px4_env["PX4_PARAM_FILES"]) == "sim.params"
    px4_volumes = " ".join(
        volume if isinstance(volume, str) else volume.get("target", "")
        for volume in rendered["services"]["PX4"]["volumes"]
    )
    assert "/home/px4/volume/config/px4/params" in px4_volumes
    rtab = rendered["services"]["Rtabmap"]
    rtab_cmd = " ".join(rtab["healthcheck"]["test"])
    assert "/home/px4/volume/HealthCheck/healthcheck.py" in rtab_cmd
    rtab_volumes = " ".join(
        volume if isinstance(volume, str) else f"{volume.get('source', '')}:{volume.get('target', '')}"
        for volume in rtab["volumes"]
    )
    assert "/home/px4/volume/HealthCheck" in rtab_volumes


CAMERA_ENV = {
    # Unset stays closed. CPU and GPU files both default this way.
    "RTABMAPVIZ": "false",
    "CameraType": "rgbd",
    "CAM_PITCH_DEG": "17",
    "CAM_X": "0.12",
    "CAM_Y": "0.03",
    "CAM_Z": "0.242",
}

OPTIONAL_VISION_ENV = (
    "CAM_RATE_HZ",
    "CAM_STEREO_WIDTH",
    "CAM_STEREO_HEIGHT",
    "CAM_COLOR_WIDTH",
    "CAM_COLOR_HEIGHT",
    "IMU_RATE_HZ",
    "VISION_MIN_ODOM_HZ",
)


def _config_env(*files, profiles=(), env=None):
    command = ["docker", "compose"]
    for name in files:
        command.extend(["-f", str(ROOT / name)])
    for profile in profiles:
        command.extend(["--profile", profile])
    command.append("config")
    completed = subprocess.run(
        command,
        check=True,
        capture_output=True,
        text=True,
        cwd=ROOT,
        env=env,
    )
    return yaml.safe_load(completed.stdout)


def test_cpu_and_gpu_pass_the_same_camera_env():
    cpu_stacks = (("docker-compose-px4.yml",), ("compose.yml",))
    gpu_stacks = (("docker-compose-px4-GPU.yml",), ("compose.yml", "compose.gpu.yml"))
    for files in cpu_stacks + gpu_stacks:
        rendered = _config(*files, profiles=("slam",))
        for name in ("PX4", "StatePublisher", "Rtabmap"):
            env = rendered["services"][name]["environment"]
            for key, value in CAMERA_ENV.items():
                assert str(env[key]) == value, (files, name, key, env.get(key))
            for key in OPTIONAL_VISION_ENV:
                assert env.get(key) in (None, ""), (files, name, key, env.get(key))
    for files in cpu_stacks:
        assert _config(*files)["services"]["PX4"]["environment"]["VISION_PROFILE"] == "cpu"
    for files in gpu_stacks:
        assert _config(*files)["services"]["PX4"]["environment"]["VISION_PROFILE"] == "full"


def test_camera_env_follows_the_shell():
    env = os.environ.copy()
    env.update({
        "CAM_PITCH_DEG": "21",
        "CAM_X": "0.2",
        "CAM_Y": "0.04",
        "CAM_Z": "0.3",
        "RTABMAPVIZ": "false",
        "CameraType": "stereo",
        "VISION_PROFILE": "cpu",
        "CAM_RATE_HZ": "10",
        "CAM_COLOR_WIDTH": "320",
        "VISION_MIN_ODOM_HZ": "5",
    })
    for files in (("docker-compose-px4.yml",), ("docker-compose-px4-GPU.yml",)):
        rendered = _config_env(*files, profiles=("slam",), env=env)
        px4 = rendered["services"]["PX4"]["environment"]
        assert str(px4["CAM_PITCH_DEG"]) == "21"
        assert str(px4["CAM_X"]) == "0.2"
        assert str(px4["RTABMAPVIZ"]) == "false"
        assert px4["CameraType"] == "stereo"
        assert px4["VISION_PROFILE"] == "cpu"
        assert str(px4["CAM_RATE_HZ"]) == "10"
        assert str(px4["CAM_COLOR_WIDTH"]) == "320"
        assert str(px4["VISION_MIN_ODOM_HZ"]) == "5"
        assert px4.get("CAM_STEREO_WIDTH") in (None, "")
        publisher = rendered["services"]["StatePublisher"]["environment"]
        assert str(publisher["CAM_Z"]) == "0.3"
        assert publisher["VISION_PROFILE"] == "cpu"


def test_xvfb_override_replaces_the_empty_display():
    rendered = _config("compose.yml", "compose.ci.yml", "compose.xvfb.yml")
    env = rendered["services"]["PX4"]["environment"]
    assert env["DISPLAY"] == ":99"
    assert str(env["GZ_USE_XVFB"]) == "1"
    assert str(env["GZ_HEADLESS_RENDERING"]) == "0"
    assert str(env["LIBGL_ALWAYS_SOFTWARE"]) == "1"
    assert env["GALLIUM_DRIVER"] == "llvmpipe"
