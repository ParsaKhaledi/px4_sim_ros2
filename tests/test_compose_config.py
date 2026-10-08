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


def test_every_compose_file_parses():
    variants = [
        ("compose.yml",),
        ("compose.yml", "compose.gpu.yml"),
        ("compose.yml", "compose.gui.yml"),
        ("compose.yml", "compose.ci.yml"),
        ("compose.yml", "compose.gpu.yml", "compose.gui.yml"),
        ("docker-compose-px4.yml",),
        ("docker-compose-px4-GPU.yml",),
    ]
    for files in variants:
        rendered = _config(*files)
        assert "PX4" in rendered["services"]


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
        targets = []
        for volume in service.get("volumes") or []:
            if isinstance(volume, str):
                targets.append(volume)
            else:
                targets.append(volume.get("source", ""))
                targets.append(volume.get("target", ""))
        assert not any(item in ("/dev", "/dev/", "/tmp/.X11-unix") or str(item).endswith("/dev") for item in targets)
