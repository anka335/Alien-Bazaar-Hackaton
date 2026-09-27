"""run_spectacles_rover.sh: the morning start script (#56).

The script is run from a copy in a temporary directory, so there is no ros2_ws/install next to
it: even if the can0 guard were wrong, the script stops before sourcing a build or launching.
`ip` is stubbed on PATH, so the tests never depend on this machine's CAN state.
"""

import os
import shutil
import stat
import subprocess
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[4] / "run_spectacles_rover.sh"

COMMON = [
    "hardware:=real",
    "enable_motors:=true",
    "run_task:=false",
    "spectacles:=true",
    "use_rviz:=true",
    "spectacles_port:=9100",
]


def _write_exe(path: Path, text: str) -> None:
    path.write_text(text)
    path.chmod(path.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


@pytest.fixture
def run(tmp_path):
    """Run a copy of the script with `ip` reporting can0 as given (None: no can0)."""
    copy = tmp_path / "repo" / "run_spectacles_rover.sh"
    copy.parent.mkdir()
    shutil.copy(SCRIPT, copy)
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    # A ros2 that only leaves a trace, in case anything reached a launch.
    marker = tmp_path / "ros2_called"
    _write_exe(bin_dir / "ros2", f'#!/bin/sh\ntouch "{marker}"\nexit 97\n')

    def _run(*args, can0=None):
        if can0 is None:
            ip = "#!/bin/sh\necho 'Device \"can0\" does not exist.' >&2\nexit 1\n"
        else:
            ip = f"#!/bin/sh\necho 'can0             {can0}             '\n"
        _write_exe(bin_dir / "ip", ip)
        env = {k: v for k, v in os.environ.items() if not k.startswith("ROS_")}
        env["PATH"] = f"{bin_dir}:{env['PATH']}"
        result = subprocess.run(
            ["bash", str(copy), *args], env=env, capture_output=True, text=True, timeout=30
        )
        assert not marker.exists(), "the script reached ros2"
        return result

    return _run


@pytest.mark.parametrize(
    ("stage", "sim", "rover"),
    [("stage-a", "true", "true"), ("stage-b1", "false", "false"), ("stage-b2", "false", "true")],
)
def test_print_shows_the_full_command_and_launches_nothing(run, stage, sim, rover):
    result = run("--print", stage)
    assert result.returncode == 0, result.stderr
    command = next(line for line in result.stdout.splitlines() if "ros2 launch" in line)
    args = command.split()
    assert args[:4] == ["ros2", "launch", "cloth_task", "task.launch.py"]
    assert f"driver_sim:={sim}" in args
    assert f"rover:={rover}" in args
    for a in COMMON:
        assert a in args


def test_print_reports_the_default_domain_and_discovery_range(run):
    out = run("--print", "stage-a").stdout
    assert "ROS_DOMAIN_ID=unset" in out
    assert "ROS_AUTOMATIC_DISCOVERY_RANGE=unset" in out


@pytest.mark.parametrize("stage", ["stage-b1", "stage-b2"])
@pytest.mark.parametrize("can0", [None, "DOWN"])
def test_b_stages_refuse_without_can0_up(run, stage, can0):
    result = run(stage, can0=can0)
    assert result.returncode != 0
    assert "can0" in result.stderr
    assert "sudo ip link set can0 up" in result.stderr
    assert "ros2 launch" not in result.stdout


@pytest.mark.parametrize("stage", ["stage-b1", "stage-b2"])
def test_b_stages_pass_the_can0_guard_when_up(run, stage):
    # The copy has no ros2_ws/install, so it stops there, after the guard.
    result = run(stage, can0="UP")
    assert result.returncode != 0
    assert "sudo ip link" not in result.stderr
    assert "ros2_ws/install" in result.stderr


def test_stage_a_does_not_need_can0(run):
    result = run("stage-a")
    assert "sudo ip link" not in result.stderr
    assert "ros2_ws/install" in result.stderr


@pytest.mark.parametrize("args", [["stage-c"], [], ["--print"], ["stage-a", "extra"]])
def test_unknown_or_missing_stage_prints_usage(run, args):
    result = run(*args)
    assert result.returncode != 0
    assert "usage" in result.stderr.lower()
