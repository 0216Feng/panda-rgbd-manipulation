"""Exercise the nested ROS methods without starting a ROS executor."""

import ast
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from panda_manipulation.pick_plan_pipeline import clamp_near_joint_limits


class State:
    def __init__(self):
        self.joint_state = NS(header=NS(stamp=None), name=[], position=[])
        self.is_diff = False


@pytest.mark.parametrize("name", ["measured_robot_state", "end_state_from_trajectory", "fixed_home_state"])
def test_joint_only_state_preserves_scene_payload(name):
    path = Path(__file__).resolve().parents[1] / "panda_manipulation/pick_plan_pipeline.py"
    tree = ast.parse(path.read_text())
    method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
    module = ast.parse("from __future__ import annotations")
    module.body.append(method)
    namespace = {"RobotState": State, "clamp_near_joint_limits": clamp_near_joint_limits}
    exec(compile(module, str(path), "exec"), namespace)
    observer = NS(execute_trajectories=True, visualize_only_execution=False,
                  latest_joint_state=NS(name=["panda_joint1"], position=[0.1]),
                  get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: None)))
    arguments = [observer]
    if name == "end_state_from_trajectory":
        arguments.append(NS(joint_trajectory=NS(joint_names=["panda_joint1"],
                                               points=[NS(positions=[0.1])])))
    state = namespace[name](*arguments)
    assert state.is_diff is True
    assert state.joint_state.name
    assert len(state.joint_state.position) == len(state.joint_state.name)
