#!/usr/bin/env python3
"""Read back the auxiliary box from MoveIt without commanding robot motion."""

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time

from check_ros_graph_smoke import stop_launch


def verify_payload_diff(node, executor, expected):
    from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
    from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Pose

    def call(client, request):
        deadline = time.monotonic() + 15
        while not client.service_is_ready() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        if not client.service_is_ready():
            raise RuntimeError("payload probe service unavailable")
        future = client.call_async(request)
        while not future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        if not future.done():
            raise RuntimeError("payload probe service timed out")
        return future.result()

    apply_client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    validity_client = node.create_client(GetStateValidity, "/check_state_validity")
    request = ApplyPlanningScene.Request()
    request.scene.is_diff = True
    request.scene.robot_state.is_diff = True
    payload = AttachedCollisionObject()
    payload.link_name = "panda_link0"
    payload.touch_links = ["panda_link0"]
    payload.object.header.frame_id = "panda_link0"
    payload.object.id = "diagnostic_payload_probe"
    payload.object.operation = CollisionObject.ADD
    shape = SolidPrimitive()
    shape.type = SolidPrimitive.BOX
    shape.dimensions = [0.06, 0.06, 0.06]
    pose = Pose()
    pose.orientation.w = 1.0
    pose.position.x, pose.position.y, pose.position.z = expected["pose_xyz"]
    payload.object.primitives = [shape]
    payload.object.primitive_poses = [pose]
    request.scene.robot_state.attached_collision_objects = [payload]
    if not call(apply_client, request).success:
        raise RuntimeError("test payload attachment rejected")
    observations = {}
    for is_diff in (True, False):
        check = GetStateValidity.Request()
        check.robot_state.is_diff = is_diff
        # A full state with no joints is rejected before attached-body handling.
        check.robot_state.joint_state.name = ["panda_joint1"]
        check.robot_state.joint_state.position = [0.0]
        response = call(validity_client, check)
        pairs = [[c.contact_body_1, c.contact_body_2] for c in response.contacts]
        observations[str(is_diff)] = {
            "valid": response.valid, "contacts": pairs,
            "payload_obstacle_contact": any(
                set(pair) == {"diagnostic_payload_probe", expected["id"]}
                for pair in pairs),
        }
    observations["passed"] = (
        observations["True"]["payload_obstacle_contact"]
        and not observations["False"]["payload_obstacle_contact"]
    )
    return observations


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--check-payload-diff", action="store_true")
    parser.add_argument("--timeout-s", type=float, default=90)
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/runs/static_scene_check"))
    args = parser.parse_args()
    if not math.isfinite(args.timeout_s) or args.timeout_s <= 0:
        parser.error("timeout must be finite and positive")
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from moveit_msgs.srv import GetPlanningScene
    from moveit_msgs.msg import PlanningSceneComponents
    from panda_manipulation.scene_manager import static_auxiliary_obstacle

    world = args.world.resolve(strict=True)
    expected, = static_auxiliary_obstacle(str(world))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    attempt = Path(tempfile.mkdtemp(prefix="attempt_", dir=args.output_dir)).resolve()
    domain = 200 + os.getpid() % 30
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node("static_scene_observer", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    client = node.create_client(GetPlanningScene, "/get_planning_scene")
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain),
               GZ_PARTITION=f"static_scene_{os.getpid()}", ROS_LOG_DIR=str(attempt / "ros_logs"))
    result = {"final_state": "FAILED", "physical_execution_verified": False,
              "scope": "moveit_scene_readback", "expected": expected,
              "reason": "scene readback timed out", "ros_domain_id": domain}
    process = None
    started = time.monotonic()
    try:
        with (attempt / "launch.log").open("w") as log:
            process = subprocess.Popen(
                ["ros2", "launch", "panda_manipulation", "moveit_gazebo.launch.py",
                 "gui:=false", "rviz:=false", "run_smoke_test:=false",
                 f"world:={world}", f"static_environment_world:={world}"],
                stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
            future = None
            while time.monotonic() - started < args.timeout_s:
                executor.spin_once(timeout_sec=0.1)
                if process.poll() is not None:
                    result["reason"] = f"launch exited: {process.returncode}"
                    break
                if future is None and client.service_is_ready():
                    request = GetPlanningScene.Request()
                    request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                    future = client.call_async(request)
                if future is not None and future.done():
                    response = future.result()
                    future = None
                    for obj in response.scene.world.collision_objects:
                        if obj.id != expected["id"] or len(obj.primitives) != 1 or len(obj.primitive_poses) != 1:
                            continue
                        point = obj.primitive_poses[0].position
                        origin = obj.pose.position
                        rotations = [obj.pose.orientation, obj.primitive_poses[0].orientation]
                        axis_aligned = all(
                            abs(q.x) < 1e-6 and abs(q.y) < 1e-6 and abs(q.z) < 1e-6
                            and abs(abs(q.w) - 1.0) < 1e-6 for q in rotations
                        )
                        actual = {"frame": obj.header.frame_id,
                                  "size": list(obj.primitives[0].dimensions),
                                  "pose_xyz": [point.x + origin.x, point.y + origin.y, point.z + origin.z],
                                  "axis_aligned": axis_aligned,
                                  "primitive_type": obj.primitives[0].type}
                        result["observed"] = actual
                        expected_xyz = list(expected["pose_xyz"])
                        if actual["frame"] == "world":
                            expected_xyz[2] += 0.72
                        if (actual["frame"] in ("panda_link0", "world")
                                and axis_aligned and obj.primitives[0].type == obj.primitives[0].BOX
                                and all(abs(a-b) < 1e-6 for a,b in zip(actual["size"], expected["size"]))
                                and len(actual["size"]) == 3
                                and all(abs(a-b) < 1e-6 for a,b in zip(actual["pose_xyz"], expected_xyz))):
                            result.update(final_state="SUCCESS", reason="MoveIt contains expected auxiliary box")
                    if result["final_state"] == "SUCCESS":
                        if args.check_payload_diff:
                            probe = verify_payload_diff(node, executor, expected)
                            result["payload_diff_probe"] = probe
                            if not probe["passed"]:
                                result.update(final_state="FAILED", reason="payload diff semantics not verified")
                        break
    except Exception as exc:
        result["final_state"] = "FAILED"
        result["reason"] = f"{type(exc).__name__}: {exc}"
    finally:
        try:
            if process is not None:
                stop_launch(process)
        finally:
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)
    result["elapsed_s"] = time.monotonic() - started
    (attempt / "result.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)
    print(f"Artifacts: {attempt}", flush=True)
    return 0 if result["final_state"] == "SUCCESS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
