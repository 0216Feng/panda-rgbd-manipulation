"""Request-local nominal payload geometry for pre-grasp placement screening."""

from copy import deepcopy
from math import isfinite


def target_box_dimensions(scene):
    objects = [obj for obj in scene.world.collision_objects if obj.id == "target_cube"]
    if len(objects) != 1:
        raise ValueError("expected one target_cube in the planning scene")
    obj = objects[0]
    if (len(obj.primitives) != 1 or obj.meshes or obj.planes
            or obj.primitives[0].type != obj.primitives[0].BOX):
        raise ValueError("placement precheck supports a single box target only")
    dimensions = list(obj.primitives[0].dimensions)
    if len(dimensions) != 3 or not all(isfinite(v) and v > 0 for v in dimensions):
        raise ValueError("target box dimensions must be finite and positive")
    return dimensions


def placement_precheck_request(candidate, seed, dimensions, group, hand_link,
                               finger_position, clearance):
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
    from moveit_msgs.srv import GetPositionIK
    from shape_msgs.msg import SolidPrimitive
    from .pick_plan_pipeline import payload_compensated_place_pose
    from .target_object_manager import TOUCH_LINKS

    if seed is None or not seed.joint_state.name:
        raise ValueError("placement seed is unavailable")
    if len(dimensions) != 3 or not all(isfinite(v) and v > 0 for v in dimensions):
        raise ValueError("target box dimensions must be finite and positive")
    if not isfinite(finger_position) or not 0 <= finger_position <= 0.04:
        raise ValueError("finger position is outside Panda limits")
    if not isfinite(clearance) or clearance < 0:
        raise ValueError("invalid placement clearance")
    local = candidate["object_in_hand_pose"]
    if local["frame_id"] != hand_link:
        raise ValueError("payload transform does not use the requested hand frame")
    desired = deepcopy(candidate["place_pose"])
    if candidate["object_pose"]["frame_id"] != desired["frame_id"]:
        raise ValueError("object and placement frames differ")
    desired["position"]["z"] = candidate["object_pose"]["position"]["z"] + clearance
    hand_goal = payload_compensated_place_pose(desired, local)

    def pose(data):
        value = Pose()
        for key in ("x", "y", "z"):
            setattr(value.position, key, float(data["position"][key]))
        for key in ("x", "y", "z", "w"):
            setattr(value.orientation, key, float(data["orientation"][key]))
        components = [getattr(value.position, k) for k in ("x", "y", "z")]
        components += [getattr(value.orientation, k) for k in ("x", "y", "z", "w")]
        if not all(isfinite(v) for v in components):
            raise ValueError("nonfinite placement geometry")
        if abs(sum(v*v for v in components[3:]) - 1.0) > 1e-3:
            raise ValueError("placement quaternion must be normalized")
        return value

    request = GetPositionIK.Request()
    ik = request.ik_request
    ik.group_name = group
    ik.ik_link_name = hand_link
    ik.avoid_collisions = True
    ik.timeout.sec = 1
    ik.pose_stamped.header.frame_id = hand_goal["frame_id"]
    ik.pose_stamped.pose = pose(hand_goal)
    ik.robot_state = deepcopy(seed)
    ik.robot_state.is_diff = True
    joints = ik.robot_state.joint_state
    if len(joints.name) != len(joints.position) or not all(isfinite(v) for v in joints.position):
        raise ValueError("invalid seed joint positions")
    positions = dict(zip(joints.name, joints.position))
    positions.update(panda_finger_joint1=finger_position, panda_finger_joint2=finger_position)
    joints.name = list(positions)
    joints.position = list(positions.values())
    joints.velocity = []
    joints.effort = []
    payload = AttachedCollisionObject()
    payload.link_name = hand_link
    payload.touch_links = list(TOUCH_LINKS)
    payload.object.id = "target_cube"
    payload.object.header.frame_id = hand_link
    payload.object.operation = CollisionObject.ADD
    payload.object.pose.orientation.w = 1.0
    shape = SolidPrimitive()
    shape.type = SolidPrimitive.BOX
    shape.dimensions = [float(v) for v in dimensions]
    payload.object.primitives = [shape]
    payload.object.primitive_poses = [pose(local)]
    ik.robot_state.attached_collision_objects = [
        body for body in ik.robot_state.attached_collision_objects
        if body.object.id != payload.object.id
    ] + [payload]
    return request
