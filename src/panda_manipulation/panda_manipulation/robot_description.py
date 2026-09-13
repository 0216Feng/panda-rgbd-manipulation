"""Build a Gazebo Panda description with two physical finger actuators."""

from __future__ import annotations

import xml.etree.ElementTree as ET


def build_gazebo_robot_description(xacro_path: str) -> str:
    import xacro

    document = xacro.process_file(xacro_path)
    root = ET.fromstring(document.toxml())
    finger_joint = root.find("./joint[@name='panda_finger_joint2']")
    if finger_joint is None:
        raise RuntimeError("panda_finger_joint2 is missing from the generated Panda URDF")
    mimic = finger_joint.find("mimic")
    if mimic is None or mimic.attrib.get("joint") != "panda_finger_joint1":
        raise RuntimeError(
            "panda_finger_joint2 must mimic panda_finger_joint1 in the generated URDF"
        )
    # DART in Gazebo Harmonic cannot create this mimic constraint. Removing it
    # only from the spawned simulation model lets ros2_control command both
    # physical joints; MoveIt's Panda model retains the original mimic relation.
    finger_joint.remove(mimic)
    return ET.tostring(root, encoding="unicode")
