"""Static auxiliary geometry must agree with the physical SDF."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from panda_manipulation.scene_manager import static_auxiliary_obstacle


WORLD = Path(__file__).resolve().parents[1] / "worlds/panda_table.sdf"


def test_auxiliary_geometry_matches_world_frame():
    obstacle, = static_auxiliary_obstacle(str(WORLD))
    assert obstacle["pose_xyz"] == pytest.approx([0.37, -0.24, 0.15])
    assert obstacle["size"] == pytest.approx([0.12, 0.12, 0.30])


@pytest.mark.parametrize("pose", ["0 0 0 0 0 0.5", "nan 0 0 0 0 0"])
def test_unsupported_pose_fails_closed(tmp_path, pose):
    tree = ET.parse(WORLD)
    tree.getroot().find("./world/model[@name='dynamic_obstacle']/pose").text = pose
    world = tmp_path / "world.sdf"
    tree.write(world)
    with pytest.raises(ValueError):
        static_auxiliary_obstacle(str(world))


def test_absent_auxiliary_model_does_not_invent_geometry(tmp_path):
    tree = ET.parse(WORLD)
    world = tree.getroot().find("world")
    world.remove(world.find("model[@name='dynamic_obstacle']"))
    path = tmp_path / "world.sdf"
    tree.write(path)
    assert static_auxiliary_obstacle(str(path)) == []
