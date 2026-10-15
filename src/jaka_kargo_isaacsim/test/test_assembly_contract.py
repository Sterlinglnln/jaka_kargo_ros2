"""Repository contracts that require neither Isaac nor a GPU."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT / "src/jaka_kargo_moveit_config/config"
SCENE = json.loads((ROOT / "src/jaka_kargo_isaacsim/config/assembly_scene.json").read_text())
URDF = ET.parse(ROOT / "src/jaka_kargo_description/urdf/jaka_kargo.urdf").getroot()


def test_controller_exclusivity_and_joint_order():
    ros = yaml.safe_load((CONFIG / "assembly_ros2_controllers.yaml").read_text())
    moveit = yaml.safe_load((CONFIG / "assembly_moveit_controllers.yaml").read_text())
    name = "jaka_kargo_full_robot_controller"
    assert set(ros["controller_manager"]["ros__parameters"]) == {
        "update_rate", "use_sim_time", name, "joint_state_broadcaster"}
    controllers = moveit["moveit_simple_controller_manager"]
    assert controllers["controller_names"] == [name]
    joints = ros[name]["ros__parameters"]["joints"]
    assert len(joints) == len(set(joints)) == 21
    assert joints == controllers[name]["joints"]
    for joint in joints:
        assert URDF.find(f"joint[@name='{joint}']") is not None
    assert ros[name]["ros__parameters"]["allow_partial_joints_goal"]
    assert moveit["trajectory_execution"]["execution_duration_monitoring"]


def test_tcp_virtual_tool_contract():
    for side in ("l", "r"):
        joint = URDF.find(f"joint[@name='arm_{side}_tcp_joint']")
        assert joint.get("type") == "fixed"
        assert joint.find("parent").get("link") == f"arm_{side}j7_link"
        assert [float(v) for v in joint.find("origin").get("xyz").split()] == SCENE["tcp_offset"]
    srdf = ET.parse(CONFIG / "jaka_kargo.srdf").getroot()
    for side in ("l", "r"):
        chain = srdf.find(f"group[@name='jaka_kargo_arm_{side}']/chain")
        assert chain.get("tip_link") == f"arm_{side}_tcp"


def test_dual_arm_group_excludes_external_axes():
    srdf = ET.parse(CONFIG / "jaka_kargo.srdf").getroot()
    group = srdf.find("group[@name='jaka_kargo_dual_arms']")
    assert group is not None
    assert [(child.tag, child.get("name")) for child in group] == [
        ("group", "jaka_kargo_arm_l"), ("group", "jaka_kargo_arm_r")]
    for side in ("l", "r"):
        chain = srdf.find(f"group[@name='jaka_kargo_arm_{side}']/chain")
        assert chain.get("base_link") == "body_j2_link"


def test_simulation_rates_and_motion_scales():
    assert SCENE["physics_dt"] == 0.005
    assert SCENE["rendering_dt"] / SCENE["physics_dt"] == 4
    assert 0 < SCENE["precision_velocity_scaling"] < SCENE["velocity_scaling"] <= 1
    assert 0 < SCENE["precision_acceleration_scaling"] <= SCENE["acceleration_scaling"] <= 1


def test_scene_supports_and_mass():
    objects = SCENE["objects"]
    table = objects["workbench"]
    part = objects["part_a"]
    base = objects["assembly_base"]
    table_top = table["position"][2] + table["size"][2] / 2
    assert 0 <= part["position"][2] - part["size"][2] / 2 - table_top <= 0.01
    assert abs(base["position"][2] - base["size"][2] / 2 - table_top) < 1e-9
    assert abs(SCENE["target_position"][2] - part["size"][2] / 2
               - base["position"][2] - base["size"][2] / 2) < 1e-9
    assert part["mass"] > 0 and all(v > 0 for v in part["size"])
    for name in ("part_a", "assembly_base"):
        for axis in (0, 1):
            assert abs(objects[name]["position"][axis] - table["position"][axis]) + objects[name]["size"][axis] / 2 <= table["size"][axis] / 2
    assert math.isclose(sum(v*v for v in SCENE["tool_orientation_xyzw"]), 1)


def test_floor_clears_robot_base_mesh():
    # Regression: the base mesh extends below -0.30 m in the URDF world frame.
    import struct
    import numpy as np
    mesh = ROOT / "src/jaka_kargo_description/meshes/jaka_kargo_meshes/base_link.STL"
    data = mesh.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    triangles = np.frombuffer(data, offset=84, count=count, dtype=np.dtype([
        ("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")]))
    minimum = triangles["vertices"].reshape(-1, 3)[:, 2].min()
    ground = SCENE["objects"]["ground"]
    assert ground["position"][2] + ground["size"][2] / 2 < minimum - 0.002
