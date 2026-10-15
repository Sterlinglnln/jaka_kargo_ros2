"""MoveGroup client only; never starts or links the JAKA hardware bridge."""
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    scene_path = Path(get_package_share_directory("jaka_kargo_isaacsim")) / "config/assembly_scene.json"
    scene = json.loads(scene_path.read_text())
    config = (
        MoveItConfigsBuilder("jaka_kargo", package_name="jaka_kargo_moveit_config")
        .robot_description(mappings={"use_isaac_sim": "true"})
        .to_moveit_configs()
    )
    parameters = {key: scene[key] for key in (
        "target_position", "target_tolerance", "target_orientation_tolerance",
        "tool_orientation_xyzw", "right_ready_position", "approach_distance",
        "lift_distance", "precision_velocity_scaling", "precision_acceleration_scaling")}
    parameters["object_names"] = list(scene["objects"])
    for name, spec in scene["objects"].items():
        parameters[name + ".position"] = spec["position"]
        parameters[name + ".size"] = spec["size"]
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="single",
                              choices=["single", "dual", "assembly"]),
        DeclareLaunchArgument("test_delta", default_value="-0.35",
                              description="Elbow joint offset in radians for single/dual tests"),
        DeclareLaunchArgument("test_hold_seconds", default_value="5.0",
                              description="Wall seconds to observe the test pose before HOME"),
        DeclareLaunchArgument("velocity_scaling", default_value=str(scene["velocity_scaling"]),
                              description="Normal motion velocity scale, in (0, 1]"),
        DeclareLaunchArgument("acceleration_scaling", default_value=str(scene["acceleration_scaling"]),
                              description="Normal motion acceleration scale, in (0, 1]"),
        Node(package="jaka_kargo_planner", executable="assembly_task", output="screen",
             parameters=[config.robot_description, config.robot_description_semantic,
                         config.robot_description_kinematics, config.joint_limits,
                         parameters, {"use_sim_time": True, "mode": LaunchConfiguration("mode"),
                                      "velocity_scaling": ParameterValue(LaunchConfiguration("velocity_scaling"), value_type=float),
                                      "acceleration_scaling": ParameterValue(LaunchConfiguration("acceleration_scaling"), value_type=float),
                                      "test_delta": ParameterValue(LaunchConfiguration("test_delta"), value_type=float),
                                      "test_hold_seconds": ParameterValue(LaunchConfiguration("test_hold_seconds"), value_type=float)}]),
    ])
