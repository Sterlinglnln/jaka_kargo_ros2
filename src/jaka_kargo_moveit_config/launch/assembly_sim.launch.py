"""Repository-owned, SDK-free MoveIt/Isaac control launch."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    config = (
        MoveItConfigsBuilder("jaka_kargo", package_name="jaka_kargo_moveit_config")
        .robot_description(mappings={"use_isaac_sim": "true", "use_rviz_sim": "false"})
        .trajectory_execution(file_path="config/assembly_moveit_controllers.yaml",
                              moveit_manage_controllers=False)
        .to_moveit_configs()
    )
    sim = {"use_sim_time": True}
    gate = Node(package="jaka_kargo_moveit_config", executable="wait_for_assembly_ready.py", output="screen")
    spawner = Node(package="controller_manager", executable="spawner",
                   arguments=["joint_state_broadcaster", "jaka_kargo_full_robot_controller",
                              "--controller-manager-timeout", "120"], output="screen")

    def activate_after_ready(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason="Isaac zero posture was not ready; controllers remain inactive"))]
        return [spawner]

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[config.robot_description, sim], output="screen"),
        Node(package="moveit_ros_move_group", executable="move_group",
             parameters=[config.to_dict(), sim, {
                 "publish_robot_description_semantic": True,
                 "publish_planning_scene": True,
                 "publish_geometry_updates": True,
                 "publish_state_updates": True,
                 "publish_transforms_updates": True,
             }], output="screen"),
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[config.robot_description,
                         str(config.package_path / "config/assembly_ros2_controllers.yaml"), sim],
             output="screen"),
        # Before activation TopicBasedSystem retains the configured zero commands.
        # JTC activation must not capture a transient, sagged physical pose as its hold target.
        RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=activate_after_ready)),
        gate,
        Node(package="rviz2", executable="rviz2", condition=IfCondition(LaunchConfiguration("use_rviz")),
             arguments=["-d", str(Path(config.package_path) / "config/moveit.rviz")],
             parameters=[config.robot_description, config.robot_description_semantic,
                         config.robot_description_kinematics, config.joint_limits,
                         config.planning_pipelines, sim], output="screen"),
    ])
