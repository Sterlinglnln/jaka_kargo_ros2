# -*- coding: utf-8 -*-

import os
import json
from pathlib import Path

from isaacsim import SimulationApp


GRAPH_PATH = "/ActionGraph"

CONFIG = {
    "renderer": "RayTracedLighting",
    "headless": os.environ.get("KARGO_HEADLESS", "false").lower() == "true",
}

simulation_app = SimulationApp(CONFIG)

from isaacsim.core.api import SimulationContext
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import is_stage_loading, open_stage, get_current_stage

import omni.graph.core as og


enable_extension("isaacsim.ros2.bridge")

for _ in range(20):
    simulation_app.update()


def resolve_usd():
    path = os.environ.get("KARGO_USD_PATH", "")

    if path:
        return path

    try:
        from ament_index_python.packages import (
            get_package_share_directory,
        )

        package_share = get_package_share_directory(
            "jaka_kargo_description"
        )

        return str(
            Path(package_share)
            / "urdf"
            / "jaka_kargo"
            / "jaka_kargo_moveit.usd"
        )

    except Exception:
        return str(
            Path(__file__).resolve().parents[2]
            / "jaka_kargo_description"
            / "urdf"
            / "jaka_kargo"
            / "jaka_kargo_moveit.usd"
        )


def get_graph_attribute(attribute_path):
    attribute = og.Controller.attribute(attribute_path)

    if not attribute.is_valid():
        raise RuntimeError(
            f"OmniGraph attribute not found: {attribute_path}"
        )

    return attribute


KARGO_USD_PATH = resolve_usd()

print(
    f"Resolved USD path: {KARGO_USD_PATH}, "
    f"exists={os.path.exists(KARGO_USD_PATH)}"
)

if not os.path.exists(KARGO_USD_PATH):
    raise RuntimeError(
        f"USD not found: {KARGO_USD_PATH}"
    )

if not open_stage(KARGO_USD_PATH):
    raise RuntimeError(
        f"Failed to open USD stage: {KARGO_USD_PATH}"
    )

while is_stage_loading():
    simulation_app.update()

# All graph/model/scene edits are ephemeral, preserving the source USD.
stage = get_current_stage()
stage.SetEditTarget(stage.GetSessionLayer())
assembly_config = None
physics_diagnostics = None
if os.environ.get("KARGO_ASSEMBLY_CONFIG"):
    from assembly_scene import audit_robot, create_scene, configure_robot_drives
    with open(os.environ["KARGO_ASSEMBLY_CONFIG"], encoding="utf-8") as stream:
        assembly_config = json.load(stream)
    expected_joints = audit_robot(stage, os.environ["KARGO_URDF_PATH"], assembly_config)
    configure_robot_drives(stage, assembly_config, expected_joints)
    create_scene(stage, assembly_config, os.environ["KARGO_SRDF_PATH"])
    if os.environ.get("KARGO_PHYSICS_DIAGNOSTICS") == "1":
        from assembly_physics_diagnostics import PhysicsDiagnostics
        physics_diagnostics = PhysicsDiagnostics(stage)
    # Read back actual PhysX transforms for the physical grasp and pose feedback.
    import carb
    carb.settings.get_settings().set_bool("/physics/updateToUsd", True)
    carb.settings.get_settings().set_bool("/physics/updateVelocitiesToUsd", True)
    root = assembly_config["articulation_root"]
    stage.GetPrimAtPath(GRAPH_PATH + "/PublishJointState").GetRelationship("inputs:targetPrim").SetTargets([root])
    get_graph_attribute(GRAPH_PATH + "/ArticulationController.inputs:robotPath").set(root)
    # Drop the stale OnImpulseEvent references authored in the existing graph.
    for name in ("PublishJointState", "SubscribeJointState", "PublishClock"):
        stage.GetPrimAtPath(GRAPH_PATH + "/" + name).GetAttribute("inputs:execIn").SetConnections(
            [GRAPH_PATH + "/on_playback_tick.outputs:tick"])
    stage.GetPrimAtPath(GRAPH_PATH + "/ArticulationController").GetAttribute("inputs:execIn").SetConnections(
        [GRAPH_PATH + "/SubscribeJointState.outputs:execOut"])

print(
    f"Stage opened successfully: {KARGO_USD_PATH}"
)

read_sim_time = get_graph_attribute(
    f"{GRAPH_PATH}/ReadSimTime.outputs:simulationTime"
)

publish_joint_time = get_graph_attribute(
    f"{GRAPH_PATH}/PublishJointState.inputs:timeStamp"
)

publish_clock_time = get_graph_attribute(
    f"{GRAPH_PATH}/PublishClock.inputs:timeStamp"
)

publish_joint_topic = get_graph_attribute(
    f"{GRAPH_PATH}/PublishJointState.inputs:topicName"
)

subscribe_joint_topic = get_graph_attribute(
    f"{GRAPH_PATH}/SubscribeJointState.inputs:topicName"
)

publish_clock_topic = get_graph_attribute(
    f"{GRAPH_PATH}/PublishClock.inputs:topicName"
)

publish_joint_topic.set("/isaac_joint_states")
subscribe_joint_topic.set("/isaac_joint_commands")
publish_clock_topic.set("/clock")

if not read_sim_time.is_connected(publish_joint_time):
    if not read_sim_time.connect(
        publish_joint_time,
        True,
    ):
        raise RuntimeError(
            "Failed to connect simulation time to "
            "PublishJointState"
        )

if not read_sim_time.is_connected(publish_clock_time):
    if not read_sim_time.connect(
        publish_clock_time,
        True,
    ):
        raise RuntimeError(
            "Failed to connect simulation time to PublishClock"
        )

simulation_app.update()
simulation_app.update()

simulation_context = SimulationContext(
    stage_units_in_meters=1.0,
    physics_dt=assembly_config["physics_dt"] if assembly_config else 1.0 / 60.0,
    rendering_dt=assembly_config["rendering_dt"] if assembly_config else 1.0 / 60.0,
)

simulation_context.play()
simulation_app.update()

bridge = None
if assembly_config:
    from isaacsim.core.prims import SingleArticulation
    from assembly_bridge import AssemblyBridge
    import rclpy
    import numpy as np
    robot = SingleArticulation(prim_path=assembly_config["articulation_root"])
    robot.initialize()
    missing = set(expected_joints) - set(robot.dof_names)
    if missing:
        raise RuntimeError(f"Articulation does not expose expected joints: {sorted(missing)}")
    # Start from the URDF/SRDF zero state instead of saved USD simulation state.
    robot.set_joint_positions(np.zeros(robot.num_dof))
    robot.set_joint_velocities(np.zeros(robot.num_dof))
    robot.get_articulation_controller().set_gains(
        kps=np.full(robot.num_dof, assembly_config["joint_drive"]["stiffness"]),
        kds=np.full(robot.num_dof, assembly_config["joint_drive"]["damping"]))
    robot.set_solver_position_iteration_count(32)
    robot.set_solver_velocity_iteration_count(4)
    rclpy.init()
    bridge = AssemblyBridge(stage, assembly_config, robot)

try:
    while simulation_app.is_running() and (bridge is None or rclpy.ok()):
        if bridge:
            rclpy.spin_once(bridge, timeout_sec=0.0)
        simulation_context.step(render=True)
        if bridge and simulation_context.is_playing():
            bridge.publish(simulation_context.current_time)
            if physics_diagnostics:
                physics_diagnostics.report(robot, simulation_context.current_time)
finally:
    if bridge:
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    simulation_context.stop()
    simulation_app.close()
