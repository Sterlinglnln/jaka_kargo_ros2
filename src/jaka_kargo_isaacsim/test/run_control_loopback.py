#!/usr/bin/env python3
"""Exercise real MoveIt + JTC with a software echo, WITHOUT Isaac/PhysX.

For single/dual trajectory regression only. This cannot test grasping or physics.
Run in a separate ROS_DOMAIN_ID; the runner owns and stops all its child processes.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from moveit_msgs.srv import GetPositionIK, ApplyPlanningScene, GetPlanningScene
from moveit_msgs.msg import RobotState, AttachedCollisionObject, PlanningSceneComponents
from controller_manager_msgs.srv import ListControllers

ROOT = Path(__file__).resolve().parents[3]
SCENE = json.loads((ROOT / "src/jaka_kargo_isaacsim/config/assembly_scene.json").read_text())


class Echo(Node):
    def __init__(self, body_drift=0.0):
        super().__init__("assembly_software_loopback")
        self.names = ([f"arm_{s}j{i}" for s in ("l", "r") for i in range(1, 8)]
                      + [f"body_j{i}" for i in range(1, 5)] + ["agv_x", "agv_y", "agv_yaw"])
        self.positions = {name: 0.0 for name in self.names}
        self.previous = self.positions.copy()
        self.body_drift = body_drift
        self.drift_active = False
        self.clock = self.create_publisher(Clock, "/clock", 10)
        self.states = self.create_publisher(JointState, "/isaac_joint_states", 10)
        self.part = self.create_publisher(PoseStamped, "/assembly/part_pose", 10)
        self.tcp = self.create_publisher(PoseStamped, "/assembly/tcp_pose", 10)
        self.ready = self.create_publisher(Bool, "/assembly/ready", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.commands = self.create_subscription(JointState, "/isaac_joint_commands", self.command, 10)
        self.release_activation = False
        self.started = time.monotonic()
        self.last = self.started
        self.timer = self.create_timer(0.01, self.tick)

    def command(self, msg):
        if len(msg.name) != len(msg.position) or not all(math.isfinite(q) for q in msg.position):
            raise RuntimeError("Invalid command from TopicBasedSystem")
        self.positions.update(zip(msg.name, msg.position))

    def tick(self):
        now = time.monotonic()
        elapsed = now - self.started
        stamp = Time(sec=int(elapsed), nanosec=int((elapsed % 1) * 1e9))
        self.clock.publish(Clock(clock=stamp))
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = self.names
        # Reproduce physical sag after the dual outward motion, before HOME.
        if not self.drift_active and all(self.positions[f"arm_{side}j4"] < -0.30 for side in ("l", "r")):
            self.drift_active = True
            print(f"Injecting held body_j2 feedback offset {self.body_drift:.4f} rad", flush=True)
        measured = self.positions.copy()
        if self.drift_active:
            measured["body_j2"] += self.body_drift
        msg.position = [measured[n] for n in self.names]
        msg.velocity = [(measured[n] - self.previous[n]) / (now - self.last) for n in self.names]
        self.states.publish(msg)
        self.previous = measured
        self.last = now
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "world"
        pose.pose.orientation.w = 1.0
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = SCENE["objects"]["part_a"]["position"]
        self.part.publish(pose)
        # Smoke tests use joint targets; TCP pose is just a readiness heartbeat here.
        self.tcp.publish(pose)
        self.ready.publish(Bool(data=self.release_activation))


def check_activation_gate(node):
    client = node.create_client(ListControllers, "/controller_manager/list_controllers")
    if not client.wait_for_service(timeout_sec=20):
        raise RuntimeError("Controller manager did not start")
    future = client.call_async(ListControllers.Request())
    until = time.monotonic() + 10
    while not future.done() and time.monotonic() < until:
        time.sleep(0.01)
    if not future.done():
        raise RuntimeError("Controller query timed out")
    for controller in future.result().controller:
        if controller.name == "jaka_kargo_full_robot_controller" and controller.state == "active":
            raise RuntimeError("Trajectory controller activated before Isaac was ready")
    node.release_activation = True
    node.destroy_client(client)
    print("Controller activation waits for Isaac readiness PASSED (no physics)", flush=True)


def check_assembly_ik(node):
    """Collision-aware IK checks for configured poses; this executes no motion."""
    client = node.create_client(GetPositionIK, "/compute_ik")
    if not client.wait_for_service(timeout_sec=20):
        raise RuntimeError("MoveIt IK service unavailable")
    source = SCENE["objects"]["part_a"]
    top = source["position"][2] + source["size"][2] / 2
    target = SCENE["target_position"]
    cases = [
        ("pregrasp", "l", [source["position"][0], source["position"][1], top + SCENE["approach_distance"]]),
        ("grasp", "l", [source["position"][0], source["position"][1], top]),
        ("lift", "l", [source["position"][0], source["position"][1], top + SCENE["lift_distance"]]),
        ("transfer", "l", [target[0], target[1], target[2] + source["size"][2] / 2 + 0.003 + SCENE["lift_distance"]]),
        ("place", "l", [target[0], target[1], target[2] + source["size"][2] / 2 + 0.003]),
        ("right_ready", "r", SCENE["right_ready_position"]),
    ]
    seed = RobotState()
    seed.joint_state.name = node.names
    seed.joint_state.position = [node.positions[n] for n in node.names]
    for name, side, position in cases:
        request = GetPositionIK.Request()
        request.ik_request.group_name = "jaka_kargo_arm_" + side
        request.ik_request.ik_link_name = "arm_" + side + "_tcp"
        request.ik_request.robot_state = seed
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = 3
        request.ik_request.pose_stamped.header.frame_id = "world"
        pose = request.ik_request.pose_stamped.pose
        pose.position.x, pose.position.y, pose.position.z = position
        q = SCENE["tool_orientation_xyzw"]
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
        future = client.call_async(request)
        until = time.monotonic() + 10
        while not future.done() and time.monotonic() < until:
            time.sleep(0.01)
        if not future.done() or future.result().error_code.val != 1:
            code = future.result().error_code.val if future.done() else "timeout"
            raise RuntimeError(f"Collision-aware IK failed for {name}: {code}")
        seed = future.result().solution
        print("Collision-aware IK " + name + " PASSED (no physics)", flush=True)
    node.destroy_client(client)


def check_attachment_lifecycle(node):
    """Exercise actual MoveIt scene attachment semantics; no physical grasp is claimed."""
    apply = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    get = node.create_client(GetPlanningScene, "/get_planning_scene")

    def call(client, request):
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError("Scene test service unavailable")
        future = client.call_async(request)
        until = time.monotonic() + 10
        while not future.done() and time.monotonic() < until:
            time.sleep(0.01)
        if not future.done():
            raise RuntimeError("Scene test service timeout")
        return future.result()

    def snapshot():
        request = GetPlanningScene.Request()
        request.components.components = (PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
                                         | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY)
        return call(get, request).scene

    def membership(scene):
        return (any(a.object.id == "part_a" for a in scene.robot_state.attached_collision_objects),
                any(o.id == "part_a" for o in scene.world.collision_objects))

    try:
        initial = snapshot()
        part = next(o for o in initial.world.collision_objects if o.id == "part_a")
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        item = AttachedCollisionObject()
        item.link_name = "arm_l_tcp"
        item.touch_links = ["arm_l_tcp", "arm_lj7_link"]
        item.object = part
        request.scene.robot_state.attached_collision_objects = [item]
        if not call(apply, request).success or membership(snapshot()) != (True, False):
            raise RuntimeError("MoveIt attachment did not remove the world object")
        # Detachment restores geometry at the measured world pose in the same diff.
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        item = AttachedCollisionObject()
        item.link_name = "arm_l_tcp"
        item.object.id = "part_a"
        item.object.operation = item.object.REMOVE
        request.scene.robot_state.attached_collision_objects = [item]
        request.scene.world.collision_objects = [part]
        if not call(apply, request).success or membership(snapshot()) != (False, True):
            raise RuntimeError("MoveIt detachment did not restore the world object")
        print("MoveIt attach/detach scene membership PASSED (no physics)", flush=True)
    finally:
        node.destroy_client(apply)
        node.destroy_client(get)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log-dir", type=Path, default=Path("/tmp/kargo-control-loopback"))
    parser.add_argument("--body-drift", type=float, default=0.0,
                        help="Inject body_j2 feedback offset after the dual outward trajectory")
    args = parser.parse_args()
    if "ROS_DOMAIN_ID" not in os.environ:
        raise RuntimeError("Set an isolated ROS_DOMAIN_ID before running this test")
    args.log_dir.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_LOG_DIR"] = str(args.log_dir / "ros")
    rclpy.init(args=[])
    node = Echo(args.body_drift)
    spinner = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spinner.start()
    children = []
    streams = []
    try:
        for mode in ("stack", "single", "dual"):
            path = args.log_dir / (mode + ".log")
            stream = path.open("w")
            streams.append(stream)
            command = (["ros2", "launch", "jaka_kargo_moveit_config", "assembly_sim.launch.py", "use_rviz:=false"]
                       if mode == "stack" else
                       ["ros2", "launch", "jaka_kargo_planner", "assembly_task.launch.py", "mode:=" + mode])
            child = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            children.append(child)
            if mode == "stack":
                time.sleep(3)
                check_activation_gate(node)
                time.sleep(5)
                if child.poll() is not None:
                    raise RuntimeError("Control stack exited; inspect " + str(path))
                continue
            child.wait(timeout=120)
            stream.flush()
            text = path.read_text()
            if "STATE=SUCCESS mode=" + mode not in text or "STATE=FAILED" in text:
                raise RuntimeError(mode + " failed; inspect " + str(path))
            print(mode + " MoveIt/JTC software loopback PASSED (no physics)", flush=True)
            time.sleep(2)  # Allow DDS to remove the previous short-lived task client.
        check_assembly_ik(node)
        check_attachment_lifecycle(node)
    finally:
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()
        rclpy.shutdown()
        spinner.join(timeout=5)
        node.destroy_node()
        for stream in streams:
            stream.close()


if __name__ == "__main__":
    main()
