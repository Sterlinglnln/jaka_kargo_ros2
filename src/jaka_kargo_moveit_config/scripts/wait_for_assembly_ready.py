#!/usr/bin/env python3
"""Allow controller activation only after fresh, settled Isaac zero feedback."""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


JOINTS = ([f"arm_{side}j{i}" for side in ("l", "r") for i in range(1, 8)]
          + [f"body_j{i}" for i in range(1, 5)] + ["agv_x", "agv_y", "agv_yaw"])


def settled_joint_sample(msg):
    """Reject missing, duplicate or non-finite feedback; ignore passive wheels."""
    if len(msg.name) != len(set(msg.name)) or len(msg.name) != len(msg.position) or len(msg.name) != len(msg.velocity):
        return False, math.inf, math.inf
    indices = {name: i for i, name in enumerate(msg.name)}
    if not all(name in indices for name in JOINTS):
        return False, math.inf, math.inf
    positions = [msg.position[indices[name]] for name in JOINTS]
    velocities = [msg.velocity[indices[name]] for name in JOINTS]
    if not all(math.isfinite(v) for v in positions + velocities):
        return False, math.inf, math.inf
    error = max(abs(v) for v in positions)
    speed = max(abs(v) for v in velocities)
    return error <= 0.02 and speed <= 0.03, error, speed


class ActivationGate(Node):
    def __init__(self):
        super().__init__("assembly_controller_activation_gate")
        self.declare_parameter("timeout", 90.0)
        self.ready = False
        self.ready_time = -math.inf
        self.sample = None
        self.sample_time = -math.inf
        self.create_subscription(Bool, "/assembly/ready", self.on_ready,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState, "/isaac_joint_states", self.on_state, qos_profile_sensor_data)

    def on_ready(self, msg):
        self.ready = msg.data
        self.ready_time = time.monotonic()

    def on_state(self, msg):
        self.sample = msg
        self.sample_time = time.monotonic()

    def accepted(self):
        now = time.monotonic()
        if not self.ready or now - self.ready_time > 1.0 or now - self.sample_time > 1.0:
            return False
        return self.sample is not None and settled_joint_sample(self.sample)[0]


def main():
    rclpy.init()
    node = ActivationGate()
    code = 1
    try:
        deadline = time.monotonic() + node.get_parameter("timeout").value
        report = 0.0
        node.get_logger().info("Waiting for settled Isaac zero posture before activating trajectory controller")
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.accepted():
                node.get_logger().info("Fresh stable zero feedback confirmed; trajectory controller may activate")
                code = 0
                break
            if time.monotonic() >= report:
                report = time.monotonic() + 5.0
                _, error, speed = settled_joint_sample(node.sample) if node.sample is not None else (False, math.inf, math.inf)
                node.get_logger().info(f"Startup not ready: ready={node.ready}, max_zero_error={error:.6f}, max_speed={speed:.6f}")
        if code:
            node.get_logger().error("Isaac did not settle at zero; trajectory controller was NOT activated")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
