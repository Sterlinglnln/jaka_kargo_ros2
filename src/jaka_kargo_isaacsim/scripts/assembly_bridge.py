"""ROS services run on Isaac's simulation thread so USD edits are serialized."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from assembly_scene import PART_PATH, JOINT_PATH, tcp_matrix, world_matrix, set_grasp
from assembly_startup import StartupMonitor


class AssemblyBridge(Node):
    def __init__(self, stage, config, robot):
        super().__init__("isaac_assembly_bridge", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.stage = stage
        self.config = config
        self.robot = robot
        names = ([f"arm_{s}j{i}" for s in ("l", "r") for i in range(1, 8)]
                 + [f"body_j{i}" for i in range(1, 5)] + ["agv_x", "agv_y", "agv_yaw"])
        self.controlled_indices = [robot.get_dof_index(name) for name in names]
        self.startup = StartupMonitor()
        self.last_startup_log = -math.inf
        self.part = self.create_publisher(PoseStamped, "/assembly/part_pose", 10)
        self.tcp = self.create_publisher(PoseStamped, "/assembly/tcp_pose", 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ready = self.create_publisher(Bool, "/assembly/ready", qos)
        self.grasped = self.create_publisher(Bool, "/assembly/grasped", qos)
        self.grasped.publish(Bool(data=bool(stage.GetPrimAtPath(JOINT_PATH))))
        self.service = self.create_service(SetBool, "/assembly/grasp", self.grasp)
        self.get_logger().info("Simplified fixed-joint grasp enabled; no physical gripper is modeled")

    def grasp(self, request, response):
        try:
            if max(abs(float(v)) for v in self.robot.get_joint_velocities()) > 0.05:
                raise RuntimeError("Robot must be stationary before grasp/release")
            set_grasp(self.stage, self.config, request.data)
            self.grasped.publish(Bool(data=bool(self.stage.GetPrimAtPath(JOINT_PATH))))
            response.success = True
            response.message = "Fixed joint created" if request.data else "Fixed joint removed"
            self.get_logger().info(response.message)
        except Exception as error:
            response.success = False
            response.message = str(error)
            self.get_logger().error(response.message)
        return response

    def publish(self, sim_time):
        for publisher, matrix in ((self.part, world_matrix(self.stage, PART_PATH)),
                                  (self.tcp, tcp_matrix(self.stage, self.config))):
            translation = matrix.ExtractTranslation()
            rotation = matrix.ExtractRotationQuat().GetNormalized()
            if not all(math.isfinite(v) for v in translation):
                raise RuntimeError("Non-finite physical object pose")
            msg = PoseStamped()
            msg.header.frame_id = self.config["frame"]
            msg.header.stamp.sec = int(sim_time)
            msg.header.stamp.nanosec = int((sim_time - int(sim_time)) * 1e9)
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, translation)
            msg.pose.orientation.w = float(rotation.GetReal())
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z = map(float, rotation.GetImaginary())
            publisher.publish(msg)
        was_ready = self.startup.ready
        ready = self.startup.update(
            list(map(float, self.robot.get_joint_positions(self.controlled_indices))),
            list(map(float, self.robot.get_joint_velocities(self.controlled_indices))), sim_time)
        self.ready.publish(Bool(data=ready))
        if ready and not was_ready:
            self.get_logger().info("Startup ready: measured zero posture held steadily for 0.5 simulation seconds")
        elif not ready and sim_time - self.last_startup_log >= 2.0:
            self.last_startup_log = sim_time
            self.get_logger().warning(
                f"Waiting for stable zero posture: max_position_error={self.startup.position_error:.6f}, "
                f"max_velocity={self.startup.speed:.6f}; do not start task until startup is ready")
