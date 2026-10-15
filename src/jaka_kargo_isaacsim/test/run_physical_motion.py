#!/usr/bin/env python3
"""Run one arm/dual-arm MoveIt test and independently record live simulation feedback.

Requires an already running assembly Isaac/ROS stack. No echo or hardware commands.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", required=True,
                        help="Execute the selected test and return to its initial pose")
    parser.add_argument("--mode", choices=("single", "dual"), default="single")
    parser.add_argument("--log-dir", type=Path, default=Path("/tmp/kargo-physical-motion"))
    parser.add_argument("--capture-desktop", action="store_true")
    args = parser.parse_args()
    test_joints = ["arm_lj4"] + (["arm_rj4"] if args.mode == "dual" else [])
    args.log_dir.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_LOG_DIR"] = str(args.log_dir / "ros")
    rclpy.init(args=[])
    node = rclpy.create_node("assembly_physical_motion_observer")
    controlled = ([f"arm_{side}j{i}" for side in ("l", "r") for i in range(1, 8)]
                  + [f"body_j{i}" for i in range(1, 5)] + ["agv_x", "agv_y", "agv_yaw"])
    latest, trace = {}, {}
    readiness = [False]

    def joint(topic, msg):
        if len(msg.position) != len(msg.name):
            return
        sample = {"wall": time.monotonic(), "sim": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                  "q": dict(zip(msg.name, msg.position)), "v": dict(zip(msg.name, msg.velocity))}
        latest[topic] = sample
        trace.setdefault(topic, []).append(sample)

    def tcp(msg):
        p = msg.pose.position
        sample = {"wall": time.monotonic(), "xyz": [p.x, p.y, p.z]}
        latest["tcp"] = sample
        trace.setdefault("tcp", []).append(sample)

    subscriptions = [node.create_subscription(JointState, topic,
                     lambda msg, topic=topic: joint(topic, msg), qos_profile_sensor_data)
                     for topic in ("/isaac_joint_commands", "/isaac_joint_states", "/joint_states")]
    subscriptions.append(node.create_subscription(PoseStamped, "/assembly/tcp_pose", tcp, 10))
    subscriptions.append(node.create_subscription(Bool, "/assembly/ready",
                         lambda msg: readiness.__setitem__(0, msg.data),
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))

    def capture(name):
        if args.capture_desktop:
            from PIL import ImageGrab
            ImageGrab.grab().save(args.log_dir / (name + ".png"))

    child = None
    try:
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if not readiness[0] or not all(
                    key in latest and now - latest[key]["wall"] < 1
                    for key in ("/isaac_joint_states", "/joint_states", "tcp")):
                continue
            raw = latest["/isaac_joint_states"]
            if all(name in raw["q"] and name in raw["v"] and math.isfinite(raw["q"][name])
                   and math.isfinite(raw["v"][name]) and abs(raw["q"][name]) < 0.02
                   and abs(raw["v"][name]) < 0.03 for name in controlled):
                break
        else:
            raise RuntimeError("No stable zero posture and live feedback; no motion sent")
        baseline = dict(latest)
        trace.clear()
        capture("before")
        path = args.log_dir / (args.mode + ".log")
        with path.open("w") as output:
            child = subprocess.Popen(["ros2", "launch", "jaka_kargo_planner", "assembly_task.launch.py",
                                      "mode:=" + args.mode, "test_hold_seconds:=8.0"],
                                     stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 180
            captured = False
            report = 0
            while child.poll() is None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
                now = time.monotonic()
                if now > report:
                    q = {joint: latest["/isaac_joint_states"]["q"].get(joint) for joint in test_joints}
                    print(f"Isaac actual elbows={q} rad TCP={latest['tcp']['xyz']}", flush=True)
                    report = now + 1
                if not captured and "Holding elbow test pose" in path.read_text():
                    capture("hold")
                    captured = True
            if child.poll() is None:
                raise RuntimeError("Motion test timeout; stopping its action client")
        # Drain final state messages after the action client exits.
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        capture("after")
        summary = {"mode": args.mode, "process_exit": child.returncode}
        for topic in ("/isaac_joint_commands", "/isaac_joint_states", "/joint_states"):
            samples = trace.get(topic, [])
            if not samples:
                raise RuntimeError("No recorded samples: " + topic)
            summary[topic] = {"samples": len(samples), "joints": {}}
            for joint in test_joints:
                values = [s["q"][joint] for s in samples]
                summary[topic]["joints"][joint] = {
                    "min": min(values), "max": max(values), "range_rad": max(values) - min(values),
                    "final_from_initial_rad": values[-1] - baseline.get(
                        topic, baseline["/isaac_joint_states"])["q"][joint]}
        initial_tcp = baseline["tcp"]["xyz"]
        summary["isaac_tcp_displacement_m"] = max(math.dist(initial_tcp, s["xyz"]) for s in trace["tcp"])
        initial = baseline["/isaac_joint_states"]["q"]
        summary["max_held_joint_displacement"] = max(
            abs(s["q"][name] - initial[name]) for s in trace["/isaac_joint_states"]
            for name in controlled if not name.startswith("arm_l")
            and not (args.mode == "dual" and name.startswith("arm_r")))
        (args.log_dir / "trace.json").write_text(json.dumps({"baseline": baseline, "trace": trace}))
        (args.log_dir / "summary.json").write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary, indent=2), flush=True)
        text = path.read_text()
        if child.returncode != 0 or "STATE=SUCCESS mode=" + args.mode not in text or "STATE=FAILED" in text:
            raise RuntimeError("MoveIt task failed; inspect " + str(path))
        for topic in ("/isaac_joint_commands", "/isaac_joint_states", "/joint_states"):
            for joint in test_joints:
                stats = summary[topic]["joints"][joint]
                if stats["range_rad"] < 0.28 or abs(stats["final_from_initial_rad"]) > 0.035:
                    raise RuntimeError("Insufficient measured motion or failed return: " + topic + " " + joint)
        if summary["isaac_tcp_displacement_m"] < 0.10:
            raise RuntimeError("Isaac physical TCP moved less than 10 cm")
        if summary["max_held_joint_displacement"] > 0.02:
            raise RuntimeError("Unexpected motion of held joints")
        print("PHYSICAL_MOTION_FEEDBACK_PASSED", flush=True)
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
