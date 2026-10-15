"""Prevent activating JTC on the drifted posture observed in the real scene."""
import importlib.util
from pathlib import Path

from sensor_msgs.msg import JointState

spec = importlib.util.spec_from_file_location("gate", Path(__file__).resolve().parents[1] / "scripts/wait_for_assembly_ready.py")
gate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gate)


def sample():
    return JointState(name=list(reversed(gate.JOINTS)), position=[0.0] * 21, velocity=[0.0] * 21)


def test_drifted_left_arm_and_body_are_rejected():
    msg = sample()
    msg.position[msg.name.index("arm_lj1")] = 0.9671
    msg.position[msg.name.index("body_j2")] = 0.2874
    assert gate.settled_joint_sample(msg)[0] is False


def test_joint_names_are_matched_and_passive_wheels_ignored():
    msg = sample()
    msg.name.append("wheel_5")
    msg.position.append(1.0)
    msg.velocity.append(0.5)
    assert gate.settled_joint_sample(msg)[0] is True


def test_missing_velocity_and_duplicates_are_rejected():
    msg = sample()
    msg.velocity = []
    assert gate.settled_joint_sample(msg)[0] is False
    msg = sample()
    msg.name[-1] = msg.name[0]
    assert gate.settled_joint_sample(msg)[0] is False


def test_high_speed_is_not_ready_even_at_zero():
    msg = sample()
    msg.velocity[0] = 0.56
    assert gate.settled_joint_sample(msg)[0] is False
