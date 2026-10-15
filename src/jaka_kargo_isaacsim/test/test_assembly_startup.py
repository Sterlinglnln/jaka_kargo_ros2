"""Regression tests for startup feedback being advertised before settling."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from assembly_startup import StartupMonitor


def test_reject_drifted_or_moving_initial_posture():
    monitor = StartupMonitor()
    assert not monitor.update([0.2, 0.0], [0.0, 0.0], 1.0)
    assert not monitor.update([0.0, 0.0], [0.1, 0.0], 2.0)
    assert not monitor.update([0.0, 0.0], [0.0, 0.0], 3.0)
    assert not monitor.update([0.1, 0.0], [0.0, 0.0], 3.4)
    assert not monitor.update([0.0, 0.0], [0.0, 0.0], 3.5)
    assert monitor.update([0.0, 0.0], [0.0, 0.0], 4.0)


def test_task_motion_does_not_reset_commissioned_state():
    monitor = StartupMonitor()
    assert not monitor.update([0.0], [0.0], 1.0)
    assert monitor.update([0.0], [0.0], 1.5)
    assert monitor.update([0.5], [0.1], 2.0)
    assert not monitor.update([float('nan')], [0.0], 2.1)


def test_clock_reset_and_missing_velocity_are_not_ready():
    monitor = StartupMonitor()
    assert not monitor.update([0.0], [], 1.0)
    assert not monitor.update([0.0], [0.0], 5.0)
    assert not monitor.update([0.0], [0.0], 0.0)
    assert monitor.update([0.0], [0.0], 0.5)
