import importlib.util
import math
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Pose
from rclpy.action import GoalResponse


PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('sim_docking', PACKAGE / 'scripts' / 'sim_docking.py')
docking = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(docking)


def test_dock_coordinates():
    pose = Pose()
    pose.position.x = 1.0
    pose.position.y = 2.6
    pose.orientation.z = math.sin(math.pi / 4)
    pose.orientation.w = math.cos(math.pi / 4)
    assert docking.dock_coordinates(pose, 1.0, 2.0, math.pi / 2) == pytest.approx((0.6, 0.0, 0.0))


def test_reservation_requires_valid_stationary_pose(monkeypatch):
    monkeypatch.setattr(docking.time, 'monotonic', lambda: 10.0)
    pose = Pose()
    pose.orientation.w = 1.0
    state = SimpleNamespace(lock=threading.Lock(), busy=False, pose=pose, pose_at=10.0,
                            odom_at=10.0, stationary_since=9.0, dock_x=0.0, dock_y=0.0,
                            dock_yaw=0.0, stopped=threading.Event())
    assert docking.SimDocking.reserve(state, False) == GoalResponse.ACCEPT
    assert docking.SimDocking.reserve(state, False) == GoalResponse.REJECT
    state.busy = False
    state.stationary_since = None
    assert docking.SimDocking.reserve(state, False) == GoalResponse.REJECT
    state.stationary_since = 9.0
    state.pose_at = 8.0
    assert docking.SimDocking.reserve(state, False) == GoalResponse.REJECT
    state.pose_at = 10.0
    state.pose.position.y = 0.2
    assert docking.SimDocking.reserve(state, False) == GoalResponse.REJECT
    state.pose.position.y = float('nan')
    assert docking.SimDocking.reserve(state, False) == GoalResponse.REJECT


def test_stale_pose_aborts_and_stops(monkeypatch):
    monkeypatch.setattr(docking.time, 'monotonic', lambda: 10.0)
    monkeypatch.setattr(docking.rclpy, 'ok', lambda: True)
    published = []
    events = []
    state = SimpleNamespace(lock=threading.Lock(), busy=True, pose=Pose(), pose_at=8.0,
                            odom_at=10.0, stationary_since=9.0, stopped=threading.Event(),
                            publisher=SimpleNamespace(publish=published.append))
    goal = SimpleNamespace(is_cancel_requested=False, abort=lambda: events.append('aborted'))
    result = docking.SimDocking.execute(state, goal, False, 5.0)
    assert not result.success
    assert 'stale' in result.error_msg
    assert events == ['aborted']
    assert published[-1].linear.x == 0.0
    assert not state.busy