import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

from geometry_msgs.msg import Twist


PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('sim_base_command', PACKAGE / 'scripts' / 'sim_base_command.py')
base_command = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(base_command)


def test_velocity_validation_and_limits(monkeypatch):
    monkeypatch.setattr(base_command.time, 'monotonic', lambda: 10.0)
    state = SimpleNamespace(latest=Twist(), received_at=None)
    command = Twist()
    command.linear.x = -3.0
    command.angular.z = 3.0
    base_command.SimBaseCommand.command(state, command)
    assert state.latest.linear.x == -0.2
    assert state.latest.angular.z == math.pi / 3
    assert state.received_at == 10.0

    monkeypatch.setattr(base_command.time, 'monotonic', lambda: 11.0)
    command.linear.y = float('nan')
    base_command.SimBaseCommand.command(state, command)
    command.linear.y = 0.1
    base_command.SimBaseCommand.command(state, command)
    assert state.received_at == 10.0
    assert state.latest.linear.x == -0.2


def test_timeout_and_stop(monkeypatch):
    published = []
    command = Twist()
    command.linear.x = 0.5
    state = SimpleNamespace(latest=command, received_at=10.0,
                            publisher=SimpleNamespace(publish=published.append))
    monkeypatch.setattr(base_command.time, 'monotonic', lambda: 10.1)
    base_command.SimBaseCommand.publish(state)
    assert published[-1].linear.x == 0.5
    monkeypatch.setattr(base_command.time, 'monotonic', lambda: 10.21)
    base_command.SimBaseCommand.publish(state)
    assert published[-1].linear.x == 0.0


def test_docking_command_owns_base(monkeypatch):
    published = []
    state = SimpleNamespace(latest=Twist(), received_at=None, docking_received_at=None,
                            publisher=SimpleNamespace(publish=published.append))
    monkeypatch.setattr(base_command.time, 'monotonic', lambda: 10.0)
    dock_command = Twist()
    dock_command.linear.x = -0.1
    base_command.SimBaseCommand.docking_command(state, dock_command)
    external_command = Twist()
    external_command.linear.x = 0.8
    base_command.SimBaseCommand.command(state, external_command)
    base_command.SimBaseCommand.publish(state)
    assert published[-1].linear.x == -0.1
    assert state.received_at is None
    monkeypatch.setattr(base_command.time, 'monotonic', lambda: 10.21)
    base_command.SimBaseCommand.publish(state)
    assert published[-1].linear.x == 0.0
    base_command.SimBaseCommand.stop(state, None)
    assert state.received_at is None
    assert published[-1].linear.x == 0.0