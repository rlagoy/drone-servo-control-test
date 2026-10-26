"""Shared fixtures: the simulated FC for software tests, and the real FC for HIL tests."""

from __future__ import annotations

from collections.abc import Iterator

import pytest
from fake_autopilot import AckMode, FakeAutopilot

from drone_servo_control_test import MAVLinkDevice
from drone_servo_control_test.ports import resolve_device


def pytest_addoption(parser: pytest.Parser) -> None:
    parser.addoption(
        "--device",
        default=None,
        help="Flight controller port for hardware tests (e.g. COM5, /dev/ttyACM0, or 'auto')",
    )
    parser.addoption("--servo", type=int, default=4, help="Servo output used by hardware tests")


@pytest.fixture
def fake_fc() -> Iterator[FakeAutopilot]:
    """A simulated autopilot that accepts DO_SET_SERVO."""
    with FakeAutopilot(AckMode.ACCEPT) as fc:
        yield fc


@pytest.fixture
def connected(fake_fc: FakeAutopilot) -> Iterator[MAVLinkDevice]:
    """A MAVLinkDevice already connected to ``fake_fc`` and past the first heartbeat."""
    with MAVLinkDevice(fake_fc.connection_string) as dev:
        assert dev.wait_heartbeat(timeout=2.0) is not None
        yield dev


@pytest.fixture
def hw_device(request: pytest.FixtureRequest) -> str:
    """Port of the real flight controller; skips the test if --device wasn't given."""
    device = request.config.getoption("--device")
    if device is None:
        pytest.skip("hardware test: pass --device <port> to run")
    return resolve_device(str(device))
