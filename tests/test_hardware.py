"""
Hardware-in-the-loop tests: need the SpeedyBee F405 Wing on USB with the servo powered.

Skipped by default. Run with::

    uv run pytest -m hardware --device auto        # or COM5, /dev/cu.usbmodem1101, ...
"""

from __future__ import annotations

import pytest

from drone_servo_control_test import MAVLinkDevice, ack_ok

pytestmark = pytest.mark.hardware


@pytest.fixture
def fc(hw_device):
    with MAVLinkDevice(hw_device) as dev:
        assert dev.wait_heartbeat(timeout=5.0) is not None, "no heartbeat from FC"
        yield dev


def test_heartbeat_from_ardupilot(fc):
    assert fc.device.target_system >= 1


@pytest.mark.parametrize("pwm", [1000, 2000, 1500])
def test_servo_output_follows_command(fc, request, pwm):
    servo = request.config.getoption("--servo")
    assert ack_ok(fc.set_servo(servo, pwm)), "FC did not accept DO_SET_SERVO (see SERVOn_FUNCTION)"
    reported = fc.wait_servo_pwm(servo, pwm, timeout=2.0)
    assert reported == pwm, f"FC reports {reported} us (None: check SR0_* stream rates)"
