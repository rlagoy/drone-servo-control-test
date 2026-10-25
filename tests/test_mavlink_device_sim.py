"""
Component tests: MAVLinkDevice talking real MAVLink to the simulated flight controller.

Same code path as the hardware test, but the SpeedyBee is replaced by FakeAutopilot.
"""

from __future__ import annotations

import pytest
from fake_autopilot import AckMode, FakeAutopilot
from pymavlink import mavutil

from drone_servo_control_test import MAVLinkDevice, ack_ok

FLAP_SERVO = 4


def test_heartbeat_identifies_autopilot(connected):
    assert connected.device.target_system == 1


@pytest.mark.parametrize("pwm", [1000, 2000, 1500])
def test_set_servo_moves_output(connected, fake_fc, pwm):
    ack = connected.set_servo(FLAP_SERVO, pwm)

    assert ack_ok(ack)
    assert fake_fc.servo_pwm[FLAP_SERVO - 1] == pwm
    assert fake_fc.commands == [(FLAP_SERVO, pwm)]


def test_reported_output_matches_command(connected):
    connected.set_servo(FLAP_SERVO, 1800)
    assert connected.wait_servo_pwm(FLAP_SERVO, 1800) == 1800


def test_full_sweep_sequence(connected, fake_fc):
    for pwm in (1000, 2000, 1500):
        assert ack_ok(connected.set_servo(FLAP_SERVO, pwm))
    assert [pwm for _, pwm in fake_fc.commands] == [1000, 2000, 1500]


def test_servo_output_stream_request_accepted(connected):
    assert connected.stream_servo_outputs(rate_hz=20)


def test_unsupported_command_is_rejected(connected):
    ack = connected.command_long(mavutil.mavlink.MAV_CMD_DO_SET_RELAY, (0, 1))
    assert ack is not None
    assert ack.result == mavutil.mavlink.MAV_RESULT_UNSUPPORTED


def test_denied_command_reported():
    with (
        FakeAutopilot(AckMode.DENY) as fc,
        MAVLinkDevice(fc.connection_string) as dev,
    ):
        assert dev.wait_heartbeat(timeout=2.0)
        ack = dev.set_servo(FLAP_SERVO, 1500)
        assert ack is not None
        assert not ack_ok(ack)
        assert fc.servo_pwm[FLAP_SERVO - 1] == 0


def test_missing_ack_times_out():
    with (
        FakeAutopilot(AckMode.SILENT) as fc,
        MAVLinkDevice(fc.connection_string) as dev,
    ):
        assert dev.wait_heartbeat(timeout=2.0)
        assert dev.set_servo(FLAP_SERVO, 1500, ack_timeout=0.3) is None


def test_no_autopilot_no_heartbeat():
    fc = FakeAutopilot()  # created but never started: nothing is transmitting
    try:
        with MAVLinkDevice(fc.connection_string) as dev:
            assert dev.wait_heartbeat(timeout=0.3) is None
    finally:
        fc.stop()
