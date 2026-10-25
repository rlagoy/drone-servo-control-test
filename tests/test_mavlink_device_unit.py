"""
Unit tests for MAVLinkDevice with the MAVLink connection replaced by a mock.

These check *what bytes we ask pymavlink to send* and how we react to replies,
without any socket, serial port, or flight controller.
"""

from __future__ import annotations

from types import SimpleNamespace
from unittest import mock

import pytest
from pymavlink import mavutil

from drone_servo_control_test import MAVLinkDevice, ack_ok

mavlink = mavutil.mavlink
DO_SET_SERVO = mavlink.MAV_CMD_DO_SET_SERVO


def ack(command: int = DO_SET_SERVO, result: int = mavlink.MAV_RESULT_ACCEPTED):
    return SimpleNamespace(command=command, result=result)


@pytest.fixture
def conn():
    """A mock pymavlink connection with a known target system/component."""
    c = mock.MagicMock(name="mavlink_connection")
    c.target_system = 1
    c.target_component = 1
    c.recv_match.return_value = None
    return c


@pytest.fixture
def dev(conn):
    with mock.patch.object(mavutil, "mavlink_connection", return_value=conn) as factory:
        d = MAVLinkDevice("COM5", baud=57600)
        d.connect()
        factory.assert_called_once_with("COM5", baud=57600)
    return d


# --- not connected ------------------------------------------------------------


@pytest.mark.parametrize(
    "call",
    [
        lambda d: d.wait_heartbeat(),
        lambda d: d.command_long(DO_SET_SERVO),
        lambda d: d.set_servo(4, 1500),
        lambda d: d.set_rc_override({1: 1500}),
        lambda d: d.read_servo_output_raw(),
        lambda d: d.read_servo_pwm(4),
    ],
    ids=["wait_heartbeat", "command_long", "set_servo", "rc_override", "read_raw", "read_pwm"],
)
def test_methods_raise_if_not_connected(call):
    with pytest.raises(RuntimeError, match="Not connected"):
        call(MAVLinkDevice("unused"))


def test_close_releases_connection(dev, conn):
    dev.close()
    conn.close.assert_called_once()
    assert dev.device is None
    dev.close()  # second close is a no-op


def test_context_manager_connects_and_closes(conn):
    with (
        mock.patch.object(mavutil, "mavlink_connection", return_value=conn),
        MAVLinkDevice("COM5") as d,
    ):
        assert d.device is conn
    conn.close.assert_called_once()


def test_wait_heartbeat_passes_timeout(dev, conn):
    dev.wait_heartbeat(timeout=2.5)
    conn.wait_heartbeat.assert_called_once_with(timeout=2.5)


# --- COMMAND_LONG -------------------------------------------------------------


def test_command_long_pads_params_to_seven(dev, conn):
    dev.command_long(DO_SET_SERVO, (4, 1500), expect_ack=False)
    conn.mav.command_long_send.assert_called_once_with(
        1, 1, DO_SET_SERVO, 0, 4.0, 1500.0, 0.0, 0.0, 0.0, 0.0, 0.0
    )


def test_command_long_accepts_generator(dev, conn):
    """Regression: params used to be iterated twice, so generators lost their values."""
    dev.command_long(DO_SET_SERVO, (x for x in (4, 1500)), expect_ack=False)
    args = conn.mav.command_long_send.call_args.args
    assert args[4:6] == (4.0, 1500.0)


def test_command_long_rejects_too_many_params(dev):
    with pytest.raises(ValueError, match="at most 7"):
        dev.command_long(DO_SET_SERVO, range(8))


def test_command_long_explicit_target(dev, conn):
    dev.command_long(DO_SET_SERVO, target_sys=7, target_comp=3, confirm=1, expect_ack=False)
    assert conn.mav.command_long_send.call_args.args[:4] == (7, 3, DO_SET_SERVO, 1)


def test_command_long_defaults_target_to_1_before_heartbeat(dev, conn):
    conn.target_system = 0
    conn.target_component = 0
    dev.command_long(DO_SET_SERVO, expect_ack=False)
    assert conn.mav.command_long_send.call_args.args[:2] == (1, 1)


def test_command_long_returns_matching_ack(dev, conn):
    expected = ack()
    conn.recv_match.side_effect = [None, ack(command=999), expected]
    assert dev.command_long(DO_SET_SERVO) is expected


def test_command_long_times_out_without_ack(dev, conn):
    assert dev.command_long(DO_SET_SERVO, ack_timeout=0.05) is None
    conn.recv_match.assert_called_with(type="COMMAND_ACK", blocking=False)


@pytest.mark.parametrize(("rate_hz", "interval_us"), [(10, 100_000.0), (0, -1.0)])
def test_request_message_interval(dev, conn, rate_hz, interval_us):
    dev.request_message_interval(36, rate_hz)
    args = conn.mav.command_long_send.call_args.args
    assert args[2] == mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
    assert args[4:6] == (36.0, interval_us)


def test_stream_servo_outputs_reports_ack(dev, conn):
    conn.recv_match.return_value = ack(command=mavlink.MAV_CMD_SET_MESSAGE_INTERVAL)
    assert dev.stream_servo_outputs() is True


# --- set_servo ----------------------------------------------------------------


def test_set_servo_sends_do_set_servo(dev, conn):
    expected = ack()
    conn.recv_match.return_value = expected
    assert dev.set_servo(4, 1000) is expected
    conn.mav.command_long_send.assert_called_once_with(
        1, 1, DO_SET_SERVO, 0, 4.0, 1000.0, 0.0, 0.0, 0.0, 0.0, 0.0
    )


@pytest.mark.parametrize("pwm", [799, 2201, 0, -1500])
def test_set_servo_rejects_out_of_range_pwm(dev, conn, pwm):
    with pytest.raises(ValueError, match="outside allowed range"):
        dev.set_servo(4, pwm)
    conn.mav.command_long_send.assert_not_called()


@pytest.mark.parametrize("pwm", [800, 1000, 1500, 2000, 2200])
def test_set_servo_accepts_range_edges(dev, conn, pwm):
    dev.set_servo(4, pwm, ack_timeout=0)
    conn.mav.command_long_send.assert_called_once()


def test_set_servo_custom_range(conn):
    with mock.patch.object(mavutil, "mavlink_connection", return_value=conn):
        d = MAVLinkDevice("COM5", pwm_range=(1100, 1900))
        d.connect()
    with pytest.raises(ValueError, match="1100-1900"):
        d.set_servo(4, 1000)


def test_set_servo_rejects_servo_zero(dev):
    with pytest.raises(ValueError, match="servo must be >= 1"):
        dev.set_servo(0, 1500)


def test_set_servo_rc_override_path(dev, conn):
    assert dev.set_servo(3, 1700, prefer_rc_override=True) is None
    conn.mav.command_long_send.assert_not_called()
    conn.mav.rc_channels_override_send.assert_called_once_with(1, 1, 0, 0, 1700, 0, 0, 0, 0, 0)


# --- RC override --------------------------------------------------------------


def test_rc_override_ignores_channels_outside_1_to_8(dev, conn):
    dev.set_rc_override({1: 1100, 8: 1900, 0: 1500, 9: 1500})
    conn.mav.rc_channels_override_send.assert_called_once_with(1, 1, 1100, 0, 0, 0, 0, 0, 0, 1900)


# --- reading outputs ----------------------------------------------------------


def test_read_servo_pwm(dev, conn):
    conn.recv_match.return_value = SimpleNamespace(servo4_raw=1234)
    assert dev.read_servo_pwm(4) == 1234


def test_read_servo_pwm_unknown_field(dev, conn):
    conn.recv_match.return_value = SimpleNamespace(servo1_raw=1000)
    assert dev.read_servo_pwm(4) is None


def test_read_servo_pwm_timeout(dev):
    assert dev.read_servo_pwm(4, timeout=0.03) is None


def test_wait_servo_pwm_skips_stale_telemetry(dev, conn):
    conn.recv_match.side_effect = [
        SimpleNamespace(servo4_raw=1500),  # queued before the command took effect
        SimpleNamespace(servo4_raw=1000),
    ]
    assert dev.wait_servo_pwm(4, 1000) == 1000


def test_wait_servo_pwm_returns_last_seen_on_timeout(dev, conn):
    conn.recv_match.return_value = SimpleNamespace(servo4_raw=1500)
    assert dev.wait_servo_pwm(4, 1000, timeout=0.05) == 1500


def test_wait_servo_pwm_no_telemetry(dev):
    assert dev.wait_servo_pwm(4, 1000, timeout=0.05) is None


# --- ack_ok -------------------------------------------------------------------


@pytest.mark.parametrize(
    ("msg", "expected"),
    [
        (ack(), True),
        (ack(result=mavlink.MAV_RESULT_DENIED), False),
        (ack(result=mavlink.MAV_RESULT_FAILED), False),
        (None, False),
    ],
)
def test_ack_ok(msg, expected):
    assert ack_ok(msg) is expected
