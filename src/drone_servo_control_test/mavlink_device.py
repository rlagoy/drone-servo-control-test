"""MAVLinkDevice: a small helper around pymavlink for heartbeats and servo control."""

from __future__ import annotations

import logging
import time
from collections.abc import Iterable
from types import TracebackType
from typing import Any

from pymavlink import mavutil

log = logging.getLogger(__name__)

#: Default safe PWM window (µs). Standard hobby servos expect 1000-2000 µs.
PWM_MIN_US = 800
PWM_MAX_US = 2200


def ack_ok(msg: Any) -> bool:
    """Return True if ``msg`` is a COMMAND_ACK with result ``MAV_RESULT_ACCEPTED``."""
    return msg is not None and int(msg.result) == mavutil.mavlink.MAV_RESULT_ACCEPTED


class MAVLinkDevice:
    """
    Generalized MAVLink device helper.

    Parameters
    ----------
    device_str : str
        pymavlink connection string, e.g. ``"COM5"``, ``"/dev/ttyACM0"`` or
        ``"udpin:127.0.0.1:14550"``.
    baud : int
        Serial baud rate (ignored for network connections).
    pwm_range : tuple[int, int]
        Inclusive (min, max) PWM in µs accepted by :meth:`set_servo`.
    """

    def __init__(
        self,
        device_str: str,
        baud: int = 115200,
        pwm_range: tuple[int, int] = (PWM_MIN_US, PWM_MAX_US),
    ) -> None:
        self.device_str = device_str
        self.baud = baud
        self.pwm_range = pwm_range
        self.device: Any = None

    # --- connection & heartbeat -------------------------------------------------

    def connect(self) -> None:
        """Open the pymavlink connection."""
        self.device = mavutil.mavlink_connection(self.device_str, baud=self.baud)

    def close(self) -> None:
        """Close the connection (releases the serial port for other programs)."""
        if self.device is not None:
            self.device.close()
            self.device = None

    def __enter__(self) -> MAVLinkDevice:
        self.connect()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.close()

    def _require_device(self) -> Any:
        if self.device is None:
            raise RuntimeError("Not connected.")
        return self.device

    def wait_heartbeat(self, timeout: float = 5.0) -> Any | None:
        """
        Block until a HEARTBEAT from a vehicle (autopilot) is received.

        Heartbeats from ground stations, gimbals, ADS-B receivers and companion computers
        are skipped: ArduPilot forwards these between its MAVLink ports, so they show up on
        USB too. pymavlink locks ``target_system``/``target_component`` onto the first
        vehicle heartbeat, so they are set once this returns a message.

        Returns the HEARTBEAT, or None if none arrived within ``timeout`` seconds.
        """
        device = self._require_device()
        deadline = time.monotonic() + timeout
        while (remaining := deadline - time.monotonic()) > 0:
            msg = device.recv_match(type="HEARTBEAT", blocking=True, timeout=remaining)
            if msg is not None and device.probably_vehicle_heartbeat(msg):
                return msg
        return None

    def command_long(  # noqa: PLR0913, PLR0917
        self,
        command: int,
        params: Iterable[float] = (0, 0, 0, 0, 0, 0, 0),
        target_sys: int | None = None,
        target_comp: int | None = None,
        confirm: int = 0,
        expect_ack: bool = True,
        ack_timeout: float = 1.0,
    ) -> Any | None:
        """
        Send a COMMAND_LONG and optionally wait for COMMAND_ACK.

        Returns the ACK message if ``expect_ack`` is True and one arrives, else None.
        """
        device = self._require_device()
        t_sys = target_sys if target_sys is not None else device.target_system or 1
        t_comp = target_comp if target_comp is not None else device.target_component or 1
        p = [float(x) for x in params]
        if len(p) > 7:
            raise ValueError(f"COMMAND_LONG takes at most 7 params, got {len(p)}")
        p += [0.0] * (7 - len(p))

        log.debug("Sending COMMAND_LONG %d to %d.%d with params %s", command, t_sys, t_comp, p)

        device.mav.command_long_send(t_sys, t_comp, command, confirm, *p)

        if not expect_ack:
            return None

        # Wait for COMMAND_ACK on the same command
        t0 = time.time()
        while time.time() - t0 < ack_timeout:
            msg = device.recv_match(type="COMMAND_ACK", blocking=False)
            if msg and int(msg.command) == int(command):
                return msg
            time.sleep(0.01)
        return None

    def request_message_interval(self, message_id: int, rate_hz: float) -> Any | None:
        """
        Ask the autopilot to stream ``message_id`` at ``rate_hz`` (MAV_CMD_SET_MESSAGE_INTERVAL).

        Returns the COMMAND_ACK, or None if none arrived.
        """
        interval_us = -1.0 if rate_hz <= 0 else 1e6 / rate_hz
        return self.command_long(
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, (float(message_id), interval_us)
        )

    def stream_servo_outputs(self, rate_hz: float = 10.0) -> bool:
        """Request SERVO_OUTPUT_RAW telemetry; True if the autopilot accepted."""
        return ack_ok(
            self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, rate_hz)
        )

    # --- servo control ----------------------------------------------------------

    def set_servo(
        self,
        servo: int,
        pwm_us: int,
        *,
        prefer_rc_override: bool = False,
        ack_timeout: float = 1.0,
    ) -> Any | None:
        """
        Set a servo output.

        Preferred: MAV_CMD_DO_SET_SERVO (param1=servo number, param2=pulse usec).
        Fallback: RC_CHANNELS_OVERRIDE (for setups that map RC channels to servos).

        Parameters
        ----------
        servo : int
            1-based servo output number (``SERVOn_*`` in ArduPilot).
        pwm_us : int
            Pulse width in microseconds; must lie within ``pwm_range``.
        prefer_rc_override : bool
            If True, use RC override instead of DO_SET_SERVO.
        ack_timeout : float
            Seconds to wait for COMMAND_ACK.

        Returns
        -------
        COMMAND_ACK message, or None if no ACK arrived (or RC override was used).
        """
        self._require_device()
        if servo < 1:
            raise ValueError(f"servo must be >= 1, got {servo}")
        lo, hi = self.pwm_range
        if not lo <= pwm_us <= hi:
            raise ValueError(f"pwm_us {pwm_us} outside allowed range {lo}-{hi} µs")

        if not prefer_rc_override:
            return self.command_long(
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                (float(servo), float(pwm_us)),
                expect_ack=True,
                ack_timeout=ack_timeout,
            )

        self.set_rc_override({servo: pwm_us})
        return None

    def set_rc_override(self, channel_pwm: dict[int, int]) -> None:
        """
        Send RC_CHANNELS_OVERRIDE.

        channel_pwm keys are 1-based RC channel numbers. Values are PWM µs (1000-2000),
        or 0 to release a channel from override.
        """
        device = self._require_device()

        # Channels outside 1..8 require MAVLink2 extensions; here we handle 1..8 only.
        ch_values = [0] * 8
        for ch, pwm in channel_pwm.items():
            if 1 <= ch <= 8:
                ch_values[ch - 1] = int(pwm)

        device.mav.rc_channels_override_send(
            device.target_system or 1,
            device.target_component or 1,
            *ch_values,
        )

    # --- reading servo outputs (best-effort) ------------------------------------

    def read_servo_output_raw(self, timeout: float = 0.2) -> Any | None:
        """
        Try to read a SERVO_OUTPUT_RAW message (if the autopilot is emitting it).

        Returns the message or None if none seen in timeout.
        """
        device = self._require_device()
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = device.recv_match(type="SERVO_OUTPUT_RAW", blocking=False)
            if msg:
                return msg
            time.sleep(0.01)
        return None

    def read_servo_pwm(self, servo: int, timeout: float = 0.5) -> int | None:
        """Return the latest reported PWM (µs) for a 1-based servo output, or None."""
        msg = self.read_servo_output_raw(timeout=timeout)
        if msg is None:
            return None
        value = getattr(msg, f"servo{servo}_raw", None)
        return None if value is None else int(value)

    def wait_servo_pwm(self, servo: int, pwm_us: int, timeout: float = 1.0) -> int | None:
        """
        Poll SERVO_OUTPUT_RAW until ``servo`` reports ``pwm_us`` or ``timeout`` expires.

        Telemetry sent before the command was applied may still be queued, so a single read
        can show the old value. Returns the last value seen (``pwm_us`` on success), or None
        if no SERVO_OUTPUT_RAW arrived at all.
        """
        last: int | None = None
        t0 = time.time()
        while time.time() - t0 < timeout:
            value = self.read_servo_pwm(servo, timeout=min(0.2, timeout))
            if value is not None:
                last = value
                if value == pwm_us:
                    break
        return last
