"""
A simulated flight controller for software-only (no hardware) tests.

It speaks real MAVLink over a local UDP socket, just like ArduPilot does over USB:

* sends HEARTBEAT and SERVO_OUTPUT_RAW at 10 Hz,
* answers COMMAND_LONG(MAV_CMD_DO_SET_SERVO) with COMMAND_ACK,
* remembers the PWM commanded on each servo output.

The code under test cannot tell the difference between this and the SpeedyBee, which is
what lets us test the MAVLink component on any laptop or in CI.
"""

from __future__ import annotations

import contextlib
import socket
import threading
import time
from enum import Enum

from pymavlink import mavutil

mavlink = mavutil.mavlink


class AckMode(Enum):
    """How the fake FC responds to DO_SET_SERVO."""

    ACCEPT = "accept"  # apply the PWM and ACK with MAV_RESULT_ACCEPTED
    DENY = "deny"  # ACK with MAV_RESULT_DENIED, output unchanged
    SILENT = "silent"  # apply the PWM but never send an ACK (lossy link)


DEFAULT_PORT = 14550


def free_udp_port() -> int:
    """Ask the OS for an unused local UDP port."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


class UdpLink:
    """
    MAVLink over a UDP socket bound to its own ephemeral port, sending to ``127.0.0.1:port``.

    Used instead of pymavlink's ``udpout:`` connection, which on Windows binds its socket to
    the *destination* port: the simulator and the device under test then share one port and
    the device's replies loop back to itself instead of reaching the simulator.
    """

    def __init__(self, port: int, source_system: int, source_component: int) -> None:
        self._dest = ("127.0.0.1", port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("127.0.0.1", 0))
        if hasattr(socket, "SIO_UDP_CONNRESET"):
            # Windows: don't fail the next recv because an earlier send hit a closed port.
            self._sock.ioctl(socket.SIO_UDP_CONNRESET, False)  # type: ignore[attr-defined]
        self.mav = mavlink.MAVLink(self, srcSystem=source_system, srcComponent=source_component)

    def write(self, buf: bytes) -> None:
        """Send one encoded MAVLink packet (called by ``self.mav``'s ``*_send`` methods)."""
        # Nobody listening yet is fine: UDP is lossy anyway.
        with contextlib.suppress(ConnectionError):
            self._sock.sendto(buf, self._dest)

    def recv_messages(self, timeout: float) -> list[object]:
        """Return the MAVLink messages in the next datagram, or [] after ``timeout`` seconds."""
        self._sock.settimeout(timeout)
        try:
            data = self._sock.recv(65535)
        except (TimeoutError, ConnectionError):
            return []
        return self.mav.parse_buffer(data) or []

    def close(self) -> None:
        self._sock.close()


class FakeAutopilot:
    """
    Background-thread MAVLink autopilot.

    The device under test binds ``self.connection_string`` (``udpin:...``); the fake
    FC sends to it, mirroring how SITL/ArduPilot talks to a ground station.
    """

    def __init__(
        self,
        ack_mode: AckMode = AckMode.ACCEPT,
        rate_hz: float = 10.0,
        port: int | None = None,
    ) -> None:
        self.port = port if port is not None else free_udp_port()
        self.connection_string = f"udpin:127.0.0.1:{self.port}"
        self.ack_mode = ack_mode
        self.period = 1.0 / rate_hz
        self.servo_pwm = [0] * 16
        self.commands: list[tuple[int, int]] = []  # (servo, pwm) received, in order
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._link = UdpLink(self.port, source_system=1, source_component=1)

    # --- lifecycle -------------------------------------------------------------

    def start(self) -> FakeAutopilot:
        self._thread = threading.Thread(target=self._run, name="fake-autopilot", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
        self._link.close()

    def __enter__(self) -> FakeAutopilot:
        return self.start()

    def __exit__(self, *exc: object) -> None:
        self.stop()

    # --- simulation loop -------------------------------------------------------

    def _run(self) -> None:
        next_tx = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            if now >= next_tx:
                self._send_telemetry()
                next_tx = now + self.period
            for msg in self._link.recv_messages(timeout=0.01):
                if msg.get_type() == "COMMAND_LONG":  # type: ignore[attr-defined]
                    self._handle_command(msg)

    def _send_telemetry(self) -> None:
        self._link.mav.heartbeat_send(
            mavlink.MAV_TYPE_FIXED_WING, mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 0
        )
        time_usec = int(time.monotonic() * 1e6) & 0xFFFFFFFF
        self._link.mav.servo_output_raw_send(time_usec, 0, *self.servo_pwm[:8])

    def _handle_command(self, msg: object) -> None:
        command = int(msg.command)  # type: ignore[attr-defined]
        if command == mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
            self._link.mav.command_ack_send(command, mavlink.MAV_RESULT_ACCEPTED)
            return
        if command != mavlink.MAV_CMD_DO_SET_SERVO:
            self._link.mav.command_ack_send(command, mavlink.MAV_RESULT_UNSUPPORTED)
            return
        servo = int(msg.param1)  # type: ignore[attr-defined]
        pwm = int(msg.param2)  # type: ignore[attr-defined]
        self.commands.append((servo, pwm))
        if self.ack_mode is AckMode.DENY or not 1 <= servo <= len(self.servo_pwm):
            self._link.mav.command_ack_send(command, mavlink.MAV_RESULT_DENIED)
            return
        self.servo_pwm[servo - 1] = pwm
        self._send_telemetry()  # report the new output right away
        if self.ack_mode is AckMode.ACCEPT:
            self._link.mav.command_ack_send(command, mavlink.MAV_RESULT_ACCEPTED)


if __name__ == "__main__":
    # Stand-alone simulator for live demos:
    #   terminal 1: uv run python tests/fake_autopilot.py
    #   terminal 2: uv run servo-bench-test udpin:127.0.0.1:14550
    import argparse

    parser = argparse.ArgumentParser(description="Simulated ArduPilot flight controller")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--mode", choices=[m.value for m in AckMode], default="accept")
    args = parser.parse_args()

    with FakeAutopilot(AckMode(args.mode), port=args.port) as fc:
        print(f"Fake FC running. Connect with: servo-bench-test {fc.connection_string}", flush=True)
        print("Ctrl+C to stop.", flush=True)
        last: list[tuple[int, int]] = []
        try:
            while True:
                time.sleep(0.1)
                if fc.commands != last:
                    for servo, pwm in fc.commands[len(last) :]:
                        print(f"  DO_SET_SERVO servo={servo} pwm={pwm} us", flush=True)
                    last = list(fc.commands)
        except KeyboardInterrupt:
            pass
