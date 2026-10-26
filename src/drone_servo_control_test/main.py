"""Interactive bench test that exercises the flap servo through the flight controller."""

from __future__ import annotations

import argparse
import sys
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass

from serial.tools import list_ports

from .mavlink_device import MAVLinkDevice, ack_ok
from .ports import PortNotFoundError, candidate_ports, describe, resolve_device

Prompt = Callable[[str], object]


@dataclass(frozen=True)
class Step:
    """One bench-test step: an optional PWM command followed by an operator prompt."""

    instruction: str
    pwm_us: int | None = None


def build_steps(pwm_min: int, pwm_max: int, pwm_center: int) -> list[Step]:
    """Return the ordered bench-test procedure."""
    return [
        Step("Power on the FC with 7 V (battery or bench supply). Press Enter to continue..."),
        Step("Measure the 5 V BEC output on the servo rail..."),
        Step(
            f"Setting PWM to {pwm_min} us. Measure upward deflection angle and PWM waveform.",
            pwm_min,
        ),
        Step(
            f"Setting PWM to {pwm_max} us. Measure downward deflection angle and PWM waveform.",
            pwm_max,
        ),
        Step(f"Setting PWM to {pwm_center} us. Measure deflection angle.", pwm_center),
        Step("Deflect the servo horn by hand and measure current."),
        Step("Remove the servo and measure current."),
    ]


def main(  # noqa: PLR0913
    dev: str,
    *,
    baud: int = 115200,
    servo: int = 4,
    pwm_min: int = 1000,
    pwm_max: int = 2000,
    pwm_center: int = 1500,
    heartbeat_timeout: float = 5.0,
    prompt: Prompt = input,
) -> list[int]:
    """
    Run the bench test.

    Returns the list of PWM values commanded, in order (useful for testing).
    """
    commanded: list[int] = []
    start_time = time.time()
    with MAVLinkDevice(dev, baud=baud) as mav_device:
        print(f"Waiting for heartbeat on {dev}")
        if mav_device.wait_heartbeat(timeout=heartbeat_timeout) is None:
            raise TimeoutError(f"No heartbeat from {dev} within {heartbeat_timeout} s")
        print("Heartbeat received!")
        print(f"Connected within {time.time() - start_time:.2f} seconds.")
        if not mav_device.stream_servo_outputs():
            print("  (SERVO_OUTPUT_RAW stream request not accepted; set SR0_RC_CHAN instead)")

        for n, step in enumerate(build_steps(pwm_min, pwm_max, pwm_center), start=1):
            if step.pwm_us is not None:
                ack = mav_device.set_servo(servo, step.pwm_us)
                commanded.append(step.pwm_us)
                status = "ACCEPTED" if ack_ok(ack) else f"NOT ACCEPTED ({ack})"
                print(f"  COMMAND_ACK: {status}")
                reported = mav_device.wait_servo_pwm(servo, step.pwm_us)
                if reported is not None:
                    print(f"  FC reports SERVO{servo} = {reported} us")
            prompt(f"\n{n}. {step.instruction}")
    return commanded


def build_parser() -> argparse.ArgumentParser:
    """Command-line interface definition."""
    p = argparse.ArgumentParser(description="Flap servo bench test (MAVLink DO_SET_SERVO)")
    p.add_argument(
        "device",
        nargs="?",
        default="auto",
        help="Serial port or pymavlink connection string, e.g. COM5, /dev/cu.usbmodem1101, "
        "/dev/ttyACM0, udpin:127.0.0.1:14550 (default: auto-detect)",
    )
    p.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    p.add_argument("--servo", type=int, default=4, help="1-based servo output (default 4)")
    p.add_argument("--min", dest="pwm_min", type=int, default=1000, help="Min PWM us")
    p.add_argument("--max", dest="pwm_max", type=int, default=2000, help="Max PWM us")
    p.add_argument("--center", dest="pwm_center", type=int, default=1500, help="Center PWM us")
    p.add_argument(
        "--yes", action="store_true", help="Don't wait for Enter between steps (automated run)"
    )
    p.add_argument("--list-ports", action="store_true", help="List serial ports and exit")
    return p


def cli(argv: Sequence[str] | None = None) -> int:
    """Entry point for the ``servo-bench-test`` command."""
    args = build_parser().parse_args(argv)

    if args.list_ports:
        ports = list(list_ports.comports())
        fc = {p.device for p in candidate_ports(ports)}
        for port in ports:
            print(("* " if port.device in fc else "  ") + describe(port))
        if not ports:
            print("No serial ports found.")
        return 0

    try:
        device = resolve_device(args.device)
    except PortNotFoundError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    prompt: Prompt = print if args.yes else input
    try:
        main(
            device,
            baud=args.baud,
            servo=args.servo,
            pwm_min=args.pwm_min,
            pwm_max=args.pwm_max,
            pwm_center=args.pwm_center,
            prompt=prompt,
        )
    except TimeoutError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(cli())
