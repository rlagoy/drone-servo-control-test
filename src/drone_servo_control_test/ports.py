"""Cross-platform discovery of a flight controller's USB serial port."""

from __future__ import annotations

from typing import Any

from serial.tools import list_ports

#: USB vendor IDs used by ArduPilot (pid.codes) and STM32 virtual COM ports.
FC_USB_VIDS = frozenset({0x1209, 0x0483})


class PortNotFoundError(RuntimeError):
    """No (or no unambiguous) flight controller serial port was found."""


def describe(port: Any) -> str:
    """One-line description of a pyserial ``ListPortInfo``."""
    vid = f"{port.vid:04x}" if port.vid is not None else "----"
    pid = f"{port.pid:04x}" if port.pid is not None else "----"
    return f"{port.device}  [{vid}:{pid}]  {port.description}"


def candidate_ports(ports: list[Any] | None = None) -> list[Any]:
    """Return ports that look like a flight controller, sorted by device name."""
    if ports is None:
        ports = list(list_ports.comports())
    found = [p for p in ports if p.vid in FC_USB_VIDS]
    # macOS lists each USB modem twice (/dev/tty.* and /dev/cu.*); keep the cu.* one.
    names = {p.device for p in found}
    found = [
        p
        for p in found
        if not (
            p.device.startswith("/dev/tty.")
            and p.device.replace("/dev/tty.", "/dev/cu.", 1) in names
        )
    ]
    return sorted(found, key=lambda p: p.device)


def resolve_device(device: str, ports: list[Any] | None = None) -> str:
    """
    Turn ``"auto"`` into a concrete port name; return anything else unchanged.

    Raises
    ------
    PortNotFoundError
        If ``device == "auto"`` and zero or several candidate ports are found.
    """
    if device != "auto":
        return device
    found = candidate_ports(ports)
    if not found:
        raise PortNotFoundError(
            "No flight controller found on USB. Check the cable (must be a data cable), "
            "close any ground station using the port, or pass the port explicitly "
            "(e.g. COM5, /dev/cu.usbmodem1101, /dev/ttyACM0)."
        )
    if len(found) > 1:
        names = ", ".join(p.device for p in found)
        raise PortNotFoundError(f"Several possible flight controllers ({names}); pick one.")
    return str(found[0].device)
