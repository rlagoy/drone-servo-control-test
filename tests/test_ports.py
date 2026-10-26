"""Serial-port auto-detection, with port lists as Windows, macOS and Linux report them."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from drone_servo_control_test.ports import (
    PortNotFoundError,
    candidate_ports,
    describe,
    resolve_device,
)


def port(device, vid=0x1209, pid=0x5741, description="ArduPilot"):
    return SimpleNamespace(device=device, vid=vid, pid=pid, description=description)


BLUETOOTH = port("/dev/cu.Bluetooth-Incoming-Port", vid=None, pid=None, description="n/a")

PORT_LISTS = {
    "windows": ([port("COM1", vid=None, pid=None), port("COM5")], "COM5"),
    "macos": (
        [BLUETOOTH, port("/dev/tty.usbmodem1101"), port("/dev/cu.usbmodem1101")],
        "/dev/cu.usbmodem1101",
    ),
    "linux": ([port("/dev/ttyS0", vid=None, pid=None), port("/dev/ttyACM0")], "/dev/ttyACM0"),
    "stm32-vcp": ([port("/dev/ttyACM0", vid=0x0483, pid=0x5740)], "/dev/ttyACM0"),
}


@pytest.mark.parametrize(("ports", "expected"), PORT_LISTS.values(), ids=PORT_LISTS.keys())
def test_auto_finds_flight_controller(ports, expected):
    assert resolve_device("auto", ports) == expected


@pytest.mark.parametrize("explicit", ["COM7", "/dev/ttyUSB0", "udpin:127.0.0.1:14550"])
def test_explicit_device_passes_through(explicit):
    assert resolve_device(explicit, []) == explicit


def test_no_flight_controller():
    with pytest.raises(PortNotFoundError, match="No flight controller"):
        resolve_device("auto", [BLUETOOTH, port("COM1", vid=0x10C4)])


def test_ambiguous_flight_controllers():
    with pytest.raises(PortNotFoundError, match="COM5, COM6"):
        resolve_device("auto", [port("COM6"), port("COM5")])


def test_candidate_ports_queries_system_when_not_given(monkeypatch):
    monkeypatch.setattr(
        "drone_servo_control_test.ports.list_ports.comports", lambda: [port("COM9")]
    )
    assert [p.device for p in candidate_ports()] == ["COM9"]


def test_describe():
    assert describe(port("COM5")) == "COM5  [1209:5741]  ArduPilot"
    assert describe(BLUETOOTH).startswith("/dev/cu.Bluetooth-Incoming-Port  [----:----]")
