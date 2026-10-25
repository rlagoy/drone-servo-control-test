"""Tests for the bench-test procedure and CLI, run against the simulated FC."""

from __future__ import annotations

from types import SimpleNamespace

import pytest
from fake_autopilot import FakeAutopilot

from drone_servo_control_test import main as bench


def test_bench_sequence_against_simulator(fake_fc, capsys):
    prompts: list[str] = []

    commanded = bench.main(fake_fc.connection_string, prompt=prompts.append)

    assert commanded == [1000, 2000, 1500]
    assert [pwm for _, pwm in fake_fc.commands] == [1000, 2000, 1500]
    assert fake_fc.servo_pwm[3] == 1500  # flap returned to neutral
    assert len(prompts) == 7
    assert prompts[0].startswith("\n1. Power on")
    out = capsys.readouterr().out
    assert "Heartbeat received!" in out
    assert out.count("COMMAND_ACK: ACCEPTED") == 3


def test_bench_custom_servo_and_limits(fake_fc):
    commanded = bench.main(
        fake_fc.connection_string,
        servo=2,
        pwm_min=1100,
        pwm_max=1900,
        pwm_center=1450,
        prompt=lambda _msg: None,
    )
    assert commanded == [1100, 1900, 1450]
    assert fake_fc.commands == [(2, 1100), (2, 1900), (2, 1450)]


def test_bench_times_out_without_heartbeat():
    fc = FakeAutopilot()  # never started
    try:
        with pytest.raises(TimeoutError, match="No heartbeat"):
            bench.main(fc.connection_string, heartbeat_timeout=0.2, prompt=print)
    finally:
        fc.stop()


def test_build_steps_order():
    steps = bench.build_steps(1000, 2000, 1500)
    assert [s.pwm_us for s in steps if s.pwm_us is not None] == [1000, 2000, 1500]


# --- CLI ----------------------------------------------------------------------


def test_parser_defaults():
    args = bench.build_parser().parse_args([])
    assert args.device == "auto"
    assert (args.baud, args.servo) == (115200, 4)
    assert (args.pwm_min, args.pwm_max, args.pwm_center) == (1000, 2000, 1500)
    assert not args.yes


def test_cli_yes_runs_against_simulator(fake_fc, capsys):
    assert bench.cli([fake_fc.connection_string, "--yes"]) == 0
    assert [pwm for _, pwm in fake_fc.commands] == [1000, 2000, 1500]
    assert "7. Remove the servo" in capsys.readouterr().out


def test_cli_auto_without_fc_reports_error(monkeypatch, capsys):
    monkeypatch.setattr("drone_servo_control_test.ports.list_ports.comports", lambda: [])
    assert bench.cli([]) == 2
    assert "No flight controller" in capsys.readouterr().err


def test_cli_heartbeat_timeout_exit_code(monkeypatch, capsys):
    def no_heartbeat(*_args, **_kwargs):
        raise TimeoutError("No heartbeat from X")

    monkeypatch.setattr(bench, "main", no_heartbeat)
    assert bench.cli(["COM5"]) == 1
    assert "No heartbeat" in capsys.readouterr().err


def test_cli_list_ports(monkeypatch, capsys):
    ports = [
        SimpleNamespace(device="COM1", vid=None, pid=None, description="Serial"),
        SimpleNamespace(device="COM5", vid=0x1209, pid=0x5741, description="ArduPilot"),
    ]
    monkeypatch.setattr("drone_servo_control_test.main.list_ports.comports", lambda: ports)
    monkeypatch.setattr("drone_servo_control_test.ports.list_ports.comports", lambda: ports)
    assert bench.cli(["--list-ports"]) == 0
    out = capsys.readouterr().out.splitlines()
    assert out == ["  COM1  [----:----]  Serial", "* COM5  [1209:5741]  ArduPilot"]


def test_cli_list_ports_none(monkeypatch, capsys):
    monkeypatch.setattr("drone_servo_control_test.main.list_ports.comports", lambda: [])
    assert bench.cli(["--list-ports"]) == 0
    assert "No serial ports found." in capsys.readouterr().out
