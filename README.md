# drone-servo-control-test

[![ci](https://github.com/rlagoy/drone-servo-control-test/actions/workflows/ci.yml/badge.svg)](https://github.com/rlagoy/drone-servo-control-test/actions/workflows/ci.yml)

Flap-control component for a fixed-wing drone, plus the tests that verify it: a computer sends
a MAVLink command to an ArduPilot flight controller (SpeedyBee F405 Wing), which outputs PWM to a
servo that deflects the flap.

```mermaid
flowchart LR
    PC["PC (Windows / macOS / Linux)<br/>drone_servo_control_test"] -- "USB · MAVLink<br/>DO_SET_SERVO" --> FC["SpeedyBee F405 Wing<br/>ArduPlane"]
    FC -- "COMMAND_ACK<br/>SERVO_OUTPUT_RAW" --> PC
    FC -- "PWM on S4<br/>1000-2000 µs" --> S[Servo] --> F[Flap]
```

The repo shows two levels of testing:

| Level | What runs | Needs hardware | Command |
|---|---|---|---|
| Unit | `MAVLinkDevice` with pymavlink mocked out | No | `uv run pytest tests/test_mavlink_device_unit.py` |
| Component (software) | Real MAVLink against a **simulated flight controller** | No | `uv run pytest` |
| Hardware-in-the-loop | Real SpeedyBee + servo, bench measurements | Yes | `uv run servo-bench-test` / `uv run pytest -m hardware --device auto` |

**Full walkthrough, including flight controller setup and firmware updates:
[docs/SETUP_AND_TUTORIAL.md](docs/SETUP_AND_TUTORIAL.md).**

## Quick start

Install [uv](https://docs.astral.sh/uv/getting-started/installation/), then:

```bash
git clone https://github.com/rlagoy/drone-servo-control-test.git
cd drone-servo-control-test
uv sync                       # creates .venv with Python + all dependencies
uv run pytest                 # software tests, no hardware needed
```

With the flight controller plugged in:

```bash
uv run servo-bench-test --list-ports   # the flight controller is marked with *
uv run servo-bench-test                # auto-detect the port, or pass COM5 / /dev/ttyACM0 / ...
```

No hardware but want to see the bench test run? Start the simulator in one terminal and point
the bench test at it from another:

```bash
uv run python tests/fake_autopilot.py
uv run servo-bench-test udpin:127.0.0.1:14550
```

## Layout

```
src/drone_servo_control_test/
    mavlink_device.py   MAVLinkDevice: connect, heartbeat, COMMAND_LONG, DO_SET_SERVO, telemetry
    ports.py            cross-platform auto-detection of the flight controller's USB port
    main.py             servo-bench-test: interactive hardware bench procedure
tests/
    fake_autopilot.py   simulated ArduPilot FC (MAVLink over local UDP)
    test_*_unit.py      mocked unit tests
    test_*_sim.py       component tests against the simulator
    test_main.py        bench procedure + CLI against the simulator
    test_ports.py       port detection with Windows/macOS/Linux port lists
    test_hardware.py    HIL tests (skipped unless --device is given)
docs/
    SETUP_AND_TUTORIAL.md
    speedybee_f405wing_servo_test.param
```

## Development

```bash
uv run pre-commit install     # once: run ruff, mypy, etc. on every commit
uv run ruff check . && uv run ruff format .
uv run mypy
uv run pytest --cov
```

Contributions: branch, open a pull request, and make sure CI is green. CI
(`.github/workflows/ci.yml`) runs lint + type checks and the test suite on Windows, macOS and
Linux for Python 3.11-3.14, then builds the package. Pushing a `v*` tag creates a GitHub Release
with the wheel and sdist attached.
