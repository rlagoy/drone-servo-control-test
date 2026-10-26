# Flap servo control: setup and test tutorial

This tutorial walks through testing one component of a fixed-wing drone: **flap control**.

```mermaid
flowchart LR
    PC["Computer<br/>servo-bench-test"] -- "USB · MAVLink DO_SET_SERVO" --> FC["SpeedyBee F405 Wing<br/>ArduPlane firmware"]
    FC -- "PWM, output S4" --> SERVO[Servo] --> FLAP[Flap]
    FC -. "COMMAND_ACK, SERVO_OUTPUT_RAW" .-> PC
```

We test it at two levels, the way you would on a real program:

1. **Part A: software sub-component test (no hardware).** The MAVLink code is tested against
   mocks and a simulated flight controller. It runs on any laptop and in CI on every commit.
2. **Part B: hardware-in-the-loop (HIL) test.** The same code drives the real flight controller
   and servo while you measure voltage, PWM, deflection angle and current.

```
 Requirements ─────────────────────────────── System test
     Design ──────────────────────── HIL bench test        (Part B)
        Component design ───── Component test vs. simulator (Part A)
              Code ───── Unit tests with mocks              (Part A)
```

Part A can be done with nothing but a computer. Sections 1-5 prepare the hardware for Part B.

Contents

0. [Bill of materials](#0-bill-of-materials)
1. [Prepare your computer](#1-prepare-your-computer-windows--macos--linux)
2. [Updating the SpeedyBee firmware](#2-updating-the-speedybee-firmware)
3. [Configure the flight controller](#3-configure-the-flight-controller)
4. [Wiring and power](#4-wiring-and-power)
5. [Check the connection](#5-check-the-connection)
6. [Part A: software sub-component test](#6-part-a-software-sub-component-test-no-hardware)
7. [Part B: hardware-in-the-loop bench test](#7-part-b-hardware-in-the-loop-bench-test)
8. [CI/CD](#8-cicd)
9. [Troubleshooting](#9-troubleshooting)

---

## 0. Bill of materials

| Item | Notes |
|---|---|
| SpeedyBee F405 Wing flight controller | Runs ArduPlane (section 2) |
| Standard PWM hobby servo + flap/horn | 1000-2000 µs; check its voltage rating |
| Power: 2S-6S LiPo, or bench supply set to ~7 V with current limit (~1 A) | USB alone does **not** power the servo rail |
| USB-C **data** cable | Many cheap cables are charge-only and won't enumerate |
| Multimeter | Voltage and current |
| Oscilloscope (optional) | To view the PWM waveform |
| Protractor / angle gauge | Flap deflection |
| Computer: Windows 10/11, macOS 12+, or Linux | |

---

## 1. Prepare your computer (Windows / macOS / Linux)

### 1.1 Install uv, git and the project

[uv](https://docs.astral.sh/uv/) installs the right Python version and all dependencies.

**Windows** (PowerShell):

```powershell
winget install --id=astral-sh.uv -e
winget install --id=Git.Git -e
```

**macOS:**

```bash
brew install uv git            # or: curl -LsSf https://astral.sh/uv/install.sh | sh
```

**Linux:**

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
sudo apt install git           # or your distro's package manager
```

Open a new terminal, then on any OS:

```bash
git clone https://github.com/rlagoy/drone-servo-control-test.git
cd drone-servo-control-test
uv sync
uv run pytest                  # should end with "passed"; hardware tests are deselected
```

### 1.2 Install a ground control station

You need one to flash firmware and set parameters.

| Tool | Windows | macOS | Linux |
|---|---|---|---|
| [Mission Planner](https://ardupilot.org/planner/docs/mission-planner-installation.html) | Yes (recommended) | No | No |
| [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html) | Yes | Yes | Yes (AppImage) |
| MAVProxy (command line, `uv sync --extra tools`) | Yes | Yes | Yes |

### 1.3 Serial port access

**Windows:** no driver needed for ArduPilot. The board appears in Device Manager under
*Ports (COM & LPT)* as a COM port, e.g. `COM5`.

**macOS:** no driver needed. The board appears as `/dev/cu.usbmodemXXXX` (there is also a
`/dev/tty.usbmodemXXXX`; use the `cu.` one).

**Linux:** the board appears as `/dev/ttyACM0`. Two one-time fixes:

```bash
sudo usermod -aG dialout $USER          # serial port permission (Arch/Fedora: uucp group)
                                        # then log out and back in
sudo systemctl disable --now ModemManager   # it grabs /dev/ttyACM* thinking it is a modem
```

For DFU flashing on Linux (section 2.2), add a udev rule so you don't need `sudo`:

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/49-stm32-dfu.rules
sudo udevadm control --reload-rules
```

**All platforms:** find the board with

```bash
uv run servo-bench-test --list-ports    # the flight controller is marked with *
```

---

## 2. Updating the SpeedyBee firmware

The test uses the MAVLink command `MAV_CMD_DO_SET_SERVO`, which **ArduPilot** supports.
SpeedyBee F405 Wing boards often ship with INAV or Betaflight, which do not. Use the ArduPilot
board target **`SpeedyBeeF405WING`** and the **ArduPlane** vehicle firmware (stable release).

### 2.1 Check what is installed

* If the board already runs ArduPilot, connect with your ground station: the version (e.g.
  `ArduPlane V4.x.y`) is shown in the Mission Planner title bar / message tab, in QGC under
  *Vehicle Setup → Summary*, or with `version` in MAVProxy.
* If INAV/Betaflight Configurator connects to it, it is not running ArduPilot yet → 2.2.

### 2.2 First-time install (from INAV/Betaflight, or a bricked board)

This writes the ArduPilot bootloader and firmware through the STM32's built-in DFU mode.

1. Download **`arduplane_with_bl.hex`** from
   `https://firmware.ardupilot.org/Plane/stable/SpeedyBeeF405WING/`.
2. Enter DFU mode: unplug everything, **hold the BOOT button**, plug in USB, release.
   (The board shows up as "STM32 BOOTLOADER" / "DFU in FS Mode", USB ID `0483:df11`.)
3. Flash, using any one of:
   * **STM32CubeProgrammer** (Windows/macOS/Linux): connect via *USB*, open the `.hex`,
     *Download*. It installs the DFU driver on Windows.
   * **INAV or Betaflight Configurator** (all OSes): *Firmware Flasher → Load Firmware
     [Local]*, choose the `.hex`, tick **Full chip erase**, *Flash Firmware*.
     On Windows, if the board isn't seen in DFU, run *ImpulseRC Driver Fixer* or Zadig (WinUSB).
   * **dfu-util** (macOS `brew install dfu-util`, Linux `apt install dfu-util`):

     ```bash
     uvx --from intelhex hex2bin.py arduplane_with_bl.hex arduplane_with_bl.bin
     dfu-util -a 0 --dfuse-address 0x08000000:leave -D arduplane_with_bl.bin
     ```
4. Unplug and re-plug USB **without** holding BOOT. The board now enumerates as an ArduPilot
   device (`uv run servo-bench-test --list-ports` marks it with `*`).
5. Connect your ground station once to let it fetch parameters, then continue to section 3.

### 2.3 Routine updates (ArduPilot already installed)

Once the ArduPilot bootloader is on the board, no DFU or button is needed.

**Before updating, back up the parameters:**
Mission Planner *Config → Full Parameter List → Save to file*; QGC *Vehicle Setup →
Parameters → Tools → Save to file*; or MAVProxy `param save backup.param`.

Then update with one of:

* **Mission Planner** (Windows): disconnect, *Setup → Install Firmware*, pick the *Plane* icon
  and accept the board it detects (`SpeedyBeeF405WING`).
* **QGroundControl** (all OSes): *Vehicle Setup → Firmware*, then unplug/replug the board when
  asked, choose *ArduPilot*, *ChibiOS*, vehicle *Plane*, *OK*.
* **Command line** (all OSes): download `arduplane.apj` from the same firmware page and the
  ArduPilot uploader script, then:

  ```bash
  curl -LO https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/scripts/uploader.py
  uv run python uploader.py --port COM5 arduplane.apj      # or /dev/cu.usbmodem1101, /dev/ttyACM0
  ```

Close every ground station before using the command line: only one program can open the
serial port at a time.

After the update: reconnect, confirm the new version, compare parameters against your backup
(re-load it if anything was reset), reboot, and run section 5 again.

---

## 3. Configure the flight controller

Set these parameters. They are also in
[`speedybee_f405wing_servo_test.param`](speedybee_f405wing_servo_test.param) so you can load
them in one step.

| Parameter | Value | Why |
|---|---|---|
| `SERIAL0_PROTOCOL` | 2 | USB port speaks MAVLink 2 (default) |
| `SERVO4_FUNCTION` | 0 (Disabled) | ArduPilot leaves output S4 alone, so `DO_SET_SERVO` from the PC controls it. On a flying aircraft this would be 2 (Flap); then the autopilot drives it and overrides our command. |
| `SERVO4_MIN` / `SERVO4_MAX` / `SERVO4_TRIM` | 1000 / 2000 / 1500 | Servo travel limits and neutral |
| `SERVO4_REVERSED` | 0 | |
| `BRD_SAFETY_DEFLT` | 0 | Board has no safety switch; don't hold outputs off waiting for one |
| `SR0_RC_CHAN` | 10 | Sends `SERVO_OUTPUT_RAW` over USB at 10 Hz so we can read back the output. The bench script also requests this itself; this is the fallback. (Newer firmware may name stream-rate parameters differently: search for `RC_CHAN`.) |

How to set them:

* **Mission Planner:** *Config → Full Parameter List → Load from file* (select the `.param`),
  *Write Params*.
* **QGroundControl:** *Vehicle Setup → Parameters*, search each name, edit, *Save*.
* **MAVProxy:**

  ```bash
  uv sync --extra tools
  uv run mavproxy.py --master=COM5            # or /dev/cu.usbmodem1101, /dev/ttyACM0
  param load docs/speedybee_f405wing_servo_test.param
  reboot
  ```

Reboot the board, then check: in Mission Planner *Data → Servo/Relay* or QGC *Analyze Tools →
MAVLink Inspector → SERVO_OUTPUT_RAW*, you should see output 4.

**Then close the ground station.** Only one program at a time can hold the serial port on any OS.

---

## 4. Wiring and power

> Safety: no propeller on any motor. Use a current-limited bench supply where possible.
> Keep fingers clear of the flap when commanding it.

1. Plug the servo into the **S4** header: signal (white/yellow) to S4 signal pin, red to the
   servo-rail `+`, black/brown to `-`. Check polarity against the board silk screen.
2. Check the board's **servo-rail voltage selection** (jumper or solder pads, see the SpeedyBee
   F405 Wing manual for your board revision) matches your servo's rating, e.g. 5 V.
3. Connect power to the battery pads/connector: battery, or bench supply at ~7 V, current
   limit ~1 A.
4. Connect USB to the computer.

The servo rail is powered from the board's BEC, which runs from the battery input.
**With USB only, the FC talks to the PC but the servo won't move.**

---

## 5. Check the connection

```bash
uv run servo-bench-test --list-ports
uv run pytest -m hardware --device auto -v
```

Expect `4 passed`: one heartbeat test, and servo moves to 1000, 2000 and 1500 µs, each read back
from the FC. If anything fails, see [Troubleshooting](#9-troubleshooting).

---

## 6. Part A: software sub-component test (no hardware)

**Goal:** show the MAVLink control code is correct *before* touching hardware, and keep it
correct on every commit.

```bash
uv run pytest -v
```

What runs:

| File | Level | What it shows |
|---|---|---|
| `tests/test_mavlink_device_unit.py` | Unit | pymavlink is replaced by a `unittest.mock`. Tests check the exact `COMMAND_LONG` fields sent for `DO_SET_SERVO`, PWM range checks (800-2200 µs), ACK handling, timeouts and errors when not connected. |
| `tests/test_mavlink_device_sim.py` | Component | Real MAVLink packets to `tests/fake_autopilot.py`, a simulated ArduPilot that sends heartbeats, ACKs commands and reports `SERVO_OUTPUT_RAW`. Also tests a FC that *denies* the command and one that never ACKs. |
| `tests/test_main.py` | Component | The whole bench procedure against the simulator: commands go out in the order 1000 → 2000 → 1500 µs and the flap ends at neutral. |
| `tests/test_ports.py` | Unit | Auto-detection of the FC's port using Windows, macOS and Linux port lists. |
| `tests/test_hardware.py` | HIL | Skipped here: shown as *deselected*. |

Talking points:

* **Why mock?** Unit tests isolate one class; they are fast (milliseconds) and pinpoint failures.
* **Why a simulator?** The component is tested through its real interface (MAVLink over a
  link), so serialization, ACK matching and timing are exercised. The code can't tell the
  simulator from the SpeedyBee.
* **Negative tests** (denied command, lost ACK, no heartbeat, out-of-range PWM) are easy in
  software and hard or unsafe on hardware.

Show coverage:

```bash
uv run pytest --cov
```

**Demonstrate a failing test.** In `src/drone_servo_control_test/main.py`, in `build_steps`,
change the third step's PWM from `pwm_min` to `pwm_max`, then run `uv run pytest -v`.
`test_main.py` fails and shows the wrong sequence. Revert the change.

**Watch the bench script run against the simulator.** Two terminals:

```bash
uv run python tests/fake_autopilot.py                     # terminal 1: the "flight controller"
uv run servo-bench-test udpin:127.0.0.1:14550             # terminal 2: press Enter at each step
```

Terminal 1 prints every `DO_SET_SERVO` it receives.

---

## 7. Part B: hardware-in-the-loop bench test

**Goal:** check that the real hardware meets its requirements, with the same code verified in
Part A.

Make sure the ground station is closed and the board is powered (section 4). Then:

```bash
uv run servo-bench-test                         # auto-detects the port
uv run servo-bench-test COM5                    # Windows, explicit
uv run servo-bench-test /dev/cu.usbmodem1101    # macOS
uv run servo-bench-test /dev/ttyACM0            # Linux
```

Options: `--servo N` (output, default 4), `--min/--max/--center` (PWM, defaults 1000/2000/1500),
`--yes` (no pauses).

At each command the script prints whether the FC accepted it (`COMMAND_ACK: ACCEPTED`) and
the PWM the FC reports on that output (`FC reports SERVO4 = 1000 us`). Press Enter after each
measurement.

| Step | Script does | Measure | Expected | Measured |
|---|---|---|---|---|
| 1 | — | Supply voltage at FC input | ~7 V | |
| 2 | — | Servo rail (BEC) voltage, `+` to `-` at the S4 header | 5 V (or as selected) | |
| 3 | S4 = 1000 µs | Flap angle (up); scope on S4 signal: pulse width and period | 1000 µs pulse, ~20 ms period (50 Hz) | |
| 4 | S4 = 2000 µs | Flap angle (down); pulse width | 2000 µs pulse | |
| 5 | S4 = 1500 µs | Flap angle | Neutral | |
| 6 | — | Servo current while holding the horn against deflection | Stall current, within servo/BEC rating | |
| 7 | — | Current with servo unplugged | FC idle current | |

Then run the automated HIL tests:

```bash
uv run pytest -m hardware --device auto -v
```

Compare with Part A: same `MAVLinkDevice` code, same checks, now against real hardware. These
can't run in CI, which is why the software tests exist.

---

## 8. CI/CD

* **Pre-commit** (`uv run pre-commit install`, once): every `git commit` runs ruff
  (lint + format), mypy, YAML/TOML checks and lockfile check locally.
* **CI** (`.github/workflows/ci.yml`), on every pull request and push to `main`:
  1. *lint*: `ruff check`, `ruff format --check`, `mypy`
  2. *test*: `pytest --cov` on Windows, macOS and Linux × Python 3.11-3.14. Hardware tests
     are deselected; the simulator runs on any OS.
  3. *build*: `uv build` produces the wheel and sdist as artifacts.
* **CD** (`.github/workflows/release.yml`): pushing a tag like `v0.2.0` re-runs tests, builds,
  and publishes a GitHub Release with the wheel attached.
* **Dependabot** opens monthly PRs for GitHub Action and Python dependency updates; CI checks
  each one.

Flow for a change: branch → commit (pre-commit runs) → push → open PR → CI green → review →
merge → (optionally) tag a release. Then run Part B on the bench before flight.

---

## 9. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `No flight controller found on USB` | Charge-only cable; board not powered by USB; board still in DFU (replug without BOOT); not ArduPilot firmware (section 2). Try `--list-ports` and pass the port explicitly. |
| `No heartbeat from ... within 5 s` | Wrong port; a ground station or MAVProxy still has the port open; `SERIAL0_PROTOCOL` not 2. |
| Linux: `Permission denied: /dev/ttyACM0` | Not in `dialout` group (log out/in after adding). |
| Linux: port vanishes or no heartbeat for ~10 s after plugging in | ModemManager probing the port: `sudo systemctl disable --now ModemManager`. |
| macOS: port busy | Use `/dev/cu.usbmodem…`, not `/dev/tty.usbmodem…`; close other serial apps. |
| Windows: `could not open port 'COM5': PermissionError` | Port open in Mission Planner/QGC, or a previous script still running. Close it and replug. |
| `COMMAND_ACK: NOT ACCEPTED` | `SERVO4_FUNCTION` isn't 0, so the autopilot owns that output. |
| ACK accepted, `FC reports SERVO4 = 1000 us`, but servo doesn't move | No power on the servo rail (USB only, battery not connected) or wrong rail voltage jumper; servo in wrong header/polarity. |
| No `FC reports ...` line | `SERVO_OUTPUT_RAW` not streamed: set `SR0_RC_CHAN` to 10 and reboot. |
| Board not seen in DFU | Hold BOOT *while* plugging in; Windows: install WinUSB driver via STM32CubeProgrammer/Zadig/ImpulseRC; Linux: udev rule (section 1.3). |
| Ground station can't find board after flashing | Unplug/replug; you may have flashed `arduplane.hex` without the bootloader. Re-flash `arduplane_with_bl.hex` via DFU. |
| `pwm_us ... outside allowed range` | The library refuses values outside 800-2200 µs to protect the servo. |
