# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

AUV Control Board: a motion coprocessor for Autonomous Underwater Vehicles with fixed-position thrusters (up to 8), controlled over USB by a main computer (Jetson/Raspberry Pi). Two hardware revisions exist and are built from the same firmware source: **v1** (Adafruit ItsyBitsy M4, SAMD51) and **v2** (WeAct Studio Black Pill, STM32F411).

Full documentation lives in `docs/` (mkdocs project, published at https://mb3hel.github.io/AUVControlBoard/). The communication protocol and message formats are specified in `docs/docs/user_guide/comm_protocol.md` and `messages.md` — firmware (`cmdctrl.c`/`pccomm.c`) and the Python interface (`iface/control_board.py`) must stay in sync with these.

## Build Commands

### Firmware (run in `firmware/`)

Requires CMake ≥ 3.20, Ninja, and the GNU Arm toolchain (`arm-none-eabi-gcc`) on PATH.

```sh
cmake --preset=v1                  # or v2
cmake --build --preset=v1-debug    # configs: debug, release, minsizerel, relwithdebinfo
```

Flash (board must be in bootloader mode; `reboot_bootloader.py` gets it there if firmware is already running):

```sh
python3 flash.py [v1|v2] [Debug|Release|...] -u [tool]
# tools: v1 → bossa or uf2conv; v2 → dfu-util or stm32-dfu
```

There is no test suite. Verification is done by running interface scripts against a real board or the simulator.

### Interface scripts (run in `iface/`)

Scripts are never run directly — `launch.py` connects to the board (or simulator), applies a vehicle configuration, and calls the script's `run(cb, s)` function:

```sh
python3 launch.py example/mode_local.py            # real board, default /dev/ttyACM0
python3 launch.py -p /dev/ttyACM1 example/mode_local.py
python3 launch.py -s example/mode_local.py         # simulator (must already be running)
python3 launch.py -v <vehicle> example/mode_local.py
```

The simulator is a separate GUI application; `-s` connects to it via TCP on localhost ports 5011 (sim commands) and 5012 (control board comms).

### Docs (run in `docs/`)

```sh
mkdocs serve    # deps in requirements.txt
```

`package.sh` at repo root builds release zips (both firmware versions + iface + docs site).

## Architecture

### Firmware (`firmware/`)

FreeRTOS-based C firmware; CMake with presets selects the board via `CBOARD_REV`. Note: README's mention of PlatformIO is outdated.

- `src/` + `include/`: portable application code. Key modules:
  - `cmdctrl.c`: core command/control logic — parses messages from the PC and implements the control modes (RAW, LOCAL, GLOBAL, SASSIST1/2, DHOLD).
  - `motor_control.c` + `matrix.c`: 6-DoF math mapping motion targets to thruster speeds via the vehicle's motor matrix.
  - `pccomm.c` / `usb.c`: USB communication layer (TinyUSB) and message framing (CRC16-CCITT-FALSE).
  - `pid.c`: PID controllers used by the stability-assist (SASSIST) and depth-hold modes.
  - `bno055.c` (IMU) and `ms5837.c` (depth sensor): I2C sensor drivers; `calibration.c` + `eeprom.c` persist calibration.
  - `simulator.c`: "sim hijack" support — the real firmware can be driven by the simulator instead of real sensors/thrusters.
- `src/v1`, `src/v2`, `include/v1`, `include/v2`: board-specific implementations of the same interfaces (led, thruster PWM, i2c, eeprom, wdt, usb descriptors).
- `thirdparty/`: FreeRTOS, TinyUSB, and vendor-generated code (`v1_generated` from MPLAB Harmony, `v2_generated` from STM32CubeMX). Regenerated via `import_from_generator.py` from the projects in `generator_projects/` — do not hand-edit generated code.

### Python interface (`iface/`)

- `control_board.py`: the reference implementation of the comm protocol. `ControlBoard` (serial) and `Simulator` (TCP) expose the same API; one method per protocol message (e.g. `set_local`, `set_sassist1`, `read_bno055_periodic`). All commands return an `AckError`.
- `vehicle.py`: `Vehicle` abstract base class (motor matrix, thruster inversions, relative DoF speeds, IMU axis config, PID tunings) and the vehicle registry. Users add vehicles in an untracked `user_vehicles.py` (see `user_vehicles_template.py`); `launch.py` imports it automatically if present.
- `example/`: runnable demos, one per control mode/feature. Each defines `run(cb: ControlBoard, s: Simulator) -> int` (`s` is `None` on real hardware); new scripts should follow `scripts/template.py`.
