# DONUT_MELT

DONUT_MELT is an open-source translational drift ("melty brain") robot controller built on the **Raspberry Pi Pico 2**.

Melty brain robots spin their entire body using their drive wheels, but achieves directional control by adjusting motor power at specific points during each rotation. 

DONUT_MELT tracks RPM using a H3LIS331DL (a high-g accelerometer) and drives brushless motors via bidirectional DShot (although currently we aren't using the data we get from bidirectional DShot lol).

DONUT_MELT supports various configurations for different weight classes (or just different sets of settings) via config files. The file donut_config.h contains general settings while donut_1lb_config.h and donut_3lb_config.h are examples of different configurations.

DONUT_MELT also supports the usage of 1 or 2 accelerometers for reading RPM and you can configure which you want to do in the config files.

---

## Hardware Requirements

| Component | Details |
|---|---|
| Microcontroller | Raspberry Pi Pico 2 |
| Accelerometer | [H3LIS331DL](https://www.st.com/resource/en/datasheet/h3lis331dl.pdf) (400g range) |
| RC Receiver | Any CRSF-compatible receiver (e.g. ExpressLRS / TBS Crossfire) |
| ESC(s) | Brushless ESCs supporting bidirectional DShot |
| Heading LED | Any suitable LED with current-limiting resistor |

The H3LIS331DL is a 3.3v part and is natively compatible with the Pico 2's 3.3v I/O.

---

## Source Code Overview

All source files live in the `src/` directory.

**`donut_meltier.c`** - The main program entry point and core melty brain control loop. Ties together all subsystems: reads RC input, polls the accelerometer, computes rotation state, and fires motor and LED outputs at the correct phase each rotation.

**`donut_drive.c`** - Translational drift drive logic. Calculates which portion of each rotation each motor should be powered based on the driver's stick input and the robot's current heading. This is where the melty brain math lives.

**`donut_accel.c`** - Accelerometer interface and RPM calculation. Reads G-force data from the H3LIS331DL and converts it to a spin rate estimate using the configured mount radius.

**`donut_rssi_rpm.c`** - (3lb) Measures RPM from the once-per-rotation pattern in the receiver's uplink RSSI. Runs in shadow mode by default and drives when `USE_RSSI_RPM` is defined, falling back to the accelerometer whenever it loses lock. Needs the custom ELRS receiver firmware. See [docs/RSSI_RPM.md](docs/RSSI_RPM.md).

**`donut_rpm_curve.c`** - (3lb) Accelerometer RPM correction curve stored in flash. Points are saved with CH10, either from the left-stick trim or measured from RSSI.

**`donut_logger.c`** - Flash run logger: records RSSI, RPM estimates and the RPM source during a live run and saves them on kill. Dump and plot with `tools/dump_log.py`.

**`donut_pc.c`** - A basic P controller for dampening left_stick_y throttle adjustments (can be adapted to control other values in-code).

**`motor_driver.c`** - This file controls the motors. Sends DShot packets to the ESCs via the `c_pico_bidir_dshot` library, and reads back RPM telemetry when available.

**`receiver.c`** - Handles CRSF RC receiver input. Parses the CRSF serial stream over UART and exposes decoded channel values (throttle, forward/back, left/right, etc.) to the rest of the system.

**`led_driver.c`** - Heading LED driver. Pulses the heading beacon LED once per rotation at the configured heading angle, giving the driver a visible indication of what is "forward."

**`donut_telemetry.c`** - USB serial telemetry output. When the robot is idle (throttle at zero), diagnostic data including RC channel values, accelerometer readings, RPM, and configuration parameters is logged over USB. Use any serial terminal at the appropriate baud rate to view output. Also attempts to send select data back to the transmitter. 

---

## Libraries (submodules)

DONUT_MELT depends on three libraries, included in the `lib/` directory:

**`lib/c_pico_bidir_dshot`** - [link](https://github.com/bastian2001/pico-bidir-dshot/blob/main/README.md) - Bidirectional DShot implementation for the RP2040/RP2350. Uses PIO state machines for precise timing. Handles both sending throttle commands and receiving RPM telemetry from the ESC.

**`lib/pico_crsf`** - [link](https://github.com/britannio/pico_crsf) - CRSF (Crossfire Serial Protocol) receiver library for the Pico. Decodes the CRSF serial stream from a CRSF-compatible RC receiver.

**`lib/pico_H3LIS331DL`** - [link](https://github.com/Slipperee-CODE/pico_H3LIS331DL) - Driver library for the H3LIS331DL high-g accelerometer over I2C.

---

## Building

DONUT_MELT uses CMake and the Raspberry Pi Pico SDK.

**Prerequisites:**
- Raspberry Pi Pico SDK 2.2.0
- CMake 3.13+
- ARM GCC toolchain (14.2 Rel1 recommended)
- `picotool` 2.2.0

Flash the resulting `.uf2` file to your Pico 2 by holding BOOTSEL while connecting USB, then dragging the file onto the mass storage device that appears.

OR

The VS Code Pico extension is also supported. The `.vscode/` directory contains appropriate settings for building and flashing directly from the editor.

#### Tests

A set of standalone test executables are included in the `tests/` directory and listed (commented out) in `CMakeLists.txt`. To run a specific test, uncomment the relevant `add_executable` line and comment out the main executable. Tests cover:

- Basic LED blink
- Accelerometer readout
- CRSF receiver + telemetry
- DShot ramp (motor spin-up test)
- DShot from TX (full transmitter-to-motor test)

---

## Telemetry / Debugging

With the motor battery **disconnected**, connect the Pico 2 via USB. If activated in the donut_config.h file, DONUT_MELT streams diagnostic data over USB serial. Open any serial terminal to view RC channel values, accelerometer G readings, calculated RPM, and active config parameters.

For RSSI RPM tracking (transmitter telemetry page, heading LED fallback strobe, flash run log and plots, RPM curve calibration), see [docs/RSSI_RPM.md](docs/RSSI_RPM.md).

> ⚠️ **Warning:** Do not connect USB while the motor battery is connected unless you are certain the robot cannot spin up. Always verify fail-safe is configured on your RC transmitter and receiver.

---

## Safety

**Building and operating a combat robot, even at the 1lb or 3lb weight class, is inherently dangerous. By using this code you accept all risks. The author (me) cannot be held responsible for injury, damage, or loss resulting from the use of this project. Please be safe!**

## License

GPL-3.0 — see [LICENSE](LICENSE) for full terms.
