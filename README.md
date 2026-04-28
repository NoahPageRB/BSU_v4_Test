# BSU v4 — PCB Bring-Up & Test Procedure

End-to-end procedure for taking a freshly assembled BSU v4 board from
solder-rework to fully tested. Two parallel paths are documented:

- **Manual** — work through each section's CLI commands by hand.
- **Automated** — run [`bsu_test.py`](bsu_test.py) once the firmware is on
  the board (covers Steps 6 onward).

> **Tip:** flash everything first (Steps 1–5), then run `bsu_test.py` for
> the functional tests. Drop back to the manual commands only when you need
> to debug a failure.

---

## Required Hardware

| Tool | Purpose |
|---|---|
| Soldering iron | Solder rework |
| Tweezer soldering iron | Rework on small components |
| Hot air reflow station | Required for v4 only (skip on v4.1) |
| XGecu T48 programmer | Flashing the AT24C02C EEPROM |
| SOIC-8 SOIC clip *or* 8× test hooks | Connecting T48 to the EEPROM |
| Atmel ICE | Flashing both MCU bootloaders via SWD |

## Required Software

| Software | Purpose |
|---|---|
| VS Code (or VSCodium) + PlatformIO | Building & uploading MCU firmware |
| David Griffith's `minipro` | Driving the T48 from the command line |
| Microchip Studio (Windows-only) | Driving the Atmel ICE via SWD |

---

## Procedure Overview

1. [Solder rework](#1-solder-rework)
2. [Flash the AT24C02C EEPROM](#2-flash-the-at24c02c-eeprom)
3. [Flash the Main MCU bootloader](#3-flash-the-main-mcu-bootloader)
4. [Flash the Main MCU firmware](#4-flash-the-main-mcu-firmware)
5. [Verify serial comms](#5-verify-serial-comms)
6. [Motor test A — MCU direct STEP/DIR](#6-motor-test-a--mcu-direct-stepdir)
7. [Motor test B — TMC429 velocity mode](#7-motor-test-b--tmc429-velocity-mode)
8. [Limit switch test](#8-limit-switch-test)
9. [Drawer handle test](#9-drawer-handle-test)
10. [Solenoid test (12 valves)](#10-solenoid-test-12-valves)
11. [Flash the LED MCU bootloader and firmware](#11-flash-the-led-mcu-bootloader-and-firmware)
12. [LED functional test](#12-led-functional-test)
13. [Sign off](#13-sign-off)

---

## 1. Solder Rework

Inspect under the microscope and rework any obvious bridges, tombstones, or
missing joints before applying power.

## 2. Flash the AT24C02C EEPROM

The on-board EEPROM holds the USB descriptor data. Flash it with the T48
before powering the rest of the board.

1. Apply 24 V to the board.
2. Connect a USB-C device (any laptop will do — this just provides the bus
   reference).
3. Connect the T48 to the EEPROM via the SOIC-8 clip or test hooks.
4. Verify the programmer is reachable:
   ```
   minipro -k
   ```
5. Write the EEPROM (run from the repo root):
   ```
   minipro -p AT24C02C -w Binaries/2513.bin
   ```
6. Connect a USB-C cable from the board to your laptop and confirm a new
   COM port appears.

> **VS Code note:** PlatformIO only shows the COM picker if you open the
> `ATSAMD51J19A/` folder directly (where its `platformio.ini` lives), not
> the repo root.

## 3. Flash the Main MCU Bootloader

> Microchip Studio is Windows-only — this step requires a Windows host.

1. Power off the board.
2. Connect the Atmel ICE to the **Main MCU** SWD header (mind the
   orientation).
3. Power on the board. The Atmel ICE should show a steady green light.
4. In Microchip Studio: **Tools → Device Programming**.
5. **Device:** `ATSAMD51J19` · **Interface:** SWD · click **Apply**.
6. Click **Read** next to *Device signature* — it should populate.
7. Open the **Memories** tab. Next to **Flash**, click the `…` button.
8. Switch the file picker to **Binary**, select
   `Binaries/bootloader-metro_m4_revb-v3.16.0.bin`, click **Open**, then
   **Program**.
9. Wait for `Verifying Flash OK`.
10. Power off and disconnect the Atmel ICE.

## 4. Flash the Main MCU Firmware

1. Connect USB-C from the board to your laptop and power on.
2. In VS Code, open the `ATSAMD51J19A/` folder.
3. In the PlatformIO toolbar, select the **Metro M4** COM port.
4. Click **Upload**. Wait for success.

## 5. Verify Serial Comms

Open a serial monitor (VS Code's, Arduino's, anything) at **115200 baud** to
the Metro M4 port.

```
help
```
You should see the full command list. If you do, comms are good.

```
io_init
```
Initializes both MCP23017 expanders. Motor enables and the drawer button
live on the EXTRA expander, so this must run before any motor or button
test.

## 6. Motor Test A — MCU Direct STEP/DIR

> **Shared Z enable:** the BSU drives Z1 and Z2 from a single enable line on
> the EXTRA expander (GPA6) — they're the two halves of one gantry, like the
> twin Z motors on a Prusa. The CLI exposes only `z` and `m3` as canonical
> axes; `z1`/`z2` are accepted as aliases that map back to the shared `z`
> enable. M3 has its own enable on GPA7.

For each axis, connect the test stepper motor to that axis's port and run:

```
tmc_init z1
tmc_microstep z1 8
tmc_current z1 50
tmc_enable z1
enable z
step z1 1600 2000
```
The motor should make a full rotation.

Repeat for **Z2** (leave `z` enabled — same physical pin):
```
tmc_init z2
tmc_microstep z2 8
tmc_current z2 50
tmc_enable z2
step z2 1600 2000
disable z
```

Repeat for **M3** (independent enable):
```
tmc_init m3
tmc_microstep m3 8
tmc_current m3 50
tmc_enable m3
enable m3
step m3 1600 2000
disable m3
```

## 7. Motor Test B — TMC429 Velocity Mode

Power off, flip the STEP/DIR toggle switch to the **TMC429** position, then
power back on.

```
io_init
mc_init
```

For each axis, connect the test stepper to that axis's port and run:

```
tmc_init z1
tmc_microstep z1 8
tmc_current z1 50
tmc_enable z1
mc_limits z1 100 1000 1000
enable z1
mc_velocity z1
mc_vtarget z1 1000
```
The motor should spin continuously. Then:
```
disable z1
```

Repeat with `z2`, then `m3`, swapping the test motor between ports.

## 8. Limit Switch Test

The board has four IR limit-switch sensor inputs:

| Sensor | Used by |
|---|---|
| `Z_TOP` | z1 + z2 (shared) |
| `Z_BOT` | z1 + z2 (shared) |
| `SPARE_TOP` | m3 only |
| `SPARE_BOT` | m3 only |

```
io_init
mc_swpol high
mc_rightsw on
io_read
```
You should see all four switches `open` and the motor enable lines HIGH
(disabled — recall enables are active-LOW).

```
mc_switches
```
All TMC429-side switches should be `inactive`.

For each of the four sensors, **bridge the connector high** (jumper the
center pin to the pin furthest from the board edge) and verify:

1. `io_read` shows that sensor `TRIGGERED`, others `open`.
2. `mc_switches` shows the corresponding motor(s) `ACTIVE` — `Z_TOP`/`Z_BOT`
   light up both `z1` and `z2`; `SPARE_TOP`/`SPARE_BOT` light up only `m3`.
3. Remove the jumper and confirm everything returns to `open` / `inactive`.

## 9. Drawer Handle Test

Connect the drawer-handle harness (button + bi-color LED).

```
drawer_button
```
Run repeatedly while pressing the button — should report `PRESSED` while
held, `released` otherwise.

```
drawer_led red       # LED lights red
drawer_led green     # LED lights green
drawer_led off       # LED off
```

## 10. Solenoid Test (12 valves)

The CLI walks valves in physical tray/valve order:

| CLI index | Valves |
|---|---|
| `sol 0..3` | T1V1 – T1V4 (Tray 1) |
| `sol 4..7` | T2V1 – T2V4 (Tray 2) |
| `sol 8..11` | T3V1 – T3V4 (Tray 3) |

Move a single test solenoid (or a click/LED test harness) between the 12
ports. For each:
```
sol 0 on        # verify T1V1 fires
sol 0 off
```
Move the test solenoid to the next port and increment the index. Repeat
through `sol 11` / T3V4.

To drive all 12 at once (watch your total current draw):
```
sol_all on
sol_all off
```

Power off when done.

## 11. Flash the LED MCU Bootloader and Firmware

### Bootloader (Atmel ICE, Windows)

Same flow as the Main MCU bootloader (Step 3), but:
- Connect the Atmel ICE to the **LED MCU** SWD header.
- **Device:** `ATSAMD21E18A`.
- **Binary:** `Binaries/bootloader-trinket_m0-v3.16.0.bin`. *(For the older
  ATSAMD21E17A variant, use `Binaries/bootloader-trinket_m0_e17a.bin`
  instead.)*

Power off and disconnect the Atmel ICE when done.

### Firmware (PlatformIO + UF2)

1. Connect USB-C and power on. A new COM port should appear (Trinket M0).
2. In VS Code, open the `ATSAMD21E18A/` folder.
3. Select the Trinket M0 port.
4. **Double-tap** the reset button on the bottom of the board. A USB drive
   named `TRINKETBOOT` should mount. (May take a couple of tries.)
5. Either click **Upload** in PlatformIO, or drag-drop
   `ATSAMD21E18A/.pio/build/trinket_m0/firmware.uf2` onto `TRINKETBOOT`.
   The drive will disconnect on completion.
6. **Single-tap** the reset button (don't double-tap or you'll re-enter the
   bootloader).
7. Connect a serial monitor at **115200 baud** to the Trinket port. If it
   doesn't appear, single-press reset; do not double-press.

## 12. LED Functional Test

Connect the NeoPixel ring(s) to the LED port. Switch your serial monitor
back to the **Main MCU** port.

```
led_set all spin 255 0 0 255
```
LEDs should run a red spin animation.

```
led_set all off 0 0 0 0
```
LEDs off.

## 13. Sign Off

Write a serial number under the Resolve logo on the silkscreen. Done.

---

## Automated Alternative

Once the firmware is flashed (through Step 4), the Python tester drives all
of Steps 5–12 interactively, prompting you to swap test fixtures at the
right moments and emitting a JSON pass/fail log:

```
pip install pyserial
python bsu_test.py
```

The tester writes a `bsu_test_<timestamp>_test_set_<A|B>.json` file into
the working directory after every step. Past runs are archived under
[`Results/`](Results/) for reference.

The tester also performs a firmware version handshake on startup. If the
firmware on the board doesn't match the version the tester expects, it
fails the run with a clear message — see [`CLAUDE.md`](CLAUDE.md) for the
versioning rules.
