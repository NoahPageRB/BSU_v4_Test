# BSU Motor, Sensor, Drawer & Solenoid Test CLI

A Serial CLI firmware for the **Adafruit Metro M4 Express** (ATSAMD51J19A) that
provides interactive control and diagnostics for the BSU v4 PCB:

- **3x TMC2209** stepper drivers on three independent axes (Z1, Z2, M3)
- **TMC429** motion controller (autonomous step/dir generator with ramps)
- **2x MCP23017** I/O expanders (motor enables/DIAGs/switches + 12 solenoids)
- **Drawer-handle** bi-color LED (via DRV8231A H-bridge) and pushbutton input
- **LED MCU** (ATSAMD21E18A on I2C) for NeoPixel ring animation

## Hardware Overview

### Motors

| Motor | STEP | DIR | nEnable (expander) | TMC2209 UART    | SERCOM            |
|-------|------|-----|--------------------|-----------------|-------------------|
| Z1    | A1   | A0  | EXTRA GPA4         | TX=D13, RX=D12  | SERCOM1           |
| Z2    | A3   | A2  | EXTRA GPA5         | TX=D3,  RX=D2   | SERCOM5           |
| M3    | A5   | A4  | EXTRA GPA6         | TX=D1,  RX=D0   | SERCOM3 (Serial1) |

All three motors are fully independent: each has its own STEP, DIR, nEnable,
and TMC2209 UART. Unlike the PSU, no STEP/DIR lines are shared.

### STEP/DIR Source Selection

The PCB has a physical toggle switch to select whether STEP/DIR signals come
from the **MCU** (direct GPIO, using `dir`/`step` commands) or from the
**TMC429** motion controller (using `mc_*` commands). When using the TMC429,
the MCU communicates with it over SPI, and the TMC429 autonomously generates
the step/direction pulses with proper acceleration ramps.

### TMC429 Motion Controller

- **SPI bus:** SERCOM4 — MOSI=D7 (PB12), SCK=D4 (PB13), MISO=D5 (PB14)
- **nSCS (chip select):** D6
- **Clock:** 32 MHz (TOGNJING XOS20032000LT00351005 oscillator)
- **Motor mapping:** z1 = motor 0 (M1), z2 = motor 1 (M2), m3 = motor 2 (M3)
- **Limit switch mapping:** END1 = Left (REFxL), END2 = Right (REFxR)

The janelia-arduino/TMC429 library hardcodes the global `SPI` object; we
subclass `TMC429` as `TMC429Custom` and override its protected virtual SPI
hooks (`spiBegin`, `spiBeginTransaction`, `spiEndTransaction`, `spiTransfer`)
to use a SERCOM4 `SPIClass` instance instead.

### TMC2209 UART Wiring

Each TMC2209 uses a single-wire UART interface (PDN_UART). The MCU TX is
connected through a **1kΩ resistor** to the PDN_UART line, and the MCU RX is
connected **directly** to the same line. This causes TX echoes on RX, which the
janelia-arduino/TMC2209 library handles transparently.

### I/O Expanders (2x MCP23017)

Both expanders share a common nRESET line on **D8** (pulled HIGH via 10kΩ).

#### EXTRA_IO_EXPANDER — I2C 0x21 (A0=VDD)

Motor enables (outputs), driver diagnostics, Z/spare limit switches, drawer
pushbutton.

| Pin  | Direction | Signal       | Description                                 |
|------|-----------|--------------|---------------------------------------------|
| GPA0 | In        | Z1 DIAG      | TMC2209 DIAG output for Z1                  |
| GPA1 | In        | Z2 DIAG      | TMC2209 DIAG output for Z2                  |
| GPA2 | In        | M3 DIAG      | TMC2209 DIAG output for M3                  |
| GPA3 | In        | *(unused)*   | —                                           |
| GPA4 | **Out**   | Z1 nEN       | HIGH = driver disabled (default at boot)    |
| GPA5 | **Out**   | Z2 nEN       | HIGH = driver disabled (default at boot)    |
| GPA6 | **Out**   | M3 nEN       | HIGH = driver disabled (default at boot)    |
| GPA7 | In        | DRAWER_BTN   | Drawer-handle pushbutton                    |
| GPB0 | In        | Z_TOP        | Z1/Z2 shared top limit switch               |
| GPB1 | In        | Z_BOT        | Z1/Z2 shared bottom limit switch            |
| GPB2 | In        | SPARE_TOP    | M3 top limit switch                         |
| GPB3 | In        | SPARE_BOT    | M3 bottom limit switch                      |
| GPB4-7 | In      | *(unused)*   | —                                           |

During `io_init`, the OLAT register is written **before** IODIR so the nEN
outputs never briefly pulse LOW (which would momentarily enable motors) when
the pins transition from input to output.

#### TRAY_IO_EXPANDER — I2C 0x20 (A2=A1=A0=GND)

12 solenoid outputs. Solenoids are driven HIGH to fire; default all OFF.

| Pin    | Direction | Signal        |
|--------|-----------|---------------|
| GPA0-7 | **Out**   | SOL0 – SOL7   |
| GPB0-3 | **Out**   | SOL8 – SOL11  |
| GPB4-7 | In        | *(unused)*    |

### Drawer-Handle Bi-color LED (DRV8231A H-bridge)

A single DRV8231A drives a two-terminal bi-color LED; polarity selects color.

| MCU Pin | DRV8231A Input | State |
|---------|----------------|-------|
| D10     | IN1            | HIGH=drive one way |
| D11     | IN2            | HIGH=drive the other way |

- `IN1=1, IN2=0` → **RED**
- `IN1=0, IN2=1` → **GREEN**
- `IN1=0, IN2=0` → **OFF** (coast)

The PCB has a hardware AND-gate interlock that forces both H-bridge inputs
LOW if both MCU inputs are HIGH, preventing active braking and LED shorting.
The firmware sequences color changes by lowering the opposite input **before**
raising the new one, so the LED never briefly flickers through the interlock
gate.

### LED MCU

See [ATSAMD21E18A/README.md](../ATSAMD21E18A/README.md) for the LED MCU
firmware (NeoPixel ring animation engine). It is an I2C slave at **0x30** on
the same bitbang I2C bus as the MCP23017s.

## Building and Uploading

```bash
pio run -e metro_m4            # Build
pio run -e metro_m4 -t upload  # Upload
pio device monitor -b 115200   # Serial monitor
```

## CLI Commands

Connect at **115200 baud** over USB serial. Commands are case-insensitive.

---

### General

| Command | Description |
|---------|-------------|
| `help`  | Print all available commands |

---

### I/O Expanders (MCP23017)

| Command    | Description |
|------------|-------------|
| `io_init`  | Initialize (or re-initialize) both MCP23017s |
| `io_reset` | Hardware-reset both MCP23017s via the common nRESET pin |
| `io_read`  | Read all expander inputs (DIAGs, drawer button, switches, solenoid readback) |

---

### MCU Pin Readback

| Command    | Description |
|------------|-------------|
| `mcu_read` | Read back MCU-controlled pin states (STEP, DIR, drawer IN1/IN2, nCS, nRESET) |

---

### Motor Enable / Disable (via EXTRA_IO_EXPANDER GPA4..GPA6)

| Command                   | Description |
|---------------------------|-------------|
| `enable <z1\|z2\|m3>`     | Clear nEnable bit (driver ON) |
| `disable <z1\|z2\|m3>`    | Set nEnable bit (driver OFF) |

Motors are **disabled** at boot (all three nEN bits HIGH). Run `io_init` and
then `enable` before expecting any motion.

---

### TMC2209 UART Commands (`tmc_`)

Communicate with individual TMC2209 drivers. Run `tmc_init` first per motor.

| Command | Description |
|---------|-------------|
| `tmc_init <z1\|z2\|m3> [baud]` | Open UART (default 115200) |
| `tmc_status <z1\|z2\|m3>` | Read settings + fault flags |
| `tmc_current <z1\|z2\|m3> <percent>` | Set run current (0–100) |
| `tmc_hold <z1\|z2\|m3> <percent>` | Set hold current (0–100) |
| `tmc_microstep <z1\|z2\|m3> <n>` | Set microsteps (1–256, power of 2) |
| `tmc_stealthchop <z1\|z2\|m3> <on\|off>` | Toggle StealthChop |
| `tmc_enable <z1\|z2\|m3>` | Software-enable via UART |
| `tmc_disable <z1\|z2\|m3>` | Software-disable via UART |

---

### Direct Pin Control (`dir`, `step`)

For use when the STEP/DIR toggle switch is set to **MCU** mode.

| Command | Description |
|---------|-------------|
| `dir <z1\|z2\|m3> <0\|1>` | Set direction pin level |
| `step <z1\|z2\|m3> <count> <delay_us>` | Pulse STEP pin (blocking, min 2µs) |

---

### TMC429 Motion Controller Commands (`mc_`)

For use when the STEP/DIR toggle switch is set to **TMC429** mode.
Run `mc_init` first.

#### Initialization

| Command | Description |
|---------|-------------|
| `mc_init [clock_mhz]` | Init SPI (SERCOM4) and TMC429. Clock defaults to 32 MHz. |
| `mc_status` | Read version, status flags, position and velocity for all 3 axes. |

#### Motion Parameters

| Command | Description |
|---------|-------------|
| `mc_limits <z1\|z2\|m3> <vmin_hz> <vmax_hz> <amax_hz_per_s>` | Set velocity and acceleration limits. |

#### Mode Selection

| Command | Description |
|---------|-------------|
| `mc_ramp <z1\|z2\|m3>` | **Ramp mode:** position control with trapezoidal acceleration profile. |
| `mc_velocity <z1\|z2\|m3>` | **Velocity mode:** continuous rotation at a target velocity. |
| `mc_hold <z1\|z2\|m3>` | **Hold mode:** lock current position, no ramp generator. |

#### Motion Commands

| Command | Description |
|---------|-------------|
| `mc_target <z1\|z2\|m3> <position>` | Set target position (ramp mode). |
| `mc_vtarget <z1\|z2\|m3> <velocity_hz>` | Set target velocity (velocity mode). Negative = reverse. |
| `mc_pos <z1\|z2\|m3>` | Read actual + target position, actual velocity, and at-target flag. |
| `mc_setpos <z1\|z2\|m3> <position>` | Overwrite actual position register (e.g., zero after homing). |
| `mc_stop <z1\|z2\|m3>` | Stop one axis (ramps velocity to zero). |
| `mc_stopall` | Stop all three axes. |

#### Limit Switch Configuration

| Command | Description |
|---------|-------------|
| `mc_switches` | Read all limit switch states and auto-stop configuration. |
| `mc_swpol <high\|low>` | Set switch active polarity (applies to all switches). |
| `mc_leftstop <z1\|z2\|m3> <on\|off>` | Enable/disable auto-stop on left switch (END1). |
| `mc_rightstop <z1\|z2\|m3> <on\|off>` | Enable/disable auto-stop on right switch (END2). |
| `mc_rightsw <on\|off>` | Enable/disable right switch inputs globally. |

---

### LED MCU Commands (`led_`)

Control the LED MCU (ATSAMD21E18A) over I2C. See
[../ATSAMD21E18A/README.md](../ATSAMD21E18A/README.md) for the full protocol.

| Command | Description |
|---------|-------------|
| `led_set <ring\|all> <anim> <r> <g> <b> <brightness>` | Set animation + color + brightness. Anim: off, static, spin, bounce, pulse. |
| `led_status <ring>` | Query ring's current animation, color, brightness. |
| `led_config <param> <value>` | Set persistent config (saved to flash on LED MCU). |
| `led_getconfig` | Read all persistent config parameters from LED MCU. |

---

### Drawer Handle

| Command | Description |
|---------|-------------|
| `drawer_led <off\|red\|green>` | Drive bi-color LED via the DRV8231A H-bridge. |
| `drawer_button` | Read the drawer pushbutton (via EXTRA_IO_EXPANDER GPA7). |

---

### Solenoids (TRAY_IO_EXPANDER @ 0x20)

| Command | Description |
|---------|-------------|
| `sol <0..11> <on\|off>` | Drive a single solenoid (0–11). |
| `sol_all <on\|off>` | Drive all 12 solenoids at once. |

---

## Typical Test Sessions

### Session 1: Direct stepping (MCU mode)

```
> io_init
> tmc_init z1
> tmc_current z1 50
> tmc_microstep z1 8
> enable z1
> dir z1 1
> step z1 1600 500
```

### Session 2: TMC429 ramp mode (TMC429 mode)

```
> io_init
> tmc_init m3
> tmc_current m3 50
> enable m3
> mc_init
> mc_limits m3 100 2000 500
> mc_ramp m3
> mc_setpos m3 0
> mc_target m3 10000
> mc_pos m3
> mc_target m3 0
```

### Session 3: TMC429 velocity mode

```
> io_init
> tmc_init z2
> tmc_current z2 50
> enable z2
> mc_init
> mc_limits z2 100 5000 1000
> mc_velocity z2
> mc_vtarget z2 3000
> mc_pos z2
> mc_vtarget z2 -3000
> mc_stop z2
```

### Session 4: Limit switch testing with TMC429

```
> mc_init
> mc_swpol high
> mc_rightsw on
> mc_leftstop z1 on
> mc_rightstop z1 on
> mc_switches
```

### Session 5: Drawer + solenoids

```
> io_init
> drawer_led red
> drawer_button
> drawer_led green
> drawer_led off
> sol 0 on
> sol 0 off
> sol_all on
> sol_all off
```

### Session 6: LED ring control

```
> led_getconfig
> led_set all static 255 0 0 128
> led_set 0 spin 0 255 0 200
> led_status 0
> led_config spin_width 3
> led_set all off 0 0 0 0
```

## Architecture Notes

### SERCOM Allocation

| SERCOM  | Usage              | Managed by             |
|---------|--------------------|------------------------|
| SERCOM0 | (unused)           | —                      |
| SERCOM1 | Z1 TMC2209 UART    | Custom `Uart` instance |
| SERCOM2 | (unused)           | —                      |
| SERCOM3 | M3 TMC2209 UART    | `Serial1` (BSP)        |
| SERCOM4 | TMC429 SPI         | Custom `SPIClass`      |
| SERCOM5 | Z2 TMC2209 UART    | Custom `Uart` instance |

### SERCOM5 Conflict and Resolution

Wire (I2C) and Z2 UART both need SERCOM5 on this board. Wire is excluded via
`lib_ignore`, and a bitbang I2C master (`soft_i2c.h`) is used for the two
MCP23017s and the LED MCU instead. ~100 kHz bitbang is more than adequate
for these peripherals.

### Library Dependencies

| Library | Purpose | PlatformIO ID |
|---------|---------|---------------|
| TMC2209 | TMC2209 UART communication | `janelia-arduino/TMC2209` |
| TMC429  | TMC429 SPI motion controller (subclassed for custom SPI) | `janelia-arduino/TMC429` |

The MCP23017s and LED MCU are accessed via bitbang I2C — no external library
needed.

### File Overview

| File | Purpose |
|------|---------|
| `platformio.ini` | Build config, library deps, Wire exclusion |
| `src/main.cpp` | CLI, motor control, TMC2209/TMC429, expanders, drawer, solenoids, LED MCU |
| `src/soft_i2c.h` | Minimal bitbang I2C master |
| `README.md` | This file |
