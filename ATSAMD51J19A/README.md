# Motor & Sensor Test CLI

A Serial CLI firmware for the **Adafruit Metro M4 Express** (ATSAMD51J19A) that
provides interactive control and diagnostics for a 4-motor stepper system with
TMC2209 drivers, a TMC429 motion controller, and an MCP23017 I/O expander.

## Hardware Overview

### Motors

| Motor | STEP | DIR | nEnable | TMC2209 UART         | SERCOM  |
|-------|------|-----|---------|----------------------|---------|
| X     | A1   | A0  | D10     | TX=D7, RX=D4         | SERCOM4 |
| Y     | A3   | A2  | D9      | TX=D1, RX=D0         | SERCOM3 (Serial1) |
| Z1    | A5*  | A4* | D8*     | TX=D13, RX=D12       | SERCOM1 |
| Z2    | A5*  | A4* | D8*     | TX=D3, RX=D2         | SERCOM5 |

\*Z1 and Z2 share STEP, DIR, and nEnable lines. They always move together.
Each has its own TMC2209 UART for independent configuration.

### STEP/DIR Source Selection

The PCB has a physical toggle switch to select whether STEP/DIR signals come
from the **MCU** (direct GPIO, using `dir`/`step` commands) or from the
**TMC429** motion controller (using `mc_*` commands). When using the TMC429,
the MCU communicates with it over SPI, and the TMC429 autonomously generates
the step/direction pulses with proper acceleration ramps.

### TMC429 Motion Controller

- **SPI bus:** Default Metro M4 SPI (MOSI, MISO, SCK on SERCOM2)
- **nSCS (chip select):** D5
- **Clock:** 32 MHz (TOGNJING XOS20032000LT00351005 oscillator)
- **Motor mapping:** X = motor 0 (M1), Y = motor 1 (M2), Z = motor 2 (M3)
- **Limit switch mapping:** END1 = Left (REFxL), END2 = Right (REFxR)

### TMC2209 UART Wiring

Each TMC2209 uses a single-wire UART interface (PDN_UART). The MCU TX is
connected through a **1kΩ resistor** to the PDN_UART line, and the MCU RX is
connected **directly** to the same line. This causes TX echoes on RX, which the
janelia-arduino/TMC2209 library handles transparently.

### MCP23017 I/O Expander

- **I2C address:** `0x20` (A2=A1=A0=GND)
- **nRESET:** Connected to D6 (MCU can toggle it; pulled HIGH via 10kΩ)
- **All pins configured as inputs**

| Pin  | Signal      | Description                          |
|------|-------------|--------------------------------------|
| GPA0 | X DIAG      | TMC2209 DIAG output for X            |
| GPA1 | Y DIAG      | TMC2209 DIAG output for Y            |
| GPA2 | Z1 DIAG     | TMC2209 DIAG output for Z1           |
| GPA3 | Z2 DIAG     | TMC2209 DIAG output for Z2           |
| GPB2 | X END1      | X-axis limit switch 1 (active HIGH)  |
| GPB3 | X END2      | X-axis limit switch 2 (active HIGH)  |
| GPB4 | Y END1      | Y-axis limit switch 1 (active HIGH)  |
| GPB5 | Y END2      | Y-axis limit switch 2 (active HIGH)  |
| GPB6 | Z END1      | Z-axis limit switch 1 (active HIGH)  |
| GPB7 | Z END2      | Z-axis limit switch 2 (active HIGH)  |

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

### I/O Expander (MCP23017)

| Command    | Description |
|------------|-------------|
| `io_init`  | Initialize (or re-initialize) the MCP23017 |
| `io_reset` | Hardware-reset the MCP23017 via nRESET pin |
| `io_read`  | Read all expander inputs (DIAG pins + limit switches) |

---

### MCU Pin Readback

| Command    | Description |
|------------|-------------|
| `mcu_read` | Read back all MCU output pin states (STEP, DIR, nEnable, nRESET) |

---

### Motor Enable / Disable (Hardware nEnable Pin)

| Command              | Description |
|----------------------|-------------|
| `enable <x\|y\|z>`  | Drive nEnable LOW (driver ON) |
| `disable <x\|y\|z>` | Drive nEnable HIGH (driver OFF) |

---

### TMC2209 UART Commands (`tmc_`)

Communicate with individual TMC2209 drivers. Run `tmc_init` first.

| Command | Description |
|---------|-------------|
| `tmc_init <x\|y\|z1\|z2> [baud]` | Open UART (default 115200) |
| `tmc_status <x\|y\|z1\|z2>` | Read settings + fault flags |
| `tmc_current <x\|y\|z1\|z2> <percent>` | Set run current (0–100) |
| `tmc_hold <x\|y\|z1\|z2> <percent>` | Set hold current (0–100) |
| `tmc_microstep <x\|y\|z1\|z2> <n>` | Set microsteps (1–256, power of 2) |
| `tmc_stealthchop <x\|y\|z1\|z2> <on\|off>` | Toggle StealthChop |
| `tmc_enable <x\|y\|z1\|z2>` | Software-enable via UART |
| `tmc_disable <x\|y\|z1\|z2>` | Software-disable via UART |

---

### Direct Pin Control (`dir`, `step`)

For use when the STEP/DIR toggle switch is set to **MCU** mode.

| Command | Description |
|---------|-------------|
| `dir <x\|y\|z> <0\|1>` | Set direction pin level |
| `step <x\|y\|z> <count> <delay_us>` | Pulse STEP pin (blocking, min 2µs) |

---

### TMC429 Motion Controller Commands (`mc_`)

For use when the STEP/DIR toggle switch is set to **TMC429** mode.
The TMC429 generates step pulses autonomously with proper acceleration ramps.
Run `mc_init` first.

#### Initialization

| Command | Description |
|---------|-------------|
| `mc_init [clock_mhz]` | Init SPI and TMC429. Clock defaults to 32 MHz (TOGNJING XOS20032000LT00351005 oscillator). |
| `mc_status` | Read version, status flags, and position/velocity for all axes. |

#### Motion Parameters

| Command | Description |
|---------|-------------|
| `mc_limits <x\|y\|z> <vmin_hz> <vmax_hz> <amax_hz_per_s>` | Set velocity and acceleration limits. All values in Hz (microsteps/sec) and Hz/s. |

#### Mode Selection

| Command | Description |
|---------|-------------|
| `mc_ramp <x\|y\|z>` | **Ramp mode:** position control with trapezoidal acceleration profile. Set target with `mc_target`. |
| `mc_velocity <x\|y\|z>` | **Velocity mode:** continuous rotation at a target velocity. Set speed with `mc_vtarget`. |
| `mc_hold <x\|y\|z>` | **Hold mode:** lock current position, no ramp generator. |

#### Motion Commands

| Command | Description |
|---------|-------------|
| `mc_target <x\|y\|z> <position>` | Set target position (use in ramp mode). The TMC429 will ramp the motor to this position automatically. |
| `mc_vtarget <x\|y\|z> <velocity_hz>` | Set target velocity in Hz (use in velocity mode). Negative values reverse direction. |
| `mc_pos <x\|y\|z>` | Read actual position, target position, actual velocity, and at-target flag. |
| `mc_setpos <x\|y\|z> <position>` | Write the actual position register (e.g., to zero it after homing). |
| `mc_stop <x\|y\|z>` | Stop one axis (ramps velocity to zero). |
| `mc_stopall` | Stop all three axes. |

#### Limit Switch Configuration

| Command | Description |
|---------|-------------|
| `mc_switches` | Read all limit switch states and auto-stop configuration. |
| `mc_swpol <high\|low>` | Set switch active polarity (applies to all switches). |
| `mc_leftstop <x\|y\|z> <on\|off>` | Enable/disable automatic stop on left switch (END1). |
| `mc_rightstop <x\|y\|z> <on\|off>` | Enable/disable automatic stop on right switch (END2). |
| `mc_rightsw <on\|off>` | Enable/disable right switch inputs globally. |

---

### LED MCU Commands (`led_`)

Control the LED MCU (ATSAMD21E18A) over I2C. The LED MCU drives NeoPixel rings
autonomously. See `led-mcu/README.md` for full protocol documentation.

| Command | Description |
|---------|-------------|
| `led_set <ring\|all> <anim> <r> <g> <b> <brightness>` | Set animation + color + brightness. Anim: off, static, spin, bounce, pulse. RGB and brightness: 0–255. |
| `led_status <ring>` | Query ring's current animation, color, brightness. |
| `led_config <param> <value>` | Set persistent config (saved to flash on LED MCU). Params: num_rings, leds_per_ring, spin_width, bounce_width, spin_speed, bounce_speed, pulse_speed. |
| `led_getconfig` | Read all persistent config parameters from LED MCU. |

---

## Typical Test Sessions

### Session 1: Direct stepping (MCU mode)

```
> tmc_init x
> tmc_current x 50
> tmc_microstep x 8
> enable x
> dir x 1
> step x 1600 500
```

### Session 2: TMC429 ramp mode (TMC429 mode)

```
> tmc_init x
> tmc_current x 50
> enable x
> mc_init
> mc_limits x 100 2000 500
> mc_ramp x
> mc_setpos x 0
> mc_target x 10000
> mc_pos x
> mc_target x 0
```

### Session 3: TMC429 velocity mode

```
> tmc_init x
> tmc_current x 50
> enable x
> mc_init
> mc_limits x 100 5000 1000
> mc_velocity x
> mc_vtarget x 3000
> mc_pos x
> mc_vtarget x -3000
> mc_stop x
```

### Session 4: Limit switch testing with TMC429

```
> mc_init
> mc_swpol high
> mc_rightsw on
> mc_leftstop x on
> mc_rightstop x on
> mc_switches
```

### Session 5: LED ring control

```
> led_getconfig
> led_set all static 255 0 0 128
> led_set 0 spin 0 255 0 200
> led_set 1 bounce 0 0 255 200
> led_status 0
> led_status 1
> led_config spin_width 3
> led_config spin_speed 50
> led_set all off 0 0 0 0
```

## Architecture Notes

### SERCOM Allocation

| SERCOM  | Usage              | Managed by                |
|---------|--------------------|---------------------------|
| SERCOM0 | (unused)           | —                         |
| SERCOM1 | Z1 TMC2209 UART    | Custom `Uart` instance    |
| SERCOM2 | SPI (TMC429)       | Arduino SPI library       |
| SERCOM3 | Y TMC2209 UART     | `Serial1` (BSP)          |
| SERCOM4 | X TMC2209 UART     | Custom `Uart` instance    |
| SERCOM5 | Z2 TMC2209 UART    | Custom `Uart` instance    |

### SERCOM5 Conflict and Resolution

Wire (I2C) and Z2 UART both need SERCOM5. Wire is excluded via `lib_ignore`,
and bitbang I2C (`soft_i2c.h`) is used for the MCP23017 instead.

### Library Dependencies

| Library | Purpose | PlatformIO ID |
|---------|---------|---------------|
| TMC2209 | TMC2209 UART communication | `janelia-arduino/TMC2209` |
| TMC429  | TMC429 SPI motion controller | `janelia-arduino/TMC429` |

The MCP23017 is accessed via bitbang I2C — no external library needed.
The LED MCU is controlled via the same bitbang I2C bus.

### File Overview

| File | Purpose |
|------|---------|
| `platformio.ini` | Build config, library deps, Wire exclusion |
| `src/main.cpp` | CLI, motor control, TMC2209/TMC429/MCP23017/LED MCU |
| `src/soft_i2c.h` | Minimal bitbang I2C master |
| `README.md` | This file |
| `led-mcu/` | Separate PlatformIO project for the LED MCU firmware |
