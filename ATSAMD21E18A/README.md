# LED MCU Firmware

NeoPixel ring animation engine for the **ATSAMD21E18A** (Adafruit Trinket M0,
256KB flash / 32KB RAM), controlled via I2C by the main MCU (ATSAMD51J19A).

## Hardware

| Pin | Function | Description |
|-----|----------|-------------|
| D0 (PA08) | I2C SDA | Shared bus with main MCU |
| D1 (PA02) | NeoPixel data | Level-shifted 3.3V ↔ 5V on PCB |
| D2 (PA09) | I2C SCL | Shared bus with main MCU |

- **MCU:** ATSAMD21E18A (256KB flash, 32KB RAM)
- **I2C slave address:** `0x30`
- **Default config:** 2 rings × 12 LEDs each (24 LEDs total)
- **Max supported:** 8 rings, 64 LEDs total

## Bootloader

The stock Adafruit Trinket M0 bootloader
(`bootloader-trinket_m0-v3.16.0.bin`) is used as-is. Flash via ATMEL ICE or
another SWD programmer to address `0x00000000`, then double-tap the reset
button — you should see a `TRINKETBOOT` USB mass storage drive appear.

## Building and Uploading

```bash
pio run                    # Build
pio run -t upload          # Upload via sam-ba
```

### Manual upload

1. Double-tap the reset button on the LED MCU
2. A `TRINKETBOOT` USB drive will appear
3. Drag and drop `firmware.uf2` onto the drive

## Animations

| Name | Description |
|------|-------------|
| `off` | All LEDs off |
| `static` | All LEDs on at the specified color and brightness |
| `spin` | A window of `spin_width` LEDs rotates continuously around the ring |
| `bounce` | A window of `bounce_width` LEDs bounces back and forth on the ring |
| `pulse` | All LEDs alternate between on and off |

Each ring can run its own independent animation with its own color and
brightness. Animation speeds and window sizes are global (shared across rings)
and saved to flash.

## Persistent Configuration

The following parameters are saved to internal flash and survive power cycles:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `num_rings` | 2 | Number of NeoPixel rings on the data line |
| `leds_per_ring` | 12 | LEDs per ring |
| `spin_width` | 1 | Number of lit LEDs in spin animation |
| `bounce_width` | 1 | Number of lit LEDs in bounce animation |
| `spin_speed` | 100 ms | Time between spin animation steps |
| `bounce_speed` | 100 ms | Time between bounce animation steps |
| `pulse_speed` | 500 ms | Time between pulse on/off transitions |

## I2C Protocol

The main MCU (master) sends commands via I2C write, then reads the response
via I2C read. The LED MCU never initiates communication.

### Response Codes

| Code | Meaning |
|------|---------|
| `0xA0` | OK — command executed successfully |
| `0xE1` | Unknown command |
| `0xE2` | Bad format (wrong length, invalid parameter, etc.) |

### Command 0x01: SET_ANIMATION

Sets animation, color, and brightness for one or all rings in a single command.

**Master writes 7 bytes:**

| Byte | Field | Values |
|------|-------|--------|
| 0 | Command | `0x01` |
| 1 | Ring | 0-based index, or `0xFF` for all rings |
| 2 | Animation | 0=off, 1=static, 2=spin, 3=bounce, 4=pulse |
| 3 | Red | 0–255 |
| 4 | Green | 0–255 |
| 5 | Blue | 0–255 |
| 6 | Brightness | 0–255 |

**Master reads 1 byte:** response code.

### Command 0x02: GET_STATUS

Query the current animation state of a specific ring.

**Master writes 2 bytes:**

| Byte | Field | Values |
|------|-------|--------|
| 0 | Command | `0x02` |
| 1 | Ring | 0-based index |

**Master reads 7 bytes:**

| Byte | Field |
|------|-------|
| 0 | Response code |
| 1 | Current animation type |
| 2 | Red |
| 3 | Green |
| 4 | Blue |
| 5 | Brightness |
| 6 | Number of configured rings |

### Command 0x03: SET_CONFIG

Set a persistent configuration parameter. The value is saved to flash
immediately.

**Master writes 4 bytes:**

| Byte | Field | Values |
|------|-------|--------|
| 0 | Command | `0x03` |
| 1 | Parameter ID | See table below |
| 2 | Value low byte | |
| 3 | Value high byte | 0 for 8-bit params |

| Param ID | Name | Type |
|----------|------|------|
| `0x01` | num_rings | uint8 |
| `0x02` | leds_per_ring | uint8 |
| `0x03` | spin_width | uint8 |
| `0x04` | bounce_width | uint8 |
| `0x05` | spin_speed | uint16 (ms) |
| `0x06` | bounce_speed | uint16 (ms) |
| `0x07` | pulse_speed | uint16 (ms) |

**Master reads 1 byte:** response code.

### Command 0x04: GET_CONFIG

Read all configuration parameters at once.

**Master writes 1 byte:**

| Byte | Field | Values |
|------|-------|--------|
| 0 | Command | `0x04` |

**Master reads 11 bytes:**

| Byte | Field |
|------|-------|
| 0 | Response code |
| 1 | num_rings |
| 2 | leds_per_ring |
| 3 | spin_width |
| 4 | bounce_width |
| 5 | spin_speed low byte |
| 6 | spin_speed high byte |
| 7 | bounce_speed low byte |
| 8 | bounce_speed high byte |
| 9 | pulse_speed low byte |
| 10 | pulse_speed high byte |

## Testing from Main MCU CLI

The main MCU firmware includes `led_*` CLI commands that wrap this protocol:

```
> led_getconfig
[LED] LED MCU configuration (from flash):
[LED]   num_rings:     2
[LED]   leds_per_ring: 12
[LED]   spin_width:    1
[LED]   bounce_width:  1
[LED]   spin_speed:    100 ms
[LED]   bounce_speed:  100 ms
[LED]   pulse_speed:   500 ms

> led_set all static 255 0 0 128
[LED] Response: OK

> led_set 0 spin 0 255 0 200
[LED] Response: OK

> led_status 0
[LED] Ring 0 status:
[LED]   Animation:  spin
[LED]   Color:      (0, 255, 0)
[LED]   Brightness: 200
[LED]   Num rings:  2

> led_config spin_width 3
[LED] Config saved to flash on LED MCU.

> led_config spin_speed 50
[LED] Config saved to flash on LED MCU.
```
