# PSU v4 Interactive Python PCB Tester

## Objective

Build a self-contained, interactive Python CLI program (`psu_test.py`) that guides a technician through the complete PCB functional test procedure defined in `README.md`. The program communicates exclusively with the main MCU (ATSAMD51J19A) over USB serial at 115200 baud using PySerial. It sends the correct command sequences automatically, parses and verifies every MCU response, prompts the user only when a physical action or a judgment call is required, and produces a pass/fail log at the end.

The secondary MCU (ATSAMD21E17A) is indirectly validated: if the `led_set` commands over I2C produce a visible LED response, the LED MCU, the I2C bus, and the NeoPixel chain are all confirmed working.

---

## Assumptions

- The bootloader has already been flashed to both MCUs and firmware has already been uploaded before the Python tester is run. The tester covers only the serial-CLI test phase (everything from "Send the following commands" onward in `README.md`).
- The RS232 loopback test requires a second serial port (USB-to-RS232 adapter). The program will open that port automatically and handle both ends of the loopback in software.
- The physical STEP/DIR toggle switch must be repositioned by the user between Section 2 (MCU-direct stepping) and Section 3 (TMC429 stepping). The program will explicitly instruct the user to do this at the appropriate transition point.
- `tmc_init` may need to be sent twice due to a known timing issue noted in `README.md:63`. The program will send it, wait, read the response, and retry once automatically if it does not receive a clean acknowledgement.
- All MCU responses are text lines terminated with `\n`. The program will read until a `\n` (or a configurable timeout) and strip whitespace before comparison.
- A test is considered "passed" when the MCU response contains a known success string (e.g., no `[ERROR]` or `[FAIL]` prefix) AND, for user-judgment items, the user explicitly answers `y`.
- On any failure the program logs the failure with the raw MCU response, asks the user whether to abort or continue, and records the outcome regardless.

---

## Program Architecture

### Module Structure

The program is a single file (`psu_test.py`) with clearly delineated internal sections:

```
psu_test.py
├── Imports & constants
├── SerialPort helper class
├── Response-verification helpers
├── Test section functions (one per subsystem)
│   ├── test_basic_comms()
│   ├── test_motors_direct()          # Section 2 — MCU-direct step
│   ├── test_motors_tmc429()          # Section 3 — TMC429 velocity mode
│   ├── test_limit_switches()         # Section 4 — MCP23017 + TMC429 IR sensors
│   ├── test_leds()                   # Section 5 — LED MCU via I2C
│   └── test_rs232()                  # Section 6 — RS232 loopback
├── Results summary & log writer
└── main() / entry point
```

---

## Implementation Plan

- [ ] **Task 1 — Project scaffolding and dependency declaration**
  Create `psu_test.py` with a top-level docstring describing purpose and usage, imports for `pyserial`, `time`, `sys`, `argparse`, `datetime`, and `json`. Add a `requirements.txt` containing `pyserial>=3.5`. Define all constants at the top: baud rate (115200), default read timeout (2 s), poll interval for sensor polling (1 s), motor step parameters (`count=1600`, `delay_us=2000`), TMC429 velocity mode parameters (`vmin=100`, `vmax=1000`, `amax=1000`, `vtarget=1000`), and the ordered list of limit switch names (`["ZEND2","ZEND1","YEND2","YEND1","XEND2","XEND1"]`).

- [ ] **Task 2 — CLI argument parsing**
  Use `argparse` to accept:
  - `--port` (required): COM port for the main MCU (e.g., `/dev/ttyACM0` or `COM3`)
  - `--rs232-port` (optional): COM port for the USB-to-RS232 adapter used in the RS232 loopback test; if omitted the RS232 test is skipped with a warning
  - `--timeout` (optional, default 2.0 s): serial read timeout
  - `--log` (optional): path for the JSON result log file (defaults to `psu_test_<timestamp>.json`)
  - `--skip` (optional, multi-value): list of section names to skip (e.g., `--skip rs232 leds`)

- [ ] **Task 3 — SerialPort helper class**
  Implement a thin wrapper around `serial.Serial` that exposes:
  - `send(cmd: str)`: writes `cmd + "\n"` encoded as ASCII and flushes
  - `readline_timeout() -> str`: reads one line with the configured timeout; returns empty string on timeout
  - `read_until_prompt(timeout=None) -> list[str]`: reads lines until the `"> "` prompt is seen or timeout expires, returning all lines collected (useful for `help` and multi-line responses)
  - `flush_input()`: discards any buffered incoming bytes before sending a command

- [ ] **Task 4 — Response verification helpers**
  Implement utility functions used by every test section:
  - `expect_ok(lines, keyword=None)`: returns `True` if none of the lines contain `[ERROR]` and optionally a `keyword` substring is present in at least one line
  - `check_and_log(section, step_name, result, raw_response)`: appends a `{section, step, pass/fail, raw}` record to a running results list; if `result` is `False`, prints the failure in red (via ANSI codes) and returns the user's choice to abort or continue
  - `ask_user(prompt_text) -> bool`: prints the prompt followed by `[y/n]: `, waits for the user to press a single key (no Enter required, using `sys.stdin` in raw mode on Unix or `msvcrt.getch` on Windows), returns `True` for `y`/`Y`

- [ ] **Task 5 — Section 1: Basic communications test (`test_basic_comms`)**
  - Print section header
  - Send `help` and call `read_until_prompt()`
  - Verify that the response contains at least one known command keyword (e.g., `tmc_init`)
  - Log result as `BASIC_COMMS / help_response`
  - This section has zero user prompts; it is entirely automated

- [ ] **Task 6 — Section 2: Motor direct-step test (`test_motors_direct`)**
  This section tests all four motor axes using MCU GPIO-direct step pulses (STEP/DIR toggle switch must be in MCU position). Each axis follows the identical sub-sequence below; implement as a loop over `[("x","X"),("y","Y"),("z1","Z1"),("z2","Z2")]` with a shared inner helper.

  For each axis:
  - Print `"--- Motor [axis]: direct step test ---"`
  - If axis is `z1` or `z2`: instruct the user `"Connect the test motor to the [Z1|Z2] port"` and wait for `[Enter]` (the only unavoidable physical action for this section, since you cannot detect motor connection)
  - If axis is `x` or `y`: instruct the user to move the motor connector to the correct header and wait for `[Enter]`
  - Send `tmc_init [axis]`; read response; if response does not look like a successful init, send `tmc_init [axis]` once more (automatic retry for the known timing issue — `README.md:63`)
  - Log `tmc_init` result
  - Send `tmc_microstep [axis] 8` → log
  - Send `tmc_current [axis] 50` → log
  - Send `tmc_enable [axis]` → log
  - Send `enable [z for z1/z2, else axis]` → log
  - Print `"Sending 1600 steps to [axis]..."`
  - Send `step [z/axis] 1600 2000` — this command is blocking on the MCU (~3.2 s at 2000 µs/step); wait up to 6 s for the prompt to return
  - Log `step` result
  - Send `disable [z/axis]` → log
  - Ask user: `"Is the motor spinning / did it make one full rotation? [y/n]: "` → log result

- [ ] **Task 7 — Section 3: Motor TMC429 velocity-mode test (`test_motors_tmc429`)**
  - Print transition notice: `"You must now flip the STEP/DIR toggle switch to the TMC429 position. Press [Enter] when done."`
  - Wait for Enter
  - Send `mc_init`; read response; verify it contains a version register readout (no error)
  - Log `mc_init` result
  - Loop over `[("x","x"),("y","y"),("z1","z"),("z2","z")]`:
    - If axis is `z2`, note `"Z2 shares STEP/DIR/nEnable with Z1. Move motor to Z2 connector."` and wait for Enter
    - Otherwise instruct to move motor to the correct connector and wait for Enter
    - Send `tmc_init [driver_axis]` (with auto-retry)
    - Send `tmc_microstep [driver_axis] 8`
    - Send `tmc_current [driver_axis] 50`
    - Send `tmc_enable [driver_axis]`
    - Send `mc_limits [motion_axis] 100 1000 1000`
    - Send `enable [motion_axis]`
    - Send `mc_velocity [motion_axis]`
    - Send `mc_vtarget [motion_axis] 1000`
    - Log all of the above steps
    - Ask user: `"Is the motor moving? [y/n]: "` → log result
    - Send `disable [motion_axis]`
    - Send `mc_stop [motion_axis]`

- [ ] **Task 8 — Section 4: Limit switch / IR sensor test (`test_limit_switches`)**
  Instruct the user that the motor should be disconnected for this section. Then:
  - Send `io_init` → log
  - Send `mc_swpol high` → log
  - Send `mc_rightsw on` → log
  - Send `io_read`; parse the response and verify all six switch signals read as OPEN/LOW; log the baseline result
  - Send `mc_switches`; verify all six switches read as inactive; log
  - For each of the six switches in order `["ZEND2","ZEND1","YEND2","YEND1","XEND2","XEND1"]`:
    - Print: `"Bridge the [NAME] connector (center pin → far pin). Waiting for state change..."`
    - Enter a polling loop: send `io_read` every 1 second, parse the response, check whether `[NAME]` is now TRIGGERED or HIGH
    - If the state change is detected within 15 seconds, log PASS and print `"[NAME] detected HIGH — remove the jumper."`
    - Wait in another polling loop for the signal to return LOW (timeout 15 s); once it returns LOW, move to the next switch
    - If timeout occurs on either the HIGH detection or the LOW-removal detection, log FAIL and ask user to abort or continue
  - Send `mc_switches` one final time; verify all switches are inactive again after all jumpers removed; log

  **Implementation note on `io_read` parsing:** The firmware prints labeled lines like `X END1: OPEN` or `Z END2: TRIGGERED`. Parse these by splitting on `:` and stripping whitespace; build a dict of `{signal_name: state}` to drive the polling logic. The exact label format is visible in `ATSAMD51J19A/src/main.cpp`.

- [ ] **Task 9 — Section 5: LED test (`test_leds`)**
  - Instruct the user to ensure the LED ring harness is connected to the LED port on the PCB
  - Wait for Enter (one unavoidable physical check)
  - Print `"Sending LED spin animation (red, full brightness) via I2C..."`
  - Send `led_set all spin 255 0 0 255`; read response; verify no error; log
  - Ask user: `"Are the LEDs showing a red spinning animation? [y/n]: "` → log
  - Send `led_status 0`; verify response code is `0xA0` or that the returned fields match what was sent (`anim=spin`, `r=255`, `g=0`, `b=0`, `brightness=255`); log
  - Send `led_getconfig`; verify a valid config is returned (num_rings ≥ 1, leds_per_ring ≥ 1); log
  - Send `led_set all off 0 0 0 0` to clean up

- [ ] **Task 10 — Section 6: RS232 loopback test (`test_rs232`)**
  - If `--rs232-port` was not provided, print a warning and mark section as SKIPPED in the log
  - Instruct the user: `"Ensure the USB-to-RS232 adapter is connected to the RS232 port on the PCB."`
  - Wait for Enter
  - Open the RS232 adapter port at 115200 baud using a second `SerialPort` instance
  - Generate a unique test string (e.g., `PSU_TEST_<random 8-char hex>`)
  - On the main MCU's serial port, the RS232 port is a hardware pass-through — the MCU firmware does not need any special command for this; communication goes directly through the RS232 transceiver chip
  - Write the test string + `\n` to the RS232 adapter port; read back from the main MCU serial port, expecting to see the same string reflected (loopback from the PCB RS232 connector back through USB serial)
  - **Then reverse:** write the test string to the main MCU serial port; read it back from the RS232 adapter port
  - Log both directions; if either direction fails, mark RS232 FAIL

  **Note:** This test requires careful analysis of the actual RS232 circuit on the PCB. The firmware currently has no dedicated RS232 passthrough command; the RS232 port routes directly to a SERCOM UART on the MCU that may not be exposed through the CLI. **This is flagged as an open question (see Risks section).** The tester will include a fallback: if no response is received within the timeout, instruct the user to manually verify using their own serial monitor and record their yes/no confirmation.

- [ ] **Task 11 — Results summary and JSON log**
  After all sections complete:
  - Print a formatted table of all test steps with PASS / FAIL / SKIP status
  - Print an overall PASS/FAIL verdict (all non-skipped steps must be PASS)
  - Write the full results list to the JSON log file specified by `--log`, including:
    - Timestamp (ISO 8601)
    - Port used
    - Each step's section, step name, status, and raw MCU response
  - Print the path to the log file

- [ ] **Task 12 — Cross-platform single-keypress input**
  Implement `getch()` that works on both Unix (using `tty`/`termios`) and Windows (using `msvcrt`) for the `ask_user` function. On platforms where this is not possible, fall back to `input()` requiring Enter. This minimizes keystrokes as requested.

- [ ] **Task 13 — ANSI color output helper**
  Implement `print_pass(msg)`, `print_fail(msg)`, `print_info(msg)`, and `print_warn(msg)` using ANSI escape codes for green, red, cyan, and yellow respectively. Auto-detect whether the terminal supports color (check `sys.stdout.isatty()`) and disable colors if not.

- [ ] **Task 14 — Graceful error handling and keyboard interrupt**
  Wrap the `main()` function body in a `try/except KeyboardInterrupt` block that closes the serial port, writes a partial log, and exits cleanly. Wrap each individual `send()` call in a try/except for `serial.SerialException` so a dropped connection produces a helpful message rather than a traceback.

---

## Test Section Order and User Interaction Summary

| Section | Name | Automated Steps | User Prompts |
|---|---|---|---|
| 1 | Basic comms | Send `help`, verify response | None |
| 2 | Direct motor stepping (×4 axes) | `tmc_init`+retry, microstep, current, enable, step, disable | Move connector (×4), confirm rotation (×4) |
| 3 | TMC429 velocity mode (×4 axes) | `mc_init`, full per-axis sequence | Flip toggle switch (×1), move connector (×4), confirm motion (×4) |
| 4 | Limit switches / IR sensors (×6) | `io_init`, `mc_swpol`, `mc_rightsw`, polling loop | Bridge jumper (×6, no keypresses — polling detects automatically) |
| 5 | LED rings | `led_set`, `led_status`, `led_getconfig` | Connect harness (×1), confirm LED animation (×1) |
| 6 | RS232 loopback | Open second port, write/read both directions | Connect adapter (×1) |

---

## Verification Criteria

- All six sections complete without raising an unhandled exception on a known-good board.
- Every automated MCU command produces a response that passes `expect_ok()` without modification on a known-good board.
- The IR sensor polling loop detects all six jumper insertions without any keyboard input from the user.
- The RS232 loopback test confirms bidirectional transmission using two serial ports.
- The JSON log file is written correctly and contains a complete record of every test step.
- The `tmc_init` auto-retry recovers silently when the first attempt times out.
- `ask_user()` returns without requiring Enter when a single `y` or `n` key is pressed.
- Running with `--skip rs232` completes without error and marks that section SKIPPED in the log.

---

## Potential Risks and Mitigations

1. **RS232 loopback mechanism unclear from firmware**
   The `README.md:186` describes the RS232 test as a direct hardware pass-through, but the firmware source does not expose a CLI command for it. The RS232 SERCOM UART may be configured in loopback/passthrough mode at the hardware level, or it may share the same USB serial as the CLI (unlikely). **Mitigation:** Before implementing `test_rs232()`, inspect the MCU's SERCOM allocation table — SERCOM0 is listed as unused in `ATSAMD51J19A/README.md:277`, which is likely the RS232 UART. If it is a hardware passthrough independent of the CLI, the Python program can open two ports simultaneously. If it shares the USB serial, the RS232 test cannot be automated without firmware changes, and the fallback user-confirmation path will be used.

2. **`io_read` output format not fully specified in source examined**
   The exact string format of `io_read` output (e.g., `"X END1: OPEN"` vs `"XEND1 = LOW"`) must be confirmed against `ATSAMD51J19A/src/main.cpp` before writing the parser. **Mitigation:** Read the `io_read` handler source directly and write the parser to match exactly. Include a raw-response dump in the log for every `io_read` call so failures can be diagnosed.

3. **`step` command is blocking and slow**
   `step x 1600 2000` takes ~3.2 seconds. During this time the MCU produces no serial output, so a short `readline_timeout()` would falsely report failure. **Mitigation:** Use a dedicated long timeout (at least 6 s) for the `step` command specifically, passed as a parameter to `read_until_prompt()`.

4. **TMC429 `mc_init` timing / version register validation**
   If the TMC429 SPI clock oscillator is not populated or is faulty, `mc_init` will return a bad version register. The program must distinguish between a firmware-level error and a hardware fault. **Mitigation:** Parse the version register value from the `mc_init` response and compare it against the known-good TMC429 version byte. Log the raw value regardless.

5. **Z axis ambiguity between `tmc_init z1`/`z2` and `step z` / `enable z`**
   Z1 and Z2 share STEP/DIR/nEnable. When testing Z2, `enable z` and `step z` will also affect Z1's hardware lines. If Z1's motor is still connected, it will also move. **Mitigation:** The program must explicitly instruct the user to disconnect the Z1 motor before connecting the Z2 motor in Section 2, Step 4. Add a clear prompt for this.

6. **No DIAG pin test coverage**
   The MCP23017 exposes TMC2209 DIAG pins (GPA0–GPA3). The `README.md` procedure does not test these explicitly, only limit switches. Triggering a real DIAG condition requires deliberately faulting a motor driver, which is not desirable in a production test. **Mitigation:** The program will call `io_read` during Section 2 motor testing and log the DIAG pin states for informational purposes (as a passive check), but will not treat a DIAG HIGH as a hard failure unless the user confirms a fault is expected.

7. **Single-keypress input on Windows vs. macOS/Linux**
   `tty`/`termios` is Unix-only. `msvcrt.getch()` is Windows-only. **Mitigation:** Detect the platform at runtime with `sys.platform` and import the appropriate module inside the `getch()` function. Fall back to `input()` on unsupported platforms.

8. **Serial port enumeration / wrong port selected**
   A technician may pass the wrong port. **Mitigation:** On startup, before opening the port, send `help` and verify the response. If it does not look like the expected PSU CLI, print an error and exit without running any tests.

---

## Alternative Approaches

1. **pytest + pytest-serial plugin instead of a standalone script:** Structures each section as a pytest test case, enabling standard pass/fail reporting and integration with CI. Trade-off: adds a heavier testing framework dependency and is less ergonomic for a production-floor operator who just wants to run a single script.

2. **Curses-based TUI instead of linear CLI:** Provides a live dashboard view of all test statuses updating in real time. Trade-off: significantly more implementation complexity, and curses is not natively supported on Windows without `windows-curses`. Not appropriate for a simple linear test flow.

3. **Separate test script per section:** Easier to run individual sections in isolation during development or partial re-testing. Trade-off: no unified log, more files to manage, and the operator must know which script to run. Can be offered as a future enhancement after the monolithic script is validated.

---

## Self-Critique: What This Plan Misses or Gets Wrong

The following gaps were identified upon review:

1. **No handling for the `tmc_init` known timing issue beyond a single retry.** The README notes "you may have to send the init twice, still figuring that timing out." A single retry may not be sufficient. The plan should specify a retry count of at least 3 with a 500 ms delay between attempts, and define what constitutes a "successful init" response string precisely so the retry condition is unambiguous.

2. **The RS232 test is architecturally underspecified.** The plan acknowledges the ambiguity but defers resolution. Without knowing whether SERCOM0 (the likely RS232 UART) is exposed as a second virtual COM port (CDC ACM) or as a dedicated hardware UART on a separate connector, it is impossible to correctly implement the test. A concrete pre-implementation step of reading the full MCU `main.cpp` SERCOM0 initialization code should be listed as a prerequisite task.

3. **No power-cycle or reset detection.** The README procedure includes "power off" steps between sections (e.g., after limit switch testing before flashing the LED MCU — `README.md:160`). The Python tester assumes the board is already fully flashed and powered on, which is a valid assumption, but the plan does not explicitly state what state the board must be in at the start of each section. The plan should include a startup checklist printed to the user before any commands are sent.

4. **`mc_status` and `tmc_status` are never called.** These diagnostic commands are valuable for verifying that the TMC429 and TMC2209 registers were actually written correctly (e.g., that microstep setting took effect). The plan should add a `tmc_status [axis]` call after the configuration sequence for each motor and parse key fields (MRES for microstep, IRUN for current) to validate programmatically rather than relying solely on motor rotation as evidence.

5. **LED `led_status` response parsing is underspecified.** The plan says "verify response code is `0xA0` or that returned fields match." But `led_status` output is printed as human-readable text by the MCU firmware, not raw hex bytes. The exact output format of the `led_status` CLI command must be confirmed from `ATSAMD51J19A/src/main.cpp` before the parser can be written.

6. **No teardown / cleanup sequence.** If a test fails midway, a motor may remain enabled and holding current, or the TMC429 may still be commanding velocity. The plan should include a cleanup function (`disable_all_motors()`) called on exit (normal or abnormal) that sends `mc_stopall`, `disable x`, `disable y`, `disable z`.

7. **IR sensor polling uses `io_read` but `mc_switches` also provides switch state.** The plan uses only `io_read` for polling. For completeness and to validate both the MCP23017 path and the TMC429 switch-input path simultaneously, the polling loop should call both `io_read` and `mc_switches` and require both to show the triggered state before logging PASS. This validates two independent read paths in one operation.

8. **No explicit test of `led_config` write/readback.** The plan calls `led_getconfig` to verify defaults but never writes a config value and reads it back. A write-then-readback of one parameter (e.g., `led_config spin_speed 200` followed by `led_getconfig` and verifying `spin_speed=200`) would confirm the I2C write path and the LED MCU's flash persistence in addition to just the read path.

9. **The plan does not address what happens if the board is a v4 with the ATSAMD21E18A vs. a v4.1 with the ATSAMD21E17A.** The LED firmware is the same, but the presence of the correct LED MCU affects what the operator sees. The startup checklist should ask the user to confirm the LED MCU variant so it is recorded in the log.

10. **No timeout differentiation for slow vs. fast commands.** Using a single `--timeout` value for all commands is insufficient. `step x 1600 2000` needs ~6 s; `help` needs <200 ms; `led_config` (with its 50 ms flash write delay) needs ~1 s. The plan should define a per-command timeout map and apply it in `read_until_prompt()`.
