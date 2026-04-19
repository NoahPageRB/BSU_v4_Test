#!/usr/bin/env python3
"""
PSU v4 Interactive PCB Tester

Guides a technician through the complete functional test procedure for PSU v4
PCBs. Communicates with the main MCU (ATSAMD51J19A) over USB serial at 115200
baud. Sends command sequences, verifies responses, prompts the user only when
physical action or visual confirmation is required, and produces a pass/fail
JSON log at the end.

Usage:
    python psu_test.py

    The program will scan for available serial ports, display their details,
    and prompt you to select which port is the Main MCU, which is the on-board
    RS232, and which is the external RS232 adapter (or skip those).

Requirements:
    pip install pyserial
"""

import argparse
import json
import random
import string
import sys
import time
from datetime import datetime

import serial
import serial.tools.list_ports

# =============================================================================
# Constants
# =============================================================================

BAUD_RATE = 115200
DEFAULT_TIMEOUT = 2.0
LONG_TIMEOUT = 8.0        # For blocking commands like 'step'
POLL_INTERVAL = 0.25        # Seconds between io_read polls for limit switches

MOTOR_STEP_COUNT = 1600
MOTOR_STEP_DELAY_US = 2000

TMC429_VMIN = 100
TMC429_VMAX = 1000
TMC429_AMAX = 1000
TMC429_VTARGET = 1000

LIMIT_SWITCHES = ["ZEND2", "ZEND1", "YEND2", "YEND1", "XEND2", "XEND1"]

# Map from switch name to the label substring in io_read output
SWITCH_IO_LABELS = {
    "XEND1": "X  END1",
    "XEND2": "X  END2",
    "YEND1": "Y  END1",
    "YEND2": "Y  END2",
    "ZEND1": "Z  END1",
    "ZEND2": "Z  END2",
}

# Map from switch name to the label substring in mc_switches output
SWITCH_MC_LABELS = {
    "XEND1": "X Left",
    "XEND2": "X Right",
    "YEND1": "Y Left",
    "YEND2": "Y Right",
    "ZEND1": "Z Left",
    "ZEND2": "Z Right",
}

# Per-command timeout overrides (command prefix -> timeout in seconds)
COMMAND_TIMEOUTS = {
    "step": LONG_TIMEOUT,
    "help": 3.0,
    "led_config": 3.0,
    "mc_init": 4.0,
}

# =============================================================================
# ANSI Color Helpers
# =============================================================================

_USE_COLOR = sys.stdout.isatty()


def _ansi(code, msg):
    if _USE_COLOR:
        return f"\033[{code}m{msg}\033[0m"
    return msg


def print_pass(msg):
    print(_ansi("1;32", f"[PASS] {msg}"))


def print_fail(msg):
    print(_ansi("1;31", f"[FAIL] {msg}"))


def print_info(msg):
    print(_ansi("36", f"[INFO] {msg}"))


def print_warn(msg):
    print(_ansi("1;33", f"[WARN] {msg}"))


def print_header(msg):
    sep = "=" * 60
    print(f"\n{_ansi('1;36', sep)}")
    print(_ansi("1;36", f"  {msg}"))
    print(f"{_ansi('1;36', sep)}\n")


def print_subheader(msg):
    print(f"\n{_ansi('1;35', f'--- {msg} ---')}\n")


# =============================================================================
# Cross-Platform Single-Keypress Input
# =============================================================================

def getch():
    """Read a single keypress without requiring Enter."""
    try:
        if sys.platform == "win32":
            import msvcrt
            ch = msvcrt.getwch()
            return ch
        else:
            import tty
            import termios
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch
    except Exception:
        # Fallback: require Enter
        return input()[0] if True else ""


def ask_user(prompt_text):
    """Prompt user with [y/n] and return True for y/Y. Single keypress."""
    sys.stdout.write(f"{prompt_text} [y/n]: ")
    sys.stdout.flush()
    ch = getch()
    print(ch)  # Echo the character
    return ch.lower() == "y"


def wait_for_enter(prompt_text="Press [Enter] to continue..."):
    """Wait for the user to press Enter."""
    input(f"{prompt_text}")


# =============================================================================
# SerialPort Helper Class
# =============================================================================

class SerialPort:
    """Thin wrapper around serial.Serial for MCU CLI communication."""

    def __init__(self, port, baud=BAUD_RATE, timeout=DEFAULT_TIMEOUT):
        self.port_name = port
        self.timeout = timeout
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(0.1)
        self.flush_input()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def flush_input(self):
        """Discard any buffered incoming bytes."""
        self.ser.reset_input_buffer()

    def send(self, cmd):
        """Write a command string followed by newline."""
        self.flush_input()
        self.ser.write((cmd + "\n").encode("ascii"))
        self.ser.flush()

    def readline_timeout(self, timeout=None):
        """Read one line with the given timeout. Returns empty string on timeout."""
        old_timeout = self.ser.timeout
        if timeout is not None:
            self.ser.timeout = timeout
        try:
            raw = self.ser.readline()
            return raw.decode("ascii", errors="replace").rstrip("\r\n")
        except serial.SerialException:
            return ""
        finally:
            self.ser.timeout = old_timeout

    def read_until_prompt(self, timeout=None):
        """
        Read lines until the '> ' prompt or timeout. Returns all lines collected.
        The prompt line itself is not included in the output.
        """
        if timeout is None:
            timeout = self.timeout
        lines = []
        deadline = time.time() + timeout
        old_timeout = self.ser.timeout
        try:
            while time.time() < deadline:
                remaining = max(0.1, deadline - time.time())
                self.ser.timeout = remaining
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="replace").rstrip("\r\n")
                # The MCU prompt is "> " alone on a line (no leading content).
                # Must not match "> " inside help text angle brackets like <x|y|z>.
                stripped = line.strip()
                if stripped == ">":
                    break
                # Check if line is exactly the prompt (possibly with whitespace)
                if line == "> " or line == ">":
                    break
                # Sometimes the prompt appears at the end of the last output line
                # e.g. "some output\r\n> " — but only when "> " is at the very end
                # and preceded by a newline-like boundary (start of line)
                if line.endswith("\n> ") or line.endswith("\r> "):
                    content = line[:-2].strip()
                    if content:
                        lines.append(content)
                    break
                lines.append(line)
        finally:
            self.ser.timeout = old_timeout
        return lines

    def send_command(self, cmd, timeout=None):
        """Send a command and return all response lines (up to the next prompt)."""
        if timeout is None:
            # Look up per-command timeout
            cmd_prefix = cmd.split()[0] if cmd.strip() else ""
            timeout = COMMAND_TIMEOUTS.get(cmd_prefix, self.timeout)
        self.send(cmd)
        # Small delay to let the MCU start processing
        time.sleep(0.05)
        return self.read_until_prompt(timeout=timeout)


# =============================================================================
# Response Verification Helpers & Real-Time JSON Log
# =============================================================================


class TestLog:
    """Accumulates test results and writes them to disk after every step."""

    def __init__(self, log_path):
        self.log_path = log_path
        self.results = []
        self._flush()  # Create the file immediately

    def append(self, record):
        self.results.append(record)
        self._flush()

    def _flush(self):
        """Rewrite the JSON log file with current results."""
        pass_count = sum(1 for r in self.results if r["status"] == "PASS")
        fail_count = sum(1 for r in self.results if r["status"] == "FAIL")
        skip_count = sum(1 for r in self.results if r["status"] == "SKIP")
        log_data = {
            "timestamp": datetime.now().isoformat(),
            "overall": "FAIL" if fail_count > 0 else "PASS",
            "summary": {
                "total": len(self.results),
                "passed": pass_count,
                "failed": fail_count,
                "skipped": skip_count,
            },
            "results": self.results,
        }
        try:
            with open(self.log_path, "w") as f:
                json.dump(log_data, f, indent=2)
        except IOError:
            pass  # Best effort; don't crash the test over a log write


# Module-level log instance; set in main() before tests run
_log: TestLog = None


def expect_ok(lines, keyword=None):
    """
    Return True if no line contains '[ERROR]' and optionally a keyword
    substring is present in at least one line.
    """
    for line in lines:
        if "[ERROR]" in line:
            return False
    if keyword:
        return any(keyword in line for line in lines)
    return True


def lines_str(lines):
    """Join lines into a single string for logging."""
    return "\n".join(lines)


def check_and_log(section, step_name, result, raw_lines, abort_on_fail=True):
    """
    Record a test step result. Returns True to continue, False to abort.
    Writes to disk immediately via TestLog.
    """
    status = "PASS" if result else "FAIL"
    raw = lines_str(raw_lines) if isinstance(raw_lines, list) else str(raw_lines)

    _log.append({
        "section": section,
        "step": step_name,
        "status": status,
        "raw": raw,
    })

    if result:
        print_pass(f"{section} / {step_name}")
    else:
        print_fail(f"{section} / {step_name}")
        print(f"  Raw response:\n  {raw[:500]}")
        if abort_on_fail:
            if not ask_user("Continue testing despite failure?"):
                return False
    return True


# =============================================================================
# Cleanup / Teardown
# =============================================================================

def cleanup(sp):
    """Disable all motors and stop TMC429 to leave the board in a safe state."""
    print_info("Cleaning up: disabling motors and stopping TMC429...")
    try:
        sp.send_command("mc_stopall", timeout=2.0)
    except Exception:
        pass
    for axis in ["x", "y", "z"]:
        try:
            sp.send_command(f"disable {axis}", timeout=2.0)
        except Exception:
            pass
    try:
        sp.send_command("led_set all off 0 0 0 0", timeout=2.0)
    except Exception:
        pass


# =============================================================================
# Section 1: Basic Communications Test
# =============================================================================

def test_basic_comms(sp):
    print_header("Section 1: Basic Communications Test")
    print_info("Sending 'help' command to verify MCU is responding...")

    lines = sp.send_command("help", timeout=3.0)
    ok = expect_ok(lines, keyword="tmc_init")

    if not check_and_log("BASIC_COMMS", "help_response", ok, lines):
        return False

    print_info("MCU is communicating. CLI help received successfully.")
    return True


# =============================================================================
# Section 2: Motor Direct-Step Test (MCU mode)
# =============================================================================

def _tmc_init_with_retry(sp, motor, section):
    """
    Send tmc_init for the given motor. Retry once if the first attempt
    fails (known timing issue). Returns (success, lines).
    """
    for attempt in range(1, 3):
        lines = sp.send_command(f"tmc_init {motor}", timeout=3.0)
        if expect_ok(lines, keyword="Communication OK"):
            return True, lines
        if expect_ok(lines) and not any("not responding" in l for l in lines):
            # No error but also no "Communication OK" -- might be ok
            # Check if there's a warning
            if not any("WARNING" in l for l in lines):
                return True, lines
        if attempt == 1:
            print_warn(f"tmc_init {motor} did not get clean ack on attempt 1, retrying...")
            time.sleep(0.5)

    # Both attempts failed
    return False, lines


def test_motors_direct(sp):
    print_header("Section 2: Motor Direct-Step Test (MCU Mode)")
    print_info("Ensure the STEP/DIR toggle switch is in the MCU position.")
    print_info("You will need a test stepper motor to move between headers.")

    axes = [
        ("x", "x", "X"),
        ("y", "y", "Y"),
        ("z1", "z", "Z1"),
        ("z2", "z", "Z2"),
    ]

    for driver_axis, motion_axis, label in axes:
        print_subheader(f"Motor {label}: Direct Step Test")

        wait_for_enter(f"Connect the test motor to the {label} port, then press [Enter]...")

        # tmc_init with retry
        ok, lines = _tmc_init_with_retry(sp, driver_axis, "MOTORS_DIRECT")
        if not check_and_log("MOTORS_DIRECT", f"tmc_init_{driver_axis}", ok, lines):
            return False

        # tmc_microstep
        lines = sp.send_command(f"tmc_microstep {driver_axis} 8")
        ok = expect_ok(lines, keyword="microsteps set to")
        if not check_and_log("MOTORS_DIRECT", f"tmc_microstep_{driver_axis}", ok, lines):
            return False

        # tmc_current
        lines = sp.send_command(f"tmc_current {driver_axis} 50")
        ok = expect_ok(lines, keyword="run current set to")
        if not check_and_log("MOTORS_DIRECT", f"tmc_current_{driver_axis}", ok, lines):
            return False

        # tmc_enable (software enable)
        lines = sp.send_command(f"tmc_enable {driver_axis}")
        ok = expect_ok(lines, keyword="software-ENABLED")
        if not check_and_log("MOTORS_DIRECT", f"tmc_enable_{driver_axis}", ok, lines):
            return False

        # enable (hardware nEnable)
        lines = sp.send_command(f"enable {motion_axis}")
        ok = expect_ok(lines, keyword="ENABLED")
        if not check_and_log("MOTORS_DIRECT", f"enable_{motion_axis}", ok, lines):
            return False

        # step command (blocking ~3.2s)
        print_info(f"Sending {MOTOR_STEP_COUNT} steps to {label} (this takes a few seconds)...")
        lines = sp.send_command(
            f"step {motion_axis} {MOTOR_STEP_COUNT} {MOTOR_STEP_DELAY_US}",
            timeout=LONG_TIMEOUT,
        )
        ok = expect_ok(lines, keyword="done")
        if not check_and_log("MOTORS_DIRECT", f"step_{motion_axis}_{label}", ok, lines):
            return False

        # disable
        lines = sp.send_command(f"disable {motion_axis}")
        ok = expect_ok(lines)
        if not check_and_log("MOTORS_DIRECT", f"disable_{motion_axis}_{label}", ok, lines):
            return False

        # User confirmation
        user_ok = ask_user(f"Did motor {label} spin / make approximately one full rotation?")
        if not check_and_log("MOTORS_DIRECT", f"user_confirm_{label}", user_ok, ["user response"]):
            return False

    print_pass("All direct motor stepping tests complete.")
    return True


# =============================================================================
# Section 3: Motor TMC429 Velocity-Mode Test
# =============================================================================

def test_motors_tmc429(sp):
    print_header("Section 3: Motor TMC429 Velocity-Mode Test")
    print_info("STEP/DIR jumpers should already be in the TMC429 position.")

    # mc_init
    print_info("Initializing TMC429 motion controller...")
    lines = sp.send_command("mc_init", timeout=4.0)
    ok = expect_ok(lines, keyword="Communication OK")
    if not check_and_log("MOTORS_TMC429", "mc_init", ok, lines):
        return False

    axes = [
        ("x", "x", "X"),
        ("y", "y", "Y"),
        ("z1", "z", "Z1"),
        ("z2", "z", "Z2"),
    ]

    for driver_axis, motion_axis, label in axes:
        print_subheader(f"Motor {label}: TMC429 Velocity Test")

        wait_for_enter(f"Connect the test motor to the {label} port, then press [Enter]...")

        # tmc_init with retry
        ok, lines = _tmc_init_with_retry(sp, driver_axis, "MOTORS_TMC429")
        if not check_and_log("MOTORS_TMC429", f"tmc_init_{driver_axis}", ok, lines):
            return False

        # tmc_microstep
        lines = sp.send_command(f"tmc_microstep {driver_axis} 8")
        ok = expect_ok(lines, keyword="microsteps set to")
        if not check_and_log("MOTORS_TMC429", f"tmc_microstep_{driver_axis}", ok, lines):
            return False

        # tmc_current
        lines = sp.send_command(f"tmc_current {driver_axis} 50")
        ok = expect_ok(lines, keyword="run current set to")
        if not check_and_log("MOTORS_TMC429", f"tmc_current_{driver_axis}", ok, lines):
            return False

        # tmc_enable
        lines = sp.send_command(f"tmc_enable {driver_axis}")
        ok = expect_ok(lines, keyword="software-ENABLED")
        if not check_and_log("MOTORS_TMC429", f"tmc_enable_{driver_axis}", ok, lines):
            return False

        # mc_limits
        lines = sp.send_command(
            f"mc_limits {motion_axis} {TMC429_VMIN} {TMC429_VMAX} {TMC429_AMAX}"
        )
        ok = expect_ok(lines, keyword="Setting limits")
        if not check_and_log("MOTORS_TMC429", f"mc_limits_{motion_axis}_{label}", ok, lines):
            return False

        # enable (hardware)
        lines = sp.send_command(f"enable {motion_axis}")
        ok = expect_ok(lines, keyword="ENABLED")
        if not check_and_log("MOTORS_TMC429", f"enable_{motion_axis}_{label}", ok, lines):
            return False

        # mc_velocity
        lines = sp.send_command(f"mc_velocity {motion_axis}")
        ok = expect_ok(lines, keyword="VELOCITY mode")
        if not check_and_log("MOTORS_TMC429", f"mc_velocity_{motion_axis}_{label}", ok, lines):
            return False

        # mc_vtarget
        lines = sp.send_command(f"mc_vtarget {motion_axis} {TMC429_VTARGET}")
        ok = expect_ok(lines, keyword="target velocity set to")
        if not check_and_log("MOTORS_TMC429", f"mc_vtarget_{motion_axis}_{label}", ok, lines):
            return False

        # Give the motor a moment to spin
        time.sleep(1.5)

        # User confirmation
        user_ok = ask_user(f"Is motor {label} moving?")
        if not check_and_log("MOTORS_TMC429", f"user_confirm_{label}", user_ok, ["user response"]):
            return False

        # Stop and disable
        sp.send_command(f"disable {motion_axis}")
        sp.send_command(f"mc_stop {motion_axis}")

    print_pass("All TMC429 velocity mode tests complete.")
    return True


# =============================================================================
# Section 4: Limit Switch / IR Sensor Test
# =============================================================================

def _parse_io_read(lines):
    """
    Parse io_read response into a dict of switch states.
    Example line: '[IO]   X  END1 (GPB2): TRIGGERED'
    Returns dict like {'XEND1': 'TRIGGERED', 'XEND2': 'open', ...}
    """
    states = {}
    for line in lines:
        for sw_name, label in SWITCH_IO_LABELS.items():
            if label in line:
                if "TRIGGERED" in line:
                    states[sw_name] = "TRIGGERED"
                else:
                    states[sw_name] = "open"
    return states


def _parse_mc_switches(lines):
    """
    Parse mc_switches response into a dict of switch states.
    Example line: '[MC]   X Left  (END1/REF1L): ACTIVE'
    Returns dict like {'XEND1': 'ACTIVE', 'XEND2': 'inactive', ...}
    """
    states = {}
    for line in lines:
        for sw_name, label in SWITCH_MC_LABELS.items():
            if label in line:
                if "ACTIVE" in line:
                    states[sw_name] = "ACTIVE"
                else:
                    states[sw_name] = "inactive"
    return states


def test_limit_switches(sp):
    print_header("Section 4: Limit Switch Test")
    print_info("Disconnect the test motor from all headers before proceeding.")
    wait_for_enter("Press [Enter] when the motor is disconnected...")

    # io_init
    lines = sp.send_command("io_init")
    ok = expect_ok(lines, keyword="initialized successfully")
    if not check_and_log("LIMIT_SWITCHES", "io_init", ok, lines):
        return False

    # mc_swpol high
    lines = sp.send_command("mc_swpol high")
    ok = expect_ok(lines, keyword="ACTIVE HIGH")
    if not check_and_log("LIMIT_SWITCHES", "mc_swpol_high", ok, lines):
        return False

    # mc_rightsw on
    lines = sp.send_command("mc_rightsw on")
    ok = expect_ok(lines, keyword="ENABLED")
    if not check_and_log("LIMIT_SWITCHES", "mc_rightsw_on", ok, lines):
        return False

    # Baseline io_read: all should be open
    lines = sp.send_command("io_read")
    states = _parse_io_read(lines)
    all_open = all(states.get(sw) == "open" for sw in LIMIT_SWITCHES)
    if not check_and_log("LIMIT_SWITCHES", "baseline_io_read_all_open", all_open, lines):
        return False

    # Baseline mc_switches: all should be inactive
    lines = sp.send_command("mc_switches")
    mc_states = _parse_mc_switches(lines)
    all_inactive = all(mc_states.get(sw) == "inactive" for sw in LIMIT_SWITCHES)
    if not check_and_log("LIMIT_SWITCHES", "baseline_mc_switches_all_inactive", all_inactive, lines):
        return False

    # Test each switch via polling
    for sw_name in LIMIT_SWITCHES:
        print_subheader(f"Testing {sw_name}")
        print_info(
            f"Bridge the {sw_name} connector (center pin to far pin from board edge)."
        )
        print_info("Waiting for state change (polling every 1s, Ctrl+C to abort)...")

        # Poll for TRIGGERED state (no timeout — wait as long as the user needs)
        while True:
            lines = sp.send_command("io_read")
            states = _parse_io_read(lines)
            if states.get(sw_name) == "TRIGGERED":
                break
            time.sleep(POLL_INTERVAL)

        print_pass(f"{sw_name} detected TRIGGERED")

        # Also verify via mc_switches
        lines = sp.send_command("mc_switches")
        mc_states = _parse_mc_switches(lines)
        mc_ok = mc_states.get(sw_name) == "ACTIVE"
        if not check_and_log("LIMIT_SWITCHES", f"{sw_name}_mc_confirm", mc_ok, lines):
            return False

        check_and_log("LIMIT_SWITCHES", f"{sw_name}_detect", True, [f"{sw_name} TRIGGERED"])

        # Wait for jumper removal (no timeout)
        print_info(f"Now remove the jumper from {sw_name}. Waiting for it to return to open...")
        while True:
            lines = sp.send_command("io_read")
            states = _parse_io_read(lines)
            if states.get(sw_name) == "open":
                break
            time.sleep(POLL_INTERVAL)

        print_pass(f"{sw_name} returned to open")

    # Final mc_switches verification: all inactive
    lines = sp.send_command("mc_switches")
    mc_states = _parse_mc_switches(lines)
    all_inactive = all(mc_states.get(sw) == "inactive" for sw in LIMIT_SWITCHES)
    if not check_and_log("LIMIT_SWITCHES", "final_mc_switches_all_inactive", all_inactive, lines):
        return False

    print_pass("All limit switch tests complete.")
    return True


# =============================================================================
# Section 5: LED Test
# =============================================================================

def test_leds(sp):
    print_header("Section 5: LED Ring Test")
    print_info("Ensure the LED ring harness is connected to the LED port on the PCB.")
    wait_for_enter("Press [Enter] when the LED harness is connected...")

    # led_set all spin red full brightness
    print_info("Sending LED spin animation (red, full brightness) via I2C...")
    lines = sp.send_command("led_set all spin 255 0 0 255")
    ok = expect_ok(lines, keyword="OK")
    if not check_and_log("LEDS", "led_set_spin_red", ok, lines):
        return False

    # User confirmation
    user_ok = ask_user("Are the LEDs showing a red spinning animation?")
    if not check_and_log("LEDS", "user_confirm_led_spin", user_ok, ["user response"]):
        return False

    # Restore default spin_speed (100ms)
    sp.send_command("led_config spin_speed 100")

    # Turn LEDs off
    sp.send_command("led_set all off 0 0 0 0")

    print_pass("LED tests complete.")
    return True


# =============================================================================
# Section 6: RS232 Loopback Test
# =============================================================================

def test_rs232(sp, rs232_port):
    print_header("Section 6: RS232 Loopback Test")

    if not rs232_port:
        print_warn("RS232 adapter port was skipped. Falling back to manual verification.")
        print_info(
            "To test RS232: connect a USB-to-RS232 adapter to the RS232 port on the PCB,"
        )
        print_info(
            "open a separate serial monitor at 115200 baud on the adapter, and verify "
            "bidirectional communication manually."
        )
        user_ok = ask_user("Did you verify RS232 communication works in both directions? (n to skip)")
        status = "PASS" if user_ok else "SKIP"
        _log.append({
            "section": "RS232",
            "step": "manual_verification",
            "status": status,
            "raw": "Manual user verification",
        })
        if user_ok:
            print_pass("RS232 manual verification passed.")
        else:
            print_warn("RS232 test skipped.")
        return True

    print_info(f"Opening RS232 adapter port: {rs232_port}")
    wait_for_enter("Ensure the USB-to-RS232 adapter is connected to the PCB's RS232 port. Press [Enter]...")

    try:
        rs232 = serial.Serial(rs232_port, BAUD_RATE, timeout=2.0)
        time.sleep(0.1)
        rs232.reset_input_buffer()
    except serial.SerialException as e:
        print_fail(f"Could not open RS232 port: {e}")
        _log.append({
            "section": "RS232",
            "step": "open_rs232_port",
            "status": "FAIL",
            "raw": str(e),
        })
        return ask_user("Continue testing despite RS232 port failure?")

    test_string = "PSU_TEST_" + "".join(random.choices(string.hexdigits[:16], k=8))

    # Direction 1: RS232 adapter -> MCU USB serial
    print_info(f"Sending test string via RS232 adapter: {test_string}")
    rs232.write((test_string + "\n").encode("ascii"))
    rs232.flush()

    # Read from MCU serial port
    time.sleep(1.0)
    received = ""
    deadline = time.time() + 3.0
    while time.time() < deadline:
        raw = sp.ser.readline()
        if raw:
            line = raw.decode("ascii", errors="replace").strip()
            if test_string in line:
                received = line
                break

    ok1 = test_string in received
    check_and_log("RS232", "rs232_to_mcu", ok1,
                  [f"Sent: {test_string}", f"Received: {received}"])

    # Direction 2: MCU USB serial -> RS232 adapter
    test_string2 = "PSU_TEST_" + "".join(random.choices(string.hexdigits[:16], k=8))
    print_info(f"Sending test string via MCU serial: {test_string2}")
    sp.ser.write((test_string2 + "\n").encode("ascii"))
    sp.ser.flush()

    time.sleep(1.0)
    received2 = ""
    deadline = time.time() + 3.0
    while time.time() < deadline:
        raw = rs232.readline()
        if raw:
            line = raw.decode("ascii", errors="replace").strip()
            if test_string2 in line:
                received2 = line
                break

    ok2 = test_string2 in received2
    check_and_log("RS232", "mcu_to_rs232", ok2,
                  [f"Sent: {test_string2}", f"Received: {received2}"])

    rs232.close()

    if not ok1 or not ok2:
        print_warn("Automated RS232 loopback did not succeed. This may be a firmware/routing issue.")
        print_info("Falling back to manual verification.")
        user_ok = ask_user("Did you manually verify RS232 works in both directions?")
        if user_ok:
            _log.append({
                "section": "RS232",
                "step": "manual_fallback",
                "status": "PASS",
                "raw": "Manual user verification after auto-test failure",
            })
        else:
            _log.append({
                "section": "RS232",
                "step": "manual_fallback",
                "status": "FAIL",
                "raw": "User could not verify RS232",
            })
            return ask_user("Continue testing despite RS232 failure?")

    print_pass("RS232 test complete.")
    return True


# =============================================================================
# Results Summary & JSON Log
# =============================================================================

def print_results_summary():
    """Print a formatted results table. The JSON log is already on disk."""
    print_header("Test Results Summary")

    # Table header
    print(f"  {'Section':<20} {'Step':<40} {'Status':<8}")
    print(f"  {'-'*20} {'-'*40} {'-'*8}")

    pass_count = 0
    fail_count = 0
    skip_count = 0

    for r in _log.results:
        status = r["status"]
        section = r["section"][:20]
        step = r["step"][:40]

        if status == "PASS":
            status_str = _ansi("1;32", "PASS")
            pass_count += 1
        elif status == "FAIL":
            status_str = _ansi("1;31", "FAIL")
            fail_count += 1
        else:
            status_str = _ansi("1;33", "SKIP")
            skip_count += 1

        print(f"  {section:<20} {step:<40} {status_str}")

    print()
    print(f"  Total: {len(_log.results)} steps | "
          f"{_ansi('1;32', f'{pass_count} passed')} | "
          f"{_ansi('1;31', f'{fail_count} failed')} | "
          f"{_ansi('1;33', f'{skip_count} skipped')}")

    overall = "PASS" if fail_count == 0 else "FAIL"
    if overall == "PASS":
        print(f"\n  {_ansi('1;32', '*** OVERALL: PASS ***')}")
    else:
        print(f"\n  {_ansi('1;31', '*** OVERALL: FAIL ***')}")

    print_info(f"Log file: {_log.log_path}")


# =============================================================================
# Startup Checklist
# =============================================================================

def print_startup_checklist(test_set):
    """Print a pre-test checklist for the operator, tailored to the test set."""
    print_header("PSU v4 PCB Tester - Startup Checklist")
    print("  Before starting, verify the following:")
    print("    1. Both MCU bootloaders have been flashed (Main + LED)")
    print("    2. Both MCU firmwares have been uploaded")
    print("    3. The board is powered on (24V applied)")
    print("    4. USB-C cable is connected from the board to this computer")
    print("    5. A test stepper motor is available")
    if test_set == "A":
        print("    6. STEP/DIR toggle switch is in the MCU position")
    else:
        print("    6. A jumper wire is available (for limit switch testing)")
        print("    7. LED ring harness is available")
        print("    8. STEP/DIR toggle switch is in the TMC429 position")
    print()


# =============================================================================
# --continue: Resume from a previous log's first failure
# =============================================================================

# Ordered list of sections per test set, used to determine skip/resume logic.
SECTIONS_A = ["BASIC_COMMS", "MOTORS_DIRECT"]
SECTIONS_B = ["BASIC_COMMS", "MOTORS_TMC429", "LIMIT_SWITCHES", "LEDS", "RS232"]


def parse_continue_log(log_path):
    """
    Read a previous test log JSON, find the first FAIL entry, and return:
      (test_set, resume_section, prior_results)
    where prior_results is the list of PASS results from sections before
    the failed section (to be pre-populated into the new log).
    Returns None if the file can't be parsed or has no failures.
    """
    try:
        with open(log_path) as f:
            data = json.load(f)
    except (IOError, json.JSONDecodeError) as e:
        print_fail(f"Could not read log file: {e}")
        sys.exit(1)

    results = data.get("results", [])
    if not results:
        print_fail("Log file contains no test results.")
        sys.exit(1)

    # Find the first failure
    fail_section = None
    for r in results:
        if r["status"] == "FAIL":
            fail_section = r["section"]
            break

    if fail_section is None:
        print_warn("No failures found in the log file. Nothing to resume.")
        sys.exit(0)

    # Determine which test set this section belongs to
    if fail_section in SECTIONS_A:
        test_set = "A"
        section_order = SECTIONS_A
    elif fail_section in SECTIONS_B:
        test_set = "B"
        section_order = SECTIONS_B
    else:
        print_fail(f"Unknown section '{fail_section}' in log. Cannot determine test set.")
        sys.exit(1)

    # Collect passing results from sections strictly before the failed section
    resume_idx = section_order.index(fail_section)
    sections_to_keep = set(section_order[:resume_idx])
    prior_results = [r for r in results if r["section"] in sections_to_keep and r["status"] == "PASS"]

    print_info(f"Resuming from section: {fail_section}")
    if prior_results:
        print_info(f"Carrying forward {len(prior_results)} passing results from earlier sections.")

    return test_set, fail_section, prior_results


# =============================================================================
# Interactive Port Selection
# =============================================================================

def scan_ports():
    """Scan and return a list of available serial ports with details."""
    ports = sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)
    return ports


def print_port_table(ports):
    """Print a numbered table of available serial ports with details."""
    if not ports:
        print_fail("No serial ports found!")
        return

    # Determine column widths
    dev_w = max(len(p.device) for p in ports)
    dev_w = max(dev_w, len("Device"))
    desc_w = max((len(p.description) for p in ports), default=11)
    desc_w = max(min(desc_w, 50), len("Description"))

    # Header
    print(f"  {'#':<4} {'Device':<{dev_w}}  {'Description':<{desc_w}}  {'Hardware ID'}")
    print(f"  {'─'*4} {'─'*dev_w}  {'─'*desc_w}  {'─'*30}")

    for i, p in enumerate(ports):
        desc = p.description[:50] if p.description else "n/a"
        hwid = p.hwid if p.hwid else "n/a"
        # Highlight common board identifiers
        marker = ""
        desc_lower = (p.description or "").lower()
        if "metro" in desc_lower or "m4" in desc_lower:
            marker = _ansi("1;32", " <-- likely Main MCU")
        elif "trinket" in desc_lower or "m0" in desc_lower:
            marker = _ansi("1;33", " <-- likely LED MCU")
        elif "rs232" in desc_lower or "rs-232" in desc_lower or "uart" in desc_lower:
            marker = _ansi("1;36", " <-- likely RS232 adapter")
        elif "usbserial" in desc_lower or "usb-serial" in desc_lower or "ft232" in desc_lower or "ch340" in desc_lower or "cp210" in desc_lower or "pl2303" in desc_lower:
            marker = _ansi("1;36", " <-- likely USB-Serial adapter")
        print(f"  {i+1:<4} {p.device:<{dev_w}}  {desc:<{desc_w}}  {hwid}{marker}")

    print()


def pick_port(ports, role, allow_skip=False):
    """
    Ask the user to pick a port by number for the given role.
    If allow_skip is True, the user can press 's' to skip.
    Returns the port device string, or None if skipped.
    """
    skip_hint = " (or 's' to skip)" if allow_skip else ""
    while True:
        choice = input(f"  Select port # for {role}{skip_hint}: ").strip().lower()
        if allow_skip and choice == "s":
            return None
        try:
            idx = int(choice) - 1
            if 0 <= idx < len(ports):
                print_info(f"Selected {ports[idx].device} for {role}")
                return ports[idx].device
            else:
                print_warn(f"Enter a number between 1 and {len(ports)}")
        except ValueError:
            print_warn(f"Enter a port number (1-{len(ports)})" +
                       (" or 's' to skip" if allow_skip else ""))


def choose_test_set():
    """
    Ask the user which test set to run based on STEP/DIR jumper position.
    Returns 'A' or 'B'.
    """
    print_header("Test Set Selection")
    print("  The test is split into two sets based on the STEP/DIR jumper position:")
    print()
    print(f"    {_ansi('1;36', 'A')} - Jumpers in MCU position")
    print(f"        Section 1: Basic communications")
    print(f"        Section 2: Direct motor stepping (all 4 axes)")
    print()
    print(f"    {_ansi('1;36', 'B')} - Jumpers in TMC429 position (requires power cycle)")
    print(f"        Section 3: TMC429 velocity-mode motor test (all 4 axes)")
    print(f"        Section 4: Limit switch test")
    print(f"        Section 5: LED ring test")
    print(f"        Section 6: RS232 loopback test")
    print()

    while True:
        choice = input("  Select test set [A/B]: ").strip().upper()
        if choice in ("A", "B"):
            return choice
        print_warn("Enter 'A' or 'B'")


def select_ports(test_set):
    """Scan ports and let the user pick the Main MCU (and RS232 for set B)."""
    print_header("Serial Port Selection")
    print_info("Scanning for available serial ports...\n")

    ports = scan_ports()
    if not ports:
        print_fail("No serial ports detected. Is the board connected via USB?")
        sys.exit(1)

    print_port_table(ports)

    main_port = pick_port(ports, "Main MCU (ATSAMD51J19A)")

    rs232_port = None
    if test_set == "B":
        print()
        print_info("The RS232 test requires a USB-to-RS232 adapter connected to the PCB's")
        print_info("RS232 port, which appears as a separate serial port on this computer.")
        rs232_port = pick_port(ports, "RS232 adapter", allow_skip=True)
        if rs232_port is None:
            print_warn("RS232 test will fall back to manual verification.")

    print()
    return main_port, rs232_port


# =============================================================================
# Section runner
# =============================================================================

# Maps section name -> (callable, extra_args_needed)
# extra_args_needed is True for sections that need rs232_port
def _run_sections(sp, sections_to_run, rs232_port):
    """
    Run the given list of section names in order.
    Returns True if all passed, False on abort.
    """
    section_funcs = {
        "BASIC_COMMS":    lambda: test_basic_comms(sp),
        "MOTORS_DIRECT":  lambda: test_motors_direct(sp),
        "MOTORS_TMC429":  lambda: test_motors_tmc429(sp),
        "LIMIT_SWITCHES": lambda: test_limit_switches(sp),
        "LEDS":           lambda: test_leds(sp),
        "RS232":          lambda: test_rs232(sp, rs232_port),
    }

    for section in sections_to_run:
        fn = section_funcs.get(section)
        if fn is None:
            print_warn(f"Unknown section: {section}, skipping.")
            continue
        if not fn():
            print_fail(f"Aborting: {section} failed.")
            cleanup(sp)
            print_results_summary()
            return False

    return True


# =============================================================================
# Main Entry Point
# =============================================================================

def main():
    global _log

    parser = argparse.ArgumentParser(
        description="PSU v4 Interactive PCB Tester",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--continue", dest="continue_log", metavar="LOG_FILE", default=None,
        help="Resume testing from the first failure in a previous log file",
    )
    args = parser.parse_args()

    # Determine test set, resume point, and prior results
    resume_section = None
    prior_results = []

    if args.continue_log:
        test_set, resume_section, prior_results = parse_continue_log(args.continue_log)
        print_info(f"Test set {test_set} determined from log file.")
    else:
        test_set = choose_test_set()

    print_startup_checklist(test_set)
    main_port, rs232_port = select_ports(test_set)

    # Build log path
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = f"psu_test_{ts}_test_set_{test_set}.json"
    _log = TestLog(log_path)

    # Pre-populate with prior passing results when resuming
    for r in prior_results:
        _log.append(r)

    # Determine which sections to run
    if test_set == "A":
        all_sections = SECTIONS_A
    else:
        all_sections = SECTIONS_B

    if resume_section:
        # Start from the resume section onward
        idx = all_sections.index(resume_section)
        sections_to_run = all_sections[idx:]
        print_info(f"Will run sections: {', '.join(sections_to_run)}")
    else:
        sections_to_run = all_sections

    print_info(f"Log file: {log_path}  (written after every step)")
    wait_for_enter("Press [Enter] to begin testing...")

    sp = None
    try:
        print_info(f"Opening serial port: {main_port} at {BAUD_RATE} baud...")
        sp = SerialPort(main_port, BAUD_RATE, DEFAULT_TIMEOUT)
        print_pass(f"Serial port opened: {main_port}")

        # Wait a moment for the MCU boot banner, then flush it
        time.sleep(1.0)
        sp.flush_input()

        ok = _run_sections(sp, sections_to_run, rs232_port)

        cleanup(sp)
        print_results_summary()

        if ok and test_set == "A":
            print()
            print_info("Test Set A complete. To continue testing:")
            print_info("  1. Power off the board")
            print_info("  2. Move the STEP/DIR jumpers to the TMC429 position")
            print_info("  3. Power on and re-run this program, selecting Test Set B")

    except KeyboardInterrupt:
        print("\n")
        print_warn("Test interrupted by user (Ctrl+C).")
        if sp:
            cleanup(sp)
        print_results_summary()
        sys.exit(1)

    except serial.SerialException as e:
        print_fail(f"Serial port error: {e}")
        print_results_summary()
        sys.exit(1)

    finally:
        if sp:
            sp.close()


if __name__ == "__main__":
    main()
