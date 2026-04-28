#!/usr/bin/env python3
"""
BSU v4 Interactive PCB Tester

Guides a technician through the complete functional test procedure for BSU v4
PCBs. Communicates with the main MCU (ATSAMD51J19A) over USB serial at 115200
baud. Sends command sequences, verifies responses, prompts the user only when
physical action or visual confirmation is required, and produces a pass/fail
JSON log at the end.

Usage:
    python bsu_test.py

Requirements:
    pip install pyserial
"""

import argparse
import json
import re
import sys
import time
from datetime import datetime

import serial
import serial.tools.list_ports

# =============================================================================
# Constants
# =============================================================================

# =============================================================================
# Versions
# =============================================================================
#
# IMPORTANT: bump these on every change. Major = breaking, Minor = feature,
# Patch = bug fix. See CLAUDE.md at the repo root.
#
# Any change to either MCU's firmware MUST come with a Python change here:
# bump the matching EXPECTED_*_FW_VERSION to the new firmware version, AND
# bump TEST_SUITE_VERSION because the Python file just changed.
TEST_SUITE_VERSION       = "1.0.0"
EXPECTED_MAIN_FW_VERSION = "1.0.0"
EXPECTED_LED_FW_VERSION  = "1.0.0"

BAUD_RATE = 115200
DEFAULT_TIMEOUT = 2.0
LONG_TIMEOUT = 8.0          # For blocking commands like 'step'
POLL_INTERVAL = 0.25        # Seconds between io_read polls for user-triggered events

MOTOR_STEP_COUNT = 1600
MOTOR_STEP_DELAY_US = 2000

TMC429_VMIN = 100
TMC429_VMAX = 500
TMC429_AMAX = 500
TMC429_VTARGET = 500

# BSU has 3 independent motors; each has its own STEP/DIR/UART/ENABLE.
MOTORS = ["z1", "z2", "m3"]
MOTOR_LABELS = {"z1": "Z1", "z2": "Z2", "m3": "M3"}

# BSU limit switches (via EXTRA_IO_EXPANDER GPB0..GPB3).
# Z_TOP/Z_BOT are shared between the z1 and z2 TMC429 motors (both see them);
# SPARE_TOP/SPARE_BOT feed only the m3 TMC429 motor.
LIMIT_SWITCHES = ["Z_TOP", "Z_BOT", "SPARE_TOP", "SPARE_BOT"]

# io_read label substrings for each switch (matched against the output line).
SWITCH_IO_LABELS = {
    "Z_TOP":     "Z_TOP",
    "Z_BOT":     "Z_BOT",
    "SPARE_TOP": "SPARE_TOP",
    "SPARE_BOT": "SPARE_BOT",
}

# For each expander switch, which TMC429-motor-prefix(es) in mc_switches output
# should be expected to show ACTIVE when the switch is triggered.
SWITCH_MC_MOTOR_PREFIXES = {
    "Z_TOP":     ["z1", "z2"],
    "Z_BOT":     ["z1", "z2"],
    "SPARE_TOP": ["m3"],
    "SPARE_BOT": ["m3"],
}

# Solenoid count (matches firmware NUM_SOLENOIDS)
NUM_SOLENOIDS = 12

# Per-command timeout overrides (command prefix -> timeout in seconds)
COMMAND_TIMEOUTS = {
    "step": LONG_TIMEOUT,
    "help": 3.0,
    "led_config": 3.0,
    "mc_init": 4.0,
    "io_init": 3.0,
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
        return input()[0] if True else ""


def ask_user(prompt_text):
    """Prompt user with [y/n] and return True for y/Y. Single keypress, no timeout."""
    sys.stdout.write(f"{prompt_text} [y/n]: ")
    sys.stdout.flush()
    ch = getch()
    print(ch)
    return ch.lower() == "y"


def wait_for_enter(prompt_text="Press [Enter] to continue..."):
    """Wait for the user to press Enter. Never times out."""
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
        self.ser.reset_input_buffer()

    def send(self, cmd):
        self.flush_input()
        self.ser.write((cmd + "\n").encode("ascii"))
        self.ser.flush()

    def readline_timeout(self, timeout=None):
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

    # Prompt terminator. The firmware writes "> " with no trailing newline,
    # but it always emits a "\n" immediately before the prompt (the previous
    # command output ends with println). Matching on "\n> " avoids false
    # positives from usage strings like "tmc_init <z1|z2|m3> [baud]".
    _PROMPT_TERMINATOR = b"\n> "

    def read_until_prompt(self, timeout=None):
        """
        Read bytes until the firmware emits its '\\n> ' prompt or timeout.
        Returns lines collected before the prompt; the prompt itself is not
        included.

        Uses pyserial's read_until() so we don't pay a full readline()
        timeout waiting for a newline that never arrives at the prompt.
        """
        if timeout is None:
            timeout = self.timeout
        old_timeout = self.ser.timeout
        try:
            self.ser.timeout = timeout
            raw = self.ser.read_until(self._PROMPT_TERMINATOR)
        finally:
            self.ser.timeout = old_timeout
        text = raw.decode("ascii", errors="replace")
        # Drop the trailing "\n> " prompt if present.
        if text.endswith("\n> "):
            text = text[:-3]
        # Split into lines, stripping CR/LF and dropping empties.
        lines = [ln.rstrip("\r") for ln in text.split("\n")]
        return [ln for ln in lines if ln]

    def send_command(self, cmd, timeout=None):
        if timeout is None:
            cmd_prefix = cmd.split()[0] if cmd.strip() else ""
            timeout = COMMAND_TIMEOUTS.get(cmd_prefix, self.timeout)
        self.send(cmd)
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
        self._flush()

    def append(self, record):
        self.results.append(record)
        self._flush()

    def _flush(self):
        pass_count = sum(1 for r in self.results if r["status"] == "PASS")
        fail_count = sum(1 for r in self.results if r["status"] == "FAIL")
        skip_count = sum(1 for r in self.results if r["status"] == "SKIP")
        log_data = {
            "timestamp": datetime.now().isoformat(),
            "versions": {
                "test_suite":       TEST_SUITE_VERSION,
                "main_fw_expected": EXPECTED_MAIN_FW_VERSION,
                "led_fw_expected":  EXPECTED_LED_FW_VERSION,
                "main_fw_reported": _detected_main_fw_version,
                "led_fw_reported":  _detected_led_fw_version,
            },
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
            pass


_log: TestLog = None

# Populated during the BASIC_COMMS section's version-check step. Stays None
# until that step runs, in which case the JSON shows null for "reported"
# fields — useful when the comms test fails before version can be parsed.
_detected_main_fw_version: str = None
_detected_led_fw_version:  str = None


def expect_ok(lines, keyword=None):
    """Return True if no line contains '[ERROR]' and optional keyword is found."""
    for line in lines:
        if "[ERROR]" in line:
            return False
    if keyword:
        return any(keyword in line for line in lines)
    return True


def lines_str(lines):
    return "\n".join(lines)


def check_and_log(section, step_name, result, raw_lines, abort_on_fail=True):
    """Record a test step result. Returns True to continue, False to abort."""
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
    """Disable all motors, stop TMC429, drop the drawer LED, and clear solenoids."""
    print_info("Cleaning up: disabling motors, stopping TMC429, clearing outputs...")
    try:
        sp.send_command("mc_stopall", timeout=2.0)
    except Exception:
        pass
    for axis in MOTORS:
        try:
            sp.send_command(f"disable {axis}", timeout=2.0)
        except Exception:
            pass
    try:
        sp.send_command("drawer_led off", timeout=2.0)
    except Exception:
        pass
    try:
        sp.send_command("sol_all off", timeout=2.0)
    except Exception:
        pass
    try:
        sp.send_command("led_set all off 0 0 0 0", timeout=2.0)
    except Exception:
        pass


# =============================================================================
# Section 1: Basic Communications Test
# =============================================================================

_VERSION_RE_MAIN = re.compile(r"BSU Main MCU firmware:\s*v?([\d]+\.[\d]+\.[\d]+)")
_VERSION_RE_LED  = re.compile(r"LED MCU firmware:\s*v?([\d]+\.[\d]+\.[\d]+)")


def _parse_version_response(lines, regex):
    for ln in lines:
        m = regex.search(ln)
        if m:
            return m.group(1)
    return None


def test_basic_comms(sp):
    global _detected_main_fw_version, _detected_led_fw_version

    print_header("Section 1: Basic Communications Test")
    print_info("Sending 'help' command to verify MCU is responding...")

    lines = sp.send_command("help", timeout=3.0)
    ok = expect_ok(lines, keyword="tmc_init")
    if not check_and_log("BASIC_COMMS", "help_response", ok, lines):
        return False
    print_info("MCU is communicating. CLI help received successfully.")

    # ---- Firmware version check ----
    print_info(
        f"Verifying firmware versions (Main expected v{EXPECTED_MAIN_FW_VERSION}, "
        f"LED expected v{EXPECTED_LED_FW_VERSION}, suite v{TEST_SUITE_VERSION})..."
    )
    lines = sp.send_command("version", timeout=3.0)

    main_v = _parse_version_response(lines, _VERSION_RE_MAIN)
    _detected_main_fw_version = main_v
    main_ok = (main_v == EXPECTED_MAIN_FW_VERSION)
    if main_v is None:
        print_fail("Main MCU firmware version not found in response.")
    elif not main_ok:
        print_fail(
            f"Main MCU firmware mismatch: expected v{EXPECTED_MAIN_FW_VERSION}, "
            f"got v{main_v}. Bump EXPECTED_MAIN_FW_VERSION (and TEST_SUITE_VERSION) "
            "or reflash the MCU."
        )
    if not check_and_log(
        "BASIC_COMMS",
        f"main_fw_version_v{EXPECTED_MAIN_FW_VERSION}",
        main_ok,
        lines if main_v is None else [f"reported v{main_v}"],
    ):
        return False

    led_v = _parse_version_response(lines, _VERSION_RE_LED)
    _detected_led_fw_version = led_v
    led_ok = (led_v == EXPECTED_LED_FW_VERSION)
    if led_v is None:
        print_fail("LED MCU firmware version not found in response (LED MCU may be unflashed or unreachable on I2C).")
    elif not led_ok:
        print_fail(
            f"LED MCU firmware mismatch: expected v{EXPECTED_LED_FW_VERSION}, "
            f"got v{led_v}. Bump EXPECTED_LED_FW_VERSION (and TEST_SUITE_VERSION) "
            "or reflash the LED MCU."
        )
    if not check_and_log(
        "BASIC_COMMS",
        f"led_fw_version_v{EXPECTED_LED_FW_VERSION}",
        led_ok,
        lines if led_v is None else [f"reported v{led_v}"],
    ):
        return False

    print_pass(
        f"Versions OK: Main v{main_v}, LED v{led_v}, suite v{TEST_SUITE_VERSION}"
    )
    return True


# =============================================================================
# Section 2: Motor Direct-Step Test (MCU mode)
# =============================================================================

def _tmc_init_with_retry(sp, motor):
    """Send tmc_init for the given motor; retry once on failure."""
    for attempt in range(1, 3):
        lines = sp.send_command(f"tmc_init {motor}", timeout=3.0)
        if expect_ok(lines, keyword="Communication OK"):
            return True, lines
        if expect_ok(lines) and not any("not responding" in l for l in lines):
            if not any("WARNING" in l for l in lines):
                return True, lines
        if attempt == 1:
            print_warn(f"tmc_init {motor} did not ack on attempt 1, retrying...")
            time.sleep(0.5)

    return False, lines


def test_motors_direct(sp):
    print_header("Section 2: Motor Direct-Step Test (MCU Mode)")
    print_info("Ensure the STEP/DIR toggle switch is in the MCU position.")
    print_info("You will need a test stepper motor to move between headers.")

    # Expanders must be initialized so 'enable'/'disable' work (they go via
    # the EXTRA_IO_EXPANDER on BSU, not direct GPIO).
    lines = sp.send_command("io_init", timeout=3.0)
    ok = expect_ok(lines, keyword="EXTRA_IO_EXPANDER (0x21) initialized")
    if not check_and_log("MOTORS_DIRECT", "io_init", ok, lines):
        return False

    for motor in MOTORS:
        label = MOTOR_LABELS[motor]
        print_subheader(f"Motor {label}: Direct Step Test")

        wait_for_enter(f"Connect the test motor to the {label} port, then press [Enter]...")

        ok, lines = _tmc_init_with_retry(sp, motor)
        if not check_and_log("MOTORS_DIRECT", f"tmc_init_{motor}", ok, lines):
            return False

        lines = sp.send_command(f"tmc_microstep {motor} 8")
        ok = expect_ok(lines, keyword="microsteps set to")
        if not check_and_log("MOTORS_DIRECT", f"tmc_microstep_{motor}", ok, lines):
            return False

        lines = sp.send_command(f"tmc_current {motor} 50")
        ok = expect_ok(lines, keyword="run current set to")
        if not check_and_log("MOTORS_DIRECT", f"tmc_current_{motor}", ok, lines):
            return False

        lines = sp.send_command(f"tmc_enable {motor}")
        ok = expect_ok(lines, keyword="software-ENABLED")
        if not check_and_log("MOTORS_DIRECT", f"tmc_enable_{motor}", ok, lines):
            return False

        lines = sp.send_command(f"enable {motor}")
        ok = expect_ok(lines, keyword="ENABLED")
        if not check_and_log("MOTORS_DIRECT", f"enable_{motor}", ok, lines):
            return False

        print_info(f"Sending {MOTOR_STEP_COUNT} steps to {label} (this takes a few seconds)...")
        lines = sp.send_command(
            f"step {motor} {MOTOR_STEP_COUNT} {MOTOR_STEP_DELAY_US}",
            timeout=LONG_TIMEOUT,
        )
        ok = expect_ok(lines, keyword="done")
        if not check_and_log("MOTORS_DIRECT", f"step_{motor}", ok, lines):
            return False

        lines = sp.send_command(f"disable {motor}")
        ok = expect_ok(lines)
        if not check_and_log("MOTORS_DIRECT", f"disable_{motor}", ok, lines):
            return False

        user_ok = ask_user(f"Did motor {label} spin / make approximately one full rotation?")
        if not check_and_log("MOTORS_DIRECT", f"user_confirm_{motor}", user_ok, ["user response"]):
            return False

    print_pass("All direct motor stepping tests complete.")
    return True


# =============================================================================
# Section 3: Motor TMC429 Velocity-Mode Test
# =============================================================================

def test_motors_tmc429(sp):
    print_header("Section 3: Motor TMC429 Velocity-Mode Test")
    print_info("STEP/DIR toggle should already be in the TMC429 position.")

    lines = sp.send_command("io_init", timeout=3.0)
    ok = expect_ok(lines, keyword="EXTRA_IO_EXPANDER (0x21) initialized")
    if not check_and_log("MOTORS_TMC429", "io_init", ok, lines):
        return False

    print_info("Initializing TMC429 motion controller...")
    lines = sp.send_command("mc_init", timeout=4.0)
    ok = expect_ok(lines, keyword="Communication OK")
    if not check_and_log("MOTORS_TMC429", "mc_init", ok, lines):
        return False

    for motor in MOTORS:
        label = MOTOR_LABELS[motor]
        print_subheader(f"Motor {label}: TMC429 Velocity Test")

        wait_for_enter(f"Connect the test motor to the {label} port, then press [Enter]...")

        ok, lines = _tmc_init_with_retry(sp, motor)
        if not check_and_log("MOTORS_TMC429", f"tmc_init_{motor}", ok, lines):
            return False

        lines = sp.send_command(f"tmc_microstep {motor} 8")
        ok = expect_ok(lines, keyword="microsteps set to")
        if not check_and_log("MOTORS_TMC429", f"tmc_microstep_{motor}", ok, lines):
            return False

        lines = sp.send_command(f"tmc_current {motor} 50")
        ok = expect_ok(lines, keyword="run current set to")
        if not check_and_log("MOTORS_TMC429", f"tmc_current_{motor}", ok, lines):
            return False

        lines = sp.send_command(f"tmc_enable {motor}")
        ok = expect_ok(lines, keyword="software-ENABLED")
        if not check_and_log("MOTORS_TMC429", f"tmc_enable_{motor}", ok, lines):
            return False

        lines = sp.send_command(
            f"mc_limits {motor} {TMC429_VMIN} {TMC429_VMAX} {TMC429_AMAX}"
        )
        ok = expect_ok(lines, keyword="Setting limits")
        if not check_and_log("MOTORS_TMC429", f"mc_limits_{motor}", ok, lines):
            return False

        lines = sp.send_command(f"enable {motor}")
        ok = expect_ok(lines, keyword="ENABLED")
        if not check_and_log("MOTORS_TMC429", f"enable_{motor}", ok, lines):
            return False

        lines = sp.send_command(f"mc_velocity {motor}")
        ok = expect_ok(lines, keyword="VELOCITY mode")
        if not check_and_log("MOTORS_TMC429", f"mc_velocity_{motor}", ok, lines):
            return False

        lines = sp.send_command(f"mc_vtarget {motor} {TMC429_VTARGET}")
        ok = expect_ok(lines, keyword="target velocity set to")
        if not check_and_log("MOTORS_TMC429", f"mc_vtarget_{motor}", ok, lines):
            return False

        time.sleep(1.5)

        user_ok = ask_user(f"Is motor {label} moving?")
        if not check_and_log("MOTORS_TMC429", f"user_confirm_{motor}", user_ok, ["user response"]):
            return False

        sp.send_command(f"mc_stop {motor}")
        sp.send_command(f"disable {motor}")

    print_pass("All TMC429 velocity mode tests complete.")
    return True


# =============================================================================
# Section 4: Limit Switch Test
# =============================================================================

def _parse_io_read_switches(lines):
    """
    Parse 'io_read' output into a dict of switch states.
    Only the EXTRA_IO_EXPANDER limit-switch lines are parsed.
    Returns {"Z_TOP": "TRIGGERED"|"open", ...}.
    """
    states = {}
    in_switches = False
    for line in lines:
        if "--- Limit Switches ---" in line:
            in_switches = True
            continue
        if in_switches:
            # Stop parsing at the next section boundary
            if "===" in line or "---" in line:
                in_switches = False
                continue
        # Match by label regardless of section — the only lines containing these
        # labels are the switch lines themselves.
        for sw_name, label in SWITCH_IO_LABELS.items():
            if label in line and ("TRIGGERED" in line or "open" in line):
                states[sw_name] = "TRIGGERED" if "TRIGGERED" in line else "open"
    return states


def _parse_mc_switches(lines):
    """
    Parse mc_switches response into a flat list of (motor, side, active)
    tuples, e.g. [("z1", "Left", False), ("z1", "Right", False), ...].
    """
    out = []
    for line in lines:
        for motor in ("z1", "z2", "m3"):
            if f"{motor} Left" in line:
                out.append((motor, "Left", "ACTIVE" in line))
            elif f"{motor} Right" in line:
                out.append((motor, "Right", "ACTIVE" in line))
    return out


def _mc_any_motor_active(mc_list, motor_prefixes):
    """Return True if any motor in motor_prefixes has at least one ACTIVE side."""
    return any(active and motor in motor_prefixes for (motor, _side, active) in mc_list)


SWITCH_SETTLE_S = 1.0  # Debounce window after first detect before mc_switches


def _wait_for_stable_trigger(sp, sw_name, settle_s=SWITCH_SETTLE_S):
    """
    Poll io_read until `sw_name` reads TRIGGERED, then wait `settle_s` and
    re-check. If the switch is still TRIGGERED, return — caller can run
    mc_switches with confidence the jumper is settled. If it returned to
    open during the settle window, the user pulled too fast; loop back to
    polling instead of running mc_switches mid-jiggle.
    """
    while True:
        # Phase 1: poll until first TRIGGERED edge.
        while True:
            lines = sp.send_command("io_read")
            states = _parse_io_read_switches(lines)
            if states.get(sw_name) == "TRIGGERED":
                break
            time.sleep(POLL_INTERVAL)

        print_info(
            f"{sw_name} TRIGGERED — hold the jumper steady for {settle_s:.1f}s..."
        )
        time.sleep(settle_s)

        # Phase 2: confirm still triggered after settle.
        lines = sp.send_command("io_read")
        states = _parse_io_read_switches(lines)
        if states.get(sw_name) == "TRIGGERED":
            return

        print_warn(
            f"{sw_name} returned to open during settle — jumper bounced. "
            "Re-polling; press and hold the jumper steady."
        )


def test_limit_switches(sp):
    print_header("Section 4: Limit Switch Test")
    print_info("Disconnect the test motor from all headers before proceeding.")
    wait_for_enter("Press [Enter] when the motor is disconnected...")

    lines = sp.send_command("io_init", timeout=3.0)
    ok = expect_ok(lines, keyword="EXTRA_IO_EXPANDER (0x21) initialized")
    if not check_and_log("LIMIT_SWITCHES", "io_init", ok, lines):
        return False

    lines = sp.send_command("mc_swpol high")
    ok = expect_ok(lines, keyword="ACTIVE HIGH")
    if not check_and_log("LIMIT_SWITCHES", "mc_swpol_high", ok, lines):
        return False

    lines = sp.send_command("mc_rightsw on")
    ok = expect_ok(lines, keyword="ENABLED")
    if not check_and_log("LIMIT_SWITCHES", "mc_rightsw_on", ok, lines):
        return False

    # Baseline: all switches should be open
    lines = sp.send_command("io_read")
    states = _parse_io_read_switches(lines)
    all_open = all(states.get(sw) == "open" for sw in LIMIT_SWITCHES)
    if not check_and_log("LIMIT_SWITCHES", "baseline_io_read_all_open", all_open, lines):
        return False

    # Baseline: mc_switches should report no motor active
    lines = sp.send_command("mc_switches")
    mc_states = _parse_mc_switches(lines)
    all_inactive = not any(active for (_motor, _side, active) in mc_states)
    if not check_and_log("LIMIT_SWITCHES", "baseline_mc_switches_all_inactive",
                         all_inactive, lines):
        return False

    for sw_name in LIMIT_SWITCHES:
        print_subheader(f"Testing {sw_name}")
        print_info(
            f"Bridge the {sw_name} connector (center pin to the pin furthest from "
            "the board edge) to simulate a triggered switch."
        )
        print_info("Polling every 0.25s; take your time — no timeout.")

        # Debounce the user's jumper press: poll for TRIGGERED, then wait 1s
        # and re-check that the switch is still TRIGGERED. If it flipped back
        # to open during the settle window, the user pulled the jumper too
        # fast — go back to polling instead of running mc_switches mid-jiggle.
        _wait_for_stable_trigger(sp, sw_name)

        print_pass(f"{sw_name} held TRIGGERED through 1s settle — confirming with mc_switches")

        # Verify the TMC429 also sees at least one of the expected motor(s) as
        # ACTIVE on some side.
        lines = sp.send_command("mc_switches")
        mc_states = _parse_mc_switches(lines)
        expected_motors = SWITCH_MC_MOTOR_PREFIXES[sw_name]
        mc_ok = _mc_any_motor_active(mc_states, expected_motors)
        if not check_and_log("LIMIT_SWITCHES", f"{sw_name}_mc_confirm", mc_ok, lines):
            return False

        check_and_log("LIMIT_SWITCHES", f"{sw_name}_detect", True,
                      [f"{sw_name} TRIGGERED"])

        # Wait for jumper removal (no timeout)
        print_info(f"Remove the jumper from {sw_name}. Waiting for it to return to open...")
        while True:
            lines = sp.send_command("io_read")
            states = _parse_io_read_switches(lines)
            if states.get(sw_name) == "open":
                break
            time.sleep(POLL_INTERVAL)

        print_pass(f"{sw_name} returned to open")

    lines = sp.send_command("mc_switches")
    mc_states = _parse_mc_switches(lines)
    all_inactive = not any(active for (_motor, _side, active) in mc_states)
    if not check_and_log("LIMIT_SWITCHES", "final_mc_switches_all_inactive",
                         all_inactive, lines):
        return False

    print_pass("All limit switch tests complete.")
    return True


# =============================================================================
# Section 5: LED Ring Test
# =============================================================================

def test_leds(sp):
    print_header("Section 5: LED Ring Test")
    print_info("Ensure the LED ring harness is connected to the LED port on the PCB.")
    wait_for_enter("Press [Enter] when the LED harness is connected...")

    print_info("Sending LED spin animation (red, full brightness) via I2C...")
    lines = sp.send_command("led_set all spin 255 0 0 255")
    ok = expect_ok(lines, keyword="OK")
    if not check_and_log("LEDS", "led_set_spin_red", ok, lines):
        return False

    user_ok = ask_user("Are the LEDs showing a red spinning animation?")
    if not check_and_log("LEDS", "user_confirm_led_spin", user_ok, ["user response"]):
        return False

    sp.send_command("led_set all off 0 0 0 0")

    print_pass("LED tests complete.")
    return True


# =============================================================================
# Section 6: Drawer Handle Test
# =============================================================================

def _parse_drawer_button(lines):
    """Return 'PRESSED' or 'released' from the 'drawer_button' response, or None."""
    for line in lines:
        if "Button:" in line:
            if "PRESSED" in line:
                return "PRESSED"
            if "released" in line:
                return "released"
    return None


def test_drawer(sp):
    print_header("Section 6: Drawer Handle Test")
    print_info("This tests the drawer-handle button and the bi-color (R/G) LED.")
    wait_for_enter("Press [Enter] when the drawer-handle harness is connected...")

    # io_init is needed for the drawer button read (via EXTRA_IO_EXPANDER)
    lines = sp.send_command("io_init", timeout=3.0)
    ok = expect_ok(lines, keyword="EXTRA_IO_EXPANDER (0x21) initialized")
    if not check_and_log("DRAWER", "io_init", ok, lines):
        return False

    # --- Button test (press then release, polled, no timeout) ---
    print_subheader("Drawer button")
    print_info("Press and hold the drawer handle button.")

    while True:
        lines = sp.send_command("drawer_button")
        state = _parse_drawer_button(lines)
        if state == "PRESSED":
            break
        time.sleep(POLL_INTERVAL)
    check_and_log("DRAWER", "button_press_detected", True, ["user pressed button"])
    print_pass("Drawer button PRESSED detected.")

    print_info("Now release the drawer handle button.")
    while True:
        lines = sp.send_command("drawer_button")
        state = _parse_drawer_button(lines)
        if state == "released":
            break
        time.sleep(POLL_INTERVAL)
    check_and_log("DRAWER", "button_release_detected", True, ["user released button"])
    print_pass("Drawer button released detected.")

    # --- Red LED ---
    print_subheader("Drawer LED: RED")
    lines = sp.send_command("drawer_led red")
    ok = expect_ok(lines, keyword="RED")
    if not check_and_log("DRAWER", "drawer_led_red_cmd", ok, lines):
        return False
    user_ok = ask_user("Is the drawer-handle LED RED?")
    sp.send_command("drawer_led off")
    if not check_and_log("DRAWER", "user_confirm_red", user_ok, ["user response"]):
        return False

    # --- Green LED ---
    print_subheader("Drawer LED: GREEN")
    lines = sp.send_command("drawer_led green")
    ok = expect_ok(lines, keyword="GREEN")
    if not check_and_log("DRAWER", "drawer_led_green_cmd", ok, lines):
        return False
    user_ok = ask_user("Is the drawer-handle LED GREEN?")
    sp.send_command("drawer_led off")
    if not check_and_log("DRAWER", "user_confirm_green", user_ok, ["user response"]):
        return False

    print_pass("Drawer handle tests complete.")
    return True


# =============================================================================
# Section 7: Solenoid Test
# =============================================================================

def test_solenoids(sp):
    print_header("Section 7: Solenoid Test")
    print_info("Each of the 12 solenoid ports is tested individually.")
    print_info("Order: Tray 1 valves 1-4, then Tray 2 valves 1-4, then Tray 3 valves 1-4.")
    print_info("You'll be prompted to move a test solenoid between ports.")
    wait_for_enter("Press [Enter] when ready...")

    lines = sp.send_command("io_init", timeout=3.0)
    ok = expect_ok(lines, keyword="TRAY_IO_EXPANDER (0x20) initialized")
    if not check_and_log("SOLENOIDS", "io_init", ok, lines):
        return False

    # Safety: explicitly drive all off first
    sp.send_command("sol_all off")

    for idx in range(NUM_SOLENOIDS):
        tray  = (idx // 4) + 1
        valve = (idx % 4) + 1
        label = f"T{tray}V{valve}"
        print_subheader(f"Solenoid {label} (sol {idx}) — Tray {tray}, Valve {valve}")
        wait_for_enter(f"Connect the test solenoid to port {label}, then press [Enter]...")

        lines = sp.send_command(f"sol {idx} on")
        ok = expect_ok(lines, keyword=f"SOL{idx}")
        if not check_and_log("SOLENOIDS", f"sol_{idx}_{label}_on_cmd", ok, lines):
            return False

        user_ok = ask_user(f"Did solenoid {label} fire (click / actuate)?")
        sp.send_command(f"sol {idx} off")
        if not check_and_log("SOLENOIDS", f"user_confirm_{label}", user_ok,
                             ["user response"]):
            return False

    print_pass("All 12 solenoid tests complete (T1V1..T3V4).")
    return True


# =============================================================================
# Results Summary & JSON Log
# =============================================================================

def print_results_summary():
    print_header("Test Results Summary")

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
    print_header("BSU v4 PCB Tester - Startup Checklist")
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
        print("    8. Drawer-handle harness is available (button + bi-color LED)")
        print("    9. A test solenoid is available (to be moved between 12 ports)")
        print("   10. STEP/DIR toggle switch is in the TMC429 position")
    print()


# =============================================================================
# --continue: Resume from a previous log's first failure
# =============================================================================

SECTIONS_A = ["BASIC_COMMS", "MOTORS_DIRECT"]
SECTIONS_B = ["BASIC_COMMS", "MOTORS_TMC429", "LIMIT_SWITCHES", "LEDS", "DRAWER", "SOLENOIDS"]


def parse_continue_log(log_path):
    """
    Read a previous test log JSON, find the first FAIL entry, and return:
      (test_set, resume_section, prior_results)
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

    fail_section = None
    for r in results:
        if r["status"] == "FAIL":
            fail_section = r["section"]
            break

    if fail_section is None:
        print_warn("No failures found in the log file. Nothing to resume.")
        sys.exit(0)

    if fail_section in SECTIONS_A:
        test_set = "A"
        section_order = SECTIONS_A
    elif fail_section in SECTIONS_B:
        test_set = "B"
        section_order = SECTIONS_B
    else:
        print_fail(f"Unknown section '{fail_section}' in log.")
        sys.exit(1)

    resume_idx = section_order.index(fail_section)
    sections_to_keep = set(section_order[:resume_idx])
    prior_results = [r for r in results if r["section"] in sections_to_keep
                     and r["status"] == "PASS"]

    print_info(f"Resuming from section: {fail_section}")
    if prior_results:
        print_info(f"Carrying forward {len(prior_results)} passing results from earlier sections.")

    return test_set, fail_section, prior_results


# =============================================================================
# Interactive Port Selection
# =============================================================================

def scan_ports():
    return sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)


def print_port_table(ports):
    if not ports:
        print_fail("No serial ports found!")
        return

    dev_w = max(len(p.device) for p in ports)
    dev_w = max(dev_w, len("Device"))
    desc_w = max((len(p.description) for p in ports), default=11)
    desc_w = max(min(desc_w, 50), len("Description"))

    print(f"  {'#':<4} {'Device':<{dev_w}}  {'Description':<{desc_w}}  {'Hardware ID'}")
    print(f"  {'─'*4} {'─'*dev_w}  {'─'*desc_w}  {'─'*30}")

    for i, p in enumerate(ports):
        desc = p.description[:50] if p.description else "n/a"
        hwid = p.hwid if p.hwid else "n/a"
        marker = ""
        desc_lower = (p.description or "").lower()
        if "metro" in desc_lower or "m4" in desc_lower:
            marker = _ansi("1;32", " <-- likely Main MCU")
        elif "trinket" in desc_lower or "m0" in desc_lower:
            marker = _ansi("1;33", " <-- likely LED MCU")
        elif ("usbserial" in desc_lower or "usb-serial" in desc_lower or
              "ft232" in desc_lower or "ch340" in desc_lower or
              "cp210" in desc_lower or "pl2303" in desc_lower):
            marker = _ansi("1;36", " <-- likely USB-Serial adapter")
        print(f"  {i+1:<4} {p.device:<{dev_w}}  {desc:<{desc_w}}  {hwid}{marker}")

    print()


def pick_port(ports, role):
    while True:
        choice = input(f"  Select port # for {role}: ").strip().lower()
        try:
            idx = int(choice) - 1
            if 0 <= idx < len(ports):
                print_info(f"Selected {ports[idx].device} for {role}")
                return ports[idx].device
            else:
                print_warn(f"Enter a number between 1 and {len(ports)}")
        except ValueError:
            print_warn(f"Enter a port number (1-{len(ports)})")


def choose_test_set():
    print_header("Test Set Selection")
    print("  The test is split into two sets based on the STEP/DIR toggle position:")
    print()
    print(f"    {_ansi('1;36', 'A')} - Toggle in MCU position")
    print(f"        Section 1: Basic communications")
    print(f"        Section 2: Direct motor stepping (z1, z2, m3)")
    print()
    print(f"    {_ansi('1;36', 'B')} - Toggle in TMC429 position (requires power cycle)")
    print(f"        Section 3: TMC429 velocity-mode motor test (z1, z2, m3)")
    print(f"        Section 4: Limit switch test (Z_TOP, Z_BOT, SPARE_TOP, SPARE_BOT)")
    print(f"        Section 5: LED ring test")
    print(f"        Section 6: Drawer handle test (button + R/G LED)")
    print(f"        Section 7: Solenoid test (12 solenoids, one at a time)")
    print()

    while True:
        choice = input("  Select test set [A/B]: ").strip().upper()
        if choice in ("A", "B"):
            return choice
        print_warn("Enter 'A' or 'B'")


def select_port():
    print_header("Serial Port Selection")
    print_info("Scanning for available serial ports...\n")

    ports = scan_ports()
    if not ports:
        print_fail("No serial ports detected. Is the board connected via USB?")
        sys.exit(1)

    print_port_table(ports)

    main_port = pick_port(ports, "Main MCU (ATSAMD51J19A)")
    print()
    return main_port


# =============================================================================
# Section runner
# =============================================================================

def _run_sections(sp, sections_to_run):
    section_funcs = {
        "BASIC_COMMS":    lambda: test_basic_comms(sp),
        "MOTORS_DIRECT":  lambda: test_motors_direct(sp),
        "MOTORS_TMC429":  lambda: test_motors_tmc429(sp),
        "LIMIT_SWITCHES": lambda: test_limit_switches(sp),
        "LEDS":           lambda: test_leds(sp),
        "DRAWER":         lambda: test_drawer(sp),
        "SOLENOIDS":      lambda: test_solenoids(sp),
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
        description="BSU v4 Interactive PCB Tester",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--continue", dest="continue_log", metavar="LOG_FILE", default=None,
        help="Resume testing from the first failure in a previous log file",
    )
    args = parser.parse_args()

    resume_section = None
    prior_results = []

    if args.continue_log:
        test_set, resume_section, prior_results = parse_continue_log(args.continue_log)
        print_info(f"Test set {test_set} determined from log file.")
    else:
        test_set = choose_test_set()

    print_startup_checklist(test_set)
    main_port = select_port()

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = f"bsu_test_{ts}_test_set_{test_set}.json"
    _log = TestLog(log_path)

    for r in prior_results:
        _log.append(r)

    if test_set == "A":
        all_sections = SECTIONS_A
    else:
        all_sections = SECTIONS_B

    if resume_section:
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

        time.sleep(1.0)
        sp.flush_input()

        ok = _run_sections(sp, sections_to_run)

        cleanup(sp)
        print_results_summary()

        if ok and test_set == "A":
            print()
            print_info("Test Set A complete. To continue testing:")
            print_info("  1. Power off the board")
            print_info("  2. Move the STEP/DIR toggle to the TMC429 position")
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
