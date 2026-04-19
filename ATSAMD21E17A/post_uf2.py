"""
PlatformIO post-configuration script for the ATSAMD21E17A board.

Fixes three issues with using the stock adafruit_trinket_m0 board definition
on the E17A (128KB flash / 16KB RAM) with a UF2 bootloader:

  1. Linker script: Swaps the stock E18A linker script (32KB RAM / 256KB flash)
     for our E17A version, so the stack pointer and memory layout are correct.

  2. bossac flags: PlatformIO's SAMD21 code path passes `-U true` as two args
     (bossac 1.9 chokes: "extra arguments found") and omits `--offset 0x2000`
     (only SAMD51 gets it). We fix both.

  3. UF2 generation: Converts firmware.bin to firmware.uf2 after every build
     so you can drag-and-drop onto the UF2 drive as an alternative to bossac.
"""

import os
import subprocess
Import("env")  # noqa: F821 — PlatformIO SCons builtin

PROJECT_DIR      = env.subst("$PROJECT_DIR")
SAMD21_FAMILY_ID = "0x68ed2b88"
APP_BASE_ADDR    = "0x2000"

# =============================================================================
# 1. Replace linker script with E17A version (16KB RAM, 128KB flash)
# =============================================================================
# By the time this post-script runs, PlatformIO has already prepended
# `-T flash_with_bootloader.ld` to LINKFLAGS. We swap in our custom script.

custom_ld = os.path.join(PROJECT_DIR, "linker", "flash_with_bootloader_e17a.ld")
linkflags = env.get("LINKFLAGS", [])
new_linkflags = []
skip_next = False

for i, flag in enumerate(linkflags):
    if skip_next:
        skip_next = False
        continue
    f = str(flag)
    if f == "-T" and i + 1 < len(linkflags):
        # Replace with our custom linker script
        new_linkflags.append("-T")
        new_linkflags.append(custom_ld)
        skip_next = True  # skip the original .ld path
    else:
        new_linkflags.append(flag)

env.Replace(LINKFLAGS=new_linkflags)

# =============================================================================
# 2. Fix bossac upload flags
# =============================================================================
# PlatformIO's SAMD21 path (builder/main.py:226-231) produces:
#     ["--erase", "-U", "true"]          ← broken: -U and true are split
# and omits --offset (only added for SAMD51).
#
# We need:
#     ["--erase", "-U", "--offset", "0x2000"]

flags = [str(f) for f in env.get("UPLOADERFLAGS", [])]
fixed_flags = []
skip_next = False

for i, f in enumerate(flags):
    if skip_next:
        skip_next = False
        continue
    if f == "-U" and i + 1 < len(flags) and flags[i + 1] in ("true", "false"):
        fixed_flags.append("-U")   # standalone -U (defaults to USB detection)
        skip_next = True           # drop the stray "true"/"false"
    else:
        fixed_flags.append(f)

if "--offset" not in fixed_flags:
    fixed_flags.extend(["--offset", APP_BASE_ADDR])

env.Replace(UPLOADERFLAGS=fixed_flags)

# =============================================================================
# 3. Generate .uf2 after build
# =============================================================================

def generate_uf2(source, target, env):
    build_dir = env.subst("$BUILD_DIR")
    prog_name = env.subst("$PROGNAME")
    bin_path  = os.path.join(build_dir, prog_name + ".bin")
    uf2_path  = os.path.join(build_dir, prog_name + ".uf2")
    script    = os.path.join(PROJECT_DIR, "uf2conv.py")

    cmd = [
        "python3", script,
        "--base",    APP_BASE_ADDR,
        "--family",  SAMD21_FAMILY_ID,
        "--convert",
        "--output",  uf2_path,
        bin_path,
    ]
    print(f"Generating UF2: {uf2_path}")
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print("uf2conv error:", result.stderr)
    else:
        size = os.path.getsize(uf2_path)
        print(f"UF2 ready: {os.path.basename(uf2_path)} ({size} bytes)")


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", generate_uf2)
