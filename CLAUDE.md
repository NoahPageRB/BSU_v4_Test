# CLAUDE.md — Persistent Instructions for This Repo

## Versioning Rule (BLOCKING — applies to every change)

This repo has three independently versioned components. **Every change to a
component MUST bump that component's version.** No exceptions.

| Component | Where the version lives | Version macro/constant |
|---|---|---|
| Main MCU firmware (ATSAMD51J19A) | `ATSAMD51J19A/src/main.cpp` | `BSU_FW_VERSION_MAJOR/MINOR/PATCH` |
| LED MCU firmware (ATSAMD21E18A) | `ATSAMD21E18A/src/main.cpp` | `LED_FW_VERSION_MAJOR/MINOR/PATCH` |
| Python test suite | `bsu_test.py` (top of file) | `TEST_SUITE_VERSION` |

### Increment rules (semver)

- **MAJOR** — breaking changes (CLI command renamed/removed, I2C protocol byte
  layout changes, JSON schema breaks).
- **MINOR** — new feature, new command, new test step that adds capability
  without breaking existing behavior.
- **PATCH** — bug fix, doc change, refactor, label/log fix, no behavior change
  visible to existing callers.

When in doubt, prefer the higher bump.

### The cascading rule (CRITICAL)

**Any change to either MCU's firmware also requires a Python change**, because
the Python test suite has expected-version constants for both firmwares:

```python
EXPECTED_MAIN_FW_VERSION = "x.y.z"
EXPECTED_LED_FW_VERSION  = "x.y.z"
TEST_SUITE_VERSION       = "x.y.z"
```

If you change the Main MCU firmware:
1. Bump `BSU_FW_VERSION_*` in `ATSAMD51J19A/src/main.cpp`.
2. Bump `EXPECTED_MAIN_FW_VERSION` in `bsu_test.py` to match.
3. Bump `TEST_SUITE_VERSION` (Python file just changed).

If you change the LED MCU firmware:
1. Bump `LED_FW_VERSION_*` in `ATSAMD21E18A/src/main.cpp`.
2. Bump `EXPECTED_LED_FW_VERSION` in `bsu_test.py` to match.
3. Bump `TEST_SUITE_VERSION`.

**The reverse is NOT true** — Python-only changes (e.g., bug fix in test logic,
new section ordering) bump only `TEST_SUITE_VERSION` and require no firmware
changes.

### Version handshake

The Python tester runs `version` over serial during the BASIC_COMMS section.
The Main MCU prints its own firmware version, then queries the LED MCU over
I2C (`LED_CMD_GET_VERSION = 0x05`) and prints that too. Python parses both
and fails the run if either doesn't match the expected constants.

Reported and expected versions are written to the top of the run's JSON log
under the `versions` key, alongside the timestamp.

### Pre-commit checklist

Before committing any change:

- [ ] If you touched `ATSAMD51J19A/src/main.cpp`: did you bump
      `BSU_FW_VERSION_*` AND `EXPECTED_MAIN_FW_VERSION` AND `TEST_SUITE_VERSION`?
- [ ] If you touched `ATSAMD21E18A/src/main.cpp`: did you bump
      `LED_FW_VERSION_*` AND `EXPECTED_LED_FW_VERSION` AND `TEST_SUITE_VERSION`?
- [ ] If you only touched `bsu_test.py`: did you bump `TEST_SUITE_VERSION`?

Forgetting a bump means future test runs will report a stale version, or
worse — a passing run that's actually testing a different firmware than the
JSON log claims.
