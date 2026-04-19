// ============================================================================
// Motor & Sensor Test CLI for Adafruit Metro M4 Express
// ============================================================================
//
// Provides a Serial CLI (115200 baud over USB) for interactively testing:
//   - 4x TMC2209 stepper motor drivers (X, Y, Z1, Z2)
//   - MCP23017 I/O expander (limit switches + TMC2209 DIAG pins)
//
// Physical motor mapping:
//   X  — independent axis (own STEP, DIR, ENABLE, UART)
//   Y  — independent axis (own STEP, DIR, ENABLE, UART)
//   Z1 — shares STEP/DIR/ENABLE with Z2, but has own UART
//   Z2 — shares STEP/DIR/ENABLE with Z1, but has own UART
//
// UART wiring note:
//   Each TMC2209 uses a single-wire UART interface. The MCU TX is connected
//   through a 1k resistor to the TMC2209 PDN_UART pin, and the MCU RX is
//   connected directly to the same line. This causes TX echoes on RX, which
//   the janelia-arduino/TMC2209 library handles internally.
//
// SERCOM assignments (Metro M4 / ATSAMD51J19A):
//   X  UART -> SERCOM4  (TX=D7/PB12 PAD[0], RX=D4/PB13 PAD[1])
//   Y  UART -> SERCOM3  (TX=D1/PA22 PAD[0], RX=D0/PA23 PAD[1])  = Serial1
//   Z1 UART -> SERCOM1  (TX=D13/PA16 PAD[0], RX=D12/PA17 PAD[1])
//   Z2 UART -> SERCOM5  (TX=D3/PB16 PAD[0], RX=D2/PB17 PAD[1])
//
// SERCOM5 CONFLICT NOTE:
//   The Metro M4 board support package assigns SERCOM5 to Wire (I2C) on the
//   SDA/SCL header pins (PB2/PB3). The Z2 motor UART also needs SERCOM5 on
//   different physical pins (PB16/PB17). Since a SERCOM can only operate in
//   one mode at a time, and the Wire library's interrupt handlers are non-weak
//   symbols that cannot be overridden, this firmware uses a bitbang I2C
//   implementation (SoftI2C) for the MCP23017. This frees SERCOM5 entirely
//   for the Z2 UART. The MCP23017 is only used for low-speed input reads,
//   so bitbang I2C at ~100 kHz is perfectly adequate.
//
// ============================================================================

#include <Arduino.h>
#include <SPI.h>
#include <TMC429.h>
#include <TMC2209.h>
#include "wiring_private.h" // Required for pinPeripheral()
#include "soft_i2c.h"       // Bitbang I2C (avoids SERCOM5 conflict with Wire)

// ============================================================================
// Pin Definitions
// ============================================================================

// Motor STEP and DIR pins (active-high step pulse, direction level)
static constexpr uint8_t PIN_X_DIR  = A0;
static constexpr uint8_t PIN_X_STEP = A1;
static constexpr uint8_t PIN_Y_DIR  = A2;
static constexpr uint8_t PIN_Y_STEP = A3;
static constexpr uint8_t PIN_Z_DIR  = A4;  // Shared by Z1 and Z2
static constexpr uint8_t PIN_Z_STEP = A5;  // Shared by Z1 and Z2

// Motor driver nEnable pins (active LOW: LOW = enabled, HIGH = disabled)
static constexpr uint8_t PIN_X_NENABLE = 10;
static constexpr uint8_t PIN_Y_NENABLE = 9;
static constexpr uint8_t PIN_Z_NENABLE = 8;  // Shared by Z1 and Z2

// I/O expander nRESET (active LOW, externally pulled HIGH through 10k)
static constexpr uint8_t PIN_IO_NRESET = 6;

// TMC429 motion controller SPI chip select (active LOW)
// SPI bus uses default Metro M4 SPI pins: MOSI, MISO, SCK (SERCOM2)
static constexpr uint8_t PIN_MC_NCS = 5;

// I2C pins (driven by SoftI2C, NOT the Wire library)
static constexpr uint8_t PIN_I2C_SDA = SDA; // PB2 on Metro M4
static constexpr uint8_t PIN_I2C_SCL = SCL; // PB3 on Metro M4

// UART pins (Uart objects manage these; listed for documentation)
static constexpr uint8_t PIN_X_UART_TX  = 7;   // PB12 SERCOM4 PAD[0]
static constexpr uint8_t PIN_X_UART_RX  = 4;   // PB13 SERCOM4 PAD[1]
// Y uses Serial1: TX=D1 (PA22 SERCOM3 PAD[0]), RX=D0 (PA23 SERCOM3 PAD[1])
static constexpr uint8_t PIN_Z1_UART_TX = 13;  // PA16 SERCOM1 PAD[0]
static constexpr uint8_t PIN_Z1_UART_RX = 12;  // PA17 SERCOM1 PAD[1]
static constexpr uint8_t PIN_Z2_UART_TX = 3;   // PB16 SERCOM5 PAD[0]
static constexpr uint8_t PIN_Z2_UART_RX = 2;   // PB17 SERCOM5 PAD[1]

// ============================================================================
// MCP23017 I/O Expander Configuration
// ============================================================================

// I2C address: A2=A1=A0=GND -> base address 0x20
static constexpr uint8_t IO_EXPANDER_ADDR = 0x20;

// MCP23017 register addresses (IOCON.BANK=0, the power-on default)
static constexpr uint8_t MCP_REG_IODIRA = 0x00; // Port A direction (1=input)
static constexpr uint8_t MCP_REG_IODIRB = 0x01; // Port B direction (1=input)
static constexpr uint8_t MCP_REG_GPIOA  = 0x12; // Port A input values
static constexpr uint8_t MCP_REG_GPIOB  = 0x13; // Port B input values

// GPA bit positions within GPIOA register
static constexpr uint8_t BIT_X_DIAG  = 0; // GPA0
static constexpr uint8_t BIT_Y_DIAG  = 1; // GPA1
static constexpr uint8_t BIT_Z1_DIAG = 2; // GPA2
static constexpr uint8_t BIT_Z2_DIAG = 3; // GPA3

// GPB bit positions within GPIOB register
static constexpr uint8_t BIT_X_END1  = 2; // GPB2
static constexpr uint8_t BIT_X_END2  = 3; // GPB3
static constexpr uint8_t BIT_Y_END1  = 4; // GPB4
static constexpr uint8_t BIT_Y_END2  = 5; // GPB5
static constexpr uint8_t BIT_Z_END1  = 6; // GPB6
static constexpr uint8_t BIT_Z_END2  = 7; // GPB7

// ============================================================================
// LED MCU I2C Protocol Constants
// ============================================================================

static constexpr uint8_t LED_MCU_ADDR       = 0x30;
// Commands (master writes)
static constexpr uint8_t LED_CMD_SET_ANIM   = 0x01;
static constexpr uint8_t LED_CMD_GET_STATUS = 0x02;
static constexpr uint8_t LED_CMD_SET_CONFIG = 0x03;
static constexpr uint8_t LED_CMD_GET_CONFIG = 0x04;
// Response codes
static constexpr uint8_t LED_RSP_OK         = 0xA0;
static constexpr uint8_t LED_RSP_BAD_CMD    = 0xE1;
static constexpr uint8_t LED_RSP_BAD_FMT   = 0xE2;
// Animation types
static constexpr uint8_t LED_ANIM_OFF       = 0;
static constexpr uint8_t LED_ANIM_STATIC    = 1;
static constexpr uint8_t LED_ANIM_SPIN      = 2;
static constexpr uint8_t LED_ANIM_BOUNCE    = 3;
static constexpr uint8_t LED_ANIM_PULSE     = 4;
// Config parameter IDs
static constexpr uint8_t LED_CFG_NUM_RINGS     = 0x01;
static constexpr uint8_t LED_CFG_LEDS_PER_RING = 0x02;
static constexpr uint8_t LED_CFG_SPIN_WIDTH    = 0x03;
static constexpr uint8_t LED_CFG_BOUNCE_WIDTH  = 0x04;
static constexpr uint8_t LED_CFG_SPIN_SPEED    = 0x05;
static constexpr uint8_t LED_CFG_BOUNCE_SPEED  = 0x06;
static constexpr uint8_t LED_CFG_PULSE_SPEED   = 0x07;

// ============================================================================
// Custom SERCOM UART Instances
// ============================================================================
//
// Y Motor uses Serial1 (SERCOM3, TX=D1/PA22, RX=D0/PA23) from the BSP.
//
// X, Z1, Z2 need custom Uart objects. We exclude Wire (lib_ignore in
// platformio.ini) to free SERCOM5's interrupt handlers for Z2.

// X Motor UART: SERCOM4 (TX=PB12/PAD[0], RX=PB13/PAD[1])
static Uart XMotorSerial(&sercom4,
                          PIN_X_UART_RX, PIN_X_UART_TX,
                          SERCOM_RX_PAD_1, UART_TX_PAD_0);

// Z1 Motor UART: SERCOM1 (TX=PA16/PAD[0], RX=PA17/PAD[1])
static Uart Z1MotorSerial(&sercom1,
                           PIN_Z1_UART_RX, PIN_Z1_UART_TX,
                           SERCOM_RX_PAD_1, UART_TX_PAD_0);

// Z2 Motor UART: SERCOM5 (TX=PB16/PAD[0], RX=PB17/PAD[1])
static Uart Z2MotorSerial(&sercom5,
                           PIN_Z2_UART_RX, PIN_Z2_UART_TX,
                           SERCOM_RX_PAD_1, UART_TX_PAD_0);

// --- SERCOM interrupt handlers -----------------------------------------------
// SAMD51 has 4 interrupt vectors per SERCOM.
// SERCOM3 handlers are provided by the BSP for Serial1 — do NOT redefine.
// SERCOM5 handlers are normally provided by Wire.cpp, but we excluded Wire
// via lib_ignore so we can define them here for Z2 UART.

extern "C" {

void SERCOM4_0_Handler() { XMotorSerial.IrqHandler(); }
void SERCOM4_1_Handler() { XMotorSerial.IrqHandler(); }
void SERCOM4_2_Handler() { XMotorSerial.IrqHandler(); }
void SERCOM4_3_Handler() { XMotorSerial.IrqHandler(); }

void SERCOM1_0_Handler() { Z1MotorSerial.IrqHandler(); }
void SERCOM1_1_Handler() { Z1MotorSerial.IrqHandler(); }
void SERCOM1_2_Handler() { Z1MotorSerial.IrqHandler(); }
void SERCOM1_3_Handler() { Z1MotorSerial.IrqHandler(); }

void SERCOM5_0_Handler() { Z2MotorSerial.IrqHandler(); }
void SERCOM5_1_Handler() { Z2MotorSerial.IrqHandler(); }
void SERCOM5_2_Handler() { Z2MotorSerial.IrqHandler(); }
void SERCOM5_3_Handler() { Z2MotorSerial.IrqHandler(); }

} // extern "C"

// ============================================================================
// TMC2209 Driver Instances
// ============================================================================

static TMC2209 tmcX;
static TMC2209 tmcY;
static TMC2209 tmcZ1;
static TMC2209 tmcZ2;

static bool tmcXInitialized  = false;
static bool tmcYInitialized  = false;
static bool tmcZ1Initialized = false;
static bool tmcZ2Initialized = false;

// ============================================================================
// TMC429 Motion Controller Instance
// ============================================================================
// Motor index mapping: X=0 (M1), Y=1 (M2), Z=2 (M3)

static TMC429 motionController;
static bool mcInitialized = false;

// ============================================================================
// I2C and I/O Expander
// ============================================================================

static SoftI2C i2c(PIN_I2C_SDA, PIN_I2C_SCL);
static bool ioExpanderInitialized = false;

// ============================================================================
// CLI Infrastructure
// ============================================================================

static constexpr size_t CMD_BUF_SIZE = 128;
static char cmdBuffer[CMD_BUF_SIZE];
static size_t cmdIndex = 0;
static constexpr size_t MAX_TOKENS = 8;

// ============================================================================
// Forward Declarations
// ============================================================================

static void processCommand(char *line);
static void cmdHelp();
static void cmdIoInit();
static void cmdIoReset();
static void cmdIoRead();
static void cmdMcuRead();
static void cmdEnable(const char *axis);
static void cmdDisable(const char *axis);
static void cmdTmcInit(const char *motor, const char *baudStr);
static void cmdTmcStatus(const char *motor);
static void cmdTmcCurrent(const char *motor, const char *pctStr);
static void cmdTmcHold(const char *motor, const char *pctStr);
static void cmdTmcMicrostep(const char *motor, const char *msStr);
static void cmdTmcStealthchop(const char *motor, const char *state);
static void cmdTmcEnable(const char *motor);
static void cmdTmcDisable(const char *motor);
static void cmdDir(const char *axis, const char *valStr);
static void cmdStep(const char *axis, const char *countStr, const char *delayStr);
// TMC429 motion controller commands
static void cmdMcInit(const char *clkStr);
static void cmdMcStatus();
static void cmdMcLimits(const char *axis, const char *vminStr, const char *vmaxStr, const char *amaxStr);
static void cmdMcRamp(const char *axis);
static void cmdMcVelocity(const char *axis);
static void cmdMcHold(const char *axis);
static void cmdMcTarget(const char *axis, const char *posStr);
static void cmdMcVtarget(const char *axis, const char *velStr);
static void cmdMcPos(const char *axis);
static void cmdMcSetpos(const char *axis, const char *posStr);
static void cmdMcStop(const char *axis);
static void cmdMcStopall();
static void cmdMcSwitches();
static void cmdMcSwpol(const char *state);
static void cmdMcLeftstop(const char *axis, const char *state);
static void cmdMcRightstop(const char *axis, const char *state);
static void cmdMcRightsw(const char *state);
static int  lookupMcMotor(const char *axis); // returns 0/1/2 or -1
// LED MCU commands
static void cmdLedSet(char *tokens[], size_t count);
static void cmdLedStatus(const char *ringStr);
static void cmdLedConfig(const char *param, const char *valStr);
static void cmdLedGetconfig();
static TMC2209 *lookupTmc(const char *motor);
static bool     isTmcInitialized(const char *motor);
static void     getAxisPins(const char *axis, uint8_t &stepPin, uint8_t &dirPin,
                            uint8_t &enablePin, bool &valid);

// ============================================================================
// setup()
// ============================================================================

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    Serial.println();
    Serial.println(F("============================================================"));
    Serial.println(F("  Motor & Sensor Test CLI"));
    Serial.println(F("  Board: Adafruit Metro M4 Express (ATSAMD51J19A)"));
    Serial.println(F("============================================================"));

    // Motor STEP pins (default LOW)
    pinMode(PIN_X_STEP, OUTPUT); digitalWrite(PIN_X_STEP, LOW);
    pinMode(PIN_Y_STEP, OUTPUT); digitalWrite(PIN_Y_STEP, LOW);
    pinMode(PIN_Z_STEP, OUTPUT); digitalWrite(PIN_Z_STEP, LOW);

    // Motor DIR pins (default LOW)
    pinMode(PIN_X_DIR, OUTPUT); digitalWrite(PIN_X_DIR, LOW);
    pinMode(PIN_Y_DIR, OUTPUT); digitalWrite(PIN_Y_DIR, LOW);
    pinMode(PIN_Z_DIR, OUTPUT); digitalWrite(PIN_Z_DIR, LOW);

    // Motor nEnable pins: HIGH = disabled (safe default)
    pinMode(PIN_X_NENABLE, OUTPUT); digitalWrite(PIN_X_NENABLE, HIGH);
    pinMode(PIN_Y_NENABLE, OUTPUT); digitalWrite(PIN_Y_NENABLE, HIGH);
    pinMode(PIN_Z_NENABLE, OUTPUT); digitalWrite(PIN_Z_NENABLE, HIGH);

    // I/O expander nRESET: start as input (external pull-up keeps it HIGH)
    pinMode(PIN_IO_NRESET, INPUT);

    Serial.println(F("[BOOT] GPIO pins configured."));
    Serial.println(F("[BOOT]   STEP/DIR pins: LOW"));
    Serial.println(F("[BOOT]   nEnable pins:  HIGH (all motors DISABLED)"));

    // Initialize bitbang I2C and MCP23017
    Serial.println(F("[BOOT] Initializing bitbang I2C on SDA/SCL pins..."));
    Serial.println(F("[BOOT]   NOTE: Wire library excluded. SERCOM5 reserved for Z2 UART."));
    i2c.begin();
    cmdIoInit();

    Serial.println();
    Serial.println(F("Type 'help' for a list of commands."));
    Serial.print(F("> "));
}

// ============================================================================
// loop()
// ============================================================================

void loop() {
    while (Serial.available()) {
        char c = (char)Serial.read();

        if (c == '\n' || c == '\r') {
            if (cmdIndex > 0) {
                cmdBuffer[cmdIndex] = '\0';
                Serial.println();
                processCommand(cmdBuffer);
                cmdIndex = 0;
                Serial.print(F("> "));
            }
        } else if (c == '\b' || c == 127) {
            if (cmdIndex > 0) {
                cmdIndex--;
                Serial.print(F("\b \b"));
            }
        } else if (cmdIndex < CMD_BUF_SIZE - 1) {
            cmdBuffer[cmdIndex++] = c;
            Serial.print(c);
        }
    }
}

// ============================================================================
// Command Dispatcher
// ============================================================================

static void processCommand(char *line) {
    char *tokens[MAX_TOKENS] = {};
    size_t tokenCount = 0;
    char *tok = strtok(line, " \t");
    while (tok && tokenCount < MAX_TOKENS) {
        tokens[tokenCount++] = tok;
        tok = strtok(nullptr, " \t");
    }
    if (tokenCount == 0) return;

    const char *cmd = tokens[0];

    if      (strcasecmp(cmd, "help") == 0)       { cmdHelp(); }
    else if (strcasecmp(cmd, "io_init") == 0)    { cmdIoInit(); }
    else if (strcasecmp(cmd, "io_reset") == 0)   { cmdIoReset(); }
    else if (strcasecmp(cmd, "io_read") == 0)    { cmdIoRead(); }
    else if (strcasecmp(cmd, "mcu_read") == 0)   { cmdMcuRead(); }
    else if (strcasecmp(cmd, "enable") == 0) {
        if (tokenCount < 2) { Serial.println(F("[ERROR] Usage: enable <x|y|z>")); return; }
        cmdEnable(tokens[1]);
    }
    else if (strcasecmp(cmd, "disable") == 0) {
        if (tokenCount < 2) { Serial.println(F("[ERROR] Usage: disable <x|y|z>")); return; }
        cmdDisable(tokens[1]);
    }
    else if (strcasecmp(cmd, "tmc_init") == 0) {
        if (tokenCount < 2) { Serial.println(F("[ERROR] Usage: tmc_init <x|y|z1|z2> [baud]")); return; }
        cmdTmcInit(tokens[1], tokenCount >= 3 ? tokens[2] : nullptr);
    }
    else if (strcasecmp(cmd, "tmc_status") == 0) {
        if (tokenCount < 2) { Serial.println(F("[ERROR] Usage: tmc_status <x|y|z1|z2>")); return; }
        cmdTmcStatus(tokens[1]);
    }
    else if (strcasecmp(cmd, "tmc_current") == 0) {
        if (tokenCount < 3) { Serial.println(F("[ERROR] Usage: tmc_current <x|y|z1|z2> <percent>")); return; }
        cmdTmcCurrent(tokens[1], tokens[2]);
    }
    else if (strcasecmp(cmd, "tmc_hold") == 0) {
        if (tokenCount < 3) { Serial.println(F("[ERROR] Usage: tmc_hold <x|y|z1|z2> <percent>")); return; }
        cmdTmcHold(tokens[1], tokens[2]);
    }
    else if (strcasecmp(cmd, "tmc_microstep") == 0) {
        if (tokenCount < 3) { Serial.println(F("[ERROR] Usage: tmc_microstep <x|y|z1|z2> <1..256>")); return; }
        cmdTmcMicrostep(tokens[1], tokens[2]);
    }
    else if (strcasecmp(cmd, "tmc_stealthchop") == 0) {
        if (tokenCount < 3) { Serial.println(F("[ERROR] Usage: tmc_stealthchop <x|y|z1|z2> <on|off>")); return; }
        cmdTmcStealthchop(tokens[1], tokens[2]);
    }
    else if (strcasecmp(cmd, "tmc_enable") == 0) {
        if (tokenCount < 2) { Serial.println(F("[ERROR] Usage: tmc_enable <x|y|z1|z2>")); return; }
        cmdTmcEnable(tokens[1]);
    }
    else if (strcasecmp(cmd, "tmc_disable") == 0) {
        if (tokenCount < 2) { Serial.println(F("[ERROR] Usage: tmc_disable <x|y|z1|z2>")); return; }
        cmdTmcDisable(tokens[1]);
    }
    else if (strcasecmp(cmd, "dir") == 0) {
        if (tokenCount < 3) { Serial.println(F("[ERROR] Usage: dir <x|y|z> <0|1>")); return; }
        cmdDir(tokens[1], tokens[2]);
    }
    else if (strcasecmp(cmd, "step") == 0) {
        if (tokenCount < 4) { Serial.println(F("[ERROR] Usage: step <x|y|z> <count> <delay_us>")); return; }
        cmdStep(tokens[1], tokens[2], tokens[3]);
    }
    // --- TMC429 motion controller commands ---
    else if (strcasecmp(cmd, "mc_init") == 0) {
        cmdMcInit(tokenCount >= 2 ? tokens[1] : nullptr);
    }
    else if (strcasecmp(cmd, "mc_status") == 0) {
        cmdMcStatus();
    }
    else if (strcasecmp(cmd, "mc_limits") == 0) {
        if (tokenCount < 5) { Serial.println(F("[ERROR] Usage: mc_limits <x|y|z> <vmin_hz> <vmax_hz> <amax_hz_per_s>")); return; }
        cmdMcLimits(tokens[1], tokens[2], tokens[3], tokens[4]);
    }
    else if (strcasecmp(cmd, "mc_ramp") == 0) {
        if (tokenCount < 2) { Serial.println(F("[ERROR] Usage: mc_ramp <x|y|z>")); return; }
        cmdMcRamp(tokens[1]);
    }
    else if (strcasecmp(cmd, "mc_velocity") == 0) {
        if (tokenCount < 2) { Serial.println(F("[ERROR] Usage: mc_velocity <x|y|z>")); return; }
        cmdMcVelocity(tokens[1]);
    }
    else if (strcasecmp(cmd, "mc_hold") == 0) {
        if (tokenCount < 2) { Serial.println(F("[ERROR] Usage: mc_hold <x|y|z>")); return; }
        cmdMcHold(tokens[1]);
    }
    else if (strcasecmp(cmd, "mc_target") == 0) {
        if (tokenCount < 3) { Serial.println(F("[ERROR] Usage: mc_target <x|y|z> <position>")); return; }
        cmdMcTarget(tokens[1], tokens[2]);
    }
    else if (strcasecmp(cmd, "mc_vtarget") == 0) {
        if (tokenCount < 3) { Serial.println(F("[ERROR] Usage: mc_vtarget <x|y|z> <velocity_hz>")); return; }
        cmdMcVtarget(tokens[1], tokens[2]);
    }
    else if (strcasecmp(cmd, "mc_pos") == 0) {
        if (tokenCount < 2) { Serial.println(F("[ERROR] Usage: mc_pos <x|y|z>")); return; }
        cmdMcPos(tokens[1]);
    }
    else if (strcasecmp(cmd, "mc_setpos") == 0) {
        if (tokenCount < 3) { Serial.println(F("[ERROR] Usage: mc_setpos <x|y|z> <position>")); return; }
        cmdMcSetpos(tokens[1], tokens[2]);
    }
    else if (strcasecmp(cmd, "mc_stop") == 0) {
        if (tokenCount < 2) { Serial.println(F("[ERROR] Usage: mc_stop <x|y|z>")); return; }
        cmdMcStop(tokens[1]);
    }
    else if (strcasecmp(cmd, "mc_stopall") == 0) {
        cmdMcStopall();
    }
    else if (strcasecmp(cmd, "mc_switches") == 0) {
        cmdMcSwitches();
    }
    else if (strcasecmp(cmd, "mc_swpol") == 0) {
        if (tokenCount < 2) { Serial.println(F("[ERROR] Usage: mc_swpol <high|low>")); return; }
        cmdMcSwpol(tokens[1]);
    }
    else if (strcasecmp(cmd, "mc_leftstop") == 0) {
        if (tokenCount < 3) { Serial.println(F("[ERROR] Usage: mc_leftstop <x|y|z> <on|off>")); return; }
        cmdMcLeftstop(tokens[1], tokens[2]);
    }
    else if (strcasecmp(cmd, "mc_rightstop") == 0) {
        if (tokenCount < 3) { Serial.println(F("[ERROR] Usage: mc_rightstop <x|y|z> <on|off>")); return; }
        cmdMcRightstop(tokens[1], tokens[2]);
    }
    else if (strcasecmp(cmd, "mc_rightsw") == 0) {
        if (tokenCount < 2) { Serial.println(F("[ERROR] Usage: mc_rightsw <on|off>")); return; }
        cmdMcRightsw(tokens[1]);
    }
    // --- LED MCU commands ---
    else if (strcasecmp(cmd, "led_set") == 0) {
        if (tokenCount < 7) { Serial.println(F("[ERROR] Usage: led_set <ring|all> <off|static|spin|bounce|pulse> <r> <g> <b> <brightness>")); return; }
        cmdLedSet(tokens, tokenCount);
    }
    else if (strcasecmp(cmd, "led_status") == 0) {
        if (tokenCount < 2) { Serial.println(F("[ERROR] Usage: led_status <ring>")); return; }
        cmdLedStatus(tokens[1]);
    }
    else if (strcasecmp(cmd, "led_config") == 0) {
        if (tokenCount < 3) { Serial.println(F("[ERROR] Usage: led_config <param> <value>")); return; }
        cmdLedConfig(tokens[1], tokens[2]);
    }
    else if (strcasecmp(cmd, "led_getconfig") == 0) {
        cmdLedGetconfig();
    }
    else {
        Serial.print(F("[ERROR] Unknown command: '"));
        Serial.print(cmd);
        Serial.println(F("'. Type 'help' for a list of commands."));
    }
}

// ============================================================================
// Command: help
// ============================================================================

static void cmdHelp() {
    Serial.println(F(
        "\n"
        "===================== AVAILABLE COMMANDS =====================\n"
        "\n"
        "--- General ---\n"
        "  help                                  Show this help text\n"
        "\n"
        "--- I/O Expander (MCP23017) ---\n"
        "  io_init                               Initialize the MCP23017\n"
        "  io_reset                              Hardware-reset the MCP23017\n"
        "  io_read                               Read all expander inputs\n"
        "\n"
        "--- MCU Pin Readback ---\n"
        "  mcu_read                              Read all MCU output pin states\n"
        "\n"
        "--- Motor Enable/Disable (hardware nEnable pin) ---\n"
        "  enable  <x|y|z>                       Pull nEnable LOW  (driver ON)\n"
        "  disable <x|y|z>                       Pull nEnable HIGH (driver OFF)\n"
        "\n"
        "--- TMC2209 UART ---\n"
        "  tmc_init       <x|y|z1|z2> [baud]     Init UART (default 115200)\n"
        "  tmc_status     <x|y|z1|z2>            Read TMC2209 status/settings\n"
        "  tmc_current    <x|y|z1|z2> <percent>  Set run current (0-100%)\n"
        "  tmc_hold       <x|y|z1|z2> <percent>  Set hold current (0-100%)\n"
        "  tmc_microstep  <x|y|z1|z2> <n>        Set microsteps (1..256)\n"
        "  tmc_stealthchop <x|y|z1|z2> <on|off>  Toggle StealthChop mode\n"
        "  tmc_enable     <x|y|z1|z2>            Software-enable via UART\n"
        "  tmc_disable    <x|y|z1|z2>            Software-disable via UART\n"
        "\n"
        "--- Direct Pin Control ---\n"
        "  dir  <x|y|z> <0|1>                    Set direction pin level\n"
        "  step <x|y|z> <count> <delay_us>       Pulse STEP pin <count> times\n"
        "\n"
        "--- TMC429 Motion Controller (mc_) ---\n"
        "  mc_init [clock_mhz]                    Init SPI + TMC429 (default 32)\n"
        "  mc_status                              Read version + status flags\n"
        "  mc_limits <x|y|z> <vmin> <vmax> <amax> Set limits (all in Hz/Hz-per-s)\n"
        "  mc_ramp <x|y|z>                        Ramp mode (position w/ trapezoid)\n"
        "  mc_velocity <x|y|z>                    Velocity mode (continuous)\n"
        "  mc_hold <x|y|z>                        Hold mode (lock position)\n"
        "  mc_target <x|y|z> <position>           Set target position (ramp mode)\n"
        "  mc_vtarget <x|y|z> <velocity_hz>       Set target velocity (vel mode)\n"
        "  mc_pos <x|y|z>                         Read actual + target position\n"
        "  mc_setpos <x|y|z> <position>           Write actual position register\n"
        "  mc_stop <x|y|z>                        Stop one axis\n"
        "  mc_stopall                             Stop all three axes\n"
        "  mc_switches                            Read all limit switch states\n"
        "  mc_swpol <high|low>                    Set switch active polarity\n"
        "  mc_leftstop <x|y|z> <on|off>           Left switch auto-stop\n"
        "  mc_rightstop <x|y|z> <on|off>          Right switch auto-stop\n"
        "  mc_rightsw <on|off>                    Enable right switch inputs\n"
        "\n"
        "--- LED MCU (I2C slave at 0x30) ---\n"
        "  led_set <ring|all> <anim> <r> <g> <b> <brightness>\n"
        "                                        Set animation + color + brightness\n"
        "                                        anim: off|static|spin|bounce|pulse\n"
        "                                        r/g/b: 0-255, brightness: 0-255\n"
        "  led_status <ring>                     Query ring animation state\n"
        "  led_config <param> <value>            Set persistent config parameter\n"
        "                                        params: num_rings, leds_per_ring,\n"
        "                                        spin_width, bounce_width,\n"
        "                                        spin_speed, bounce_speed,\n"
        "                                        pulse_speed (speeds in ms)\n"
        "  led_getconfig                         Read all config parameters\n"
        "\n"
        "NOTES:\n"
        "  - 'z' controls the shared Z STEP/DIR/ENABLE (both Z1 and Z2).\n"
        "  - 'z1' and 'z2' address individual TMC2209 UARTs only.\n"
        "  - Motors are DISABLED at boot. You must 'enable' before stepping.\n"
        "  - You must 'tmc_init' before any tmc_* command for that motor.\n"
        "==============================================================\n"
    ));
}

// ============================================================================
// Command: io_init
// ============================================================================

static void cmdIoInit() {
    Serial.println(F("[IO] Probing MCP23017 at I2C address 0x20..."));

    if (!i2c.probe(IO_EXPANDER_ADDR)) {
        Serial.println(F("[IO] ERROR: MCP23017 not found on I2C bus!"));
        Serial.println(F("[IO]   Check wiring, power, and address jumpers (A2=A1=A0=GND)."));
        ioExpanderInitialized = false;
        return;
    }
    Serial.println(F("[IO]   Device ACKed at 0x20."));

    // Configure all ports as inputs (0xFF). This is the power-on default,
    // but we set it explicitly to be safe after a partial init or soft reset.
    bool ok = true;
    ok &= i2c.writeRegister(IO_EXPANDER_ADDR, MCP_REG_IODIRA, 0xFF);
    ok &= i2c.writeRegister(IO_EXPANDER_ADDR, MCP_REG_IODIRB, 0xFF);

    if (!ok) {
        Serial.println(F("[IO] ERROR: Failed to write direction registers!"));
        ioExpanderInitialized = false;
        return;
    }

    ioExpanderInitialized = true;
    Serial.println(F("[IO] MCP23017 initialized successfully."));
    Serial.println(F("[IO]   IODIRA=0xFF, IODIRB=0xFF (all inputs)."));
    Serial.println(F("[IO]   GPA0-GPA3: TMC2209 DIAG pins (X, Y, Z1, Z2)"));
    Serial.println(F("[IO]   GPB2-GPB3: X limit switches (END1, END2)"));
    Serial.println(F("[IO]   GPB4-GPB5: Y limit switches (END1, END2)"));
    Serial.println(F("[IO]   GPB6-GPB7: Z limit switches (END1, END2)"));
}

// ============================================================================
// Command: io_reset
// ============================================================================

static void cmdIoReset() {
    Serial.println(F("[IO] Performing hardware reset of MCP23017..."));
    Serial.println(F("[IO]   Driving nRESET LOW for 10 ms..."));

    pinMode(PIN_IO_NRESET, OUTPUT);
    digitalWrite(PIN_IO_NRESET, LOW);
    delay(10);
    digitalWrite(PIN_IO_NRESET, HIGH);
    pinMode(PIN_IO_NRESET, INPUT); // Release to external pull-up

    Serial.println(F("[IO]   nRESET released (external 10k pull-up)."));
    Serial.println(F("[IO]   Waiting 50 ms for startup..."));
    delay(50);

    Serial.println(F("[IO]   Re-initializing..."));
    cmdIoInit();
}

// ============================================================================
// Command: io_read
// ============================================================================

static void cmdIoRead() {
    if (!ioExpanderInitialized) {
        Serial.println(F("[IO] ERROR: MCP23017 not initialized. Run 'io_init' first."));
        return;
    }

    uint8_t gpioA = 0, gpioB = 0;
    if (!i2c.readRegister16(IO_EXPANDER_ADDR, MCP_REG_GPIOA, gpioA, gpioB)) {
        Serial.println(F("[IO] ERROR: Failed to read GPIO registers!"));
        return;
    }

    Serial.println(F("[IO] Reading all MCP23017 inputs:"));
    Serial.print(F("[IO]   Raw: GPIOA=0x")); Serial.print(gpioA, HEX);
    Serial.print(F(", GPIOB=0x")); Serial.println(gpioB, HEX);

    Serial.println(F("[IO] --- TMC2209 DIAG Pins ---"));
    Serial.print(F("[IO]   X  DIAG (GPA0): ")); Serial.println((gpioA >> BIT_X_DIAG)  & 1 ? "HIGH" : "LOW");
    Serial.print(F("[IO]   Y  DIAG (GPA1): ")); Serial.println((gpioA >> BIT_Y_DIAG)  & 1 ? "HIGH" : "LOW");
    Serial.print(F("[IO]   Z1 DIAG (GPA2): ")); Serial.println((gpioA >> BIT_Z1_DIAG) & 1 ? "HIGH" : "LOW");
    Serial.print(F("[IO]   Z2 DIAG (GPA3): ")); Serial.println((gpioA >> BIT_Z2_DIAG) & 1 ? "HIGH" : "LOW");

    Serial.println(F("[IO] --- Limit Switches (active HIGH) ---"));
    Serial.print(F("[IO]   X  END1 (GPB2): ")); Serial.println((gpioB >> BIT_X_END1) & 1 ? "TRIGGERED" : "open");
    Serial.print(F("[IO]   X  END2 (GPB3): ")); Serial.println((gpioB >> BIT_X_END2) & 1 ? "TRIGGERED" : "open");
    Serial.print(F("[IO]   Y  END1 (GPB4): ")); Serial.println((gpioB >> BIT_Y_END1) & 1 ? "TRIGGERED" : "open");
    Serial.print(F("[IO]   Y  END2 (GPB5): ")); Serial.println((gpioB >> BIT_Y_END2) & 1 ? "TRIGGERED" : "open");
    Serial.print(F("[IO]   Z  END1 (GPB6): ")); Serial.println((gpioB >> BIT_Z_END1) & 1 ? "TRIGGERED" : "open");
    Serial.print(F("[IO]   Z  END2 (GPB7): ")); Serial.println((gpioB >> BIT_Z_END2) & 1 ? "TRIGGERED" : "open");
}

// ============================================================================
// Command: mcu_read
// ============================================================================

static void cmdMcuRead() {
    Serial.println(F("[MCU] Reading back all MCU-controlled pin states:"));

    Serial.println(F("[MCU] --- Motor STEP Pins ---"));
    Serial.print(F("[MCU]   X  STEP (A1):  ")); Serial.println(digitalRead(PIN_X_STEP) ? "HIGH" : "LOW");
    Serial.print(F("[MCU]   Y  STEP (A3):  ")); Serial.println(digitalRead(PIN_Y_STEP) ? "HIGH" : "LOW");
    Serial.print(F("[MCU]   Z  STEP (A5):  ")); Serial.println(digitalRead(PIN_Z_STEP) ? "HIGH" : "LOW");

    Serial.println(F("[MCU] --- Motor DIR Pins ---"));
    Serial.print(F("[MCU]   X  DIR  (A0):  ")); Serial.println(digitalRead(PIN_X_DIR) ? "HIGH" : "LOW");
    Serial.print(F("[MCU]   Y  DIR  (A2):  ")); Serial.println(digitalRead(PIN_Y_DIR) ? "HIGH" : "LOW");
    Serial.print(F("[MCU]   Z  DIR  (A4):  ")); Serial.println(digitalRead(PIN_Z_DIR) ? "HIGH" : "LOW");

    Serial.println(F("[MCU] --- Motor nEnable Pins (LOW=enabled, HIGH=disabled) ---"));
    Serial.print(F("[MCU]   X  nEN  (D10): ")); Serial.println(digitalRead(PIN_X_NENABLE) ? "HIGH (disabled)" : "LOW (enabled)");
    Serial.print(F("[MCU]   Y  nEN  (D9):  ")); Serial.println(digitalRead(PIN_Y_NENABLE) ? "HIGH (disabled)" : "LOW (enabled)");
    Serial.print(F("[MCU]   Z  nEN  (D8):  ")); Serial.println(digitalRead(PIN_Z_NENABLE) ? "HIGH (disabled)" : "LOW (enabled)");

    Serial.println(F("[MCU] --- I/O Expander nRESET (D6) ---"));
    Serial.print(F("[MCU]   IO nRST (D6):  "));
    Serial.println(digitalRead(PIN_IO_NRESET) ? "HIGH (not in reset)" : "LOW (in reset!)");
}

// ============================================================================
// enable / disable
// ============================================================================

static void cmdEnable(const char *axis) {
    uint8_t stepPin, dirPin, enablePin;
    bool valid;
    getAxisPins(axis, stepPin, dirPin, enablePin, valid);
    if (!valid) { Serial.println(F("[ERROR] Invalid axis. Use: x, y, or z")); return; }

    digitalWrite(enablePin, LOW);
    Serial.print(F("[MOTOR] Axis ")); Serial.print(axis);
    Serial.println(F(" ENABLED (nEnable driven LOW)."));
}

static void cmdDisable(const char *axis) {
    uint8_t stepPin, dirPin, enablePin;
    bool valid;
    getAxisPins(axis, stepPin, dirPin, enablePin, valid);
    if (!valid) { Serial.println(F("[ERROR] Invalid axis. Use: x, y, or z")); return; }

    digitalWrite(enablePin, HIGH);
    Serial.print(F("[MOTOR] Axis ")); Serial.print(axis);
    Serial.println(F(" DISABLED (nEnable driven HIGH)."));
}

// ============================================================================
// tmc_init
// ============================================================================

static void cmdTmcInit(const char *motor, const char *baudStr) {
    long baud = 115200;
    if (baudStr) {
        baud = atol(baudStr);
        if (baud <= 0) { Serial.println(F("[ERROR] Invalid baud rate.")); return; }
    }

    Serial.print(F("[TMC] Initializing UART for motor '"));
    Serial.print(motor);
    Serial.print(F("' at ")); Serial.print(baud); Serial.println(F(" baud..."));

    if (strcasecmp(motor, "x") == 0) {
        tmcX.setup(XMotorSerial, baud);
        pinPeripheral(PIN_X_UART_RX, PIO_SERCOM);
        pinPeripheral(PIN_X_UART_TX, PIO_SERCOM);
        // Flush garbage bytes caused by SERCOM init before pin mux was set
        while (XMotorSerial.available()) XMotorSerial.read();
        tmcXInitialized = true;
        Serial.println(F("[TMC]   SERCOM4 (TX=D7/PB12, RX=D4/PB13)."));
    }
    else if (strcasecmp(motor, "y") == 0) {
        tmcY.setup(Serial1, baud);
        while (Serial1.available()) Serial1.read();
        tmcYInitialized = true;
        Serial.println(F("[TMC]   Serial1/SERCOM3 (TX=D1/PA22, RX=D0/PA23)."));
    }
    else if (strcasecmp(motor, "z1") == 0) {
        tmcZ1.setup(Z1MotorSerial, baud);
        pinPeripheral(PIN_Z1_UART_RX, PIO_SERCOM);
        pinPeripheral(PIN_Z1_UART_TX, PIO_SERCOM);
        while (Z1MotorSerial.available()) Z1MotorSerial.read();
        tmcZ1Initialized = true;
        Serial.println(F("[TMC]   SERCOM1 (TX=D13/PA16, RX=D12/PA17)."));
    }
    else if (strcasecmp(motor, "z2") == 0) {
        tmcZ2.setup(Z2MotorSerial, baud);
        pinPeripheral(PIN_Z2_UART_RX, PIO_SERCOM);
        pinPeripheral(PIN_Z2_UART_TX, PIO_SERCOM);
        while (Z2MotorSerial.available()) Z2MotorSerial.read();
        tmcZ2Initialized = true;
        Serial.println(F("[TMC]   SERCOM5 (TX=D3/PB16, RX=D2/PB17)."));
    }
    else {
        Serial.println(F("[ERROR] Invalid motor. Use: x, y, z1, or z2"));
        return;
    }

    // Allow TMC2209 UART to settle after pin mux + flush
    delay(200);
    TMC2209 *tmc = lookupTmc(motor);
    if (tmc && tmc->isSetupAndCommunicating()) {
        Serial.println(F("[TMC]   Communication OK."));
    } else {
        Serial.println(F("[TMC]   WARNING: TMC2209 not responding."));
        Serial.println(F("[TMC]   Check: wiring, power, baud rate, UART address."));
    }
}

// ============================================================================
// tmc_status
// ============================================================================

static void cmdTmcStatus(const char *motor) {
    TMC2209 *tmc = lookupTmc(motor);
    if (!tmc) { Serial.println(F("[ERROR] Invalid motor. Use: x, y, z1, or z2")); return; }
    if (!isTmcInitialized(motor)) {
        Serial.print(F("[ERROR] '")); Serial.print(motor);
        Serial.println(F("' not initialized. Run 'tmc_init' first.")); return;
    }

    Serial.print(F("[TMC] Status for '")); Serial.print(motor); Serial.println(F("':"));

    bool comm = tmc->isCommunicating();
    Serial.print(F("[TMC]   Communicating:       ")); Serial.println(comm ? "YES" : "NO");
    if (!comm) { Serial.println(F("[TMC]   Cannot read further.")); return; }

    bool hwDis = tmc->hardwareDisabled();
    Serial.print(F("[TMC]   Hardware disabled:    "));
    Serial.println(hwDis ? "YES (nEnable HIGH)" : "NO (nEnable LOW)");

    TMC2209::Settings s = tmc->getSettings();
    Serial.print(F("[TMC]   Software enabled:     ")); Serial.println(s.software_enabled ? "YES" : "NO");
    Serial.print(F("[TMC]   Microsteps/step:      ")); Serial.println(s.microsteps_per_step);
    Serial.print(F("[TMC]   Inverse motor dir:    ")); Serial.println(s.inverse_motor_direction_enabled ? "YES" : "NO");
    Serial.print(F("[TMC]   StealthChop:          ")); Serial.println(s.stealth_chop_enabled ? "ON" : "OFF");
    Serial.print(F("[TMC]   Standstill mode:      "));
    switch (s.standstill_mode) {
        case TMC2209::NORMAL:         Serial.println(F("NORMAL"));         break;
        case TMC2209::FREEWHEELING:   Serial.println(F("FREEWHEELING"));   break;
        case TMC2209::STRONG_BRAKING: Serial.println(F("STRONG_BRAKING")); break;
        case TMC2209::BRAKING:        Serial.println(F("BRAKING"));        break;
    }
    Serial.print(F("[TMC]   IRUN  (run current):  ")); Serial.print(s.irun_percent);
    Serial.print(F("% (reg=")); Serial.print(s.irun_register_value); Serial.println(F(")"));
    Serial.print(F("[TMC]   IHOLD (hold current): ")); Serial.print(s.ihold_percent);
    Serial.print(F("% (reg=")); Serial.print(s.ihold_register_value); Serial.println(F(")"));

    TMC2209::Status st = tmc->getStatus();
    Serial.println(F("[TMC]   --- Fault Flags ---"));
    Serial.print(F("[TMC]   Over-temp warning:    ")); Serial.println(st.over_temperature_warning  ? "YES" : "no");
    Serial.print(F("[TMC]   Over-temp shutdown:   ")); Serial.println(st.over_temperature_shutdown ? "YES" : "no");
    Serial.print(F("[TMC]   Short-to-GND A:       ")); Serial.println(st.short_to_ground_a         ? "YES" : "no");
    Serial.print(F("[TMC]   Short-to-GND B:       ")); Serial.println(st.short_to_ground_b         ? "YES" : "no");
    Serial.print(F("[TMC]   Low-side short A:     ")); Serial.println(st.low_side_short_a          ? "YES" : "no");
    Serial.print(F("[TMC]   Low-side short B:     ")); Serial.println(st.low_side_short_b          ? "YES" : "no");
    Serial.print(F("[TMC]   Open load A:          ")); Serial.println(st.open_load_a               ? "YES" : "no");
    Serial.print(F("[TMC]   Open load B:          ")); Serial.println(st.open_load_b               ? "YES" : "no");
    Serial.print(F("[TMC]   StealthChop active:   ")); Serial.println(st.stealth_chop_mode         ? "YES" : "no");
    Serial.print(F("[TMC]   Standstill:           ")); Serial.println(st.standstill                ? "YES" : "no");
}

// ============================================================================
// tmc_current / tmc_hold
// ============================================================================

static void cmdTmcCurrent(const char *motor, const char *pctStr) {
    TMC2209 *tmc = lookupTmc(motor);
    if (!tmc) { Serial.println(F("[ERROR] Invalid motor.")); return; }
    if (!isTmcInitialized(motor)) {
        Serial.print(F("[ERROR] '")); Serial.print(motor);
        Serial.println(F("' not initialized.")); return;
    }
    int pct = atoi(pctStr);
    if (pct < 0 || pct > 100) { Serial.println(F("[ERROR] Percent must be 0-100.")); return; }

    tmc->setRunCurrent((uint8_t)pct);
    Serial.print(F("[TMC] '")); Serial.print(motor);
    Serial.print(F("' run current set to ")); Serial.print(pct); Serial.println(F("%."));
}

static void cmdTmcHold(const char *motor, const char *pctStr) {
    TMC2209 *tmc = lookupTmc(motor);
    if (!tmc) { Serial.println(F("[ERROR] Invalid motor.")); return; }
    if (!isTmcInitialized(motor)) {
        Serial.print(F("[ERROR] '")); Serial.print(motor);
        Serial.println(F("' not initialized.")); return;
    }
    int pct = atoi(pctStr);
    if (pct < 0 || pct > 100) { Serial.println(F("[ERROR] Percent must be 0-100.")); return; }

    tmc->setHoldCurrent((uint8_t)pct);
    Serial.print(F("[TMC] '")); Serial.print(motor);
    Serial.print(F("' hold current set to ")); Serial.print(pct); Serial.println(F("%."));
}

// ============================================================================
// tmc_microstep
// ============================================================================

static void cmdTmcMicrostep(const char *motor, const char *msStr) {
    TMC2209 *tmc = lookupTmc(motor);
    if (!tmc) { Serial.println(F("[ERROR] Invalid motor.")); return; }
    if (!isTmcInitialized(motor)) {
        Serial.print(F("[ERROR] '")); Serial.print(motor);
        Serial.println(F("' not initialized.")); return;
    }
    int ms = atoi(msStr);
    bool valid = false;
    for (int v = 1; v <= 256; v *= 2) { if (ms == v) { valid = true; break; } }
    if (!valid) {
        Serial.println(F("[ERROR] Must be 1, 2, 4, 8, 16, 32, 64, 128, or 256."));
        return;
    }

    tmc->setMicrostepsPerStep((uint16_t)ms);
    Serial.print(F("[TMC] '")); Serial.print(motor);
    Serial.print(F("' microsteps set to ")); Serial.print(ms);
    Serial.println(F(" per full step."));
}

// ============================================================================
// tmc_stealthchop
// ============================================================================

static void cmdTmcStealthchop(const char *motor, const char *state) {
    TMC2209 *tmc = lookupTmc(motor);
    if (!tmc) { Serial.println(F("[ERROR] Invalid motor.")); return; }
    if (!isTmcInitialized(motor)) {
        Serial.print(F("[ERROR] '")); Serial.print(motor);
        Serial.println(F("' not initialized.")); return;
    }
    if (strcasecmp(state, "on") == 0) {
        tmc->enableStealthChop();
        Serial.print(F("[TMC] '")); Serial.print(motor);
        Serial.println(F("' StealthChop ENABLED."));
    } else if (strcasecmp(state, "off") == 0) {
        tmc->disableStealthChop();
        Serial.print(F("[TMC] '")); Serial.print(motor);
        Serial.println(F("' StealthChop DISABLED (SpreadCycle)."));
    } else {
        Serial.println(F("[ERROR] State must be 'on' or 'off'."));
    }
}

// ============================================================================
// tmc_enable / tmc_disable (software, via UART)
// ============================================================================

static void cmdTmcEnable(const char *motor) {
    TMC2209 *tmc = lookupTmc(motor);
    if (!tmc) { Serial.println(F("[ERROR] Invalid motor.")); return; }
    if (!isTmcInitialized(motor)) {
        Serial.print(F("[ERROR] '")); Serial.print(motor);
        Serial.println(F("' not initialized.")); return;
    }
    tmc->enable();
    Serial.print(F("[TMC] '")); Serial.print(motor);
    Serial.println(F("' software-ENABLED."));
    Serial.println(F("[TMC]   (Hardware nEnable must also be LOW for driver to run.)"));
}

static void cmdTmcDisable(const char *motor) {
    TMC2209 *tmc = lookupTmc(motor);
    if (!tmc) { Serial.println(F("[ERROR] Invalid motor.")); return; }
    if (!isTmcInitialized(motor)) {
        Serial.print(F("[ERROR] '")); Serial.print(motor);
        Serial.println(F("' not initialized.")); return;
    }
    tmc->disable();
    Serial.print(F("[TMC] '")); Serial.print(motor);
    Serial.println(F("' software-DISABLED."));
}

// ============================================================================
// dir / step
// ============================================================================

static void cmdDir(const char *axis, const char *valStr) {
    uint8_t stepPin, dirPin, enablePin;
    bool valid;
    getAxisPins(axis, stepPin, dirPin, enablePin, valid);
    if (!valid) { Serial.println(F("[ERROR] Invalid axis. Use: x, y, or z")); return; }

    int val = atoi(valStr);
    if (val != 0 && val != 1) { Serial.println(F("[ERROR] Must be 0 or 1.")); return; }

    digitalWrite(dirPin, val ? HIGH : LOW);
    Serial.print(F("[MOTOR] Axis ")); Serial.print(axis);
    Serial.print(F(" DIR set to ")); Serial.println(val ? "HIGH (1)" : "LOW (0)");
}

static void cmdStep(const char *axis, const char *countStr, const char *delayStr) {
    uint8_t stepPin, dirPin, enablePin;
    bool valid;
    getAxisPins(axis, stepPin, dirPin, enablePin, valid);
    if (!valid) { Serial.println(F("[ERROR] Invalid axis. Use: x, y, or z")); return; }

    long count = atol(countStr);
    long delayUs = atol(delayStr);
    if (count <= 0) { Serial.println(F("[ERROR] Count must be positive.")); return; }
    if (delayUs < 2) { Serial.println(F("[ERROR] Delay must be >= 2 us.")); return; }

    if (digitalRead(enablePin) == HIGH) {
        Serial.println(F("[MOTOR] WARNING: nEnable is HIGH - driver disabled!"));
        Serial.println(F("[MOTOR]   Pulses will be generated but motor will not move."));
    }

    Serial.print(F("[MOTOR] Stepping ")); Serial.print(axis);
    Serial.print(F(": ")); Serial.print(count);
    Serial.print(F(" steps @ ")); Serial.print(delayUs); Serial.println(F(" us..."));

    for (long i = 0; i < count; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(2); // Min pulse width for TMC2209 (~100 ns); 2 us is safe
        digitalWrite(stepPin, LOW);
        if (delayUs > 2) {
            delayMicroseconds(delayUs - 2);
        }
    }

    Serial.print(F("[MOTOR] ")); Serial.print(axis);
    Serial.print(F(" done (")); Serial.print(count); Serial.println(F(" steps)."));
}

// ============================================================================
// TMC429 Motion Controller Commands
// ============================================================================

// Helper: convert axis string to TMC429 motor index (0=X/M1, 1=Y/M2, 2=Z/M3)
static int lookupMcMotor(const char *axis) {
    if (strcasecmp(axis, "x") == 0) return 0;
    if (strcasecmp(axis, "y") == 0) return 1;
    if (strcasecmp(axis, "z") == 0) return 2;
    return -1;
}

// Helper: check if mc is initialized and print error if not
static bool requireMc() {
    if (!mcInitialized) {
        Serial.println(F("[MC] ERROR: TMC429 not initialized. Run 'mc_init' first."));
        return false;
    }
    return true;
}

// mc_init [clock_mhz]
static void cmdMcInit(const char *clkStr) {
    uint8_t clkMhz = 32; // Default: 32 MHz (TOGNJING XOS20032000LT00351005)
    if (clkStr) {
        int val = atoi(clkStr);
        if (val < 4 || val > 32) {
            Serial.println(F("[MC] ERROR: Clock must be 4-32 MHz."));
            return;
        }
        clkMhz = (uint8_t)val;
    }

    Serial.print(F("[MC] Initializing TMC429 on SPI (nCS=D5) with clock="));
    Serial.print(clkMhz);
    Serial.println(F(" MHz..."));

    motionController.setup(PIN_MC_NCS, clkMhz);

    if (motionController.communicating()) {
        uint32_t ver = motionController.getVersion();
        Serial.println(F("[MC]   Communication OK."));
        Serial.print(F("[MC]   Version register: 0x"));
        Serial.println(ver, HEX);
        mcInitialized = true;

        // Configure for step/direction output mode (not SPI driver chain).
        // The library's setup() calls setStepDirOutput() internally.
        Serial.println(F("[MC]   Step/Dir output mode active."));
        Serial.println(F("[MC]   Motor mapping: X=motor0(M1), Y=motor1(M2), Z=motor2(M3)"));
        Serial.println(F("[MC]   Limit switches: END1=Left, END2=Right"));
    } else {
        Serial.println(F("[MC]   ERROR: TMC429 not responding!"));
        Serial.println(F("[MC]   Check: SPI wiring, clock, power, nCS (D5)."));
        mcInitialized = false;
    }
}

// mc_status
static void cmdMcStatus() {
    if (!requireMc()) return;

    uint32_t ver = motionController.getVersion();
    Serial.print(F("[MC] Version: 0x")); Serial.println(ver, HEX);

    TMC429::Status st = motionController.getStatus();
    Serial.println(F("[MC] Status flags:"));
    Serial.print(F("[MC]   At target pos M0 (X): ")); Serial.println(st.at_target_position_0 ? "YES" : "no");
    Serial.print(F("[MC]   At target pos M1 (Y): ")); Serial.println(st.at_target_position_1 ? "YES" : "no");
    Serial.print(F("[MC]   At target pos M2 (Z): ")); Serial.println(st.at_target_position_2 ? "YES" : "no");
    Serial.print(F("[MC]   Switch left M0 (X):   ")); Serial.println(st.switch_left_0 ? "ACTIVE" : "inactive");
    Serial.print(F("[MC]   Switch left M1 (Y):   ")); Serial.println(st.switch_left_1 ? "ACTIVE" : "inactive");
    Serial.print(F("[MC]   Switch left M2 (Z):   ")); Serial.println(st.switch_left_2 ? "ACTIVE" : "inactive");
    Serial.print(F("[MC]   Interrupt:             ")); Serial.println(st.interrupt ? "YES" : "no");

    // Print position and velocity for all three axes
    for (int m = 0; m < 3; m++) {
        const char *name = (m == 0) ? "X" : (m == 1) ? "Y" : "Z";
        Serial.print(F("[MC]   Motor ")); Serial.print(name);
        Serial.print(F(": pos=")); Serial.print(motionController.getActualPosition(m));
        Serial.print(F(" target=")); Serial.print(motionController.getTargetPosition(m));
        Serial.print(F(" vel=")); Serial.print(motionController.getActualVelocityInHz(m));
        Serial.println(F(" Hz"));
    }
}

// mc_limits <axis> <vmin_hz> <vmax_hz> <amax_hz_per_s>
static void cmdMcLimits(const char *axis, const char *vminStr, const char *vmaxStr, const char *amaxStr) {
    if (!requireMc()) return;
    int m = lookupMcMotor(axis);
    if (m < 0) { Serial.println(F("[ERROR] Invalid axis. Use: x, y, or z")); return; }

    uint32_t vmin = (uint32_t)atol(vminStr);
    uint32_t vmax = (uint32_t)atol(vmaxStr);
    uint32_t amax = (uint32_t)atol(amaxStr);

    if (vmin == 0) vmin = 1; // Library minimum is 1
    if (vmax <= vmin) {
        Serial.println(F("[ERROR] vmax must be greater than vmin."));
        return;
    }

    Serial.print(F("[MC] Setting limits for axis ")); Serial.print(axis);
    Serial.print(F(": vmin=")); Serial.print(vmin);
    Serial.print(F(" Hz, vmax=")); Serial.print(vmax);
    Serial.print(F(" Hz, amax=")); Serial.print(amax);
    Serial.println(F(" Hz/s"));

    motionController.setLimitsInHz(m, vmin, vmax, amax);

    // Read back what was actually set (may differ due to register resolution)
    Serial.print(F("[MC]   Actual vmin=")); Serial.print(motionController.getVelocityMinInHz(m));
    Serial.print(F(" Hz, vmax=")); Serial.print(motionController.getVelocityMaxInHz(m));
    Serial.print(F(" Hz, amax=")); Serial.print(motionController.getAccelerationMaxInHzPerS(m));
    Serial.println(F(" Hz/s"));
}

// mc_ramp <axis>
static void cmdMcRamp(const char *axis) {
    if (!requireMc()) return;
    int m = lookupMcMotor(axis);
    if (m < 0) { Serial.println(F("[ERROR] Invalid axis. Use: x, y, or z")); return; }

    motionController.setRampMode(m);
    Serial.print(F("[MC] Axis ")); Serial.print(axis);
    Serial.println(F(" set to RAMP mode (position control with trapezoidal profile)."));
}

// mc_velocity <axis>
static void cmdMcVelocity(const char *axis) {
    if (!requireMc()) return;
    int m = lookupMcMotor(axis);
    if (m < 0) { Serial.println(F("[ERROR] Invalid axis. Use: x, y, or z")); return; }

    motionController.setVelocityMode(m);
    Serial.print(F("[MC] Axis ")); Serial.print(axis);
    Serial.println(F(" set to VELOCITY mode (continuous rotation)."));
}

// mc_hold <axis>
static void cmdMcHold(const char *axis) {
    if (!requireMc()) return;
    int m = lookupMcMotor(axis);
    if (m < 0) { Serial.println(F("[ERROR] Invalid axis. Use: x, y, or z")); return; }

    motionController.setHoldMode(m);
    Serial.print(F("[MC] Axis ")); Serial.print(axis);
    Serial.println(F(" set to HOLD mode (hold current position)."));
}

// mc_target <axis> <position>
static void cmdMcTarget(const char *axis, const char *posStr) {
    if (!requireMc()) return;
    int m = lookupMcMotor(axis);
    if (m < 0) { Serial.println(F("[ERROR] Invalid axis. Use: x, y, or z")); return; }

    int32_t pos = (int32_t)atol(posStr);
    motionController.setTargetPosition(m, pos);
    Serial.print(F("[MC] Axis ")); Serial.print(axis);
    Serial.print(F(" target position set to ")); Serial.println(pos);
}

// mc_vtarget <axis> <velocity_hz>
static void cmdMcVtarget(const char *axis, const char *velStr) {
    if (!requireMc()) return;
    int m = lookupMcMotor(axis);
    if (m < 0) { Serial.println(F("[ERROR] Invalid axis. Use: x, y, or z")); return; }

    int32_t vel = (int32_t)atol(velStr);
    motionController.setTargetVelocityInHz(m, vel);
    Serial.print(F("[MC] Axis ")); Serial.print(axis);
    Serial.print(F(" target velocity set to ")); Serial.print(vel);
    Serial.println(F(" Hz (negative = reverse direction)"));
}

// mc_pos <axis>
static void cmdMcPos(const char *axis) {
    if (!requireMc()) return;
    int m = lookupMcMotor(axis);
    if (m < 0) { Serial.println(F("[ERROR] Invalid axis. Use: x, y, or z")); return; }

    int32_t actual = motionController.getActualPosition(m);
    int32_t target = motionController.getTargetPosition(m);
    int32_t vel    = motionController.getActualVelocityInHz(m);
    bool    atPos  = motionController.atTargetPosition(m);

    Serial.print(F("[MC] Axis ")); Serial.print(axis); Serial.println(F(":"));
    Serial.print(F("[MC]   Actual position:  ")); Serial.println(actual);
    Serial.print(F("[MC]   Target position:  ")); Serial.println(target);
    Serial.print(F("[MC]   Actual velocity:  ")); Serial.print(vel); Serial.println(F(" Hz"));
    Serial.print(F("[MC]   At target:        ")); Serial.println(atPos ? "YES" : "no");
}

// mc_setpos <axis> <position>
static void cmdMcSetpos(const char *axis, const char *posStr) {
    if (!requireMc()) return;
    int m = lookupMcMotor(axis);
    if (m < 0) { Serial.println(F("[ERROR] Invalid axis. Use: x, y, or z")); return; }

    int32_t pos = (int32_t)atol(posStr);
    motionController.setActualPosition(m, pos);
    Serial.print(F("[MC] Axis ")); Serial.print(axis);
    Serial.print(F(" actual position register set to ")); Serial.println(pos);
}

// mc_stop <axis>
static void cmdMcStop(const char *axis) {
    if (!requireMc()) return;
    int m = lookupMcMotor(axis);
    if (m < 0) { Serial.println(F("[ERROR] Invalid axis. Use: x, y, or z")); return; }

    motionController.stop(m);
    Serial.print(F("[MC] Axis ")); Serial.print(axis);
    Serial.println(F(" STOPPED (velocity ramped to zero)."));
}

// mc_stopall
static void cmdMcStopall() {
    if (!requireMc()) return;

    motionController.stopAll();
    Serial.println(F("[MC] ALL axes STOPPED."));
}

// mc_switches — read all limit switch states
static void cmdMcSwitches() {
    if (!requireMc()) return;

    Serial.println(F("[MC] Limit switch states (as seen by TMC429):"));
    Serial.print(F("[MC]   X Left  (END1/REF1L): ")); Serial.println(motionController.leftSwitchActive(0)  ? "ACTIVE" : "inactive");
    Serial.print(F("[MC]   X Right (END2/REF1R): ")); Serial.println(motionController.rightSwitchActive(0) ? "ACTIVE" : "inactive");
    Serial.print(F("[MC]   Y Left  (END1/REF2L): ")); Serial.println(motionController.leftSwitchActive(1)  ? "ACTIVE" : "inactive");
    Serial.print(F("[MC]   Y Right (END2/REF2R): ")); Serial.println(motionController.rightSwitchActive(1) ? "ACTIVE" : "inactive");
    Serial.print(F("[MC]   Z Left  (END1/REF3L): ")); Serial.println(motionController.leftSwitchActive(2)  ? "ACTIVE" : "inactive");
    Serial.print(F("[MC]   Z Right (END2/REF3R): ")); Serial.println(motionController.rightSwitchActive(2) ? "ACTIVE" : "inactive");

    // Also report stop-on-switch settings
    Serial.println(F("[MC] Auto-stop configuration:"));
    for (int m = 0; m < 3; m++) {
        const char *name = (m == 0) ? "X" : (m == 1) ? "Y" : "Z";
        Serial.print(F("[MC]   ")); Serial.print(name);
        Serial.print(F(" left-stop: ")); Serial.print(motionController.leftSwitchStopEnabled(m) ? "ON" : "OFF");
        Serial.print(F(", right-stop: ")); Serial.println(motionController.rightSwitchStopEnabled(m) ? "ON" : "OFF");
    }
    Serial.print(F("[MC]   Right switches globally: ")); Serial.println(motionController.rightSwitchesEnabled() ? "ENABLED" : "DISABLED");
}

// mc_swpol <high|low>
static void cmdMcSwpol(const char *state) {
    if (!requireMc()) return;

    if (strcasecmp(state, "high") == 0) {
        motionController.setSwitchesActiveHigh();
        Serial.println(F("[MC] Switches set to ACTIVE HIGH."));
    } else if (strcasecmp(state, "low") == 0) {
        motionController.setSwitchesActiveLow();
        Serial.println(F("[MC] Switches set to ACTIVE LOW."));
    } else {
        Serial.println(F("[ERROR] Use 'high' or 'low'."));
    }
}

// mc_leftstop <axis> <on|off>
static void cmdMcLeftstop(const char *axis, const char *state) {
    if (!requireMc()) return;
    int m = lookupMcMotor(axis);
    if (m < 0) { Serial.println(F("[ERROR] Invalid axis. Use: x, y, or z")); return; }

    if (strcasecmp(state, "on") == 0) {
        motionController.enableLeftSwitchStop(m);
        Serial.print(F("[MC] Axis ")); Serial.print(axis);
        Serial.println(F(" left switch auto-stop ENABLED."));
    } else if (strcasecmp(state, "off") == 0) {
        motionController.disableLeftSwitchStop(m);
        Serial.print(F("[MC] Axis ")); Serial.print(axis);
        Serial.println(F(" left switch auto-stop DISABLED."));
    } else {
        Serial.println(F("[ERROR] Use 'on' or 'off'."));
    }
}

// mc_rightstop <axis> <on|off>
static void cmdMcRightstop(const char *axis, const char *state) {
    if (!requireMc()) return;
    int m = lookupMcMotor(axis);
    if (m < 0) { Serial.println(F("[ERROR] Invalid axis. Use: x, y, or z")); return; }

    if (strcasecmp(state, "on") == 0) {
        motionController.enableRightSwitchStop(m);
        Serial.print(F("[MC] Axis ")); Serial.print(axis);
        Serial.println(F(" right switch auto-stop ENABLED."));
    } else if (strcasecmp(state, "off") == 0) {
        motionController.disableRightSwitchStop(m);
        Serial.print(F("[MC] Axis ")); Serial.print(axis);
        Serial.println(F(" right switch auto-stop DISABLED."));
    } else {
        Serial.println(F("[ERROR] Use 'on' or 'off'."));
    }
}

// mc_rightsw <on|off> — enable/disable right switch inputs globally
static void cmdMcRightsw(const char *state) {
    if (!requireMc()) return;

    if (strcasecmp(state, "on") == 0) {
        motionController.enableRightSwitches();
        Serial.println(F("[MC] Right switch inputs ENABLED globally."));
    } else if (strcasecmp(state, "off") == 0) {
        motionController.disableRightSwitches();
        Serial.println(F("[MC] Right switch inputs DISABLED globally."));
    } else {
        Serial.println(F("[ERROR] Use 'on' or 'off'."));
    }
}

// ============================================================================
// LED MCU Commands (I2C communication with LED MCU at address 0x30)
// ============================================================================

/// Helper: print LED MCU response code
static void printLedRsp(uint8_t rsp) {
    switch (rsp) {
        case LED_RSP_OK:      Serial.println(F("OK"));              break;
        case LED_RSP_BAD_CMD: Serial.println(F("UNKNOWN COMMAND")); break;
        case LED_RSP_BAD_FMT: Serial.println(F("BAD FORMAT"));      break;
        default: Serial.print(F("UNKNOWN (0x")); Serial.print(rsp, HEX); Serial.println(F(")")); break;
    }
}

/// Helper: parse animation name string to protocol byte. Returns 0xFF on error.
static uint8_t parseAnimName(const char *name) {
    if (strcasecmp(name, "off")    == 0) return LED_ANIM_OFF;
    if (strcasecmp(name, "static") == 0) return LED_ANIM_STATIC;
    if (strcasecmp(name, "spin")   == 0) return LED_ANIM_SPIN;
    if (strcasecmp(name, "bounce") == 0) return LED_ANIM_BOUNCE;
    if (strcasecmp(name, "pulse")  == 0) return LED_ANIM_PULSE;
    return 0xFF;
}

/// Helper: animation byte to name string
static const char *animName(uint8_t anim) {
    switch (anim) {
        case LED_ANIM_OFF:    return "off";
        case LED_ANIM_STATIC: return "static";
        case LED_ANIM_SPIN:   return "spin";
        case LED_ANIM_BOUNCE: return "bounce";
        case LED_ANIM_PULSE:  return "pulse";
        default:              return "unknown";
    }
}

// led_set <ring|all> <off|static|spin|bounce|pulse> <r> <g> <b> <brightness>
static void cmdLedSet(char *tokens[], size_t count) {
    // tokens[0]="led_set", [1]=ring, [2]=anim, [3]=r, [4]=g, [5]=b, [6]=brightness
    uint8_t ring;
    if (strcasecmp(tokens[1], "all") == 0) {
        ring = 0xFF;
    } else {
        int r = atoi(tokens[1]);
        if (r < 0 || r > 254) { Serial.println(F("[ERROR] Ring must be 0-254 or 'all'.")); return; }
        ring = (uint8_t)r;
    }

    uint8_t anim = parseAnimName(tokens[2]);
    if (anim == 0xFF) {
        Serial.println(F("[ERROR] Animation must be: off, static, spin, bounce, or pulse."));
        return;
    }

    int rv = atoi(tokens[3]), gv = atoi(tokens[4]), bv = atoi(tokens[5]);
    int brv = atoi(tokens[6]);
    if (rv < 0 || rv > 255 || gv < 0 || gv > 255 || bv < 0 || bv > 255 || brv < 0 || brv > 255) {
        Serial.println(F("[ERROR] R, G, B, and brightness must be 0-255."));
        return;
    }

    uint8_t cmd[7] = {
        LED_CMD_SET_ANIM, ring, anim,
        (uint8_t)rv, (uint8_t)gv, (uint8_t)bv, (uint8_t)brv
    };

    Serial.print(F("[LED] Sending: ring="));
    Serial.print(ring == 0xFF ? "all" : tokens[1]);
    Serial.print(F(" anim=")); Serial.print(animName(anim));
    Serial.print(F(" color=(")); Serial.print(rv);
    Serial.print(F(",")); Serial.print(gv);
    Serial.print(F(",")); Serial.print(bv);
    Serial.print(F(") brightness=")); Serial.println(brv);

    if (!i2c.writeBytes(LED_MCU_ADDR, cmd, sizeof(cmd))) {
        Serial.println(F("[LED] ERROR: I2C write failed (no ACK from LED MCU)."));
        return;
    }

    delay(5); // Allow LED MCU to process command

    uint8_t rsp;
    if (!i2c.readBytes(LED_MCU_ADDR, &rsp, 1)) {
        Serial.println(F("[LED] ERROR: I2C read failed (no ACK from LED MCU)."));
        return;
    }

    Serial.print(F("[LED] Response: ")); printLedRsp(rsp);
}

// led_status <ring>
static void cmdLedStatus(const char *ringStr) {
    int ringVal = atoi(ringStr);
    if (ringVal < 0 || ringVal > 254) {
        Serial.println(F("[ERROR] Ring must be 0-254."));
        return;
    }

    uint8_t cmd[2] = { LED_CMD_GET_STATUS, (uint8_t)ringVal };

    if (!i2c.writeBytes(LED_MCU_ADDR, cmd, sizeof(cmd))) {
        Serial.println(F("[LED] ERROR: I2C write failed."));
        return;
    }

    delay(5);

    uint8_t rsp[7];
    if (!i2c.readBytes(LED_MCU_ADDR, rsp, 7)) {
        Serial.println(F("[LED] ERROR: I2C read failed."));
        return;
    }

    if (rsp[0] != LED_RSP_OK) {
        Serial.print(F("[LED] Response: ")); printLedRsp(rsp[0]);
        return;
    }

    Serial.print(F("[LED] Ring ")); Serial.print(ringVal); Serial.println(F(" status:"));
    Serial.print(F("[LED]   Animation:  ")); Serial.println(animName(rsp[1]));
    Serial.print(F("[LED]   Color:      (")); Serial.print(rsp[2]);
    Serial.print(F(", ")); Serial.print(rsp[3]);
    Serial.print(F(", ")); Serial.print(rsp[4]); Serial.println(F(")"));
    Serial.print(F("[LED]   Brightness: ")); Serial.println(rsp[5]);
    Serial.print(F("[LED]   Num rings:  ")); Serial.println(rsp[6]);
}

// led_config <param> <value>
static void cmdLedConfig(const char *param, const char *valStr) {
    uint8_t paramId;
    if      (strcasecmp(param, "num_rings")     == 0) paramId = LED_CFG_NUM_RINGS;
    else if (strcasecmp(param, "leds_per_ring") == 0) paramId = LED_CFG_LEDS_PER_RING;
    else if (strcasecmp(param, "spin_width")    == 0) paramId = LED_CFG_SPIN_WIDTH;
    else if (strcasecmp(param, "bounce_width")  == 0) paramId = LED_CFG_BOUNCE_WIDTH;
    else if (strcasecmp(param, "spin_speed")    == 0) paramId = LED_CFG_SPIN_SPEED;
    else if (strcasecmp(param, "bounce_speed")  == 0) paramId = LED_CFG_BOUNCE_SPEED;
    else if (strcasecmp(param, "pulse_speed")   == 0) paramId = LED_CFG_PULSE_SPEED;
    else {
        Serial.println(F("[ERROR] Unknown param. Use: num_rings, leds_per_ring,"));
        Serial.println(F("        spin_width, bounce_width, spin_speed, bounce_speed, pulse_speed"));
        return;
    }

    uint16_t value = (uint16_t)atoi(valStr);
    uint8_t cmd[4] = {
        LED_CMD_SET_CONFIG, paramId,
        (uint8_t)(value & 0xFF), (uint8_t)(value >> 8)
    };

    Serial.print(F("[LED] Setting config: ")); Serial.print(param);
    Serial.print(F(" = ")); Serial.println(value);

    if (!i2c.writeBytes(LED_MCU_ADDR, cmd, sizeof(cmd))) {
        Serial.println(F("[LED] ERROR: I2C write failed."));
        return;
    }

    delay(50); // Flash write on LED MCU takes time

    uint8_t rsp;
    if (!i2c.readBytes(LED_MCU_ADDR, &rsp, 1)) {
        Serial.println(F("[LED] ERROR: I2C read failed."));
        return;
    }

    Serial.print(F("[LED] Response: ")); printLedRsp(rsp);
    if (rsp == LED_RSP_OK) {
        Serial.println(F("[LED] Config saved to flash on LED MCU."));
    }
}

// led_getconfig
static void cmdLedGetconfig() {
    uint8_t cmd[1] = { LED_CMD_GET_CONFIG };

    if (!i2c.writeBytes(LED_MCU_ADDR, cmd, 1)) {
        Serial.println(F("[LED] ERROR: I2C write failed."));
        return;
    }

    delay(5);

    uint8_t rsp[11];
    if (!i2c.readBytes(LED_MCU_ADDR, rsp, 11)) {
        Serial.println(F("[LED] ERROR: I2C read failed."));
        return;
    }

    if (rsp[0] != LED_RSP_OK) {
        Serial.print(F("[LED] Response: ")); printLedRsp(rsp[0]);
        return;
    }

    uint16_t spinSpeed   = (uint16_t)rsp[5] | ((uint16_t)rsp[6] << 8);
    uint16_t bounceSpeed = (uint16_t)rsp[7] | ((uint16_t)rsp[8] << 8);
    uint16_t pulseSpeed  = (uint16_t)rsp[9] | ((uint16_t)rsp[10] << 8);

    Serial.println(F("[LED] LED MCU configuration (from flash):"));
    Serial.print(F("[LED]   num_rings:     ")); Serial.println(rsp[1]);
    Serial.print(F("[LED]   leds_per_ring: ")); Serial.println(rsp[2]);
    Serial.print(F("[LED]   spin_width:    ")); Serial.println(rsp[3]);
    Serial.print(F("[LED]   bounce_width:  ")); Serial.println(rsp[4]);
    Serial.print(F("[LED]   spin_speed:    ")); Serial.print(spinSpeed);   Serial.println(F(" ms"));
    Serial.print(F("[LED]   bounce_speed:  ")); Serial.print(bounceSpeed); Serial.println(F(" ms"));
    Serial.print(F("[LED]   pulse_speed:   ")); Serial.print(pulseSpeed);  Serial.println(F(" ms"));
}

// ============================================================================
// Helpers
// ============================================================================

static TMC2209 *lookupTmc(const char *motor) {
    if (strcasecmp(motor, "x") == 0)  return &tmcX;
    if (strcasecmp(motor, "y") == 0)  return &tmcY;
    if (strcasecmp(motor, "z1") == 0) return &tmcZ1;
    if (strcasecmp(motor, "z2") == 0) return &tmcZ2;
    return nullptr;
}

static bool isTmcInitialized(const char *motor) {
    if (strcasecmp(motor, "x") == 0)  return tmcXInitialized;
    if (strcasecmp(motor, "y") == 0)  return tmcYInitialized;
    if (strcasecmp(motor, "z1") == 0) return tmcZ1Initialized;
    if (strcasecmp(motor, "z2") == 0) return tmcZ2Initialized;
    return false;
}

static void getAxisPins(const char *axis, uint8_t &stepPin, uint8_t &dirPin,
                        uint8_t &enablePin, bool &valid) {
    valid = true;
    if      (strcasecmp(axis, "x") == 0) { stepPin = PIN_X_STEP; dirPin = PIN_X_DIR; enablePin = PIN_X_NENABLE; }
    else if (strcasecmp(axis, "y") == 0) { stepPin = PIN_Y_STEP; dirPin = PIN_Y_DIR; enablePin = PIN_Y_NENABLE; }
    else if (strcasecmp(axis, "z") == 0) { stepPin = PIN_Z_STEP; dirPin = PIN_Z_DIR; enablePin = PIN_Z_NENABLE; }
    else { valid = false; }
}
