// ============================================================================
// BSU v4 — Motor, Sensor, Drawer & Solenoid Test CLI
// Target: Adafruit Metro M4 Express (ATSAMD51J19A)
// ============================================================================
//
// Provides a Serial CLI (115200 baud over USB) for interactively testing:
//   - 3x TMC2209 stepper motor drivers (Z1, Z2, M3)
//   - TMC429 motion controller (SPI on SERCOM4)
//   - 2x MCP23017 I/O expanders
//       - EXTRA_IO_EXPANDER @ 0x21 : motor enables, DIAG pins, Z/spare end
//         switches, drawer-handle button
//       - TRAY_IO_EXPANDER  @ 0x20 : 12 solenoid outputs + tray signals
//   - Drawer-handle bi-color LED via DRV8231A H-bridge on D10/D11
//   - LED MCU (I2C slave at 0x30) — unchanged from PSU
//
// Physical motor mapping:
//   Z1 — own STEP, DIR, UART; EN shared with Z2 (GPA6 on EXTRA expander)
//   Z2 — own STEP, DIR, UART; EN shared with Z1 (GPA6 on EXTRA expander)
//   M3 — own STEP, DIR, UART; EN on GPA7 of EXTRA expander
//
// TMC429 channel mapping:
//   z1 -> TMC429 motor 0 (M1 pads)  — uses Z_TOP/Z_BOT switches (shared)
//   m3 -> TMC429 motor 1 (M2 pads)  — uses SPARE_TOP/SPARE_BOT switches
//   z2 -> TMC429 motor 2 (M3 pads)  — uses Z_TOP/Z_BOT switches (shared)
// (The CLI axis name to TMC429 motor index is NOT a 1:1 z1/z2/m3 order
//  because the PCB routes M2 pads to M3's driver and M3 pads to Z2's driver.)
//
// UART wiring note:
//   Each TMC2209 uses a single-wire UART. The MCU TX is connected through a
//   1k resistor to the TMC2209 PDN_UART pin, and MCU RX is connected directly
//   to the same line. The janelia-arduino/TMC2209 library handles the echo.
//
// SERCOM assignments (Metro M4 / ATSAMD51J19A):
//   Z1 UART -> SERCOM1  (TX=D13/PA16 PAD[0], RX=D12/PA17 PAD[1])
//   Z2 UART -> SERCOM5  (TX=D3/PB16 PAD[0], RX=D2/PB17 PAD[1])
//   M3 UART -> SERCOM3  (TX=D1/PA22 PAD[0], RX=D0/PA23 PAD[1])  = Serial1
//   MC SPI  -> SERCOM4  (MOSI=D7/PB12 PAD[0], SCK=D4/PB13 PAD[1], MISO=D5/PB14 PAD[2])
//
// SERCOM5 CONFLICT NOTE:
//   The Metro M4 BSP assigns SERCOM5 to Wire (I2C) on SDA/SCL (PB2/PB3).
//   The Z2 motor UART also needs SERCOM5 on PB16/PB17. Since a SERCOM can
//   only operate in one mode at a time, and Wire's ISR symbols are strong,
//   we use bitbang I2C (SoftI2C) for the MCP23017s. That frees SERCOM5 for
//   the Z2 UART. ~100 kHz bitbang is more than adequate for IO expanders.
//
// SERCOM4 / TMC429 SPI:
//   The janelia-arduino/TMC429 library hardcodes use of the global `SPI`
//   object. We derive TMC429Custom from TMC429, overriding the protected
//   virtual spiBegin/spiBeginTransaction/spiEndTransaction/spiTransfer
//   methods to use our SERCOM4 SPI instance (SPI_MC) instead.
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
static constexpr uint8_t PIN_Z1_DIR  = A0;
static constexpr uint8_t PIN_Z1_STEP = A1;
static constexpr uint8_t PIN_M3_DIR  = A2;
static constexpr uint8_t PIN_M3_STEP = A3;
static constexpr uint8_t PIN_Z2_DIR  = A4;
static constexpr uint8_t PIN_Z2_STEP = A5;

// I/O expander nRESETs — independent pins per MCP23017. Active LOW,
// externally pulled HIGH. We hold them as INPUT during normal operation
// and only drive LOW transiently from cmdIoReset().
static constexpr uint8_t PIN_TRAY_NRESET  = 8; // TRAY_IO_EXPANDER  (0x20)
static constexpr uint8_t PIN_EXTRA_NRESET = 9; // EXTRA_IO_EXPANDER (0x21)

// TMC429 motion controller SPI chip select (active LOW, plain GPIO)
static constexpr uint8_t PIN_MC_NCS = 6;

// TMC429 SPI on SERCOM4
static constexpr uint8_t PIN_MC_MOSI = 7;  // PB12 SERCOM4 PAD[0]
static constexpr uint8_t PIN_MC_SCK  = 4;  // PB13 SERCOM4 PAD[1]
static constexpr uint8_t PIN_MC_MISO = 5;  // PB14 SERCOM4 PAD[2]

// Drawer handle: bi-color LED via DRV8231A H-bridge.
// The PCB has a hardware AND-gate interlock that prevents both inputs being
// HIGH simultaneously (which would otherwise active-brake and short the LED).
//   IN1=1, IN2=0 -> RED
//   IN1=0, IN2=1 -> GREEN
//   IN1=0, IN2=0 -> OFF (coast)
static constexpr uint8_t PIN_DRAWER_LED_IN1 = 11;
static constexpr uint8_t PIN_DRAWER_LED_IN2 = 10;

// I2C pins (driven by SoftI2C, NOT the Wire library)
static constexpr uint8_t PIN_I2C_SDA = SDA; // PB2 on Metro M4
static constexpr uint8_t PIN_I2C_SCL = SCL; // PB3 on Metro M4

// UART pins (Uart objects manage these; listed for documentation)
static constexpr uint8_t PIN_Z1_UART_TX = 13;  // PA16 SERCOM1 PAD[0]
static constexpr uint8_t PIN_Z1_UART_RX = 12;  // PA17 SERCOM1 PAD[1]
static constexpr uint8_t PIN_Z2_UART_TX = 3;   // PB16 SERCOM5 PAD[0]
static constexpr uint8_t PIN_Z2_UART_RX = 2;   // PB17 SERCOM5 PAD[1]
// M3 uses Serial1: TX=D1 (PA22 SERCOM3 PAD[0]), RX=D0 (PA23 SERCOM3 PAD[1])

// ============================================================================
// MCP23017 I/O Expander Configuration
// ============================================================================

// I2C addresses
static constexpr uint8_t TRAY_IO_EXPANDER_ADDR  = 0x20; // A2=A1=A0=GND
static constexpr uint8_t EXTRA_IO_EXPANDER_ADDR = 0x21; // A0=VDD (others GND)

// MCP23017 register addresses (IOCON.BANK=0, the power-on default)
static constexpr uint8_t MCP_REG_IODIRA = 0x00;
static constexpr uint8_t MCP_REG_IODIRB = 0x01;
static constexpr uint8_t MCP_REG_GPIOA  = 0x12;
static constexpr uint8_t MCP_REG_GPIOB  = 0x13;
static constexpr uint8_t MCP_REG_OLATA  = 0x14;
static constexpr uint8_t MCP_REG_OLATB  = 0x15;

// --- EXTRA_IO_EXPANDER (0x21) bit layout ---
// I2C address: A2=GND, A1=GND, A0=3V3  -> 0x21
//
// PORT A:
//   GPA0 = Z_BOT           limit switch (input)
//   GPA1 = SPARE_TOP       limit switch for M3 (input)
//   GPA2 = SPARE_BOT       limit switch for M3 (input)
//   GPA3 = MOTOR_Z1_DIAG   TMC2209 Z1 DIAG (input)
//   GPA4 = MOTOR_Z2_DIAG   TMC2209 Z2 DIAG (input)
//   GPA5 = MOTOR_M3_DIAG   TMC2209 M3 DIAG (input)
//   GPA6 = MCU_Z_EN        SHARED enable for Z1 + Z2 (output)
//   GPA7 = MCU_M3_EN       enable for M3 (output)
//
// PORT B:
//   GPB0 = DRAWER_SW       drawer handle switch (input)
//   GPB1 = T1SW1           tray 1 switch 1 (input)
//   GPB2 = T1SW2           tray 1 switch 2 (input)
//   GPB3 = T2SW1           tray 2 switch 1 (input)
//   GPB4 = T2SW2           tray 2 switch 2 (input)
//   GPB5 = T3SW1           tray 3 switch 1 (input)
//   GPB6 = T3SW2           tray 3 switch 2 (input)
//   GPB7 = Z_TOP           limit switch shared by Z1 + Z2 (input)
//
// Z1 and Z2 share a single enable line because they drive the same physical
// gantry (like Prusa Z motors). `enable z1` and `enable z2` both toggle GPA6.
//
// Enable polarity: despite the "MCU_Z_EN" / "MCU_M3_EN" labels, the MCP23017
// output runs DIRECTLY to the TMC2209's EN pin (no inverter), and the TMC2209
// EN pin is active-LOW. So on this board LOW = driver enabled, HIGH = driver
// disabled, and the safe boot state is HIGH (bits 6+7 set in OLATA => 0xC0).
// If a future PCB rev adds an inverter, flip EN_ACTIVE_HIGH below.
static constexpr bool EN_ACTIVE_HIGH = false;

// GPA bit positions
static constexpr uint8_t BIT_Z_BOT     = 0; // GPA0 (input)
static constexpr uint8_t BIT_SPARE_TOP = 1; // GPA1 (input)
static constexpr uint8_t BIT_SPARE_BOT = 2; // GPA2 (input)
static constexpr uint8_t BIT_Z1_DIAG   = 3; // GPA3 (input)
static constexpr uint8_t BIT_Z2_DIAG   = 4; // GPA4 (input)
static constexpr uint8_t BIT_M3_DIAG   = 5; // GPA5 (input)
static constexpr uint8_t BIT_Z_EN      = 6; // GPA6 (output, shared Z1+Z2)
static constexpr uint8_t BIT_M3_EN     = 7; // GPA7 (output)

// GPB bit positions
static constexpr uint8_t BIT_DRAWER_BTN = 0; // GPB0 (input)
static constexpr uint8_t BIT_T1SW1      = 1; // GPB1 (input)
static constexpr uint8_t BIT_T1SW2      = 2; // GPB2 (input)
static constexpr uint8_t BIT_T2SW1      = 3; // GPB3 (input)
static constexpr uint8_t BIT_T2SW2      = 4; // GPB4 (input)
static constexpr uint8_t BIT_T3SW1      = 5; // GPB5 (input)
static constexpr uint8_t BIT_T3SW2      = 6; // GPB6 (input)
static constexpr uint8_t BIT_Z_TOP      = 7; // GPB7 (input)

// Port A direction mask: 1 = input, 0 = output.
// Bits 0..5 are inputs; bits 6..7 are motor enable outputs.
static constexpr uint8_t EXTRA_IODIRA = 0b00111111; // 0x3F
// Port B: all inputs
static constexpr uint8_t EXTRA_IODIRB = 0xFF;

// Safe initial OLATA: motor enables in DISABLED state.
// With EN_ACTIVE_HIGH=true, disabled == LOW, so clear bits 6 and 7.
static constexpr uint8_t EXTRA_OLATA_SAFE =
    EN_ACTIVE_HIGH ? 0x00
                   : (uint8_t)((1 << BIT_Z_EN) | (1 << BIT_M3_EN));

// --- TRAY_IO_EXPANDER (0x20) bit layout ---
// 12 solenoid outputs, organized by tray (3 trays × 4 valves each).
//   GPB0 = T1V1   GPB4 = T2V1   GPA0 = T3V1
//   GPB1 = T1V2   GPB5 = T2V2   GPA1 = T3V2
//   GPB2 = T1V3   GPB6 = T2V3   GPA2 = T3V3
//   GPB3 = T1V4   GPB7 = T2V4   GPA3 = T3V4
//   GPA4..GPA7 = nc (no connection)
//
// CLI index (`sol N`) is physical tray/valve order so testing walks
// T1V1, T1V2, ... T3V4 naturally:
//   sol 0..3  -> T1V1..T1V4 (GPB0..GPB3)
//   sol 4..7  -> T2V1..T2V4 (GPB4..GPB7)
//   sol 8..11 -> T3V1..T3V4 (GPA0..GPA3)
static constexpr uint8_t TRAY_IODIRA = 0xF0; // GPA0..GPA3 outputs (T3V1..T3V4), GPA4..GPA7 inputs (nc)
static constexpr uint8_t TRAY_IODIRB = 0x00; // GPB all outputs (T1V1..T2V4)

// Solenoids are driven HIGH to fire. Default all OFF (0).
static constexpr uint8_t TRAY_OLATA_SAFE = 0x00;
static constexpr uint8_t TRAY_OLATB_SAFE = 0x00;

// Number of solenoids exposed
static constexpr uint8_t NUM_SOLENOIDS = 12;

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
static constexpr uint8_t LED_RSP_BAD_FMT    = 0xE2;
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
// M3 Motor uses Serial1 (SERCOM3, TX=D1/PA22, RX=D0/PA23) from the BSP.
// Z1, Z2 need custom Uart objects. Wire is lib_ignored in platformio.ini so
// SERCOM5's interrupt handlers are free for Z2.

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
// SERCOM4 is used for the TMC429 SPI master; SPI master mode typically runs
// blocking without interrupts, so we do not define SERCOM4 handlers here.
// SERCOM5 handlers are normally provided by Wire.cpp, but we excluded Wire
// via lib_ignore so we can define them here for Z2 UART.

extern "C" {

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
// Custom SPI on SERCOM4 for TMC429
// ============================================================================

// MOSI=PB12 (PAD0), SCK=PB13 (PAD1), MISO=PB14 (PAD2)
// DOPO: MOSI on PAD0, SCK on PAD1 -> SPI_PAD_0_SCK_1
// DIPO: MISO on PAD2              -> SERCOM_RX_PAD_2
static SPIClass SPI_MC(&sercom4,
                        PIN_MC_MISO, PIN_MC_SCK, PIN_MC_MOSI,
                        SPI_PAD_0_SCK_1, SERCOM_RX_PAD_2);

// Override TMC429 library's SPI hook methods to use SPI_MC.
class TMC429Custom : public TMC429 {
public:
    TMC429Custom(SPIClass &spi) : _spi(spi) {}
protected:
    void spiBegin() override { _spi.begin(); }
    void spiBeginTransaction(SPISettings s) override { _spi.beginTransaction(s); }
    void spiEndTransaction() override { _spi.endTransaction(); }
    uint8_t spiTransfer(uint8_t b) override { return _spi.transfer(b); }
private:
    SPIClass &_spi;
};

// ============================================================================
// TMC2209 Driver Instances
// ============================================================================

static TMC2209 tmcZ1;
static TMC2209 tmcZ2;
static TMC2209 tmcM3;

static bool tmcZ1Initialized = false;
static bool tmcZ2Initialized = false;
static bool tmcM3Initialized = false;

// ============================================================================
// TMC429 Motion Controller Instance
// ============================================================================
// CLI mapping: z1=0 (M1), m3=1 (M2), z2=2 (M3)  — matches PCB routing

static TMC429Custom motionController(SPI_MC);
static bool mcInitialized = false;

// ============================================================================
// I2C and I/O Expanders
// ============================================================================

static SoftI2C i2c(PIN_I2C_SDA, PIN_I2C_SCL);
static bool ioExpanderExtraInit = false;
static bool ioExpanderTrayInit  = false;

// Cached OLAT values so we can update a single bit without a read-modify-write.
static uint8_t extraOlatA = EXTRA_OLATA_SAFE;
static uint8_t trayOlatA  = TRAY_OLATA_SAFE;
static uint8_t trayOlatB  = TRAY_OLATB_SAFE;

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
// Drawer-handle commands
static void cmdDrawerLed(const char *color);
static void cmdDrawerButton();
// Solenoid commands
static void cmdSolenoid(const char *indexStr, const char *state);
static void cmdSolenoidAll(const char *state);
static void solLabel(int idx, char *out);
// Lookup helpers
static TMC2209 *lookupTmc(const char *motor);
static bool     isTmcInitialized(const char *motor);
static void     getAxisPins(const char *axis, uint8_t &stepPin, uint8_t &dirPin,
                            bool &valid);
static int      enableBitForAxis(const char *axis); // returns EXTRA GPA bit or -1

// ============================================================================
// setup()
// ============================================================================

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    Serial.println();
    Serial.println(F("============================================================"));
    Serial.println(F("  BSU v4 — Motor, Sensor, Drawer & Solenoid Test CLI"));
    Serial.println(F("  Board: Adafruit Metro M4 Express (ATSAMD51J19A)"));
    Serial.println(F("============================================================"));

    // Motor STEP pins (default LOW)
    pinMode(PIN_Z1_STEP, OUTPUT); digitalWrite(PIN_Z1_STEP, LOW);
    pinMode(PIN_Z2_STEP, OUTPUT); digitalWrite(PIN_Z2_STEP, LOW);
    pinMode(PIN_M3_STEP, OUTPUT); digitalWrite(PIN_M3_STEP, LOW);

    // Motor DIR pins (default LOW)
    pinMode(PIN_Z1_DIR, OUTPUT); digitalWrite(PIN_Z1_DIR, LOW);
    pinMode(PIN_Z2_DIR, OUTPUT); digitalWrite(PIN_Z2_DIR, LOW);
    pinMode(PIN_M3_DIR, OUTPUT); digitalWrite(PIN_M3_DIR, LOW);

    // Drawer LED H-bridge inputs default LOW (LED off)
    pinMode(PIN_DRAWER_LED_IN1, OUTPUT); digitalWrite(PIN_DRAWER_LED_IN1, LOW);
    pinMode(PIN_DRAWER_LED_IN2, OUTPUT); digitalWrite(PIN_DRAWER_LED_IN2, LOW);

    // I/O expander nRESETs: start as input (external pull-ups keep them HIGH)
    pinMode(PIN_TRAY_NRESET,  INPUT);
    pinMode(PIN_EXTRA_NRESET, INPUT);

    // TMC429 chip select HIGH (inactive) before SPI configured
    pinMode(PIN_MC_NCS, OUTPUT); digitalWrite(PIN_MC_NCS, HIGH);

    Serial.println(F("[BOOT] GPIO pins configured."));
    Serial.println(F("[BOOT]   STEP/DIR pins: LOW"));
    Serial.println(F("[BOOT]   Drawer LED:    OFF"));
    Serial.println(F("[BOOT]   TMC429 nCS:    HIGH (inactive)"));

    // Initialize bitbang I2C and MCP23017s
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
        if (tokenCount < 2) { Serial.println(F("[ERROR] Usage: enable <z1|z2|m3>")); return; }
        cmdEnable(tokens[1]);
    }
    else if (strcasecmp(cmd, "disable") == 0) {
        if (tokenCount < 2) { Serial.println(F("[ERROR] Usage: disable <z1|z2|m3>")); return; }
        cmdDisable(tokens[1]);
    }
    else if (strcasecmp(cmd, "tmc_init") == 0) {
        if (tokenCount < 2) { Serial.println(F("[ERROR] Usage: tmc_init <z1|z2|m3> [baud]")); return; }
        cmdTmcInit(tokens[1], tokenCount >= 3 ? tokens[2] : nullptr);
    }
    else if (strcasecmp(cmd, "tmc_status") == 0) {
        if (tokenCount < 2) { Serial.println(F("[ERROR] Usage: tmc_status <z1|z2|m3>")); return; }
        cmdTmcStatus(tokens[1]);
    }
    else if (strcasecmp(cmd, "tmc_current") == 0) {
        if (tokenCount < 3) { Serial.println(F("[ERROR] Usage: tmc_current <z1|z2|m3> <percent>")); return; }
        cmdTmcCurrent(tokens[1], tokens[2]);
    }
    else if (strcasecmp(cmd, "tmc_hold") == 0) {
        if (tokenCount < 3) { Serial.println(F("[ERROR] Usage: tmc_hold <z1|z2|m3> <percent>")); return; }
        cmdTmcHold(tokens[1], tokens[2]);
    }
    else if (strcasecmp(cmd, "tmc_microstep") == 0) {
        if (tokenCount < 3) { Serial.println(F("[ERROR] Usage: tmc_microstep <z1|z2|m3> <1..256>")); return; }
        cmdTmcMicrostep(tokens[1], tokens[2]);
    }
    else if (strcasecmp(cmd, "tmc_stealthchop") == 0) {
        if (tokenCount < 3) { Serial.println(F("[ERROR] Usage: tmc_stealthchop <z1|z2|m3> <on|off>")); return; }
        cmdTmcStealthchop(tokens[1], tokens[2]);
    }
    else if (strcasecmp(cmd, "tmc_enable") == 0) {
        if (tokenCount < 2) { Serial.println(F("[ERROR] Usage: tmc_enable <z1|z2|m3>")); return; }
        cmdTmcEnable(tokens[1]);
    }
    else if (strcasecmp(cmd, "tmc_disable") == 0) {
        if (tokenCount < 2) { Serial.println(F("[ERROR] Usage: tmc_disable <z1|z2|m3>")); return; }
        cmdTmcDisable(tokens[1]);
    }
    else if (strcasecmp(cmd, "dir") == 0) {
        if (tokenCount < 3) { Serial.println(F("[ERROR] Usage: dir <z1|z2|m3> <0|1>")); return; }
        cmdDir(tokens[1], tokens[2]);
    }
    else if (strcasecmp(cmd, "step") == 0) {
        if (tokenCount < 4) { Serial.println(F("[ERROR] Usage: step <z1|z2|m3> <count> <delay_us>")); return; }
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
        if (tokenCount < 5) { Serial.println(F("[ERROR] Usage: mc_limits <z1|z2|m3> <vmin_hz> <vmax_hz> <amax_hz_per_s>")); return; }
        cmdMcLimits(tokens[1], tokens[2], tokens[3], tokens[4]);
    }
    else if (strcasecmp(cmd, "mc_ramp") == 0) {
        if (tokenCount < 2) { Serial.println(F("[ERROR] Usage: mc_ramp <z1|z2|m3>")); return; }
        cmdMcRamp(tokens[1]);
    }
    else if (strcasecmp(cmd, "mc_velocity") == 0) {
        if (tokenCount < 2) { Serial.println(F("[ERROR] Usage: mc_velocity <z1|z2|m3>")); return; }
        cmdMcVelocity(tokens[1]);
    }
    else if (strcasecmp(cmd, "mc_hold") == 0) {
        if (tokenCount < 2) { Serial.println(F("[ERROR] Usage: mc_hold <z1|z2|m3>")); return; }
        cmdMcHold(tokens[1]);
    }
    else if (strcasecmp(cmd, "mc_target") == 0) {
        if (tokenCount < 3) { Serial.println(F("[ERROR] Usage: mc_target <z1|z2|m3> <position>")); return; }
        cmdMcTarget(tokens[1], tokens[2]);
    }
    else if (strcasecmp(cmd, "mc_vtarget") == 0) {
        if (tokenCount < 3) { Serial.println(F("[ERROR] Usage: mc_vtarget <z1|z2|m3> <velocity_hz>")); return; }
        cmdMcVtarget(tokens[1], tokens[2]);
    }
    else if (strcasecmp(cmd, "mc_pos") == 0) {
        if (tokenCount < 2) { Serial.println(F("[ERROR] Usage: mc_pos <z1|z2|m3>")); return; }
        cmdMcPos(tokens[1]);
    }
    else if (strcasecmp(cmd, "mc_setpos") == 0) {
        if (tokenCount < 3) { Serial.println(F("[ERROR] Usage: mc_setpos <z1|z2|m3> <position>")); return; }
        cmdMcSetpos(tokens[1], tokens[2]);
    }
    else if (strcasecmp(cmd, "mc_stop") == 0) {
        if (tokenCount < 2) { Serial.println(F("[ERROR] Usage: mc_stop <z1|z2|m3>")); return; }
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
        if (tokenCount < 3) { Serial.println(F("[ERROR] Usage: mc_leftstop <z1|z2|m3> <on|off>")); return; }
        cmdMcLeftstop(tokens[1], tokens[2]);
    }
    else if (strcasecmp(cmd, "mc_rightstop") == 0) {
        if (tokenCount < 3) { Serial.println(F("[ERROR] Usage: mc_rightstop <z1|z2|m3> <on|off>")); return; }
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
    // --- Drawer-handle commands ---
    else if (strcasecmp(cmd, "drawer_led") == 0) {
        if (tokenCount < 2) { Serial.println(F("[ERROR] Usage: drawer_led <off|red|green>")); return; }
        cmdDrawerLed(tokens[1]);
    }
    else if (strcasecmp(cmd, "drawer_button") == 0) {
        cmdDrawerButton();
    }
    // --- Solenoid commands ---
    else if (strcasecmp(cmd, "sol") == 0) {
        if (tokenCount < 3) { Serial.println(F("[ERROR] Usage: sol <0..11> <on|off>")); return; }
        cmdSolenoid(tokens[1], tokens[2]);
    }
    else if (strcasecmp(cmd, "sol_all") == 0) {
        if (tokenCount < 2) { Serial.println(F("[ERROR] Usage: sol_all <on|off>")); return; }
        cmdSolenoidAll(tokens[1]);
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
        "--- I/O Expanders (MCP23017 @ 0x20 + 0x21) ---\n"
        "  io_init                               Initialize both MCP23017s\n"
        "  io_reset                              Hardware-reset both MCP23017s\n"
        "  io_read                               Read all expander inputs\n"
        "\n"
        "--- MCU Pin Readback ---\n"
        "  mcu_read                              Read all MCU output pin states\n"
        "\n"
        "--- Motor Enable/Disable (via EXTRA_IO_EXPANDER GPA6..GPA7) ---\n"
        "  enable  <z|m3>                        Drive EN active (driver ON)\n"
        "  disable <z|m3>                        Drive EN inactive (driver OFF)\n"
        "                                        Z1 and Z2 share a single EN line;\n"
        "                                        'z1'/'z2' are accepted as aliases for 'z'.\n"
        "\n"
        "--- TMC2209 UART ---\n"
        "  tmc_init       <z1|z2|m3> [baud]      Init UART (default 115200)\n"
        "  tmc_status     <z1|z2|m3>             Read TMC2209 status/settings\n"
        "  tmc_current    <z1|z2|m3> <percent>   Set run current (0-100%)\n"
        "  tmc_hold       <z1|z2|m3> <percent>   Set hold current (0-100%)\n"
        "  tmc_microstep  <z1|z2|m3> <n>         Set microsteps (1..256)\n"
        "  tmc_stealthchop <z1|z2|m3> <on|off>   Toggle StealthChop mode\n"
        "  tmc_enable     <z1|z2|m3>             Software-enable via UART\n"
        "  tmc_disable    <z1|z2|m3>             Software-disable via UART\n"
        "\n"
        "--- Direct Pin Control ---\n"
        "  dir  <z1|z2|m3> <0|1>                 Set direction pin level\n"
        "  step <z1|z2|m3> <count> <delay_us>    Pulse STEP pin <count> times\n"
        "\n"
        "--- TMC429 Motion Controller (mc_) ---\n"
        "  mc_init [clock_mhz]                   Init SPI + TMC429 (default 32)\n"
        "  mc_status                             Read version + status flags\n"
        "  mc_limits <z1|z2|m3> <vmin> <vmax> <amax>\n"
        "                                        Set limits (all Hz or Hz/s)\n"
        "  mc_ramp <z1|z2|m3>                    Ramp mode (position w/ trapezoid)\n"
        "  mc_velocity <z1|z2|m3>                Velocity mode (continuous)\n"
        "  mc_hold <z1|z2|m3>                    Hold mode (lock position)\n"
        "  mc_target <z1|z2|m3> <position>       Set target position (ramp mode)\n"
        "  mc_vtarget <z1|z2|m3> <velocity_hz>   Set target velocity (vel mode)\n"
        "  mc_pos <z1|z2|m3>                     Read actual + target position\n"
        "  mc_setpos <z1|z2|m3> <position>       Write actual position register\n"
        "  mc_stop <z1|z2|m3>                    Stop one axis\n"
        "  mc_stopall                            Stop all three axes\n"
        "  mc_switches                           Read all limit switch states\n"
        "  mc_swpol <high|low>                   Set switch active polarity\n"
        "  mc_leftstop <z1|z2|m3> <on|off>       Left switch auto-stop\n"
        "  mc_rightstop <z1|z2|m3> <on|off>      Right switch auto-stop\n"
        "  mc_rightsw <on|off>                   Enable right switch inputs\n"
        "\n"
        "--- LED MCU (I2C slave at 0x30) ---\n"
        "  led_set <ring|all> <anim> <r> <g> <b> <brightness>\n"
        "                                        anim: off|static|spin|bounce|pulse\n"
        "                                        r/g/b: 0-255, brightness: 0-255\n"
        "  led_status <ring>                     Query ring animation state\n"
        "  led_config <param> <value>            Set persistent config param\n"
        "                                        params: num_rings, leds_per_ring,\n"
        "                                        spin_width, bounce_width,\n"
        "                                        spin_speed, bounce_speed,\n"
        "                                        pulse_speed (speeds in ms)\n"
        "  led_getconfig                         Read all config parameters\n"
        "\n"
        "--- Drawer Handle ---\n"
        "  drawer_led <off|red|green>            Drive bi-color LED\n"
        "  drawer_button                         Read drawer button state\n"
        "\n"
        "--- Solenoids (TRAY_IO_EXPANDER @ 0x20) ---\n"
        "  sol <0..11> <on|off>                  Drive single solenoid\n"
        "                                        0-3=T1V1-T1V4, 4-7=T2V1-T2V4, 8-11=T3V1-T3V4\n"
        "  sol_all <on|off>                      Drive all 12 solenoids\n"
        "\n"
        "NOTES:\n"
        "  - All three motors (z1, z2, m3) use independent STEP/DIR/ENABLE.\n"
        "  - Motor enables are on EXTRA_IO_EXPANDER: 'io_init' first.\n"
        "  - Motors are DISABLED at boot. Run 'enable' before 'step'.\n"
        "  - You must 'tmc_init' before any tmc_* command for that motor.\n"
        "==============================================================\n"
    ));
}

// ============================================================================
// Command: io_init  (both MCP23017s)
// ============================================================================

static bool initExtraExpander() {
    Serial.println(F("[IO] Probing EXTRA_IO_EXPANDER at 0x21..."));
    if (!i2c.probe(EXTRA_IO_EXPANDER_ADDR)) {
        Serial.println(F("[IO] ERROR: EXTRA_IO_EXPANDER (0x21) not found!"));
        ioExpanderExtraInit = false;
        return false;
    }
    Serial.println(F("[IO]   Device ACKed at 0x21."));

    // IMPORTANT: write OLATA (to the DISABLED state) BEFORE IODIRA, so the EN
    // lines never briefly drive to the enabled polarity at the moment the
    // pins transition from input to output.
    bool ok = true;
    extraOlatA = EXTRA_OLATA_SAFE;
    ok &= i2c.writeRegister(EXTRA_IO_EXPANDER_ADDR, MCP_REG_OLATA,  extraOlatA);
    ok &= i2c.writeRegister(EXTRA_IO_EXPANDER_ADDR, MCP_REG_OLATB,  0x00);
    ok &= i2c.writeRegister(EXTRA_IO_EXPANDER_ADDR, MCP_REG_IODIRA, EXTRA_IODIRA);
    ok &= i2c.writeRegister(EXTRA_IO_EXPANDER_ADDR, MCP_REG_IODIRB, EXTRA_IODIRB);
    if (!ok) {
        Serial.println(F("[IO] ERROR: Failed to configure 0x21!"));
        ioExpanderExtraInit = false;
        return false;
    }

    ioExpanderExtraInit = true;
    Serial.println(F("[IO] EXTRA_IO_EXPANDER (0x21) initialized."));
    Serial.println(F("[IO]   GPA0-GPA2: Z_BOT / SPARE_TOP / SPARE_BOT limit switches (inputs)"));
    Serial.println(F("[IO]   GPA3-GPA5: DIAG Z1/Z2/M3 (inputs)"));
    Serial.print  (F("[IO]   GPA6:      Z  EN shared Z1+Z2 (output, active-"));
    Serial.print(EN_ACTIVE_HIGH ? "HIGH" : "LOW");
    Serial.println(F(")"));
    Serial.print  (F("[IO]   GPA7:      M3 EN (output, active-"));
    Serial.print(EN_ACTIVE_HIGH ? "HIGH" : "LOW");
    Serial.println(F(")"));
    Serial.println(F("[IO]   GPB0:      DRAWER_SW (input)"));
    Serial.println(F("[IO]   GPB1-GPB6: T1SW1/T1SW2/T2SW1/T2SW2/T3SW1/T3SW2 (inputs)"));
    Serial.println(F("[IO]   GPB7:      Z_TOP shared Z1+Z2 (input)"));
    return true;
}

static bool initTrayExpander() {
    Serial.println(F("[IO] Probing TRAY_IO_EXPANDER at 0x20..."));
    if (!i2c.probe(TRAY_IO_EXPANDER_ADDR)) {
        Serial.println(F("[IO] ERROR: TRAY_IO_EXPANDER (0x20) not found!"));
        ioExpanderTrayInit = false;
        return false;
    }
    Serial.println(F("[IO]   Device ACKed at 0x20."));

    // Solenoids default OFF (OLAT = 0). Write OLATs before IODIRs so pins
    // never transition to output with undefined drive.
    bool ok = true;
    trayOlatA = TRAY_OLATA_SAFE;
    trayOlatB = TRAY_OLATB_SAFE;
    ok &= i2c.writeRegister(TRAY_IO_EXPANDER_ADDR, MCP_REG_OLATA,  trayOlatA);
    ok &= i2c.writeRegister(TRAY_IO_EXPANDER_ADDR, MCP_REG_OLATB,  trayOlatB);
    ok &= i2c.writeRegister(TRAY_IO_EXPANDER_ADDR, MCP_REG_IODIRA, TRAY_IODIRA);
    ok &= i2c.writeRegister(TRAY_IO_EXPANDER_ADDR, MCP_REG_IODIRB, TRAY_IODIRB);
    if (!ok) {
        Serial.println(F("[IO] ERROR: Failed to configure 0x20!"));
        ioExpanderTrayInit = false;
        return false;
    }

    ioExpanderTrayInit = true;
    Serial.println(F("[IO] TRAY_IO_EXPANDER (0x20) initialized."));
    Serial.println(F("[IO]   GPB0-GPB3: T1V1-T1V4 (sol 0-3)  (outputs, LOW=off)"));
    Serial.println(F("[IO]   GPB4-GPB7: T2V1-T2V4 (sol 4-7)  (outputs, LOW=off)"));
    Serial.println(F("[IO]   GPA0-GPA3: T3V1-T3V4 (sol 8-11) (outputs, LOW=off)"));
    Serial.println(F("[IO]   GPA4-GPA7: nc (inputs)"));
    return true;
}

static void cmdIoInit() {
    initExtraExpander();
    initTrayExpander();
}

// ============================================================================
// Command: io_reset  (independent nRESET lines: TRAY=D8, EXTRA=D9)
// ============================================================================

static void cmdIoReset() {
    Serial.println(F("[IO] Hardware-resetting both MCP23017s..."));
    Serial.println(F("[IO]   Driving TRAY nRESET (D8) and EXTRA nRESET (D9) LOW for 10 ms..."));

    pinMode(PIN_TRAY_NRESET,  OUTPUT); digitalWrite(PIN_TRAY_NRESET,  LOW);
    pinMode(PIN_EXTRA_NRESET, OUTPUT); digitalWrite(PIN_EXTRA_NRESET, LOW);
    delay(10);
    digitalWrite(PIN_TRAY_NRESET,  HIGH);
    digitalWrite(PIN_EXTRA_NRESET, HIGH);
    pinMode(PIN_TRAY_NRESET,  INPUT); // Release to external pull-up
    pinMode(PIN_EXTRA_NRESET, INPUT);

    Serial.println(F("[IO]   nRESETs released (external pull-ups)."));
    Serial.println(F("[IO]   Waiting 50 ms for startup..."));
    delay(50);

    Serial.println(F("[IO]   Re-initializing..."));
    cmdIoInit();
}

// ============================================================================
// Command: io_read
// ============================================================================

static void cmdIoRead() {
    if (!ioExpanderExtraInit && !ioExpanderTrayInit) {
        Serial.println(F("[IO] ERROR: No expanders initialized. Run 'io_init'."));
        return;
    }

    // --- EXTRA (0x21) ---
    if (ioExpanderExtraInit) {
        uint8_t gpioA = 0, gpioB = 0;
        if (!i2c.readRegister16(EXTRA_IO_EXPANDER_ADDR, MCP_REG_GPIOA, gpioA, gpioB)) {
            Serial.println(F("[IO] ERROR: Failed to read 0x21 GPIO registers!"));
        } else {
            Serial.println(F("[IO] === EXTRA_IO_EXPANDER (0x21) ==="));
            Serial.print(F("[IO]   Raw: GPIOA=0x")); Serial.print(gpioA, HEX);
            Serial.print(F(", GPIOB=0x")); Serial.println(gpioB, HEX);

            Serial.println(F("[IO]   --- Limit switches ---"));
            Serial.print(F("[IO]     Z_TOP     (GPB7): ")); Serial.println((gpioB >> BIT_Z_TOP)     & 1 ? "TRIGGERED" : "open");
            Serial.print(F("[IO]     Z_BOT     (GPA0): ")); Serial.println((gpioA >> BIT_Z_BOT)     & 1 ? "TRIGGERED" : "open");
            Serial.print(F("[IO]     SPARE_TOP (GPA1): ")); Serial.println((gpioA >> BIT_SPARE_TOP) & 1 ? "TRIGGERED" : "open");
            Serial.print(F("[IO]     SPARE_BOT (GPA2): ")); Serial.println((gpioA >> BIT_SPARE_BOT) & 1 ? "TRIGGERED" : "open");

            Serial.println(F("[IO]   --- TMC2209 DIAG ---"));
            Serial.print(F("[IO]     Z1 DIAG (GPA3): ")); Serial.println((gpioA >> BIT_Z1_DIAG) & 1 ? "HIGH" : "LOW");
            Serial.print(F("[IO]     Z2 DIAG (GPA4): ")); Serial.println((gpioA >> BIT_Z2_DIAG) & 1 ? "HIGH" : "LOW");
            Serial.print(F("[IO]     M3 DIAG (GPA5): ")); Serial.println((gpioA >> BIT_M3_DIAG) & 1 ? "HIGH" : "LOW");

            const char *enHigh = EN_ACTIVE_HIGH ? "HIGH (enabled)"  : "HIGH (disabled)";
            const char *enLow  = EN_ACTIVE_HIGH ? "LOW (disabled)"  : "LOW (enabled)";
            Serial.println(F("[IO]   --- Motor enables (active-HIGH) ---"));
            Serial.print(F("[IO]     Z  EN  (GPA6, shared Z1+Z2): ")); Serial.println((gpioA >> BIT_Z_EN)  & 1 ? enHigh : enLow);
            Serial.print(F("[IO]     M3 EN  (GPA7):                ")); Serial.println((gpioA >> BIT_M3_EN) & 1 ? enHigh : enLow);

            Serial.println(F("[IO]   --- Drawer + tray switches ---"));
            Serial.print(F("[IO]     DRAWER (GPB0): ")); Serial.println((gpioB >> BIT_DRAWER_BTN) & 1 ? "PRESSED" : "released");
            Serial.print(F("[IO]     T1SW1  (GPB1): ")); Serial.println((gpioB >> BIT_T1SW1) & 1 ? "TRIGGERED" : "open");
            Serial.print(F("[IO]     T1SW2  (GPB2): ")); Serial.println((gpioB >> BIT_T1SW2) & 1 ? "TRIGGERED" : "open");
            Serial.print(F("[IO]     T2SW1  (GPB3): ")); Serial.println((gpioB >> BIT_T2SW1) & 1 ? "TRIGGERED" : "open");
            Serial.print(F("[IO]     T2SW2  (GPB4): ")); Serial.println((gpioB >> BIT_T2SW2) & 1 ? "TRIGGERED" : "open");
            Serial.print(F("[IO]     T3SW1  (GPB5): ")); Serial.println((gpioB >> BIT_T3SW1) & 1 ? "TRIGGERED" : "open");
            Serial.print(F("[IO]     T3SW2  (GPB6): ")); Serial.println((gpioB >> BIT_T3SW2) & 1 ? "TRIGGERED" : "open");
        }
    } else {
        Serial.println(F("[IO] EXTRA_IO_EXPANDER (0x21) NOT INITIALIZED."));
    }

    // --- TRAY (0x20) ---
    if (ioExpanderTrayInit) {
        uint8_t gpioA = 0, gpioB = 0;
        if (!i2c.readRegister16(TRAY_IO_EXPANDER_ADDR, MCP_REG_GPIOA, gpioA, gpioB)) {
            Serial.println(F("[IO] ERROR: Failed to read 0x20 GPIO registers!"));
        } else {
            Serial.println(F("[IO] === TRAY_IO_EXPANDER (0x20) ==="));
            Serial.print(F("[IO]   Raw: GPIOA=0x")); Serial.print(gpioA, HEX);
            Serial.print(F(", GPIOB=0x")); Serial.println(gpioB, HEX);
            Serial.print(F("[IO]   Solenoid readback: "));
            for (uint8_t i = 0; i < NUM_SOLENOIDS; i++) {
                // sol 0..7 -> GPB bit i; sol 8..11 -> GPA bit (i-8).
                uint8_t bitSet = (i < 8) ? ((gpioB >> i) & 1) : ((gpioA >> (i - 8)) & 1);
                if (i > 0) Serial.print(F(","));
                char lbl[5]; solLabel(i, lbl);
                Serial.print(lbl);
                Serial.print(F("=")); Serial.print(bitSet ? "ON" : "off");
            }
            Serial.println();
        }
    } else {
        Serial.println(F("[IO] TRAY_IO_EXPANDER (0x20) NOT INITIALIZED."));
    }
}

// ============================================================================
// Command: mcu_read
// ============================================================================

static void cmdMcuRead() {
    Serial.println(F("[MCU] Reading back all MCU-controlled pin states:"));

    Serial.println(F("[MCU] --- Motor STEP Pins ---"));
    Serial.print(F("[MCU]   Z1 STEP (A1): ")); Serial.println(digitalRead(PIN_Z1_STEP) ? "HIGH" : "LOW");
    Serial.print(F("[MCU]   Z2 STEP (A5): ")); Serial.println(digitalRead(PIN_Z2_STEP) ? "HIGH" : "LOW");
    Serial.print(F("[MCU]   M3 STEP (A3): ")); Serial.println(digitalRead(PIN_M3_STEP) ? "HIGH" : "LOW");

    Serial.println(F("[MCU] --- Motor DIR Pins ---"));
    Serial.print(F("[MCU]   Z1 DIR  (A0): ")); Serial.println(digitalRead(PIN_Z1_DIR) ? "HIGH" : "LOW");
    Serial.print(F("[MCU]   Z2 DIR  (A4): ")); Serial.println(digitalRead(PIN_Z2_DIR) ? "HIGH" : "LOW");
    Serial.print(F("[MCU]   M3 DIR  (A2): ")); Serial.println(digitalRead(PIN_M3_DIR) ? "HIGH" : "LOW");

    Serial.println(F("[MCU] --- Drawer LED H-Bridge ---"));
    Serial.print(F("[MCU]   IN1 (D11): ")); Serial.println(digitalRead(PIN_DRAWER_LED_IN1) ? "HIGH" : "LOW");
    Serial.print(F("[MCU]   IN2 (D10): ")); Serial.println(digitalRead(PIN_DRAWER_LED_IN2) ? "HIGH" : "LOW");

    Serial.println(F("[MCU] --- TMC429 SPI ---"));
    Serial.print(F("[MCU]   nCS (D6):  ")); Serial.println(digitalRead(PIN_MC_NCS) ? "HIGH (inactive)" : "LOW (selected)");

    Serial.println(F("[MCU] --- I/O Expander nRESETs ---"));
    Serial.print(F("[MCU]   TRAY  nRST (D8): "));
    Serial.println(digitalRead(PIN_TRAY_NRESET)  ? "HIGH (not in reset)" : "LOW (in reset!)");
    Serial.print(F("[MCU]   EXTRA nRST (D9): "));
    Serial.println(digitalRead(PIN_EXTRA_NRESET) ? "HIGH (not in reset)" : "LOW (in reset!)");
}

// ============================================================================
// enable / disable  (via EXTRA_IO_EXPANDER OLATA)
// ============================================================================

// Returns the EXTRA GPA bit driving the given axis's enable line, or -1 for an
// invalid axis name. The Z axis has a single EN line shared by Z1 and Z2
// (same physical gantry); `z`, `z1`, and `z2` all select BIT_Z_EN. `z` is the
// canonical form; `z1`/`z2` are kept as aliases so the rest of the CLI (step,
// tmc_*, mc_*) can address the physically-independent motors by the same name.
static int enableBitForAxis(const char *axis) {
    if (strcasecmp(axis, "z")  == 0) return BIT_Z_EN;
    if (strcasecmp(axis, "z1") == 0) return BIT_Z_EN;
    if (strcasecmp(axis, "z2") == 0) return BIT_Z_EN;
    if (strcasecmp(axis, "m3") == 0) return BIT_M3_EN;
    return -1;
}

static bool writeExtraOlatA(uint8_t value) {
    if (!i2c.writeRegister(EXTRA_IO_EXPANDER_ADDR, MCP_REG_OLATA, value)) {
        Serial.println(F("[MOTOR] ERROR: I2C write to EXTRA_IO_EXPANDER failed!"));
        return false;
    }
    extraOlatA = value;
    return true;
}

static void cmdEnable(const char *axis) {
    if (!ioExpanderExtraInit) {
        Serial.println(F("[MOTOR] ERROR: EXTRA_IO_EXPANDER not initialized. Run 'io_init'.")); return;
    }
    int bit = enableBitForAxis(axis);
    if (bit < 0) { Serial.println(F("[ERROR] Invalid axis. Use: z or m3 (z1/z2 also accepted)")); return; }

    uint8_t mask = (uint8_t)(1 << bit);
    uint8_t newVal = EN_ACTIVE_HIGH ? (extraOlatA | mask) : (extraOlatA & (uint8_t)~mask);
    if (!writeExtraOlatA(newVal)) return;

    Serial.print(F("[MOTOR] Axis ")); Serial.print(axis);
    Serial.print(F(" ENABLED (EN driven "));
    Serial.print(EN_ACTIVE_HIGH ? "HIGH" : "LOW");
    Serial.println(F(")."));
    if (bit == BIT_Z_EN && strcasecmp(axis, "z") != 0) {
        Serial.println(F("[MOTOR] Note: 'z1' and 'z2' share one EN line — both enabled."));
    }
}

static void cmdDisable(const char *axis) {
    if (!ioExpanderExtraInit) {
        Serial.println(F("[MOTOR] ERROR: EXTRA_IO_EXPANDER not initialized. Run 'io_init'.")); return;
    }
    int bit = enableBitForAxis(axis);
    if (bit < 0) { Serial.println(F("[ERROR] Invalid axis. Use: z or m3 (z1/z2 also accepted)")); return; }

    uint8_t mask = (uint8_t)(1 << bit);
    uint8_t newVal = EN_ACTIVE_HIGH ? (extraOlatA & (uint8_t)~mask) : (extraOlatA | mask);
    if (!writeExtraOlatA(newVal)) return;

    Serial.print(F("[MOTOR] Axis ")); Serial.print(axis);
    Serial.print(F(" DISABLED (EN driven "));
    Serial.print(EN_ACTIVE_HIGH ? "LOW" : "HIGH");
    Serial.println(F(")."));
    if (bit == BIT_Z_EN && strcasecmp(axis, "z") != 0) {
        Serial.println(F("[MOTOR] Note: 'z1' and 'z2' share one EN line — both disabled."));
    }
}

// Query whether the given axis is hardware-enabled. Does not log on its own.
static bool axisHardwareEnabled(const char *axis) {
    int bit = enableBitForAxis(axis);
    if (bit < 0) return false;
    bool bitSet = (extraOlatA & (1 << bit)) != 0;
    return EN_ACTIVE_HIGH ? bitSet : !bitSet;
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

    if (strcasecmp(motor, "z1") == 0) {
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
    else if (strcasecmp(motor, "m3") == 0) {
        tmcM3.setup(Serial1, baud);
        while (Serial1.available()) Serial1.read();
        tmcM3Initialized = true;
        Serial.println(F("[TMC]   Serial1/SERCOM3 (TX=D1/PA22, RX=D0/PA23)."));
    }
    else {
        Serial.println(F("[ERROR] Invalid motor. Use: z1, z2, or m3"));
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
    if (!tmc) { Serial.println(F("[ERROR] Invalid motor. Use: z1, z2, or m3")); return; }
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
    uint8_t stepPin, dirPin;
    bool valid;
    getAxisPins(axis, stepPin, dirPin, valid);
    if (!valid) { Serial.println(F("[ERROR] Invalid axis. Use: z1, z2, or m3")); return; }

    int val = atoi(valStr);
    if (val != 0 && val != 1) { Serial.println(F("[ERROR] Must be 0 or 1.")); return; }

    digitalWrite(dirPin, val ? HIGH : LOW);
    Serial.print(F("[MOTOR] Axis ")); Serial.print(axis);
    Serial.print(F(" DIR set to ")); Serial.println(val ? "HIGH (1)" : "LOW (0)");
}

static void cmdStep(const char *axis, const char *countStr, const char *delayStr) {
    uint8_t stepPin, dirPin;
    bool valid;
    getAxisPins(axis, stepPin, dirPin, valid);
    if (!valid) { Serial.println(F("[ERROR] Invalid axis. Use: z1, z2, or m3")); return; }

    long count = atol(countStr);
    long delayUs = atol(delayStr);
    if (count <= 0) { Serial.println(F("[ERROR] Count must be positive.")); return; }
    if (delayUs < 2) { Serial.println(F("[ERROR] Delay must be >= 2 us.")); return; }

    if (!axisHardwareEnabled(axis)) {
        Serial.println(F("[MOTOR] WARNING: EN pin is in DISABLED state!"));
        Serial.println(F("[MOTOR]   Pulses will be generated but motor will not move."));
        Serial.println(F("[MOTOR]   Run 'enable <axis>' first."));
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

// Helper: convert axis string to TMC429 motor index (0=z1, 1=z2, 2=m3)
static int lookupMcMotor(const char *axis) {
    if (strcasecmp(axis, "z1") == 0) return 0;
    if (strcasecmp(axis, "m3") == 0) return 1;
    if (strcasecmp(axis, "z2") == 0) return 2;
    return -1;
}

static const char *mcMotorName(int m) {
    switch (m) {
        case 0: return "Z1";
        case 1: return "M3";
        case 2: return "Z2";
        default: return "??";
    }
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
    uint8_t clkMhz = 32; // Default: 32 MHz
    if (clkStr) {
        int val = atoi(clkStr);
        if (val < 4 || val > 32) {
            Serial.println(F("[MC] ERROR: Clock must be 4-32 MHz."));
            return;
        }
        clkMhz = (uint8_t)val;
    }

    Serial.print(F("[MC] Initializing TMC429 on SERCOM4 SPI (nCS=D6) with clock="));
    Serial.print(clkMhz);
    Serial.println(F(" MHz..."));

    // setup() will call spiBegin() on our SPI_MC instance via the override.
    motionController.setup(PIN_MC_NCS, clkMhz);

    // After SPI.begin(), re-mux the SPI pins to SERCOM4 (pinPeripheral is a
    // no-op for the default BSP SPI pins, but required for SERCOM4's pads).
    pinPeripheral(PIN_MC_MOSI, PIO_SERCOM);
    pinPeripheral(PIN_MC_SCK,  PIO_SERCOM);
    pinPeripheral(PIN_MC_MISO, PIO_SERCOM);

    if (motionController.communicating()) {
        uint32_t ver = motionController.getVersion();
        Serial.println(F("[MC]   Communication OK."));
        Serial.print(F("[MC]   Version register: 0x"));
        Serial.println(ver, HEX);
        mcInitialized = true;

        Serial.println(F("[MC]   Step/Dir output mode active."));
        Serial.println(F("[MC]   Motor mapping: z1=motor0 (M1), m3=motor1 (M2), z2=motor2 (M3)"));
        Serial.println(F("[MC]   Limit switches: END1=Left, END2=Right"));
    } else {
        Serial.println(F("[MC]   ERROR: TMC429 not responding!"));
        Serial.println(F("[MC]   Check: SPI wiring (D4-D7), clock, power, nCS (D6)."));
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
    Serial.print(F("[MC]   At target pos M0 (z1): ")); Serial.println(st.at_target_position_0 ? "YES" : "no");
    Serial.print(F("[MC]   At target pos M1 (m3): ")); Serial.println(st.at_target_position_1 ? "YES" : "no");
    Serial.print(F("[MC]   At target pos M2 (z2): ")); Serial.println(st.at_target_position_2 ? "YES" : "no");
    Serial.print(F("[MC]   Switch left M0 (z1):   ")); Serial.println(st.switch_left_0 ? "ACTIVE" : "inactive");
    Serial.print(F("[MC]   Switch left M1 (m3):   ")); Serial.println(st.switch_left_1 ? "ACTIVE" : "inactive");
    Serial.print(F("[MC]   Switch left M2 (z2):   ")); Serial.println(st.switch_left_2 ? "ACTIVE" : "inactive");
    Serial.print(F("[MC]   Interrupt:              ")); Serial.println(st.interrupt ? "YES" : "no");

    for (int m = 0; m < 3; m++) {
        Serial.print(F("[MC]   Motor ")); Serial.print(mcMotorName(m));
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
    if (m < 0) { Serial.println(F("[ERROR] Invalid axis. Use: z1, z2, or m3")); return; }

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

    Serial.print(F("[MC]   Actual vmin=")); Serial.print(motionController.getVelocityMinInHz(m));
    Serial.print(F(" Hz, vmax=")); Serial.print(motionController.getVelocityMaxInHz(m));
    Serial.print(F(" Hz, amax=")); Serial.print(motionController.getAccelerationMaxInHzPerS(m));
    Serial.println(F(" Hz/s"));
}

// mc_ramp <axis>
static void cmdMcRamp(const char *axis) {
    if (!requireMc()) return;
    int m = lookupMcMotor(axis);
    if (m < 0) { Serial.println(F("[ERROR] Invalid axis. Use: z1, z2, or m3")); return; }

    motionController.setRampMode(m);
    Serial.print(F("[MC] Axis ")); Serial.print(axis);
    Serial.println(F(" set to RAMP mode (position control with trapezoidal profile)."));
}

// mc_velocity <axis>
static void cmdMcVelocity(const char *axis) {
    if (!requireMc()) return;
    int m = lookupMcMotor(axis);
    if (m < 0) { Serial.println(F("[ERROR] Invalid axis. Use: z1, z2, or m3")); return; }

    motionController.setVelocityMode(m);
    Serial.print(F("[MC] Axis ")); Serial.print(axis);
    Serial.println(F(" set to VELOCITY mode (continuous rotation)."));
}

// mc_hold <axis>
static void cmdMcHold(const char *axis) {
    if (!requireMc()) return;
    int m = lookupMcMotor(axis);
    if (m < 0) { Serial.println(F("[ERROR] Invalid axis. Use: z1, z2, or m3")); return; }

    motionController.setHoldMode(m);
    Serial.print(F("[MC] Axis ")); Serial.print(axis);
    Serial.println(F(" set to HOLD mode (hold current position)."));
}

// mc_target <axis> <position>
static void cmdMcTarget(const char *axis, const char *posStr) {
    if (!requireMc()) return;
    int m = lookupMcMotor(axis);
    if (m < 0) { Serial.println(F("[ERROR] Invalid axis. Use: z1, z2, or m3")); return; }

    int32_t pos = (int32_t)atol(posStr);
    motionController.setTargetPosition(m, pos);
    Serial.print(F("[MC] Axis ")); Serial.print(axis);
    Serial.print(F(" target position set to ")); Serial.println(pos);
}

// mc_vtarget <axis> <velocity_hz>
static void cmdMcVtarget(const char *axis, const char *velStr) {
    if (!requireMc()) return;
    int m = lookupMcMotor(axis);
    if (m < 0) { Serial.println(F("[ERROR] Invalid axis. Use: z1, z2, or m3")); return; }

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
    if (m < 0) { Serial.println(F("[ERROR] Invalid axis. Use: z1, z2, or m3")); return; }

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
    if (m < 0) { Serial.println(F("[ERROR] Invalid axis. Use: z1, z2, or m3")); return; }

    int32_t pos = (int32_t)atol(posStr);
    motionController.setActualPosition(m, pos);
    Serial.print(F("[MC] Axis ")); Serial.print(axis);
    Serial.print(F(" actual position register set to ")); Serial.println(pos);
}

// mc_stop <axis>
static void cmdMcStop(const char *axis) {
    if (!requireMc()) return;
    int m = lookupMcMotor(axis);
    if (m < 0) { Serial.println(F("[ERROR] Invalid axis. Use: z1, z2, or m3")); return; }

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
    Serial.print(F("[MC]   z1 Left  (END1/REF1L): ")); Serial.println(motionController.leftSwitchActive(0)  ? "ACTIVE" : "inactive");
    Serial.print(F("[MC]   z1 Right (END2/REF1R): ")); Serial.println(motionController.rightSwitchActive(0) ? "ACTIVE" : "inactive");
    Serial.print(F("[MC]   m3 Left  (END1/REF2L): ")); Serial.println(motionController.leftSwitchActive(1)  ? "ACTIVE" : "inactive");
    Serial.print(F("[MC]   m3 Right (END2/REF2R): ")); Serial.println(motionController.rightSwitchActive(1) ? "ACTIVE" : "inactive");
    Serial.print(F("[MC]   z2 Left  (END1/REF3L): ")); Serial.println(motionController.leftSwitchActive(2)  ? "ACTIVE" : "inactive");
    Serial.print(F("[MC]   z2 Right (END2/REF3R): ")); Serial.println(motionController.rightSwitchActive(2) ? "ACTIVE" : "inactive");

    Serial.println(F("[MC] Auto-stop configuration:"));
    for (int m = 0; m < 3; m++) {
        Serial.print(F("[MC]   ")); Serial.print(mcMotorName(m));
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
    if (m < 0) { Serial.println(F("[ERROR] Invalid axis. Use: z1, z2, or m3")); return; }

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
    if (m < 0) { Serial.println(F("[ERROR] Invalid axis. Use: z1, z2, or m3")); return; }

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

static void printLedRsp(uint8_t rsp) {
    switch (rsp) {
        case LED_RSP_OK:      Serial.println(F("OK"));              break;
        case LED_RSP_BAD_CMD: Serial.println(F("UNKNOWN COMMAND")); break;
        case LED_RSP_BAD_FMT: Serial.println(F("BAD FORMAT"));      break;
        default: Serial.print(F("UNKNOWN (0x")); Serial.print(rsp, HEX); Serial.println(F(")")); break;
    }
}

static uint8_t parseAnimName(const char *name) {
    if (strcasecmp(name, "off")    == 0) return LED_ANIM_OFF;
    if (strcasecmp(name, "static") == 0) return LED_ANIM_STATIC;
    if (strcasecmp(name, "spin")   == 0) return LED_ANIM_SPIN;
    if (strcasecmp(name, "bounce") == 0) return LED_ANIM_BOUNCE;
    if (strcasecmp(name, "pulse")  == 0) return LED_ANIM_PULSE;
    return 0xFF;
}

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

static void cmdLedSet(char *tokens[], size_t count) {
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

    delay(5);

    uint8_t rsp;
    if (!i2c.readBytes(LED_MCU_ADDR, &rsp, 1)) {
        Serial.println(F("[LED] ERROR: I2C read failed (no ACK from LED MCU)."));
        return;
    }

    Serial.print(F("[LED] Response: ")); printLedRsp(rsp);
}

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
// Drawer-handle Commands
// ============================================================================

static void cmdDrawerLed(const char *color) {
    if (strcasecmp(color, "off") == 0) {
        digitalWrite(PIN_DRAWER_LED_IN1, LOW);
        digitalWrite(PIN_DRAWER_LED_IN2, LOW);
        Serial.println(F("[DRAWER] LED OFF (IN1=LOW, IN2=LOW)."));
    } else if (strcasecmp(color, "red") == 0) {
        // Drive one direction of the H-bridge; make sure other input is LOW
        // BEFORE we raise this one (the external AND gate interlock would
        // gate both sides LOW during the brief overlap otherwise).
        digitalWrite(PIN_DRAWER_LED_IN2, LOW);
        digitalWrite(PIN_DRAWER_LED_IN1, HIGH);
        Serial.println(F("[DRAWER] LED RED (IN1=HIGH, IN2=LOW)."));
    } else if (strcasecmp(color, "green") == 0) {
        digitalWrite(PIN_DRAWER_LED_IN1, LOW);
        digitalWrite(PIN_DRAWER_LED_IN2, HIGH);
        Serial.println(F("[DRAWER] LED GREEN (IN1=LOW, IN2=HIGH)."));
    } else {
        Serial.println(F("[ERROR] Usage: drawer_led <off|red|green>"));
    }
}

static void cmdDrawerButton() {
    if (!ioExpanderExtraInit) {
        Serial.println(F("[DRAWER] ERROR: EXTRA_IO_EXPANDER not initialized. Run 'io_init'."));
        return;
    }

    uint8_t gpioB = 0;
    if (!i2c.readRegister(EXTRA_IO_EXPANDER_ADDR, MCP_REG_GPIOB, gpioB)) {
        Serial.println(F("[DRAWER] ERROR: I2C read failed."));
        return;
    }
    bool pressed = (gpioB >> BIT_DRAWER_BTN) & 1;
    Serial.print(F("[DRAWER] Button: "));
    Serial.println(pressed ? "PRESSED" : "released");
}

// ============================================================================
// Solenoid Commands
// ============================================================================

static bool writeTrayOlat() {
    bool ok = true;
    ok &= i2c.writeRegister(TRAY_IO_EXPANDER_ADDR, MCP_REG_OLATA, trayOlatA);
    ok &= i2c.writeRegister(TRAY_IO_EXPANDER_ADDR, MCP_REG_OLATB, trayOlatB);
    return ok;
}

// Solenoid CLI index -> tray/valve label.
// sol 0..3 = T1V1..T1V4, sol 4..7 = T2V1..T2V4, sol 8..11 = T3V1..T3V4
static void solLabel(int idx, char *out) {
    int tray  = (idx / 4) + 1;
    int valve = (idx % 4) + 1;
    out[0] = 'T'; out[1] = (char)('0' + tray);
    out[2] = 'V'; out[3] = (char)('0' + valve);
    out[4] = '\0';
}

static void cmdSolenoid(const char *indexStr, const char *state) {
    if (!ioExpanderTrayInit) {
        Serial.println(F("[SOL] ERROR: TRAY_IO_EXPANDER not initialized. Run 'io_init'."));
        return;
    }
    int idx = atoi(indexStr);
    if (idx < 0 || idx >= (int)NUM_SOLENOIDS) {
        Serial.print(F("[ERROR] Solenoid index must be 0..")); Serial.println(NUM_SOLENOIDS - 1);
        return;
    }

    bool turnOn;
    if      (strcasecmp(state, "on") == 0)  turnOn = true;
    else if (strcasecmp(state, "off") == 0) turnOn = false;
    else { Serial.println(F("[ERROR] State must be 'on' or 'off'.")); return; }

    // sol 0..7  -> GPB bit idx         (T1V1..T2V4)
    // sol 8..11 -> GPA bit (idx - 8)   (T3V1..T3V4)
    if (idx < 8) {
        uint8_t mask = (uint8_t)(1 << idx);
        trayOlatB = turnOn ? (trayOlatB | mask) : (trayOlatB & (uint8_t)~mask);
    } else {
        uint8_t mask = (uint8_t)(1 << (idx - 8));
        trayOlatA = turnOn ? (trayOlatA | mask) : (trayOlatA & (uint8_t)~mask);
    }

    if (!writeTrayOlat()) {
        Serial.println(F("[SOL] ERROR: I2C write failed."));
        return;
    }

    char lbl[5]; solLabel(idx, lbl);
    Serial.print(F("[SOL] SOL")); Serial.print(idx);
    Serial.print(F(" (")); Serial.print(lbl); Serial.print(F(")"));
    Serial.print(F(" -> ")); Serial.println(turnOn ? "ON" : "off");
}

static void cmdSolenoidAll(const char *state) {
    if (!ioExpanderTrayInit) {
        Serial.println(F("[SOL] ERROR: TRAY_IO_EXPANDER not initialized. Run 'io_init'."));
        return;
    }
    bool turnOn;
    if      (strcasecmp(state, "on") == 0)  turnOn = true;
    else if (strcasecmp(state, "off") == 0) turnOn = false;
    else { Serial.println(F("[ERROR] State must be 'on' or 'off'.")); return; }

    // sol 0..7 -> GPB0..GPB7 (T1V1..T2V4); sol 8..11 -> GPA0..GPA3 (T3V1..T3V4).
    trayOlatB = turnOn ? 0xFF : 0x00;
    trayOlatA = (trayOlatA & 0xF0) | (turnOn ? 0x0F : 0x00);

    if (!writeTrayOlat()) {
        Serial.println(F("[SOL] ERROR: I2C write failed."));
        return;
    }

    Serial.print(F("[SOL] All 12 solenoids -> "));
    Serial.println(turnOn ? "ON" : "off");
}

// ============================================================================
// Helpers
// ============================================================================

static TMC2209 *lookupTmc(const char *motor) {
    if (strcasecmp(motor, "z1") == 0) return &tmcZ1;
    if (strcasecmp(motor, "z2") == 0) return &tmcZ2;
    if (strcasecmp(motor, "m3") == 0) return &tmcM3;
    return nullptr;
}

static bool isTmcInitialized(const char *motor) {
    if (strcasecmp(motor, "z1") == 0) return tmcZ1Initialized;
    if (strcasecmp(motor, "z2") == 0) return tmcZ2Initialized;
    if (strcasecmp(motor, "m3") == 0) return tmcM3Initialized;
    return false;
}

static void getAxisPins(const char *axis, uint8_t &stepPin, uint8_t &dirPin,
                        bool &valid) {
    valid = true;
    if      (strcasecmp(axis, "z1") == 0) { stepPin = PIN_Z1_STEP; dirPin = PIN_Z1_DIR; }
    else if (strcasecmp(axis, "z2") == 0) { stepPin = PIN_Z2_STEP; dirPin = PIN_Z2_DIR; }
    else if (strcasecmp(axis, "m3") == 0) { stepPin = PIN_M3_STEP; dirPin = PIN_M3_DIR; }
    else { valid = false; }
}
