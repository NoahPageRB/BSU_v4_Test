// ============================================================================
// soft_i2c.h — Minimal bitbang I2C master for MCP23017 register access
// ============================================================================
//
// WHY: The Metro M4's Wire library uses SERCOM5 (on SDA=PB2, SCL=PB3). The
// Z2 motor TMC2209 UART also requires SERCOM5 (on D3=PB16, D2=PB17). Since a
// SERCOM can only operate in one mode at a time and the Wire library claims
// SERCOM5's interrupt handlers as strong symbols, we use bitbang I2C instead.
// The MCP23017 runs fine at the ~100 kHz speeds achievable with GPIO toggling.
//
// This is intentionally minimal: only write-byte and read-byte transactions
// are implemented, which is all we need for MCP23017 register access.
// ============================================================================

#pragma once

#include <Arduino.h>

class SoftI2C {
public:
    /// Construct with the SDA and SCL pin numbers.
    SoftI2C(uint8_t sdaPin, uint8_t sclPin)
        : _sda(sdaPin), _scl(sclPin) {}

    /// Initialize the bus (both lines released HIGH via input mode + external pull-ups).
    void begin() {
        pinMode(_sda, INPUT); // External pull-up → HIGH
        pinMode(_scl, INPUT);
        delayMicroseconds(10);
    }

    /// Write a single byte to a register on a device.
    /// Returns true if all ACKs were received.
    bool writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
        startCondition();
        if (!writeByte((addr << 1) | 0)) { stopCondition(); return false; } // W
        if (!writeByte(reg))              { stopCondition(); return false; }
        if (!writeByte(value))            { stopCondition(); return false; }
        stopCondition();
        return true;
    }

    /// Read a single byte from a register on a device.
    /// Returns true if successful, storing result in `out`.
    bool readRegister(uint8_t addr, uint8_t reg, uint8_t &out) {
        // Write phase: send register address
        startCondition();
        if (!writeByte((addr << 1) | 0)) { stopCondition(); return false; } // W
        if (!writeByte(reg))              { stopCondition(); return false; }

        // Repeated start, then read phase
        startCondition();
        if (!writeByte((addr << 1) | 1)) { stopCondition(); return false; } // R
        out = readByte(false); // NACK after single byte
        stopCondition();
        return true;
    }

    /// Read two consecutive bytes (e.g. GPIOA + GPIOB in one transaction).
    /// Returns true if successful.
    bool readRegister16(uint8_t addr, uint8_t reg, uint8_t &outA, uint8_t &outB) {
        startCondition();
        if (!writeByte((addr << 1) | 0)) { stopCondition(); return false; }
        if (!writeByte(reg))              { stopCondition(); return false; }

        startCondition();
        if (!writeByte((addr << 1) | 1)) { stopCondition(); return false; }
        outA = readByte(true);  // ACK — more bytes coming
        outB = readByte(false); // NACK — last byte
        stopCondition();
        return true;
    }

    /// Probe whether a device ACKs its address.
    bool probe(uint8_t addr) {
        startCondition();
        bool ack = writeByte((addr << 1) | 0);
        stopCondition();
        return ack;
    }

    /// Write an array of bytes to a device (raw write, no register addressing).
    /// Suitable for sending multi-byte commands to I2C slave devices.
    bool writeBytes(uint8_t addr, const uint8_t *data, size_t len) {
        startCondition();
        if (!writeByte((addr << 1) | 0)) { stopCondition(); return false; }
        for (size_t i = 0; i < len; i++) {
            if (!writeByte(data[i])) { stopCondition(); return false; }
        }
        stopCondition();
        return true;
    }

    /// Read an array of bytes from a device (raw read, no register addressing).
    /// The slave must have a response ready (e.g., prepared during a prior write).
    bool readBytes(uint8_t addr, uint8_t *data, size_t len) {
        startCondition();
        if (!writeByte((addr << 1) | 1)) { stopCondition(); return false; }
        for (size_t i = 0; i < len; i++) {
            data[i] = readByte(i < len - 1); // ACK all but last byte
        }
        stopCondition();
        return true;
    }

private:
    uint8_t _sda;
    uint8_t _scl;

    // Timing: ~5 µs half-period → ~100 kHz. Generous for reliability.
    static constexpr uint8_t HALF_PERIOD_US = 5;

    void sdaHigh() { pinMode(_sda, INPUT);  } // Release to pull-up
    void sdaLow()  { pinMode(_sda, OUTPUT); digitalWrite(_sda, LOW); }
    void sclHigh() { pinMode(_scl, INPUT);  delayMicroseconds(HALF_PERIOD_US); }
    void sclLow()  { pinMode(_scl, OUTPUT); digitalWrite(_scl, LOW); delayMicroseconds(HALF_PERIOD_US); }

    void startCondition() {
        sdaHigh();
        sclHigh();
        sdaLow();  // SDA falls while SCL is HIGH
        sclLow();
    }

    void stopCondition() {
        sdaLow();
        sclHigh();
        sdaHigh(); // SDA rises while SCL is HIGH
        delayMicroseconds(HALF_PERIOD_US);
    }

    /// Write 8 bits, MSB first. Returns true if ACK received.
    bool writeByte(uint8_t data) {
        for (int i = 7; i >= 0; i--) {
            if (data & (1 << i)) sdaHigh(); else sdaLow();
            sclHigh();
            sclLow();
        }
        // Read ACK
        sdaHigh(); // Release SDA for slave to pull LOW
        sclHigh();
        bool ack = (digitalRead(_sda) == LOW);
        sclLow();
        return ack;
    }

    /// Read 8 bits, MSB first. Sends ACK if `ack` is true, NACK otherwise.
    uint8_t readByte(bool ack) {
        sdaHigh(); // Release SDA
        uint8_t data = 0;
        for (int i = 7; i >= 0; i--) {
            sclHigh();
            if (digitalRead(_sda)) data |= (1 << i);
            sclLow();
        }
        // Send ACK/NACK
        if (ack) sdaLow(); else sdaHigh();
        sclHigh();
        sclLow();
        sdaHigh();
        return data;
    }
};
