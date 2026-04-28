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
//
// Reliability: every public transaction is automatically retried up to 3
// times with backoff (200 µs, 800 µs) on NACK or other failure. UART
// interrupts (Z1/Z2/M3) can preempt the bitbang and stretch a 5 µs SCL
// half-period enough to violate I2C timing; without retries that produces
// "LED command lost" / "expander write glitched" artifacts.
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

    // ------------------------------------------------------------------------
    // Public retrying wrappers. Each retries the underlying transaction up to
    // RETRY_ATTEMPTS times before returning false.
    // ------------------------------------------------------------------------

    bool writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
        return _retry([&] { return _writeRegisterOnce(addr, reg, value); });
    }

    bool readRegister(uint8_t addr, uint8_t reg, uint8_t &out) {
        return _retry([&] { return _readRegisterOnce(addr, reg, out); });
    }

    bool readRegister16(uint8_t addr, uint8_t reg, uint8_t &outA, uint8_t &outB) {
        return _retry([&] { return _readRegister16Once(addr, reg, outA, outB); });
    }

    bool probe(uint8_t addr) {
        return _retry([&] { return _probeOnce(addr); });
    }

    bool writeBytes(uint8_t addr, const uint8_t *data, size_t len) {
        return _retry([&] { return _writeBytesOnce(addr, data, len); });
    }

    bool readBytes(uint8_t addr, uint8_t *data, size_t len) {
        return _retry([&] { return _readBytesOnce(addr, data, len); });
    }

private:
    uint8_t _sda;
    uint8_t _scl;

    // Timing: ~5 µs half-period → ~100 kHz. Generous for reliability.
    static constexpr uint8_t HALF_PERIOD_US = 5;

    // Retry policy. 3 attempts total with backoff between: 0 µs (first try),
    // 200 µs (second), 800 µs (third). The backoff lets a transient ISR
    // preemption clear and any contending master release the bus.
    static constexpr uint8_t RETRY_ATTEMPTS = 3;

    template <typename Fn>
    bool _retry(Fn &&fn) {
        if (fn()) return true;
        delayMicroseconds(200);
        if (fn()) return true;
        delayMicroseconds(800);
        return fn();
    }

    // ------------------------------------------------------------------------
    // Single-attempt transaction implementations. Public wrappers above call
    // these via _retry().
    // ------------------------------------------------------------------------

    bool _writeRegisterOnce(uint8_t addr, uint8_t reg, uint8_t value) {
        startCondition();
        if (!writeByte((addr << 1) | 0)) { stopCondition(); return false; } // W
        if (!writeByte(reg))              { stopCondition(); return false; }
        if (!writeByte(value))            { stopCondition(); return false; }
        stopCondition();
        return true;
    }

    bool _readRegisterOnce(uint8_t addr, uint8_t reg, uint8_t &out) {
        startCondition();
        if (!writeByte((addr << 1) | 0)) { stopCondition(); return false; } // W
        if (!writeByte(reg))              { stopCondition(); return false; }

        startCondition();
        if (!writeByte((addr << 1) | 1)) { stopCondition(); return false; } // R
        out = readByte(false); // NACK after single byte
        stopCondition();
        return true;
    }

    bool _readRegister16Once(uint8_t addr, uint8_t reg, uint8_t &outA, uint8_t &outB) {
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

    bool _probeOnce(uint8_t addr) {
        startCondition();
        bool ack = writeByte((addr << 1) | 0);
        stopCondition();
        return ack;
    }

    bool _writeBytesOnce(uint8_t addr, const uint8_t *data, size_t len) {
        startCondition();
        if (!writeByte((addr << 1) | 0)) { stopCondition(); return false; }
        for (size_t i = 0; i < len; i++) {
            if (!writeByte(data[i])) { stopCondition(); return false; }
        }
        stopCondition();
        return true;
    }

    bool _readBytesOnce(uint8_t addr, uint8_t *data, size_t len) {
        startCondition();
        if (!writeByte((addr << 1) | 1)) { stopCondition(); return false; }
        for (size_t i = 0; i < len; i++) {
            data[i] = readByte(i < len - 1); // ACK all but last byte
        }
        stopCondition();
        return true;
    }

    // ------------------------------------------------------------------------
    // Bit-level primitives.
    // ------------------------------------------------------------------------

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
