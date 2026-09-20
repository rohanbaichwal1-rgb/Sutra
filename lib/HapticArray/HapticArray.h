#pragma once

#include <HapticPatterns.h>
#include <stddef.h>

namespace haptics
{
// Thin transport interface also used by the host-side simulated I2C bus.
class I2cBus
{
public:
    virtual ~I2cBus() = default;
    virtual bool write(uint8_t address, const uint8_t *data, size_t size) = 0;
    virtual bool readRegister(uint8_t address, uint8_t reg, uint8_t &value) = 0;
};

struct ArrayConfig
{
    uint8_t muxAddress = 0x70;
    uint8_t driverAddress = 0x4A;
    uint8_t channels[MOTOR_COUNT] = {0, 1, 2};
};

enum class ArrayError : uint8_t { NONE, CONFIG, I2C, CHIP_ID, DRIVER_FAULT };

class Array
{
public:
    explicit Array(I2cBus &bus, const ArrayConfig &config = ArrayConfig{});
    bool begin(); // Call after power has been stable for at least 1.5 ms.
    bool apply(const Frame &frame);
    bool pollFaults();
    bool stopAll();
    bool ready() const { return ready_; }
    bool shutdownPending() const { return shutdownPending_; }
    ArrayError error() const { return error_; }
    uint8_t errorMotor() const { return errorMotor_; } // 1..3, or 0 for mux/config
    uint8_t faultBits() const { return faultBits_; }
    uint8_t warningBits() const { return warningBits_; }
    static uint8_t amplitudeCode(uint16_t level);

private:
    I2cBus &bus_;
    ArrayConfig config_;
    uint8_t levels_[MOTOR_COUNT] = {0, 0, 0};
    bool driving_[MOTOR_COUNT] = {false, false, false};
    bool ready_ = false;
    bool shutdownPending_ = false;
    ArrayError error_ = ArrayError::NONE;
    uint8_t errorMotor_ = 0;
    uint8_t faultBits_ = 0;
    uint8_t warningBits_ = 0;

    bool select(unsigned motor);
    bool disconnect();
    bool writeReg(uint8_t reg, uint8_t value);
    bool readReg(uint8_t reg, uint8_t &value);
    bool modifyReg(uint8_t reg, uint8_t clearMask, uint8_t setMask);
    bool fail(ArrayError error, uint8_t motor = 0, uint8_t bits = 0);
};
} // namespace haptics
