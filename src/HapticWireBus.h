#pragma once

#include <Wire.h>
#include <HapticArray.h>

class HapticWireBus : public haptics::I2cBus
{
public:
    explicit HapticWireBus(TwoWire &wire) : wire_(wire) {}

    bool write(uint8_t address, const uint8_t *data, size_t size) override
    {
        wire_.beginTransmission(address);
        size_t written = wire_.write(data, size);
        uint8_t result = wire_.endTransmission();
        return written == size && result == 0;
    }

    bool readRegister(uint8_t address, uint8_t reg, uint8_t &value) override
    {
        wire_.beginTransmission(address);
        wire_.write(reg);
        if (wire_.endTransmission(false) != 0) return false;
        if (wire_.requestFrom(address, static_cast<uint8_t>(1)) != 1) return false;
        value = static_cast<uint8_t>(wire_.read());
        return true;
    }

private:
    TwoWire &wire_;
};
