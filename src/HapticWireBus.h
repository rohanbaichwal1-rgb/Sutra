#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <HapticArray.h>

class HapticWireBus : public haptics::I2cBus
{
public:
    explicit HapticWireBus(
        TwoWire &wire,
        uint32_t hapticClockHz = 100000UL,
        uint32_t sensorClockHz = 400000UL)
        : wire_(wire),
          hapticClockHz_(hapticClockHz),
          sensorClockHz_(sensorClockHz)
    {
    }

    bool write(uint8_t address, const uint8_t *data, size_t size) override
    {
        for (unsigned attempt = 0; attempt < 2; ++attempt)
        {
            wire_.setClock(hapticClockHz_);

            wire_.beginTransmission(address);

            const size_t written = wire_.write(data, size);

            const uint8_t result = wire_.endTransmission();

            if (written == size && result == 0)
            {
                // TCA9548A mux settling time.
                // Matches the proven working P1 sketch behavior.
                if (address == 0x70)
                {
                    delayMicroseconds(500);
                }

                wire_.setClock(sensorClockHz_);
                return true;
            }

            wire_.setClock(sensorClockHz_);
        }

        return false;
    }

    bool readRegister(
        uint8_t address,
        uint8_t reg,
        uint8_t &value) override
    {
        for (unsigned attempt = 0; attempt < 2; ++attempt)
        {
            wire_.setClock(hapticClockHz_);

            wire_.beginTransmission(address);

            wire_.write(reg);

            const bool addressed =
                wire_.endTransmission(false) == 0;

            const bool received =
                addressed &&
                wire_.requestFrom(
                    address,
                    static_cast<uint8_t>(1)) == 1;

            if (received)
            {
                value =
                    static_cast<uint8_t>(
                        wire_.read());
            }

            wire_.setClock(sensorClockHz_);

            if (received)
            {
                return true;
            }
        }

        return false;
    }

private:
    TwoWire &wire_;
    uint32_t hapticClockHz_;
    uint32_t sensorClockHz_;
};