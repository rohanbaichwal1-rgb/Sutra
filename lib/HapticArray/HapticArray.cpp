#include "HapticArray.h"

namespace haptics
{
// Renesas DA7280 datasheet rev. 3.2, sections 5.6.5.2 and 6.2.
namespace reg
{
constexpr uint8_t CHIP_REV = 0x00;
constexpr uint8_t IRQ_EVENT1 = 0x03;
constexpr uint8_t IRQ_WARNING = 0x04;
constexpr uint8_t IRQ_STATUS1 = 0x06;
constexpr uint8_t FRQ_LRA_PER_H = 0x0A;
constexpr uint8_t FRQ_LRA_PER_L = 0x0B;
constexpr uint8_t ACTUATOR1 = 0x0C;
constexpr uint8_t ACTUATOR2 = 0x0D;
constexpr uint8_t ACTUATOR3 = 0x0E;
constexpr uint8_t CALIB_V2I_H = 0x0F;
constexpr uint8_t CALIB_V2I_L = 0x10;
constexpr uint8_t TOP_CFG1 = 0x13;
constexpr uint8_t TOP_CTL1 = 0x22;
constexpr uint8_t TOP_CTL2 = 0x23;
constexpr uint8_t FAULT_MASK = 0xDA; // OC, actuator, sequence, temperature, UVLO
}

Array::Array(I2cBus &bus, const ArrayConfig &config) : bus_(bus), config_(config) {}

bool Array::select(unsigned motor)
{
    if (!configuredMotor(motor) || config_.channels[motor] > 7)
        return false;

    const uint8_t mask =
        static_cast<uint8_t>(1U << config_.channels[motor]);

    return bus_.write(config_.muxAddress, &mask, 1);
}
bool Array::disconnect()
{
    const uint8_t mask = 0;
    return bus_.write(config_.muxAddress, &mask, 1);
}

bool Array::writeReg(uint8_t address, uint8_t value)
{
    const uint8_t data[] = {address, value};
    return bus_.write(config_.driverAddress, data, sizeof(data));
}

bool Array::readReg(uint8_t address, uint8_t &value)
{
    return bus_.readRegister(config_.driverAddress, address, value);
}

bool Array::modifyReg(uint8_t address, uint8_t clearMask, uint8_t setMask)
{
    uint8_t value;
    return readReg(address, value) &&
           writeReg(address, (value & ~clearMask) | setMask);
}

bool Array::fail(ArrayError error, uint8_t motor, uint8_t bits)
{
    ready_ = false;
    error_ = error;
    errorMotor_ = motor;
    faultBits_ = bits;
    stopAll(); // best effort on every channel, even if one is unreachable
    return false;
}

bool Array::begin()
{
    ready_ = false;
    error_ = ArrayError::NONE;
    errorMotor_ = faultBits_ = 0;
    warningBits_ = 0;
    if (config_.muxAddress < 0x70 || config_.muxAddress > 0x77 ||
        config_.driverAddress != 0x4A || config_.motorCount == 0 ||
        config_.motorCount > MOTOR_COUNT || config_.outputScalePercent > 100)
    {
        error_ = ArrayError::CONFIG;
        return false;
    }
    for (unsigned i = 0; i < config_.motorCount; ++i)
    {
        if (config_.channels[i] > 7) { error_ = ArrayError::CONFIG; return false; }
        for (unsigned j = 0; j < i; ++j)
            if (config_.channels[i] == config_.channels[j])
            { error_ = ArrayError::CONFIG; return false; }
    }

    if (!disconnect()) return fail(ArrayError::I2C);
    for (unsigned motor = 0; motor < config_.motorCount; ++motor)
    {
        uint8_t revision;
        if (!select(motor) || !readReg(reg::CHIP_REV, revision))
            return fail(ArrayError::I2C, motor + 1);
        if (revision != 0xBA) return fail(ArrayError::CHIP_ID, motor + 1);

        // Register-equivalent of the known-working sketch's:
        // begin(); defaultMotor(); enableFreqTrack(false);
        // setOperationMode(DRO_MODE); setVibrate(0).
        // defaultMotor() installs SparkFun's LRA profile: 2.106 V nominal,
        // 2.26 V absolute, 165.4 mA, 13.8 ohm and 170 Hz.  Keep the same
        // read-modify-write behavior for TOP_CFG1/ACTUATOR3/Frequency-LSB.
        if (!modifyReg(reg::TOP_CFG1, 0x28, 0x00) || // LRA + frequency tracking OFF
            !writeReg(reg::ACTUATOR2, 0x60) ||      // 2.26 V
            !writeReg(reg::ACTUATOR1, 0x5A) ||      // 2.106 V
            !modifyReg(reg::ACTUATOR3, 0x1F, 0x13) || // 165.4 mA
            !writeReg(reg::CALIB_V2I_L, 0xC5) ||    // 13.8 ohm at 165.4 mA
            !writeReg(reg::CALIB_V2I_H, 0x00) ||
            !writeReg(reg::FRQ_LRA_PER_H, 0x22) ||  // 170 Hz LRA period
            !modifyReg(reg::FRQ_LRA_PER_L, 0x7F, 0x3B) ||
            !modifyReg(reg::TOP_CTL1, 0x07, 0x01) || // DRO mode
            !writeReg(reg::TOP_CTL2, 0))            // setVibrate(0)
            return fail(ArrayError::I2C, motor + 1);
        levels_[motor] = 0;
        driving_[motor] = false;
    }
    if (!disconnect()) return fail(ArrayError::I2C);
    shutdownPending_ = false;
    ready_ = true;
    // The working sketches begin/configure each selected DA7280 and then
    // proceed to playback. Do not add a full three-driver diagnostic sweep
    // here: a transient read on another mux branch must not prevent startup.
    return true;
}

bool Array::readDiagnostics(unsigned motor, DriverDiagnostics &diagnostics)
{
    if (!ready_ || !configuredMotor(motor)) return false;

    const bool ok = select(motor) &&
                    readReg(reg::IRQ_EVENT1, diagnostics.event) &&
                    readReg(reg::IRQ_WARNING, diagnostics.warning) &&
                    readReg(reg::IRQ_STATUS1, diagnostics.status);
    const bool disconnected = disconnect();
    return ok && disconnected;
}

bool Array::hasFatalFault(const DriverDiagnostics &diagnostics)
{
    return ((diagnostics.event | diagnostics.status) & reg::FAULT_MASK) != 0;
}

uint8_t Array::amplitudeCode(uint16_t level, uint8_t scalePercent)
{
    if (level > 1000) level = 1000;
    if (scalePercent > 100) scalePercent = 100;
    // With acceleration enabled, DRO input is 0..127, not 0..255.
    return static_cast<uint8_t>(
        (static_cast<uint32_t>(level) * 127 * scalePercent + 50000UL) / 100000UL);
}

bool Array::apply(const Frame &frame)
{
    if (!ready_) return false;
    // Send reductions before increases when handing a sweep to another motor.
    // Switching the mux does not stop the previously addressed DA7280.
    for (unsigned pass = 0; pass < 2; ++pass)
    {
        for (unsigned motor = 0; motor < config_.motorCount; ++motor)
        {
            uint8_t next = amplitudeCode(frame.level[motor], config_.outputScalePercent);
            if (next == levels_[motor]) continue;
            bool reduction = next < levels_[motor];
            if (reduction != (pass == 0)) continue;
            if (!select(motor) || !writeReg(reg::TOP_CTL2, next))
                return fail(ArrayError::I2C, motor + 1);
            if (!driving_[motor] && next > 0)
            {
                // Initial amplitude is set before entering DRO playback.
                // Mark BEFORE the transfer: a lost acknowledgement does not
                // prove that the driver ignored the playback command.
                driving_[motor] = true;
                if (!writeReg(reg::TOP_CTL1, 0x01))
                    return fail(ArrayError::I2C, motor + 1);
            }
            levels_[motor] = next;
        }
    }
    // Leave the root bus isolated for the MAX30101 and BMI270.
    if (!disconnect()) return fail(ArrayError::I2C);
    return true;
}

bool Array::pollFaults()
{
    if (!ready_) return false;
    for (unsigned motor = 0; motor < config_.motorCount; ++motor)
    {
        uint8_t events, status;
        if (!select(motor) || !readReg(reg::IRQ_EVENT1, events) ||
            !readReg(reg::IRQ_STATUS1, status))
            return fail(ArrayError::I2C, motor + 1);
        if ((events | status) & reg::FAULT_MASK)
            return fail(ArrayError::DRIVER_FAULT, motor + 1,
                        (events | status) & reg::FAULT_MASK);
        if ((events | status) & 0x20)
        {
            uint8_t warning;
            if (!readReg(reg::IRQ_WARNING, warning))
                return fail(ArrayError::I2C, motor + 1);
            // Temperature/configuration warnings stop the array. Supply
            // limiting is reported but the driver can continue its bounded
            // output; actual physical strength may be below the request.
            warningBits_ |= warning;
            if (warning & 0x18)
                return fail(ArrayError::DRIVER_FAULT, motor + 1, warning);
            if (!writeReg(reg::IRQ_EVENT1, 0x20))
                return fail(ArrayError::I2C, motor + 1);
        }
    }
    if (!disconnect()) return fail(ArrayError::I2C);
    return true;
}

bool Array::outputStopPending() const
{
    if (!shutdownPending_) return false;
    for (unsigned motor = 0; motor < config_.motorCount; ++motor)
        if (driving_[motor]) return true;
    return false;
}

bool Array::stopAll()
{
    bool ok = true;
    for (unsigned motor = 0; motor < config_.motorCount; ++motor)
    {
        if (!select(motor)) { ok = false; continue; }
        // Do not short-circuit: try both stop writes even if one fails.
        bool amplitudeStopped = writeReg(reg::TOP_CTL2, 0);
        bool modeStopped = writeReg(reg::TOP_CTL1, 0);
        if (!amplitudeStopped || !modeStopped) ok = false;
        else { levels_[motor] = 0; driving_[motor] = false; }
    }
    if (!disconnect()) ok = false;
    shutdownPending_ = !ok;
    if (!ok)
    {
        ready_ = false;
        if (error_ == ArrayError::NONE) error_ = ArrayError::I2C;
    }
    return ok;
}
} // namespace haptics
