#include "drax_hardware/motor.hpp"

#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

namespace drax_hardware
{

Motor::Motor(CanOpenNode &node,
             const MotorConfig & config,
             const rclcpp::Logger & logger)
: node_(node),
  config_(config),
  logger_(logger)
{
}

int32_t Motor::rpmToDec(int rpm)
{
    return static_cast<int32_t>(
        (rpm * 512.0 * config_.feedback_resolution) / 1875.0);
}

bool Motor::initSpeedMode(int8_t mode)
{
    RCLCPP_INFO(logger_, "Initializing speed mode");

    RCLCPP_DEBUG(logger_, "Writing operation mode 0x6060 = %d", mode);
    if (!node_.write(0x6060, 0x00, mode, 1, true))
    {
        RCLCPP_ERROR(logger_, "Failed to write operation mode 0x6060");
        return false;
    }

    RCLCPP_DEBUG(logger_, "Writing control word 0x6040 = 0x0086");
    if (!node_.write(0x6040, 0x00, 0x86, 2, true))
    {
        RCLCPP_ERROR(logger_, "Failed to write control word 0x0086");
        return false;
    }

    RCLCPP_DEBUG(logger_, "Writing quick-stop option code 0x605A:11 = 0");
    if (!node_.write(0x605A, 0x11, 0, 1, false))
    {
        RCLCPP_ERROR(logger_, "Failed to write quick-stop option code 0x605A:11");
        return false;
    }

    if (config_.fix_din3)
    {
        RCLCPP_DEBUG(logger_, "Applying vendor DIN3 fix");

        uint16_t pol_val;
        if (!node_.readU16(0x2010, 0x01, pol_val))
        {
            RCLCPP_ERROR(logger_, "Failed to read DIN polarity 0x2010:01");
            return false;
        }
        if (!node_.write(0x2010, 0x01, pol_val & ~(1 << 2), 2, false))
        {
            RCLCPP_ERROR(logger_, "Failed to write DIN polarity 0x2010:01");
            return false;
        }

        uint16_t sim_val;
        if (!node_.readU16(0x2010, 0x02, sim_val))
        {
            RCLCPP_ERROR(logger_, "Failed to read DIN simulation 0x2010:02");
            return false;
        }
        if (!node_.write(0x2010, 0x02, sim_val & ~(1 << 2), 2, false))
        {
            RCLCPP_ERROR(logger_, "Failed to write DIN simulation 0x2010:02");
            return false;
        }
    }

    RCLCPP_DEBUG(logger_, "Writing control word 0x6040 = 0x0006");
    if (!node_.write(0x6040, 0x00, 0x06, 2, true))
    {
        RCLCPP_ERROR(logger_, "Failed to write control word 0x0006");
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    RCLCPP_DEBUG(logger_, "Writing control word 0x6040 = 0x0007");
    if (!node_.write(0x6040, 0x00, 0x07, 2, true))
    {
        RCLCPP_ERROR(logger_, "Failed to write control word 0x0007");
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    RCLCPP_DEBUG(logger_, "Writing control word 0x6040 = 0x000F");
    if (!node_.write(0x6040, 0x00, 0x0F, 2, true))
    {
        RCLCPP_ERROR(logger_, "Failed to write control word 0x000F");
        return false;
    }

    RCLCPP_INFO(logger_, "Speed mode initialized");

    return setRPM(0);
}

bool Motor::setRPM(int16_t rpm)
{
    RCLCPP_DEBUG(logger_, "Sending RPM command %d", rpm);

    if (!node_.write(
        0x2FF0,
        0x09,
        rpm,
        2,
        true))
    {
        RCLCPP_ERROR(logger_, "Failed to write RPM command 0x2FF0:09");
        return false;
    }

    if (!node_.write(
        0x60FF,
        0x00,
        rpmToDec(rpm),
        4,
        true))
    {
        RCLCPP_ERROR(logger_, "Failed to write target velocity 0x60FF:00");
        return false;
    }

    RCLCPP_DEBUG(logger_, "RPM command sent");

    return true;
}
bool Motor::readEncoder(int32_t &encoder)
{
    return node_.readI32(
        0x6063,
        0x00,
        encoder);
}
bool Motor::readStatusWord(uint16_t &status)
{
    return node_.readU16(
        0x6041,
        0x00,
        status);
}
bool Motor::stop()
{
    return setRPM(0);
}

bool Motor::disable()
{
    return node_.write(
        0x6040,
        0x00,
        0x06,
        2,
        true);
}

bool Motor::quickStop()
{
    RCLCPP_WARN(logger_, "Sending CANopen quick stop");

    return node_.write(
        0x6040,
        0x00,
        0x02,
        2,
        true);
}

bool Motor::readTemperature(double &temperature)
{
    if (config_.temperature_index == 0)
    {
        return false;
    }

    int32_t raw_temperature = 0;
    if (!node_.readI32(config_.temperature_index,
                       config_.temperature_subindex,
                       raw_temperature))
    {
        return false;
    }

    temperature = raw_temperature * config_.temperature_scale;
    return true;
}

bool Motor::readBusVoltage(double &voltage)
{
    if (config_.voltage_index == 0)
    {
        return false;
    }

    uint32_t raw_voltage = 0;
    if (!node_.readU32(config_.voltage_index,
                       config_.voltage_subindex,
                       raw_voltage))
    {
        return false;
    }

    voltage = raw_voltage * config_.voltage_scale;
    return true;
}

} // namespace drax_hardware
