#pragma once

#include "drax_hardware/canopen_node.hpp"

#include <cstdint>

#include <rclcpp/logger.hpp>

namespace drax_hardware
{

struct MotorConfig
{
    int feedback_resolution = 4096;
    bool fix_din3 = true;
    uint16_t temperature_index = 0;
    uint8_t temperature_subindex = 0;
    double temperature_scale = 1.0;
    uint16_t voltage_index = 0;
    uint8_t voltage_subindex = 0;
    double voltage_scale = 1.0;
};

class Motor
{
public:

    Motor(CanOpenNode &node,
          const MotorConfig & config,
          const rclcpp::Logger & logger);

    bool initSpeedMode(int8_t mode = -3);

    bool setRPM(int16_t rpm);

    bool stop();

    bool disable();

    bool quickStop();

    bool readEncoder(int32_t &encoder);

    bool readStatusWord(uint16_t &status);

    bool readTemperature(double &temperature);

    bool readBusVoltage(double &voltage);

private:

    int32_t rpmToDec(int rpm);

    CanOpenNode &node_;

    MotorConfig config_;

    rclcpp::Logger logger_;
};
}
