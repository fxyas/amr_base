#pragma once

#include "drax_hardware/socket_can.hpp"

#include <chrono>
#include <cstdint>

#include <rclcpp/logger.hpp>

namespace drax_hardware
{

class CanOpenNode
{
public:
    CanOpenNode(SocketCanBus &bus,
                uint8_t node_id,
                const rclcpp::Logger & logger,
                std::chrono::milliseconds request_timeout,
                int request_retries);

    bool write(uint16_t index,
               uint8_t subindex,
               int32_t value,
               uint8_t size,
               bool is_signed = true);

    bool readI32(uint16_t index,
                 uint8_t subindex,
                 int32_t &value);

    bool readU16(uint16_t index,
                 uint8_t subindex,
                 uint16_t &value);

    bool readU32(uint16_t index,
                 uint8_t subindex,
                 uint32_t &value);

private:

    bool request(const can_frame &tx,
                 can_frame &rx);

    SocketCanBus &bus_;

    uint8_t node_id_;

    uint16_t tx_id_;

    uint16_t rx_id_;

    rclcpp::Logger logger_;

    std::chrono::milliseconds request_timeout_;

    int request_retries_;
};

}
