#pragma once

#include <cstdint>
#include <string>
#include <chrono>
#include <linux/can.h>

#include <rclcpp/logger.hpp>

namespace drax_hardware
{

class SocketCanBus
{
public:
    explicit SocketCanBus(const rclcpp::Logger & logger);
    ~SocketCanBus();

    bool open(const std::string &interface);

    void close();

    bool send(const can_frame &frame);

    bool receive(can_frame &frame, std::chrono::milliseconds timeout);

    bool sendNMTStart(uint8_t node_id);

    bool isOpen() const;

private:
    int socket_fd_;
    rclcpp::Logger logger_;
};

} // namespace drax_hardware
