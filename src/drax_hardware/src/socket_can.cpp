#include "drax_hardware/socket_can.hpp"

#include <cstring>
#include <cerrno>

#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <poll.h>

#include <net/if.h>

#include <linux/can/raw.h>

#include <rclcpp/rclcpp.hpp>

namespace drax_hardware
{

SocketCanBus::SocketCanBus(const rclcpp::Logger & logger)
: socket_fd_(-1),
  logger_(logger)
{
}

SocketCanBus::~SocketCanBus()
{
    close();
}

bool SocketCanBus::isOpen() const
{
    return socket_fd_ >= 0;
}

bool SocketCanBus::open(const std::string &interface)
{
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    if (socket_fd_ < 0)
    {
        RCLCPP_ERROR(logger_, "Failed to create CAN socket: %s", std::strerror(errno));
        return false;
    }

    struct ifreq ifr {};

    std::strncpy(
        ifr.ifr_name,
        interface.c_str(),
        IFNAMSIZ - 1);

    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
    {
        RCLCPP_ERROR(logger_, "Cannot find CAN interface %s: %s",
                     interface.c_str(), std::strerror(errno));

        close();

        return false;
    }

    sockaddr_can addr {};

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_,
             reinterpret_cast<sockaddr*>(&addr),
             sizeof(addr)) < 0)
    {
        RCLCPP_ERROR(logger_, "Failed to bind CAN socket to %s: %s",
                     interface.c_str(), std::strerror(errno));

        close();

        return false;
    }

    return true;
}

void SocketCanBus::close()
{
    if (socket_fd_ >= 0)
    {
        ::close(socket_fd_);

        socket_fd_ = -1;
    }
}

bool SocketCanBus::send(const can_frame &frame)
{
    if (!isOpen())
    {
        RCLCPP_ERROR(logger_, "Cannot send CAN frame: socket is closed");
        return false;
    }

    ssize_t bytes =
        write(socket_fd_,
              &frame,
              sizeof(can_frame));

    if (bytes != sizeof(can_frame))
    {
        RCLCPP_ERROR(logger_, "Failed to send CAN frame 0x%03X: %s",
                     frame.can_id, std::strerror(errno));
        return false;
    }

    return true;
}

bool SocketCanBus::receive(can_frame &frame, std::chrono::milliseconds timeout)
{
    if (!isOpen())
    {
        RCLCPP_ERROR(logger_, "Cannot receive CAN frame: socket is closed");
        return false;
    }

    pollfd fds{};
    fds.fd = socket_fd_;
    fds.events = POLLIN;

    const int poll_result = poll(&fds, 1, static_cast<int>(timeout.count()));

    if (poll_result == 0)
    {
        return false;
    }

    if (poll_result < 0)
    {
        if (errno != EINTR)
        {
            RCLCPP_ERROR(logger_, "CAN poll failed: %s", std::strerror(errno));
        }
        return false;
    }

    if ((fds.revents & POLLIN) == 0)
    {
        RCLCPP_ERROR(logger_, "CAN socket poll returned unexpected events: 0x%X", fds.revents);
        return false;
    }

    ssize_t bytes =
        read(socket_fd_,
             &frame,
             sizeof(can_frame));

    if (bytes != sizeof(can_frame))
    {
        RCLCPP_ERROR(logger_, "Failed to read CAN frame: %s", std::strerror(errno));
        return false;
    }

    return true;
}

bool SocketCanBus::sendNMTStart(uint8_t node_id)
{
    can_frame frame{};

    frame.can_id = 0x000;
    frame.can_dlc = 2;

    // NMT Start Remote Node
    frame.data[0] = 0x01;
    frame.data[1] = node_id;

    return send(frame);
}
} // namespace drax_hardware
