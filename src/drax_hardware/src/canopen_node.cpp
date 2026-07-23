#include "drax_hardware/canopen_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>

#include <rclcpp/rclcpp.hpp>

namespace drax_hardware
{

CanOpenNode::CanOpenNode(SocketCanBus &bus,
                         uint8_t node_id,
                         const rclcpp::Logger & logger,
                         std::chrono::milliseconds request_timeout,
                         int request_retries)
: bus_(bus),
  node_id_(node_id),
  tx_id_(0x600 + node_id),
  rx_id_(0x580 + node_id),
  logger_(logger),
  request_timeout_(request_timeout),
  request_retries_(std::max(1, request_retries))
{
}

bool CanOpenNode::request(const can_frame &tx,
                          can_frame &rx)
{
    for (int attempt = 1; attempt <= request_retries_; ++attempt)
    {
        if (!bus_.send(tx))
        {
            RCLCPP_ERROR(logger_, "Node %u: CAN send failed for SDO request 0x%03X",
                         node_id_, tx.can_id);
            return false;
        }

        const auto deadline = std::chrono::steady_clock::now() + request_timeout_;

        while (std::chrono::steady_clock::now() < deadline)
        {
            const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now());

            if (!bus_.receive(rx, std::max(std::chrono::milliseconds(1), remaining)))
            {
                break;
            }

            if (rx.can_id == rx_id_)
            {
                return true;
            }

            RCLCPP_DEBUG(logger_, "Node %u: ignoring CAN frame 0x%03X while waiting for 0x%03X",
                         node_id_, rx.can_id, rx_id_);
        }

        RCLCPP_WARN(logger_, "Node %u: SDO response timeout on attempt %d/%d",
                    node_id_, attempt, request_retries_);
    }

    RCLCPP_ERROR(logger_, "Node %u: SDO request failed after %d attempts",
                 node_id_, request_retries_);

    return false;
}

namespace
{
uint32_t abortCode(const can_frame &rx)
{
    uint32_t code = 0;
    std::memcpy(&code, &rx.data[4], sizeof(code));
    return code;
}
}

bool CanOpenNode::write(uint16_t index,
                        uint8_t subindex,
                        int32_t value,
                        uint8_t size,
                        bool is_signed)
{
    (void)is_signed;

    can_frame tx{};
    can_frame rx{};

    tx.can_id = tx_id_;
    tx.can_dlc = 8;

    switch(size)
    {
        case 1:
            tx.data[0] = 0x2F;
            break;

        case 2:
            tx.data[0] = 0x2B;
            break;

        case 4:
            tx.data[0] = 0x23;
            break;

        default:
            RCLCPP_ERROR(logger_, "Node %u: unsupported SDO write size %u for 0x%04X:%02X",
                         node_id_, size, index, subindex);
            return false;
    }

    tx.data[1] = index & 0xFF;
    tx.data[2] = index >> 8;
    tx.data[3] = subindex;

    std::memset(&tx.data[4], 0, 4);
    std::memcpy(&tx.data[4], &value, size);

    if (!request(tx, rx))
    {
        return false;
    }

    if (rx.data[0] == 0x80)
    {
        RCLCPP_ERROR(logger_, "Node %u: SDO abort writing 0x%04X:%02X, code 0x%08X",
                     node_id_, index, subindex, abortCode(rx));
        return false;
    }

    if (rx.data[0] != 0x60)
    {
        RCLCPP_ERROR(logger_, "Node %u: unexpected SDO write response 0x%02X for 0x%04X:%02X",
                     node_id_, rx.data[0], index, subindex);
        return false;
    }

    return true;
}

bool CanOpenNode::readI32(uint16_t index,
                          uint8_t subindex,
                          int32_t &value)
{
    can_frame tx{};
    can_frame rx{};

    tx.can_id = tx_id_;
    tx.can_dlc = 8;

    tx.data[0] = 0x40;
    tx.data[1] = index & 0xFF;
    tx.data[2] = index >> 8;
    tx.data[3] = subindex;

    if (!request(tx, rx))
    {
        return false;
    }

    if (rx.data[0] == 0x80)
    {
        RCLCPP_ERROR(logger_, "Node %u: SDO abort reading 0x%04X:%02X, code 0x%08X",
                     node_id_, index, subindex, abortCode(rx));
        return false;
    }

    if (rx.data[0] != 0x43 && rx.data[0] != 0x42)
    {
        RCLCPP_ERROR(logger_, "Node %u: unexpected SDO I32 read response 0x%02X for 0x%04X:%02X",
                     node_id_, rx.data[0], index, subindex);
        return false;
    }

    std::memcpy(&value, &rx.data[4], sizeof(value));

    return true;
}

bool CanOpenNode::readU16(uint16_t index,
                          uint8_t subindex,
                          uint16_t &value)
{
    can_frame tx{};
    can_frame rx{};

    tx.can_id = tx_id_;
    tx.can_dlc = 8;

    tx.data[0] = 0x40;
    tx.data[1] = index & 0xFF;
    tx.data[2] = index >> 8;
    tx.data[3] = subindex;

    if (!request(tx, rx))
    {
        return false;
    }

    if (rx.data[0] == 0x80)
    {
        RCLCPP_ERROR(logger_, "Node %u: SDO abort reading 0x%04X:%02X, code 0x%08X",
                     node_id_, index, subindex, abortCode(rx));
        return false;
    }

    if (rx.data[0] != 0x4B)
    {
        RCLCPP_ERROR(logger_, "Node %u: unexpected SDO U16 read response 0x%02X for 0x%04X:%02X",
                     node_id_, rx.data[0], index, subindex);
        return false;
    }

    std::memcpy(&value, &rx.data[4], sizeof(value));

    return true;
}

bool CanOpenNode::readU32(uint16_t index,
                          uint8_t subindex,
                          uint32_t &value)
{
    can_frame tx{};
    can_frame rx{};

    tx.can_id = tx_id_;
    tx.can_dlc = 8;

    tx.data[0] = 0x40;
    tx.data[1] = index & 0xFF;
    tx.data[2] = index >> 8;
    tx.data[3] = subindex;

    if (!request(tx, rx))
    {
        return false;
    }

    if (rx.data[0] == 0x80)
    {
        RCLCPP_ERROR(logger_, "Node %u: SDO abort reading 0x%04X:%02X, code 0x%08X",
                     node_id_, index, subindex, abortCode(rx));
        return false;
    }

    if (rx.data[0] != 0x43)
    {
        RCLCPP_ERROR(logger_, "Node %u: unexpected SDO U32 read response 0x%02X for 0x%04X:%02X",
                     node_id_, rx.data[0], index, subindex);
        return false;
    }

    std::memcpy(&value, &rx.data[4], sizeof(value));

    return true;
}

} // namespace drax_hardware
