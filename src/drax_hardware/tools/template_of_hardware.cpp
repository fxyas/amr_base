#include "drax_hardware/drax_hardware.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace drax_hardware
{

// on_init()

hardware_interface::CallbackReturn
DraxHardware::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;

    RCLCPP_INFO(
        rclcpp::get_logger("DraxHardware"),
        "Hardware initialized.");

    return hardware_interface::CallbackReturn::SUCCESS;
}

// on_configure()

hardware_interface::CallbackReturn
DraxHardware::on_configure(
    const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(
        rclcpp::get_logger("DraxHardware"),
        "Hardware configured.");

    return hardware_interface::CallbackReturn::SUCCESS;
}

// on_activate()

hardware_interface::CallbackReturn
DraxHardware::on_activate(
    const rclcpp_lifecycle::State &)
{
    left_wheel_position_ = 0.0;
    right_wheel_position_ = 0.0;

    left_wheel_velocity_ = 0.0;
    right_wheel_velocity_ = 0.0;

    left_wheel_command_ = 0.0;
    right_wheel_command_ = 0.0;

    RCLCPP_INFO(
        rclcpp::get_logger("DraxHardware"),
        "Hardware activated.");

    return hardware_interface::CallbackReturn::SUCCESS;
}

// on_deactivate()

hardware_interface::CallbackReturn
DraxHardware::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(
        rclcpp::get_logger("DraxHardware"),
        "Hardware deactivated.");

    return hardware_interface::CallbackReturn::SUCCESS;
}

// export_state_interfaces()

std::vector<hardware_interface::StateInterface>
DraxHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(
        info_.joints[0].name,
        hardware_interface::HW_IF_POSITION,
        &left_wheel_position_);

    state_interfaces.emplace_back(
        info_.joints[0].name,
        hardware_interface::HW_IF_VELOCITY,
        &left_wheel_velocity_);

    state_interfaces.emplace_back(
        info_.joints[1].name,
        hardware_interface::HW_IF_POSITION,
        &right_wheel_position_);

    state_interfaces.emplace_back(
        info_.joints[1].name,
        hardware_interface::HW_IF_VELOCITY,
        &right_wheel_velocity_);

    return state_interfaces;
}

// export_command_interfaces()

std::vector<hardware_interface::CommandInterface>
DraxHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(
        info_.joints[0].name,
        hardware_interface::HW_IF_VELOCITY,
        &left_wheel_command_);

    command_interfaces.emplace_back(
        info_.joints[1].name,
        hardware_interface::HW_IF_VELOCITY,
        &right_wheel_command_);

    return command_interfaces;
}

// read()

hardware_interface::return_type
DraxHardware::read(
    const rclcpp::Time &,
    const rclcpp::Duration & period)
{
    double dt = period.seconds();

    // Simulate wheel motion using the commanded velocity
    left_wheel_velocity_ = left_wheel_command_;
    right_wheel_velocity_ = right_wheel_command_;

    left_wheel_position_ += left_wheel_velocity_ * dt;
    right_wheel_position_ += right_wheel_velocity_ * dt;

    return hardware_interface::return_type::OK;
}

// write()

hardware_interface::return_type
DraxHardware::write(
    const rclcpp::Time &,
    const rclcpp::Duration &)
{
    // Later this will send commands to the Arduino.

    RCLCPP_INFO(
        rclcpp::get_logger("DraxHardware"),
        "Left Cmd: %.2f Right Cmd: %.2f",
        left_wheel_command_,
        right_wheel_command_);

    return hardware_interface::return_type::OK;
}

} // namespace drax_hardware

PLUGINLIB_EXPORT_CLASS(
    drax_hardware::DraxHardware,
    hardware_interface::SystemInterface)










//============================
#ifndef DRAX_HARDWARE_HPP_
#define DRAX_HARDWARE_HPP_

#include <vector>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"   // <-- Add this
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"

#include "rclcpp_lifecycle/state.hpp"
namespace drax_hardware
{

class DraxHardware : public hardware_interface::SystemInterface
{
public:
    DraxHardware() = default;
    ~DraxHardware() = default;

    //=========================================================
    // Lifecycle Functions
    //=========================================================

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    //=========================================================
    // Export Interfaces
    //=========================================================

    std::vector<hardware_interface::StateInterface>
    export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface>
    export_command_interfaces() override;

    //=========================================================
    // Read / Write
    //=========================================================

    hardware_interface::return_type read(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

private:

    //=========================================================
    // Wheel States
    //=========================================================

    double left_wheel_position_ = 0.0;
    double right_wheel_position_ = 0.0;

    double left_wheel_velocity_ = 0.0;
    double right_wheel_velocity_ = 0.0;

    //=========================================================
    // Commands from diff_drive_controller
    //=========================================================

    double left_wheel_command_ = 0.0;
    double right_wheel_command_ = 0.0;
};

} // namespace drax_hardware

#endif