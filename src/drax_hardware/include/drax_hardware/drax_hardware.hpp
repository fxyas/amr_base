#ifndef DRAX_HARDWARE_HPP_
#define DRAX_HARDWARE_HPP_

#include <vector>
#include <string>
#include <chrono>

#include "drax_hardware/socket_can.hpp"
#include "drax_hardware/canopen_node.hpp"
#include "drax_hardware/motor.hpp"

#include <memory>
#include <cstdint>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"   // <-- Add this
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"

#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/node.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"

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
    void updateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

    void addMotorDiagnostics(
        diagnostic_updater::DiagnosticStatusWrapper & stat,
        const std::string & name,
        Motor * motor);

    //--------------------------------------------------
    // Hardware Information
    //--------------------------------------------------

    hardware_interface::HardwareInfo info_;

    //--------------------------------------------------
    // CAN Configuration
    //--------------------------------------------------

    std::string can_interface_;

    uint8_t left_node_id_;
    uint8_t right_node_id_;

    std::chrono::milliseconds sdo_timeout_{20};
    int sdo_retries_ = 3;
    
    //--------------------------------------------------
    // CAN Communication
    //--------------------------------------------------

    std::unique_ptr<SocketCanBus> bus_;

    std::unique_ptr<CanOpenNode> left_node_;
    std::unique_ptr<CanOpenNode> right_node_;

    std::unique_ptr<Motor> left_motor_;
    std::unique_ptr<Motor> right_motor_;

    MotorConfig motor_config_;

    //--------------------------------------------------
    // Diagnostics
    //--------------------------------------------------

    rclcpp::Node::SharedPtr diagnostics_node_;
    std::unique_ptr<diagnostic_updater::Updater> diagnostics_updater_;
    rclcpp::Time last_diagnostics_update_;
    double diagnostics_period_sec_ = 1.0;

    //--------------------------------------------------
    // Encoder Values
    //--------------------------------------------------

    int32_t left_encoder_ = 0;
    int32_t right_encoder_ = 0;

    int32_t last_left_encoder_ = 0;
    int32_t last_right_encoder_ = 0;

    //--------------------------------------------------
    // Wheel States
    //--------------------------------------------------

    double left_wheel_position_ = 0.0;
    double right_wheel_position_ = 0.0;

    double left_wheel_velocity_ = 0.0;
    double right_wheel_velocity_ = 0.0;

    //--------------------------------------------------
    // Commands
    //--------------------------------------------------

    double left_wheel_command_ = 0.0;
    double right_wheel_command_ = 0.0;

    //--------------------------------------------------
    // Robot Parameters
    //--------------------------------------------------

    static constexpr int MAX_CONSECUTIVE_READ_FAILURES = 10;

    int left_consecutive_read_failures_ = 0;
    int right_consecutive_read_failures_ = 0;

    //--------------------------------------------------
    // Robot Parameters
    //--------------------------------------------------

    double encoder_resolution_;

    double wheel_radius_;
};

} // namespace drax_hardware

#endif
