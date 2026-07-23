#include "drax_hardware/drax_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <thread>
#include <chrono>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace drax_hardware
{
namespace
{
constexpr double PI = 3.14159265358979323846;

rclcpp::Logger logger()
{
    return rclcpp::get_logger("DraxHardware");
}

std::string paramOr(
    const hardware_interface::HardwareInfo & info,
    const std::string & name,
    const std::string & default_value)
{
    const auto it = info.hardware_parameters.find(name);
    return it == info.hardware_parameters.end() ? default_value : it->second;
}

int intParamOr(
    const hardware_interface::HardwareInfo & info,
    const std::string & name,
    int default_value)
{
    return std::stoi(paramOr(info, name, std::to_string(default_value)));
}

double doubleParamOr(
    const hardware_interface::HardwareInfo & info,
    const std::string & name,
    double default_value)
{
    return std::stod(paramOr(info, name, std::to_string(default_value)));
}

bool boolParamOr(
    const hardware_interface::HardwareInfo & info,
    const std::string & name,
    bool default_value)
{
    const auto value = paramOr(info, name, default_value ? "true" : "false");
    return value == "true" || value == "1" || value == "yes" || value == "on";
}

uint16_t indexParamOr(
    const hardware_interface::HardwareInfo & info,
    const std::string & name,
    uint16_t default_value)
{
    return static_cast<uint16_t>(
        std::stoul(paramOr(info, name, std::to_string(default_value)), nullptr, 0));
}

uint8_t subindexParamOr(
    const hardware_interface::HardwareInfo & info,
    const std::string & name,
    uint8_t default_value)
{
    return static_cast<uint8_t>(intParamOr(info, name, default_value));
}

} // namespace

hardware_interface::CallbackReturn
DraxHardware::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;

    try
    {
        can_interface_ = info_.hardware_parameters.at("can_interface");
        left_node_id_ = static_cast<uint8_t>(intParamOr(info_, "left_node_id", 1));
        right_node_id_ = static_cast<uint8_t>(intParamOr(info_, "right_node_id", 2));
        encoder_resolution_ = doubleParamOr(info_, "encoder_resolution", 4096.0);
        wheel_radius_ = doubleParamOr(info_, "wheel_radius", 0.09);

        motor_config_.feedback_resolution = intParamOr(info_, "feedback_resolution", 4096);
        motor_config_.fix_din3 = boolParamOr(info_, "fix_din3", true);

        sdo_timeout_ = std::chrono::milliseconds(intParamOr(info_, "sdo_timeout_ms", 20));
        sdo_retries_ = std::max(1, intParamOr(info_, "sdo_retries", 3));
        diagnostics_period_sec_ = doubleParamOr(info_, "diagnostics_period_sec", 1.0);

        motor_config_.temperature_index =
            indexParamOr(info_, "diagnostic_temperature_index", 0);
        motor_config_.temperature_subindex =
            subindexParamOr(info_, "diagnostic_temperature_subindex", 0);
        motor_config_.temperature_scale =
            doubleParamOr(info_, "diagnostic_temperature_scale", 1.0);

        motor_config_.voltage_index =
            indexParamOr(info_, "diagnostic_voltage_index", 0);
        motor_config_.voltage_subindex =
            subindexParamOr(info_, "diagnostic_voltage_subindex", 0);
        motor_config_.voltage_scale =
            doubleParamOr(info_, "diagnostic_voltage_scale", 1.0);
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(logger(), "Missing or invalid hardware parameter: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(logger(), "Hardware initialized");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
DraxHardware::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(logger(), "Opening CAN interface %s", can_interface_.c_str());

    bus_ = std::make_unique<SocketCanBus>(logger());

    if (!bus_->open(can_interface_))
    {
        RCLCPP_ERROR(logger(), "Failed to open CAN interface");
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (!bus_->sendNMTStart(0))
    {
        RCLCPP_WARN(logger(), "Failed to send CANopen NMT start");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    left_node_ = std::make_unique<CanOpenNode>(
        *bus_, left_node_id_, rclcpp::get_logger("DraxHardware.LeftCanOpenNode"),
        sdo_timeout_, sdo_retries_);

    right_node_ = std::make_unique<CanOpenNode>(
        *bus_, right_node_id_, rclcpp::get_logger("DraxHardware.RightCanOpenNode"),
        sdo_timeout_, sdo_retries_);

    left_motor_ = std::make_unique<Motor>(
        *left_node_, motor_config_, rclcpp::get_logger("DraxHardware.LeftMotor"));

    right_motor_ = std::make_unique<Motor>(
        *right_node_, motor_config_, rclcpp::get_logger("DraxHardware.RightMotor"));

    if (!left_motor_->initSpeedMode() || !right_motor_->initSpeedMode())
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    diagnostics_node_ = rclcpp::Node::make_shared("drax_hardware_diagnostics");
    diagnostics_updater_ = std::make_unique<diagnostic_updater::Updater>(diagnostics_node_);
    diagnostics_updater_->setHardwareID("drax_hardware");
    diagnostics_updater_->add("motor_status", this, &DraxHardware::updateDiagnostics);
    last_diagnostics_update_ = diagnostics_node_->now();

    uint16_t status = 0;
    if (left_motor_->readStatusWord(status))
    {
        RCLCPP_INFO(logger(), "Left status = 0x%04X", status);
    }
    if (right_motor_->readStatusWord(status))
    {
        RCLCPP_INFO(logger(), "Right status = 0x%04X", status);
    }

    RCLCPP_INFO(logger(), "Hardware configured successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
DraxHardware::on_activate(const rclcpp_lifecycle::State &)
{
    left_wheel_position_ = 0.0;
    right_wheel_position_ = 0.0;
    left_wheel_velocity_ = 0.0;
    right_wheel_velocity_ = 0.0;
    left_wheel_command_ = 0.0;
    right_wheel_command_ = 0.0;

    left_consecutive_read_failures_ = 0;
    right_consecutive_read_failures_ = 0;

    if (!left_motor_->stop() || !right_motor_->stop())
    {
        RCLCPP_ERROR(logger(), "Failed to stop motors during activation");
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (!left_motor_->readEncoder(last_left_encoder_))
    {
        RCLCPP_ERROR(logger(), "Failed to read left encoder");
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (!right_motor_->readEncoder(last_right_encoder_))
    {
        RCLCPP_ERROR(logger(), "Failed to read right encoder");
        return hardware_interface::CallbackReturn::ERROR;
    }

    left_encoder_ = last_left_encoder_;
    right_encoder_ = last_right_encoder_;

    RCLCPP_INFO(logger(), "Hardware activated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
DraxHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
    if (left_motor_ && !left_motor_->quickStop())
    {
        RCLCPP_WARN(logger(), "Failed to quick-stop left motor");
    }

    if (right_motor_ && !right_motor_->quickStop())
    {
        RCLCPP_WARN(logger(), "Failed to quick-stop right motor");
    }

    if (left_motor_ && !left_motor_->disable())
    {
        RCLCPP_WARN(logger(), "Failed to disable left motor");
    }

    if (right_motor_ && !right_motor_->disable())
    {
        RCLCPP_WARN(logger(), "Failed to disable right motor");
    }

    if (bus_)
    {
        bus_->close();
    }

    diagnostics_updater_.reset();
    diagnostics_node_.reset();

    RCLCPP_INFO(logger(), "Hardware deactivated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DraxHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(
        info_.joints[0].name, hardware_interface::HW_IF_POSITION, &left_wheel_position_);
    state_interfaces.emplace_back(
        info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &left_wheel_velocity_);
    state_interfaces.emplace_back(
        info_.joints[1].name, hardware_interface::HW_IF_POSITION, &right_wheel_position_);
    state_interfaces.emplace_back(
        info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &right_wheel_velocity_);

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DraxHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(
        info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &left_wheel_command_);
    command_interfaces.emplace_back(
        info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &right_wheel_command_);

    return command_interfaces;
}

hardware_interface::return_type
DraxHardware::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
    int32_t left_ticks = last_left_encoder_;
    int32_t right_ticks = last_right_encoder_;

    if (!left_motor_->readEncoder(left_ticks))
    {
        ++left_consecutive_read_failures_;
        RCLCPP_WARN(logger(), "Left motor encoder read failed (%d/%d)",
                    left_consecutive_read_failures_, MAX_CONSECUTIVE_READ_FAILURES);

        if (left_consecutive_read_failures_ > MAX_CONSECUTIVE_READ_FAILURES)
        {
            RCLCPP_ERROR(logger(), "Left motor exceeded consecutive encoder read failure limit");
            return hardware_interface::return_type::ERROR;
        }
    }
    else
    {
        left_consecutive_read_failures_ = 0;
    }

    if (!right_motor_->readEncoder(right_ticks))
    {
        ++right_consecutive_read_failures_;
        RCLCPP_WARN(logger(), "Right motor encoder read failed (%d/%d)",
                    right_consecutive_read_failures_, MAX_CONSECUTIVE_READ_FAILURES);

        if (right_consecutive_read_failures_ > MAX_CONSECUTIVE_READ_FAILURES)
        {
            RCLCPP_ERROR(logger(), "Right motor exceeded consecutive encoder read failure limit");
            return hardware_interface::return_type::ERROR;
        }
    }
    else
    {
        right_consecutive_read_failures_ = 0;
    }

    const double dt = period.seconds();
    if (dt <= 0.0)
    {
        return hardware_interface::return_type::ERROR;
    }

    const int32_t delta_left = static_cast<int32_t>(
        static_cast<uint32_t>(left_ticks) - static_cast<uint32_t>(last_left_encoder_));
    const int32_t delta_right = static_cast<int32_t>(
        static_cast<uint32_t>(right_ticks) - static_cast<uint32_t>(last_right_encoder_));

    last_left_encoder_ = left_ticks;
    last_right_encoder_ = right_ticks;
    left_encoder_ = left_ticks;
    right_encoder_ = right_ticks;

    const double left_delta_rad = delta_left * 2.0 * PI / encoder_resolution_;
    const double right_delta_rad = delta_right * 2.0 * PI / encoder_resolution_;

    left_wheel_position_ += left_delta_rad;
    right_wheel_position_ += right_delta_rad;
    left_wheel_velocity_ = left_delta_rad / dt;
    right_wheel_velocity_ = right_delta_rad / dt;

    if (diagnostics_updater_ && diagnostics_node_ &&
        (diagnostics_node_->now() - last_diagnostics_update_).seconds() >= diagnostics_period_sec_)
    {
        diagnostics_updater_->force_update();
        last_diagnostics_update_ = diagnostics_node_->now();
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
DraxHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    const int left_rpm = static_cast<int>(left_wheel_command_ * 60.0 / (2.0 * PI));
    const int right_rpm = static_cast<int>(right_wheel_command_ * 60.0 / (2.0 * PI));

    RCLCPP_DEBUG(
        logger(),
        "Command: left=%.2f rad/s right=%.2f rad/s -> left=%d RPM right=%d RPM",
        left_wheel_command_, right_wheel_command_, left_rpm, right_rpm);

    const bool ok = left_motor_->setRPM(left_rpm) && right_motor_->setRPM(right_rpm);

    return ok ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

void DraxHardware::updateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Motors communicating");

    addMotorDiagnostics(stat, "left", left_motor_.get());
    addMotorDiagnostics(stat, "right", right_motor_.get());
}

void DraxHardware::addMotorDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper & stat,
    const std::string & name,
    Motor * motor)
{
    if (!motor)
    {
        stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, name + " motor unavailable");
        return;
    }

    uint16_t status = 0;
    if (motor->readStatusWord(status))
    {
        stat.add(name + "_status_word", status);
    }
    else
    {
        stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                          name + " motor status read failed");
    }

    double temperature = 0.0;
    if (motor->readTemperature(temperature))
    {
        stat.add(name + "_temperature", temperature);
    }
    else if (motor_config_.temperature_index != 0)
    {
        stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                          name + " motor temperature read failed");
    }

    double voltage = 0.0;
    if (motor->readBusVoltage(voltage))
    {
        stat.add(name + "_bus_voltage", voltage);
    }
    else if (motor_config_.voltage_index != 0)
    {
        stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                          name + " motor bus voltage read failed");
    }
}

} // namespace drax_hardware

PLUGINLIB_EXPORT_CLASS(
    drax_hardware::DraxHardware,
    hardware_interface::SystemInterface)
