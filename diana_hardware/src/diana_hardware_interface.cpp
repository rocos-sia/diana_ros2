//
// Created by think on 1/8/25.
//

#include "diana_hardware/diana_hardware_interface.hpp"



using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


namespace diana_hardware {

    DianaHardwareInterface::DianaHardwareInterface() {

    }

    hardware_interface::return_type
    DianaHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
        return hardware_interface::return_type::ERROR;
    }

    hardware_interface::return_type
    DianaHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period) {
        return hardware_interface::return_type::ERROR;
    }

    std::vector<hardware_interface::CommandInterface> DianaHardwareInterface::export_command_interfaces() {
        return {};
    }

    std::vector<hardware_interface::StateInterface> DianaHardwareInterface::export_state_interfaces() {
        return {};
    }

    hardware_interface::return_type
    DianaHardwareInterface::prepare_command_mode_switch(const std::vector<std::string> &start_interfaces,
                                                        const std::vector<std::string> &stop_interfaces) {
        return SystemInterface::prepare_command_mode_switch(start_interfaces, stop_interfaces);
    }

    hardware_interface::return_type
    DianaHardwareInterface::perform_command_mode_switch(const std::vector<std::string> &start_interfaces,
                                                        const std::vector<std::string> &stop_interfaces) {
        return SystemInterface::perform_command_mode_switch(start_interfaces, stop_interfaces);
    }

    CallbackReturn
    DianaHardwareInterface::on_init(const hardware_interface::HardwareInfo &hardware_info) {
        return SystemInterface::on_init(hardware_info);
    }

    rclcpp::Logger DianaHardwareInterface::getLogger() {
        return rclcpp::get_logger("DianaHardwareInterface");
    }

} // diana_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(diana_hardware::DianaHardwareInterface, hardware_interface::SystemInterface)
