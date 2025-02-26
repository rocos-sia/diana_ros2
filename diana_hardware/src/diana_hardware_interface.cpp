//
// Created by think on 1/8/25.
//

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "diana_hardware/diana_hardware_interface.hpp"


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


namespace diana_hardware {

    using StateInterface = hardware_interface::StateInterface;
    using CommandInterface = hardware_interface::CommandInterface;

    DianaHardwareInterface::DianaHardwareInterface() : command_interfaces_info_({
                                                                                        {hardware_interface::HW_IF_EFFORT,   kNumberOfJoints,                 effort_interface_claimed_},
                                                                                        {hardware_interface::HW_IF_VELOCITY, kNumberOfJoints,                 velocity_joint_interface_claimed_},
                                                                                        {hardware_interface::HW_IF_POSITION, kNumberOfJoints,                 position_joint_interface_claimed_},
                                                                                        {k_HW_IF_CARTESIAN_VELOCITY,         hw_cartesian_velocities_.size(), velocity_cartesian_interface_claimed_},
                                                                                        {k_HW_IF_CARTESIAN_POSE,             hw_cartesian_pose_.size(),       pose_cartesian_interface_claimed_},
                                                                                }) {

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
        std::vector<CommandInterface> command_interfaces;
        command_interfaces.reserve(info_.joints.size());
        for (auto i = 0U; i < info_.joints.size(); i++) {
            command_interfaces.emplace_back(CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
                                                             &hw_effort_commands_.at(i)));
            command_interfaces.emplace_back(CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
                                                             &hw_velocity_commands_.at(i)));
            command_interfaces.emplace_back(CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION,
                                                             &hw_position_commands_.at(i)));
        }

        // cartesian velocity command interface 6 in order: dx, dy, dz, wx, wy, wz
        for (auto i = 0U; i < hw_cartesian_velocities_.size(); i++) {
            command_interfaces.emplace_back(
                    CommandInterface(hw_cartesian_velocities_names_.at(i), k_HW_IF_CARTESIAN_VELOCITY,
                                     &hw_cartesian_velocities_.at(i)));
        }

        // cartesian pose command interface 16 element pose matrix
        for (auto i = 0U; i < 16; i++) {
            command_interfaces.emplace_back(
                    CommandInterface(std::to_string(i), k_HW_IF_CARTESIAN_POSE, &hw_cartesian_pose_.at(i)));
        }

        return command_interfaces;
    }

    std::vector<hardware_interface::StateInterface> DianaHardwareInterface::export_state_interfaces() {
        std::vector<StateInterface> state_interfaces;
        for (auto i = 0U; i < info_.joints.size(); i++) {
            state_interfaces.emplace_back(
                    StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_.at(i)));
            state_interfaces.emplace_back(
                    StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_.at(i)));
            state_interfaces.emplace_back(
                    StateInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_.at(i)));
            state_interfaces.emplace_back(
                    StateInterface(info_.joints[i].name, k_HW_IF_INITIAL_POSITION, &initial_joint_positions_.at(i)));
        }

        // initial cartesian pose state interface 16 element pose matrix
        for (auto i = 0U; i < 16; i++) {
            state_interfaces.emplace_back(
                    StateInterface(std::to_string(i), k_HW_IF_INITIAL_CARTESIAN_POSE, &initial_robot_pose_.at(i)));
        }

        return state_interfaces;
    }

    CallbackReturn
    DianaHardwareInterface::on_init(const hardware_interface::HardwareInfo &hardware_info) {
        // First pass to parent as ros2 control required.
        if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }


        return CallbackReturn::SUCCESS;
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

    rclcpp::Logger DianaHardwareInterface::getLogger() {
        return rclcpp::get_logger("DianaHardwareInterface");
    }

} // diana_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(diana_hardware::DianaHardwareInterface, hardware_interface::SystemInterface)
