// Copyright (c) 2025 Yang Luo
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// @Author
// Yang Luo, PHD
// Shenyang Institute of Automation, Chinese Academy of Sciences.
// email: luoyang@sia.cn

#pragma once

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <vector>

#include "diana_hardware/robot.hpp"

namespace diana_hardware {

    class DianaHardwareInterface : public hardware_interface::SystemInterface {
    public:
        explicit DianaHardwareInterface(std::shared_ptr<Robot> robot);

        DianaHardwareInterface();

        DianaHardwareInterface(const DianaHardwareInterface&) = delete;

        ~DianaHardwareInterface() override = default;


        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string> &start_interfaces,
                                                                    const std::vector<std::string> &stop_interfaces) override;

        hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string> &start_interfaces,
                                                                    const std::vector<std::string> &stop_interfaces) override;

        CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;

        static const size_t kNumberOfJoints = 7;

    private:
        struct InterfaceInfo {
            std::string interface_type;
            size_t size;
            bool& claim_flag;
        };

        std::shared_ptr<Robot> robot_;

        // Torque joint commands for the effort command interface
        std::array<double, kNumberOfJoints> hw_effort_commands_ {0, 0, 0, 0, 0, 0, 0};
        // Velocity joint commands for the velocity command interface
        std::array<double, kNumberOfJoints> hw_velocity_commands_ {0, 0, 0, 0, 0, 0, 0};
        // Position joint commands for the position command interface
        std::array<double, kNumberOfJoints> hw_position_commands_ {0, 0, 0, 0, 0, 0, 0};

        // Robot joint states
        std::array<double, kNumberOfJoints> hw_positions_ {0, 0, 0, 0, 0, 0, 0};
        std::array<double, kNumberOfJoints> hw_velocities_ {0, 0, 0, 0, 0, 0, 0};
        std::array<double, kNumberOfJoints> hw_efforts_ {0, 0, 0, 0, 0, 0, 0};
        std::array<double, kNumberOfJoints> initial_joint_positions_ {0, 0, 0, 0, 0, 0, 0};

        // Cartesian states
        std::array<double, 16> initial_robot_pose_ {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}; // Matrix 4x4 column major


        /**
         * Desired Cartesian velocity with respect to the o-frame
         * "base frame O" with (vx, vy, vz)\ in [m/s] and
         * (wx, wy, wz) in [rad/s].
         */
        std::array<std::string, 6> hw_cartesian_velocities_names_{"vx", "vy", "vz", "wx", "wy", "wz"};
        std::array<double, 6> hw_cartesian_velocities_{0, 0, 0, 0, 0, 0};

        // Pose is represented as a column-major homogeneous transformation matrix.
        std::array<double, 16> hw_cartesian_pose_{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};


        const std::string k_HW_IF_CARTESIAN_VELOCITY = "cartesian_velocity";
        const std::string k_HW_IF_CARTESIAN_POSE = "cartesian_pose";

        const std::string k_HW_IF_INITIAL_CARTESIAN_POSE = "initial_cartesian_pose";
        const std::string k_HW_IF_INITIAL_POSITION = "initial_joint_position";

        const std::vector<InterfaceInfo> command_interfaces_info_;

        bool effort_interface_claimed_ = false;
        bool effort_interface_running_ = false;

        bool velocity_joint_interface_claimed_ = false;
        bool velocity_joint_interface_running_ = false;

        bool position_joint_interface_claimed_ = false;
        bool position_joint_interface_running_ = false;

        bool velocity_cartesian_interface_claimed_ = false;
        bool velocity_cartesian_interface_running_ = false;

        bool pose_cartesian_interface_claimed_ = false;
        bool pose_cartesian_interface_running_ = false;

        static rclcpp::Logger getLogger();

        const std::string k_robot_state_interface_name {"robot_state"};

    };

} // diana_hardware

