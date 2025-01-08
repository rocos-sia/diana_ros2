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

#include <array>
#include <atomic>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>


#include <rclcpp/logger.hpp>

#include "DianaAPI.h"

namespace diana_hardware {

class Robot {
 public:

    explicit Robot(std::string  robot_ip, const rclcpp::Logger& logger);

    virtual ~Robot();

    /// Stops the continuous communication read with the connected robot
    virtual void stopRobot();

    /// Starts torque control
    virtual void initializeTorqueInterface();

    /// Starts joint velocity control
    virtual void initializeJointVelocityInterface();

    /// Starts joint position control
    virtual void initializeJointPositionInterface();

    /// Starts cartesian velocity control
    virtual void initializeCartesianVelocityInterface();

    /// Starts cartesian pose control
    virtual void initializeCartesianPoseInterface();


protected:
    Robot() = default;

private:
    std::string ip_addr_str_ {"192.168.10.75"}; // Store the IP address string of Diana, default "192.168.10.75"


    static void errorControl(int e, const char* strIpAddress);
    static void logRobotState(StrRobotStateInfo *pinfo, const char* strIpAddress);


};
}  // namespace diana_hardware
