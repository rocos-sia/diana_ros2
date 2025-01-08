// Copyright (c) 2023 Franka Robotics GmbH
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

#include <cassert>
#include <mutex>

#include <rclcpp/logging.hpp>
#include <utility>

#include "diana_hardware/robot.hpp"

namespace diana_hardware {

    Robot::Robot(std::string robot_ip, const rclcpp::Logger& logger) : ip_addr_str_(std::move(robot_ip)) {
        auto version = getLibraryVersion();
        RCLCPP_INFO(logger, "Diana API version: %d.%d", version >> 8, version & 0xFF);

        // construct srv_net_st for Diana
        srv_net_st info;
        memset(info.SrvIp, 0x00, sizeof(info.SrvIp));
        memcpy(info.SrvIp, ip_addr_str_.c_str(), ip_addr_str_.size());
        info.LocHeartbeatPort = 0;
        info.LocRobotStatePort = 0;
        info.LocSrvPort = 0;
        int ret = initSrv(errorControl, logRobotState, &info);
        if(ret < 0) {
            throw std::runtime_error("Failed to initialize the robot.");
        }


    }

    void Robot::errorControl(int e, const char *strIpAddress) {
        const char* strError = formatError(e, strIpAddress);

    }

    void Robot::logRobotState(StrRobotStateInfo *pinfo, const char *strIpAddress) {

    }

    Robot::~Robot() {
        stopRobot();
    }

    void Robot::stopRobot() {
        /// Stop Diana 7 first.
        int ret = stop(ip_addr_str_.c_str());
        if(ret < 0) {
            throw std::runtime_error("Failed to stop the robot.");
        }
        /// Destroy the connection.
        ret = destroySrv(ip_addr_str_.c_str());
        if(ret < 0) {
            throw std::runtime_error("Failed to destroy the connection.");
        }

    }


}  // namespace franka_hardware