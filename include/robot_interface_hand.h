/*
# Copyright (c) 2023 Juan Jose Quiroz Omana
#
#    This file is part of sas_robot_driver_franka.
#
#    sas_robot_driver_franka is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_robot_driver_franka is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_robot_driver.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Juan Jose Quiroz Omana, email: juanjqogm@gmail.com
#
# ################################################################
#
# Contributors:
#      1. Juan Jose Quiroz Omana (juanjqogm@gmail.com)
#         - Original implementation
#
# ################################################################
*/


#pragma once

#include <dqrobotics/DQ.h>
#include <memory.h>
#include <tuple>
#include <exception>
#include <vector>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/exception.h>
#include <thread>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <atomic>

using namespace DQ_robotics;
using namespace Eigen;



class RobotInterfaceHand
{
protected:
    double speed_gripper_ = 0.02;
    std::string ip_ = "172.16.0.2";
    std::shared_ptr<franka::Gripper> gripper_sptr_;
    void _check_if_hand_is_connected();
public:
    enum GRIPPER_MODE_STATES{WIDTH=0, MAX_WIDTH=1};
    RobotInterfaceHand() = delete;
    RobotInterfaceHand(const RobotInterfaceHand&) = delete;
    RobotInterfaceHand& operator= (const RobotInterfaceHand&) = delete;
    RobotInterfaceHand(const std::string &ROBOT_IP);

    double read_gripper(const GRIPPER_MODE_STATES& gripper_state = WIDTH);
    bool read_grasped_status();
    void set_gripper(const double& width);
    void gripper_homing();
    bool grasp_object(const double& width,
                      const double& speed,
                      const double& force,
                      double 	epsilon_inner = 0.005,
                      double 	epsilon_outer = 0.005);
    void release_object();
    VectorXd get_home_robot_configuration();
};

