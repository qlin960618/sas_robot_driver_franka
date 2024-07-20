#pragma once
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
#      1. Quenitin Lin
#
# ################################################################
*/
#include <exception>
#include <tuple>
#include <atomic>
#include <vector>
#include <memory>
#include <franka/robot.h>
#include <franka/gripper.h>
// #include <thread>

#include <dqrobotics/DQ.h>

#include <sas_robot_driver/sas_robot_driver.h>
#include <sas_robot_driver/sas_robot_driver_ros.h>
#include <sas_clock/sas_clock.h>
#include <ros/common.h>

using namespace DQ_robotics;
using namespace Eigen;


namespace sas {

struct RobotDriverFrankaHandConfiguration
{
    std::string robot_ip;

};

class RobotDriverFrankaHand{
private:
    RobotDriverFrankaHandConfiguration configuration_;
    RobotDriverROSConfiguration ros_configuration_;

    std::shared_ptr<franka::Gripper> gripper_sptr_;

    //Joint positions
    VectorXd joint_positions_;
    //VectorXd joint_velocities_;
    //VectorXd end_effector_pose_;


    // std::thread control_loop_thread_;
    std::atomic_bool* break_loops_;

    bool _is_connected() const;

public:
    //const static int SLAVE_MODE_JOINT_CONTROL;
    //const static int SLAVE_MODE_END_EFFECTOR_CONTROL;

    RobotDriverFrankaHand(const RobotDriverFrankaHand&)=delete;
    RobotDriverFrankaHand()=delete;
    ~RobotDriverFrankaHand();

    RobotDriverFrankaHand(
        const RobotDriverFrankaHandConfiguration& configuration,
        const RobotDriverROSConfiguration& ros_configuration,
        std::atomic_bool* break_loops);

    void start_control_loop();



    VectorXd get_joint_positions();
    void set_target_joint_positions(const VectorXd& desired_joint_positions_rad);

    VectorXd get_joint_velocities();
    void set_target_joint_velocities(const VectorXd& desired_joint_velocities);

    VectorXd get_joint_forces();

    void gripper_homing();


    void connect() ;
    void disconnect() noexcept;

    void initialize() ;
    void deinitialize() ;

    //bool set_end_effector_pose_dq(const DQ& pose);
    //DQ get_end_effector_pose_dq();

};

} // sas

