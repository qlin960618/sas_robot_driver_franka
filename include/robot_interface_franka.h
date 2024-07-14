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
#include "motion_generator.h"
#include <thread>
#include "quadratic_program_motion_generator.h"
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <atomic>
#include "custom_motion_generation.h"

using namespace DQ_robotics;
using namespace Eigen;

class RobotInterfaceFranka
{
public: enum class MODE{
        None=0,
        PositionControl,
        VelocityControl,
        ForceControl,
        Homing,
        ClearPositions,
    };

private:
    using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
    std::string ip_ = "172.16.0.2";
    double speed_factor_joint_position_controller_ = 0.2;
    double speed_gripper_ = 0.02;
    VectorXd desired_joint_positions_;
    VectorXd desired_joint_velocities_ = VectorXd::Zero(7);

    VectorXd current_joint_positions_;
    std::array<double, 7> current_joint_positions_array_;

    VectorXd current_joint_velocities_;
    std::array<double, 7> current_joint_velocities_array_;

    VectorXd current_joint_forces_;
    std::array<double, 7> current_joint_forces_array_;

    franka::RobotMode robot_mode_;

    double time_ = 0;
    bool initialize_flag_ = false;
    void _check_if_robot_is_connected();
    void _check_if_hand_is_connected();

    RobotInterfaceFranka::MODE mode_;
    void _set_driver_mode(const RobotInterfaceFranka::MODE& mode);

    void _restart_default_parameters();


    void _update_robot_state(const franka::RobotState& robot_state, const double& time);

    std::shared_ptr<franka::Robot> robot_sptr_;
    std::shared_ptr<franka::Gripper> gripper_sptr_;
    std::unique_ptr<QuadraticProgramMotionGenerator> trajectory_generator_sptr_;
    std::unique_ptr<CustomMotionGeneration> custom_generator_sptr_;

    void _setDefaultRobotBehavior();
    //bool hand_enabled_ = false;
    const VectorXd q_home_configuration_ =(VectorXd(7)<<0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4).finished();

    const VectorXd q_max_ = ((VectorXd(7) <<  2.3093, 1.5133, 2.4937, -0.4461, 2.4800, 4.2094,  2.6895).finished());
    const VectorXd q_min_ = ((VectorXd(7) << -2.3093,-1.5133,-2.4937, -2.7478,-2.4800, 0.8521, -2.6895).finished());
    const VectorXd q_min_dot_ = ((VectorXd(7) << -2, -1, -1.5, -1.25, -3, -1.5, -3).finished());
    const VectorXd q_max_dot_ = ((VectorXd(7) <<  2,  1,  1.5,  1.25,  3,  1.5,  3).finished());

    const double samples_ = 20;

    VectorXd desired_mean_joint_positions_;

    bool verbose_ = true;
    std::string status_message_ = " ";
    void _update_status_message(const std::string& message, const bool& verbose = true);


    //-------To handle the threads-----------------
    std::atomic<bool> exit_on_err_={false};


    void _start_joint_position_control_mode();
    std::thread joint_position_control_mode_thread_;
    void _start_joint_position_control_thread();
    std::atomic<bool> finish_motion_;

    void _start_echo_robot_state_mode();
    std::thread echo_robot_state_mode_thread_;
    void _start_echo_robot_state_mode_thread();
    std::atomic<bool> finish_echo_robot_state_;
    void _finish_echo_robot_state();

    void _start_joint_velocity_control_mode();
    std::thread joint_velocity_control_mode_thread_;
    void _start_joint_velocity_control_thread();
    //----------------------------------------------



    VectorXd _compute_recursive_mean(const double& samples, const VectorXd& q);
    VectorXd _read_once_smooth_initial_positions(const double& samples);


public:


    enum HAND{ON=0, OFF};
    enum CONNECTION{AUTOMATIC};
    enum GRIPPER_STATES{WIDTH=0, MAX_WIDTH=1};
    RobotInterfaceFranka() = delete;
    RobotInterfaceFranka(const RobotInterfaceFranka&) = delete;
    RobotInterfaceFranka& operator= (const RobotInterfaceFranka&) = delete;
    RobotInterfaceFranka(const std::string &ROBOT_IP, const MODE& mode, const HAND& hand = ON);



    VectorXd read_once_initial_positions();

    VectorXd get_joint_target_positions();

    void move_robot_to_target_joint_positions(const VectorXd& q_target);




    double read_gripper(const GRIPPER_STATES& gripper_state = WIDTH);
    bool read_grasped_status();
    void set_gripper(const double& width);
    void gripper_homing();
    VectorXd get_home_robot_configuration();

    std::string get_status_message();

    bool get_err_state() const {return exit_on_err_.load();}

    std::string get_robot_mode();


    void finish_motion();

    std::shared_ptr<franka::Robot> get_robot_pointer();


    //--------sas compatible methods----------//
    VectorXd get_joint_positions();
    VectorXd get_joint_velocities();
    VectorXd get_joint_forces();

    double get_time();

    void set_target_joint_positions(const VectorXd& set_target_joint_positions_rad);
    void connect();
    void initialize();
    void deinitialize();
    void disconnect();

    void set_target_joint_velocities(const VectorXd& target_joint_velocities);






    /* To be implemented
    std::tuple<VectorXd, VectorXd> get_joint_limits() const;
    void set_joint_limits(const std::tuple<VectorXd, VectorXd>& joint_limits);
    */


};


