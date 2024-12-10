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
#include <thread>
#include <mutex>

// #define BLOCK_READ_IN_USED
// #define IN_TESTING

// #include <sas_robot_driver/sas_robot_driver.h>
#include <sas_core/sas_clock.hpp>

#include <rclcpp/service.hpp>
#include <rclcpp/rclcpp.hpp>

#ifdef IN_TESTING
#include "local/srv/grasp.hpp"
#include "local/srv/move.hpp"
#include "local/msg/gripper_state.hpp"
#else
#include <sas_robot_driver_franka_interfaces/srv/grasp.hpp>
#include <sas_robot_driver_franka_interfaces/srv/move.hpp>
#include <sas_robot_driver_franka_interfaces/msg/gripper_state.hpp>
#endif
#include <std_srvs/srv/trigger.hpp>

// using namespace DQ_robotics;
// using namespace Eigen;
using namespace rclcpp;
using namespace sas_robot_driver_franka_interfaces;


namespace qros {

struct EffectorDriverFrankaHandConfiguration
{
    std::string robot_ip;
    double thread_sampeling_time_s = 1e8;  // 10Hz
    double default_force = 3.0;
    double default_speed = 0.1;
    double default_epsilon_inner = 0.005;
    double default_epsilon_outer = 0.005;
};

class EffectorDriverFrankaHand{
private:
    std::string driver_node_prefix_;
    EffectorDriverFrankaHandConfiguration configuration_;
    std::shared_ptr<Node> node_;

    std::shared_ptr<franka::Gripper> gripper_sptr_;


    std::atomic_bool* break_loops_;

    bool _is_connected() const;

    // thread specific functions
    void _gripper_status_loop();
    std::thread status_loop_thread_;
    std::atomic_bool status_loop_running_{false};

    Publisher<msg::GripperState>::SharedPtr gripper_status_publisher_;

    std::mutex gripper_in_use_;
    Service<srv::Grasp>::SharedPtr grasp_srv_;
    Service<srv::Move>::SharedPtr move_srv_;
    Service<std_srvs::srv::Trigger>::SharedPtr homing_srv_;

public:

    bool _grasp_srv_callback(
        const std::shared_ptr<srv::Grasp::Request> &req,
        std::shared_ptr<srv::Grasp::Response> res
        );

    bool _move_srv_callback(
        const std::shared_ptr<srv::Move::Request> &req,
        std::shared_ptr<srv::Move::Response> res
        );

    bool _home_srv_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> &req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res
        );

    EffectorDriverFrankaHand(const EffectorDriverFrankaHand&)=delete;
    EffectorDriverFrankaHand()=delete;
    ~EffectorDriverFrankaHand();

    EffectorDriverFrankaHand(
        const std::string &driver_node_prefix,
        const EffectorDriverFrankaHandConfiguration& configuration,
        std::shared_ptr<Node> node,
        std::atomic_bool* break_loops);

    void start_control_loop();

    void gripper_homing();


    void connect() ;
    void disconnect() noexcept;

    void initialize() ;
    void deinitialize() ;
};

} // qros

