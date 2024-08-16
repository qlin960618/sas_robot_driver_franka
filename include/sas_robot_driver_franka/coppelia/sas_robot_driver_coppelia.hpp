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
#         - Adapted from sas_robot_driver_denso.cpp
#           (https://github.com/SmartArmStack/sas_robot_driver_denso/blob/master/src/sas_robot_driver_denso.cpp)
#
# ################################################################
*/


#pragma once
#include <exception>
#include <tuple>
#include <atomic>
#include <vector>
#include <memory>
#include <dqrobotics/DQ.h>
#include <sas_core/sas_clock.hpp>
// #include <dqrobotics/interfaces/vrep/DQ_VrepRobot.h>
#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
#include <dqrobotics/interfaces/json11/DQ_JsonReader.h>
// #include <dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>
#include <thread>
#include <atomic>
#include <sas_robot_driver/sas_robot_driver_server.hpp>
#include <sas_robot_driver/sas_robot_driver_client.hpp>

#define VIRTUAL_ROBOT_SIMULATION_SAMPLING_TIME_SEC 0.002  // 2ms, 500Hz
#define REAL_ROBOT_INTERFACE_INIT_TIMEOUT_COUNT 500
using namespace DQ_robotics;
using namespace Eigen;

namespace qros
{



struct RobotDriverCoppeliaConfiguration
{

    double thread_sampling_time_sec;  // frontend vrep update rate
    int vrep_port;
    std::string vrep_ip;
    std::vector<std::string> vrep_joint_names;
    bool vrep_dynamically_enabled = false;
    std::string robot_mode;
    bool using_real_robot;
    std::string robot_topic_prefix;
    std::string robot_parameter_file_path;
    // VectorXd q_min;
    // VectorXd q_max;
};

class RobotDriverCoppelia
{
private:
    enum ControlMode{
        Position=0,
        Velocity
    };

    RobotDriverCoppeliaConfiguration configuration_;
    std::shared_ptr<Node> node_sptr_;

    sas::Clock clock_;
    std::atomic_bool* break_loops_;
    bool _should_shutdown() const {return (*break_loops_);}
    ControlMode robot_mode_;

    std::shared_ptr<DQ_VrepInterface> vi_;
    std::shared_ptr<sas::RobotDriverClient> real_robot_interface_ = nullptr;
    std::shared_ptr<sas::RobotDriverServer> robot_provider_ = nullptr;

    // backend thread for simulaton
    /**
     * Current simulation mechanism is not accounting for any robot dynamics, just the joint limits
     */
    std::thread velocity_control_simulation_thread_;
    VectorXd simulated_joint_positions_;
    VectorXd simulated_joint_velocities_;
    void start_simulation_thread();  // thread entry point
    void _velocity_control_simulation_thread_main();
    std::atomic_bool simulation_thread_started_{false};

    bool initialized_ = false;
    inline void _assert_initialized() const{
        if (!initialized_){throw std::runtime_error("[RobotDriverCoppelia] Robot driver not initialized");}
    }
    inline void _assert_is_alive() const{
        if(!configuration_.using_real_robot && !simulation_thread_started_){throw std::runtime_error("[RobotDriverCoppelia] Robot Simulation is not alive");}
    }

    void _start_control_loop();
protected:
    inline void _update_vrep_position(const VectorXd &joint_positions, const bool& force_update=false) const;
    inline void _update_vrep_velocity(const VectorXd & joint_velocity) const;

public:
    RobotDriverCoppelia(const RobotDriverCoppelia&)=delete;
    RobotDriverCoppelia()=delete;
    ~RobotDriverCoppelia();

    RobotDriverCoppelia(const std::shared_ptr<Node> &node_sptr, const RobotDriverCoppeliaConfiguration& configuration, std::atomic_bool* break_loops);

    int start_control_loop();  // public entry point

    void connect();
    void disconnect();

    void initialize();
    void deinitialize();

    std::tuple<VectorXd, VectorXd> joint_limits_;

};
}
