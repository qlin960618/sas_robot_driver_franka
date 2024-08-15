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
#         - Adapted from sas_robot_driver_denso_node.cpp
#           (https://github.com/SmartArmStack/sas_robot_driver_denso/blob/master/src/sas_robot_driver_denso_node.cpp)
#      2. Quentin Lin
#         - Adaption to ROS2
#
# ################################################################
*/


#include <sstream>

#include <exception>
#include <dqrobotics/utils/DQ_Math.h>
#include <dqrobotics/interfaces/json11/DQ_JsonReader.h>
#include <sas_common/sas_common.hpp>
#include <sas_conversions/eigen3_std_conversions.hpp>
#include <sas_robot_driver/sas_robot_driver_ros.hpp>
#include <sas_robot_driver_franka/sas_robot_driver_franka.h>
#include <sas_robot_driver_franka/robot_dynamic/qros_robot_dynamics_server.hpp>


/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>
static std::atomic_bool kill_this_process(false);
void sig_int_handler(int)
{
    kill_this_process = true;
}

template<typename T, std::size_t N>
std::array<T, N> apply_scale_to_std_array(const std::array<T, N>& array, const T& scale)
{
    std::array<T, N> scaled_array;
    for(std::size_t i = 0; i < N; i++)
    {
        scaled_array[i] = array[i] * scale;
    }
    return scaled_array;
}
template<typename T, std::size_t N>
std::array<T, N> std_vec_to_std_array(const std::vector<T>& vector)
{
    if(N != vector.size()){throw std::runtime_error("Size mismatch between vector and array.");}
    std::array<T, N> array;
    for(std::size_t i = 0; i < N; i++)
    {
        array[i] = vector[i];
    }
    return array;
}


int main(int argc, char **argv)
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
    {
        throw std::runtime_error("Error setting the signal int handler.");
    }

    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("sas_robot_driver_franka_node");

    const auto node_name = std::string(node->get_name());
    RCLCPP_WARN(node->get_logger(),"=====================================================================");
    RCLCPP_WARN(node->get_logger(),"-----------------Adapted by Quentin Lin ------------------------");
    RCLCPP_WARN(node->get_logger(),"=====================================================================");
    RCLCPP_INFO_STREAM_ONCE(node->get_logger(),"["+node_name+"]::Loading parameters from parameter server.");
    
    sas::RobotDriverFrankaConfiguration robot_driver_franka_configuration;
    RobotInterfaceFranka::FrankaInterfaceConfiguration franka_interface_configuration;

    sas::get_ros_parameter(node,"robot_ip_address",robot_driver_franka_configuration.ip_address);
    sas::get_ros_parameter(node,"robot_mode",      robot_driver_franka_configuration.mode);
    double upper_scale_factor = 1.0;
    std::vector<std::string> all_params;

    try {
        sas::get_ros_parameter(node,"force_upper_limits_scaling_factor",upper_scale_factor);
        RCLCPP_WARN_STREAM_ONCE(node->get_logger(),"Set force upper limits scaling factor: " << upper_scale_factor);
    }catch(...) {
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(),"Force upper limits scaling factor is not set.");
    }

    try{node->declare_parameter<std::vector<double>>("upper_torque_threshold");}catch (...){}
    if(node->has_parameter("upper_torque_threshold")) {
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(),"["+node_name+"]::Upper torque threshold override and set.");
        std::vector<double> upper_torque_threshold_std_vec;
        sas::get_ros_parameter(node,"upper_torque_threshold",upper_torque_threshold_std_vec);
        franka_interface_configuration.upper_torque_threshold = std_vec_to_std_array<double,7>(upper_torque_threshold_std_vec);
    }else {
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(),"["+node_name+"]::Upper torque threshold not set. Using default with value scalling.");
        franka_interface_configuration.upper_torque_threshold = apply_scale_to_std_array(franka_interface_configuration.upper_torque_threshold, upper_scale_factor);
    }
    try{node->declare_parameter<std::vector<double>>("upper_force_threshold");}catch (...){}
    if(node->has_parameter("upper_force_threshold")) {
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(),"["+node_name+"]::Upper force threshold override and set.");
        std::vector<double> upper_torque_threshold_std_vec;
        sas::get_ros_parameter(node,"upper_force_threshold",upper_torque_threshold_std_vec);
        franka_interface_configuration.upper_force_threshold = std_vec_to_std_array<double,6>(upper_torque_threshold_std_vec);
    }else {
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(),"["+node_name+"]::Upper force threshold not set. Using default with value scalling.");
        franka_interface_configuration.upper_force_threshold = apply_scale_to_std_array(franka_interface_configuration.upper_force_threshold, upper_scale_factor);
    }
    try{node->declare_parameter<std::string>("robot_parameter_file_path");}catch (...){}
    if(node->has_parameter("robot_parameter_file_path"))
    {
        std::string robot_parameter_file_path;
        sas::get_ros_parameter(node,"robot_parameter_file_path",robot_parameter_file_path);
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(),"["+node_name+"]::Loading robot parameters from file: " + robot_parameter_file_path);
        const auto robot = DQ_JsonReader::get_from_json<DQ_SerialManipulatorDH>(robot_parameter_file_path);
        robot_driver_franka_configuration.robot_reference_frame = robot.get_reference_frame();
    }else{RCLCPP_INFO_STREAM_ONCE(node->get_logger(),"["+node_name+"]::Robot parameter file path not set. Robot Model Unknow.");}

    robot_driver_franka_configuration.interface_configuration = franka_interface_configuration;

    sas::RobotDriverROSConfiguration robot_driver_ros_configuration;

    sas::get_ros_parameter(node,"thread_sampling_time_sec",robot_driver_ros_configuration.thread_sampling_time_sec);
    sas::get_ros_parameter(node,"q_min",robot_driver_ros_configuration.q_min);
    sas::get_ros_parameter(node,"q_max",robot_driver_ros_configuration.q_max);

    // initialize the robot dynamic provider
    robot_driver_ros_configuration.robot_driver_provider_prefix = node_name;
    std::shared_ptr<qros::RobotDynamicsServer> robot_dynamic_provider_ptr = std::make_shared<qros::RobotDynamicsServer>(node, robot_driver_ros_configuration.robot_driver_provider_prefix);
    if(robot_driver_franka_configuration.robot_reference_frame!=0)
    {
        robot_dynamic_provider_ptr->set_world_to_base_tf(robot_driver_franka_configuration.robot_reference_frame);
    }

    try
        {
            RCLCPP_INFO_STREAM_ONCE(node->get_logger(),"["+node_name+"]::Instantiating Franka robot.");
            auto robot_driver_franka = std::make_shared<sas::RobotDriverFranka>(
                node,
                robot_dynamic_provider_ptr,
                robot_driver_franka_configuration,
                &kill_this_process
            );
            //robot_driver_franka.set_joint_limits({qmin, qmax});
            RCLCPP_INFO_STREAM_ONCE(node->get_logger(),"["+node_name+"]::Instantiating RobotDriverROS.");
            sas::RobotDriverROS robot_driver_ros(node,
                                                robot_driver_franka, 
                                                robot_driver_ros_configuration,
                                                &kill_this_process);
            robot_driver_ros.control_loop();
        }
    catch (const std::exception& e)
    {
        kill_this_process = true;
        RCLCPP_ERROR_STREAM_ONCE(node->get_logger(),"["+node_name+"]::Exception::" + e.what());
    }     
 

  return 0;
}
