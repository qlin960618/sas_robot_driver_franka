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
#
# ################################################################
*/


#include <sstream>

#include <exception>
#include <dqrobotics/utils/DQ_Math.h>
#include <dqrobotics/interfaces/json11/DQ_JsonReader.h>
#include <sas_common/sas_common.h>
#include <sas_conversions/eigen3_std_conversions.h>
#include <sas_robot_driver/sas_robot_driver_ros.h>
#include "sas_robot_driver_franka.h"
#include <robot_dynamic/qros_robot_dynamics_provider.h>


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
        throw std::runtime_error(ros::this_node::getName() + "::Error setting the signal int handler.");
    }

    ros::init(argc, argv, "sas_robot_driver_franka_node", ros::init_options::NoSigintHandler);
    ROS_WARN("=====================================================================");
    ROS_WARN("----------------------Juan Jose Quiroz Omana------------------------");
    ROS_WARN("=====================================================================");
    ROS_INFO_STREAM(ros::this_node::getName()+"::Loading parameters from parameter server.");


    ros::NodeHandle nh;
    sas::RobotDriverFrankaConfiguration robot_driver_franka_configuration;
    RobotInterfaceFranka::FrankaInterfaceConfiguration franka_interface_configuration;

    sas::get_ros_param(nh,"/robot_ip_address",robot_driver_franka_configuration.ip_address);
    sas::get_ros_param(nh,"/robot_mode",      robot_driver_franka_configuration.mode);
    double upper_scale_factor = 1.0;
    std::vector<std::string> all_params;
    if(nh.hasParam(ros::this_node::getName()+"/force_upper_limits_scaling_factor"))
    {
        sas::get_ros_param(nh,"/force_upper_limits_scaling_factor",upper_scale_factor);
        ROS_WARN_STREAM(ros::this_node::getName()+"::Set force upper limits scaling factor: " << upper_scale_factor);
    }
    if(nh.hasParam(ros::this_node::getName()+"/upper_torque_threshold")) {
        ROS_INFO_STREAM(ros::this_node::getName()+"::Upper torque threshold override and set.");
        std::vector<double> upper_torque_threshold_std_vec;
        sas::get_ros_param(nh,"/upper_torque_threshold",upper_torque_threshold_std_vec);
        franka_interface_configuration.upper_torque_threshold = std_vec_to_std_array<double,7>(upper_torque_threshold_std_vec);
    }else {
        ROS_INFO_STREAM(ros::this_node::getName()+"::Upper torque threshold not set. Using default with value scalling.");
        franka_interface_configuration.upper_torque_threshold = apply_scale_to_std_array(franka_interface_configuration.upper_torque_threshold, upper_scale_factor);
    }
    if(nh.hasParam(ros::this_node::getName()+"/upper_force_threshold")) {
        ROS_INFO_STREAM(ros::this_node::getName()+"::Upper force threshold override and set.");
        std::vector<double> upper_torque_threshold_std_vec;
        sas::get_ros_param(nh,"/upper_force_threshold",upper_torque_threshold_std_vec);
        franka_interface_configuration.upper_force_threshold = std_vec_to_std_array<double,6>(upper_torque_threshold_std_vec);
    }else {
        ROS_INFO_STREAM(ros::this_node::getName()+"::Upper force threshold not set. Using default with value scalling.");
        franka_interface_configuration.upper_force_threshold = apply_scale_to_std_array(franka_interface_configuration.upper_force_threshold, upper_scale_factor);
    }
    if(nh.hasParam(ros::this_node::getName()+"/robot_parameter_file_path"))
    {
        std::string robot_parameter_file_path;
        sas::get_ros_param(nh,"/robot_parameter_file_path",robot_parameter_file_path);
        ROS_INFO_STREAM(ros::this_node::getName()+"::Loading robot parameters from file: " + robot_parameter_file_path);
        const auto robot = DQ_JsonReader::get_from_json<DQ_SerialManipulatorDH>(robot_parameter_file_path);
        robot_driver_franka_configuration.robot_reference_frame = robot.get_reference_frame();
    }else{ROS_INFO_STREAM(ros::this_node::getName()+"::Robot parameter file path not set. Robot Model Unknow.");}

    robot_driver_franka_configuration.interface_configuration = franka_interface_configuration;

    sas::RobotDriverROSConfiguration robot_driver_ros_configuration;

    sas::get_ros_param(nh,"/thread_sampling_time_nsec",robot_driver_ros_configuration.thread_sampling_time_nsec);
    sas::get_ros_param(nh,"/q_min",robot_driver_ros_configuration.q_min);
    sas::get_ros_param(nh,"/q_max",robot_driver_ros_configuration.q_max);

    // initialize the robot dynamic provider
    robot_driver_ros_configuration.robot_driver_provider_prefix = ros::this_node::getName();
    qros::RobotDynamicProvider robot_dynamic_provider(nh, robot_driver_ros_configuration.robot_driver_provider_prefix);
    if(robot_driver_franka_configuration.robot_reference_frame!=0)
    {
        robot_dynamic_provider.set_world_to_base_tf(robot_driver_franka_configuration.robot_reference_frame);
    }

    try
        {
            ROS_INFO_STREAM(ros::this_node::getName()+"::Instantiating Franka robot.");
            sas::RobotDriverFranka robot_driver_franka(
                &robot_dynamic_provider,
                robot_driver_franka_configuration,
                &kill_this_process
            );
            //robot_driver_franka.set_joint_limits({qmin, qmax});
            ROS_INFO_STREAM(ros::this_node::getName()+"::Instantiating RobotDriverROS.");
            sas::RobotDriverROS robot_driver_ros(nh,
                                                &robot_driver_franka, 
                                                robot_driver_ros_configuration,
                                                &kill_this_process);
            robot_driver_ros.control_loop();
        }
    catch (const std::exception& e)
    {
        kill_this_process = true;
        ROS_ERROR_STREAM(ros::this_node::getName() + "::Exception::" + e.what());
    }     
 

  return 0;
}
