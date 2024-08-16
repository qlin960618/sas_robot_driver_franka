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
#      2. Quentin Lin (qlin1806@g.ecc.u-tokyo.ac.jp)
#         - Adapted for simplify operation
#         - porting to ROS2
#
# ################################################################
*/


#include <sstream>

#include <exception>
#include <dqrobotics/utils/DQ_Math.h>
//#include <dqrobotics/interfaces/json11/DQ_JsonReader.h>
#include <sas_common/sas_common.hpp>
#include <sas_conversions/eigen3_std_conversions.hpp>
#include <sas_robot_driver_franka/coppelia/sas_robot_driver_coppelia.hpp>

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>
static std::atomic_bool kill_this_process(false);
void sig_int_handler(int)
{
    kill_this_process = true;
}



int main(int argc, char **argv)
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
    {
        throw std::runtime_error("Error setting the signal int handler.");
    }

    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    auto context = rclcpp::contexts::get_global_default_context();

    auto node = std::make_shared<rclcpp::Node>("sas_robot_driver_franka_coppelia_node");

    const auto node_name = std::string(node->get_name());
    RCLCPP_WARN(node->get_logger(),"=====================================================================");
    RCLCPP_WARN(node->get_logger(),"-----------------Adapted by Quentin Lin ------------------------");
    RCLCPP_WARN(node->get_logger(),"=====================================================================");
    RCLCPP_INFO_STREAM_ONCE(node->get_logger(),":Loading parameters from parameter server.");


    qros::RobotDriverCoppeliaConfiguration robot_driver_coppelia_configuration;
    sas::get_ros_parameter(node,"thread_sampling_time_sec",robot_driver_coppelia_configuration.thread_sampling_time_sec);
    sas::get_ros_parameter(node,"vrep_ip",robot_driver_coppelia_configuration.vrep_ip);
    sas::get_ros_parameter(node,"vrep_port",robot_driver_coppelia_configuration.vrep_port);
    sas::get_ros_parameter(node,"vrep_joint_names", robot_driver_coppelia_configuration.vrep_joint_names);
    sas::get_ros_parameter(node,"vrep_dynamically_enabled", robot_driver_coppelia_configuration.vrep_dynamically_enabled);
    sas::get_ros_parameter(node,"robot_mode", robot_driver_coppelia_configuration.robot_mode);
    sas::get_ros_parameter(node,"using_real_robot", robot_driver_coppelia_configuration.using_real_robot);
    sas::get_ros_parameter(node,"robot_topic_prefix", robot_driver_coppelia_configuration.robot_topic_prefix);
    sas::get_ros_parameter(node,"robot_parameter_file_path", robot_driver_coppelia_configuration.robot_parameter_file_path);

    try
    {
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(),"::Instantiating Coppelia robot mirror.");
        qros::RobotDriverCoppelia robot_driver_coppelia(node, robot_driver_coppelia_configuration,
                                                       &kill_this_process);

        return robot_driver_coppelia.start_control_loop();
    }
    catch (const std::exception& e)
    {
        kill_this_process = true;
        RCLCPP_ERROR_STREAM_ONCE(node->get_logger(),"["+node_name+"]::Exception::" + e.what());
        shutdown(context, "Exception in main function: " + std::string(e.what()));
        return -1;
    }


    return 0;
}
