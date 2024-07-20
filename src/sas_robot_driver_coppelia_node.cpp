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
//#include <dqrobotics/interfaces/json11/DQ_JsonReader.h>
#include <sas_common/sas_common.h>
#include <sas_conversions/eigen3_std_conversions.h>
#include <sas_robot_driver/sas_robot_driver_ros.h>
#include "coppelia/sas_robot_driver_coppelia.h"

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
        throw std::runtime_error(ros::this_node::getName() + "::Error setting the signal int handler.");
    }

    ros::init(argc, argv, "sas_robot_driver_coppelia_node", ros::init_options::NoSigintHandler);
    ROS_WARN("=====================================================================");
    ROS_WARN("----------------------Juan Jose Quiroz Omana------------------------");
    ROS_WARN("=====================================================================");
    ROS_INFO_STREAM(ros::this_node::getName()+"::Loading parameters from parameter server.");


    ros::NodeHandle nh;
    sas::RobotDriverCoppeliaConfiguration robot_driver_coppelia_configuration;

    sas::get_ros_param(nh,"/robot_ip_address",robot_driver_coppelia_configuration.ip);
    sas::get_ros_param(nh,"/robot_mode",      robot_driver_coppelia_configuration.robot_mode);
    sas::get_ros_param(nh,"/vrep_port", robot_driver_coppelia_configuration.port);
    sas::get_ros_param(nh,"/vrep_robot_joint_names", robot_driver_coppelia_configuration.jointnames);
    sas::get_ros_param(nh,"/mirror_mode", robot_driver_coppelia_configuration.mirror_mode);
    sas::get_ros_param(nh, "/real_robot_topic_prefix", robot_driver_coppelia_configuration.real_robot_topic_prefix);
    sas::RobotDriverROSConfiguration robot_driver_ros_configuration;

    sas::get_ros_param(nh,"/thread_sampling_time_nsec",robot_driver_ros_configuration.thread_sampling_time_nsec);
    sas::get_ros_param(nh,"/q_min",robot_driver_ros_configuration.q_min);
    sas::get_ros_param(nh,"/q_max",robot_driver_ros_configuration.q_max);

    robot_driver_ros_configuration.robot_driver_provider_prefix = ros::this_node::getName();

    try
    {
        ROS_INFO_STREAM(ros::this_node::getName()+"::Instantiating Coppelia robot.");
        sas::RobotDriverCoppelia robot_driver_coppelia(robot_driver_coppelia_configuration,
                                                       &kill_this_process);
        ROS_INFO_STREAM(ros::this_node::getName()+"::Instantiating RobotDriverROS.");
        sas::RobotDriverROS robot_driver_ros(nh,
                                             &robot_driver_coppelia,
                                             robot_driver_ros_configuration,
                                             &kill_this_process);
        robot_driver_ros.control_loop();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM(ros::this_node::getName() + "::Exception::" + e.what());
    }


    return 0;
}
