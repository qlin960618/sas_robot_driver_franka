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
#include "../../include/sas_robot_driver_franka/sas_robot_driver_franka.h"
#include "robot_coppelia_ros_interface.h"

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

    ros::init(argc, argv, "robot_coppelia_ros_interface", ros::init_options::NoSigintHandler);
    ROS_WARN("=====================================================================");
    ROS_WARN("----------------------Juan Jose Quiroz Omana------------------------");
    ROS_WARN("=====================================================================");
    ROS_INFO_STREAM(ros::this_node::getName()+"::Loading parameters from parameter server.");


    ros::NodeHandle nh;
    //sas::RobotDriverFrankaConfiguration robot_driver_franka_configuration;

    //sas::get_ros_param(nh,"/robot_ip_address",robot_driver_franka_configuration.ip_address);
    //sas::get_ros_param(nh,"/robot_mode",      robot_driver_franka_configuration.mode);

    //sas::RobotDriverROSConfiguration robot_driver_ros_configuration;

    //sas::get_ros_param(nh,"/thread_sampling_time_nsec",robot_driver_ros_configuration.thread_sampling_time_nsec);
    //sas::get_ros_param(nh,"/q_min",robot_driver_ros_configuration.q_min);
    //sas::get_ros_param(nh,"/q_max",robot_driver_ros_configuration.q_max);

    //robot_driver_ros_configuration.robot_driver_provider_prefix = ros::this_node::getName();

    //4000000

    RobotCoppeliaROSConfiguration configuration;

    sas::get_ros_param(nh,"/vrep_ip", configuration.ip);
    sas::get_ros_param(nh,"/robot_mode",      configuration.robot_mode);
    sas::get_ros_param(nh,"/thread_sampling_time_nsec",configuration.thread_sampling_time_nsec);
    sas::get_ros_param(nh,"/vrep_port", configuration.port);
    sas::get_ros_param(nh,"/vrep_robot_joint_names", configuration.jointnames);
    sas::get_ros_param(nh,"/mirror_mode", configuration.mirror_mode);

  /*
    configuration.thread_sampling_time_nsec = 4000000;
    configuration.ip = "10.198.113.159";
    configuration.port = 20021;
    std::vector<std::string> jointnames = {"Franka_joint1","Franka_joint2","Franka_joint3",
                                           "Franka_joint4","Franka_joint5","Franka_joint6","Franka_joint7"};
    configuration.robot_mode = std::string("VelocityControl");
    configuration.mirror_mode = true;

    configuration.jointnames = jointnames;
 */
    RobotCoppeliaRosInterface robot_coppelia_ros_interface(nh, "/franka_1_sim", configuration, &kill_this_process);

    try
    {
        ROS_INFO_STREAM(ros::this_node::getName()+"::Instantiating Franka robot.");
        //sas::RobotDriverFranka robot_driver_franka(robot_driver_franka_configuration,
        //                                           &kill_this_process);
        //robot_driver_franka.set_joint_limits({qmin, qmax});
        ROS_INFO_STREAM(ros::this_node::getName()+"::Instantiating RobotDriverROS.");
        //sas::RobotDriverROS robot_driver_ros(nh,
        //                                     &robot_driver_franka,
        //                                     robot_driver_ros_configuration,
        //                                     &kill_this_process);
        //robot_driver_ros.control_loop();
        robot_coppelia_ros_interface.control_loop();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM(ros::this_node::getName() + "::Exception::" + e.what());
    }


    return 0;
}
