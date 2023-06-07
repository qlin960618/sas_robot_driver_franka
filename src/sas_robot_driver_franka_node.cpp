//#include "ros/ros.h"
//#include "std_msgs/String.h"

#include <sstream>

#include <exception>
#include <dqrobotics/utils/DQ_Math.h>
//#include <dqrobotics/interfaces/json11/DQ_JsonReader.h>
#include <sas_common/sas_common.h>
#include <sas_conversions/eigen3_std_conversions.h>
#include <sas_robot_driver/sas_robot_driver_ros.h>
#include "sas_robot_driver_franka.h"


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

    ros::init(argc, argv, "sas_robot_driver_franka_node", ros::init_options::NoSigintHandler);
    ROS_WARN("=====================================================================");
    ROS_WARN("----------------------Juan Jose Quiroz Omana------------------------");
    ROS_WARN("=====================================================================");
    ROS_INFO_STREAM(ros::this_node::getName()+"::Loading parameters from parameter server.");


    ros::NodeHandle nh;
    sas::RobotDriverFrankaConfiguration robot_driver_franka_configuration;

    sas::get_ros_param(nh,"/robot_ip_address",robot_driver_franka_configuration.ip_address);
   // sas::get_ros_param(nh,"/robot_port",      robot_driver_franka_configuration.port);
    //sas::get_ros_param(nh,"/robot_speed",     robot_driver_franka_configuration.speed);

    //robot_driver_franka_configuration.ip_address = "172.16.0.2";
    //robot_driver_franka_configuration.port =  0;
    //robot_driver_franka_configuration.speed = 0.0;

    sas::RobotDriverROSConfiguration robot_driver_ros_configuration;

    sas::get_ros_param(nh,"/thread_sampling_time_nsec",robot_driver_ros_configuration.thread_sampling_time_nsec);
    sas::get_ros_param(nh,"/q_min",robot_driver_ros_configuration.q_min);
    sas::get_ros_param(nh,"/q_max",robot_driver_ros_configuration.q_max);

    //auto qmin = sas::std_vector_double_to_vectorxd(robot_driver_ros_configuration.q_min);
    //auto qmax = sas::std_vector_double_to_vectorxd(robot_driver_ros_configuration.q_max);


    //std::vector<double> q_min = {-2.3093,-1.5133,-2.4937, -2.7478,-2.4800, 0.8521, -2.6895};
    //std::vector<double> q_max = { 2.3093, 1.5133, 2.4937, -0.4461, 2.4800, 4.2094,  2.6895};

    //robot_driver_ros_configuration.thread_sampling_time_nsec = 4000000;
    //robot_driver_ros_configuration.q_min = q_min;
    //robot_driver_ros_configuration.q_max = q_max;

    robot_driver_ros_configuration.robot_driver_provider_prefix = ros::this_node::getName();
    
    try
        {
            ROS_INFO_STREAM(ros::this_node::getName()+"::Instantiating Franka robot.");
            sas::RobotDriverFranka robot_driver_franka(robot_driver_franka_configuration, 
                                                        &kill_this_process);
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
        ROS_ERROR_STREAM(ros::this_node::getName() + "::Exception::" + e.what());
    }     
 

  return 0;
}
