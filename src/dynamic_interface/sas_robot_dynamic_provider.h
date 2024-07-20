#pragma once
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sas_common/sas_common.h>
#include <sas_conversions/sas_conversions.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <dqrobotics/DQ.h>

namespace sas {

using namespace DQ_robotics;

class RobotDynamicProvider {
private:
    std::string node_prefix_;

    ros::Publisher publisher_cartesian_contact_;
    ros::Publisher publisher_cartesian_pose_;

public:
    RobotDynamicProvider(ros::NodeHandle& nodehandle, const std::string& node_prefix=ros::this_node::getName());
    RobotDynamicProvider(ros::NodeHandle& publisher_nodehandle, ros::NodeHandle& subscriber_nodehandle, const std::string& node_prefix=ros::this_node::getName());

    void publish_cartesian_contact(const Vector3d& force, const Vector3d& torque);
    void publish_cartesian_pose(const DQ& pose);

};



} // namespace sas