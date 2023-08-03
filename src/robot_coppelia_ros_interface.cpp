#include "robot_coppelia_ros_interface.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include <sas_common/sas_common.h>
#include <sas_conversions/sas_conversions.h>

/**
 * @brief robot_ros_interface
 * @param nh
 */
RobotCoppeliaRosInterface::RobotCoppeliaRosInterface(const ros::NodeHandle& nh,
                                                     const std::string& topic_prefix,
                                                     const RobotCoppeliaROSConfiguration& configuration,
                                                     std::atomic_bool* kill_this_node)
    :nh_(nh),
    topic_prefix_(topic_prefix),
    kill_this_node_(kill_this_node),
    configuration_(configuration),
    robot_mode_ (configuration.robot_mode),
    jointnames_(configuration.jointnames),
    mirror_mode_(configuration.mirror_mode),
    clock_(configuration.thread_sampling_time_nsec)
{
    subscriber_target_joint_positions_  = nh_.subscribe(topic_prefix_ + "/set/target_joint_positions", 1,  &RobotCoppeliaRosInterface::_callback_target_joint_positions, this);
    subscriber_target_joint_velocities_ = nh_.subscribe(topic_prefix_ + "/set/target_joint_velocities", 1, &RobotCoppeliaRosInterface::_callback_target_joint_velocities, this);
    publisher_joint_states_ = nh_.advertise<sensor_msgs::JointState>(topic_prefix_+ "/get/joint_states", 1);

    ROS_INFO_STREAM(ros::this_node::getName() << "::Connecting with CoppeliaSim...");
    vi_ = std::make_shared<DQ_VrepInterface>();
    vi_->connect(configuration_.ip, configuration_.port, 500, 10);
    vi_->start_simulation();
    ROS_INFO_STREAM(ros::this_node::getName() << "::Connection ok!");
}

RobotCoppeliaRosInterface::~RobotCoppeliaRosInterface()
{
    vi_->disconnect();
}

/*
VectorXd RobotCoppeliaRosInterface::get_joint_positions() const
{
    return joint_positions_;
}


/*
void RobotCoppeliaRosInterface::set_joint_target_positions(const VectorXd &target_joint_positions)
{

    std_msgs::Float64MultiArray ros_msg;
    ros_msg.data = sas::vectorxd_to_std_vector_double(target_joint_positions);
    publisher_target_joint_positions_.publish(ros_msg);

}
 */

void RobotCoppeliaRosInterface::send_joint_states(const VectorXd &joint_positions, const VectorXd &joint_velocities, const VectorXd &joint_forces)
{
    sensor_msgs::JointState ros_msg;
    if(joint_positions.size()>0)
        ros_msg.position = sas::vectorxd_to_std_vector_double(joint_positions);
    if(joint_velocities.size()>0)
        ros_msg.velocity = sas::vectorxd_to_std_vector_double(joint_velocities);
    if(joint_forces.size()>0)
        ros_msg.effort = sas::vectorxd_to_std_vector_double(joint_forces);
    publisher_joint_states_.publish(ros_msg);
}

int RobotCoppeliaRosInterface::control_loop()
{
    try{
        clock_.init();
        ROS_INFO_STREAM(ros::this_node::getName() << "::Starting control loop...");


        while(not _should_shutdown())
        {
            clock_.update_and_sleep();
            ros::spinOnce();

            VectorXd q = vi_->get_joint_positions(jointnames_);
            joint_positions_ = q;
            send_joint_states(q, VectorXd(), VectorXd());

            if (robot_mode_  == std::string("VelocityControl"))
            {
                //if (mirror_mode_ == true)
                //{
                    if (target_joint_positions_.size()>0)
                    {
                        vi_->set_joint_target_velocities(jointnames_, gain_*(target_joint_positions_-q));
                    }
                //}
                //else{
                    if (target_joint_velocities_.size()>0)
                    {
                        vi_->set_joint_target_velocities(jointnames_, target_joint_velocities_);
                    }
                //}
            }
            else if (robot_mode_  == std::string("PositionControl"))
                {
                    if (target_joint_positions_.size()>0)
                    {
                        vi_->set_joint_target_positions(jointnames_, target_joint_positions_);
                    }
                }

            ros::spinOnce();
        }
    }
    catch(const std::exception& e)
    {
        ROS_WARN_STREAM(ros::this_node::getName() + "::Error or exception caught::" << e.what());
    }
    catch(...)
    {
        ROS_WARN_STREAM(ros::this_node::getName() + "::Unexpected error or exception caught");
    }

    return 0;
}



/**
 * @brief _joint_states_callback
 * @param jointstate
 */
bool RobotCoppeliaRosInterface::_should_shutdown() const
{
    return (*kill_this_node_);
}

void RobotCoppeliaRosInterface::_joint_states_callback(const sensor_msgs::JointState::ConstPtr& jointstate)
{
    joint_positions_ = sas::std_vector_double_to_vectorxd(jointstate->position);
}

void RobotCoppeliaRosInterface::_callback_target_joint_positions(const std_msgs::Float64MultiArrayConstPtr &msg)
{
    target_joint_positions_ = sas::std_vector_double_to_vectorxd(msg->data);
}

void RobotCoppeliaRosInterface::_callback_target_joint_velocities(const std_msgs::Float64MultiArrayConstPtr &msg)
{
    target_joint_velocities_ = sas::std_vector_double_to_vectorxd(msg->data);
}
