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
                                                     const std::vector<std::string>& jointnames,
                                                     const RobotCoppeliaROSConfiguration& configuration,
                                                     std::atomic_bool* kill_this_node)
    :nh_(nh),
    topic_prefix_(topic_prefix),
    jointnames_(jointnames),
    kill_this_node_(kill_this_node),
    configuration_(configuration),
    clock_(configuration.thread_sampling_time_nsec)
{

    //subscriber_joint_state_ = nh_.subscribe(topic_prefix_+ "/get/joint_states", 1,
    //                                        &RobotCoppeliaRosInterface::_joint_states_callback, this);

    //publisher_target_joint_positions_ = nh_.advertise<std_msgs::Float64MultiArray>
    //                                    (topic_prefix_ + "/set/target_joint_positions", 1);

    subscriber_target_joint_positions_ = nh_.subscribe(topic_prefix_ + "/set/target_joint_positions", 1, &RobotCoppeliaRosInterface::_callback_target_joint_positions, this);
    publisher_joint_states_ = nh_.advertise<sensor_msgs::JointState>(topic_prefix_+ "/get/joint_states", 1);

    vi_ = std::make_shared<DQ_VrepInterface>();
    vi_->connect(configuration_.ip, configuration_.port, 500, 10);
    vi_->start_simulation();
}

RobotCoppeliaRosInterface::~RobotCoppeliaRosInterface()
{
    vi_->disconnect();
}

/**
 * @brief get_joint_positions
 * @return
 */
VectorXd RobotCoppeliaRosInterface::get_joint_positions() const
{
    return joint_positions_;
}


/**
 * @brief set_joint_target_positions
 * @param target_joint_positions
 */
void RobotCoppeliaRosInterface::set_joint_target_positions(const VectorXd &target_joint_positions)
{

    std_msgs::Float64MultiArray ros_msg;
    ros_msg.data = sas::vectorxd_to_std_vector_double(target_joint_positions);
    publisher_target_joint_positions_.publish(ros_msg);

}

void RobotCoppeliaRosInterface::set_joint_target_velocities(const VectorXd &target_joint_velocities)
{

}


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
        ROS_INFO_STREAM(ros::this_node::getName() << "::Waiting to connect with CoppeliaSim...");


        while(not _should_shutdown())
        {
            clock_.update_and_sleep();
            ros::spinOnce();

            VectorXd q = vi_->get_joint_positions(jointnames_);
            joint_positions_ = q;
            send_joint_states(q, VectorXd(), VectorXd());

            if (target_joint_positions_.size()>0)
            {
                vi_->set_joint_target_positions(jointnames_, target_joint_positions_);
            }

            //auto q = get_joint_positions();
            std::cout<<"q: "<<q.transpose()<<std::endl;
            std::cout<<"target q: "<<target_joint_positions_.transpose()<<std::endl;



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
