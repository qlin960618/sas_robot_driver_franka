#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <sas_common/sas_common.h>
#include <sas_conversions/sas_conversions.h>
#include <dqrobotics/interfaces/vrep/DQ_VrepRobot.h>
#include <atomic>
#include <sas_clock/sas_clock.h>
#include <sas_robot_driver/sas_robot_driver_provider.h>
#include <sas_conversions/sas_conversions.h>


using namespace Eigen;


struct RobotCoppeliaROSConfiguration
{

    int thread_sampling_time_nsec;
    int port;
    std::string ip;
    std::vector<std::string> jointnames;
    std::string robot_mode;
    bool mirror_mode;
};

class RobotCoppeliaRosInterface
{
private:
    std::atomic_bool* kill_this_node_;
    bool _should_shutdown() const;
    sas::Clock clock_;
    RobotCoppeliaROSConfiguration configuration_;
    std::string robot_mode_ = std::string("VelocityControl");   // PositionControl
    bool mirror_mode_ = false;
    double gain_ = 0.5;


protected:
    ros::NodeHandle nh_;
    VectorXd joint_positions_;
    VectorXd target_joint_positions_;
    VectorXd target_joint_velocities_;
    std::string topic_prefix_;
    //ros::Publisher   publisher_target_joint_positions_;
    ros::Publisher publisher_joint_states_;
    //ros::Subscriber subscriber_joint_state_;
    ros::Subscriber subscriber_target_joint_positions_;
    ros::Subscriber subscriber_target_joint_velocities_;
    std::shared_ptr<DQ_VrepRobot> coppelia_robot_;
    std::vector<std::string> jointnames_;
    std::shared_ptr<DQ_VrepInterface> vi_;
    void _joint_states_callback(const sensor_msgs::JointState::ConstPtr& jointstate);
    void _callback_target_joint_positions(const std_msgs::Float64MultiArrayConstPtr &msg);
    void _callback_target_joint_velocities(const std_msgs::Float64MultiArrayConstPtr& msg);

    void send_joint_states(const VectorXd &joint_positions, const VectorXd &joint_velocities, const VectorXd &joint_forces);


public:


    RobotCoppeliaRosInterface()=delete;

    ~RobotCoppeliaRosInterface();


    RobotCoppeliaRosInterface(const RobotCoppeliaRosInterface&) = delete;
    RobotCoppeliaRosInterface& operator= (const RobotCoppeliaRosInterface&) = delete;

    RobotCoppeliaRosInterface(const ros::NodeHandle& nh,
                              const std::string& topic_prefix,
                              const RobotCoppeliaROSConfiguration& configuration,
                              std::atomic_bool* kill_this_node);

    //VectorXd get_joint_positions() const;
    //VectorXd get_joint_velocities() const;
    //void set_joint_target_positions(const VectorXd& target_joint_positions);

    //void set_joint_target_positions_(const VectorXd& target_joint_positions);

    void connect();
    void disconnect();

    void initialize();
    void deinitialize();
    int control_loop();




};
