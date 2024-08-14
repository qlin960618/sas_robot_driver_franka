#include "sas_robot_driver_coppelia.h"


namespace sas
{

RobotDriverCoppelia::~RobotDriverCoppelia() {
    deinitialize();
    disconnect();
}

RobotDriverCoppelia::RobotDriverCoppelia(ros::NodeHandle nh, const RobotDriverCoppeliaConfiguration &configuration, std::atomic_bool *break_loops):
        configuration_(configuration),
        clock_(configuration.thread_sampling_time_nsec),
        break_loops_(break_loops),
        robot_mode_(ControlMode::Position),
        vi_(std::make_shared<DQ_VrepInterface>())
{
    // should initialize robot driver interface to real robot
    DQ_SerialManipulatorDH smdh = DQ_JsonReader::get_from_json<DQ_SerialManipulatorDH>(configuration_.robot_parameter_file_path);
    joint_limits_ = {smdh.get_lower_q_limit(),smdh.get_upper_q_limit()};
    if(configuration_.using_real_robot)
    {
        ROS_INFO_STREAM("[" + ros::this_node::getName() + "]::Using real robot, Instantiating robot interface to real driver.");
        real_robot_interface_ = std::make_shared<RobotDriverInterface>(nh, configuration_.robot_topic_prefix);
    }else{
        ROS_INFO_STREAM("[" + ros::this_node::getName() + "]::Using simulation robot, simulation mode is set to "+ configuration_.robot_mode);
        robot_provider_ = std::make_shared<RobotDriverProvider>(nh, configuration_.robot_topic_prefix);
        std::string _mode_upper = configuration_.robot_mode;
        std::transform(_mode_upper.begin(), _mode_upper.end(), _mode_upper.begin(), ::toupper);
        if(_mode_upper == "POSITIONCONTROL"){
            robot_mode_ = ControlMode::Position;
        }else if(_mode_upper == "VELOCITYCONTROL"){
            robot_mode_ = ControlMode::Velocity;
        }else{
            throw std::invalid_argument("[" + ros::this_node::getName() + "]::Robot mode must be either 'position' or 'velocity'");
        }
    }

}

void RobotDriverCoppelia::_update_vrep_position(const VectorXd &joint_positions, const bool& force_update) const{
    if(configuration_.vrep_dynamically_enabled){
        if(force_update){
            vi_->set_joint_positions(configuration_.vrep_joint_names, joint_positions);
        }
        vi_->set_joint_target_positions(configuration_.vrep_joint_names, joint_positions);
    }else{
        vi_->set_joint_positions(configuration_.vrep_joint_names, joint_positions);
    }
}

void RobotDriverCoppelia::_update_vrep_velocity(const VectorXd & joint_velocity) const{
    if(!configuration_.vrep_dynamically_enabled){
        throw std::runtime_error("[RobotDriverCoppelia]::[_update_vrep_velocity]::Vrep is not dynamically enabled");
    }
    vi_->set_joint_target_velocities(configuration_.vrep_joint_names, joint_velocity);
}

void RobotDriverCoppelia::_start_control_loop(){
    clock_.init();
    ROS_INFO_STREAM("[" + ros::this_node::getName() + "]::Starting control loop...");
    while(!_should_shutdown()){
        clock_.update_and_sleep();
        ros::spinOnce();
        if(!ros::ok()){break_loops_->store(true);}
        if(configuration_.using_real_robot){
            // real_robot_interface_
            auto joint_position = real_robot_interface_->get_joint_positions();
            _update_vrep_position(joint_position);
        }else{
            // robot_provider_
            VectorXd target_joint_positions;
            auto current_joint_positions = vi_->get_joint_positions(configuration_.vrep_joint_names);
            VectorXd current_joint_velocity;
            if(robot_provider_->is_enabled()) {
                if(robot_mode_ == ControlMode::Velocity)
                {
                    if(robot_provider_->is_enabled(sas::RobotDriver::Functionality::VelocityControl)) {
                        simulated_joint_velocities_ = robot_provider_->get_target_joint_velocities();
                        current_joint_velocity = simulated_joint_velocities_;
                        //                    try{_update_vrep_velocity(simulated_joint_velocities_);}catch (...){}
                    }
                    else{
                        ROS_DEBUG_STREAM("[" + ros::this_node::getName() + "]::Velocity control is not enabled.");
                    }
                    target_joint_positions = simulated_joint_positions_;
                }else{
                    target_joint_positions = robot_provider_->get_target_joint_positions();
                }
            }else {
                target_joint_positions = current_joint_positions;
            }
            _update_vrep_position(target_joint_positions);
            robot_provider_->send_joint_states(current_joint_positions, current_joint_velocity, VectorXd());
            robot_provider_->send_joint_limits(joint_limits_);

        }
    }


}

int RobotDriverCoppelia::start_control_loop(){
    try{
        ROS_INFO_STREAM(ros::this_node::getName() << "::Waiting to connect with coppelia...");
        connect();
        ROS_INFO_STREAM(ros::this_node::getName() << "::Connected to coppelia.");
        ROS_INFO_STREAM(ros::this_node::getName() << "::Initializing...");
        initialize();
        ROS_INFO_STREAM(ros::this_node::getName() << "::initialized.");

        _start_control_loop();
    }
    catch(const std::exception& e)
    {
        ROS_WARN_STREAM(ros::this_node::getName() + "::Error or exception caught::" << e.what());
    }
    catch(...)
    {
        ROS_WARN_STREAM(ros::this_node::getName() + "::Unexpected error or exception caught");
    }
    ROS_INFO_STREAM(ros::this_node::getName() << "::Deinitializing...");
    deinitialize();
    ROS_INFO_STREAM(ros::this_node::getName() << "::deinitialized.");
    ROS_INFO_STREAM(ros::this_node::getName() << "::Disconnecting from coppelia...");
    disconnect();
    ROS_INFO_STREAM(ros::this_node::getName() << "::Disconnected from coppelia.");

    return 0;
}


void RobotDriverCoppelia::connect(){
    auto ret = vi_->connect(configuration_.vrep_ip, configuration_.vrep_port, 100, 10);
    if(!ret){
        throw std::runtime_error("[RobotDriverCoppelia]::connect::Could not connect to Vrep");
    }
    ROS_INFO_STREAM("[" + ros::this_node::getName() + "]::[RobotDriverCoppelia]::connect::Connected to Vrep");
}
void RobotDriverCoppelia::disconnect(){
    vi_->disconnect_all();
}

void RobotDriverCoppelia::initialize(){
    if(configuration_.using_real_robot)
    {
        ROS_INFO_STREAM("[" + ros::this_node::getName() +"]::[RobotDriverCoppelia]::initialize::Waiting for real robot interface to initialize...");
        ros::spinOnce();
        int count = 0;
        while (!real_robot_interface_->is_enabled()) {
            ros::spinOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            count++;
            if(count > REAL_ROBOT_INTERFACE_INIT_TIMEOUT_COUNT){
                ROS_ERROR_STREAM("[" + ros::this_node::getName() +"]::[RobotDriverCoppelia]::initialize::Real robot interface not initialized. Exiting on TIMEOUT...");
                throw std::runtime_error("[" + ros::this_node::getName() +"]::[RobotDriverCoppelia]::initialize::Real robot interface not initialized.");
            }
            if(!ros::ok()) {
                ROS_WARN_STREAM("[" + ros::this_node::getName() +"]::[RobotDriverCoppelia]::initialize::ROS shutdown recieved. Exiting...");
                throw std::runtime_error("[" + ros::this_node::getName() +"]::[RobotDriverCoppelia]::initialize::ROS shutdown recieved, not OK.");
            }
        }
        ROS_INFO_STREAM("[" + ros::this_node::getName() +"]::[RobotDriverCoppelia]::initialize::Real robot interface initialized.");
        joint_limits_ = real_robot_interface_->get_joint_limits();
        _update_vrep_position(real_robot_interface_->get_joint_positions(), true);
    }else{
        ROS_INFO_STREAM("[" + ros::this_node::getName() +"]::[RobotDriverCoppelia]::initialize::Simulation mode.");
        // initialization information for robot driver provider
        /**
        * TODO: check for making sure real robot is not actually connected
        */
        auto current_joint_positions = vi_->get_joint_positions(configuration_.vrep_joint_names);
        VectorXd current_joint_velocity;
        if(robot_mode_ == ControlMode::Velocity)
        {current_joint_velocity = VectorXd::Zero(current_joint_positions.size());}
        robot_provider_->send_joint_states(current_joint_positions, current_joint_velocity, VectorXd());
        robot_provider_->send_joint_limits(joint_limits_);
        // start velocity control simulation thread if needed
        if(robot_mode_ == ControlMode::Velocity)
        {
            simulated_joint_positions_ = current_joint_positions;
            simulated_joint_velocities_ = current_joint_velocity;
            start_simulation_thread();
        }
    }
}
void RobotDriverCoppelia::deinitialize(){
    // nothing to do
}

void RobotDriverCoppelia::start_simulation_thread(){ // thread entry point
    if(simulation_thread_started_){
        throw std::runtime_error("[RobotDriverCoppelia]::start_simulation_thread::Simulation thread already started");
    }
    if(velocity_control_simulation_thread_.joinable()){
        velocity_control_simulation_thread_.join();
    }

    velocity_control_simulation_thread_ = std::thread(&RobotDriverCoppelia::_velocity_control_simulation_thread_main, this);
}

void RobotDriverCoppelia::_velocity_control_simulation_thread_main(){
    /**
     * This thread should not access vrep
     */
    simulation_thread_started_ = true;
    try{
        ROS_INFO_STREAM("[" + ros::this_node::getName() +
                        "]::[RobotDriverCoppelia]::[_velocity_control_simulation_thread_main]::Simulation thread started.");
        sas::Clock clock = sas::Clock(VIRTUAL_ROBOT_SIMULATION_SAMPLING_TIME_NSEC);
        double tau = static_cast<double>(VIRTUAL_ROBOT_SIMULATION_SAMPLING_TIME_NSEC) * 1e-9;
        auto current_joint_positions = simulated_joint_positions_;
        clock.init();
        while (!(*break_loops_) && ros::ok()) {

            current_joint_positions += tau * simulated_joint_velocities_; // no dynamic model
            // cap joint limit
            auto q_min = std::get<0>(joint_limits_);
            auto q_max = std::get<1>(joint_limits_);
            for (int i = 0; i < current_joint_positions.size(); i++) {
                if (current_joint_positions(i) < q_min(i)) {
                    current_joint_positions(i) = q_min(i);
                }
                if (current_joint_positions(i) > q_max(i)) {
                    current_joint_positions(i) = q_max(i);
                }
            }
            simulated_joint_positions_ = current_joint_positions;
            clock.update_and_sleep();
        }
    }catch(std::exception &e){
        ROS_ERROR_STREAM("[" + ros::this_node::getName() + "]::[RobotDriverCoppelia]::[_velocity_control_simulation_thread_main]::Exception::" + e.what());
        simulation_thread_started_ = false;
    }catch(...){
        ROS_ERROR_STREAM("[" + ros::this_node::getName() + "]::[RobotDriverCoppelia]::[_velocity_control_simulation_thread_main]::Exception::Unknown");
        simulation_thread_started_ = false;
    }

}



} // sas namespace
