#include <ros/ros.h>

#include <sas_common/sas_common.h>
#include <sas_robot_kinematics/sas_robot_kinematics_provider.h>
#include <sas_robot_driver/sas_robot_driver_interface.h>
#include <iostream>
#include <Eigen/Dense>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
//#include <dqrobotics/robots/FrankaEmikaPandaMDHRobot.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorMDH.h>
//#include <dqrobotics/solvers/DQ_SerialManipulatorDynamicsGaussPrincipleSolver.h>
#include <dqrobotics/solvers/DQ_QuadraticProgrammingSolver.h>
#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
#include <dqrobotics/interfaces/vrep/robots/FrankaEmikaPandaVrepRobot.h>
#include <thread>
#include <dqrobotics/DQ.h>
#include <dqrobotics/solvers/DQ_QPOASESSolver.h>
#include <dqrobotics/robot_control/DQ_ClassicQPController.h>
#include <chrono>
#include <yaml-cpp/yaml.h>
#include <RobotConstraintsManager.h>
#include "franka_ros_interface.h"
#include <QuadraticProgramMotionGenerator.h>


std::string config_vfi_path = "/home/moonshot/Documents/git/franka_emika/real_franka_developments/real_franka_developments/cfg/vfi_constraints.yaml";



int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_franka_node");
    auto nodehandle = ros::NodeHandle();
    sas::RobotKinematicsProvider pose1_provider(nodehandle, "/arm1_kinematics");

    //auto franka1_ros = FrankaRosInterface(nodehandle, 50, "/franka_1");
    sas::RobotDriverInterface franka1_ros(nodehandle, "/franka_1");
    //##########################################################################################

    DQ_VrepInterface vi;
    vi.connect("10.198.113.159", 19997,100,10);
    std::cout << "Starting V-REP simulation..." << std::endl;
    vi.start_simulation();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    auto franka_sim = FrankaEmikaPandaVrepRobot("Franka", std::make_shared<DQ_VrepInterface>(vi));
    auto robot = franka_sim.kinematics();
    DQ xoffset = (1 + E_*0.5*0.21*k_)*(cos((M_PI/4)/2)-k_*sin((M_PI/4)/2));
    robot.set_effector(xoffset);



    //###########################################################################################
    auto manager = RobotConstraintsManager(std::make_shared<DQ_VrepInterface>(vi),
                                           (std::static_pointer_cast<DQ_Kinematics>(std::make_shared<DQ_SerialManipulatorMDH>(robot))),
                                           (std::static_pointer_cast<DQ_VrepRobot>(std::make_shared<FrankaEmikaPandaVrepRobot>(franka_sim))),
                                           config_vfi_path,
                                           RobotConstraintsManager::ORDER::FIRST_ORDER);
    //##########################################################################################
    //#######------------------------Controller------------------------------------------#######
    DQ_QPOASESSolver solver;
    DQ_ClassicQPController controller
        (std::static_pointer_cast<DQ_Kinematics>(std::make_shared<DQ_SerialManipulatorMDH>(robot)),
         std::static_pointer_cast<DQ_QuadraticProgrammingSolver>(std::make_shared<DQ_QPOASESSolver>(solver)));
    controller.set_gain(20.0);
    controller.set_damping(0.01);
    controller.set_control_objective(DQ_robotics::Translation); //DistanceToPlane Translation
    controller.set_stability_threshold(0.0001);
    //##########################################################################################

    //#########################################################################################
    DQ xd;
    DQ x;
    MatrixXd A;
    VectorXd b;
    VectorXd u;
    VectorXd q_u = VectorXd::Zero(7);
    VectorXd q;

    VectorXd q_dot_initial = VectorXd::Zero(7);
    VectorXd q_dot = VectorXd::Zero(7);



    while (ros::ok()) {
        if (franka1_ros.is_enabled())
        {
            q = franka1_ros.get_joint_positions();
            break;

        }
        ros::spinOnce();
    }


    q_u = q;

    franka_sim.set_target_configuration_space_positions(q);
    vi.set_object_pose("xd", robot.fkm(q));

    /*
    DQ_QPOASESSolver solver2;
    MatrixXd H = MatrixXd::Identity(7, 7);
    VectorXd f ;
    VectorXd qvrep;
    MatrixXd A_;
    VectorXd b_;

    for (int i=0;i<3000;i++)
    {
        qvrep = franka_sim.get_configuration_space_positions();
        f = 2*0.1*(qvrep-q);
         auto u_ = solver2.solve_quadratic_program(H, f, A_, b_, A_, b_);
        franka_sim.set_target_configuration_space_velocities(u_);
         std::cout<<"initialiazing..."<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    */


    std::cout<<"-------------------------"<<std::endl;
    std::cout<<"Initial q: "<<std::endl;
    std::cout<<q.transpose()<<std::endl;
    std::cout<<"-------------------------"<<std::endl;
    double T = 0.001;
    try {

        while(ros::ok())
        {

            if (q.size() == 7)
            {
                q = franka1_ros.get_joint_positions();
                franka_sim.set_target_configuration_space_positions(q);
                //q = franka_sim.get_configuration_space_positions();
                x = robot.fkm(q);
                vi.set_object_pose("effector", x);

                if(not pose1_provider.is_enabled())
                {
                    pose1_provider.send_pose(x);

                }
                pose1_provider.send_reference_frame(robot.get_reference_frame());


                try{

                    xd =  pose1_provider.get_desired_pose();
                    pose1_provider.send_pose(xd);

                }
                catch(const std::exception& e){
                    xd = vi.get_object_pose("xd");
                    std::cout<<"Nope"<<std::endl;
                    std::cout << e.what() << std::endl;
                }

                xd = vi.get_object_pose("xd");

                std::tie(A, b) = manager.get_inequality_constraints(q);
                controller.set_inequality_constraint(A,b);



                //-----------------------------------------------------------
                switch(controller.get_control_objective())
                {
                case ControlObjective::Distance:
                {
                    VectorXd pdesired = vec4(translation(xd));
                    VectorXd distance(1);
                    distance(0) = pdesired.transpose()*pdesired;
                    u = controller.compute_setpoint_control_signal(q, pdesired.transpose()*pdesired);
                    break;
                }
                case ControlObjective::Translation:
                {
                    u = controller.compute_setpoint_control_signal(q, vec4(xd.translation()));
                    break;
                }
                case ControlObjective::Pose:
                {
                    u = controller.compute_setpoint_control_signal(q, vec8(xd));
                    break;
                }
                case ControlObjective::DistanceToPlane:
                {
                    DQ plane_normal = xd.P()*k_*xd.P().conj();
                    DQ plane_point =  xd.translation();
                    DQ xdesired_plane = (plane_normal + E_ * dot(plane_point, plane_normal));
                    controller.set_target_primitive(xdesired_plane);
                    u = controller.compute_setpoint_control_signal(q, VectorXd::Zero(1)); //DistanceToPlane
                    break;
                }
                default:
                    throw std::runtime_error("ControlObjective not suppported.");
                    break;
                }
                //q = q + T*u;
                //std::cout<<"error: "<<std::endl;
                //std::cout<<controller.get_last_error_signal().norm()<<std::endl;

                //franka_sim.set_target_configuration_space_positions(franka1_ros.get_joint_positions());
                //auto new_u = trajectory_generator_sptr_->compute_new_configuration_velocities(u, T);
                //franka_sim.set_target_configuration_space_velocities(new_u);
                //franka1_ros.send_target_joint_positions(new_u);

                //auto new_u = trajectory_generator_sptr_->compute_new_configuration_velocities(u, T);
                //franka1_ros.send_target_joint_positions(q);
                franka1_ros.send_target_joint_velocities(u);





            }

            ros::spinOnce();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::cout << "Stopping V-REP simulation..." << std::endl;
        vi.stop_simulation();
        vi.disconnect();

        return 0;
    }
    catch (const std::exception& e) {
        vi.stop_simulation();
        vi.disconnect();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::cout << e.what() << std::endl;
        return -1;
    }
}
