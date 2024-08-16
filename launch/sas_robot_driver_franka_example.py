from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sas_robot_driver_franka',
            executable='sas_robot_driver_franka_node',
            name='arm3',
            parameters=[{
                "robot_ip_address": "172.16.0.4",
                "thread_sampling_time_sec": 0.004,
                # "thread_sampling_time_nsec": 4000000,
                "q_min": [-2.3093, -1.5133, -2.4937, -2.7478, -2.4800, 0.8521, -2.6895],
                "q_max": [2.3093, 1.5133, 2.4937, -0.4461, 2.4800, 4.2094, 2.6895],
                "force_upper_limits_scaling_factor": 4.0,
                "upper_torque_threshold": [40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0],
                "upper_force_threshold": [40.0, 40.0, 40.0, 40.0, 40.0, 40.0],
                "robot_mode": "VelocityControl",
                "robot_parameter_file_path": os.environ["ROBOT_3_JSON_PATH"]
            }],
            output="screen"
        ),
        Node(
            package='sas_robot_driver_franka',
            executable='sas_robot_driver_franka_coppelia_node',
            name='arm3_coppelia',
            parameters=[{
                "thread_sampling_time_sec": 0.008,
                "vrep_ip": os.environ["VREP_IP"],
                "vrep_port": 20012,
                "vrep_dynamically_enabled": True,
                "using_real_robot": True,
                "vrep_joint_names": ["Franka_joint1#1", "Franka_joint2#1", "Franka_joint3#1", "Franka_joint4#1",
                                     "Franka_joint5#1", "Franka_joint6#1", "Franka_joint7#1"],
                "robot_topic_prefix": "/arm3",
                "robot_mode": "VelocityControl",
                "robot_parameter_file_path": os.environ["ROBOT_3_JSON_PATH"]
            }],
            output="screen"
        ),

    ])
