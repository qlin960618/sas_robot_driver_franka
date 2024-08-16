from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sas_robot_driver_franka',
            executable='sas_robot_driver_franka_coppelia_node',
            name='arm3_coppelia',
            parameters=[{
                "thread_sampling_time_sec": 0.008,
                "vrep_ip": os.environ["VREP_IP"],
                "vrep_port": 20012,
                "vrep_dynamically_enabled": True,
                "using_real_robot": False,
                "vrep_joint_names": ["Franka_joint1#1", "Franka_joint2#1", "Franka_joint3#1", "Franka_joint4#1",
                                     "Franka_joint5#1", "Franka_joint6#1", "Franka_joint7#1"],
                "robot_topic_prefix": "/arm3",
                "robot_mode": "VelocityControl",
                "robot_parameter_file_path": os.environ["ROBOT_3_JSON_PATH"]
            }],
            output="screen"
        ),

    ])
