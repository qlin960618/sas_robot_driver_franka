from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sas_robot_driver_franka',
            executable='sas_robot_driver_franka_hand_node',
            name='arm3_hand',
            parameters=[{
                "robot_ip_address": "172.16.0.2",
                # "thread_sampling_time_nsec": 20000000,  # 20ms , 50Hz
                "thread_sampling_time_sec": 0.02,
                "default_force": 2.0,
                "default_speed": 0.07,
                "default_epsilon_inner": 0.007,
                "default_epsilon_outer": 0.007,
            }],
            output="screen"
        ),

    ])
