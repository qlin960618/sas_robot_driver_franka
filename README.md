![](https://img.shields.io/github/license/qlin960618/sas_robot_driver_franka)![](https://img.shields.io/github/contributors/qlin960618/sas_robot_driver_franka)![](https://img.shields.io/github/last-commit/qlin960618/sas_robot_driver_franka)![](https://img.shields.io/github/last-commit/qlin960618/sas_robot_driver_franka/ros2_jazzy)![](https://img.shields.io/badge/status-experimental-red)
# sas_robot_driver_franka
![ezgif com-video-to-gif (1)](https://github.com/SmartArmStack/sas_robot_driver/assets/23158313/b4e1efa7-8d93-4a67-ab87-a74c41d8f4bc)
Robot driver for the Franka Research 3 Robot. 

## Original Contributors
- [Juan José Quiroz Omaña](https://github.com/juanjqo/sas_robot_driver_franka)

## dependencies
SAS: 
- sas_common
- sas_core
- sas_msgs
- sas_conversions
- sas_robot_driver
- [sas_robot_driver_franka_interfaces](https://www.github.com/qlin960618/sas_robot_driver_franka_interfaces)

Misc: 
- geometry_msgs
- std_msgs
- std_srvs
- tf2_ros
- tf2

Franka Related

- Franka (libfranka)
- pinocchio

## Components

### sas_robot_driver_franka
Kernel-space real-time robot driver for the SmartArm compatible control

#### node parameters:
```yaml
"robot_ip_address": "172.16.0.x"  # robot ip address
"thread_sampling_time_sec": 0.004  # control sampling time (1.0/loop_rate)
"q_min": [-2.3093, -1.5133, -2.4937, -2.7478, -2.4800, 0.8521, -2.6895]  # joint limit minimum, if not define will obtain from [ROBOT_JSON]
"q_max": [2.3093, 1.5133, 2.4937, -0.4461, 2.4800, 4.2094, 2.6895]  # joint limit maximum, if not define will obtain from [ROBOT_JSON]
"force_upper_limits_scaling_factor": 4.0  # force and torque error limit scaling factor from default, (if any of the below is not defined)
"upper_torque_threshold": [40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0]  # joint torque safety threshold, [j1, j2, ..., j7]
"upper_force_threshold": [40.0, 40.0, 40.0, 40.0, 40.0, 40.0]  # end-effector frame safety force threshold [x, y, z, rx, ry, rz] 
"robot_mode": "VelocityControl"  # control mode [VelocityControl/PositionControl]: currently PositionControl can be unstable 
"robot_parameter_file_path": os.environ["ROBOT_3_JSON_PATH"]  # Robot definition JSON path
```

### sas_robot_driver_franka_hand
Franka Hand driver for basic control of gripping functionality.

#### special dependencies
- sas_robot_driver_franka_interfaces
  - <sas_robot_driver_franka_interfaces/srv/grasp.hpp>
  - <sas_robot_driver_franka_interfaces/srv/move.hpp>
  - <sas_robot_driver_franka_interfaces/msg/gripper_state.hpp>

#### node parameters:
```yaml
"robot_ip_address": "172.16.0.x"  # robot ip address
"thread_sampling_time_sec": 0.02  # control sampling time, (1.0/status update rate)
"default_force": 2.0  # default gripping force (overridable from service call)
"default_speed": 0.07  # default gripping speed (overridable from service call)
"default_epsilon_inner": 0.007  # default grip call position epsilon (overridable from service call)
"default_epsilon_outer": 0.007  # default grip call position epsilon (overridable from service call)
```


### sas_robot_driver_coppelia_node
Digital twin mirroring node for driver node to CoppeliaSim


#### node parameters:
```yaml
"thread_sampling_time_sec": 0.008  # control sampling time (1.0/loop_rate)
"vrep_ip": os.environ["VREP_IP"]  # ip of the vrep computer
"vrep_port": 20012  # vrep port
"vrep_dynamically_enabled": True  # if vrep scene is dynamically enabled
"using_real_robot": True  # if node should read-only from real robot driver, False for simulation mode
"vrep_joint_names": ["Franka_joint1#1", "Franka_joint2#1", "Franka_joint3#1", "Franka_joint4#1",
                     "Franka_joint5#1", "Franka_joint6#1", "Franka_joint7#1"]  # coppelia scene joint names
"robot_topic_prefix": "/arm3"  # robot driver namespace
"robot_mode": "PositionControl"  # only use when in simulation node, aka using_real_robot==False, for simulating velocity control mode. (TODO: currently VelocityControl is not stable)
"robot_parameter_file_path": os.environ["ROBOT_3_JSON_PATH"]  # Robot definition JSON path
```
