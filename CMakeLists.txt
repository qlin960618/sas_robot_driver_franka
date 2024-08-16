cmake_minimum_required(VERSION 3.8)
project(sas_robot_driver_franka)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
#add_compile_options(-std=c++11)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(pybind11 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sas_common REQUIRED)
find_package(sas_core REQUIRED)
find_package(sas_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(sas_robot_driver REQUIRED)
find_package(Franka REQUIRED)
find_package(sas_robot_driver_franka_interfaces REQUIRED)  # Add this line

## To correctly find and link with QT
set(CMAKE_PREFIX_PATH $ENV{QT_PATH})
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
if (CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif ()
find_package(Qt5 COMPONENTS Widgets REQUIRED)

##### CPP LIBRARY initial #####
include_directories(
        include
        constraints_manager/include
)
install(
        DIRECTORY include/
        DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 rclcpp geometry_msgs std_msgs
        sas_common sas_core sas_conversions sas_robot_driver_franka_interfaces
        tf2_ros tf2 sas_robot_driver)


##### CPP LIBRARY initial end  #####

##### CPP LIBRARY ROBOT_DYNAMICS #####

add_library(${PROJECT_NAME}_robot_dynamics SHARED
        src/robot_dynamics/qros_robot_dynamics_client.cpp
        src/robot_dynamics/qros_robot_dynamics_server.cpp
)

ament_target_dependencies(${PROJECT_NAME}_robot_dynamics
        rclcpp geometry_msgs std_msgs
        sas_common sas_core sas_conversions
        tf2_ros tf2 sas_robot_driver)


target_include_directories(${PROJECT_NAME}_robot_dynamics
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)


target_link_libraries(${PROJECT_NAME}_robot_dynamics
        -ldqrobotics
        Eigen3::Eigen
)

install(
        TARGETS ${PROJECT_NAME}_robot_dynamics # ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME} #export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        #ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

##END## CPP LIBRARY ROBOT_DYNAMICS #####

##### CPP LIBRARY Motion Generation  #####

add_library(MotionGenerator src/generator/motion_generator.cpp)
target_link_libraries(MotionGenerator Franka::Franka Eigen3::Eigen)

add_library(ConstraintsManager constraints_manager/src/constraints_manager.cpp)
target_link_libraries(ConstraintsManager Eigen3::Eigen)
install(
        DIRECTORY constraints_manager/include/
        DESTINATION include/constraints_manager
)
target_include_directories(ConstraintsManager
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/constraints_manager/include>
        $<INSTALL_INTERFACE:include/constraints_manager>)



add_library(QuadraticProgramMotionGenerator src/generator/quadratic_program_motion_generator.cpp)
target_link_libraries(QuadraticProgramMotionGenerator
        qpOASES
        dqrobotics
        ConstraintsManager)


add_library(CustomMotionGeneration src/generator/custom_motion_generation.cpp)
target_link_libraries(CustomMotionGeneration
        qpOASES
        dqrobotics
        ConstraintsManager)

##### CPP LIBRARY Motion Generation end  #####


##### CPP LIBRARY franka low level interface  #####

add_library(robot_interface_franka src/joint/robot_interface_franka.cpp)
target_link_libraries(robot_interface_franka Franka::Franka
        dqrobotics
        MotionGenerator
        ConstraintsManager
        QuadraticProgramMotionGenerator
        CustomMotionGeneration)

add_library(robot_interface_hand src/hand/robot_interface_hand.cpp)
target_link_libraries(robot_interface_hand Franka::Franka
        dqrobotics)


##### CPP LIBRARY franka low level interface end  #####

##### SAS Franka Robot Driver #####
add_library(sas_robot_driver_franka src/joint/sas_robot_driver_franka.cpp)
target_link_libraries(sas_robot_driver_franka
        ${PROJECT_NAME}_robot_dynamics
        dqrobotics
        dqrobotics-interface-json11
        robot_interface_franka)

##### SAS Franka Robot Driver end  #####
##### qros hand effector driver #####

add_library(qros_effector_driver_franka_hand src/hand/qros_effector_driver_franka_hand.cpp)

ament_target_dependencies(qros_effector_driver_franka_hand
        rclcpp sas_common sas_core
        sas_robot_driver_franka_interfaces
)

#ament_target_dependencies(${PROJECT_NAME}_robot_dynamics
#        rclcpp geometry_msgs std_msgs
#        sas_common sas_core sas_conversions
#        tf2_ros tf2 sas_robot_driver)

target_link_libraries(qros_effector_driver_franka_hand
        dqrobotics
        Franka::Franka)

#target_include_directories(qros_effector_driver_franka_hand
#        PUBLIC
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/interface>
#        $<INSTALL_INTERFACE:include/interface>)

##### qros hand effector driver end  #####


##### SAS Franka Robot Driver Node #####

add_executable(sas_robot_driver_franka_node src/sas_robot_driver_franka_node.cpp)
ament_target_dependencies(sas_robot_driver_franka_node
        rclcpp sas_common sas_core
        sas_robot_driver)
target_link_libraries(sas_robot_driver_franka_node sas_robot_driver_franka)

install(TARGETS
        sas_robot_driver_franka_node
        DESTINATION lib/${PROJECT_NAME})


##### SAS Franka Robot Hand Driver Node #####
add_executable(sas_robot_driver_franka_hand_node src/sas_robot_driver_franka_hand_node.cpp)
ament_target_dependencies(sas_robot_driver_franka_node
        rclcpp sas_common sas_core)
target_link_libraries(sas_robot_driver_franka_hand_node
        qros_effector_driver_franka_hand)

install(TARGETS
        sas_robot_driver_franka_hand_node
        DESTINATION lib/${PROJECT_NAME})

##### Coppelia Mirror Node #####

add_executable(sas_robot_driver_franka_coppelia_node
    src/sas_robot_driver_coppelia_node.cpp
    src/coppelia/sas_robot_driver_coppelia.cpp
)
ament_target_dependencies(sas_robot_driver_franka_coppelia_node
        rclcpp sas_common sas_core sas_conversions sas_robot_driver
)

target_link_libraries(sas_robot_driver_franka_coppelia_node
        dqrobotics
        dqrobotics-interface-json11
        dqrobotics-interface-vrep
)
install(TARGETS
        sas_robot_driver_franka_coppelia_node
        DESTINATION lib/${PROJECT_NAME})

##### JuanEmika

add_executable(JuankaEmika
        qt/configuration_window/main.cpp
        qt/configuration_window/mainwindow.cpp
        qt/configuration_window/mainwindow.ui
)
ament_target_dependencies(sas_robot_driver_franka_node
        rclcpp sas_common sas_core
        sas_robot_driver)

target_link_libraries(JuankaEmika PRIVATE Qt${QT_VERSION_MAJOR}::Widgets
        dqrobotics
        robot_interface_franka
)

if (QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(JuankaEmika)
endif ()

install(TARGETS
        JuankaEmika
        DESTINATION lib/${PROJECT_NAME})


##### PYBIND11 LIBRARY ROBOT_DYNAMICS #####
ament_python_install_package(${PROJECT_NAME})

pybind11_add_module(_robot_dynamics SHARED
        src/robot_dynamics/qros_robot_dynamics_py.cpp
        src/robot_dynamics/qros_robot_dynamics_client.cpp
        src/robot_dynamics/qros_robot_dynamics_server.cpp
)

target_include_directories(_robot_dynamics
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/robot_dynamics>
        $<INSTALL_INTERFACE:include/robot_dynamics>)

target_compile_definitions(_robot_dynamics PRIVATE IS_SAS_PYTHON_BUILD)
# https://github.com/pybind/pybind11/issues/387
target_link_libraries(_robot_dynamics PRIVATE ${PROJECT_NAME}_robot_dynamics -ldqrobotics)

install(TARGETS _robot_dynamics
        DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)
##END## PYBIND11 LIBRARY ROBOT_DYNAMICS #####


install(
        DIRECTORY include/
        DESTINATION include
)




##### LAUNCH FILES #####

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
)

##END## LAUNCH FILES #####

ament_package()







