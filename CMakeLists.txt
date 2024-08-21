cmake_minimum_required(VERSION 3.0.2)
project(sas_robot_driver_franka)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
#add_compile_options(-std=c++11)
add_compile_options(-Werror=return-type)


#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
#add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})
add_custom_target(cfg ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        tf2_ros
        tf2
        sas_common
        sas_clock
        sas_robot_driver
        sas_patient_side_manager
        message_generation
        pybind11_catkin
)


add_service_files(
        DIRECTORY srv
        FILES
        Move.srv
        Grasp.srv
)

add_message_files(
        DIRECTORY msg
        FILES
        GripperState.msg
)

catkin_python_setup()

generate_messages(
        DEPENDENCIES
        std_msgs

)


catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp rospy sas_common sas_clock sas_robot_driver tf2_ros tf2 pybind11_catkin message_runtime std_msgs
)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# To correctly find and link with QT
set(CMAKE_PREFIX_PATH $ENV{QT_PATH})
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
if (CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif ()
find_package(Qt5 COMPONENTS Widgets REQUIRED)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)


add_library(MotionGenerator src/generator/motion_generator.cpp)
target_link_libraries(MotionGenerator Franka::Franka)


add_library(ConstraintsManager constraints_manager/src/constraints_manager.cpp)

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


############
## Build ###
############

## Specify additional locations of header files
## Your package locations should be listed before other locations


include_directories(
        include
        src/
        src/robot_dynamics
        src/hand
        src/joint
        ${catkin_INCLUDE_DIRS}
        constraints_manager/include
)


add_library(qros_robot_dynamics_provider src/robot_dynamics/qros_robot_dynamics_provider.cpp)
target_link_libraries(qros_robot_dynamics_provider
        ${catkin_LIBRARIES}
        dqrobotics)

add_library(qros_robot_dynamics_interface src/robot_dynamics/qros_robot_dynamics_interface.cpp)
target_link_libraries(qros_robot_dynamics_interface
        ${catkin_LIBRARIES}
        dqrobotics)

add_dependencies(qros_robot_dynamics_interface ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(qros_robot_dynamics_interface ${catkin_EXPORTED_TARGETS})


add_library(sas_robot_driver_franka src/joint/sas_robot_driver_franka.cpp)
target_link_libraries(sas_robot_driver_franka
        qros_robot_dynamics_provider
        dqrobotics
        dqrobotics-interface-json11
        robot_interface_franka)

add_library(qros_effector_driver_franka_hand src/hand/qros_effector_driver_franka_hand.cpp)
target_link_libraries(qros_effector_driver_franka_hand
        dqrobotics
        Franka::Franka)
add_dependencies(qros_effector_driver_franka_hand ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(qros_effector_driver_franka_hand ${catkin_EXPORTED_TARGETS})

add_library(sas_robot_driver_coppelia src/coppelia/sas_robot_driver_coppelia.cpp)
target_link_libraries(sas_robot_driver_coppelia
        dqrobotics
        dqrobotics-interface-json11
        dqrobotics-interface-vrep)

add_dependencies(sas_robot_driver_franka ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sas_robot_driver_coppelia ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(qros_effector_driver_franka_hand ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(sas_robot_driver_coppelia_node src/sas_robot_driver_coppelia_node.cpp)
target_link_libraries(sas_robot_driver_coppelia_node
        sas_robot_driver_coppelia
        ${catkin_LIBRARIES})

add_executable(sas_robot_driver_franka_node src/sas_robot_driver_franka_node.cpp)
target_link_libraries(sas_robot_driver_franka_node
        sas_robot_driver_franka
        ${catkin_LIBRARIES})


add_executable(sas_robot_driver_franka_hand_node src/sas_robot_driver_franka_hand_node.cpp)
target_link_libraries(sas_robot_driver_franka_hand_node
        qros_effector_driver_franka_hand
        ${catkin_LIBRARIES})


add_executable(JuankaEmika
        qt/configuration_window/main.cpp
        qt/configuration_window/mainwindow.cpp
        qt/configuration_window/mainwindow.ui
)

target_link_libraries(JuankaEmika PRIVATE Qt${QT_VERSION_MAJOR}::Widgets
        dqrobotics
        ${catkin_LIBRARIES}
        robot_interface_franka
)


#####################################################################################
# python binding
include_directories(
        include/sas_robot_driver_franka/robot_dynamic
)
pybind_add_module(_qros_robot_dynamic SHARED
        src/robot_dynamics/qros_robot_dynamic_py.cpp src/robot_dynamics/qros_robot_dynamics_interface.cpp src/robot_dynamics/qros_robot_dynamics_provider.cpp
)
add_dependencies(_qros_robot_dynamic ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_compile_definitions(_qros_robot_dynamic PRIVATE BUILD_PYBIND)
# https://github.com/pybind/pybind11/issues/387
target_link_libraries(_qros_robot_dynamic PRIVATE ${catkin_LIBRARIES} -ldqrobotics)


if (QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(JuankaEmika)
endif ()

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS sas_robot_driver_franka_node
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS sas_robot_driver_coppelia_node
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS sas_robot_driver_franka_hand_node
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS qros_robot_dynamics_provider
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS qros_robot_dynamics_interface
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        #   PATTERN ".svn" EXCLUDE
)


install(TARGETS _qros_robot_dynamic
        LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}
)

