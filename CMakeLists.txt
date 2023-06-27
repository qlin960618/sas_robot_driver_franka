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
  sas_common
  sas_clock
  sas_robot_driver
  sas_patient_side_manager
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy sas_common sas_clock sas_robot_driver
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
if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()
find_package(Qt5 COMPONENTS Widgets REQUIRED)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)


add_library(MotionGenerator src/motion_generator.cpp)
target_link_libraries(MotionGenerator Franka::Franka)


add_library(ConstraintsManager constraints_manager/src/constraints_manager.cpp)

add_library(QuadraticProgramMotionGenerator src/quadratic_program_motion_generator.cpp)
target_link_libraries(QuadraticProgramMotionGenerator
                       qpOASES
                       dqrobotics
                       ConstraintsManager)

add_library(CustomMotionGeneration src/custom_motion_generation.cpp)
target_link_libraries(CustomMotionGeneration
                        qpOASES
                        dqrobotics
                        ConstraintsManager)


add_library(robot_interface_franka src/robot_interface_franka.cpp)
target_link_libraries(robot_interface_franka Franka::Franka
                      dqrobotics
                      MotionGenerator
                      QuadraticProgramMotionGenerator
                      CustomMotionGeneration)



############
## Build ###
############

## Specify additional locations of header files
## Your package locations should be listed before other locations


include_directories(
    include
    src/
    ${catkin_INCLUDE_DIRS}
    constraints_manager/include
    )



add_library(sas_robot_driver_franka src/sas_robot_driver_franka.cpp)
target_link_libraries(sas_robot_driver_franka
                      dqrobotics
                      robot_interface_franka)




add_dependencies(sas_robot_driver_franka ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})                        

add_executable(sas_robot_driver_franka_node src/sas_robot_driver_franka_node.cpp)                        
target_link_libraries(sas_robot_driver_franka_node
                      sas_robot_driver_franka 
                      ${catkin_LIBRARIES})



add_executable(JuankaEmika
  qt/configuration_window/main.cpp
  qt/configuration_window/mainwindow.cpp
  qt/configuration_window/mainwindow.ui
  )

target_link_libraries(JuankaEmika  PRIVATE Qt${QT_VERSION_MAJOR}::Widgets
    dqrobotics

    ${catkin_LIBRARIES}
    robot_interface_franka
    )




if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(JuankaEmika)
endif()

install(TARGETS ${PROJECT_NAME}
DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS sas_robot_driver_franka_node
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
)                        
