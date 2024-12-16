cmake_minimum_required(VERSION 3.5)

project(constraints_manager_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
ADD_COMPILE_OPTIONS(-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../include

    )


add_library(ConstraintsManager ${CMAKE_CURRENT_SOURCE_DIR}/../src/constraints_manager.cpp)
add_executable(constraints_manager_example main.cpp)
target_link_libraries(constraints_manager_example
                      ConstraintsManager)

