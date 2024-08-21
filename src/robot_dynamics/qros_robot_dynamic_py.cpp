/*
# Copyright (c) 2023 Juan Jose Quiroz Omana
#
#    This file is part of sas_robot_driver_franka.
#
#    sas_robot_driver_franka is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_robot_driver_franka is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_robot_driver.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Quenitin Lin
#
# ################################################################
#
# Contributors:
#      1. Quenitin Lin
#
# ################################################################
*/

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <sas_robot_driver_franka/robot_dynamic/qros_robot_dynamics_provider.h>
#include <sas_robot_driver_franka/robot_dynamic/qros_robot_dynamics_interface.h>

namespace py = pybind11;
using RDI = qros::RobotDynamicInterface;
using RDP = qros::RobotDynamicProvider;


PYBIND11_MODULE(_qros_robot_dynamic, m)
{
    py::class_<RDI>(m, "RobotDynamicsInterface")
        .def(py::init<const std::string&>())
        .def("get_stiffness_force",&RDI::get_stiffness_force)
        .def("get_stiffness_torque",&RDI::get_stiffness_torque)
        .def("get_stiffness_frame_pose",&RDI::get_stiffness_frame_pose)
        .def("is_enabled",&RDI::is_enabled,"Returns true if the RobotDynamicInterface is enabled.")
        .def("get_topic_prefix",&RDI::get_topic_prefix);

    py::class_<RDP>(m, "RobotDynamicsProvider")
        .def(py::init<const std::string&>())
        .def("publish_stiffness",&RDP::publish_stiffness)
        .def("set_world_to_base_tf", &RDP::set_world_to_base_tf)
        .def("is_enabled",&RDP::is_enabled,"Returns true if the RobotDynamicProvider is enabled.")
        .def("get_topic_prefix",&RDP::get_topic_prefix);

}
