// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sas_robot_driver_franka_interfaces:srv/Grasp.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sas_robot_driver_franka_interfaces/srv/grasp.hpp"


#ifndef SAS_ROBOT_DRIVER_FRANKA_INTERFACES__SRV__DETAIL__GRASP__BUILDER_HPP_
#define SAS_ROBOT_DRIVER_FRANKA_INTERFACES__SRV__DETAIL__GRASP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sas_robot_driver_franka_interfaces/srv/detail/grasp__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sas_robot_driver_franka_interfaces
{

namespace srv
{

namespace builder
{

class Init_Grasp_Request_epsilon_outer
{
public:
  explicit Init_Grasp_Request_epsilon_outer(::sas_robot_driver_franka_interfaces::srv::Grasp_Request & msg)
  : msg_(msg)
  {}
  ::sas_robot_driver_franka_interfaces::srv::Grasp_Request epsilon_outer(::sas_robot_driver_franka_interfaces::srv::Grasp_Request::_epsilon_outer_type arg)
  {
    msg_.epsilon_outer = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sas_robot_driver_franka_interfaces::srv::Grasp_Request msg_;
};

class Init_Grasp_Request_epsilon_inner
{
public:
  explicit Init_Grasp_Request_epsilon_inner(::sas_robot_driver_franka_interfaces::srv::Grasp_Request & msg)
  : msg_(msg)
  {}
  Init_Grasp_Request_epsilon_outer epsilon_inner(::sas_robot_driver_franka_interfaces::srv::Grasp_Request::_epsilon_inner_type arg)
  {
    msg_.epsilon_inner = std::move(arg);
    return Init_Grasp_Request_epsilon_outer(msg_);
  }

private:
  ::sas_robot_driver_franka_interfaces::srv::Grasp_Request msg_;
};

class Init_Grasp_Request_force
{
public:
  explicit Init_Grasp_Request_force(::sas_robot_driver_franka_interfaces::srv::Grasp_Request & msg)
  : msg_(msg)
  {}
  Init_Grasp_Request_epsilon_inner force(::sas_robot_driver_franka_interfaces::srv::Grasp_Request::_force_type arg)
  {
    msg_.force = std::move(arg);
    return Init_Grasp_Request_epsilon_inner(msg_);
  }

private:
  ::sas_robot_driver_franka_interfaces::srv::Grasp_Request msg_;
};

class Init_Grasp_Request_speed
{
public:
  explicit Init_Grasp_Request_speed(::sas_robot_driver_franka_interfaces::srv::Grasp_Request & msg)
  : msg_(msg)
  {}
  Init_Grasp_Request_force speed(::sas_robot_driver_franka_interfaces::srv::Grasp_Request::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_Grasp_Request_force(msg_);
  }

private:
  ::sas_robot_driver_franka_interfaces::srv::Grasp_Request msg_;
};

class Init_Grasp_Request_width
{
public:
  Init_Grasp_Request_width()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Grasp_Request_speed width(::sas_robot_driver_franka_interfaces::srv::Grasp_Request::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_Grasp_Request_speed(msg_);
  }

private:
  ::sas_robot_driver_franka_interfaces::srv::Grasp_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::sas_robot_driver_franka_interfaces::srv::Grasp_Request>()
{
  return sas_robot_driver_franka_interfaces::srv::builder::Init_Grasp_Request_width();
}

}  // namespace sas_robot_driver_franka_interfaces


namespace sas_robot_driver_franka_interfaces
{

namespace srv
{

namespace builder
{

class Init_Grasp_Response_success
{
public:
  Init_Grasp_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::sas_robot_driver_franka_interfaces::srv::Grasp_Response success(::sas_robot_driver_franka_interfaces::srv::Grasp_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sas_robot_driver_franka_interfaces::srv::Grasp_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::sas_robot_driver_franka_interfaces::srv::Grasp_Response>()
{
  return sas_robot_driver_franka_interfaces::srv::builder::Init_Grasp_Response_success();
}

}  // namespace sas_robot_driver_franka_interfaces


namespace sas_robot_driver_franka_interfaces
{

namespace srv
{

namespace builder
{

class Init_Grasp_Event_response
{
public:
  explicit Init_Grasp_Event_response(::sas_robot_driver_franka_interfaces::srv::Grasp_Event & msg)
  : msg_(msg)
  {}
  ::sas_robot_driver_franka_interfaces::srv::Grasp_Event response(::sas_robot_driver_franka_interfaces::srv::Grasp_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sas_robot_driver_franka_interfaces::srv::Grasp_Event msg_;
};

class Init_Grasp_Event_request
{
public:
  explicit Init_Grasp_Event_request(::sas_robot_driver_franka_interfaces::srv::Grasp_Event & msg)
  : msg_(msg)
  {}
  Init_Grasp_Event_response request(::sas_robot_driver_franka_interfaces::srv::Grasp_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_Grasp_Event_response(msg_);
  }

private:
  ::sas_robot_driver_franka_interfaces::srv::Grasp_Event msg_;
};

class Init_Grasp_Event_info
{
public:
  Init_Grasp_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Grasp_Event_request info(::sas_robot_driver_franka_interfaces::srv::Grasp_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_Grasp_Event_request(msg_);
  }

private:
  ::sas_robot_driver_franka_interfaces::srv::Grasp_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::sas_robot_driver_franka_interfaces::srv::Grasp_Event>()
{
  return sas_robot_driver_franka_interfaces::srv::builder::Init_Grasp_Event_info();
}

}  // namespace sas_robot_driver_franka_interfaces

#endif  // SAS_ROBOT_DRIVER_FRANKA_INTERFACES__SRV__DETAIL__GRASP__BUILDER_HPP_
