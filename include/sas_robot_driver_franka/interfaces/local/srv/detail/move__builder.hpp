// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sas_robot_driver_franka_interfaces:srv/Move.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sas_robot_driver_franka_interfaces/srv/move.hpp"


#ifndef SAS_ROBOT_DRIVER_FRANKA_INTERFACES__SRV__DETAIL__MOVE__BUILDER_HPP_
#define SAS_ROBOT_DRIVER_FRANKA_INTERFACES__SRV__DETAIL__MOVE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sas_robot_driver_franka_interfaces/srv/detail/move__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sas_robot_driver_franka_interfaces
{

namespace srv
{

namespace builder
{

class Init_Move_Request_speed
{
public:
  explicit Init_Move_Request_speed(::sas_robot_driver_franka_interfaces::srv::Move_Request & msg)
  : msg_(msg)
  {}
  ::sas_robot_driver_franka_interfaces::srv::Move_Request speed(::sas_robot_driver_franka_interfaces::srv::Move_Request::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sas_robot_driver_franka_interfaces::srv::Move_Request msg_;
};

class Init_Move_Request_width
{
public:
  Init_Move_Request_width()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Move_Request_speed width(::sas_robot_driver_franka_interfaces::srv::Move_Request::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_Move_Request_speed(msg_);
  }

private:
  ::sas_robot_driver_franka_interfaces::srv::Move_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::sas_robot_driver_franka_interfaces::srv::Move_Request>()
{
  return sas_robot_driver_franka_interfaces::srv::builder::Init_Move_Request_width();
}

}  // namespace sas_robot_driver_franka_interfaces


namespace sas_robot_driver_franka_interfaces
{

namespace srv
{

namespace builder
{

class Init_Move_Response_success
{
public:
  Init_Move_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::sas_robot_driver_franka_interfaces::srv::Move_Response success(::sas_robot_driver_franka_interfaces::srv::Move_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sas_robot_driver_franka_interfaces::srv::Move_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::sas_robot_driver_franka_interfaces::srv::Move_Response>()
{
  return sas_robot_driver_franka_interfaces::srv::builder::Init_Move_Response_success();
}

}  // namespace sas_robot_driver_franka_interfaces


namespace sas_robot_driver_franka_interfaces
{

namespace srv
{

namespace builder
{

class Init_Move_Event_response
{
public:
  explicit Init_Move_Event_response(::sas_robot_driver_franka_interfaces::srv::Move_Event & msg)
  : msg_(msg)
  {}
  ::sas_robot_driver_franka_interfaces::srv::Move_Event response(::sas_robot_driver_franka_interfaces::srv::Move_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sas_robot_driver_franka_interfaces::srv::Move_Event msg_;
};

class Init_Move_Event_request
{
public:
  explicit Init_Move_Event_request(::sas_robot_driver_franka_interfaces::srv::Move_Event & msg)
  : msg_(msg)
  {}
  Init_Move_Event_response request(::sas_robot_driver_franka_interfaces::srv::Move_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_Move_Event_response(msg_);
  }

private:
  ::sas_robot_driver_franka_interfaces::srv::Move_Event msg_;
};

class Init_Move_Event_info
{
public:
  Init_Move_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Move_Event_request info(::sas_robot_driver_franka_interfaces::srv::Move_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_Move_Event_request(msg_);
  }

private:
  ::sas_robot_driver_franka_interfaces::srv::Move_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::sas_robot_driver_franka_interfaces::srv::Move_Event>()
{
  return sas_robot_driver_franka_interfaces::srv::builder::Init_Move_Event_info();
}

}  // namespace sas_robot_driver_franka_interfaces

#endif  // SAS_ROBOT_DRIVER_FRANKA_INTERFACES__SRV__DETAIL__MOVE__BUILDER_HPP_
