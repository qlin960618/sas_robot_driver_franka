// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sas_robot_driver_franka_interfaces:srv/Move.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sas_robot_driver_franka_interfaces/srv/move.hpp"


#ifndef SAS_ROBOT_DRIVER_FRANKA_INTERFACES__SRV__DETAIL__MOVE__TRAITS_HPP_
#define SAS_ROBOT_DRIVER_FRANKA_INTERFACES__SRV__DETAIL__MOVE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sas_robot_driver_franka_interfaces/srv/detail/move__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace sas_robot_driver_franka_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Move_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: width
  {
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << ", ";
  }

  // member: speed
  {
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Move_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << "\n";
  }

  // member: speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Move_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace sas_robot_driver_franka_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use sas_robot_driver_franka_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const sas_robot_driver_franka_interfaces::srv::Move_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  sas_robot_driver_franka_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sas_robot_driver_franka_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const sas_robot_driver_franka_interfaces::srv::Move_Request & msg)
{
  return sas_robot_driver_franka_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<sas_robot_driver_franka_interfaces::srv::Move_Request>()
{
  return "sas_robot_driver_franka_interfaces::srv::Move_Request";
}

template<>
inline const char * name<sas_robot_driver_franka_interfaces::srv::Move_Request>()
{
  return "sas_robot_driver_franka_interfaces/srv/Move_Request";
}

template<>
struct has_fixed_size<sas_robot_driver_franka_interfaces::srv::Move_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<sas_robot_driver_franka_interfaces::srv::Move_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<sas_robot_driver_franka_interfaces::srv::Move_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace sas_robot_driver_franka_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Move_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Move_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Move_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace sas_robot_driver_franka_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use sas_robot_driver_franka_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const sas_robot_driver_franka_interfaces::srv::Move_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  sas_robot_driver_franka_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sas_robot_driver_franka_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const sas_robot_driver_franka_interfaces::srv::Move_Response & msg)
{
  return sas_robot_driver_franka_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<sas_robot_driver_franka_interfaces::srv::Move_Response>()
{
  return "sas_robot_driver_franka_interfaces::srv::Move_Response";
}

template<>
inline const char * name<sas_robot_driver_franka_interfaces::srv::Move_Response>()
{
  return "sas_robot_driver_franka_interfaces/srv/Move_Response";
}

template<>
struct has_fixed_size<sas_robot_driver_franka_interfaces::srv::Move_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<sas_robot_driver_franka_interfaces::srv::Move_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<sas_robot_driver_franka_interfaces::srv::Move_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace sas_robot_driver_franka_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Move_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Move_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Move_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace sas_robot_driver_franka_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use sas_robot_driver_franka_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const sas_robot_driver_franka_interfaces::srv::Move_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  sas_robot_driver_franka_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sas_robot_driver_franka_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const sas_robot_driver_franka_interfaces::srv::Move_Event & msg)
{
  return sas_robot_driver_franka_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<sas_robot_driver_franka_interfaces::srv::Move_Event>()
{
  return "sas_robot_driver_franka_interfaces::srv::Move_Event";
}

template<>
inline const char * name<sas_robot_driver_franka_interfaces::srv::Move_Event>()
{
  return "sas_robot_driver_franka_interfaces/srv/Move_Event";
}

template<>
struct has_fixed_size<sas_robot_driver_franka_interfaces::srv::Move_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<sas_robot_driver_franka_interfaces::srv::Move_Event>
  : std::integral_constant<bool, has_bounded_size<sas_robot_driver_franka_interfaces::srv::Move_Request>::value && has_bounded_size<sas_robot_driver_franka_interfaces::srv::Move_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<sas_robot_driver_franka_interfaces::srv::Move_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<sas_robot_driver_franka_interfaces::srv::Move>()
{
  return "sas_robot_driver_franka_interfaces::srv::Move";
}

template<>
inline const char * name<sas_robot_driver_franka_interfaces::srv::Move>()
{
  return "sas_robot_driver_franka_interfaces/srv/Move";
}

template<>
struct has_fixed_size<sas_robot_driver_franka_interfaces::srv::Move>
  : std::integral_constant<
    bool,
    has_fixed_size<sas_robot_driver_franka_interfaces::srv::Move_Request>::value &&
    has_fixed_size<sas_robot_driver_franka_interfaces::srv::Move_Response>::value
  >
{
};

template<>
struct has_bounded_size<sas_robot_driver_franka_interfaces::srv::Move>
  : std::integral_constant<
    bool,
    has_bounded_size<sas_robot_driver_franka_interfaces::srv::Move_Request>::value &&
    has_bounded_size<sas_robot_driver_franka_interfaces::srv::Move_Response>::value
  >
{
};

template<>
struct is_service<sas_robot_driver_franka_interfaces::srv::Move>
  : std::true_type
{
};

template<>
struct is_service_request<sas_robot_driver_franka_interfaces::srv::Move_Request>
  : std::true_type
{
};

template<>
struct is_service_response<sas_robot_driver_franka_interfaces::srv::Move_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // SAS_ROBOT_DRIVER_FRANKA_INTERFACES__SRV__DETAIL__MOVE__TRAITS_HPP_
