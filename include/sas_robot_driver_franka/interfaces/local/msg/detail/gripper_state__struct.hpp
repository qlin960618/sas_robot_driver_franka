// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sas_robot_driver_franka_interfaces:msg/GripperState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sas_robot_driver_franka_interfaces/msg/gripper_state.hpp"


#ifndef SAS_ROBOT_DRIVER_FRANKA_INTERFACES__MSG__DETAIL__GRIPPER_STATE__STRUCT_HPP_
#define SAS_ROBOT_DRIVER_FRANKA_INTERFACES__MSG__DETAIL__GRIPPER_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__sas_robot_driver_franka_interfaces__msg__GripperState __attribute__((deprecated))
#else
# define DEPRECATED__sas_robot_driver_franka_interfaces__msg__GripperState __declspec(deprecated)
#endif

namespace sas_robot_driver_franka_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GripperState_
{
  using Type = GripperState_<ContainerAllocator>;

  explicit GripperState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->width = 0.0f;
      this->max_width = 0.0f;
      this->is_grasped = false;
      this->temperature = 0;
      this->duration_ms = 0ull;
    }
  }

  explicit GripperState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->width = 0.0f;
      this->max_width = 0.0f;
      this->is_grasped = false;
      this->temperature = 0;
      this->duration_ms = 0ull;
    }
  }

  // field types and members
  using _width_type =
    float;
  _width_type width;
  using _max_width_type =
    float;
  _max_width_type max_width;
  using _is_grasped_type =
    bool;
  _is_grasped_type is_grasped;
  using _temperature_type =
    uint16_t;
  _temperature_type temperature;
  using _duration_ms_type =
    uint64_t;
  _duration_ms_type duration_ms;

  // setters for named parameter idiom
  Type & set__width(
    const float & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__max_width(
    const float & _arg)
  {
    this->max_width = _arg;
    return *this;
  }
  Type & set__is_grasped(
    const bool & _arg)
  {
    this->is_grasped = _arg;
    return *this;
  }
  Type & set__temperature(
    const uint16_t & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__duration_ms(
    const uint64_t & _arg)
  {
    this->duration_ms = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sas_robot_driver_franka_interfaces::msg::GripperState_<ContainerAllocator> *;
  using ConstRawPtr =
    const sas_robot_driver_franka_interfaces::msg::GripperState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sas_robot_driver_franka_interfaces::msg::GripperState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sas_robot_driver_franka_interfaces::msg::GripperState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sas_robot_driver_franka_interfaces::msg::GripperState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sas_robot_driver_franka_interfaces::msg::GripperState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sas_robot_driver_franka_interfaces::msg::GripperState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sas_robot_driver_franka_interfaces::msg::GripperState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sas_robot_driver_franka_interfaces::msg::GripperState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sas_robot_driver_franka_interfaces::msg::GripperState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sas_robot_driver_franka_interfaces__msg__GripperState
    std::shared_ptr<sas_robot_driver_franka_interfaces::msg::GripperState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sas_robot_driver_franka_interfaces__msg__GripperState
    std::shared_ptr<sas_robot_driver_franka_interfaces::msg::GripperState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GripperState_ & other) const
  {
    if (this->width != other.width) {
      return false;
    }
    if (this->max_width != other.max_width) {
      return false;
    }
    if (this->is_grasped != other.is_grasped) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->duration_ms != other.duration_ms) {
      return false;
    }
    return true;
  }
  bool operator!=(const GripperState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GripperState_

// alias to use template instance with default allocator
using GripperState =
  sas_robot_driver_franka_interfaces::msg::GripperState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sas_robot_driver_franka_interfaces

#endif  // SAS_ROBOT_DRIVER_FRANKA_INTERFACES__MSG__DETAIL__GRIPPER_STATE__STRUCT_HPP_
