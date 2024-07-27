// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sas_robot_driver_franka_interfaces:srv/Move.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sas_robot_driver_franka_interfaces/srv/move.hpp"


#ifndef SAS_ROBOT_DRIVER_FRANKA_INTERFACES__SRV__DETAIL__MOVE__STRUCT_HPP_
#define SAS_ROBOT_DRIVER_FRANKA_INTERFACES__SRV__DETAIL__MOVE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__sas_robot_driver_franka_interfaces__srv__Move_Request __attribute__((deprecated))
#else
# define DEPRECATED__sas_robot_driver_franka_interfaces__srv__Move_Request __declspec(deprecated)
#endif

namespace sas_robot_driver_franka_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Move_Request_
{
  using Type = Move_Request_<ContainerAllocator>;

  explicit Move_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->width = 0.0f;
      this->speed = 0.0f;
    }
  }

  explicit Move_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->width = 0.0f;
      this->speed = 0.0f;
    }
  }

  // field types and members
  using _width_type =
    float;
  _width_type width;
  using _speed_type =
    float;
  _speed_type speed;

  // setters for named parameter idiom
  Type & set__width(
    const float & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__speed(
    const float & _arg)
  {
    this->speed = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sas_robot_driver_franka_interfaces::srv::Move_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const sas_robot_driver_franka_interfaces::srv::Move_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sas_robot_driver_franka_interfaces::srv::Move_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sas_robot_driver_franka_interfaces::srv::Move_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sas_robot_driver_franka_interfaces::srv::Move_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sas_robot_driver_franka_interfaces::srv::Move_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sas_robot_driver_franka_interfaces::srv::Move_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sas_robot_driver_franka_interfaces::srv::Move_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sas_robot_driver_franka_interfaces::srv::Move_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sas_robot_driver_franka_interfaces::srv::Move_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sas_robot_driver_franka_interfaces__srv__Move_Request
    std::shared_ptr<sas_robot_driver_franka_interfaces::srv::Move_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sas_robot_driver_franka_interfaces__srv__Move_Request
    std::shared_ptr<sas_robot_driver_franka_interfaces::srv::Move_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Move_Request_ & other) const
  {
    if (this->width != other.width) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    return true;
  }
  bool operator!=(const Move_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Move_Request_

// alias to use template instance with default allocator
using Move_Request =
  sas_robot_driver_franka_interfaces::srv::Move_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace sas_robot_driver_franka_interfaces


#ifndef _WIN32
# define DEPRECATED__sas_robot_driver_franka_interfaces__srv__Move_Response __attribute__((deprecated))
#else
# define DEPRECATED__sas_robot_driver_franka_interfaces__srv__Move_Response __declspec(deprecated)
#endif

namespace sas_robot_driver_franka_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Move_Response_
{
  using Type = Move_Response_<ContainerAllocator>;

  explicit Move_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit Move_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sas_robot_driver_franka_interfaces::srv::Move_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const sas_robot_driver_franka_interfaces::srv::Move_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sas_robot_driver_franka_interfaces::srv::Move_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sas_robot_driver_franka_interfaces::srv::Move_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sas_robot_driver_franka_interfaces::srv::Move_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sas_robot_driver_franka_interfaces::srv::Move_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sas_robot_driver_franka_interfaces::srv::Move_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sas_robot_driver_franka_interfaces::srv::Move_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sas_robot_driver_franka_interfaces::srv::Move_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sas_robot_driver_franka_interfaces::srv::Move_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sas_robot_driver_franka_interfaces__srv__Move_Response
    std::shared_ptr<sas_robot_driver_franka_interfaces::srv::Move_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sas_robot_driver_franka_interfaces__srv__Move_Response
    std::shared_ptr<sas_robot_driver_franka_interfaces::srv::Move_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Move_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const Move_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Move_Response_

// alias to use template instance with default allocator
using Move_Response =
  sas_robot_driver_franka_interfaces::srv::Move_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace sas_robot_driver_franka_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__sas_robot_driver_franka_interfaces__srv__Move_Event __attribute__((deprecated))
#else
# define DEPRECATED__sas_robot_driver_franka_interfaces__srv__Move_Event __declspec(deprecated)
#endif

namespace sas_robot_driver_franka_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Move_Event_
{
  using Type = Move_Event_<ContainerAllocator>;

  explicit Move_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit Move_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<sas_robot_driver_franka_interfaces::srv::Move_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sas_robot_driver_franka_interfaces::srv::Move_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<sas_robot_driver_franka_interfaces::srv::Move_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sas_robot_driver_franka_interfaces::srv::Move_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<sas_robot_driver_franka_interfaces::srv::Move_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sas_robot_driver_franka_interfaces::srv::Move_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<sas_robot_driver_franka_interfaces::srv::Move_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sas_robot_driver_franka_interfaces::srv::Move_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sas_robot_driver_franka_interfaces::srv::Move_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const sas_robot_driver_franka_interfaces::srv::Move_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sas_robot_driver_franka_interfaces::srv::Move_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sas_robot_driver_franka_interfaces::srv::Move_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sas_robot_driver_franka_interfaces::srv::Move_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sas_robot_driver_franka_interfaces::srv::Move_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sas_robot_driver_franka_interfaces::srv::Move_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sas_robot_driver_franka_interfaces::srv::Move_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sas_robot_driver_franka_interfaces::srv::Move_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sas_robot_driver_franka_interfaces::srv::Move_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sas_robot_driver_franka_interfaces__srv__Move_Event
    std::shared_ptr<sas_robot_driver_franka_interfaces::srv::Move_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sas_robot_driver_franka_interfaces__srv__Move_Event
    std::shared_ptr<sas_robot_driver_franka_interfaces::srv::Move_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Move_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const Move_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Move_Event_

// alias to use template instance with default allocator
using Move_Event =
  sas_robot_driver_franka_interfaces::srv::Move_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace sas_robot_driver_franka_interfaces

namespace sas_robot_driver_franka_interfaces
{

namespace srv
{

struct Move
{
  using Request = sas_robot_driver_franka_interfaces::srv::Move_Request;
  using Response = sas_robot_driver_franka_interfaces::srv::Move_Response;
  using Event = sas_robot_driver_franka_interfaces::srv::Move_Event;
};

}  // namespace srv

}  // namespace sas_robot_driver_franka_interfaces

#endif  // SAS_ROBOT_DRIVER_FRANKA_INTERFACES__SRV__DETAIL__MOVE__STRUCT_HPP_
