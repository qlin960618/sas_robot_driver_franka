// Generated by gencpp from file sas_robot_driver_franka/MoveResponse.msg
// DO NOT EDIT!


#ifndef SAS_ROBOT_DRIVER_FRANKA_MESSAGE_MOVERESPONSE_H
#define SAS_ROBOT_DRIVER_FRANKA_MESSAGE_MOVERESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace sas_robot_driver_franka
{
template <class ContainerAllocator>
struct MoveResponse_
{
  typedef MoveResponse_<ContainerAllocator> Type;

  MoveResponse_()
    : success(false)  {
    }
  MoveResponse_(const ContainerAllocator& _alloc)
    : success(false)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;





  typedef boost::shared_ptr< ::sas_robot_driver_franka::MoveResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::sas_robot_driver_franka::MoveResponse_<ContainerAllocator> const> ConstPtr;

}; // struct MoveResponse_

typedef ::sas_robot_driver_franka::MoveResponse_<std::allocator<void> > MoveResponse;

typedef boost::shared_ptr< ::sas_robot_driver_franka::MoveResponse > MoveResponsePtr;
typedef boost::shared_ptr< ::sas_robot_driver_franka::MoveResponse const> MoveResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::sas_robot_driver_franka::MoveResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::sas_robot_driver_franka::MoveResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::sas_robot_driver_franka::MoveResponse_<ContainerAllocator1> & lhs, const ::sas_robot_driver_franka::MoveResponse_<ContainerAllocator2> & rhs)
{
  return lhs.success == rhs.success;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::sas_robot_driver_franka::MoveResponse_<ContainerAllocator1> & lhs, const ::sas_robot_driver_franka::MoveResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace sas_robot_driver_franka

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::sas_robot_driver_franka::MoveResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sas_robot_driver_franka::MoveResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sas_robot_driver_franka::MoveResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sas_robot_driver_franka::MoveResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sas_robot_driver_franka::MoveResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sas_robot_driver_franka::MoveResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::sas_robot_driver_franka::MoveResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const ::sas_robot_driver_franka::MoveResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::sas_robot_driver_franka::MoveResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sas_robot_driver_franka/MoveResponse";
  }

  static const char* value(const ::sas_robot_driver_franka::MoveResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::sas_robot_driver_franka::MoveResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool success\n"
;
  }

  static const char* value(const ::sas_robot_driver_franka::MoveResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::sas_robot_driver_franka::MoveResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MoveResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sas_robot_driver_franka::MoveResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::sas_robot_driver_franka::MoveResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SAS_ROBOT_DRIVER_FRANKA_MESSAGE_MOVERESPONSE_H
