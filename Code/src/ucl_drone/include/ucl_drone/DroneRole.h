// Generated by gencpp from file ucl_drone/DroneRole.msg
// DO NOT EDIT!


#ifndef UCL_DRONE_MESSAGE_DRONEROLE_H
#define UCL_DRONE_MESSAGE_DRONEROLE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ucl_drone
{
template <class ContainerAllocator>
struct DroneRole_
{
  typedef DroneRole_<ContainerAllocator> Type;

  DroneRole_()
    : name()
    , role(0.0)
    , params()  {
    }
  DroneRole_(const ContainerAllocator& _alloc)
    : name(_alloc)
    , role(0.0)
    , params(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  _name_type name;

   typedef double _role_type;
  _role_type role;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _params_type;
  _params_type params;




  typedef boost::shared_ptr< ::ucl_drone::DroneRole_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ucl_drone::DroneRole_<ContainerAllocator> const> ConstPtr;

}; // struct DroneRole_

typedef ::ucl_drone::DroneRole_<std::allocator<void> > DroneRole;

typedef boost::shared_ptr< ::ucl_drone::DroneRole > DroneRolePtr;
typedef boost::shared_ptr< ::ucl_drone::DroneRole const> DroneRoleConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ucl_drone::DroneRole_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ucl_drone::DroneRole_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ucl_drone

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'ucl_drone': ['/home/laboinmastudent/Bureau/TFE_Alex_Arnaud/ucl_drone_ws/src/ucl_drone/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'ardrone_autonomy': ['/home/laboinmastudent/Bureau/TFE_Alex_Arnaud/ucl_drone_ws/src/ardrone_autonomy/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ucl_drone::DroneRole_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ucl_drone::DroneRole_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ucl_drone::DroneRole_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ucl_drone::DroneRole_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ucl_drone::DroneRole_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ucl_drone::DroneRole_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ucl_drone::DroneRole_<ContainerAllocator> >
{
  static const char* value()
  {
    return "aa46bcf5f74539f0bbc0ce8ce5b59675";
  }

  static const char* value(const ::ucl_drone::DroneRole_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xaa46bcf5f74539f0ULL;
  static const uint64_t static_value2 = 0xbbc0ce8ce5b59675ULL;
};

template<class ContainerAllocator>
struct DataType< ::ucl_drone::DroneRole_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ucl_drone/DroneRole";
  }

  static const char* value(const ::ucl_drone::DroneRole_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ucl_drone::DroneRole_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
# This message contains the object drone role, formulated by the multi-agent\n\
# strategy and read by each drone's IA.\n\
\n\
# Drone name\n\
string name\n\
\n\
# Role code\n\
float64 role\n\
\n\
# List of input-output topic names. Refer to the role code for convention about\n\
# the specific use of each field\n\
string[] params\n\
";
  }

  static const char* value(const ::ucl_drone::DroneRole_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ucl_drone::DroneRole_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.name);
      stream.next(m.role);
      stream.next(m.params);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DroneRole_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ucl_drone::DroneRole_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ucl_drone::DroneRole_<ContainerAllocator>& v)
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "role: ";
    Printer<double>::stream(s, indent + "  ", v.role);
    s << indent << "params[]" << std::endl;
    for (size_t i = 0; i < v.params.size(); ++i)
    {
      s << indent << "  params[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.params[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // UCL_DRONE_MESSAGE_DRONEROLE_H
