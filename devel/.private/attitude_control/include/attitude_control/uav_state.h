// Generated by gencpp from file attitude_control/uav_state.msg
// DO NOT EDIT!


#ifndef ATTITUDE_CONTROL_MESSAGE_UAV_STATE_H
#define ATTITUDE_CONTROL_MESSAGE_UAV_STATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>

namespace attitude_control
{
template <class ContainerAllocator>
struct uav_state_
{
  typedef uav_state_<ContainerAllocator> Type;

  uav_state_()
    : header()
    , position_W()
    , velocity_W()
    , euler_angle()
    , rotation_speed_B()
    , commanded_thrust(0.0)
    , moment()
    , position_ref()
    , velocity_ref()
    , accel_ref()
    , yaw_ref(0.0)
    , speed(0.0)
    , launch_flag(false)  {
    }
  uav_state_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , position_W(_alloc)
    , velocity_W(_alloc)
    , euler_angle(_alloc)
    , rotation_speed_B(_alloc)
    , commanded_thrust(0.0)
    , moment(_alloc)
    , position_ref(_alloc)
    , velocity_ref(_alloc)
    , accel_ref(_alloc)
    , yaw_ref(0.0)
    , speed(0.0)
    , launch_flag(false)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _position_W_type;
  _position_W_type position_W;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _velocity_W_type;
  _velocity_W_type velocity_W;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _euler_angle_type;
  _euler_angle_type euler_angle;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _rotation_speed_B_type;
  _rotation_speed_B_type rotation_speed_B;

   typedef double _commanded_thrust_type;
  _commanded_thrust_type commanded_thrust;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _moment_type;
  _moment_type moment;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _position_ref_type;
  _position_ref_type position_ref;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _velocity_ref_type;
  _velocity_ref_type velocity_ref;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _accel_ref_type;
  _accel_ref_type accel_ref;

   typedef double _yaw_ref_type;
  _yaw_ref_type yaw_ref;

   typedef double _speed_type;
  _speed_type speed;

   typedef uint8_t _launch_flag_type;
  _launch_flag_type launch_flag;





  typedef boost::shared_ptr< ::attitude_control::uav_state_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::attitude_control::uav_state_<ContainerAllocator> const> ConstPtr;

}; // struct uav_state_

typedef ::attitude_control::uav_state_<std::allocator<void> > uav_state;

typedef boost::shared_ptr< ::attitude_control::uav_state > uav_statePtr;
typedef boost::shared_ptr< ::attitude_control::uav_state const> uav_stateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::attitude_control::uav_state_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::attitude_control::uav_state_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::attitude_control::uav_state_<ContainerAllocator1> & lhs, const ::attitude_control::uav_state_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.position_W == rhs.position_W &&
    lhs.velocity_W == rhs.velocity_W &&
    lhs.euler_angle == rhs.euler_angle &&
    lhs.rotation_speed_B == rhs.rotation_speed_B &&
    lhs.commanded_thrust == rhs.commanded_thrust &&
    lhs.moment == rhs.moment &&
    lhs.position_ref == rhs.position_ref &&
    lhs.velocity_ref == rhs.velocity_ref &&
    lhs.accel_ref == rhs.accel_ref &&
    lhs.yaw_ref == rhs.yaw_ref &&
    lhs.speed == rhs.speed &&
    lhs.launch_flag == rhs.launch_flag;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::attitude_control::uav_state_<ContainerAllocator1> & lhs, const ::attitude_control::uav_state_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace attitude_control

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::attitude_control::uav_state_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::attitude_control::uav_state_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::attitude_control::uav_state_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::attitude_control::uav_state_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::attitude_control::uav_state_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::attitude_control::uav_state_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::attitude_control::uav_state_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f9313a03a91ff0a9c49a7d3dc8670c6b";
  }

  static const char* value(const ::attitude_control::uav_state_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf9313a03a91ff0a9ULL;
  static const uint64_t static_value2 = 0xc49a7d3dc8670c6bULL;
};

template<class ContainerAllocator>
struct DataType< ::attitude_control::uav_state_<ContainerAllocator> >
{
  static const char* value()
  {
    return "attitude_control/uav_state";
  }

  static const char* value(const ::attitude_control::uav_state_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::attitude_control::uav_state_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"\n"
"# state\n"
"geometry_msgs/Vector3 position_W\n"
"geometry_msgs/Vector3 velocity_W\n"
"geometry_msgs/Vector3 euler_angle\n"
"geometry_msgs/Vector3 rotation_speed_B\n"
"\n"
"# control\n"
"float64 commanded_thrust\n"
"geometry_msgs/Vector3 moment\n"
"\n"
"# ref trajectory\n"
"geometry_msgs/Vector3 position_ref\n"
"geometry_msgs/Vector3 velocity_ref\n"
"geometry_msgs/Vector3 accel_ref\n"
"float64 yaw_ref\n"
"float64 speed\n"
"bool launch_flag\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::attitude_control::uav_state_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::attitude_control::uav_state_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.position_W);
      stream.next(m.velocity_W);
      stream.next(m.euler_angle);
      stream.next(m.rotation_speed_B);
      stream.next(m.commanded_thrust);
      stream.next(m.moment);
      stream.next(m.position_ref);
      stream.next(m.velocity_ref);
      stream.next(m.accel_ref);
      stream.next(m.yaw_ref);
      stream.next(m.speed);
      stream.next(m.launch_flag);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct uav_state_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::attitude_control::uav_state_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::attitude_control::uav_state_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "position_W: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.position_W);
    s << indent << "velocity_W: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.velocity_W);
    s << indent << "euler_angle: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.euler_angle);
    s << indent << "rotation_speed_B: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.rotation_speed_B);
    s << indent << "commanded_thrust: ";
    Printer<double>::stream(s, indent + "  ", v.commanded_thrust);
    s << indent << "moment: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.moment);
    s << indent << "position_ref: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.position_ref);
    s << indent << "velocity_ref: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.velocity_ref);
    s << indent << "accel_ref: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.accel_ref);
    s << indent << "yaw_ref: ";
    Printer<double>::stream(s, indent + "  ", v.yaw_ref);
    s << indent << "speed: ";
    Printer<double>::stream(s, indent + "  ", v.speed);
    s << indent << "launch_flag: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.launch_flag);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ATTITUDE_CONTROL_MESSAGE_UAV_STATE_H