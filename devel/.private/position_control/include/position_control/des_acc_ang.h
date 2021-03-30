// Generated by gencpp from file position_control/des_acc_ang.msg
// DO NOT EDIT!


#ifndef POSITION_CONTROL_MESSAGE_DES_ACC_ANG_H
#define POSITION_CONTROL_MESSAGE_DES_ACC_ANG_H


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

namespace position_control
{
template <class ContainerAllocator>
struct des_acc_ang_
{
  typedef des_acc_ang_<ContainerAllocator> Type;

  des_acc_ang_()
    : header()
    , accel_out()
    , ang_vel_out()  {
    }
  des_acc_ang_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , accel_out(_alloc)
    , ang_vel_out(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _accel_out_type;
  _accel_out_type accel_out;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _ang_vel_out_type;
  _ang_vel_out_type ang_vel_out;





  typedef boost::shared_ptr< ::position_control::des_acc_ang_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::position_control::des_acc_ang_<ContainerAllocator> const> ConstPtr;

}; // struct des_acc_ang_

typedef ::position_control::des_acc_ang_<std::allocator<void> > des_acc_ang;

typedef boost::shared_ptr< ::position_control::des_acc_ang > des_acc_angPtr;
typedef boost::shared_ptr< ::position_control::des_acc_ang const> des_acc_angConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::position_control::des_acc_ang_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::position_control::des_acc_ang_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::position_control::des_acc_ang_<ContainerAllocator1> & lhs, const ::position_control::des_acc_ang_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.accel_out == rhs.accel_out &&
    lhs.ang_vel_out == rhs.ang_vel_out;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::position_control::des_acc_ang_<ContainerAllocator1> & lhs, const ::position_control::des_acc_ang_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace position_control

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::position_control::des_acc_ang_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::position_control::des_acc_ang_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::position_control::des_acc_ang_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::position_control::des_acc_ang_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::position_control::des_acc_ang_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::position_control::des_acc_ang_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::position_control::des_acc_ang_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f9a93238a0cb64bcae38d4c887a94d2a";
  }

  static const char* value(const ::position_control::des_acc_ang_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf9a93238a0cb64bcULL;
  static const uint64_t static_value2 = 0xae38d4c887a94d2aULL;
};

template<class ContainerAllocator>
struct DataType< ::position_control::des_acc_ang_<ContainerAllocator> >
{
  static const char* value()
  {
    return "position_control/des_acc_ang";
  }

  static const char* value(const ::position_control::des_acc_ang_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::position_control::des_acc_ang_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"\n"
"# references\n"
"geometry_msgs/Vector3 accel_out\n"
"geometry_msgs/Vector3 ang_vel_out\n"
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

  static const char* value(const ::position_control::des_acc_ang_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::position_control::des_acc_ang_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.accel_out);
      stream.next(m.ang_vel_out);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct des_acc_ang_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::position_control::des_acc_ang_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::position_control::des_acc_ang_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "accel_out: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.accel_out);
    s << indent << "ang_vel_out: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.ang_vel_out);
  }
};

} // namespace message_operations
} // namespace ros

#endif // POSITION_CONTROL_MESSAGE_DES_ACC_ANG_H