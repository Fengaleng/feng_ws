// Generated by gencpp from file mavros_msgs/SetMavFrameRequest.msg
// DO NOT EDIT!


#ifndef MAVROS_MSGS_MESSAGE_SETMAVFRAMEREQUEST_H
#define MAVROS_MSGS_MESSAGE_SETMAVFRAMEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace mavros_msgs
{
template <class ContainerAllocator>
struct SetMavFrameRequest_
{
  typedef SetMavFrameRequest_<ContainerAllocator> Type;

  SetMavFrameRequest_()
    : mav_frame(0)  {
    }
  SetMavFrameRequest_(const ContainerAllocator& _alloc)
    : mav_frame(0)  {
  (void)_alloc;
    }



   typedef uint8_t _mav_frame_type;
  _mav_frame_type mav_frame;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(FRAME_GLOBAL)
  #undef FRAME_GLOBAL
#endif
#if defined(_WIN32) && defined(FRAME_LOCAL_NED)
  #undef FRAME_LOCAL_NED
#endif
#if defined(_WIN32) && defined(FRAME_MISSION)
  #undef FRAME_MISSION
#endif
#if defined(_WIN32) && defined(FRAME_GLOBAL_RELATIVE_ALT)
  #undef FRAME_GLOBAL_RELATIVE_ALT
#endif
#if defined(_WIN32) && defined(FRAME_LOCAL_ENU)
  #undef FRAME_LOCAL_ENU
#endif
#if defined(_WIN32) && defined(FRAME_GLOBAL_INT)
  #undef FRAME_GLOBAL_INT
#endif
#if defined(_WIN32) && defined(FRAME_GLOBAL_RELATIVE_ALT_INT)
  #undef FRAME_GLOBAL_RELATIVE_ALT_INT
#endif
#if defined(_WIN32) && defined(FRAME_LOCAL_OFFSET_NED)
  #undef FRAME_LOCAL_OFFSET_NED
#endif
#if defined(_WIN32) && defined(FRAME_BODY_NED)
  #undef FRAME_BODY_NED
#endif
#if defined(_WIN32) && defined(FRAME_BODY_OFFSET_NED)
  #undef FRAME_BODY_OFFSET_NED
#endif
#if defined(_WIN32) && defined(FRAME_GLOBAL_TERRAIN_ALT)
  #undef FRAME_GLOBAL_TERRAIN_ALT
#endif
#if defined(_WIN32) && defined(FRAME_GLOBAL_TERRAIN_ALT_INT)
  #undef FRAME_GLOBAL_TERRAIN_ALT_INT
#endif
#if defined(_WIN32) && defined(FRAME_BODY_FRD)
  #undef FRAME_BODY_FRD
#endif
#if defined(_WIN32) && defined(FRAME_BODY_FLU)
  #undef FRAME_BODY_FLU
#endif
#if defined(_WIN32) && defined(FRAME_MOCAP_NED)
  #undef FRAME_MOCAP_NED
#endif
#if defined(_WIN32) && defined(FRAME_MOCAP_ENU)
  #undef FRAME_MOCAP_ENU
#endif
#if defined(_WIN32) && defined(FRAME_VISION_NED)
  #undef FRAME_VISION_NED
#endif
#if defined(_WIN32) && defined(FRAME_VISION_ENU)
  #undef FRAME_VISION_ENU
#endif
#if defined(_WIN32) && defined(FRAME_ESTIM_NED)
  #undef FRAME_ESTIM_NED
#endif
#if defined(_WIN32) && defined(FRAME_ESTIM_ENU)
  #undef FRAME_ESTIM_ENU
#endif

  enum {
    FRAME_GLOBAL = 0u,
    FRAME_LOCAL_NED = 1u,
    FRAME_MISSION = 2u,
    FRAME_GLOBAL_RELATIVE_ALT = 3u,
    FRAME_LOCAL_ENU = 4u,
    FRAME_GLOBAL_INT = 5u,
    FRAME_GLOBAL_RELATIVE_ALT_INT = 6u,
    FRAME_LOCAL_OFFSET_NED = 7u,
    FRAME_BODY_NED = 8u,
    FRAME_BODY_OFFSET_NED = 9u,
    FRAME_GLOBAL_TERRAIN_ALT = 10u,
    FRAME_GLOBAL_TERRAIN_ALT_INT = 11u,
    FRAME_BODY_FRD = 12u,
    FRAME_BODY_FLU = 13u,
    FRAME_MOCAP_NED = 14u,
    FRAME_MOCAP_ENU = 15u,
    FRAME_VISION_NED = 16u,
    FRAME_VISION_ENU = 17u,
    FRAME_ESTIM_NED = 18u,
    FRAME_ESTIM_ENU = 19u,
  };


  typedef boost::shared_ptr< ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetMavFrameRequest_

typedef ::mavros_msgs::SetMavFrameRequest_<std::allocator<void> > SetMavFrameRequest;

typedef boost::shared_ptr< ::mavros_msgs::SetMavFrameRequest > SetMavFrameRequestPtr;
typedef boost::shared_ptr< ::mavros_msgs::SetMavFrameRequest const> SetMavFrameRequestConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator1> & lhs, const ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator2> & rhs)
{
  return lhs.mav_frame == rhs.mav_frame;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator1> & lhs, const ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mavros_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4d2cf24886f660cde0f73cf6fc86e24c";
  }

  static const char* value(const ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4d2cf24886f660cdULL;
  static const uint64_t static_value2 = 0xe0f73cf6fc86e24cULL;
};

template<class ContainerAllocator>
struct DataType< ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mavros_msgs/SetMavFrameRequest";
  }

  static const char* value(const ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"uint8 FRAME_GLOBAL = 0\n"
"uint8 FRAME_LOCAL_NED = 1\n"
"uint8 FRAME_MISSION = 2\n"
"uint8 FRAME_GLOBAL_RELATIVE_ALT = 3\n"
"uint8 FRAME_LOCAL_ENU = 4\n"
"uint8 FRAME_GLOBAL_INT = 5\n"
"uint8 FRAME_GLOBAL_RELATIVE_ALT_INT = 6\n"
"uint8 FRAME_LOCAL_OFFSET_NED = 7\n"
"uint8 FRAME_BODY_NED = 8\n"
"uint8 FRAME_BODY_OFFSET_NED = 9\n"
"uint8 FRAME_GLOBAL_TERRAIN_ALT = 10\n"
"uint8 FRAME_GLOBAL_TERRAIN_ALT_INT = 11\n"
"uint8 FRAME_BODY_FRD = 12\n"
"uint8 FRAME_BODY_FLU = 13\n"
"uint8 FRAME_MOCAP_NED = 14\n"
"uint8 FRAME_MOCAP_ENU = 15\n"
"uint8 FRAME_VISION_NED = 16\n"
"uint8 FRAME_VISION_ENU = 17\n"
"uint8 FRAME_ESTIM_NED = 18\n"
"uint8 FRAME_ESTIM_ENU = 19\n"
"\n"
"\n"
"uint8 mav_frame\n"
;
  }

  static const char* value(const ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.mav_frame);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetMavFrameRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mavros_msgs::SetMavFrameRequest_<ContainerAllocator>& v)
  {
    s << indent << "mav_frame: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.mav_frame);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MAVROS_MSGS_MESSAGE_SETMAVFRAMEREQUEST_H
