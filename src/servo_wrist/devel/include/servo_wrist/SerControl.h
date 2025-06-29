// Generated by gencpp from file servo_wrist/SerControl.msg
// DO NOT EDIT!


#ifndef SERVO_WRIST_MESSAGE_SERCONTROL_H
#define SERVO_WRIST_MESSAGE_SERCONTROL_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace servo_wrist
{
template <class ContainerAllocator>
struct SerControl_
{
  typedef SerControl_<ContainerAllocator> Type;

  SerControl_()
    : servo_id(0)
    , target_position(0)
    , velocity(0)
    , acceleration(0)  {
    }
  SerControl_(const ContainerAllocator& _alloc)
    : servo_id(0)
    , target_position(0)
    , velocity(0)
    , acceleration(0)  {
  (void)_alloc;
    }



   typedef int32_t _servo_id_type;
  _servo_id_type servo_id;

   typedef int32_t _target_position_type;
  _target_position_type target_position;

   typedef int32_t _velocity_type;
  _velocity_type velocity;

   typedef int32_t _acceleration_type;
  _acceleration_type acceleration;





  typedef boost::shared_ptr< ::servo_wrist::SerControl_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::servo_wrist::SerControl_<ContainerAllocator> const> ConstPtr;

}; // struct SerControl_

typedef ::servo_wrist::SerControl_<std::allocator<void> > SerControl;

typedef boost::shared_ptr< ::servo_wrist::SerControl > SerControlPtr;
typedef boost::shared_ptr< ::servo_wrist::SerControl const> SerControlConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::servo_wrist::SerControl_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::servo_wrist::SerControl_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::servo_wrist::SerControl_<ContainerAllocator1> & lhs, const ::servo_wrist::SerControl_<ContainerAllocator2> & rhs)
{
  return lhs.servo_id == rhs.servo_id &&
    lhs.target_position == rhs.target_position &&
    lhs.velocity == rhs.velocity &&
    lhs.acceleration == rhs.acceleration;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::servo_wrist::SerControl_<ContainerAllocator1> & lhs, const ::servo_wrist::SerControl_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace servo_wrist

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::servo_wrist::SerControl_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::servo_wrist::SerControl_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::servo_wrist::SerControl_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::servo_wrist::SerControl_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::servo_wrist::SerControl_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::servo_wrist::SerControl_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::servo_wrist::SerControl_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3a9602bc216147f85b9a19097afea1cd";
  }

  static const char* value(const ::servo_wrist::SerControl_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3a9602bc216147f8ULL;
  static const uint64_t static_value2 = 0x5b9a19097afea1cdULL;
};

template<class ContainerAllocator>
struct DataType< ::servo_wrist::SerControl_<ContainerAllocator> >
{
  static const char* value()
  {
    return "servo_wrist/SerControl";
  }

  static const char* value(const ::servo_wrist::SerControl_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::servo_wrist::SerControl_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 servo_id\n"
"int32 target_position\n"
"int32 velocity\n"
"int32 acceleration\n"
"\n"
;
  }

  static const char* value(const ::servo_wrist::SerControl_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::servo_wrist::SerControl_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.servo_id);
      stream.next(m.target_position);
      stream.next(m.velocity);
      stream.next(m.acceleration);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SerControl_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::servo_wrist::SerControl_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::servo_wrist::SerControl_<ContainerAllocator>& v)
  {
    s << indent << "servo_id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.servo_id);
    s << indent << "target_position: ";
    Printer<int32_t>::stream(s, indent + "  ", v.target_position);
    s << indent << "velocity: ";
    Printer<int32_t>::stream(s, indent + "  ", v.velocity);
    s << indent << "acceleration: ";
    Printer<int32_t>::stream(s, indent + "  ", v.acceleration);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SERVO_WRIST_MESSAGE_SERCONTROL_H
