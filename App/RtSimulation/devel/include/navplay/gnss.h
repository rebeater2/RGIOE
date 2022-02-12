// Generated by gencpp from file navplay/gnss.msg
// DO NOT EDIT!


#ifndef NAVPLAY_MESSAGE_GNSS_H
#define NAVPLAY_MESSAGE_GNSS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace navplay
{
template <class ContainerAllocator>
struct gnss_
{
  typedef gnss_<ContainerAllocator> Type;

  gnss_()
    : gpst(0.0)
    , lat(0.0)
    , lon(0.0)
    , h(0.0)
    , pos_std()
    , mode(0)
    , ns(0)  {
      pos_std.assign(0.0);
  }
  gnss_(const ContainerAllocator& _alloc)
    : gpst(0.0)
    , lat(0.0)
    , lon(0.0)
    , h(0.0)
    , pos_std()
    , mode(0)
    , ns(0)  {
  (void)_alloc;
      pos_std.assign(0.0);
  }



   typedef double _gpst_type;
  _gpst_type gpst;

   typedef double _lat_type;
  _lat_type lat;

   typedef double _lon_type;
  _lon_type lon;

   typedef float _h_type;
  _h_type h;

   typedef boost::array<float, 3>  _pos_std_type;
  _pos_std_type pos_std;

   typedef uint8_t _mode_type;
  _mode_type mode;

   typedef uint8_t _ns_type;
  _ns_type ns;





  typedef boost::shared_ptr< ::navplay::gnss_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::navplay::gnss_<ContainerAllocator> const> ConstPtr;

}; // struct gnss_

typedef ::navplay::gnss_<std::allocator<void> > gnss;

typedef boost::shared_ptr< ::navplay::gnss > gnssPtr;
typedef boost::shared_ptr< ::navplay::gnss const> gnssConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::navplay::gnss_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::navplay::gnss_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::navplay::gnss_<ContainerAllocator1> & lhs, const ::navplay::gnss_<ContainerAllocator2> & rhs)
{
  return lhs.gpst == rhs.gpst &&
    lhs.lat == rhs.lat &&
    lhs.lon == rhs.lon &&
    lhs.h == rhs.h &&
    lhs.pos_std == rhs.pos_std &&
    lhs.mode == rhs.mode &&
    lhs.ns == rhs.ns;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::navplay::gnss_<ContainerAllocator1> & lhs, const ::navplay::gnss_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace navplay

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::navplay::gnss_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::navplay::gnss_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::navplay::gnss_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::navplay::gnss_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::navplay::gnss_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::navplay::gnss_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::navplay::gnss_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d8b3bc143a1901908ef5f7a6d985bd0b";
  }

  static const char* value(const ::navplay::gnss_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd8b3bc143a190190ULL;
  static const uint64_t static_value2 = 0x8ef5f7a6d985bd0bULL;
};

template<class ContainerAllocator>
struct DataType< ::navplay::gnss_<ContainerAllocator> >
{
  static const char* value()
  {
    return "navplay/gnss";
  }

  static const char* value(const ::navplay::gnss_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::navplay::gnss_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 gpst\n"
"float64 lat\n"
"float64 lon\n"
"float32 h\n"
"float32[3] pos_std\n"
"uint8 mode\n"
"uint8 ns\n"
;
  }

  static const char* value(const ::navplay::gnss_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::navplay::gnss_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.gpst);
      stream.next(m.lat);
      stream.next(m.lon);
      stream.next(m.h);
      stream.next(m.pos_std);
      stream.next(m.mode);
      stream.next(m.ns);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct gnss_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::navplay::gnss_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::navplay::gnss_<ContainerAllocator>& v)
  {
    s << indent << "gpst: ";
    Printer<double>::stream(s, indent + "  ", v.gpst);
    s << indent << "lat: ";
    Printer<double>::stream(s, indent + "  ", v.lat);
    s << indent << "lon: ";
    Printer<double>::stream(s, indent + "  ", v.lon);
    s << indent << "h: ";
    Printer<float>::stream(s, indent + "  ", v.h);
    s << indent << "pos_std[]" << std::endl;
    for (size_t i = 0; i < v.pos_std.size(); ++i)
    {
      s << indent << "  pos_std[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.pos_std[i]);
    }
    s << indent << "mode: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.mode);
    s << indent << "ns: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.ns);
  }
};

} // namespace message_operations
} // namespace ros

#endif // NAVPLAY_MESSAGE_GNSS_H
