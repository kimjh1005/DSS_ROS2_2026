// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dss_lio_sam:srv/SaveMap.idl
// generated code does not contain a copyright notice

#ifndef DSS_LIO_SAM__SRV__DETAIL__SAVE_MAP__TRAITS_HPP_
#define DSS_LIO_SAM__SRV__DETAIL__SAVE_MAP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dss_lio_sam/srv/detail/save_map__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dss_lio_sam
{

namespace srv
{

inline void to_flow_style_yaml(
  const SaveMap_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: resolution
  {
    out << "resolution: ";
    rosidl_generator_traits::value_to_yaml(msg.resolution, out);
    out << ", ";
  }

  // member: destination
  {
    out << "destination: ";
    rosidl_generator_traits::value_to_yaml(msg.destination, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SaveMap_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: resolution
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "resolution: ";
    rosidl_generator_traits::value_to_yaml(msg.resolution, out);
    out << "\n";
  }

  // member: destination
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "destination: ";
    rosidl_generator_traits::value_to_yaml(msg.destination, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SaveMap_Request & msg, bool use_flow_style = false)
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

}  // namespace dss_lio_sam

namespace rosidl_generator_traits
{

[[deprecated("use dss_lio_sam::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dss_lio_sam::srv::SaveMap_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  dss_lio_sam::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dss_lio_sam::srv::to_yaml() instead")]]
inline std::string to_yaml(const dss_lio_sam::srv::SaveMap_Request & msg)
{
  return dss_lio_sam::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dss_lio_sam::srv::SaveMap_Request>()
{
  return "dss_lio_sam::srv::SaveMap_Request";
}

template<>
inline const char * name<dss_lio_sam::srv::SaveMap_Request>()
{
  return "dss_lio_sam/srv/SaveMap_Request";
}

template<>
struct has_fixed_size<dss_lio_sam::srv::SaveMap_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dss_lio_sam::srv::SaveMap_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dss_lio_sam::srv::SaveMap_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace dss_lio_sam
{

namespace srv
{

inline void to_flow_style_yaml(
  const SaveMap_Response & msg,
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
  const SaveMap_Response & msg,
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

inline std::string to_yaml(const SaveMap_Response & msg, bool use_flow_style = false)
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

}  // namespace dss_lio_sam

namespace rosidl_generator_traits
{

[[deprecated("use dss_lio_sam::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dss_lio_sam::srv::SaveMap_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  dss_lio_sam::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dss_lio_sam::srv::to_yaml() instead")]]
inline std::string to_yaml(const dss_lio_sam::srv::SaveMap_Response & msg)
{
  return dss_lio_sam::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dss_lio_sam::srv::SaveMap_Response>()
{
  return "dss_lio_sam::srv::SaveMap_Response";
}

template<>
inline const char * name<dss_lio_sam::srv::SaveMap_Response>()
{
  return "dss_lio_sam/srv/SaveMap_Response";
}

template<>
struct has_fixed_size<dss_lio_sam::srv::SaveMap_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<dss_lio_sam::srv::SaveMap_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<dss_lio_sam::srv::SaveMap_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dss_lio_sam::srv::SaveMap>()
{
  return "dss_lio_sam::srv::SaveMap";
}

template<>
inline const char * name<dss_lio_sam::srv::SaveMap>()
{
  return "dss_lio_sam/srv/SaveMap";
}

template<>
struct has_fixed_size<dss_lio_sam::srv::SaveMap>
  : std::integral_constant<
    bool,
    has_fixed_size<dss_lio_sam::srv::SaveMap_Request>::value &&
    has_fixed_size<dss_lio_sam::srv::SaveMap_Response>::value
  >
{
};

template<>
struct has_bounded_size<dss_lio_sam::srv::SaveMap>
  : std::integral_constant<
    bool,
    has_bounded_size<dss_lio_sam::srv::SaveMap_Request>::value &&
    has_bounded_size<dss_lio_sam::srv::SaveMap_Response>::value
  >
{
};

template<>
struct is_service<dss_lio_sam::srv::SaveMap>
  : std::true_type
{
};

template<>
struct is_service_request<dss_lio_sam::srv::SaveMap_Request>
  : std::true_type
{
};

template<>
struct is_service_response<dss_lio_sam::srv::SaveMap_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // DSS_LIO_SAM__SRV__DETAIL__SAVE_MAP__TRAITS_HPP_
