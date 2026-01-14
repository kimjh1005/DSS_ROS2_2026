// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from dss_lio_sam:msg/CloudInfo.idl
// generated code does not contain a copyright notice

#ifndef DSS_LIO_SAM__MSG__DETAIL__CLOUD_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define DSS_LIO_SAM__MSG__DETAIL__CLOUD_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "dss_lio_sam/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "dss_lio_sam/msg/detail/cloud_info__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace dss_lio_sam
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dss_lio_sam
cdr_serialize(
  const dss_lio_sam::msg::CloudInfo & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dss_lio_sam
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  dss_lio_sam::msg::CloudInfo & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dss_lio_sam
get_serialized_size(
  const dss_lio_sam::msg::CloudInfo & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dss_lio_sam
max_serialized_size_CloudInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace dss_lio_sam

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dss_lio_sam
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dss_lio_sam, msg, CloudInfo)();

#ifdef __cplusplus
}
#endif

#endif  // DSS_LIO_SAM__MSG__DETAIL__CLOUD_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
