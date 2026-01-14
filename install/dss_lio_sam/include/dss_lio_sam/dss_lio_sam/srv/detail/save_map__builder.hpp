// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dss_lio_sam:srv/SaveMap.idl
// generated code does not contain a copyright notice

#ifndef DSS_LIO_SAM__SRV__DETAIL__SAVE_MAP__BUILDER_HPP_
#define DSS_LIO_SAM__SRV__DETAIL__SAVE_MAP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dss_lio_sam/srv/detail/save_map__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dss_lio_sam
{

namespace srv
{

namespace builder
{

class Init_SaveMap_Request_destination
{
public:
  explicit Init_SaveMap_Request_destination(::dss_lio_sam::srv::SaveMap_Request & msg)
  : msg_(msg)
  {}
  ::dss_lio_sam::srv::SaveMap_Request destination(::dss_lio_sam::srv::SaveMap_Request::_destination_type arg)
  {
    msg_.destination = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dss_lio_sam::srv::SaveMap_Request msg_;
};

class Init_SaveMap_Request_resolution
{
public:
  Init_SaveMap_Request_resolution()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SaveMap_Request_destination resolution(::dss_lio_sam::srv::SaveMap_Request::_resolution_type arg)
  {
    msg_.resolution = std::move(arg);
    return Init_SaveMap_Request_destination(msg_);
  }

private:
  ::dss_lio_sam::srv::SaveMap_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dss_lio_sam::srv::SaveMap_Request>()
{
  return dss_lio_sam::srv::builder::Init_SaveMap_Request_resolution();
}

}  // namespace dss_lio_sam


namespace dss_lio_sam
{

namespace srv
{

namespace builder
{

class Init_SaveMap_Response_success
{
public:
  Init_SaveMap_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dss_lio_sam::srv::SaveMap_Response success(::dss_lio_sam::srv::SaveMap_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dss_lio_sam::srv::SaveMap_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dss_lio_sam::srv::SaveMap_Response>()
{
  return dss_lio_sam::srv::builder::Init_SaveMap_Response_success();
}

}  // namespace dss_lio_sam

#endif  // DSS_LIO_SAM__SRV__DETAIL__SAVE_MAP__BUILDER_HPP_
