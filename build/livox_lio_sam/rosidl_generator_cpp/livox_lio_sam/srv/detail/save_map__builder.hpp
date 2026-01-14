// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from livox_lio_sam:srv/SaveMap.idl
// generated code does not contain a copyright notice

#ifndef LIVOX_LIO_SAM__SRV__DETAIL__SAVE_MAP__BUILDER_HPP_
#define LIVOX_LIO_SAM__SRV__DETAIL__SAVE_MAP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "livox_lio_sam/srv/detail/save_map__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace livox_lio_sam
{

namespace srv
{

namespace builder
{

class Init_SaveMap_Request_destination
{
public:
  explicit Init_SaveMap_Request_destination(::livox_lio_sam::srv::SaveMap_Request & msg)
  : msg_(msg)
  {}
  ::livox_lio_sam::srv::SaveMap_Request destination(::livox_lio_sam::srv::SaveMap_Request::_destination_type arg)
  {
    msg_.destination = std::move(arg);
    return std::move(msg_);
  }

private:
  ::livox_lio_sam::srv::SaveMap_Request msg_;
};

class Init_SaveMap_Request_resolution
{
public:
  Init_SaveMap_Request_resolution()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SaveMap_Request_destination resolution(::livox_lio_sam::srv::SaveMap_Request::_resolution_type arg)
  {
    msg_.resolution = std::move(arg);
    return Init_SaveMap_Request_destination(msg_);
  }

private:
  ::livox_lio_sam::srv::SaveMap_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::livox_lio_sam::srv::SaveMap_Request>()
{
  return livox_lio_sam::srv::builder::Init_SaveMap_Request_resolution();
}

}  // namespace livox_lio_sam


namespace livox_lio_sam
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
  ::livox_lio_sam::srv::SaveMap_Response success(::livox_lio_sam::srv::SaveMap_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::livox_lio_sam::srv::SaveMap_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::livox_lio_sam::srv::SaveMap_Response>()
{
  return livox_lio_sam::srv::builder::Init_SaveMap_Response_success();
}

}  // namespace livox_lio_sam

#endif  // LIVOX_LIO_SAM__SRV__DETAIL__SAVE_MAP__BUILDER_HPP_
