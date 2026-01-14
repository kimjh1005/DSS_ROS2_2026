// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from livox_lio_sam:srv/Reset.idl
// generated code does not contain a copyright notice

#ifndef LIVOX_LIO_SAM__SRV__DETAIL__RESET__BUILDER_HPP_
#define LIVOX_LIO_SAM__SRV__DETAIL__RESET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "livox_lio_sam/srv/detail/reset__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace livox_lio_sam
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::livox_lio_sam::srv::Reset_Request>()
{
  return ::livox_lio_sam::srv::Reset_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace livox_lio_sam


namespace livox_lio_sam
{

namespace srv
{

namespace builder
{

class Init_Reset_Response_success
{
public:
  Init_Reset_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::livox_lio_sam::srv::Reset_Response success(::livox_lio_sam::srv::Reset_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::livox_lio_sam::srv::Reset_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::livox_lio_sam::srv::Reset_Response>()
{
  return livox_lio_sam::srv::builder::Init_Reset_Response_success();
}

}  // namespace livox_lio_sam

#endif  // LIVOX_LIO_SAM__SRV__DETAIL__RESET__BUILDER_HPP_
