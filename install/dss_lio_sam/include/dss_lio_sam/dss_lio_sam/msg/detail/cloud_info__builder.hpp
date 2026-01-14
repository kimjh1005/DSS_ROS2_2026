// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dss_lio_sam:msg/CloudInfo.idl
// generated code does not contain a copyright notice

#ifndef DSS_LIO_SAM__MSG__DETAIL__CLOUD_INFO__BUILDER_HPP_
#define DSS_LIO_SAM__MSG__DETAIL__CLOUD_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dss_lio_sam/msg/detail/cloud_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dss_lio_sam
{

namespace msg
{

namespace builder
{

class Init_CloudInfo_cloud_surface
{
public:
  explicit Init_CloudInfo_cloud_surface(::dss_lio_sam::msg::CloudInfo & msg)
  : msg_(msg)
  {}
  ::dss_lio_sam::msg::CloudInfo cloud_surface(::dss_lio_sam::msg::CloudInfo::_cloud_surface_type arg)
  {
    msg_.cloud_surface = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dss_lio_sam::msg::CloudInfo msg_;
};

class Init_CloudInfo_cloud_corner
{
public:
  explicit Init_CloudInfo_cloud_corner(::dss_lio_sam::msg::CloudInfo & msg)
  : msg_(msg)
  {}
  Init_CloudInfo_cloud_surface cloud_corner(::dss_lio_sam::msg::CloudInfo::_cloud_corner_type arg)
  {
    msg_.cloud_corner = std::move(arg);
    return Init_CloudInfo_cloud_surface(msg_);
  }

private:
  ::dss_lio_sam::msg::CloudInfo msg_;
};

class Init_CloudInfo_cloud_deskewed
{
public:
  explicit Init_CloudInfo_cloud_deskewed(::dss_lio_sam::msg::CloudInfo & msg)
  : msg_(msg)
  {}
  Init_CloudInfo_cloud_corner cloud_deskewed(::dss_lio_sam::msg::CloudInfo::_cloud_deskewed_type arg)
  {
    msg_.cloud_deskewed = std::move(arg);
    return Init_CloudInfo_cloud_corner(msg_);
  }

private:
  ::dss_lio_sam::msg::CloudInfo msg_;
};

class Init_CloudInfo_initial_guess_yaw
{
public:
  explicit Init_CloudInfo_initial_guess_yaw(::dss_lio_sam::msg::CloudInfo & msg)
  : msg_(msg)
  {}
  Init_CloudInfo_cloud_deskewed initial_guess_yaw(::dss_lio_sam::msg::CloudInfo::_initial_guess_yaw_type arg)
  {
    msg_.initial_guess_yaw = std::move(arg);
    return Init_CloudInfo_cloud_deskewed(msg_);
  }

private:
  ::dss_lio_sam::msg::CloudInfo msg_;
};

class Init_CloudInfo_initial_guess_pitch
{
public:
  explicit Init_CloudInfo_initial_guess_pitch(::dss_lio_sam::msg::CloudInfo & msg)
  : msg_(msg)
  {}
  Init_CloudInfo_initial_guess_yaw initial_guess_pitch(::dss_lio_sam::msg::CloudInfo::_initial_guess_pitch_type arg)
  {
    msg_.initial_guess_pitch = std::move(arg);
    return Init_CloudInfo_initial_guess_yaw(msg_);
  }

private:
  ::dss_lio_sam::msg::CloudInfo msg_;
};

class Init_CloudInfo_initial_guess_roll
{
public:
  explicit Init_CloudInfo_initial_guess_roll(::dss_lio_sam::msg::CloudInfo & msg)
  : msg_(msg)
  {}
  Init_CloudInfo_initial_guess_pitch initial_guess_roll(::dss_lio_sam::msg::CloudInfo::_initial_guess_roll_type arg)
  {
    msg_.initial_guess_roll = std::move(arg);
    return Init_CloudInfo_initial_guess_pitch(msg_);
  }

private:
  ::dss_lio_sam::msg::CloudInfo msg_;
};

class Init_CloudInfo_initial_guess_z
{
public:
  explicit Init_CloudInfo_initial_guess_z(::dss_lio_sam::msg::CloudInfo & msg)
  : msg_(msg)
  {}
  Init_CloudInfo_initial_guess_roll initial_guess_z(::dss_lio_sam::msg::CloudInfo::_initial_guess_z_type arg)
  {
    msg_.initial_guess_z = std::move(arg);
    return Init_CloudInfo_initial_guess_roll(msg_);
  }

private:
  ::dss_lio_sam::msg::CloudInfo msg_;
};

class Init_CloudInfo_initial_guess_y
{
public:
  explicit Init_CloudInfo_initial_guess_y(::dss_lio_sam::msg::CloudInfo & msg)
  : msg_(msg)
  {}
  Init_CloudInfo_initial_guess_z initial_guess_y(::dss_lio_sam::msg::CloudInfo::_initial_guess_y_type arg)
  {
    msg_.initial_guess_y = std::move(arg);
    return Init_CloudInfo_initial_guess_z(msg_);
  }

private:
  ::dss_lio_sam::msg::CloudInfo msg_;
};

class Init_CloudInfo_initial_guess_x
{
public:
  explicit Init_CloudInfo_initial_guess_x(::dss_lio_sam::msg::CloudInfo & msg)
  : msg_(msg)
  {}
  Init_CloudInfo_initial_guess_y initial_guess_x(::dss_lio_sam::msg::CloudInfo::_initial_guess_x_type arg)
  {
    msg_.initial_guess_x = std::move(arg);
    return Init_CloudInfo_initial_guess_y(msg_);
  }

private:
  ::dss_lio_sam::msg::CloudInfo msg_;
};

class Init_CloudInfo_imu_yaw_init
{
public:
  explicit Init_CloudInfo_imu_yaw_init(::dss_lio_sam::msg::CloudInfo & msg)
  : msg_(msg)
  {}
  Init_CloudInfo_initial_guess_x imu_yaw_init(::dss_lio_sam::msg::CloudInfo::_imu_yaw_init_type arg)
  {
    msg_.imu_yaw_init = std::move(arg);
    return Init_CloudInfo_initial_guess_x(msg_);
  }

private:
  ::dss_lio_sam::msg::CloudInfo msg_;
};

class Init_CloudInfo_imu_pitch_init
{
public:
  explicit Init_CloudInfo_imu_pitch_init(::dss_lio_sam::msg::CloudInfo & msg)
  : msg_(msg)
  {}
  Init_CloudInfo_imu_yaw_init imu_pitch_init(::dss_lio_sam::msg::CloudInfo::_imu_pitch_init_type arg)
  {
    msg_.imu_pitch_init = std::move(arg);
    return Init_CloudInfo_imu_yaw_init(msg_);
  }

private:
  ::dss_lio_sam::msg::CloudInfo msg_;
};

class Init_CloudInfo_imu_roll_init
{
public:
  explicit Init_CloudInfo_imu_roll_init(::dss_lio_sam::msg::CloudInfo & msg)
  : msg_(msg)
  {}
  Init_CloudInfo_imu_pitch_init imu_roll_init(::dss_lio_sam::msg::CloudInfo::_imu_roll_init_type arg)
  {
    msg_.imu_roll_init = std::move(arg);
    return Init_CloudInfo_imu_pitch_init(msg_);
  }

private:
  ::dss_lio_sam::msg::CloudInfo msg_;
};

class Init_CloudInfo_odom_available
{
public:
  explicit Init_CloudInfo_odom_available(::dss_lio_sam::msg::CloudInfo & msg)
  : msg_(msg)
  {}
  Init_CloudInfo_imu_roll_init odom_available(::dss_lio_sam::msg::CloudInfo::_odom_available_type arg)
  {
    msg_.odom_available = std::move(arg);
    return Init_CloudInfo_imu_roll_init(msg_);
  }

private:
  ::dss_lio_sam::msg::CloudInfo msg_;
};

class Init_CloudInfo_imu_available
{
public:
  explicit Init_CloudInfo_imu_available(::dss_lio_sam::msg::CloudInfo & msg)
  : msg_(msg)
  {}
  Init_CloudInfo_odom_available imu_available(::dss_lio_sam::msg::CloudInfo::_imu_available_type arg)
  {
    msg_.imu_available = std::move(arg);
    return Init_CloudInfo_odom_available(msg_);
  }

private:
  ::dss_lio_sam::msg::CloudInfo msg_;
};

class Init_CloudInfo_point_range
{
public:
  explicit Init_CloudInfo_point_range(::dss_lio_sam::msg::CloudInfo & msg)
  : msg_(msg)
  {}
  Init_CloudInfo_imu_available point_range(::dss_lio_sam::msg::CloudInfo::_point_range_type arg)
  {
    msg_.point_range = std::move(arg);
    return Init_CloudInfo_imu_available(msg_);
  }

private:
  ::dss_lio_sam::msg::CloudInfo msg_;
};

class Init_CloudInfo_point_col_ind
{
public:
  explicit Init_CloudInfo_point_col_ind(::dss_lio_sam::msg::CloudInfo & msg)
  : msg_(msg)
  {}
  Init_CloudInfo_point_range point_col_ind(::dss_lio_sam::msg::CloudInfo::_point_col_ind_type arg)
  {
    msg_.point_col_ind = std::move(arg);
    return Init_CloudInfo_point_range(msg_);
  }

private:
  ::dss_lio_sam::msg::CloudInfo msg_;
};

class Init_CloudInfo_end_ring_index
{
public:
  explicit Init_CloudInfo_end_ring_index(::dss_lio_sam::msg::CloudInfo & msg)
  : msg_(msg)
  {}
  Init_CloudInfo_point_col_ind end_ring_index(::dss_lio_sam::msg::CloudInfo::_end_ring_index_type arg)
  {
    msg_.end_ring_index = std::move(arg);
    return Init_CloudInfo_point_col_ind(msg_);
  }

private:
  ::dss_lio_sam::msg::CloudInfo msg_;
};

class Init_CloudInfo_start_ring_index
{
public:
  explicit Init_CloudInfo_start_ring_index(::dss_lio_sam::msg::CloudInfo & msg)
  : msg_(msg)
  {}
  Init_CloudInfo_end_ring_index start_ring_index(::dss_lio_sam::msg::CloudInfo::_start_ring_index_type arg)
  {
    msg_.start_ring_index = std::move(arg);
    return Init_CloudInfo_end_ring_index(msg_);
  }

private:
  ::dss_lio_sam::msg::CloudInfo msg_;
};

class Init_CloudInfo_header
{
public:
  Init_CloudInfo_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CloudInfo_start_ring_index header(::dss_lio_sam::msg::CloudInfo::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_CloudInfo_start_ring_index(msg_);
  }

private:
  ::dss_lio_sam::msg::CloudInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dss_lio_sam::msg::CloudInfo>()
{
  return dss_lio_sam::msg::builder::Init_CloudInfo_header();
}

}  // namespace dss_lio_sam

#endif  // DSS_LIO_SAM__MSG__DETAIL__CLOUD_INFO__BUILDER_HPP_
