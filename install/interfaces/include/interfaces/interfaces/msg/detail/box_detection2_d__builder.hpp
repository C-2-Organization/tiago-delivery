// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/BoxDetection2D.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__BOX_DETECTION2_D__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__BOX_DETECTION2_D__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/box_detection2_d__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_BoxDetection2D_y2
{
public:
  explicit Init_BoxDetection2D_y2(::interfaces::msg::BoxDetection2D & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::BoxDetection2D y2(::interfaces::msg::BoxDetection2D::_y2_type arg)
  {
    msg_.y2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::BoxDetection2D msg_;
};

class Init_BoxDetection2D_x2
{
public:
  explicit Init_BoxDetection2D_x2(::interfaces::msg::BoxDetection2D & msg)
  : msg_(msg)
  {}
  Init_BoxDetection2D_y2 x2(::interfaces::msg::BoxDetection2D::_x2_type arg)
  {
    msg_.x2 = std::move(arg);
    return Init_BoxDetection2D_y2(msg_);
  }

private:
  ::interfaces::msg::BoxDetection2D msg_;
};

class Init_BoxDetection2D_y1
{
public:
  explicit Init_BoxDetection2D_y1(::interfaces::msg::BoxDetection2D & msg)
  : msg_(msg)
  {}
  Init_BoxDetection2D_x2 y1(::interfaces::msg::BoxDetection2D::_y1_type arg)
  {
    msg_.y1 = std::move(arg);
    return Init_BoxDetection2D_x2(msg_);
  }

private:
  ::interfaces::msg::BoxDetection2D msg_;
};

class Init_BoxDetection2D_x1
{
public:
  explicit Init_BoxDetection2D_x1(::interfaces::msg::BoxDetection2D & msg)
  : msg_(msg)
  {}
  Init_BoxDetection2D_y1 x1(::interfaces::msg::BoxDetection2D::_x1_type arg)
  {
    msg_.x1 = std::move(arg);
    return Init_BoxDetection2D_y1(msg_);
  }

private:
  ::interfaces::msg::BoxDetection2D msg_;
};

class Init_BoxDetection2D_confidence
{
public:
  explicit Init_BoxDetection2D_confidence(::interfaces::msg::BoxDetection2D & msg)
  : msg_(msg)
  {}
  Init_BoxDetection2D_x1 confidence(::interfaces::msg::BoxDetection2D::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_BoxDetection2D_x1(msg_);
  }

private:
  ::interfaces::msg::BoxDetection2D msg_;
};

class Init_BoxDetection2D_label
{
public:
  explicit Init_BoxDetection2D_label(::interfaces::msg::BoxDetection2D & msg)
  : msg_(msg)
  {}
  Init_BoxDetection2D_confidence label(::interfaces::msg::BoxDetection2D::_label_type arg)
  {
    msg_.label = std::move(arg);
    return Init_BoxDetection2D_confidence(msg_);
  }

private:
  ::interfaces::msg::BoxDetection2D msg_;
};

class Init_BoxDetection2D_source
{
public:
  explicit Init_BoxDetection2D_source(::interfaces::msg::BoxDetection2D & msg)
  : msg_(msg)
  {}
  Init_BoxDetection2D_label source(::interfaces::msg::BoxDetection2D::_source_type arg)
  {
    msg_.source = std::move(arg);
    return Init_BoxDetection2D_label(msg_);
  }

private:
  ::interfaces::msg::BoxDetection2D msg_;
};

class Init_BoxDetection2D_header
{
public:
  Init_BoxDetection2D_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BoxDetection2D_source header(::interfaces::msg::BoxDetection2D::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_BoxDetection2D_source(msg_);
  }

private:
  ::interfaces::msg::BoxDetection2D msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::BoxDetection2D>()
{
  return interfaces::msg::builder::Init_BoxDetection2D_header();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__BOX_DETECTION2_D__BUILDER_HPP_
