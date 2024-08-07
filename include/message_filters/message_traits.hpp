// Copyright 2009, Willow Garage, Inc. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// File imported from
// https://github.com/ros/roscpp_core/blob/38b9663/roscpp_traits/include/ros/message_traits.h

#ifndef MESSAGE_FILTERS__MESSAGE_TRAITS_HPP_
#define MESSAGE_FILTERS__MESSAGE_TRAITS_HPP_

#include <string>
#include <type_traits>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

namespace message_filters
{
namespace message_traits
{

/**
 * False if the message does not have a header
 * @tparam M
 */
template<typename M, typename = void>
struct HasHeader : public std::false_type {};

/**
 * True if the message has a field named 'header' with a type of std_msgs::msg::Header
 * @tparam M
 */
template<typename M>
struct HasHeader<M, typename std::enable_if<std::is_same<std_msgs::msg::Header,
  decltype(M().header)>::value>::type>: public std::true_type {};

/**
 * \brief FrameId trait.  In the default implementation pointer()
 * returns &m.header.frame_id if HasHeader<M>::value is true, otherwise returns NULL.  value()
 * does not exist, and causes a compile error
 */
template<typename M, typename Enable = void>
struct FrameId
{
  static std::string * pointer(M & m) {(void)m; return nullptr;}
  static std::string const * pointer(const M & m) {(void)m; return nullptr;}
};
template<typename M>
struct FrameId<M, typename std::enable_if<HasHeader<M>::value>::type>
{
  static std::string * pointer(M & m) {return &m.header.frame_id;}
  static std::string const * pointer(const M & m) {return &m.header.frame_id;}
  static std::string value(const M & m) {return m.header.frame_id;}
};

/**
 * \brief TimeStamp trait.  In the default implementation pointer()
 * returns &m.header.stamp if HasHeader<M>::value is true, otherwise returns NULL.  value()
 * does not exist, and causes a compile error
 */
template<typename M, typename Enable = void>
struct TimeStamp
{
  static rclcpp::Time value(const M & m)
  {
    (void)m;
    return rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
};

template<typename M>
struct TimeStamp<M, typename std::enable_if<HasHeader<M>::value>::type>
{
  static rclcpp::Time value(const M & m)
  {
    return rclcpp::Time(m.header.stamp, RCL_ROS_TIME);
  }
};

}  // namespace message_traits
}  // namespace message_filters

#endif  // MESSAGE_FILTERS__MESSAGE_TRAITS_HPP_
