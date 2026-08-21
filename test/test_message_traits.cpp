// Copyright 2022, Kenji Brameld All rights reserved.
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

#include <gtest/gtest.h>

#include "message_filters/message_traits.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/header.hpp"

namespace
{

struct Msg
{
  std_msgs::msg::Header header;
};

struct HeaderlessMsg
{
  int32_t seconds;
  int32_t nanoseconds;
};

struct SpecializedMsg
{
  int32_t seconds;
  int32_t nanoseconds;
};
}  // namespace

namespace message_filters
{
namespace message_traits
{
// The historical customization point: a full TimeStamp specialization.
template<>
struct TimeStamp<SpecializedMsg>
{
  static rclcpp::Time value(const SpecializedMsg & m)
  {
    return rclcpp::Time(m.seconds, m.nanoseconds, RCL_ROS_TIME);
  }
};
}  // namespace message_traits
}  // namespace message_filters

namespace
{

template<typename MessageType>
struct TimeGetterNoHeaderCustom
{
  static rclcpp::Time getTime(const MessageType & message)
  {
    return rclcpp::Time(message.seconds, message.nanoseconds, RCL_ROS_TIME);
  }
};

template<typename MessageType>
struct TimeGetterZeroTimestampCustom
{
  static rclcpp::Time getTime(const MessageType & message)
  {
    (void)message;
    return rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
};

template<typename MessageType>
struct NotATimeGetter {};

// The TimeGetterFor concept accepts valid getters and rejects everything else.
static_assert(
  message_filters::message_traits::TimeGetterFor<
    message_filters::message_traits::DefaultTimeGetter, Msg>);
static_assert(
  message_filters::message_traits::TimeGetterFor<TimeGetterNoHeaderCustom, HeaderlessMsg>);
static_assert(
  !message_filters::message_traits::TimeGetterFor<NotATimeGetter, Msg>);

// Test that message_filters::message_traits::TimeStamp<Msg>::value returns RCL_ROS_TIME.
TEST(MessageTraits, timeSource)
{
  Msg msg;
  rclcpp::Time time = message_filters::message_traits::TimeStamp<Msg>::value(msg);

  EXPECT_EQ(time.get_clock_type(), RCL_ROS_TIME);

  // Ensure an exception isn't thrown when compared with a RCL_ROS_TIME time.
  bool unused;
  EXPECT_NO_THROW(unused = (time == rclcpp::Time{msg.header.stamp, RCL_ROS_TIME}));
  (void)unused;
}

TEST(MessageTraits, defaultTimeGetterHasHeader)
{
  Msg msg;
  msg.header.stamp.sec = 1;
  msg.header.stamp.nanosec = 2u;
  rclcpp::Time time = message_filters::message_traits::DefaultTimeGetter<Msg>::getTime(msg);

  EXPECT_EQ(time.get_clock_type(), RCL_ROS_TIME);
  EXPECT_EQ(time.nanoseconds(), 1000000002);

  msg.header.stamp.nanosec = 3u;
  time = message_filters::message_traits::DefaultTimeGetter<Msg>::getTime(msg);

  EXPECT_EQ(time.nanoseconds(), 1000000003);
}

TEST(MessageTraits, defaultTimeGetterNoHeader)
{
  HeaderlessMsg msg;
  msg.seconds = 1;
  msg.nanoseconds = 2;

  rclcpp::Time time =
    message_filters::message_traits::DefaultTimeGetter<HeaderlessMsg>::getTime(msg);

  EXPECT_EQ(time.get_clock_type(), RCL_ROS_TIME);
  EXPECT_EQ(time.nanoseconds(), 0);
}

TEST(MessageTraits, defaultTimeGetterHonorsTimeStampSpecialization)
{
  SpecializedMsg msg;
  msg.seconds = 1;
  msg.nanoseconds = 2;

  rclcpp::Time time =
    message_filters::message_traits::DefaultTimeGetter<SpecializedMsg>::getTime(msg);

  EXPECT_EQ(time.get_clock_type(), RCL_ROS_TIME);
  EXPECT_EQ(time.nanoseconds(), 1000000002);
}

TEST(MessageTraits, customTimeGetterOverridesHeader)
{
  Msg msg;
  msg.header.stamp.sec = 1;
  msg.header.stamp.nanosec = 2u;

  rclcpp::Time time = TimeGetterZeroTimestampCustom<Msg>::getTime(msg);

  EXPECT_EQ(time.get_clock_type(), RCL_ROS_TIME);
  EXPECT_EQ(time.nanoseconds(), 0);
}

TEST(MessageTraits, customTimeGetterHeaderless)
{
  HeaderlessMsg msg;
  msg.seconds = 1;
  msg.nanoseconds = 2;

  rclcpp::Time time = TimeGetterNoHeaderCustom<HeaderlessMsg>::getTime(msg);

  EXPECT_EQ(time.get_clock_type(), RCL_ROS_TIME);
  EXPECT_EQ(time.nanoseconds(), 1000000002);

  msg.nanoseconds = 3;
  time = TimeGetterNoHeaderCustom<HeaderlessMsg>::getTime(msg);

  EXPECT_EQ(time.nanoseconds(), 1000000003);
}
}   // namespace
