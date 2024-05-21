/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2022, Kenji Brameld
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <gtest/gtest.h>

#include "message_filters/message_traits.h"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/header.hpp"

struct Msg
{
  std_msgs::msg::Header header;
};

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
