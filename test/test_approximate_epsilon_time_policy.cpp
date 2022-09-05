
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Willow Garage, Inc.
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

#include <vector>


#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_epsilon_time.h"
#include "message_filters/message_traits.h"

using namespace std::placeholders;
using namespace message_filters;
using namespace message_filters::sync_policies;

struct Header
{
  rclcpp::Time stamp;
};


struct Msg
{
  Header header;
  int data;
};
using MsgPtr = std::shared_ptr<Msg>;
using MsgConstPtr = std::shared_ptr<const Msg>;

namespace message_filters
{
namespace message_traits
{
template<>
struct TimeStamp<Msg>
{
  static rclcpp::Time value(const Msg& m)
  {
    return m.header.stamp;
  }
};
}
}

typedef std::pair<rclcpp::Time, rclcpp::Time> TimePair;
typedef std::pair<rclcpp::Time, unsigned int> TimeAndTopic;
struct TimeQuad
{
  TimeQuad(rclcpp::Time p, rclcpp::Time q, rclcpp::Time r, rclcpp::Time s)
  {
    time[0] = p;
    time[1] = q;
    time[2] = r;
    time[3] = s;
  }
  rclcpp::Time time[4];
};


class ApproximateEpsilonTimeSynchronizerTest
{
public:

  ApproximateEpsilonTimeSynchronizerTest(const std::vector<TimeAndTopic> &input,
				  const std::vector<TimePair> &output,
				  uint32_t queue_size, rclcpp::Duration epsilon) :
    input_(input), output_(output), output_position_(0), sync_(
      ApproximateEpsilonTime<Msg, Msg>{queue_size, epsilon})
  {
    sync_.registerCallback(std::bind(&ApproximateEpsilonTimeSynchronizerTest::callback, this, _1, _2));
  }

  void callback(const MsgConstPtr& p, const MsgConstPtr& q)
  {
    ASSERT_TRUE(p);
    ASSERT_TRUE(q);
    ASSERT_LT(output_position_, output_.size());
    EXPECT_EQ(output_[output_position_].first, p->header.stamp);
    EXPECT_EQ(output_[output_position_].second, q->header.stamp);
    ++output_position_;
  }

  void run()
  {
    for (unsigned int i = 0; i < input_.size(); i++)
    {
      if (input_[i].second == 0)
      {
        MsgPtr p(std::make_shared<Msg>());
        p->header.stamp = input_[i].first;
        sync_.add<0>(p);
      }
      else
      {
        MsgPtr q(std::make_shared<Msg>());
        q->header.stamp = input_[i].first;
        sync_.add<1>(q);
      }
    }
    //printf("Done running test\n");
    EXPECT_EQ(output_.size(), output_position_);
  }

private:
  const std::vector<TimeAndTopic> &input_;
  const std::vector<TimePair> &output_;
  unsigned int output_position_;
  typedef Synchronizer<ApproximateEpsilonTime<Msg, Msg> > Sync2;
public:
  Sync2 sync_;
};

TEST(ApproxTimeSync, ExactMatch) {
  // Input A:  a..b..c
  // Input B:  A..B..C
  // Output:   a..b..c
  //           A..B..C
  std::vector<TimeAndTopic> input;
  std::vector<TimePair> output;

  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration s(1, 0);

  input.push_back(TimeAndTopic(t,0));     // a
  input.push_back(TimeAndTopic(t,1));   // A
  input.push_back(TimeAndTopic(t+s*3,0)); // b
  input.push_back(TimeAndTopic(t+s*3,1)); // B
  input.push_back(TimeAndTopic(t+s*6,0)); // c
  input.push_back(TimeAndTopic(t+s*6,1)); // C
  output.push_back(TimePair(t, t));
  output.push_back(TimePair(t+s*3, t+s*3));
  output.push_back(TimePair(t+s*6, t+s*6));

  ApproximateEpsilonTimeSynchronizerTest sync_test(
    input, output, 10, rclcpp::Duration::from_seconds(0.5));
  sync_test.run();
}


TEST(ApproxTimeSync, PerfectMatch) {
  // Input A:  a..b..c.
  // Input B:  .A..B..C
  // Output:   ...a..b.c
  //           ...A..B.C
  std::vector<TimeAndTopic> input;
  std::vector<TimePair> output;

  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration s(1, 0);

  input.push_back(TimeAndTopic(t,0));     // a
  input.push_back(TimeAndTopic(t+s,1));   // A
  input.push_back(TimeAndTopic(t+s*3,0)); // b
  input.push_back(TimeAndTopic(t+s*4,1)); // B
  input.push_back(TimeAndTopic(t+s*6,0)); // c
  input.push_back(TimeAndTopic(t+s*7,1)); // C
  output.push_back(TimePair(t, t+s));
  output.push_back(TimePair(t+s*3, t+s*4));
  output.push_back(TimePair(t+s*6, t+s*7));

  ApproximateEpsilonTimeSynchronizerTest sync_test(
    input, output, 10, rclcpp::Duration::from_seconds(1.5));
  sync_test.run();
}


TEST(ApproxTimeSync, ImperfectMatch) {
  // Input A:  a.xb..c.
  // Input B:  .A...B.C
  // Output:   ..a...b.c
  //           ..A...B.C
  std::vector<TimeAndTopic> input;
  std::vector<TimePair> output;

  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration s(1, 0);

  input.push_back(TimeAndTopic(t,0));     // a
  input.push_back(TimeAndTopic(t+s,1));   // A
  input.push_back(TimeAndTopic(t+s*2,0)); // x
  input.push_back(TimeAndTopic(t+s*3,0)); // b
  input.push_back(TimeAndTopic(t+s*4,1)); // B
  input.push_back(TimeAndTopic(t+s*6,0)); // c
  input.push_back(TimeAndTopic(t+s*7,1)); // C
  output.push_back(TimePair(t, t+s));
  output.push_back(TimePair(t+s*3, t+s*4));
  output.push_back(TimePair(t+s*6, t+s*7));

  ApproximateEpsilonTimeSynchronizerTest sync_test(
    input, output, 10, rclcpp::Duration::from_seconds(1.5));
  sync_test.run();
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
