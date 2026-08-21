// Copyright 2026, Open Source Robotics Foundation, Inc. All rights reserved.
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

// End-to-end tests for the custom TimeGetter support: every filter must use
// the user-provided getter to extract timestamps from messages that carry
// their timestamp outside of a std_msgs Header.

#include <gtest/gtest.h>

#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "message_filters/cache.hpp"
#include "message_filters/input_aligner.hpp"
#include "message_filters/message_traits.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"
#include "message_filters/sync_policies/exact_time.hpp"
#include "message_filters/synchronizer.hpp"
#include "message_filters/time_sequencer.hpp"
#include "message_filters/time_synchronizer.hpp"

// A message that carries its timestamp in plain fields instead of a header.
struct Msg
{
  int32_t sec;
  uint32_t nanosec;
  int data;
};
using MsgConstPtr = std::shared_ptr<Msg const>;

template<typename M>
struct FieldTimeGetter
{
  static rclcpp::Time getTime(const M & m)
  {
    return rclcpp::Time(m.sec, m.nanosec, RCL_ROS_TIME);
  }
};

namespace
{

MsgConstPtr makeMsg(int32_t sec, int data)
{
  auto msg = std::make_shared<Msg>();
  msg->sec = sec;
  msg->nanosec = 0u;
  msg->data = data;
  return msg;
}

}  // namespace

class PairHelper
{
public:
  void cb(const MsgConstPtr & a, const MsgConstPtr & b)
  {
    pairs_.emplace_back(a, b);
  }

  std::vector<std::pair<MsgConstPtr, MsgConstPtr>> pairs_;
};

class OrderHelper
{
public:
  void cb(const MsgConstPtr & msg)
  {
    order_.push_back(msg->data);
  }

  std::vector<int> order_;
};

namespace
{

TEST(CustomTimeGetter, exactTimePolicy)
{
  using Policy = message_filters::sync_policies::ExactTimeBase<FieldTimeGetter, Msg, Msg>;
  message_filters::Synchronizer<Policy> sync(Policy(5));
  PairHelper h;
  sync.registerCallback(
    std::bind(&PairHelper::cb, &h, std::placeholders::_1, std::placeholders::_2));

  sync.add<0>(makeMsg(1, 10));
  sync.add<1>(makeMsg(2, 20));  // different custom stamp: no match
  EXPECT_EQ(h.pairs_.size(), 0u);

  sync.add<1>(makeMsg(1, 11));  // matches the first message's custom stamp
  ASSERT_EQ(h.pairs_.size(), 1u);
  EXPECT_EQ(h.pairs_[0].first->data, 10);
  EXPECT_EQ(h.pairs_[0].second->data, 11);
}

TEST(CustomTimeGetter, approximateTimePolicy)
{
  using Policy = message_filters::sync_policies::ApproximateTimeBase<FieldTimeGetter, Msg, Msg>;
  message_filters::Synchronizer<Policy> sync(Policy(5));
  PairHelper h;
  sync.registerCallback(
    std::bind(&PairHelper::cb, &h, std::placeholders::_1, std::placeholders::_2));

  // Three rounds of matching custom stamps: every pair must be emitted,
  // matched by the timestamps the custom getter extracted.
  for (int32_t sec = 1; sec <= 3; ++sec) {
    sync.add<0>(makeMsg(sec, sec * 100));
    sync.add<1>(makeMsg(sec, sec * 100 + 1));
  }
  ASSERT_EQ(h.pairs_.size(), 3u);
  for (size_t i = 0; i < h.pairs_.size(); i++) {
    const int expected = static_cast<int>(i + 1) * 100;
    EXPECT_EQ(h.pairs_[i].first->data, expected);
    EXPECT_EQ(h.pairs_[i].second->data, expected + 1);
  }
}

TEST(CustomTimeGetter, timeSynchronizer)
{
  message_filters::TimeSynchronizerBase<FieldTimeGetter, Msg, Msg> sync(5);
  PairHelper h;
  sync.registerCallback(
    std::bind(&PairHelper::cb, &h, std::placeholders::_1, std::placeholders::_2));

  sync.add<0>(makeMsg(1, 10));
  sync.add<1>(makeMsg(1, 11));
  ASSERT_EQ(h.pairs_.size(), 1u);
  EXPECT_EQ(h.pairs_[0].first->data, 10);
  EXPECT_EQ(h.pairs_[0].second->data, 11);
}

TEST(CustomTimeGetter, cache)
{
  // Must not throw despite the message being headerless: the custom getter
  // provides the timestamp, so allow_headerless is not required.
  message_filters::Cache<Msg, FieldTimeGetter> cache(10);

  cache.add(makeMsg(10, 1));
  cache.add(makeMsg(30, 3));
  cache.add(makeMsg(20, 2));  // out of order: must be sorted by custom stamp

  const auto all = cache.getSurroundingInterval(
    rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Time(100, 0, RCL_ROS_TIME));
  ASSERT_EQ(all.size(), 3u);
  EXPECT_EQ(all[0]->data, 1);
  EXPECT_EQ(all[1]->data, 2);
  EXPECT_EQ(all[2]->data, 3);

  const auto interval = cache.getInterval(
    rclcpp::Time(15, 0, RCL_ROS_TIME), rclcpp::Time(35, 0, RCL_ROS_TIME));
  ASSERT_EQ(interval.size(), 2u);
  EXPECT_EQ(interval[0]->data, 2);
  EXPECT_EQ(interval[1]->data, 3);
}

TEST(CustomTimeGetter, inputAligner)
{
  message_filters::InputAlignerBase<FieldTimeGetter, Msg, Msg> aligner(
    rclcpp::Duration(100, 0));
  OrderHelper h;
  aligner.registerCallback<0>(&OrderHelper::cb, &h);
  aligner.registerCallback<1>(&OrderHelper::cb, &h);
  aligner.setInputPeriod<0>(rclcpp::Duration(2, 0));
  aligner.setInputPeriod<1>(rclcpp::Duration(2, 0));

  // Feed messages out of order across the two inputs; the aligner must
  // dispatch them ordered by the custom timestamps.
  aligner.add<0>(makeMsg(3, 3));
  aligner.add<0>(makeMsg(1, 1));
  aligner.add<1>(makeMsg(2, 2));
  aligner.add<1>(makeMsg(4, 4));
  aligner.add<0>(makeMsg(5, 5));
  aligner.dispatchMessages();

  ASSERT_EQ(h.order_.size(), 5u);
  for (size_t i = 0; i < h.order_.size(); i++) {
    EXPECT_EQ(h.order_[i], static_cast<int>(i) + 1);
  }
}

TEST(CustomTimeGetter, timeSequencerCompilation)
{
  auto node = std::make_shared<rclcpp::Node>("test_custom_time_getter_node");
  message_filters::TimeSequencer<Msg, FieldTimeGetter> seq(
    rclcpp::Duration(1, 0), rclcpp::Duration(0, 10000000), 10, node);
  seq.add(makeMsg(1, 1));
}

}   // namespace
