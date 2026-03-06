// Copyright 2008, Willow Garage, Inc. All rights reserved.
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

#include <chrono>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "message_filters/time_sequencer.hpp"

struct Header
{
  rclcpp::Time stamp;
};

struct Msg
{
  Header header;
  int data;
};
typedef std::shared_ptr<Msg> MsgPtr;
typedef std::shared_ptr<Msg const> MsgConstPtr;

namespace message_filters
{
namespace message_traits
{
template<>
struct TimeStamp<Msg>
{
  static rclcpp::Time value(const Msg & m)
  {
    return rclcpp::Time(m.header.stamp, RCL_ROS_TIME);
  }
};
}  // namespace message_traits
}  // namespace message_filters

class Helper
{
public:
  Helper()
  : count_(0)
  {}

  void cb(const MsgConstPtr & msg)
  {
    ++count_;
    received_.push_back(msg);
  }

  int32_t count_;
  std::vector<MsgConstPtr> received_;
};

// Spin the executor until predicate returns true or timeout_ms elapses.
template<typename Pred>
static bool spinUntil(
  rclcpp::executors::SingleThreadedExecutor & executor,
  Pred pred,
  int timeout_ms = 3000)
{
  using clock = std::chrono::steady_clock;
  auto deadline = clock::now() + std::chrono::milliseconds(timeout_ms);
  while (!pred() && clock::now() < deadline) {
    executor.spin_some();
    rclcpp::Rate(100).sleep();
  }
  return pred();
}

TEST(TimeSequencer, simple)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("test_node");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  message_filters::TimeSequencer<Msg> seq(rclcpp::Duration(0, 250000000),
    rclcpp::Duration(0, 10000000), 10, node);
  Helper h;
  seq.registerCallback(std::bind(&Helper::cb, &h, std::placeholders::_1));
  MsgPtr msg(std::make_shared<Msg>());
  msg->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  seq.add(msg);

  rclcpp::Rate(10).sleep();
  executor.spin_some();
  ASSERT_EQ(h.count_, 0);

  // Must be longer than the first duration above
  rclcpp::Rate(3).sleep();
  executor.spin_some();

  ASSERT_EQ(h.count_, 1);
}

TEST(TimeSequencer, compilation)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("test_node");
  message_filters::TimeSequencer<Msg> seq(rclcpp::Duration(1, 0), rclcpp::Duration(0, 10000000), 10,
    node);
  message_filters::TimeSequencer<Msg> seq2(rclcpp::Duration(1, 0), rclcpp::Duration(0, 10000000),
    10,
    node);
  seq2.connectInput(seq);
}

struct EventHelper
{
public:
  void cb(const message_filters::MessageEvent<Msg const> & evt)
  {
    event_ = evt;
  }

  message_filters::MessageEvent<Msg const> event_;
};

TEST(TimeSequencer, eventInEventOut)
{
  rclcpp::Node::SharedPtr nh = std::make_shared<rclcpp::Node>("test_node");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(nh);
  message_filters::TimeSequencer<Msg> seq(rclcpp::Duration(1, 0), rclcpp::Duration(0, 10000000), 10,
    nh);
  message_filters::TimeSequencer<Msg> seq2(seq, rclcpp::Duration(1, 0), rclcpp::Duration(
      0,
      10000000),
    10, nh);
  EventHelper h;
  seq2.registerCallback(&EventHelper::cb, &h);

  message_filters::MessageEvent<Msg const> evt(std::make_shared<Msg const>(),
    rclcpp::Clock(RCL_ROS_TIME).now());
  seq.add(evt);

  while (!h.event_.getMessage()) {
    rclcpp::Rate(100).sleep();
    executor.spin_some();
  }

  EXPECT_EQ(h.event_.getReceiptTime(), evt.getReceiptTime());
  EXPECT_EQ(h.event_.getMessage(), evt.getMessage());
}

// Messages added out-of-order must be dispatched in ascending timestamp order.
TEST(TimeSequencer, outOfOrderDeliveredInOrder)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("test_node");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  const rclcpp::Duration delay(0, 250000000);   // 250 ms
  const rclcpp::Duration rate(0, 10000000);     // 10 ms
  message_filters::TimeSequencer<Msg> seq(delay, rate, 10, node);

  Helper h;
  seq.registerCallback(std::bind(&Helper::cb, &h, std::placeholders::_1));

  rclcpp::Time base = rclcpp::Clock(RCL_ROS_TIME).now();

  // Add messages with deliberately reversed timestamps
  auto msg_late = std::make_shared<Msg>();
  msg_late->header.stamp = base;
  msg_late->data = 1;

  auto msg_early = std::make_shared<Msg>();
  msg_early->header.stamp = base - rclcpp::Duration(0, 50000000);  // 50 ms earlier
  msg_early->data = 2;

  seq.add(msg_late);
  seq.add(msg_early);

  ASSERT_TRUE(spinUntil(executor, [&h]() {return h.count_ >= 2;}));
  ASSERT_EQ(h.count_, 2);
  // Earlier timestamp must have been delivered first
  EXPECT_EQ(h.received_[0]->data, 2);
  EXPECT_EQ(h.received_[1]->data, 1);
}

// A message whose timestamp is older than the last dispatched timestamp must be discarded.
TEST(TimeSequencer, oldMessageDiscarded)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("test_node");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  const rclcpp::Duration delay(0, 250000000);
  const rclcpp::Duration rate(0, 10000000);
  message_filters::TimeSequencer<Msg> seq(delay, rate, 10, node);

  Helper h;
  seq.registerCallback(std::bind(&Helper::cb, &h, std::placeholders::_1));

  rclcpp::Time base = rclcpp::Clock(RCL_ROS_TIME).now();

  // Add a message and wait for it to be dispatched
  auto msg1 = std::make_shared<Msg>();
  msg1->header.stamp = base;
  msg1->data = 10;
  seq.add(msg1);

  ASSERT_TRUE(spinUntil(executor, [&h]() {return h.count_ >= 1;}));
  ASSERT_EQ(h.count_, 1);

  // Now add a message with a timestamp before the one just dispatched
  auto msg_old = std::make_shared<Msg>();
  msg_old->header.stamp = base - rclcpp::Duration(0, 100000000);  // 100 ms before msg1
  msg_old->data = 99;
  seq.add(msg_old);

  // Spin for a full dispatch cycle; the old message must be silently dropped
  rclcpp::Rate(3).sleep();
  executor.spin_some();
  EXPECT_EQ(h.count_, 1);
}

// When queue_size is exceeded the oldest message in the queue is dropped.
TEST(TimeSequencer, queueSizeEviction)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("test_node");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Long delay so nothing dispatches during the test; small queue
  const rclcpp::Duration delay(10, 0);
  const rclcpp::Duration rate(0, 10000000);
  message_filters::TimeSequencer<Msg> seq(delay, rate, /*queue_size=*/ 2, node);

  Helper h;
  seq.registerCallback(std::bind(&Helper::cb, &h, std::placeholders::_1));

  rclcpp::Time base = rclcpp::Clock(RCL_ROS_TIME).now();

  // Fill the queue beyond capacity; the first (oldest) message should be evicted
  for (int i = 0; i < 3; ++i) {
    auto msg = std::make_shared<Msg>();
    msg->header.stamp = base + rclcpp::Duration(i, 0);
    msg->data = i;
    seq.add(msg);
  }

  // Nothing should have been dispatched (delay is 10 s)
  executor.spin_some();
  EXPECT_EQ(h.count_, 0);
}

// queue_size == 0 means unlimited; no messages should be evicted.
TEST(TimeSequencer, zeroQueueSizeIsUnlimited)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("test_node");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  const rclcpp::Duration delay(0, 250000000);
  const rclcpp::Duration rate(0, 10000000);
  message_filters::TimeSequencer<Msg> seq(delay, rate, /*queue_size=*/ 0, node);

  Helper h;
  seq.registerCallback(std::bind(&Helper::cb, &h, std::placeholders::_1));

  rclcpp::Time base = rclcpp::Clock(RCL_ROS_TIME).now();

  const int n = 5;
  for (int i = 0; i < n; ++i) {
    auto msg = std::make_shared<Msg>();
    msg->header.stamp = base - rclcpp::Duration(0, static_cast<uint32_t>(i) * 1000000u);
    msg->data = i;
    seq.add(msg);
  }

  ASSERT_TRUE(spinUntil(executor, [&h, n]() {return h.count_ >= n;}));

  // All n messages should arrive (none evicted)
  EXPECT_EQ(h.count_, n);
}

// Multiple messages past their delay should all be dispatched in a single tick.
TEST(TimeSequencer, multipleMsgsDispatchedInOneTick)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("test_node");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  const rclcpp::Duration delay(0, 250000000);
  const rclcpp::Duration rate(0, 10000000);
  message_filters::TimeSequencer<Msg> seq(delay, rate, 10, node);

  Helper h;
  seq.registerCallback(std::bind(&Helper::cb, &h, std::placeholders::_1));

  rclcpp::Time base = rclcpp::Clock(RCL_ROS_TIME).now();

  const int n = 3;
  for (int i = 0; i < n; ++i) {
    auto msg = std::make_shared<Msg>();
    // All timestamps are in the past relative to base, well beyond the delay
    msg->header.stamp = base - rclcpp::Duration(1, 0) + rclcpp::Duration(0,
      static_cast<uint32_t>(i) * 10000000u);
    msg->data = i;
    seq.add(msg);
  }

  ASSERT_TRUE(spinUntil(executor, [&h, n]() {return h.count_ >= n;}));
  EXPECT_EQ(h.count_, n);
}

// add(MsgConstPtr) overload should work identically to add(EventType).
TEST(TimeSequencer, addMsgPtrOverload)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("test_node");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  const rclcpp::Duration delay(0, 250000000);
  const rclcpp::Duration rate(0, 10000000);
  message_filters::TimeSequencer<Msg> seq(delay, rate, 10, node);

  Helper h;
  seq.registerCallback(std::bind(&Helper::cb, &h, std::placeholders::_1));

  MsgPtr msg = std::make_shared<Msg>();
  msg->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  seq.add(msg);  // uses the MConstPtr overload

  ASSERT_TRUE(spinUntil(executor, [&h]() {return h.count_ >= 1;}));
  EXPECT_EQ(h.count_, 1);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);

  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
