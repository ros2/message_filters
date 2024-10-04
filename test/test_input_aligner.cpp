// Copyright 2024, Kraken Robotics. All rights reserved.
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

#include <array>
#include <chrono>
#include <cstddef>
#include <memory>
#include <type_traits>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "message_filters/input_aligner.hpp"

class InputAligner;

struct Header
{
  rclcpp::Time stamp;
};

struct Msg1
{
  Header header;
  int data;
};

struct Msg2
{
  Header header;
  int data;
};

namespace message_filters
{
namespace message_traits
{
template<typename M>
struct HasHeader<M, typename std::enable_if<std::is_same<Header,
  decltype(M().header)>::value>::type>: public std::true_type {};
}  // namespace message_traits
}  // namespace message_filters

class InputAlignerTest : public ::testing::Test
{
public:
  InputAlignerTest()
  : timeout_(1, 0), update_rate_(0, 1000000)
  {
  }

  virtual void SetUp()
  {
    // Shutdown in case there was a dangling global context from other test fixtures
    rclcpp::shutdown();
    rclcpp::init(0, nullptr);

    node_ = std::make_shared<rclcpp::Node>("test_node");
    timeout_ = rclcpp::Duration(1, 0);
    update_rate_ = rclcpp::Duration(0, 10000000);
  }

  void TearDown()
  {
    node_.reset();
    rclcpp::shutdown();
  }

  template<typename MsgType>
  void cb(const std::shared_ptr<const MsgType> & msg)
  {
    cb_content_.push_back(msg->data);
  }

  template<typename MsgType>
  std::shared_ptr<const MsgType> createMsg(const std::chrono::nanoseconds & ns, int data)
  {
    auto msg = std::make_shared<MsgType>();
    msg->header.stamp = rclcpp::Time(ns.count(), RCL_ROS_TIME);
    msg->data = data;
    return msg;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Duration timeout_;
  rclcpp::Duration update_rate_;
  std::vector<int> cb_content_;
};

TEST_F(InputAlignerTest, init)
{
  message_filters::NullFilter<Msg1> f0, f1;
  message_filters::NullFilter<Msg2> f2, f3;
  message_filters::InputAligner<Msg1, Msg1, Msg2, Msg2> aligner1(timeout_, f0, f1, f2, f3);

  message_filters::InputAligner<Msg1, Msg2, Msg2> aligner2(timeout_);
  aligner2.connectInput(f0, f2, f3);
}


TEST_F(InputAlignerTest, dispatch_inputs_in_order)
{
  message_filters::InputAligner<Msg1, Msg2, Msg1, Msg2> aligner(timeout_);

  // register callbacks
  InputAlignerTest * test_fixture = dynamic_cast<InputAlignerTest *>(this);
  aligner.registerCallback<0>(&InputAlignerTest::cb<Msg1>, test_fixture);
  aligner.registerCallback<1>(&InputAlignerTest::cb<Msg2>, test_fixture);
  aligner.registerCallback<2>(&InputAlignerTest::cb<Msg1>, test_fixture);
  aligner.registerCallback<3>(&InputAlignerTest::cb<Msg2>, test_fixture);

  // set period for all inputs to one millisecond
  aligner.setInputPeriod<0>(rclcpp::Duration(0, 4e6));
  aligner.setInputPeriod<1>(rclcpp::Duration(0, 4e6));
  aligner.setInputPeriod<2>(rclcpp::Duration(0, 4e6));
  aligner.setInputPeriod<3>(rclcpp::Duration(0, 4e6));

  // send msgs in unaligned order
  aligner.add<2>(createMsg<Msg1>(std::chrono::milliseconds(3), 3));
  aligner.add<0>(createMsg<Msg1>(std::chrono::milliseconds(1), 1));
  aligner.add<2>(createMsg<Msg1>(std::chrono::milliseconds(7), 7));
  aligner.add<0>(createMsg<Msg1>(std::chrono::milliseconds(5), 5));
  aligner.add<3>(createMsg<Msg2>(std::chrono::milliseconds(2), 2));
  aligner.add<0>(createMsg<Msg1>(std::chrono::milliseconds(9), 9));
  aligner.add<1>(createMsg<Msg2>(std::chrono::milliseconds(4), 4));
  aligner.add<1>(createMsg<Msg2>(std::chrono::milliseconds(8), 8));
  aligner.add<3>(createMsg<Msg2>(std::chrono::milliseconds(6), 6));

  // dispatch messages
  aligner.dispatchMessages();

  // compare msg output
  ASSERT_EQ(cb_content_.size(), 9);
  for (size_t i = 0; i < cb_content_.size(); i++) {
    EXPECT_EQ(cb_content_[i], i + 1);
  }
}

TEST_F(InputAlignerTest, ignores_inactive_inputs)
{
  message_filters::InputAligner<Msg1, Msg2, Msg1> aligner(timeout_);

  // register callbacks
  InputAlignerTest * test_fixture = dynamic_cast<InputAlignerTest *>(this);
  aligner.registerCallback<0>(&InputAlignerTest::cb<Msg1>, test_fixture);
  aligner.registerCallback<1>(&InputAlignerTest::cb<Msg2>, test_fixture);
  aligner.registerCallback<2>(&InputAlignerTest::cb<Msg1>, test_fixture);

  // set period for all inputs to one millisecond
  aligner.setInputPeriod<0>(rclcpp::Duration(0, 2e6));
  aligner.setInputPeriod<1>(rclcpp::Duration(0, 2e6));
  aligner.setInputPeriod<2>(rclcpp::Duration(0, 2e6));

  // send msgs in unaligned order only to two inputs
  aligner.add<2>(createMsg<Msg1>(std::chrono::milliseconds(2), 2));
  aligner.add<1>(createMsg<Msg2>(std::chrono::milliseconds(1), 1));
  aligner.add<2>(createMsg<Msg1>(std::chrono::milliseconds(4), 4));
  aligner.add<1>(createMsg<Msg2>(std::chrono::milliseconds(3), 3));
  aligner.add<1>(createMsg<Msg2>(std::chrono::milliseconds(5), 5));

  // dispatch messages
  aligner.dispatchMessages();

  // compare msg output
  ASSERT_EQ(cb_content_.size(), 5);
  for (size_t i = 0; i < cb_content_.size(); i++) {
    EXPECT_EQ(cb_content_[i], i + 1);
  }
}

TEST_F(InputAlignerTest, input_timeout)
{
  // set timeout of 10 ms
  timeout_ = rclcpp::Duration(0, 1e7);
  message_filters::InputAligner<Msg1, Msg2> aligner(timeout_);

  // register callbacks
  InputAlignerTest * test_fixture = dynamic_cast<InputAlignerTest *>(this);
  aligner.registerCallback<0>(&InputAlignerTest::cb<Msg1>, test_fixture);
  aligner.registerCallback<1>(&InputAlignerTest::cb<Msg2>, test_fixture);

  // set period for all inputs to 2 milliseconds
  aligner.setInputPeriod<0>(rclcpp::Duration(0, 2e6));
  aligner.setInputPeriod<1>(rclcpp::Duration(0, 2e6));

  // adds 8 msgs on input 0
  for (int i = 1; i < 17; i = i + 2) {
    aligner.add<0>(createMsg<Msg1>(std::chrono::milliseconds(i), i));
  }
  // adds 2 msgs on input 1
  aligner.add<1>(createMsg<Msg2>(std::chrono::milliseconds(2), 2));
  aligner.add<1>(createMsg<Msg2>(std::chrono::milliseconds(4), 4));

  // dispatch messages
  aligner.dispatchMessages();

  // compare msg output
  // only msgs < 6 ms are forwarded, it will wait for msgs on input 1
  ASSERT_EQ(cb_content_.size(), 5);
  for (size_t i = 0; i < cb_content_.size(); i++) {
    EXPECT_EQ(cb_content_[i], i + 1);
  }

  // a new msg with a time stamp > 10 ms from the
  // last forwarded one will trigger the timeout for input 1
  aligner.add<0>(createMsg<Msg1>(std::chrono::milliseconds(17), 17));

  // dispatch messages
  aligner.dispatchMessages();
  ASSERT_EQ(cb_content_.size(), 11);
  EXPECT_EQ(cb_content_[8], 13);
  EXPECT_EQ(cb_content_[9], 15);
  EXPECT_EQ(cb_content_[10], 17);
}

TEST_F(InputAlignerTest, drops_msgs)
{
  message_filters::InputAligner<Msg1, Msg2> aligner(timeout_);

  // register callbacks
  InputAlignerTest * test_fixture = dynamic_cast<InputAlignerTest *>(this);
  aligner.registerCallback<0>(&InputAlignerTest::cb<Msg1>, test_fixture);
  aligner.registerCallback<1>(&InputAlignerTest::cb<Msg2>, test_fixture);

  // set period for all inputs to 2 milliseconds
  aligner.setInputPeriod<0>(rclcpp::Duration(0, 2e6));
  aligner.setInputPeriod<1>(rclcpp::Duration(0, 2e6));


  aligner.add<1>(createMsg<Msg2>(std::chrono::milliseconds(4), 4));
  aligner.add<0>(createMsg<Msg1>(std::chrono::milliseconds(3), 3));

  // dispatch messages
  aligner.dispatchMessages();

  ASSERT_EQ(cb_content_.size(), 2);
  for (size_t i = 0; i < cb_content_.size(); i++) {
    EXPECT_EQ(cb_content_[i], i + 3);
  }

  // add also msg backwards in time
  aligner.add<0>(createMsg<Msg1>(std::chrono::milliseconds(1), 1));
  aligner.add<0>(createMsg<Msg1>(std::chrono::milliseconds(5), 5));
  aligner.add<0>(createMsg<Msg1>(std::chrono::milliseconds(7), 7));
  aligner.add<1>(createMsg<Msg2>(std::chrono::milliseconds(2), 2));
  aligner.add<1>(createMsg<Msg2>(std::chrono::milliseconds(6), 6));

  // dispatch messages
  aligner.dispatchMessages();

  ASSERT_EQ(cb_content_.size(), 5);
  for (size_t i = 0; i < cb_content_.size(); i++) {
    EXPECT_EQ(cb_content_[i], i + 3);
  }
}

TEST_F(InputAlignerTest, dispatch_by_timer)
{
  message_filters::InputAligner<Msg1, Msg2> aligner(timeout_);
  aligner.setupDispatchTimer(node_, update_rate_);

  // register callbacks
  InputAlignerTest * test_fixture = dynamic_cast<InputAlignerTest *>(this);
  aligner.registerCallback<0>(&InputAlignerTest::cb<Msg1>, test_fixture);
  aligner.registerCallback<1>(&InputAlignerTest::cb<Msg2>, test_fixture);

  // set period for all inputs to 2 milliseconds
  aligner.setInputPeriod<0>(rclcpp::Duration(0, 2e6));
  aligner.setInputPeriod<1>(rclcpp::Duration(0, 2e6));

  aligner.add<1>(createMsg<Msg2>(std::chrono::milliseconds(2), 2));
  aligner.add<0>(createMsg<Msg1>(std::chrono::milliseconds(1), 1));

  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(node_);

  ASSERT_EQ(cb_content_.size(), 2);
  for (size_t i = 0; i < cb_content_.size(); i++) {
    EXPECT_EQ(cb_content_[i], i + 1);
  }
}

TEST_F(InputAlignerTest, no_period_information)
{
  // set timeout of 10 ms
  timeout_ = rclcpp::Duration(0, 1e7);
  message_filters::InputAligner<Msg1, Msg2, Msg1> aligner(timeout_);

  // register callbacks
  InputAlignerTest * test_fixture = dynamic_cast<InputAlignerTest *>(this);
  aligner.registerCallback<0>(&InputAlignerTest::cb<Msg1>, test_fixture);
  aligner.registerCallback<1>(&InputAlignerTest::cb<Msg2>, test_fixture);
  aligner.registerCallback<2>(&InputAlignerTest::cb<Msg1>, test_fixture);

  // send msgs in unaligned order only to two inputs
  aligner.add<0>(createMsg<Msg1>(std::chrono::milliseconds(6), 6));
  aligner.add<2>(createMsg<Msg1>(std::chrono::milliseconds(2), 2));
  aligner.add<2>(createMsg<Msg1>(std::chrono::milliseconds(4), 4));
  aligner.add<1>(createMsg<Msg2>(std::chrono::milliseconds(1), 1));
  aligner.add<1>(createMsg<Msg2>(std::chrono::milliseconds(3), 3));
  aligner.add<1>(createMsg<Msg2>(std::chrono::milliseconds(5), 5));

  // dispatch messages
  aligner.dispatchMessages();

  // the last two messages won't get dispatched
  ASSERT_EQ(cb_content_.size(), 4);
  for (size_t i = 0; i < cb_content_.size(); i++) {
    EXPECT_EQ(cb_content_[i], i + 1);
  }

  // trigger timeout on all other inputs to flush msgs
  aligner.add<0>(createMsg<Msg1>(std::chrono::milliseconds(16), 16));

  // dispatch messages
  aligner.dispatchMessages();
  ASSERT_EQ(cb_content_.size(), 7);
}

TEST_F(InputAlignerTest, get_queue_status)
{
  // set timeout of 10 ms
  timeout_ = rclcpp::Duration(0, 1e7);
  message_filters::InputAligner<Msg1, Msg2> aligner(timeout_);

  // register callbacks
  InputAlignerTest * test_fixture = dynamic_cast<InputAlignerTest *>(this);
  aligner.registerCallback<0>(&InputAlignerTest::cb<Msg1>, test_fixture);
  aligner.registerCallback<1>(&InputAlignerTest::cb<Msg2>, test_fixture);

  // set period for all inputs to 2 milliseconds
  aligner.setInputPeriod<0>(rclcpp::Duration(0, 2e6));
  aligner.setInputPeriod<1>(rclcpp::Duration(0, 2e6));

  aligner.add<1>(createMsg<Msg2>(std::chrono::milliseconds(2), 2));
  aligner.add<0>(createMsg<Msg1>(std::chrono::milliseconds(3), 3));
  aligner.add<0>(createMsg<Msg1>(std::chrono::milliseconds(5), 5));

  auto status_0 = aligner.getQueueStatus<0>();
  EXPECT_FALSE(status_0.active);
  EXPECT_EQ(status_0.queue_size, 2);
  EXPECT_EQ(status_0.msgs_processed, 0);
  EXPECT_EQ(status_0.msgs_dropped, 0);
  auto status_1 = aligner.getQueueStatus<1>();
  EXPECT_FALSE(status_1.active);
  EXPECT_EQ(status_1.queue_size, 1);
  EXPECT_EQ(status_1.msgs_processed, 0);
  EXPECT_EQ(status_1.msgs_dropped, 0);

  // dispatch messages
  aligner.dispatchMessages();

  status_0 = aligner.getQueueStatus<0>();
  EXPECT_TRUE(status_0.active);
  EXPECT_EQ(status_0.queue_size, 1);
  EXPECT_EQ(status_0.msgs_processed, 1);
  EXPECT_EQ(status_0.msgs_dropped, 0);
  status_1 = aligner.getQueueStatus<1>();
  EXPECT_TRUE(status_1.active);
  EXPECT_EQ(status_1.queue_size, 0);
  EXPECT_EQ(status_1.msgs_processed, 1);
  EXPECT_EQ(status_1.msgs_dropped, 0);

  // adds sample outside of filter scope
  aligner.add<0>(createMsg<Msg1>(std::chrono::milliseconds(1), 1));
  // adds a sample ahead forcing input 1 to time out
  aligner.add<0>(createMsg<Msg1>(std::chrono::milliseconds(17), 17));

  // dispatch messages
  aligner.dispatchMessages();

  status_0 = aligner.getQueueStatus<0>();
  EXPECT_TRUE(status_0.active);
  EXPECT_EQ(status_0.queue_size, 0);
  EXPECT_EQ(status_0.msgs_processed, 3);
  EXPECT_EQ(status_0.msgs_dropped, 1);
  status_1 = aligner.getQueueStatus<1>();
  EXPECT_FALSE(status_1.active);
  EXPECT_EQ(status_1.queue_size, 0);
  EXPECT_EQ(status_1.msgs_processed, 1);
  EXPECT_EQ(status_1.msgs_dropped, 0);
}
