/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/time_source.hpp"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/latest_time.h"

#include "rosgraph_msgs/msg/clock.hpp"

using namespace std::chrono;

struct Header
{
  rclcpp::Time stamp;
};

struct Msg
{
  Header header;
  size_t data;
};
typedef std::shared_ptr<Msg> MsgPtr;
typedef std::shared_ptr<Msg const> MsgConstPtr;

class Helper
{
public:
  Helper()
  {
  }

  void cb(const MsgConstPtr & p, const MsgConstPtr & q, const MsgConstPtr & r)
  {
    EXPECT_TRUE(p);
    EXPECT_TRUE(q);
    EXPECT_TRUE(r);

    p_ = p; q_ = q; r_ = r;
    ++count_;
  }

  MsgConstPtr p_{nullptr}, q_{nullptr}, r_{nullptr};
  uint16_t count_{0U};
};

typedef message_filters::sync_policies::LatestTime<Msg, Msg, Msg> Policy3;
typedef message_filters::Synchronizer<Policy3> Sync3;

class LatestTimePolicy : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr node;
  rclcpp::SyncParametersClient::SharedPtr param_client;
  Sync3 sync;
  Helper h;
  std::vector<MsgPtr> p;
  std::vector<MsgPtr> q;
  std::vector<MsgPtr> r;

  virtual void SetUp()
  {
    // Shutdown in case there was a dangling global context from other test fixtures
    rclcpp::shutdown();
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("clock_sleep_node");
    param_client = std::make_shared<rclcpp::SyncParametersClient>(node);
    ASSERT_TRUE(param_client->wait_for_service(5s));

    sync.registerCallback(std::bind(&Helper::cb, &h, std::placeholders::_1,
                                                     std::placeholders::_2,
                                                     std::placeholders::_3));
    p.reserve(12U);
    q.reserve(6U);
    r.reserve(3U);
    for(std::size_t idx = 0U; idx < 12U; ++idx) {
      MsgPtr p_idx(std::make_shared<Msg>()); p_idx->data = idx; p.push_back(p_idx);
      if (idx % 2U == 0U) {
        MsgPtr q_idx(std::make_shared<Msg>()); q_idx->data = idx; q.push_back(q_idx);
      }
      if (idx % 4U == 0U) {
        MsgPtr r_idx(std::make_shared<Msg>()); r_idx->data = idx; r.push_back(r_idx);
      }
    }
  }

  void TearDown()
  {
    node.reset();
    rclcpp::shutdown();
  }
};


TEST_F(LatestTimePolicy, Leading)
{
  rclcpp::Rate rate(50.0);
  for (std::size_t idx = 0U; idx < 8U; ++idx) {
    if (idx % 2U == 0U) {
      sync.add<1>(q[idx / 2U]);
    }
    if (idx % 4U == 0U) {
      sync.add<2>(r[idx / 4U]);
    }
    sync.add<0>(p[idx]);

    EXPECT_EQ(h.count_, idx);
    if (idx > 0) {
      EXPECT_EQ(h.p_->data, p[idx]->data);
      EXPECT_EQ(h.q_->data, q[idx / 2U]->data);
      EXPECT_EQ(h.r_->data, r[idx / 4U]->data);
    } else {
      EXPECT_FALSE(h.p_);
      EXPECT_FALSE(h.q_);
      EXPECT_FALSE(h.r_);
    }

    rate.sleep();
  }
}

TEST_F(LatestTimePolicy, Trailing)
{
  rclcpp::Rate rate(50.0);
  for (std::size_t idx = 0U; idx < 8U; ++idx) {
    if (idx % 2U == 1U) {
      sync.add<1>(q[(idx - 1U) / 2U]);
    }
    if (idx % 4U == 3U) {
      sync.add<2>(r[(idx - 3U) / 4U]);
    }
    sync.add<0>(p[idx]);

    if (idx > 2U) {
      EXPECT_EQ(h.count_, idx - 2U);
      EXPECT_EQ(h.p_->data, p[idx]->data);
      EXPECT_EQ(h.q_->data, q[(idx - 1U) / 2U]->data);
      EXPECT_EQ(h.r_->data, r[(idx - 3U) / 4U]->data);
    } else {
      EXPECT_FALSE(h.p_);
      EXPECT_FALSE(h.q_);
      EXPECT_FALSE(h.r_);
    }

    rate.sleep();
  }
}

TEST_F(LatestTimePolicy, ChangeRateLeading)
{
  param_client->set_parameters({rclcpp::Parameter("use_sim_time", true)});
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto clock_publisher = node->create_publisher<rosgraph_msgs::msg::Clock>(
    "/clock", rclcpp::ClockQoS());
  rclcpp::TimeSource time_source;
  time_source.attachNode(node);
  time_source.attachClock(clock);
  sync.setClock(clock);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  for (std::size_t idx = 0U; idx < 12U; ++idx) {
    if (idx % 2U == 0U) {
      sync.add<1>(q[idx/2U]);
    }

    if (idx % 4U == 0U) {
      sync.add<2>(r[idx/4U]);
    }

    if (idx < 4U) {
      sync.add<0>(p[idx]);
    } else { // Change rate of p
      if (idx % 3U == 0U) {
        static std::size_t p_idx = 3U;
        sync.add<0>(p[++p_idx]);
      }
    }

    // operates like "Leading" test for idx <= 3
    if (idx >= 1U && idx < 4U) {
      EXPECT_EQ(h.count_, idx);
      EXPECT_EQ(h.p_->data, p[idx]->data);
      EXPECT_EQ(h.q_->data, q[idx / 2U]->data);
      EXPECT_EQ(h.r_->data, r[idx / 4U]->data);
    }
    // p rate is changed but isn't detected as late until idx==6,
    // since q is 500Hz and p isn't late yet when q checks at idx==4.
    // Will not publish again until idx==6 when q is found as new pivot.
    // Same behavior as initialization dropping faster messages until rates of all are known
    // or found to be late.
    else if (idx >= 4U && idx < 6U) {
      EXPECT_EQ(h.count_, (idx + 2U) / 2U);
      EXPECT_EQ(h.p_->data, p[3]->data);
      EXPECT_EQ(h.q_->data, q[1]->data);
      EXPECT_EQ(h.r_->data, r[0]->data);
    }
    // New actual rate of p computed when is received after q when idx==6
    // for idx >= 6, follows normal "Leading" pattern again
    // with pivot on q
    else if (idx >= 6U) {
      EXPECT_EQ(h.count_, (idx + 2U) / 2U);
      EXPECT_EQ(h.p_->data, p[idx / 2U]->data);
      EXPECT_EQ(h.q_->data, q[idx / 2U]->data);
      EXPECT_EQ(h.r_->data, r[(idx - 2U) / 4U]->data);
    } else {
      EXPECT_FALSE(h.p_);
      EXPECT_FALSE(h.q_);
      EXPECT_FALSE(h.r_);
    }

    double period = 20e6;
    auto new_time = clock->now() + rclcpp::Duration(0, static_cast<uint32_t>(period));
    auto msg = rosgraph_msgs::msg::Clock();
    msg.clock = rclcpp::Time(new_time);
    clock_publisher->publish(msg);
    while (rclcpp::ok() && clock->now() < new_time) {
      executor.spin_once(10ms);
    }
  }
}

TEST_F(LatestTimePolicy, ChangeRateTrailing)
{
  param_client->set_parameters({rclcpp::Parameter("use_sim_time", true)});
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto clock_publisher = node->create_publisher<rosgraph_msgs::msg::Clock>(
    "/clock", rclcpp::ClockQoS());
  rclcpp::TimeSource time_source;
  time_source.attachNode(node);
  time_source.attachClock(clock);
  sync.setClock(clock);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  for (std::size_t idx = 0U; idx < 12U; ++idx) {
    if(idx % 2U == 1U) {
      sync.add<1>(q[(idx - 1U) / 2U]);
    }

    if (idx % 4U == 3U) {
      sync.add<2>(r[(idx - 3U) / 4U]);
    }

    if (idx < 4U) {
      sync.add<0>(p[idx]);
    } else {  // Change rate of p (still 1kHz @ idx == 4)
      if (idx % 3U == 1U) {
        static std::size_t p_idx = 3U;
        sync.add<0>(p[++p_idx]);
      }
    }

    // operates like "Trailing" test for idx <= 3
    if (idx > 2U && idx <= 4U) {
      EXPECT_EQ(h.count_, idx - 2U);
      EXPECT_EQ(h.p_->data, p[idx]->data);
      EXPECT_EQ(h.q_->data, q[(idx - 1U) / 2U]->data);
      EXPECT_EQ(h.r_->data, r[(idx - 3U) / 4U]->data);
    }
    // Rate of p still 1kHz @ idx==4.
    // Then, change rate of p lower than q when idx==5.
    // At idx==5, policy still doesn't know that p is late when q is received.
    // Same behavior as initialization dropping faster messages until rates of all are known
    // or found to be late.
    else if (idx > 4U && idx < 7U) {
      EXPECT_EQ(h.count_, 2U);
      EXPECT_EQ(h.p_->data, p[4U]->data);
      EXPECT_EQ(h.q_->data, q[1U]->data);
      EXPECT_EQ(h.r_->data, r[0U]->data);
    }
    // Will not publish again until idx==7, since rate of q is 500Hz
    // and p is calculated as late when q recieved when idx==7 -- this makes q new pivot.
    // Since q is new pivot and publishes when idx==7,
    // and r comes in after q, r is now trailing.
    // New actual rate of p computed when is received after q when idx==7
    // for idx >= 7, follows normal "Trailing" pattern again
    // with pivot on q
    else if (idx >= 7U) {
      EXPECT_EQ(h.count_, (idx - 1U) / 2U);
      EXPECT_EQ(h.p_->data, p[(idx + 1U) / 2U]->data);
      EXPECT_EQ(h.q_->data, q[(idx - 1U) / 2U]->data);
      EXPECT_EQ(h.r_->data, r[(idx - 5U) / 4U]->data);
    } else {
      EXPECT_FALSE(h.p_);
      EXPECT_FALSE(h.q_);
      EXPECT_FALSE(h.r_);
    }

    double period = 20e6;
    auto new_time = clock->now() + rclcpp::Duration(0, static_cast<uint32_t>(period));
    auto msg = rosgraph_msgs::msg::Clock();
    msg.clock = rclcpp::Time(new_time);
    clock_publisher->publish(msg);
    while (rclcpp::ok() && clock->now() < new_time) {
      executor.spin_once(10ms);
    }
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
