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

#include <rclcpp/rclcpp.hpp>
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/latest_time.h"

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
typedef std::shared_ptr<Msg> MsgPtr;
typedef std::shared_ptr<Msg const> MsgConstPtr;

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

class Helper
{
public:
  Helper()
  : count_(0),
    ros_clock_(RCL_ROS_TIME)
  {}

  void cb(const MsgConstPtr& p, const MsgConstPtr& q, const MsgConstPtr& r)
  {
    EXPECT_TRUE(p);
    EXPECT_TRUE(q);
    EXPECT_TRUE(r);

    p_ = p; q_ = q; r_ = r;
    ++count_;
  }

  MsgConstPtr p_, q_, r_;
  uint16_t count_;
  rclcpp::Clock ros_clock_;
};

typedef LatestTime<Msg, Msg, Msg> Policy3;
typedef Synchronizer<Policy3> Sync3;

class LatestTimePolicy : public ::testing::Test
{
protected:
  Sync3 sync{1};
  Helper h;
  std::vector<MsgPtr> p;
  std::vector<MsgPtr> q;
  std::vector<MsgPtr> r;

  virtual void SetUp()
  {
    sync.registerCallback(std::bind(&Helper::cb, &h, std::placeholders::_1,
                                                     std::placeholders::_2,
                                                     std::placeholders::_3));
    p.reserve(12U);
    q.reserve(6U);
    r.reserve(3U);
    for(std::size_t idx = 0U; idx < 12U; ++idx)
    {
      MsgPtr p_idx(std::make_shared<Msg>()); p_idx->data = idx; p.push_back(p_idx);
      if(idx % 2U == 0U)
      {
        MsgPtr q_idx(std::make_shared<Msg>()); q_idx->data = idx; q.push_back(q_idx);
      }
      if(idx % 4U == 0U)
      {
        MsgPtr r_idx(std::make_shared<Msg>()); r_idx->data = idx; r.push_back(r_idx);
      }
    }
  }
};


TEST_F(LatestTimePolicy, Leading)
{
  rclcpp::Rate rate(50.0);
  for(std::size_t idx = 0U; idx < 8U; ++idx)
  {
    if(idx % 2U == 0U)
    {
      sync.add<1>(q[idx / 2U]);
    }
    if(idx % 4U == 0U)
    {
      sync.add<2>(r[idx / 4U]);
    }
    sync.add<0>(p[idx]);

    EXPECT_EQ(h.count_, idx);
    if(idx > 0)
    {
      EXPECT_EQ(h.p_->data, p[idx]->data);
      EXPECT_EQ(h.q_->data, q[idx / 2U]->data);
      EXPECT_EQ(h.r_->data, r[idx / 4U]->data);
    }
    else
    {
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
  for(std::size_t idx = 0U; idx < 8U; ++idx)
  {
    if(idx % 2U == 1U)
    {
      sync.add<1>(q[(idx - 1U) / 2U]);
    }
    if(idx % 4U == 3U)
    {
      sync.add<2>(r[(idx - 3U) / 4U]);
    }
    sync.add<0>(p[idx]);

    if (idx > 2U)
    {
      EXPECT_EQ(h.count_, idx - 2U);
      EXPECT_EQ(h.p_->data, p[idx]->data);
      EXPECT_EQ(h.q_->data, q[(idx - 1U) / 2U]->data);
      EXPECT_EQ(h.r_->data, r[(idx - 3U) / 4U]->data);
    }
    else
    {
      EXPECT_FALSE(h.p_);
      EXPECT_FALSE(h.q_);
      EXPECT_FALSE(h.r_);
    }

    rate.sleep();
  }
}

TEST_F(LatestTimePolicy, ChangeRate)
{
  rclcpp::Rate rate(50.0);
  for(std::size_t idx = 0U; idx < 12U; ++idx)
  {
    if(idx % 2U == 1U)
    {
      sync.add<1>(q[(idx - 1U) / 2U]);
    }

    if(idx % 4U == 3U)
    {
      sync.add<2>(r[(idx - 3U) / 4U]);
    }

    if (idx < 4U)
    {
      sync.add<0>(p[idx]);
    }
    else  // Change rate of p (still 50Hz @ idx == 4)
    {
      if(idx % 3U == 1U)
      {
        static std::size_t p_idx = 3U;
        sync.add<0>(p[++p_idx]);
      }
    }

    // operates like "Trailing" test for idx <= 3
    if (idx == 3U)
    {
      EXPECT_EQ(h.count_, idx - 2U);
      EXPECT_EQ(h.p_->data, p[idx]->data);
      EXPECT_EQ(h.q_->data, q[(idx - 1U) / 2U]->data);
      EXPECT_EQ(h.r_->data, r[(idx - 3U) / 4U]->data);
    }
    // rate of p still 50Hz @ idx==4, then change rate of p lower than q
    // will not publish again until idx==9
    // since rate of q is 25Hz, p was 50Hz @ idx==4,
    // and new rate of p computed after q received @ idx==7
    else if (idx >= 4U && idx < 9U)
    {
      EXPECT_EQ(h.count_, 2U);
      EXPECT_EQ(h.p_->data, p[4U]->data);
      EXPECT_EQ(h.q_->data, q[1U]->data);
      EXPECT_EQ(h.r_->data, r[0U]->data);
    }
    // for idx >= 9, follows normal "Trailing" pattern again
    // with pivot on q
    else if (idx >= 9U && idx < 11U)
    {
      EXPECT_EQ(h.count_, 3U);
      EXPECT_EQ(h.p_->data, p[5U]->data);
      EXPECT_EQ(h.q_->data, q[4U]->data);
      EXPECT_EQ(h.r_->data, r[1U]->data);
    }
    else if (idx == 11U)
    {
      EXPECT_EQ(h.count_, 4U);
      EXPECT_EQ(h.p_->data, p[6U]->data);
      EXPECT_EQ(h.q_->data, q[5U]->data);
      EXPECT_EQ(h.r_->data, r[1U]->data);
    }
    else
    {
      EXPECT_FALSE(h.p_);
      EXPECT_FALSE(h.q_);
      EXPECT_FALSE(h.r_);
    }

    rate.sleep();
  }
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}

