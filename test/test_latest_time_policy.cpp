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
  : count_(0)
  {}

  void cb(const MsgConstPtr& p, const MsgConstPtr& q, const MsgConstPtr& r)
  {
    ASSERT_TRUE(p);
    ASSERT_TRUE(q);
    ASSERT_TRUE(r);

    p_ = p; q_ = q; r_ = r;
    ++count_;
  }

  MsgConstPtr p_, q_, r_;
  uint16_t count_;
};

typedef LatestTime<Msg, Msg, Msg> Policy3;
typedef Synchronizer<Policy3> Sync3;

TEST(LatestTime, Leading)
{
  Sync3 sync(1);
  Helper h;
  sync.registerCallback(std::bind(&Helper::cb, &h, std::placeholders::_1,
                                                   std::placeholders::_2,
                                                   std::placeholders::_3));
  std::vector<MsgPtr> p; p.reserve(8U);
  std::vector<MsgPtr> q; q.reserve(4U);
  std::vector<MsgPtr> r; r.reserve(2U);
  for(std::size_t idx = 0U; idx < 8U; ++idx)
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

  rclcpp::Rate rate(50.0);
  for(std::size_t idx = 0U; idx < 8U; ++idx)
  {
    if(idx % 2U == 0U)
    {
      sync.add<1>(q[idx/2]);
    }
    if(idx % 4U == 0U)
    {
      sync.add<2>(r[idx/4]);
    }
    sync.add<0>(p[idx]);
    
    ASSERT_EQ(h.count_, idx);
    if(idx > 0)
    {
      ASSERT_EQ(h.p_->data, p[idx]->data);
      ASSERT_EQ(h.q_->data, q[idx/2]->data);
      ASSERT_EQ(h.r_->data, r[idx/4]->data);
    }
    else
    {
      ASSERT_FALSE(h.p_);
      ASSERT_FALSE(h.q_);
      ASSERT_FALSE(h.r_);
    }

    rate.sleep();
  }
}

TEST(LatestTime, Trailing)
{
  Sync3 sync(1);
  Helper h;
  sync.registerCallback(std::bind(&Helper::cb, &h, std::placeholders::_1,
                                                   std::placeholders::_2,
                                                   std::placeholders::_3));
  std::vector<MsgPtr> p; p.reserve(8U);
  std::vector<MsgPtr> q; q.reserve(4U);
  std::vector<MsgPtr> r; r.reserve(2U);
  for(std::size_t idx = 0U; idx < 8U; ++idx)
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

  rclcpp::Rate rate(50.0);
  for(std::size_t idx = 0U; idx < 8U; ++idx)
  {
    if(idx % 2U == 1U)
    {
      sync.add<1>(q[(idx-1U)/2U]);
    }
    if(idx % 4U == 3U)
    {
      sync.add<2>(r[(idx-3U)/4U]);
    }
    sync.add<0>(p[idx]);
    
    switch(idx)
    {
    case 0U:
    case 1U:
    case 2U:
      ASSERT_EQ(h.count_, 0U);
      ASSERT_FALSE(h.p_);
      ASSERT_FALSE(h.q_);
      ASSERT_FALSE(h.r_);
      break;
    case 3U:
      ASSERT_EQ(h.count_, 1U);
      ASSERT_EQ(h.p_->data, p[idx]->data);
      ASSERT_EQ(h.q_->data, q[1U]->data);
      ASSERT_EQ(h.r_->data, r[0U]->data);
      break;
    case 4U:
      ASSERT_EQ(h.count_, 2U);
      ASSERT_EQ(h.p_->data, p[idx]->data);
      ASSERT_EQ(h.q_->data, q[1U]->data);
      ASSERT_EQ(h.r_->data, r[0U]->data);
      break;
    case 5U:
      ASSERT_EQ(h.count_, 3U);
      ASSERT_EQ(h.p_->data, p[idx]->data);
      ASSERT_EQ(h.q_->data, q[2U]->data);
      ASSERT_EQ(h.r_->data, r[0U]->data);
      break;
    case 6U:
      ASSERT_EQ(h.count_, 4U);
      ASSERT_EQ(h.p_->data, p[idx]->data);
      ASSERT_EQ(h.q_->data, q[2U]->data);
      ASSERT_EQ(h.r_->data, r[0U]->data);
      break;
    case 7U:
      ASSERT_EQ(h.count_, 5U);
      ASSERT_EQ(h.p_->data, p[idx]->data);
      ASSERT_EQ(h.q_->data, q[3U]->data);
      ASSERT_EQ(h.r_->data, r[1U]->data);
      break;
    }
    
    rate.sleep();
  }
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}


