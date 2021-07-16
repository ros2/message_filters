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
#include "message_filters/sync_policies/latest.h"
#include "message_filters/sync_policies/latest_stamped.h"
#include "message_filters/sync_policies/exact_time.h"

using namespace message_filters;
using namespace message_filters::sync_policies;

struct Header
{
  rclcpp::Time stamp;
};


struct Msg
{
  Header header;
  Msg(rclcpp::Time stamp = rclcpp::Time(0,0))
  {
    header.stamp = stamp;
  }
};
typedef std::shared_ptr<Msg> MsgPtr;
typedef std::shared_ptr<Msg const> MsgConstPtr;

struct MsgHeaderless
{
  int data;
  MsgHeaderless(int data = -1) : data(data)
  {
  }
};
typedef std::shared_ptr<MsgHeaderless> MsgHeaderlessPtr;
typedef std::shared_ptr<MsgHeaderless const> MsgHeaderlessConstPtr;

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

struct EventHelper
{
  void callback(const MessageEvent<Msg const>& e1, const MessageEvent<Msg const>& e2, const MessageEvent<Msg const>& e3)
  {
    e1_ = e1;
    e2_ = e2;
    e3_ = e3;
  }

  MessageEvent<Msg const> e1_;
  MessageEvent<Msg const> e2_;
  MessageEvent<Msg const> e3_;
};

struct EventHelperHeaderless
{
  void callback(const MessageEvent<Msg const>& e1, const MessageEvent<MsgHeaderless const>& e2, const MessageEvent<Msg const>& e3)
  {
    e1_ = e1;
    e2_ = e2;
    e3_ = e3;
  }

  MessageEvent<Msg const> e1_;
  MessageEvent<MsgHeaderless const> e2_;
  MessageEvent<Msg const> e3_;
};

typedef LatestStamped<Msg, Msg, Msg> Policy;
typedef Synchronizer<Policy> Sync;

TEST(LatestStamped, eventInEventOut)
{
  Sync sync;
  EventHelper helper;
  sync.registerCallback(&EventHelper::callback, &helper);
  MessageEvent<Msg const> evt_1(std::make_shared<Msg>( rclcpp::Time(1, 0) ));
  MessageEvent<Msg const> evt_2(std::make_shared<Msg>( rclcpp::Time(2, 1) ));
  MessageEvent<Msg const> evt_3(std::make_shared<Msg>( rclcpp::Time(2, 2) ));

  auto assert_events_1and2 = [&helper, &evt_1, &evt_2, &evt_3] (void) -> void
  {
    ASSERT_TRUE(helper.e1_.getMessage());
    ASSERT_TRUE(helper.e2_.getMessage());
    ASSERT_TRUE(helper.e1_ == evt_3);
    ASSERT_TRUE(helper.e2_ == evt_2);
    ASSERT_FALSE(helper.e1_ == evt_1);
    ASSERT_FALSE(helper.e1_ == evt_2);
    ASSERT_FALSE(helper.e2_ == evt_1);
    ASSERT_FALSE(helper.e2_ == evt_3);
  };

  sync.add<0>(evt_1);
  sync.add<0>(evt_2);
  sync.add<0>(evt_3);
  sync.add<1>(evt_2);
  sync.add<1>(evt_1);

  sync.getPolicy()->call();
  assert_events_1and2();
  ASSERT_FALSE(helper.e3_.getMessage());

  sync.add<2>(evt_1);
  sync.getPolicy()->call();
  assert_events_1and2();
  ASSERT_TRUE(helper.e3_.getMessage());
  ASSERT_TRUE(helper.e3_ == evt_1);
  ASSERT_FALSE(helper.e3_ == evt_2);
  ASSERT_FALSE(helper.e3_ == evt_3);
}


typedef Latest<Msg, MsgHeaderless, Msg> PolicyHeaderless;
typedef Synchronizer<PolicyHeaderless> SyncHeaderless;

TEST(Latest, eventInEventOut)
{
  SyncHeaderless sync;
  EventHelperHeaderless helper;
  sync.registerCallback(&EventHelperHeaderless::callback, &helper);

  MessageEvent<Msg const> evt_1(std::make_shared<Msg>( rclcpp::Time(1, 0) ));
  MessageEvent<Msg const> evt_2(std::make_shared<Msg>( rclcpp::Time(1, 0) ));
  MessageEvent<Msg const> evt_3(std::make_shared<Msg>( rclcpp::Time(1, 0) ));

  MessageEvent<MsgHeaderless const> evt_1_hl(std::make_shared<MsgHeaderless>(1));
  MessageEvent<MsgHeaderless const> evt_2_hl(std::make_shared<MsgHeaderless>(2));

  sync.getPolicy()->call();
  ASSERT_FALSE(helper.e1_.getMessage());
  ASSERT_FALSE(helper.e2_.getMessage());
  ASSERT_FALSE(helper.e3_.getMessage());

  sync.add<0>(evt_1);
  sync.add<0>(evt_2);
  sync.add<1>(evt_1_hl);
  sync.add<1>(evt_2_hl);
  sync.add<2>(evt_1);
  sync.add<2>(evt_2);
  sync.add<2>(evt_3);

  sync.getPolicy()->call();
  ASSERT_TRUE(helper.e1_ == evt_2);
  ASSERT_TRUE(helper.e2_ == evt_2_hl);
  ASSERT_TRUE(helper.e3_ == evt_3);
  //
  ASSERT_FALSE(helper.e1_ == evt_1);
  ASSERT_FALSE(helper.e1_ == evt_3);
  ASSERT_FALSE(helper.e2_ == evt_1_hl);
  ASSERT_FALSE(helper.e3_ == evt_1);
  ASSERT_FALSE(helper.e3_ == evt_2);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}


