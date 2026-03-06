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

#include <functional>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include "message_filters/cache.hpp"
#include "message_filters/message_traits.hpp"

struct Msg
{
  std_msgs::msg::Header header;
  int data;
};
typedef std::shared_ptr<Msg const> MsgConstPtr;
struct HeaderlessMsg
{
  int data;
};
typedef std::shared_ptr<HeaderlessMsg const> HeaderlessMsgConstPtr;

void fillCacheEasy(message_filters::Cache<Msg> & cache, unsigned int start, unsigned int end)
{
  for (unsigned int i = start; i < end; i++) {
    Msg * msg = new Msg;
    msg->data = i;
    msg->header.stamp = rclcpp::Time(i * 10, 0);

    std::shared_ptr<Msg const> msg_ptr(msg);
    cache.add(msg_ptr);
  }
}

TEST(Cache, emptySurroundingInterval)
{
  message_filters::Cache<Msg> cache(10);
  const std::vector<std::shared_ptr<Msg const>> interval_data = cache.getSurroundingInterval(
    rclcpp::Time(5, 0), rclcpp::Time(9, 0));

  // empty cache shall return empty interval
  EXPECT_EQ(interval_data.size(), static_cast<unsigned int>(0));
}

TEST(Cache, emptySurroundingIntervalHeaderless) {
  message_filters::Cache<HeaderlessMsg> cache(10, true);

  const std::vector<std::shared_ptr<HeaderlessMsg const>> interval_data =
    cache.getSurroundingInterval(rclcpp::Time(5, 0), rclcpp::Time(9, 0));

  // empty cache shall return empty interval
  EXPECT_EQ(interval_data.size(), static_cast<unsigned int>(0));
}

TEST(Cache, easyInterval)
{
  message_filters::Cache<Msg> cache(10);
  fillCacheEasy(cache, 0, 5);

  std::vector<std::shared_ptr<Msg const>> interval_data = cache.getInterval(
    rclcpp::Time(5, 0, RCL_ROS_TIME),
    rclcpp::Time(35, 0, RCL_ROS_TIME)
  );

  ASSERT_EQ(interval_data.size(), (unsigned int) 3);
  EXPECT_EQ(interval_data[0]->data, 1);
  EXPECT_EQ(interval_data[1]->data, 2);
  EXPECT_EQ(interval_data[2]->data, 3);

  // Look for an interval past the end of the cache
  interval_data = cache.getInterval(
    rclcpp::Time(55, 0, RCL_ROS_TIME),
    rclcpp::Time(65, 0, RCL_ROS_TIME)
  );
  EXPECT_EQ(interval_data.size(), (unsigned int) 0);

  // Look for an interval that fell off the back of the cache
  fillCacheEasy(cache, 5, 20);
  interval_data = cache.getInterval(
    rclcpp::Time(5, 0, RCL_ROS_TIME),
    rclcpp::Time(35, 0, RCL_ROS_TIME)
  );
  EXPECT_EQ(interval_data.size(), (unsigned int) 0);
}

TEST(Cache, easySurroundingInterval)
{
  message_filters::Cache<Msg> cache(10);
  fillCacheEasy(cache, 1, 6);

  std::vector<std::shared_ptr<Msg const>> interval_data;
  interval_data = cache.getSurroundingInterval(
    rclcpp::Time(15, 0, RCL_ROS_TIME),
    rclcpp::Time(35, 0, RCL_ROS_TIME));
  ASSERT_EQ(interval_data.size(), (unsigned int) 4);
  EXPECT_EQ(interval_data[0]->data, 1);
  EXPECT_EQ(interval_data[1]->data, 2);
  EXPECT_EQ(interval_data[2]->data, 3);
  EXPECT_EQ(interval_data[3]->data, 4);

  interval_data = cache.getSurroundingInterval(
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    rclcpp::Time(35, 0, RCL_ROS_TIME)
  );
  ASSERT_EQ(interval_data.size(), (unsigned int) 4);
  EXPECT_EQ(interval_data[0]->data, 1);

  interval_data = cache.getSurroundingInterval(
    rclcpp::Time(35, 0, RCL_ROS_TIME),
    rclcpp::Time(35, 0, RCL_ROS_TIME)
  );
  ASSERT_EQ(interval_data.size(), (unsigned int) 2);
  EXPECT_EQ(interval_data[0]->data, 3);
  EXPECT_EQ(interval_data[1]->data, 4);

  interval_data = cache.getSurroundingInterval(
    rclcpp::Time(55, 0, RCL_ROS_TIME),
    rclcpp::Time(55, 0, RCL_ROS_TIME)
  );
  ASSERT_EQ(interval_data.size(), (unsigned int) 1);
  EXPECT_EQ(interval_data[0]->data, 5);
}


std::shared_ptr<Msg const> buildMsg(int32_t seconds, int data)
{
  Msg * msg = new Msg;
  msg->data = data;
  msg->header.stamp = rclcpp::Time(seconds, 0);

  std::shared_ptr<Msg const> msg_ptr(msg);
  return msg_ptr;
}

TEST(Cache, easyUnsorted)
{
  message_filters::Cache<Msg> cache(10);

  cache.add(buildMsg(10, 1));
  cache.add(buildMsg(30, 3));
  cache.add(buildMsg(70, 7));
  cache.add(buildMsg(5, 0));
  cache.add(buildMsg(20, 2));

  std::vector<std::shared_ptr<Msg const>> interval_data = cache.getInterval(
    rclcpp::Time(3, 0, RCL_ROS_TIME),
    rclcpp::Time(15, 0, RCL_ROS_TIME)
  );

  ASSERT_EQ(interval_data.size(), (unsigned int) 2);
  EXPECT_EQ(interval_data[0]->data, 0);
  EXPECT_EQ(interval_data[1]->data, 1);

  // Grab all the data
  interval_data = cache.getInterval(
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    rclcpp::Time(80, 0, RCL_ROS_TIME)
  );
  ASSERT_EQ(interval_data.size(), (unsigned int) 5);
  EXPECT_EQ(interval_data[0]->data, 0);
  EXPECT_EQ(interval_data[1]->data, 1);
  EXPECT_EQ(interval_data[2]->data, 2);
  EXPECT_EQ(interval_data[3]->data, 3);
  EXPECT_EQ(interval_data[4]->data, 7);
}


TEST(Cache, easyElemBeforeAfter)
{
  message_filters::Cache<Msg> cache(10);
  std::shared_ptr<Msg const> elem_ptr;

  fillCacheEasy(cache, 5, 10);

  elem_ptr = cache.getElemAfterTime(rclcpp::Time(85, 0, RCL_ROS_TIME));

  ASSERT_FALSE(!elem_ptr);
  EXPECT_EQ(elem_ptr->data, 9);

  elem_ptr = cache.getElemBeforeTime(rclcpp::Time(85, 0, RCL_ROS_TIME));
  ASSERT_FALSE(!elem_ptr);
  EXPECT_EQ(elem_ptr->data, 8);

  elem_ptr = cache.getElemBeforeTime(rclcpp::Time(45, 0, RCL_ROS_TIME));
  EXPECT_TRUE(!elem_ptr);
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

TEST(Cache, eventInEventOut)
{
  message_filters::Cache<Msg> c0(10);
  message_filters::Cache<Msg> c1(c0, 10);
  EventHelper h;
  c1.registerCallback(&EventHelper::cb, &h);

  message_filters::MessageEvent<Msg const> evt(std::make_shared<Msg const>(), rclcpp::Time(4, 0));
  c0.add(evt);

  EXPECT_EQ(h.event_.getReceiptTime(), evt.getReceiptTime());
  EXPECT_EQ(h.event_.getMessage(), evt.getMessage());
}

// ──────────────────────────────────────────────────────────────────────────────
// getLatestTime / getOldestTime
// ──────────────────────────────────────────────────────────────────────────────

TEST(Cache, latestAndOldestTimeEmpty)
{
  message_filters::Cache<Msg> cache(10);
  // When the cache is empty both methods return a default-constructed rclcpp::Time (epoch 0)
  EXPECT_EQ(cache.getLatestTime().nanoseconds(), 0);
  EXPECT_EQ(cache.getOldestTime().nanoseconds(), 0);
}

TEST(Cache, latestAndOldestTime)
{
  message_filters::Cache<Msg> cache(10);
  fillCacheEasy(cache, 2, 6);  // stamps: 20, 30, 40, 50

  EXPECT_EQ(cache.getOldestTime(), rclcpp::Time(20, 0, RCL_ROS_TIME));
  EXPECT_EQ(cache.getLatestTime(), rclcpp::Time(50, 0, RCL_ROS_TIME));
}

TEST(Cache, latestOldestTimeAfterEviction)
{
  // Cache size 3; add 5 messages — the two oldest should be evicted
  message_filters::Cache<Msg> cache(3);
  fillCacheEasy(cache, 1, 6);  // stamps: 10..50, only last 3 survive

  EXPECT_EQ(cache.getOldestTime(), rclcpp::Time(30, 0, RCL_ROS_TIME));
  EXPECT_EQ(cache.getLatestTime(), rclcpp::Time(50, 0, RCL_ROS_TIME));
}

// ──────────────────────────────────────────────────────────────────────────────
// getElemBeforeTime / getElemAfterTime edge cases
// ──────────────────────────────────────────────────────────────────────────────

TEST(Cache, elemBeforeAfterEmptyCache)
{
  message_filters::Cache<Msg> cache(10);
  EXPECT_FALSE(cache.getElemBeforeTime(rclcpp::Time(100, 0, RCL_ROS_TIME)));
  EXPECT_FALSE(cache.getElemAfterTime(rclcpp::Time(0, 0, RCL_ROS_TIME)));
}

TEST(Cache, elemAfterTimeNoMatch)
{
  // All messages are before the requested time — getElemAfterTime should return null
  message_filters::Cache<Msg> cache(10);
  fillCacheEasy(cache, 1, 4);  // stamps 10, 20, 30

  EXPECT_FALSE(cache.getElemAfterTime(rclcpp::Time(30, 0, RCL_ROS_TIME)));
}

TEST(Cache, elemBeforeTimeExactBoundary)
{
  // getElemBeforeTime is strictly less-than; an exact match should NOT be returned
  message_filters::Cache<Msg> cache(10);
  fillCacheEasy(cache, 1, 4);  // stamps 10, 20, 30

  // Exact match at 10 → nothing before it
  EXPECT_FALSE(cache.getElemBeforeTime(rclcpp::Time(10, 0, RCL_ROS_TIME)));

  // Exact match at 20 → element at 10 is returned
  auto elem = cache.getElemBeforeTime(rclcpp::Time(20, 0, RCL_ROS_TIME));
  ASSERT_TRUE(elem);
  EXPECT_EQ(elem->data, 1);
}

TEST(Cache, elemAfterTimeExactBoundary)
{
  // getElemAfterTime is strictly greater-than; an exact match should NOT be returned
  message_filters::Cache<Msg> cache(10);
  fillCacheEasy(cache, 1, 4);  // stamps 10, 20, 30

  // Exact match at 30 → nothing after it
  EXPECT_FALSE(cache.getElemAfterTime(rclcpp::Time(30, 0, RCL_ROS_TIME)));

  // Time just before 20 → element at 20 is returned
  auto elem = cache.getElemAfterTime(rclcpp::Time(19, 0, RCL_ROS_TIME));
  ASSERT_TRUE(elem);
  EXPECT_EQ(elem->data, 2);
}

// ──────────────────────────────────────────────────────────────────────────────
// getInterval boundary behaviour
// ──────────────────────────────────────────────────────────────────────────────

TEST(Cache, intervalInclusiveBoundaries)
{
  message_filters::Cache<Msg> cache(10);
  fillCacheEasy(cache, 1, 5);  // stamps 10, 20, 30, 40

  // Exact match on both boundaries — both endpoints must be included
  auto data = cache.getInterval(
    rclcpp::Time(10, 0, RCL_ROS_TIME),
    rclcpp::Time(30, 0, RCL_ROS_TIME));
  ASSERT_EQ(data.size(), 3u);
  EXPECT_EQ(data.front()->data, 1);
  EXPECT_EQ(data.back()->data, 3);
}

TEST(Cache, intervalEmptyRange)
{
  message_filters::Cache<Msg> cache(10);
  fillCacheEasy(cache, 1, 5);  // stamps 10..40

  // Range between two existing messages with no entries in between
  auto data = cache.getInterval(
    rclcpp::Time(11, 0, RCL_ROS_TIME),
    rclcpp::Time(19, 0, RCL_ROS_TIME));
  EXPECT_EQ(data.size(), 0u);
}

// ──────────────────────────────────────────────────────────────────────────────
// getSurroundingInterval boundary behaviour
// ──────────────────────────────────────────────────────────────────────────────

TEST(Cache, surroundingIntervalExactTimestampMatch)
{
  message_filters::Cache<Msg> cache(10);
  fillCacheEasy(cache, 1, 5);  // stamps 10, 20, 30, 40

  // Both start and end land exactly on existing stamps
  auto data = cache.getSurroundingInterval(
    rclcpp::Time(20, 0, RCL_ROS_TIME),
    rclcpp::Time(30, 0, RCL_ROS_TIME));

  // Should include one element before start (10) and one at/after end (30), plus those in between
  ASSERT_GE(data.size(), 2u);
  EXPECT_EQ(data.front()->data, 1);  // element before start=20
  EXPECT_EQ(data.back()->data, 3);   // element at end=30
}

TEST(Cache, surroundingIntervalBeyondCache)
{
  message_filters::Cache<Msg> cache(10);
  fillCacheEasy(cache, 1, 4);  // stamps 10, 20, 30

  // Interval completely beyond all cached messages → should return last element
  auto data = cache.getSurroundingInterval(
    rclcpp::Time(100, 0, RCL_ROS_TIME),
    rclcpp::Time(200, 0, RCL_ROS_TIME));
  ASSERT_EQ(data.size(), 1u);
  EXPECT_EQ(data[0]->data, 3);
}

// ──────────────────────────────────────────────────────────────────────────────
// Cache overflow (eviction)
// ──────────────────────────────────────────────────────────────────────────────

TEST(Cache, overflowEvictsOldest)
{
  message_filters::Cache<Msg> cache(3);
  fillCacheEasy(cache, 1, 5);  // stamps 10s, 20s, 30s, 40s; capacity=3 → 10s evicted

  // The element at stamp 10s must have been evicted; oldest remaining is 20s
  EXPECT_EQ(cache.getOldestTime(), rclcpp::Time(20, 0, RCL_ROS_TIME));
  EXPECT_EQ(cache.getLatestTime(), rclcpp::Time(40, 0, RCL_ROS_TIME));

  // getInterval starting before 20s should return nothing before it
  auto data = cache.getInterval(
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    rclcpp::Time(15, 0, RCL_ROS_TIME));
  EXPECT_EQ(data.size(), 0u);
}

// ──────────────────────────────────────────────────────────────────────────────
// setCacheSize(0) is a no-op
// ──────────────────────────────────────────────────────────────────────────────

TEST(Cache, setCacheSizeZeroIsNoop)
{
  message_filters::Cache<Msg> cache(5);
  fillCacheEasy(cache, 0, 4);  // 4 messages, well within limit

  cache.setCacheSize(0);  // must be silently ignored

  // All 4 messages still accessible
  auto data = cache.getInterval(
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    rclcpp::Time(100, 0, RCL_ROS_TIME));
  EXPECT_EQ(data.size(), 4u);
}

// ──────────────────────────────────────────────────────────────────────────────
// Headerless cache throws without allow_headerless
// ──────────────────────────────────────────────────────────────────────────────

TEST(Cache, headerlessThrowsWithoutFlag)
{
  EXPECT_THROW(
    (message_filters::Cache<HeaderlessMsg>(10)),
    std::runtime_error);
}

TEST(Cache, headerlessWorksWithFlag)
{
  EXPECT_NO_THROW(
    (message_filters::Cache<HeaderlessMsg>(10, true)));
}

// ──────────────────────────────────────────────────────────────────────────────
// connectInput wires a filter and populates the cache
// ──────────────────────────────────────────────────────────────────────────────

TEST(Cache, connectInputPopulatesCache)
{
  message_filters::Cache<Msg> source(10);
  message_filters::Cache<Msg> downstream(source, 10);

  fillCacheEasy(source, 1, 4);  // pushes through source → downstream

  auto data = downstream.getInterval(
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    rclcpp::Time(100, 0, RCL_ROS_TIME));
  ASSERT_EQ(data.size(), 3u);
  EXPECT_EQ(data[0]->data, 1);
  EXPECT_EQ(data[2]->data, 3);
}

// ──────────────────────────────────────────────────────────────────────────────
// Out-of-order insertion preserves sorted order for all query methods
// ──────────────────────────────────────────────────────────────────────────────

TEST(Cache, outOfOrderInsertionSorted)
{
  message_filters::Cache<Msg> cache(10);

  cache.add(buildMsg(30, 3));
  cache.add(buildMsg(10, 1));
  cache.add(buildMsg(20, 2));

  EXPECT_EQ(cache.getOldestTime(), rclcpp::Time(10, 0, RCL_ROS_TIME));
  EXPECT_EQ(cache.getLatestTime(), rclcpp::Time(30, 0, RCL_ROS_TIME));

  auto data = cache.getInterval(
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    rclcpp::Time(100, 0, RCL_ROS_TIME));
  ASSERT_EQ(data.size(), 3u);
  EXPECT_EQ(data[0]->data, 1);
  EXPECT_EQ(data[1]->data, 2);
  EXPECT_EQ(data[2]->data, 3);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
