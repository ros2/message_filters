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

#include <gtest/gtest.h>

#include <memory>
#include <optional>
#include <string>

#include <message_filters/cache.hpp>
#include "message_filters/connection.hpp"
#include <message_filters/delta_filter.hpp>
#include <message_filters/simple_filter.hpp>


namespace test_messages
{
struct IntMsg
{
  int data;
};

struct FloatMsg
{
  float data;
};

struct BoolMsg
{
  bool data;
};

}   // namespace test_messages

using TestMessageTypes = ::testing::Types<
  test_messages::IntMsg,
  test_messages::FloatMsg,
  test_messages::BoolMsg
>;

template<typename T>
struct ParametrizedTest : public ::testing::Test
{
  using TypeParam = T;
  static constexpr T value_0 {0};
  static constexpr T value_1 {1};
};


template<typename M>
class ComparisonHandlerMock : public message_filters::CachedComparisonHandler<M>
{
public:
  typedef std::shared_ptr<M const> MConstPtr;
  typedef std::function<message_filters::MFieldType(const MConstPtr &)> FieldGetterFunctionType;

  ComparisonHandlerMock(
    // Parameter not used but expected in a base class signature
    [[maybe_unused]] std::forward_list<FieldGetterFunctionType> _ = {}
  )
  : message_filters::CachedComparisonHandler<M>(
      {[] (const MConstPtr & msg) {return msg->data;}}
  ) {}

  void set_comparison_success(bool success)
  {
    comparison_success = success;
  }

  bool do_fields_fit(
    // Parameters not used but expected in a base class signature
    [[maybe_unused]] message_filters::MFieldType field_a,
    [[maybe_unused]] message_filters::MFieldType field_b
  ) const
  {
    return comparison_success;
  }

private:
  bool comparison_success {true};
};

template<typename M>
class SimpleCachefilter : public message_filters::SimpleFilter<M>
{
public:
  typedef message_filters::MessageEvent<M const> EventType;

  template<typename F>
  explicit SimpleCachefilter(F & f)
  : message_filters::SimpleFilter<M>()
  {
    connectInput(f);
  }

  template<class F>
  void connectInput(F & f)
  {
    incoming_connection_ = f.registerCallback(
            typename message_filters::SimpleFilter<M>::EventCallback(
                std::bind(&SimpleCachefilter::callback, this, std::placeholders::_1)
            )
    );
  }

  void callback(const EventType & evt)
  {
    event_cache = evt;
  }

public:
  std::optional<EventType> event_cache;

private:
  message_filters::Connection incoming_connection_;
};

template<typename T>
struct ParametrizedCachedCompareTest : public ParametrizedTest<T> {};
TYPED_TEST_SUITE(ParametrizedCachedCompareTest, TestMessageTypes);
TYPED_TEST(ParametrizedCachedCompareTest, DoFieldsFitCall)
{
  using MsgType = typename TestFixture::TypeParam;
  typedef message_filters::MessageEvent<MsgType const> EventType;

  MsgType test_msg_0 = TestFixture::value_0;
  MsgType test_msg_1 = TestFixture::value_1;

  auto handler = ComparisonHandlerMock<MsgType>();

  EXPECT_TRUE(
      handler.message_fits(
          EventType(
              std::make_shared<MsgType>(test_msg_0)
          )
      )
  );

  EXPECT_TRUE(
      handler.message_fits(
          EventType(
              std::make_shared<MsgType>(test_msg_1)
          )
      )
  );

  handler.set_comparison_success(false);

  EXPECT_FALSE(
      handler.message_fits(
          EventType(
              std::make_shared<MsgType>(test_msg_0)
          )
      )
  );
}

template<typename T>
struct ParametrizedDeltaCompareTest : public ParametrizedCachedCompareTest<T> {};
TYPED_TEST_SUITE(ParametrizedDeltaCompareTest, TestMessageTypes);
TYPED_TEST(ParametrizedDeltaCompareTest, TestComparison)
{
  using MsgType = typename TestFixture::TypeParam;
  typedef message_filters::MessageEvent<MsgType const> EventType;
  typedef std::shared_ptr<MsgType const> MConstPtr;

  MsgType test_msg_0 = TestFixture::value_0;
  MsgType test_msg_1 = TestFixture::value_1;

  auto handler = message_filters::DeltaCompare<MsgType>(
    {[] (const MConstPtr & msg) {return msg->data;}}
  );

  // New message for empty comparison handler -> True
  EXPECT_TRUE(
      handler.message_fits(
          EventType(
              std::make_shared<MsgType>(test_msg_0)
          )
      )
  );

  // New message for non empty comparison handler -> True
  EXPECT_TRUE(
      handler.message_fits(
          EventType(
              std::make_shared<MsgType>(test_msg_1)
          )
      )
  );

  // Message duplicate for non empty comparison handler -> False
  EXPECT_FALSE(
      handler.message_fits(
          EventType(
              std::make_shared<MsgType>(test_msg_1)
          )
      )
  );
}

template<typename T>
struct ParametrizedComparisonFilterTest : public ParametrizedCachedCompareTest<T> {};
TYPED_TEST_SUITE(ParametrizedComparisonFilterTest, TestMessageTypes);
TYPED_TEST(ParametrizedComparisonFilterTest, TestComparison)
{
  using MsgType = typename TestFixture::TypeParam;
  typedef std::shared_ptr<MsgType const> MConstPtr;
  typedef message_filters::MessageEvent<MsgType const> EventType;

  MsgType test_msg_0 = TestFixture::value_0;
  MsgType test_msg_1 = TestFixture::value_1;

  auto simple_filter = message_filters::SimpleFilter<MsgType>();
  auto comparison_filter = message_filters::ComparisonFilter<MsgType, ComparisonHandlerMock>(
    simple_filter,
    {[] (const MConstPtr & msg) {return msg->data;}}
  );
  auto cache_filter = SimpleCachefilter<MsgType>(comparison_filter);

  comparison_filter.add(
      EventType(
          std::make_shared<MsgType>(test_msg_0)
      )
  );
  EXPECT_TRUE(cache_filter.event_cache.has_value());
  EXPECT_EQ(cache_filter.event_cache->getMessage()->data, test_msg_0.data);

  comparison_filter.add(
      EventType(
          std::make_shared<MsgType>(test_msg_1)
      )
  );
  EXPECT_TRUE(cache_filter.event_cache.has_value());
  EXPECT_EQ(cache_filter.event_cache->getMessage()->data, test_msg_1.data);
}


template<typename T>
struct ParametrizedDeltaFilterTest : public ParametrizedTest<T> {};
TYPED_TEST_SUITE(ParametrizedDeltaFilterTest, TestMessageTypes);
TYPED_TEST(ParametrizedDeltaFilterTest, TestComparison)
{
  using MsgType = typename TestFixture::TypeParam;
  typedef std::shared_ptr<MsgType const> MConstPtr;
  typedef message_filters::MessageEvent<MsgType const> EventType;

  MsgType test_msg_0 = TestFixture::value_0;
  MsgType test_msg_1 = TestFixture::value_1;

  auto simple_filter = message_filters::SimpleFilter<MsgType>();
  auto delta_filter = message_filters::DeltaFilter<MsgType>(
    simple_filter,
    {[] (const MConstPtr & msg) {return msg->data;}}
  );
  auto cache_filter = SimpleCachefilter<MsgType>(delta_filter);

  // New message received and passed down the line
  delta_filter.add(
      EventType(
          std::make_shared<MsgType>(test_msg_0)
      )
  );
  EXPECT_TRUE(cache_filter.event_cache.has_value());
  EXPECT_EQ(cache_filter.event_cache->getMessage()->data, test_msg_0.data);

  // Same message received and not passed down the line
  cache_filter.event_cache.reset();
  delta_filter.add(
      EventType(
          std::make_shared<MsgType>(test_msg_0)
      )
  );
  EXPECT_FALSE(cache_filter.event_cache.has_value());

  // New message received and passed down the line
  delta_filter.add(
      EventType(
          std::make_shared<MsgType>(test_msg_1)
      )
  );
  EXPECT_TRUE(cache_filter.event_cache.has_value());
  EXPECT_EQ(cache_filter.event_cache->getMessage()->data, test_msg_1.data);
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  auto ret = RUN_ALL_TESTS();
  return ret;
}
