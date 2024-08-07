// Copyright 2010, Willow Garage, Inc. All rights reserved.
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

#include <rclcpp/rclcpp.hpp>
#include "message_filters/chain.hpp"

struct Msg
{
};
typedef std::shared_ptr<Msg> MsgPtr;
typedef std::shared_ptr<Msg const> MsgConstPtr;

class Helper
{
public:
  Helper()
  : count_(0)
  {}

  void cb()
  {
    ++count_;
  }

  int32_t count_;
};

typedef std::shared_ptr<message_filters::PassThrough<Msg>> PassThroughPtr;

TEST(Chain, simple)
{
  Helper h;
  message_filters::Chain<Msg> c;
  c.addFilter(std::make_shared<message_filters::PassThrough<Msg>>());
  c.registerCallback(std::bind(&Helper::cb, &h));

  c.add(std::make_shared<Msg>());
  EXPECT_EQ(h.count_, 1);
  c.add(std::make_shared<Msg>());
  EXPECT_EQ(h.count_, 2);
}

TEST(Chain, multipleFilters)
{
  Helper h;
  message_filters::Chain<Msg> c;
  c.addFilter(std::make_shared<message_filters::PassThrough<Msg>>());
  c.addFilter(std::make_shared<message_filters::PassThrough<Msg>>());
  c.addFilter(std::make_shared<message_filters::PassThrough<Msg>>());
  c.addFilter(std::make_shared<message_filters::PassThrough<Msg>>());
  c.registerCallback(std::bind(&Helper::cb, &h));

  c.add(std::make_shared<Msg>());
  EXPECT_EQ(h.count_, 1);
  c.add(std::make_shared<Msg>());
  EXPECT_EQ(h.count_, 2);
}

TEST(Chain, addingFilters)
{
  Helper h;
  message_filters::Chain<Msg> c;
  c.addFilter(std::make_shared<message_filters::PassThrough<Msg>>());
  c.addFilter(std::make_shared<message_filters::PassThrough<Msg>>());
  c.registerCallback(std::bind(&Helper::cb, &h));

  c.add(std::make_shared<Msg>());
  EXPECT_EQ(h.count_, 1);

  c.addFilter(std::make_shared<message_filters::PassThrough<Msg>>());
  c.addFilter(std::make_shared<message_filters::PassThrough<Msg>>());

  c.add(std::make_shared<Msg>());
  EXPECT_EQ(h.count_, 2);
}

TEST(Chain, inputFilter)
{
  Helper h;
  message_filters::Chain<Msg> c;
  c.addFilter(std::make_shared<message_filters::PassThrough<Msg>>());
  c.registerCallback(std::bind(&Helper::cb, &h));

  message_filters::PassThrough<Msg> p;
  c.connectInput(p);
  p.add(std::make_shared<Msg>());
  EXPECT_EQ(h.count_, 1);

  p.add(std::make_shared<Msg>());
  EXPECT_EQ(h.count_, 2);
}

TEST(Chain, nonSharedPtrFilter)
{
  Helper h;
  message_filters::Chain<Msg> c;
  message_filters::PassThrough<Msg> p;
  c.addFilter(&p);
  c.registerCallback(std::bind(&Helper::cb, &h));

  c.add(std::make_shared<Msg>());
  EXPECT_EQ(h.count_, 1);
  c.add(std::make_shared<Msg>());
  EXPECT_EQ(h.count_, 2);
}

TEST(Chain, retrieveFilter)
{
  message_filters::Chain<Msg> c;

  ASSERT_FALSE(c.getFilter<message_filters::PassThrough<Msg>>(0));

  c.addFilter(std::make_shared<message_filters::PassThrough<Msg>>());

  ASSERT_TRUE(c.getFilter<message_filters::PassThrough<Msg>>(0));
  ASSERT_FALSE(c.getFilter<message_filters::PassThrough<Msg>>(1));
}

TEST(Chain, retrieveFilterThroughBaseClass)
{
  message_filters::Chain<Msg> c;
  message_filters::ChainBase * cb = &c;

  ASSERT_FALSE(cb->getFilter<message_filters::PassThrough<Msg>>(0));

  c.addFilter(std::make_shared<message_filters::PassThrough<Msg>>());

  ASSERT_TRUE(cb->getFilter<message_filters::PassThrough<Msg>>(0));
  ASSERT_FALSE(cb->getFilter<message_filters::PassThrough<Msg>>(1));
}

struct PTDerived : public message_filters::PassThrough<Msg>
{
};

TEST(Chain, retrieveBaseClass)
{
  message_filters::Chain<Msg> c;
  c.addFilter(std::make_shared<PTDerived>());
  ASSERT_TRUE(c.getFilter<message_filters::PassThrough<Msg>>(0));
  ASSERT_TRUE(c.getFilter<PTDerived>(0));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
