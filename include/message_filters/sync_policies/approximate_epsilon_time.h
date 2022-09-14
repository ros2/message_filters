/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2022, Open Source Robotics Foundation, Inc.
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

#ifndef MESSAGE_FILTERS__SYNC_POLICIES__APPROXIMATE_EPSILON_TIME_H_
#define MESSAGE_FILTERS__SYNC_POLICIES__APPROXIMATE_EPSILON_TIME_H_

#include <deque>
#include <string>
#include <tuple>

#include <rclcpp/rclcpp.hpp>

#include "message_filters/connection.h"
#include "message_filters/message_traits.h"
#include "message_filters/null_types.h"
#include "message_filters/signal9.h"
#include "message_filters/synchronizer.h"

namespace message_filters
{
namespace sync_policies
{

template<typename M0, typename M1, typename M2 = NullType, typename M3 = NullType, typename M4 = NullType,
         typename M5 = NullType, typename M6 = NullType, typename M7 = NullType, typename M8 = NullType>
class ApproximateEpsilonTime : public PolicyBase<M0, M1, M2, M3, M4, M5, M6, M7, M8>
{
public:
  typedef Synchronizer<ApproximateEpsilonTime> Sync;
  typedef PolicyBase<M0, M1, M2, M3, M4, M5, M6, M7, M8> Super;
  typedef typename Super::Messages Messages;
  typedef typename Super::Signal Signal;
  typedef typename Super::Events Events;
  typedef typename Super::RealTypeCount RealTypeCount;
  typedef typename Super::M0Event M0Event;
  typedef typename Super::M1Event M1Event;
  typedef typename Super::M2Event M2Event;
  typedef typename Super::M3Event M3Event;
  typedef typename Super::M4Event M4Event;
  typedef typename Super::M5Event M5Event;
  typedef typename Super::M6Event M6Event;
  typedef typename Super::M7Event M7Event;
  typedef typename Super::M8Event M8Event;
  typedef Events Tuple;

  ApproximateEpsilonTime(uint32_t queue_size, rclcpp::Duration epsilon)
  : parent_(nullptr)
  , queue_size_(queue_size)
  , epsilon_{epsilon}
  {
  }

  ApproximateEpsilonTime(const ApproximateEpsilonTime& e)
  : epsilon_{e.epsilon_}
  {
    *this = e;
  }

  ApproximateEpsilonTime& operator=(const ApproximateEpsilonTime& rhs)
  {
    parent_ = rhs.parent_;
    queue_size_ = rhs.queue_size_;
    events_ = rhs.events_;
    epsilon_ = rhs.epsilon_;

    return *this;
  }

  void initParent(Sync* parent)
  {
    parent_ = parent;
  }

  template<size_t i>
  void add(const typename std::tuple_element<i, Events>::type& evt)
  {
    RCUTILS_ASSERT(parent_);

    std::lock_guard<std::mutex> lock(mutex_);

    auto & events_of_this_type = std::get<i>(events_);
    if (0u == events_of_this_type.size()) {
      ++number_of_non_empty_events_;
    }
    events_of_this_type.push_back(evt);
    if (number_of_non_empty_events_ == RealTypeCount::value) {
      process();
    } else if (events_of_this_type.size() > queue_size_) {
      erase_beginning_of_vector<i>();
    }
  }

private:
  using TimeIndexPair = std::pair<rclcpp::Time, size_t>;

  template <size_t Is>
  TimeIndexPair
  get_older_timestamp_between(const TimeIndexPair & current)
  {
    namespace mt = message_filters::message_traits;
    using ThisEventType = typename std::tuple_element<Is, Events>::type;
    if constexpr (Is >= RealTypeCount::value) {
      return current;
    }
    const auto & events_of_this_type = std::get<Is>(events_);
    if (0u == events_of_this_type.size()) {
      // this condition should not happen
      return current;
    }
    auto candidate = mt::TimeStamp<typename ThisEventType::Message>::value(*events_of_this_type.at(0).getMessage());
    if (current.first > candidate) {
      return std::make_pair(candidate, Is);
    }
    return current;
  }

  template <size_t ... Is>
  TimeIndexPair
  get_older_timestamp_helper(std::index_sequence<Is...> const &)
  {
    TimeIndexPair older{
      rclcpp::Time(std::numeric_limits<int64_t>::max(), RCL_ROS_TIME), std::numeric_limits<size_t>::max()};
    ((older = get_older_timestamp_between<Is>(older)), ...);
    return older;
  }

  TimeIndexPair
  get_older_timestamp()
  {
    return get_older_timestamp_helper(std::make_index_sequence<9u>());
  }

  template <size_t Is>
  bool
  check_timestamp_within_epsilon(const TimeIndexPair & older)
  {
    namespace mt = message_filters::message_traits;
    using ThisEventType = typename std::tuple_element<Is, Events>::type;
    if constexpr (Is >= RealTypeCount::value) {
      return true;
    }
    if (Is == older.second) {
      return true;
    }
    const auto & events_of_this_type = std::get<Is>(events_);
    if (0u == events_of_this_type.size()) {
      // this condition should not happen
      return false;
    }
    auto ts = mt::TimeStamp<typename ThisEventType::Message>::value(*events_of_this_type.at(0).getMessage());
    if (older.first + epsilon_ >= ts) {
      return true;
    }
    return false;
  }

  template <size_t ... Is>
  bool
  check_all_timestamp_within_epsilon_helper(
    const TimeIndexPair & older, std::index_sequence<Is...> const &)
  {
    bool valid = true;
    ((valid &= check_timestamp_within_epsilon<Is>(older)), ...);
    return valid;
  }

  bool
  check_all_timestamp_within_epsilon(const TimeIndexPair & older)
  {
    return check_all_timestamp_within_epsilon_helper(
      older, std::make_index_sequence<9u>());
  }

  template<size_t Is>
  void
  erase_beginning_of_vector()
  {
    if constexpr (Is >= RealTypeCount::value) {
      return;
    }
    auto & this_vector = std::get<Is>(events_);
    if (this_vector.begin() != this_vector.end()) {
      this_vector.erase(this_vector.begin());
      if (this_vector.empty()) {
        --number_of_non_empty_events_;
      }
    }
  }

  template <size_t ... Is>
  void
  erase_beginning_of_vectors_helper(std::index_sequence<Is...> const &)
  {
    ((erase_beginning_of_vector<Is>()), ...);
  }

  void erase_beginning_of_vectors()
  {
    return erase_beginning_of_vectors_helper(std::make_index_sequence<9u>());
  }

  template<size_t Is>
  void
  erase_beginning_of_vector_if_on_sync_with_ts(rclcpp::Time timestamp)
  {
    namespace mt = message_filters::message_traits;
    using ThisEventType = typename std::tuple_element<Is, Events>::type;
    if constexpr (Is >= RealTypeCount::value) {
      return;
    }
    auto & this_vector = std::get<Is>(events_);
    if (this_vector.begin() == this_vector.end()) {
      return;
    }
    auto event_ts = mt::TimeStamp<typename ThisEventType::Message>::value(
      *this_vector.at(0).getMessage());
    if (timestamp + epsilon_ < event_ts) {
      return;
    }
    this_vector.erase(this_vector.begin());
    if (this_vector.empty()) {
      --number_of_non_empty_events_;
    }
  }

  template <size_t ... Is>
  void
  erase_old_events_if_on_sync_with_ts_helper(rclcpp::Time timestamp, std::index_sequence<Is...> const &)
  {
    ((erase_beginning_of_vector_if_on_sync_with_ts<Is>(timestamp)), ...);
  }

  void erase_old_events_if_on_sync_with_ts(rclcpp::Time timestamp)
  {
    return erase_old_events_if_on_sync_with_ts_helper(timestamp, std::make_index_sequence<9u>());
  }

  void signal()
  {
    if constexpr (RealTypeCount::value == 2) {
      parent_->signal(
        std::get<0>(events_).at(0), std::get<1>(events_).at(0),
        M2Event{}, M3Event{}, M4Event{}, M5Event{}, M6Event{}, M7Event{}, M8Event{});
    } else if constexpr (RealTypeCount::value == 3) {
      parent_->signal(
        std::get<0>(events_).at(0), std::get<1>(events_).at(0), std::get<2>(events_).at(0),
        M3Event{}, M4Event{}, M5Event{}, M6Event{}, M7Event{}, M8Event{});
    } else if constexpr (RealTypeCount::value == 4) {
      parent_->signal(
        std::get<0>(events_).at(0), std::get<1>(events_).at(0), std::get<2>(events_).at(0),
        std::get<3>(events_).at(0),
        M4Event{}, M5Event{}, M6Event{}, M7Event{}, M8Event{});
    } else if constexpr (RealTypeCount::value == 5) {
      parent_->signal(
        std::get<0>(events_).at(0), std::get<1>(events_).at(0), std::get<2>(events_).at(0),
        std::get<3>(events_).at(0), std::get<4>(events_).at(0),
        M5Event{}, M6Event{}, M7Event{}, M8Event{});
    } else if constexpr (RealTypeCount::value == 6) {
      parent_->signal(
        std::get<0>(events_).at(0), std::get<1>(events_).at(0), std::get<2>(events_).at(0),
        std::get<3>(events_).at(0), std::get<4>(events_).at(0), std::get<5>(events_).at(0),
        M6Event{}, M7Event{}, M8Event{});
    } else if constexpr (RealTypeCount::value == 7) {
      parent_->signal(
        std::get<0>(events_).at(0), std::get<1>(events_).at(0), std::get<2>(events_).at(0),
        std::get<3>(events_).at(0), std::get<4>(events_).at(0), std::get<5>(events_).at(0),
        std::get<6>(events_).at(0),
        M7Event{}, M8Event{});
    } else if constexpr (RealTypeCount::value == 8) {
      parent_->signal(
        std::get<0>(events_).at(0), std::get<1>(events_).at(0), std::get<2>(events_).at(0),
        std::get<3>(events_).at(0), std::get<4>(events_).at(0), std::get<5>(events_).at(0),
        std::get<6>(events_).at(0), std::get<7>(events_).at(0),
        M8Event{});
    } else if constexpr (RealTypeCount::value == 9) {
      parent_->signal(
        std::get<0>(events_).at(0), std::get<1>(events_).at(0), std::get<2>(events_).at(0),
        std::get<3>(events_).at(0), std::get<4>(events_).at(0), std::get<5>(events_).at(0),
        std::get<6>(events_).at(0), std::get<7>(events_).at(0), std::get<8>(events_).at(0));
    } else {
      static_assert("RealTypeCount::value should be >=2 and <=9");
    }
  }

  // assumes mutex_ is already locked
  void process()
  {
    while (number_of_non_empty_events_ == RealTypeCount::value) {
      auto old_ts = get_older_timestamp();
      if (check_all_timestamp_within_epsilon(old_ts)) {
        signal();
        erase_beginning_of_vectors();
      } else {
        erase_old_events_if_on_sync_with_ts(old_ts.first);
      }
    }
  }

  Sync* parent_;

  uint32_t queue_size_;
  rclcpp::Duration epsilon_;
  size_t number_of_non_empty_events_{0};
  using TupleOfVecOfEvents = std::tuple<
    std::vector<M0Event>, std::vector<M1Event>, std::vector<M2Event>,
    std::vector<M3Event>, std::vector<M4Event>, std::vector<M5Event>,
    std::vector<M6Event>, std::vector<M7Event>, std::vector<M8Event>>;
  TupleOfVecOfEvents events_;

  std::mutex mutex_;
};

}  // namespace sync_policies
}  // namespace message_filters

#endif  // MESSAGE_FILTERS__SYNC_POLICIES__APPROXIMATE_EPSILON_TIME_H_

