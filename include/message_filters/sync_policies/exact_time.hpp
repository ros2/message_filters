// Copyright 2009, Willow Garage, Inc. All rights reserved.
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

#ifndef MESSAGE_FILTERS__SYNC_POLICIES__EXACT_TIME_HPP_
#define MESSAGE_FILTERS__SYNC_POLICIES__EXACT_TIME_HPP_

#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <tuple>

#include <rclcpp/rclcpp.hpp>

#include "message_filters/connection.hpp"
#include "message_filters/message_traits.hpp"
#include "message_filters/null_types.hpp"
#include "message_filters/signal9.hpp"
#include "message_filters/synchronizer.hpp"

namespace message_filters
{
namespace sync_policies
{

template<typename ... Ms>
struct ExactTime : public PolicyBase<Ms...>
{
  typedef Synchronizer<ExactTime> Sync;
  typedef PolicyBase<Ms...> Super;
  typedef typename Super::Messages Messages;
  typedef typename Super::Signal Signal;
  typedef typename Super::Events Events;
  typedef typename Super::RealTypeCount RealTypeCount;
  typedef Events Tuple;

  ExactTime(uint32_t queue_size)  // NOLINT(runtime/explicit)
  : parent_(0)
    , queue_size_(queue_size)
  {
  }

  ExactTime(const ExactTime & e)
  {
    *this = e;
  }

  ExactTime & operator=(const ExactTime & rhs)
  {
    parent_ = rhs.parent_;
    queue_size_ = rhs.queue_size_;
    last_signal_time_ = rhs.last_signal_time_;
    tuples_ = rhs.tuples_;

    return *this;
  }

  void initParent(Sync * parent)
  {
    parent_ = parent;
  }

  template<int i>
  void add(const typename std::tuple_element<i, Events>::type & evt)
  {
    assert(parent_);

    namespace mt = message_filters::message_traits;

    std::lock_guard<std::mutex> lock(mutex_);

    Tuple & t =
      tuples_[mt::TimeStamp<typename std::tuple_element<i,
        Messages>::type>::value(*evt.getMessage())];
    std::get<i>(t) = evt;

    checkTuple(t);
  }

  template<class C>
  Connection registerDropCallback(const C & callback)
  {
    return drop_signal_.addCallback(callback);
  }

  template<class C>
  Connection registerDropCallback(C & callback)
  {
    return drop_signal_.addCallback(callback);
  }

  template<class C, typename T>
  Connection registerDropCallback(const C & callback, T * t)
  {
    return drop_signal_.addCallback(callback, t);
  }

  template<class C, typename T>
  Connection registerDropCallback(C & callback, T * t)
  {
    return drop_signal_.addCallback(callback, t);
  }

private:
  template<std::size_t... Is>
  static bool isFull(const Tuple & t, std::index_sequence<Is...>)
  {
    /* *INDENT-OFF* */
    // uncrustify messes with the brackets which are required (at least by GCC)
    return (... && static_cast<bool>(std::get<Is>(t).getMessage()));
    /* *INDENT-ON* */
  }

  // assumes mutex_ is already locked
  void checkTuple(Tuple & t)
  {
    namespace mt = message_filters::message_traits;

    const bool full = isFull(t, std::make_index_sequence<std::tuple_size_v<Tuple>>{});

    if (full) {
      std::apply([this](auto &&... args) {this->parent_->signal(args ...);}, t);

      using M0 = std::tuple_element_t<0, std::tuple<Ms...>>;
      last_signal_time_ = mt::TimeStamp<M0>::value(*std::get<0>(t).getMessage());

      tuples_.erase(last_signal_time_);

      clearOldTuples();
    }

    if (queue_size_ > 0) {
      while (tuples_.size() > queue_size_) {
        Tuple & t2 = tuples_.begin()->second;
        std::apply([this](auto &&... args) {this->drop_signal_.call(args ...);}, t2);
        tuples_.erase(tuples_.begin());
      }
    }
  }

  // assumes mutex_ is already locked
  void clearOldTuples()
  {
    typename M_TimeToTuple::iterator it = tuples_.begin();
    typename M_TimeToTuple::iterator end = tuples_.end();
    for (; it != end; ) {
      if (it->first <= last_signal_time_) {
        typename M_TimeToTuple::iterator old = it;
        ++it;

        Tuple & t = old->second;
        std::apply([this](auto &&... args) {this->drop_signal_.call(args ...);}, t);
        tuples_.erase(old);
      } else {
        // the map is sorted by time, so we can ignore anything after this if this one's time is ok
        break;
      }
    }
  }

private:
  Sync * parent_;

  uint32_t queue_size_;
  typedef std::map<rclcpp::Time, Tuple> M_TimeToTuple;
  M_TimeToTuple tuples_;
  rclcpp::Time last_signal_time_;

  Signal drop_signal_;

  std::mutex mutex_;
};

}  // namespace sync_policies
}  // namespace message_filters

#endif  // MESSAGE_FILTERS__SYNC_POLICIES__EXACT_TIME_HPP_
