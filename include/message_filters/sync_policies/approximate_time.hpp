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

#ifndef MESSAGE_FILTERS__SYNC_POLICIES__APPROXIMATE_TIME_HPP_
#define MESSAGE_FILTERS__SYNC_POLICIES__APPROXIMATE_TIME_HPP_

#include <inttypes.h>

#include <rcutils/logging_macros.h>

#include <cassert>
#include <deque>
#include <limits>
#include <string>
#include <tuple>
#include <vector>

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
struct ApproximateTime : public PolicyBase<Ms...>
{
  typedef Synchronizer<ApproximateTime> Sync;
  typedef PolicyBase<Ms...> Super;
  typedef typename Super::Messages Messages;
  typedef typename Super::Signal Signal;
  typedef typename Super::Events Events;
  typedef Events Tuple;
  typedef std::tuple<std::deque<MessageEvent<Ms const>>...> DequeTuple;
  typedef std::tuple<std::vector<MessageEvent<Ms const>>...> VectorTuple;

  using Super::N_MESSAGES;

  ApproximateTime(uint32_t queue_size)  // NOLINT(runtime/explicit)
  : parent_(0)
    , queue_size_(queue_size)
    , num_non_empty_deques_(0)
    , pivot_(NO_PIVOT)
    , max_interval_duration_(rclcpp::Duration(std::numeric_limits<int32_t>::max(), 999999999))
    , age_penalty_(0.1)
    , has_dropped_messages_(N_MESSAGES, false)
    , inter_message_lower_bounds_(N_MESSAGES, rclcpp::Duration(0, 0))
    , warned_about_incorrect_bound_(N_MESSAGES, false)
  {
    // The synchronizer will tend to drop many messages with a queue size of 1.
    // At least 2 is recommended.
    assert(queue_size_ > 0);
  }

  ApproximateTime(const ApproximateTime & e)
  : max_interval_duration_(rclcpp::Duration(std::numeric_limits<int32_t>::max(), 999999999))
  {
    *this = e;
  }

  ApproximateTime & operator=(const ApproximateTime & rhs)
  {
    parent_ = rhs.parent_;
    queue_size_ = rhs.queue_size_;
    num_non_empty_deques_ = rhs.num_non_empty_deques_;
    pivot_time_ = rhs.pivot_time_;
    pivot_ = rhs.pivot_;
    max_interval_duration_ = rhs.max_interval_duration_;
    age_penalty_ = rhs.age_penalty_;
    candidate_start_ = rhs.candidate_start_;
    candidate_end_ = rhs.candidate_end_;
    deques_ = rhs.deques_;
    past_ = rhs.past_;
    has_dropped_messages_ = rhs.has_dropped_messages_;
    inter_message_lower_bounds_ = rhs.inter_message_lower_bounds_;
    warned_about_incorrect_bound_ = rhs.warned_about_incorrect_bound_;

    return *this;
  }

  void initParent(Sync * parent)
  {
    parent_ = parent;
  }

  template<int i>
  void checkInterMessageBound()
  {
    namespace mt = message_filters::message_traits;
    if (warned_about_incorrect_bound_[i]) {
      return;
    }
    std::deque<typename std::tuple_element<i, Events>::type> & deque = std::get<i>(deques_);
    std::vector<typename std::tuple_element<i, Events>::type> & v = std::get<i>(past_);
    assert(!deque.empty());
    const typename std::tuple_element<i, Messages>::type & msg = *(deque.back()).getMessage();
    rclcpp::Time msg_time =
      mt::TimeStamp<typename std::tuple_element<i, Messages>::type>::value(msg);
    rclcpp::Time previous_msg_time;
    if (deque.size() == static_cast<size_t>(1)) {
      if (v.empty()) {
        // We have already published (or have never received) the previous message,
        // we cannot check the bound
        return;
      }
      const typename std::tuple_element<i,
        Messages>::type & previous_msg = *(v.back()).getMessage();
      previous_msg_time = mt::TimeStamp<typename std::tuple_element<i, Messages>::type>::value(
        previous_msg);
    } else {
      // There are at least 2 elements in the deque.
      // Check that the gap respects the bound if it was provided.
      const typename std::tuple_element<i,
        Messages>::type & previous_msg = *(deque[deque.size() - 2]).getMessage();
      previous_msg_time = mt::TimeStamp<typename std::tuple_element<i, Messages>::type>::value(
        previous_msg);
    }
    if (msg_time < previous_msg_time) {
      RCUTILS_LOG_WARN_ONCE("Messages of type %d arrived out of order (will print only once)", i);
      warned_about_incorrect_bound_[i] = true;
    } else if ((msg_time - previous_msg_time) < inter_message_lower_bounds_[i]) {
      RCUTILS_LOG_WARN_ONCE(
        "Messages of type %d arrived closer ("
        "%" PRId64 ") than the lower bound you provided ("
        "%" PRId64 ") (will print only once)",
        i,
        (msg_time - previous_msg_time).nanoseconds(),
        inter_message_lower_bounds_[i].nanoseconds());
      warned_about_incorrect_bound_[i] = true;
    }
  }


  template<int i>
  void add(const typename std::tuple_element<i, Events>::type & evt)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    std::deque<typename std::tuple_element<i, Events>::type> & deque = std::get<i>(deques_);
    deque.push_back(evt);
    if (deque.size() == static_cast<size_t>(1)) {
      // We have just added the first message, so it was empty before
      ++num_non_empty_deques_;
      if (num_non_empty_deques_ == (uint32_t)N_MESSAGES) {
        // All deques have messages
        process();
      }
    } else {
      checkInterMessageBound<i>();
    }
    // Check whether we have more messages than allowed in the queue.
    // Note that during the above call to process(), queue i may contain queue_size_+1 messages.
    std::vector<typename std::tuple_element<i, Events>::type> & past = std::get<i>(past_);
    if (deque.size() + past.size() > queue_size_) {
      // Cancel ongoing candidate search, if any:
      num_non_empty_deques_ = 0;  // We will recompute it from scratch
      recoverAll(std::make_index_sequence<N_MESSAGES>{});
      // Drop the oldest message in the offending topic
      assert(!deque.empty());
      deque.pop_front();
      has_dropped_messages_[i] = true;
      if (pivot_ != NO_PIVOT) {
        // The candidate is no longer valid. Destroy it.
        candidate_ = Tuple();
        pivot_ = NO_PIVOT;
        // There might still be enough messages to create a new candidate:
        process();
      }
    }
  }

  void setAgePenalty(double age_penalty)
  {
    // For correctness we only need age_penalty > -1.0,
    // but most likely a negative age_penalty is a mistake.
    assert(age_penalty >= 0);
    age_penalty_ = age_penalty;
  }

  void setInterMessageLowerBound(int i, rclcpp::Duration lower_bound)
  {
    // For correctness we only need age_penalty > -1.0,
    // but most likely a negative age_penalty is a mistake.
    assert(lower_bound >= rclcpp::Duration(0, 0));
    inter_message_lower_bounds_[i] = lower_bound;
  }

  void setMaxIntervalDuration(rclcpp::Duration max_interval_duration)
  {
    // For correctness we only need age_penalty > -1.0,
    // but most likely a negative age_penalty is a mistake.
    assert(max_interval_duration >= rclcpp::Duration(0, 0));
    max_interval_duration_ = max_interval_duration;
  }

private:
  // Assumes that deque number <index> is non empty
  template<int i>
  void dequeDeleteFront()
  {
    std::deque<typename std::tuple_element<i, Events>::type> & deque = std::get<i>(deques_);
    assert(!deque.empty());
    deque.pop_front();
    if (deque.empty()) {
      --num_non_empty_deques_;
    }
  }

  template<std::size_t I>
  void dequeDeleteFrontImpl(std::size_t i)
  {
    if (I == i) {
      dequeDeleteFront<I>();
    } else {
      if constexpr (I > 0) {
        dequeDeleteFrontImpl<I - 1>(i);
      } else {
        std::abort();
      }
    }
  }

  // Assumes that deque number <index> is non empty
  void dequeDeleteFront(uint32_t index)
  {
    dequeDeleteFrontImpl<N_MESSAGES - 1>(index);
  }

  // Assumes that deque number <index> is non empty
  template<int i>
  void dequeMoveFrontToPast()
  {
    std::deque<typename std::tuple_element<i, Events>::type> & deque = std::get<i>(deques_);
    std::vector<typename std::tuple_element<i, Events>::type> & vector = std::get<i>(past_);
    assert(!deque.empty());
    vector.push_back(deque.front());
    deque.pop_front();
    if (deque.empty()) {
      --num_non_empty_deques_;
    }
  }

  template<std::size_t I>
  void dequeMoveFrontToPastImpl(std::size_t i)
  {
    if (I == i) {
      dequeMoveFrontToPast<I>();
    } else {
      if constexpr (I > 0) {
        dequeMoveFrontToPastImpl<I - 1>(i);
      } else {
        std::abort();
      }
    }
  }

  // Assumes that deque number <index> is non empty
  void dequeMoveFrontToPast(uint32_t index)
  {
    dequeMoveFrontToPastImpl<N_MESSAGES - 1>(index);
  }

  template<std::size_t... Is>
  void assignFront(Tuple & candidate, DequeTuple & deques, std::index_sequence<Is...>)
  {
    ((std::get<Is>(candidate) = std::get<Is>(deques).front()), ...);
  }

  template<std::size_t... Is>
  void clear(VectorTuple & tuple, std::index_sequence<Is...>)
  {
    (std::get<Is>(tuple).clear(), ...);
  }

  void makeCandidate()
  {
    // Create candidate tuple
    candidate_ = Tuple();  // Discards old one if any
    assignFront(candidate_, deques_, std::make_index_sequence<N_MESSAGES>{});

    // Delete all past messages, since we have found a better candidate
    clear(past_, std::make_index_sequence<N_MESSAGES>{});
  }

  template<std::size_t... Is>
  void recoverAll(const std::array<int, N_MESSAGES> & num_messages, std::index_sequence<Is...>)
  {
    (recover<Is>(num_messages[Is]), ...);
  }

  // ASSUMES: num_messages <= past_[i].size()
  template<int i>
  void recover(size_t num_messages)
  {
    std::vector<typename std::tuple_element<i, Events>::type> & v = std::get<i>(past_);
    std::deque<typename std::tuple_element<i, Events>::type> & q = std::get<i>(deques_);
    assert(num_messages <= v.size());
    while (num_messages > 0) {
      q.push_front(v.back());
      v.pop_back();
      num_messages--;
    }

    if (!q.empty()) {
      ++num_non_empty_deques_;
    }
  }

  template<std::size_t... Is>
  void recoverAll(std::index_sequence<Is...>)
  {
    (recover<Is>(), ...);
  }

  template<int i>
  void recover()
  {
    std::vector<typename std::tuple_element<i, Events>::type> & v = std::get<i>(past_);
    std::deque<typename std::tuple_element<i, Events>::type> & q = std::get<i>(deques_);
    while (!v.empty()) {
      q.push_front(v.back());
      v.pop_back();
    }

    if (!q.empty()) {
      ++num_non_empty_deques_;
    }
  }

  template<std::size_t... Is>
  void recoverAndDeleteAll(std::index_sequence<Is...>)
  {
    (recoverAndDelete<Is>(), ...);
  }

  template<int i>
  void recoverAndDelete()
  {
    std::vector<typename std::tuple_element<i, Events>::type> & v = std::get<i>(past_);
    std::deque<typename std::tuple_element<i, Events>::type> & q = std::get<i>(deques_);
    while (!v.empty()) {
      q.push_front(v.back());
      v.pop_back();
    }

    assert(!q.empty());

    q.pop_front();
    if (!q.empty()) {
      ++num_non_empty_deques_;
    }
  }

  // Assumes: all deques are non empty, i.e. num_non_empty_deques_ == N_MESSAGES
  void publishCandidate()
  {
    // Publish
    std::apply([this](auto &&... args) {this->parent_->signal(args ...);}, candidate_);
    // Delete this candidate
    candidate_ = Tuple();
    pivot_ = NO_PIVOT;

    // Recover hidden messages, and delete the ones corresponding to the candidate
    num_non_empty_deques_ = 0;  // We will recompute it from scratch
    recoverAndDeleteAll(std::make_index_sequence<N_MESSAGES>{});
  }

  // Assumes: all deques are non empty, i.e. num_non_empty_deques_ == N_MESSAGES
  // Returns: the oldest message on the deques
  void getCandidateStart(uint32_t & start_index, rclcpp::Time & start_time)
  {
    return getCandidateBoundary(start_index, start_time, false);
  }

  // Assumes: all deques are non empty, i.e. num_non_empty_deques_ == N_MESSAGES
  // Returns: the latest message among the heads of the deques, i.e. the minimum
  //          time to end an interval started at getCandidateStart_index()
  void getCandidateEnd(uint32_t & end_index, rclcpp::Time & end_time)
  {
    return getCandidateBoundary(end_index, end_time, true);
  }

  template<std::size_t I>
  void checkBoundary(uint32_t & index, rclcpp::Time & time, bool end)
  {
    namespace mt = message_filters::message_traits;
    using MEvent = typename std::tuple_element<I, Events>::type;
    MEvent & m = std::get<I>(deques_).front();
    using M = typename MEvent::Message;
    if ((I == 0) || ((mt::TimeStamp<M>::value(*m.getMessage()) < time) ^ end)) {
      time = mt::TimeStamp<M>::value(*m.getMessage());
      index = I;
    }
  }

  template<std::size_t... Is>
  void checkAllBoudnaries(
    uint32_t & index, rclcpp::Time & time, bool end,
    std::index_sequence<Is...>)
  {
    (checkBoundary<Is>(index, time, end), ...);
  }

  // ASSUMES: all deques are non-empty
  // end = true: look for the latest head of deque
  //       false: look for the earliest head of deque
  void getCandidateBoundary(uint32_t & index, rclcpp::Time & time, bool end)
  {
    checkAllBoudnaries(index, time, end, std::make_index_sequence<N_MESSAGES>{});
  }

  // ASSUMES: we have a pivot and candidate
  template<int i>
  rclcpp::Time getVirtualTime()
  {
    namespace mt = message_filters::message_traits;

    assert(pivot_ != NO_PIVOT);

    std::vector<typename std::tuple_element<i, Events>::type> & v = std::get<i>(past_);
    std::deque<typename std::tuple_element<i, Events>::type> & q = std::get<i>(deques_);
    if (q.empty()) {
      assert(!v.empty());  // Because we have a candidate
      rclcpp::Time last_msg_time =
        mt::TimeStamp<typename std::tuple_element<i, Messages>::type>::value(
        *(v.back()).getMessage());
      rclcpp::Time msg_time_lower_bound = last_msg_time + inter_message_lower_bounds_[i];
      if (msg_time_lower_bound > pivot_time_) {  // Take the max
        return msg_time_lower_bound;
      }
      return pivot_time_;
    }
    rclcpp::Time current_msg_time =
      mt::TimeStamp<typename std::tuple_element<i, Messages>::type>::value(
      *(q.front()).getMessage());
    return current_msg_time;
  }


  // ASSUMES: we have a pivot and candidate
  void getVirtualCandidateStart(uint32_t & start_index, rclcpp::Time & start_time)
  {
    return getVirtualCandidateBoundary(start_index, start_time, false);
  }

  // ASSUMES: we have a pivot and candidate
  void getVirtualCandidateEnd(uint32_t & end_index, rclcpp::Time & end_time)
  {
    return getVirtualCandidateBoundary(end_index, end_time, true);
  }

  template<std::size_t... Is>
  std::array<rclcpp::Time, N_MESSAGES> getVirtualTimes(std::index_sequence<Is...>)
  {
    return {getVirtualTime<Is>()...};
  }

  // ASSUMES: we have a pivot and candidate
  // end = true: look for the latest head of deque
  //       false: look for the earliest head of deque
  void getVirtualCandidateBoundary(uint32_t & index, rclcpp::Time & time, bool end)
  {
    const auto virtual_times{getVirtualTimes(std::make_index_sequence<N_MESSAGES>{})};

    time = virtual_times[0];
    index = 0;
    for (uint32_t i = 0u; i < virtual_times.size(); i++) {
      if ((virtual_times[i] < time) ^ end) {
        time = virtual_times[i];
        index = i;
      }
    }
  }


  // assumes data_mutex_ is already locked
  void process()
  {
    // While no deque is empty
    while (num_non_empty_deques_ == (uint32_t)N_MESSAGES) {
      // Find the start and end of the current interval
      rclcpp::Time end_time, start_time;
      uint32_t end_index, start_index;
      getCandidateEnd(end_index, end_time);
      getCandidateStart(start_index, start_time);
      for (uint32_t i = 0; i < (uint32_t)N_MESSAGES; i++) {
        if (i != end_index) {
          // No dropped message could have been better to use than the ones we have,
          // so it becomes ok to use this topic as pivot in the future
          has_dropped_messages_[i] = false;
        }
      }
      if (pivot_ == NO_PIVOT) {
        // We do not have a candidate
        // INVARIANT: the past_ vectors are empty
        // INVARIANT: (candidate_ has no filled members)
        if (end_time - start_time > max_interval_duration_) {
          // This interval is too big to be a valid candidate, move to the next
          dequeDeleteFront(start_index);
          continue;
        }
        if (has_dropped_messages_[end_index]) {
          // The topic that would become pivot has dropped messages, so it is not a good pivot
          dequeDeleteFront(start_index);
          continue;
        }
        // This is a valid candidate, and we don't have any, so take it
        makeCandidate();
        candidate_start_ = start_time;
        candidate_end_ = end_time;
        pivot_ = end_index;
        pivot_time_ = end_time;
        dequeMoveFrontToPast(start_index);
      } else {
        // We already have a candidate
        // Is this one better than the current candidate?
        // INVARIANT: has_dropped_messages_ is all false
        if ((end_time - candidate_end_) * (1 + age_penalty_) >=
          (start_time - candidate_start_))
        {
          // This is not a better candidate, move to the next
          dequeMoveFrontToPast(start_index);
        } else {
          // This is a better candidate
          makeCandidate();
          candidate_start_ = start_time;
          candidate_end_ = end_time;
          dequeMoveFrontToPast(start_index);
          // Keep the same pivot (and pivot time)
        }
      }
      // INVARIANT: we have a candidate and pivot
      assert(pivot_ != NO_PIVOT);
      rclcpp::Duration age_check = (end_time - candidate_end_) * (1 + age_penalty_);
      if (start_index == pivot_) {  // TODO(anyone): replace with start_time == pivot_time_
        // We have exhausted all possible candidates for this pivot, we now can output the best one
        publishCandidate();
      } else if (age_check >= (pivot_time_ - candidate_start_)) {
        // We have not exhausted all candidates, but this candidate is already provably optimal
        // Indeed, any future candidate must contain the interval [pivot_time_ end_time], which
        // is already too big.
        // Note: this case is subsumed by the next, but it may save some unnecessary work and
        //       it makes things (a little) easier to understand
        publishCandidate();
      } else if (num_non_empty_deques_ < (uint32_t)N_MESSAGES) {
        uint32_t num_non_empty_deques_before_virtual_search = num_non_empty_deques_;

        // Before giving up, use the rate bounds, if provided, to further try to prove optimality
        std::array<int, N_MESSAGES> num_virtual_moves{};
        while (1) {
          rclcpp::Time end_time, start_time;
          uint32_t end_index, start_index;
          getVirtualCandidateEnd(end_index, end_time);
          getVirtualCandidateStart(start_index, start_time);
          if ((end_time - candidate_end_) * (1 + age_penalty_) >=
            (pivot_time_ - candidate_start_))
          {
            // We have proved optimality
            // As above, any future candidate must contain the interval [pivot_time_ end_time],
            // which is already too big.
            publishCandidate();  // This cleans up the virtual moves as a byproduct
            break;  // From the while(1) loop only
          }
          if ((end_time - candidate_end_) * (1 + age_penalty_) <
            (start_time - candidate_start_))
          {
            // We cannot prove optimality
            // Indeed, we have a virtual (i.e. optimistic) candidate that is better than the current
            // candidate
            // Cleanup the virtual search:
            num_non_empty_deques_ = 0;  // We will recompute it from scratch
            recoverAll(num_virtual_moves, std::make_index_sequence<N_MESSAGES>{});
            (void)num_non_empty_deques_before_virtual_search;  // unused variable warning stopper
            assert(num_non_empty_deques_before_virtual_search == num_non_empty_deques_);
            break;
          }
          // Note: we cannot reach this point with start_index == pivot_ since in that case we would
          // have start_time == pivot_time, in which case the two tests above are the negation
          // of each other, so that one must be true. Therefore the while loop always terminates.
          assert(start_index != pivot_);
          assert(start_time < pivot_time_);
          dequeMoveFrontToPast(start_index);
          num_virtual_moves[start_index]++;
        }  // while(1)
      }
    }  // while(num_non_empty_deques_ == (uint32_t)N_MESSAGES)
  }

  Sync * parent_;
  uint32_t queue_size_;

  // Special value for the pivot indicating that no pivot has been selected
  static const uint32_t NO_PIVOT = std::numeric_limits<uint32_t>::max();

  DequeTuple deques_;
  uint32_t num_non_empty_deques_;
  VectorTuple past_;
  Tuple candidate_;  // NULL if there is no candidate, in which case there is no pivot.
  rclcpp::Time candidate_start_;
  rclcpp::Time candidate_end_;
  rclcpp::Time pivot_time_;
  uint32_t pivot_;  // Equal to NO_PIVOT if there is no candidate
  std::mutex data_mutex_;  // Protects all of the above

  rclcpp::Duration max_interval_duration_;  // TODO(anyone): initialize with a parameter
  double age_penalty_;

  std::vector<bool> has_dropped_messages_;
  std::vector<rclcpp::Duration> inter_message_lower_bounds_;
  std::vector<bool> warned_about_incorrect_bound_;
};

}  // namespace sync_policies
}  // namespace message_filters

#endif  // MESSAGE_FILTERS__SYNC_POLICIES__APPROXIMATE_TIME_HPP_
