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

#ifndef MESSAGE_FILTERS__CACHE_HPP_
#define MESSAGE_FILTERS__CACHE_HPP_

#include <algorithm>
#include <cstddef>
#include <deque>
#include <functional>
#include <memory>
#include <stdexcept>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "message_filters/connection.hpp"
#include "message_filters/simple_filter.hpp"
#include "message_filters/message_traits.hpp"

namespace message_filters
{
/**
 * \brief Stores a time history of messages
 *
 * Given a stream of messages, the most recent N messages are cached in a ring buffer,
 * from which time intervals of the cache can then be retrieved by the client.
 *
 * Cache immediately passes messages through to its output connections.
 *
 * \section connections CONNECTIONS
 *
 * Cache's input and output connections are both of the same signature as rclcpp subscription callbacks, ie.
\verbatim
void callback(const std::shared_ptr<M const> &);
\endverbatim
 */
template<class M>
class Cache : public SimpleFilter<M>
{
public:
  typedef std::shared_ptr<M const> MConstPtr;
  typedef MessageEvent<M const> EventType;

  template<class F>
  explicit Cache(F & f, unsigned int cache_size = 1)
  : Cache(f, cache_size, false) {}

  template<class F>
  explicit Cache(F & f, unsigned int cache_size, bool allow_headerless)
  {
    setCacheSize(cache_size);
    connectInput(f);

    if (message_filters::message_traits::HasHeader<M>::value) {
      getMessageTime = getMessageTimeFromHeader;
    } else if (allow_headerless) {
      getMessageTime = getMessageReceiveTime;
    } else {
      throw std::runtime_error("Caching messages with no header not allowed");
    }
  }

  /**
   * Initializes a Message Cache without specifying a parent filter. This implies that in
   * order to populate the cache, the user then has to call add themselves, or connectInput() is
   * called later
   */
  explicit Cache(unsigned int cache_size = 1)
  : Cache(cache_size, false) {}

  /**
   * Initializes a Message Cache without specifying a parent filter. This implies that in
   * order to populate the cache, the user then has to call add themselves, or connectInput() is
   * called later
   */
  explicit Cache(unsigned int cache_size, bool allow_headerless)
  {
    setCacheSize(cache_size);

    if (message_filters::message_traits::HasHeader<M>::value) {
      getMessageTime = getMessageTimeFromHeader;
    } else if (allow_headerless) {
      getMessageTime = getMessageReceiveTime;
    } else {
      throw std::runtime_error("Caching messages with no header not allowed");
    }
  }

  template<class F>
  void connectInput(F & f)
  {
    incoming_connection_ = f.registerCallback(
      typename SimpleFilter<M>::EventCallback(
        std::bind(&Cache::callback, this, std::placeholders::_1)));
  }

  ~Cache()
  {
    incoming_connection_.disconnect();
  }

  /**
   * Set the size of the cache.
   * \param cache_size The new size the cache should be. Must be > 0
   */
  void setCacheSize(unsigned int cache_size)
  {
    if (cache_size == 0) {
      // ROS_ERROR("Cannot set max_size to 0");
      return;
    }

    cache_size_ = cache_size;
  }

  /**
   * \brief Add a message to the cache, and pop off any elements that are too old.
   * This method is registered with a data provider when connectTo is called.
   */
  void add(const MConstPtr & msg)
  {
    add(EventType(msg));
  }

  /**
   * \brief Add a message to the cache, and pop off any elements that are too old.
   * This method is registered with a data provider when connectTo is called.
   */
  void add(const EventType & evt)
  {
    {
      std::lock_guard<std::mutex> lock(cache_lock_);

      // Keep popping off old data until we have space for a new msg.
      // The front of the deque has the oldest elem.
      while (cache_.size() >= cache_size_) {
        cache_.pop_front();
      }

      // Insert in sorted position using binary search (deque is kept time-ordered).
      const rclcpp::Time evt_stamp = getMessageTime(evt);
      auto it = std::lower_bound(
        cache_.begin(), cache_.end(), evt_stamp,
        [this](const EventType & e, const rclcpp::Time & t) {
          return getMessageTime(e) < t;
        });
      cache_.insert(it, evt);
    }

    this->signalMessage(evt);
  }

  /**
   * \brief Receive a vector of messages that occur between a start and end time (inclusive).
   *
   * This call is non-blocking, and only aggregates messages it has already received.
   * It will not wait for messages have not yet been received, but occur in the interval.
   * \param start The start of the requested interval
   * \param end The end of the requested interval
   */
  std::vector<MConstPtr> getInterval(const rclcpp::Time & start, const rclcpp::Time & end) const
  {
    std::lock_guard<std::mutex> lock(cache_lock_);

    // Binary search for the first element >= start
    auto start_it = std::lower_bound(
      cache_.begin(), cache_.end(), start,
      [this](const EventType & e, const rclcpp::Time & t) {
        return getMessageTime(e) < t;
      });

    // Binary search for the first element > end
    auto end_it = std::upper_bound(
      start_it, cache_.end(), end,
      [this](const rclcpp::Time & t, const EventType & e) {
        return t < getMessageTime(e);
      });

    std::vector<MConstPtr> interval_elems;
    interval_elems.reserve(static_cast<size_t>(std::distance(start_it, end_it)));
    for (auto it = start_it; it != end_it; ++it) {
      interval_elems.push_back(it->getMessage());
    }

    return interval_elems;
  }


  /**
   * \brief Retrieve the smallest interval of messages that surrounds an interval from start to end.
   *
   * If the messages in the cache do not surround (start,end), then this will return the interval
   * that gets closest to surrounding (start,end)
   */
  std::vector<MConstPtr> getSurroundingInterval(
    const rclcpp::Time & start, const rclcpp::Time & end) const
  {
    std::lock_guard<std::mutex> lock(cache_lock_);
    if (cache_.empty()) {
      return {};
    }

    // Find the last element whose time <= start (i.e. one before lower_bound(start))
    auto start_it = std::lower_bound(
      cache_.begin(), cache_.end(), start,
      [this](const EventType & e, const rclcpp::Time & t) {
        return getMessageTime(e) < t;
      });
    if (start_it != cache_.begin()) {
      --start_it;
    }

    // Find the first element whose time >= end (i.e. lower_bound(end))
    auto end_it = std::lower_bound(
      start_it, cache_.end(), end,
      [this](const EventType & e, const rclcpp::Time & t) {
        return getMessageTime(e) < t;
      });
    if (end_it != cache_.end()) {
      ++end_it;  // include the element at/after end
    }

    std::vector<MConstPtr> interval_elems;
    interval_elems.reserve(static_cast<size_t>(std::distance(start_it, end_it)));
    for (auto it = start_it; it != end_it; ++it) {
      interval_elems.push_back(it->getMessage());
    }

    return interval_elems;
  }

  /**
   * \brief Grab the newest element that occurs right before the specified time.
   * \param time Time that must occur right after the returned elem
   * \returns shared_ptr to the newest elem that occurs before 'time'. NULL if doesn't exist
   */
  MConstPtr getElemBeforeTime(const rclcpp::Time & time) const
  {
    std::lock_guard<std::mutex> lock(cache_lock_);

    // lower_bound finds the first element >= time; the element before it is < time
    auto it = std::lower_bound(
      cache_.begin(), cache_.end(), time,
      [this](const EventType & e, const rclcpp::Time & t) {
        return getMessageTime(e) < t;
      });

    if (it == cache_.begin()) {
      return {};
    }
    --it;
    return it->getMessage();
  }

  /**
   * \brief Grab the oldest element that occurs right after the specified time.
   * \param time Time that must occur right before the returned elem
   * \returns shared_ptr to the oldest elem that occurs after 'time'. NULL if doesn't exist
   */
  MConstPtr getElemAfterTime(const rclcpp::Time & time) const
  {
    std::lock_guard<std::mutex> lock(cache_lock_);

    // upper_bound finds the first element strictly > time
    auto it = std::upper_bound(
      cache_.begin(), cache_.end(), time,
      [this](const rclcpp::Time & t, const EventType & e) {
        return t < getMessageTime(e);
      });

    if (it == cache_.end()) {
      return {};
    }
    return it->getMessage();
  }

  /**
   * \brief Returns the timestamp associated with the newest packet cache
   */
  rclcpp::Time getLatestTime() const
  {
    namespace mt = message_filters::message_traits;

    std::lock_guard<std::mutex> lock(cache_lock_);

    rclcpp::Time latest_time;

    if (cache_.size() > 0) {
      latest_time = getMessageTime(cache_.back());
    }

    return latest_time;
  }

  /**
   * \brief Returns the timestamp associated with the oldest packet cache
   */
  rclcpp::Time getOldestTime() const
  {
    namespace mt = message_filters::message_traits;

    std::lock_guard<std::mutex> lock(cache_lock_);

    rclcpp::Time oldest_time;

    if (cache_.size() > 0) {
      oldest_time = getMessageTime(cache_.front());
    }

    return oldest_time;
  }

private:
  void callback(const EventType & evt)
  {
    add(evt);
  }

  static rclcpp::Time getMessageTimeFromHeader(const EventType & evt)
  {
    return message_filters::message_traits::TimeStamp<M>::value(*(evt.getMessage()));
  }

  static rclcpp::Time getMessageReceiveTime(const EventType & evt)
  {
    return evt.getReceiptTime();
  }

  mutable std::mutex cache_lock_;      //!< Lock for cache_
  std::deque<EventType> cache_;        //!< Cache for the messages
  unsigned int cache_size_;            //!< Maximum number of elements allowed in the cache.

  Connection incoming_connection_;

  std::function<rclcpp::Time(const EventType & evt)> getMessageTime;
};
}  // namespace message_filters

#endif  // MESSAGE_FILTERS__CACHE_HPP_
