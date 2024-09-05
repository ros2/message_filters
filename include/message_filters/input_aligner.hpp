// Copyright 2024, Kraken Robotics. All rights reserved.
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

#ifndef MESSAGE_FILTERS__INPUT_ALIGNER_HPP_
#define MESSAGE_FILTERS__INPUT_ALIGNER_HPP_

#include <cstdint>
#include <string>
#include <tuple>
#include <set>
#include <memory>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>

#include "message_filters/connection.hpp"
#include "message_filters/message_traits.hpp"
#include "message_filters/null_types.hpp"
#include "message_filters/signal1.hpp"

namespace message_filters
{

/**
 * \brief Aligns multiple inputs in time and passing them through in order.
 * For N inputs this filter provides N outputs.
 * Often sensors or pre-processing chains might introduce delays
 * to messages until they arrive at a target node.
 * The input aligner ensures that the messages are forwarded in order.
 *
 * If the input messages are periodic a hint about their frequency can be
 * set via `setInputPeriod` for each input. This allows the filter to look ahead
 * for the given time and reduces delays.
 * The period should be selected optimistically at the lower bound.
 *
 * If for an input no messages arrive the input will timeout and will be ignored until
 * new messages arrive.
 *
 * Forwarding messages can be either triggered by calling `dispatchMessages` periodically
 * or by setting up a timer with `setupDispatchTimer`.
 *
 * This implementation was inspired by ROCK's stream aligner.
 * https://www.rock-robotics.org/documentation/data_processing/stream_aligner.html
 */
template<typename ... Ms>
class InputAligner : public noncopyable
{
public:
  static constexpr std::size_t N_INPUTS = sizeof...(Ms);
  typedef std::tuple<Ms...> Messages;
  typedef std::tuple<MessageEvent<Ms const>...> Events;
  typedef std::tuple<Signal1<Ms>...> Signals;

  /**
   * \brief Status information of a input queue
   */
  struct QueueStatus
  {
    bool active;
    std::size_t queue_size;
    std::size_t msgs_processed;
    std::size_t msgs_dropped;
  };

  template<class F0, class F1, class ... Fs>
  InputAligner(const rclcpp::Duration & timeout, F0 & f0, F1 & f1, Fs &... fs)
  : timeout_(timeout)
    , last_in_ts_(0, 0, RCL_ROS_TIME)
    , last_out_ts_(0, 0, RCL_ROS_TIME)
  {
    connectInput(f0, f1, fs ...);
  }

  explicit InputAligner(const rclcpp::Duration & timeout)
  : timeout_(timeout)
    , last_in_ts_(0, 0, RCL_ROS_TIME)
    , last_out_ts_(0, 0, RCL_ROS_TIME)
  {
  }

  virtual ~InputAligner()
  {
    disconnectAll();
  }

  /**
   * \brief Connects |Fs| inputs.
   * Disconnects existing inputs.
   */
  template<class ... Fs>
  void connectInput(Fs &... fs)
  {
    disconnectAll();

    std::tuple<Fs &...> tuple{fs ...};
    connectInputImpl(tuple, std::make_index_sequence<sizeof...(Fs)>{});
  }

  /**
   * \brief Register a callback for the I-th input.
   * \param callback The callback to call
   */
  template<std::size_t I, typename P>
  Connection registerCallback(const std::function<void(P)> & callback)
  {
    using Message = typename std::tuple_element_t<I, Messages>;
    using Signal = typename std::tuple_element_t<I, Signals>;
    typename CallbackHelper1<Message>::Ptr helper = std::get<I>(signals_).addCallback(callback);
    return Connection(std::bind(&Signal::removeCallback, &std::get<I>(signals_), helper));
  }

  /**
   * \brief Register a callback for the I-th input.
   * \param callback The callback to call
   */
  template<std::size_t I, typename P>
  Connection registerCallback(void (* callback)(P))
  {
    using Message = typename std::tuple_element_t<I, Messages>;
    using Signal = typename std::tuple_element_t<I, Signals>;
    typename CallbackHelper1<Message>::Ptr helper =
      std::get<I>(signals_).template addCallback<P>(std::bind(callback, std::placeholders::_1));
    return Connection(std::bind(&Signal::removeCallback, &std::get<I>(signals_), helper));
  }

  /**
   * \brief Register a callback for the I-th input.
   * \param callback The callback to call
   */
  template<std::size_t I, typename T, typename P>
  Connection registerCallback(void (T::* callback)(P), T * t)
  {
    using Message = typename std::tuple_element_t<I, Messages>;
    using Signal = typename std::tuple_element_t<I, Signals>;
    typename CallbackHelper1<Message>::Ptr helper =
      std::get<I>(signals_).template addCallback<P>(std::bind(callback, t, std::placeholders::_1));
    return Connection(std::bind(&Signal::removeCallback, &std::get<I>(signals_), helper));
  }

  /**
   * \brief Set the name of this filter. For debugging use.
   */
  void setName(const std::string & name) {name_ = name;}

  /**
   * \brief Get the name of this filter. For debugging use.
   */
  const std::string & getName() {return name_;}

  /**
   * \brief Adds a message to the I-th input queue.
   * With this function this filter can also be used without the use of other filter classes.
   */
  template<std::size_t I>
  void add(const std::shared_ptr<typename std::tuple_element_t<I, Messages> const> & msg)
  {
    this->template addEvent<I>(typename std::tuple_element_t<I, Events>(msg));
  }

  /**
   * \brief Sets the input period for the I-th input.
   * This allows the filter to look ahead for the given time and reduces delays.
   * The period should be selected optimistically at the lower bound.
   * The default value is 0, which means there is no look ahead for this input.
   */
  template<std::size_t I>
  void setInputPeriod(const rclcpp::Duration & period)
  {
    std::get<I>(event_queues_).setPeriod(period);
  }

  template<std::size_t I>
  QueueStatus getQueueStatus() const
  {
    return std::get<I>(event_queues_).getStatus();
  }

  /**
   * \brief Sets up a timer calling the `dispatchMessages` function.
   * \param node The ROS node.
   * \param update_rate The rate in which `dispatchMessages` is triggered.
   */
  template<class NodeType>
  void setupDispatchTimer(std::shared_ptr<NodeType> node, const rclcpp::Duration & update_rate)
  {
    dispatch_timer_ = node->create_wall_timer(
      std::chrono::nanoseconds(update_rate.nanoseconds()), [this]() {
        dispatchMessages();
      });
  }

  /**
   * \brief Forwards all messages that can be aligned.
   * This function should be called periodically.
   */
  void dispatchMessages()
  {
    auto index_sequence = std::make_index_sequence<N_INPUTS>{};

    std::lock_guard<std::mutex> lock(mutex_);

    // check if msgs are available
    if (inputsAvailable(index_sequence)) {
      bool input_available = true;
      // iterate event queues as long as there are msgs to dispatch
      while (input_available) {
        input_available = dispatchFirstMessage(index_sequence);
      }
    }
  }

protected:
  template<typename MsgType>
  struct EventSort
  {
    bool operator()(
      const MessageEvent<MsgType const> & lhs, const MessageEvent<MsgType const> & rhs) const
    {
      return message_filters::message_traits::TimeStamp<MsgType>::value(*lhs.getConstMessage()) <
             message_filters::message_traits::TimeStamp<MsgType>::value(*rhs.getConstMessage());
    }
  };

  template<typename MsgType>
  class EventQueue
    : public std::multiset<MessageEvent<MsgType const>,
      EventSort<MsgType>>
  {
public:
    EventQueue()
    : next_ts_(rclcpp::Time::max(RCL_ROS_TIME)), period_(0, 0),
      active_(false), msgs_processed_(0), msgs_dropped_(0) {}

    rclcpp::Time firstTimeStamp()
    {
      if (!this->empty()) {
        rclcpp::Time first_ts =
          message_filters::message_traits::TimeStamp<MsgType>::value(
            *(this->begin()->getConstMessage()));
        next_ts_ = first_ts + period_;
        active_ = true;
        return first_ts;
      } else if (active_) {
        return next_ts_;
      }
      return rclcpp::Time::max(RCL_ROS_TIME);
    }

    void popFirst()
    {
      this->erase(this->begin());
      msgs_processed_++;
    }

    void msgDropped()
    {
      msgs_dropped_++;
    }

    void setPeriod(const rclcpp::Duration & period)
    {
      period_ = period;
    }

    void setActive(bool active)
    {
      active_ = active;
    }

    QueueStatus getStatus() const
    {
      return QueueStatus{active_, this->size(), msgs_processed_, msgs_dropped_};
    }

protected:
    rclcpp::Time next_ts_;
    rclcpp::Duration period_;
    bool active_;
    std::size_t msgs_processed_;
    std::size_t msgs_dropped_;
  };

  typedef std::tuple<EventQueue<Ms>...> EventQueues;

  template<std::size_t I, class FTuple>
  void connect(FTuple & ftuple)
  {
    using MEvent = typename std::tuple_element_t<I, Events>;
    input_connections_[I] =
      std::get<I>(ftuple).registerCallback(
      std::function<void(const MEvent &)>(
        std::bind(
          &InputAligner::template addEvent<I>, this, std::placeholders::_1)));
  }

  template<class FTuple, std::size_t... Is>
  void connectInputImpl(FTuple & ftuple, std::index_sequence<Is...>)
  {
    (connect<Is>(ftuple), ...);
  }

  void disconnectAll()
  {
    for (auto & input_connection : input_connections_) {
      input_connection.disconnect();
    }
  }

  template<std::size_t I>
  void addEvent(const typename std::tuple_element_t<I, Events> & evt)
  {
    using Message = typename std::tuple_element_t<I, Messages>;

    rclcpp::Time msg_timestamp =
      message_filters::message_traits::TimeStamp<Message>::value(*evt.getConstMessage());

    std::lock_guard<std::mutex> lock(mutex_);
    auto & event_queue = std::get<I>(event_queues_);

    // evaluate age of msg
    if (msg_timestamp < last_out_ts_) {
      event_queue.msgDropped();
      RCUTILS_LOG_WARN("Messages of type %ld arrived too late and will be dropped", I);
      return;
    }

    // set latest ts
    if (msg_timestamp > last_in_ts_) {
      last_in_ts_ = msg_timestamp;
    }

    // add new event to queue
    event_queue.insert(evt);
  }

  template<std::size_t... Is>
  bool inputsAvailable(std::index_sequence<Is...>) const
  {
    /* *INDENT-OFF* */
    // uncrustify messes with the brackets which are required (at least by GCC)
    return (!std::get<Is>(event_queues_).empty() || ...);
    /* *INDENT-ON* */
  }

  template<std::size_t... Is>
  bool dispatchFirstMessage(std::index_sequence<Is...>)
  {
    // get index of first sample
    const std::array<rclcpp::Time, sizeof...(Is)> ts_array =
    {std::get<Is>(event_queues_).firstTimeStamp()...};
    const std::size_t idx = getFirstSampleIdx(ts_array);

    // forward first message
    /* *INDENT-OFF* */
    // uncrustify messes with the brackets which are required (at least by GCC)
    return (dispatchFirstMessage<Is>(idx) || ...);
    /* *INDENT-ON* */
  }

  template<std::size_t I>
  bool dispatchFirstMessage(std::size_t idx)
  {
    using MEvent = typename std::tuple_element_t<I, Events>;
    using Message = typename std::tuple_element_t<I, Messages>;

    if (idx == I) {
      auto & event_queue = std::get<I>(event_queues_);
      if (!event_queue.empty()) {
        // dispatch first event
        const MEvent & evt = *event_queue.begin();
        last_out_ts_ =
          message_filters::message_traits::TimeStamp<Message>::value(*evt.getConstMessage());
        std::get<I>(signals_).call(evt);
        event_queue.popFirst();
        return true;
      } else if ((last_in_ts_ - event_queue.firstTimeStamp()) >= timeout_) {
        // timeout exceeded
        event_queue.setActive(false);
        return true;
      }
    }
    // timeout not exceeded or queue not selected
    return false;
  }

  template<std::size_t I>
  std::size_t getFirstSampleIdx(const std::array<rclcpp::Time, I> & ts) const
  {
    return std::distance(ts.begin(), std::min_element(ts.begin(), ts.end()));
  }

  rclcpp::Duration timeout_;
  rclcpp::Time last_in_ts_;
  rclcpp::Time last_out_ts_;

  EventQueues event_queues_;
  Connection input_connections_[N_INPUTS];
  Signals signals_;
  std::string name_;
  std::mutex mutex_;
  rclcpp::TimerBase::SharedPtr dispatch_timer_;
};

}  // namespace message_filters

#endif  // MESSAGE_FILTERS__INPUT_ALIGNER_HPP_
