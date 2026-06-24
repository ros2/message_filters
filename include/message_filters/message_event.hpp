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

// File imported from
// https://github.com/ros/roscpp_core/blob/38b9663/roscpp_traits/include/ros/message_event.h

#ifndef MESSAGE_FILTERS__MESSAGE_EVENT_HPP_
#define MESSAGE_FILTERS__MESSAGE_EVENT_HPP_

#include <cassert>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <type_traits>

#include <rclcpp/time.hpp>

#include "message_filters/visibility_control.hpp"

namespace message_filters
{
/**
 * Defined in message_event.cpp so that this widely-included header does not need
 * rclcpp/clock.hpp, which transitively pulls in the entire rcl/rmw C API.
 */
MESSAGE_FILTERS_PUBLIC rclcpp::Time systemClockNow();

using M_string = std::map<std::string, std::string>;
using M_stringPtr = std::shared_ptr<M_string>;

template<typename M>
struct DefaultMessageCreator
{
  std::shared_ptr<M> operator()()
  {
    return std::make_shared<M>();
  }
};


/**
 * \brief Event type for subscriptions, const message_filters::MessageEvent<M const> & can be used in your callback instead of const std::shared_ptr<M const>&
 *
 * Useful if you need to retrieve meta-data about the message, such as the full connection header, or the publisher's node name
 */
template<typename M>
class MessageEvent
{
public:
  using ConstMessage = std::add_const_t<M>;
  using Message = std::remove_const_t<M>;
  using MessagePtr = std::shared_ptr<Message>;
  using ConstMessagePtr = std::shared_ptr<ConstMessage>;
  using CreateFunction = std::function<MessagePtr()>;

  MessageEvent()
  : nonconst_need_copy_(true)
  {}

  MessageEvent(const MessageEvent<Message> & rhs)
  {
    *this = rhs;
  }

  MessageEvent(const MessageEvent<ConstMessage> & rhs)
  {
    *this = rhs;
  }

  MessageEvent(const MessageEvent<Message> & rhs, bool nonconst_need_copy)
  {
    *this = rhs;
    nonconst_need_copy_ = nonconst_need_copy;
  }

  MessageEvent(const MessageEvent<ConstMessage> & rhs, bool nonconst_need_copy)
  {
    *this = rhs;
    nonconst_need_copy_ = nonconst_need_copy;
  }

  MessageEvent(const MessageEvent<void const> & rhs, const CreateFunction & create)
  {
    init(
      std::const_pointer_cast<Message>(
        std::static_pointer_cast<ConstMessage>(
          rhs.getMessage())), rhs.getReceiptTime(), rhs.nonConstWillCopy(), create);
  }

  /**
   * \todo Make this explicit in ROS 2.0.  Keep as auto-converting for now to maintain backwards compatibility in some places (message_filters)
   */
  MessageEvent(const ConstMessagePtr & message)  // NOLINT(runtime/explicit)
  {
    init(
      message, message_filters::systemClockNow(), true,
      message_filters::DefaultMessageCreator<Message>());
  }

  MessageEvent(const ConstMessagePtr & message, rclcpp::Time receipt_time)
  {
    init(message, receipt_time, true, message_filters::DefaultMessageCreator<Message>());
  }

  MessageEvent(
    const ConstMessagePtr & message, rclcpp::Time receipt_time, bool nonconst_need_copy,
    const CreateFunction & create)
  {
    init(message, receipt_time, nonconst_need_copy, create);
  }

  void init(
    const ConstMessagePtr & message, rclcpp::Time receipt_time, bool nonconst_need_copy,
    const CreateFunction & create)
  {
    message_ = message;
    receipt_time_ = receipt_time;
    nonconst_need_copy_ = nonconst_need_copy;
    create_ = create;
  }

  void operator=(const MessageEvent<Message> & rhs)
  {
    init(
      std::static_pointer_cast<Message>(rhs.getMessage()),
      rhs.getReceiptTime(), rhs.nonConstWillCopy(), rhs.getMessageFactory());
    message_copy_.reset();
  }

  void operator=(const MessageEvent<ConstMessage> & rhs)
  {
    init(
      std::const_pointer_cast<Message>(
        std::static_pointer_cast<ConstMessage>(
          rhs.getMessage())), rhs.getReceiptTime(), rhs.nonConstWillCopy(),
      rhs.getMessageFactory());
    message_copy_.reset();
  }

  /**
   * \brief Retrieve the message.  If M is const, this returns a reference to it.  If M is non const
   * and this event requires it, returns a copy.  Note that it caches this copy for later use, so it will
   * only every make the copy once
   */
  std::shared_ptr<M> getMessage() const
  {
    if constexpr (std::is_void_v<M>) {
      return std::const_pointer_cast<Message>(message_);
    } else {
      if (std::is_const_v<M>|| !nonconst_need_copy_) {
        return std::const_pointer_cast<Message>(message_);
      }

      if (message_copy_) {
        return message_copy_;
      }

      assert(create_);
      message_copy_ = create_();
      *message_copy_ = *message_;

      return message_copy_;
    }
  }

  /**
   * \brief Retrieve a const version of the message
   */
  [[nodiscard]] const std::shared_ptr<ConstMessage> & getConstMessage() const {return message_;}

  /**
   * \brief Returns the time at which this message was received
   */
  [[nodiscard]] rclcpp::Time getReceiptTime() const {return receipt_time_;}

  [[nodiscard]] bool nonConstWillCopy() const {return nonconst_need_copy_;}
  [[nodiscard]] bool getMessageWillCopy() const {return !std::is_const_v<M>&& nonconst_need_copy_;}

  // Note: not noexcept. rclcpp::Time relational operators throw std::runtime_error
  // when the two times have different clock sources.
  bool operator<(const MessageEvent<M> & rhs) const
  {
    if (message_ != rhs.message_) {
      return message_ < rhs.message_;
    }

    if (receipt_time_ != rhs.receipt_time_) {
      return receipt_time_ < rhs.receipt_time_;
    }

    return nonconst_need_copy_ < rhs.nonconst_need_copy_;
  }

  // operator!= is synthesized from operator== by C++20's rewritten candidates.
  bool operator==(const MessageEvent<M> & rhs) const
  {
    return message_ == rhs.message_ && receipt_time_ == rhs.receipt_time_ &&
           nonconst_need_copy_ == rhs.nonconst_need_copy_;
  }

  [[nodiscard]] const CreateFunction & getMessageFactory() const {return create_;}

private:
  ConstMessagePtr message_;
  // Kind of ugly to make this mutable, but it means we can pass a const MessageEvent
  // to a callback and not worry about other things being modified
  mutable MessagePtr message_copy_;
  rclcpp::Time receipt_time_;
  bool nonconst_need_copy_;
  CreateFunction create_;
};

}  // namespace message_filters

#endif  // MESSAGE_FILTERS__MESSAGE_EVENT_HPP_
