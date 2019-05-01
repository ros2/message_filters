
/*
 * Copyright (C) 2010, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// File imported from
// https://github.com/ros/roscpp_core/blob/38b9663/roscpp_traits/include/ros/message_event.h

#ifndef MESSAGE_FILTERS__MESSAGE_EVENT_H_
#define MESSAGE_FILTERS__MESSAGE_EVENT_H_

#include <type_traits>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#ifndef RCUTILS_ASSERT
// TODO(tfoote) remove this after it's implemented upstream
// https://github.com/ros2/rcutils/pull/112
#define RCUTILS_ASSERT assert
#endif
// Uncomment below intead
//#include <rcutils/assert.h>

namespace message_filters
{
typedef std::map< std::string, std::string > 	M_string;
typedef std::shared_ptr< M_string > 	M_stringPtr;

template<typename M>
struct DefaultMessageCreator
{
  std::shared_ptr<M> operator()()
  {
    return std::make_shared<M>();
  }
};

/*
template<typename M>
ROS_DEPRECATED inline std::shared_ptr<M> defaultMessageCreateFunction()
{
  return DefaultMessageCreator<M>()();
}
*/
/**
 * \brief Event type for subscriptions, const message_filters::MessageEvent<M const>& can be used in your callback instead of const std::shared_ptr<M const>&
 *
 * Useful if you need to retrieve meta-data about the message, such as the full connection header, or the publisher's node name
 */
template<typename M>
class MessageEvent
{
public:
  typedef typename std::add_const<M>::type ConstMessage;
  typedef typename std::remove_const<M>::type Message;
  typedef std::shared_ptr<Message> MessagePtr;
  typedef std::shared_ptr<ConstMessage> ConstMessagePtr;
  typedef std::function<MessagePtr()> CreateFunction;

  MessageEvent()
  : nonconst_need_copy_(true)
  {}

  MessageEvent(const MessageEvent<Message>& rhs)
  {
    *this = rhs;
  }

  MessageEvent(const MessageEvent<ConstMessage>& rhs)
  {
    *this = rhs;
  }

  MessageEvent(const MessageEvent<Message>& rhs, bool nonconst_need_copy)
  {
    *this = rhs;
    nonconst_need_copy_ = nonconst_need_copy;
  }

  MessageEvent(const MessageEvent<ConstMessage>& rhs, bool nonconst_need_copy)
  {
    *this = rhs;
    nonconst_need_copy_ = nonconst_need_copy;
  }

  MessageEvent(const MessageEvent<void const>& rhs, const CreateFunction& create)
  {
    init(std::const_pointer_cast<Message>(std::static_pointer_cast<ConstMessage>(rhs.getMessage())), rhs.getReceiptTime(), rhs.nonConstWillCopy(), create);
  }

  /**
   * \todo Make this explicit in ROS 2.0.  Keep as auto-converting for now to maintain backwards compatibility in some places (message_filters)
   */
  MessageEvent(const ConstMessagePtr& message)
  {
    init(message, rclcpp::Clock().now(), true, message_filters::DefaultMessageCreator<Message>());
  }

  MessageEvent(const ConstMessagePtr& message, rclcpp::Time receipt_time)
  {
    init(message, receipt_time, true, message_filters::DefaultMessageCreator<Message>());
  }

  MessageEvent(const ConstMessagePtr& message, rclcpp::Time receipt_time, bool nonconst_need_copy, const CreateFunction& create)
  {
    init(message, receipt_time, nonconst_need_copy, create);
  }

  void init(const ConstMessagePtr& message, rclcpp::Time receipt_time, bool nonconst_need_copy, const CreateFunction& create)
  {
    message_ = message;
    receipt_time_ = receipt_time;
    nonconst_need_copy_ = nonconst_need_copy;
    create_ = create;
  }

  void operator=(const MessageEvent<Message>& rhs)
  {
    init(std::static_pointer_cast<Message>(rhs.getMessage()), rhs.getReceiptTime(), rhs.nonConstWillCopy(), rhs.getMessageFactory());
    message_copy_.reset();
  }

  void operator=(const MessageEvent<ConstMessage>& rhs)
  {
    init(std::const_pointer_cast<Message>(std::static_pointer_cast<ConstMessage>(rhs.getMessage())), rhs.getReceiptTime(), rhs.nonConstWillCopy(), rhs.getMessageFactory());
    message_copy_.reset();
  }

  /**
   * \brief Retrieve the message.  If M is const, this returns a reference to it.  If M is non const
   * and this event requires it, returns a copy.  Note that it caches this copy for later use, so it will
   * only every make the copy once
   */
  std::shared_ptr<M> getMessage() const
  {
    return copyMessageIfNecessary<M>();
  }

  /**
   * \brief Retrieve a const version of the message
   */
  const std::shared_ptr<ConstMessage>& getConstMessage() const { return message_; }

  /**
   * \brief Returns the time at which this message was received
   */
  rclcpp::Time getReceiptTime() const { return receipt_time_; }

  bool nonConstWillCopy() const { return nonconst_need_copy_; }
  bool getMessageWillCopy() const { return !std::is_const<M>::value && nonconst_need_copy_; }

  bool operator<(const MessageEvent<M>& rhs)
  {
    if (message_ != rhs.message_)
    {
      return message_ < rhs.message_;
    }

    if (receipt_time_ != rhs.receipt_time_)
    {
      return receipt_time_ < rhs.receipt_time_;
    }

    return nonconst_need_copy_ < rhs.nonconst_need_copy_;
  }

  bool operator==(const MessageEvent<M>& rhs)
  {
    return message_ == rhs.message_ && receipt_time_ == rhs.receipt_time_ && nonconst_need_copy_ == rhs.nonconst_need_copy_;
  }

  bool operator!=(const MessageEvent<M>& rhs)
  {
    return !(*this == rhs);
  }

  const CreateFunction& getMessageFactory() const { return create_; }

private:
  template<typename M2>
  typename std::enable_if<!std::is_void<M2>::value, std::shared_ptr<M> >::type copyMessageIfNecessary() const
  {
    if (std::is_const<M>::value || !nonconst_need_copy_)
    {
      return std::const_pointer_cast<Message>(message_);
    }

    if (message_copy_)
    {
      return message_copy_;
    }

    RCUTILS_ASSERT(create_);
    message_copy_ = create_();
    *message_copy_ = *message_;

    return message_copy_;
  }

  template<typename M2>
  typename std::enable_if<std::is_void<M2>::value, std::shared_ptr<M> >::type copyMessageIfNecessary() const
  {
    return std::const_pointer_cast<Message>(message_);
  }

  ConstMessagePtr message_;
  // Kind of ugly to make this mutable, but it means we can pass a const MessageEvent to a callback and not worry about other things being modified
  mutable MessagePtr message_copy_;
  rclcpp::Time receipt_time_;
  bool nonconst_need_copy_;
  CreateFunction create_;

  static const std::string s_unknown_publisher_string_;
};

template<typename M> const std::string MessageEvent<M>::s_unknown_publisher_string_("unknown_publisher");

}  // namespace message_filters

#endif  // MESSAGE_FILTERS__MESSAGE_EVENT_H_
