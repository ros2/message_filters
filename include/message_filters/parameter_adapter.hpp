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

// Imported from
// https://github.com/ros/ros_comm/blob/a2ca01ecf31d677dd5c1b08cc6852bd0dac02b97/clients/roscpp/include/ros/parameter_adapter.h

#ifndef MESSAGE_FILTERS__PARAMETER_ADAPTER_HPP_
#define MESSAGE_FILTERS__PARAMETER_ADAPTER_HPP_

#include <memory>
#include <type_traits>

#include "message_filters/message_event.hpp"

namespace message_filters
{

/**
 * \brief Generally not for outside use.  Adapts a function parameter type into the message type, event type and parameter.  Allows you to
 * retrieve a parameter type from an event type.
 *
 * ParameterAdapter is generally only useful for outside use when implementing things that require message callbacks
 * (such as the message_filters package)and you would like to support all the rclcpp message parameter types
 *
 * The ParameterAdapter is templated on the callback parameter type (\b not the bare message type), and provides 3 things:
 *  - Message typedef, which provides the bare message type, no const or reference qualifiers
 *  - Event typedef, which provides the message_filters::MessageEvent type
 *  - Parameter typedef, which provides the actual parameter type (may be slightly different from M)
 *  - static getParameter(event) function, which returns a parameter type given the event
 *  - static bool is_const informs you whether or not the parameter type is a const message
 *
 *  ParameterAdapter is specialized to allow callbacks of any of the forms:
\verbatim
void callback(const std::shared_ptr<M const> &);
void callback(const std::shared_ptr<M> &);
void callback(std::shared_ptr<M const>);
void callback(std::shared_ptr<M>);
void callback(const M &);
void callback(M);
void callback(const MessageEvent<M const> &);
void callback(const MessageEvent<M> &);
\endverbatim
 */
template<typename M>
struct ParameterAdapter
{
  using Message = std::remove_reference_t<std::remove_const_t<M>>;
  using Event = MessageEvent<Message const>;
  using Parameter = M;
  static const bool is_const = true;

  static Parameter getParameter(const Event & event)
  {
    return *event.getMessage();
  }
};

template<typename M>
struct ParameterAdapter<const std::shared_ptr<M const> &>
{
  using Message = std::remove_reference_t<std::remove_const_t<M>>;
  using Event = MessageEvent<Message const>;
  using Parameter = const std::shared_ptr<Message const>;
  static const bool is_const = true;

  static Parameter getParameter(const Event & event)
  {
    return event.getMessage();
  }
};

template<typename M>
struct ParameterAdapter<const std::shared_ptr<M> &>
{
  using Message = std::remove_reference_t<std::remove_const_t<M>>;
  using Event = MessageEvent<Message const>;
  using Parameter = std::shared_ptr<Message>;
  static const bool is_const = false;

  static Parameter getParameter(const Event & event)
  {
    return MessageEvent<Message>(event).getMessage();
  }
};

template<typename M>
struct ParameterAdapter<const M &>
{
  using Message = std::remove_reference_t<std::remove_const_t<M>>;
  using Event = MessageEvent<Message const>;
  using Parameter = const M &;
  static const bool is_const = true;

  static Parameter getParameter(const Event & event)
  {
    return *event.getMessage();
  }
};

template<typename M>
struct ParameterAdapter<std::shared_ptr<M const>>
{
  using Message = std::remove_reference_t<std::remove_const_t<M>>;
  using Event = MessageEvent<Message const>;
  using Parameter = std::shared_ptr<Message const>;
  static const bool is_const = true;

  static Parameter getParameter(const Event & event)
  {
    return event.getMessage();
  }
};

template<typename M>
struct ParameterAdapter<std::shared_ptr<M>>
{
  using Message = std::remove_reference_t<std::remove_const_t<M>>;
  using Event = MessageEvent<Message const>;
  using Parameter = std::shared_ptr<Message>;
  static const bool is_const = false;

  static Parameter getParameter(const Event & event)
  {
    return MessageEvent<Message>(event).getMessage();
  }
};

template<typename M>
struct ParameterAdapter<const MessageEvent<M const> &>
{
  using Message = std::remove_reference_t<std::remove_const_t<M>>;
  using Event = MessageEvent<Message const>;
  using Parameter = const MessageEvent<Message const> &;
  static const bool is_const = true;

  static Parameter getParameter(const Event & event)
  {
    return event;
  }
};

template<typename M>
struct ParameterAdapter<const MessageEvent<M> &>
{
  using Message = std::remove_reference_t<std::remove_const_t<M>>;
  using Event = MessageEvent<Message const>;
  using Parameter = MessageEvent<Message>;
  static const bool is_const = false;

  static Parameter getParameter(const Event & event)
  {
    return MessageEvent<Message>(event);
  }
};

}  // namespace message_filters

#endif  // MESSAGE_FILTERS__PARAMETER_ADAPTER_HPP_
