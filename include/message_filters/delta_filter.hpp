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

#ifndef MESSAGE_FILTERS__DELTA_FILTER_HPP_
#define MESSAGE_FILTERS__DELTA_FILTER_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <forward_list>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <type_traits>
#include <variant>

#include <message_filters/connection.hpp>
#include <message_filters/message_event.hpp>
#include <message_filters/simple_filter.hpp>

namespace message_filters
{

// Supported final field types
typedef std::variant<
    bool,
    std::byte,
    char,
    std::uint8_t,
    std::int16_t, std::uint16_t,
    std::int32_t, std::uint32_t,
    std::int64_t, std::uint64_t,
    float,
    double,
    std::string
> MFieldType;


/**
 * \brief Base class for comparison handlers.
 *
 * Provides class interface but nothing more.
 */
template<typename MessageType>
class ComparisonHandler
{
public:
  typedef std::shared_ptr<MessageType const> MConstPtr;
  typedef MessageEvent<MessageType const> EventType;
  typedef std::function<MFieldType(const MConstPtr &)> FieldGetterFunctionType;

  virtual ~ComparisonHandler() = default;

  virtual bool message_fits(const EventType & message) = 0;

  virtual bool do_fields_fit(MFieldType field_a, MFieldType field_b) const = 0;
};

/**
 * \brief CachedComparisonHandler implements messages comparison field by field.
 *
 * A successor to this class should implement 'do_fields_fit' method.
 * If any of the fields provided to the 'do_fields_fit' method do satisfy
 * a comparison conditions the message is accepted. That means that the message
 * is stored in the 'message_cache_' and the 'message_fits' method returns 'true'.
 */
template<typename MessageType>
class CachedComparisonHandler : public ComparisonHandler<MessageType>
{
public:
  typedef std::shared_ptr<MessageType const> MConstPtr;
  typedef MessageEvent<MessageType const> EventType;
  typedef std::function<MFieldType(const MConstPtr &)> FieldGetterFunctionType;

  /**
   * Initialize handler.
   * \param field_getters A list of callable objects that are expected
   * to retrieve a value of a basic type from a message field for comparison.
   * Any of the 'field_getters' is applied to the cached and to the current
   * message. The returned values are compared. If any of these values differ
   * between cached and current message, the current message is accepted.
   */
  explicit CachedComparisonHandler(
    std::forward_list<FieldGetterFunctionType> field_getters
  )
  : ComparisonHandler<MessageType>(),
    field_getters_(field_getters)
  {}

  virtual ~CachedComparisonHandler() = default;

  bool message_fits(const EventType & message)
  {
    std::lock_guard<std::mutex> lock(message_cache_mutex_);

    if (!message_cache_.has_value()) {
      message_cache_.emplace(message);
      return true;
    }

    const bool any_field_fits = std::any_of(
      field_getters_.begin(),
      field_getters_.end(),
      [&] (FieldGetterFunctionType get_field)
      {
        return do_fields_fit(
          get_field(message_cache_.value().getConstMessage()),
          get_field(message.getConstMessage())
        );
      }
    );

    if (any_field_fits) {
      message_cache_.emplace(message);
    }
    return any_field_fits;
  }

  virtual bool do_fields_fit(MFieldType field_a, MFieldType field_b) const = 0;

private:
  std::mutex message_cache_mutex_;

  std::optional<const EventType> message_cache_;

  std::forward_list<FieldGetterFunctionType> field_getters_;
};

/**
 * \brief DeltaCompare implements delta comparison.
 *
 * If any of the fields, acquired by any of the 'field_getters' do differ
 * between previously cached and current message, the current message is accepted.
 */
template<typename MessageType>
class DeltaCompare : public CachedComparisonHandler<MessageType>
{
public:
  typedef std::shared_ptr<MessageType const> MConstPtr;
  typedef std::function<MFieldType(const MConstPtr &)> FieldGetterFunctionType;

  /**
   * Initialize handler.
   *
   * \param field_getters A list of callable objects that are expected
   * to retrieve a value of a basic type from a message field for comparison.
   * Any of the 'field_getters' is applied to the cached and to the current
   * message. The returned values are compared. If any of these values differ
   * between cached and current message, the current message is accepted.
   */
  explicit DeltaCompare(
    std::forward_list<FieldGetterFunctionType> field_getters
  )
  : CachedComparisonHandler<MessageType>(field_getters)
  {}

  virtual ~DeltaCompare() = default;

  bool do_fields_fit(MFieldType field_a, MFieldType field_b) const
  {
    return field_a != field_b;
  }
};

/**
 * \brief ROS 2 Comparison filter.
 *
 * Given a stream of messages, the message is passed down to the next filter
 * if 'comparison_handler' 'message_fits' method returns 'true' for that message.
 *
 * \param HandlerType<HandlerMessageType> [template] Specification
 * of the CachedComparisonHandler successor class.
 * Is expected implement a ``message_fits`` method. If ``message_fits`` returns ``True``
 * for a provided message, that message is considered valid and is passed
 * to a next filter if any.
 */
template<typename MessageType, template<typename HandlerMessageType> typename HandlerType>
class ComparisonFilter : public SimpleFilter<MessageType>
{
public:
  typedef std::shared_ptr<MessageType const> MConstPtr;
  typedef MessageEvent<MessageType const> EventType;
  typedef std::function<MFieldType(const MConstPtr &)> FieldGetterFunctionType;

  /**
   * Initialize ComparisonFilter.
   *
   * \param filter An instance of the 'SimpleFilter' successor class.
   * The input filter to connect to.
   *
   * \param field_getters A list of callable objects that are expected
   * to retrieve a value of a basic type from a message field for comparison.
   * Any of the 'field_getters' is applied to the cached and to the current
   * message. The returned values are compared. If any of these values differ
   * between cached and current message, the current message is accepted.
   */
  template<typename FilterType>
  explicit ComparisonFilter(
    FilterType & filter,
    std::forward_list<FieldGetterFunctionType> field_getters
  )
  :SimpleFilter<MessageType>(),
    comparison_handler_(field_getters)
  {
    connectInput(filter);
  }

  virtual ~ComparisonFilter() = default;

  template<typename FilterType>
  void connectInput(FilterType & filter)
  {
    incoming_connection_.disconnect();
    incoming_connection_ = filter.registerCallback(
      typename SimpleFilter<MessageType>::EventCallback(
        std::bind(
          &ComparisonFilter::add,
          this,
          std::placeholders::_1
        )
      )
    );
  }

  void add(const EventType & evt)
  {
    if (comparison_handler_.message_fits(evt)) {
      this->signalMessage(evt);
    }
  }

private:
  Connection incoming_connection_;

  HandlerType<MessageType> comparison_handler_;
};


/**
 * \brief ROS 2 Delta filter.
 *
 * Given a stream of messages, the message is passed down to the next filter
 * if any of the message fields, that may be acquired by ``field_getters``
 * have changed compared to the previously accepted message.
 */
template<typename MessageType>
class DeltaFilter : public ComparisonFilter<MessageType, DeltaCompare>
{
public:
  typedef std::shared_ptr<MessageType const> MConstPtr;
  typedef std::function<MFieldType(const MConstPtr &)> FieldGetterFunctionType;

  /**
   * \brief Initialize DeltaFilter
   *
   * \param filter An instance of the 'SimpleFilter' successor class.
   * The input filter to connect to.
   *
   * \param field_getters A list of callable objects that are expected
   * to retrieve a value of a basic type from a message field for comparison.
   * Any of the 'field_getters' is applied to the cached and to the current
   * message. The returned values are compared. If any of these values differ
   * between cached and current message, the current message is accepted.
   */
  template<typename FilterType>
  explicit DeltaFilter(
    FilterType & filter,
    std::forward_list<FieldGetterFunctionType> field_getters
  )
  : ComparisonFilter<MessageType, DeltaCompare>(
      filter,
      field_getters
  ) {}

  virtual ~DeltaFilter() = default;
};

}   // namespace message_filters

#endif  // MESSAGE_FILTERS__DELTA_FILTER_HPP_
