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

#ifndef MESSAGE_FILTERS__SYNCHRONIZER_HPP_
#define MESSAGE_FILTERS__SYNCHRONIZER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <tuple>
#include <type_traits>

#include "message_filters/connection.hpp"
#include "message_filters/null_types.hpp"
#include "message_filters/message_event.hpp"
#include "message_filters/signal9.hpp"

namespace message_filters
{

template<class Policy>
class Synchronizer : public noncopyable, public Policy
{
public:
  using Messages = typename Policy::Messages;
  using Events = typename Policy::Events;
  using Signal = typename Policy::Signal;

  template<class F0, class F1, class ... Fs>
  Synchronizer(F0 & f0, F1 & f1, Fs &... fs)
  {
    connectInput(f0, f1, fs ...);
    init();
  }

  Synchronizer()
  {
    init();
  }

  template<class ... Fs>
  Synchronizer(const Policy & policy, Fs &... fs)
  : Policy(policy)
  {
    connectInput(fs ...);
    init();
  }

  Synchronizer(const Policy & policy)  // NOLINT(runtime/explicit)
  : Policy(policy)
  {
    init();
  }

  ~Synchronizer()
  {
    disconnectAll();
  }

  void init()
  {
    Policy::initParent(this);
  }

  template<std::size_t I, class FTuple>
  void connect(FTuple & ftuple)
  {
    using MEvent = std::tuple_element_t<I, Events>;
    input_connections_[I] =
      std::get<I>(ftuple).registerCallback(
      std::function<void(const MEvent &)>(
        [this](const MEvent & evt) {this->template cb<I>(evt);}));
  }

  template<class FTuple, std::size_t... Is>
  void connectInputImpl(FTuple & ftuple, std::index_sequence<Is...>)
  {
    (connect<Is>(ftuple), ...);
  }

  template<class ... Fs>
  void connectInput(Fs &... fs)
  {
    disconnectAll();

    std::tuple<Fs &...> tuple{fs ...};
    connectInputImpl(tuple, std::make_index_sequence<sizeof...(Fs)>{});
  }

  template<class C>
  Connection registerCallback(C & callback)
  {
    return signal_.addCallback(callback);
  }

  template<class C>
  Connection registerCallback(const C & callback)
  {
    return signal_.addCallback(callback);
  }

  template<class C, typename T>
  Connection registerCallback(const C & callback, T * t)
  {
    return signal_.addCallback(callback, t);
  }

  template<class C, typename T>
  Connection registerCallback(C & callback, T * t)
  {
    return signal_.addCallback(callback, t);
  }

  void setName(const std::string & name) {name_ = name;}
  const std::string & getName() {return name_;}


  template<class ... MEvent>
  void signal(const MEvent &... es)
  {
    signal_.call(es ...);
  }

  Policy * getPolicy() {return static_cast<Policy *>(this);}

  using Policy::add;

  template<int i>
  void add(const std::shared_ptr<std::tuple_element_t<i, Messages> const> & msg)
  {
    this->template add<i>(std::tuple_element_t<i, Events>(msg));
  }

private:
  void disconnectAll()
  {
    for (auto & input_connection : input_connections_) {
      input_connection.disconnect();
    }
  }

  template<int i>
  void cb(const std::tuple_element_t<i, Events> & evt)
  {
    this->template add<i>(evt);
  }

  Signal signal_;

  Connection input_connections_[Policy::N_MESSAGES];

  std::string name_;
};

template<typename ... Ms>
struct PolicyBase
{
  static constexpr std::size_t N_MESSAGES = sizeof...(Ms);
  using RealTypeCount = std::integral_constant<int, N_MESSAGES>;
  using Messages = std::tuple<Ms...>;
  using Signal = Signal9<Ms...>;
  using Events = std::tuple<MessageEvent<Ms const>...>;
};

}  // namespace message_filters

#endif  // MESSAGE_FILTERS__SYNCHRONIZER_HPP_
