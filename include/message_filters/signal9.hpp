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

#ifndef MESSAGE_FILTERS__SIGNAL9_HPP_
#define MESSAGE_FILTERS__SIGNAL9_HPP_


#include <functional>
#include <mutex>
#include <memory>
#include <vector>

#include "message_filters/connection.hpp"
#include "message_filters/null_types.hpp"
#include "message_filters/message_event.hpp"
#include "message_filters/parameter_adapter.hpp"

namespace message_filters
{

template<typename ... Ms>
class CallbackHelper9
{
public:
  virtual ~CallbackHelper9() {}

  virtual void call(bool nonconst_force_copy, const MessageEvent<Ms const> & ... es) = 0;

  typedef std::shared_ptr<CallbackHelper9> Ptr;
};

template<typename ... Ps>
class CallbackHelper9T
  : public CallbackHelper9<typename ParameterAdapter<Ps>::Message...>
{
public:
  typedef std::function<void (typename ParameterAdapter<Ps>::Parameter...)> Callback;

  CallbackHelper9T(const Callback & cb)  // NOLINT(runtime/explicit)
  : callback_(cb)
  {
  }

  virtual void call(bool nonconst_force_copy, const typename ParameterAdapter<Ps>::Event &... es)
  {
    auto my_es{std::make_tuple(
        typename ParameterAdapter<Ps>::Event(
          es,
          nonconst_force_copy || es.nonConstWillCopy())...)};
    callback_(ParameterAdapter<Ps>::getParameter(es)...);
  }

private:
  Callback callback_;
};

template<typename ... Ms>
class Signal9
{
  typedef std::shared_ptr<CallbackHelper9<Ms...>> CallbackHelper9Ptr;
  typedef std::vector<CallbackHelper9Ptr> V_CallbackHelper9;

public:
  typedef const std::shared_ptr<NullType const> & NullP;

  template<typename ... Ps>
  Connection addCallback(const std::function<void(Ps...)> & callback)
  {
    CallbackHelper9T<Ps...> * helper = new CallbackHelper9T<Ps...>(callback);

    std::lock_guard<std::mutex> lock(mutex_);
    callbacks_.push_back(CallbackHelper9Ptr(helper));
    return Connection(std::bind(&Signal9::removeCallback, this, callbacks_.back()));
  }

  template<typename ... Ps>
  Connection addCallback(void (* callback)(Ps...))
  {
    return addCallback(
      std::function<void(Ps...)>(
        [callback](auto &&... args) {
          callback(args ...);
        }));
  }

  template<typename T, typename ... Ps>
  Connection addCallback(void (T::* callback)(Ps...), T * t)
  {
    return addCallback(
      std::function<void(Ps...)>(
        [ = ](const Ps &... ps) {
          (t->*callback)(ps ...);
        }));
  }

  template<typename C>
  Connection addCallback(C & callback)
  {
    return addCallback(
      std::function<void(const std::shared_ptr<const Ms> & ...)>(
        [callback](const std::shared_ptr<const Ms> & ... msgs) {callback(msgs ...);}));
  }

  void removeCallback(const CallbackHelper9Ptr & helper)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    typename V_CallbackHelper9::iterator it =
      std::find(callbacks_.begin(), callbacks_.end(), helper);
    if (it != callbacks_.end()) {
      callbacks_.erase(it);
    }
  }

  void call(const MessageEvent<Ms const> & ... es)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    bool nonconst_force_copy = callbacks_.size() > 1;
    typename V_CallbackHelper9::iterator it = callbacks_.begin();
    typename V_CallbackHelper9::iterator end = callbacks_.end();
    for (; it != end; ++it) {
      const CallbackHelper9Ptr & helper = *it;
      helper->call(nonconst_force_copy, es ...);
    }
  }

private:
  std::mutex mutex_;
  V_CallbackHelper9 callbacks_;
};

}  // namespace message_filters

#endif  // MESSAGE_FILTERS__SIGNAL9_HPP_
