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


#ifndef MESSAGE_FILTERS__TIME_SYNCHRONIZER_HPP_
#define MESSAGE_FILTERS__TIME_SYNCHRONIZER_HPP_

#include <memory>

#include "message_filters/message_event.hpp"
#include "message_filters/synchronizer.hpp"
#include "message_filters/sync_policies/exact_time.hpp"

namespace message_filters
{

/**
 * \brief Synchronizes up to N messages by their timestamps.
 *
 * TimeSynchronizer synchronizes up to N incoming channels by the timestamps contained in their messages' headers.
 * TimeSynchronizer takes anywhere from 2 to N message types as template parameters, and passes them through to a
 * callback which takes a shared pointer of each.
 *
 * The required queue size parameter when constructing the TimeSynchronizer tells it how many sets of messages it should
 * store (by timestamp) while waiting for messages to arrive and complete their "set"
 *
 * \section connections CONNECTIONS
 *
 * The input connections for the TimeSynchronizer object is the same signature as for rclcpp subscription callbacks, ie.
\verbatim
void callback(const std::shared_ptr<M const>&);
\endverbatim
 * The output connection for the TimeSynchronizer object is dependent on the number of messages being synchronized. For
 * a 3-message synchronizer for example, it would be:
\verbatim
void callback(const std::shared_ptr<M0 const>&, const std::shared_ptr<M1 const>&, const std::shared_ptr<M2 const>&);
\endverbatim
 * \section usage USAGE
 * Example usage would be:
\verbatim
TimeSynchronizer<sensor_msgs::msg::CameraInfo, sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync_policies(caminfo_sub, limage_sub, rimage_sub, 3);
sync_policies.registerCallback(callback);
\endverbatim

 * The callback is then of the form:
\verbatim
void callback(const sensor_msgs::msg::CameraInfo::SharedPtr, const sensor_msgs::msg::Image::SharedPtr, const sensor_msgs::msg::Image::SharedPtr);
\endverbatim
 *
 */
template<class ... Ms>
class TimeSynchronizer : public Synchronizer<sync_policies::ExactTime<Ms...>>
{
public:
  typedef sync_policies::ExactTime<Ms...> Policy;
  typedef Synchronizer<Policy> Base;

  using Base::add;
  using Base::connectInput;
  using Base::registerCallback;
  using Base::setName;
  using Base::getName;
  using Policy::registerDropCallback;

  template<class ... Fs>
  TimeSynchronizer(uint32_t queue_size, Fs &... fs)
  : Base(Policy(queue_size))
  {
    connectInput(fs ...);
  }

  template<class F0, class F1>
  [[deprecated("Use variade constructor instead")]]
  TimeSynchronizer(F0 & f0, F1 & f1, uint32_t queue_size)
  : TimeSynchronizer(queue_size, f0, f1) {}

  template<class F0, class F1, class F2>
  [[deprecated("Use variade constructor instead")]]
  TimeSynchronizer(F0 & f0, F1 & f1, F2 & f2, uint32_t queue_size)
  : TimeSynchronizer(queue_size, f0, f1, f2) {}

  template<class F0, class F1, class F2, class F3>
  [[deprecated("Use variade constructor instead")]]
  TimeSynchronizer(F0 & f0, F1 & f1, F2 & f2, F3 & f3, uint32_t queue_size)
  : TimeSynchronizer(queue_size, f0, f1, f2, f3) {}

  template<class F0, class F1, class F2, class F3, class F4>
  [[deprecated("Use variade constructor instead")]]
  TimeSynchronizer(F0 & f0, F1 & f1, F2 & f2, F3 & f3, F4 & f4, uint32_t queue_size)
  : TimeSynchronizer(queue_size, f0, f1, f2, f3, f4) {}

  template<class F0, class F1, class F2, class F3, class F4, class F5>
  [[deprecated("Use variade constructor instead")]]
  TimeSynchronizer(F0 & f0, F1 & f1, F2 & f2, F3 & f3, F4 & f4, F5 & f5, uint32_t queue_size)
  : TimeSynchronizer(queue_size, f0, f1, f2, f3, f4, f5) {}

  template<class F0, class F1, class F2, class F3, class F4, class F5, class F6>
  [[deprecated("Use variade constructor instead")]]
  TimeSynchronizer(
    F0 & f0, F1 & f1, F2 & f2, F3 & f3, F4 & f4, F5 & f5, F6 & f6,
    uint32_t queue_size)
  : TimeSynchronizer(queue_size, f0, f1, f2, f3, f4, f5, f6) {}

  template<class F0, class F1, class F2, class F3, class F4, class F5, class F6, class F7>
  [[deprecated("Use variade constructor instead")]]
  TimeSynchronizer(
    F0 & f0, F1 & f1, F2 & f2, F3 & f3, F4 & f4, F5 & f5, F6 & f6, F7 & f7,
    uint32_t queue_size)
  : TimeSynchronizer(queue_size, f0, f1, f2, f3, f4, f5, f6, f7) {}

  template<class F0, class F1, class F2, class F3, class F4, class F5, class F6, class F7, class F8>
  [[deprecated("Use variade constructor instead")]]
  TimeSynchronizer(
    F0 & f0, F1 & f1, F2 & f2, F3 & f3, F4 & f4, F5 & f5, F6 & f6, F7 & f7, F8 & f8,
    uint32_t queue_size)
  : TimeSynchronizer(queue_size, f0, f1, f2, f3, f4, f5, f6, f7, f8) {}
};
}  // namespace message_filters

#endif  // MESSAGE_FILTERS__TIME_SYNCHRONIZER_HPP_
