/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef MESSAGE_FILTERS__SYNC_LatestStamped_H_
#define MESSAGE_FILTERS__SYNC_LatestStamped_H_

#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "message_filters/connection.h"
#include "message_filters/message_traits.h"
#include "message_filters/null_types.h"
#include "message_filters/signal9.h"
#include "message_filters/synchronizer.h"

namespace message_filters
{
namespace sync_policies
{

/**
 * The LatestStamped policy stores the LatestStamped received messages (based on their timestamp) and DO NOT call any callback automatically.
 * It is up to the programmer to call #call when it is time to process the information. This policy class
 * is meant to be used in networks where information is processed periodically.
 */
template<typename M0, typename M1, typename M2 = NullType, typename M3 = NullType, typename M4 = NullType,
         typename M5 = NullType, typename M6 = NullType, typename M7 = NullType, typename M8 = NullType>
struct LatestStamped : public PolicyBase<M0, M1, M2, M3, M4, M5, M6, M7, M8>
{
  typedef Synchronizer<LatestStamped> Sync;
  typedef PolicyBase<M0, M1, M2, M3, M4, M5, M6, M7, M8> Super;
  typedef typename Super::Messages Messages;
  typedef typename Super::Signal Signal;
  typedef typename Super::Events Events;
  typedef typename Super::RealTypeCount RealTypeCount;
  typedef typename Super::M0Event M0Event;
  typedef typename Super::M1Event M1Event;
  typedef typename Super::M2Event M2Event;
  typedef typename Super::M3Event M3Event;
  typedef typename Super::M4Event M4Event;
  typedef typename Super::M5Event M5Event;
  typedef typename Super::M6Event M6Event;
  typedef typename Super::M7Event M7Event;
  typedef typename Super::M8Event M8Event;

  LatestStamped()
  : parent_(0)
  {
  }

  LatestStamped(const LatestStamped& e)
  {
    *this = e;
  }

  LatestStamped& operator=(const LatestStamped& rhs)
  {
    parent_ = rhs.parent_;
    events_ = rhs.events_;

    return *this;
  }

  void initParent(Sync* parent)
  {
    parent_ = parent;
  }

  template<int i>
  void add(const typename std::tuple_element<i, Events>::type& evt)
  {
    RCUTILS_ASSERT(parent_);

    namespace mt = message_filters::message_traits;

    std::lock_guard<std::mutex> lock(mutex_);

    typedef typename std::tuple_element<i, Messages>::type MessageType;
    auto& evt_old = std::get<i>(events_);
    bool has_old = (bool)evt_old.getMessage();
    if (has_old)
    {
      rclcpp::Time stamp_new = mt::TimeStamp<MessageType>::value(*evt.getMessage());
      rclcpp::Time stamp_old = mt::TimeStamp<MessageType>::value(*evt_old.getMessage());
      if (stamp_new > stamp_old)
        evt_old = evt;
    }
    else
      evt_old = evt;
  }

  void call() const
  {
    parent_->signal(std::get<0>(events_), std::get<1>(events_), std::get<2>(events_),
                    std::get<3>(events_), std::get<4>(events_), std::get<5>(events_),
                    std::get<6>(events_), std::get<7>(events_), std::get<8>(events_));
  }

private:
  Sync* parent_;

  Events events_;

  std::mutex mutex_;
};

}  // namespace sync_policies
}  // namespace message_filters

#endif  // MESSAGE_FILTERS__SYNC_LatestStamped_H_
