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

/**
 * \brief Synchronizes up to 9 messages by their rates with upsampling via zero-order-hold.
 *
 * LatestTime policy synchronizes up to 9 incoming channels by the rates they are received.
 * The callback with all the messages will be triggered whenever the fastest message is received.
 * The slower messages will be repeated at the rate of the fastest message and will be updated
 * whenever a new one is received. This is essentially an upsampling of slower messages using a
 * zero-order hold (no interpolation).

 * \section usage USAGE
 * Example usage would be:
\verbatim
typedef LatestTime<sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::Image> latest_policy;
Synchronizer<latest_policy> sync_policies(latest_policy(1U), caminfo_sub, limage_sub, rimage_sub);
sync_policies.registerCallback(callback);
\endverbatim

 * The callback is then of the form:
\verbatim
void callback(const sensor_msgs::CameraInfo::ConstPtr&, const sensor_msgs::Image::ConstPtr&, const sensor_msgs::Image::ConstPtr&);
\endverbatim
 *
 */

#ifndef MESSAGE_FILTERS__SYNC_LATEST_TIME_H_
#define MESSAGE_FILTERS__SYNC_LATEST_TIME_H_

#include <algorithm>
#include <deque>
#include <numeric>
#include <string>
#include <tuple>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "message_filters/message_traits.h"
#include "message_filters/null_types.h"
#include "message_filters/signal9.h"
#include "message_filters/synchronizer.h"


namespace message_filters
{
namespace sync_policies
{

template<typename M0, typename M1,
         typename M2 = NullType, typename M3 = NullType, typename M4 = NullType,
         typename M5 = NullType, typename M6 = NullType, typename M7 = NullType,
         typename M8 = NullType>
struct LatestTime : public PolicyBase<M0, M1, M2, M3, M4, M5, M6, M7, M8>
{
  typedef Synchronizer<LatestTime> Sync;
  typedef PolicyBase<M0, M1, M2, M3, M4, M5, M6, M7, M8> Super;
  typedef typename Super::Messages Messages;
  typedef typename Super::Signal Signal;
  typedef typename Super::Events Events;
  typedef typename Super::RealTypeCount RealTypeCount;

  LatestTime(uint32_t)
  : parent_(0),
    ros_clock_(RCL_ROS_TIME)
  {
  }

  LatestTime(const LatestTime& e)
  {
    *this = e;
  }

  LatestTime& operator=(const LatestTime& rhs)
  {
    parent_ = rhs.parent_;
    events_ = rhs.events_;
    rates_ = rhs.rates_;
    ros_clock_ = rhs.ros_clock_;

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
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if(!received_msg<i>())
    {
      rates_.push_back(Rate(ros_clock_.now()));
      // wait until we get each message once to publish
      // then wait until we got each message twice to compute rates
      // NOTE: this will drop a few messages of the faster topics until
      //       we get one of the slowest so we can sync
      std::get<i>(events_) = evt;  // adding here ensures we see even the slowest
                                   // message twice before computing rate
      return;
    }

    std::get<i>(events_) = evt;

    std::cout << "compute hz for " << i << std::endl;
    rates_[i].compute_hz(ros_clock_.now());
    if(i == find_pivot() && is_full())
    {
      publish();
    }
  }

private:
  // assumed data_mutex_ is locked
  void publish()
  {
    std::cout << "publish!" << std::endl;
    parent_->signal(std::get<0>(events_), std::get<1>(events_), std::get<2>(events_),
                    std::get<3>(events_), std::get<4>(events_), std::get<5>(events_),
                    std::get<6>(events_), std::get<7>(events_), std::get<8>(events_));
  }

  struct Rate
  {
    rclcpp::Time prev;
    double hz{0.0};
    bool do_init{true};
    Rate(const rclcpp::Time &start)
      : prev(start)
    {
    }

    bool operator>(const Rate &that) const
    {
      return this->hz > that.hz;
    }

    void compute_hz(const rclcpp::Time &now)
    {
      double period = (now-prev).seconds();
      prev = now;
      if (do_init) {
        hz = 1.0/period;
        do_init = false;
      } else {
        hz = 0.95/period + 0.05*hz;
      }
      std::cout << "hz:" << hz << std::endl;
    }
  };

  // assumed data_mutex_ is locked

  template <typename T>
  std::vector<std::size_t> sort_indices(const std::vector<T> &v)
  {

    // initialize original index locations
    std::vector<std::size_t> idx(v.size());
    std::iota(idx.begin(), idx.end(), 0U);

    // sort indexes based on comparing values in v
    // using std::stable_sort instead of std::sort
    // to avoid unnecessary index re-orderings
    // when v contains elements of equal values 
    std::stable_sort(idx.begin(), idx.end(),
                     [&v](std::size_t i1, std::size_t i2) {return v[i1] > v[i2];});

    return idx;
  }

  // assumed data_mutex_ is locked
  template<int i>
  bool received_msg()
  {
    return (RealTypeCount::value > i ? (bool)std::get<i>(events_).getMessage() : true);
  }

  // assumed data_mutex_ is locked
  bool is_full()
  {
    bool full = received_msg<0>();
    full = full && received_msg<1>();
    full = full && received_msg<2>();
    full = full && received_msg<3>();
    full = full && received_msg<4>();
    full = full && received_msg<5>();
    full = full && received_msg<6>();
    full = full && received_msg<7>();
    full = full && received_msg<8>();

    return full;
  }

  // assumed data_mutex_ is locked
  int find_pivot()
  {
    // find arg max rate
    return sort_indices(rates_)[0U];
  }

  Sync* parent_;
  Events events_;
  std::vector<Rate> rates_;
  std::mutex data_mutex_;  // Protects all of the above
  
  rclcpp::Clock ros_clock_;
};

}  // namespace sync
}  // namespace message_filters

#endif // MESSAGE_FILTERS__SYNC_LATEST_TIME_H_
