// Copyright 2022, Open Source Robotics Foundation, Inc. All rights reserved.
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

/**
 * \brief Synchronizes up to N messages by their rates with upsampling via zero-order-hold.
 *
 * LatestTime policy synchronizes up to N incoming channels by the rates they are received.
 * The callback with all the messages will be triggered whenever the fastest message is received.
 * The slower messages will be repeated at the rate of the fastest message and will be updated
 * whenever a new one is received. This is essentially an upsampling of slower messages using a
 * zero-order hold (no interpolation).

 * \section usage USAGE
 * Example usage would be:
\verbatim
typedef LatestTime<sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::Image> latest_policy;
Synchronizer<latest_policy> sync_policies(latest_policy(), caminfo_sub, limage_sub, rimage_sub);
sync_policies.registerCallback(callback);
\endverbatim

 * May also take an instance of a `rclcpp::Clock::SharedPtr` from `rclpp::Node::get_clock()`
 * to use the node's time source (e.g. sim time) as in:
\verbatim
typedef LatestTime<sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::Image> latest_policy;
Synchronizer<latest_policy> sync_policies(latest_policy(node->get_clock()), caminfo_sub, limage_sub, rimage_sub);
sync_policies.registerCallback(callback);
\endverbatim

 * The callback is then of the form:
\verbatim
void callback(const sensor_msgs::CameraInfo::ConstPtr &, const sensor_msgs::Image::ConstPtr&, const sensor_msgs::Image::ConstPtr&);
\endverbatim
 *
 */

#ifndef MESSAGE_FILTERS__SYNC_POLICIES__LATEST_TIME_HPP_
#define MESSAGE_FILTERS__SYNC_POLICIES__LATEST_TIME_HPP_

#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>
#include <tuple>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "message_filters/message_traits.hpp"
#include "message_filters/null_types.hpp"
#include "message_filters/signal9.hpp"
#include "message_filters/synchronizer.hpp"


namespace message_filters
{
namespace sync_policies
{

template<typename ... M>
struct LatestTime : public PolicyBase<M...>
{
  typedef Synchronizer<LatestTime> Sync;
  typedef PolicyBase<M...> Super;
  typedef typename Super::Messages Messages;
  typedef typename Super::Signal Signal;
  typedef typename Super::Events Events;
  typedef typename Super::RealTypeCount RealTypeCount;

  /// \brief filter coeffs and error margin factor:
  /// <rate_ema_alpha, error_ema_alpha, rate_step_change_margin_factor>
  typedef std::tuple<double, double, double> RateConfig;

  LatestTime()
  : LatestTime(rclcpp::Clock::SharedPtr(new rclcpp::Clock(RCL_ROS_TIME)))
  {
  }

  explicit LatestTime(rclcpp::Clock::SharedPtr clock)
  : parent_(0),
    ros_clock_{clock}
  {
  }

  LatestTime(const LatestTime & e)
  {
    *this = e;
  }

  LatestTime & operator=(const LatestTime & rhs)
  {
    parent_ = rhs.parent_;
    events_ = rhs.events_;
    rates_ = rhs.rates_;
    ros_clock_ = rhs.ros_clock_;

    return *this;
  }

  void initParent(Sync * parent)
  {
    parent_ = parent;
  }

  void setRateConfigPerMessage(const std::vector<RateConfig> & configs)
  {
    rate_configs_.assign(configs.begin(), configs.end());
  }

  void setRateConfig(const RateConfig & config)
  {
    rate_configs_.assign(1U, config);
  }

  void setClock(rclcpp::Clock::SharedPtr clock)
  {
    ros_clock_ = clock;
  }

  template<int i>
  void add(const typename std::tuple_element<i, Events>::type & evt)
  {
    assert(parent_);

    std::lock_guard<std::mutex> lock(data_mutex_);

    if (!received_msg<i>()) {
      initialize_rate<i>();
      // wait until we get each message once to publish
      // then wait until we got each message twice to compute rates
      // NOTE: this will drop a few messages of the faster topics until
      //       we get one of the slowest so we can sync
      std::get<i>(events_) = evt;  // adding here ensures we see even the slowest
                                   // message twice before computing rate
      return;
    }

    std::get<i>(events_) = evt;
    rclcpp::Time now = ros_clock_->now();
    rclcpp::Time event_time_received_prev = rates_[i].prev;
    bool valid_rate = rates_[i].compute_hz(now);
    int pivot = find_pivot(now);
    if (valid_rate && is_full()) {
      if (i == pivot) {
        publish(now);
      } else if (i == pivot_prev_) {
        rclcpp::Duration period_event_received = now - event_time_received_prev;
        if (last_time_published_ + period_event_received * SLACK_DURATION_FACTOR_EXAMPLE <
          event_time_received_prev)
        {
          publish(now);
        }
      }
    }
    pivot_prev_ = pivot;
  }

private:
  // assumed data_mutex_ is locked
  template<int i>
  void initialize_rate()
  {
    if (rate_configs_.size() > 0U) {
      double rate_ema_alpha{Rate::DEFAULT_RATE_EMA_ALPHA};
      double error_ema_alpha{Rate::DEFAULT_ERROR_EMA_ALPHA};
      double rate_step_change_margin_factor{Rate::DEFAULT_MARGIN_FACTOR};
      if (rate_configs_.size() == RealTypeCount::value) {
        std::tie(
          rate_ema_alpha,
          error_ema_alpha,
          rate_step_change_margin_factor) = rate_configs_[i];
      } else if (rate_configs_.size() == 1U) {
        std::tie(
          rate_ema_alpha,
          error_ema_alpha,
          rate_step_change_margin_factor) = rate_configs_[0U];
      }
      rates_.push_back(
        Rate(
          ros_clock_->now(),
          rate_ema_alpha,
          error_ema_alpha,
          rate_step_change_margin_factor));
    } else {
      rates_.push_back(Rate(ros_clock_->now()));
    }
  }

  // assumed data_mutex_ is locked
  void publish(const rclcpp::Time & now)
  {
    std::apply([this](auto &&... args) {this->parent_->signal(args ...);}, events_);
    last_time_published_ = now;
  }

  struct Rate
  {
    static constexpr double DEFAULT_RATE_EMA_ALPHA{0.9};
    static constexpr double DEFAULT_ERROR_EMA_ALPHA{0.3};
    static constexpr double DEFAULT_MARGIN_FACTOR{10.0};

    rclcpp::Time prev;
    double hz{0.0};
    double error{0.0};
    double rate_ema_alpha{DEFAULT_RATE_EMA_ALPHA};
    double error_ema_alpha{DEFAULT_ERROR_EMA_ALPHA};
    double rate_step_change_margin_factor{DEFAULT_MARGIN_FACTOR};
    bool do_hz_init{true};
    bool do_error_init{true};

    explicit Rate(const rclcpp::Time & start)
    : Rate(start, DEFAULT_RATE_EMA_ALPHA, DEFAULT_ERROR_EMA_ALPHA, DEFAULT_MARGIN_FACTOR)
    {
    }

    Rate(
      const rclcpp::Time & start,
      const double & rate_ema_alpha, const double & error_ema_alpha,
      const double & rate_step_change_margin_factor)
    : prev{start},
      rate_ema_alpha{rate_ema_alpha},
      error_ema_alpha{error_ema_alpha},
      rate_step_change_margin_factor{rate_step_change_margin_factor}
    {
    }

    bool operator>(const Rate & that) const
    {
      return this->hz > that.hz;
    }

    bool compute_hz(const rclcpp::Time & now)
    {
      bool step_change_detected = false;
      do {
        double period = 0.0;
        try {
          period = (now - prev).seconds();
        } catch (const std::runtime_error & /*e*/) {
          // Different time sources that might happen on initialization if the messages are not yet
          // available.
          // std::cout << "Exception: " << e.what() << std::endl;
          return false;
        }
        if (period <= 0.0) {
          // multiple messages and time isn't updating
          return false;
        }

        if (do_hz_init) {
          hz = 1.0 / period;
          do_hz_init = false;
          step_change_detected = false;
        } else {
          if (do_error_init) {
            error = fabs(hz - 1.0 / period);
            do_error_init = false;
          } else {
            // check if rate is some multiple of mean error from mean
            if (fabs(hz - 1.0 / period) > rate_step_change_margin_factor * error) {
              // detected step change in rate so reset
              do_hz_init = true;
              do_error_init = true;
              step_change_detected = true;
              continue;
            }
            // on-line mean error from mean
            error = error_ema_alpha * fabs(hz - 1.0 / period) + (1.0 - error_ema_alpha) * error;
          }
          hz = rate_ema_alpha / period + (1.0 - rate_ema_alpha) * hz;
        }
      } while (step_change_detected);
      prev = now;
      return true;
    }
  };

  // assumed data_mutex_ is locked
  template<typename T>
  std::vector<std::size_t> sort_indices(const std::vector<T> & v)
  {
    // initialize original index locations
    std::vector<std::size_t> idx(v.size());
    std::iota(idx.begin(), idx.end(), 0U);

    // sort indexes based on comparing values in v
    // using std::stable_sort instead of std::sort
    // to avoid unnecessary index re-orderings
    // when v contains elements of equal values
    std::stable_sort(
      idx.begin(), idx.end(),
      [&v](std::size_t i1, std::size_t i2) {return v[i1] > v[i2];});

    return idx;
  }

  // assumed data_mutex_ is locked
  template<int i>
  bool received_msg()
  {
    return static_cast<bool>(std::get<i>(events_).getMessage());
  }

  // assumed data_mutex_ is locked
  bool is_full()
  {
    return is_full_impl(std::make_index_sequence<std::tuple_size_v<Events>>{});
  }

  template<std::size_t... Is>
  bool is_full_impl(std::index_sequence<Is...>)
  {
    /* *INDENT-OFF* */
    // uncrustify messes with the brackets which are required (at least by GCC)
    return (... && received_msg<Is>());
    /* *INDENT-ON* */
  }

  // assumed data_mutex_ is locked
  int find_pivot(const rclcpp::Time & now)
  {
    // find arg max rate
    std::vector<std::size_t> sorted_idx = sort_indices(rates_);

    // use fastest message that isn't late as pivot
    for (size_t pivot : sorted_idx) {
      double period = (now - rates_[pivot].prev).seconds();
      if (period == 0.0) {
        if (rates_[pivot].hz > 0.0) {
          // we just updated updated this one,
          // and it's fastest, so use as pivot
          return static_cast<int>(pivot);
        } else {
          // haven't calculated rate for this message yet
          continue;
        }
      }

      if (!rates_[pivot].do_error_init) {
        // can now check if new messages are late
        double rate_delta = rates_[pivot].hz - 1.0 / period;
        double margin = rates_[pivot].rate_step_change_margin_factor * rates_[pivot].error;
        if (rate_delta > margin) {
          // this pivot is late
          continue;
        }
      }

      if (rates_[pivot].hz > 0.0) {
        // found fastest message with a calculated rate
        return static_cast<int>(pivot);
      } else {
        // haven't calculated rate for this message yet
        continue;
      }
    }
    return NO_PIVOT;
  }

  Sync * parent_;
  Events events_;
  std::vector<Rate> rates_;
  std::mutex data_mutex_;  // Protects all of the above

  std::vector<RateConfig> rate_configs_;

  const int NO_PIVOT{Super::N_MESSAGES};

  rclcpp::Clock::SharedPtr ros_clock_{nullptr};

  int pivot_prev_ = NO_PIVOT;

  rclcpp::Time last_time_published_ = rclcpp::Time(0, 0);

  static constexpr double SLACK_DURATION_FACTOR_EXAMPLE{1.1};
};

}  // namespace sync_policies
}  // namespace message_filters

#endif  // MESSAGE_FILTERS__SYNC_POLICIES__LATEST_TIME_HPP_
