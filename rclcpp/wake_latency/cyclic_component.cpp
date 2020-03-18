#include <iostream>
#include <fstream>
#include <thread>

#include <rttest/utils.h>

#include "cyclic_component.hpp"

using namespace std::chrono_literals;

#define NS std::chrono::nanoseconds

namespace cyclic_component
{

void getnow(struct timespec *t)
{
  // clock_gettime(CLOCK_REALTIME, t);
  clock_gettime(CLOCK_MONOTONIC, t);
}

CyclicNode::CyclicNode(const rclcpp::NodeOptions & options)
    : Node("cyclic_node", options),
      period_ns_(10 * 1000 * 1000), sleep_ns_(1000 * 1000),
      fout_("cyclic_component.txt")
{
  os_.open(fout_, std::ios::out);
  os_ << "now, expect, last_fin, now-expect, now-last_fin" << std::endl;
  auto callback =
      [this]()
      {
        struct timespec now_ts;
        getnow(&now_ts);
        struct timespec now_expect_diff_ts, now_last_fin_diff_ts;
        subtract_timespecs(&now_ts, &expect_ts_, &now_expect_diff_ts);
        subtract_timespecs(&now_ts, &last_fin_ts_, &now_last_fin_diff_ts);

        os_ << timespec_to_long(&now_ts) << ", "
            << timespec_to_long(&expect_ts_) << ", "
            << timespec_to_long(&last_fin_ts_) << ","
            << timespec_to_long(&now_expect_diff_ts) << ","
            << timespec_to_long(&now_last_fin_diff_ts)
            << std::endl;

        /*
          Sleep a while.
          I doubt thread waits <period_ns_>[ns] after callback finished? => not matters.
          So comment out next line for simplicity.
        */
        // std::this_thread::sleep_for(NS(sleep_ns_));

        getnow(&last_fin_ts_);

        // set next expected wakeup time
        add_timespecs(&expect_ts_, &period_ns_ts_, &expect_ts_);
      };

  // set epoch
  getnow(&epoch_ts_);
  getnow(&last_fin_ts_);

  // period
  period_ns_ts_.tv_sec = 0;
  period_ns_ts_.tv_nsec = period_ns_;

  // calcurate nexe wake up time
  add_timespecs(&epoch_ts_, &period_ns_ts_, &expect_ts_);

  // start timer
  timer_ = create_wall_timer(NS(period_ns_), callback);
}

CyclicNode::~CyclicNode()
{
  os_.close();
}

}  // namespace cyclic_component

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cyclic_component::CyclicNode)
