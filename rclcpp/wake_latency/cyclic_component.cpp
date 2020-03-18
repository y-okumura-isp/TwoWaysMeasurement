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
      period_ns_(10 * 1000 * 1000), sleep_ns_(1000 * 1000), count_(0),
      fout_("cyclic_component.txt")
{
  os_.open(fout_, std::ios::out);
  os_ << "now, expect, last_fin, now-expect, now-last_fin" << std::endl;
  auto callback =
      [this]()
      {
        count_++;
        auto now = SC::now();
        auto expect = epoch_ + NS(period_ns_) * count_;
        os_ << tp2ns(now) << ", "
            << tp2ns(expect) << ", "
            << tp2ns(last_fin_) << ", "
            << diff_ns(now, expect) << ", "
            << diff_ns(now, last_fin_)
            << std::endl;

        struct timespec now_ts;
        getnow(&now_ts);
        struct timespec now_expect_diff_ts, now_last_fin_diff_ts;
        subtract_timespecs(&now_ts, &expect_ts_, &now_expect_diff_ts);
        subtract_timespecs(&now_ts, &last_fin_ts_, &now_last_fin_diff_ts);

        os_ << "ts "
            << timespec_to_long(&now_ts) << ", "
            << timespec_to_long(&expect_ts_) << ", "
            << timespec_to_long(&last_fin_ts_) << ","
            << timespec_to_long(&now_expect_diff_ts) << ","
            << timespec_to_long(&now_last_fin_diff_ts)
            << std::endl;

        //std::this_thread::sleep_for(NS(sleep_ns_));
        last_fin_ = SC::now();
        getnow(&last_fin_ts_);

        add_timespecs(&expect_ts_, &period_ns_ts_, &expect_ts_);
      };

  epoch_ = SC::now();
  last_fin_ = SC::now();

  getnow(&epoch_ts_);
  getnow(&last_fin_ts_);
  period_ns_ts_.tv_sec = 0;
  period_ns_ts_.tv_nsec = period_ns_;
  add_timespecs(&epoch_ts_, &period_ns_ts_, &expect_ts_);

  timer_ = create_wall_timer(NS(period_ns_), callback);
}

CyclicNode::~CyclicNode()
{
  os_.close();
}

int CyclicNode::diff_ns(TIME_POINT lhs, TIME_POINT rhs)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(lhs - rhs).count();
}

int CyclicNode::tp2ns(TIME_POINT tp)
{
  return std::chrono::nanoseconds(tp.time_since_epoch()).count();
}


}  // namespace cyclic_component

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cyclic_component::CyclicNode)
