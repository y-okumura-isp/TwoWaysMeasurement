#include <iostream>
#include <fstream>
#include <thread>
#include "cyclic_component.hpp"

using namespace std::chrono_literals;

#define NS std::chrono::nanoseconds

namespace cyclic_component
{

CyclicNode::CyclicNode(const rclcpp::NodeOptions & options)
    : Node("cyclic_node", options),
      period_ns_(10 * 1000 * 1000), sleep_ns_(100 * 1000), count_(0),
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

        std::this_thread::sleep_for(NS(sleep_ns_));
        last_fin_ = SC::now();
      };

  epoch_ = SC::now();
  last_fin_ = SC::now();
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
