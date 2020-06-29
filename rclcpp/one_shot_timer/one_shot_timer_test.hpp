#ifndef ONE_SHOT_TIMER_TEST_HPP_
#define ONE_SHOT_TIMER_TEST_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>

#include "two_ways_node.hpp"
#include "twmsgs/msg/data.hpp"

class OneShotTimerTestNode : public rclcpp::Node
{
public:
  explicit OneShotTimerTestNode(
      const TwoWaysNodeOptions & tw_options,
      const rclcpp::NodeOptions & options);

  void setup_periodic_trigger();
  void setup_oneshot_trigger();

private:
  const TwoWaysNodeOptions & tw_options_;

  // periodic trigger staffs
  rclcpp::Publisher<twmsgs::msg::Data>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr periodic_timer_;
  uint64_t periodic_period_ns_;

  // oneshot trigger staffs
  rclcpp::Subscription<twmsgs::msg::Data>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr oneshot_timer_;
  uint64_t oneshot_period_ns_;
};

#endif  // ONE_SHOT_TIMER_TEST_HPP_
