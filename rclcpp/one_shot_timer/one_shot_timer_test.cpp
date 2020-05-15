#include "one_shot_timer_test.hpp"

OneShotTimerTestNode::OneShotTimerTestNode(
    const TwoWaysNodeOptions & tw_options,
    const rclcpp::NodeOptions & options)
    : Node("one_shot_timer_test", options),
      oneshot_timer_(nullptr),
      oneshot_period_ns_(10 * 1000 * 1000)
{
  (void) tw_options;
}

void OneShotTimerTestNode::setup_periodic_trigger()
{
}

void OneShotTimerTestNode::setup_oneshot_trigger()
{
  if(oneshot_timer_) {
    return;
  }

  auto callback =
      [this]() -> void
      {
        std::cout << "1-shot callback called" << std::endl;
        this->oneshot_timer_->cancel();
      };
  oneshot_timer_ = create_wall_timer(std::chrono::nanoseconds(oneshot_period_ns_), callback);
}

void OneShotTimerTestNode::reset_oneshot_trigger()
{
  if(!oneshot_timer_) {
    return;
  }
  oneshot_timer_.reset();
}
