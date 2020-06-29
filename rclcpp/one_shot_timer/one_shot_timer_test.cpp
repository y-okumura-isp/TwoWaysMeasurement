#include "one_shot_timer_test.hpp"

OneShotTimerTestNode::OneShotTimerTestNode(
    const TwoWaysNodeOptions & tw_options,
    const rclcpp::NodeOptions & options)
    : Node("one_shot_timer_test", options),
      tw_options_(tw_options),
      periodic_timer_(nullptr),
      periodic_period_ns_(1 * 1000 * 1000 * 1000),  // 1 sec
      oneshot_timer_(nullptr),
      oneshot_period_ns_(10 * 1000 * 1000)  // 10 ms
{
}

void OneShotTimerTestNode::setup_periodic_trigger()
{
  if(periodic_timer_) {
    return;
  }

  auto topic_name = this->tw_options_.topic_name;
  auto qos = this->tw_options_.qos;
  pub_ = this->create_publisher<twmsgs::msg::Data>(topic_name, qos);

  auto timer_callback =
      [this]() -> void
      {
        std::cout << "periodic timer fire" << std::endl;
        twmsgs::msg::Data msg;

        // TODO: do some heavy task
        
        this->pub_->publish(msg);
      };
  periodic_timer_ = create_wall_timer(std::chrono::nanoseconds(periodic_period_ns_),
                                      timer_callback);
}

void OneShotTimerTestNode::setup_oneshot_trigger()
{
  if(oneshot_timer_) {
    return;
  }

  // setup 1-shot timer
  auto timer_callback =
      [this]() -> void
      {
        std::cout << "1-shot callback called" << std::endl;
        this->oneshot_timer_->cancel();
      };
  oneshot_timer_ = create_wall_timer(std::chrono::nanoseconds(oneshot_period_ns_), timer_callback);
  oneshot_timer_->cancel();  // fire only when topic comes

  // setup subscription
  auto topic_name = tw_options_.topic_name;
  auto qos = tw_options_.qos;
  auto sub_callback =
      [this](const twmsgs::msg::Data::SharedPtr msg) -> void
      {
        (void) msg;
        std::cout << "subscriber callback" << std::endl;
        this->oneshot_timer_->reset();
        bool is_canceld = this->oneshot_timer_->is_canceled();
        std::cout << "oneshot_timer_->is_canceled: "
                  << (is_canceld ? "true": "false")
                  << std::endl;
      };

  rclcpp::SubscriptionOptions subscription_options;
  sub_ = create_subscription<twmsgs::msg::Data>(topic_name,
                                                qos,
                                                sub_callback,
                                                subscription_options);
}
