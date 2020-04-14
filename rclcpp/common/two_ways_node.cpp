#include <rosidl_generator_cpp/traits.hpp>
#include <rclcpp/strategies/message_pool_memory_strategy.hpp>
#include <rttest/utils.h>
#include "two_ways_node.hpp"

using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;

void TwoWaysNode::setup_ping_publisher()
{
  auto topic_name = this->tw_options_.topic_name;
  auto qos = this->tw_options_.qos;
  auto period_ns = this->tw_options_.period_ns;

  // pub
  this->ping_pub_ = this->create_publisher<twmsgs::msg::Data>(topic_name, qos);

  auto callback_pub =
      [this, period_ns]() -> void
      {
        struct timespec time_wake_ts;
        getnow(&time_wake_ts);

        // calc wakeup jitter
        struct timespec now_expect_diff_ts;
        subtract_timespecs(&time_wake_ts, &expect_ts_, &now_expect_diff_ts);
        ping_wakeup_report_.add(timespec_to_long(&now_expect_diff_ts));
        struct timespec diff_from_last_wakeup_ts;
        subtract_timespecs(&time_wake_ts, &last_wake_ts_, &diff_from_last_wakeup_ts);
        // minus period because distribution mean is period_ns.
        subtract_timespecs(&diff_from_last_wakeup_ts, &period_ts_, &diff_from_last_wakeup_ts);
        diff_wakeup_report_.add(timespec_to_long(&diff_from_last_wakeup_ts));

        // prepere to next
        add_timespecs(&epoch_ts_, &period_ts_, &expect_ts_);
        ping_pub_count_++;
        last_wake_ts_ = time_wake_ts;

        // pub
        twmsgs::msg::Data msg;
        msg.data = ping_pub_count_;
        // msg.data = "HelloWorld" + std::to_string(this->ping_pub_count_);
        auto time_wake_ns = timespec_to_long(&time_wake_ts);
        msg.time_sent_ns = time_wake_ns;
        ping_pub_->publish(msg);

        if(ping_pub_count_ == tw_options_.num_loops_) {
          std::raise(SIGINT);
        }
      };

  // set timer
  getnow(&epoch_ts_);
  period_ts_.tv_sec = 0;
  period_ts_.tv_nsec = tw_options_.period_ns;
  add_timespecs(&epoch_ts_, &period_ts_, &expect_ts_);
  last_wake_ts_ = epoch_ts_;
  this->ping_timer_ = this->create_wall_timer(std::chrono::nanoseconds(period_ns), callback_pub);
}

void TwoWaysNode::setup_ping_subscriber(bool send_pong)
{
  auto topic_name = tw_options_.topic_name;
  auto topic_name_pong = tw_options_.topic_name_pong;
  auto qos = tw_options_.qos;

  // pong publisher
  send_pong_ = send_pong;
  if (send_pong) {
    pong_pub_ = create_publisher<twmsgs::msg::Data>(topic_name_pong, qos);
  }

  // subscriber callback
  auto callback_sub =
      [this](const twmsgs::msg::Data::SharedPtr msg) -> void
      {
        struct timespec now_ts;
        getnow(&now_ts);
        auto now_ns = timespec_to_long(&now_ts);
        ping_sub_report_.add(now_ns - msg->time_sent_ns);
        ping_sub_count_++;

        if (!send_pong_) {
          return;
        }

        auto pong = twmsgs::msg::Data();
        pong.time_sent_ns = msg->time_sent_ns;
        pong.time_sent_pong_ns = now_ns;
        pong.data = msg->data;
        pong_pub_->publish(pong);
        pong_pub_count_++;
      };

  // create subscriber
  rclcpp::SubscriptionOptions subscription_options;
  ping_sub_ =
      create_subscription<twmsgs::msg::Data>(topic_name,
                                             qos,
                                             callback_sub,
                                             subscription_options);

  // set message pool memory strategy
  if (tw_options_.use_message_pool_memory_strategy) {
    auto data_msg_strategy =
        std::make_shared<MessagePoolMemoryStrategy<twmsgs::msg::Data, 1>>();
    ping_sub_->set_message_memory_strategy(data_msg_strategy);
  }

}

void TwoWaysNode::setup_pong_subscriber()
{
  auto topic_name_pong = tw_options_.topic_name_pong;
  auto qos = tw_options_.qos;
  auto callback_pong_sub =
      [this](const twmsgs::msg::Data::SharedPtr msg) -> void
      {
        struct timespec now;
        getnow(&now);

        auto now_ns = timespec_to_long(&now);
        ping_pong_report_.add(now_ns - msg->time_sent_ns);
        pong_sub_report_.add(now_ns - msg->time_sent_pong_ns);
        pong_sub_count_++;
      };
  rclcpp::SubscriptionOptions subscription_options;
  pong_sub_ = create_subscription<twmsgs::msg::Data>(topic_name_pong,
                                                     qos,
                                                     callback_pong_sub,
                                                     subscription_options);
}
