#include <rosidl_generator_cpp/traits.hpp>
#include <rclcpp/strategies/message_pool_memory_strategy.hpp>
#include <rttest/utils.h>
#include "two_ways_node.hpp"

using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;

const std::string PERIOD_NS = "period_ns";
const std::string NUM_LOOPS = "num_loops";
const std::string DEBUG_PRINT = "debug_print";

TwoWaysNode::TwoWaysNode(
    const std::string name,
    const std::string namespace_,
    const TwoWaysNodeOptions & tw_options,
    const rclcpp::NodeOptions & options)
    : Node(name, namespace_, options),
      ping_pub_count_(0), ping_sub_count_(0),
      pong_pub_count_(0), pong_sub_count_(0),
      tw_options_(tw_options),
      send_pong_(false)
{
  declare_parameter(PERIOD_NS, 10 * 1000 * 1000);
  declare_parameter(NUM_LOOPS, 10000);
  declare_parameter(DEBUG_PRINT, false);

  // setup reports
  ping_wakeup_report_.init(
      tw_options.ping_wakeup.bin,
      tw_options.ping_wakeup.round_ns,
      tw_options.ping_wakeup.num_skip);
  diff_wakeup_report_.init(
      tw_options.common_report_option.bin,
      tw_options.common_report_option.round_ns,
      tw_options.common_report_option.num_skip);
  ping_sub_report_.init(
      tw_options.ping_sub.bin,
      tw_options.ping_sub.round_ns,
      tw_options.ping_sub.num_skip);
  pong_sub_report_.init(
      tw_options.ping_sub.bin,
      tw_options.ping_sub.round_ns,
      tw_options.ping_sub.num_skip);
  ping_pong_report_.init(
      tw_options.ping_sub.bin,
      tw_options.ping_sub.round_ns,
      tw_options.ping_sub.num_skip);
}

void TwoWaysNode::setup_ping_publisher()
{
  auto topic_name = this->tw_options_.topic_name;
  auto qos = this->tw_options_.qos;
  auto period_ns = get_parameter(PERIOD_NS).get_value<int>();
  auto num_loops = get_parameter(NUM_LOOPS).get_value<int>();
  auto debug_print = get_parameter(DEBUG_PRINT).get_value<bool>();

  std::cout << PERIOD_NS   << ": " << period_ns << std::endl;
  std::cout << NUM_LOOPS   << ": " << num_loops << std::endl;
  std::cout << DEBUG_PRINT << ": " << (debug_print ? "true" : "false") << std::endl;

  // pub
  this->ping_pub_ = this->create_publisher<twmsgs::msg::Data>(topic_name, qos);

  auto callback_pub =
      [this, period_ns, num_loops, debug_print]() -> void
      {
        struct timespec time_wake_ts;
        getnow(&time_wake_ts);

        // calc wakeup jitter
        struct timespec now_expect_diff_ts;
        subtract_timespecs(&time_wake_ts, &expect_ts_, &now_expect_diff_ts);
        ping_wakeup_report_.add(_timespec_to_long(&now_expect_diff_ts));
        struct timespec diff_from_last_wakeup_ts;
        subtract_timespecs(&time_wake_ts, &last_wake_ts_, &diff_from_last_wakeup_ts);
        // minus period because distribution mean is period_ns.
        subtract_timespecs(&diff_from_last_wakeup_ts, &period_ts_, &diff_from_last_wakeup_ts);
        diff_wakeup_report_.add(_timespec_to_long(&diff_from_last_wakeup_ts));

        // prepere to next
        add_timespecs(&epoch_ts_, &period_ts_, &expect_ts_);
        ping_pub_count_++;
        last_wake_ts_ = time_wake_ts;

        // pub
        twmsgs::msg::Data msg;
        msg.data = ping_pub_count_;
        // msg.data = "HelloWorld" + std::to_string(this->ping_pub_count_);
        auto time_wake_ns = _timespec_to_long(&time_wake_ts);
        msg.time_sent_ns = time_wake_ns;
        ping_pub_->publish(msg);

        if(debug_print) {
          struct timespec time_print;
          getnow(&time_print);
          std::cout << "time_print.tv_sec: " << time_print.tv_sec << " "
                    << "time_print.tv_nsec: " << time_print.tv_nsec << std::endl;
          std::cout << "sent ping id = " << ping_pub_count_
                    << " @" << _timespec_to_long(&time_print)
                    << " waked up @ " << time_wake_ns
                    << std::endl;
        }

        if(ping_pub_count_ == num_loops) {
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
  auto debug_print = get_parameter(DEBUG_PRINT).get_value<bool>();

  // pong publisher
  send_pong_ = send_pong;
  if (send_pong) {
    pong_pub_ = create_publisher<twmsgs::msg::Data>(topic_name_pong, qos);
  }

  // subscriber callback
  auto callback_sub =
      [this, debug_print](const twmsgs::msg::Data::SharedPtr msg) -> void
      {
        struct timespec now_ts;
        getnow(&now_ts);
        auto now_ns = _timespec_to_long(&now_ts);
        ping_sub_report_.add(now_ns - msg->time_sent_ns);
        ping_sub_count_++;

        if(debug_print) {
          struct timespec time_print;
          getnow(&time_print);
          std::cout << "recv ping id = " << msg->data
                    << " @" << now_ns
                    << std::endl;
        }

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

        auto now_ns = _timespec_to_long(&now);
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
