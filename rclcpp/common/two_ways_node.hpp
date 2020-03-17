#ifndef TWO_WAYS_NODE_HPP_
#define TWO_WAYS_NODE_HPP_

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>
#include "twmsgs/msg/data.hpp"
#include "tw_node_options.hpp"
#include "tw_utils.hpp"

class TwoWaysNode : public rclcpp::Node
{
  using _SC = std::chrono::system_clock;
  using TIME_POINT = std::chrono::time_point<std::chrono::system_clock>;

public:
  explicit TwoWaysNode(
      const std::string name,
      const std::string namespace_,
      const TwoWaysNodeOptions & tw_options = TwoWaysNodeOptions(),
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
      : Node(name, namespace_, options),
        ping_pub_count_(0),
        ping_sub_count_(0),
        pong_pub_count_(0),
        pong_sub_count_(0),
        tw_options_(tw_options),
        send_pong_(false)
  {
    ping_wakeup_report_.init(
        tw_options.ping_wakeup.bin,
        tw_options.ping_wakeup.round_ns);
    ping_sub_report_.init(
        tw_options.ping_sub.bin,
        tw_options.ping_sub.round_ns);
    pong_sub_report_.init(
        tw_options.ping_sub.bin,
        tw_options.ping_sub.round_ns);
    ping_pong_report_.init(
        tw_options.ping_sub.bin,
        tw_options.ping_sub.round_ns);
  }

  virtual ~TwoWaysNode()
  {}

  // number of ping publish
  int ping_pub_count_;

  // ping epoch
  TIME_POINT ping_epoch_;

  // number of ping subscribe
  int ping_sub_count_;

  // number of pong publish
  int pong_pub_count_;

  // number of pong subscribe
  int pong_sub_count_;

  void print_ping_wakeup_report() {
    ping_wakeup_report_.print("ping_wakeup");
  }
  void print_ping_sub_report() {
    ping_sub_report_.print("ping_sub");
  }
  void print_pong_sub_report() {
    pong_sub_report_.print("pong_sub");
  }
  void print_ping_pong_report() {
    ping_pong_report_.print("ping_pong");
  }

  void setup_ping_publisher();
  void setup_ping_subscriber(bool send_pong=false);
  void setup_pong_subscriber();

protected:
  const TwoWaysNodeOptions & tw_options_;

private:
  rclcpp::TimerBase::SharedPtr ping_timer_;
  rclcpp::Publisher<twmsgs::msg::Data>::SharedPtr ping_pub_;
  rclcpp::Subscription<twmsgs::msg::Data>::SharedPtr ping_sub_;
  rclcpp::Publisher<twmsgs::msg::Data>::SharedPtr pong_pub_;
  rclcpp::Subscription<twmsgs::msg::Data>::SharedPtr pong_sub_;

  bool send_pong_;

  // wakeup jitter report
  JitterReport ping_wakeup_report_;
  // sub jitter report
  JitterReport ping_sub_report_;
  // pong sub jitter report
  JitterReport pong_sub_report_;
  // ping-pong jitter report
  JitterReport ping_pong_report_;

};

#endif  // TWO_WAYS_NODE_HPP_
