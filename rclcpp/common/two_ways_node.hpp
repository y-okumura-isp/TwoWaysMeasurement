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
public:
  // explicit TwoWaysNode(
  //     const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  explicit TwoWaysNode(
      const std::string name,
      const std::string namespace_,
      const TwoWaysNodeOptions & tw_options = TwoWaysNodeOptions(),
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  virtual ~TwoWaysNode()
  {}

  // number of ping publish
  int ping_pub_count_;

  // number of ping subscribe
  int ping_sub_count_;

  // number of pong publish
  int pong_pub_count_;

  // number of pong subscribe
  int pong_sub_count_;

  void print_ping_wakeup_report() {
    ping_wakeup_report_.print("ping_wakeup");
  }
  void print_diff_wakeup_report() {
    diff_wakeup_report_.print("diff_wakeup");
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
  struct timespec epoch_ts_;
  struct timespec period_ts_;
  struct timespec expect_ts_;
  struct timespec last_wake_ts_;

  rclcpp::TimerBase::SharedPtr ping_timer_;
  rclcpp::Publisher<twmsgs::msg::Data>::SharedPtr ping_pub_;
  rclcpp::Subscription<twmsgs::msg::Data>::SharedPtr ping_sub_;
  rclcpp::Publisher<twmsgs::msg::Data>::SharedPtr pong_pub_;
  rclcpp::Subscription<twmsgs::msg::Data>::SharedPtr pong_sub_;

  bool send_pong_;

  // wakeup jitter report
  JitterReportWithSkip ping_wakeup_report_;
  // wakeup jitter from last wakeup
  JitterReportWithSkip diff_wakeup_report_;
  // sub jitter report
  JitterReportWithSkip ping_sub_report_;
  // pong sub jitter report
  JitterReportWithSkip pong_sub_report_;
  // ping-pong jitter report
  JitterReportWithSkip ping_pong_report_;

};

#endif  // TWO_WAYS_NODE_HPP_
