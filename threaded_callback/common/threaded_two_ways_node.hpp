#ifndef THREADED_TWO_WAYS_NODE_HPP
#define THREADED_TWO_WAYS_NODE_HPP

#include "../../../ROS2_ThreadedCallback/include/threaded_wall_timer.hpp"

#include "twmsgs/msg/data.hpp"
#include "../../rclcpp/common/tw_node_options.hpp"
#include "../../rclcpp/common/tw_utils.hpp"

class PingPublisherByTimer : public ThreadedWallTimer
{
  using MyMsg = twmsgs::msg::Data;

public:
  PingPublisherByTimer(
      const TwoWaysNodeOptions &tw_options,
      rclcpp::Node *node, const std::string &topic_name, const rclcpp::QoS qos,
      long period_ns, bool debug_print, uint64_t num_loops,
      size_t sched_priority=0, int policy=SCHED_OTHER, size_t core_id=1);

  void print_ping_wakeup_report() {
    ping_wakeup_report_.print("ping_wakeup");
  }
  void print_diff_wakeup_report() {
    diff_wakeup_report_.print("diff_wakeup");
  }
  void print_timer_callback_process_time_report() {
    timer_callback_process_time_report_.print("timer_callback");
  }

protected:
  void on_callback() override;

  void on_overrun() override;

private:
  rclcpp::Publisher<MyMsg>::SharedPtr ping_pub_;
  uint64_t ping_pub_count_;
  const std::string topic_;
  MyMsg msg_;

  struct timespec epoch_ts_;
  struct timespec period_ts_;
  struct timespec expect_ts_;
  struct timespec last_wake_ts_;

  // wakeup jitter report
  JitterReportWithSkip ping_wakeup_report_;
  // wakeup jitter from last wakeup
  JitterReportWithSkip diff_wakeup_report_;
  // timer callback process time report
  JitterReportWithSkip timer_callback_process_time_report_;

  bool debug_print_;
  uint64_t num_loops_;

  int64_t get_now_int64() {
    struct timespec now_ts;
    getnow(&now_ts);
    return _timespec_to_long(&now_ts);
  }
};

class ThreadedTwoWaysNode : public rclcpp::Node
{
public:
  explicit ThreadedTwoWaysNode(
      const std::string name,
      const std::string namespace_,
      const TwoWaysNodeOptions & tw_options = TwoWaysNodeOptions(),
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  virtual ~ThreadedTwoWaysNode()
  {}

  void setup_ping_publisher();

  void print_ping_wakeup_report() {
    ping_helper_->print_ping_wakeup_report();
  }
  void print_diff_wakeup_report() {
    ping_helper_->print_diff_wakeup_report();
  }
  void print_timer_callback_process_time_report() {
    ping_helper_->print_timer_callback_process_time_report();
  }

private:
  rclcpp::TimerBase::SharedPtr ping_timer_;
  std::unique_ptr<PingPublisherByTimer> ping_helper_;

  const TwoWaysNodeOptions & tw_options_;
};

#endif // THREADED_TWO_WAYS_NODE_HPP
