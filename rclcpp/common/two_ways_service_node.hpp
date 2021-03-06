#ifndef TWO_WAYS_SERVICE_NODE_HPP_
#define TWO_WAYS_SERVICE_NODE_HPP_

#include <time.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "twmsgs/srv/data.hpp"
#include "../common/tw_node_options.hpp"
#include "tw_utils.hpp"

class TwoWaysServiceNode : public rclcpp::Node
{
public:
  TwoWaysServiceNode(
      const std::string node_name,
      const TwoWaysNodeOptions & tw_options,
      const rclcpp::NodeOptions & options)
      : Node(node_name, tw_options.namespace_, options), tw_options_(tw_options)
  {
    JitterReportWithSkip* reports[] = {
      &ping_wakeup_report_,
      &diff_wakeup_report_,
      &ping_sub_report_,
      &pong_trans_report_,
      &ping_pong_report_};
    for(auto l : reports) {
      l->init(tw_options.common_report_option.bin,
              tw_options.common_report_option.round_ns);
    }
  }

  bool setup_ping_client();
  bool setup_ping_service();

  void print_ping_wakeup_report() {
    ping_wakeup_report_.print("ping_wakeup");
  }
  void print_diff_wakeup_report() {
    diff_wakeup_report_.print("diff_wakeup");
  }
  void print_ping_sub_report() {
    ping_sub_report_.print("ping_sub");
  }
  void print_pong_trans_report() {
    pong_trans_report_.print("pong_trans");
  }
  void print_ping_pong_report() {
    ping_pong_report_.print("ping_pong");
  }

protected:
  const TwoWaysNodeOptions & tw_options_;

private:
  struct timespec epoch_ts_;
  struct timespec period_ts_;
  struct timespec expect_ts_;
  struct timespec last_wake_ts_;
  twmsgs::srv::Data::Request::SharedPtr req;

  rclcpp::TimerBase::SharedPtr ping_timer_;
  rclcpp::Client<twmsgs::srv::Data>::SharedPtr ping_client_;
  rclcpp::Service<twmsgs::srv::Data>::SharedPtr ping_server_;
  int ping_send_count_ = 0;
  int ping_recv_count_ = 0;

  // wakeup jitter report
  JitterReportWithSkip ping_wakeup_report_;
  // wakeup jitter from last wakeup
  JitterReportWithSkip diff_wakeup_report_;
  // sub jitter report
  JitterReportWithSkip ping_sub_report_;
  // pong_recv - pong_send jitter report
  JitterReportWithSkip pong_trans_report_;
  // ping-pong jitter report
  JitterReportWithSkip ping_pong_report_;
};

#endif  // TWO_WAYS_SERVICE_NODE_HPP_
