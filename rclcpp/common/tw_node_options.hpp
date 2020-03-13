#ifndef SETTING_H_
#define SETTING_H_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

struct JitterReportOptions
{
  int bin;
  int round_ns;
};

class TwoWaysNodeOptions {
public:
  TwoWaysNodeOptions()
  {
    ping_wakeup.bin = 600;
    ping_wakeup.round_ns = 1000;

    ping_sub.bin = 600;
    ping_sub.round_ns = 1000;
  }

  /// Init NodeOptions
  void set_node_options(rclcpp::NodeOptions & node_options);

  /// Get Executor
  rclcpp::executor::Executor::SharedPtr get_executor();

  const char * node_name = "node";
  const char * node_name_pub = "node_pub";
  const char * node_name_sub = "node_sub";
  const char * namespace_ = "ns";
  const char * topic_name = "ping";
  const char * service_name = "ping";
  rclcpp::QoS qos = rclcpp::QoS(1).best_effort();

  // wake up period[ns]
  const int period_ns = 10 * 1000 * 1000;
  JitterReportOptions ping_wakeup;
  JitterReportOptions ping_sub;

  // Options for rclcpp::Executor
  const bool use_tlsf_allocator = true;

  // Options for rclcpp::Subscription
  const bool use_message_pool_memory_strategy = true;

  // Options for rclcpp::Node
  const bool use_intra_process_comms = false;
};

#endif  /* SETTING_H_ */
