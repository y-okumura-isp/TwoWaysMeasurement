#ifndef SETTING_H_
#define SETTING_H_

#include <sched.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

struct JitterReportOptions
{
  int bin;
  int round_ns;
};

class TwoWaysNodeOptions {
public:
  TwoWaysNodeOptions();

  TwoWaysNodeOptions(int argc, char *argv[]);

  /// Init NodeOptions
  void set_node_options(rclcpp::NodeOptions & node_options);

  /// Get Executor
  rclcpp::executor::Executor::SharedPtr get_executor();

  /// Realtime settings
  bool set_realtime_settings();

  // scheduler
  bool sets_realtime_settings;
  size_t sched_priority;
  int sched_policy;

  // prefault
  size_t prefault_dynamic_size;

  // node, topic, qos
  std::string node_name;
  std::string node_name_pub;
  std::string node_name_sub;
  std::string namespace_;
  std::string topic_name;
  std::string topic_name_pong;
  std::string service_name;
  rclcpp::QoS qos;

  // wake up period[ns]
  const int period_ns;
  JitterReportOptions common_report_option;
  JitterReportOptions ping_wakeup;
  JitterReportOptions ping_sub;

  /////// test conditions
  // number of loops
  const int num_loops_;
  // Options for rclcpp::Executor
  const bool use_tlsf_allocator;
  // Options for rclcpp::Subscription
  const bool use_message_pool_memory_strategy;
  // Options for rclcpp::Node
  const bool use_intra_process_comms;
};

#endif  /* SETTING_H_ */
