#ifndef SETTING_H_
#define SETTING_H_

#include <rclcpp/rclcpp.hpp>

struct JitterReportOptions
{
  int bin;
  int round_ns;
};

class TwoWaysNodeOptions {
public:
  TwoWaysNodeOptions()
      : node_name_pub("one_node_sub"),
        node_name_sub("one_node_pub"),
        namespace_("ns"),
        topic_name("ping"),
        qos(10),
        period_ns(10 * 1000 * 1000),
        use_tlsf_allocator(true),
        use_message_pool_memory_strategy(true),
        use_intra_process_comms(false)
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

  const char * node_name_pub;
  const char * node_name_sub;
  const char * namespace_;
  const char * topic_name;
  const int qos;
  // wake up period[ns]
  const int period_ns;
  JitterReportOptions ping_wakeup;
  JitterReportOptions ping_sub;

  // Options for rclcpp::Executor
  const bool use_tlsf_allocator;

  // Options for rclcpp::Subscription
  const bool use_message_pool_memory_strategy;

  // Options for rclcpp::Node
  const bool use_intra_process_comms;
};

#endif  /* SETTING_H_ */
