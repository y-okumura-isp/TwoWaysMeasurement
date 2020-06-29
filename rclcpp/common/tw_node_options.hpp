#ifndef SETTING_H_
#define SETTING_H_

#include <sched.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

struct JitterReportOptions
{
  int bin;
  int round_ns;
  int num_skip;
};

#define SET_REALTIME_SETTING_RRRR(tw_option) \
  if (tw_options.sched_rrrr > 0) { \
    std::cout << "rrrr" << std::endl; \
    if (!tw_options.set_realtime_settings()) { \
      std::cerr << "set_realtime_setting failed" << std::endl; \
      return -1; \
    } \
  }

#define SET_REALTIME_SETTING_RRTS(tw_option) \
  if (tw_options.sched_rrts > 0) { \
    std::cout << "rrts" << std::endl; \
    if (!tw_options.set_realtime_settings()) { \
      std::cerr << "set_realtime_setting failed" << std::endl; \
      return -1; \
    } \
  }

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
  int sched_rrts;  // 0: false, 1: true
  int sched_rrrr;  // 0: false, 1: true
  size_t sched_priority;
  int sched_policy;

  // executor
  int use_static_executor;

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

  /////// test conditions
  // number of loops
  const int num_loops_;
  // Options for rclcpp::Executor
  const bool use_tlsf_allocator;
  // Options for rclcpp::Subscription
  const bool use_message_pool_memory_strategy;
  // Options for rclcpp::Node
  int use_intra_process_comms; // 0: false, 1: true

private:
  void init_report_option(int bin, int round_ns, int num_skip);

};

#endif  /* SETTING_H_ */
