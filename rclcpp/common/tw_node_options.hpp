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

enum SCHED_POLICY {
  RR98,
  RR97,
  TS
};

enum RunType {
  E1N1, // 1 executor, 1 node
  E1N2, // 1 executor, 2 nodes
};

class TwoWaysNodeOptions {
public:
  TwoWaysNodeOptions();

  TwoWaysNodeOptions(int argc, char *argv[]);

  /// Init NodeOptions
  void set_node_options(rclcpp::NodeOptions & node_options) const;

  /// Get Executor
  rclcpp::executor::Executor::SharedPtr get_executor();

  /// Get main thread policy
  void get_main_thread_policy(size_t &priority, int &policy) {
    get_sched(main_sched, priority, policy);
  }

  /// Get child thread policy
  void get_child_thread_policy(size_t &priority, int &policy) {
    get_sched(child_sched, priority, policy);
  }

  // scheduler
  int sched_rrts;  // 0: false, 1: true
  int sched_rrrr;  // 0: false, 1: true
  SCHED_POLICY main_sched;
  SCHED_POLICY child_sched;

  size_t sched_priority;
  int sched_policy;

  // run type
  RunType run_type;

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
  // convert SCHED_POLICY to size_t and int.
  void get_sched(SCHED_POLICY sp, size_t &priority, int &policy);
  SCHED_POLICY get_schedule_policy(const std::string &opt);
  RunType parse_run_type(const std::string &type);
};

#endif  /* SETTING_H_ */
