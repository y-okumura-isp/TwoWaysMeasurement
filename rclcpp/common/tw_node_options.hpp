#ifndef SETTING_H_
#define SETTING_H_

#include <rclcpp/strategies/allocator_memory_strategy.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tlsf_cpp/tlsf.hpp>

using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;

template<typename T = void>
using TLSFAllocator = tlsf_heap_allocator<T>;

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
        use_intra_process_comms(false)
  {
    ping_wakeup.bin = 600;
    ping_wakeup.round_ns = 1000;

    ping_sub.bin = 600;
    ping_sub.round_ns = 1000;
  }

  void set_node_options(rclcpp::NodeOptions & node_options)
  {
    node_options.use_intra_process_comms(use_intra_process_comms);
  }

  rclcpp::executor::Executor::SharedPtr get_executor()
  {
    rclcpp::executor::ExecutorArgs args;

    if (use_tlsf_allocator) {
      rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy =
        std::make_shared<AllocatorMemoryStrategy<TLSFAllocator<void>>>();
      args.memory_strategy = memory_strategy;
    }

    return std::make_shared<rclcpp::executors::SingleThreadedExecutor>(args);
  }

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


  // Options for rclcpp::Node
  const bool use_intra_process_comms;
};

#endif  /* SETTING_H_ */
