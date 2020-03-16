#include <rttest/rttest.h>
#include <rclcpp/strategies/allocator_memory_strategy.hpp>
#include <tlsf_cpp/tlsf.hpp>
#include "tw_node_options.hpp"

using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;

template<typename T = void>
using TLSFAllocator = tlsf_heap_allocator<T>;

void TwoWaysNodeOptions::set_node_options(rclcpp::NodeOptions & node_options)
{
  node_options.use_intra_process_comms(use_intra_process_comms);
}

rclcpp::executor::Executor::SharedPtr TwoWaysNodeOptions::get_executor()
{
  rclcpp::executor::ExecutorArgs args;

  if (use_tlsf_allocator) {
    rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy =
        std::make_shared<AllocatorMemoryStrategy<TLSFAllocator<void>>>();
    args.memory_strategy = memory_strategy;
  }

  return std::make_shared<rclcpp::executors::SingleThreadedExecutor>(args);
}

bool TwoWaysNodeOptions::set_realtime_settings()
{
  if (!sets_realtime_settings) {
    return true;
  }

  // scheduler
  if (rttest_set_sched_priority(sched_priority,
                                sched_policy) != 0) {
    std::cerr << "Couldn't set scheduling priority and policy" << std::endl;
    return false;
  }

  // malloc
  if (rttest_lock_and_prefault_dynamic() != 0) {
    std::cerr << "Couldn't lock all cached virtual memory.\n";
    std::cerr << "Pagefaults from reading pages not yet mapped into RAM will be recorded.\n";
    return false;
  }

  return true;
}
