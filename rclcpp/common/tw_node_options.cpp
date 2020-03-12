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

