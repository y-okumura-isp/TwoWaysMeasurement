#include <malloc.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>


#include <rttest/rttest.h>
#include <rclcpp/strategies/allocator_memory_strategy.hpp>
#include <tlsf_cpp/tlsf.hpp>
#include "tw_node_options.hpp"

using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;

template<typename T = void>
using TLSFAllocator = tlsf_heap_allocator<T>;

TwoWaysNodeOptions::TwoWaysNodeOptions():
    sets_realtime_settings(false),
    sched_priority(98),
    sched_policy(SCHED_RR),
    prefault_dynamic_size(209715200UL),  // 200MB
    node_name("node"),
    node_name_pub("node_pub"),
    node_name_sub("node_sub"),
    namespace_("ns"),
    topic_name("ping"),
    topic_name_pong("pong"),
    service_name("ping"),
    qos(rclcpp::QoS(1).best_effort()),
    period_ns(10 * 1000 * 1000),
    num_loops_(10000),
    use_tlsf_allocator(true),
    use_message_pool_memory_strategy(true),
    use_intra_process_comms(false)
{
    common_report_option.bin = 600;
    common_report_option.round_ns = 1000;

    ping_wakeup.bin = 600;
    ping_wakeup.round_ns = 1000;

    ping_sub.bin = 600;
    ping_sub.round_ns = 1000;
}

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

int lock_and_prefault_dynamic(size_t prefault_dynamic_size)
{
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    perror("mlockall failed");
    return -1;
  }

  // Turn off malloc trimming.
  if (mallopt(M_TRIM_THRESHOLD, -1) == 0) {
    perror("mallopt for trim threshold failed");
    munlockall();
    return -1;
  }

  // Turn off mmap usage.
  if (mallopt(M_MMAP_MAX, 0) == 0) {
    perror("mallopt for mmap failed");
    mallopt(M_TRIM_THRESHOLD, 128 * 1024);
    munlockall();
    return -1;
  }

  struct rusage usage;
  size_t page_size = sysconf(_SC_PAGESIZE);
  getrusage(RUSAGE_SELF, &usage);
  size_t prev_minflts = usage.ru_minflt;
  size_t prev_majflts = usage.ru_majflt;
  size_t encountered_minflts = 1;
  size_t encountered_majflts = 1;

  size_t array_size = sizeof(char) * 64 * page_size;
  size_t total_size = 0;
  size_t max_size = prefault_dynamic_size;
  std::vector<char *> prefaulter;
  prefaulter.reserve((size_t)(max_size / array_size));

  // prefault until you see no more pagefaults
  while (encountered_minflts > 0 || encountered_majflts > 0) {
    char * ptr;
    try {
      ptr = new char[array_size];
      memset(ptr, 0, array_size);
      total_size += array_size;
    } catch (std::bad_alloc & e) {
      fprintf(stderr, "Caught exception: %s\n", e.what());
      fprintf(stderr, "Unlocking memory and continuing.\n");
      for (auto & ptr : prefaulter) {
        delete[] ptr;
      }

      mallopt(M_TRIM_THRESHOLD, 128 * 1024);
      mallopt(M_MMAP_MAX, 65536);
      munlockall();
      return -1;
    }

    // If we reached max_size then delete created char array.
    // This will prevent pagefault on next allocation.
    if (total_size >= max_size) {
      delete[] ptr;
    } else {
      prefaulter.push_back(ptr);
    }

    getrusage(RUSAGE_SELF, &usage);
    size_t current_minflt = usage.ru_minflt;
    size_t current_majflt = usage.ru_majflt;
    encountered_minflts = current_minflt - prev_minflts;
    encountered_majflts = current_majflt - prev_majflts;
    prev_minflts = current_minflt;
    prev_majflts = current_majflt;
  }

  for (auto & ptr : prefaulter) {
    delete[] ptr;
  }
  return 0;
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
  if (lock_and_prefault_dynamic(prefault_dynamic_size) != 0) {
    perror("mlockall failed");
    return -1;
  }

  return true;
}
