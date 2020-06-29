#include <malloc.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>
#include <getopt.h>

#include <rttest/rttest.h>
#include <rclcpp/strategies/allocator_memory_strategy.hpp>
#include <tlsf_cpp/tlsf.hpp>
#include "tw_node_options.hpp"

using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;

template<typename T = void>
using TLSFAllocator = tlsf_heap_allocator<T>;

#define TRUE 1
#define FALSE 0

const int REPORT_BIN_DEFAULT = 600;
const int REPORT_ROUND_NS_DEFAULT = 1000;
const int REPORT_NUM_SKIP_DEFAULT = 10;

TwoWaysNodeOptions::TwoWaysNodeOptions(int argc, char *argv[])
    : TwoWaysNodeOptions()
{
  int c = 0;
  int _optind = optind, _opterr = opterr, _optopt = optopt;

  optind = 1;
  opterr = 0;

  // prevent permutation of argv
  const std::string optstring = "-";
  const struct option longopts[] = {
    {"sched-rrts",      no_argument,            &sched_rrts,                    TRUE},
    {"sched-rrrr",      no_argument,            &sched_rrrr,                    TRUE},
    {"ipm",             no_argument,            &use_intra_process_comms,       TRUE},
    {"round-ns",        required_argument,      0,                              'i'},
    {"num-skip",        required_argument,      0,                              'r'},
    {"static-executor", no_argument,            &use_static_executor,           TRUE},
    {0,                 0,                      0,                               0},
  };

  int longindex = 0;
  int tmp = -1;
  int round_ns = REPORT_ROUND_NS_DEFAULT;
  int num_skip = REPORT_NUM_SKIP_DEFAULT;
  bool needs_reinit = false;
  while ((c=getopt_long(argc, argv, optstring.c_str(), longopts, &longindex)) != -1) {
    switch(c)
    {
      case('i'):
        tmp = std::stoi(optarg);
        if(tmp > 0) {
          round_ns = tmp;
          needs_reinit = true;
        }
        break;
      case('r'):
        tmp = std::stoi(optarg);
        if(tmp > 0) {
          num_skip = tmp;
          needs_reinit = true;
        }
        break;
      default:
        break;
    }
  }

  if(needs_reinit) {
    std::cout << "num_skip: " << num_skip << std::endl;
    init_report_option(REPORT_BIN_DEFAULT, round_ns, num_skip);
  }

  optind = _optind;
  opterr = _opterr;
  optopt = _optopt;
}

TwoWaysNodeOptions::TwoWaysNodeOptions():
    sched_rrts(0),
    sched_rrrr(0),
    sched_priority(98),
    sched_policy(SCHED_RR),
    use_static_executor(FALSE),
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
    use_intra_process_comms(FALSE)
{
  init_report_option(REPORT_BIN_DEFAULT, REPORT_ROUND_NS_DEFAULT, REPORT_NUM_SKIP_DEFAULT);
}

void TwoWaysNodeOptions::set_node_options(rclcpp::NodeOptions & node_options)
{
  std::cout << "use_intra_process_comms = " << (use_intra_process_comms == TRUE ? "true" : "false") << std::endl;
  node_options.use_intra_process_comms(use_intra_process_comms == TRUE);
}

rclcpp::executor::Executor::SharedPtr TwoWaysNodeOptions::get_executor()
{
  rclcpp::ExecutorOptions args;

  if (use_tlsf_allocator) {
    rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy =
        std::make_shared<AllocatorMemoryStrategy<TLSFAllocator<void>>>();
    args.memory_strategy = memory_strategy;
  }

  if (use_static_executor) {
    std::cout << "use StaticSingleThreadedExecutor" << std::endl;
    return std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>(args);
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

void TwoWaysNodeOptions::init_report_option(int bin, int round_ns, int num_skip)
{
  common_report_option.bin = bin;
  common_report_option.round_ns = round_ns;
  common_report_option.num_skip = num_skip;
}
