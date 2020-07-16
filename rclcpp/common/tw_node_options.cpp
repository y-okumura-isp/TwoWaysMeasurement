#include <unistd.h>
#include <getopt.h>

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
    {"main-sched",      required_argument,      0,                              'm'},
    {"child-sched",     required_argument,      0,                              'c'},
    {"callback-sched",  required_argument,      0,                              'b'},
    {"run-type",        required_argument,      0,                              't'},
    {"default-memory-strategy", no_argument,    0,                              's'},
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
      case('m'): {
        main_sched = get_schedule_policy(std::string(optarg));
        break;
      }
      case('c'): {
        child_sched = get_schedule_policy(std::string(optarg));
        break;
      }
      case('b'):{
        callback_sched = get_schedule_policy(std::string(optarg));
        break;
      }
      case('t'): {
        run_type = parse_run_type(std::string(optarg));
        break;
      }
      case('s'): {
        use_message_pool_memory_strategy = false;
        break;
      }
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
    main_sched(SCHED_POLICY::TS),
    child_sched(SCHED_POLICY::TS),
    callback_sched(SCHED_POLICY::TS),
    sched_priority(98),
    sched_policy(SCHED_RR),
    run_type(E1N1),
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

void TwoWaysNodeOptions::set_node_options(rclcpp::NodeOptions & node_options) const
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

void TwoWaysNodeOptions::get_sched(SCHED_POLICY sp, size_t &priority, int &policy) const
{
  switch(sp)
  {
    case(TS): {
      priority = 0;
      policy = SCHED_OTHER;
      break;
    }
    case(RR98): {
      priority = 98;
      policy = SCHED_RR;
      break;
    }
    case(RR97): {
      priority = 97;
      policy = SCHED_RR;
      break;
    }
    case(RR96): {
      priority = 96;
      policy = SCHED_RR;
      break;
    }
  }
}

void TwoWaysNodeOptions::init_report_option(int bin, int round_ns, int num_skip)
{
  common_report_option.bin = bin;
  common_report_option.round_ns = round_ns;
  common_report_option.num_skip = num_skip;
}

SCHED_POLICY TwoWaysNodeOptions::get_schedule_policy(const std::string &opt)
{
  std::map<std::string, SCHED_POLICY> name2policy = {
    {"RR98",    SCHED_POLICY::RR98},
    {"RR97",    SCHED_POLICY::RR97},
    {"RR96",    SCHED_POLICY::RR96},
    {"TS",      SCHED_POLICY::TS},
  };
  auto ret = name2policy.find(opt);
  if(ret != name2policy.end()) {
    return ret->second;
  } else {
    throw std::invalid_argument("unknown run-type: RR98, RR97, RR96, TS");
  }
}

RunType TwoWaysNodeOptions::parse_run_type(const std::string &name)
{
  std::map<std::string, RunType> name2type {
    {"1e1n", E1N1},
    {"1e2n", E1N2},
    {"2e_ping", E2_PING},
    {"2e_pong", E2_PONG},

    {"1e1n_threaded", E1N1_THREADED},
    {"1e2n_threaded", E1N2_THREADED},
    {"2e_ping_threaded", E2_PING_THREADED},
    {"2e_pong_threaded", E2_PONG_THREADED},
  };

  auto ret = name2type.find(name);
  if(ret != name2type.end()) {
    return ret->second;
  } else {
    throw std::invalid_argument("unknown run-type: 1e1n, 1e2n, 2e_ping, 2e_pong, 1e1n_threaded, ...");
  }
}
