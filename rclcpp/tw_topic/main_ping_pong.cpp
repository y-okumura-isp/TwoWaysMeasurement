#include <chrono>
#include <memory>
#include <time.h>
#include <vector>
#include <numeric>

#include "../common/tw_utils.hpp"
#include "../common/two_ways_node.hpp"
#include "../common/threaded_two_ways_node.hpp""
#include "main_ping_pong.hpp"

const char * node_name = "one_node_ping_pong";


std::unique_ptr<Runner>
make_runner(RunType type, rclcpp::executor::Executor::SharedPtr e)
{
  std::unique_ptr<Runner> p(nullptr);
  switch(type) {
    case(E1N1): {
      p.reset(new Runner_1e1n<TwoWaysNode>(e));
      break;
    }
    case(E1N2): {
      p.reset(new Runner_1e2n<TwoWaysNode>(e));
      break;
    }
    case(E2_PING): {
      p.reset(new Runner_2e_ping<TwoWaysNode>(e));
      break;
    }
    case(E2_PONG): {
      p.reset(new Runner_2e_pong<TwoWaysNode>(e));
      break;
    }
    case(E1N1_THREADED): {
      p.reset(new Runner_1e1n<ThreadedTwoWaysNode>(e));
      break;
    }
    case(E1N2_THREADED): {
      p.reset(new Runner_1e2n<ThreadedTwoWaysNode>(e));
      break;
    }
    case(E2_PING_THREADED): {
      p.reset(new Runner_2e_ping<ThreadedTwoWaysNode>(e));
      break;
    }
    case(E2_PONG_THREADED): {
      p.reset(new Runner_2e_pong<ThreadedTwoWaysNode>(e));
      break;
    }
  }

  return p;
}

int main(int argc, char *argv[])
{
  std::cout << "Press C-c to quit" << std::endl;

  TwoWaysNodeOptions tw_options(argc, argv);

  if(lock_and_prefault_dynamic(tw_options.prefault_dynamic_size) < 0) {
    std::cerr << "lock_and_prefault_dynamic failed" << std::endl;
  }

  // setup child process scheule
  size_t priority = 0;
  int policy = 0;
  tw_options.get_child_thread_policy(priority, policy);
  set_sched_priority("child", priority, policy);

  rclcpp::init(argc, argv);
  auto exec = tw_options.get_executor();

  auto runner = make_runner(tw_options.run_type, exec);
  runner->setup(tw_options);

  // setup child process scheule
  tw_options.get_main_thread_policy(priority, policy);
  set_sched_priority("main", priority, policy);

  exec->spin();

  runner->cleanup();

  runner->report();

  rclcpp::shutdown();
}
