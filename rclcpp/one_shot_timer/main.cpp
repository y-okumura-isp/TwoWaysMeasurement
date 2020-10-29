#include "one_shot_timer_test.hpp"

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
  rclcpp::NodeOptions node_options;
  tw_options.set_node_options(node_options);
  auto n = std::make_shared<OneShotTimerTestNode>(tw_options, node_options);
  n->setup_oneshot_trigger();
  n->setup_periodic_trigger();

  exec->add_node(n);

  tw_options.get_main_thread_policy(priority, policy);
  set_sched_priority("main", priority, policy);

  exec->spin();
  exec->remove_node(n);

  rclcpp::shutdown();
}
