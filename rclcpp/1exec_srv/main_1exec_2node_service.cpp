#include <rclcpp/rclcpp.hpp>

#include "../common/tw_node_options.hpp"
#include "../common/tw_utils.hpp"
#include "../common/two_ways_service_node.hpp"

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

  auto client_node = std::make_shared<TwoWaysServiceNode>(tw_options.node_name_pub, tw_options, node_options);
  auto service_node = std::make_shared<TwoWaysServiceNode>(tw_options.node_name_sub, tw_options, node_options);
  if (!service_node->setup_ping_service()) {
     std::cerr << "cannot setup service" << std::endl;
  }
  if (!client_node->setup_ping_client()) {
    std::cerr << "cannot setup client" << std::endl;
  }

  exec->add_node(service_node);
  exec->add_node(client_node);

  // setup child process scheule
  tw_options.get_main_thread_policy(priority, policy);
  set_sched_priority("main", priority, policy);

  exec->spin();
  exec->remove_node(client_node);
  exec->remove_node(service_node);

  client_node->print_ping_wakeup_report();
  client_node->print_diff_wakeup_report();
  service_node->print_ping_sub_report();
  client_node->print_pong_trans_report();
  client_node->print_ping_pong_report();
}
