#include <chrono>
#include <memory>
#include <time.h>
#include <vector>
#include <numeric>

#include "rclcpp/rclcpp.hpp"
#include "../common/two_ways_node.hpp"
#include "../common/tw_utils.hpp"

const char * node_name = "one_node_ping_pong";

int main(int argc, char *argv[])
{
  std::cout << "Press C-c to quit" << std::endl;

  TwoWaysNodeOptions tw_options(argc, argv);
  SET_REALTIME_SETTING_RRRR(tw_options);

  rclcpp::init(argc, argv);
  auto exec = tw_options.get_executor();
  rclcpp::NodeOptions node_options;
  tw_options.set_node_options(node_options);
  auto n = std::make_shared<TwoWaysNode>("node_1e1n", "ns", tw_options, node_options);
  n->setup_ping_publisher();
  n->setup_ping_subscriber(true);
  n->setup_pong_subscriber();

  exec->add_node(n);
  SET_REALTIME_SETTING_RRTS(tw_options);
  exec->spin();
  exec->remove_node(n);

  n->print_ping_wakeup_report();
  n->print_diff_wakeup_report();
  n->print_ping_sub_report();
  n->print_pong_sub_report();
  n->print_ping_pong_report();

  rclcpp::shutdown();
}
