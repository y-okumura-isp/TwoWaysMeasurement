#include <chrono>
#include <memory>
#include <time.h>
#include <vector>
#include <numeric>

#include "rclcpp/rclcpp.hpp"
#include "twmsgs/msg/data.hpp"
#include "../common/two_ways_node.hpp"
#include "../common/tw_utils.hpp"
#include "../common/tw_node_options.hpp"

int main(int argc, char *argv[])
{
  std::cout << "Press C-c to quit" << std::endl;

  rclcpp::init(argc, argv);

  TwoWaysNodeOptions tw_options;
  if (!tw_options.set_realtime_settings()) {
    std::cerr << "set_realtime_setting failed" << std::endl;;
    return -1;
  }

  auto exec = tw_options.get_executor();
  rclcpp::NodeOptions node_options;
  tw_options.set_node_options(node_options);
  auto npub = std::make_shared<TwoWaysNode>("pub", "ns", tw_options, node_options);
  auto nsub = std::make_shared<TwoWaysNode>("sub", "ns", tw_options, node_options);

  npub->setup_ping_publisher();
  npub->setup_pong_subscriber();
  nsub->setup_ping_subscriber(true);

  exec->add_node(npub);
  exec->add_node(nsub);
  exec->spin();
  exec->remove_node(nsub);
  exec->remove_node(npub);

  npub->print_ping_wakeup_report();
  npub->print_diff_wakeup_report();
  nsub->print_ping_sub_report();
  npub->print_pong_sub_report();
  npub->print_ping_pong_report();

  rclcpp::shutdown();
}
