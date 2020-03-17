#include <chrono>
#include <memory>
#include <time.h>
#include <vector>
#include <numeric>

#include "rclcpp/rclcpp.hpp"
#include "twmsgs/msg/data.hpp"
#include "../common/two_ways_node.hpp"
#include "../common/tw_utils.hpp"

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
  auto nsub = std::make_shared<TwoWaysNode>("main_sub_ping_pong", "ns", tw_options, node_options);

  nsub->setup_ping_subscriber(true);

  exec->add_node(nsub);
  exec->spin();
  exec->remove_node(nsub);

  nsub->print_ping_sub_report();

  rclcpp::shutdown();
}
