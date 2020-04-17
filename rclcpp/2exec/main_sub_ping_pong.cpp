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

  TwoWaysNodeOptions tw_options(argc, argv);
  SET_REALTIME_SETTING_RRRR(tw_options);

  rclcpp::init(argc, argv);
  auto exec = tw_options.get_executor();
  rclcpp::NodeOptions node_options;
  tw_options.set_node_options(node_options);
  auto nsub = std::make_shared<TwoWaysNode>("main_sub_ping_pong", "ns", tw_options, node_options);

  nsub->setup_ping_subscriber(true);

  exec->add_node(nsub);
  SET_REALTIME_SETTING_RRTS(tw_options);
  exec->spin();
  exec->remove_node(nsub);

  nsub->print_ping_sub_report();

  rclcpp::shutdown();
}
