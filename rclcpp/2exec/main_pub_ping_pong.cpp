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

  TwoWaysNodeOptions tw_options(argc, argv);
  SET_REALTIME_SETTING_RRRR(tw_options);

  rclcpp::init(argc, argv);
  auto exec = tw_options.get_executor();
  rclcpp::NodeOptions node_options;
  tw_options.set_node_options(node_options);
  auto npub = std::make_shared<TwoWaysNode>("main_pub_ping_pong", "ns", tw_options, node_options);

  npub->setup_ping_publisher();
  npub->setup_pong_subscriber();

  exec->add_node(npub);
  SET_REALTIME_SETTING_RRTS(tw_options);
  exec->spin();
  exec->remove_node(npub);

  npub->print_ping_wakeup_report();
  npub->print_diff_wakeup_report();
  npub->print_pong_sub_report();
  npub->print_ping_pong_report();

  rclcpp::shutdown();
}
