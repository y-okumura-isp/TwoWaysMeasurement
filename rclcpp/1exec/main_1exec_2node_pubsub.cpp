#include <chrono>
#include <memory>
#include <time.h>
#include <vector>
#include <numeric>

#include "rclcpp/rclcpp.hpp"
#include "twmsgs/msg/data.hpp"
#include "../common/pub_node.hpp"
#include "../common/sub_node.hpp"
#include "../common/tw_utils.hpp"
#include "../common/tw_node_options.hpp"

int main(int argc, char *argv[])
{
  std::cout << "Press C-c to quit" << std::endl;

  rclcpp::init(argc, argv);

  TwoWaysNodeOptions tw_options;

  auto exec = tw_options.get_executor();
  rclcpp::NodeOptions node_options;
  tw_options.set_node_options(node_options);
  auto npub = std::make_shared<PubNode>(tw_options, node_options);
  auto nsub = std::make_shared<SubNode>(tw_options, node_options);

  exec->add_node(npub);
  exec->add_node(nsub);
  exec->spin();
  exec->remove_node(nsub);
  exec->remove_node(npub);

  npub->print_ping_wakeup_report();
  nsub->print_ping_sub_report();

  rclcpp::shutdown();
}
