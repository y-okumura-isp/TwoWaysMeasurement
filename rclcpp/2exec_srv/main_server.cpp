#include <rclcpp/rclcpp.hpp>

#include "../common/tw_node_options.hpp"
#include "../common/tw_utils.hpp"
#include "../common/two_ways_service_node.hpp"

int main(int argc, char *argv[])
{
  std::cout << "Press C-c to quit" << std::endl;

  TwoWaysNodeOptions tw_options(argc, argv);
  SET_REALTIME_SETTING_RRRR(tw_options);

  rclcpp::init(argc, argv);
  auto exec = tw_options.get_executor();
  rclcpp::NodeOptions node_options;
  tw_options.set_node_options(node_options);
  auto n = std::make_shared<TwoWaysServiceNode>(tw_options.node_name_sub, tw_options, node_options);
  if (!n->setup_ping_service()) {
     std::cerr << "cannot setup service" << std::endl;
  }

  exec->add_node(n);
  SET_REALTIME_SETTING_RRTS(tw_options);
  exec->spin();
  exec->remove_node(n);

  n->print_ping_sub_report();
}
